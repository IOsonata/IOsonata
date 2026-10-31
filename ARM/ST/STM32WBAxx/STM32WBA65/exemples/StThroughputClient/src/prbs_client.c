/*
 * Copyright (c) 2026, I-SYST inc.
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
 *
 * PRBS comparison application. ST owns startup, radio, host, timers and event
 * delivery. Only application procedures and measurement are implemented here.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_common.h"
#include "ble_core.h"
#include "svc_ctl.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "log_module.h"
#include "prbs_client.h"

#define PRBS_TASK CFG_TASK_DISCOVER_SERVICES_ID
#define PRBS_RX_TASK CFG_TASK_DATA_FROM_SERVER_ID
#define PRBS_DURATION 60000U
#define PRBS_SETUP_TIMEOUT 30000U
#define PRBS_QUEUE_COUNT 12U
#define PRBS_PAYLOAD 244U
#define PRBS_INVALID 0xffffU

/* BlueIO UUIDs in ATT little-endian order. Compare the entire UUID. */
static const UUID_t s_ServiceUuid = {
	.UUID_128 = {0x1b, 0xc5, 0xd5, 0xa5, 2, 0, 0x74, 0xab,
		0xe4, 0x11, 0x7c, 0x28, 1, 1, 0, 0}
};
static const uint8_t s_ValueUuid[16] = {
	0x1b, 0xc5, 0xd5, 0xa5, 2, 0, 0x74, 0xab,
	0xe4, 0x11, 0x7c, 0x28, 2, 1, 0, 0
};
static const char s_Target[] = "BlePrbs_SDC";

typedef enum {
	PRBS_SCAN,
	PRBS_SCANNING,
	PRBS_STOP_SCAN,
	PRBS_SCAN_STOPPING,
	PRBS_CONNECT,
	PRBS_CONNECTING,
	PRBS_PHY,
	PRBS_PHY_WAIT,
	PRBS_DLE,
	PRBS_DLE_WAIT,
	PRBS_MTU,
	PRBS_MTU_WAIT,
	PRBS_SERVICE,
	PRBS_SERVICE_WAIT,
	PRBS_CHARS,
	PRBS_CHARS_WAIT,
	PRBS_DESCRIPTORS,
	PRBS_DESCRIPTORS_WAIT,
	PRBS_SUBSCRIBE,
	PRBS_SUBSCRIBE_WAIT,
	PRBS_RECEIVE,
	PRBS_DISCONNECT,
	PRBS_DISCONNECT_WAIT,
	PRBS_DONE
} PrbsPhase_t;

typedef struct {
	uint32_t Time;
	uint16_t Len;
	uint8_t Data[PRBS_PAYLOAD];
} PrbsPacket_t;

typedef struct {
	uint32_t Bytes;
	uint32_t Notifications;
	uint32_t Breaks;
	uint32_t OutOfFrame;
	uint32_t FirstMs;
	uint16_t MinLen;
	uint16_t MaxLen;
	uint8_t Previous;
	uint8_t ValidRun;
	bool Started;
	bool Synced;
} PrbsStats_t;

static PrbsPhase_t s_Phase;
static UTIL_TIMER_Object_t s_Timer;
static PrbsPacket_t s_Queue[PRBS_QUEUE_COUNT];
static PrbsStats_t s_Stats;
static uint8_t s_Head;
static uint8_t s_Tail;
static uint8_t s_Count;
static uint8_t s_Phy = 1;
static uint8_t s_TxPhy;
static uint8_t s_RxPhy;
static uint8_t s_Address[6];
static uint8_t s_AddressType;
static uint16_t s_Connection = PRBS_INVALID;
static uint16_t s_ServiceStart;
static uint16_t s_ServiceEnd;
static uint16_t s_Value;
static uint16_t s_DescEnd;
static uint16_t s_Cccd;
static uint16_t s_Mtu;
static uint16_t s_RxOctets;
static uint16_t s_TxOctets;
static uint32_t s_SetupMs;
static uint32_t s_PhaseMs;
static uint32_t s_LastReport;
static uint32_t s_LastBytes;
static uint32_t s_Dropped;
static bool s_Failed;
static bool s_Final;
static bool s_Complete;

static void PrbsWork(void);
static void PrbsReceive(void);

static uint16_t PrbsU16(const uint8_t *pData)
{
	return (uint16_t)pData[0] | ((uint16_t)pData[1] << 8);
}

static void PrbsSchedule(void)
{
	UTIL_SEQ_SetTask(1U << PRBS_TASK, CFG_SEQ_PRIO_0);
}

static void PrbsPhase(PrbsPhase_t Phase)
{
	s_Phase = Phase;
	s_PhaseMs = UTIL_TIMER_GetCurrentTime();
	PrbsSchedule();
}

static void PrbsFail(const char *pReason, uint8_t Status)
{
	if (!s_Failed)
	{
		LOG_INFO_APP("ST PRBS: %s phase=%u status=0x%02x\n", pReason, s_Phase, Status);
	}
	s_Failed = true;
	PrbsSchedule();
}

static bool PrbsRetry(uint8_t Status)
{
	return Status == BLE_STATUS_BUSY || Status == BLE_STATUS_PENDING ||
		Status == BLE_STATUS_INSUFFICIENT_RESOURCES;
}

static void PrbsReset(void)
{
	memset(&s_Stats, 0, sizeof(s_Stats));
	s_Stats.MinLen = UINT16_MAX;
	s_Head = 0;
	s_Tail = 0;
	s_Count = 0;
	s_Dropped = 0;
	s_LastReport = 0;
	s_LastBytes = 0;
	s_ServiceStart = 0;
	s_ServiceEnd = 0;
	s_Value = 0;
	s_Cccd = 0;
	s_DescEnd = 0;
	s_Mtu = 0;
	s_RxOctets = 0;
	s_TxOctets = 0;
	s_TxPhy = 0;
	s_RxPhy = 0;
	s_Failed = false;
	s_Final = false;
	s_Complete = false;
	s_SetupMs = UTIL_TIMER_GetCurrentTime();
	PrbsPhase(PRBS_SCAN);
}

static void PrbsCheck(const PrbsPacket_t *pPacket)
{
	uint16_t i;
	uint8_t value;
	uint8_t expected;
	bool valid;

	if (s_Final || pPacket->Len == 0)
	{
		return;
	}
	if (s_Stats.Started && (uint32_t)(pPacket->Time - s_Stats.FirstMs) >= PRBS_DURATION)
	{
		return;
	}
	if (!s_Stats.Started)
	{
		s_Stats.FirstMs = pPacket->Time;
		s_LastReport = pPacket->Time;
	}
	for (i = 0; i < pPacket->Len; i++)
	{
		value = pPacket->Data[i];
		expected = ((s_Stats.Previous << 1) |
			(((s_Stats.Previous >> 6) ^ (s_Stats.Previous >> 5)) & 1)) & 0x7f;
		if (s_Stats.Started && value != expected)
		{
			s_Stats.Breaks++;
		}
		if (value > 0x7f && value != 0xff)
		{
			s_Stats.OutOfFrame++;
		}
		if (!s_Stats.Synced)
		{
			valid = (value > 0 && value <= 0x7f) || value == 0xff;
			if (!valid)
			{
				s_Stats.ValidRun = 0;
			}
			else if (!s_Stats.Started || value != expected)
			{
				s_Stats.ValidRun = 1;
			}
			else if (s_Stats.ValidRun < 8)
			{
				s_Stats.ValidRun++;
			}
			if (s_Stats.ValidRun >= 8)
			{
				s_Stats.Synced = true;
			}
		}
		s_Stats.Previous = value;
		s_Stats.Started = true;
	}
	s_Stats.Bytes += pPacket->Len;
	s_Stats.Notifications++;
	if (pPacket->Len < s_Stats.MinLen)
	{
		s_Stats.MinLen = pPacket->Len;
	}
	if (pPacket->Len > s_Stats.MaxLen)
	{
		s_Stats.MaxLen = pPacket->Len;
	}
}

static void PrbsReceive(void)
{
	/* Producer and consumer are ST sequencer tasks, never IRQ callbacks. */
	if (s_Count != 0)
	{
		PrbsCheck(&s_Queue[s_Tail]);
		s_Tail = (s_Tail + 1) % PRBS_QUEUE_COUNT;
		s_Count--;
	}
	if (s_Count != 0)
	{
		UTIL_SEQ_SetTask(1U << PRBS_RX_TASK, CFG_SEQ_PRIO_0);
	}
}

static void PrbsReport(uint32_t Now, bool Final)
{
	uint32_t elapsed;
	uint32_t interval;
	uint32_t rate;
	uint32_t average;
	uint32_t perInterval;
	uint32_t target;
	bool valid;

	elapsed = s_Stats.Started ? (uint32_t)(Now - s_Stats.FirstMs) : 0;
	if (elapsed > PRBS_DURATION)
	{
		elapsed = PRBS_DURATION;
	}
	interval = s_Stats.Started ? (uint32_t)(s_Stats.FirstMs + elapsed - s_LastReport) : 0;
	rate = interval ? (uint32_t)((uint64_t)(s_Stats.Bytes - s_LastBytes) * 1000 / interval) : 0;
	average = elapsed ? (uint32_t)((uint64_t)s_Stats.Bytes * 1000 / elapsed) : 0;
	perInterval = elapsed ? (uint32_t)((uint64_t)s_Stats.Notifications * 15000 / elapsed) : 0;
	target = s_Phy == 1 ? 4 : 8;
	LOG_INFO_APP("%sPHY=%u Bytes/sec=%lu average=%lu elapsed_ms=%lu\n",
		Final ? "Final " : "", s_Phy, (unsigned long)rate, (unsigned long)average, (unsigned long)elapsed);
	LOG_INFO_APP("bytes=%lu notifications=%lu drop=%lu out_of_frame=%lu\n",
		(unsigned long)s_Stats.Bytes, (unsigned long)s_Stats.Notifications,
		(unsigned long)s_Stats.Breaks, (unsigned long)s_Stats.OutOfFrame);
	LOG_INFO_APP("len_min=%u len_max=%u notifications/15ms=%lu.%03lu\n",
		s_Stats.Notifications ? s_Stats.MinLen : 0, s_Stats.MaxLen,
		(unsigned long)(perInterval / 1000), (unsigned long)(perInterval % 1000));
	s_LastReport = s_Stats.FirstMs + elapsed;
	s_LastBytes = s_Stats.Bytes;
	if (Final)
	{
		s_Final = true;
		s_Complete = s_Stats.Started && elapsed == PRBS_DURATION && !s_Failed;
		valid = s_Complete && s_Stats.Synced && s_Stats.Breaks == 0 &&
			s_Stats.OutOfFrame == 0 && s_Dropped == 0;
		LOG_INFO_APP("Notification queue dropped=%lu\n", (unsigned long)s_Dropped);
		LOG_INFO_APP("PRBS integrity: %s\n", valid ? "PASS" : "FAIL");
		LOG_INFO_APP("Notification target=%lu: %s (average only)\n", (unsigned long)target,
			perInterval >= target * 1000 ? "MET" : "NOT MET");
		LOG_INFO_APP("PHY=%u RESULT: %s\n", s_Phy,
			valid && perInterval >= target * 1000 ? "PASS" : "FAIL");
		LOG_INFO_APP("PHY=%u %s\n", s_Phy,
			s_Complete ? "trial complete." : "trial aborted; benchmark stopped.");
	}
}

static void PrbsTick(void *pContext)
{
	(void)pContext;
	/* Timer/IRQ context only posts work. No stack calls or UART output. */
	PrbsSchedule();
}

static void PrbsWork(void)
{
	static const Scan_Param_Phy_t scan[2] = {{HCI_SCAN_TYPE_ACTIVE, 1600, 160}, {0}};
	static const uint8_t enable[2] = {1, 0};
	uint32_t now;
	PrbsPhase_t phase;
	uint8_t status;

	now = UTIL_TIMER_GetCurrentTime();
	if (s_Phase == PRBS_DONE)
	{
		return;
	}
	while (s_Count != 0)
	{
		PrbsReceive();
	}
	if (!s_Final && !s_Stats.Started && (uint32_t)(now - s_SetupMs) >= PRBS_SETUP_TIMEOUT)
	{
		PrbsFail("setup/first notification timeout", 0);
	}
	if (!s_Final && (s_Failed || (s_Stats.Started &&
		(uint32_t)(now - s_Stats.FirstMs) >= PRBS_DURATION)))
	{
		PrbsReport(now, true);
		if (s_Connection != PRBS_INVALID)
		{
			PrbsPhase(PRBS_DISCONNECT);
		}
		else
		{
			(void)aci_gap_terminate_gap_proc(s_Phase == PRBS_CONNECTING ?
				GAP_DIRECT_CONNECTION_ESTABLISHMENT_PROC : GAP_GENERAL_DISCOVERY_PROC);
			PrbsPhase(PRBS_DONE);
			return;
		}
	}
	else if (!s_Final && s_Stats.Started && (uint32_t)(now - s_LastReport) >= 1000)
	{
		PrbsReport(now, false);
	}
	phase = s_Phase;
	status = BLE_STATUS_SUCCESS;
	switch (phase)
	{
		case PRBS_SCAN:
			LOG_INFO_APP("Starting 60-second %uM trial; target=BlePrbs_SDC\n", s_Phy);
			status = aci_gap_start_scan(0, GAP_GENERAL_DISCOVERY_PROC, CFG_BD_ADDRESS_TYPE,
				0, 0, 0, HCI_SCAN_FILTER_NO, HCI_SCANNING_PHYS_LE_1M, scan);
			if (status == 0)
			{
				PrbsPhase(PRBS_SCANNING);
			}
			break;
		case PRBS_STOP_SCAN:
			status = aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
			if (status == 0)
			{
				PrbsPhase(PRBS_SCAN_STOPPING);
			}
			break;
		case PRBS_CONNECT:
			status = aci_gap_create_connection(1600, 160, s_AddressType, s_Address,
				CFG_BD_ADDRESS_TYPE, 12, 12, 0, 400, 0, 80);
			if (status == 0)
			{
				PrbsPhase(PRBS_CONNECTING);
			}
			break;
		case PRBS_PHY:
			status = hci_le_set_phy(s_Connection, 0, s_Phy, s_Phy, 0);
			if (status == 0)
			{
				PrbsPhase(PRBS_PHY_WAIT);
			}
			break;
		case PRBS_DLE:
			status = hci_le_set_data_length(s_Connection, 251, 2120);
			if (status == 0)
			{
				PrbsPhase(s_RxOctets >= 251 ? PRBS_MTU : PRBS_DLE_WAIT);
			}
			break;
		case PRBS_MTU:
			status = aci_gatt_exchange_config(s_Connection);
			if (status == 0)
			{
				PrbsPhase(PRBS_MTU_WAIT);
			}
			break;
		case PRBS_SERVICE:
			status = aci_gatt_disc_primary_service_by_uuid(s_Connection, UUID_TYPE_128, &s_ServiceUuid);
			if (status == 0)
			{
				PrbsPhase(PRBS_SERVICE_WAIT);
			}
			break;
		case PRBS_CHARS:
			status = aci_gatt_disc_all_char_of_service(s_Connection, s_ServiceStart, s_ServiceEnd);
			if (status == 0)
			{
				PrbsPhase(PRBS_CHARS_WAIT);
			}
			break;
		case PRBS_DESCRIPTORS:
			/* ST starts after Char_Handle; pass the value handle itself. */
			status = aci_gatt_disc_all_char_desc(s_Connection, s_Value, s_DescEnd);
			if (status == 0)
			{
				PrbsPhase(PRBS_DESCRIPTORS_WAIT);
			}
			break;
		case PRBS_SUBSCRIBE:
			status = aci_gatt_write_char_value(s_Connection, s_Cccd, sizeof(enable), enable);
			if (status == 0)
			{
				PrbsPhase(PRBS_SUBSCRIBE_WAIT);
			}
			break;
		case PRBS_DISCONNECT:
			status = aci_gap_terminate(s_Connection, HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE);
			if (status == 0)
			{
				PrbsPhase(PRBS_DISCONNECT_WAIT);
			}
			break;
		default:
			break;
	}
	if (status != 0 && !PrbsRetry(status))
	{
		PrbsFail("command rejected", status);
		if (s_Final)
		{
			PrbsPhase(PRBS_DONE);
		}
	}
	/* Commands and reporting can advance time after PrbsPhase records it. */
	now = UTIL_TIMER_GetCurrentTime();
	if (s_Final && (s_Phase == PRBS_DISCONNECT || s_Phase == PRBS_DISCONNECT_WAIT) &&
		(uint32_t)(now - s_PhaseMs) >= 5000)
	{
		LOG_INFO_APP("Disconnect timeout; benchmark stopped.\n");
		PrbsPhase(PRBS_DONE);
	}
}

static void PrbsAdvertising(const uint8_t *pData, uint16_t Len)
{
	uint16_t pos;
	uint16_t end;
	uint16_t ad;
	uint8_t count;
	uint8_t size;

	if (s_Phase != PRBS_SCANNING || Len < 1)
	{
		return;
	}
	count = pData[0];
	pos = 1;
	while (count-- != 0)
	{
		if (Len - pos < 10)
		{
			return;
		}
		end = pos + 9 + pData[pos + 8];
		if (end >= Len)
		{
			return;
		}
		ad = pos + 9;
		while (ad < end)
		{
			size = pData[ad];
			if (size == 0 || ad + 1 + size > end)
			{
				break;
			}
			if (pData[ad + 1] == 9 && size == sizeof(s_Target) &&
				memcmp(&pData[ad + 2], s_Target, sizeof(s_Target) - 1) == 0)
			{
				s_AddressType = pData[pos + 1];
				memcpy(s_Address, &pData[pos + 2], 6);
				LOG_INFO_APP("Found BlePrbs_SDC RSSI=%d\n", (int8_t)pData[end]);
				PrbsPhase(PRBS_STOP_SCAN);
				return;
			}
			ad += size + 1;
		}
		pos = end + 1;
	}
}

static void PrbsConnected(const uint8_t *pData, uint16_t Len, bool Enhanced)
{
	uint16_t offset;

	offset = Enhanced ? 23 : 11;
	if (s_Phase != PRBS_CONNECTING || Len < offset + 7)
	{
		return;
	}
	if (pData[0] != 0)
	{
		PrbsFail("connection failed", pData[0]);
		return;
	}
	s_Connection = PrbsU16(pData + 1);
	LOG_INFO_APP("Connected hdl=%u interval=%u latency=%u requested_PHY=%u\n",
		s_Connection, PrbsU16(pData + offset), PrbsU16(pData + offset + 2), s_Phy);
	if (pData[3] != HCI_ROLE_CENTRAL || PrbsU16(pData + offset) != 12 ||
		PrbsU16(pData + offset + 2) != 0)
	{
		PrbsFail("unexpected connection parameters", 0);
		return;
	}
	PrbsPhase(PRBS_PHY);
}

static void PrbsGattComplete(uint8_t Status)
{
	if (Status != 0)
	{
		PrbsFail("GATT procedure failed", Status);
		return;
	}
	switch (s_Phase)
	{
		case PRBS_MTU_WAIT:
			if (s_Mtu != 247)
			{
				PrbsFail("MTU is not 247", 0);
				return;
			}
			PrbsPhase(PRBS_SERVICE);
			break;
		case PRBS_SERVICE_WAIT:
			if (s_ServiceStart == 0 || s_ServiceStart >= s_ServiceEnd)
			{
				PrbsFail("BlueIO service not found", 0);
				return;
			}
			s_DescEnd = s_ServiceEnd;
			PrbsPhase(PRBS_CHARS);
			break;
		case PRBS_CHARS_WAIT:
			if (s_Value == 0 || s_Value >= s_DescEnd)
			{
				PrbsFail("BlueIO notify characteristic not found", 0);
				return;
			}
			PrbsPhase(PRBS_DESCRIPTORS);
			break;
		case PRBS_DESCRIPTORS_WAIT:
			if (s_Cccd == 0)
			{
				PrbsFail("CCCD not found", 0);
				return;
			}
			PrbsPhase(PRBS_SUBSCRIBE);
			break;
		case PRBS_SUBSCRIBE_WAIT:
			LOG_INFO_APP("Subscribed: PHY tx=%u rx=%u MTU=%u LL tx=%u rx=%u\n",
				s_TxPhy, s_RxPhy, s_Mtu, s_TxOctets, s_RxOctets);
			PrbsPhase(PRBS_RECEIVE);
			break;
		default:
			break;
	}
}

static void PrbsGatt(uint16_t Code, const uint8_t *pData, uint16_t Len)
{
	uint16_t pos;
	uint16_t handle;
	uint16_t value;
	uint8_t stride;
	PrbsPacket_t *packet;

	if (Len < 2 || PrbsU16(pData) != s_Connection || s_Connection == PRBS_INVALID)
	{
		return;
	}
	switch (Code)
	{
		case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
			if (s_Phase == PRBS_MTU_WAIT && Len >= 4)
			{
				s_Mtu = PrbsU16(pData + 2);
				if (s_Mtu > 247)
				{
					s_Mtu = 247;
				}
			}
			break;
		case ACI_ATT_FIND_BY_TYPE_VALUE_RESP_VSEVT_CODE:
			if (s_Phase != PRBS_SERVICE_WAIT || Len < 3 || Len != 3 + pData[2] * 4)
			{
				break;
			}
			for (pos = 3; pos + 4 <= Len; pos += 4)
			{
				if (s_ServiceStart != 0)
				{
					PrbsFail("duplicate BlueIO service", 0);
					return;
				}
				s_ServiceStart = PrbsU16(pData + pos);
				s_ServiceEnd = PrbsU16(pData + pos + 2);
			}
			break;
		case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
			if (s_Phase != PRBS_CHARS_WAIT || Len < 4 || pData[3] != Len - 4)
			{
				break;
			}
			stride = pData[2];
			if ((stride != 7 && stride != 21) || (Len - 4) % stride != 0)
			{
				PrbsFail("malformed characteristic response", 0);
				return;
			}
			for (pos = 4; pos + stride <= Len; pos += stride)
			{
				handle = PrbsU16(pData + pos);
				value = PrbsU16(pData + pos + 3);
				if (handle < s_ServiceStart || value <= handle || value > s_ServiceEnd)
				{
					PrbsFail("invalid characteristic handles", 0);
					return;
				}
				if (s_Value != 0 && handle > s_Value && handle <= s_DescEnd)
				{
					s_DescEnd = handle - 1;
				}
				if (stride == 21 && memcmp(pData + pos + 5, s_ValueUuid, 16) == 0 &&
					(pData[pos + 2] & 0x10) != 0)
				{
					s_Value = value;
				}
			}
			break;
		case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
			if (s_Phase != PRBS_DESCRIPTORS_WAIT || Len < 4 || pData[3] != Len - 4)
			{
				break;
			}
			stride = pData[2] == 1 ? 4 : 18;
			if ((pData[2] != 1 && pData[2] != 2) || (Len - 4) % stride != 0)
			{
				PrbsFail("malformed descriptor response", 0);
				return;
			}
			for (pos = 4; pos + stride <= Len; pos += stride)
			{
				handle = PrbsU16(pData + pos);
				if (stride == 4 && handle > s_Value && handle <= s_DescEnd &&
					PrbsU16(pData + pos + 2) == 0x2902)
				{
					s_Cccd = handle;
				}
			}
			break;
		case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
			if (Len >= 3)
			{
				PrbsGattComplete(pData[2]);
			}
			break;
		case ACI_GATT_NOTIFICATION_VSEVT_CODE:
			if (s_Final || (s_Phase != PRBS_RECEIVE && s_Phase != PRBS_SUBSCRIBE_WAIT) ||
				Len < 5 || PrbsU16(pData + 2) != s_Value)
			{
				break;
			}
			if (pData[4] == 0 || pData[4] > PRBS_PAYLOAD || pData[4] != Len - 5 ||
				s_Count == PRBS_QUEUE_COUNT)
			{
				s_Dropped++;
				PrbsFail("notification malformed or queue full", 0);
				return;
			}
			packet = &s_Queue[s_Head];
			packet->Time = UTIL_TIMER_GetCurrentTime();
			packet->Len = pData[4];
			memcpy(packet->Data, pData + 5, packet->Len);
			s_Head = (s_Head + 1) % PRBS_QUEUE_COUNT;
			s_Count++;
			UTIL_SEQ_SetTask(1U << PRBS_RX_TASK, CFG_SEQ_PRIO_0);
			break;
		default:
			break;
	}
}

static SVCCTL_EvtAckStatus_t PrbsEvent(void *pEvent)
{
	const uint8_t *data;
	uint16_t len;
	uint16_t code;

	/* Called from ST's Ble_UserEvtRx task, outside BleStack_Process. */
	data = (const uint8_t *)pEvent;
	len = data[2];
	code = data[1];
	data += 3;
	if (code == HCI_DISCONNECTION_COMPLETE_EVT_CODE && len >= 4 &&
		PrbsU16(data + 1) == s_Connection)
	{
		LOG_INFO_APP("Disconnected hdl=%u status=0x%02x reason=0x%02x\n", s_Connection, data[0], data[3]);
		s_Connection = PRBS_INVALID;
		if (s_Phase == PRBS_DISCONNECT_WAIT && s_Complete && s_Phy == 1 && data[0] == 0)
		{
			s_Phy = 2;
			PrbsReset();
		}
		else if (!s_Final)
		{
			PrbsFail("disconnected before trial completion", data[3]);
		}
		else
		{
			PrbsPhase(PRBS_DONE);
		}
	}
	else if (code == HCI_LE_META_EVT_CODE && len >= 1)
	{
		code = *data++;
		len--;
		switch (code)
		{
			case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
				PrbsAdvertising(data, len);
				break;
			case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
				PrbsConnected(data, len, true);
				break;
			case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
				PrbsConnected(data, len, false);
				break;
			case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
				if (len >= 5 && PrbsU16(data + 1) == s_Connection)
				{
					s_TxPhy = data[3];
					s_RxPhy = data[4];
					if (data[0] != 0 || s_TxPhy != s_Phy || s_RxPhy != s_Phy)
					{
						PrbsFail("PHY update failed or differs from trial", data[0]);
					}
					else if (s_Phase == PRBS_PHY_WAIT)
					{
						PrbsPhase(PRBS_DLE);
					}
				}
				break;
			case HCI_LE_DATA_LENGTH_CHANGE_SUBEVT_CODE:
				if (len >= 10 && PrbsU16(data) == s_Connection)
				{
					s_TxOctets = PrbsU16(data + 2);
					s_RxOctets = PrbsU16(data + 6);
					if (s_Phase == PRBS_DLE_WAIT && s_RxOctets >= 251)
					{
						PrbsPhase(PRBS_MTU);
					}
				}
				break;
			case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
				if (len >= 9 && PrbsU16(data + 1) == s_Connection &&
					(data[0] != 0 || PrbsU16(data + 3) != 12 || PrbsU16(data + 5) != 0))
				{
					PrbsFail("connection interval changed", data[0]);
				}
				break;
			default:
				break;
		}
	}
	else if (code == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE && len >= 2)
	{
		code = PrbsU16(data);
		data += 2;
		len -= 2;
		if (code == ACI_GAP_PROC_COMPLETE_VSEVT_CODE && len >= 2 &&
			data[0] == GAP_GENERAL_DISCOVERY_PROC && s_Phase == PRBS_SCAN_STOPPING)
		{
			if (data[1] != 0)
			{
				PrbsFail("scan termination failed", data[1]);
			}
			else
			{
				PrbsPhase(PRBS_CONNECT);
			}
		}
		else
		{
			PrbsGatt(code, data, len);
		}
	}
	/* Own the application events; do not run ST's interactive DT client. */
	return SVCCTL_EvtAckFlowEnable;
}

void PrbsClientInit(void)
{
	UTIL_SEQ_RegTask(1U << PRBS_TASK, UTIL_SEQ_RFU, PrbsWork);
	UTIL_SEQ_RegTask(1U << PRBS_RX_TASK, UTIL_SEQ_RFU, PrbsReceive);
	SVCCTL_RegisterHandler(PrbsEvent);
	SVCCTL_RegisterCltHandler(PrbsEvent);
	LOG_INFO_APP("ST PRBS comparison: BlePrbs_SDC; 60 s at 1M then 60 s at 2M\n");
	LOG_INFO_APP("Interval=15 ms MTU=247; firmware PRBS, UART results only\n");
#ifdef __OPTIMIZE__
	LOG_INFO_APP("Compiler optimization: enabled\n");
#else
	LOG_INFO_APP("Compiler optimization: disabled - use Release for comparison\n");
#endif
	s_Phy = 1;
	s_Connection = PRBS_INVALID;
	PrbsReset();
	if (UTIL_TIMER_Create(&s_Timer, 20, UTIL_TIMER_PERIODIC, PrbsTick, NULL) != UTIL_TIMER_OK ||
		UTIL_TIMER_Start(&s_Timer) != UTIL_TIMER_OK)
	{
		PrbsFail("ST timer initialization failed", 0);
	}
}
