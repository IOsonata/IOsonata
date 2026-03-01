/**-------------------------------------------------------------------------
@file	bt_app_bm.cpp

@brief	Bluetooth application creation helper using baremetal SDK (S145)

This implementation targets the nRF54L15 with SoftDevice S145 via the
sdk-nrf-bm bare-metal framework.  It replaces both the legacy nRF5_SDK
(bt_app_nrf52) and the SDC/MPSL (bt_app_sdc) paths with a thin layer
over the S145 sd_ble_* SVC API, wired through the sdk-nrf-bm observer
infrastructure (nrf_sdh / nrf_sdh_ble).

@author	Hoang Nguyen Hoan
@date	Feb. 13, 2026

@license

MIT License

Copyright (c) 2026 I-SYST INC, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <atomic>
#include <stdlib.h>
#include <string.h>

#include "nrf_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "ble_gattc.h"

#include "nrf_sdm.h"
#include "bm/softdevice_handler/nrf_sdh.h"
#include "bm/softdevice_handler/nrf_sdh_ble.h"
#include "bm/softdevice_handler/nrf_sdh_soc.h"
#include "bm/bluetooth/ble_conn_state.h"
#include "bm/bluetooth/ble_conn_params.h"
#include "bm/bluetooth/services/ble_dis.h"
#include "nrfx_cracen.h"

#include "nrf_soc.h"
//#include "mpsl.h"

#include "istddef.h"
#include "idelay.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_dev.h"
#include "app_evt_handler.h"

extern "C" bool sdh_state_evt_observer_notify(enum nrf_sdh_state_evt state);

/******** For DEBUG ************/
#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#ifdef NDEBUG
#define DEBUG_PRINTF(...)
#else
#define DEBUG_PRINTF(...)		printf(__VA_ARGS__)
#endif
#endif
/*******************************/

/// Unit conversion macros (same as old nRF5_SDK definitions)
#define UNIT_0_625_MS					625			/**< Number of microseconds in 0.625 milliseconds. */
#define UNIT_10_MS						10000		/**< Number of microseconds in 10 milliseconds. */
#define MSEC_TO_UNITS(MS, UNIT)			(((MS) * 1000) / (UNIT))

#define BTAPP_CONN_CFG_TAG				CONFIG_NRF_SDH_BLE_CONN_TAG

#define BTAPP_OBSERVER_PRIO				USER		/**< Application's BLE observer priority. */

#ifndef GATT_MTU_SIZE_DEFAULT
#if defined(BLE_GATT_ATT_MTU_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT			BLE_GATT_ATT_MTU_DEFAULT
#elif defined(BLE_GATT_MTU_SIZE_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT			BLE_GATT_MTU_SIZE_DEFAULT
#else
#define GATT_MTU_SIZE_DEFAULT			23
#endif
#endif

#define SEC_PARAM_MIN_KEY_SIZE			7			/**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE			16			/**< Maximum encryption key size. */

// S145 tx_power values (nRF54L15)
static const int8_t s_TxPowerdBm[] = {
	-40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8
};
static const int s_NbTxPowerdBm = sizeof(s_TxPowerdBm) / sizeof(int8_t);

#pragma pack(push, 4)

typedef struct __Bt_App_Data {
	BTAPP_STATE State;
	BTAPP_ROLE Role;
	uint8_t AdvHdl;		///< Advertisement handle
	uint16_t ConnHdl;	///< BLE connection handle
	int ConnLedPort;
	int ConnLedPin;
	uint8_t ConnLedActLevel;
	uint16_t VendorId;
	uint16_t ProductId;
	uint16_t ProductVer;
	uint16_t Appearance;
	ble_gap_adv_params_t AdvParam;
	void (*SDEvtHandler)(void) ;
	int MaxMtu;
	bool bSecure;
	bool bExtAdv;
	bool bScan;
} BtAppData_t;

#pragma pack(pop)

static BtAppData_t s_BtAppData = {
	BTAPP_STATE_UNKNOWN, BTAPP_ROLE_PERIPHERAL,
	BLE_GAP_ADV_SET_HANDLE_NOT_SET, BLE_CONN_HANDLE_INVALID,
};

// --- Advertisement packet buffers ---

alignas(4) static uint8_t s_BtAppAdvBuff[256];
alignas(4) static BtAdvPacket_t s_BtAppAdvPkt = { 31, 0, s_BtAppAdvBuff };
alignas(4) static BtAdvPacket_t s_BtAppExtAdvPkt = { 255, 0, s_BtAppAdvBuff };

alignas(4) static uint8_t s_BtAppSrBuff[256];
alignas(4) static BtAdvPacket_t s_BtAppSrPkt = { 31, 0, s_BtAppSrBuff };
alignas(4) static BtAdvPacket_t s_BtAppExtSrPkt = { 255, 0, s_BtAppSrBuff };

static ble_gap_adv_data_t s_BtAppAdvData = {
	.adv_data = { s_BtAppAdvBuff, 0 },
	.scan_rsp_data = { s_BtAppSrBuff, 0 }
};


const static TimerCfg_t s_BtAppSdTimerCfg = {
    .DevNo = 3,	// GRTC3 needed for Softdevice
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 0,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static Timer s_BtAppSdGrtc3;

// --- Helper functions ---

bool BtInitialized()
{
	return s_BtAppData.State != BTAPP_STATE_UNKNOWN;
}

bool BtConnected()
{
	return s_BtAppData.ConnHdl != BLE_CONN_HANDLE_INVALID;
}

bool isConnected()
{
	return s_BtAppData.ConnHdl != BLE_CONN_HANDLE_INVALID;
}

uint16_t BtAppGetConnHandle()
{
	return s_BtAppData.ConnHdl;
}

int8_t GetValidTxPower(int TxPwr)
{
	int8_t retval = s_TxPowerdBm[0];

	for (int i = 1; i < s_NbTxPowerdBm; i++)
	{
		if (s_TxPowerdBm[i] > TxPwr)
			break;
		retval = s_TxPowerdBm[i];
	}

	return retval;
}

// --- Connection LED helpers ---

static void BtAppConnLedOff()
{
	if (s_BtAppData.ConnLedPort < 0 || s_BtAppData.ConnLedPin < 0)
		return;

	if (s_BtAppData.ConnLedActLevel)
		IOPinClear(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
	else
		IOPinSet(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
}

static void BtAppConnLedOn()
{
	if (s_BtAppData.ConnLedPort < 0 || s_BtAppData.ConnLedPin < 0)
		return;

	if (s_BtAppData.ConnLedActLevel)
		IOPinSet(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
	else
		IOPinClear(s_BtAppData.ConnLedPort, s_BtAppData.ConnLedPin);
}

// --- Conn params event handler ---

static void on_conn_params_evt(const struct ble_conn_params_evt *p_evt)
{
	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_ERROR)
	{
		sd_ble_gap_disconnect(s_BtAppData.ConnHdl,
							  BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
	}
}

// --- BLE event dispatch (registered as observer) ---

static void ble_evt_dispatch(const ble_evt_t *p_ble_evt, void *p_context)
{
	uint32_t err_code;
	const ble_gap_evt_t *p_gap_evt = &p_ble_evt->evt.gap_evt;
	//uint8_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);
	uint8_t role = s_BtAppData.Role;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			BtAppConnLedOn();
			BtGapAddConnection(p_gap_evt->conn_handle, role,
							   p_gap_evt->params.connected.peer_addr.addr_type,
							   (uint8_t *)p_gap_evt->params.connected.peer_addr.addr);
			s_BtAppData.ConnHdl = p_ble_evt->evt.gap_evt.conn_handle;
			s_BtAppData.State = BTAPP_STATE_CONNECTED;
			BtAppEvtConnected(p_ble_evt->evt.gap_evt.conn_handle);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			BtAppConnLedOff();
			s_BtAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
			s_BtAppData.State = BTAPP_STATE_IDLE;
			BtAppEvtDisconnected(p_ble_evt->evt.gap_evt.conn_handle);
			if (s_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
			{
				BtAdvStart();
			}
			break;

		case BLE_GAP_EVT_ADV_SET_TERMINATED:
			BtAppAdvTimeoutHandler();
			break;

		case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
		{
			const ble_gap_phys_t phys = {
				BLE_GAP_PHY_AUTO,
				BLE_GAP_PHY_AUTO,
			};
			err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
			(void)err_code;
		}
		break;

		case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
		{
			ble_gap_data_length_params_t dl_params;
			memset(&dl_params, 0, sizeof(dl_params));
			err_code = sd_ble_gap_data_length_update(
				p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
			(void)err_code;
		}
		break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			err_code = sd_ble_gatts_sys_attr_set(
				s_BtAppData.ConnHdl, NULL, 0, 0);
			(void)err_code;
			break;

		case BLE_GATTC_EVT_TIMEOUT:
			sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
								  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			break;

		case BLE_GATTS_EVT_TIMEOUT:
			sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
								  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			break;

		case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
		{
			uint16_t mtu = s_BtAppData.MaxMtu;
			err_code = sd_ble_gatts_exchange_mtu_reply(
				p_ble_evt->evt.gatts_evt.conn_handle, mtu);
			(void)err_code;
		}
		break;

		default:
			break;
	}

	// Forward to central/observer handlers
	if ((role == BLE_GAP_ROLE_CENTRAL) ||
		(s_BtAppData.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER)))
	{
		switch (p_ble_evt->header.evt_id)
		{
			case BLE_GAP_EVT_ADV_REPORT:
			{
				const ble_gap_evt_adv_report_t *p_adv_report =
					&p_gap_evt->params.adv_report;
				BtAppScanReport(p_adv_report->rssi,
								p_adv_report->peer_addr.addr_type,
								(uint8_t *)p_adv_report->peer_addr.addr,
								p_adv_report->data.len,
								p_adv_report->data.p_data);
			}
			break;

			case BLE_GAP_EVT_TIMEOUT:
			{
				const ble_gap_evt_timeout_t *p_timeout =
					&p_gap_evt->params.timeout;
				if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
				{
					s_BtAppData.bScan = false;
				}
			}
			break;

			default:
				break;
		}
		BtAppCentralEvtHandler((uint32_t)p_ble_evt->header.evt_id, (void *)p_ble_evt);
	}

	// Forward to peripheral handlers
	if (s_BtAppData.Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtGattEvtHandler((uint32_t)p_ble_evt->header.evt_id, (void *)p_ble_evt);
		BtAppPeriphEvtHandler((uint32_t)p_ble_evt->header.evt_id, (void *)p_ble_evt);
	}
}

// Note: SoC event polling is handled internally by nrf_sdh_soc.c
// which registers its own NRF_SDH_STACK_EVT_OBSERVER.

// --- Advertising ---

void BtAdvStart()
{
	if (s_BtAppData.State == BTAPP_STATE_ADVERTISING ||
		s_BtAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
	{
		return;
	}

	uint32_t err_code = sd_ble_gap_adv_start(s_BtAppData.AdvHdl, BTAPP_CONN_CFG_TAG);
	if (err_code == NRF_SUCCESS)
	{
		s_BtAppData.State = BTAPP_STATE_ADVERTISING;
	}
	else
	{
		DEBUG_PRINTF("BtAdvStart failed: 0x%x\r\n", err_code);
	}
}

void BtAdvStop()
{
	sd_ble_gap_adv_stop(s_BtAppData.AdvHdl);
	s_BtAppData.State = BTAPP_STATE_IDLE;
}

/**
 * @brief Overloadable advertising initialization
 */
__attribute__((weak)) bool BtAppAdvInit(const BtAppCfg_t *pCfg)
{
	uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;
	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	memset(&s_BtAppData.AdvParam, 0, sizeof(ble_gap_adv_params_t));

	if (s_BtAppData.bExtAdv)
	{
		advpkt = &s_BtAppExtAdvPkt;
		srpkt = &s_BtAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BtAppAdvPkt;
		srpkt = &s_BtAppSrPkt;
	}

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		if (pCfg->AdvTimeout != 0)
			flags |= BT_GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
		else
			flags |= BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE;

		s_BtAppData.AdvParam.properties.type = pCfg->bExtAdv ?
			BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED :
			BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	}
	else if (pCfg->Role & BTAPP_ROLE_BROADCASTER)
	{
		s_BtAppData.AdvParam.properties.type = pCfg->bExtAdv ?
			BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED :
			BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
	}

	if (BtAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1) == false)
	{
		return false;
	}

	if (pCfg->Appearance != BT_APPEAR_UNKNOWN_GENERIC)
	{
		// Appearance is optional - don't fail if it doesn't fit
		BtAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_APPEARANCE,
					 (uint8_t *)&pCfg->Appearance, 2);
	}

	// Manufacturer specific data in adv packet
	if (pCfg->bExtAdv == false)
	{
		if (pCfg->pAdvManData != NULL)
		{
			int l = pCfg->AdvManDataLen + 2;
			BtAdvData_t *p = BtAdvDataAllocate(advpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
			if (p)
			{
				*(uint16_t *)p->Data = pCfg->VendorId;
				memcpy(&p->Data[2], pCfg->pAdvManData, pCfg->AdvManDataLen);
			}
		}

		if (pCfg->pSrManData != NULL)
		{
			int l = pCfg->SrManDataLen + 2;
			BtAdvData_t *p = BtAdvDataAllocate(srpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
			if (p)
			{
				*(uint16_t *)p->Data = pCfg->VendorId;
				memcpy(&p->Data[2], pCfg->pSrManData, pCfg->SrManDataLen);
			}
		}
	}
	else
	{
		// Extended advertising: combine manuf data into single adv packet
		int l = 2;
		if (pCfg->pAdvManData)
			l += pCfg->AdvManDataLen;
		if (pCfg->pSrManData)
			l += pCfg->SrManDataLen;

		if (l > 2)
		{
			BtAdvData_t *p = BtAdvDataAllocate(advpkt,
				BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
			if (p)
			{
				*(uint16_t *)p->Data = pCfg->VendorId;
				int off = 2;
				if (pCfg->pAdvManData)
				{
					memcpy(&p->Data[off], pCfg->pAdvManData, pCfg->AdvManDataLen);
					off += pCfg->AdvManDataLen;
				}
				if (pCfg->pSrManData)
				{
					memcpy(&p->Data[off], pCfg->pSrManData, pCfg->SrManDataLen);
				}
			}
		}
	}

	BtAdvPacket_t *uidadvpkt;

	// Device name
	if (pCfg->pDevName != NULL)
	{
		if (BtAdvDataSetDevName(advpkt, pCfg->pDevName) == false)
		{
			return false;
		}
		uidadvpkt = pCfg->bExtAdv ? advpkt : srpkt;
	}
	else
	{
		uidadvpkt = advpkt;
	}

	// Service UUIDs
	if (pCfg->pAdvUuid != NULL && (pCfg->Role & BTAPP_ROLE_PERIPHERAL))
	{
		BtAdvDataAddUuid(uidadvpkt, pCfg->pAdvUuid, pCfg->bCompleteUuidList);
	}

	// Configure adv params
	s_BtAppData.AdvParam.p_peer_addr	= NULL;
	s_BtAppData.AdvParam.interval		= MSEC_TO_UNITS(pCfg->AdvInterval, UNIT_0_625_MS);
	s_BtAppData.AdvParam.duration		= MSEC_TO_UNITS(pCfg->AdvTimeout, UNIT_10_MS);
	s_BtAppData.AdvParam.filter_policy	= BLE_GAP_ADV_FP_ANY;
	s_BtAppData.AdvParam.primary_phy	= BLE_GAP_PHY_1MBPS;
	s_BtAppData.AdvParam.secondary_phy	= BLE_GAP_PHY_2MBPS;

	s_BtAppAdvData.adv_data.len = advpkt->Len;
	s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;

	uint32_t err_code = sd_ble_gap_adv_set_configure(
		&s_BtAppData.AdvHdl, &s_BtAppAdvData, &s_BtAppData.AdvParam);

	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("sd_ble_gap_adv_set_configure failed: 0x%x\r\n", err_code);
		return false;
	}

	return true;
}

bool BtAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
	if (s_BtAppData.State != BTAPP_STATE_ADVERTISING &&
		s_BtAppData.State != BTAPP_STATE_IDLE)
	{
		return false;
	}

	BtAdvPacket_t *advpkt;
	BtAdvPacket_t *srpkt;

	if (s_BtAppData.bExtAdv)
	{
		advpkt = &s_BtAppExtAdvPkt;
		srpkt = &s_BtAppExtSrPkt;
	}
	else
	{
		advpkt = &s_BtAppAdvPkt;
		srpkt = &s_BtAppSrPkt;
	}

	if (pAdvData)
	{
		int l = AdvLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(advpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
		if (p == NULL)
			return false;
		*(uint16_t *)p->Data = s_BtAppData.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);
		s_BtAppAdvData.adv_data.len = advpkt->Len;
	}

	if (pSrData)
	{
		int l = SrLen + 2;
		BtAdvData_t *p = BtAdvDataAllocate(srpkt,
			BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);
		if (p == NULL)
			return false;
		*(uint16_t *)p->Data = s_BtAppData.VendorId;
		memcpy(&p->Data[2], pSrData, SrLen);
		s_BtAppAdvData.scan_rsp_data.len = srpkt->Len;
	}

	// SDK15+ doesn't allow dynamic adv data update: stop and reconfigure
	if (s_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		sd_ble_gap_adv_stop(s_BtAppData.AdvHdl);
	}

	sd_ble_gap_adv_set_configure(&s_BtAppData.AdvHdl, &s_BtAppAdvData, NULL);

	if (s_BtAppData.State == BTAPP_STATE_ADVERTISING)
	{
		s_BtAppData.State = BTAPP_STATE_IDLE;
		BtAdvStart();
	}

	return s_BtAppData.State == BTAPP_STATE_ADVERTISING;
}

// --- Notify ---

bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (s_BtAppData.ConnHdl == BLE_CONN_HANDLE_INVALID)
		return false;

	BtGattCharSetValue(pChar, pData, DataLen);

	if (pChar->bNotify == false)
		return false;

	ble_gatts_hvx_params_t params;
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = pChar->ValHdl;
	params.p_data = pData;
	params.p_len = &DataLen;

	sd_ble_gatts_hvx(s_BtAppData.ConnHdl, &params);

	return true;
}

// --- Disconnect ---

void BtAppDisconnect()
{
	if (s_BtAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
	{
		uint32_t err_code = sd_ble_gap_disconnect(
			s_BtAppData.ConnHdl,
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code == NRF_ERROR_INVALID_STATE)
		{
			s_BtAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
		}
	}
}

void BtAppGapDeviceNameSet(const char *pDeviceName)
{
	BtGapSetDevName(pDeviceName);
}

void BtAppEnterDfu()
{
	// Not supported on nRF54L15 S145 yet
	NVIC_SystemReset();
}

// --- DIS initialization ---

static void BtDisInit(const BtAppCfg_t *pCfg)
{
	// sdk-nrf-bm DIS uses Kconfig values for device info strings.
	// Call ble_dis_init() with security settings based on SecType.
	// Note: BLE_DIS_CONFIG_SEC_MODE_DEFAULT uses C nested designated
	// initializers (.device_info_char.read = ...) which are not valid
	// in C++. Initialize manually.
	struct ble_dis_config dis_cfg = {};
	dis_cfg.sec_mode.device_info_char.read.sm = 1;
	dis_cfg.sec_mode.device_info_char.read.lv = 1;

	switch (pCfg->SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
		case BTGAP_SECTYPE_LESC_MITM:
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			// Just works / signed
			break;
		case BTGAP_SECTYPE_NONE:
		default:
			// Open access
			break;
	}

	ble_dis_init(&dis_cfg);
}

// --- Stack Init ---


__attribute__((weak)) void SoftdeviceFaultHandler(uint32_t id, uint32_t pc, uint32_t info)
{
	DEBUG_PRINTF("SoftDevice fault! ID %#x, PC %#x, Info %#x", id, pc, info);

	switch (id) {
	case NRF_FAULT_ID_SD_ASSERT:
		DEBUG_PRINTF("NRF_FAULT_ID_SD_ASSERT: SoftDevice assert");
		break;
	case NRF_FAULT_ID_APP_MEMACC:
		DEBUG_PRINTF("NRF_FAULT_ID_APP_MEMACC: Application bad memory access");
		if (info == 0x00) {
			DEBUG_PRINTF("Application tried to access SoftDevice RAM");
		} else {
			DEBUG_PRINTF("Application tried to access SoftDevice peripheral at %#x", info);
		}
		break;
	}

	for (;;) {
		/* loop */
	}
}

/* Extern in nrf_sdh.c (called directly at enable time for scheduler model) */
void SDBleRandSeed(uint32_t evt, void *ctx)
{
	uint32_t nrf_err;
	uint8_t seed[SD_RAND_SEED_SIZE];

	(void)ctx;

	if (evt == NRF_EVT_RAND_SEED_REQUEST)
	{
		if (nrfx_cracen_entropy_get(seed, sizeof(seed)) != 0)
		{
			return;
		}

		nrf_err = sd_rand_seed_set(seed);
		(void)nrf_err;
	}
}

static uint32_t SDBleDefaultCfgSet(const BtAppCfg_t *pCfg, uint32_t ConnCfgTag, uint32_t RamStart)
{
	uint32_t err = 0;
	ble_cfg_t ble_cfg;

	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag = ConnCfgTag;
	ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count = pCfg->PeriLinkCount + pCfg->CentLinkCount; //CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT;
	ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = CONFIG_NRF_SDH_BLE_GAP_EVENT_LENGTH;

	err = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, RamStart);
	if (err)
	{
		DEBUG_PRINTF("Failed to set BLE_CONN_CFG_GAP, nrf_error %#x", err);

		return err;
	}

	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.gap_cfg.role_count_cfg.periph_role_count = pCfg->PeriLinkCount;

	// TODO: make this configurable
	ble_cfg.gap_cfg.role_count_cfg.adv_set_count = BLE_GAP_ADV_SET_COUNT_DEFAULT;

	ble_cfg.gap_cfg.role_count_cfg.central_role_count = pCfg->CentLinkCount;
	ble_cfg.gap_cfg.role_count_cfg.central_sec_count =
		MIN(pCfg->CentLinkCount, BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT);

	err = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, RamStart);
	if (err)
	{
		DEBUG_PRINTF("Failed to set BLE_GAP_CFG_ROLE_COUNT, nrf_error %#x", err);

		return err;
	}

	// Max MTU
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag = ConnCfgTag;
	ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = pCfg->MaxMtu > 0 ? pCfg->MaxMtu : CONFIG_NRF_SDH_BLE_GATT_MAX_MTU_SIZE;

	err = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, RamStart);
	if (err)
	{
		DEBUG_PRINTF("Failed to set BLE_CONN_CFG_GATT, nrf_error %#x", err);

		return err;
	}

	// Custom UUID
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 2;//CONFIG_NRF_SDH_BLE_VS_UUID_COUNT;

	err = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, RamStart);
	if (err)
	{
		DEBUG_PRINTF("Failed to set BLE_COMMON_CFG_VS_UUID, nrf_error %#x", err);

		return err;
	}

	// Configure the GATTS attribute table.
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = CONFIG_NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE;

	err = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, RamStart);
	if (err)
	{
		DEBUG_PRINTF("Failed to set BLE_GATTS_CFG_ATTR_TAB_SIZE, nrf_error %#x", err);

		return err;
	}

	// Configure Service Changed characteristic.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.gatts_cfg.service_changed.service_changed =
		IS_ENABLED(CONFIG_NRF_SDH_BLE_SERVICE_CHANGED);

	err = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, RamStart);
	if (err)
	{
		DEBUG_PRINTF("Failed to set BLE_GATTS_CFG_SERVICE_CHANGED, nrf_error %#x", err);

		return err;
	}


	return err;
}

/**
 * @brief Initialize the SoftDevice BLE stack.
 *
 * In the bm framework, nrf_sdh.c handles the clock configuration
 * automatically from g_McuOsc, and nrf_sdh_ble.c handles the BLE
 * configuration from CONFIG_ defaults.  This function just
 * requests SD enable and then enables the BLE stack.
 */
bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	uint32_t err;

	DEBUG_PRINTF("BtAppStackInit: nrf_sdh_enable_request\r\n");

	// GRTC3 must be enabled with interrupt disbled before calling nrf_sdh_enable_request
	s_BtAppSdGrtc3.Init(s_BtAppSdTimerCfg);

	// Enable SoftDevice
	err = nrf_sdh_enable_request();
	if (err && err != -EALREADY)
	{
		DEBUG_PRINTF("nrf_sdh_enable_request failed: %d\r\n", err);
		return false;
	}

	uint32_t ramstart = (uint32_t)SystemRamStart();

	err = SDBleDefaultCfgSet(pCfg, BTAPP_CONN_CFG_TAG, ramstart);

	if (err != NRF_SUCCESS)
	{
		return false;
	}

	// Require before call to nrf_sdh_ble_enable
	SDBleRandSeed(NRF_EVT_RAND_SEED_REQUEST, NULL);

	DEBUG_PRINTF("BtAppStackInit: nrf_sdh_ble_enable\r\n");

	uint32_t ramreq = ramstart;

	err = sd_ble_enable(&ramreq);

	if (ramreq > ramstart)
	{
		DEBUG_PRINTF("%x - Insufficient RAM allocated for the SoftDevice need %x", err, ramreq);
	}

	(void)sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_BLE_ENABLED);

	// Enable BLE stack (cfg_set from CONFIG_ defaults, handled in nrf_sdh_ble.c)
	//err = nrf_sdh_ble_enable(BTAPP_CONN_CFG_TAG);
	//if (err)
	//{
//		DEBUG_PRINTF("nrf_sdh_ble_enable failed: %d\r\n", err);
//		return false;
//	}

	// Initialize connection state tracking
	//ble_conn_state_init();

	return true;
}

/**
 * @brief Initialize the Bluetooth application.
 *
 * This function initializes the SoftDevice, BLE stack, GAP parameters,
 * GATT services, advertising, and security.
 */
bool BtAppInit(const BtAppCfg_t *pCfg)
{
	uint32_t err_code;

	// Initialize application event handler
	if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem, pCfg->EvtHandlerQueMemSize) == false)
	{
		return false;
	}

	// Populate internal app data from config
	s_BtAppData.Role = pCfg->Role;
	s_BtAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
	s_BtAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
	s_BtAppData.bExtAdv = pCfg->bExtAdv;
	s_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	s_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	s_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;
	s_BtAppData.bScan = false;
	s_BtAppData.VendorId = pCfg->VendorId;
	s_BtAppData.ProductId = pCfg->ProductId;
	s_BtAppData.ProductVer = pCfg->ProductVer;
	s_BtAppData.Appearance = pCfg->Appearance;

	s_BtAppData.MaxMtu = GATT_MTU_SIZE_DEFAULT;
	if (pCfg->MaxMtu > GATT_MTU_SIZE_DEFAULT)
		s_BtAppData.MaxMtu = pCfg->MaxMtu;

	// Setup connection LED
	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
	{
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		BtAppConnLedOff();
	}

	// Initialize SoftDevice + BLE stack
	if (BtAppStackInit(pCfg) == false)
	{
		DEBUG_PRINTF("BtAppStackInit failed\r\n");
		return false;
	}


	// Set GAP appearance
	err_code = sd_ble_gap_appearance_set(pCfg->Appearance);
	(void)err_code;

	// Initialize GAP parameters
	BtGapCfg_t gapcfg = {
		.Role = pCfg->Role,
		.SecType = pCfg->SecType,
		.AdvInterval = pCfg->AdvInterval,
		.AdvTimeout = pCfg->AdvTimeout,
		.ConnIntervalMin = pCfg->ConnIntervalMin,
		.ConnIntervalMax = pCfg->ConnIntervalMax,
		.SlaveLatency = BT_GAP_CONN_SLAVE_LATENCY,
		.SupTimeout = BT_GAP_CONN_SUP_TIMEOUT
	};

	BtGapInit(&gapcfg);

	if (pCfg->pDevName != NULL)
	{
		BtGapSetDevName(pCfg->pDevName);
	}

	// Register connection parameters event handler
	ble_conn_params_evt_handler_set(on_conn_params_evt);

	// Initialize user services
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();
	}

	// Initialize Device Information Service
	if (pCfg->pDevInfo != NULL)
	{
		BtDisInit(pCfg);
	}

	// Initialize user data
	BtAppInitUserData();

	s_BtAppData.bSecure = pCfg->SecType != BTGAP_SECTYPE_NONE;

	// Initialize advertising
	if (s_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		if (BtAppAdvInit(pCfg) == false)
		{
			DEBUG_PRINTF("BtAppAdvInit failed\r\n");
			return false;
		}

		err_code = sd_ble_gap_tx_power_set(
			BLE_GAP_TX_POWER_ROLE_ADV,
			s_BtAppData.AdvHdl,
			GetValidTxPower(pCfg->TxPower));
		(void)err_code;
	}
	else
	{
		err_code = sd_ble_gap_tx_power_set(
			BLE_GAP_TX_POWER_ROLE_SCAN_INIT,
			0,
			GetValidTxPower(pCfg->TxPower));
		(void)err_code;
	}

	s_BtAppData.State = BTAPP_STATE_INITIALIZED;

	DEBUG_PRINTF("BtAppInit: success\r\n");

	return true;
}

/**
 * @brief Main BLE application run loop.
 *
 * Starts advertising (if peripheral/broadcaster) then enters the
 * main event loop.  SoftDevice events are dispatched through the
 * observer infrastructure triggered by the SD_EVT interrupt.
 */
void BtAppRun()
{
	if (s_BtAppData.State != BTAPP_STATE_INITIALIZED)
	{
		return;
	}

	if (s_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAdvStart();
	}

	DEBUG_PRINTF("BtAppRun: entering main loop\r\n");

	while (1)
	{
    	if (s_BtAppData.SDEvtHandler != NULL)
		{
    		// TODO
			//BtAppRtosWaitEvt();
		}
		else
		{
			AppEvtHandlerExec();

			// Wait for event (low power sleep)
			// bm framework uses __WFE() directly, not sd_app_evt_wait()
			__WFE();
			//__SEV();
			//__WFE();
		}
	}
}

static void soc_evt_poll(void *context)
{
	uint32_t nrf_err;
	uint32_t evt_id;

	DEBUG_PRINTF("soc_evt_poll\r\n");
	while (true) {
		nrf_err = sd_evt_get(&evt_id);
		if (nrf_err) {
			break;
		}

		/* Forward the event to SoC observers. */
		TYPE_SECTION_FOREACH(
			struct nrf_sdh_soc_evt_observer, nrf_sdh_soc_evt_observers, obs) {
			obs->handler(evt_id, obs->context);
		}
	}

	__ASSERT(nrf_err == NRF_ERROR_NOT_FOUND,
		 "Failed to receive SoftDevice SoC event, nrf_error %#x", nrf_err);
}

static void ble_evt_poll(void *context)
{
	int err;

	__aligned(4) static uint8_t evt_buffer[NRF_SDH_BLE_EVT_BUF_SIZE];
	ble_evt_t * const ble_evt = (ble_evt_t *)evt_buffer;

	DEBUG_PRINTF("ble_evt_poll evt %x\r\n", ble_evt->header.evt_id);

	while (true) {
		uint16_t evt_len = (uint16_t)sizeof(evt_buffer);

		err = sd_ble_evt_get(evt_buffer, &evt_len);
		if (err) {
			break;
		}


		//DEBUG_PRINTF()("%s", nrf_sdh_ble_evt_to_str(ble_evt->header.evt_id));

		ble_evt_dispatch(ble_evt, context);

#if 0
		if (ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED) {
			idx_assign(ble_evt->evt.gap_evt.conn_handle);
		}

		/* Forward the event to BLE observers. */
		TYPE_SECTION_FOREACH(
			struct nrf_sdh_ble_evt_observer, nrf_sdh_ble_evt_observers, obs) {
			obs->handler(ble_evt, obs->context);
		}

		if (ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED) {
			idx_unassign(ble_evt->evt.gap_evt.conn_handle);
		}
#endif
	}

	/* An SoC event may have triggered this round of polling, and BLE may not be enabled */
//	__ASSERT((err == NRF_ERROR_NOT_FOUND) || (err == BLE_ERROR_NOT_ENABLED),
//		 "Failed to receive SoftDevice BLE event, nrf_error %#x", err);
}

/* Auto-handle seed requests as a SoC event observer */
NRF_SDH_SOC_OBSERVER(rand_seed, SDBleRandSeed, NULL, HIGH);
/* Listen to SoftDevice events */
NRF_SDH_STACK_EVT_OBSERVER(soc_evt_obs, soc_evt_poll, NULL, HIGHEST);
NRF_SDH_STACK_EVT_OBSERVER(ble_evt_obs, ble_evt_poll, NULL, HIGH);
// Register as BLE event observer
NRF_SDH_BLE_OBSERVER(s_BtAppBleObserver, ble_evt_dispatch, NULL, USER);


