/**-------------------------------------------------------------------------
@file	bt_app_sdc.cpp

@brief	Bluetooth application creation helper using softdevice controller


@author	Hoang Nguyen Hoan
@date	Mar. 8, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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

//#include "mpsl.h"
//#include "mpsl_fem_init.h"
#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_cmd_le.h"
#include "sdc_hci.h"
#include "sdc_hci_vs.h"
#include "sdc_hci_cmd_controller_baseband.h"
#include "sdc_hci_cmd_link_control.h"

#include "istddef.h"
#include "convutil.h"
#include "nrf_mac.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"		// BtSmpLocalAddrGet override
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_appearance.h"
#include "bt_pds_sdc.h"				// BtSmpBondSdcInit (flash-backed bond persistence)
#include "nrf_mpsl.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"

#define BT_SDC_RX_MAX_PACKET_COUNT			2
#define BT_SDC_TX_MAX_PACKET_COUNT			3


/******** For DEBUG ************/
#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

static inline uint32_t BtAppSendData(void *pData, uint32_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
}
void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt);
void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]);
void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason);
void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);
bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *DavData);

static void BtAppSdcTimerHandler(TimerDev_t * const pTimer, uint32_t Evt);


// BtAppData_t now declared in bluetooth/bt_app.h, accessed via g_BtAppData.
// SDC port has no SDK-specific state in BtAppData_t scope; everything that
// remains here is already in the cross-arch struct.

// g_BtAppData definition and helpers (isConnected, BtAppConnLedOff/On) moved to
// src/bluetooth/bt_app.cpp.

// On-air local address SMP needs for c1/f5/f6 (set when we configure the
// random static address below). Defaults to public/zero until then.
static uint8_t s_BtSmpLocalAddr[6] = {0};
static uint8_t s_BtSmpLocalAddrType = 0;

// Override the weak SMP accessor so the toolbox uses our real address.
extern "C" void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = s_BtSmpLocalAddrType;
	memcpy(pAddr, s_BtSmpLocalAddr, 6);
}

// Answer the controller LE Long Term Key Request via the real SDC HCI command
// function. The generic SMP path would push this through the ACL data channel
// (sdc_hci_data_put), which the controller ignores - encryption then stalls.
extern "C" void BtSmpHciLtkReply(BtHciDevice_t * const pDev, uint16_t ConnHdl,
								 const uint8_t Ltk[16])
{
	(void)pDev;
	sdc_hci_cmd_le_long_term_key_request_reply_t cmd;
	sdc_hci_cmd_le_long_term_key_request_reply_return_t ret;
	cmd.conn_handle = ConnHdl;
	memcpy(cmd.long_term_key, Ltk, 16);
	sdc_hci_cmd_le_long_term_key_request_reply(&cmd, &ret);
}

extern "C" void BtSmpHciLtkNegReply(BtHciDevice_t * const pDev, uint16_t ConnHdl)
{
	(void)pDev;
	sdc_hci_cmd_le_long_term_key_request_negative_reply_t cmd;
	sdc_hci_cmd_le_long_term_key_request_negative_reply_return_t ret;
	cmd.conn_handle = ConnHdl;
	sdc_hci_cmd_le_long_term_key_request_negative_reply(&cmd, &ret);
}

static BtHciDevice_t s_BtHciDev = {
	.pCtx = (void*)&g_BtAppData,
	.SendData = BtAppSendData,
	.EvtHandler = BtAppEvtHandler,
	.Connected = BtAppConnected,
	.Disconnected = BtAppDisconnected,
	.SendCompleted = BtAppSendCompleted,
	.ScanReport = BtAppScanReport,
	.AdvTimeout = BtAppAdvTimeoutHandler,
	//.DiscoverDevice = BtAppDiscoverDevice,
};

/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};


alignas(8) static uint8_t s_BtStackSdcMemPool[10000];
#if 0
static BtHciDevCfg_t s_BtHciDevCfg = {
	.SendData = HciSdcSendData,
	.EvtHandler = BleAppEvtHandler,
	.ConnectedHandler = BleAppConnected,
	.DisconnectedHandler = BleAppDisconnected,
	.SendCompletedHandler = BleAppSendCompleted,
};

alignas(4) static BtHciDevice_t s_HciDevice = {
		251, 251,
		.SendData = HciSdcSendData,
		.EvtHandler = BleAppEvtHandler,
		.Connected = BleAppConnected,
		.Disconnected = BleAppDisconnected,
};
#endif

const static TimerCfg_t s_BtAppSdcTimerCfg = {
    .DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtAppSdcTimerHandler
};

static Timer g_BtAppSdcTimer;
volatile int s_SdcAclTxPktAvail = BT_SDC_TX_MAX_PACKET_COUNT + 1;

#if 0
static void BtStackMpslAssert(const char * const file, const uint32_t line)
{
	DEBUG_PRINTF("MPSL Fault: %s, %d\n", file, line);
	while(1);
}
#endif

static void BtStackSdcAssert(const char * file, const uint32_t line)
{
	DEBUG_PRINTF("SDC Fault: %s, %d\n", file, line);
	while(1);
}


static void BtStackSdcCB()
{
	// SDC invokes this from the low-priority SWI when HCI messages are
	// queued. Wake any RTOS waiter so deferred work runs promptly; the
	// weak BtAppEvtNotify default is empty so bare-metal apps see no
	// effect. Then drain EVERYTHING available - the SDC can queue several
	// messages (e.g. a command completion followed by an Encryption Change
	// event during pairing). Draining only one per callback can strand the
	// later events (encryption never completes, pairing hangs).
	BtAppEvtNotify();

	uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
	sdc_hci_msg_type_t mtype;

	while (sdc_hci_get(buf, (uint8_t*)&mtype) == 0)
	{
		switch (mtype)
		{
			case SDC_HCI_MSG_TYPE_EVT:
				BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)buf);
				break;
			case SDC_HCI_MSG_TYPE_DATA:
				BtHciProcessData(&s_BtHciDev, (BtHciACLDataPacket_t*)buf);
				break;
		}
	}
}

static void BtAppSdcTimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {

    }
}

void BtAppSetDevName(const char *pName)
{
	BtGapSetDevName(pName);
}
/*
char * const BleAppGetDevName()
{
	//return s_BtGapCharDevName;
}

*/

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt)
{

}

void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t PeerAddrType, uint8_t PeerAddr[6])
{
	// Allocate and populate the peer record in one step. BtPeerConnected
	// allocs (or reuses) the slot for ConnHdl and fills Role/PeerAddr.
	// The state machine in bt_attrsp.cpp looks the peer up by ConnHdl.
	BtDevice_t *pPeer = BtPeerConnected(ConnHdl, Role, PeerAddrType, PeerAddr);
	if (pPeer != NULL)
	{
		pPeer->pHciDev = (BtHciDevice_t*) &s_BtHciDev;
		s_BtHciDev.pBtDev = (void*) pPeer;
	}

	// Defer MTU exchange until after encryption/service discovery.
	// BtAttExchangeMtuRequest(&s_BtHciDev, ConnHdl, BtAttGetMtu());

	//DEBUG_PRINTF("This device's Role = %d\r\n", g_BtAppData.AppDevice.Conn.Role);
	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
	{
		// TODO: obtain the connected peripheral device's name and store to pPeer->Name;
		//BtAppDiscoverDevice(&s_BtHciDev, ConnHdl);
	}

	// If a secure SecType was configured, the SDC backend requests security on
	// the link itself (host-driven SMP). Backend-internal so the application
	// stays SDK-neutral - it does not call any backend-specific function.
	if (g_BtAppData.AppDevice.bSecure)
	{
		BtSmpRequestSecurity(ConnHdl);
	}

	BtAppEvtConnected(ConnHdl);
}

bool BtAppDiscoverDevice(BtDev_t * const pDev)
{
	DEBUG_PRINTF("Start discovering device\r\n");

	// Reset counter and Service list
	pDev->NbSrvc = 0;
	memset(pDev->Services, 0, sizeof(BtGattDBSrvc_t) * BLEPERIPH_DEV_SERVICE_MAXCNT);

	// Start the discover process by discovering the Primary services
	BtUuid_t Uuid = {
			.BaseIdx = 0, // Standard bluetooth
			.Type = BT_UUID_TYPE_16,
			.Uuid16 = BT_UUID_DECLARATIONS_PRIMARY_SERVICE,
	};

	return BtAttStartReadByGroupTypeRequest(pDev->pHciDev, pDev->Conn.Hdl, 1, 0xFFFF, &Uuid);
}

void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason)
{
//	s_BtGapSrvc.ConnHdl = BT_GATT_HANDLE_INVALID;
//	s_BtGattSrvc.ConnHdl = BT_GATT_HANDLE_INVALID;

	DEBUG_PRINTF("BtAppDisconnected: ConnHdl= %d (0x%x); Reason = %d (0x%x)\r\n",
			ConnHdl, ConnHdl, Reason, Reason);

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	BtPeerFree(pPeer);

	bool bConnected = BtPeerIsConnected();

	if (bConnected == false)
	{
//		BtGattSrvcDisconnected(&s_BtGapSrvc);
//		BtGattSrvcDisconnected(&s_BtGattSrvc);

		g_BtAppData.State = BTAPP_STATE_IDLE;
	}

	BtAppEvtDisconnected(ConnHdl);

	if (bConnected == false &&
		(g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER)))
	{
		BtAppAdvStart();
	}
}

void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent)
{
	BtGattSendCompleted(ConnHdl, NbPktSent);

	s_SdcAclTxPktAvail += NbPktSent;
}

void BtAppEnterDfu()
{
	/* TODO: implement */
}

void BtAppDisconnect()
{
	uint16_t connHdl = BtPeerActiveHdl();

	if (connHdl == BT_CONN_HDL_INVALID)
	{
		DEBUG_PRINTF("BtAppDisconnect: no active connection\r\n");
		return;
	}

	// HCI Disconnect, Link Control OGF=0x01/OCF=0x0006.
	// Reason 0x13 = Remote User Terminated Connection.
	sdc_hci_cmd_lc_disconnect_t cmd;

	cmd.conn_handle = connHdl;
	cmd.reason = 0x13;

	uint8_t rc = sdc_hci_cmd_lc_disconnect(&cmd);
	DEBUG_PRINTF("BtAppDisconnect: hdl=%u rc=%u\r\n", connHdl, rc);
}
/*
void BleAppGapDeviceNameSet(const char* pDeviceName)
{
	BleAdvPacket_t *advpkt;

	if (g_BleAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
	}

	size_t l = strlen(pDeviceName);
	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;

	if (l < 14)
	{
		// Short name
		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
	}

	BleAdvDataAdd(advpkt, type, (uint8_t*)pDeviceName, l);

	BtGattCharSetValue(&s_BtGapChar[0], (void*)pDeviceName, l);
}
*/






uint16_t BleAppGetConnHandle()
{
	return BtAppGetConnHandle();
}

static uint8_t BtStackRandPrioLowGet(uint8_t *pBuff, uint8_t Len)
{
	DEBUG_PRINTF("BtStackRandPrioLowGet\r\n");
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = rand();
	}

	return Len;
}

static uint8_t BtStackRandPrioHighGet(uint8_t *pBuff, uint8_t Len)
{
	return BtStackRandPrioLowGet(pBuff, Len);
}

static void BtStackRandPrioLowGetBlocking(uint8_t *pBuff, uint8_t Len)
{
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
	NRF_CRACEN_Type *reg = NRF_CRACEN_S;

	BtStackRandPrioLowGet(pBuff, Len);

#else
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_RNG_Type *reg = NRF_RNG_NS;
#else
	NRF_RNG_Type *reg = NRF_RNG_S;
#endif
#else
	NRF_RNG_Type *reg = NRF_RNG;
#endif

	reg->CONFIG = RNG_CONFIG_DERCEN_Enabled;

	reg->EVENTS_VALRDY = 0;
	reg->TASKS_START = 1;

	for (int i = 0; i < Len; i++)
	{
		while (reg->EVENTS_VALRDY == 0);

		pBuff[i] = reg->VALUE;
		reg->EVENTS_VALRDY = 0;		// clear so the next VALUE is fresh entropy
	}

	reg->TASKS_STOP = 1;

	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;
#endif
}


bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	// Initialize Nordic Softdevice controller

	DEBUG_PRINTF("BtAppStackInit\r\n");

	int32_t res = sdc_init(BtStackSdcAssert);

	//sdc_hci_cmd_cb_reset();

	sdc_rand_source_t rand_functions = {
		//.rand_prio_low_get = BtStackRandPrioLowGet,
		//.rand_prio_high_get = BtStackRandPrioHighGet,
		.rand_poll = BtStackRandPrioLowGetBlocking
	};

	res = sdc_rand_source_register(&rand_functions);

	sdc_support_le_2m_phy();
	sdc_support_le_coded_phy();
	//sdc_support_le_power_control();

	if (pCfg->Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		// Config for peripheral role
		sdc_support_adv();
		sdc_support_ext_adv();
		sdc_support_le_periodic_adv();
		sdc_support_le_periodic_sync();
		sdc_support_peripheral();
		sdc_support_dle_peripheral();
		sdc_support_phy_update_peripheral();
		sdc_support_le_power_control_peripheral();
		sdc_support_le_conn_cte_rsp_peripheral();

		if (pCfg->CoexMode != BTAPP_COEXMODE_NONE)
		{
//			sdc_coex_adv_mode_configure(true);
		}
	}
	if (pCfg->Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
	{
		// Config for central role
		sdc_support_scan();
		sdc_support_ext_scan();
		sdc_support_central();
		sdc_support_ext_central();
		sdc_support_dle_central();
		sdc_support_phy_update_central();
		sdc_support_le_power_control_central();
		sdc_support_le_conn_cte_rsp_central();
	}

    uint32_t ram = 0;
	sdc_cfg_t cfg;

	DEBUG_PRINTF("BtAttSetMtu\r\n");

	uint16_t mtu = 	BtAttSetMtu(pCfg->MaxMtu);

	//int l = pCfg->MaxMtu == 0 ? BTAPP_DEFAULT_MAX_DATA_LEN : mtu + 4;

	// Reserve max always. It seems sdc lib is not capable of changing it in runtime
	cfg.buffer_cfg.rx_packet_size = BTAPP_DEFAULT_MAX_DATA_LEN;
	cfg.buffer_cfg.tx_packet_size = BTAPP_DEFAULT_MAX_DATA_LEN;
	cfg.buffer_cfg.rx_packet_count = BT_SDC_RX_MAX_PACKET_COUNT;
	cfg.buffer_cfg.tx_packet_count = BT_SDC_TX_MAX_PACKET_COUNT;

	DEBUG_PRINTF("sdc_cfg_set\r\n");

	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_BUFFER_CFG,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}


	sdc_hci_cmd_vs_event_length_set_t evlen = {
		.event_length_us = 7500,
	};
	sdc_hci_cmd_vs_event_length_set(&evlen);

	/*
	cfg.event_length.event_length_us = 7500;
	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_EVENT_LENGTH,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}
*/
	if (pCfg->Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		// Config for peripheral role
		cfg.peripheral_count.count = pCfg->PeriLinkCount;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
					       	  SDC_CFG_TYPE_PERIPHERAL_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_count.count = 1;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_buffer_cfg.max_adv_data = 255;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_BUFFER_CFG,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (pCfg->Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
	{
		// Config for central role
		cfg.central_count.count = pCfg->CentLinkCount;
		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
					       	  SDC_CFG_TYPE_CENTRAL_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}


		cfg.scan_buffer_cfg.count = 10;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
						  SDC_CFG_TYPE_SCAN_BUFFER_CFG,
						  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (sizeof(s_BtStackSdcMemPool) < ram)
	{
		return false;
	}

	if (MpslInit() == false)
	{
		return false;
	}

	// Enable BLE stack.

	DEBUG_PRINTF("sdc_enable\r\n");

	res = sdc_enable(BtStackSdcCB, s_BtStackSdcMemPool);
	if (res != 0)
	{
		return false;
	}

	if (pCfg->AttDBMemSize > 0)
	{
		BtAttDBInit(pCfg->AttDBMemSize);
	}

    return true;
}

/**
 * @brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
bool BtAppInit(const BtAppCfg_t *pCfg)
{
	int32_t res = 0;

	// Initialize the peer/connection table (and its long-write pool) before
	// the stack can produce any connection or data event. Matches the
	// nRF52/BM ordering and avoids an event-before-peer-table window.
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}
	BtPeerLongWrInit(pCfg->pLongWrPoolMem, pCfg->LongWrPoolMemSize);
#if 0
	mpsl_clock_lfclk_cfg_t lfclk = {MPSL_CLOCK_LF_SRC_RC, 0,};
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

	// Set default clock based on system oscillator settings
	if (lfosc->Type == OSC_TYPE_RC)
	{
		lfclk.source = MPSL_CLOCK_LF_SRC_RC;
		lfclk.rc_ctiv = MPSL_RECOMMENDED_RC_CTIV;
		lfclk.rc_temp_ctiv = MPSL_RECOMMENDED_RC_TEMP_CTIV;
	}
	else
	{
		lfclk.accuracy_ppm = lfosc->Accuracy;
		lfclk.source = MPSL_CLOCK_LF_SRC_XTAL;
	}

	lfclk.skip_wait_lfclk_started = MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED;

	mpsl_fem_init();

	DEBUG_PRINTF("mpsl_init\r\n");


	// Initialize Nordic multi-protocol support library (MPSL)
#ifdef NRF54L15_XXAA
	//NVIC_SetPriority(SWI00_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	//NVIC_EnableIRQ(SWI00_IRQn);

	res = mpsl_init(&lfclk, SWI00_IRQn, BtStackMpslAssert);
	res = mpsl_clock_hfclk_latency_set(MPSL_CLOCK_HF_LATENCY_TYPICAL);
	mpsl_pan_rfu();
#else
	res = mpsl_init(&lfclk, PendSV_IRQn, BtStackMpslAssert);
#endif

	if (res < 0)
	{
		return false;
	}

#ifdef NRF54L15_XXAA
	NVIC_SetPriority(SWI00_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(SWI00_IRQn);
//	NVIC_SetPriority(RADIO_0_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
//	NVIC_EnableIRQ(RADIO_0_IRQn);
#else
	NVIC_SetPriority(PendSV_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(PendSV_IRQn);
#endif
#endif

//	if (MpslInit() == false)
//	{
//		return false;
//	}

	g_BtAppData.CoexMode = pCfg->CoexMode;

	if (pCfg->CoexMode == BTAPP_COEXMODE_1W)
	{
		//mpsl_coex_support_1wire_gpiote_if();
	}
	else if (pCfg->CoexMode == BTAPP_COEXMODE_3W)
	{
		//mpsl_coex_support_802152_3wire_gpiote_if();
	}

	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	DEBUG_PRINTF("g_BtAppData.AppDevice.Conn.Role = %d\r\n", g_BtAppData.AppDevice.Conn.Role);

	g_BtAppData.bScan = false;
//	g_BtAppData.bAdvertising = false;
	g_BtAppData.AppDevice.VendorId = pCfg->VendorId;
	g_BtAppData.AppDevice.ProductId = pCfg->ProductId;
	g_BtAppData.AppDevice.ProductVer = pCfg->ProductVer;
	g_BtAppData.AppDevice.Appearance = pCfg->Appearance;
	g_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;

	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
    {
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BtAppConnLedOff();
    }

   // g_BleAppData.Role = pBleAppCfg->Role;

    if (BtAppStackInit(pCfg) == false)
    {
    	DEBUG_PRINTF("BtAppStackInit failed\r\n");
    	return false;
    }

	// Device address: read the factory-unique value from FICR (NRF_FICR->
	// DEVICEADDR) and use it as a RANDOM STATIC address. This is the proper
	// nRF mechanism and avoids the Zephyr-specific vendor HCI commands
	// (sdc_hci_cmd_vs_zephyr_*). The top two bits of the MSO must be 1 for a
	// static random address; nrf_get_mac_address() already sets them.
	uint64_t mac = nrf_get_mac_address();

	sdc_hci_cmd_le_set_random_address_t ranaddr;
	for (int i = 0; i < 6; i++)
	{
		ranaddr.random_address[i] = (uint8_t)(mac >> (8 * i));	// LSB first
	}
	// Ensure the static-random marker even if the FICR value changes.
	ranaddr.random_address[5] = (ranaddr.random_address[5] & 0x3f) | 0xc0;

	if (sdc_hci_cmd_le_set_random_address(&ranaddr) != 0)
	{
		return false;
	}

	// SMP f5/f6 must use the same local address/type the peer sees.
	memcpy(s_BtSmpLocalAddr, ranaddr.random_address, 6);
	s_BtSmpLocalAddrType = 1;	// random

	DEBUG_PRINTF("local addr %02x:%02x:%02x:%02x:%02x:%02x type=1\r\n",
				 ranaddr.random_address[5], ranaddr.random_address[4],
				 ranaddr.random_address[3], ranaddr.random_address[2],
				 ranaddr.random_address[1], ranaddr.random_address[0]);

	sdc_hci_cmd_le_read_max_data_length_return_t maxlen;

	res = sdc_hci_cmd_le_read_max_data_length(&maxlen);

	sdc_hci_cmd_le_write_suggested_default_data_length_t datalen = {
		(uint16_t)max(maxlen.supported_max_tx_octets, pCfg->MaxMtu),
		maxlen.supported_max_tx_time
	};

	res = sdc_hci_cmd_le_write_suggested_default_data_length(&datalen);

	sdc_hci_cmd_le_read_buffer_size_return_t rbr;

	res = sdc_hci_cmd_le_read_buffer_size(&rbr);

	sdc_default_tx_power_set(pCfg->TxPower);

	sdc_hci_cmd_le_set_event_mask_t evmask = {0, };

	// Enable all LE meta events EXCEPT the Remote Connection Parameter Request
	// event. When that event is unmasked the controller defers every peer
	// connection-parameter-update to the host and waits for an explicit
	// LE Remote Connection Parameter Request Reply. The SDC HCI app layer does
	// not issue that reply, so leaving the event enabled stalls the link-layer
	// parameter update a central runs right after connecting/pairing - the
	// procedure never completes and the link drops on supervision timeout.
	// Masking the event off lets the controller accept the update autonomously.
	memset(evmask.raw, 0xff, sizeof(evmask.raw));
	evmask.params.le_remote_connection_parameter_request_event = 0;
	if (sdc_hci_cmd_le_set_event_mask(&evmask))
	{
		return false;
	}

	sdc_hci_cmd_cb_set_event_mask_t cbevmask;

	memset(cbevmask.raw, 0xff, sizeof(cbevmask.raw));
	if (sdc_hci_cmd_cb_set_event_mask(&cbevmask))
	{
		return false;
	}

	sdc_hci_cmd_cb_set_event_mask_page_2_t cbevmask2;
	memset(cbevmask2.raw, 0xff, sizeof(cbevmask2.raw));
	if (sdc_hci_cmd_cb_set_event_mask_page_2(&cbevmask2))
	{
		return false;
	}

	BtGapCfg_t gapcfg = {
		.Role = pCfg->Role,
		.SecType = pCfg->SecType,
		.AdvInterval = pCfg->AdvInterval,
		.AdvTimeout = pCfg->AdvTimeout,
		.ConnIntervalMin = pCfg->ConnIntervalMin,
		.ConnIntervalMax = pCfg->ConnIntervalMax,
		.SlaveLatency = 0,
		.SupTimeout = 400
	};

	DEBUG_PRINTF("BtGapInit\r\n");

	BtGapInit(&gapcfg);

	// The SDC backend owns its SMP crypto: software uECC for ECDH and the BLE
	// controller (HCI LE Encrypt) for AES. These are backend-internal - the
	// application does not supply or see them; it only requests security via
	// SecType. Randomness comes from the RngGet utility.
	static CryptoDev_t s_CryptoEcdh;
	static CryptoDev_t s_CryptoAes;
	CryptoUeccInit(&s_CryptoEcdh);
	BtCryptoCtlrSdcInit(&s_CryptoAes);
	BtSmpInit(&s_CryptoEcdh, &s_CryptoAes);

	// Record whether security was requested, so the connected handler can
	// initiate it. Backend-internal - mirrors the SoftDevice backend.
	g_BtAppData.AppDevice.bSecure = (pCfg->SecType != BTGAP_SECTYPE_NONE);

	// Bring up flash-backed bond persistence when security is enabled. This is
	// backend-internal: it loads any stored bonds into the SMP bond table and
	// links the strong BtSmpBondSave/Load/Erase overrides. The application does
	// not call it - persistence follows from the configured SecType.
	if (g_BtAppData.AppDevice.bSecure)
	{
		BtSmpBondSdcInit();
	}

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		//BtGapServiceInit();//&s_BtDevSdc.Srvc[s_BtDevSdc.NbSrvc]);
		BtAppInitUserServices();

		BtGapSetAppearance(pCfg->Appearance);

		BtGattPreferedConnParams_t connparm = {
			MSEC_TO_1_25(pCfg->ConnIntervalMin), MSEC_TO_1_25(pCfg->ConnIntervalMax),
			0, 400};
		BtGapSetPreferedConnParam(&connparm);
	}

	BtAppInitUserData();

    if (pCfg->Role & (BTAPP_ROLE_BROADCASTER | BTAPP_ROLE_PERIPHERAL))
    {
		if (BtAppAdvInit(pCfg) == false)
		{
			return false;
		}
    }
/*
    BtGapInit(pCfg->Role);

    if (pCfg->Role & (BTDEV_ROLE_BROADCASTER | BTDEV_ROLE_PERIPHERAL))
    {
    	if (pCfg->Role & BTDEV_ROLE_PERIPHERAL)
    	{
//    		BtGattSrvcAdd(&s_BtGattSrvc, &s_BtGattSrvcCfg);
//    		BtGattSrvcAdd(&s_BtGapSrvc, &s_BtGapSrvcCfg);

    		BleAppInitUserServices();
    	}

    	if (BleAppAdvInit(pBleAppCfg) == false)
    	{
    		return false;
    	}

    	size_t count = 0;
    	BtGattListEntry_t *tbl = GetEntryTable(&count);
#if 0
    	for (int i = 0; i < count; i++)
    	{
    		DEBUG_PRINTF("tbl[%d]: Hdl: %d (0x%04x), Uuid: %04x, Data: ", i, tbl[i].Hdl, tbl[i].Hdl, tbl[i].TypeUuid.Uuid);
    		uint8_t *p = (uint8_t*)&tbl[i].Val32;
    		for (int j = 0; j < 20; j++)
    		{
    			DEBUG_PRINTF("0x%02x ", p[j]);
    		}
    		DEBUG_PRINTF("\r\n");
    	}
#endif
    }
*/
	BtGapSetDevName(pCfg->pDevName);

    //BleAppGapDeviceNameSet(pBleAppCfg->pDevName);
#ifndef NRF54L15_XXAA
#if (__FPU_USED == 1)
    // Patch for softdevice & FreeRTOS to sleep properly when FPU is in used
    NVIC_SetPriority(FPU_IRQn, 6);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif
#endif

    if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem, pCfg->EvtHandlerQueMemSize) == false)
    {
    	return false;
    }

	// Connection pool removed: the peer manager (BtPeerInit above) owns
	// the single connection table now.
	//BtHciInit(&s_BtDevCfg);

    g_BtAppData.State = BTAPP_STATE_INITIALIZED;


	return true;
}

void BtAppRun()
{
	if (g_BtAppData.State != BTAPP_STATE_INITIALIZED)
	{
		return;
	}

	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAppAdvStart();
	}

DEBUG_PRINTF("Loop\r\n");

	while (1)
	{
		BtAppEvtWait();
		AppEvtHandlerExec();

#if 1
		uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
		sdc_hci_msg_type_t mtype;
		// Drain ALL queued HCI messages, not just one. The SDC can queue
		// several at once (command completion + Encryption Change event during
		// pairing); processing one per wake can strand the later ones and hang
		// encryption/pairing completion.
		while (sdc_hci_get(buf, (uint8_t*)&mtype) == 0)
		{
			switch (mtype)
			{
				case SDC_HCI_MSG_TYPE_EVT:
					BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)buf);
					break;
				case SDC_HCI_MSG_TYPE_DATA:
					BtHciProcessData(&s_BtHciDev, (BtHciACLDataPacket_t*)buf);
					break;
			}
		}
#endif
	}
}

// Port-level weak default for BtAppEvtWait. Bare-metal apps fall through to
// __WFE; RTOS apps provide a strong override that does a semaphore take.
__attribute__((weak)) void BtAppEvtWait(void)
{
	__WFE();
}

#if 0
bool BleAppScanInit(ble_uuid128_t * const pBaseUid, ble_uuid_t * const pServUid)
{
    ble_uuid128_t base_uid = *pBaseUid;
    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ret_code_t err_code = sd_ble_uuid_vs_add(&base_uid, &uidtype);
    APP_ERROR_CHECK(err_code);

    //ble_db_discovery_evt_register(pServUid);
    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}
#endif

#if 0
bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam)
{
	ret_code_t err_code = sd_ble_gap_connect(pDevAddr, &s_BleScanParams,
                                  	  	  	 pConnParam,
											 BLEAPP_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = false;

    return err_code == NRF_SUCCESS;
}
#endif

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle)//ble_uuid_t * const pCharUid)
{
//	BtGattCharNotify();

	return false;
#if 0
    uint32_t                 err_code;
    ble_gattc_write_params_t write_params;
    uint8_t                  buf[BLE_CCCD_VALUE_LEN];

    buf[0] = BLE_GATT_HVX_NOTIFICATION;
    buf[1] = 0;

    write_params.write_op = BLE_GATT_OP_WRITE_CMD;//BLE_GATT_OP_WRITE_REQ;
    write_params.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
    write_params.handle   = CharHandle;
    write_params.offset   = 0;
    write_params.len      = sizeof(buf);
    write_params.p_value  = buf;

    err_code = sd_ble_gattc_write(ConnHandle, &write_params);

    return err_code == NRF_SUCCESS;
#endif
}

bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	return BtGattCharNotify(BtPeerActiveHdl(), pChar, pData, DataLen);
	/*
	if (BtGattCharSetValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	if (isBtGattCharNotifyEnabled(pChar) == false)
	{
		return false;
	}

	BtHciMotify(&s_HciDevice, g_BleAppData.ConnHdl, pChar->ValHdl, pData, DataLen);
//	BtHciMotify(g_BleAppData.ConnHdl, pChar->ValHdl, pData, DataLen);

	return true;*/
}

bool BleAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
	return false;
#if 0
	if (ConnHandle == BLE_CONN_HANDLE_INVALID || CharHandle == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = CharHandle,
        .offset   = 0,
        .len      = DataLen,
        .p_value  = pData
    };

    return sd_ble_gattc_write(ConnHandle, &write_params) == NRF_SUCCESS;
#endif
}

#if 0
extern "C" {
#ifdef NRF54L15_XXAA
void SWI00_IRQHandler(void)
#else
void PendSV_Handler(void)
#endif
{
	DEBUG_PRINTF("mpsl_low_priority_process\r\n");
	mpsl_low_priority_process();
}


#ifdef NRF54L15_XXAA
void RADIO_0_IRQHandler(void)
#else
void RADIO_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RADIO_Handler\r\n");
	MPSL_IRQ_RADIO_Handler();
}

#ifdef NRF54L15_XXAA
void CLOCK_POWER_IRQHandler()
#else
void POWER_CLOCK_IRQHandler()
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_CLOCK_Handler\r\n");
	MPSL_IRQ_CLOCK_Handler();
}

#ifdef NRF54L15_XXAA
void GRTC_3_IRQHandler(void)
#else
void RTC0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RTC0_Handler\r\n");
	MPSL_IRQ_RTC0_Handler();
}

#ifdef NRF54L15_XXAA
void TIMER10_IRQHandler(void)
#else
void TIMER0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_TIMER0_Handler\r\n");
	MPSL_IRQ_TIMER0_Handler();
}

/** @brief MPSL requesting CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it needs CONSTLAT to be on.
 * It only calls the function on nRF54L Series devices.
 */
void mpsl_constlat_request_callback(void)
{
	DEBUG_PRINTF("mpsl_constlat_request_callback\r\n");
}

/** @brief De-request CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it no longer needs CONSTLAT to be on.
 * It only only calls the function on nRF54L Series devices.
 */
void mpsl_lowpower_request_callback(void)
{
	DEBUG_PRINTF("mpsl_lowpower_request_callback\r\n");
}

}

#endif
