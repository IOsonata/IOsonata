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
#include "sdc_hci.h"
#include "sdc_hci_vs.h"

#include "istddef.h"
#include "convutil.h"
#include "nrf_mac.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"		// BtSmpLocalAddrGet override

#include "crypto/crypto_uecc.h"
#include "crypto_rng_nrf.h"
#if defined(NRF54L15_XXAA) || defined(NRF54H20_XXAA)
#include "cracen_intrf.h"
#include "crypto/ba414ep.h"
#elif defined(NRF52840_XXAA)
#include "crypto_cc3xx.h"
#endif
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/services/bt_dis.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_hci_ctlr.h"
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

void BtAppEvtHandler(BtHciDevice_t * const pDev, uint32_t Evt);
void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]);
void BtAppDisconnected(uint16_t ConnHdl, uint8_t Reason);
void BtAppSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);
bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *DavData);

static void BtAppSdcTimerHandler(TimerDev_t * const pTimer, uint32_t Evt);
static inline uint32_t BtAppSendData(void *pData, uint32_t Len);

static BtHciDevice_t s_BtHciDev = {
	.pCtx = (void*)&g_BtAppData,
	.SendData = BtAppSendData,
	.Command = BtHciCmdSdc,
	.EvtHandler = BtAppEvtHandler,
	.Connected = BtAppConnected,
	.Disconnected = BtAppDisconnected,
	.SendCompleted = BtAppSendCompleted,
	.ScanReport = BtAppScanReport,
	.AdvTimeout = BtAppAdvTimeoutHandler,
	//.DiscoverDevice = BtAppDiscoverDevice,
};

// SDC controller instance. The HCI pump and transport live in
// bt_hci_ctlr_sdc; this app wires the receive handler to the host.
static BtHciCtlrDev_t s_BtHciCtlr;


// BtAppData_t now declared in bluetooth/bt_app.h, accessed via g_BtAppData.
// SDC port has no SDK-specific state in BtAppData_t scope; everything that
// remains here is already in the cross-arch struct.

// g_BtAppData definition and helpers (isConnected, BtAppConnLedOff/On) moved to
// src/bluetooth/bt_app.cpp.

// On-air local address SMP needs for c1/f5/f6 (set when we configure the
// random static address below). Defaults to public/zero until then.
static uint8_t s_BtSmpLocalAddr[6] = {0};
static uint8_t s_BtSmpLocalAddrType = 0;



/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};



const static TimerCfg_t s_BtAppSdcTimerCfg = {
    .DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtAppSdcTimerHandler
};

static Timer g_BtAppSdcTimer;

static inline uint32_t BtAppSendData(void *pData, uint32_t Len) {
	return (uint32_t)BtHciCtlrSdcSend(pData, Len);
}

// Override the weak SMP accessor so the toolbox uses the device's configured
// address. Declared in bt_smp.h, so no linkage specifier is needed here.
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = s_BtSmpLocalAddrType;
	memcpy(pAddr, s_BtSmpLocalAddr, 6);
}

// Surface a secured link (fresh pairing or bonded reconnect) to the application.
// The generic SMP engine calls this on every successful encryption; translate it
// to the port-neutral BtAppEvtSecured hook the example gates discovery on.
// Declared in bt_smp.h, so no linkage specifier is needed here.
void BtSmpPairingComplete(uint16_t ConnHdl, bool Success,
						  const BtSmpKeys_t *pKeys)
{
	(void)pKeys;
	if (Success)
	{
		BtAppEvtSecured(ConnHdl);
	}
}

// Route each HCI packet the controller drains to the host process entry.
static void BtAppSdcCtlrRx(BtHciCtlrDev_t * const pDev, bool bIsEvent, uint8_t *pPacket)
{
	if (bIsEvent)
	{
		BtHciProcessEvent(&s_BtHciDev, (BtHciEvtPacket_t*)pPacket);
	}
	else
	{
		BtHciProcessData(&s_BtHciDev, (BtHciACLDataPacket_t*)pPacket);
	}
}

// Millisecond clock for the generic SMP/GATT transaction timeouts. These
// override the weak BtSmpMsTick/BtGattMsTick defaults (which return 0, leaving
// the timeouts inert). Both are declared in bt_smp.h / bt_gatt.h, so no
// linkage specifier is needed here. g_BtAppSdcTimer is started in BtAppInit.
uint32_t BtSmpMsTick(void)
{
	return g_BtAppSdcTimer.mSecond();
}

uint32_t BtGattMsTick(void)
{
	return g_BtAppSdcTimer.mSecond();
}

// Spec-strict indication transaction timeout: Core Vol 3 Part F 3.3.3 requires
// closing the bearer, so disconnect the link. The generic weak default only
// clears the outstanding-indication flag.
void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	uint8_t param[3];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)(ConnHdl >> 8);
	param[2] = 0x13;	// Remote User Terminated Connection

	BtHciCommand(&s_BtHciDev, BT_HCI_CMD_LINKCTRL_DISCONNECT, param, sizeof(param), NULL, 0);
}

#if 0
static void BtStackMpslAssert(const char * const file, const uint32_t line)
{
	DEBUG_PRINTF("MPSL Fault: %s, %d\n", file, line);
	while(1);
}
#endif




static void BtAppSdcTimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
        // Drive the generic transaction timeouts (Core Vol 3 Part H 3.4,
        // Part F 3.3.3). Both are cheap no-ops when nothing is pending.
        BtSmpTimeoutCheck();
        BtGattIndicationTimeoutCheck();
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

	// If a secure SecType was configured, secure the link. As the central we
	// initiate pairing (or re-encrypt from a bond); as the peripheral we send a
	// Security Request. Host-driven SMP, internal so the application stays
	// SDK-neutral - it does not call any stack-specific function.
	if (g_BtAppData.AppDevice.bSecure)
	{
		if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
		{
			BtSmpStartPairing(ConnHdl);
		}
		else
		{
			BtSmpRequestSecurity(ConnHdl);
		}
	}

	BtAppEvtConnected(ConnHdl);
}

bool BtAppDiscoverDevice(BtDevice_t * const pDev)
{
	DEBUG_PRINTF("Start discovering device\r\n");

	// Reset counter and Service list
	pDev->NbSrvc = 0;
	memset(pDev->Services, 0, sizeof(BtGattDBSrvc_t) * BT_DEV_SERVICE_MAXCNT);

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
	uint8_t param[3];
	param[0] = (uint8_t)(connHdl & 0xFF);
	param[1] = (uint8_t)(connHdl >> 8);
	param[2] = 0x13;

	uint8_t rc = BtHciCommand(&s_BtHciDev, BT_HCI_CMD_LINKCTRL_DISCONNECT, param, sizeof(param), NULL, 0);
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



bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	BtAttSetMtu(pCfg->MaxMtu);

	BtHciCtlrCfg_t ctlrcfg = { };
	ctlrcfg.RxHandler = BtAppSdcCtlrRx;
	ctlrcfg.OnWake = BtAppEvtNotify;
	ctlrcfg.Role = pCfg->Role;
	ctlrcfg.PeriLinkCount = pCfg->PeriLinkCount;
	ctlrcfg.CentLinkCount = pCfg->CentLinkCount;
	ctlrcfg.RxPktCount = BT_SDC_RX_MAX_PACKET_COUNT;
	ctlrcfg.TxPktCount = BT_SDC_TX_MAX_PACKET_COUNT;
	ctlrcfg.MaxDataLen = BTAPP_DEFAULT_MAX_DATA_LEN;

	if (BtHciCtlrEnable(&s_BtHciCtlr, &ctlrcfg) == false)
	{
		return false;
	}

	// The controller was configured with these ACL buffer parameters above.
	// Use the generic HCI host credit gate instead of the old SDC-local
	// s_SdcAclTxPktAvail counter.
	BtHciSetLeAclBuffer(&s_BtHciDev, ctlrcfg.MaxDataLen, ctlrcfg.TxPktCount);

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
	g_BtAppData.AppDevice.pHciDev = &s_BtHciDev;		// host used by the HCI operation layer (bt_adv_hci etc.)
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

	uint8_t ranaddr[6];
	for (int i = 0; i < 6; i++)
	{
		ranaddr[i] = (uint8_t)(mac >> (8 * i));	// LSB first
	}
	// Ensure the static-random marker even if the FICR value changes.
	ranaddr[5] = (ranaddr[5] & 0x3f) | 0xc0;

	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, ranaddr, sizeof(ranaddr), NULL, 0) != 0)
	{
		return false;
	}

	// SMP f5/f6 must use the same local address/type the peer sees.
	memcpy(s_BtSmpLocalAddr, ranaddr, 6);
	s_BtSmpLocalAddrType = 1;	// random

	DEBUG_PRINTF("local addr %02x:%02x:%02x:%02x:%02x:%02x type=1\r\n",
				 ranaddr[5], ranaddr[4], ranaddr[3], ranaddr[2], ranaddr[1], ranaddr[0]);

	// LE Read Maximum Data Length return: supported max TX octets, TX time, RX
	// octets, RX time, each 2 bytes little endian.
	uint8_t maxlen[8] = {0};
	BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN, NULL, 0, maxlen, sizeof(maxlen));
	uint16_t maxTxOctets = (uint16_t)(maxlen[0] | (maxlen[1] << 8));
	uint16_t maxTxTime   = (uint16_t)(maxlen[2] | (maxlen[3] << 8));

	uint16_t txOctets = (uint16_t)min(maxTxOctets, pCfg->MaxMtu);
	uint8_t datalen[4];
	datalen[0] = (uint8_t)(txOctets & 0xff);
	datalen[1] = (uint8_t)(txOctets >> 8);
	datalen[2] = (uint8_t)(maxTxTime & 0xff);
	datalen[3] = (uint8_t)(maxTxTime >> 8);
	BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_WRITE_SUGG_DEFAULT_DATA_LEN, datalen, sizeof(datalen), NULL, 0);

	sdc_default_tx_power_set(pCfg->TxPower);

	// Enable all LE meta events EXCEPT the LE Remote Connection Parameter Request
	// event (LE event mask octet 0, bit 5). When that event is unmasked the
	// controller defers every peer connection-parameter-update to the host and
	// waits for an explicit reply. The app layer does not issue that reply, so
	// leaving it enabled stalls the link-layer parameter update a central runs
	// right after connecting or pairing, and the link drops on supervision timeout.
	uint8_t evmask[8];
	memset(evmask, 0xff, sizeof(evmask));
	evmask[0] &= ~(1 << 5);		// LE Remote Connection Parameter Request event
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_CTLR_SET_EVENT_MASK, evmask, sizeof(evmask), NULL, 0))
	{
		return false;
	}

	uint8_t cbevmask[8];
	memset(cbevmask, 0xff, sizeof(cbevmask));
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_BASEBAND_SET_EVENT_MASK, cbevmask, sizeof(cbevmask), NULL, 0))
	{
		return false;
	}

	uint8_t cbevmask2[8];
	memset(cbevmask2, 0xff, sizeof(cbevmask2));
	if (BtHciCommand(&s_BtHciDev, BT_HCI_CMD_BASEBAND_SET_EVENT_MASK_PAGE2, cbevmask2, sizeof(cbevmask2), NULL, 0))
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

	// The SDC path owns its SMP crypto: P-256 ECDH and the BLE controller
	// (HCI LE Encrypt) for AES. These are internal to this path - the
	// application does not supply or see them; it only requests security via
	// SecType. Randomness comes from the Nordic hardware RNG.
	//
	// The SDC runs on several families with different P-256 hardware, selected
	// at compile time:
	//   nRF54L15 / nRF54H20 : CRACEN            -> Ba414ep (hardware)
	//   nRF52840            : CryptoCell CC310  -> CryptoCc3xx (hardware)
	//   nRF52832            : no accelerator    -> CryptoUecc (software)
	//   nRF5340 (net core)  : CC312 is on the app core secure domain, not
	//                         reachable from the network core -> CryptoUecc
	// The controller supplies AES-128 ECB through the HCI LE Encrypt path
	// (CryptoCtlrSdc). SMP composes the ECDH engine and the AES engine.
	KeyAgreeEngine *pEcdh = nullptr;
#if defined(NRF54L15_XXAA) || defined(NRF54H20_XXAA)
	static Ba414ep s_Ecdh;								// CRACEN engine object
	if (s_Ecdh.Init(CracenIntrfInstance(), CryptoRngNrfInstance()))
	{
		pEcdh = &s_Ecdh;
		DEBUG_PRINTF("Crypto ECDH engine: Ba414ep (CRACEN hardware P-256)\r\n");
	}
#elif defined(NRF52840_XXAA)
	alignas(CryptoCc3xx) static uint8_t s_CryptoEcdhMem[CRYPTO_CC3XX_MEMSIZE];
	pEcdh = CryptoCc3xxCreate(s_CryptoEcdhMem, sizeof(s_CryptoEcdhMem),
							 CryptoRngNrfInstance());
	if (pEcdh != nullptr)
	{
		DEBUG_PRINTF("Crypto ECDH engine: CryptoCc3xx (CC310 hardware P-256)\r\n");
	}
#else
	alignas(uint64_t) static uint8_t s_CryptoEcdhMem[CRYPTO_UECC_MEMSIZE];
	pEcdh = CryptoUeccCreate(s_CryptoEcdhMem, sizeof(s_CryptoEcdhMem),
							 CryptoRngNrfInstance());
	if (pEcdh != nullptr)
	{
		DEBUG_PRINTF("Crypto ECDH engine: CryptoUecc (software P-256)\r\n");
	}
#endif
	if (pEcdh == nullptr)
	{
		// No P-256 engine came up. LE Secure Connections pairing cannot run:
		// SmpLocalKeyGen fails and SMP answers every pairing with
		// BT_SMP_ERR_UNSPECIFIED. Say so here rather than at the first pairing.
		DEBUG_PRINTF("Crypto ECDH engine MISSING, LESC pairing will fail\r\n");
	}

	CipherEngine *pAes = BtCryptoCtlrSdcInit();
	BtSmpInit(pEcdh, pAes, CryptoRngNrfInstance());

	// Verify the SMP crypto toolbox against the specification sample data
	// before any pairing can run. These exercise the AES-CMAC based SC
	// functions (f4) and the legacy c1/s1 confirm functions, plus the RPA and
	// signing helpers, catching a byte-order or AES engine fault at startup
	// rather than as a confirm mismatch against a peer. A failure here means
	// the composed AES engine is wrong; refuse to continue.
	if (BtSmpF4SelfTest() != 0 || BtSmpC1S1SelfTest() != 0 ||
		BtSmpRpaSelfTest() != 0 || BtSmpSignSelfTest() != 0)
	{
		DEBUG_PRINTF("BtAppInit: SMP crypto self-test FAILED\r\n");
		return false;
	}

	// Translate the application security configuration into the SMP IO
	// capability, authentication requirements and association-model callbacks.
	// SecExchg selects the IO capability; SecType selects bonding and MITM. The
	// Secure Connections bit is forced inside BtSmpAuthConfig.
	uint8_t smpIoCaps;
	if ((pCfg->SecExchg & BTAPP_SECEXCHG_KEYBOARD) &&
		(pCfg->SecExchg & BTAPP_SECEXCHG_DISPLAY))
	{
		smpIoCaps = BT_SMP_IOCAPS_KEYBOARD_DISPLAY;
	}
	else if ((pCfg->SecExchg & BTAPP_SECEXCHG_DISPLAY) &&
			 (pCfg->SecExchg & BTAPP_SECEXCHG_YESNO))
	{
		smpIoCaps = BT_SMP_IOCAPS_DISPLAY_YESNO;
	}
	else if (pCfg->SecExchg & BTAPP_SECEXCHG_KEYBOARD)
	{
		smpIoCaps = BT_SMP_IOCAPS_KEYBOARD_ONLY;
	}
	else if (pCfg->SecExchg & BTAPP_SECEXCHG_DISPLAY)
	{
		smpIoCaps = BT_SMP_IOCAPS_DISPLAY_ONLY;
	}
	else
	{
		smpIoCaps = BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT;
	}

	uint8_t smpAuthReq = 0;
	if (pCfg->SecType != BTGAP_SECTYPE_NONE)
	{
		smpAuthReq |= BT_SMP_AUTHREQ_BONDING_FLAG_BONDING;
	}
	if (pCfg->SecType == BTGAP_SECTYPE_STATICKEY_MITM ||
		pCfg->SecType == BTGAP_SECTYPE_LESC_MITM ||
		pCfg->SecType == BTGAP_SECTYPE_SIGNED_MITM)
	{
		smpAuthReq |= BT_SMP_AUTHREQ_MITM;
	}

	BtSmpAuthConfig(smpIoCaps, smpAuthReq);

	// Record whether security was requested, so the connected handler can
	// initiate it. Internal to this path - mirrors the SoftDevice implementation.
	g_BtAppData.AppDevice.bSecure = (pCfg->SecType != BTGAP_SECTYPE_NONE);

	// Bring up flash-backed bond persistence when security is enabled. This is
	// internal to this path: it loads any stored bonds into the SMP bond table and
	// links the strong BtSmpBondSave/Load/Erase overrides. The application does
	// not call it - persistence follows from the configured SecType.
	if (g_BtAppData.AppDevice.bSecure)
	{
		BtSmpBondSdcInit();
	}

	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();

		// Register Device Information Service when the app supplies device
		// info. Generic bt_dis adds it to the same ATT DB as user services.
		if (pCfg->pDevInfo != NULL)
		{
			BtDisInit(pCfg);
		}

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

	// Start the app timer with a 1 s continuous trigger. It sources the SMP/GATT
	// millisecond clock (BtSmp/GattMsTick above) and its handler drives the
	// 30 s transaction-timeout checks. 1 s cadence is ample for a 30 s deadline.
	g_BtAppSdcTimer.Init(s_BtAppSdcTimerCfg);
	g_BtAppSdcTimer.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS, nullptr);

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

		BtHciCtlrProcess(&s_BtHciCtlr);
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

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CccdHandle)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHandle);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return false;
	}

	// Enable notifications by writing 0x0001 to the characteristic's CCCD.
	// A Write Request is used so the server acknowledges the configuration.
	uint8_t cccd[2] = { 0x01, 0x00 };
	return BtAttWriteRequest(pPeer->pHciDev, ConnHandle, CccdHandle, cccd, sizeof(cccd));
}

bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHandle);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return false;
	}

	// Write without response, matching the write-command path on the SoftDevice
	// ports and the BlueIO UART TX characteristic (WRITE | WRITEWORESP).
	return BtAttWriteCommand(pPeer->pHciDev, ConnHandle, CharHandle, pData, DataLen);
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

bool BtAppIndicate(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	return BtGattCharIndicate(BtPeerActiveHdl(), pChar, pData, DataLen);
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
