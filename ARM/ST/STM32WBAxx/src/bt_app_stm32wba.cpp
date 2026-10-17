/**-------------------------------------------------------------------------
@file	bt_app_stm32wba.cpp

@brief	STM32WBAxx BLE port - top-level application init, stack bring-up,
        and run loop. Models ST's full BLE host stack (ACI commands), the
        STM32CubeWBA middleware path. Mirrors the vendor-middleware shape of
        the BM and nRF52 ports.

        File responsibilities:
            - BtAppStackInit:  bring up ST BLE stack, register HCI callback,
                               set TX power and BD address.
            - BtAppInit:       populate g_BtAppData, init GAP/GATT, register
                               user services, init DIS, init advertising.
	- Stack callbacks (hci_notify_asynch_evt, BLE_RESUME_FLOW):
		       queue the HCI event drain and the host work
		       with BtEvtQue. A 1 s timer queues the
		       timeout checks. Advertising starts from the
		       first queued event.
            - on_conn_params_evt: callback for the connection-update
                                   procedure handled in bt_cp_stm32wba.cpp.
            - BtAppNotify, BtAppDisconnect, BtAppGapDeviceNameSet,
              BtAppWrite, BtAppEnableNotify: standard BLE app helpers
              implemented via ACI commands.

        Adv/scan/dev/cp/dis/isr code lives in bt_*_stm32wba.cpp peers.

@author	Hoang Nguyen Hoan
@date	May 25, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include <stdlib.h>
#include <string.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal.h"

// STM32CubeWBA BLE middleware, release 1.10.0 (WPAN middleware commit
// 7e764835). The project provides the Target files of the Cube reference
// application (bleplat.c, linklayer_plat.c, ll_sys_if.c, bpka.c, power_table.c)
// and its app_conf.h set. host_stack_if.c is not used: BleStackCB_Process and
// HostStack_Process are defined here and run the host stack from the
// Bluetooth event queue.
#include "bt_wba_stack.h"
#include "bt_wba_platform.h"
#include "stm32wbaxx_ll_system.h"

#include "istddef.h"
#include "idelay.h"
#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "iopinctrl.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_att.h"
#include "bt_wba_event_router.h"

/******** For DEBUG ************/
//#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// ST BLE stack expects success = 0; everything else is an error code.
#ifndef BLE_STATUS_SUCCESS
#define BLE_STATUS_SUCCESS				0x00
#endif

// ATT MTU. Default per BT spec is 23. The stack negotiates higher via
// aci_gatt_exchange_config. Apps that set BtAppCfg_t::MaxMtu request a
// higher value; the cap depends on the stack configuration buffer sizes.
#ifndef GATT_MTU_SIZE_DEFAULT
#define GATT_MTU_SIZE_DEFAULT			23
#endif

// Period of the timeout check event, in ms
#define BT_APP_TICK_PERIOD_MS			1000U

// Connection-parameter update timing knobs - same intent as the BM port.
#define BT_APP_CONN_PARAMS_FIRST_DELAY_MS	5000
#define BT_APP_CONN_PARAMS_NEXT_DELAY_MS	30000
#define BT_APP_CONN_PARAMS_MAX_ATTEMPTS		3

// SMP key-size limits per BT spec.
#define SEC_PARAM_MIN_KEY_SIZE			7
#define SEC_PARAM_MAX_KEY_SIZE			16

// HCI error used when the application cannot complete an authenticated user
// interaction. aci_gap_pass_key_resp has no reject form; submitting zero would
// accept the valid passkey 000000 rather than cancel pairing.
#define BT_APP_HCI_AUTH_FAILURE			0x05

// Mapping from BtGap SecType to ST's IO-capability + auth-requirement
// values is done at init time. Default to NoInputNoOutput / no MITM.
#ifndef BT_APP_DEFAULT_IO_CAPABILITY
#define BT_APP_DEFAULT_IO_CAPABILITY	IO_CAP_NO_INPUT_NO_OUTPUT
#endif

// Forward decls from peer port files (defined in 9b..9e).
extern "C" bool BtAppAdvInit(const BtAppCfg_t *pCfg);
extern "C" bool BtDisInit(const struct __Bt_App_Cfg *pCfg);
void BtAdvWbaConnected(void);

// User hooks - weak symbols overridden by the app.
__attribute__((weak)) void BtAppInitUserServices(void) {}
__attribute__((weak)) void BtAppInitUserData(void) {}

// --- Port-private data ---
//
// Sized smaller than the BM equivalent because ST handles adv handle
// allocation internally - we don't track a separate AdvHdl on this port.
typedef struct __Bt_App_WbaData {
	uint16_t	GattSrvcStartHdl;	//!< First handle of the user GATT service
	uint16_t	PasskeyRejectedHdl;	//!< Synchronous passkey UI rejection marker
	uint8_t		IoCapability;		//!< SMP IO cap - mapped from SecType at init
	uint8_t		AuthRequirement;	//!< SMP auth requirement bits
	bool		bOobEnabled;		//!< LESC OOB explicitly enabled by app config
	bool		bStackInited;		//!< true after BleStack_Init succeeded
	uint16_t	NvmCacheWords;		//!< 64 bit words preloaded into the cache
} BtAppWbaData_t;

static BtAppWbaData_t s_WbaData = {
	.GattSrvcStartHdl = 0,
	.PasskeyRejectedHdl = BT_CONN_HDL_INVALID,
	.IoCapability     = BT_APP_DEFAULT_IO_CAPABILITY,
	.AuthRequirement  = 0,
	.bOobEnabled      = false,
	.bStackInited     = false,
};

// DIVC and MAX are used by ble_bufsize.h. The Cube projects take them from
// utilities_common.h.
#ifndef DIVC
#define DIVC(x, y)						(((x) + (y) - 1) / (y))
#endif
#ifndef MAX
#define MAX(x, y)						(((x) > (y)) ? (x) : (y))
#endif

// Set while the host work is in the queue, so that it is queued once
static volatile bool s_bBleHostQueued = false;
static volatile bool s_bBleHostOwed = false;
// Set while the event drain is in the queue, so that it is queued once
static volatile bool s_bEvtDrainQueued = false;
static volatile bool s_bEvtDrainOwed = false;

// HCI events from the host stack. BLECB_Indication runs inside
// BleStack_Process and no stack function may be called from there, so each
// event is copied here and handled later from the Bluetooth event queue. A slot
// holds the packet as the stack gives it: packet type, event code, parameter
// length, parameters.
#ifndef BT_APP_WBA_EVT_SLOTS
#define BT_APP_WBA_EVT_SLOTS			8
#endif
#define BT_APP_WBA_EVT_MAXLEN			(3 + 255)

typedef struct __Bt_App_Wba_Evt {
	uint16_t	Len;
	uint8_t		Data[BT_APP_WBA_EVT_MAXLEN];
} BtAppWbaEvt_t;

static BtAppWbaEvt_t s_EvtRing[BT_APP_WBA_EVT_SLOTS];
static volatile uint8_t s_EvtHead = 0;		// next slot to write
static volatile uint8_t s_EvtTail = 0;		// next slot to read
static volatile uint8_t s_EvtCnt = 0;

static BtWbaEvtFlow_t BtAppHciEvtHandler(void *pPayload);
extern "C" void BleStackCB_Process(void);

// Let the BLE host stack process its pending work. It runs the state machines
// that issue the ACI commands and forwards the events to BLECB_Indication.
static void BleHostEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bBleHostQueued = false;
	if (BleStack_Process() == 0)
	{
		// The stack has more to do
		BleStackCB_Process();
	}
}

// Host stack request, from the ACI wrappers (BLE_WRAP_POSTPROC) and from the
// link layer. Can come from interrupt context. Replaces the version in the Cube
// host_stack_if.c, which uses the Cube sequencer.
extern "C" void BleStackCB_Process(void)
{
	if (s_bBleHostQueued == false)
	{
		s_bBleHostQueued = true;
		s_bBleHostOwed = BtEvtQue(0, nullptr, BleHostEvt) == false;
		if (s_bBleHostOwed)
		{
			s_bBleHostQueued = false;
		}
	}
}

// Host stack processing request from the link layer.
extern "C" void HostStack_Process(void)
{
	BleStackCB_Process();
}

// Handle the queued events. Runs from the Bluetooth event queue, outside
// BleStack_Process.
static void BtAppEvtDrain(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bEvtDrainQueued = false;

	bool bFreed = false;

	while (s_EvtCnt > 0)
	{
		BtWbaEventRouterRun(s_EvtRing[s_EvtTail].Data);
		s_EvtTail = (uint8_t)((s_EvtTail + 1) % BT_APP_WBA_EVT_SLOTS);
		__disable_irq();
		s_EvtCnt--;
		__enable_irq();
		bFreed = true;
	}

	if (bFreed)
	{
		// A refused indication stays in the stack until it runs again
		BleStackCB_Process();
	}
}

static void BtAppEvtDrainQue(void)
{
	if (s_bEvtDrainQueued == false)
	{
		s_bEvtDrainQueued = true;
		s_bEvtDrainOwed = BtEvtQue(0, nullptr, BtAppEvtDrain) == false;
		if (s_bEvtDrainOwed)
		{
			s_bEvtDrainQueued = false;
		}
	}
}

// Event indication from the host stack, called inside BleStack_Process. ACL
// data is not used by this port. Returns a failure when the ring is full and
// the stack offers the event again after the next BleStackCB_Process.
extern "C" uint8_t BLECB_Indication(const uint8_t *pData, uint16_t Length,
									   const uint8_t *pExtData, uint16_t ExtLength)
{
	(void)pExtData;
	(void)ExtLength;

	if (pData[0] != HCI_EVENT_PKT_TYPE)
	{
		return BLE_STATUS_SUCCESS;
	}

	if (s_EvtCnt >= BT_APP_WBA_EVT_SLOTS || Length < 3 ||
		Length > BT_APP_WBA_EVT_MAXLEN)
	{
		return BLE_STATUS_FAILED;
	}

	BtAppWbaEvt_t *pEvt = &s_EvtRing[s_EvtHead];

	pEvt->Len = Length;
	memcpy(pEvt->Data, pData, Length);
	s_EvtHead = (uint8_t)((s_EvtHead + 1) % BT_APP_WBA_EVT_SLOTS);
	__disable_irq();
	s_EvtCnt++;
	__enable_irq();

	BtAppEvtDrainQue();

	return BLE_STATUS_SUCCESS;
}

static void BtAppTickQue(void);

extern void BtDfuSmpCheckStatus(void) __attribute__((weak));

void BtAppCheckStatus(void)
{
	BtAppTickQue();
	if (s_bEvtDrainOwed)
	{
		BtAppEvtDrainQue();
	}
	if (s_bBleHostOwed)
	{
		BleStackCB_Process();
	}
	BtWbaPlatformRetry();
	if (BtDfuSmpCheckStatus != nullptr)
	{
		BtDfuSmpCheckStatus();
	}
}

// --- GATT client discovery (central role) ---
//
// ST runs each discovery procedure to completion and signals the end with a
// single ACI_GATT_PROC_COMPLETE event; the per-entry data arrives in
// ACI_ATT_READ_BY_GROUP_TYPE_RESP (services), ACI_ATT_READ_BY_TYPE_RESP
// (characteristics) and ACI_ATT_FIND_INFO_RESP (descriptors) events before it.
// The cursor walks services, then characteristics per service, then descriptors
// per characteristic (to locate the CCCD), then calls BtDeviceDiscovered.
// Values for BtDevDiscState_t::Phase on this port. Idle is 0, which is what a
// freshly cleared peer record holds.
enum {
	DISC_IDLE = 0,
	DISC_SERVICES,
	DISC_CHARS,
	DISC_DESCS
};

// Discovery progress lives in the peer record, one cursor per link
// (pDev->Discovery). It used to be four file scope variables, so a second
// central link discovering at the same time overwrote the first one's position.
// The ACI response events all name their link in Connection_Handle, so each
// callback resolves its own peer.

static void BtAppDiscStartChar(BtDevice_t *pDev);
static void BtAppDiscStartDesc(BtDevice_t *pDev);
static void WbaDiscSrvc(const aci_att_read_by_group_type_resp_event_rp0 *p);
static void WbaDiscChar(const aci_att_read_by_type_resp_event_rp0 *p);
static void WbaDiscDesc(const aci_att_find_info_resp_event_rp0 *p);
static void WbaDiscProcComplete(const aci_gatt_proc_complete_event_rp0 *p);

// Server side write from a connected client. ST reports CCCD writes and
// characteristic value writes through ACI_GATT_ATTRIBUTE_MODIFIED. Route a CCCD
// write to BtGattCccdSet (which stores the per connection value and fires the
// notify/indicate subscribe callbacks) and a value write to the char WrCB. This
// mirrors the nRF52 BLE_GATTS_EVT_WRITE handler.
static void WbaGattAttrModified(uint16_t ConnHdl, uint16_t AttrHdl,
								uint16_t Offset, uint8_t *pData, uint16_t Len)
{
	BtGattSrvc_t *pSrvc = BtGattGetSrvcList();

	while (pSrvc != nullptr)
	{
		for (int i = 0; i < pSrvc->NbChar; i++)
		{
			BtGattChar_t *pChar = &pSrvc->pCharArray[i];

			if (AttrHdl == pChar->CccdHdl)
			{
				if (Len == 2 && pData != nullptr)
				{
					uint16_t cccd = (uint16_t)(pData[0] | (pData[1] << 8));
					BtGattCccdSet(ConnHdl, pChar->CccdHdl, cccd);
				}
				return;
			}

			if (AttrHdl == pChar->ValHdl && pChar->WrCB != nullptr)
			{
				pChar->WrCB(pChar, pData, (int)Offset, (int)Len);
				return;
			}
		}
		pSrvc = pSrvc->pNext;
	}
}

// On a bonded reconnection the ST stack restores each subscribed client's CCCD
// from its security database but does not emit ACI_GATT_ATTRIBUTE_MODIFIED for
// the restore, so the notify/indicate re-arm callbacks never fire. Read the
// per-connection CCCD the stack restored and replay each enabled one through
// BtGattCccdSet, which fires SetNotifCB/SetIndCB. This is the WBA equivalent of
// the SDC port's BtGattCccdRestoreBonded, sourced from the stack instead of the
// IOsonata bond table (which ST owns on this port). Called on encryption enabled.
static void WbaGattCccdReArm(uint16_t ConnHdl)
{
	BtGattSrvc_t *pSrvc = BtGattGetSrvcList();

	while (pSrvc != nullptr)
	{
		for (int i = 0; i < pSrvc->NbChar; i++)
		{
			BtGattChar_t *pChar = &pSrvc->pCharArray[i];

			if (pChar->CccdHdl == BT_ATT_HANDLE_INVALID)
			{
				continue;
			}

			uint16_t length = 0;
			uint16_t valLen = 0;
			uint8_t val[2] = { 0, 0 };

			// A Connection_Handle selects the per-connection CCCD instance.
			tBleStatus st = aci_gatt_get_attribute_value(ConnHdl, pChar->CccdHdl,
														 0, sizeof(val),
														 &length, &valLen, val);
			if (st == BLE_STATUS_SUCCESS && valLen == 2)
			{
				uint16_t cccd = (uint16_t)(val[0] | (val[1] << 8));
				if (cccd != 0)
				{
					BtGattCccdSet(ConnHdl, pChar->CccdHdl, cccd);
				}
			}
		}
		pSrvc = pSrvc->pNext;
	}
}

// ===========================================================================
// SMP user interaction bridge (STM32WBA / ST host stack).
//
// The ST stack owns the SMP exchange and surfaces the user steps as ACI GAP
// events: ACI_GAP_NUMERIC_COMPARISON_VALUE (Numeric Comparison) and
// ACI_GAP_PASS_KEY_REQ (Passkey Entry, display or input depending on IO
// capability). These are bridged to the same BtSmp* interaction API the
// generic SMP host exposes, so an application sees one set of callbacks across
// ports. ST passes the passkey as a plain integer, so no ASCII conversion is
// needed. The reply functions route the user answer back through
// aci_gap_numeric_comparison_value_confirm_yesno and aci_gap_pass_key_resp.
// The generic weak defaults live in bt_smp.cpp, which is not linked on this
// port, so equivalent weak defaults are provided here.
// ===========================================================================

// Decide whether the local device displays the passkey (true) or inputs it
// (false), from the local and peer IO capabilities. KeyboardOnly inputs;
// DisplayOnly and DisplayYesNo display; KeyboardDisplay displays only against a
// KeyboardOnly peer.
static bool WbaPasskeyLocalDisplays(uint8_t LocalIo, uint8_t PeerIo)
{
	if (LocalIo == IO_CAP_KEYBOARD_ONLY)
	{
		return false;
	}
	if (LocalIo == IO_CAP_DISPLAY_ONLY || LocalIo == IO_CAP_DISPLAY_YES_NO)
	{
		return true;
	}
	// IO_CAP_KEYBOARD_DISPLAY
	return (PeerIo == IO_CAP_KEYBOARD_ONLY);
}

static void WbaPasskeyReject(uint16_t ConnHdl)
{
	s_WbaData.PasskeyRejectedHdl = ConnHdl;
	(void)aci_gap_terminate(ConnHdl, BT_APP_HCI_AUTH_FAILURE);
}

// Resume Numeric Comparison. Confirm true reports a match, false rejects and
// aborts pairing.
void BtSmpNumericComparisonReply(uint16_t ConnHdl, bool Confirm)
{
	(void)aci_gap_numeric_comparison_value_confirm_yesno(ConnHdl, Confirm ? 1 : 0);
}

// Resume Passkey Entry on the input side. A value in 0..999999 is handed to the
// stack. A value above that range cancels by terminating the pending link with
// Authentication Failure. Sending zero is not a cancellation because 000000 is
// a valid passkey.
void BtSmpPasskeyReply(uint16_t ConnHdl, uint32_t Passkey)
{
	if (Passkey > 999999u)
	{
		WbaPasskeyReject(ConnHdl);
		return;
	}

	s_WbaData.PasskeyRejectedHdl = BT_CONN_HDL_INVALID;
	(void)aci_gap_pass_key_resp(ConnHdl, Passkey);
}

// Set the local SMP IO capability and authentication requirement at runtime,
// the same entry point the SDC and BM ports expose. Maps the generic
// BT_SMP_IOCAPS_* / BT_SMP_AUTHREQ_* values to ST's IO_CAP_* and the split
// arguments of aci_gap_set_authentication_requirement, then applies them so a
// call after BtAppInit reconfigures the next pairing. An AuthReq with the SC
// bit set is configured as SC-only; legacy fallback is not permitted.
void BtSmpAuthConfig(uint8_t IoCaps, uint8_t AuthReq)
{
	uint8_t io;
	switch (IoCaps)
	{
		case BT_SMP_IOCAPS_DISPLAY_ONLY:
			io = IO_CAP_DISPLAY_ONLY;
			break;
		case BT_SMP_IOCAPS_DISPLAY_YESNO:
			io = IO_CAP_DISPLAY_YES_NO;
			break;
		case BT_SMP_IOCAPS_KEYBOARD_ONLY:
			io = IO_CAP_KEYBOARD_ONLY;
			break;
		case BT_SMP_IOCAPS_KEYBOARD_DISPLAY:
			io = IO_CAP_KEYBOARD_DISPLAY;
			break;
		case BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT:
		default:
			io = IO_CAP_NO_INPUT_NO_OUTPUT;
			break;
	}
	s_WbaData.IoCapability    = io;
	s_WbaData.AuthRequirement = AuthReq;

	uint8_t bonding   = (AuthReq & BT_SMP_AUTHREQ_BONDING_FLAG_MASK) != 0 ? 1 : 0;
	uint8_t mitm      = (AuthReq & BT_SMP_AUTHREQ_MITM) != 0 ? 1 : 0;
	uint8_t scSupport = (AuthReq & BT_SMP_AUTHREQ_SC) != 0 ? 2 : 0;
	aci_gap_set_authentication_requirement(bonding, mitm,
	                                       scSupport,
	                                       0,	// keypress not supported
	                                       SEC_PARAM_MIN_KEY_SIZE,
	                                       SEC_PARAM_MAX_KEY_SIZE,
	                                       0,	// use fixed pin = no
	                                       0,	// fixed pin value
	                                       0);	// identity address type
	aci_gap_set_io_capability(io);
}

// LE Secure Connections OOB data (strong overrides of the generic weak API).
// The ST stack owns the SC key pair: aci_gap_set_oob_data with a zero length
// makes the stack generate its OOB Random and Confirm, and aci_gap_get_oob_data
// extracts them. The peer set is staged here and pushed to the stack at
// connection time, when the peer address is known; without privacy the
// connection address is the identity address the command expects.
// The public staging API has no connection handle, so the staged set is bound
// to the first link that consumes it and cannot be reused by another pairing.
static uint8_t s_BtAppPeerOobRand[16];
static uint8_t s_BtAppPeerOobConf[16];
static bool s_BtAppPeerOobValid = false;
static uint16_t s_BtAppPeerOobHdl = BT_CONN_HDL_INVALID;

static void BtSmpOobDataClearInternal(void)
{
	s_BtAppPeerOobValid = false;
	s_BtAppPeerOobHdl = BT_CONN_HDL_INVALID;
	memset(s_BtAppPeerOobRand, 0, sizeof(s_BtAppPeerOobRand));
	memset(s_BtAppPeerOobConf, 0, sizeof(s_BtAppPeerOobConf));
}

static void BtSmpOobDataClearConn(uint16_t ConnHdl)
{
	if (s_BtAppPeerOobValid && s_BtAppPeerOobHdl == ConnHdl)
	{
		BtSmpOobDataClearInternal();
	}
}

int BtSmpOobLocalDataGen(BtHciDevice_t * const pDev, uint8_t * const pRand, uint8_t * const pConf)
{
	(void)pDev;

	if (!s_WbaData.bOobEnabled || pRand == NULL || pConf == NULL ||
		s_BtAppPeerOobHdl != BT_CONN_HDL_INVALID)
	{
		return -1;
	}

	uint8_t zaddr[6] = {0};

	// Length 0 = generate a fresh SC Random/Confirm pair in the stack.
	if (aci_gap_set_oob_data(0, 0, zaddr, 0, 0, NULL) != BLE_STATUS_SUCCESS)
	{
		return -1;
	}

	uint8_t atype;
	uint8_t addr[6];
	uint8_t len;

	// OOB_Data_Type 1 = SC Random, 2 = SC Confirm.
	if (aci_gap_get_oob_data(1, &atype, addr, &len, pRand) != BLE_STATUS_SUCCESS ||
		len != 16)
	{
		return -1;
	}
	if (aci_gap_get_oob_data(2, &atype, addr, &len, pConf) != BLE_STATUS_SUCCESS ||
		len != 16)
	{
		return -1;
	}

	return 0;
}

void BtSmpOobPeerDataSet(const uint8_t * const pRand, const uint8_t * const pConf)
{
	if (!s_WbaData.bOobEnabled || pRand == NULL || pConf == NULL)
	{
		return;
	}

	if (s_BtAppPeerOobValid && s_BtAppPeerOobHdl != BT_CONN_HDL_INVALID &&
		BtPeerFindByHdl(s_BtAppPeerOobHdl) != nullptr)
	{
		DEBUG_PRINTF("SEC: OOB data already owned by hdl=%u\r\n",
					 s_BtAppPeerOobHdl);
		return;
	}

	BtSmpOobDataClearInternal();
	memcpy(s_BtAppPeerOobRand, pRand, sizeof(s_BtAppPeerOobRand));
	memcpy(s_BtAppPeerOobConf, pConf, sizeof(s_BtAppPeerOobConf));
	s_BtAppPeerOobValid = true;
}

void BtSmpOobDataClear(void)
{
	if (s_BtAppPeerOobValid && s_BtAppPeerOobHdl != BT_CONN_HDL_INVALID &&
		BtPeerFindByHdl(s_BtAppPeerOobHdl) != nullptr)
	{
		DEBUG_PRINTF("SEC: OOB data busy on hdl=%u\r\n",
					 s_BtAppPeerOobHdl);
		return;
	}

	BtSmpOobDataClearInternal();
}

// Push the staged peer OOB data into the stack for this link. Called from the
// connection complete event, before any pairing initiation. Reserve the data
// before issuing either ACI command so a partial command failure cannot make
// the same peer values available to another link.
static void BtAppOobPeerDataPush(uint16_t ConnHdl, uint8_t AddrType,
								 const uint8_t *pAddr)
{
	if (!s_WbaData.bOobEnabled || !g_BtAppData.AppDevice.bSecure ||
		!s_BtAppPeerOobValid || pAddr == NULL)
	{
		return;
	}
	if (s_BtAppPeerOobHdl != BT_CONN_HDL_INVALID &&
		s_BtAppPeerOobHdl != ConnHdl)
	{
		DEBUG_PRINTF("SEC: OOB data busy on hdl=%u, reject hdl=%u\r\n",
					 s_BtAppPeerOobHdl, ConnHdl);
		return;
	}

	s_BtAppPeerOobHdl = ConnHdl;
	tBleStatus randStatus = aci_gap_set_oob_data(1, AddrType, pAddr, 1, 16,
											  s_BtAppPeerOobRand);
	tBleStatus confStatus = aci_gap_set_oob_data(1, AddrType, pAddr, 2, 16,
											  s_BtAppPeerOobConf);
	if (randStatus != BLE_STATUS_SUCCESS || confStatus != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("SEC: set OOB data failed hdl=%u rand=0x%02X conf=0x%02X\r\n",
					 ConnHdl, randStatus, confStatus);
	}
}

// The rejecting defaults for BtSmpNumericComparison, BtSmpPasskeyDisplay and
// BtSmpPasskeyRequest are weak in bt_smp.cpp. A second weak definition here
// left the choice to link order. Both reach this port through the strong
// BtSmpNumericComparisonReply and BtSmpPasskeyReply above, and the display
// default that called WbaPasskeyReject arrives at the same place, since
// BtSmpPasskeyReply routes an out of range passkey there.

// A link was created: LE Connection Complete or LE Enhanced Connection Complete
static void BtAppWbaConnected(uint8_t Status, uint16_t ConnHdl, uint8_t Role,
							  uint8_t PeerAddrType, uint8_t *pPeerAddr)
{
	if (Status == BLE_STATUS_SUCCESS)
	{
		// Own address not stamped: SMP runs in the ST host, not
		// the IOsonata toolbox, so the record is left for the
		// BtSmpLocalAddrGet fallback.
		BtDevice_t *pPeer = BtPeerConnected(ConnHdl,
		                   Role,
		                   PeerAddrType,
		                   pPeerAddr,
		                   0, NULL);

		if (Role == BT_CONN_ROLE_PERIPHERAL)
		{
			// A connectable advertising set stops after creating this
			// link. Update the local set state before deciding whether
			// another peripheral link may be accepted.
			BtAdvWbaConnected();
		}

		if (pPeer == nullptr)
		{
			// BtPeerConnected already routes rejection through
			// BtPeerConnectionRejected, which terminates the link.
			return;
		}

		g_BtAppData.State = BTAPP_STATE_CONNECTED;
		BtAppConnLedOn();
		BtAppEvtConnected(ConnHdl);

		// The link starts at the default ATT MTU of 23. Ask for the larger one
		// the application configured. The server answer raises the link MTU, see
		// ACI_ATT_EXCHANGE_MTU_RESP below.
		if (g_BtAppData.AppDevice.Conn.MaxMtu > GATT_MTU_SIZE_DEFAULT)
		{
			(void)aci_gatt_exchange_config(ConnHdl);
		}

		// Secure link setup. ST owns the SMP exchange and emits
		// ACI_GAP_PAIRING_COMPLETE, which surfaces below as
		// BtAppEvtSecured. A central starts pairing directly; a
		// peripheral asks the central with a Security Request.
		if (g_BtAppData.AppDevice.bSecure)
		{
			// Only a LESC link configured for OOB may claim the
			// single staged peer OOB set.
			if (s_WbaData.bOobEnabled)
			{
				BtAppOobPeerDataPush(ConnHdl,
				                     PeerAddrType,
				                     pPeerAddr);
			}

			switch (Role)
			{
				case BT_CONN_ROLE_CENTRAL:
					aci_gap_send_pairing_req(ConnHdl, 0);
					break;

				case BT_CONN_ROLE_PERIPHERAL:
					aci_gap_peripheral_security_req(ConnHdl);
					break;

				default:
					break;
			}
		}

		if (Role == BT_CONN_ROLE_PERIPHERAL)
		{
			BtAdvStart();
		}
	}
}

// LE Advertising Report, wire layout: number of reports, then per report
// event type, address type, address, data length, data, RSSI. The stack sends
// one report per event.
static void BtAppWbaScanReport(const uint8_t *pData, size_t Len)
{
	if (g_BtAppData.bScan == false || Len < 12)
	{
		return;
	}

	uint8_t nb = pData[0];
	size_t pos = 1;

	for (uint8_t i = 0; i < nb; i++)
	{
		if (pos + 9 > Len)
		{
			return;
		}

		uint8_t addrType = pData[pos + 1];
		uint8_t addr[6];
		uint8_t advLen = pData[pos + 8];

		memcpy(addr, &pData[pos + 2], 6);
		if (pos + 9 + advLen + 1 > Len)
		{
			return;
		}

		int8_t rssi = (int8_t)pData[pos + 9 + advLen];

		if (BtAppScanReport(rssi, addrType, addr, advLen,
							(uint8_t *)&pData[pos + 9]) == false)
		{
			BtAppScanStop();
			return;
		}
		pos += 9 + advLen + 1;
	}
}

// LE Extended Advertising Report. The stack sends one report per event.
static void BtAppWbaExtScanReport(const uint8_t *pData, size_t Len)
{
	if (g_BtAppData.bScan == false ||
		Len < offsetof(hci_le_extended_advertising_report_event_rp0, Data))
	{
		return;
	}

	const hci_le_extended_advertising_report_event_rp0 *p =
		(const hci_le_extended_advertising_report_event_rp0 *)pData;

	if (offsetof(hci_le_extended_advertising_report_event_rp0, Data) +
		p->Data_Length > Len)
	{
		return;
	}

	uint8_t addr[6];

	memcpy(addr, p->Address, 6);
	if (BtAppScanReport((int8_t)p->RSSI, p->Address_Type, addr, p->Data_Length,
						(uint8_t *)p->Data) == false)
	{
		BtAppScanStop();
	}
}

// HCI event callback registered with the stack. ST's stack invokes this
// for every HCI / vendor event; we route by type to the appropriate
// handler. Adv-report events go to the scan path (handled in
// bt_scan_stm32wba.cpp via a separately registered observer).
static BtWbaEvtFlow_t BtAppHciEvtHandler(void *pPayload)
{
	hci_event_pckt *pEvtPkt = (hci_event_pckt *)((hci_uart_pckt *)pPayload)->data;

	switch (pEvtPkt->evt)
	{
		case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
		{
			hci_disconnection_complete_event_rp0 *p =
				(hci_disconnection_complete_event_rp0 *)pEvtPkt->data;
			if (p->Status == BLE_STATUS_SUCCESS)
			{
				if (s_WbaData.PasskeyRejectedHdl == p->Connection_Handle)
				{
					s_WbaData.PasskeyRejectedHdl = BT_CONN_HDL_INVALID;
				}
				BtSmpOobDataClearConn(p->Connection_Handle);
				BtDevice_t *pPeer = BtPeerFindByHdl(p->Connection_Handle);
				if (pPeer != nullptr)
				{
					BtPeerFree(pPeer);
					bool connected = BtPeerIsConnected();

					if (connected)
					{
						g_BtAppData.State = BTAPP_STATE_CONNECTED;
						BtAppConnLedOn();
					}
					else
					{
						g_BtAppData.State = BTAPP_STATE_IDLE;
						BtAppConnLedOff();
					}

					BtAppEvtDisconnected(p->Connection_Handle);
				}

				// A disconnect can free either peripheral capacity or the last
				// peer-table slot. BtAdvStart performs both checks and is a no-op
				// when the advertising set is already active.
				BtAdvStart();
			}
			break;
		}

		case HCI_ENCRYPTION_CHANGE_EVT_CODE:
		{
			hci_encryption_change_event_rp0 *p =
				(hci_encryption_change_event_rp0 *)pEvtPkt->data;
			// Link encrypted. On a bonded reconnect the stack has restored the
			// peer's CCCDs by this point, so re-arm the notify/indicate callbacks.
			// On a fresh pairing the CCCDs are still clear, so this is a no-op and
			// the later client subscribe drives the callbacks instead.
			if (p->Status == 0 && p->Encryption_Enabled != 0 &&
				BtPeerFindByHdl(p->Connection_Handle) != nullptr)
			{
				WbaGattCccdReArm(p->Connection_Handle);
			}
			break;
		}

		case HCI_LE_META_EVT_CODE:
		{
			evt_le_meta_event *pMeta = (evt_le_meta_event *)pEvtPkt->data;
			switch (pMeta->subevent)
			{
				case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
				{
					hci_le_connection_complete_event_rp0 *p =
						(hci_le_connection_complete_event_rp0 *)pMeta->data;
					BtAppWbaConnected(p->Status, p->Connection_Handle, p->Role,
									  p->Peer_Address_Type, p->Peer_Address);
					break;
				}

				case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
				{
					// Sent instead of LE Connection Complete when the stack has
					// both unmasked, which is the case with privacy on.
					hci_le_enhanced_connection_complete_event_rp0 *p =
						(hci_le_enhanced_connection_complete_event_rp0 *)pMeta->data;
					BtAppWbaConnected(p->Status, p->Connection_Handle, p->Role,
									  p->Peer_Address_Type, p->Peer_Address);
					break;
				}

				case HCI_LE_ADVERTISING_SET_TERMINATED_SUBEVT_CODE:
					if (pEvtPkt->plen >= 1 + sizeof(
							hci_le_advertising_set_terminated_event_rp0))
					{
						auto *p = (hci_le_advertising_set_terminated_event_rp0 *)pMeta->data;
						if (p->Advertising_Handle == g_BtAppData.AdvHdl &&
							p->Status != BLE_STATUS_SUCCESS)
						{
							// Clear advertising state before the application can restart it.
							BtAdvWbaConnected();
							if (p->Status == HCI_ADVERTISING_TIMEOUT_ERR_CODE ||
								p->Status == HCI_ADVERTISING_LIMIT_REACHED_ERR_CODE)
							{
								BtAppAdvTimeoutHandler();
							}
						}
					}
					break;

				case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
					BtAppWbaScanReport(pMeta->data, (size_t)pEvtPkt->plen - 1);
					break;

				case HCI_LE_EXTENDED_ADVERTISING_REPORT_SUBEVT_CODE:
					BtAppWbaExtScanReport(pMeta->data, (size_t)pEvtPkt->plen - 1);
					break;

				case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
					// Conn-param updates handled by bt_cp_stm32wba.cpp.
					break;

				default:
					break;
			}
			break;
		}

		case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
		{
			evt_blecore_aci *pAci = (evt_blecore_aci *)pEvtPkt->data;
			switch (pAci->ecode)
			{
				case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
					WbaDiscSrvc((aci_att_read_by_group_type_resp_event_rp0 *)pAci->data);
					break;

				case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
					WbaDiscChar((aci_att_read_by_type_resp_event_rp0 *)pAci->data);
					break;

				case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
					WbaDiscDesc((aci_att_find_info_resp_event_rp0 *)pAci->data);
					break;

				case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
				{
					aci_att_exchange_mtu_resp_event_rp0 *pMtu =
						(aci_att_exchange_mtu_resp_event_rp0 *)pAci->data;
					BtDevice_t *pPeer = BtPeerFindByHdl(pMtu->Connection_Handle);

					if (pPeer != nullptr)
					{
						uint16_t mtu = pMtu->Server_RX_MTU;

						if (mtu > g_BtAppData.AppDevice.Conn.MaxMtu)
						{
							mtu = g_BtAppData.AppDevice.Conn.MaxMtu;
						}
						if (mtu >= GATT_MTU_SIZE_DEFAULT)
						{
							pPeer->Conn.MaxMtu = mtu;
						}
					}
					break;
				}

				case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
					WbaDiscProcComplete((aci_gatt_proc_complete_event_rp0 *)pAci->data);
					break;

				case ACI_GATT_NOTIFICATION_VSEVT_CODE:
				{
					aci_gatt_notification_event_rp0 *pN =
						(aci_gatt_notification_event_rp0 *)pAci->data;
					BtGattClientNotified(pN->Connection_Handle, pN->Attribute_Handle,
										 pN->Attribute_Value, pN->Attribute_Value_Length);
					break;
				}

				case ACI_GATT_INDICATION_VSEVT_CODE:
				{
					aci_gatt_indication_event_rp0 *pI =
						(aci_gatt_indication_event_rp0 *)pAci->data;
					BtGattClientNotified(pI->Connection_Handle, pI->Attribute_Handle,
										 pI->Attribute_Value, pI->Attribute_Value_Length);
					// ST requires the client to confirm a received indication.
					aci_gatt_confirm_indication(pI->Connection_Handle);
					break;
				}

				case ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE:
				{
					// The client confirmed our indication. Clear the pending state
					// and drain the send-order ring so TxCompleteCB fires.
					uint16_t ch = *(uint16_t*)pAci->data;
					BtGattHandleValueConfirm(ch);
					BtGattSendCompleted(ch, 1);
					break;
				}

				case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
				{
					aci_gatt_attribute_modified_event_rp0 *pM =
						(aci_gatt_attribute_modified_event_rp0 *)pAci->data;
					WbaGattAttrModified(pM->Connection_Handle, pM->Attr_Handle,
										pM->Offset, pM->Attr_Data, pM->Attr_Data_Length);
					break;
				}

				case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
				{
					aci_gap_pairing_complete_event_rp0 *pPc =
						(aci_gap_pairing_complete_event_rp0 *)pAci->data;
					if (s_WbaData.PasskeyRejectedHdl == pPc->Connection_Handle)
					{
						s_WbaData.PasskeyRejectedHdl = BT_CONN_HDL_INVALID;
					}
					BtSmpOobDataClearConn(pPc->Connection_Handle);
					if (pPc->Status == 0)
					{
						BtAppEvtSecured(pPc->Connection_Handle);
					}
					break;
				}

				case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
				{
					aci_gap_numeric_comparison_value_event_rp0 *pNc =
						(aci_gap_numeric_comparison_value_event_rp0 *)pAci->data;
					// LESC Numeric Comparison: the application confirms the
					// match through BtSmpNumericComparisonReply.
					BtSmpNumericComparison(pNc->Connection_Handle, pNc->Numeric_Value);
					break;
				}

				case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
				{
					aci_gap_pass_key_req_event_rp0 *pPk =
						(aci_gap_pass_key_req_event_rp0 *)pAci->data;
					uint16_t connHdl = pPk->Connection_Handle;
					s_WbaData.PasskeyRejectedHdl = BT_CONN_HDL_INVALID;
					// The event reports the peer IO capability; the local one is
					// the configured value.
					if (WbaPasskeyLocalDisplays(s_WbaData.IoCapability, pPk->IO_Capability))
					{
						// Display side: generate a six digit passkey, show it,
						// and hand the same value to the stack unless the callback
						// rejected synchronously.
						uint8_t rnd[8] = {0};
						(void)BtWbaLeRand(rnd);
						uint32_t val = ((uint32_t)rnd[0] | ((uint32_t)rnd[1] << 8) |
										((uint32_t)rnd[2] << 16) | ((uint32_t)rnd[3] << 24)) % 1000000u;
						BtSmpPasskeyDisplay(connHdl, val);
						if (s_WbaData.PasskeyRejectedHdl != connHdl)
						{
							(void)aci_gap_pass_key_resp(connHdl, val);
						}
					}
					else
					{
						// Input side: the application provides the value the user
						// reads from the peer through BtSmpPasskeyReply.
						BtWbaPasskeyRequest(connHdl);
					}
					break;
				}

				default:
					break;
			}
			break;
		}

		default:
			break;
	}

	return BTWBA_EVT_FLOW_ENABLE;
}

// --- Stack sizing ---
//
// Every number can come from the application's app_conf.h in ST's own macro
// names; what is not said gets a default that fits the examples. The buffer
// sizes then follow from ble_bufsize.h, so the numbers are stated once and
// the buffers cannot disagree with them.

#ifndef CFG_BLE_NUM_LINK
#define CFG_BLE_NUM_LINK				2
#endif
#ifndef CFG_BLE_NUM_GATT_SERVICES
#define CFG_BLE_NUM_GATT_SERVICES		8
#endif
#ifndef CFG_BLE_NUM_GATT_ATTRIBUTES
#define CFG_BLE_NUM_GATT_ATTRIBUTES		68
#endif
#ifndef CFG_BLE_ATT_VALUE_ARRAY_SIZE
#define CFG_BLE_ATT_VALUE_ARRAY_SIZE	1344
#endif
#ifndef CFG_BLE_ATT_MTU_MAX
#define CFG_BLE_ATT_MTU_MAX				251
#endif
#ifndef CFG_BLE_PREPARE_WRITE_LIST_SIZE
#define CFG_BLE_PREPARE_WRITE_LIST_SIZE	BLE_DEFAULT_PREP_WRITE_LIST_SIZE
#endif
#ifndef CFG_BLE_MBLOCK_COUNT
#define CFG_BLE_MBLOCK_COUNT			BLE_MBLOCKS_CALC(CFG_BLE_PREPARE_WRITE_LIST_SIZE, \
										CFG_BLE_ATT_MTU_MAX, CFG_BLE_NUM_LINK)
#endif
#ifndef CFG_BLE_OPTIONS
#define CFG_BLE_OPTIONS					0
#endif

// The stack's NVM cache: bonds and GATT server state, preloaded from the
// PDS before init and persisted through BLEPLAT_NvmStore. In 64 bit words,
// 1..1024 per blestack.h.
#ifndef CFG_BLE_NVM_MAX_SIZE
#define CFG_BLE_NVM_MAX_SIZE			512
#endif

// Host event FIFO, in 16 bit words.
#ifndef CFG_BLE_HOST_EVT_FIFO_SIZE
#define CFG_BLE_HOST_EVT_FIFO_SIZE		256
#endif

// The RAM the stack runs in, sized by the middleware's own arithmetic from
// the numbers above. 32 bit aligned per blestack.h.
static uint32_t s_BleStackRam[DIVC(BLE_TOTAL_BUFFER_SIZE(CFG_BLE_NUM_LINK,
									CFG_BLE_MBLOCK_COUNT, 0), 4)];
static uint32_t s_BleGattRam[DIVC(BLE_TOTAL_BUFFER_SIZE_GATT(
									CFG_BLE_NUM_GATT_ATTRIBUTES,
									CFG_BLE_NUM_GATT_SERVICES,
									CFG_BLE_ATT_VALUE_ARRAY_SIZE), 4)];
static uint64_t s_BleNvmCache[CFG_BLE_NVM_MAX_SIZE];
static uint16_t s_BleHostEvtFifo[CFG_BLE_HOST_EVT_FIFO_SIZE];

// Preload the cache from the PDS and say how many words came back.
// Implemented in bt_smp_bond_stm32wba.cpp; weak no-op here so a build
// without security still links and starts with an empty cache.
extern "C" __attribute__((weak))
uint16_t BtSmpBondWbaCacheInit(uint64_t *pCache, uint16_t MaxWords)
{
	(void)pCache;
	(void)MaxWords;
	return 0;
}

// PA level of aci_hal_set_tx_power_level, from the STM32WBA BLE Wireless
// Interface Annex: output power at the IC pin in tenths of dBm, index is the
// PA level 0x00 to 0x23.
static const int16_t s_PaLevelTenthDbm[] = {
	-204, -204, -204, -204, -204, -204, -194, -183, -173, -164,
	-154, -143, -133, -123, -114, -104, -95, -84, -75, -65,
	-54, -45, -35, -24, -15, -3, 9, 23, 28, 39,
	48, 56, 69, 75, 85, 100
};

// Closest PA level for a power in dBm. A request outside the range of the
// radio selects the first or last level.
static uint8_t BtAppWbaPaLevel(int8_t Dbm)
{
	int target = (int)Dbm * 10;
	uint8_t best = 0;
	int bestDiff = 10000;

	for (uint8_t i = 0; i < sizeof(s_PaLevelTenthDbm) / sizeof(s_PaLevelTenthDbm[0]); i++)
	{
		int d = target - s_PaLevelTenthDbm[i];

		if (d < 0)
		{
			d = -d;
		}
		if (d < bestDiff)
		{
			bestDiff = d;
			best = i;
		}
	}

	return best;
}

// Device identity. The application overrides this to give its own public
// address and keys: pAddr is the public address, little endian, pIr and pEr
// are the 16 byte identity and encryption roots. The default derives all three
// from the 64 bit unique ID of the chip, so each chip has its own address, but
// the keys are not secret. A product that pairs with bonding should override
// this with keys of its own.
extern "C" __attribute__((weak))
void BtAppWbaIdentityGet(uint8_t *pAddr, uint8_t *pIr, uint8_t *pEr)
{
	uint32_t udn = LL_FLASH_GetUDN();
	uint32_t devid = LL_FLASH_GetDeviceID();
	uint32_t company = LL_FLASH_GetSTCompanyID();

	// Layout of the Cube reference application: company ID in the upper three
	// bytes, device ID, then the last 16 bits of the unique device number. A
	// chip without a unique number gets the fixed ST default address.
	if (udn != 0xFFFFFFFFUL)
	{
		pAddr[0] = (uint8_t)udn;
		pAddr[1] = (uint8_t)(udn >> 8);
		pAddr[2] = (uint8_t)devid;
		pAddr[3] = (uint8_t)company;
		pAddr[4] = (uint8_t)(company >> 8);
		pAddr[5] = (uint8_t)(company >> 16);
	}
	else
	{
		pAddr[0] = 0;
		pAddr[1] = 0;
		pAddr[2] = 0;
		pAddr[3] = 0xE1;
		pAddr[4] = 0x80;
		pAddr[5] = 0;
	}

	uint32_t seed = udn ^ (devid << 24) ^ (company << 7) ^ 0x6C078965UL;

	for (int i = 0; i < 16; i++)
	{
		seed = seed * 1664525UL + 1013904223UL;
		pIr[i] = (uint8_t)(seed >> 24);
		seed = seed * 1664525UL + 1013904223UL;
		pEr[i] = (uint8_t)(seed >> 24);
	}
}

static void BtAppWbaIdentitySet(void)
{
	uint8_t addr[CONFIG_DATA_PUBLIC_ADDRESS_LEN];
	uint8_t ir[CONFIG_DATA_IR_LEN];
	uint8_t er[CONFIG_DATA_ER_LEN];

	BtAppWbaIdentityGet(addr, ir, er);

	aci_hal_write_config_data(CONFIG_DATA_PUBLIC_ADDRESS_OFFSET,
							  CONFIG_DATA_PUBLIC_ADDRESS_LEN, addr);
	aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, ir);
	aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, er);

	memset(ir, 0, sizeof(ir));
	memset(er, 0, sizeof(er));
}

// --- Public API ---

bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	if (s_WbaData.bStackInited)
	{
		return true;
	}

	// The system clock is set by SystemInit (system_stm32wbaxx.c), so the Cube
	// System Clock Manager is not used: build the project with CFG_SCM_SUPPORTED
	// set to 0. The radio needs HSE32, select it with g_McuOsc. The link layer
	// is started by BleStack_Init through the Target files of the project.

	// Public-key accelerator for LESC pairing. Safe to init even if SMP
	// isn't enabled; the stack just won't use it.
	PKACTRL_Reset();

	// The bond store mounts and preloads the stack's NVM cache before the
	// stack initializes, because BleStack_init_t takes the preloaded word
	// count: nvm_cache_size is what the restore answered. A first boot, a
	// torn flush or a store that would not mount all answer 0 and the
	// stack starts empty. Guarded on the security type the way the Nordic
	// ports guard on bSecure; without security the weak no-op answers and
	// nothing is pulled in.
	if (pCfg->SecType != BTGAP_SECTYPE_NONE)
	{
		s_WbaData.NvmCacheWords =
				BtSmpBondWbaCacheInit(s_BleNvmCache, CFG_BLE_NVM_MAX_SIZE);
	}

	// Host stack bring-up, sized by the numbers above; an app_conf.h that
	// says its own CFG_BLE_* wins over every default.
	BleStack_init_t init;

	memset(&init, 0, sizeof(init));

	init.bleStartRamAddress = (uint8_t *)s_BleStackRam;
	init.total_buffer_size = sizeof(s_BleStackRam);
	init.bleStartRamAddress_GATT = (uint8_t *)s_BleGattRam;
	init.total_buffer_size_GATT = sizeof(s_BleGattRam);
	init.nvm_cache_buffer = s_BleNvmCache;
	init.nvm_cache_size = s_WbaData.NvmCacheWords;
	init.nvm_cache_max_size = CFG_BLE_NVM_MAX_SIZE;
	init.host_event_fifo_buffer = s_BleHostEvtFifo;
	init.host_event_fifo_buffer_size = CFG_BLE_HOST_EVT_FIFO_SIZE;
	init.numAttrRecord = CFG_BLE_NUM_GATT_ATTRIBUTES;
	init.numAttrServ = CFG_BLE_NUM_GATT_SERVICES;
	init.attrValueArrSize = CFG_BLE_ATT_VALUE_ARRAY_SIZE;
	init.numOfLinks = CFG_BLE_NUM_LINK;
	init.prWriteListSize = CFG_BLE_PREPARE_WRITE_LIST_SIZE;
	init.mblockCount = CFG_BLE_MBLOCK_COUNT;
	init.attMtu = CFG_BLE_ATT_MTU_MAX;
	init.options = CFG_BLE_OPTIONS;

	BtWbaPlatformInit();

	uint8_t ret = BleStack_Init(&init);
	if (ret != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("BleStack_Init failed: 0x%02x\r\n", ret);
		return false;
	}

	// Register the central HCI event handler behind the event router.
	BtWbaEventRouterRegister(BtAppHciEvtHandler);

	// Device identity: public address and the identity and encryption roots.
	// The stack is reset by BleStack_Init, so they are written after it.
	BtAppWbaIdentitySet();

	// TX power in dBm, mapped to the closest PA level of the stack. Zero keeps
	// the stack default, which is the maximum level.
	if (pCfg->TxPower != 0)
	{
		aci_hal_set_tx_power_level(0, BtAppWbaPaLevel(pCfg->TxPower));
	}

	s_WbaData.bStackInited = true;
	return true;
}

/**
 * @brief	Start the security module.
 *
 * Called by the application, normally from BtAppInitUserData. Uses the
 * SecType and SecExchg given to BtAppInit. A pairing configuration set with
 * BtSmpAuthConfig after this call replaces the one derived from them.
 *
 * @return	true - security started
 */
bool BtAppSecInit(void)
{
	if (g_BtAppData.bSecInit)
	{
		// Already started
		return true;
	}

	if (s_WbaData.bStackInited == false)
	{
		// BtAppInit has not brought up the stack yet
		return false;
	}

	// SMP / pairing config. Map BtGap SecType to ST auth requirement.
	uint8_t authReq      = 0;	// bonding NO, MITM NO by default
	uint8_t scSupport    = 0;	// LE Secure Connections off by default
	switch (g_BtAppData.SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			authReq = 0x05;	// bonding + MITM
			break;
		case BTGAP_SECTYPE_LESC_MITM:
			authReq   = 0x05;
			scSupport = 2;	// SC-only, no legacy fallback
			break;
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			authReq = 0x01;	// bonding, no MITM (encryption without authentication)
			break;
		case BTGAP_SECTYPE_NONE:
		default:
			authReq = 0;
			break;
	}
	s_WbaData.AuthRequirement = authReq;

	aci_gap_set_authentication_requirement(authReq & 0x01,	// bonding
	                                       (authReq >> 2) & 0x01,	// MITM
	                                       scSupport,
	                                       0,	// keypress not supported
	                                       SEC_PARAM_MIN_KEY_SIZE,
	                                       SEC_PARAM_MAX_KEY_SIZE,
	                                       0,	// use fixed pin = no
	                                       0,	// fixed pin value
	                                       0);	// identity address type

	// Map the application key-exchange capability to the ST IO capability so the
	// MITM models (Numeric Comparison, Passkey Entry) can be selected. With no
	// exchange bits the configured default (No Input No Output) stays.
	uint8_t ioExchg = g_BtAppData.SecExchg &
			(BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO);
	switch (ioExchg)
	{
		case BTAPP_SECEXCHG_KEYBOARD:
			s_WbaData.IoCapability = IO_CAP_KEYBOARD_ONLY;
			break;
		case BTAPP_SECEXCHG_DISPLAY:
			s_WbaData.IoCapability = IO_CAP_DISPLAY_ONLY;
			break;
		case (BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO):
			s_WbaData.IoCapability = IO_CAP_DISPLAY_YES_NO;
			break;
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY):
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO):
			s_WbaData.IoCapability = IO_CAP_KEYBOARD_DISPLAY;
			break;
		default:
			break;
	}

	aci_gap_set_io_capability(s_WbaData.IoCapability);

	g_BtAppData.bSecInit = true;

	return true;
}

static bool BtAppTickStart(void);

bool BtAppInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	BtSmpOobDataClearInternal();
	s_WbaData.PasskeyRejectedHdl = BT_CONN_HDL_INVALID;
	s_WbaData.bOobEnabled =
		pCfg->SecType == BTGAP_SECTYPE_LESC_MITM &&
		(pCfg->SecExchg & BTAPP_SECEXCHG_OOB) != 0;

	// Initialize peer manager.
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}

	if (pCfg->PeriphDevMax + pCfg->CentralDevMax > (int)BtPeerCount())
	{
		// Peer pool holds fewer slots than the number of links requested.
		// Provide a larger pool, see g_BtPeerPoolCfg in bt_peer.h
		return false;
	}

	// Connection pool removed: the peer manager (BtPeerInit above) owns
	// the single connection table now.
	// Populate generic app data from cfg.
	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	g_BtAppData.AdvHdl          = 0;	// WBA stack manages adv internally
	g_BtAppData.bExtAdv         = false;	// decided by the advertising encoder
	g_BtAppData.ConnLedPort     = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin      = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;
	g_BtAppData.bScan           = false;
	g_BtAppData.AppDevice.VendorId   = pCfg->VendorId;
	g_BtAppData.AppDevice.ProductId  = pCfg->ProductId;
	g_BtAppData.AppDevice.ProductVer = pCfg->ProductVer;
	g_BtAppData.AppDevice.Appearance = pCfg->Appearance;

	g_BtAppData.AppDevice.Conn.MaxMtu = GATT_MTU_SIZE_DEFAULT;
	if (pCfg->MaxMtu > GATT_MTU_SIZE_DEFAULT)
	{
		g_BtAppData.AppDevice.Conn.MaxMtu = pCfg->MaxMtu;
	}

	// Connection LED.
	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
	{
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
		            IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		BtAppConnLedOff();
	}

	// Bring up the stack.
	if (BtAppStackInit(pCfg) == false)
	{
		DEBUG_PRINTF("BtAppStackInit failed\r\n");
		return false;
	}

	// GATT must be initialized before GAP - ST's stack uses GATT handles
	// internally during GAP service creation.
	uint8_t ret = aci_gatt_init();
	if (ret != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("aci_gatt_init: 0x%02x\r\n", ret);
		return false;
	}

	// GAP init creates the GAP service and returns its handles. ST puts
	// device-name + appearance chars in the service automatically.
	uint16_t gapSrvcHdl, devNameHdl, appearanceHdl;
	uint8_t role = 0;
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)  role |= GAP_PERIPHERAL_ROLE;
	if (pCfg->Role & BTAPP_ROLE_CENTRAL)     role |= GAP_CENTRAL_ROLE;
	if (pCfg->Role & BTAPP_ROLE_OBSERVER)    role |= GAP_OBSERVER_ROLE;
	if (pCfg->Role & BTAPP_ROLE_BROADCASTER) role |= GAP_BROADCASTER_ROLE;

	ret = BtWbaGapInitCapture(role,
	                   0x00,	// privacy disabled by default
	                   (uint8_t)(pCfg->pDevName ? strlen(pCfg->pDevName) : 0),
	                   &gapSrvcHdl, &devNameHdl, &appearanceHdl);
	if (ret != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("aci_gap_init: 0x%02x\r\n", ret);
		return false;
	}
	s_WbaData.GattSrvcStartHdl = gapSrvcHdl;

	// Push the device name into the GAP service char value.
	if (pCfg->pDevName != NULL)
	{
		BtWbaGattUpdateCharValue(gapSrvcHdl, devNameHdl,
		                           0, (uint8_t)strlen(pCfg->pDevName),
		                           (uint8_t *)pCfg->pDevName);
	}

	// Appearance.
	uint16_t appearance = pCfg->Appearance;
	BtWbaGattUpdateCharValue(gapSrvcHdl, appearanceHdl,
	                           0, sizeof(appearance), (uint8_t *)&appearance);

	// Store the app security type so the GATT layer can gate characteristic
	// permissions before user services are registered.
	g_BtAppData.SecType = pCfg->SecType;

	// The pairing configuration is set by BtAppSecInit, which the application
	// calls from BtAppInitUserData when it uses security.
	g_BtAppData.SecExchg = pCfg->SecExchg;
	g_BtAppData.bSecInit = false;

	// Generic GAP init - sets up the generic-layer GAP/GATT service mirror.
	BtGapCfg_t gapcfg = {
		.Role            = pCfg->Role,
		.SecType         = pCfg->SecType,
		.AdvInterval     = pCfg->AdvInterval,
		.AdvTimeout      = pCfg->AdvTimeout,
		.ConnIntervalMin = pCfg->ConnIntervalMin,
		.ConnIntervalMax = pCfg->ConnIntervalMax,
		.SlaveLatency    = BT_GAP_CONN_SLAVE_LATENCY,
		.SupTimeout      = BT_GAP_CONN_SUP_TIMEOUT
	};
	// The GAP and GATT services are the table a client reads first. Without
	// them there is nothing to advertise, so this stops here.
	if (BtGapInit(&gapcfg) == false)
	{
		return false;
	}

	if (pCfg->pDevName != NULL)
	{
		BtGapSetDevName(pCfg->pDevName);
	}

	// User services + DIS + user data, in the order the BM port uses.
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		BtAppInitUserServices();
	}

	if (pCfg->pDevInfo != NULL)
	{
		BtDisInit((const struct __Bt_App_Cfg *)pCfg);
	}

	BtAppInitUserData();

	// Security is started only when the application calls BtAppSecInit,
	// normally from BtAppInitUserData above. A configuration that asks for
	// security without starting it must not run unprotected.
	if (pCfg->SecType != BTGAP_SECTYPE_NONE && g_BtAppData.bSecInit == false)
	{
		DEBUG_PRINTF("BtAppInit FAIL: SecType=%d but BtAppSecInit was not called\r\n",
					 (int)pCfg->SecType);
		return false;
	}

	g_BtAppData.AppDevice.bSecure = (pCfg->SecType != BTGAP_SECTYPE_NONE);

	// Advertising init - actual ACI calls live in bt_adv_stm32wba.cpp.
	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		if (BtAppAdvInit(pCfg) == false)
		{
			DEBUG_PRINTF("BtAppAdvInit failed\r\n");
			return false;
		}
	}

	g_BtAppData.State = BTAPP_STATE_INITIALIZED;

	if (BtAppTickStart() == false)
	{
		DEBUG_PRINTF("BtAppInit: timeout check timer failed\r\n");
		return false;
	}

	// Advertising starts here and not from the event queue: an application
	// interrupt can fill the queue before the application runs it.
	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAdvStart();
	}

	DEBUG_PRINTF("BtAppInit: success\r\n");
	return true;
}

// Application tick: the millisecond clock of the SMP and GATT transaction
// timeouts and their one second check. It is the timer device and not the Timer
// class, the same as the Nordic port. DevNo 1 is the second low power timer
// (LPTIM2), see timer_stm32.h.
static TimerDev_t s_BtAppTimer;

enum {
	BT_APP_TICK_IDLE,
	BT_APP_TICK_PENDING,
	BT_APP_TICK_QUEUED,
};

// Keep a refused timer event pending until the Bluetooth queue has room.
static volatile uint8_t s_BtAppTickState = BT_APP_TICK_IDLE;

// Checks that need time to pass, queued once per second by the timer:
// connection parameter requests that have come due and the indication
// transaction timeout (Core Vol 3 Part F 3.3.3). The ST host runs SMP and its
// pairing timeout, so there is no generic SMP check here.
static void BtAppTickEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_BtAppTickState = BT_APP_TICK_IDLE;
	BtGapWbaConnParamProcess();
	BtGattIndicationTimeoutCheck();
}

static void BtAppTickQue(void)
{
	if (s_BtAppTickState == BT_APP_TICK_PENDING)
	{
		s_BtAppTickState = BT_APP_TICK_QUEUED;
		if (BtEvtQue(0, nullptr, BtAppTickEvt) == false)
		{
			s_BtAppTickState = BT_APP_TICK_PENDING;
		}
	}
}

// Timer trigger, interrupt context
static void BtAppTimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
	(void)pTimer;

	if (Evt & TIMER_EVT_TRIGGER(0))
	{
		if (s_BtAppTickState == BT_APP_TICK_IDLE)
		{
			s_BtAppTickState = BT_APP_TICK_PENDING;
		}
		BtAppTickQue();
	}
}

static const TimerCfg_t s_BtAppTimerCfg = {
	.DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtAppTimerHandler
};

// Millisecond clock for the generic SMP/GATT transaction timeouts, overriding
// the weak BtSmpMsTick/BtGattMsTick defaults. Declared in bt_smp.h / bt_gatt.h,
// so no linkage specifier is needed here.
uint32_t BtSmpMsTick(void)
{
	if (s_BtAppTimer.GetTickCount == nullptr)
	{
		// Timer not started
		return 0;
	}

	return TimerGetMilisecond(&s_BtAppTimer);
}

uint32_t BtGattMsTick(void)
{
	return BtSmpMsTick();
}

// Spec-strict indication transaction timeout: Core Vol 3 Part F 3.3.3 requires
// closing the bearer, so terminate the link. Overrides the generic weak
// default that only clears the outstanding-indication flag.
void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	aci_gap_terminate(ConnHdl, 0x13);	// remote user terminated connection
}

// Start the periodic timeout check.
static bool BtAppTickStart(void)
{
	if (TimerInit(&s_BtAppTimer, &s_BtAppTimerCfg) == false)
	{
		return false;
	}

	msTimerEnableTrigger(&s_BtAppTimer, 0, BT_APP_TICK_PERIOD_MS,
						 TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);

	return true;
}

// BtAppNotify/BtAppIndicate, their Conn and All forms are the shared weak
// implementations in src/bluetooth/bt_app.cpp. This port used to carry a copy
// that sent to the active connection. What stays here is the disconnect
// command, which is the one part only this port can issue.

void BtAppDisconnectConn(uint16_t ConnHdl)
{
	if (ConnHdl != BT_CONN_HDL_INVALID)
	{
		aci_gap_terminate(ConnHdl, 0x13);	// remote user term
	}
}

void BtAppGapDeviceNameSet(const char *pDeviceName)
{
	if (pDeviceName == NULL || s_WbaData.GattSrvcStartHdl == 0)
	{
		return;
	}

	// Device name char is at offset 0x0002 from the GAP service start
	// handle in ST's standard GAP service layout.
	uint16_t devNameHdl = s_WbaData.GattSrvcStartHdl + 2;
	BtWbaGattUpdateCharValue(s_WbaData.GattSrvcStartHdl, devNameHdl,
	                           0, (uint8_t)strlen(pDeviceName),
	                           (uint8_t *)pDeviceName);
	BtGapSetDevName(pDeviceName);
}

void BtAppEnterDfu(void)
{
	// STM32WBA exposes the system bootloader at 0x0BF88000. Apps that
	// want OTA DFU typically jump there via a vector remap. Implementation
	// is board-specific - left to the app.
}

// Extract a 16-bit UUID (or the 16-bit alias of a 128-bit UUID) from an event
// field that is Len bytes long, little-endian on air. A 128-bit UUID holds
// its 16-bit alias at byte offset 12..13 (standard base layout, also used by
// the BlueIO base), matching the 16-bit storage the GATT DB uses.
static uint16_t WbaUuid16(const uint8_t *pUuid, uint8_t Len)
{
	if (Len <= 2)
	{
		return (uint16_t)(pUuid[0] | (pUuid[1] << 8));
	}
	return (uint16_t)(pUuid[12] | (pUuid[13] << 8));
}

// Parse one ACI_ATT_READ_BY_GROUP_TYPE_RESP (primary service discovery). Each
// entry is Start(2) End(2) UUID(2 or 16); entry length is Attribute_Data_Length
// (6 for 16-bit, 20 for 128-bit).
static void WbaDiscSrvc(const aci_att_read_by_group_type_resp_event_rp0 *p)
{
	BtDevice_t *pDev = BtPeerFindByHdl(p->Connection_Handle);
	if (pDev == nullptr || pDev->pServices == nullptr)
	{
		// Unknown link, or discovery not started by BtAppDiscoverDevice
		return;
	}

	uint8_t entryLen = p->Attribute_Data_Length;
	uint8_t total    = p->Data_Length;
	if (entryLen < 6)
	{
		return;
	}
	uint8_t uuidLen = (uint8_t)(entryLen - 4);

	for (uint8_t off = 0; (uint16_t)(off + entryLen) <= total; off += entryLen)
	{
		if (pDev->NbSrvc >= BT_DEV_SERVICE_MAXCNT)
		{
			break;
		}
		const uint8_t *e = &p->Attribute_Data_List[off];
		BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->NbSrvc];
		pSrvc->srv_uuid.BaseIdx = 0;
		pSrvc->srv_uuid.Type    = BT_UUID_TYPE_16;
		pSrvc->srv_uuid.Uuid    = WbaUuid16(&e[4], uuidLen);
		pSrvc->handle_range.StartHdl = (uint16_t)(e[0] | (e[1] << 8));
		pSrvc->handle_range.EndHdl   = (uint16_t)(e[2] | (e[3] << 8));
		pSrvc->char_count = 0;
		pDev->NbSrvc++;
	}
}

// Parse one ACI_ATT_READ_BY_TYPE_RESP (characteristic discovery). Each entry is
// DeclHandle(2) Properties(1) ValueHandle(2) UUID(2 or 16); entry length is
// Handle_Value_Pair_Length (7 for 16-bit, 21 for 128-bit).
static void WbaDiscChar(const aci_att_read_by_type_resp_event_rp0 *p)
{
	BtDevice_t *pDev = BtPeerFindByHdl(p->Connection_Handle);
	if (pDev == nullptr || pDev->Discovery.SrvIdx >= pDev->NbSrvc)
	{
		return;
	}
	BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];

	uint8_t entryLen = p->Handle_Value_Pair_Length;
	uint8_t total    = p->Data_Length;
	if (entryLen < 7)
	{
		return;
	}
	uint8_t uuidLen = (uint8_t)(entryLen - 5);

	for (uint8_t off = 0; (uint16_t)(off + entryLen) <= total; off += entryLen)
	{
		if (pSrvc->char_count >= BT_GATT_DB_MAX_CHARS)
		{
			break;
		}
		const uint8_t *e = &p->Handle_Value_Pair_Data[off];
		BtGattDBChar_t *pCh = &pSrvc->characteristics[pSrvc->char_count];
		uint8_t props = e[2];

		pCh->characteristic.uuid.BaseIdx = 0;
		pCh->characteristic.uuid.Type    = BT_UUID_TYPE_16;
		pCh->characteristic.uuid.Uuid    = WbaUuid16(&e[5], uuidLen);

		pCh->characteristic.char_props.broadcast      = (props >> 0) & 1;
		pCh->characteristic.char_props.read           = (props >> 1) & 1;
		pCh->characteristic.char_props.write_wo_resp  = (props >> 2) & 1;
		pCh->characteristic.char_props.write          = (props >> 3) & 1;
		pCh->characteristic.char_props.notify         = (props >> 4) & 1;
		pCh->characteristic.char_props.indicate       = (props >> 5) & 1;
		pCh->characteristic.char_props.auth_signed_wr = (props >> 6) & 1;
		pCh->characteristic.char_ext_props            = (props >> 7) & 1;

		pCh->characteristic.handle_decl  = (uint16_t)(e[0] | (e[1] << 8));
		pCh->characteristic.handle_value = (uint16_t)(e[3] | (e[4] << 8));

		pCh->cccd_handle       = BT_ATT_HANDLE_INVALID;
		pCh->ext_prop_handle   = BT_ATT_HANDLE_INVALID;
		pCh->user_desc_handle  = BT_ATT_HANDLE_INVALID;
		pCh->report_ref_handle = BT_ATT_HANDLE_INVALID;

		pSrvc->char_count++;
	}
}

// Parse one ACI_ATT_FIND_INFO_RESP (descriptor discovery). Format 0x01 is
// Handle(2) UUID16(2) pairs (4 bytes); 0x02 is Handle(2) UUID128(16) (18 bytes).
// Records the CCCD and the other standard descriptor handles on the current
// characteristic.
static void WbaDiscDesc(const aci_att_find_info_resp_event_rp0 *p)
{
	BtDevice_t *pDev = BtPeerFindByHdl(p->Connection_Handle);
	if (pDev == nullptr || pDev->Discovery.SrvIdx >= pDev->NbSrvc)
	{
		return;
	}
	BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];
	if (pDev->Discovery.CharIdx >= pSrvc->char_count)
	{
		return;
	}
	BtGattDBChar_t *pCh = &pSrvc->characteristics[pDev->Discovery.CharIdx];

	uint8_t pairLen = (p->Format == 0x02) ? 18 : 4;
	uint8_t total   = p->Event_Data_Length;

	for (uint8_t off = 0; (uint16_t)(off + pairLen) <= total; off += pairLen)
	{
		const uint8_t *e = &p->Handle_UUID_Pair[off];
		uint16_t hdl  = (uint16_t)(e[0] | (e[1] << 8));
		uint16_t uuid = WbaUuid16(&e[2], (uint8_t)(pairLen - 2));

		switch (uuid)
		{
			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				pCh->cccd_handle = hdl;
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				pCh->ext_prop_handle = hdl;
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				pCh->user_desc_handle = hdl;
				break;
			case BT_UUID_DESCRIPTOR_REPORT_REFERENCE:
				pCh->report_ref_handle = hdl;
				break;
			default:
				break;
		}
	}
}

// Issue characteristic discovery for the current service, skipping empty
// services. When every service has been covered, move on to descriptors.
static void BtAppDiscStartChar(BtDevice_t *pDev)
{
	while (pDev->Discovery.SrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];
		uint16_t start = (uint16_t)(pSrvc->handle_range.StartHdl + 1); // skip decl
		uint16_t end   = pSrvc->handle_range.EndHdl;

		if (start <= end)
		{
			pDev->Discovery.Phase = DISC_CHARS;
			if (aci_gatt_disc_all_char_of_service(pDev->Conn.Hdl, start, end) ==
				BLE_STATUS_SUCCESS)
			{
				return;
			}
		}
		pDev->Discovery.SrvIdx++;
	}

	pDev->Discovery.SrvIdx  = 0;
	pDev->Discovery.CharIdx = 0;
	BtAppDiscStartDesc(pDev);
}

// Issue descriptor discovery for the current characteristic, skipping ones with
// no descriptor range. When every characteristic has been covered, discovery is
// complete.
static void BtAppDiscStartDesc(BtDevice_t *pDev)
{
	while (pDev->Discovery.SrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->pServices[pDev->Discovery.SrvIdx];

		while (pDev->Discovery.CharIdx < pSrvc->char_count)
		{
			BtGattDBChar_t *pCh    = &pSrvc->characteristics[pDev->Discovery.CharIdx];
			uint16_t        valHdl = pCh->characteristic.handle_value;
			uint16_t        start  = (uint16_t)(valHdl + 1);
			uint16_t        end;

			if ((uint8_t)(pDev->Discovery.CharIdx + 1) < pSrvc->char_count)
			{
				end = (uint16_t)(pSrvc->characteristics[pDev->Discovery.CharIdx + 1]
									 .characteristic.handle_decl - 1);
			}
			else
			{
				end = pSrvc->handle_range.EndHdl;
			}

			if (valHdl != BT_ATT_HANDLE_INVALID && start <= end)
			{
				pDev->Discovery.Phase = DISC_DESCS;
				// ST takes the characteristic value handle and walks the Find
				// Information range internally up to End_Handle.
				if (aci_gatt_disc_all_char_desc(pDev->Conn.Hdl, valHdl, end) ==
					BLE_STATUS_SUCCESS)
				{
					return;
				}
			}
			pDev->Discovery.CharIdx++;
		}
		pDev->Discovery.SrvIdx++;
		pDev->Discovery.CharIdx = 0;
	}

	pDev->Discovery.Phase = DISC_IDLE;
	BtDeviceDiscovered(pDev);
}

// ACI_GATT_PROC_COMPLETE advances the discovery state machine to the next phase
// or the next entry in the current phase.
static void WbaDiscProcComplete(const aci_gatt_proc_complete_event_rp0 *p)
{
	BtDevice_t *pDev = BtPeerFindByHdl(p->Connection_Handle);
	if (pDev == nullptr)
	{
		return;
	}

	switch (pDev->Discovery.Phase)
	{
		case DISC_SERVICES:
			pDev->Discovery.SrvIdx = 0;
			BtAppDiscStartChar(pDev);
			break;

		case DISC_CHARS:
			pDev->Discovery.SrvIdx++;
			BtAppDiscStartChar(pDev);
			break;

		case DISC_DESCS:
			pDev->Discovery.CharIdx++;
			BtAppDiscStartDesc(pDev);
			break;

		default:
			break;
	}
}

bool BtAppDiscoverDevice(BtDevice_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}

	pDev->Discovery.SrvIdx  = 0;
	pDev->Discovery.CharIdx = 0;
	pDev->Discovery.Phase   = DISC_SERVICES;

	pDev->NbSrvc = 0;
	if (BtDeviceSrvcCacheAttach(pDev) == false)
	{
		// No free discovery cache, see g_BtDevSrvcCacheCfg
		return false;
	}
	memset(pDev->pServices, 0, sizeof(BtGattDBSrvc_t) * BT_DEV_SERVICE_MAXCNT);

	return aci_gatt_disc_all_primary_services(pDev->Conn.Hdl) == BLE_STATUS_SUCCESS;
}

bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle,
                uint8_t *pData, uint16_t DataLen)
{
	if (ConnHandle == BT_CONN_HDL_INVALID || pData == NULL)
	{
		return false;
	}

	uint8_t ret = aci_gatt_write_without_resp(ConnHandle, CharHandle,
	                                          (uint8_t)DataLen, pData);
	return ret == BLE_STATUS_SUCCESS;
}

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CccdHandle)
{
	if (ConnHandle == BT_CONN_HDL_INVALID)
	{
		return false;
	}

	// Write 0x0001 to the CCCD to enable notifications.
	uint8_t cccd[2] = { 0x01, 0x00 };
	// A CCCD is written with a Write Request, the server answers.
	uint8_t ret = aci_gatt_write_char_value(ConnHandle, CccdHandle,
	                                        sizeof(cccd), cccd);
	return ret == BLE_STATUS_SUCCESS;
}
