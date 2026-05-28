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
            - BtAppRun:        start adv (if peripheral) + main loop.
            - BtAppEvtWait:    weak default uses WFE. RTOS apps override.
            - BtAppEvtDispatch: drains HCI/ACI events to user handlers.
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
#include <stdlib.h>
#include <string.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal.h"

// STM32CubeWBA BLE middleware. Names are stable across 1.3.x / 1.4.x.
#include "ble_types.h"
#include "ble_std.h"
#include "ble.h"
#include "ble_gap_aci.h"
#include "ble_gatt_aci.h"
#include "ble_hal_aci.h"
#include "ble_events.h"
#include "host_stack_if.h"
#include "ll_sys.h"
#include "ll_sys_if.h"
#include "bpka.h"
#include "scm.h"
#include "stm32_seq.h"

#include "istddef.h"
#include "idelay.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_dev.h"
#include "app_evt_handler.h"

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

#ifndef BT_STM32WBA_CONN_HDL_INVALID
#define BT_STM32WBA_CONN_HDL_INVALID	0xFFFF
#endif

// ATT MTU. Default per BT spec is 23. The stack negotiates higher via
// aci_gatt_exchange_config. Apps that set BtAppCfg_t::MaxMtu request a
// higher value; the cap depends on the stack configuration buffer sizes.
#ifndef GATT_MTU_SIZE_DEFAULT
#define GATT_MTU_SIZE_DEFAULT			23
#endif

// Stack-event scheduler task id. STM32CubeWBA convention is to use the
// sequencer (UTIL_SEQ) to fan HCI/ACI events out from the interrupt
// context to the main thread. The id is local to this port.
#define BT_APP_TASK_HCI_USER_EVT		0
#define BT_APP_TASK_BLE_HOST			1

// Connection-parameter update timing knobs - same intent as the BM port.
#define BT_APP_CONN_PARAMS_FIRST_DELAY_MS	5000
#define BT_APP_CONN_PARAMS_NEXT_DELAY_MS	30000
#define BT_APP_CONN_PARAMS_MAX_ATTEMPTS		3

// SMP key-size limits per BT spec.
#define SEC_PARAM_MIN_KEY_SIZE			7
#define SEC_PARAM_MAX_KEY_SIZE			16

// Mapping from BtGap SecType to ST's IO-capability + auth-requirement
// values is done at init time. Default to NoInputNoOutput / no MITM.
#ifndef BT_APP_DEFAULT_IO_CAPABILITY
#define BT_APP_DEFAULT_IO_CAPABILITY	IO_CAP_NO_INPUT_NO_OUTPUT
#endif

// Forward decls from peer port files (defined in 9b..9e).
extern "C" bool BtAppAdvInit(const BtAppCfg_t *pCfg);
extern "C" bool BtDisInit(const struct __Bt_App_Cfg *pCfg);

// User hooks - weak symbols overridden by the app.
__attribute__((weak)) void BtAppInitUserServices(void) {}
__attribute__((weak)) void BtAppInitUserData(void) {}

// --- Port-private data ---
//
// Sized smaller than the BM equivalent because ST handles adv handle
// allocation internally - we don't track a separate AdvHdl on this port.
typedef struct __Bt_App_WbaData {
	uint16_t	GattSrvcStartHdl;	//!< First handle of the user GATT service
	uint8_t		IoCapability;		//!< SMP IO cap - mapped from SecType at init
	uint8_t		AuthRequirement;	//!< SMP auth requirement bits
	bool		bStackInited;		//!< true after BleStack_Init succeeded
} BtAppWbaData_t;

static BtAppWbaData_t s_WbaData = {
	.GattSrvcStartHdl = 0,
	.IoCapability     = BT_APP_DEFAULT_IO_CAPABILITY,
	.AuthRequirement  = 0,
	.bStackInited     = false,
};

// HCI user-event indication. ST's BLE stack calls this from IRQ context
// when an event is queued; we just schedule the user-evt processing task.
extern "C" void hci_notify_asynch_evt(void *pdata)
{
	(void)pdata;
	UTIL_SEQ_SetTask(1U << BT_APP_TASK_HCI_USER_EVT, CFG_SCH_PRIO_0);
	BtAppEvtNotify();
}

// BLE host scheduler hook - the stack calls this when the host needs CPU.
extern "C" void BLE_RESUME_FLOW_PROCESS_Callback(void)
{
	UTIL_SEQ_SetTask(1U << BT_APP_TASK_BLE_HOST, CFG_SCH_PRIO_0);
	BtAppEvtNotify();
}

// Task body: drain queued HCI events to registered callbacks. ST exposes
// hci_user_evt_proc() which walks the event queue and invokes the user
// callback per event.
static void HciUserEvtProcessTask(void)
{
	hci_user_evt_proc();
}

// Task body: let the BLE host stack process its pending work. Drives the
// internal state machines that issue ACI commands and forward events.
static void BleHostTask(void)
{
	BleStack_Process();
}

// HCI event callback registered with the stack. ST's stack invokes this
// for every HCI / vendor event; we route by type to the appropriate
// handler. Adv-report events go to the scan path (handled in
// bt_scan_stm32wba.cpp via a separately registered observer).
static SVCCTL_UserEvtFlowStatus_t BtAppHciEvtHandler(void *pPayload)
{
	hci_event_pckt *pEvtPkt = (hci_event_pckt *)((hci_uart_pckt *)pPayload)->data;

	switch (pEvtPkt->evt)
	{
		case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
		{
			hci_disconnection_complete_event_rp0 *p =
				(hci_disconnection_complete_event_rp0 *)pEvtPkt->data;
			if (p->Connection_Handle == g_BtAppData.ConnHdl)
			{
				g_BtAppData.ConnHdl = BT_STM32WBA_CONN_HDL_INVALID;
				g_BtAppData.State   = BTAPP_DISCONNECTED;
				BtAppConnLedOff();
				BtGapDeleteConnection(p->Connection_Handle);
				// Re-arm advertising for peripheral/broadcaster apps.
				if (g_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
				{
					BtAdvStart();
				}
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
					if (p->Status == BLE_STATUS_SUCCESS)
					{
						g_BtAppData.ConnHdl = p->Connection_Handle;
						g_BtAppData.State   = BTAPP_CONNECTED;
						BtAppConnLedOn();
						BtGapAddConnection(p->Connection_Handle,
						                   p->Role,
						                   p->Peer_Address_Type,
						                   p->Peer_Address);
					}
					break;
				}

				case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
					// Adv reports handled by the scan path.
					break;

				case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
					// Conn-param updates handled by bt_cp_stm32wba.cpp.
					break;

				default:
					break;
			}
			break;
		}

		default:
			break;
	}

	return SVCCTL_UserEvtFlowEnable;
}

// --- Public API ---

bool BtAppStackInit(const BtAppCfg_t *pCfg)
{
	if (s_WbaData.bStackInited)
	{
		return true;
	}

	// System Clock Manager - registers the BLE radio domain so SCM picks
	// the right clocks when the radio is active.
	scm_init();

	// Link Layer hardware setup. ll_sys_thread_init runs the radio task,
	// ll_sys_dependencies_init wires RNG/AES/PKA into the link layer.
	ll_sys_thread_init();
	ll_sys_dependencies_init();

	// Public-key accelerator for LESC pairing. Safe to init even if SMP
	// isn't enabled; the stack just won't use it.
	BPKA_Reset();

	// Host stack bring-up. CFG_BLE_NUM_LINK / CFG_BLE_NUM_GATT_SERVICES /
	// CFG_BLE_ATT_MTU_MAX etc. come from the app's app_conf.h - the port
	// does not override them. RAM start address is set by the linker.
	uint8_t ret = BleStack_Init();
	if (ret != BLE_STATUS_SUCCESS)
	{
		DEBUG_PRINTF("BleStack_Init failed: 0x%02x\r\n", ret);
		return false;
	}

	// Register the central HCI event handler with the SVCCTL dispatcher.
	SVCCTL_RegisterHandler(BtAppHciEvtHandler);

	// Sequencer tasks for HCI-evt drain + BLE host work. These match the
	// notification hooks above (hci_notify_asynch_evt, BLE_RESUME_FLOW).
	UTIL_SEQ_RegTask(1U << BT_APP_TASK_HCI_USER_EVT, UTIL_SEQ_RFU,
	                 HciUserEvtProcessTask);
	UTIL_SEQ_RegTask(1U << BT_APP_TASK_BLE_HOST,     UTIL_SEQ_RFU,
	                 BleHostTask);

	// Set TX power if requested. ACI value is -128..+127 dBm (signed
	// int8), but the stack rounds to the closest supported level.
	if (pCfg->TxPower != 0)
	{
		aci_hal_set_tx_power_level(0, (uint8_t)pCfg->TxPower);
	}

	s_WbaData.bStackInited = true;
	return true;
}

bool BtAppInit(const BtAppCfg_t *pCfg)
{
	if (pCfg == NULL)
	{
		return false;
	}

	// Initialize application event handler queue.
	if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem,
	                      pCfg->EvtHandlerQueMemSize) == false)
	{
		return false;
	}

	// Initialize peer manager.
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}

	if (!BtGapConnPoolInit(pCfg->pGapConnPoolMem, pCfg->GapConnPoolMemSize))
	{
		return false;
	}

	// Populate generic app data from cfg.
	g_BtAppData.Role            = pCfg->Role;
	g_BtAppData.AdvHdl          = 0;	// WBA stack manages adv internally
	g_BtAppData.ConnHdl         = BT_STM32WBA_CONN_HDL_INVALID;
	g_BtAppData.bExtAdv         = pCfg->bExtAdv;
	g_BtAppData.ConnLedPort     = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin      = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;
	g_BtAppData.bScan           = false;
	g_BtAppData.VendorId        = pCfg->VendorId;
	g_BtAppData.ProductId       = pCfg->ProductId;
	g_BtAppData.ProductVer      = pCfg->ProductVer;
	g_BtAppData.Appearance      = pCfg->Appearance;

	g_BtAppData.MaxMtu = GATT_MTU_SIZE_DEFAULT;
	if (pCfg->MaxMtu > GATT_MTU_SIZE_DEFAULT)
	{
		g_BtAppData.MaxMtu = pCfg->MaxMtu;
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

	ret = aci_gap_init(role,
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
		aci_gatt_update_char_value(gapSrvcHdl, devNameHdl,
		                           0, (uint8_t)strlen(pCfg->pDevName),
		                           (uint8_t *)pCfg->pDevName);
	}

	// Appearance.
	uint16_t appearance = pCfg->Appearance;
	aci_gatt_update_char_value(gapSrvcHdl, appearanceHdl,
	                           0, sizeof(appearance), (uint8_t *)&appearance);

	// SMP / pairing config. Map BtGap SecType to ST auth requirement.
	uint8_t authReq      = 0;	// bonding NO, MITM NO by default
	uint8_t scSupport    = 0;	// LE Secure Connections off by default
	switch (pCfg->SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			authReq = 0x05;	// bonding + MITM
			break;
		case BTGAP_SECTYPE_LESC_MITM:
			authReq   = 0x05;
			scSupport = 1;
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

	aci_gap_set_io_capability(s_WbaData.IoCapability);

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
	BtGapInit(&gapcfg);

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

	g_BtAppData.bSecure = (pCfg->SecType != BTGAP_SECTYPE_NONE);

	// Advertising init - actual ACI calls live in bt_adv_stm32wba.cpp.
	if (g_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		if (BtAppAdvInit(pCfg) == false)
		{
			DEBUG_PRINTF("BtAppAdvInit failed\r\n");
			return false;
		}
	}

	g_BtAppData.State = BTAPP_STATE_INITIALIZED;
	DEBUG_PRINTF("BtAppInit: success\r\n");
	return true;
}

void BtAppRun(void)
{
	if (g_BtAppData.State != BTAPP_STATE_INITIALIZED)
	{
		return;
	}

	if (g_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAdvStart();
	}

	DEBUG_PRINTF("BtAppRun: entering main loop\r\n");

	while (1)
	{
		// Run any user-queued events first - matches BM/nRF52 ordering.
		AppEvtHandlerExec();

		// Pump the sequencer. UTIL_SEQ_Run dispatches whichever task the
		// notification hooks scheduled (HCI evt drain, BLE host work).
		UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);

		BtAppEvtWait();
	}
}

// Generic-layer BtAppEvtDispatch override - drains queued stack work
// without waiting. Called by RTOS bridge code after BtAppEvtNotify.
__attribute__((weak)) void BtAppEvtDispatch(void)
{
	UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);
}

// Bare-metal default: wait for any event. RTOS apps override with a
// semaphore-take in the bridge code (same pattern as BM/nRF52 ports).
__attribute__((weak)) void BtAppEvtWait(void)
{
	__WFE();
}

bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (pChar == NULL || pData == NULL || DataLen == 0)
	{
		return false;
	}

	if (g_BtAppData.ConnHdl == BT_STM32WBA_CONN_HDL_INVALID)
	{
		return false;
	}

	// aci_gatt_update_char_value pushes the value to the attribute table;
	// if the CCCD has notifications enabled the stack sends the notify
	// PDU automatically.
	uint8_t ret = aci_gatt_update_char_value(pChar->Hdl, pChar->ValHdl,
	                                         0, (uint8_t)DataLen, pData);
	return ret == BLE_STATUS_SUCCESS;
}

void BtAppDisconnect(void)
{
	if (g_BtAppData.ConnHdl != BT_STM32WBA_CONN_HDL_INVALID)
	{
		aci_gap_terminate(g_BtAppData.ConnHdl, 0x13);	// remote user term
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
	aci_gatt_update_char_value(s_WbaData.GattSrvcStartHdl, devNameHdl,
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

bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle,
                uint8_t *pData, uint16_t DataLen)
{
	if (ConnHandle == BT_STM32WBA_CONN_HDL_INVALID || pData == NULL)
	{
		return false;
	}

	uint8_t ret = aci_gatt_write_without_resp(ConnHandle, CharHandle,
	                                          (uint8_t)DataLen, pData);
	return ret == BLE_STATUS_SUCCESS;
}

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CccdHandle)
{
	if (ConnHandle == BT_STM32WBA_CONN_HDL_INVALID)
	{
		return false;
	}

	// Write 0x0001 to the CCCD to enable notifications.
	uint8_t cccd[2] = { 0x01, 0x00 };
	uint8_t ret = aci_gatt_write_without_resp(ConnHandle, CccdHandle,
	                                          sizeof(cccd), cccd);
	return ret == BLE_STATUS_SUCCESS;
}
