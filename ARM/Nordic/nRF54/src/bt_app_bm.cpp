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
#include "bm/bluetooth/peer_manager/peer_manager.h"
#include "bm/bluetooth/peer_manager/peer_manager_handler.h"
#include "bm/bluetooth/peer_manager/nrf_ble_lesc.h"
#include "bm/bluetooth/services/ble_dis.h"

#include "crypto/crypto.h"

// Injection point for the LESC crypto engine (defined in the IOsonata
// nrf_ble_lesc replacement). The App owns the CryptoDev_t and passes it in.
extern "C" void BtLescSetCryptoEngine(CryptoDev_t *pDev);
#include "nrfx_cracen.h"
#include "nrf_soc.h"

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
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_dev.h"
#include "app_evt_handler.h"

extern "C" bool sdh_state_evt_observer_notify(enum nrf_sdh_state_evt state);

/******** For DEBUG ************/
#define BM_DEBUG_ENABLE

#ifdef BM_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
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

// g_BtAppData definition and helpers (isConnected, BtConnected, BtInitialized,
// BtAppGetConnHandle, BtAppConnLedOff/On) moved to src/bluetooth/bt_app.cpp.

// --- Advertisement packet buffers ---


// --- Scan buffer for central / observer mode ---
// S145 on nRF54L15 caps extended-adv data at BLE_GAP_SCAN_BUFFER_EXTENDED_MAX_SUPPORTED (255).
// Same buffer covers legacy 31-byte scans too.


const static TimerCfg_t s_BtAppSdTimerCfg = {
    .DevNo = 3,	// GRTC3 needed for Softdevice
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,		// App-level priority. 0, 1 and 4 are reserved by the SoftDevice;
						// priority 0 here would fault the interrupt configuration check
						// once the wakeup trigger below enables the GRTC IRQ.
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static Timer s_BtAppSdGrtc3;

// --- Helper functions ---

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

// --- Conn params event handler ---

static void on_conn_params_evt(const struct ble_conn_params_evt *p_evt)
{
	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_ERROR)
	{
		sd_ble_gap_disconnect(BtAppGetConnHandle(),
							  BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
	}
}

// Central GATT client discovery (S145). Single active discovery at a time.
// Bodies live in the Central / Observer section near BtAppConnect.
static void BtAppDiscPrimSrvcRsp(const ble_gattc_evt_t *pEvt);
static void BtAppDiscCharRsp(const ble_gattc_evt_t *pEvt);
static void BtAppDiscDescRsp(const ble_gattc_evt_t *pEvt);

// ===========================================================================
// SMP user interaction bridge (sdk-nrf-bm / Peer Manager port, S145).
//
// The SoftDevice owns the SMP exchange and surfaces the user steps as GAP
// events: BLE_GAP_EVT_PASSKEY_DISPLAY (Numeric Comparison when match_request is
// set, otherwise Passkey Entry display) and BLE_GAP_EVT_AUTH_KEY_REQUEST
// (Passkey Entry input). These are bridged to the same BtSmp* interaction API
// the generic SMP host exposes, so an application sees one set of callbacks
// across ports. The reply functions route the user answer back through
// sd_ble_gap_auth_key_reply. The generic weak defaults live in bt_smp.cpp,
// which is not linked on this port, so equivalent weak defaults are provided
// here; an application strong definition overrides them.
// ===========================================================================

// SoftDevice passkey octets are six ASCII digits, most significant first.
// Convert to the six digit integer used by the BtSmp interaction API.
static uint32_t BmPasskeyToVal(const uint8_t *pAscii)
{
	uint32_t v = 0;
	for (int i = 0; i < BLE_GAP_PASSKEY_LEN; i++)
	{
		v = v * 10 + (uint32_t)(pAscii[i] - '0');
	}
	return v;
}

// Convert the six digit integer to six ASCII digits, most significant first,
// zero padded, for sd_ble_gap_auth_key_reply.
static void BmPasskeyFromVal(uint32_t Val, uint8_t *pAscii)
{
	for (int i = BLE_GAP_PASSKEY_LEN - 1; i >= 0; i--)
	{
		pAscii[i] = (uint8_t)('0' + (Val % 10));
		Val /= 10;
	}
}

// Resume Numeric Comparison. Confirm true reports a match (reply PASSKEY type
// with no key), false reports no match (reply NONE) and aborts pairing.
void BtSmpNumericComparisonReply(uint16_t ConnHdl, bool Confirm)
{
	(void)sd_ble_gap_auth_key_reply(ConnHdl,
			Confirm ? BLE_GAP_AUTH_KEY_TYPE_PASSKEY : BLE_GAP_AUTH_KEY_TYPE_NONE,
			NULL);
}

// Resume Passkey Entry on the input side. A value in 0..999999 is sent as six
// ASCII digits; a value above that range cancels with a NONE reply.
void BtSmpPasskeyReply(uint16_t ConnHdl, uint32_t Passkey)
{
	if (Passkey > 999999u)
	{
		(void)sd_ble_gap_auth_key_reply(ConnHdl, BLE_GAP_AUTH_KEY_TYPE_NONE, NULL);
		return;
	}

	uint8_t ascii[BLE_GAP_PASSKEY_LEN];
	BmPasskeyFromVal(Passkey, ascii);
	(void)sd_ble_gap_auth_key_reply(ConnHdl, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, ascii);
}

// Weak defaults. With no application override the only safe action is to reject,
// so the user interaction cannot be performed silently. An application that can
// display or input overrides these.
__attribute__((weak)) void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value)
{
	(void)Value;
	BtSmpNumericComparisonReply(ConnHdl, false);
}

__attribute__((weak)) void BtSmpPasskeyDisplay(uint16_t ConnHdl, uint32_t Passkey)
{
	(void)ConnHdl;
	(void)Passkey;
}

__attribute__((weak)) void BtSmpPasskeyRequest(uint16_t ConnHdl)
{
	BtSmpPasskeyReply(ConnHdl, BT_SMP_PASSKEY_INVALID);
}

// --- BLE event dispatch (registered as observer) ---

static void ble_evt_dispatch(const ble_evt_t *p_ble_evt, void *p_context)
{
	uint32_t err_code;
	const ble_gap_evt_t *p_gap_evt = &p_ble_evt->evt.gap_evt;
	//uint8_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);
	uint8_t role = g_BtAppData.AppDevice.Conn.Role;

	DEBUG_PRINTF("evt: 0x%x\r\n", p_ble_evt->header.evt_id);
	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			BtAppConnLedOn();
			BtPeerConnected(p_gap_evt->conn_handle, role,
							   p_gap_evt->params.connected.peer_addr.addr_type,
							   (uint8_t *)p_gap_evt->params.connected.peer_addr.addr);
			g_BtAppData.State = BTAPP_STATE_CONNECTED;
			BtAppEvtConnected(p_ble_evt->evt.gap_evt.conn_handle);

			// Initiate link security if the app is configured secure. For an
			// already-bonded peer this encrypts with the stored LTK; otherwise
			// it starts pairing. peer_manager drives the procedure.
			if (g_BtAppData.AppDevice.bSecure)
			{
				(void)pm_conn_secure(p_gap_evt->conn_handle, false);
			}
			break;

		case BLE_GAP_EVT_DISCONNECTED:
		{
			uint16_t connHdl = p_ble_evt->evt.gap_evt.conn_handle;
			BtDevice_t *pPeer = BtPeerFindByHdl(connHdl);

			BtAppConnLedOff();
			BtPeerFree(pPeer);

			if (BtPeerIsConnected() == false)
			{
				g_BtAppData.State = BTAPP_STATE_IDLE;
			}

			BtAppEvtDisconnected(connHdl);

			if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
			{
				BtAdvStart();
			}
		}
			break;

		case BLE_GAP_EVT_CONN_SEC_UPDATE:
		{
			const ble_gap_conn_sec_t *pcs = &p_gap_evt->params.conn_sec_update.conn_sec;
			// Capture the negotiated encryption key size for BtGapConnSecGet.
			// pm_conn_sec_status_get does not report key size; this event does.
			BtDevice_t *pdev = BtPeerFindByHdl(p_gap_evt->conn_handle);
			if (pdev != nullptr)
			{
				pdev->Conn.Sec.KeySize = pcs->encr_key_size;
			}
		}
			break;

		case BLE_GAP_EVT_PASSKEY_DISPLAY:
		{
			const ble_gap_evt_passkey_display_t *pPd =
					&p_gap_evt->params.passkey_display;
			uint16_t connHdl = p_gap_evt->conn_handle;
			uint32_t val = BmPasskeyToVal(pPd->passkey);
			if (pPd->match_request)
			{
				// LESC Numeric Comparison: the application confirms the match
				// through BtSmpNumericComparisonReply.
				BtSmpNumericComparison(connHdl, val);
			}
			else
			{
				// Passkey Entry display side: show the value the peer enters.
				BtSmpPasskeyDisplay(connHdl, val);
			}
		}
			break;

		case BLE_GAP_EVT_AUTH_KEY_REQUEST:
			if (p_gap_evt->params.auth_key_request.key_type ==
				BLE_GAP_AUTH_KEY_TYPE_PASSKEY)
			{
				// Passkey Entry input side: the application provides the value
				// through BtSmpPasskeyReply.
				BtSmpPasskeyRequest(p_gap_evt->conn_handle);
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
				BtAppGetConnHandle(), NULL, 0, 0);
			(void)err_code;
			break;

		case BLE_EVT_USER_MEM_REQUEST:
			{
				// Long write: hand the SoftDevice this link's reassembly buffer.
				// Replied once per connection (not per service) to avoid
				// multiple sd_ble_user_mem_reply calls on the same request.
				BtDevice_t *pConn = BtPeerFindByHdl(p_ble_evt->evt.common_evt.conn_handle);
				if (pConn != nullptr && pConn->Conn.pLongWrBuff != nullptr)
				{
					ble_user_mem_block_t mblk;
					memset(&mblk, 0, sizeof(mblk));
					mblk.p_mem = pConn->Conn.pLongWrBuff;
					mblk.len   = pConn->Conn.LongWrBuffSize;
					memset(pConn->Conn.pLongWrBuff, 0, pConn->Conn.LongWrBuffSize);
					err_code = sd_ble_user_mem_reply(p_ble_evt->evt.common_evt.conn_handle, &mblk);
					(void)err_code;
				}
			}
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
			uint16_t mtu = g_BtAppData.AppDevice.Conn.MaxMtu;
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
		(g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER)))
	{
		switch (p_ble_evt->header.evt_id)
		{
			case BLE_GAP_EVT_ADV_REPORT:
			{
				const ble_gap_evt_adv_report_t *p_adv_report =
					&p_gap_evt->params.adv_report;
				bool keep_going = BtAppScanReport(p_adv_report->rssi,
								p_adv_report->peer_addr.addr_type,
								(uint8_t *)p_adv_report->peer_addr.addr,
								p_adv_report->data.len,
								p_adv_report->data.p_data);
				// S145 stops reporting after each adv packet; the app must
				// re-arm the scan or explicitly stop it.
				if (keep_going)
				{
					BtAppScan();
				}
				else
				{
					BtAppScanStop();
				}
			}
			break;

			case BLE_GAP_EVT_TIMEOUT:
			{
				const ble_gap_evt_timeout_t *p_timeout =
					&p_gap_evt->params.timeout;
				if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
				{
					g_BtAppData.bScan = false;
				}
			}
			break;

			case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
				BtAppDiscPrimSrvcRsp(&p_ble_evt->evt.gattc_evt);
				break;

			case BLE_GATTC_EVT_CHAR_DISC_RSP:
				BtAppDiscCharRsp(&p_ble_evt->evt.gattc_evt);
				break;

			case BLE_GATTC_EVT_DESC_DISC_RSP:
				BtAppDiscDescRsp(&p_ble_evt->evt.gattc_evt);
				break;

			case BLE_GATTC_EVT_HVX:
				{
					const ble_gattc_evt_hvx_t *p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
					BtGattClientNotified(p_ble_evt->evt.gattc_evt.conn_handle,
					                     p_hvx->handle, (uint8_t *)p_hvx->data, p_hvx->len);
					// An indication must be confirmed by the client.
					if (p_hvx->type == BLE_GATT_HVX_INDICATION)
					{
						sd_ble_gattc_hv_confirm(p_ble_evt->evt.gattc_evt.conn_handle,
						                        p_hvx->handle);
					}
				}
				break;

			default:
				break;
		}
		BtAppCentralEvtHandler((uint32_t)p_ble_evt->header.evt_id, (void *)p_ble_evt);
	}

	// Forward to peripheral handlers
	if (g_BtAppData.AppDevice.Conn.Role & BTAPP_ROLE_PERIPHERAL)
	{
		// HVN TX complete gives a count, not a handle. Drain the per-peer
		// send-order ring once per event.
		if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_HVN_TX_COMPLETE)
		{
			BtGattSendCompleted(p_ble_evt->evt.gatts_evt.conn_handle,
				p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
		}
		else if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_HVC)
		{
			// Client confirmed the outstanding indication.
			BtGattHandleValueConfirm(p_ble_evt->evt.gatts_evt.conn_handle);
		}
		BtGattEvtHandler((uint32_t)p_ble_evt->header.evt_id, (void *)p_ble_evt);
		BtAppPeriphEvtHandler((uint32_t)p_ble_evt->header.evt_id, (void *)p_ble_evt);
	}
}

// Note: SoC event polling is handled internally by nrf_sdh_soc.c
// which registers its own NRF_SDH_STACK_EVT_OBSERVER.

// --- Advertising ---


/**
 * @brief Overloadable advertising initialization
 */


// --- Notify ---

bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (DataLen > 0 && pData == nullptr)
	{
		return false;
	}

	if (DataLen > 0 && BtGattCharSetValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	// Delegate the send to BtGattCharNotify so the notification is tracked in
	// the TX-pending ring and TxCompleteCB fires on completion.
	return BtGattCharNotify(BtAppGetConnHandle(), pChar, pData, DataLen);
}

bool BtAppIndicate(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (DataLen > 0 && pData == nullptr)
	{
		return false;
	}

	if (DataLen > 0 && BtGattCharSetValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	// Delegate to BtGattCharIndicate so the indication is tracked: pending flag,
	// transaction timeout, and TX-pending ring.
	return BtGattCharIndicate(BtAppGetConnHandle(), pChar, pData, DataLen);
}

// --- Disconnect ---

void BtAppDisconnect()
{
	if (BtAppGetConnHandle() != BLE_CONN_HANDLE_INVALID)
	{
		uint32_t err_code = sd_ble_gap_disconnect(
			BtAppGetConnHandle(),
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
		{
			DEBUG_PRINTF("BtAppDisconnect failed: 0x%x\r\n", err_code);
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

	uint32_t ramstart = (uint32_t)SystemRamStart();

	memset((void *)0x20000000, 0, ramstart - 0x20000000);

	// GRTC3 must be enabled with interrupt disbled before calling nrf_sdh_enable_request
	s_BtAppSdGrtc3.Init(s_BtAppSdTimerCfg);

	// Enable SoftDevice
	err = nrf_sdh_enable_request();
	if (err && err != -EALREADY)
	{
		DEBUG_PRINTF("nrf_sdh_enable_request failed: %d\r\n", err);
		return false;
	}

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

	// 1 s continuous wakeup trigger so a fully silent link still reaches the
	// SMP/GATT transaction timeout checks in the main loop. Enabled only after
	// the SoftDevice is up: GRTC3 must be initialized with its interrupt
	// disabled before nrf_sdh_enable_request. No handler is needed, the GRTC
	// interrupt itself ends the WFE wait; the ISR tolerates a null handler.
	s_BtAppSdGrtc3.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS, nullptr);

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

// ---------------------------------------------------------------------------
// Secure Connections: peer_manager + nrf_ble_lesc, mirroring the nRF52
// SoftDevice port. The S145 SoftDevice owns the SMP state machine; peer_manager
// drives it and persists bonds (through the IOsonata bt_pds store), and
// nrf_ble_lesc performs the LESC ECDH. The application observes link security
// through these peer_manager events; no key material is surfaced to the app on
// this path (peer_manager is the authority), matching nRF52.
// ---------------------------------------------------------------------------

#define BT_APP_SEC_PARAM_MIN_KEY_SIZE	7
#define BT_APP_SEC_PARAM_MAX_KEY_SIZE	16

static void BtAppPmEvtHandler(const struct pm_evt *p_evt)
{
	// Standard peer_manager housekeeping: applies the event to internal state,
	// disconnects on security failure, and cleans flash on data update.
	pm_handler_on_pm_evt(p_evt);
	pm_handler_disconnect_on_sec_failure(p_evt);
	pm_handler_flash_clean(p_evt);

	switch (p_evt->evt_id)
	{
		case PM_EVT_CONN_SEC_SUCCEEDED:
			// Link is encrypted. peer_manager holds the bond. Notify the app so
			// it can run work that needs an encrypted link (e.g. a central
			// reading protected characteristics). (nRF52 optionally enforces
			// MITM here; left permissive.)
			BtAppEvtSecured(p_evt->conn_handle);
			break;

		case PM_EVT_CONN_SEC_FAILED:
			// Security setup failed. pm_handler_disconnect_on_sec_failure above
			// already tears the link down when required.
			break;

		case PM_EVT_CONN_SEC_CONFIG_REQ:
		{
			// Allow an already-bonded peer to re-pair.
			struct pm_conn_sec_config cfg = { .allow_repairing = true };
			pm_conn_sec_config_reply(p_evt->conn_handle, &cfg);
		}
			break;

		case PM_EVT_STORAGE_FULL:
			// The IOsonata bt_pds store compacts itself on write, so unlike the
			// nRF52 fds_gc() path there is nothing to trigger here.
			break;

		case PM_EVT_PEERS_DELETE_SUCCEEDED:
			// Bonds cleared. Resume advertising if we are a peripheral.
			if (g_BtAppData.AppDevice.Conn.Role &
				(BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
			{
				BtAdvStart();
			}
			break;

		default:
			break;
	}
}

// Initialize peer_manager and LESC. Maps the app SecType / key-exchange config
// onto ble_gap_sec_params_t exactly as the nRF52 port does (the BTAPP_SECTYPE_*
// values alias the same BT_GAP_SECTYPE_* the nRF52 BLEAPP_SECTYPE_* use).
static uint32_t BtAppPeerMngrInit(BTGAP_SECTYPE SecType, uint8_t SecKeyExchg, bool bEraseBond)
{
	ble_gap_sec_params_t sec_param;
	uint32_t err_code;

	// Provide the LESC layer its crypto engine BEFORE pm_init. With CONFIG_PM_LESC
	// defined, pm_init -> sm_init calls nrf_ble_lesc_init, which checks the engine
	// via CryptoIsCapable. The engine must already be injected at that point, else
	// nrf_ble_lesc_init fails and pm_init returns an error. The App owns the
	// CryptoDev_t and injects it, mirroring the BtSmpInit model on the SDC port.
	// ECDH goes through CryptoInit with CRYPTO_PROVIDER_AUTO: the hardware
	// engine (CryptoHwInit, PSA over CRACEN) is selected when it is linked,
	// otherwise it falls back to software uECC, with no change to the LESC
	// code. The arena is sized CRYPTO_MEMSIZE_ECDH, which fits whichever engine
	// AUTO selects. Random bytes come from the PSA DRBG on the hardware path,
	// or from the CRACEN-backed RngGet on the uECC path.
	static CryptoDev_t s_LescEcdh;
	static uint8_t     s_LescEcdhMem[CRYPTO_MEMSIZE_ECDH];	// ECDH per-instance key arena (fits HW or uECC)
	CryptoCfg_t lescCfg = { };
	lescCfg.Provider = CRYPTO_PROVIDER_AUTO;
	lescCfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
	lescCfg.pMem     = s_LescEcdhMem;
	lescCfg.MemSize  = sizeof(s_LescEcdhMem);
	CryptoInit(&s_LescEcdh, &lescCfg);
	BtLescSetCryptoEngine(&s_LescEcdh);

	err_code = pm_init();
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("pm_init failed: 0x%x\r\n", err_code);
		return err_code;
	}

	if (bEraseBond)
	{
		(void)pm_peers_delete();
	}

	memset(&sec_param, 0, sizeof(sec_param));

	sec_param.bond           = 1;
	sec_param.min_key_size   = BT_APP_SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size   = BT_APP_SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
	sec_param.kdist_own.id   = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id  = 1;

	switch (SecType)
	{
		case BTGAP_SECTYPE_NONE:
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
			break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
			sec_param.mitm = 1;
			break;
		case BTGAP_SECTYPE_LESC_MITM:
			sec_param.mitm = 1;
			sec_param.lesc = 1;
			break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			sec_param.lesc = 1;
			break;
		case BTGAP_SECTYPE_SIGNED_MITM:
			sec_param.mitm = 1;
			sec_param.lesc = 1;
			break;
		default:
			break;
	}

	int exchg = SecKeyExchg & (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO);
	switch (exchg)
	{
		case BTAPP_SECEXCHG_KEYBOARD:
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
			break;
		case BTAPP_SECEXCHG_DISPLAY:
			sec_param.io_caps  = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
			break;
		case (BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO):
			sec_param.io_caps  = BLE_GAP_IO_CAPS_DISPLAY_YESNO;
			break;
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY):
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO):
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
			break;
		default:
			break;
	}

	if (SecKeyExchg & BTAPP_SECEXCHG_OOB)
	{
		sec_param.oob = 1;
	}

	err_code = pm_sec_params_set(&sec_param);
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("pm_sec_params_set failed: 0x%x\r\n", err_code);
		return err_code;
	}

	err_code = pm_register(BtAppPmEvtHandler);
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("pm_register failed: 0x%x\r\n", err_code);
		return err_code;
	}

	// When CONFIG_PM_LESC is defined, pm_init -> sm_init already called
	// nrf_ble_lesc_init (the engine was injected before pm_init above). Only
	// call it explicitly when the SDK LESC path is not compiled in.
#if !defined(CONFIG_PM_LESC)
	err_code = nrf_ble_lesc_init();
	if (err_code != NRF_SUCCESS)
	{
		DEBUG_PRINTF("nrf_ble_lesc_init failed: 0x%x\r\n", err_code);
		return err_code;
	}
#endif

	return NRF_SUCCESS;
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
	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	g_BtAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}

	// Split the long-write reassembly pool across the peer slots so each
	// link gets its own buffer (per Conn.pLongWrBuff).
	BtPeerLongWrInit(pCfg->pLongWrPoolMem, pCfg->LongWrPoolMemSize);

	// Connection pool removed: the peer manager (BtPeerInit above) owns
	// the single connection table now.
	// bExtAdv is no longer a config field; BtAdvEncode determines extended vs
	// legacy from the encoded payload and sets g_BtAppData.bExtAdv during
	// BtAppAdvInit.
	g_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;
	g_BtAppData.bScan = false;
	g_BtAppData.AppDevice.VendorId = pCfg->VendorId;
	g_BtAppData.AppDevice.ProductId = pCfg->ProductId;
	g_BtAppData.AppDevice.ProductVer = pCfg->ProductVer;
	g_BtAppData.AppDevice.Appearance = pCfg->Appearance;

	g_BtAppData.AppDevice.Conn.MaxMtu = GATT_MTU_SIZE_DEFAULT;
	if (pCfg->MaxMtu > GATT_MTU_SIZE_DEFAULT)
		g_BtAppData.AppDevice.Conn.MaxMtu = pCfg->MaxMtu;

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
	DEBUG_PRINTF("BtAppInit: Role=0x%x PERIPHERAL=%d\r\n",
		pCfg->Role, (pCfg->Role & BTAPP_ROLE_PERIPHERAL) ? 1 : 0);
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		DEBUG_PRINTF("BtAppInit: calling BtAppInitUserServices\r\n");
		BtAppInitUserServices();
	}

	// Initialize Device Information Service
	if (pCfg->pDevInfo != NULL)
	{
		BtDisInit(pCfg);
	}

	// Initialize user data
	BtAppInitUserData();

	g_BtAppData.AppDevice.bSecure = pCfg->SecType != BTGAP_SECTYPE_NONE;

	// Initialize Secure Connections (peer_manager + LESC) when the app
	// requests security. Bonds persist through the IOsonata bt_pds store.
	if (g_BtAppData.AppDevice.bSecure)
	{
		// No erase-bond flag in BtAppCfg_t; bonds are preserved across init.
		// A dedicated clear (pm_peers_delete / BtSmpBondClearAll) can be added
		// as a separate API if forced re-bonding is needed.
		if (BtAppPeerMngrInit(pCfg->SecType, pCfg->SecExchg, false) != NRF_SUCCESS)
		{
			DEBUG_PRINTF("BtAppPeerMngrInit failed\r\n");
			return false;
		}
	}

	// Initialize advertising
	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		if (BtAppAdvInit(pCfg) == false)
		{
			DEBUG_PRINTF("BtAppAdvInit failed\r\n");
			return false;
		}

		err_code = sd_ble_gap_tx_power_set(
			BLE_GAP_TX_POWER_ROLE_ADV,
			g_BtAppData.AdvHdl,
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

	g_BtAppData.State = BTAPP_STATE_INITIALIZED;

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
// Millisecond clock for the generic SMP/GATT transaction timeouts, overriding
// the weak BtSmpMsTick/BtGattMsTick defaults. The RTC/timer is owned by the
// SDK + SoftDevice; this reads the free-running GRTC3 count via the s_BtAppSdGrtc3
// handle the port enables as a SoftDevice prerequisite (a read is
// non-destructive). Declared in bt_smp.h / bt_gatt.h, so no linkage specifier is
// needed here.
uint32_t BtSmpMsTick(void)
{
	return s_BtAppSdGrtc3.mSecond();
}

uint32_t BtGattMsTick(void)
{
	return s_BtAppSdGrtc3.mSecond();
}

// Spec-strict indication transaction timeout: Core Vol 3 Part F 3.3.3 requires
// closing the bearer, so disconnect the link. Overrides the generic weak
// default that only clears the outstanding-indication flag.
void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	sd_ble_gap_disconnect(ConnHdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

void BtAppRun()
{
	if (g_BtAppData.State != BTAPP_STATE_INITIALIZED)
	{
		return;
	}

	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtAdvStart();
	}

	DEBUG_PRINTF("BtAppRun: entering main loop\r\n");

	while (1)
	{
		// Process any pending LESC DHKey computation. Required for LE Secure
		// Connections: nrf_ble_lesc defers the ECDH to be run from the main
		// loop rather than the BLE event context.
		if (g_BtAppData.AppDevice.bSecure)
		{
			(void)nrf_ble_lesc_request_handler();
		}

		AppEvtHandlerExec();

		// Drive the generic transaction timeouts (Core Vol 3 Part H 3.4, Part F
		// 3.3.3). Cheap no-ops when nothing is pending. NOTE: this loop wakes on
		// events, so a link that goes fully silent needs a periodic wake to also
		// call these - hook them into an existing SDK/SoftDevice periodic callback
		// (do not add a trigger to GRTC3, which the SoftDevice owns).
		BtSmpTimeoutCheck();
		BtGattIndicationTimeoutCheck();

		BtAppEvtWait();
	}
}

// Port-level weak default for BtAppEvtWait. Bare-metal apps use __WFE.
// RTOS apps override with sem take in their bridge code.
__attribute__((weak)) void BtAppEvtWait(void)
{
	__WFE();
}

// Drains queued SoftDevice stack events by running the registered observers.
// Effect: pending stack events are delivered to their handlers. Precondition:
// SoftDevice enabled. Called from an RTOS BtAppEvtWait override after the
// waiter unblocks, so observers run in task context. Returns void.
void BtAppEvtDispatch()
{
	nrf_sdh_evts_poll();
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

/*
 * BLE event pumping is handled by the SDK nrf_sdh_ble.c ble_evt_poll, which is
 * registered as NRF_SDH_STACK_EVT_OBSERVER(ble_evt_obs, ...). That function
 * maintains the connection-handle index table (idx_assign on CONNECTED,
 * idx_unassign on DISCONNECTED) that nrf_sdh_ble_idx_get / peer_manager
 * conn_state rely on, then fans the event out to all NRF_SDH_BLE_OBSERVERs.
 * IOsonata's own GAP/GATT handling is one such observer (ble_evt_dispatch,
 * registered below). A second pump here would also drain sd_ble_evt_get and
 * fan out without the idx tracking, leaving nrf_sdh_ble_idx_get returning -1,
 * so it is intentionally not defined.
 */

/* Auto-handle seed requests as a SoC event observer */
NRF_SDH_SOC_OBSERVER(rand_seed, SDBleRandSeed, NULL, HIGH);
/* SoftDevice SoC event pump (BLE pump is the SDK nrf_sdh_ble.c one) */
NRF_SDH_STACK_EVT_OBSERVER(soc_evt_obs, soc_evt_poll, NULL, HIGHEST);
/* IOsonata GAP/GATT handling, driven by the SDK BLE event fan-out */
NRF_SDH_BLE_OBSERVER(s_BtAppBleObserver, ble_evt_dispatch, NULL, USER);



// =====================================================================
// Central / Observer role
// ---------------------------------------------------------------------
// BtAppScanInit programs the scan parameters and arms the first scan.
// BtAppScan continues an in-flight scan (after each ADV_REPORT) or
// starts a fresh one if scanning was idle.
// BtAppScanStop halts scanning.
// BtAppConnect issues a GAP connect request using the parameters that
// were set up by the most recent BtAppScanInit call.
// =====================================================================


bool BtAppConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)
{
	// SD requires scanning to be stopped before issuing a connect.
	g_BtAppData.bScan = false;

	return BtGapConnect(pPeerAddr, pConnParam);
}

// =====================================================================
// Central GATT client discovery (S145 SoftDevice path)
// ---------------------------------------------------------------------
// S145 owns the host. There is no BtHciDevice_t, so the ATT engine in
// bt_attreq.cpp / bt_attrsp.cpp does not apply here. Discovery runs on
// sd_ble_gattc_*_discover and the matching BLE_GATTC_EVT_* responses.
//
// Sequence: all primary services, then characteristics per service, then
// descriptors per characteristic to resolve the CCCD handle. On completion
// BtDeviceDiscovered(pDev) is called (weak default in bt_app.cpp, overridden
// by the app).
//
// Single active discovery at a time. The cursor lives in file-scope statics,
// not per peer, so concurrent central links are not supported yet. Per-link
// discovery would move this cursor into pDev->Discovery.
// =====================================================================

static BtDevice_t *s_pDiscDev    = NULL;    // peer under discovery
static uint8_t     s_DiscSrvIdx  = 0;       // service cursor (char and desc phases)
static uint8_t     s_DiscCharIdx = 0;       // characteristic cursor (desc phase)

static void BtAppDiscStartChar(BtDevice_t *pDev);
static void BtAppDiscStartDesc(BtDevice_t *pDev);

bool BtAppDiscoverDevice(BtDevice_t * const pDev)
{
	if (pDev == NULL)
	{
		return false;
	}

	s_pDiscDev    = pDev;
	s_DiscSrvIdx  = 0;
	s_DiscCharIdx = 0;

	pDev->NbSrvc = 0;
	memset(pDev->Services, 0, sizeof(BtGattDBSrvc_t) * BT_DEV_SERVICE_MAXCNT);

	// NULL uuid filter discovers every primary service from handle 1.
	return sd_ble_gattc_primary_services_discover(pDev->Conn.Hdl, 0x0001, NULL) == NRF_SUCCESS;
}

static void BtAppDiscStartChar(BtDevice_t *pDev)
{
	while (s_DiscSrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];
		ble_gattc_handle_range_t range;

		range.start_handle = pSrvc->handle_range.StartHdl + 1;  // skip service declaration
		range.end_handle   = pSrvc->handle_range.EndHdl;

		if (range.start_handle <= range.end_handle &&
		    sd_ble_gattc_characteristics_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
		{
			return;     // wait for BLE_GATTC_EVT_CHAR_DISC_RSP
		}
		s_DiscSrvIdx++; // empty service or request rejected; advance
	}

	// All services covered for characteristics. Resolve descriptors (CCCD).
	s_DiscSrvIdx  = 0;
	s_DiscCharIdx = 0;
	BtAppDiscStartDesc(pDev);
}

static void BtAppDiscStartDesc(BtDevice_t *pDev)
{
	while (s_DiscSrvIdx < pDev->NbSrvc)
	{
		BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];

		while (s_DiscCharIdx < pSrvc->char_count)
		{
			BtGattDBChar_t *pCh    = &pSrvc->characteristics[s_DiscCharIdx];
			uint16_t        valHdl = pCh->characteristic.handle_value;
			uint16_t        start  = valHdl + 1;
			uint16_t        end;

			if ((s_DiscCharIdx + 1) < pSrvc->char_count)
			{
				end = pSrvc->characteristics[s_DiscCharIdx + 1].characteristic.handle_decl - 1;
			}
			else
			{
				end = pSrvc->handle_range.EndHdl;
			}

			if (valHdl != 0xFFFF && start <= end)
			{
				ble_gattc_handle_range_t range;
				range.start_handle = start;
				range.end_handle   = end;
				if (sd_ble_gattc_descriptors_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
				{
					return; // wait for BLE_GATTC_EVT_DESC_DISC_RSP
				}
			}
			s_DiscCharIdx++; // no descriptor range or request rejected
		}
		s_DiscSrvIdx++;
		s_DiscCharIdx = 0;
	}

	// Every service and characteristic processed. Discovery complete.
	BtDeviceDiscovered(pDev);
}

static void BtAppDiscPrimSrvcRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = s_pDiscDev;
	if (pDev == NULL)
	{
		return;
	}

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		const ble_gattc_evt_prim_srvc_disc_rsp_t *p = &pEvt->params.prim_srvc_disc_rsp;
		uint16_t lastEnd = 0;

		for (uint16_t i = 0; i < p->count; i++)
		{
			if (pDev->NbSrvc >= BT_DEV_SERVICE_MAXCNT)
			{
				break;
			}

			BtGattDBSrvc_t *pSrvc = &pDev->Services[pDev->NbSrvc];
			pSrvc->srv_uuid.BaseIdx = 0;
			pSrvc->srv_uuid.Type    = BT_UUID_TYPE_16;
			pSrvc->srv_uuid.Uuid    = p->services[i].uuid.uuid;
			pSrvc->handle_range.StartHdl = p->services[i].handle_range.start_handle;
			pSrvc->handle_range.EndHdl   = p->services[i].handle_range.end_handle;
			pSrvc->char_count = 0;
			lastEnd = p->services[i].handle_range.end_handle;
			pDev->NbSrvc++;
		}

		// Continue past the last range unless DB end or local table full.
		if (lastEnd != 0xFFFF && pDev->NbSrvc < BT_DEV_SERVICE_MAXCNT &&
		    sd_ble_gattc_primary_services_discover(pDev->Conn.Hdl, lastEnd + 1, NULL) == NRF_SUCCESS)
		{
			return;
		}
	}

	// ATTRIBUTE_NOT_FOUND, DB end, or table full: service phase done.
	s_DiscSrvIdx = 0;
	BtAppDiscStartChar(pDev);
}

static void BtAppDiscCharRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = s_pDiscDev;
	if (pDev == NULL)
	{
		return;
	}

	BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		const ble_gattc_evt_char_disc_rsp_t *p = &pEvt->params.char_disc_rsp;
		uint16_t lastVal = 0;

		for (uint16_t i = 0; i < p->count; i++)
		{
			if (pSrvc->char_count >= BT_GATT_DB_MAX_CHARS)
			{
				break;
			}

			BtGattDBChar_t         *pCh  = &pSrvc->characteristics[pSrvc->char_count];
			const ble_gattc_char_t *pSrc = &p->chars[i];

			pCh->characteristic.uuid.BaseIdx = 0;
			pCh->characteristic.uuid.Type    = BT_UUID_TYPE_16;
			pCh->characteristic.uuid.Uuid    = pSrc->uuid.uuid;

			pCh->characteristic.char_props.broadcast      = pSrc->char_props.broadcast;
			pCh->characteristic.char_props.read           = pSrc->char_props.read;
			pCh->characteristic.char_props.write_wo_resp  = pSrc->char_props.write_wo_resp;
			pCh->characteristic.char_props.write          = pSrc->char_props.write;
			pCh->characteristic.char_props.notify         = pSrc->char_props.notify;
			pCh->characteristic.char_props.indicate       = pSrc->char_props.indicate;
			pCh->characteristic.char_props.auth_signed_wr = pSrc->char_props.auth_signed_wr;

			pCh->characteristic.char_ext_props = pSrc->char_ext_props;
			pCh->characteristic.handle_decl    = pSrc->handle_decl;
			pCh->characteristic.handle_value   = pSrc->handle_value;

			pCh->cccd_handle       = BLE_GATT_HANDLE_INVALID;
			pCh->ext_prop_handle   = BLE_GATT_HANDLE_INVALID;
			pCh->user_desc_handle  = BLE_GATT_HANDLE_INVALID;
			pCh->report_ref_handle = BLE_GATT_HANDLE_INVALID;

			lastVal = pSrc->handle_value;
			pSrvc->char_count++;
		}

		// Continue characteristics within the same service range.
		if (lastVal != 0 && lastVal < 0xFFFF && pSrvc->char_count < BT_GATT_DB_MAX_CHARS &&
		    (lastVal + 1) <= pSrvc->handle_range.EndHdl)
		{
			ble_gattc_handle_range_t range;
			range.start_handle = lastVal + 1;
			range.end_handle   = pSrvc->handle_range.EndHdl;
			if (sd_ble_gattc_characteristics_discover(pDev->Conn.Hdl, &range) == NRF_SUCCESS)
			{
				return;
			}
		}
	}

	// Service complete (ATTRIBUTE_NOT_FOUND, table full, or error). Advance.
	s_DiscSrvIdx++;
	BtAppDiscStartChar(pDev);
}

static void BtAppDiscDescRsp(const ble_gattc_evt_t *pEvt)
{
	BtDevice_t *pDev = s_pDiscDev;
	if (pDev == NULL)
	{
		return;
	}

	if (pEvt->gatt_status == BLE_GATT_STATUS_SUCCESS)
	{
		const ble_gattc_evt_desc_disc_rsp_t *p = &pEvt->params.desc_disc_rsp;
		BtGattDBSrvc_t *pSrvc = &pDev->Services[s_DiscSrvIdx];
		BtGattDBChar_t *pCh   = &pSrvc->characteristics[s_DiscCharIdx];

		for (uint16_t i = 0; i < p->count; i++)
		{
			switch (p->descs[i].uuid.uuid)
			{
				case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
					pCh->cccd_handle = p->descs[i].handle;
					break;
				case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
					pCh->ext_prop_handle = p->descs[i].handle;
					break;
				case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
					pCh->user_desc_handle = p->descs[i].handle;
					break;
				case BT_UUID_DESCRIPTOR_REPORT_REFERENCE:
					pCh->report_ref_handle = p->descs[i].handle;
					break;
				default:
					break;
			}
		}
	}

	// Advance to the next characteristic regardless of result.
	s_DiscCharIdx++;
	BtAppDiscStartDesc(pDev);
}

bool BtAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
	if (ConnHandle == BLE_CONN_HANDLE_INVALID ||
	    CharHandle == BLE_CONN_HANDLE_INVALID ||
	    pData == nullptr || DataLen == 0)
	{
		return false;
	}

	ble_gattc_write_params_t const write_params = {
		.write_op = BLE_GATT_OP_WRITE_CMD,
		.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
		.handle   = CharHandle,
		.offset   = 0,
		.len      = DataLen,
		.p_value  = pData,
	};

	return sd_ble_gattc_write(ConnHandle, &write_params) == NRF_SUCCESS;
}

bool BtAppEnableNotify(uint16_t ConnHandle, uint16_t CccdHandle)
{
	if (ConnHandle == BLE_CONN_HANDLE_INVALID ||
	    CccdHandle == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

	// CCCD = 16-bit little endian, bit 0 = notify, bit 1 = indicate.
	// CCCD value length is fixed at 2 bytes by the Bluetooth GATT spec.
	uint8_t buf[2] = { BLE_GATT_HVX_NOTIFICATION, 0 };

	ble_gattc_write_params_t const write_params = {
		.write_op = BLE_GATT_OP_WRITE_REQ,    // CCCD writes are write-with-response per spec
		.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
		.handle   = CccdHandle,
		.offset   = 0,
		.len      = sizeof(buf),
		.p_value  = buf,
	};

	return sd_ble_gattc_write(ConnHandle, &write_params) == NRF_SUCCESS;
}


