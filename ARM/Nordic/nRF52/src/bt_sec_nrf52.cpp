/**-------------------------------------------------------------------------
@file	bt_sec_nrf52.cpp

@brief	nRF5_SDK SoftDevice application security module.

		Extracted from bt_app_nrf52.cpp. Owns the Peer Manager start up and
		event handling, the ECDH engine used for LESC, the SMP user
		interaction bridge and the LESC OOB data.

		This object is linked only when the application calls BtAppSecInit,
		normally from BtAppInitUserData. An application that does not use
		security never references it, so the Peer Manager, its storage, the
		LESC module and the ECDH engine stay out of the image.

@author	Hoang Nguyen Hoan
@date	Oct. 1, 2026

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
#include <inttypes.h>
#include <string.h>

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_conn_state.h"
#include "peer_manager.h"
#include "fds.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"

#include "istddef.h"
#include "crypto/crypto_uecc.h"
#include "crypto_rng_nrf.h"
#if defined(NRF52840_XXAA)
#include "crypto_cc3xx.h"
#endif
#include "bt_lesc.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_dev.h"

/******** For DEBUG ************/
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

// Run after the application observer and the Peer Manager observer (both
// priority 1). On connect the peer record exists and the Peer Manager has
// seen the link before security is requested on it.
#define BT_SEC_NRF52_OBSERVER_PRIO      2

pm_peer_id_t g_PeerMngrIdToDelete = PM_PEER_ID_INVALID;

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    uint16_t role = ble_conn_state_role(p_evt->conn_handle);

    switch (p_evt->evt_id)
    {
		case PM_EVT_BONDED_PEER_CONNECTED:
			{
				// Start Security Request timer.
				//err_code = app_timer_start(g_SecReqTimerId, SECURITY_REQUEST_DELAY, NULL);
				//APP_ERROR_CHECK(err_code);
			}
			break;

			case PM_EVT_CONN_SEC_SUCCEEDED:
			{
				pm_conn_sec_status_t conn_sec_status;
				// Check if the link is authenticated (meaning at least MITM).
				err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
				APP_ERROR_CHECK(err_code);

				DEBUG_PRINTF("SEC: CONN_SEC_SUCCEEDED hdl=%d encrypted=%d mitm=%d bonded=%d lesc=%d proc=%d\r\n",
						p_evt->conn_handle,
						conn_sec_status.encrypted, conn_sec_status.mitm_protected,
						conn_sec_status.bonded, conn_sec_status.lesc,
						p_evt->params.conn_sec_succeeded.procedure);

				if (conn_sec_status.mitm_protected)
				{
				}
				else
				{
#if 0
					// The peer did not use MITM, disconnect.
					err_code = pm_peer_id_get(p_evt->conn_handle, &g_PeerMngrIdToDelete);
					APP_ERROR_CHECK(err_code);
					err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
													 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
#endif
				}

				// Link is encrypted. Notify the app so it can run work that needs
				// an encrypted link (e.g. a central reading protected chars).
				BtAppEvtSecured(p_evt->conn_handle);
			}
			break;

        case PM_EVT_CONN_SEC_FAILED:
            DEBUG_PRINTF("SEC: CONN_SEC_FAILED hdl=%d procedure=%d error=0x%X\r\n",
            		p_evt->conn_handle,
            		p_evt->params.conn_sec_failed.procedure,
            		p_evt->params.conn_sec_failed.error);
            // Drop the link that failed to secure, named by the event, not
            // whichever link was current.
            if (g_BtAppData.AppDevice.bSecure && p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
			{
				DEBUG_PRINTF("SEC: CONN_SEC_CONFIG_REQ hdl=%d (allow repairing)\r\n", p_evt->conn_handle);
				// Accept pairing request from an already bonded peer.
				pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
				pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
			}
			break;

        case PM_EVT_STORAGE_FULL:
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }

            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            BtAdvStart();
            break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();

            break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        	break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

// ===========================================================================
// SMP user interaction bridge (SoftDevice / Peer Manager port).
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
static uint32_t Nrf52PasskeyToVal(const uint8_t *pAscii)
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
static void Nrf52PasskeyFromVal(uint32_t Val, uint8_t *pAscii)
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
	Nrf52PasskeyFromVal(Passkey, ascii);
	(void)sd_ble_gap_auth_key_reply(ConnHdl, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, ascii);
}

// LE Secure Connections OOB data (strong overrides of the generic weak API).
// The IOsonata BtLesc module owns the SC key pair, so the local OOB set comes
// from it and the peer set is staged here; the peer data handler hands it to
// the pairing with the link peer address filled in.
static ble_gap_lesc_oob_data_t s_BtAppPeerOob;
static bool s_BtAppPeerOobValid = false;

static ble_gap_lesc_oob_data_t *BtAppOobPeerDataHandler(uint16_t ConnHdl)
{
	if (!s_BtAppPeerOobValid)
	{
		DEBUG_PRINTF("SEC: no OOB peer data, pairing will fail\r\n");
		return NULL;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer != nullptr)
	{
		s_BtAppPeerOob.addr.addr_type = pPeer->Conn.PeerAddrType;
		memcpy(s_BtAppPeerOob.addr.addr, pPeer->Conn.PeerAddr, 6);
	}

	return &s_BtAppPeerOob;
}

int BtSmpOobLocalDataGen(BtHciDevice_t * const pDev, uint8_t * const pRand, uint8_t * const pConf)
{
	(void)pDev;

	if (pRand == NULL || pConf == NULL)
	{
		return -1;
	}
	if (!BtLescOobLocalGen())
	{
		return -1;
	}

	ble_gap_lesc_oob_data_t *p = BtLescOobLocalGet();
	if (p == NULL)
	{
		return -1;
	}

	memcpy(pRand, p->r, 16);
	memcpy(pConf, p->c, 16);

	return 0;
}

void BtSmpOobPeerDataSet(const uint8_t * const pRand, const uint8_t * const pConf)
{
	if (pRand == NULL || pConf == NULL)
	{
		return;
	}
	memset(&s_BtAppPeerOob, 0, sizeof(s_BtAppPeerOob));
	memcpy(s_BtAppPeerOob.r, pRand, 16);
	memcpy(s_BtAppPeerOob.c, pConf, 16);
	s_BtAppPeerOobValid = true;
}

void BtSmpOobDataClear(void)
{
	s_BtAppPeerOobValid = false;
	memset(&s_BtAppPeerOob, 0, sizeof(s_BtAppPeerOob));
}

// The rejecting defaults for BtSmpNumericComparison, BtSmpPasskeyDisplay and
// BtSmpPasskeyRequest are weak in bt_smp.cpp, which is part of this library.
// A second weak definition here left the choice to link
// order. They reach this port through the strong BtSmpNumericComparisonReply
// and BtSmpPasskeyReply above, so the behaviour is what it was.

// Security events live here, not in the application dispatcher, so that the
// dispatcher holds no reference to the security module.
static void BtSecNrf52EvtHandler(ble_evt_t const *pEvt, void *pContext)
{
	(void)pContext;

	if (g_BtAppData.bSecInit == false)
	{
		return;
	}

	switch (pEvt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
		{
			uint16_t connHdl = pEvt->evt.gap_evt.conn_handle;

			// A link the peer pool rejected is not exposed to security.
			if (BtPeerFindByHdl(connHdl) == nullptr)
			{
				break;
			}

			// If a secure SecType was configured, the SoftDevice implementation requests
			// security on the link itself (pm_conn_secure -> sd_ble_gap_authenticate).
			// This is port-internal so the application stays SDK-neutral - it
			// does not call any port-specific security-request function.
			if (g_BtAppData.AppDevice.bSecure)
			{
				uint32_t err_code = pm_conn_secure(connHdl, false);
				DEBUG_PRINTF("SEC: connect hdl=%d bSecure=1 pm_conn_secure=0x%X\r\n",
						connHdl, err_code);
				if (err_code != NRF_ERROR_INVALID_STATE && err_code != NRF_ERROR_BUSY)
				{
					APP_ERROR_CHECK(err_code);
				}
			}
			else
			{
				DEBUG_PRINTF("SEC: connect hdl=%d bSecure=0 (no security requested)\r\n",
						connHdl);
			}
		}
			break;
		case BLE_GAP_EVT_PASSKEY_DISPLAY:
		{
			const ble_gap_evt_passkey_display_t *pPd =
					&pEvt->evt.gap_evt.params.passkey_display;
			uint16_t connHdl = pEvt->evt.gap_evt.conn_handle;
			uint32_t val = Nrf52PasskeyToVal(pPd->passkey);
			DEBUG_PRINTF("SEC: EVT_PASSKEY_DISPLAY match_request=%d\r\n", pPd->match_request);
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
			DEBUG_PRINTF("SEC: EVT_AUTH_KEY_REQUEST type=%d (passkey/oob needed)\r\n",
					pEvt->evt.gap_evt.params.auth_key_request.key_type);
			if (pEvt->evt.gap_evt.params.auth_key_request.key_type ==
				BLE_GAP_AUTH_KEY_TYPE_PASSKEY)
			{
				// Passkey Entry input side: the application provides the value
				// through BtSmpPasskeyReply.
				BtSmpPasskeyRequest(pEvt->evt.gap_evt.conn_handle);
			}
			break;
		default:
			break;
	}
}

NRF_SDH_BLE_OBSERVER(s_BtSecObserver, BT_SEC_NRF52_OBSERVER_PRIO,
	BtSecNrf52EvtHandler, NULL);

//
// bt_lesc port hooks. The s132 SoftDevice sd_ble_gap_lesc_dhkey_reply takes no
// security status and requires a valid key pointer: a NULL key returns
// NRF_ERROR_INVALID_ADDR and the DHKey request stays unanswered until the SMP
// transaction timeout. On failure reply with a random key instead, so the
// pairing fails cleanly at the peer DHKey check. The peer-key table is sized
// from the SoftDevice link configuration.
//
extern "C" uint32_t BtLescDhKeyReply(uint16_t ConnHdl, uint8_t SecStatus,
									 const ble_gap_lesc_dhkey_t *pDhKey)
{
	(void)SecStatus;

	if (pDhKey == NULL)
	{
		// Deliberately wrong DH key: any value that differs from the real ECDH
		// result fails the peer DHKey check. If the RNG draw fails the zero
		// filled key is still wrong, so reply regardless.
		static ble_gap_lesc_dhkey_t s_BadDhKey;

		(void)CryptoRngNrfInstance()->Random(s_BadDhKey.key, BLE_GAP_LESC_DHKEY_LEN);
		pDhKey = &s_BadDhKey;
	}

	return sd_ble_gap_lesc_dhkey_reply(ConnHdl, pDhKey);
}

extern "C" int BtLescLinkCount(void)
{
	return NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT;
}

// Strong override of the weak generic BtGapConnSecGet. On nRF52 the SoftDevice
// and Peer Manager own link security, so map their live status into BtConnSec_t.
// Level and flags come from pm_conn_sec_status_get; KeySize is the negotiated
// encryption key size captured at BLE_GAP_EVT_CONN_SEC_UPDATE.
bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (pSec == nullptr)
	{
		return false;
	}

	BtDevice_t *p = BtPeerFindByHdl(ConnHdl);
	if (p == nullptr)
	{
		return false;
	}

	pm_conn_sec_status_t st;
	if (pm_conn_sec_status_get(ConnHdl, &st) != NRF_SUCCESS)
	{
		return false;
	}

	pSec->Level = BT_GAP_SEC_LEVEL_NONE;
	pSec->KeySize = 0;
	pSec->Flags = 0;

	if (st.encrypted)
	{
		if (st.mitm_protected)
		{
			pSec->Level = st.lesc ? BT_GAP_SEC_LEVEL_LESC_AUTH : BT_GAP_SEC_LEVEL_ENC_AUTH;
		}
		else
		{
			pSec->Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
		}

		// Real negotiated size. 0 here only if the update event has not run,
		// which keeps the key-size check on the safe (reject) side.
		pSec->KeySize = p->Conn.Sec.KeySize;
	}

	if (st.bonded)
	{
		pSec->Flags |= BT_GAP_SEC_FLAG_BONDED;
	}

	if (st.lesc)
	{
		pSec->Flags |= BT_GAP_SEC_FLAG_SC;
	}

	return true;
}

// Defined in bt_app_nrf52.cpp: gives BtAppRun the main loop work of this
// module.
void BtAppNrf52SecPollSet(void (*Poll)(void));

// Main loop work of the LESC module: key pair generation and DHKey replies.
static void BtSecNrf52Poll(void)
{
	(void)BtLescRequestHandler();
}

/**
 * @brief	Start the security module.
 *
 * Called by the application, normally from BtAppInitUserData. The call is
 * what links this module: the Peer Manager, its storage, the LESC module and
 * the ECDH engine. Uses the SecType and SecExchg given to BtAppInit.
 *
 * @return	true - security started
 */
bool BtAppSecInit(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;
    BTGAP_SECTYPE        SecType = g_BtAppData.SecType;
    uint8_t              SecKeyExchg = g_BtAppData.SecExchg;

    if (g_BtAppData.bSecInit)
    {
    	// Already started
    	return true;
    }

    // Security works on a link, start connection support first
    if (BtAppConnInit() == false)
    {
    	return false;
    }

    // Select the ECDH engine and inject it before pm_init: the IOsonata
    // security manager (bt_sec_sd) calls BtLescInit() during init, so the
    // engine must already be in place or init fails. The nRF52840 has the
    // CryptoCell CC310 hardware P-256 (CryptoCc3xx); the nRF52832 has no
    // accelerator and uses software P-256 (CryptoUecc) over the SoftDevice and
    // peripheral RNG. The App owns the engine, the same model as the SDC
    // pairing path. The module owns the key pair, handles the LESC DHKey
    // request and replies to the SoftDevice; the app only pumps
    // BtLescRequestHandler in the main loop.
    KeyAgreeEngine *pLescEcdh = nullptr;
#if defined(NRF52840_XXAA)
    alignas(CryptoCc3xx) static uint8_t s_LescEcdhMem[CRYPTO_CC3XX_MEMSIZE];    // CC310 engine object
    pLescEcdh = CryptoCc3xxCreate(s_LescEcdhMem, sizeof(s_LescEcdhMem),
                                  CryptoRngNrfInstance());
    if (pLescEcdh != nullptr)
    {
        DEBUG_PRINTF("Crypto ECDH engine: CryptoCc3xx (CC310 hardware P-256)\r\n");
    }
#else
    alignas(uint64_t) static uint8_t s_LescEcdhMem[CRYPTO_UECC_MEMSIZE];
    pLescEcdh = CryptoUeccCreate(s_LescEcdhMem, sizeof(s_LescEcdhMem),
                                 CryptoRngNrfInstance());
    if (pLescEcdh != nullptr)
    {
        DEBUG_PRINTF("Crypto ECDH engine: CryptoUecc (software P-256)\r\n");
    }
#endif
    if (pLescEcdh == nullptr)
    {
        DEBUG_PRINTF("Crypto ECDH engine MISSING, LESC pairing will fail\r\n");
    }
    BtLescSetCryptoEngine(pLescEcdh);

    // Which persistence this build uses. This path is the nRF5 SDK
    // peer_manager, which stores bonds through FDS on top of fstorage. It
    // never reaches bt_pds or Nvm; that is the SDC path (bt_app_sdc.cpp,
    // BtSmpBondNvmInit). Printed so a persistence problem is not chased in
    // the wrong layer.
    DEBUG_PRINTF("STORE: nRF5 SDK peer_manager -> fds -> fstorage\r\n");
#ifdef NRF_SD_BLE_API_VERSION
    DEBUG_PRINTF("STORE: SD BLE API v%d\r\n", (int)NRF_SD_BLE_API_VERSION);
#endif
#ifdef S132
    DEBUG_PRINTF("STORE: SoftDevice S132\r\n");
#elif defined(S140)
    DEBUG_PRINTF("STORE: SoftDevice S140\r\n");
#endif
    {
        ble_version_t ver;
        memset(&ver, 0, sizeof(ver));
        if (sd_ble_version_get(&ver) == NRF_SUCCESS)
        {
            DEBUG_PRINTF("STORE: SD fwid 0x%04X company 0x%04X ver %d\r\n",
                         ver.subversion_number, ver.company_id,
                         ver.version_number);
        }
    }

    err_code = pm_init();
    DEBUG_PRINTF("SEC: pm_init=0x%X\r\n", err_code);
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    sec_param.bond = 1;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    switch (SecType)
    {
    	case BTGAP_SECTYPE_NONE:
			break;
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
			break;
		case BTGAP_SECTYPE_STATICKEY_MITM:
			sec_param.mitm = 1;
			break;
		case BTGAP_SECTYPE_LESC_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			sec_param.mitm = 1;
		    sec_param.lesc = 1;
			break;
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
		    sec_param.lesc = 1;
			break;
    }

/*    if (SecType == BLEAPP_SECTYPE_STATICKEY_MITM ||
    	SecType == BLEAPP_SECTYPE_LESC_MITM ||
		SecType == BLEAPP_SECTYPE_SIGNED_MITM)
    {
    	sec_param.mitm = 1;
    }
*/
    int type = SecKeyExchg & (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO);
    switch (type)
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
    }

    if (SecKeyExchg & BTAPP_SECEXCHG_OOB)
    {
    	sec_param.oob = 1;
//    	nfc_ble_pair_init(&g_AdvInstance, NFC_PAIRING_MODE_JUST_WORKS);
    }

    err_code = pm_sec_params_set(&sec_param);
    DEBUG_PRINTF("SEC: pm_sec_params_set=0x%X lesc=%d mitm=%d bond=%d io=%d\r\n",
    		err_code, sec_param.lesc, sec_param.mitm, sec_param.bond, sec_param.io_caps);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    DEBUG_PRINTF("SEC: pm_register=0x%X\r\n", err_code);
    APP_ERROR_CHECK(err_code);

	// Route the staged peer OOB data into the pairing when the SoftDevice
	// asks for it (LESC OOB association model).
	BtLescOobPeerHandlerSet(BtAppOobPeerDataHandler);

	// The module owns the LESC key pair and the DHKey computation. Both are
	// run from the main loop, after the queued event handlers.
	BtAppNrf52SecPollSet(BtSecNrf52Poll);

	g_BtAppData.bSecInit = true;

	return true;
}
