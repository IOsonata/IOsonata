/**-------------------------------------------------------------------------
@file	bt_sec_nrf52.cpp

@brief	nRF5_SDK SoftDevice application security module.

		Extracted from bt_app_nrf52.cpp. Owns SoftDevice security startup and
		event handling, the ECDH engine used for LESC, the SMP user
		interaction bridge and the LESC OOB data.

		This object is linked only when the application calls BtAppSecInit,
		normally from BtAppInitUserData. An application that does not use
		security never references it, so the security state, bond store, LESC
		module and ECDH engine stay out of the image.

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
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"

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

#define SEC_PARAM_MIN_KEY_SIZE          BT_SMP_CFG_MIN_ENC_KEY_SIZE
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

// Run after the application observer (priority 1), which creates the BtPeer
// record on CONNECTED before security is requested on it.
#define BT_SEC_NRF52_OBSERVER_PRIO      2

bool BtSecSdInit(const ble_gap_sec_params_t *pParams);
void BtSecSdBleEvt(const ble_evt_t *pEvt);
ret_code_t BtSecSdSecure(uint16_t ConnHdl, bool ForceRepair);
void BtSecSdAuthConfig(uint8_t IoCaps, uint8_t AuthReq);
void BtSecSdOobSet(bool Enable);


// ===========================================================================
// SMP user interaction bridge (SoftDevice port).
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
	BtSecSdOobSet(true);
}

void BtSmpOobDataClear(void)
{
	s_BtAppPeerOobValid = false;
	memset(&s_BtAppPeerOob, 0, sizeof(s_BtAppPeerOob));
	BtSecSdOobSet(false);
}

void BtSmpAuthConfig(uint8_t IoCaps, uint8_t AuthReq)
{
	BtSecSdAuthConfig(IoCaps, AuthReq);
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

	BtSecSdBleEvt(pEvt);

	switch (pEvt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
		{
			uint16_t connHdl = pEvt->evt.gap_evt.conn_handle;
			if (BtPeerFindByHdl(connHdl) != nullptr &&
				g_BtAppData.AppDevice.bSecure)
			{
				ret_code_t r = BtSecSdSecure(connHdl, false);
				DEBUG_PRINTF("SEC: connect hdl=%d secure=0x%X\r\n",
						connHdl, (unsigned)r);
			}
			break;
		}

		case BLE_GAP_EVT_PASSKEY_DISPLAY:
		{
			const ble_gap_evt_passkey_display_t *pPd =
					&pEvt->evt.gap_evt.params.passkey_display;
			uint16_t connHdl = pEvt->evt.gap_evt.conn_handle;
			uint32_t val = Nrf52PasskeyToVal(pPd->passkey);
			DEBUG_PRINTF("SEC: EVT_PASSKEY_DISPLAY match_request=%d\r\n",
					pPd->match_request);
			if (pPd->match_request)
			{
				BtSmpNumericComparison(connHdl, val);
			}
			else
			{
				BtSmpPasskeyDisplay(connHdl, val);
			}
			break;
		}

		case BLE_GAP_EVT_AUTH_KEY_REQUEST:
			DEBUG_PRINTF("SEC: EVT_AUTH_KEY_REQUEST type=%d\r\n",
					pEvt->evt.gap_evt.params.auth_key_request.key_type);
			if (pEvt->evt.gap_evt.params.auth_key_request.key_type ==
				BLE_GAP_AUTH_KEY_TYPE_PASSKEY)
			{
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

/**
 * @brief	Start the security module.
 *
 * Called by the application, normally from BtAppInitUserData. The call is
 * what links this module: the IOsonata security state/bond store, LESC module and
 * the ECDH engine. Uses the SecType and SecExchg given to BtAppInit.
 *
 * @return	true - security started
 */
bool BtAppSecInit(void)
{
	BTGAP_SECTYPE SecType = g_BtAppData.SecType;
	uint8_t SecKeyExchg = g_BtAppData.SecExchg;

	if (g_BtAppData.bSecInit)
	{
		return true;
	}
	if (!BtAppConnInit())
	{
		return false;
	}

	KeyAgreeEngine *pLescEcdh = nullptr;
#if defined(NRF52840_XXAA)
	alignas(CryptoCc3xx) static uint8_t s_LescEcdhMem[CRYPTO_CC3XX_MEMSIZE];
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
		DEBUG_PRINTF("Crypto ECDH engine missing\r\n");
		return false;
	}
	BtLescSetCryptoEngine(pLescEcdh);

	ble_gap_sec_params_t secParam;
	memset(&secParam, 0, sizeof(secParam));
	secParam.bond = 1;
	secParam.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	secParam.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
	secParam.io_caps = BLE_GAP_IO_CAPS_NONE;
	secParam.kdist_own.enc = 1;
	secParam.kdist_own.id = 1;
	secParam.kdist_peer.enc = 1;
	secParam.kdist_peer.id = 1;

	switch (SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_MITM:
			secParam.mitm = 1;
			break;

		case BTGAP_SECTYPE_LESC_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			secParam.mitm = 1;
			secParam.lesc = 1;
			break;

		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			secParam.lesc = 1;
			break;

		case BTGAP_SECTYPE_NONE:
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
		default:
			break;
	}

	int type = SecKeyExchg &
		(BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO);
	switch (type)
	{
		case BTAPP_SECEXCHG_KEYBOARD:
			secParam.keypress = 1;
			secParam.io_caps = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
			break;
		case BTAPP_SECEXCHG_DISPLAY:
			secParam.io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
			break;
		case (BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO):
			secParam.io_caps = BLE_GAP_IO_CAPS_DISPLAY_YESNO;
			break;
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY):
		case (BTAPP_SECEXCHG_KEYBOARD | BTAPP_SECEXCHG_DISPLAY |
			  BTAPP_SECEXCHG_YESNO):
			secParam.keypress = 1;
			secParam.io_caps = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
			break;
		default:
			break;
	}

	if (SecKeyExchg & BTAPP_SECEXCHG_OOB)
	{
		secParam.oob = 1;
	}

	if (!BtSecSdInit(&secParam))
	{
		return false;
	}

	BtLescOobPeerHandlerSet(BtAppOobPeerDataHandler);
	g_BtAppData.bSecInit = true;

	DEBUG_PRINTF("STORE: bt_pds -> Nvm (SoftDevice, no peer_manager/fds)\r\n");
#ifdef NRF_SD_BLE_API_VERSION
	DEBUG_PRINTF("SEC: SoftDevice API v%d\r\n", (int)NRF_SD_BLE_API_VERSION);
#endif
	return true;
}
