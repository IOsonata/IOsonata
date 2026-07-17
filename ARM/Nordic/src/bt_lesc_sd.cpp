/**-------------------------------------------------------------------------
@file	bt_lesc_sd.cpp

@brief	LE Secure Connections ECDH over an injected KeyAgreeEngine.

		The SoftDevice owns the SMP state machine and delegates P-256 work to this
		module. Peer requests are deferred from the stack callback to the main loop.
		One local key pair may serve several simultaneous peers, so Agree keeps the
		private key. The module explicitly destroys that key before replacement or
		when a different engine is installed.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <string.h>

#include "ble_gap.h"
#include "nrf_error.h"

#include "crypto/icrypto.h"
#include "bt_lesc.h"
#include "syslog.h"

#define BT_LESC_TRACE_ENABLE
#if defined(BT_LESC_TRACE_ENABLE)
#define LESC_TRACE(...)	SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define LESC_TRACE(...)
#endif

#define LESC_COORD_SIZE	(BLE_GAP_LESC_P256_PK_LEN / 2)
#define LESC_MAX_LINK	8
#define LESC_KEYCTX_SIZE	64U

typedef struct {
	uint8_t Value[BLE_GAP_LESC_P256_PK_LEN];
	uint16_t ConnHdl;
	bool bRequested;
} BtLescPeerKey_t;

static ble_gap_lesc_p256_pk_t s_LescPubKey __attribute__((aligned(4)));
static ble_gap_lesc_dhkey_t s_LescDhKey __attribute__((aligned(4)));
static bool s_bInternalError;
static bool s_bKeyPairGen;
static bool s_bRegenPending;
static BtLescPeerKey_t s_PeerKeys[LESC_MAX_LINK];
static bool s_bOobLocalGen;
static ble_gap_lesc_oob_data_t s_OobLocal;
static BtLescOobPeerHandler_t s_OobPeerHandler;
static KeyAgreeEngine *s_pLescCrypto;
// Aligned to the maximum any provider can request so a provider that needs
// wider alignment than 8 bytes still gets conforming storage. Provider
// acceptance also checks KeyCtxAlign against this bound.
alignas(CRYPTO_KEYCTX_ALIGN_MAX) static uint8_t s_LescEcdhKeyCtx[LESC_KEYCTX_SIZE];

static void ByteOrderInvert(const uint8_t *pIn, uint8_t *pOut)
{
	for (int i = 0; i < LESC_COORD_SIZE; i++)
	{
		pOut[i] = pIn[LESC_COORD_SIZE - 1 - i];
		pOut[LESC_COORD_SIZE + i] = pIn[2 * LESC_COORD_SIZE - 1 - i];
	}
}

static int LinkCount(void)
{
	int count = BtLescLinkCount();
	if (count < 1) count = 1;
	if (count > LESC_MAX_LINK) count = LESC_MAX_LINK;
	return count;
}

static int SlotFind(uint16_t ConnHdl)
{
	int freeSlot = -1;
	for (int i = 0; i < LinkCount(); i++)
	{
		if (s_PeerKeys[i].bRequested && s_PeerKeys[i].ConnHdl == ConnHdl)
		{
			return i;
		}
		if (freeSlot < 0 && !s_PeerKeys[i].bRequested)
		{
			freeSlot = i;
		}
	}
	return freeSlot;
}

static void LocalKeyReset(void)
{
	if (s_pLescCrypto != nullptr)
	{
		s_pLescCrypto->KeyReset(s_LescEcdhKeyCtx);
	}
	else
	{
		CryptoSecureWipe(s_LescEcdhKeyCtx, sizeof(s_LescEcdhKeyCtx));
	}
	s_bKeyPairGen = false;
	s_bOobLocalGen = false;
	CryptoSecureWipe(&s_LescPubKey, sizeof(s_LescPubKey));
}

void BtLescSetCryptoEngine(KeyAgreeEngine *pEcdh)
{
	LocalKeyReset();
	s_pLescCrypto = pEcdh != nullptr &&
		pEcdh->KeyCtxSize() > 0U &&
		pEcdh->KeyCtxSize() <= sizeof(s_LescEcdhKeyCtx) &&
		pEcdh->KeyCtxAlign() > 0U &&
		pEcdh->KeyCtxAlign() <= CRYPTO_KEYCTX_ALIGN_MAX ? pEcdh : nullptr;
}

bool BtLescKeyPairGen(void)
{
	for (int i = 0; i < LinkCount(); i++)
	{
		if (s_PeerKeys[i].bRequested)
		{
			return false;
		}
	}

	if (s_pLescCrypto == nullptr ||
		s_pLescCrypto->KeyCtxSize() == 0U ||
		s_pLescCrypto->KeyCtxSize() > sizeof(s_LescEcdhKeyCtx) ||
		s_pLescCrypto->KeyCtxAlign() == 0U ||
		s_pLescCrypto->KeyCtxAlign() > CRYPTO_KEYCTX_ALIGN_MAX)
	{
		return false;
	}

	LocalKeyReset();
	uint8_t pubBe[BLE_GAP_LESC_P256_PK_LEN];
	if (s_pLescCrypto->KeyGen(CRYPTO_CURVE_P256, s_LescEcdhKeyCtx, pubBe) !=
		CRYPTO_STATUS_OK)
	{
		CryptoSecureWipe(pubBe, sizeof(pubBe));
		LESC_TRACE("LESC keypair generate failed\r\n");
		return false;
	}
	ByteOrderInvert(pubBe, s_LescPubKey.pk);
	CryptoSecureWipe(pubBe, sizeof(pubBe));
	s_bKeyPairGen = true;
	return true;
}

bool BtLescInit(void)
{
	memset(s_PeerKeys, 0, sizeof(s_PeerKeys));
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
	if (s_pLescCrypto == nullptr ||
		s_pLescCrypto->KeyCtxSize() == 0U ||
		s_pLescCrypto->KeyCtxSize() > sizeof(s_LescEcdhKeyCtx))
	{
		LESC_TRACE("LESC crypto engine missing or key context too large\r\n");
		return false;
	}
	s_bInternalError = false;
	s_bRegenPending = false;
	return BtLescKeyPairGen();
}

ble_gap_lesc_p256_pk_t *BtLescPubKeyGet(void)
{
	if (!s_bKeyPairGen)
	{
		LESC_TRACE("LESC public key accessed before generation\r\n");
		return NULL;
	}
	return &s_LescPubKey;
}

bool BtLescOobLocalGen(void)
{
	s_bOobLocalGen = false;
	if (!s_bKeyPairGen)
	{
		return false;
	}
	if (sd_ble_gap_lesc_oob_data_get(BLE_CONN_HANDLE_INVALID,
		&s_LescPubKey, &s_OobLocal) != NRF_SUCCESS)
	{
		return false;
	}
	s_bOobLocalGen = true;
	return true;
}

ble_gap_lesc_oob_data_t *BtLescOobLocalGet(void)
{
	if (!s_bOobLocalGen)
	{
		LESC_TRACE("LESC OOB data accessed before generation\r\n");
		return NULL;
	}
	return &s_OobLocal;
}

void BtLescOobPeerHandlerSet(BtLescOobPeerHandler_t Handler)
{
	s_OobPeerHandler = Handler;
}

static bool ComputeAndReply(BtLescPeerKey_t *pPeer)
{
	if (pPeer == nullptr || !s_bKeyPairGen || s_pLescCrypto == nullptr)
	{
		return false;
	}

	uint8_t peerBe[BLE_GAP_LESC_P256_PK_LEN];
	uint8_t dhBe[BLE_GAP_LESC_DHKEY_LEN];
	memset(peerBe, 0, sizeof(peerBe));
	memset(dhBe, 0, sizeof(dhBe));
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));

	const ble_gap_lesc_dhkey_t *pDh = NULL;
	uint8_t secStatus = BLE_GAP_SEC_STATUS_DHKEY_FAILURE;
	if (memcmp(s_LescPubKey.pk, pPeer->Value, LESC_COORD_SIZE) != 0)
	{
		ByteOrderInvert(pPeer->Value, peerBe);
		if (s_pLescCrypto->Agree(CRYPTO_CURVE_P256, s_LescEcdhKeyCtx,
			peerBe, dhBe, true) == CRYPTO_STATUS_OK)
		{
			for (int i = 0; i < BLE_GAP_LESC_DHKEY_LEN; i++)
			{
				s_LescDhKey.key[i] = dhBe[BLE_GAP_LESC_DHKEY_LEN - 1 - i];
			}
			pDh = &s_LescDhKey;
			secStatus = BLE_GAP_SEC_STATUS_SUCCESS;
		}
		else
		{
			LESC_TRACE("LESC ECDH failed\r\n");
		}
	}
	else
	{
		LESC_TRACE("LESC peer used identical public key\r\n");
	}

	uint32_t result = BtLescDhKeyReply(pPeer->ConnHdl, secStatus, pDh);
	CryptoSecureWipe(peerBe, sizeof(peerBe));
	CryptoSecureWipe(dhBe, sizeof(dhBe));
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
	return result == NRF_SUCCESS;
}

bool BtLescRequestHandler(void)
{
	if (s_bInternalError)
	{
		return false;
	}

	for (int i = 0; i < LinkCount(); i++)
	{
		if (s_PeerKeys[i].bRequested)
		{
			bool ok = ComputeAndReply(&s_PeerKeys[i]);
			CryptoSecureWipe(&s_PeerKeys[i], sizeof(s_PeerKeys[i]));
			if (!ok)
			{
				return false;
			}
		}
	}

	if (s_bRegenPending)
	{
		for (int i = 0; i < LinkCount(); i++)
		{
			if (s_PeerKeys[i].bRequested)
			{
				return true;
			}
		}
		if (!BtLescKeyPairGen())
		{
			LESC_TRACE("LESC keypair regenerate failed\r\n");
			s_bInternalError = true;
			return false;
		}
		s_bRegenPending = false;
	}
	return true;
}

static void OnDhKeyRequest(uint16_t ConnHdl,
						   const ble_gap_evt_lesc_dhkey_request_t *pReq)
{
	if (pReq == nullptr || pReq->p_pk_peer == nullptr)
	{
		s_bInternalError = true;
		return;
	}
	int index = SlotFind(ConnHdl);
	if (index < 0)
	{
		s_bInternalError = true;
		return;
	}
	memcpy(s_PeerKeys[index].Value, pReq->p_pk_peer->pk,
		BLE_GAP_LESC_P256_PK_LEN);
	s_PeerKeys[index].ConnHdl = ConnHdl;
	s_PeerKeys[index].bRequested = true;
}

static uint32_t OobDataSet(uint16_t ConnHdl)
{
	ble_gap_lesc_oob_data_t *pOwn = s_bOobLocalGen ? &s_OobLocal : NULL;
	ble_gap_lesc_oob_data_t *pPeer = s_OobPeerHandler != nullptr ?
		s_OobPeerHandler(ConnHdl) : NULL;
	return sd_ble_gap_lesc_oob_data_set(ConnHdl, pOwn, pPeer);
}

void BtLescOnBleEvt(const ble_evt_t *pEvt)
{
	if (pEvt == NULL)
	{
		return;
	}
	uint16_t connHdl = pEvt->evt.gap_evt.conn_handle;
	switch (pEvt->header.evt_id)
	{
	case BLE_GAP_EVT_DISCONNECTED:
		{
			int index = SlotFind(connHdl);
			if (index >= 0 && s_PeerKeys[index].bRequested &&
				s_PeerKeys[index].ConnHdl == connHdl)
			{
				CryptoSecureWipe(&s_PeerKeys[index], sizeof(s_PeerKeys[index]));
			}
			CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
		}
		break;

	case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		if (pEvt->evt.gap_evt.params.lesc_dhkey_request.oobd_req &&
			OobDataSet(connHdl) != NRF_SUCCESS)
		{
			LESC_TRACE("LESC oob_data_set failed\r\n");
			s_bInternalError = true;
		}
		OnDhKeyRequest(connHdl,
			&pEvt->evt.gap_evt.params.lesc_dhkey_request);
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		// Agree kept the shared local private key for possible concurrent peers.
		// Once the pairing procedure ends, schedule explicit destruction and a
		// fresh key pair outside the stack callback.
		s_bRegenPending = true;
		break;

	default:
		break;
	}
}
