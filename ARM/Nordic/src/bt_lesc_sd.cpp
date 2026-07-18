/**-------------------------------------------------------------------------
@file	bt_lesc_sd.cpp

@brief	LE Secure Connections ECDH over an injected KeyAgreeEngine.

		The SoftDevice owns the SMP state machine and delegates P-256 work to this
		module. Peer requests are deferred from the stack callback to the main loop.
		One local key pair may serve several simultaneous peers. Each connection
		that receives the public key is tracked until authentication completes or
		the link is released, so the shared private key cannot be replaced early.

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
#include <stdint.h>
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

#define LESC_COORD_SIZE		(BLE_GAP_LESC_P256_PK_LEN / 2)
#define LESC_MAX_LINK		8
#define LESC_KEYCTX_SIZE	64U

typedef struct {
	uint8_t Value[BLE_GAP_LESC_P256_PK_LEN];
	uint16_t ConnHdl;
	bool bAssigned;
	bool bRequested;
	bool bForceFail;
} BtLescPeerKey_t;

static ble_gap_lesc_p256_pk_t s_LescPubKey __attribute__((aligned(4)));
static ble_gap_lesc_dhkey_t s_LescDhKey __attribute__((aligned(4)));
static bool s_bKeyPairGen;
static bool s_bRegenPending;
static BtLescPeerKey_t s_PeerKeys[LESC_MAX_LINK];
static bool s_bOobLocalGen;
static ble_gap_lesc_oob_data_t s_OobLocal;
static BtLescOobPeerHandler_t s_OobPeerHandler;
static KeyAgreeEngine *s_pLescCrypto;
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
	for (int i = 0; i < LinkCount(); i++)
	{
		if (s_PeerKeys[i].bAssigned && s_PeerKeys[i].ConnHdl == ConnHdl)
		{
			return i;
		}
	}
	return -1;
}

static int SlotAlloc(uint16_t ConnHdl)
{
	int index = SlotFind(ConnHdl);
	if (index >= 0)
	{
		return index;
	}
	for (int i = 0; i < LinkCount(); i++)
	{
		if (!s_PeerKeys[i].bAssigned)
		{
			CryptoSecureWipe(&s_PeerKeys[i], sizeof(s_PeerKeys[i]));
			s_PeerKeys[i].ConnHdl = ConnHdl;
			s_PeerKeys[i].bAssigned = true;
			return i;
		}
	}
	return -1;
}

static bool KeyInUse(void)
{
	for (int i = 0; i < LinkCount(); i++)
	{
		if (s_PeerKeys[i].bAssigned)
		{
			return true;
		}
	}
	return false;
}

static void SlotRelease(int Index)
{
	if (Index < 0 || Index >= LinkCount() || !s_PeerKeys[Index].bAssigned)
	{
		return;
	}
	CryptoSecureWipe(&s_PeerKeys[Index], sizeof(s_PeerKeys[Index]));
	s_bRegenPending = true;
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
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
	CryptoSecureWipe(&s_OobLocal, sizeof(s_OobLocal));
}

void BtLescSetCryptoEngine(KeyAgreeEngine *pEcdh)
{
	LocalKeyReset();
	CryptoSecureWipe(s_PeerKeys, sizeof(s_PeerKeys));
	s_bRegenPending = false;
	s_pLescCrypto = pEcdh != nullptr && !pEcdh->IsAsync() &&
		pEcdh->KeyCtxSize() > 0U &&
		pEcdh->KeyCtxSize() <= sizeof(s_LescEcdhKeyCtx) &&
		pEcdh->KeyCtxAlign() > 0U &&
		pEcdh->KeyCtxAlign() <= CRYPTO_KEYCTX_ALIGN_MAX ? pEcdh : nullptr;
}

CRYPTO_STATUS BtLescKeyPairGenStatus(void)
{
	if (KeyInUse())
	{
		return CRYPTO_STATUS_BUSY;
	}
	if (s_pLescCrypto == nullptr)
	{
		LESC_TRACE("LESC keygen: no synchronous crypto engine bound\r\n");
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (s_pLescCrypto->KeyCtxSize() == 0U ||
		s_pLescCrypto->KeyCtxSize() > sizeof(s_LescEcdhKeyCtx) ||
		s_pLescCrypto->KeyCtxAlign() == 0U ||
		s_pLescCrypto->KeyCtxAlign() > CRYPTO_KEYCTX_ALIGN_MAX)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	LocalKeyReset();
	uint8_t pubBe[BLE_GAP_LESC_P256_PK_LEN] = {};
	CRYPTO_STATUS status = s_pLescCrypto->KeyGen(CRYPTO_CURVE_P256,
											 s_LescEcdhKeyCtx, pubBe);
	if (status == CRYPTO_STATUS_PENDING)
	{
		status = CRYPTO_STATUS_FAIL;
	}
	if (status == CRYPTO_STATUS_OK)
	{
		ByteOrderInvert(pubBe, s_LescPubKey.pk);
		s_bKeyPairGen = true;
	}
	else
	{
		s_pLescCrypto->KeyReset(s_LescEcdhKeyCtx);
		CryptoSecureWipe(&s_LescPubKey, sizeof(s_LescPubKey));
		LESC_TRACE("LESC keypair generate failed st=%d\r\n", (int)status);
	}
	CryptoSecureWipe(pubBe, sizeof(pubBe));
	return status;
}

bool BtLescKeyPairGen(void)
{
	return BtLescKeyPairGenStatus() == CRYPTO_STATUS_OK;
}

bool BtLescInit(void)
{
	CryptoSecureWipe(s_PeerKeys, sizeof(s_PeerKeys));
	LocalKeyReset();
	s_bRegenPending = false;
	if (s_pLescCrypto == nullptr)
	{
		LESC_TRACE("LESC crypto engine missing or unsupported\r\n");
		return false;
	}
	return BtLescKeyPairGenStatus() == CRYPTO_STATUS_OK;
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

ble_gap_lesc_p256_pk_t *BtLescPubKeyGetForLink(uint16_t ConnHdl)
{
	if (!s_bKeyPairGen || SlotAlloc(ConnHdl) < 0)
	{
		LESC_TRACE("LESC public key unavailable for link 0x%04x\r\n",
				   (unsigned)ConnHdl);
		return NULL;
	}
	return &s_LescPubKey;
}

void BtLescLinkRelease(uint16_t ConnHdl)
{
	SlotRelease(SlotFind(ConnHdl));
}

bool BtLescOobLocalGen(void)
{
	s_bOobLocalGen = false;
	CryptoSecureWipe(&s_OobLocal, sizeof(s_OobLocal));
	if (!s_bKeyPairGen)
	{
		return false;
	}
	if (sd_ble_gap_lesc_oob_data_get(BLE_CONN_HANDLE_INVALID,
		&s_LescPubKey, &s_OobLocal) != NRF_SUCCESS)
	{
		CryptoSecureWipe(&s_OobLocal, sizeof(s_OobLocal));
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

static CRYPTO_STATUS ReplyStatus(uint16_t ConnHdl, uint8_t SecStatus,
								 const ble_gap_lesc_dhkey_t *pDhKey)
{
	uint32_t result = BtLescDhKeyReply(ConnHdl, SecStatus, pDhKey);
	if (result == NRF_SUCCESS)
	{
		return CRYPTO_STATUS_OK;
	}
	return result == NRF_ERROR_BUSY ? CRYPTO_STATUS_BUSY : CRYPTO_STATUS_FAIL;
}

static CRYPTO_STATUS ComputeAndReply(BtLescPeerKey_t *pPeer)
{
	if (pPeer == nullptr || !pPeer->bAssigned || !pPeer->bRequested)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (pPeer->bForceFail || !s_bKeyPairGen || s_pLescCrypto == nullptr)
	{
		return ReplyStatus(pPeer->ConnHdl, BLE_GAP_SEC_STATUS_DHKEY_FAILURE, NULL);
	}
	if (memcmp(s_LescPubKey.pk, pPeer->Value,
		BLE_GAP_LESC_P256_PK_LEN) == 0)
	{
		LESC_TRACE("LESC peer used identical public key\r\n");
		return ReplyStatus(pPeer->ConnHdl, BLE_GAP_SEC_STATUS_DHKEY_FAILURE, NULL);
	}

	uint8_t peerBe[BLE_GAP_LESC_P256_PK_LEN] = {};
	uint8_t dhBe[BLE_GAP_LESC_DHKEY_LEN] = {};
	ByteOrderInvert(pPeer->Value, peerBe);
	CRYPTO_STATUS status = s_pLescCrypto->Agree(CRYPTO_CURVE_P256,
		s_LescEcdhKeyCtx, peerBe, dhBe, true);
	if (status == CRYPTO_STATUS_OK)
	{
		for (int i = 0; i < BLE_GAP_LESC_DHKEY_LEN; i++)
		{
			s_LescDhKey.key[i] = dhBe[BLE_GAP_LESC_DHKEY_LEN - 1 - i];
		}
		status = ReplyStatus(pPeer->ConnHdl, BLE_GAP_SEC_STATUS_SUCCESS,
			&s_LescDhKey);
	}
	else if (status == CRYPTO_STATUS_PENDING)
	{
		status = ReplyStatus(pPeer->ConnHdl,
			BLE_GAP_SEC_STATUS_DHKEY_FAILURE, NULL);
	}
	else if (status != CRYPTO_STATUS_BUSY)
	{
		LESC_TRACE("LESC ECDH failed st=%d\r\n", (int)status);
		status = ReplyStatus(pPeer->ConnHdl,
			BLE_GAP_SEC_STATUS_DHKEY_FAILURE, NULL);
	}

	CryptoSecureWipe(peerBe, sizeof(peerBe));
	CryptoSecureWipe(dhBe, sizeof(dhBe));
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
	return status;
}

bool BtLescRequestHandler(void)
{
	bool result = true;
	for (int i = 0; i < LinkCount(); i++)
	{
		if (!s_PeerKeys[i].bRequested)
		{
			continue;
		}
		CRYPTO_STATUS status = ComputeAndReply(&s_PeerKeys[i]);
		if (status == CRYPTO_STATUS_BUSY)
		{
			continue;
		}

		CryptoSecureWipe(s_PeerKeys[i].Value,
			sizeof(s_PeerKeys[i].Value));
		s_PeerKeys[i].bRequested = false;
		s_PeerKeys[i].bForceFail = false;
		if (status != CRYPTO_STATUS_OK)
		{
			SlotRelease(i);
			result = false;
		}
	}

	if (s_bRegenPending && !KeyInUse())
	{
		CRYPTO_STATUS status = BtLescKeyPairGenStatus();
		if (status == CRYPTO_STATUS_OK)
		{
			s_bRegenPending = false;
		}
		else if (status != CRYPTO_STATUS_BUSY)
		{
			LESC_TRACE("LESC keypair regenerate failed st=%d\r\n", (int)status);
			result = false;
		}
	}
	return result;
}

static void OnDhKeyRequest(uint16_t ConnHdl,
						   const ble_gap_evt_lesc_dhkey_request_t *pReq,
						   bool bForceFail)
{
	int index = SlotAlloc(ConnHdl);
	if (index < 0)
	{
		(void)BtLescDhKeyReply(ConnHdl, BLE_GAP_SEC_STATUS_DHKEY_FAILURE, NULL);
		return;
	}

	BtLescPeerKey_t *pPeer = &s_PeerKeys[index];
	pPeer->bRequested = true;
	pPeer->bForceFail = bForceFail || pReq == nullptr ||
		pReq->p_pk_peer == nullptr;
	CryptoSecureWipe(pPeer->Value, sizeof(pPeer->Value));
	if (!pPeer->bForceFail)
	{
		memcpy(pPeer->Value, pReq->p_pk_peer->pk,
			BLE_GAP_LESC_P256_PK_LEN);
	}
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
	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		// The security manager has just placed s_LescPubKey in this link's
		// keyset. Register the user before any other BLE event can complete a
		// different pairing and request key regeneration.
		if (s_bKeyPairGen)
		{
			(void)SlotAlloc(connHdl);
		}
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		BtLescLinkRelease(connHdl);
		CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
		break;

	case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		{
			bool forceFail = false;
			if (pEvt->evt.gap_evt.params.lesc_dhkey_request.oobd_req &&
				OobDataSet(connHdl) != NRF_SUCCESS)
			{
				LESC_TRACE("LESC oob_data_set failed\r\n");
				forceFail = true;
			}
			OnDhKeyRequest(connHdl,
				&pEvt->evt.gap_evt.params.lesc_dhkey_request, forceFail);
		}
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		BtLescLinkRelease(connHdl);
		break;

	default:
		break;
	}
}
