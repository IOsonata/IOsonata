/**-------------------------------------------------------------------------
@file	bt_lesc_sd.cpp

@brief	LE Secure Connections ECDH over the OO KeyAgreeEngine for the SoftDevice stacks.

Replaces the SDK nrf_ble_lesc module. The local key pair and the ECDH shared
secret come from the injected KeyAgreeEngine (BtLescSetCryptoEngine); the
SoftDevice keeps the SMP state machine and only asks the host for the DH key.

The ECDH is deferred out of the event callback: BtLescOnBleEvt records the peer
public key, and BtLescRequestHandler, called from the main loop, computes the DH
key and replies. On failure the reply reports a DHKey-failure status and no key,
so the peer's DHKey check fails cleanly rather than the link stalling.

The key pair serves several concurrent peers: each Agree keeps the private key
one ECDH. A fresh pair is generated on BLE_GAP_EVT_AUTH_STATUS, deferred to
BtLescRequestHandler like the ECDH itself. Regeneration invalidates the local
OOB data set; the application must generate and deliver a new OOB set before the
next OOB pairing. One consequence of the single use key: only one LESC pairing
can be in flight at a time; a second link pairing concurrently fails its DHKey
check and can retry once the first completes.

Byte order: the SoftDevice holds keys little-endian per coordinate; the engine
works big-endian. ByteOrderInvert reverses each 32 byte coordinate.

Two per-stack operations are supplied by the port: BtLescDhKeyReply (reply arity
differs between s132 and sdk-nrf-bm) and BtLescLinkCount.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "ble_gap.h"
#include "nrf_error.h"

#include "crypto/icrypto.h"
#include "bt_lesc.h"
#include "syslog.h"

#define BT_LESC_TRACE_ENABLE

#if defined(BT_LESC_TRACE_ENABLE)// && BT_LESC_TRACE_ENABLE
#define LESC_TRACE(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define LESC_TRACE(...)
#endif

#define LESC_COORD_SIZE		(BLE_GAP_LESC_P256_PK_LEN / 2)	//!< 32, one P-256 coordinate
#define LESC_MAX_LINK		8								//!< Peer-key table cap; BtLescLinkCount clamps to this

// Per-link peer public key awaiting a deferred DH computation.
typedef struct {
	uint8_t  Value[BLE_GAP_LESC_P256_PK_LEN];	// peer public key, little-endian
	uint16_t ConnHdl;
	bool     bRequested;
} BtLescPeerKey_t;

// Local key material, held little-endian as the SoftDevice wants it.
static ble_gap_lesc_p256_pk_t	s_LescPubKey __attribute__((aligned(4)));
static ble_gap_lesc_dhkey_t		s_LescDhKey __attribute__((aligned(4)));

static bool				s_bInternalError;
static bool				s_bKeyPairGen;
static bool				s_bRegenPending;
static BtLescPeerKey_t	s_PeerKeys[LESC_MAX_LINK];

static bool						s_bOobLocalGen;
static ble_gap_lesc_oob_data_t	s_OobLocal;
static BtLescOobPeerHandler_t	s_OobPeerHandler;

static KeyAgreeEngine	*s_pLescCrypto;

// One ephemeral key pair serves all concurrent pairings. The private scalar
// lives in this key context from BtLescKeyPairGen until it is regenerated; each
// per-peer Agree keeps the key (bKeepKey) so several peers derive against the
// same pair, and BtLescKeyPairGen replaces it. Sized for the largest engine key
// context (32 byte scalar plus a validity flag, aligned).
static uint8_t			s_LescEcdhKeyCtx[64];

// Reverse each 32 byte coordinate of a secp256r1 key between big and little
// endian. Used both directions.
static void ByteOrderInvert(const uint8_t *pIn, uint8_t *pOut)
{
	for (int i = 0; i < LESC_COORD_SIZE; i++)
	{
		pOut[i]                    = pIn[LESC_COORD_SIZE - 1 - i];
		pOut[LESC_COORD_SIZE + i]  = pIn[2 * LESC_COORD_SIZE - 1 - i];
	}
}

static int LinkCount(void)
{
	int n = BtLescLinkCount();

	if (n < 1)
	{
		n = 1;
	}

	if (n > LESC_MAX_LINK)
	{
		n = LESC_MAX_LINK;
	}

	return n;
}

// Table slot for a connection handle: the slot already holding it, else a free
// slot. Returns -1 when full.
static int SlotFind(uint16_t ConnHdl)
{
	int n = LinkCount();
	int free = -1;

	for (int i = 0; i < n; i++)
	{
		if (s_PeerKeys[i].bRequested && s_PeerKeys[i].ConnHdl == ConnHdl)
		{
			return i;
		}

		if (free < 0 && !s_PeerKeys[i].bRequested)
		{
			free = i;
		}
	}

	return free;
}

void BtLescSetCryptoEngine(KeyAgreeEngine *pEcdh)
{
	s_pLescCrypto = pEcdh;
}

bool BtLescKeyPairGen(void)
{
	uint8_t pubBe[BLE_GAP_LESC_P256_PK_LEN];		// big-endian X||Y from the engine

	// Do not regenerate while a DH computation is pending on any link.
	for (int i = 0; i < LinkCount(); i++)
	{
		if (s_PeerKeys[i].bRequested)
		{
			return false;
		}
	}

	s_bKeyPairGen = false;
	s_bOobLocalGen = false;

	// Fresh key pair. The private key stays in the module key context for the
	// matching per-peer Agree calls and never crosses the engine interface.
	if (s_pLescCrypto == nullptr ||
		s_pLescCrypto->KeyGen(CRYPTO_CURVE_P256, s_LescEcdhKeyCtx, pubBe)
			!= CRYPTO_STATUS_OK)
	{
		CryptoSecureWipe(pubBe, sizeof(pubBe));
		LESC_TRACE("LESC keypair generate failed\r\n");
		return false;
	}

	// The SoftDevice wants the public key little-endian per coordinate.
	ByteOrderInvert(pubBe, s_LescPubKey.pk);
	CryptoSecureWipe(pubBe, sizeof(pubBe));

	s_bKeyPairGen = true;

	return true;
}

bool BtLescInit(void)
{
	memset(s_PeerKeys, 0, sizeof(s_PeerKeys));
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));

	if (s_pLescCrypto == nullptr)
	{
		LESC_TRACE("LESC crypto engine missing or lacks ECDH P-256\r\n");
		return false;
	}

	s_bInternalError = false;
	s_bKeyPairGen = false;
	s_bRegenPending = false;

	return BtLescKeyPairGen();
}

ble_gap_lesc_p256_pk_t * BtLescPubKeyGet(void)
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

	if (sd_ble_gap_lesc_oob_data_get(BLE_CONN_HANDLE_INVALID, &s_LescPubKey,
									 &s_OobLocal) != NRF_SUCCESS)
	{
		return false;
	}

	s_bOobLocalGen = true;

	return true;
}

ble_gap_lesc_oob_data_t * BtLescOobLocalGet(void)
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

// Compute the DH key for one pending peer key and answer its DHKey request.
static bool ComputeAndReply(BtLescPeerKey_t *pPeer)
{
	uint8_t peerBe[BLE_GAP_LESC_P256_PK_LEN];	// peer public key, big-endian
	uint8_t dhBe[BLE_GAP_LESC_DHKEY_LEN];		// DH key, big-endian from the engine
	const ble_gap_lesc_dhkey_t *pDh = NULL;
	uint8_t secStatus = BLE_GAP_SEC_STATUS_DHKEY_FAILURE;

	if (!s_bKeyPairGen)
	{
		return false;
	}

	memset(dhBe, 0, sizeof(dhBe));
	CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));

	// Reject a peer presenting our own public key (reflection). Compare the X
	// coordinate only; both little-endian here.
	if (memcmp(s_LescPubKey.pk, pPeer->Value, LESC_COORD_SIZE) == 0)
	{
		LESC_TRACE("LESC peer used identical public key\r\n");
	}
	else
	{
		// Peer key arrives little-endian; the engine wants big-endian.
		ByteOrderInvert(pPeer->Value, peerBe);

		// Keep the key pair: several concurrent peers derive against the same
		// ephemeral pair, and BtLescKeyPairGen replaces it. bKeepKey stops the
		// engine wiping the private scalar after this Agree.
		if (s_pLescCrypto->Agree(CRYPTO_CURVE_P256, s_LescEcdhKeyCtx,
								 peerBe, dhBe, true) == CRYPTO_STATUS_OK)
		{
			// The engine returns the DH key big-endian; BLE wants it little-endian.
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

	LESC_TRACE("LESC dhkey reply status=0x%x hdl=%d\r\n", secStatus, pPeer->ConnHdl);

	const uint32_t result = BtLescDhKeyReply(pPeer->ConnHdl, secStatus, pDh);
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

			s_PeerKeys[i].bRequested = false;

			if (!ok)
			{
				return false;
			}
		}
	}

	if (s_bRegenPending)
	{
		// A DHKey request that arrived during the loop above still needs the
		// current key pair; regenerate on the next pass instead.
		for (int i = 0; i < LinkCount(); i++)
		{
			if (s_PeerKeys[i].bRequested)
			{
				return true;
			}
		}

		if (!BtLescKeyPairGen())
		{
			// Nothing is pending, so this is an engine failure, not the
			// regenerate-while-busy guard.
			LESC_TRACE("LESC keypair regenerate failed\r\n");
			s_bInternalError = true;
			return false;
		}

		s_bRegenPending = false;
	}

	return true;
}

// Record the peer public key from a DHKey request for deferred computation.
static void OnDhKeyRequest(uint16_t ConnHdl,
						   const ble_gap_evt_lesc_dhkey_request_t *pReq)
{
	int idx = SlotFind(ConnHdl);

	if (idx < 0)
	{
		s_bInternalError = true;
		return;
	}

	memcpy(s_PeerKeys[idx].Value, pReq->p_pk_peer->pk, BLE_GAP_LESC_P256_PK_LEN);
	s_PeerKeys[idx].ConnHdl = ConnHdl;
	s_PeerKeys[idx].bRequested = true;
}

static uint32_t OobDataSet(uint16_t ConnHdl)
{
	ble_gap_lesc_oob_data_t *pOwn = s_bOobLocalGen ? &s_OobLocal : NULL;
	ble_gap_lesc_oob_data_t *pPeer = s_OobPeerHandler ? s_OobPeerHandler(ConnHdl) : NULL;

	return sd_ble_gap_lesc_oob_data_set(ConnHdl, pOwn, pPeer);
}

void BtLescOnBleEvt(const ble_evt_t * pEvt)
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
			int idx = SlotFind(connHdl);

			if (idx >= 0)
			{
				s_PeerKeys[idx].bRequested = false;
			}
			CryptoSecureWipe(&s_LescDhKey, sizeof(s_LescDhKey));
		}
		break;

	case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		if (pEvt->evt.gap_evt.params.lesc_dhkey_request.oobd_req)
		{
			if (OobDataSet(connHdl) != NRF_SUCCESS)
			{
				LESC_TRACE("LESC oob_data_set failed\r\n");
				s_bInternalError = true;
			}
		}

		OnDhKeyRequest(connHdl, &pEvt->evt.gap_evt.params.lesc_dhkey_request);
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		// Pairing ended, with or without success. The engine wiped the private
		// key after the ECDH, so a fresh pair is needed for the next pairing.
		// Defer to BtLescRequestHandler; key generation must not run in the
		// stack callback context.
		s_bRegenPending = true;
		break;

	default:
		break;
	}
}
