/**-------------------------------------------------------------------------
@file	bt_smp.cpp

@brief	Bluetooth Security Manager Protocol (SMP)

Generic implementation of Bluetooth Security Manager Protocol. Responder
(peripheral) role is implemented end to end for Just Works pairing over both
the legacy and LE Secure Connections paths, plus the controller Long Term
Key request reply that starts link encryption.

L2CAP channel 6

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#include <stddef.h>
#include <string.h>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gatt.h"
// SMP uses the OO crypto engine facets (KeyAgreeEngine, CipherEngine,
// RngEngine) and the shared CRYPTO_STATUS, all from icrypto.h via bt_smp.h.
// The legacy crypto.h is intentionally not included here.

// SMP handshake trace. Define BT_SMP_TRACE_ENABLE to 1 to print every SMP PDU
// in/out and the link state over SysLog. Defaults off for release and library
// builds.
#ifndef BT_SMP_TRACE_ENABLE
#define BT_SMP_TRACE_ENABLE 0
#endif

// DHKey byte-order interop fallback. When enabled, a failed Ea verification
// retries f5 with the raw DHKey order before failing, to tolerate a crypto
// provider that returns the shared secret in the opposite byte order. Leave
// off for release: it can mask a provider byte-order bug. Enable only for
// provider bring-up or interop debugging.
#ifndef BT_SMP_DHKEY_ORDER_FALLBACK
#define BT_SMP_DHKEY_ORDER_FALLBACK 0
#endif

#if BT_SMP_TRACE_ENABLE
#include "syslog.h"
#define SMP_TRACE(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)

static const char *SmpCodeName(uint8_t c);

#define SMP_TRACE_PDU(dir, code, state) \
		SMP_TRACE("SMP " dir " %s state=%d\r\n", SmpCodeName(code), state)
#else
#define SMP_TRACE(...)
#define SMP_TRACE_PDU(dir, code, state)
#endif

#ifndef BT_SMP_MAX_LINK
#define BT_SMP_MAX_LINK		4
#endif

#ifndef BT_SMP_CRYPTO_BUSY_RETRIES
#define BT_SMP_CRYPTO_BUSY_RETRIES	32U
#endif

#ifndef BT_SMP_LOCAL_IOCAPS
#define BT_SMP_LOCAL_IOCAPS		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#endif

#ifndef BT_SMP_LOCAL_AUTHREQ
#define BT_SMP_LOCAL_AUTHREQ	(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC)
#endif

typedef struct __Bt_Smp_Link {
	uint16_t    ConnHdl;
	uint32_t    Generation;
	BtSmpCtx_t  Ctx;
	BtSmpKeys_t Keys;
} BtSmpLink_t;

static BtSmpLink_t s_SmpLink[BT_SMP_MAX_LINK];
static uint32_t s_SmpGeneration;

//-----------------------------------------------------------------------------
// Crypto primitives
//-----------------------------------------------------------------------------

static BtHciDevice_t *s_pSmpActiveDev = nullptr;

// Runtime association configuration. Defaults mirror the compile-time
// BT_SMP_LOCAL_* values, i.e. Just Works, until BtSmpAuthConfig overrides them.
static uint8_t s_SmpIoCaps  = BT_SMP_LOCAL_IOCAPS;
static uint8_t s_SmpAuthReq = BT_SMP_LOCAL_AUTHREQ;

// Pending LE Secure Connections OOB data. The local set holds the key pair
// commitment handed to the peer out of band; the peer set holds the material
// received from the peer. Both are copied into the link context when a
// pairing selects the OOB model.
static struct {
	bool bLocalValid;
	bool bPeerValid;
	bool bReserved;
	uint16_t ConnHdl;
	uint32_t Generation;
	uint8_t LocalRand[16];
	uint8_t LocalPubKey[64];
	uint8_t EcdhKeyCtx[CRYPTO_KEYCTX_MAX] CRYPTO_ALIGNED(CRYPTO_KEYCTX_ALIGN_MAX);	//!< OOB provider key context
	uint8_t PeerRand[16];
	uint8_t PeerConfirm[16];
} s_SmpOob = {};

typedef enum __Bt_Smp_Crypto_Op {
	BT_SMP_CRYPTO_OP_NONE = 0,
	BT_SMP_CRYPTO_OP_PUBKEY,
	BT_SMP_CRYPTO_OP_DHKEY,
} BT_SMP_CRYPTO_OP;

typedef struct __Bt_Smp_Crypto_Pending {
	BT_SMP_CRYPTO_OP Op;
	uint16_t ConnHdl;
	uint32_t Generation;
	BtHciDevice_t *pDev;
	bool bRetryBusy;
} BtSmpCryptoPending_t;

static BtSmpCryptoPending_t s_SmpCryptoPending = {};

// Map the local and peer IO capabilities to an SC association model. Core
// spec Vol 3 Part H, 2.3.5.1, Table 2.8 (LE Secure Connections). Rows index
// the initiator IO capability, columns the responder, both using the
// BT_SMP_IOCAPS_* values (DisplayOnly 0 .. KeyboardDisplay 4). Cell values are
// BT_SMP_MODEL_JUST_WORKS / NUMERIC_COMPARISON / PASSKEY_ENTRY. The passkey
// display/input direction is derived from role and IO capability where it is
// used, not stored here.
static const uint8_t s_SmpModelMap[5][5] = {
	//              resp DO  DYN  KO  NIO  KD
	/* init DO  */ {  0,   0,   2,   0,   2 },
	/* init DYN */ {  0,   1,   2,   0,   1 },
	/* init KO  */ {  2,   2,   2,   0,   2 },
	/* init NIO */ {  0,   0,   0,   0,   0 },
	/* init KD  */ {  2,   1,   2,   0,   1 },
};

// Crypto engine slots, bound by BtSmpInit before first use. The engines are on
// the OO tree: a KeyAgreeEngine for P-256 ECDH (CryptoUecc or Ba414ep) and a
// CipherEngine for AES-128 (CryptoSoftAes, CryptoMaster, or the controller AES).
static KeyAgreeEngine *s_pCryptoEcdh = nullptr;
static RngEngine *s_pCryptoRng = nullptr;
static CipherEngine   *s_pCryptoAes  = nullptr;

// Set true when an AES-128 block operation fails. A failed block leaves a zeroed
// result, so any confirm, key or check derived from it is unusable and must not
// drive a compare, a stored key or link encryption. Cleared at each entry point
// that starts a fresh crypto sequence, checked where a result is consumed. This
// is an internal failure, not a peer authentication attempt.
static bool s_SmpAesFault = false;

// Destroy the ECDH private key held in a key context. KeyReset reaches a
// provider-side handle (an accelerator slot or an opaque key) that a raw wipe
// of the context bytes cannot; the caller's following wipe covers the plain
// bytes.
static void SmpEcdhCtxReset(uint8_t *pCtx)
{
	if (s_pCryptoEcdh != nullptr)
	{
		s_pCryptoEcdh->KeyReset(pCtx);
	}
}

static int SmpCryptoP256KeyGen(BtSmpLink_t *pLink, uint8_t pPubKey[64]);
static int SmpCryptoEcdh(BtSmpLink_t *pLink,
						 const uint8_t pPeerPubKey[64], uint8_t pDhKey[32]);
static void SmpSendFailed(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint8_t Reason);

#if BT_SMP_TRACE_ENABLE
static const char *SmpCodeName(uint8_t c)
{
	switch (c)
	{
		case BT_SMP_CODE_PAIRING_REQ:			return "PairingReq";
		case BT_SMP_CODE_PAIRING_RSP:			return "PairingRsp";
		case BT_SMP_CODE_PAIRING_CONFIRM:		return "Confirm";
		case BT_SMP_CODE_PAIRING_RANDOM:		return "Random";
		case BT_SMP_CODE_PAIRING_FAILED:		return "Failed";
		case BT_SMP_CODE_PAIRING_ENCRYP_INFO:	return "EncInfo";
		case BT_SMP_CODE_PAIRING_CENTRAL_ID:	return "CentralId";
		case BT_SMP_CODE_PAIRING_ID_INFO:		return "IdInfo";
		case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:	return "IdAddrInfo";
		case BT_SMP_CODE_PAIRING_SIGNING_INFO:	return "SigningInfo";
		case BT_SMP_CODE_PAIRING_SECURITY_REQ:	return "SecurityReq";
		case BT_SMP_CODE_PAIRING_PUBLIC_KEY:	return "PublicKey";
		case BT_SMP_CODE_PAIRING_DHKEY_CHECK:	return "DhKeyCheck";
		case BT_SMP_CODE_PAIRING_KEYPRESS_NOTIF:return "Keypress";
		default:								return "?";
	}
}
#endif	// BT_SMP_TRACE_ENABLE

//-----------------------------------------------------------------------------
// Per-link context lookup
//-----------------------------------------------------------------------------

static BtSmpLink_t *SmpLinkFind(uint16_t ConnHdl)
{
	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		if (s_SmpLink[i].ConnHdl == ConnHdl)
		{
			return &s_SmpLink[i];
		}
	}
	return nullptr;
}

static void SmpOobClear(void)
{
	SmpEcdhCtxReset(s_SmpOob.EcdhKeyCtx);
	CryptoSecureWipe(&s_SmpOob, sizeof(s_SmpOob));
}

static bool SmpOobReservedFor(const BtSmpLink_t *pLink)
{
	return pLink != nullptr && s_SmpOob.bReserved &&
		s_SmpOob.ConnHdl == pLink->ConnHdl &&
		s_SmpOob.Generation == pLink->Generation;
}

static bool SmpOobAvailable(const BtSmpLink_t *pLink)
{
	return s_SmpOob.bReserved == false || SmpOobReservedFor(pLink);
}

static bool SmpOobLocalReady(const BtSmpLink_t *pLink)
{
	return s_SmpOob.bLocalValid && SmpOobAvailable(pLink);
}

static bool SmpOobPeerReady(const BtSmpLink_t *pLink)
{
	return s_SmpOob.bPeerValid && SmpOobAvailable(pLink);
}

static bool SmpOobReserve(BtSmpLink_t *pLink)
{
	if (pLink == nullptr || SmpOobAvailable(pLink) == false)
	{
		return false;
	}
	s_SmpOob.bReserved = true;
	s_SmpOob.ConnHdl = pLink->ConnHdl;
	s_SmpOob.Generation = pLink->Generation;
	return true;
}

static void SmpOobRelease(BtSmpLink_t *pLink)
{
	if (SmpOobReservedFor(pLink))
	{
		SmpOobClear();
	}
}

static void SmpCryptoPendingClear(void)
{
	memset(&s_SmpCryptoPending, 0, sizeof(s_SmpCryptoPending));
}

// Abort a pairing attempt on one link: drop a pending crypto operation that
// belongs to it, destroy the ECDH private key through the provider, wipe the
// whole transient pairing context, and return the link to idle. The lock
// flag survives so an abort inside a lockout keeps the link locked.
static void SmpAbortPairing(BtSmpLink_t *pLink)
{
	SmpOobRelease(pLink);
	if (s_SmpCryptoPending.Op != BT_SMP_CRYPTO_OP_NONE &&
		s_SmpCryptoPending.ConnHdl == pLink->ConnHdl &&
		s_SmpCryptoPending.Generation == pLink->Generation)
	{
		SmpCryptoPendingClear();
	}
	SmpEcdhCtxReset(pLink->Ctx.EcdhKeyCtx);
	// The lock flag and the H5 repeated-attempts counter survive the wipe:
	// a locally detected failure (for example a wrong confirm from a peer
	// brute-forcing a passkey) must keep counting toward the lockout.
	bool locked = pLink->Ctx.bLocked;
	uint8_t failCount = pLink->Ctx.FailCount;
	CryptoSecureWipe(&pLink->Ctx, sizeof(pLink->Ctx));
	pLink->Ctx.State = BT_SMP_STATE_IDLE;
	pLink->Ctx.bLocked = locked;
	pLink->Ctx.FailCount = failCount;
}

// H5 repeated attempts (Core Vol 3 Part H 2.3.6): an authentication failure
// caused by the peer (wrong confirm value, wrong DHKey check, wrong passkey
// round, invalid OOB confirmation) counts toward the lockout and locks the
// link at the threshold. Internal crypto or resource failures never count:
// they are not attack attempts.
static void SmpAuthFailCount(BtSmpLink_t *pLink)
{
	pLink->Ctx.FailCount++;
	if (pLink->Ctx.FailCount >= BT_SMP_MAX_PAIR_ATTEMPTS)
	{
		pLink->Ctx.bLocked = true;
	}
}

static bool SmpCryptoPendingBegin(BtSmpLink_t *pLink, BtHciDevice_t *pDev,
								  BT_SMP_CRYPTO_OP Op)
{
	if (pLink == nullptr || Op == BT_SMP_CRYPTO_OP_NONE ||
		s_SmpCryptoPending.Op != BT_SMP_CRYPTO_OP_NONE)
	{
		return false;
	}
	s_SmpCryptoPending.Op = Op;
	s_SmpCryptoPending.ConnHdl = pLink->ConnHdl;
	s_SmpCryptoPending.Generation = pLink->Generation;
	s_SmpCryptoPending.pDev = pDev;
	s_SmpCryptoPending.bRetryBusy = false;
	return true;
}

static BtSmpLink_t *SmpCryptoPendingFind(BT_SMP_CRYPTO_OP Op)
{
	if (s_SmpCryptoPending.Op != Op)
	{
		return nullptr;
	}
	BtSmpLink_t *pLink = SmpLinkFind(s_SmpCryptoPending.ConnHdl);
	return pLink != nullptr && pLink->Generation == s_SmpCryptoPending.Generation ?
		pLink : nullptr;
}

static BtSmpLink_t *SmpCryptoPendingTake(BT_SMP_CRYPTO_OP Op)
{
	if (s_SmpCryptoPending.Op != Op)
	{
		return nullptr;
	}
	BtSmpLink_t *pLink = SmpCryptoPendingFind(Op);
	SmpCryptoPendingClear();
	return pLink;
}

static BtSmpLink_t *SmpLinkAlloc(uint16_t ConnHdl)
{
	BtSmpLink_t *p = SmpLinkFind(ConnHdl);
	if (p != nullptr)
	{
		return p;
	}

	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		if (s_SmpLink[i].ConnHdl == BT_CONN_HDL_INVALID)
		{
			memset(&s_SmpLink[i], 0, sizeof(BtSmpLink_t));
			s_SmpGeneration++;
			if (s_SmpGeneration == 0) { s_SmpGeneration++; }
			s_SmpLink[i].ConnHdl = ConnHdl;
			s_SmpLink[i].Generation = s_SmpGeneration;
			s_SmpLink[i].Ctx.State = BT_SMP_STATE_IDLE;
			return &s_SmpLink[i];
		}
	}

	return nullptr;
}

static void SmpLinkFree(uint16_t ConnHdl)
{
	BtSmpLink_t *p = SmpLinkFind(ConnHdl);
	if (p != nullptr)
	{
		SmpOobRelease(p);
		SmpEcdhCtxReset(p->Ctx.EcdhKeyCtx);
		CryptoSecureWipe(p, sizeof(BtSmpLink_t));
		p->ConnHdl = BT_CONN_HDL_INVALID;
	}
}

// Wipe pairing/key material after a failed attempt but keep the slot bound to
// the connection along with the repeated-attempts counter and lock flag, so
// FailCount accumulates across attempts on the same connection (the record is
// only fully freed by BtSmpDisconnected). Preserves the security property of
// SmpLinkFree - no key material survives - without losing the counter.
static void SmpLinkResetKeepCount(BtSmpLink_t *pLink)
{
	uint16_t hdl       = pLink->ConnHdl;
	uint32_t generation = pLink->Generation;
	uint8_t  fc        = pLink->Ctx.FailCount;
	bool     locked    = pLink->Ctx.bLocked;

	SmpOobRelease(pLink);
	SmpEcdhCtxReset(pLink->Ctx.EcdhKeyCtx);
	CryptoSecureWipe(&pLink->Ctx, sizeof(pLink->Ctx));

	pLink->ConnHdl       = hdl;
	pLink->Generation    = generation;
	pLink->Ctx.FailCount = fc;
	pLink->Ctx.bLocked   = locked;
	pLink->Ctx.State     = BT_SMP_STATE_IDLE;
}

//-----------------------------------------------------------------------------
// Pairing timeout (Core Vol 3 Part H 3.4)
//-----------------------------------------------------------------------------

// Millisecond tick for the pairing timeout. Weak default returns 0 so the
// timeout is inert on ports without a clock; an app with a running millisecond
// counter overrides this to enable it. Mirrors the BtGattMsTick pattern.
__attribute__((weak)) uint32_t BtSmpMsTick(void)
{
	return 0;
}

// A pairing procedure is in progress (before encryption): the states in which
// the SMP timeout applies. IDLE, KEYDIST (post-encryption key exchange) and
// DONE are excluded - the link is either idle or already secure.
static inline bool SmpPairingActive(BtSmpState_t State)
{
	return State != BT_SMP_STATE_IDLE &&
		   State != BT_SMP_STATE_KEYDIST &&
		   State != BT_SMP_STATE_DONE;
}

// True once the pairing on pLink has exceeded BT_SMP_TIMEOUT_MS. The unsigned
// subtraction handles counter rollover during the wait. The timer is anchored
// to the pairing start, so the elapsed time cannot be extended by the peer
// sending further PDUs.
static inline bool SmpPairingTimedOut(const BtSmpLink_t *pLink)
{
	return SmpPairingActive(pLink->Ctx.State) &&
		   (uint32_t)(BtSmpMsTick() - pLink->Ctx.TmrStart) >= BT_SMP_TIMEOUT_MS;
}

// Fail an overdue pairing and lock the link: send Pairing Failed, count the
// attempt, and mark the link so no further SMP is accepted until it
// disconnects (BtSmpDisconnected frees the record and clears the lock).
static void SmpFailAndLock(BtHciDevice_t * const pDev, uint16_t ConnHdl,
						   BtSmpLink_t *pLink, uint8_t Reason)
{
	SmpSendFailed(pDev, ConnHdl, Reason);
	SmpAbortPairing(pLink);
	pLink->Ctx.bLocked = true;
	BtSmpPairingComplete(ConnHdl, false, nullptr);
}

// Terminate the current attempt on an out-of-sequence PDU. Unlike a peer
// authentication mismatch this does not touch the repeated-attempts counter,
// and unlike an internal fault it does not lock the link: a protocol ordering
// error is neither a wrong passkey guess nor a hardware failure. The attempt
// state is wiped so no half-built context (for example an uncomputed DHKey)
// survives into a later PDU.
static void SmpAbortOffPhase(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							 BtSmpLink_t *pLink)
{
	SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
	SmpAbortPairing(pLink);
	BtSmpPairingComplete(ConnHdl, false, nullptr);
}

//-----------------------------------------------------------------------------
// Outbound packet helpers
//-----------------------------------------------------------------------------

// Weak fallback so builds that exclude the generic HCI host (vendor-owned stacks
// run their own SMP/GATT and never reach the native ACL send) still link. The
// strong BtHciSendAcl in bt_hci_host.cpp does the credit-gated ACL fragmentation
// and replaces this in HCI-host builds.
__attribute__((weak)) uint32_t BtHciSendAcl(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pAcl)
{
	(void)pDev;
	(void)pAcl;
	return 0;
}

static void SmpSend(BtHciDevice_t * const pDev, uint16_t ConnHdl,
					const void *pData, size_t Len)
{
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2 = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2->Hdr.Cid = BT_L2CAP_CID_SEC_MNGR;
	l2->Hdr.Len = (uint16_t)Len;
	memcpy(&l2->Smp, pData, Len);

	acl->Hdr.Len = (uint16_t)(Len + sizeof(BtL2CapHdr_t));
	BtHciSendAcl(pDev, acl);

	SMP_TRACE_PDU("TX", ((const uint8_t*)pData)[0],
				  SmpLinkFind(ConnHdl) ? (int)SmpLinkFind(ConnHdl)->Ctx.State : -1);
}

static void SmpSendFailed(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint8_t Reason)
{
	SmpOobRelease(SmpLinkFind(ConnHdl));
	BtSmpPairingFailed_t f;
	f.Code = BT_SMP_CODE_PAIRING_FAILED;
	f.Reason = Reason;
	SmpSend(pDev, ConnHdl, &f, sizeof(f));
}

static void SmpSendHciCmd(BtHciDevice_t * const pDev, uint16_t OpCode,
						  const void *pParam, uint8_t ParamLen)
{
	// Route the SMP command through the controller command op. On the SDC this
	// dispatches to the typed wrapper; on a raw HCI controller it is framed and
	// sent as a command. Sending it down the data path does not reach a typed
	// command controller.
	BtHciCommand(pDev, OpCode, pParam, ParamLen, NULL, 0);
}

static inline void SmpAes(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	BtSmpCryptoAes128(s_pSmpActiveDev, Key, In, Out);
}

static void SmpLeftShiftOne(const uint8_t In[16], uint8_t Out[16])
{
	uint8_t carry = 0;

	for (int i = 15; i >= 0; i--)
	{
		uint8_t b = In[i];
		Out[i] = (uint8_t)((b << 1) | carry);
		carry = (b & 0x80) ? 1 : 0;
	}
}

// AES-CMAC (RFC 4493).
static void SmpAesCmac(const uint8_t Key[16], const uint8_t *pMsg, size_t Len,
					   uint8_t Mac[16])
{
	static const uint8_t Zero[16] = {0};
	uint8_t l[16];
	uint8_t k1[16];
	uint8_t k2[16];

	SmpAes(Key, Zero, l);

	SmpLeftShiftOne(l, k1);
	if (l[0] & 0x80)
	{
		k1[15] ^= 0x87;
	}

	SmpLeftShiftOne(k1, k2);
	if (k1[0] & 0x80)
	{
		k2[15] ^= 0x87;
	}

	size_t n = (Len + 15) / 16;
	bool lastComplete;

	if (n == 0)
	{
		n = 1;
		lastComplete = false;
	}
	else
	{
		lastComplete = (Len % 16) == 0;
	}

	uint8_t x[16] = {0};
	uint8_t y[16];

	for (size_t i = 0; i < n - 1; i++)
	{
		for (int j = 0; j < 16; j++)
		{
			y[j] = x[j] ^ pMsg[i * 16 + j];
		}
		SmpAes(Key, y, x);
	}

	uint8_t last[16];
	size_t rem = Len - (n - 1) * 16;

	if (lastComplete)
	{
		for (int j = 0; j < 16; j++)
		{
			last[j] = pMsg[(n - 1) * 16 + j] ^ k1[j];
		}
	}
	else
	{
		for (size_t j = 0; j < 16; j++)
		{
			uint8_t mb = 0;
			if (j < rem)
			{
				mb = pMsg[(n - 1) * 16 + j];
			}
			else if (j == rem)
			{
				mb = 0x80;
			}

			last[j] = mb ^ k2[j];
		}
	}

	for (int j = 0; j < 16; j++)
	{
		y[j] = x[j] ^ last[j];
	}
	SmpAes(Key, y, Mac);
}

// c1(k, r, preq, pres, iat, ia, rat, ra)
static bool SmpC1(const uint8_t k[16], const uint8_t r[16],
				  const uint8_t preq[7], const uint8_t pres[7],
				  uint8_t iat, const uint8_t ia[6],
				  uint8_t rat, const uint8_t ra[6], uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t p1[16];
	uint8_t p2[16];
	uint8_t tmp[16];

	p1[0] = iat;
	p1[1] = rat;
	memcpy(&p1[2], preq, 7);
	memcpy(&p1[9], pres, 7);

	memset(p2, 0, 16);
	memcpy(&p2[0], ra, 6);
	memcpy(&p2[6], ia, 6);

	for (int i = 0; i < 16; i++)
	{
		tmp[i] = r[i] ^ p1[i];
	}
	SmpAes(k, tmp, tmp);

	for (int i = 0; i < 16; i++)
	{
		tmp[i] ^= p2[i];
	}
	SmpAes(k, tmp, out);
	return !s_SmpAesFault;
}

// s1(k, r1, r2)
static bool SmpS1(const uint8_t k[16], const uint8_t r1[16],
				  const uint8_t r2[16], uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t r[16];

	memcpy(&r[0], &r2[0], 8);
	memcpy(&r[8], &r1[0], 8);
	SmpAes(k, r, out);
	return !s_SmpAesFault;
}

static void SmpReverse16(const uint8_t in[16], uint8_t out[16])
{
	for (int i = 0; i < 16; i++)
	{
		out[i] = in[15 - i];
	}
}

static void SmpReverse32(const uint8_t in[32], uint8_t out[32])
{
	for (int i = 0; i < 32; i++)
	{
		out[i] = in[31 - i];
	}
}

static void SmpReverse6(const uint8_t in[6], uint8_t out[6])
{
	for (int i = 0; i < 6; i++)
	{
		out[i] = in[5 - i];
	}
}

// f4(U, V, X, Z) -> confirm
//
// U/V/X are passed in SMP byte order. The CMAC core works on the big-endian
// form used by the Bluetooth crypto definition, then the 128-bit result is
// converted back to SMP PDU order. This matches the byte-order handling used
// by Zephyr's bt_crypto_f4().
static bool SmpF4(const uint8_t u[32], const uint8_t v[32],
				  const uint8_t x[16], uint8_t z, uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t m[65];
	uint8_t xs[16];
	uint8_t mac[16];

	for (int i = 0; i < 32; i++)
	{
		m[i] = u[31 - i];
		m[32 + i] = v[31 - i];
	}
	m[64] = z;

	SmpReverse16(x, xs);
	SmpAesCmac(xs, m, sizeof(m), mac);
	SmpReverse16(mac, out);
	return !s_SmpAesFault;
}

// P-256 providers store coordinates big-endian for ECDH. SMP f4 must use the
// 32-octet X coordinates in the little-endian order carried by the Pairing
// Public Key PDU.
static void SmpP256CoordBeToSmpLe(const uint8_t be[32], uint8_t le[32])
{
	for (int i = 0; i < 32; i++)
	{
		le[i] = be[31 - i];
	}
}

// f5 -> MacKey || LTK.
//
// Inputs are in SMP byte order. The CMAC core uses the big-endian crypto
// form, then the 128-bit outputs are converted back to SMP order.
static bool SmpF5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16],
				  uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
				  uint8_t mackey[16], uint8_t ltk[16])
{
	s_SmpAesFault = false;
	static const uint8_t salt[16] = {
		0x6C,0x88,0x83,0x91,0xAA,0xF5,0xA5,0x38,
		0x60,0x37,0x0B,0xDB,0x5A,0x60,0x83,0xBE
	};

	uint8_t ws[32];
	uint8_t t[16];
	SmpReverse32(w, ws);
	SmpAesCmac(salt, ws, 32, t);

	uint8_t m[53] = {
		0x00,
		0x62, 0x74, 0x6C, 0x65,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0x01, 0x00
	};

	SmpReverse16(n1, &m[5]);
	SmpReverse16(n2, &m[21]);
	m[37] = a1t;
	SmpReverse6(a1, &m[38]);
	m[44] = a2t;
	SmpReverse6(a2, &m[45]);

	uint8_t tmp[16];

	m[0] = 0;
	SmpAesCmac(t, m, sizeof(m), tmp);
	SmpReverse16(tmp, mackey);

	m[0] = 1;
	SmpAesCmac(t, m, sizeof(m), tmp);
	SmpReverse16(tmp, ltk);
	return !s_SmpAesFault;
}

// f6 -> DHKey check value.
//
// Inputs and output are in SMP byte order.
static bool SmpF6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16],
				  const uint8_t r[16], const uint8_t iocap[3],
				  uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
				  uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t ws[16];
	uint8_t m[65];
	uint8_t mac[16];

	SmpReverse16(w, ws);
	SmpReverse16(n1, &m[0]);
	SmpReverse16(n2, &m[16]);
	SmpReverse16(r, &m[32]);

	m[48] = iocap[2];
	m[49] = iocap[1];
	m[50] = iocap[0];

	m[51] = a1t;
	SmpReverse6(a1, &m[52]);
	m[58] = a2t;
	SmpReverse6(a2, &m[59]);

	SmpAesCmac(ws, m, sizeof(m), mac);
	SmpReverse16(mac, out);
	return !s_SmpAesFault;
}

//-----------------------------------------------------------------------------
// Pairing feature negotiation
//-----------------------------------------------------------------------------

// Select the association model for this pairing. OOB takes precedence when
// present. Without a MITM requirement from either side the result is Just
// Works. Otherwise the IO capability table decides; a capability value outside
// the table range falls back to Just Works rather than indexing out of bounds.
static uint8_t SmpSelectModel(uint8_t InitIo, uint8_t RespIo, bool Mitm, bool Oob)
{
	if (Oob)
	{
		return BT_SMP_MODEL_OOB;
	}
	if (!Mitm)
	{
		return BT_SMP_MODEL_JUST_WORKS;
	}
	if (InitIo > BT_SMP_IOCAPS_KEYBOARD_DISPLAY ||
		RespIo > BT_SMP_IOCAPS_KEYBOARD_DISPLAY)
	{
		return BT_SMP_MODEL_JUST_WORKS;
	}
	return s_SmpModelMap[InitIo][RespIo];
}

// Copy the pending OOB material into the link context of an OOB pairing.
static bool SmpOobCtxLoad(BtSmpLink_t *pLink)
{
	if (SmpOobReserve(pLink) == false)
	{
		return false;
	}
	if (s_SmpOob.bLocalValid)
	{
		memcpy(pLink->Ctx.OobLocalRand, s_SmpOob.LocalRand, 16);
	}
	if (s_SmpOob.bPeerValid)
	{
		pLink->Ctx.bOobPeerData = true;
		memcpy(pLink->Ctx.OobPeerRand, s_SmpOob.PeerRand, 16);
		memcpy(pLink->Ctx.OobPeerConfirm, s_SmpOob.PeerConfirm, 16);
	}
	return true;
}

// Generate the local SC key pair, or reuse the pair the pending local OOB
// data commits to. The crypto provider retains the private key from the key
// generation done at BtSmpOobLocalDataGen.
static int SmpLocalKeyGen(BtHciDevice_t * const pDev, BtSmpLink_t *pLink)
{
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && SmpOobLocalReady(pLink))
	{
		memcpy(pLink->Ctx.LocalPubKey, s_SmpOob.LocalPubKey, 64);
		return BT_SMP_CRYPTO_OK;
	}
	if (s_SmpOob.bLocalValid ||
		SmpCryptoPendingBegin(pLink, pDev, BT_SMP_CRYPTO_OP_PUBKEY) == false)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	int rc = SmpCryptoP256KeyGen(pLink, pLink->Ctx.LocalPubKey);
	if (rc == BT_SMP_CRYPTO_BUSY)
	{
		s_SmpCryptoPending.bRetryBusy = true;
		return BT_SMP_CRYPTO_PENDING;
	}
	if (rc != BT_SMP_CRYPTO_PENDING) { SmpCryptoPendingClear(); }
	return rc;
}

// g2 -> 6 digit Numeric Comparison value.
//
// g2(U, V, X, Y) = AES-CMAC_X(U || V || Y) mod 2^32 (Core spec Vol 3 Part H,
// 2.2.9). U and V are the 32-octet public key X coordinates, X and Y are the
// 16-octet nonces, all passed in SMP byte order. The CMAC core works on the
// big-endian form (same handling as SmpF4): each field is reversed into the
// message, the nonce X is reversed into the key. The result is the least
// significant 32 bits of the big-endian MAC, i.e. the last four octets.
static uint32_t SmpG2(const uint8_t u[32], const uint8_t v[32],
					  const uint8_t x[16], const uint8_t y[16])
{
	uint8_t m[80];
	uint8_t xs[16];
	uint8_t mac[16];

	for (int i = 0; i < 32; i++)
	{
		m[i]      = u[31 - i];
		m[32 + i] = v[31 - i];
	}
	for (int i = 0; i < 16; i++)
	{
		m[64 + i] = y[15 - i];
	}

	SmpReverse16(x, xs);
	SmpAesCmac(xs, m, sizeof(m), mac);

	return ((uint32_t)mac[12] << 24) | ((uint32_t)mac[13] << 16) |
		   ((uint32_t)mac[14] << 8)  | (uint32_t)mac[15];
}

// Compute the 6 digit Numeric Comparison value both sides display. Inputs are
// g2(PKax, PKbx, Na, Nb): the initiator public key X, the responder public key
// X, the initiator nonce, the responder nonce. Both roles assemble the same
// ordered set so the displayed values match.
static bool SmpNumericValue(BtSmpLink_t *pLink, uint32_t *pValue)
{
	s_SmpAesFault = false;
	uint8_t pkaX[32];
	uint8_t pkbX[32];
	const uint8_t *na;
	const uint8_t *nb;

	if (pLink->Ctx.bInitiator)
	{
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], pkaX);
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], pkbX);
		na = pLink->Ctx.LocalRand;
		nb = pLink->Ctx.PeerRand;
	}
	else
	{
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], pkaX);
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], pkbX);
		na = pLink->Ctx.PeerRand;
		nb = pLink->Ctx.LocalRand;
	}

	*pValue = SmpG2(pkaX, pkbX, na, nb) % 1000000;
	return !s_SmpAesFault;
}

static void SmpBuildPairingRsp(BtSmpLink_t *pLink, BtSmpPairingRsp_t *pRsp)
{
	pRsp->Code = BT_SMP_CODE_PAIRING_RSP;
	pRsp->IOCaps = s_SmpIoCaps;
	pRsp->OOBFlag = (SmpOobPeerReady(pLink) && pLink->Ctx.bSc) ?
					BT_SMP_OOB_AUTH_PRESENT : BT_SMP_OOB_AUTH_NOT_PRESENT;
	pRsp->AuthReq = (uint8_t)(s_SmpAuthReq & ~BT_SMP_AUTHREQ_KEYPRESS);
	pRsp->MaxKeySize = BT_SMP_MAX_ENC_KEY_SIZE;

	const BtSmpPairingReq_t *pReq = (const BtSmpPairingReq_t*)pLink->Ctx.PReq;
	uint8_t reqInitKeyDist = pReq->InitiatorKeyDist;
	uint8_t reqRespKeyDist = pReq->ResponderKeyDist;

	// Key distribution. Mirror back the keys the initiator offered, masked to
	// what we support. Crucially, in SC the EncKey bit is LEFT SET even though
	// the LTK is derived and never transmitted: the Nordic SoftDevice host
	// (and nRF Connect Desktop on pc-ble-driver) keeps EncKey in the negotiated
	// keydist and marks it satisfied by the SC-derived LTK. Stripping EncKey in
	// SC leaves the central's phase 3 open and its bond popup spins forever.
	// We simply skip sending the EncryptInfo/CentralId PDUs for SC (see
	// BtSmpEncryptionChanged); the advertised bit is what closes the central.
	uint8_t supported = BT_SMP_KEYDIST_ENCKEY | BT_SMP_KEYDIST_IDKEY |
						BT_SMP_KEYDIST_SIGNKEY;

	pRsp->InitiatorKeyDist = reqInitKeyDist & supported;
	pRsp->ResponderKeyDist = reqRespKeyDist & supported;
	SMP_TRACE("SMP keydist req ik=%02x rk=%02x -> rsp ik=%02x rk=%02x sc=%d\r\n",
			  reqInitKeyDist, reqRespKeyDist,
			  pRsp->InitiatorKeyDist, pRsp->ResponderKeyDist,
			  pLink->Ctx.bSc ? 1 : 0);

	pLink->Ctx.IoCaps = pRsp->IOCaps;
	pLink->Ctx.AuthReq = pRsp->AuthReq;
	memcpy(pLink->Ctx.PRsp, pRsp, 7);
}

//-----------------------------------------------------------------------------
// Inbound SMP PDU handling
//-----------------------------------------------------------------------------

// True if the whole buffer is zero. Used to detect an uncomputed DHKey.
static bool SmpIsAllZero(const uint8_t *p, size_t len)
{
	uint8_t acc = 0;
	for (size_t i = 0; i < len; i++)
	{
		acc |= p[i];
	}
	return acc == 0;
}

// Constant-time comparison of two 16-byte buffers. Used for authentication
// values (Confirm, Ea/Eb MACs) which are attacker-supplied: a data-dependent
// early-out (plain memcmp) leaks how many leading bytes matched and lets a
// remote peer brute-force the value byte-by-byte across retries.
static bool SmpEqualCT(const uint8_t *a, const uint8_t *b, size_t len)
{
	uint8_t diff = 0;
	for (size_t i = 0; i < len; i++)
	{
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0;
}

// Apply the negotiated encryption key size: zero the most significant
// (16 - EncKeySize) octets of the LTK so its effective entropy matches the
// size both sides agreed during feature exchange (Core spec Vol 3 Part H,
// 2.4.4). The LTK is stored little-endian (HCI order), so the most
// significant octets are at the high indices. Without this a peer that
// negotiated a short key would still receive a full-strength key on our side.
static void SmpApplyKeySize(uint8_t Ltk[16], uint8_t EncKeySize)
{
	int ks = EncKeySize;
	if (ks < BT_SMP_MIN_ENC_KEY_SIZE)
	{
		ks = BT_SMP_MIN_ENC_KEY_SIZE;
	}
	for (int i = ks; i < 16; i++)
	{
		Ltk[i] = 0;
	}
}

// Promote the pending key only after authentication and controller encryption
// succeed. Until then Ctx owns all new material and an abort cannot overwrite a
// previously committed bond.
static void SmpCommitPendingKeys(BtSmpLink_t *pLink)
{
	BtSmpKeys_t keys = {};
	memcpy(keys.Ltk, pLink->Ctx.Ltk, sizeof(keys.Ltk));
	keys.EncKeySize = pLink->Ctx.EncKeySize;
	keys.bAuthenticated = pLink->Ctx.bAuthenticated;
	keys.bSc = pLink->Ctx.bSc;
	keys.bValid = true;
	pLink->Keys = keys;
}

static void SmpHandlePairingReq(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								uint16_t ConnHdl, const BtSmpPairingReq_t *pReq)
{
	// A central that has already completed SC pairing on this link must not
	// silently restart it. nRF Connect Desktop re-issues PairingReq after a
	// successful encrypt; without this guard each retry derives a fresh LTK,
	// overwrites the bond, and the central retries again in a loop. Reject so
	// the loop terminates and the existing encrypted link stands.
	BtDevice_t *pAlready = BtPeerFindByHdl(ConnHdl);
	if (pLink->Ctx.State == BT_SMP_STATE_DONE ||
		(pAlready != nullptr && pAlready->bSecure))
	{
		SMP_TRACE("SMP reject re-pair (state=%d already encrypted)\r\n",
				  pLink->Ctx.State);
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	// Enforce the locally-required minimum key size, not just the spec floor:
	// rejecting a peer that offers below BT_SMP_CFG_MIN_ENC_KEY_SIZE closes the
	// KNOB-style downgrade where a MITM forces MaxKeySize down (Core Vol 3 Part
	// H 2.3.4). Default floor is 7 (spec) unless the build raises it.
	if (pReq->MaxKeySize < BT_SMP_CFG_MIN_ENC_KEY_SIZE ||
		pReq->MaxKeySize > BT_SMP_MAX_ENC_KEY_SIZE)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_ENC_KEY_SIZE);
		SmpAbortPairing(pLink);
		return;
	}

	memcpy(pLink->Ctx.PReq, pReq, 7);
	pLink->Ctx.PeerAuthReq = pReq->AuthReq;
	pLink->Ctx.bInitiator = false;
	pLink->Ctx.bSc = (pReq->AuthReq & BT_SMP_AUTHREQ_SC) &&
					 (s_SmpAuthReq & BT_SMP_AUTHREQ_SC);

	// This build requires LE Secure Connections. If the peer requests legacy
	// pairing (no SC bit), reject with Authentication Requirements rather than
	// entering the legacy flow. A central that opens with legacy (e.g. nRF
	// Connect Desktop) then retries with SC. Accepting legacy here and failing
	// later at the confirm/random step leaves such a central unable to pair.
	if ((s_SmpAuthReq & BT_SMP_AUTHREQ_SC) &&
		!(pReq->AuthReq & BT_SMP_AUTHREQ_SC))
	{
		SMP_TRACE("SMP reject legacy (peer auth=0x%02x), require SC\r\n",
				  pReq->AuthReq);
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		SmpAbortPairing(pLink);
		return;
	}

	pLink->Ctx.EncKeySize = pReq->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ?
							 pReq->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;

	// Select the association model now both IO capabilities are known: the peer
	// is the initiator, the local device is the responder. MITM applies when
	// either side sets the flag. The IO capability table selects Numeric
	// Comparison or Passkey Entry when MITM is required.
	bool mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) ||
				(pReq->AuthReq & BT_SMP_AUTHREQ_MITM);
	bool oob = pLink->Ctx.bSc &&
			   ((pReq->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT) || SmpOobPeerReady(pLink));
	// The peer's OOB flag asserts it received our OOB data. Without a local
	// data set backing that claim the ra/rb inputs fall back to zero, so a
	// peer that merely set the flag would finish a Just-Works-equivalent
	// pairing that gets reported as authenticated. Fail closed instead.
	if (pLink->Ctx.bSc && pReq->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT &&
		!SmpOobLocalReady(pLink))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE);
		SmpAbortPairing(pLink);
		return;
	}
	pLink->Ctx.Model = SmpSelectModel(pReq->IOCaps, s_SmpIoCaps, mitm, oob);
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && SmpOobCtxLoad(pLink) == false)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE);
		SmpAbortPairing(pLink);
		return;
	}
	SMP_TRACE("SMP model=%d init_io=%d resp_io=%d mitm=%d\r\n",
			  pLink->Ctx.Model, pReq->IOCaps, s_SmpIoCaps, mitm ? 1 : 0);
	if (pLink->Ctx.Model != BT_SMP_MODEL_JUST_WORKS &&
		pLink->Ctx.Model != BT_SMP_MODEL_NUMERIC_COMPARISON &&
		pLink->Ctx.Model != BT_SMP_MODEL_PASSKEY_ENTRY &&
		pLink->Ctx.Model != BT_SMP_MODEL_OOB)
	{
		// Unknown model; fail closed rather than continuing to a link
		// the peer would treat as authenticated.
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		SmpAbortPairing(pLink);
		return;
	}

	BtSmpPairingRsp_t rsp;
	SmpBuildPairingRsp(pLink, &rsp);
	SmpSend(pDev, ConnHdl, &rsp, sizeof(rsp));

	if (pLink->Ctx.bSc)
	{
		pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
		int rc = SmpLocalKeyGen(pDev, pLink);
		SMP_TRACE("SMP P256KeyGen rc=%d\r\n", rc);
		if (rc == BT_SMP_CRYPTO_FAIL)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			SmpAbortPairing(pLink);
		}
	}
	else
	{
		memset(pLink->Ctx.Tk, 0, 16);
		pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
	}
}

static bool SmpKeyPresent(const uint8_t *pKey, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		if (pKey[i] != 0)
		{
			return true;
		}
	}
	return false;
}

static bool SmpStartDhKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink)
{
	if (SmpCryptoPendingBegin(pLink, pDev, BT_SMP_CRYPTO_OP_DHKEY) == false)
	{
		return false;
	}
	pLink->Ctx.State = BT_SMP_STATE_DHKEY_WAIT;
	int rc = SmpCryptoEcdh(pLink, pLink->Ctx.PeerPubKey, pLink->Ctx.DhKey);
	if (rc == BT_SMP_CRYPTO_OK)
	{
		BtSmpDhKeyReady(pDev, 0, pLink->Ctx.DhKey);
	}
	else if (rc == BT_SMP_CRYPTO_BUSY)
	{
		s_SmpCryptoPending.bRetryBusy = true;
	}
	else if (rc == BT_SMP_CRYPTO_FAIL)
	{
		SmpCryptoPendingClear();
		return false;
	}
	return true;
}

static bool SmpTryStartDhKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
							 uint16_t ConnHdl)
{
	bool localIn = SmpKeyPresent(pLink->Ctx.LocalPubKey, sizeof(pLink->Ctx.LocalPubKey));
	bool peerIn = SmpKeyPresent(pLink->Ctx.PeerPubKey, sizeof(pLink->Ctx.PeerPubKey));

	if (!localIn || !peerIn)
	{
		return true;
	}

	BtSmpPublicKey_t pk;
	pk.Code = BT_SMP_CODE_PAIRING_PUBLIC_KEY;
	for (int i = 0; i < 32; i++)
	{
		pk.KeyX[i] = pLink->Ctx.LocalPubKey[31 - i];
		pk.KeyY[i] = pLink->Ctx.LocalPubKey[32 + 31 - i];
	}

	SmpSend(pDev, ConnHdl, &pk, sizeof(pk));

	return SmpStartDhKey(pDev, pLink);
}

// Send the local SC public key PDU (coordinates little-endian on air).
static void SmpSendLocalPubKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
							   uint16_t ConnHdl)
{
	BtSmpPublicKey_t pk;
	pk.Code = BT_SMP_CODE_PAIRING_PUBLIC_KEY;
	for (int i = 0; i < 32; i++)
	{
		pk.KeyX[i] = pLink->Ctx.LocalPubKey[31 - i];
		pk.KeyY[i] = pLink->Ctx.LocalPubKey[32 + 31 - i];
	}
	SmpSend(pDev, ConnHdl, &pk, sizeof(pk));
}

// Initiator (central) only: handle the responder's Pairing Response. Caches the
// negotiated parameters and, once the local public key is available, sends it.
static void SmpHandlePairingRsp(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								uint16_t ConnHdl, const BtSmpPairingRsp_t *pRsp)
{
	if (!pLink->Ctx.bInitiator)
	{
		// We are the responder; a Pairing Response is not expected.
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CMD_NOT_SUPPORTED);
		return;
	}

	// Enforce the locally-required minimum key size (see the responder path):
	// reject a peer offering below BT_SMP_CFG_MIN_ENC_KEY_SIZE to close the
	// KNOB-style downgrade. Default floor is 7 (spec) unless the build raises it.
	if (pRsp->MaxKeySize < BT_SMP_CFG_MIN_ENC_KEY_SIZE ||
		pRsp->MaxKeySize > BT_SMP_MAX_ENC_KEY_SIZE)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_ENC_KEY_SIZE);
		SmpAbortPairing(pLink);
		return;
	}

	// LE Secure Connections only build.
	if (!(pRsp->AuthReq & BT_SMP_AUTHREQ_SC))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		SmpAbortPairing(pLink);
		return;
	}

	memcpy(pLink->Ctx.PRsp, pRsp, 7);
	pLink->Ctx.PeerAuthReq = pRsp->AuthReq;
	pLink->Ctx.bSc = true;
	pLink->Ctx.EncKeySize = pRsp->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ?
							 pRsp->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;

	// Select the association model now both IO capabilities are known: the local
	// device is the initiator, the peer is the responder. MITM applies when
	// either side sets the flag. The IO capability table selects Numeric
	// Comparison or Passkey Entry when MITM is required.
	bool mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) ||
				(pRsp->AuthReq & BT_SMP_AUTHREQ_MITM);
	bool oob = pLink->Ctx.bSc &&
			   ((pRsp->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT) || SmpOobPeerReady(pLink));
	// Same guard as the responder side: the peer may not claim receipt of
	// local OOB data that was never generated, else zero ra/rb turn this
	// into Just Works reported as authenticated.
	if (pLink->Ctx.bSc && pRsp->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT &&
		!SmpOobLocalReady(pLink))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE);
		SmpAbortPairing(pLink);
		return;
	}
	pLink->Ctx.Model = SmpSelectModel(s_SmpIoCaps, pRsp->IOCaps, mitm, oob);
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && SmpOobCtxLoad(pLink) == false)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE);
		SmpAbortPairing(pLink);
		return;
	}
	SMP_TRACE("SMP model=%d init_io=%d resp_io=%d mitm=%d\r\n",
			  pLink->Ctx.Model, s_SmpIoCaps, pRsp->IOCaps, mitm ? 1 : 0);
	if (pLink->Ctx.Model != BT_SMP_MODEL_JUST_WORKS &&
		pLink->Ctx.Model != BT_SMP_MODEL_NUMERIC_COMPARISON &&
		pLink->Ctx.Model != BT_SMP_MODEL_PASSKEY_ENTRY &&
		pLink->Ctx.Model != BT_SMP_MODEL_OOB)
	{
		// Unknown model; fail closed rather than continuing to a link
		// the peer would treat as authenticated.
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		SmpAbortPairing(pLink);
		return;
	}

	// The initiator sends its public key after the Pairing Response. If the
	// local P-256 key is not ready yet, BtSmpLocalPubKeyReady sends it.
	if (SmpKeyPresent(pLink->Ctx.LocalPubKey, sizeof(pLink->Ctx.LocalPubKey)))
	{
		SmpSendLocalPubKey(pDev, pLink, ConnHdl);
		pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
	}
	// else stay PUBKEY_LOCAL_WAIT; LocalPubKeyReady will send it.
}

static void SmpHandlePublicKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
							   uint16_t ConnHdl, const BtSmpPublicKey_t *pPk)
{
	// Public Key PDU coordinates are little-endian on air. Store big-endian
	// for ECDH. f4 converts the X coordinates back to SMP order.
	for (int i = 0; i < 32; i++)
	{
		pLink->Ctx.PeerPubKey[i]      = pPk->KeyX[31 - i];
		pLink->Ctx.PeerPubKey[32 + i] = pPk->KeyY[31 - i];
	}

	if (pLink->Ctx.bInitiator)
	{
		// Initiator already sent its public key (after Pairing Response). On the
		// responder's public key, just start ECDH; do not resend.
		if (SmpStartDhKey(pDev, pLink) == false)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			SmpAbortPairing(pLink);
		}
		return;
	}

	if (!SmpTryStartDhKey(pDev, pLink, ConnHdl))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
		SmpAbortPairing(pLink);
	}
}

// ===========================================================================
// Passkey Entry (LE Secure Connections). Core spec Vol 3 Part H, 2.3.5.6.3.
//
// Twenty rounds, one passkey bit per round. The initiator commits first each
// round: A->B Confirm(Cai), B->A Confirm(Cbi), A->B Random(Nai), B->A
// Random(Nbi), both verify, advance. After the last round f5 derives the keys
// from the final nonces and the DHKey Check uses r = passkey (not 0). One side
// displays a generated passkey, the other inputs it; the input side resumes
// from BtSmpPasskeyReply. These handlers run only when the model is
// BT_SMP_MODEL_PASSKEY_ENTRY, leaving the Just Works / Numeric Comparison path
// unchanged.
// ===========================================================================

#define BT_SMP_PASSKEY_ROUNDS	20

// Per round ra / rb: the round-th passkey bit (LSB first) in bit 0, fixed 0x80
// high bit.
static uint8_t SmpPasskeyRa(uint32_t Passkey, uint8_t Round)
{
	return (uint8_t)(0x80 | ((Passkey >> Round) & 1));
}

// Decide whether the local device displays the passkey (true) or inputs it
// (false), from the local and peer IO capabilities. KeyboardOnly always inputs;
// DisplayOnly and DisplayYesNo always display; KeyboardDisplay displays only
// against a KeyboardOnly peer (KeyboardDisplay against KeyboardDisplay resolves
// to Numeric Comparison, not Passkey Entry).
static bool SmpPasskeyLocalDisplays(uint8_t LocalIo, uint8_t PeerIo)
{
	if (LocalIo == BT_SMP_IOCAPS_KEYBOARD_ONLY)
	{
		return false;
	}
	if (LocalIo == BT_SMP_IOCAPS_DISPLAY_ONLY ||
		LocalIo == BT_SMP_IOCAPS_DISPLAY_YESNO)
	{
		return true;
	}
	// LocalIo == BT_SMP_IOCAPS_KEYBOARD_DISPLAY
	return (PeerIo == BT_SMP_IOCAPS_KEYBOARD_ONLY);
}

// Fill the 16 octet r value for the DHKey Check f6 (Core Vol 3 Part H,
// 2.3.5.6.5). Ea uses rb, Eb uses ra; bRa selects which. Zero for Just Works
// and Numeric Comparison. The passkey (SMP little-endian, low octet first)
// for Passkey Entry, identical on both sides. For OOB, ra is the random the
// initiator distributed out of band and rb the one the responder distributed;
// each is nonzero only when the other side received it, signaled by the OOB
// flag it sent in the feature exchange (initiator flag in PReq for rb,
// responder flag in PRsp for ra). On each device one of the two values is its
// own distributed random, the other the received peer random.
static void SmpDhKeyCheckR(const BtSmpLink_t *pLink, bool bRa, uint8_t r[16])
{
	memset(r, 0, 16);
	if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		uint32_t pk = pLink->Ctx.Passkey;
		r[0] = (uint8_t)(pk & 0xFF);
		r[1] = (uint8_t)((pk >> 8) & 0xFF);
		r[2] = (uint8_t)((pk >> 16) & 0xFF);
		r[3] = (uint8_t)((pk >> 24) & 0xFF);
	}
	else if (pLink->Ctx.Model == BT_SMP_MODEL_OOB)
	{
		uint8_t flag = bRa ? pLink->Ctx.PRsp[2] : pLink->Ctx.PReq[2];
		if (flag == BT_SMP_OOB_AUTH_NOT_PRESENT)
		{
			return;
		}
		bool own = (bRa == pLink->Ctx.bInitiator);
		memcpy(r, own ? pLink->Ctx.OobLocalRand : pLink->Ctx.OobPeerRand, 16);
	}
}

// Initiator: send the round Confirm Cai = f4(PKax, PKbx, Nai, rai) with a fresh
// nonce, then wait for the responder Confirm Cbi.
static void SmpPasskeySendInitiatorConfirm(BtHciDevice_t * const pDev,
										   BtSmpLink_t *pLink, uint16_t ConnHdl)
{
	uint8_t localX[32];
	uint8_t peerX[32];
	SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);	// PKax
	SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);		// PKbx

	if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16))				// Nai
	{
		SmpFailAndLock(pDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}
	uint8_t ra = SmpPasskeyRa(pLink->Ctx.Passkey, pLink->Ctx.PkRound);

	BtSmpPairingConfirm_t cf;
	cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	if (!SmpF4(localX, peerX, pLink->Ctx.LocalRand, ra, cf.Value))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}
	memcpy(pLink->Ctx.LocalConfirm, cf.Value, 16);
	SmpSend(pDev, ConnHdl, &cf, sizeof(cf));

	pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
}

// Responder: send the round Confirm Cbi = f4(PKbx, PKax, Nbi, rbi) with a fresh
// nonce, then wait for the initiator Random Nai.
static void SmpPasskeyResponderConfirm(BtHciDevice_t * const pDev,
									   BtSmpLink_t *pLink, uint16_t ConnHdl)
{
	uint8_t localX[32];
	uint8_t peerX[32];
	SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);	// PKbx
	SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);		// PKax

	if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16))				// Nbi
	{
		SmpFailAndLock(pDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}
	uint8_t rb = SmpPasskeyRa(pLink->Ctx.Passkey, pLink->Ctx.PkRound);

	BtSmpPairingConfirm_t cf;
	cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	if (!SmpF4(localX, peerX, pLink->Ctx.LocalRand, rb, cf.Value))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}
	memcpy(pLink->Ctx.LocalConfirm, cf.Value, 16);
	SmpSend(pDev, ConnHdl, &cf, sizeof(cf));

	pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
}

// Initiator: all rounds verified. Derive keys with f5 from the final nonces and
// send the DHKey Check Ea = f6(MacKey, Na, Nb, passkey, IOcapA, A, B).
static void SmpPasskeyInitiatorFinish(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
									  uint16_t ConnHdl, const uint8_t *PeerAddr,
									  uint8_t PeerAddrType)
{
	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	uint8_t dhKeySmp[32];
	SmpReverse32(pLink->Ctx.DhKey, dhKeySmp);

	if (!SmpF5(dhKeySmp, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  localAddrType, localAddr, PeerAddrType, PeerAddr,
		  pLink->Ctx.Mackey, pLink->Ctx.Ltk))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	pLink->Ctx.bAuthenticated = true;
	SmpApplyKeySize(pLink->Ctx.Ltk, pLink->Ctx.EncKeySize);

	uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
	uint8_t rChk[16];
	SmpDhKeyCheckR(pLink, false, rChk);
	uint8_t ea[16];
	if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  rChk, iocapA,
		  localAddrType, localAddr, PeerAddrType, PeerAddr, ea))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	BtSmpDhKeyCheck_t chk;
	chk.Code = BT_SMP_CODE_PAIRING_DHKEY_CHECK;
	memcpy(chk.Value, ea, 16);
	SmpSend(pDev, ConnHdl, &chk, sizeof(chk));

	pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
}

// Responder: all rounds verified. Derive keys with f5 from the final nonces and
// wait for the initiator DHKey Check Ea; the DHKey Check handler sends Eb.
static void SmpPasskeyResponderFinish(BtSmpLink_t *pLink, const uint8_t *PeerAddr,
									  uint8_t PeerAddrType)
{
	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	uint8_t dhKeySmp[32];
	SmpReverse32(pLink->Ctx.DhKey, dhKeySmp);

	if (!SmpF5(dhKeySmp, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
		  PeerAddrType, PeerAddr, localAddrType, localAddr,
		  pLink->Ctx.Mackey, pLink->Ctx.Ltk))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	pLink->Ctx.bAuthenticated = true;
	SmpApplyKeySize(pLink->Ctx.Ltk, pLink->Ctx.EncKeySize);

	pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
}

// Passkey Entry Pairing Confirm. PeerConfirm is already stored by the caller.
static void SmpPasskeyHandleConfirm(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
									uint16_t ConnHdl)
{
	if (pLink->Ctx.bInitiator)
	{
		// Received Cbi. Reveal Nai; Cbi is verified against Nbi in the random
		// handler.
		BtSmpPairingRandom_t rnd;
		rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
		memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
		SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));
		pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
		return;
	}

	// Responder received Cai. If the user has not entered the passkey yet,
	// buffer this Confirm and respond from BtSmpPasskeyReply.
	if (!pLink->Ctx.bPkReady)
	{
		pLink->Ctx.bPkPeerCommit = true;
		return;
	}
	SmpPasskeyResponderConfirm(pDev, pLink, ConnHdl);
}

// Passkey Entry Pairing Random. PeerRand is already stored by the caller.
static void SmpPasskeyHandleRandom(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								   uint16_t ConnHdl, const uint8_t *PeerAddr,
								   uint8_t PeerAddrType)
{
	uint8_t localX[32];
	uint8_t peerX[32];
	SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);
	SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);
	uint8_t r = SmpPasskeyRa(pLink->Ctx.Passkey, pLink->Ctx.PkRound);

	if (pLink->Ctx.bInitiator)
	{
		// Received Nbi. Verify Cbi = f4(PKbx, PKax, Nbi, rbi).
		uint8_t cb[16];
		if (!SmpF4(peerX, localX, pLink->Ctx.PeerRand, r, cb))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}
		if (!SmpEqualCT(cb, pLink->Ctx.PeerConfirm, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
			SmpAuthFailCount(pLink);
			SmpAbortPairing(pLink);
			return;
		}

		if (pLink->Ctx.PkRound < BT_SMP_PASSKEY_ROUNDS - 1)
		{
			pLink->Ctx.PkRound++;
			SmpPasskeySendInitiatorConfirm(pDev, pLink, ConnHdl);
			return;
		}
		SmpPasskeyInitiatorFinish(pDev, pLink, ConnHdl, PeerAddr, PeerAddrType);
		return;
	}

	// Responder received Nai. Verify Cai = f4(PKax, PKbx, Nai, rai).
	uint8_t ca[16];
	if (!SmpF4(peerX, localX, pLink->Ctx.PeerRand, r, ca))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}
	if (!SmpEqualCT(ca, pLink->Ctx.PeerConfirm, 16))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
		SmpAuthFailCount(pLink);
		SmpAbortPairing(pLink);
		return;
	}

	// Reveal Nbi.
	BtSmpPairingRandom_t rnd;
	rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
	memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
	SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));

	if (pLink->Ctx.PkRound < BT_SMP_PASSKEY_ROUNDS - 1)
	{
		pLink->Ctx.PkRound++;
		pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
		return;
	}
	SmpPasskeyResponderFinish(pLink, PeerAddr, PeerAddrType);
}

// Begin Passkey Entry after the DHKey is ready. Decide display vs input, show
// or request the passkey, and start round 0 on the initiator once it is known.
static void SmpPasskeyBegin(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
							uint16_t ConnHdl)
{
	uint8_t localIo = pLink->Ctx.bInitiator ? pLink->Ctx.PReq[1] : pLink->Ctx.PRsp[1];
	uint8_t peerIo  = pLink->Ctx.bInitiator ? pLink->Ctx.PRsp[1] : pLink->Ctx.PReq[1];

	pLink->Ctx.PkRound = 0;
	pLink->Ctx.bPkPeerCommit = false;
	pLink->Ctx.bPkDisplay = SmpPasskeyLocalDisplays(localIo, peerIo);

	// Park in the waiting state before any user callback so a synchronous reject
	// (the weak BtSmpPasskeyRequest default sets IDLE) is not overwritten here.
	pLink->Ctx.State = pLink->Ctx.bInitiator ?
		BT_SMP_STATE_PASSKEY_WAIT : BT_SMP_STATE_CONFIRM_WAIT;

	if (pLink->Ctx.bPkDisplay)
	{
		uint8_t rnd[4];
		if (!BtSmpCryptoRand(rnd, 4))
		{
			SmpFailAndLock(pDev, ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}
		uint32_t v = ((uint32_t)rnd[0] | ((uint32_t)rnd[1] << 8) |
					  ((uint32_t)rnd[2] << 16) | ((uint32_t)rnd[3] << 24)) % 1000000u;
		pLink->Ctx.Passkey = v;
		pLink->Ctx.bPkReady = true;
		BtSmpPasskeyDisplay(ConnHdl, v);
	}
	else
	{
		pLink->Ctx.Passkey = 0;
		pLink->Ctx.bPkReady = false;
		BtSmpPasskeyRequest(ConnHdl);
	}

	// The initiator drives round 0 once it knows the passkey. Proceed only if
	// still parked in PASSKEY_WAIT; a synchronous reject moved the state to
	// IDLE. The input side starts later from BtSmpPasskeyReply.
	if (pLink->Ctx.bInitiator && pLink->Ctx.bPkReady &&
		pLink->Ctx.State == BT_SMP_STATE_PASSKEY_WAIT)
	{
		SmpPasskeySendInitiatorConfirm(pDev, pLink, ConnHdl);
	}
}

static void SmpHandlePairingConfirm(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
									uint16_t ConnHdl, const BtSmpPairingConfirm_t *pC)
{
	memcpy(pLink->Ctx.PeerConfirm, pC->Value, 16);

	if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		SmpPasskeyHandleConfirm(pDev, pLink, ConnHdl);
		return;
	}

	if (pLink->Ctx.bInitiator)
	{
		// Initiator (SC Just Works): received the responder Confirm Cb. Reply
		// with our nonce Na. Cb is verified later against Nb in PairingRandom.
		BtSmpPairingRandom_t rnd;
		rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
		memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
		SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));
		pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
		return;
	}

	if (pLink->Ctx.bSc)
	{
		return;
	}

	if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16))
	{
		SmpFailAndLock(pDev, ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	uint8_t ra[6] = {0};
	uint8_t ia[6] = {0};
	uint8_t rat = 0;
	uint8_t iat = 0;

	if (pPeer != nullptr)
	{
		memcpy(ia, pPeer->Conn.PeerAddr, 6);
		iat = pPeer->Conn.PeerAddrType;
	}

	if (!SmpC1(pLink->Ctx.Tk, pLink->Ctx.LocalRand,
		  pLink->Ctx.PReq, pLink->Ctx.PRsp,
		  iat, ia, rat, ra, pLink->Ctx.LocalConfirm))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	BtSmpPairingConfirm_t cf;
	cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	memcpy(cf.Value, pLink->Ctx.LocalConfirm, 16);
	SmpSend(pDev, ConnHdl, &cf, sizeof(cf));

	pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
}

static void SmpHandlePairingRandom(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								   uint16_t ConnHdl, const BtSmpPairingRandom_t *pR)
{
	memcpy(pLink->Ctx.PeerRand, pR->Value, 16);

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	uint8_t peerAddr[6] = {0};
	uint8_t peerAddrType = 0;

	if (pPeer != nullptr)
	{
		memcpy(peerAddr, pPeer->Conn.PeerAddr, 6);
		peerAddrType = pPeer->Conn.PeerAddrType;
	}

	// Defense in depth behind the phase gate: an SC Pairing Random must never be
	// processed before the DHKey has been computed. A valid P-256 DHKey is never
	// all zero, so an all-zero value means the public-key/ECDH phase was skipped;
	// running f5/f4 over it would derive a peer-known LTK. Legacy pairing does
	// not use the DHKey.
	if (pLink->Ctx.bSc && SmpIsAllZero(pLink->Ctx.DhKey, 32))
	{
		SmpAbortOffPhase(pDev, ConnHdl, pLink);
		return;
	}

	if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		SmpPasskeyHandleRandom(pDev, pLink, ConnHdl, peerAddr, peerAddrType);
		return;
	}

	if (pLink->Ctx.bInitiator)
	{
		// Initiator: received responder nonce Nb. For SC Just Works and Numeric
		// Comparison verify the responder Confirm Cb = f4(PKbx, PKax, Nb, 0).
		// OOB has no Confirm exchange; the peer key was authenticated against
		// the out of band confirm at DHKey time.
		if (pLink->Ctx.Model != BT_SMP_MODEL_OOB)
		{
			uint8_t localX[32];
			uint8_t peerX[32];
			SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);	// PKax
			SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);		// PKbx

			uint8_t cb[16];
			if (!SmpF4(peerX, localX, pLink->Ctx.PeerRand, 0, cb))
			{
				SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
				return;
			}
			if (!SmpEqualCT(cb, pLink->Ctx.PeerConfirm, 16))
			{
				SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
				SmpAuthFailCount(pLink);
				SmpAbortPairing(pLink);
				return;
			}
		}

		uint8_t localAddr[6];
		uint8_t localAddrType = 0;
		BtSmpLocalAddrGet(&localAddrType, localAddr);

		uint8_t dhKeySmp[32];
		SmpReverse32(pLink->Ctx.DhKey, dhKeySmp);

		// f5(DHKey, Na, Nb, A1=initiator(local), A2=responder(peer)).
		if (!SmpF5(dhKeySmp, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  localAddrType, localAddr, peerAddrType, peerAddr,
			  pLink->Ctx.Mackey, pLink->Ctx.Ltk))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		if (pLink->Ctx.Model == BT_SMP_MODEL_OOB)
		{
			pLink->Ctx.bAuthenticated = true;
		}
		SmpApplyKeySize(pLink->Ctx.Ltk, pLink->Ctx.EncKeySize);

		if (pLink->Ctx.Model == BT_SMP_MODEL_NUMERIC_COMPARISON)
		{
			// Hold the initiator DHKey Check (Ea) and have the application
			// display the value for the user to confirm. The flow resumes from
			// BtSmpNumericComparisonReply.
			uint32_t v;
			if (!SmpNumericValue(pLink, &v))
			{
				SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
				return;
			}
			pLink->Ctx.State = BT_SMP_STATE_NUMERIC_WAIT;
			BtSmpNumericComparison(ConnHdl, v);
			return;
		}

		// Ea = f6(MacKey, Na, Nb, rb, IOcapA, A=initiator(local), B=responder(peer)).
		// rb is zero for Just Works, the responder OOB random for OOB.
		uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
		uint8_t rChk[16];
		SmpDhKeyCheckR(pLink, false, rChk);
		uint8_t ea[16];
		if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  rChk, iocapA,
			  localAddrType, localAddr, peerAddrType, peerAddr, ea))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		BtSmpDhKeyCheck_t chk;
		chk.Code = BT_SMP_CODE_PAIRING_DHKEY_CHECK;
		memcpy(chk.Value, ea, 16);
		SmpSend(pDev, ConnHdl, &chk, sizeof(chk));

		pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
		return;
	}

	if (pLink->Ctx.bSc)
	{
		BtSmpPairingRandom_t rnd;
		rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
		memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
		SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));

		uint8_t localAddr[6];
		uint8_t localAddrType = 0;
		BtSmpLocalAddrGet(&localAddrType, localAddr);

		uint8_t dhKeySmp[32];
		SmpReverse32(pLink->Ctx.DhKey, dhKeySmp);

		if (!SmpF5(dhKeySmp, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  peerAddrType, peerAddr, localAddrType, localAddr,
			  pLink->Ctx.Mackey, pLink->Ctx.Ltk))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		if (pLink->Ctx.Model == BT_SMP_MODEL_OOB)
		{
			pLink->Ctx.bAuthenticated = true;
		}
		SmpApplyKeySize(pLink->Ctx.Ltk, pLink->Ctx.EncKeySize);
		// Park in DHKEY_CHECK_WAIT before the user callback. A synchronous
		// reject inside BtSmpNumericComparison takes the reply path and sets
		// the state to IDLE; do not overwrite it afterward.
		pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
		if (pLink->Ctx.Model == BT_SMP_MODEL_NUMERIC_COMPARISON)
		{
			// Both nonces are known: display the value for the user. The
			// responder DHKey Check (Eb) is held in the DHKey Check handler
			// until the user confirms through BtSmpNumericComparisonReply.
			uint32_t v;
			if (!SmpNumericValue(pLink, &v))
			{
				SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
				return;
			}
			BtSmpNumericComparison(ConnHdl, v);
		}
		return;
	}

	uint8_t calc[16];
	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	if (!SmpC1(pLink->Ctx.Tk, pLink->Ctx.PeerRand,
		  pLink->Ctx.PReq, pLink->Ctx.PRsp,
		  peerAddrType, peerAddr, localAddrType, localAddr, calc))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	if (!SmpEqualCT(calc, pLink->Ctx.PeerConfirm, 16))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
		SmpAuthFailCount(pLink);
		SmpAbortPairing(pLink);
		return;
	}

	BtSmpPairingRandom_t rnd;
	rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
	memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
	SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));

	if (!SmpS1(pLink->Ctx.Tk, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand, pLink->Ctx.Ltk))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}
	SmpApplyKeySize(pLink->Ctx.Ltk, pLink->Ctx.EncKeySize);

	pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
}

static void SmpHandleDhKeyCheck(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								uint16_t ConnHdl, const BtSmpDhKeyCheck_t *pChk)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	uint8_t peerAddr[6] = {0};
	uint8_t peerAddrType = 0;

	if (pPeer != nullptr)
	{
		memcpy(peerAddr, pPeer->Conn.PeerAddr, 6);
		peerAddrType = pPeer->Conn.PeerAddrType;
	}

	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	if (pLink->Ctx.bInitiator)
	{
		// Initiator: the peer sent its DHKey Check Eb. Verify it:
		// Eb = f6(MacKey, Nb, Na, 0, IOcapB, B=responder(peer), A=initiator(local)).
		uint8_t iocapB[3] = { pLink->Ctx.PRsp[1], pLink->Ctx.PRsp[2], pLink->Ctx.PRsp[3] };
		uint8_t rChk[16];
		SmpDhKeyCheckR(pLink, true, rChk);
		uint8_t eb[16];
		if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  rChk, iocapB,
			  peerAddrType, peerAddr, localAddrType, localAddr, eb))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		if (!SmpEqualCT(eb, pChk->Value, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			SmpAuthFailCount(pLink);
			SmpAbortPairing(pLink);
			return;
		}

		// Authentication complete. As the central, start link encryption from
		// the derived LTK (SC: Rand and EDIV are zero).
		pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
		BtSmpHciEnableEncryption(pDev, ConnHdl, 0, 0, pLink->Ctx.Ltk);
		return;
	}

	uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
	uint8_t iocapB[3] = { pLink->Ctx.PRsp[1], pLink->Ctx.PRsp[2], pLink->Ctx.PRsp[3] };
	uint8_t rChk[16];
	SmpDhKeyCheckR(pLink, false, rChk);

	uint8_t ea[16];
	if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
		  rChk, iocapA,
		  peerAddrType, peerAddr, localAddrType, localAddr, ea))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	if (!SmpEqualCT(ea, pChk->Value, 16))
	{
#if BT_SMP_DHKEY_ORDER_FALLBACK
		// The Confirm stage already passed, so ECDH produced the same point as
		// the peer. If Ea fails here, the most common remaining issue is DHKey
		// byte order at f5. Try the other provider order before failing.
		uint8_t altMackey[16];
		uint8_t altLtk[16];
		uint8_t altEa[16];

		if (!SmpF5(pLink->Ctx.DhKey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  peerAddrType, peerAddr, localAddrType, localAddr,
			  altMackey, altLtk))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		if (!SmpF6(altMackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  rChk, iocapA,
			  peerAddrType, peerAddr, localAddrType, localAddr, altEa))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		SMP_TRACE("Ea mismatch cur0=%02x alt0=%02x peer0=%02x\r\n",
				  ea[0], altEa[0], pChk->Value[0]);

		if (!SmpEqualCT(altEa, pChk->Value, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			SmpAuthFailCount(pLink);
			SmpAbortPairing(pLink);
			return;
		}

		memcpy(pLink->Ctx.Mackey, altMackey, 16);
		memcpy(pLink->Ctx.Ltk, altLtk, 16);
		SmpApplyKeySize(pLink->Ctx.Ltk, pLink->Ctx.EncKeySize);
		memcpy(ea, altEa, 16);
		SMP_TRACE("Ea matched with raw DHKey f5 input\r\n");
#else
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
		SmpAuthFailCount(pLink);
		SmpAbortPairing(pLink);
		return;
#endif
	}

	if (pLink->Ctx.Model == BT_SMP_MODEL_NUMERIC_COMPARISON &&
		!pLink->Ctx.bAuthenticated)
	{
		// Ea verified. Hold the responder DHKey Check (Eb) until the user
		// confirms the displayed value through BtSmpNumericComparisonReply.
		pLink->Ctx.State = BT_SMP_STATE_NUMERIC_WAIT;
		return;
	}

	// Eb uses ra where the Ea verification above used rb; recompute.
	SmpDhKeyCheckR(pLink, true, rChk);

	uint8_t eb[16];
	if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  rChk, iocapB,
		  localAddrType, localAddr, peerAddrType, peerAddr, eb))
	{
		SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	BtSmpDhKeyCheck_t chk;
	chk.Code = BT_SMP_CODE_PAIRING_DHKEY_CHECK;
	memcpy(chk.Value, eb, 16);
	SmpSend(pDev, ConnHdl, &chk, sizeof(chk));

	pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
}

static void SmpHandleEncryptInfo(BtSmpLink_t *pLink, const BtSmpEncryptInfo_t *pInfo)
{
	memcpy(pLink->Keys.Ltk, pInfo->Ltk, 16);
}

static void SmpHandleCentralId(BtSmpLink_t *pLink, const BtSmpCentralId_t *pId)
{
	pLink->Keys.Ediv = pId->Ediv;
	pLink->Keys.Rand = pId->Rand;
}

static void SmpHandleIdInfo(BtSmpLink_t *pLink, const BtSmpIdInfo_t *pInfo)
{
	memcpy(pLink->Keys.Irk, pInfo->Irk, 16);
}

static void SmpHandleIdAddrInfo(BtSmpLink_t *pLink, const BtSmpIdAddrInfo_t *pInfo)
{
	pLink->Keys.IdAddrType = pInfo->AddrType;
	memcpy(pLink->Keys.IdAddr, pInfo->Addr, 6);
	// Peer identity now known. Refresh the stored bond so it holds the peer
	// IRK. BtSmpBondAdd keys the slot by the connection address (not this
	// identity address), so the refresh updates the same slot the lookup will
	// match on reconnect.
	if (pLink->Keys.bValid)
	{
		BtSmpBondAdd(pLink->ConnHdl, &pLink->Keys);
	}
}

static void SmpHandleSigningInfo(BtSmpLink_t *pLink, const BtSmpSigningInfo_t *pInfo)
{
	memcpy(pLink->Keys.Csrk, pInfo->Csrk, 16);
	if (pLink->Keys.bValid)
	{
		BtSmpBondAdd(pLink->ConnHdl, &pLink->Keys);
	}
}

// Mark one key type of the peer's Phase-3 key distribution as received and, once
// every negotiated key has arrived, close the key-distribution window by moving
// the link to DONE. This bounds how long the H4 gate stays open on the link.
static void SmpKeyDistReceived(BtSmpLink_t *pLink, uint8_t KeyBit)
{
	pLink->Ctx.KeyDistExp &= (uint8_t)~KeyBit;
	if (pLink->Ctx.KeyDistExp == 0)
	{
		pLink->Ctx.State = BT_SMP_STATE_DONE;
	}
}

//-----------------------------------------------------------------------------
// Public entry points
//-----------------------------------------------------------------------------

void BtProcessSmpData(BtHciDevice_t * const pDev, uint16_t ConnHdl,
					  BtL2CapSmp_t * const pSmp, size_t Len)
{
	if (pSmp == nullptr || Len < 1)
	{
		return;
	}

	s_pSmpActiveDev = pDev;

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	SMP_TRACE_PDU("RX", pSmp->Code, pLink ? (int)pLink->Ctx.State : -1);

	if (pLink == nullptr)
	{
		pLink = SmpLinkAlloc(ConnHdl);
		if (pLink == nullptr)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			return;
		}
	}

	// H5: once a link is locked (SMP timeout or too many failed attempts) no
	// further SMP is accepted until it disconnects (Core Vol 3 Part H 3.4/2.3.6).
	if (pLink->Ctx.bLocked)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_REPEATED_ATTEMPTS);
		return;
	}

	// H5: anchor the pairing timer on the first PDU of an exchange (state still
	// IDLE). The timer is not refreshed per PDU, so total pairing time is bounded
	// to BT_SMP_TIMEOUT_MS and the peer cannot extend it by sending further PDUs.
	if (pLink->Ctx.State == BT_SMP_STATE_IDLE)
	{
		pLink->Ctx.TmrStart = BtSmpMsTick();
	}

	// H5: fail an overdue in-progress pairing before handling this PDU. The
	// external BtSmpTimeoutCheck() covers the silent-peer case; this covers a
	// peer that resumes after the deadline.
	if (SmpPairingTimedOut(pLink))
	{
		pLink->Ctx.FailCount++;
		SmpFailAndLock(pDev, ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		return;
	}

	// Validate the PDU has enough bytes for its code before any handler
	// dereferences fixed-size key/nonce/confirm fields. Pairing PDUs are
	// attacker-controlled; a short PDU (e.g. a 1-byte Public Key) would
	// otherwise over-read 16-64 bytes past the received L2CAP buffer, feeding
	// adjacent memory into the crypto exchange. Each struct includes the Code
	// byte, so Len (total payload) must be >= sizeof(struct).
	{
		size_t minLen = 1;
		switch (pSmp->Code)
		{
			case BT_SMP_CODE_PAIRING_REQ:
			case BT_SMP_CODE_PAIRING_RSP:			minLen = sizeof(BtSmpPairingReq_t); break;
			case BT_SMP_CODE_PAIRING_CONFIRM:		minLen = sizeof(BtSmpPairingConfirm_t); break;
			case BT_SMP_CODE_PAIRING_RANDOM:		minLen = sizeof(BtSmpPairingRandom_t); break;
			case BT_SMP_CODE_PAIRING_FAILED:		minLen = sizeof(BtSmpPairingFailed_t); break;
			case BT_SMP_CODE_PAIRING_ENCRYP_INFO:	minLen = sizeof(BtSmpEncryptInfo_t); break;
			case BT_SMP_CODE_PAIRING_CENTRAL_ID:	minLen = sizeof(BtSmpCentralId_t); break;
			case BT_SMP_CODE_PAIRING_ID_INFO:		minLen = sizeof(BtSmpIdInfo_t); break;
			case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:	minLen = sizeof(BtSmpIdAddrInfo_t); break;
			case BT_SMP_CODE_PAIRING_SIGNING_INFO:	minLen = sizeof(BtSmpSigningInfo_t); break;
			case BT_SMP_CODE_PAIRING_PUBLIC_KEY:	minLen = sizeof(BtSmpPublicKey_t); break;
			case BT_SMP_CODE_PAIRING_DHKEY_CHECK:	minLen = sizeof(BtSmpDhKeyCheck_t); break;
			case BT_SMP_CODE_PAIRING_SECURITY_REQ:	minLen = sizeof(BtSmpSecurityReq_t); break;
			default:								minLen = 1; break;
		}

		if (Len < minLen)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_INVALID_PARAMS);
			return;
		}
	}

	// H4: phase-3 key-distribution PDUs are only valid on an encrypted link
	// during the key-distribution phase of an active pairing (Core Vol 3 Part H
	// 2.4.6 / 3.6.1). Accepting them before encryption or outside that phase
	// lets an unauthenticated / off-phase peer overwrite the stored
	// LTK/IRK/identity/CSRK in the bond record. Reject such PDUs.
	switch (pSmp->Code)
	{
		case BT_SMP_CODE_PAIRING_ENCRYP_INFO:
		case BT_SMP_CODE_PAIRING_CENTRAL_ID:
		case BT_SMP_CODE_PAIRING_ID_INFO:
		case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:
		case BT_SMP_CODE_PAIRING_SIGNING_INFO:
		{
			BtDevice_t *pKdPeer = BtPeerFindByHdl(ConnHdl);
			if (pKdPeer == nullptr || pKdPeer->bSecure == false ||
				pLink->Ctx.State != BT_SMP_STATE_KEYDIST)
			{
				SMP_TRACE("SMP drop key-dist code 0x%02x (secure=%d state=%d)\r\n",
						  pSmp->Code, (pKdPeer != nullptr) && pKdPeer->bSecure,
						  (int)pLink->Ctx.State);
				SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
				return;
			}
			break;
		}
		default:
			break;
	}

	// Phase gate: each pairing PDU is accepted only in the exact state where it
	// is expected. The coarse idle-vs-active check this replaces let an SC
	// responder in PUBKEY_WAIT accept a Pairing Random, run f5 over an
	// uncomputed DHKey and complete a Secure Connections pairing with no
	// public-key exchange. Key-distribution PDUs are gated above. An off-phase
	// PDU terminates the attempt (SmpAbortOffPhase) rather than entering a
	// handler on half-built state.
	switch (pSmp->Code)
	{
		case BT_SMP_CODE_PAIRING_REQ:
			// Responder side, and only when no pairing runs on the link. A
			// request that arrives mid-pairing or after completion is rejected
			// (the handler also refuses a re-pair on an encrypted link).
			if (pLink->Ctx.State != BT_SMP_STATE_IDLE)
			{
				SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
				return;
			}
			break;

		case BT_SMP_CODE_PAIRING_RSP:
			// Initiator only, and only right after it sent the Pairing Request.
			if (!pLink->Ctx.bInitiator ||
				pLink->Ctx.State != BT_SMP_STATE_PUBKEY_LOCAL_WAIT)
			{
				SmpAbortOffPhase(pDev, ConnHdl, pLink);
				return;
			}
			break;

		case BT_SMP_CODE_PAIRING_PUBLIC_KEY:
			// Public-key exchange phase. The peer key can arrive before the
			// local controller key is ready, so PUBKEY_LOCAL_WAIT is valid too.
			if (pLink->Ctx.State != BT_SMP_STATE_PUBKEY_WAIT &&
				pLink->Ctx.State != BT_SMP_STATE_PUBKEY_LOCAL_WAIT)
			{
				SmpAbortOffPhase(pDev, ConnHdl, pLink);
				return;
			}
			break;

		case BT_SMP_CODE_PAIRING_CONFIRM:
			// Confirm phase. Passkey Entry buffers a peer Confirm that arrives
			// while the input side still waits for the user passkey, so
			// PASSKEY_WAIT is also a valid arrival state.
			if (pLink->Ctx.State != BT_SMP_STATE_CONFIRM_WAIT &&
				pLink->Ctx.State != BT_SMP_STATE_PASSKEY_WAIT)
			{
				SmpAbortOffPhase(pDev, ConnHdl, pLink);
				return;
			}
			break;

		case BT_SMP_CODE_PAIRING_RANDOM:
			// Random is valid only once the confirm exchange advanced the link
			// to RANDOM_WAIT. Accepting it earlier (e.g. in PUBKEY_WAIT) would
			// run f5 over an uncomputed, zero DHKey and complete a Secure
			// Connections pairing with no public-key exchange or ECDH.
			if (pLink->Ctx.State != BT_SMP_STATE_RANDOM_WAIT)
			{
				SmpAbortOffPhase(pDev, ConnHdl, pLink);
				return;
			}
			break;

		case BT_SMP_CODE_PAIRING_DHKEY_CHECK:
			if (pLink->Ctx.State != BT_SMP_STATE_DHKEY_CHECK_WAIT)
			{
				SmpAbortOffPhase(pDev, ConnHdl, pLink);
				return;
			}
			break;

		default:
			break;
	}

	switch (pSmp->Code)
	{
		case BT_SMP_CODE_PAIRING_REQ:
			SmpHandlePairingReq(pDev, pLink, ConnHdl, (const BtSmpPairingReq_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_RSP:
			SmpHandlePairingRsp(pDev, pLink, ConnHdl, (const BtSmpPairingRsp_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_PUBLIC_KEY:
			SmpHandlePublicKey(pDev, pLink, ConnHdl, (const BtSmpPublicKey_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_CONFIRM:
			SmpHandlePairingConfirm(pDev, pLink, ConnHdl, (const BtSmpPairingConfirm_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_RANDOM:
			SmpHandlePairingRandom(pDev, pLink, ConnHdl, (const BtSmpPairingRandom_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_DHKEY_CHECK:
			SmpHandleDhKeyCheck(pDev, pLink, ConnHdl, (const BtSmpDhKeyCheck_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_ENCRYP_INFO:
			SmpHandleEncryptInfo(pLink, (const BtSmpEncryptInfo_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_CENTRAL_ID:
			SmpHandleCentralId(pLink, (const BtSmpCentralId_t*)pSmp);
			// Central Identification is the last PDU of the ENCKEY set (legacy).
			SmpKeyDistReceived(pLink, BT_SMP_KEYDIST_ENCKEY);
			break;

		case BT_SMP_CODE_PAIRING_ID_INFO:
			SmpHandleIdInfo(pLink, (const BtSmpIdInfo_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:
			SmpHandleIdAddrInfo(pLink, (const BtSmpIdAddrInfo_t*)pSmp);
			// Identity Address is the last PDU of the IDKEY set (IRK + address).
			SmpKeyDistReceived(pLink, BT_SMP_KEYDIST_IDKEY);
			break;

		case BT_SMP_CODE_PAIRING_SIGNING_INFO:
			SmpHandleSigningInfo(pLink, (const BtSmpSigningInfo_t*)pSmp);
			SmpKeyDistReceived(pLink, BT_SMP_KEYDIST_SIGNKEY);
			break;

		case BT_SMP_CODE_PAIRING_FAILED:
		{
			if (Len >= sizeof(BtSmpPairingFailed_t))
			{
				SMP_TRACE("SMP RX Failed reason=0x%02x\r\n",
						  ((const BtSmpPairingFailed_t*)pSmp)->Reason);
			}
			// A Pairing Failed is only meaningful during an active pairing. On
			// an idle link it is unsolicited; ignore it so a peer cannot drive
			// the repeated-attempts counter to lockout without pairing.
			if (pLink->Ctx.State != BT_SMP_STATE_IDLE)
			{
				SmpAbortPairing(pLink);
				pLink->Ctx.FailCount++;
				if (pLink->Ctx.FailCount >= BT_SMP_MAX_PAIR_ATTEMPTS)
				{
					pLink->Ctx.bLocked = true;
				}
				BtSmpPairingComplete(ConnHdl, false, nullptr);
			}
			break;
		}

		case BT_SMP_CODE_PAIRING_SECURITY_REQ:
			// A peripheral is asking us (the central) to secure the link. Start
			// pairing or re-encrypt from a bond. Idempotent if already running.
			BtSmpStartPairing(ConnHdl);
			break;

		default:
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CMD_NOT_SUPPORTED);
			break;
	}
}

void BtSmpLocalPubKeyReady(BtHciDevice_t * const pDev, uint8_t Status,
						   const uint8_t *pKeyX, const uint8_t *pKeyY)
{
	s_pSmpActiveDev = pDev;
	SMP_TRACE("SMP LocalPubKeyReady status=%d\r\n", Status);

	BtSmpLink_t *pLink = SmpCryptoPendingTake(BT_SMP_CRYPTO_OP_PUBKEY);
	if (pLink == nullptr ||
		(pLink->Ctx.State != BT_SMP_STATE_PUBKEY_WAIT &&
		 pLink->Ctx.State != BT_SMP_STATE_PUBKEY_LOCAL_WAIT))
	{
		return;
	}
	do
	{

		if (Status != 0 || pKeyX == nullptr || pKeyY == nullptr)
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			SmpAbortPairing(pLink);
			continue;
		}

		memcpy(&pLink->Ctx.LocalPubKey[0],  pKeyX, 32);
		memcpy(&pLink->Ctx.LocalPubKey[32], pKeyY, 32);

		if (pLink->Ctx.bInitiator)
		{
			// Initiator: send our public key once the Pairing Response has
			// arrived (PRsp cached). If not yet, SmpHandlePairingRsp sends it.
			if (pLink->Ctx.PRsp[0] == BT_SMP_CODE_PAIRING_RSP)
			{
				SmpSendLocalPubKey(pDev, pLink, pLink->ConnHdl);
				pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
			}
			continue;
		}

		if (!SmpTryStartDhKey(pDev, pLink, pLink->ConnHdl))
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			SmpAbortPairing(pLink);
		}
	} while (false);
}

void BtSmpDhKeyReady(BtHciDevice_t * const pDev, uint8_t Status, const uint8_t *pDhKey)
{
	s_pSmpActiveDev = pDev;
	SMP_TRACE("SMP DhKeyReady status=%d\r\n", Status);

	BtSmpLink_t *pLink = SmpCryptoPendingTake(BT_SMP_CRYPTO_OP_DHKEY);
	if (pLink == nullptr || pLink->Ctx.State != BT_SMP_STATE_DHKEY_WAIT)
	{
		return;
	}
	do
	{

		if (Status != 0 || pDhKey == nullptr)
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			SmpAbortPairing(pLink);
			continue;
		}

		memcpy(pLink->Ctx.DhKey, pDhKey, 32);

		if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
		{
			SmpPasskeyBegin(pDev, pLink, pLink->ConnHdl);
			continue;
		}

		if (pLink->Ctx.Model == BT_SMP_MODEL_OOB)
		{
			// OOB (Core Vol 3 Part H, 2.3.5.6.4): no Confirm exchange over the
			// air. When the peer data was provided, authenticate its public key
			// against the out of band confirm C = f4(PKx, PKx, r, 0) over the
			// peer own key.
			if (pLink->Ctx.bOobPeerData)
			{
				uint8_t peerX[32];
				uint8_t c[16];
				SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);
				if (!SmpF4(peerX, peerX, pLink->Ctx.OobPeerRand, 0, c))
				{
					SmpSendFailed(s_pSmpActiveDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
					SmpAbortPairing(pLink);
					continue;
				}
				if (!SmpEqualCT(c, pLink->Ctx.OobPeerConfirm, 16))
				{
					SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
					SmpAuthFailCount(pLink);
					SmpAbortPairing(pLink);
					continue;
				}
			}

			if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16))
			{
				SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
				SmpAbortPairing(pLink);
				continue;
			}

			if (pLink->Ctx.bInitiator)
			{
				// Initiator sends Na and waits for Nb.
				BtSmpPairingRandom_t rnd;
				rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
				memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
				SmpSend(pDev, pLink->ConnHdl, &rnd, sizeof(rnd));
			}
			// Responder holds Nb until the initiator Na arrives; the Pairing
			// Random handler replies with it.
			pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
			continue;
		}

		if (pLink->Ctx.bInitiator)
		{
			// Initiator (SC Just Works): generate our nonce Na and wait for the
			// responder Confirm Cb. The initiator does not send a Confirm.
			if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16))
			{
				SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
				SmpAbortPairing(pLink);
				continue;
			}
			pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
			continue;
		}

		// LE Secure Connections responder:
		// Cb = f4(PKbx, PKax, Nb, 0)
		// Public keys are stored big-endian for ECDH; f4 uses the SMP
		// little-endian X-coordinate byte order.
		if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16))
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			SmpAbortPairing(pLink);
			continue;
		}

		uint8_t localX[32];
		uint8_t peerX[32];
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);

		BtSmpPairingConfirm_t cf;
		cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
		if (!SmpF4(localX, peerX, pLink->Ctx.LocalRand, 0, cf.Value))
		{
			SmpSendFailed(s_pSmpActiveDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			SmpAbortPairing(pLink);
			continue;
		}
		memcpy(pLink->Ctx.LocalConfirm, cf.Value, 16);

#if BT_SMP_TRACE_ENABLE
		{
			// Trace-only self-check: recompute Cb to confirm f4 is deterministic.
			uint8_t cb2[16];
			if (!SmpF4(localX, peerX, pLink->Ctx.LocalRand, 0, cb2))
			{
				SmpSendFailed(s_pSmpActiveDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
				SmpAbortPairing(pLink);
				continue;
			}
			bool stable = (memcmp(cb2, cf.Value, 16) == 0);
			SMP_TRACE("Cb stable=%d firstbyte=%02x Nb0=%02x\r\n",
					  stable ? 1 : 0, cf.Value[0], pLink->Ctx.LocalRand[0]);
		}
#endif

		SmpSend(pDev, pLink->ConnHdl, &cf, sizeof(cf));

		// After responder Confirm, SC Just Works waits for initiator Random Na.
		pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
	} while (false);
}

void BtSmpProcessLtkRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint64_t Rand, uint16_t Ediv)
{
	uint8_t key[16];
	bool have = false;

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);

	if (pLink != nullptr && pLink->Ctx.State == BT_SMP_STATE_LTK_WAIT &&
		SmpKeyPresent(pLink->Ctx.Ltk, sizeof(pLink->Ctx.Ltk)))
	{
		memcpy(key, pLink->Ctx.Ltk, 16);
		have = true;
	}
	else if (pLink != nullptr && pLink->Ctx.State == BT_SMP_STATE_DONE &&
		pLink->Keys.bValid)
	{
		memcpy(key, pLink->Keys.Ltk, 16);
		have = true;
	}
	else
	{
		have = BtSmpBondLtkLookup(ConnHdl, Rand, Ediv, key);
	}

	if (have)
	{
		SMP_TRACE("SMP LTK rqst -> reply (ediv=0x%04x rand=%08x%08x)\r\n",
				  (unsigned)Ediv,
				  (unsigned)(Rand >> 32), (unsigned)(Rand & 0xFFFFFFFF));
		SMP_TRACE("SMP LTK %02x%02x%02x%02x..%02x%02x%02x%02x\r\n",
				  key[0], key[1], key[2], key[3],
				  key[12], key[13], key[14], key[15]);
		// The LTK Request Reply is an HCI COMMAND, not ACL data. On the SDC
		// implementation it must go through sdc_hci_cmd_le_long_term_key_request_reply,
		// not the ACL data path. Route via the provider hook.
		BtSmpHciLtkReply(pDev, ConnHdl, key);
	}
	else
	{
		SMP_TRACE("SMP LTK rqst -> NEG reply (no key)\r\n");
		BtSmpHciLtkNegReply(pDev, ConnHdl);
	}
}

// Map a key record to the generic connection security state the ATT
// permission checks consume. An authenticated association model (numeric
// comparison; passkey / OOB when added) raises the level above ENC_UNAUTH.
// Authenticated Secure Connections is LESC_AUTH; authenticated legacy is
// ENC_AUTH.
static void SmpConnSecFromKeys(BtConnSec_t *pSec, const BtSmpKeys_t *pKeys)
{
	pSec->KeySize = pKeys->EncKeySize;
	pSec->Level = pKeys->bAuthenticated ?
		(pKeys->bSc ? BT_GAP_SEC_LEVEL_LESC_AUTH : BT_GAP_SEC_LEVEL_ENC_AUTH) :
		BT_GAP_SEC_LEVEL_ENC_UNAUTH;
	if (pKeys->bSc)
	{
		pSec->Flags |= BT_GAP_SEC_FLAG_SC;
	}
	for (int i = 0; i < 16; i++)
	{
		if (pKeys->Csrk[i] != 0U)
		{
			pSec->Flags |= BT_GAP_SEC_FLAG_SIGNED;
			break;
		}
	}
}

void BtSmpEncryptionChanged(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint8_t Status, uint8_t Enabled)
{
	SMP_TRACE("SMP EncChange status=%d enabled=%d\r\n", Status, Enabled);

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (Status == 0 && Enabled != 0 && pLink != nullptr &&
		pLink->Ctx.State == BT_SMP_STATE_LTK_WAIT)
	{
		SmpCommitPendingKeys(pLink);
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer != nullptr)
	{
		pPeer->bSecure = (Status == 0) && (Enabled != 0);

		// Produce the generic security state the ATT permission checks consume.
		// A fresh pairing has a populated per-link key record by this point
		// (SmpCommitPendingKeys ran above). A bonded reconnect on the responder
		// side exchanges no SMP PDU and has no link record, so the properties
		// come from the stored bond whose LTK encrypted the link.
		BtConnSec_t sec;
		memset(&sec, 0, sizeof(sec));
		if (pPeer->bSecure)
		{
			BtSmpLink_t *pSecLink = SmpLinkFind(ConnHdl);
			if (pSecLink != nullptr && pSecLink->Keys.bValid)
			{
				SmpConnSecFromKeys(&sec, &pSecLink->Keys);
			}
			else
			{
				BtSmpKeys_t keys;
				if (BtSmpBondKeysLookup(ConnHdl, 0U, 0U, &keys))
				{
					SmpConnSecFromKeys(&sec, &keys);
				}
				else
				{
					// Encrypted with no key record and no stored bond. The
					// key properties are unknown; report the floor.
					sec.Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
					sec.KeySize = BT_SMP_MAX_ENC_KEY_SIZE;
				}
				CryptoSecureWipe(&keys, sizeof(keys));
			}
			if (BtSmpBonded(ConnHdl))
			{
				sec.Flags |= BT_GAP_SEC_FLAG_BONDED;
			}
		}
		BtGapConnSecSet(ConnHdl, &sec);
	}

	if (Status != 0 || Enabled == 0)
	{
		if (pLink != nullptr)
		{
			SmpAbortPairing(pLink);
		}
		BtSmpPairingComplete(ConnHdl, false, nullptr);
		return;
	}

	// Link is encrypted. Restore persisted CCCD subscriptions for a bonded peer
	// even when there is no active SMP pairing context. That is the normal
	// bonded-reconnect path when encryption starts from a stored LTK lookup.
	BtGattCccdRestoreBonded(ConnHdl);

	if (pLink == nullptr)
	{
		return;
	}

	if (pLink->Ctx.State == BT_SMP_STATE_LTK_WAIT)
	{
		// Encrypted via fresh pairing. Run the key distribution phase that was
		// negotiated. The local device distributes the keys it offered:
		// InitiatorKeyDist (PReq[6 of req = byte 5]) when we are the central,
		// ResponderKeyDist (PRsp[6]) when we are the peripheral. For SC the LTK
		// is derived (EncKey is never distributed); we send IRK + identity
		// address (IDKEY) and/or CSRK (SIGNKEY) only if negotiated.
		uint8_t localKeyDist = (pLink->Ctx.bInitiator ?
									pLink->Ctx.PReq[5] : pLink->Ctx.PRsp[6]) &
							   (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY);
		SMP_TRACE("SMP encrypted, distribute lk=%02x init=%d\r\n",
				  localKeyDist, pLink->Ctx.bInitiator ? 1 : 0);

		if (localKeyDist & BT_SMP_KEYDIST_IDKEY)
		{
			uint8_t irk[16];
			if (!BtSmpCryptoRand(irk, 16))
			{
				// Do not distribute a key drawn from a failed RNG. Skip the
				// identity key; the peer simply receives no IRK.
				SMP_TRACE("SMP IRK RNG failed, skipping id key\r\n");
			}
			else
			{
			BtSmpIdInfo_t idi;
			idi.Code = BT_SMP_CODE_PAIRING_ID_INFO;
			memcpy(idi.Irk, irk, 16);
			SmpSend(pDev, ConnHdl, &idi, sizeof(idi));

			uint8_t localAddr[6]; uint8_t localAddrType = 0;
			BtSmpLocalAddrGet(&localAddrType, localAddr);
			BtSmpIdAddrInfo_t iai;
			iai.Code = BT_SMP_CODE_PAIRING_ID_ADDR_INFO;
			iai.AddrType = localAddrType;
			memcpy(iai.Addr, localAddr, 6);
			SmpSend(pDev, ConnHdl, &iai, sizeof(iai));
			CryptoSecureWipe(irk, sizeof(irk));
			}
		}

		if (localKeyDist & BT_SMP_KEYDIST_SIGNKEY)
		{
			uint8_t csrk[16];
			if (!BtSmpCryptoRand(csrk, 16))
			{
				// Do not distribute a signing key drawn from a failed RNG.
				SMP_TRACE("SMP CSRK RNG failed, skipping sign key\r\n");
			}
			else
			{
			BtSmpSigningInfo_t si;
			si.Code = BT_SMP_CODE_PAIRING_SIGNING_INFO;
			memcpy(si.Csrk, csrk, 16);
			SmpSend(pDev, ConnHdl, &si, sizeof(si));
			CryptoSecureWipe(csrk, sizeof(csrk));
			}
		}

		// Compute which keys the peer will distribute in return. The negotiated
		// key-distribution fields live in the Pairing Response (byte 5 =
		// InitiatorKeyDist, byte 6 = ResponderKeyDist): the peer sends the field
		// for its own role. While those are still outstanding the link sits in
		// KEYDIST, the only state in which the H4 gate accepts inbound
		// key-distribution PDUs; once all have arrived SmpKeyDistReceived moves it
		// to DONE. If the peer distributes nothing, go straight to DONE.
		uint8_t peerKeyDist = (pLink->Ctx.bInitiator ?
									pLink->Ctx.PRsp[6] : pLink->Ctx.PRsp[5]) &
							   (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY);
		pLink->Ctx.KeyDistExp = peerKeyDist;

		BtSmpBondAdd(ConnHdl, &pLink->Keys);
		BtSmpPairingComplete(ConnHdl, true, &pLink->Keys);

		// OOB material is single use and is wiped after the link is secured.
		if (pLink->Ctx.Model == BT_SMP_MODEL_OOB)
		{
			SmpOobRelease(pLink);
		}

		pLink->Ctx.State = (peerKeyDist != 0) ? BT_SMP_STATE_KEYDIST
											  : BT_SMP_STATE_DONE;
	}
	else if (pLink->Ctx.State == BT_SMP_STATE_DONE)
	{
		// Encrypted from an existing bond (central reconnect). No key
		// distribution; just surface the secured link to the app.
		BtSmpPairingComplete(ConnHdl, true, &pLink->Keys);
	}
}

//-----------------------------------------------------------------------------
// Weak application hooks
//-----------------------------------------------------------------------------

__attribute__((weak))
void BtSmpPairingComplete(uint16_t ConnHdl, bool Success, const BtSmpKeys_t *pKeys)
{
	(void)ConnHdl;
	(void)Success;
	(void)pKeys;
}

__attribute__((weak))
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = 0;
	memset(pAddr, 0, 6);
}

__attribute__((weak))
bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv, uint8_t Ltk[16])
{
	(void)ConnHdl;
	(void)Rand;
	(void)Ediv;
	(void)Ltk;
	return false;
}

__attribute__((weak))
void BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	(void)ConnHdl;
	(void)pKeys;
}

__attribute__((weak))
void BtSmpBondClearAll(void)
{
}

//-----------------------------------------------------------------------------
// Crypto composition (three engine slots: ECDH, AES and RNG)
//
// SMP needs P-256 ECDH, AES-128 and security-grade randomness. bt_smp holds a
// KeyAgreeEngine, a CipherEngine and an RngEngine, bound by BtSmpInit from
// whatever engines the target provides: software (CryptoUecc + CryptoSoftAes),
// hardware (Ba414ep + CryptoMaster), or a mix (CryptoUecc for ECDH + the
// controller AES on a BLE-only part with no CryptoCell). The five BtSmpCrypto*
// wrappers keep their existing signatures so the state machine is unchanged;
// they translate CRYPTO_STATUS to the BT_SMP_CRYPTO_* codes the state machine
// expects. An async ECDH engine reports completion through the CryptoEngine
// completion handler, bound in BtSmpInit.
//-----------------------------------------------------------------------------

static int CryptoStatusToSmp(CRYPTO_STATUS st)
{
	switch (st)
	{
	case CRYPTO_STATUS_OK:      return BT_SMP_CRYPTO_OK;
	case CRYPTO_STATUS_PENDING: return BT_SMP_CRYPTO_PENDING;
	case CRYPTO_STATUS_BUSY:    return BT_SMP_CRYPTO_BUSY;
	default:                    return BT_SMP_CRYPTO_FAIL;
	}
}

void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
								  const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	(void)pDev;
	if (s_pCryptoAes == nullptr)
	{
		memset(Out, 0, 16);
		s_SmpAesFault = true;
		return;
	}

	// Build a plaintext AES-128 key for one ECB block. SMP always encrypts a
	// single 16-byte block (e, c1, s1, f4..f6 all reduce to AES-128-ECB). The
	// plain key points at the caller key buffer.
	CryptoKey key;
	key.Type       = CRYPTO_KEY_AES_128;
	key.Loc        = CRYPTO_KEY_LOC_PLAIN;
	key.Usage      = CRYPTO_KEY_USE_ENCRYPT;
	key.Plain.pData = Key;
	key.Plain.Len   = 16;

	CRYPTO_STATUS st = CRYPTO_STATUS_BUSY;
	for (uint32_t attempt = 0; attempt < BT_SMP_CRYPTO_BUSY_RETRIES &&
		 st == CRYPTO_STATUS_BUSY; attempt++)
	{
		st = s_pCryptoAes->Cipher(CRYPTO_CIPHER_ECB, 1, key,
								 nullptr, 0, In, 16, Out);
		__asm volatile("" ::: "memory");
	}
	if (st != CRYPTO_STATUS_OK)
	{
		memset(Out, 0, 16);
		s_SmpAesFault = true;
	}
}

static int SmpCryptoP256KeyGen(BtSmpLink_t *pLink, uint8_t pPubKey[64])
{
	if (s_pCryptoEcdh == nullptr || pLink == nullptr)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	// The private scalar lives in the per-link key context, held from KeyGen
	// until the matching Agree, then wiped by the engine.
	return CryptoStatusToSmp(s_pCryptoEcdh->KeyGen(CRYPTO_CURVE_P256,
												   pLink->Ctx.EcdhKeyCtx, pPubKey));
}

int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64])
{
	(void)pDev;
	// The public wrapper serves the OOB path, which keeps its own key pair. The
	// private scalar goes to the OOB key context, held until the OOB Agree.
	if (s_pCryptoEcdh == nullptr)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	return CryptoStatusToSmp(s_pCryptoEcdh->KeyGen(CRYPTO_CURVE_P256,
												   s_SmpOob.EcdhKeyCtx, pPubKey));
}

static int SmpCryptoEcdh(BtSmpLink_t *pLink,
						 const uint8_t pPeerPubKey[64], uint8_t pDhKey[32])
{
	if (s_pCryptoEcdh == nullptr || pLink == nullptr)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	return CryptoStatusToSmp(s_pCryptoEcdh->Agree(CRYPTO_CURVE_P256,
												  pLink->Ctx.EcdhKeyCtx,
												  pPeerPubKey, pDhKey));
}

int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32])
{
	(void)pDev;
	if (s_pCryptoEcdh == nullptr)
	{
		return BT_SMP_CRYPTO_FAIL;
	}
	return CryptoStatusToSmp(s_pCryptoEcdh->Agree(CRYPTO_CURVE_P256,
												  s_SmpOob.EcdhKeyCtx,
												  pPeerPubKey, pDhKey));
}

// Completion callback for an asynchronous ECDH engine. A synchronous engine
// never calls this; its KeyGen/Agree return OK and the caller drives the ready
// path inline. An async engine returns PENDING and calls this on completion,
// where the pending record identifies the link and the operation.
static void SmpCryptoComplete(CryptoEngine * const pEngine, CRYPTO_OP Op,
							  CRYPTO_STATUS Status, void *pCtx)
{
	(void)pEngine;
	if (pCtx != &s_SmpCryptoPending)
	{
		return;
	}
	uint8_t status = Status == CRYPTO_STATUS_OK ? 0 : 1;
	if (Op == CRYPTO_OP_KEYGEN &&
		s_SmpCryptoPending.Op == BT_SMP_CRYPTO_OP_PUBKEY)
	{
		BtSmpLink_t *pLink = SmpCryptoPendingFind(BT_SMP_CRYPTO_OP_PUBKEY);
		const uint8_t *pKey = pLink != nullptr ? pLink->Ctx.LocalPubKey : nullptr;
		BtSmpLocalPubKeyReady(s_SmpCryptoPending.pDev, status, pKey,
							  pKey != nullptr ? pKey + 32 : nullptr);
	}
	else if (Op == CRYPTO_OP_AGREE &&
			 s_SmpCryptoPending.Op == BT_SMP_CRYPTO_OP_DHKEY)
	{
		BtSmpLink_t *pLink = SmpCryptoPendingFind(BT_SMP_CRYPTO_OP_DHKEY);
		BtSmpDhKeyReady(s_SmpCryptoPending.pDev, status,
						 pLink != nullptr ? pLink->Ctx.DhKey : nullptr);
	}
}

static void SmpCryptoRetryPending(void)
{
	if (!s_SmpCryptoPending.bRetryBusy)
	{
		return;
	}
	BtSmpCryptoPending_t pending = s_SmpCryptoPending;
	BtSmpLink_t *pLink = SmpCryptoPendingFind(pending.Op);
	if (pLink == nullptr)
	{
		SmpCryptoPendingClear();
		return;
	}
	s_SmpCryptoPending.bRetryBusy = false;
	int rc = BT_SMP_CRYPTO_FAIL;
	if (pending.Op == BT_SMP_CRYPTO_OP_PUBKEY)
	{
		rc = SmpCryptoP256KeyGen(pLink, pLink->Ctx.LocalPubKey);
		if (rc == BT_SMP_CRYPTO_OK)
		{
			BtSmpLocalPubKeyReady(pending.pDev, 0,
							 pLink->Ctx.LocalPubKey, pLink->Ctx.LocalPubKey + 32);
			return;
		}
	}
	else if (pending.Op == BT_SMP_CRYPTO_OP_DHKEY)
	{
		rc = SmpCryptoEcdh(pLink, pLink->Ctx.PeerPubKey, pLink->Ctx.DhKey);
		if (rc == BT_SMP_CRYPTO_OK)
		{
			BtSmpDhKeyReady(pending.pDev, 0, pLink->Ctx.DhKey);
			return;
		}
	}
	if (rc == BT_SMP_CRYPTO_BUSY)
	{
		s_SmpCryptoPending.bRetryBusy = true;
		return;
	}
	if (rc == BT_SMP_CRYPTO_PENDING)
	{
		return;
	}
	if (pending.Op == BT_SMP_CRYPTO_OP_PUBKEY)
	{
		BtSmpLocalPubKeyReady(pending.pDev, 1, nullptr, nullptr);
	}
	else if (pending.Op == BT_SMP_CRYPTO_OP_DHKEY)
	{
		BtSmpDhKeyReady(pending.pDev, 1, nullptr);
	}
}

bool BtSmpCryptoRand(uint8_t *pBuf, size_t Len)
{
	// RNG is a target driver (crypto/icrypto.h), not a crypto engine. It is
	// backed by the MCU RNG peripheral; there is no software default. The result
	// must be checked: on nRF54 the non-blocking CRACEN lock can be held, so a
	// draw can fail, and a security-path caller must abort rather than proceed
	// with an unrandomized buffer.
	if (s_pCryptoRng == nullptr || !s_pCryptoRng->IsSecure())
	{
		// SMP nonces, passkeys, OOB randoms, IRK and CSRK need a secure RBG. A
		// deterministic engine (IsSecure() false) would make these predictable,
		// so refuse to draw and fail the pairing rather than weaken it.
		CryptoSecureWipe(pBuf, Len);
		return false;
	}
	CRYPTO_STATUS st = CRYPTO_STATUS_BUSY;
	for (uint32_t attempt = 0; attempt < BT_SMP_CRYPTO_BUSY_RETRIES &&
		 st != CRYPTO_STATUS_OK; attempt++)
	{
		st = s_pCryptoRng->Random(pBuf, Len);
		__asm volatile("" ::: "memory");
	}
	if (st != CRYPTO_STATUS_OK)
	{
		CryptoSecureWipe(pBuf, Len);
		return false;
	}
	return true;
}

int BtSmpCryptoSelfTest(void)
{
	// Run the ECDH engine's self-test (the security-critical path). AES/RNG
	// engines may have their own; the ECDH known-answer vector is the one that
	// matters for SC pairing.
	return s_pCryptoEcdh != nullptr ? s_pCryptoEcdh->SelfTest() : -1;
}

// LTK Request Reply hooks. The port implementation overrides these to route the
// reply through its own HCI command channel (e.g. the SDC command function).
// The weak default uses the generic HCI command builder, which is correct for
// transports where the HCI command and ACL data share one sink.

// Central-only: start link encryption from a (just-derived or stored) LTK via
// HCI LE Enable Encryption. The peripheral instead waits for the controller's
// LTK request and answers BtSmpHciLtkReply. For SC the Rand/Ediv are zero. The
// LTK is little-endian (HCI order). Weak: the port overrides this to use its own
// HCI command channel (the SDC command function), same as LtkReply.
__attribute__((weak))
void BtSmpHciEnableEncryption(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							 uint64_t Rand, uint16_t Ediv, const uint8_t Ltk[16])
{
	uint8_t param[28];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)(ConnHdl >> 8);
	for (int i = 0; i < 8; i++)
	{
		param[2 + i] = (uint8_t)(Rand >> (8 * i));
	}
	param[10] = (uint8_t)(Ediv & 0xFF);
	param[11] = (uint8_t)(Ediv >> 8);
	memcpy(&param[12], Ltk, 16);
	SmpSendHciCmd(pDev, BT_HCI_CMD_CTLR_ENABLE_ENCRYPTION, param, sizeof(param));
}

__attribute__((weak))
void BtSmpHciLtkReply(BtHciDevice_t * const pDev, uint16_t ConnHdl,
					  const uint8_t Ltk[16])
{
	uint8_t param[18];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)(ConnHdl >> 8);
	memcpy(&param[2], Ltk, 16);
	SmpSendHciCmd(pDev, BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_REPLY, param, sizeof(param));
}

__attribute__((weak))
void BtSmpHciLtkNegReply(BtHciDevice_t * const pDev, uint16_t ConnHdl)
{
	uint8_t param[2];
	param[0] = (uint8_t)(ConnHdl & 0xFF);
	param[1] = (uint8_t)(ConnHdl >> 8);
	SmpSendHciCmd(pDev, BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_NEG_REPLY, param, sizeof(param));
}

bool BtSmpInit(KeyAgreeEngine *pEcdh, CipherEngine *pAes, RngEngine *pRng)
{
	// Teardown first, through the OLD provider: contexts created by one
	// engine must be destroyed by that engine, never by its replacement (a
	// slot or opaque-key provider would leak its resource otherwise). The
	// old completion handler is detached before any pending state is
	// dropped so no stale callback can arrive mid swap.
	if (s_pCryptoEcdh != nullptr)
	{
		s_pCryptoEcdh->SetCompleteHandler(nullptr, nullptr);
	}
	SmpCryptoPendingClear();
	SmpOobClear();
	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		SmpEcdhCtxReset(s_SmpLink[i].Ctx.EcdhKeyCtx);
		CryptoSecureWipe(&s_SmpLink[i], sizeof(s_SmpLink[i]));
		s_SmpLink[i].ConnHdl = BT_CONN_HDL_INVALID;
	}

	// Compose the crypto from the engines the target provides. The per-link
	// and OOB key contexts are fixed CRYPTO_KEYCTX_MAX byte buffers; an
	// engine whose context does not fit is refused here so pairing fails
	// cleanly at the first Secure Connections attempt instead of overrunning
	// caller storage. The refusal is reported to the caller.
	bool accepted = true;
	if (pEcdh != nullptr &&
		(pEcdh->IsAsync() || pEcdh->KeyCtxSize() == 0U ||
		 pEcdh->KeyCtxSize() > CRYPTO_KEYCTX_MAX ||
		 pEcdh->KeyCtxAlign() == 0U ||
		 pEcdh->KeyCtxAlign() > CRYPTO_KEYCTX_ALIGN_MAX))
	{
		pEcdh = nullptr;
		accepted = false;
	}

	// AES-128-ECB is mandatory for SMP (e, c1, s1, f4..f6). A build with no AES
	// provider cannot pair; report the refusal so the caller fails at init
	// rather than silently at the first Secure Connections attempt. The runtime
	// AES fault guard still stops any pairing that reaches BtSmpCryptoAes128
	// with no provider.
	if (pAes == nullptr)
	{
		accepted = false;
	}

	// The RNG must be a secure RBG. A deterministic engine (IsSecure() false)
	// would make nonces, passkeys and distributed keys predictable, so refuse it
	// here and leave the slot empty; BtSmpCryptoRand then fails closed.
	if (pRng != nullptr && !pRng->IsSecure())
	{
		pRng = nullptr;
		accepted = false;
	}

	s_pCryptoEcdh = pEcdh;
	s_pCryptoRng = pRng;
	s_pCryptoAes  = pAes;

	// Bind the async completion handler. A synchronous ECDH engine never calls
	// it; an async engine (interrupt-driven hardware) reports KeyGen and Agree
	// completion through it, which drives BtSmpLocalPubKeyReady/BtSmpDhKeyReady.
	if (s_pCryptoEcdh != nullptr)
	{
		s_pCryptoEcdh->SetCompleteHandler(SmpCryptoComplete, &s_SmpCryptoPending);
	}

	// Repopulate the RAM bond table from non-volatile storage. The default
	// BtSmpBondLoad is a weak no-op (RAM-only); a flash-backed platform
	// overrides it and calls BtSmpBondRestore for each persisted slot.
	BtSmpBondLoad();
	return accepted;
}

// Weak: a port whose underlying stack owns pairing overrides this and
// routes the values into that stack security parameters instead.
__attribute__((weak))
void BtSmpAuthConfig(uint8_t IoCaps, uint8_t AuthReq)
{
	// Force the Secure Connections bit: this build does not pair with legacy,
	// so a caller that passes only bonding/MITM still negotiates SC.
	s_SmpIoCaps  = IoCaps;
	s_SmpAuthReq = (uint8_t)(AuthReq | BT_SMP_AUTHREQ_SC);
}

// Weak default for the Numeric Comparison user interaction. Unlike the event
// notification handlers, this one must drive the pairing to a conclusion: with
// no application display there is no way to perform the user check, so reject.
// An application advertising DisplayYesNo / KeyboardDisplay overrides this with
// a strong definition that displays Value and later calls the reply function.
__attribute__((weak)) void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value)
{
	(void)Value;
	BtSmpNumericComparisonReply(ConnHdl, false);
}

// Resume a Numeric Comparison pairing after the user has compared the value
// shown on both devices. Confirm true sends the held DHKey Check and raises the
// link to an authenticated level on success; Confirm false aborts with a
// Numeric Comparison Failed reason. Called from the application in response to
// the NumericComparison callback.
__attribute__((weak))
void BtSmpNumericComparisonReply(uint16_t ConnHdl, bool Confirm)
{
	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (pLink == nullptr ||
		pLink->Ctx.Model != BT_SMP_MODEL_NUMERIC_COMPARISON)
	{
		return;
	}


	// Derive the HCI device from this connection rather than the global active
	// device: the user reply is asynchronous and a concurrent link could have
	// moved s_pSmpActiveDev since the value was displayed.
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return;
	}
	BtHciDevice_t * const pDev = (BtHciDevice_t *)pPeer->pHciDev;

	if (!Confirm)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_NUMERIC_COMPARISON_FAILED);
		SmpAbortPairing(pLink);
		return;
	}

	// Values matched: the link is MITM authenticated.
	pLink->Ctx.bAuthenticated = true;

	uint8_t peerAddr[6];
	uint8_t peerAddrType;
	memcpy(peerAddr, pPeer->Conn.PeerAddr, 6);
	peerAddrType = pPeer->Conn.PeerAddrType;

	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);
	uint8_t zeroR[16] = {0};

	if (pLink->Ctx.bInitiator)
	{
		if (pLink->Ctx.State != BT_SMP_STATE_NUMERIC_WAIT)
		{
			return;
		}

		// Send the held initiator DHKey Check Ea.
		uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
		uint8_t ea[16];
		if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  zeroR, iocapA,
			  localAddrType, localAddr, peerAddrType, peerAddr, ea))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		BtSmpDhKeyCheck_t chk;
		chk.Code = BT_SMP_CODE_PAIRING_DHKEY_CHECK;
		memcpy(chk.Value, ea, 16);
		SmpSend(pDev, ConnHdl, &chk, sizeof(chk));

		pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
		return;
	}

	// Responder: send the held DHKey Check Eb only when the peer Ea has already
	// arrived and was verified (NUMERIC_WAIT). If Ea has not arrived yet the
	// DHKey Check handler sends Eb once it does, now that bAuthenticated is set.
	if (pLink->Ctx.State == BT_SMP_STATE_NUMERIC_WAIT)
	{
		uint8_t iocapB[3] = { pLink->Ctx.PRsp[1], pLink->Ctx.PRsp[2], pLink->Ctx.PRsp[3] };
		uint8_t eb[16];
		if (!SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  zeroR, iocapB,
			  localAddrType, localAddr, peerAddrType, peerAddr, eb))
		{
			SmpFailAndLock(s_pSmpActiveDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
			return;
		}

		BtSmpDhKeyCheck_t chk;
		chk.Code = BT_SMP_CODE_PAIRING_DHKEY_CHECK;
		memcpy(chk.Value, eb, 16);
		SmpSend(pDev, ConnHdl, &chk, sizeof(chk));

		pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
	}
}

// Weak default for the Passkey Entry display. The displaying device shows the
// generated 6 digit passkey so the user can enter it on the peer. With no
// application display there is nothing to show; pairing then fails at the per
// round Confirm check because the input side cannot match an unshown value. An
// application advertising a display overrides this with a strong definition.
__attribute__((weak)) void BtSmpPasskeyDisplay(uint16_t ConnHdl, uint32_t Passkey)
{
	(void)ConnHdl;
	(void)Passkey;
}

// Weak default for the Passkey Entry request. The inputting device must obtain
// the value the user reads from the peer. With no application input there is no
// way to enter it, so reject. An application with a keypad overrides this with
// a strong definition that prompts the user and calls BtSmpPasskeyReply.
__attribute__((weak)) void BtSmpPasskeyRequest(uint16_t ConnHdl)
{
	BtSmpPasskeyReply(ConnHdl, BT_SMP_PASSKEY_INVALID);
}

// Resume a Passkey Entry pairing after the user has entered the value shown on
// the peer. A value in 0..999999 drives the per round Confirm exchange; a value
// above that range (BT_SMP_PASSKEY_INVALID) cancels with a Passkey Entry Failed
// reason. Called from the application in response to BtSmpPasskeyRequest.
__attribute__((weak))
void BtSmpPasskeyReply(uint16_t ConnHdl, uint32_t Passkey)
{
	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (pLink == nullptr ||
		pLink->Ctx.Model != BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		return;
	}

	// Derive the HCI device from this connection rather than the global active
	// device: the user reply is asynchronous and a concurrent link could have
	// moved s_pSmpActiveDev since the passkey was requested.
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return;
	}
	BtHciDevice_t * const pDev = (BtHciDevice_t *)pPeer->pHciDev;

	if (Passkey > 999999u)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_PASSKEY_ENTRY_FAILED);
		SmpAbortPairing(pLink);
		return;
	}

	pLink->Ctx.Passkey = Passkey;
	pLink->Ctx.bPkReady = true;

	if (pLink->Ctx.bInitiator)
	{
		// Drive round 0 now that the passkey is known.
		if (pLink->Ctx.State == BT_SMP_STATE_PASSKEY_WAIT)
		{
			SmpPasskeySendInitiatorConfirm(pDev, pLink, ConnHdl);
		}
		return;
	}

	// Responder: if the initiator Confirm already arrived it was buffered;
	// answer it now with the round Confirm Cbi.
	if (pLink->Ctx.bPkPeerCommit)
	{
		pLink->Ctx.bPkPeerCommit = false;
		SmpPasskeyResponderConfirm(pDev, pLink, ConnHdl);
	}
}

// Peripheral-initiated security. Sends an SMP Security Request to prompt the
// central to either encrypt with an existing bond or start pairing. This is
// the standard way a peripheral signals that it wants a secure/bonded link;
// without it a central has no indication to pair (and apps like nRF Connect
// show no security/bond action), and any pairing the user forces has no
// natural begin/end in the central security flow.
// Weak: a port whose underlying stack owns pairing overrides this and
// maps it to that stack security request instead.
__attribute__((weak))
void BtSmpRequestSecurity(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return;
	}
	BtHciDevice_t *pDev = (BtHciDevice_t*)pPeer->pHciDev;
	s_pSmpActiveDev = pDev;

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (pLink == nullptr)
	{
		pLink = SmpLinkAlloc(ConnHdl);
		if (pLink == nullptr)
		{
			return;
		}
		pLink->ConnHdl = ConnHdl;
	}

	BtSmpSecurityReq_t req;
	req.Code = BT_SMP_CODE_PAIRING_SECURITY_REQ;
	req.AuthReq = (uint8_t)(s_SmpAuthReq & ~BT_SMP_AUTHREQ_KEYPRESS);
	SMP_TRACE("SMP TX SecurityReq auth=0x%02x\r\n", req.AuthReq);
	SmpSend(pDev, ConnHdl, &req, sizeof(req));
}

void BtSmpStartPairing(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return;
	}
	BtHciDevice_t *pDev = (BtHciDevice_t*)pPeer->pHciDev;
	s_pSmpActiveDev = pDev;

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (pLink == nullptr)
	{
		pLink = SmpLinkAlloc(ConnHdl);
		if (pLink == nullptr)
		{
			return;
		}
	}

	// Do not (re)start pairing on a link locked by an SMP timeout or too many
	// failed attempts; it stays locked until the connection drops.
	if (pLink->Ctx.bLocked)
	{
		return;
	}

	// Idempotent: if pairing/encryption already started on this link, do not
	// restart it (e.g. a Security Request arriving after we began on connect).
	if (pLink->Ctx.State != BT_SMP_STATE_IDLE)
	{
		return;
	}

	// Bonded reconnect: re-encrypt from a stored bond instead of pairing
	// again. An SC bond is matched by the peer address (its EDIV and Rand are
	// zero); a legacy bond supplies the EDIV/Rand for the encryption request.
	// The complete key record is restored so the link reports the stored
	// security properties (key size, authentication, SC) and the identity and
	// signing keys remain available. No key distribution runs on reconnect, so
	// go straight to DONE before enabling encryption; BtSmpEncryptionChanged
	// completes it.
	if (BtSmpBondKeysLookup(ConnHdl, 0U, 0U, &pLink->Keys))
	{
		pLink->Ctx.bInitiator = true;
		pLink->Ctx.State = BT_SMP_STATE_DONE;
		SMP_TRACE("SMP central reconnect, encrypt from bond\r\n");
		BtSmpHciEnableEncryption(pDev, ConnHdl, pLink->Keys.Rand,
								 pLink->Keys.Ediv, pLink->Keys.Ltk);
		return;
	}

	// Fresh pairing as initiator (central). Send Pairing Request and request the
	// local P-256 key. The responder's Pairing Response drives the public-key
	// send (SmpHandlePairingRsp / BtSmpLocalPubKeyReady). LE Secure Connections,
	// Just Works only.
	pLink->Ctx.bInitiator = true;
	pLink->Ctx.bSc = true;
	pLink->Ctx.TmrStart = BtSmpMsTick();	// anchor the SMP pairing timeout

	BtSmpPairingReq_t req;
	req.Code = BT_SMP_CODE_PAIRING_REQ;
	req.IOCaps = s_SmpIoCaps;
	req.OOBFlag = (SmpOobPeerReady(pLink) && (s_SmpAuthReq & BT_SMP_AUTHREQ_SC)) ?
				  BT_SMP_OOB_AUTH_PRESENT : BT_SMP_OOB_AUTH_NOT_PRESENT;
	req.AuthReq = (uint8_t)(s_SmpAuthReq & ~BT_SMP_AUTHREQ_KEYPRESS);
	req.MaxKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	req.InitiatorKeyDist = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;
	req.ResponderKeyDist = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	memcpy(pLink->Ctx.PReq, &req, 7);
	pLink->Ctx.IoCaps = req.IOCaps;
	pLink->Ctx.AuthReq = req.AuthReq;

	SMP_TRACE("SMP TX PairingReq (initiator) auth=0x%02x\r\n", req.AuthReq);
	SmpSend(pDev, ConnHdl, &req, sizeof(req));

	pLink->Ctx.State = BT_SMP_STATE_PUBKEY_LOCAL_WAIT;
	int rc = SmpLocalKeyGen(pDev, pLink);
	if (rc == BT_SMP_CRYPTO_FAIL)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
		SmpAbortPairing(pLink);
	}
}

__attribute__((weak))
int BtSmpOobLocalDataGen(BtHciDevice_t * const pDev, uint8_t * const pRand, uint8_t * const pConf)
{
	if (pRand == nullptr || pConf == nullptr || s_SmpOob.bReserved ||
		s_SmpCryptoPending.Op != BT_SMP_CRYPTO_OP_NONE ||
		s_pCryptoEcdh == nullptr || s_pCryptoEcdh->IsAsync())
	{
		return -1;
	}
	SmpOobClear();
	int rc = BtSmpCryptoP256KeyGen(pDev, s_SmpOob.LocalPubKey);
	if (rc != BT_SMP_CRYPTO_OK ||
		(s_pCryptoRng == nullptr || s_pCryptoRng->Random(s_SmpOob.LocalRand, sizeof(s_SmpOob.LocalRand)) != CRYPTO_STATUS_OK))
	{
		SmpOobClear();
		return -1;
	}
	uint8_t x[32];
	SmpP256CoordBeToSmpLe(&s_SmpOob.LocalPubKey[0], x);
	if (!SmpF4(x, x, s_SmpOob.LocalRand, 0, pConf))
	{
		return -1;
	}
	CryptoSecureWipe(x, sizeof(x));
	memcpy(pRand, s_SmpOob.LocalRand, 16);
	s_SmpOob.bLocalValid = true;
	return 0;
}

__attribute__((weak))
void BtSmpOobPeerDataSet(const uint8_t * const pRand, const uint8_t * const pConf)
{
	if (pRand == nullptr || pConf == nullptr || s_SmpOob.bReserved)
	{
		return;
	}
	CryptoSecureWipe(s_SmpOob.PeerRand, sizeof(s_SmpOob.PeerRand));
	CryptoSecureWipe(s_SmpOob.PeerConfirm, sizeof(s_SmpOob.PeerConfirm));
	memcpy(s_SmpOob.PeerRand, pRand, 16);
	memcpy(s_SmpOob.PeerConfirm, pConf, 16);
	s_SmpOob.bPeerValid = true;
}

__attribute__((weak))
void BtSmpOobDataClear(void)
{
	if (s_SmpOob.bReserved == false) { SmpOobClear(); }
}

void BtSmpDisconnected(uint16_t ConnHdl)
{
	SmpLinkFree(ConnHdl);
}

void BtSmpTimeoutCheck(void)
{
	SmpCryptoRetryPending();
	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		BtSmpLink_t *pLink = &s_SmpLink[i];

		if (pLink->ConnHdl == BT_CONN_HDL_INVALID || pLink->Ctx.bLocked)
		{
			continue;
		}

		if (SmpPairingTimedOut(pLink))
		{
			BtDevice_t *pPeer = BtPeerFindByHdl(pLink->ConnHdl);
			BtHciDevice_t *pDev = (pPeer != nullptr) ?
									(BtHciDevice_t*)pPeer->pHciDev : s_pSmpActiveDev;
			SMP_TRACE("SMP pairing timed out on hdl %d\r\n", pLink->ConnHdl);
			pLink->Ctx.FailCount++;
			SmpFailAndLock(pDev, pLink->ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED);
		}
	}
}

// f4 self-test against the BLE spec worked example (Vol 3 Part H, D.2).
// Verifies SmpF4 + AES-CMAC + AES are correct on the target. Returns 0 on PASS.
int BtSmpF4SelfTest(void)
{
	// BLE spec worked example (Vol 3 Part H, D.2). The toolbox f4 takes its
	// inputs and returns its output in little-endian (wire) order, swapping to
	// big-endian internally for AES-CMAC. The spec states these values
	// big-endian, so feed them reversed and expect the reversed output.
	static const uint8_t U[32] = {	// reversed spec U
		0xe6,0x9d,0x35,0x0e,0x48,0x01,0x03,0xcc,0xdb,0xfd,0xf4,0xac,0x11,0x91,0xf4,0xef,
		0xb9,0xa5,0xf9,0xe9,0xa7,0x83,0x2c,0x5e,0x2c,0xbe,0x97,0xf2,0xd2,0x03,0xb0,0x20 };
	static const uint8_t V[32] = {	// reversed spec V
		0xfd,0xc5,0x7f,0xf4,0x49,0xdd,0x4f,0x6b,0xfb,0x7c,0x9d,0xf1,0xc2,0x9a,0xcb,0x59,
		0x2a,0xe7,0xd4,0xee,0xfb,0xfc,0x0a,0x90,0x9a,0xbb,0xf6,0x32,0x3d,0x8b,0x18,0x55 };
	static const uint8_t X[16] = {	// reversed spec X
		0xab,0xae,0x2b,0x71,0xec,0xb2,0xff,0xff,0x3e,0x73,0x77,0xd1,0x54,0x84,0xcb,0xd5 };
	static const uint8_t expect[16] = {	// reversed spec result
		0x2d,0x87,0x74,0xa9,0xbe,0xa1,0xed,0xf1,0x1c,0xbd,0xa9,0x07,0xf1,0x16,0xc9,0xf2 };
	uint8_t out[16];
	if (!SmpF4(U, V, X, 0, out))
	{
		return -1;
	}
	return memcmp(out, expect, 16) == 0 ? 0 : -1;
}

// Resolve a resolvable private address against a peer IRK with the ah function
// (Core spec Vol 3 Part H 2.2.2). Rpa is a 6-byte BD_ADDR in little-endian wire
// order: Rpa[3..5] are prand (random part, top two bits 0b01), Rpa[0..2] are the
// hash. Returns true when Rpa was generated from Irk. Byte order matches SmpAes
// and is checked by BtSmpRpaSelfTest against the spec ah sample.
bool BtSmpRpaResolve(const uint8_t Irk[16], const uint8_t Rpa[6])
{
	if (Irk == nullptr || Rpa == nullptr)
	{
		return false;
	}

	// No IRK was distributed for this peer: nothing to resolve against.
	bool present = false;
	for (int i = 0; i < 16; i++)
	{
		if (Irk[i] != 0)
		{
			present = true;
			break;
		}
	}
	if (present == false)
	{
		return false;
	}

	uint8_t key[16];
	uint8_t rp[16];
	uint8_t out[16];

	// SmpAes is standard big-endian. The stored IRK and the address are little-
	// endian (wire order), so reverse the IRK into the key and place prand in the
	// low 24 bits big-endian.
	for (int i = 0; i < 16; i++)
	{
		key[i] = Irk[15 - i];
	}
	memset(rp, 0, sizeof(rp));
	rp[13] = Rpa[5];		// prand, most significant byte first
	rp[14] = Rpa[4];
	rp[15] = Rpa[3];

	SmpAes(key, rp, out);	// ah = e(IRK, prand'), low 24 bits

	// Hash is the low 24 bits of the address (Rpa[0..2], little-endian); compare
	// to the cipher's low 24 bits (out[13..15], big-endian).
	return out[13] == Rpa[2] && out[14] == Rpa[1] && out[15] == Rpa[0];
}

int BtSmpRpaSelfTest(void)
{
	// Core spec ah sample (Vol 3 Part H): IRK ec0234a3..0a397d9b, prand 0x708194,
	// hash 0x0dfbaa, stored little-endian here to match SmpAes. The address holds
	// hash(0dfbaa) in Rpa[0..2] and prand(708194) in Rpa[3..5].
	static const uint8_t irk[16] = {
		0x9b,0x7d,0x39,0x0a,0xa6,0x10,0x10,0x34,
		0x05,0xad,0xc8,0x57,0xa3,0x34,0x02,0xec };
	static const uint8_t rpa[6] = {
		0xaa,0xfb,0x0d,0x94,0x81,0x70 };

	return BtSmpRpaResolve(irk, rpa) ? 0 : -1;
}

// Compute the 8-byte data-signing MAC over pMsg (the signed ATT data followed by
// the 4-byte SignCounter, all in wire/little-endian order) with Csrk (stored
// little-endian). Writes the little-endian MAC to Mac (Core spec Vol 3 Part H
// 2.4.5). SmpAesCmac is standard big-endian, so the key is reversed in and the
// leading 8 bytes of the CMAC are reversed back to wire order.
void BtSmpSignMac(const uint8_t Csrk[16], const uint8_t *pMsg,
							 size_t Len, uint8_t Mac[8])
{
	uint8_t key[16];
	uint8_t mac[16];

	for (int i = 0; i < 16; i++)
	{
		key[i] = Csrk[15 - i];
	}

	SmpAesCmac(key, pMsg, Len, mac);

	for (int i = 0; i < 8; i++)
	{
		Mac[i] = mac[7 - i];
	}
}

int BtSmpSignSelfTest(void)
{
	// Implementation check for BtSmpSignMac byte order using the documented
	// signing convention. CSRK little-endian 01..10; signed message is
	// opcode(0xD2) || handle(0x0003) || value(0x55) || SignCounter(0).
	static const uint8_t csrk[16] = {
		0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
		0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10 };
	static const uint8_t msg[8] = {
		0xd2,0x03,0x00,0x55,0x00,0x00,0x00,0x00 };
	static const uint8_t expect[8] = {
		0x28,0xda,0x5d,0x44,0x8a,0xce,0x8e,0xd5 };
	uint8_t mac[8];

	BtSmpSignMac(csrk, msg, sizeof(msg), mac);

	return memcmp(mac, expect, 8) == 0 ? 0 : -1;
}
