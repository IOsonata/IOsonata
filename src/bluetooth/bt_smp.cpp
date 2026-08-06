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

#include "crc.h"

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gatt.h"

/******** For DEBUG Trace ************/
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
static const char *SmpCodeName(uint8_t c);
#define SMP_TRACE_PDU(dir, code, state) \
		DEBUG_PRINTF("SMP " dir " %s state=%d\r\n", SmpCodeName(code), state)
#else
#define DEBUG_PRINTF(...)
#define SMP_TRACE_PDU(dir, code, state)
#endif

#ifndef BT_SMP_DHKEY_ORDER_FALLBACK
#define BT_SMP_DHKEY_ORDER_FALLBACK 0
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

typedef enum __Bt_Smp_Crypto_Op {
	BT_SMP_CRYPTO_OP_NONE = 0,
	BT_SMP_CRYPTO_OP_PUBKEY,
	BT_SMP_CRYPTO_OP_DHKEY,
} BT_SMP_CRYPTO_OP;

typedef struct __Bt_Smp_Link {
	uint16_t    ConnHdl;
	uint32_t    Generation;
	BtSmpCtx_t  Ctx;
	BtSmpKeys_t Keys;
	BT_SMP_CRYPTO_OP CryptoOp;
	bool             bCryptoWait;
	bool             bRetryBusy;
	BtHciDevice_t   *pCryptoDev;
	bool             bTimedOut;
	bool             bLocalKeyDistSent;
} BtSmpLink_t;

static BtSmpLink_t s_SmpLink[BT_SMP_MAX_LINK];
static uint32_t s_SmpGeneration;
static BtHciDevice_t *s_pSmpActiveDev = nullptr;
static uint8_t s_SmpIoCaps  = BT_SMP_LOCAL_IOCAPS;
static uint8_t s_SmpAuthReq = BT_SMP_LOCAL_AUTHREQ;

static struct {
	bool bLocalValid;
	bool bPeerValid;
	bool bReserved;
	uint16_t ConnHdl;
	uint32_t Generation;
	uint8_t LocalRand[16];
	uint8_t LocalPubKey[64];
	uint8_t EcdhKeyCtx[CRYPTO_KEYCTX_MAX] CRYPTO_ALIGNED(CRYPTO_KEYCTX_ALIGN_MAX);
	uint8_t PeerRand[16];
	uint8_t PeerConfirm[16];
} s_SmpOob = {};

typedef struct __Bt_Smp_Crypto_Inflight {
	BT_SMP_CRYPTO_OP Op;
	uint16_t ConnHdl;
	uint32_t Generation;
	BtHciDevice_t *pDev;
} BtSmpCryptoInflight_t;

static BtSmpCryptoInflight_t s_CryptoInflight = {};

static const uint8_t s_SmpModelMap[5][5] = {
	{ 0, 0, 2, 0, 2 },
	{ 0, 1, 2, 0, 1 },
	{ 2, 2, 2, 0, 2 },
	{ 0, 0, 0, 0, 0 },
	{ 2, 1, 2, 0, 1 },
};

static KeyAgreeEngine *s_pCryptoEcdh = nullptr;
static RngEngine *s_pCryptoRng = nullptr;
static CipherEngine *s_pCryptoAes = nullptr;
static bool s_SmpAesFault = false;

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

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
static const char *SmpCodeName(uint8_t c)
{
	switch (c)
	{
		case BT_SMP_CODE_PAIRING_REQ: return "PairingReq";
		case BT_SMP_CODE_PAIRING_RSP: return "PairingRsp";
		case BT_SMP_CODE_PAIRING_CONFIRM: return "Confirm";
		case BT_SMP_CODE_PAIRING_RANDOM: return "Random";
		case BT_SMP_CODE_PAIRING_FAILED: return "Failed";
		case BT_SMP_CODE_PAIRING_ENCRYP_INFO: return "EncInfo";
		case BT_SMP_CODE_PAIRING_CENTRAL_ID: return "CentralId";
		case BT_SMP_CODE_PAIRING_ID_INFO: return "IdInfo";
		case BT_SMP_CODE_PAIRING_ID_ADDR_INFO: return "IdAddrInfo";
		case BT_SMP_CODE_PAIRING_SIGNING_INFO: return "SigningInfo";
		case BT_SMP_CODE_PAIRING_SECURITY_REQ: return "SecurityReq";
		case BT_SMP_CODE_PAIRING_PUBLIC_KEY: return "PublicKey";
		case BT_SMP_CODE_PAIRING_DHKEY_CHECK: return "DhKeyCheck";
		case BT_SMP_CODE_PAIRING_KEYPRESS_NOTIF: return "Keypress";
		default: return "?";
	}
}
#endif

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
	return !s_SmpOob.bReserved || SmpOobReservedFor(pLink);
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
	if (pLink == nullptr || !SmpOobAvailable(pLink))
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

static void SmpCryptoPump(void);
static int SmpCryptoStart(BtSmpLink_t *pLink);

static void SmpCryptoLinkClear(BtSmpLink_t *pLink)
{
	if (pLink == nullptr)
	{
		return;
	}
	if (s_CryptoInflight.Op != BT_SMP_CRYPTO_OP_NONE &&
		s_CryptoInflight.ConnHdl == pLink->ConnHdl &&
		s_CryptoInflight.Generation == pLink->Generation)
	{
		memset(&s_CryptoInflight, 0, sizeof(s_CryptoInflight));
	}
	pLink->CryptoOp = BT_SMP_CRYPTO_OP_NONE;
	pLink->bCryptoWait = false;
	pLink->bRetryBusy = false;
	pLink->pCryptoDev = nullptr;
}

static BtSmpLink_t *SmpCryptoInflightLink(BT_SMP_CRYPTO_OP Op)
{
	if (s_CryptoInflight.Op != Op)
	{
		return nullptr;
	}
	BtSmpLink_t *pLink = SmpLinkFind(s_CryptoInflight.ConnHdl);
	return pLink != nullptr && pLink->Generation == s_CryptoInflight.Generation ? pLink : nullptr;
}

static BtSmpLink_t *SmpCryptoInflightTake(BT_SMP_CRYPTO_OP Op)
{
	BtSmpLink_t *pLink = SmpCryptoInflightLink(Op);
	memset(&s_CryptoInflight, 0, sizeof(s_CryptoInflight));
	if (pLink != nullptr)
	{
		pLink->CryptoOp = BT_SMP_CRYPTO_OP_NONE;
		pLink->bCryptoWait = false;
		pLink->bRetryBusy = false;
		pLink->pCryptoDev = nullptr;
	}
	return pLink;
}

static bool SmpCryptoRequest(BtSmpLink_t *pLink, BtHciDevice_t *pDev,
		BT_SMP_CRYPTO_OP Op)
{
	if (pLink == nullptr || Op == BT_SMP_CRYPTO_OP_NONE ||
		pLink->CryptoOp != BT_SMP_CRYPTO_OP_NONE)
	{
		return false;
	}
	pLink->CryptoOp = Op;
	pLink->bCryptoWait = true;
	pLink->bRetryBusy = false;
	pLink->pCryptoDev = pDev;
	return true;
}

static void SmpCryptoPendingClear(void)
{
	memset(&s_CryptoInflight, 0, sizeof(s_CryptoInflight));
	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		s_SmpLink[i].CryptoOp = BT_SMP_CRYPTO_OP_NONE;
		s_SmpLink[i].bCryptoWait = false;
		s_SmpLink[i].bRetryBusy = false;
		s_SmpLink[i].pCryptoDev = nullptr;
	}
}

static void SmpAbortPairing(BtSmpLink_t *pLink)
{
	SmpOobRelease(pLink);
	bool ownedEngine = s_CryptoInflight.Op != BT_SMP_CRYPTO_OP_NONE &&
		s_CryptoInflight.ConnHdl == pLink->ConnHdl &&
		s_CryptoInflight.Generation == pLink->Generation;
	SmpCryptoLinkClear(pLink);
	SmpEcdhCtxReset(pLink->Ctx.EcdhKeyCtx);
	bool locked = pLink->Ctx.bLocked;
	uint8_t failCount = pLink->Ctx.FailCount;
	CryptoSecureWipe(&pLink->Ctx, sizeof(pLink->Ctx));
	pLink->Ctx.State = BT_SMP_STATE_IDLE;
	pLink->Ctx.bLocked = locked;
	pLink->Ctx.FailCount = failCount;
	pLink->bLocalKeyDistSent = false;
	if (ownedEngine)
	{
		SmpCryptoPump();
	}
}

static void SmpAuthFailCount(BtSmpLink_t *pLink)
{
	pLink->Ctx.FailCount++;
	if (pLink->Ctx.FailCount >= BT_SMP_MAX_PAIR_ATTEMPTS)
	{
		pLink->Ctx.bLocked = true;
	}
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
		bool ownedEngine = s_CryptoInflight.Op != BT_SMP_CRYPTO_OP_NONE &&
			s_CryptoInflight.ConnHdl == p->ConnHdl &&
			s_CryptoInflight.Generation == p->Generation;
		SmpOobRelease(p);
		SmpEcdhCtxReset(p->Ctx.EcdhKeyCtx);
		CryptoSecureWipe(p, sizeof(BtSmpLink_t));
		p->ConnHdl = BT_CONN_HDL_INVALID;
		if (ownedEngine)
		{
			memset(&s_CryptoInflight, 0, sizeof(s_CryptoInflight));
			SmpCryptoPump();
		}
	}
}

static void SmpLinkResetKeepCount(BtSmpLink_t *pLink)
{
	uint16_t hdl = pLink->ConnHdl;
	uint32_t generation = pLink->Generation;
	uint8_t fc = pLink->Ctx.FailCount;
	bool locked = pLink->Ctx.bLocked;
	SmpOobRelease(pLink);
	SmpEcdhCtxReset(pLink->Ctx.EcdhKeyCtx);
	CryptoSecureWipe(&pLink->Ctx, sizeof(pLink->Ctx));
	pLink->ConnHdl = hdl;
	pLink->Generation = generation;
	pLink->Ctx.FailCount = fc;
	pLink->Ctx.bLocked = locked;
	pLink->Ctx.State = BT_SMP_STATE_IDLE;
	pLink->bTimedOut = false;
	pLink->bLocalKeyDistSent = false;
}

__attribute__((weak)) uint32_t BtSmpMsTick(void)
{
	return 0;
}

static inline bool SmpPairingActive(BtSmpState_t State)
{
	return State != BT_SMP_STATE_IDLE &&
		State != BT_SMP_STATE_KEYDIST && State != BT_SMP_STATE_DONE;
}

static inline bool SmpPairingTimedOut(const BtSmpLink_t *pLink)
{
	return SmpPairingActive(pLink->Ctx.State) &&
		(uint32_t)(BtSmpMsTick() - pLink->Ctx.TmrStart) >= BT_SMP_TIMEOUT_MS;
}

static void SmpFailAndLock(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		BtSmpLink_t *pLink, uint8_t Reason)
{
	SmpSendFailed(pDev, ConnHdl, Reason);
	SmpAbortPairing(pLink);
	pLink->Ctx.bLocked = true;
	BtSmpPairingComplete(ConnHdl, false, nullptr);
}

static void SmpTimeoutAbort(BtSmpLink_t *pLink)
{
	if (pLink == nullptr)
	{
		return;
	}
	uint16_t connHdl = pLink->ConnHdl;
	SmpAbortPairing(pLink);
	CryptoSecureWipe(&pLink->Keys, sizeof(pLink->Keys));
	pLink->Ctx.bLocked = true;
	pLink->bTimedOut = true;
	BtSmpPairingComplete(connHdl, false, nullptr);
}

static void SmpAbortOffPhase(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		BtSmpLink_t *pLink)
{
	SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
	SmpAbortPairing(pLink);
	BtSmpPairingComplete(ConnHdl, false, nullptr);
}

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
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2->Hdr.Cid = BT_L2CAP_CID_SEC_MNGR;
	l2->Hdr.Len = (uint16_t)Len;
	memcpy(&l2->Smp, pData, Len);
	acl->Hdr.Len = (uint16_t)(Len + sizeof(BtL2CapHdr_t));
	BtHciSendAcl(pDev, acl);
	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (pLink != nullptr)
	{
		pLink->Ctx.TmrStart = BtSmpMsTick();
	}
	SMP_TRACE_PDU("TX", ((const uint8_t*)pData)[0], pLink ? (int)pLink->Ctx.State : -1);
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

static void SmpAesCmac(const uint8_t Key[16], const uint8_t *pMsg, size_t Len,
		uint8_t Mac[16])
{
	static const uint8_t Zero[16] = {0};
	uint8_t l[16], k1[16], k2[16];
	SmpAes(Key, Zero, l);
	SmpLeftShiftOne(l, k1);
	if (l[0] & 0x80) { k1[15] ^= 0x87; }
	SmpLeftShiftOne(k1, k2);
	if (k1[0] & 0x80) { k2[15] ^= 0x87; }
	size_t n = (Len + 15) / 16;
	bool lastComplete;
	if (n == 0) { n = 1; lastComplete = false; }
	else { lastComplete = (Len % 16) == 0; }
	uint8_t x[16] = {0}, y[16];
	for (size_t i = 0; i < n - 1; i++)
	{
		for (int j = 0; j < 16; j++) { y[j] = x[j] ^ pMsg[i * 16 + j]; }
		SmpAes(Key, y, x);
	}
	uint8_t last[16];
	size_t rem = Len - (n - 1) * 16;
	if (lastComplete)
	{
		for (int j = 0; j < 16; j++) { last[j] = pMsg[(n - 1) * 16 + j] ^ k1[j]; }
	}
	else
	{
		for (size_t j = 0; j < 16; j++)
		{
			uint8_t mb = j < rem ? pMsg[(n - 1) * 16 + j] : (j == rem ? 0x80 : 0);
			last[j] = mb ^ k2[j];
		}
	}
	for (int j = 0; j < 16; j++) { y[j] = x[j] ^ last[j]; }
	SmpAes(Key, y, Mac);
}

static void SmpReverse16(const uint8_t in[16], uint8_t out[16])
{
	for (int i = 0; i < 16; i++) { out[i] = in[15 - i]; }
}
static void SmpReverse32(const uint8_t in[32], uint8_t out[32])
{
	for (int i = 0; i < 32; i++) { out[i] = in[31 - i]; }
}
static void SmpReverse6(const uint8_t in[6], uint8_t out[6])
{
	for (int i = 0; i < 6; i++) { out[i] = in[5 - i]; }
}

static bool SmpC1(const uint8_t k[16], const uint8_t r[16],
		const uint8_t preq[7], const uint8_t pres[7], uint8_t iat,
		const uint8_t ia[6], uint8_t rat, const uint8_t ra[6], uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t ks[16], p1[16], p2[16], tmp[16], blk[16];
	p1[0] = iat; p1[1] = rat;
	memcpy(&p1[2], preq, 7); memcpy(&p1[9], pres, 7);
	memset(p2, 0, 16); memcpy(&p2[0], ra, 6); memcpy(&p2[6], ia, 6);
	SmpReverse16(k, ks);
	for (int i = 0; i < 16; i++) { tmp[i] = r[i] ^ p1[i]; }
	SmpReverse16(tmp, blk); SmpAes(ks, blk, blk); SmpReverse16(blk, tmp);
	for (int i = 0; i < 16; i++) { tmp[i] ^= p2[i]; }
	SmpReverse16(tmp, blk); SmpAes(ks, blk, blk); SmpReverse16(blk, out);
	return !s_SmpAesFault;
}

static bool SmpS1(const uint8_t k[16], const uint8_t r1[16],
		const uint8_t r2[16], uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t ks[16], r[16], blk[16];
	memcpy(&r[0], &r2[0], 8); memcpy(&r[8], &r1[0], 8);
	SmpReverse16(k, ks); SmpReverse16(r, blk); SmpAes(ks, blk, blk); SmpReverse16(blk, out);
	return !s_SmpAesFault;
}

static bool SmpF4(const uint8_t u[32], const uint8_t v[32],
		const uint8_t x[16], uint8_t z, uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t m[65], xs[16], mac[16];
	for (int i = 0; i < 32; i++) { m[i] = u[31 - i]; m[32 + i] = v[31 - i]; }
	m[64] = z; SmpReverse16(x, xs); SmpAesCmac(xs, m, sizeof(m), mac); SmpReverse16(mac, out);
	return !s_SmpAesFault;
}

static void SmpP256CoordBeToSmpLe(const uint8_t be[32], uint8_t le[32])
{
	for (int i = 0; i < 32; i++) { le[i] = be[31 - i]; }
}

static bool SmpF5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16],
		uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
		uint8_t mackey[16], uint8_t ltk[16])
{
	s_SmpAesFault = false;
	static const uint8_t salt[16] = {0x6C,0x88,0x83,0x91,0xAA,0xF5,0xA5,0x38,0x60,0x37,0x0B,0xDB,0x5A,0x60,0x83,0xBE};
	uint8_t ws[32], t[16]; SmpReverse32(w, ws); SmpAesCmac(salt, ws, 32, t);
	uint8_t m[53] = {0x00,0x62,0x74,0x6C,0x65};
	SmpReverse16(n1, &m[5]); SmpReverse16(n2, &m[21]);
	m[37] = a1t; SmpReverse6(a1, &m[38]); m[44] = a2t; SmpReverse6(a2, &m[45]);
	m[51] = 0x01; m[52] = 0x00;
	uint8_t tmp[16]; m[0] = 0; SmpAesCmac(t, m, sizeof(m), tmp); SmpReverse16(tmp, mackey);
	m[0] = 1; SmpAesCmac(t, m, sizeof(m), tmp); SmpReverse16(tmp, ltk);
	return !s_SmpAesFault;
}

static bool SmpF6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16],
		const uint8_t r[16], const uint8_t iocap[3], uint8_t a1t,
		const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6], uint8_t out[16])
{
	s_SmpAesFault = false;
	uint8_t ws[16], m[65], mac[16]; SmpReverse16(w, ws);
	SmpReverse16(n1, &m[0]); SmpReverse16(n2, &m[16]); SmpReverse16(r, &m[32]);
	m[48] = iocap[2]; m[49] = iocap[1]; m[50] = iocap[0];
	m[51] = a1t; SmpReverse6(a1, &m[52]); m[58] = a2t; SmpReverse6(a2, &m[59]);
	SmpAesCmac(ws, m, sizeof(m), mac); SmpReverse16(mac, out);
	return !s_SmpAesFault;
}

static void SmpSendLocalPubKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink, uint16_t ConnHdl);

static uint8_t SmpSelectModel(uint8_t InitIo, uint8_t RespIo, bool Mitm, bool Oob)
{
	if (Oob) return BT_SMP_MODEL_OOB;
	if (!Mitm) return BT_SMP_MODEL_JUST_WORKS;
	if (InitIo > BT_SMP_IOCAPS_KEYBOARD_DISPLAY || RespIo > BT_SMP_IOCAPS_KEYBOARD_DISPLAY)
		return BT_SMP_MODEL_JUST_WORKS;
	return s_SmpModelMap[InitIo][RespIo];
}

static bool SmpOobCtxLoad(BtSmpLink_t *pLink)
{
	if (!s_SmpOob.bLocalValid && !s_SmpOob.bPeerValid) return false;
	if (!SmpOobReserve(pLink)) return false;
	if (s_SmpOob.bLocalValid) memcpy(pLink->Ctx.OobLocalRand, s_SmpOob.LocalRand, 16);
	if (s_SmpOob.bPeerValid)
	{
		pLink->Ctx.bOobPeerData = true;
		memcpy(pLink->Ctx.OobPeerRand, s_SmpOob.PeerRand, 16);
		memcpy(pLink->Ctx.OobPeerConfirm, s_SmpOob.PeerConfirm, 16);
	}
	return true;
}

static int SmpLocalKeyGen(BtHciDevice_t * const pDev, BtSmpLink_t *pLink)
{
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && SmpOobLocalReady(pLink))
	{
		memcpy(pLink->Ctx.LocalPubKey, s_SmpOob.LocalPubKey, 64);
		return BT_SMP_CRYPTO_OK;
	}
	if (s_SmpOob.bLocalValid || !SmpCryptoRequest(pLink, pDev, BT_SMP_CRYPTO_OP_PUBKEY))
		return BT_SMP_CRYPTO_FAIL;
	int rc = SmpCryptoStart(pLink);
	return rc == BT_SMP_CRYPTO_FAIL ? BT_SMP_CRYPTO_FAIL : BT_SMP_CRYPTO_PENDING;
}

static uint32_t SmpG2(const uint8_t u[32], const uint8_t v[32],
		const uint8_t x[16], const uint8_t y[16])
{
	uint8_t m[80], xs[16], mac[16];
	for (int i = 0; i < 32; i++) { m[i] = u[31 - i]; m[32+i] = v[31-i]; }
	for (int i = 0; i < 16; i++) { m[64+i] = y[15-i]; }
	SmpReverse16(x, xs); SmpAesCmac(xs, m, sizeof(m), mac);
	return ((uint32_t)mac[12] << 24) | ((uint32_t)mac[13] << 16) | ((uint32_t)mac[14] << 8) | mac[15];
}

static bool SmpNumericValue(BtSmpLink_t *pLink, uint32_t *pValue)
{
	s_SmpAesFault = false;
	uint8_t pkaX[32], pkbX[32]; const uint8_t *na, *nb;
	if (pLink->Ctx.bInitiator)
	{
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], pkaX);
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], pkbX);
		na = pLink->Ctx.LocalRand; nb = pLink->Ctx.PeerRand;
	}
	else
	{
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], pkaX);
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], pkbX);
		na = pLink->Ctx.PeerRand; nb = pLink->Ctx.LocalRand;
	}
	*pValue = SmpG2(pkaX, pkbX, na, nb) % 1000000;
	return !s_SmpAesFault;
}

static void SmpBuildPairingRsp(BtSmpLink_t *pLink, BtSmpPairingRsp_t *pRsp)
{
	pRsp->Code = BT_SMP_CODE_PAIRING_RSP;
	pRsp->IOCaps = s_SmpIoCaps;
	pRsp->OOBFlag = SmpOobPeerReady(pLink) && pLink->Ctx.bSc ? BT_SMP_OOB_AUTH_PRESENT : BT_SMP_OOB_AUTH_NOT_PRESENT;
	pRsp->AuthReq = (uint8_t)(s_SmpAuthReq & ~BT_SMP_AUTHREQ_KEYPRESS);
	pRsp->MaxKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	const BtSmpPairingReq_t *pReq = (const BtSmpPairingReq_t*)pLink->Ctx.PReq;
	uint8_t supported = BT_SMP_KEYDIST_ENCKEY | BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;
	pRsp->InitiatorKeyDist = pReq->InitiatorKeyDist & supported;
	pRsp->ResponderKeyDist = pReq->ResponderKeyDist & supported;
	pLink->Ctx.IoCaps = pRsp->IOCaps;
	pLink->Ctx.AuthReq = pRsp->AuthReq;
	memcpy(pLink->Ctx.PRsp, pRsp, 7);
}

static bool SmpIsAllZero(const uint8_t *p, size_t len)
{
	uint8_t acc = 0; for (size_t i = 0; i < len; i++) acc |= p[i]; return acc == 0;
}
static bool SmpEqualCT(const uint8_t *a, const uint8_t *b, size_t len)
{
	uint8_t diff = 0; for (size_t i = 0; i < len; i++) diff |= (uint8_t)(a[i] ^ b[i]); return diff == 0;
}
static void SmpApplyKeySize(uint8_t Ltk[16], uint8_t EncKeySize)
{
	int ks = EncKeySize < BT_SMP_MIN_ENC_KEY_SIZE ? BT_SMP_MIN_ENC_KEY_SIZE : EncKeySize;
	for (int i = ks; i < 16; i++) Ltk[i] = 0;
}
static void SmpCommitPendingKeys(BtSmpLink_t *pLink)
{
	BtSmpKeys_t keys = {}; memcpy(keys.Ltk, pLink->Ctx.Ltk, 16);
	keys.EncKeySize = pLink->Ctx.EncKeySize; keys.bAuthenticated = pLink->Ctx.bAuthenticated;
	keys.bSc = pLink->Ctx.bSc; keys.bValid = true; pLink->Keys = keys;
}

static void SmpHandlePairingReq(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
		uint16_t ConnHdl, const BtSmpPairingReq_t *pReq)
{
	BtDevice_t *pAlready = BtPeerFindByHdl(ConnHdl);
	if (pLink->Ctx.State == BT_SMP_STATE_DONE || (pAlready != nullptr && pAlready->bSecure))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED); return;
	}
	if (pReq->IOCaps > BT_SMP_IOCAPS_KEYBOARD_DISPLAY || pReq->OOBFlag > BT_SMP_OOB_AUTH_PRESENT)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_INVALID_PARAMS); SmpAbortPairing(pLink); return;
	}
	if (pReq->MaxKeySize < BT_SMP_CFG_MIN_ENC_KEY_SIZE || pReq->MaxKeySize > BT_SMP_MAX_ENC_KEY_SIZE)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_ENC_KEY_SIZE); SmpAbortPairing(pLink); return;
	}
	memcpy(pLink->Ctx.PReq, pReq, 7); pLink->Ctx.PeerAuthReq = pReq->AuthReq;
	pLink->Ctx.bInitiator = false;
	pLink->Ctx.bSc = (pReq->AuthReq & BT_SMP_AUTHREQ_SC) && (s_SmpAuthReq & BT_SMP_AUTHREQ_SC);
	if ((s_SmpAuthReq & BT_SMP_AUTHREQ_SC) && !(pReq->AuthReq & BT_SMP_AUTHREQ_SC))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS); SmpAbortPairing(pLink); return;
	}
	pLink->Ctx.EncKeySize = pReq->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ? pReq->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;
	bool mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) || (pReq->AuthReq & BT_SMP_AUTHREQ_MITM);
	bool oob = pLink->Ctx.bSc && (pReq->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT || SmpOobPeerReady(pLink));
	if (pLink->Ctx.bSc && pReq->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT && !SmpOobLocalReady(pLink))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE); SmpAbortPairing(pLink); return;
	}
	pLink->Ctx.Model = SmpSelectModel(pReq->IOCaps, s_SmpIoCaps, mitm, oob);
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && !SmpOobCtxLoad(pLink))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE); SmpAbortPairing(pLink); return;
	}
	if (pLink->Ctx.Model > BT_SMP_MODEL_OOB)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS); SmpAbortPairing(pLink); return;
	}
	BtSmpPairingRsp_t rsp; SmpBuildPairingRsp(pLink, &rsp); SmpSend(pDev, ConnHdl, &rsp, sizeof(rsp));
	if (pLink->Ctx.bSc)
	{
		pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
		if (SmpLocalKeyGen(pDev, pLink) == BT_SMP_CRYPTO_FAIL)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE); SmpAbortPairing(pLink);
		}
	}
	else
	{
		memset(pLink->Ctx.Tk, 0, 16); pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
	}
}

static bool SmpKeyPresent(const uint8_t *pKey, size_t Len)
{
	for (size_t i = 0; i < Len; i++) if (pKey[i] != 0) return true; return false;
}

static bool SmpStartDhKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink)
{
	if (memcmp(pLink->Ctx.PeerPubKey, pLink->Ctx.LocalPubKey, 32) == 0)
	{
		return false;
	}
	if (!SmpCryptoRequest(pLink, pDev, BT_SMP_CRYPTO_OP_DHKEY)) return false;
	pLink->Ctx.State = BT_SMP_STATE_DHKEY_WAIT;
	return SmpCryptoStart(pLink) != BT_SMP_CRYPTO_FAIL;
}

static bool SmpTryStartDhKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink, uint16_t ConnHdl)
{
	if (!SmpKeyPresent(pLink->Ctx.LocalPubKey, 64) || !SmpKeyPresent(pLink->Ctx.PeerPubKey, 64)) return true;
	BtSmpPublicKey_t pk; pk.Code = BT_SMP_CODE_PAIRING_PUBLIC_KEY;
	for (int i = 0; i < 32; i++) { pk.KeyX[i] = pLink->Ctx.LocalPubKey[31-i]; pk.KeyY[i] = pLink->Ctx.LocalPubKey[63-i]; }
	SmpSend(pDev, ConnHdl, &pk, sizeof(pk)); return SmpStartDhKey(pDev, pLink);
}

static void SmpSendLocalPubKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink, uint16_t ConnHdl)
{
	BtSmpPublicKey_t pk; pk.Code = BT_SMP_CODE_PAIRING_PUBLIC_KEY;
	for (int i = 0; i < 32; i++) { pk.KeyX[i] = pLink->Ctx.LocalPubKey[31-i]; pk.KeyY[i] = pLink->Ctx.LocalPubKey[63-i]; }
	SmpSend(pDev, ConnHdl, &pk, sizeof(pk));
}

static void SmpHandlePairingRsp(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
		uint16_t ConnHdl, const BtSmpPairingRsp_t *pRsp)
{
	if (!pLink->Ctx.bInitiator) { SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CMD_NOT_SUPPORTED); return; }
	if (pRsp->IOCaps > BT_SMP_IOCAPS_KEYBOARD_DISPLAY || pRsp->OOBFlag > BT_SMP_OOB_AUTH_PRESENT)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_INVALID_PARAMS); SmpAbortPairing(pLink); return;
	}
	if (pRsp->MaxKeySize < BT_SMP_CFG_MIN_ENC_KEY_SIZE || pRsp->MaxKeySize > BT_SMP_MAX_ENC_KEY_SIZE)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_ENC_KEY_SIZE); SmpAbortPairing(pLink); return;
	}
	if (!(pRsp->AuthReq & BT_SMP_AUTHREQ_SC))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS); SmpAbortPairing(pLink); return;
	}
	memcpy(pLink->Ctx.PRsp, pRsp, 7); pLink->Ctx.PeerAuthReq = pRsp->AuthReq; pLink->Ctx.bSc = true;
	pLink->Ctx.EncKeySize = pRsp->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ? pRsp->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;
	bool mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) || (pRsp->AuthReq & BT_SMP_AUTHREQ_MITM);
	bool oob = pRsp->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT || SmpOobPeerReady(pLink);
	if (pRsp->OOBFlag != BT_SMP_OOB_AUTH_NOT_PRESENT && !SmpOobLocalReady(pLink))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE); SmpAbortPairing(pLink); return;
	}
	pLink->Ctx.Model = SmpSelectModel(s_SmpIoCaps, pRsp->IOCaps, mitm, oob);
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && !SmpOobCtxLoad(pLink))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE); SmpAbortPairing(pLink); return;
	}
	if (pLink->Ctx.Model > BT_SMP_MODEL_OOB)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS); SmpAbortPairing(pLink); return;
	}
	if (pLink->Ctx.Model == BT_SMP_MODEL_OOB && !SmpKeyPresent(pLink->Ctx.LocalPubKey, 64))
	{
		if (SmpLocalKeyGen(pDev, pLink) != BT_SMP_CRYPTO_OK)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_OOB_NOT_AVAILABLE); SmpAbortPairing(pLink); return;
		}
	}
	if (SmpKeyPresent(pLink->Ctx.LocalPubKey, 64))
	{
		SmpSendLocalPubKey(pDev, pLink, ConnHdl); pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
	}
}

static void SmpHandlePublicKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
		uint16_t ConnHdl, const BtSmpPublicKey_t *pPk)
{
	for (int i = 0; i < 32; i++) { pLink->Ctx.PeerPubKey[i] = pPk->KeyX[31-i]; pLink->Ctx.PeerPubKey[32+i] = pPk->KeyY[31-i]; }
	if (pLink->Ctx.bInitiator)
	{
		if (!SmpStartDhKey(pDev, pLink)) { SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED); SmpAbortPairing(pLink); }
		return;
	}
	if (!SmpTryStartDhKey(pDev, pLink, ConnHdl)) { SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED); SmpAbortPairing(pLink); }
}

#define BT_SMP_PASSKEY_ROUNDS 20
static uint8_t SmpPasskeyRa(uint32_t Passkey, uint8_t Round) { return (uint8_t)(0x80 | ((Passkey >> Round) & 1)); }
static bool SmpPasskeyLocalDisplays(uint8_t LocalIo, uint8_t PeerIo)
{
	if (LocalIo == BT_SMP_IOCAPS_KEYBOARD_ONLY) return false;
	if (LocalIo == BT_SMP_IOCAPS_DISPLAY_ONLY || LocalIo == BT_SMP_IOCAPS_DISPLAY_YESNO) return true;
	return PeerIo == BT_SMP_IOCAPS_KEYBOARD_ONLY;
}
static void SmpDhKeyCheckR(const BtSmpLink_t *pLink, bool bRa, uint8_t r[16])
{
	memset(r, 0, 16);
	if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		uint32_t pk = pLink->Ctx.Passkey; r[0]=pk; r[1]=pk>>8; r[2]=pk>>16; r[3]=pk>>24;
	}
	else if (pLink->Ctx.Model == BT_SMP_MODEL_OOB)
	{
		uint8_t flag = bRa ? pLink->Ctx.PRsp[2] : pLink->Ctx.PReq[2]; if (flag == BT_SMP_OOB_AUTH_NOT_PRESENT) return;
		bool own = bRa == pLink->Ctx.bInitiator; memcpy(r, own ? pLink->Ctx.OobLocalRand : pLink->Ctx.OobPeerRand, 16);
	}
}

static void SmpPasskeySendInitiatorConfirm(BtHciDevice_t * const pDev, BtSmpLink_t *pLink, uint16_t ConnHdl)
{
	uint8_t localX[32], peerX[32]; SmpP256CoordBeToSmpLe(pLink->Ctx.LocalPubKey, localX); SmpP256CoordBeToSmpLe(pLink->Ctx.PeerPubKey, peerX);
	if (!BtSmpCryptoRand(pLink->Ctx.LocalRand, 16)) { SmpFailAndLock(pDev, ConnHdl, pLink, BT_SMP_ERR_UNSPECIFIED); return; }
	BtSmpPairingConfirm_t cf; cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	if (!SmpF4(localX, peerX, pLink->Ctx.LocalRand, SmpPasskeyRa(pLink->Ctx.Passkey,pLink->Ctx.PkRound), cf.Value)) { SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED); return; }
	memcpy(pLink->Ctx.LocalConfirm, cf.Value, 16); SmpSend(pDev,ConnHdl,&cf,sizeof(cf)); pLink->Ctx.State=BT_SMP_STATE_CONFIRM_WAIT;
}
static void SmpPasskeyResponderConfirm(BtHciDevice_t * const pDev, BtSmpLink_t *pLink, uint16_t ConnHdl)
{
	uint8_t localX[32], peerX[32]; SmpP256CoordBeToSmpLe(pLink->Ctx.LocalPubKey,localX); SmpP256CoordBeToSmpLe(pLink->Ctx.PeerPubKey,peerX);
	if (!BtSmpCryptoRand(pLink->Ctx.LocalRand,16)) { SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED); return; }
	BtSmpPairingConfirm_t cf; cf.Code=BT_SMP_CODE_PAIRING_CONFIRM;
	if (!SmpF4(localX,peerX,pLink->Ctx.LocalRand,SmpPasskeyRa(pLink->Ctx.Passkey,pLink->Ctx.PkRound),cf.Value)) { SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED); return; }
	memcpy(pLink->Ctx.LocalConfirm,cf.Value,16); SmpSend(pDev,ConnHdl,&cf,sizeof(cf)); pLink->Ctx.State=BT_SMP_STATE_RANDOM_WAIT;
}

static void SmpOwnAddrGet(uint16_t ConnHdl, uint8_t *pType, uint8_t Addr[6])
{
	BtDevice_t *pPeer=BtPeerFindByHdl(ConnHdl);
	if (pPeer!=nullptr)
	{
		for(int i=0;i<6;i++) if(pPeer->Conn.OwnAddr[i]!=0){*pType=pPeer->Conn.OwnAddrType;memcpy(Addr,pPeer->Conn.OwnAddr,6);return;}
	}
	BtSmpLocalAddrGet(pType,Addr);
}

static void SmpPasskeyInitiatorFinish(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl,const uint8_t *PeerAddr,uint8_t PeerAddrType)
{
	uint8_t localAddr[6],localAddrType=0,dh[32];SmpOwnAddrGet(ConnHdl,&localAddrType,localAddr);SmpReverse32(pLink->Ctx.DhKey,dh);
	if(!SmpF5(dh,pLink->Ctx.LocalRand,pLink->Ctx.PeerRand,localAddrType,localAddr,PeerAddrType,PeerAddr,pLink->Ctx.Mackey,pLink->Ctx.Ltk)){CryptoSecureWipe(dh,32);SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}CryptoSecureWipe(dh,32);
	pLink->Ctx.bAuthenticated=true;SmpApplyKeySize(pLink->Ctx.Ltk,pLink->Ctx.EncKeySize);uint8_t io[3]={pLink->Ctx.PReq[1],pLink->Ctx.PReq[2],pLink->Ctx.PReq[3]},r[16],ea[16];SmpDhKeyCheckR(pLink,false,r);
	if(!SmpF6(pLink->Ctx.Mackey,pLink->Ctx.LocalRand,pLink->Ctx.PeerRand,r,io,localAddrType,localAddr,PeerAddrType,PeerAddr,ea)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpDhKeyCheck_t c;c.Code=BT_SMP_CODE_PAIRING_DHKEY_CHECK;memcpy(c.Value,ea,16);SmpSend(pDev,ConnHdl,&c,sizeof(c));pLink->Ctx.State=BT_SMP_STATE_DHKEY_CHECK_WAIT;
}
static void SmpPasskeyResponderFinish(BtSmpLink_t *pLink,const uint8_t *PeerAddr,uint8_t PeerAddrType)
{
	uint8_t localAddr[6],localAddrType=0,dh[32];SmpOwnAddrGet(pLink->ConnHdl,&localAddrType,localAddr);SmpReverse32(pLink->Ctx.DhKey,dh);
	if(!SmpF5(dh,pLink->Ctx.PeerRand,pLink->Ctx.LocalRand,PeerAddrType,PeerAddr,localAddrType,localAddr,pLink->Ctx.Mackey,pLink->Ctx.Ltk)){CryptoSecureWipe(dh,32);SmpFailAndLock(s_pSmpActiveDev,pLink->ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}CryptoSecureWipe(dh,32);pLink->Ctx.bAuthenticated=true;SmpApplyKeySize(pLink->Ctx.Ltk,pLink->Ctx.EncKeySize);pLink->Ctx.State=BT_SMP_STATE_DHKEY_CHECK_WAIT;
}

static void SmpPasskeyHandleConfirm(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl)
{
	if(pLink->Ctx.bInitiator){BtSmpPairingRandom_t r;r.Code=BT_SMP_CODE_PAIRING_RANDOM;memcpy(r.Value,pLink->Ctx.LocalRand,16);SmpSend(pDev,ConnHdl,&r,sizeof(r));pLink->Ctx.State=BT_SMP_STATE_RANDOM_WAIT;return;}
	if(!pLink->Ctx.bPkReady){pLink->Ctx.bPkPeerCommit=true;return;}SmpPasskeyResponderConfirm(pDev,pLink,ConnHdl);
}
static void SmpPasskeyHandleRandom(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl,const uint8_t *PeerAddr,uint8_t PeerAddrType)
{
	uint8_t localX[32],peerX[32];SmpP256CoordBeToSmpLe(pLink->Ctx.LocalPubKey,localX);SmpP256CoordBeToSmpLe(pLink->Ctx.PeerPubKey,peerX);uint8_t bit=SmpPasskeyRa(pLink->Ctx.Passkey,pLink->Ctx.PkRound);
	if(pLink->Ctx.bInitiator){uint8_t c[16];if(!SmpF4(peerX,localX,pLink->Ctx.PeerRand,bit,c)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}if(!SmpEqualCT(c,pLink->Ctx.PeerConfirm,16)){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CONFIRM_VALUE_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}if(pLink->Ctx.PkRound<19){pLink->Ctx.PkRound++;SmpPasskeySendInitiatorConfirm(pDev,pLink,ConnHdl);return;}SmpPasskeyInitiatorFinish(pDev,pLink,ConnHdl,PeerAddr,PeerAddrType);return;}
	uint8_t c[16];if(!SmpF4(peerX,localX,pLink->Ctx.PeerRand,bit,c)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}if(!SmpEqualCT(c,pLink->Ctx.PeerConfirm,16)){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CONFIRM_VALUE_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}BtSmpPairingRandom_t r;r.Code=BT_SMP_CODE_PAIRING_RANDOM;memcpy(r.Value,pLink->Ctx.LocalRand,16);SmpSend(pDev,ConnHdl,&r,sizeof(r));if(pLink->Ctx.PkRound<19){pLink->Ctx.PkRound++;pLink->Ctx.State=BT_SMP_STATE_CONFIRM_WAIT;return;}SmpPasskeyResponderFinish(pLink,PeerAddr,PeerAddrType);
}
static void SmpPasskeyBegin(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl)
{
	uint8_t li=pLink->Ctx.bInitiator?pLink->Ctx.PReq[1]:pLink->Ctx.PRsp[1],pi=pLink->Ctx.bInitiator?pLink->Ctx.PRsp[1]:pLink->Ctx.PReq[1];pLink->Ctx.PkRound=0;pLink->Ctx.bPkPeerCommit=false;pLink->Ctx.bPkDisplay=SmpPasskeyLocalDisplays(li,pi);pLink->Ctx.State=pLink->Ctx.bInitiator?BT_SMP_STATE_PASSKEY_WAIT:BT_SMP_STATE_CONFIRM_WAIT;
	if(pLink->Ctx.bPkDisplay){uint8_t r[4];if(!BtSmpCryptoRand(r,4)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}pLink->Ctx.Passkey=((uint32_t)r[0]|((uint32_t)r[1]<<8)|((uint32_t)r[2]<<16)|((uint32_t)r[3]<<24))%1000000;pLink->Ctx.bPkReady=true;BtSmpPasskeyDisplay(ConnHdl,pLink->Ctx.Passkey);}else{pLink->Ctx.Passkey=0;pLink->Ctx.bPkReady=false;BtSmpPasskeyRequest(ConnHdl);}if(pLink->Ctx.bInitiator&&pLink->Ctx.bPkReady&&pLink->Ctx.State==BT_SMP_STATE_PASSKEY_WAIT)SmpPasskeySendInitiatorConfirm(pDev,pLink,ConnHdl);
}

static void SmpHandlePairingConfirm(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl,const BtSmpPairingConfirm_t *pC)
{
	memcpy(pLink->Ctx.PeerConfirm,pC->Value,16);if(pLink->Ctx.Model==BT_SMP_MODEL_PASSKEY_ENTRY){SmpPasskeyHandleConfirm(pDev,pLink,ConnHdl);return;}if(pLink->Ctx.bInitiator){BtSmpPairingRandom_t r;r.Code=BT_SMP_CODE_PAIRING_RANDOM;memcpy(r.Value,pLink->Ctx.LocalRand,16);SmpSend(pDev,ConnHdl,&r,sizeof(r));pLink->Ctx.State=BT_SMP_STATE_RANDOM_WAIT;return;}if(pLink->Ctx.bSc)return;if(!BtSmpCryptoRand(pLink->Ctx.LocalRand,16)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);uint8_t ra[6]={0},ia[6]={0},rat=0,iat=0;if(peer){memcpy(ia,peer->Conn.PeerAddr,6);iat=peer->Conn.PeerAddrType;}SmpOwnAddrGet(ConnHdl,&rat,ra);if(!SmpC1(pLink->Ctx.Tk,pLink->Ctx.LocalRand,pLink->Ctx.PReq,pLink->Ctx.PRsp,iat,ia,rat,ra,pLink->Ctx.LocalConfirm)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpPairingConfirm_t c;c.Code=BT_SMP_CODE_PAIRING_CONFIRM;memcpy(c.Value,pLink->Ctx.LocalConfirm,16);SmpSend(pDev,ConnHdl,&c,sizeof(c));pLink->Ctx.State=BT_SMP_STATE_RANDOM_WAIT;
}

static void SmpHandlePairingRandom(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl,const BtSmpPairingRandom_t *pR)
{
	memcpy(pLink->Ctx.PeerRand,pR->Value,16);BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);uint8_t pa[6]={0},pat=0;if(peer){memcpy(pa,peer->Conn.PeerAddr,6);pat=peer->Conn.PeerAddrType;}if(pLink->Ctx.bSc&&SmpIsAllZero(pLink->Ctx.DhKey,32)){SmpAbortOffPhase(pDev,ConnHdl,pLink);return;}if(pLink->Ctx.Model==BT_SMP_MODEL_PASSKEY_ENTRY){SmpPasskeyHandleRandom(pDev,pLink,ConnHdl,pa,pat);return;}
	if(pLink->Ctx.bInitiator){if(pLink->Ctx.Model!=BT_SMP_MODEL_OOB){uint8_t lx[32],px[32],cb[16];SmpP256CoordBeToSmpLe(pLink->Ctx.LocalPubKey,lx);SmpP256CoordBeToSmpLe(pLink->Ctx.PeerPubKey,px);if(!SmpF4(px,lx,pLink->Ctx.PeerRand,0,cb)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}if(!SmpEqualCT(cb,pLink->Ctx.PeerConfirm,16)){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CONFIRM_VALUE_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}}uint8_t la[6],lat=0,dh[32];SmpOwnAddrGet(ConnHdl,&lat,la);SmpReverse32(pLink->Ctx.DhKey,dh);if(!SmpF5(dh,pLink->Ctx.LocalRand,pLink->Ctx.PeerRand,lat,la,pat,pa,pLink->Ctx.Mackey,pLink->Ctx.Ltk)){CryptoSecureWipe(dh,32);SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}CryptoSecureWipe(dh,32);if(pLink->Ctx.Model==BT_SMP_MODEL_OOB)pLink->Ctx.bAuthenticated=true;SmpApplyKeySize(pLink->Ctx.Ltk,pLink->Ctx.EncKeySize);if(pLink->Ctx.Model==BT_SMP_MODEL_NUMERIC_COMPARISON){uint32_t v;if(!SmpNumericValue(pLink,&v)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}pLink->Ctx.State=BT_SMP_STATE_NUMERIC_WAIT;BtSmpNumericComparison(ConnHdl,v);return;}uint8_t io[3]={pLink->Ctx.PReq[1],pLink->Ctx.PReq[2],pLink->Ctx.PReq[3]},r[16],ea[16];SmpDhKeyCheckR(pLink,false,r);if(!SmpF6(pLink->Ctx.Mackey,pLink->Ctx.LocalRand,pLink->Ctx.PeerRand,r,io,lat,la,pat,pa,ea)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpDhKeyCheck_t c;c.Code=BT_SMP_CODE_PAIRING_DHKEY_CHECK;memcpy(c.Value,ea,16);SmpSend(pDev,ConnHdl,&c,sizeof(c));pLink->Ctx.State=BT_SMP_STATE_DHKEY_CHECK_WAIT;return;}
	if(pLink->Ctx.bSc){BtSmpPairingRandom_t r;r.Code=BT_SMP_CODE_PAIRING_RANDOM;memcpy(r.Value,pLink->Ctx.LocalRand,16);SmpSend(pDev,ConnHdl,&r,sizeof(r));uint8_t la[6],lat=0,dh[32];SmpOwnAddrGet(ConnHdl,&lat,la);SmpReverse32(pLink->Ctx.DhKey,dh);if(!SmpF5(dh,pLink->Ctx.PeerRand,pLink->Ctx.LocalRand,pat,pa,lat,la,pLink->Ctx.Mackey,pLink->Ctx.Ltk)){CryptoSecureWipe(dh,32);SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}CryptoSecureWipe(dh,32);if(pLink->Ctx.Model==BT_SMP_MODEL_OOB)pLink->Ctx.bAuthenticated=true;SmpApplyKeySize(pLink->Ctx.Ltk,pLink->Ctx.EncKeySize);pLink->Ctx.State=BT_SMP_STATE_DHKEY_CHECK_WAIT;if(pLink->Ctx.Model==BT_SMP_MODEL_NUMERIC_COMPARISON){uint32_t v;if(!SmpNumericValue(pLink,&v)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpNumericComparison(ConnHdl,v);}return;}
	uint8_t calc[16],la[6],lat=0;SmpOwnAddrGet(ConnHdl,&lat,la);if(!SmpC1(pLink->Ctx.Tk,pLink->Ctx.PeerRand,pLink->Ctx.PReq,pLink->Ctx.PRsp,pat,pa,lat,la,calc)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}if(!SmpEqualCT(calc,pLink->Ctx.PeerConfirm,16)){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CONFIRM_VALUE_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}BtSmpPairingRandom_t r;r.Code=BT_SMP_CODE_PAIRING_RANDOM;memcpy(r.Value,pLink->Ctx.LocalRand,16);SmpSend(pDev,ConnHdl,&r,sizeof(r));if(!SmpS1(pLink->Ctx.Tk,pLink->Ctx.LocalRand,pLink->Ctx.PeerRand,pLink->Ctx.Ltk)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}SmpApplyKeySize(pLink->Ctx.Ltk,pLink->Ctx.EncKeySize);pLink->Ctx.State=BT_SMP_STATE_LTK_WAIT;
}

static void SmpHandleDhKeyCheck(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint16_t ConnHdl,const BtSmpDhKeyCheck_t *pChk)
{
	BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);uint8_t pa[6]={0},pat=0;if(peer){memcpy(pa,peer->Conn.PeerAddr,6);pat=peer->Conn.PeerAddrType;}uint8_t la[6],lat=0;SmpOwnAddrGet(ConnHdl,&lat,la);
	if(pLink->Ctx.bInitiator){uint8_t io[3]={pLink->Ctx.PRsp[1],pLink->Ctx.PRsp[2],pLink->Ctx.PRsp[3]},r[16],eb[16];SmpDhKeyCheckR(pLink,true,r);if(!SmpF6(pLink->Ctx.Mackey,pLink->Ctx.PeerRand,pLink->Ctx.LocalRand,r,io,pat,pa,lat,la,eb)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}if(!SmpEqualCT(eb,pChk->Value,16)){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_DHKEY_CHECK_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}pLink->Ctx.State=BT_SMP_STATE_LTK_WAIT;BtSmpHciEnableEncryption(pDev,ConnHdl,0,0,pLink->Ctx.Ltk);return;}
	uint8_t ioA[3]={pLink->Ctx.PReq[1],pLink->Ctx.PReq[2],pLink->Ctx.PReq[3]},ioB[3]={pLink->Ctx.PRsp[1],pLink->Ctx.PRsp[2],pLink->Ctx.PRsp[3]},r[16],ea[16];SmpDhKeyCheckR(pLink,false,r);if(!SmpF6(pLink->Ctx.Mackey,pLink->Ctx.PeerRand,pLink->Ctx.LocalRand,r,ioA,pat,pa,lat,la,ea)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}if(!SmpEqualCT(ea,pChk->Value,16)){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_DHKEY_CHECK_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}if(pLink->Ctx.Model==BT_SMP_MODEL_NUMERIC_COMPARISON&&!pLink->Ctx.bAuthenticated){pLink->Ctx.State=BT_SMP_STATE_NUMERIC_WAIT;return;}SmpDhKeyCheckR(pLink,true,r);uint8_t eb[16];if(!SmpF6(pLink->Ctx.Mackey,pLink->Ctx.LocalRand,pLink->Ctx.PeerRand,r,ioB,lat,la,pat,pa,eb)){SmpFailAndLock(pDev,ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpDhKeyCheck_t c;c.Code=BT_SMP_CODE_PAIRING_DHKEY_CHECK;memcpy(c.Value,eb,16);SmpSend(pDev,ConnHdl,&c,sizeof(c));pLink->Ctx.State=BT_SMP_STATE_LTK_WAIT;
}

static void SmpHandleEncryptInfo(BtSmpLink_t *pLink,const BtSmpEncryptInfo_t *pInfo){memcpy(pLink->Keys.Ltk,pInfo->Ltk,16);}
static void SmpHandleCentralId(BtSmpLink_t *pLink,const BtSmpCentralId_t *pId){pLink->Keys.Ediv=pId->Ediv;pLink->Keys.Rand=pId->Rand;}
static void SmpHandleIdInfo(BtSmpLink_t *pLink,const BtSmpIdInfo_t *pInfo){memcpy(pLink->Keys.Irk,pInfo->Irk,16);}
static void SmpHandleIdAddrInfo(BtSmpLink_t *pLink,const BtSmpIdAddrInfo_t *pInfo){pLink->Keys.IdAddrType=pInfo->AddrType;memcpy(pLink->Keys.IdAddr,pInfo->Addr,6);}
static void SmpHandleSigningInfo(BtSmpLink_t *pLink,const BtSmpSigningInfo_t *pInfo){memcpy(pLink->Keys.Csrk,pInfo->Csrk,16);}

static uint8_t SmpLocalKeyDist(const BtSmpLink_t *pLink)
{
	return (pLink->Ctx.bInitiator ? pLink->Ctx.PRsp[5] : pLink->Ctx.PRsp[6]) & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY);
}
static uint8_t SmpPeerKeyDist(const BtSmpLink_t *pLink)
{
	return (pLink->Ctx.bInitiator ? pLink->Ctx.PRsp[6] : pLink->Ctx.PRsp[5]) & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY);
}
static bool SmpSendLocalKeyDist(BtHciDevice_t * const pDev,BtSmpLink_t *pLink)
{
	if(pLink->bLocalKeyDistSent)return true;uint8_t d=SmpLocalKeyDist(pLink);
	if(d&BT_SMP_KEYDIST_IDKEY){uint8_t irk[16];if(!BtSmpLocalIrkGet(irk))return false;BtSmpIdInfo_t i;i.Code=BT_SMP_CODE_PAIRING_ID_INFO;memcpy(i.Irk,irk,16);SmpSend(pDev,pLink->ConnHdl,&i,sizeof(i));uint8_t a[6],t=0;BtSmpLocalAddrGet(&t,a);BtSmpIdAddrInfo_t ai;ai.Code=BT_SMP_CODE_PAIRING_ID_ADDR_INFO;ai.AddrType=t;memcpy(ai.Addr,a,6);SmpSend(pDev,pLink->ConnHdl,&ai,sizeof(ai));CryptoSecureWipe(irk,16);}
	if(d&BT_SMP_KEYDIST_SIGNKEY){uint8_t csrk[16];if(!BtSmpCryptoRand(csrk,16))return false;BtSmpSigningInfo_t s;s.Code=BT_SMP_CODE_PAIRING_SIGNING_INFO;memcpy(s.Csrk,csrk,16);SmpSend(pDev,pLink->ConnHdl,&s,sizeof(s));CryptoSecureWipe(csrk,16);}pLink->bLocalKeyDistSent=true;return true;
}
static void SmpKeyDistComplete(BtHciDevice_t * const pDev,BtSmpLink_t *pLink)
{
	if(pLink->Ctx.KeyDistExp!=0)return;if(pLink->Ctx.bInitiator&&!SmpSendLocalKeyDist(pDev,pLink)){CryptoSecureWipe(&pLink->Keys,sizeof(pLink->Keys));SmpFailAndLock(pDev,pLink->ConnHdl,pLink,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpBondAdd(pLink->ConnHdl,&pLink->Keys);if(pLink->Ctx.Model==BT_SMP_MODEL_OOB)SmpOobRelease(pLink);pLink->Ctx.State=BT_SMP_STATE_DONE;BtSmpPairingComplete(pLink->ConnHdl,true,&pLink->Keys);
}
static void SmpKeyDistReceived(BtHciDevice_t * const pDev,BtSmpLink_t *pLink,uint8_t KeyBit)
{
	pLink->Ctx.KeyDistExp&=(uint8_t)~KeyBit;SmpKeyDistComplete(pDev,pLink);
}

void BtProcessSmpData(BtHciDevice_t * const pDev,uint16_t ConnHdl,BtL2CapSmp_t * const pSmp,size_t Len)
{
	if(pSmp==nullptr||Len<1)return;s_pSmpActiveDev=pDev;BtSmpLink_t *pLink=SmpLinkFind(ConnHdl);SMP_TRACE_PDU("RX",pSmp->Code,pLink?(int)pLink->Ctx.State:-1);if(!pLink){pLink=SmpLinkAlloc(ConnHdl);if(!pLink){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_UNSPECIFIED);return;}}
	if(pLink->Ctx.bLocked){if(!pLink->bTimedOut)SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_REPEATED_ATTEMPTS);return;}if(pLink->Ctx.State==BT_SMP_STATE_IDLE)pLink->Ctx.TmrStart=BtSmpMsTick();if(SmpPairingTimedOut(pLink)){SmpTimeoutAbort(pLink);return;}
	size_t minLen=1;switch(pSmp->Code){case BT_SMP_CODE_PAIRING_REQ:case BT_SMP_CODE_PAIRING_RSP:minLen=sizeof(BtSmpPairingReq_t);break;case BT_SMP_CODE_PAIRING_CONFIRM:minLen=sizeof(BtSmpPairingConfirm_t);break;case BT_SMP_CODE_PAIRING_RANDOM:minLen=sizeof(BtSmpPairingRandom_t);break;case BT_SMP_CODE_PAIRING_FAILED:minLen=sizeof(BtSmpPairingFailed_t);break;case BT_SMP_CODE_PAIRING_ENCRYP_INFO:minLen=sizeof(BtSmpEncryptInfo_t);break;case BT_SMP_CODE_PAIRING_CENTRAL_ID:minLen=sizeof(BtSmpCentralId_t);break;case BT_SMP_CODE_PAIRING_ID_INFO:minLen=sizeof(BtSmpIdInfo_t);break;case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:minLen=sizeof(BtSmpIdAddrInfo_t);break;case BT_SMP_CODE_PAIRING_SIGNING_INFO:minLen=sizeof(BtSmpSigningInfo_t);break;case BT_SMP_CODE_PAIRING_PUBLIC_KEY:minLen=sizeof(BtSmpPublicKey_t);break;case BT_SMP_CODE_PAIRING_DHKEY_CHECK:minLen=sizeof(BtSmpDhKeyCheck_t);break;case BT_SMP_CODE_PAIRING_SECURITY_REQ:minLen=sizeof(BtSmpSecurityReq_t);break;default:break;}if(Len<minLen){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_INVALID_PARAMS);return;}
	switch(pSmp->Code){case BT_SMP_CODE_PAIRING_ENCRYP_INFO:case BT_SMP_CODE_PAIRING_CENTRAL_ID:case BT_SMP_CODE_PAIRING_ID_INFO:case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:case BT_SMP_CODE_PAIRING_SIGNING_INFO:{BtDevice_t *q=BtPeerFindByHdl(ConnHdl);if(!q||!q->bSecure||pLink->Ctx.State!=BT_SMP_STATE_KEYDIST){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_UNSPECIFIED);return;}}break;default:break;}
	switch(pSmp->Code){case BT_SMP_CODE_PAIRING_REQ:if(BtPeerRole(ConnHdl)==BT_CONN_ROLE_CENTRAL){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CMD_NOT_SUPPORTED);return;}if(pLink->Ctx.State!=BT_SMP_STATE_IDLE){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_UNSPECIFIED);return;}break;case BT_SMP_CODE_PAIRING_RSP:if(!pLink->Ctx.bInitiator||pLink->Ctx.State!=BT_SMP_STATE_PUBKEY_LOCAL_WAIT){SmpAbortOffPhase(pDev,ConnHdl,pLink);return;}break;case BT_SMP_CODE_PAIRING_PUBLIC_KEY:if(pLink->Ctx.State!=BT_SMP_STATE_PUBKEY_WAIT&&pLink->Ctx.State!=BT_SMP_STATE_PUBKEY_LOCAL_WAIT){SmpAbortOffPhase(pDev,ConnHdl,pLink);return;}break;case BT_SMP_CODE_PAIRING_CONFIRM:if(pLink->Ctx.State!=BT_SMP_STATE_CONFIRM_WAIT&&pLink->Ctx.State!=BT_SMP_STATE_PASSKEY_WAIT){SmpAbortOffPhase(pDev,ConnHdl,pLink);return;}break;case BT_SMP_CODE_PAIRING_RANDOM:if(pLink->Ctx.State!=BT_SMP_STATE_RANDOM_WAIT){SmpAbortOffPhase(pDev,ConnHdl,pLink);return;}break;case BT_SMP_CODE_PAIRING_DHKEY_CHECK:if(pLink->Ctx.State!=BT_SMP_STATE_DHKEY_CHECK_WAIT){SmpAbortOffPhase(pDev,ConnHdl,pLink);return;}break;default:break;}
	switch(pSmp->Code){case BT_SMP_CODE_PAIRING_REQ:SmpHandlePairingReq(pDev,pLink,ConnHdl,(const BtSmpPairingReq_t*)pSmp);break;case BT_SMP_CODE_PAIRING_RSP:SmpHandlePairingRsp(pDev,pLink,ConnHdl,(const BtSmpPairingRsp_t*)pSmp);break;case BT_SMP_CODE_PAIRING_PUBLIC_KEY:SmpHandlePublicKey(pDev,pLink,ConnHdl,(const BtSmpPublicKey_t*)pSmp);break;case BT_SMP_CODE_PAIRING_CONFIRM:SmpHandlePairingConfirm(pDev,pLink,ConnHdl,(const BtSmpPairingConfirm_t*)pSmp);break;case BT_SMP_CODE_PAIRING_RANDOM:SmpHandlePairingRandom(pDev,pLink,ConnHdl,(const BtSmpPairingRandom_t*)pSmp);break;case BT_SMP_CODE_PAIRING_DHKEY_CHECK:SmpHandleDhKeyCheck(pDev,pLink,ConnHdl,(const BtSmpDhKeyCheck_t*)pSmp);break;case BT_SMP_CODE_PAIRING_ENCRYP_INFO:SmpHandleEncryptInfo(pLink,(const BtSmpEncryptInfo_t*)pSmp);break;case BT_SMP_CODE_PAIRING_CENTRAL_ID:SmpHandleCentralId(pLink,(const BtSmpCentralId_t*)pSmp);SmpKeyDistReceived(pDev,pLink,BT_SMP_KEYDIST_ENCKEY);break;case BT_SMP_CODE_PAIRING_ID_INFO:SmpHandleIdInfo(pLink,(const BtSmpIdInfo_t*)pSmp);break;case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:SmpHandleIdAddrInfo(pLink,(const BtSmpIdAddrInfo_t*)pSmp);SmpKeyDistReceived(pDev,pLink,BT_SMP_KEYDIST_IDKEY);break;case BT_SMP_CODE_PAIRING_SIGNING_INFO:SmpHandleSigningInfo(pLink,(const BtSmpSigningInfo_t*)pSmp);SmpKeyDistReceived(pDev,pLink,BT_SMP_KEYDIST_SIGNKEY);break;case BT_SMP_CODE_PAIRING_FAILED:if(pLink->Ctx.State!=BT_SMP_STATE_IDLE){SmpAbortPairing(pLink);pLink->Ctx.FailCount++;if(pLink->Ctx.FailCount>=BT_SMP_MAX_PAIR_ATTEMPTS)pLink->Ctx.bLocked=true;BtSmpPairingComplete(ConnHdl,false,nullptr);}break;case BT_SMP_CODE_PAIRING_SECURITY_REQ:if(BtPeerRole(ConnHdl)==BT_CONN_ROLE_PERIPHERAL){SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CMD_NOT_SUPPORTED);break;}BtSmpStartPairing(ConnHdl);break;default:SmpSendFailed(pDev,ConnHdl,BT_SMP_ERR_CMD_NOT_SUPPORTED);break;}
}

void BtSmpLocalPubKeyReady(BtHciDevice_t * const pDev,uint8_t Status,const uint8_t *pKeyX,const uint8_t *pKeyY)
{
	s_pSmpActiveDev=pDev;BtSmpLink_t *pLink=SmpCryptoInflightTake(BT_SMP_CRYPTO_OP_PUBKEY);if(!pLink||(pLink->Ctx.State!=BT_SMP_STATE_PUBKEY_WAIT&&pLink->Ctx.State!=BT_SMP_STATE_PUBKEY_LOCAL_WAIT))return;if(Status||!pKeyX||!pKeyY){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_UNSPECIFIED);SmpAbortPairing(pLink);return;}memcpy(pLink->Ctx.LocalPubKey,pKeyX,32);memcpy(pLink->Ctx.LocalPubKey+32,pKeyY,32);if(pLink->Ctx.bInitiator){if(pLink->Ctx.PRsp[0]==BT_SMP_CODE_PAIRING_RSP){SmpSendLocalPubKey(pDev,pLink,pLink->ConnHdl);pLink->Ctx.State=BT_SMP_STATE_PUBKEY_WAIT;}return;}if(!SmpTryStartDhKey(pDev,pLink,pLink->ConnHdl)){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_DHKEY_CHECK_FAILED);SmpAbortPairing(pLink);}
}
void BtSmpDhKeyReady(BtHciDevice_t * const pDev,uint8_t Status,const uint8_t *pDhKey)
{
	s_pSmpActiveDev=pDev;BtSmpLink_t *pLink=SmpCryptoInflightTake(BT_SMP_CRYPTO_OP_DHKEY);if(!pLink||pLink->Ctx.State!=BT_SMP_STATE_DHKEY_WAIT)return;if(Status||!pDhKey){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_DHKEY_CHECK_FAILED);SmpAbortPairing(pLink);return;}memcpy(pLink->Ctx.DhKey,pDhKey,32);if(pLink->Ctx.Model==BT_SMP_MODEL_PASSKEY_ENTRY){SmpPasskeyBegin(pDev,pLink,pLink->ConnHdl);return;}if(pLink->Ctx.Model==BT_SMP_MODEL_OOB){if(pLink->Ctx.bOobPeerData){uint8_t x[32],c[16];SmpP256CoordBeToSmpLe(pLink->Ctx.PeerPubKey,x);if(!SmpF4(x,x,pLink->Ctx.OobPeerRand,0,c)||!SmpEqualCT(c,pLink->Ctx.OobPeerConfirm,16)){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_CONFIRM_VALUE_FAILED);SmpAuthFailCount(pLink);SmpAbortPairing(pLink);return;}}if(!BtSmpCryptoRand(pLink->Ctx.LocalRand,16)){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_UNSPECIFIED);SmpAbortPairing(pLink);return;}if(pLink->Ctx.bInitiator){BtSmpPairingRandom_t r;r.Code=BT_SMP_CODE_PAIRING_RANDOM;memcpy(r.Value,pLink->Ctx.LocalRand,16);SmpSend(pDev,pLink->ConnHdl,&r,sizeof(r));}pLink->Ctx.State=BT_SMP_STATE_RANDOM_WAIT;return;}if(pLink->Ctx.bInitiator){if(!BtSmpCryptoRand(pLink->Ctx.LocalRand,16)){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_UNSPECIFIED);SmpAbortPairing(pLink);return;}pLink->Ctx.State=BT_SMP_STATE_CONFIRM_WAIT;return;}if(!BtSmpCryptoRand(pLink->Ctx.LocalRand,16)){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_UNSPECIFIED);SmpAbortPairing(pLink);return;}uint8_t lx[32],px[32];SmpP256CoordBeToSmpLe(pLink->Ctx.LocalPubKey,lx);SmpP256CoordBeToSmpLe(pLink->Ctx.PeerPubKey,px);BtSmpPairingConfirm_t c;c.Code=BT_SMP_CODE_PAIRING_CONFIRM;if(!SmpF4(lx,px,pLink->Ctx.LocalRand,0,c.Value)){SmpSendFailed(pDev,pLink->ConnHdl,BT_SMP_ERR_UNSPECIFIED);SmpAbortPairing(pLink);return;}memcpy(pLink->Ctx.LocalConfirm,c.Value,16);SmpSend(pDev,pLink->ConnHdl,&c,sizeof(c));pLink->Ctx.State=BT_SMP_STATE_RANDOM_WAIT;
}

void BtSmpProcessLtkRequest(BtHciDevice_t * const pDev,uint16_t ConnHdl,uint64_t Rand,uint16_t Ediv)
{
	uint8_t key[16];bool have=false;BtSmpLink_t *p=SmpLinkFind(ConnHdl);if(p&&p->Ctx.State==BT_SMP_STATE_LTK_WAIT&&SmpKeyPresent(p->Ctx.Ltk,16)){memcpy(key,p->Ctx.Ltk,16);have=true;}else if(p&&p->Ctx.State==BT_SMP_STATE_DONE&&p->Keys.bValid){memcpy(key,p->Keys.Ltk,16);have=true;}else have=BtSmpBondLtkLookup(ConnHdl,Rand,Ediv,key);if(have)BtSmpHciLtkReply(pDev,ConnHdl,key);else BtSmpHciLtkNegReply(pDev,ConnHdl);
}
static void SmpConnSecFromKeys(BtConnSec_t *s,const BtSmpKeys_t *k)
{
	s->KeySize=k->EncKeySize;s->Level=k->bAuthenticated?(k->bSc?BT_GAP_SEC_LEVEL_LESC_AUTH:BT_GAP_SEC_LEVEL_ENC_AUTH):BT_GAP_SEC_LEVEL_ENC_UNAUTH;if(k->bSc)s->Flags|=BT_GAP_SEC_FLAG_SC;for(int i=0;i<16;i++)if(k->Csrk[i]){s->Flags|=BT_GAP_SEC_FLAG_SIGNED;break;}
}
void BtSmpEncryptionChanged(BtHciDevice_t * const pDev,uint16_t ConnHdl,uint8_t Status,uint8_t Enabled)
{
	BtSmpLink_t *p=SmpLinkFind(ConnHdl);if(Status==0&&Enabled&&p&&p->Ctx.State==BT_SMP_STATE_LTK_WAIT){SmpCommitPendingKeys(p);p->bLocalKeyDistSent=false;}BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);if(peer){peer->bSecure=Status==0&&Enabled;BtConnSec_t sec={};if(peer->bSecure){if(p&&p->Keys.bValid)SmpConnSecFromKeys(&sec,&p->Keys);else{BtSmpKeys_t k;if(BtSmpBondKeysLookup(ConnHdl,0,0,&k))SmpConnSecFromKeys(&sec,&k);else{sec.Level=BT_GAP_SEC_LEVEL_ENC_UNAUTH;sec.KeySize=BT_SMP_MAX_ENC_KEY_SIZE;}CryptoSecureWipe(&k,sizeof(k));}if(BtSmpBonded(ConnHdl))sec.Flags|=BT_GAP_SEC_FLAG_BONDED;}BtGapConnSecSet(ConnHdl,&sec);}if(Status||!Enabled){if(p)SmpAbortPairing(p);BtSmpPairingComplete(ConnHdl,false,nullptr);return;}BtGattCccdRestoreBonded(ConnHdl);if(!p)return;if(p->Ctx.State==BT_SMP_STATE_LTK_WAIT){p->Ctx.KeyDistExp=SmpPeerKeyDist(p);p->Ctx.TmrStart=BtSmpMsTick();p->Ctx.State=BT_SMP_STATE_KEYDIST;if(!p->Ctx.bInitiator&&!SmpSendLocalKeyDist(pDev,p)){CryptoSecureWipe(&p->Keys,sizeof(p->Keys));SmpFailAndLock(pDev,ConnHdl,p,BT_SMP_ERR_UNSPECIFIED);return;}if(p->Ctx.Model==BT_SMP_MODEL_OOB)SmpOobRelease(p);SmpKeyDistComplete(pDev,p);}else if(p->Ctx.State==BT_SMP_STATE_DONE)BtSmpPairingComplete(ConnHdl,true,&p->Keys);
}

__attribute__((weak)) void BtSmpPairingComplete(uint16_t ConnHdl,bool Success,const BtSmpKeys_t *pKeys){(void)ConnHdl;(void)Success;(void)pKeys;}
__attribute__((weak)) void BtSmpLocalAddrGet(uint8_t *pType,uint8_t pAddr[6]){*pType=0;memset(pAddr,0,6);}
__attribute__((weak)) bool BtSmpBondLtkLookup(uint16_t ConnHdl,uint64_t Rand,uint16_t Ediv,uint8_t Ltk[16]){(void)ConnHdl;(void)Rand;(void)Ediv;(void)Ltk;return false;}
__attribute__((weak)) void BtSmpBondAdd(uint16_t ConnHdl,const BtSmpKeys_t *pKeys){(void)ConnHdl;(void)pKeys;}
__attribute__((weak)) void BtSmpBondClearAll(void){}

static int CryptoStatusToSmp(CRYPTO_STATUS st){switch(st){case CRYPTO_STATUS_OK:return BT_SMP_CRYPTO_OK;case CRYPTO_STATUS_PENDING:return BT_SMP_CRYPTO_PENDING;case CRYPTO_STATUS_BUSY:return BT_SMP_CRYPTO_BUSY;default:return BT_SMP_CRYPTO_FAIL;}}
void BtSmpCryptoAes128(BtHciDevice_t * const pDev,const uint8_t Key[16],const uint8_t In[16],uint8_t Out[16])
{
	(void)pDev;if(!s_pCryptoAes){memset(Out,0,16);s_SmpAesFault=true;return;}CryptoKey key;key.Type=CRYPTO_KEY_AES_128;key.Loc=CRYPTO_KEY_LOC_PLAIN;key.Usage=CRYPTO_KEY_USE_ENCRYPT;key.Plain.pData=Key;key.Plain.Len=16;CRYPTO_STATUS st=CRYPTO_STATUS_BUSY;for(uint32_t n=0;n<BT_SMP_CRYPTO_BUSY_RETRIES&&st==CRYPTO_STATUS_BUSY;n++){st=s_pCryptoAes->Cipher(CRYPTO_CIPHER_ECB,1,key,nullptr,0,In,16,Out);__asm volatile("":::"memory");}if(st!=CRYPTO_STATUS_OK){memset(Out,0,16);s_SmpAesFault=true;}
}
static int SmpCryptoP256KeyGen(BtSmpLink_t *p,uint8_t k[64]){if(!s_pCryptoEcdh||!p)return BT_SMP_CRYPTO_FAIL;return CryptoStatusToSmp(s_pCryptoEcdh->KeyGen(CRYPTO_CURVE_P256,p->Ctx.EcdhKeyCtx,k));}
int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev,uint8_t k[64]){(void)pDev;if(!s_pCryptoEcdh)return BT_SMP_CRYPTO_FAIL;return CryptoStatusToSmp(s_pCryptoEcdh->KeyGen(CRYPTO_CURVE_P256,s_SmpOob.EcdhKeyCtx,k));}
static int SmpCryptoEcdh(BtSmpLink_t *p,const uint8_t pk[64],uint8_t dh[32]){if(!s_pCryptoEcdh||!p)return BT_SMP_CRYPTO_FAIL;uint8_t *ctx=SmpOobReservedFor(p)&&s_SmpOob.bLocalValid?s_SmpOob.EcdhKeyCtx:p->Ctx.EcdhKeyCtx;return CryptoStatusToSmp(s_pCryptoEcdh->Agree(CRYPTO_CURVE_P256,ctx,pk,dh));}
int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,const uint8_t pk[64],uint8_t dh[32]){(void)pDev;if(!s_pCryptoEcdh)return BT_SMP_CRYPTO_FAIL;return CryptoStatusToSmp(s_pCryptoEcdh->Agree(CRYPTO_CURVE_P256,s_SmpOob.EcdhKeyCtx,pk,dh));}
static int SmpCryptoStart(BtSmpLink_t *p)
{
	if(!p||p->CryptoOp==BT_SMP_CRYPTO_OP_NONE)return BT_SMP_CRYPTO_FAIL;if(s_CryptoInflight.Op!=BT_SMP_CRYPTO_OP_NONE)return BT_SMP_CRYPTO_OK;s_CryptoInflight.Op=p->CryptoOp;s_CryptoInflight.ConnHdl=p->ConnHdl;s_CryptoInflight.Generation=p->Generation;s_CryptoInflight.pDev=p->pCryptoDev;p->bCryptoWait=false;int rc=p->CryptoOp==BT_SMP_CRYPTO_OP_PUBKEY?SmpCryptoP256KeyGen(p,p->Ctx.LocalPubKey):SmpCryptoEcdh(p,p->Ctx.PeerPubKey,p->Ctx.DhKey);BtHciDevice_t *d=p->pCryptoDev;BT_SMP_CRYPTO_OP op=p->CryptoOp;if(rc==BT_SMP_CRYPTO_OK){if(op==BT_SMP_CRYPTO_OP_PUBKEY)BtSmpLocalPubKeyReady(d,0,p->Ctx.LocalPubKey,p->Ctx.LocalPubKey+32);else BtSmpDhKeyReady(d,0,p->Ctx.DhKey);return rc;}if(rc==BT_SMP_CRYPTO_PENDING)return rc;if(rc==BT_SMP_CRYPTO_BUSY){memset(&s_CryptoInflight,0,sizeof(s_CryptoInflight));p->bCryptoWait=true;p->bRetryBusy=true;return rc;}SmpCryptoLinkClear(p);return BT_SMP_CRYPTO_FAIL;
}
static void SmpCryptoPump(void){if(s_CryptoInflight.Op!=BT_SMP_CRYPTO_OP_NONE)return;for(int i=0;i<BT_SMP_MAX_LINK;i++){BtSmpLink_t *p=&s_SmpLink[i];if(p->ConnHdl!=BT_CONN_HDL_INVALID&&p->CryptoOp!=BT_SMP_CRYPTO_OP_NONE&&p->bCryptoWait&&!p->bRetryBusy){SmpCryptoStart(p);return;}}}
static void SmpCryptoComplete(CryptoEngine * const pEngine,CRYPTO_OP Op,CRYPTO_STATUS Status,void *pCtx){(void)pEngine;if(pCtx!=&s_CryptoInflight)return;uint8_t st=Status==CRYPTO_STATUS_OK?0:1;if(Op==CRYPTO_OP_KEYGEN&&s_CryptoInflight.Op==BT_SMP_CRYPTO_OP_PUBKEY){BtSmpLink_t *p=SmpCryptoInflightLink(BT_SMP_CRYPTO_OP_PUBKEY);BtHciDevice_t *d=s_CryptoInflight.pDev;const uint8_t *k=p?p->Ctx.LocalPubKey:nullptr;BtSmpLocalPubKeyReady(d,st,k,k?k+32:nullptr);SmpCryptoPump();}else if(Op==CRYPTO_OP_AGREE&&s_CryptoInflight.Op==BT_SMP_CRYPTO_OP_DHKEY){BtSmpLink_t *p=SmpCryptoInflightLink(BT_SMP_CRYPTO_OP_DHKEY);BtHciDevice_t *d=s_CryptoInflight.pDev;BtSmpDhKeyReady(d,st,p?p->Ctx.DhKey:nullptr);SmpCryptoPump();}}
static void SmpCryptoRetryPending(void){for(int i=0;i<BT_SMP_MAX_LINK;i++){BtSmpLink_t *p=&s_SmpLink[i];if(p->ConnHdl==BT_CONN_HDL_INVALID||p->CryptoOp==BT_SMP_CRYPTO_OP_NONE||!p->bRetryBusy)continue;p->bRetryBusy=false;p->bCryptoWait=true;BtHciDevice_t *d=p->pCryptoDev;BT_SMP_CRYPTO_OP op=p->CryptoOp;int rc=SmpCryptoStart(p);if(rc==BT_SMP_CRYPTO_FAIL){if(op==BT_SMP_CRYPTO_OP_PUBKEY)BtSmpLocalPubKeyReady(d,1,nullptr,nullptr);else BtSmpDhKeyReady(d,1,nullptr);}return;}SmpCryptoPump();}
bool BtSmpCryptoRand(uint8_t *pBuf,size_t Len){if(!s_pCryptoRng||!s_pCryptoRng->IsSecure()){CryptoSecureWipe(pBuf,Len);return false;}CRYPTO_STATUS st=CRYPTO_STATUS_BUSY;for(uint32_t n=0;n<BT_SMP_CRYPTO_BUSY_RETRIES&&st!=CRYPTO_STATUS_OK;n++){st=s_pCryptoRng->Random(pBuf,Len);__asm volatile("":::"memory");}if(st!=CRYPTO_STATUS_OK){CryptoSecureWipe(pBuf,Len);return false;}return true;}
int BtSmpCryptoSelfTest(void){return s_pCryptoEcdh?s_pCryptoEcdh->SelfTest():-1;}
__attribute__((weak)) void BtSmpHciEnableEncryption(BtHciDevice_t * const pDev,uint16_t ConnHdl,uint64_t Rand,uint16_t Ediv,const uint8_t Ltk[16]){uint8_t p[28];p[0]=ConnHdl;p[1]=ConnHdl>>8;for(int i=0;i<8;i++)p[2+i]=Rand>>(8*i);p[10]=Ediv;p[11]=Ediv>>8;memcpy(p+12,Ltk,16);SmpSendHciCmd(pDev,BT_HCI_CMD_CTLR_ENABLE_ENCRYPTION,p,sizeof(p));}
__attribute__((weak)) void BtSmpHciLtkReply(BtHciDevice_t * const pDev,uint16_t ConnHdl,const uint8_t Ltk[16]){uint8_t p[18];p[0]=ConnHdl;p[1]=ConnHdl>>8;memcpy(p+2,Ltk,16);SmpSendHciCmd(pDev,BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_REPLY,p,sizeof(p));}
__attribute__((weak)) void BtSmpHciLtkNegReply(BtHciDevice_t * const pDev,uint16_t ConnHdl){uint8_t p[2]={(uint8_t)ConnHdl,(uint8_t)(ConnHdl>>8)};SmpSendHciCmd(pDev,BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_NEG_REPLY,p,sizeof(p));}

bool BtSmpInit(KeyAgreeEngine *pEcdh,CipherEngine *pAes,RngEngine *pRng)
{
	if(s_pCryptoEcdh)s_pCryptoEcdh->SetCompleteHandler(nullptr,nullptr);SmpCryptoPendingClear();SmpOobClear();for(int i=0;i<BT_SMP_MAX_LINK;i++){SmpEcdhCtxReset(s_SmpLink[i].Ctx.EcdhKeyCtx);CryptoSecureWipe(&s_SmpLink[i],sizeof(s_SmpLink[i]));s_SmpLink[i].ConnHdl=BT_CONN_HDL_INVALID;}bool ok=true;if(!pEcdh||!pEcdh->KeyCtxSize()||pEcdh->KeyCtxSize()>CRYPTO_KEYCTX_MAX||!pEcdh->KeyCtxAlign()||pEcdh->KeyCtxAlign()>CRYPTO_KEYCTX_ALIGN_MAX){pEcdh=nullptr;ok=false;}if(!pAes)ok=false;if(!pRng||!pRng->IsSecure()){pRng=nullptr;ok=false;}s_pCryptoEcdh=pEcdh;s_pCryptoRng=pRng;s_pCryptoAes=pAes;if(s_pCryptoEcdh)s_pCryptoEcdh->SetCompleteHandler(SmpCryptoComplete,&s_CryptoInflight);BtSmpBondLoad();return ok;
}
__attribute__((weak)) void BtSmpAuthConfig(uint8_t IoCaps,uint8_t AuthReq){s_SmpIoCaps=IoCaps;s_SmpAuthReq=(uint8_t)(AuthReq|BT_SMP_AUTHREQ_SC);}
__attribute__((weak)) void BtSmpNumericComparison(uint16_t ConnHdl,uint32_t Value){(void)Value;BtSmpNumericComparisonReply(ConnHdl,false);}
__attribute__((weak)) void BtSmpNumericComparisonReply(uint16_t ConnHdl,bool Confirm)
{
	BtSmpLink_t *p=SmpLinkFind(ConnHdl);if(!p||p->Ctx.Model!=BT_SMP_MODEL_NUMERIC_COMPARISON)return;BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);if(!peer||!peer->pHciDev)return;BtHciDevice_t *d=(BtHciDevice_t*)peer->pHciDev;if(!Confirm){SmpSendFailed(d,ConnHdl,BT_SMP_ERR_NUMERIC_COMPARISON_FAILED);SmpAbortPairing(p);return;}p->Ctx.bAuthenticated=true;uint8_t pa[6],pat;memcpy(pa,peer->Conn.PeerAddr,6);pat=peer->Conn.PeerAddrType;uint8_t la[6],lat=0,z[16]={0};SmpOwnAddrGet(ConnHdl,&lat,la);if(p->Ctx.bInitiator){if(p->Ctx.State!=BT_SMP_STATE_NUMERIC_WAIT)return;uint8_t io[3]={p->Ctx.PReq[1],p->Ctx.PReq[2],p->Ctx.PReq[3]},ea[16];if(!SmpF6(p->Ctx.Mackey,p->Ctx.LocalRand,p->Ctx.PeerRand,z,io,lat,la,pat,pa,ea)){SmpFailAndLock(d,ConnHdl,p,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpDhKeyCheck_t c;c.Code=BT_SMP_CODE_PAIRING_DHKEY_CHECK;memcpy(c.Value,ea,16);SmpSend(d,ConnHdl,&c,sizeof(c));p->Ctx.State=BT_SMP_STATE_DHKEY_CHECK_WAIT;return;}if(p->Ctx.State==BT_SMP_STATE_NUMERIC_WAIT){uint8_t io[3]={p->Ctx.PRsp[1],p->Ctx.PRsp[2],p->Ctx.PRsp[3]},eb[16];if(!SmpF6(p->Ctx.Mackey,p->Ctx.LocalRand,p->Ctx.PeerRand,z,io,lat,la,pat,pa,eb)){SmpFailAndLock(d,ConnHdl,p,BT_SMP_ERR_UNSPECIFIED);return;}BtSmpDhKeyCheck_t c;c.Code=BT_SMP_CODE_PAIRING_DHKEY_CHECK;memcpy(c.Value,eb,16);SmpSend(d,ConnHdl,&c,sizeof(c));p->Ctx.State=BT_SMP_STATE_LTK_WAIT;}
}
__attribute__((weak)) void BtSmpPasskeyDisplay(uint16_t ConnHdl,uint32_t Passkey){(void)Passkey;BtSmpPasskeyReply(ConnHdl,BT_SMP_PASSKEY_INVALID);}
__attribute__((weak)) void BtSmpPasskeyRequest(uint16_t ConnHdl){BtSmpPasskeyReply(ConnHdl,BT_SMP_PASSKEY_INVALID);}
__attribute__((weak)) void BtSmpPasskeyReply(uint16_t ConnHdl,uint32_t Passkey){BtSmpLink_t *p=SmpLinkFind(ConnHdl);if(!p||p->Ctx.Model!=BT_SMP_MODEL_PASSKEY_ENTRY)return;BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);if(!peer||!peer->pHciDev)return;BtHciDevice_t *d=(BtHciDevice_t*)peer->pHciDev;if(Passkey>999999){SmpSendFailed(d,ConnHdl,BT_SMP_ERR_PASSKEY_ENTRY_FAILED);SmpAbortPairing(p);return;}p->Ctx.Passkey=Passkey;p->Ctx.bPkReady=true;if(p->Ctx.bInitiator){if(p->Ctx.State==BT_SMP_STATE_PASSKEY_WAIT)SmpPasskeySendInitiatorConfirm(d,p,ConnHdl);return;}if(p->Ctx.bPkPeerCommit){p->Ctx.bPkPeerCommit=false;SmpPasskeyResponderConfirm(d,p,ConnHdl);}}
__attribute__((weak)) void BtSmpRequestSecurity(uint16_t ConnHdl){BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);if(!peer||!peer->pHciDev)return;BtHciDevice_t *d=(BtHciDevice_t*)peer->pHciDev;s_pSmpActiveDev=d;BtSmpLink_t *p=SmpLinkFind(ConnHdl);if(!p){p=SmpLinkAlloc(ConnHdl);if(!p)return;}BtSmpSecurityReq_t r;r.Code=BT_SMP_CODE_PAIRING_SECURITY_REQ;r.AuthReq=(uint8_t)(s_SmpAuthReq&~BT_SMP_AUTHREQ_KEYPRESS);SmpSend(d,ConnHdl,&r,sizeof(r));}
void BtSmpStartPairing(uint16_t ConnHdl){BtDevice_t *peer=BtPeerFindByHdl(ConnHdl);if(!peer||!peer->pHciDev)return;BtHciDevice_t *d=(BtHciDevice_t*)peer->pHciDev;s_pSmpActiveDev=d;BtSmpLink_t *p=SmpLinkFind(ConnHdl);if(!p){p=SmpLinkAlloc(ConnHdl);if(!p)return;}if(p->Ctx.bLocked||p->Ctx.State!=BT_SMP_STATE_IDLE)return;if(BtSmpBondKeysLookup(ConnHdl,0,0,&p->Keys)){p->Ctx.bInitiator=true;p->Ctx.State=BT_SMP_STATE_DONE;BtSmpHciEnableEncryption(d,ConnHdl,p->Keys.Rand,p->Keys.Ediv,p->Keys.Ltk);return;}p->Ctx.bInitiator=true;p->Ctx.bSc=true;p->Ctx.TmrStart=BtSmpMsTick();p->bTimedOut=false;p->bLocalKeyDistSent=false;BtSmpPairingReq_t r;r.Code=BT_SMP_CODE_PAIRING_REQ;r.IOCaps=s_SmpIoCaps;r.OOBFlag=SmpOobPeerReady(p)&&(s_SmpAuthReq&BT_SMP_AUTHREQ_SC)?BT_SMP_OOB_AUTH_PRESENT:BT_SMP_OOB_AUTH_NOT_PRESENT;r.AuthReq=(uint8_t)(s_SmpAuthReq&~BT_SMP_AUTHREQ_KEYPRESS);r.MaxKeySize=BT_SMP_MAX_ENC_KEY_SIZE;r.InitiatorKeyDist=BT_SMP_KEYDIST_IDKEY|BT_SMP_KEYDIST_SIGNKEY;r.ResponderKeyDist=BT_SMP_KEYDIST_IDKEY|BT_SMP_KEYDIST_SIGNKEY;memcpy(p->Ctx.PReq,&r,7);p->Ctx.IoCaps=r.IOCaps;p->Ctx.AuthReq=r.AuthReq;SmpSend(d,ConnHdl,&r,sizeof(r));p->Ctx.State=BT_SMP_STATE_PUBKEY_LOCAL_WAIT;if(s_SmpOob.bLocalValid)return;if(SmpLocalKeyGen(d,p)==BT_SMP_CRYPTO_FAIL){SmpSendFailed(d,ConnHdl,BT_SMP_ERR_UNSPECIFIED);SmpAbortPairing(p);}}

__attribute__((weak)) int BtSmpOobLocalDataGen(BtHciDevice_t * const pDev,uint8_t * const pRand,uint8_t * const pConf){if(!pRand||!pConf||s_SmpOob.bReserved||s_CryptoInflight.Op!=BT_SMP_CRYPTO_OP_NONE||!s_pCryptoEcdh||s_pCryptoEcdh->IsAsync())return -1;SmpOobClear();int rc=BtSmpCryptoP256KeyGen(pDev,s_SmpOob.LocalPubKey);if(rc!=BT_SMP_CRYPTO_OK||!s_pCryptoRng||s_pCryptoRng->Random(s_SmpOob.LocalRand,16)!=CRYPTO_STATUS_OK){SmpOobClear();return -1;}uint8_t x[32];SmpP256CoordBeToSmpLe(s_SmpOob.LocalPubKey,x);if(!SmpF4(x,x,s_SmpOob.LocalRand,0,pConf)){SmpOobClear();return -1;}CryptoSecureWipe(x,32);memcpy(pRand,s_SmpOob.LocalRand,16);s_SmpOob.bLocalValid=true;return 0;}
__attribute__((weak)) void BtSmpOobPeerDataSet(const uint8_t * const pRand,const uint8_t * const pConf){if(!pRand||!pConf||s_SmpOob.bReserved)return;CryptoSecureWipe(s_SmpOob.PeerRand,16);CryptoSecureWipe(s_SmpOob.PeerConfirm,16);memcpy(s_SmpOob.PeerRand,pRand,16);memcpy(s_SmpOob.PeerConfirm,pConf,16);s_SmpOob.bPeerValid=true;}
__attribute__((weak)) void BtSmpOobDataClear(void){if(!s_SmpOob.bReserved)SmpOobClear();}
void BtSmpDisconnected(uint16_t ConnHdl){SmpLinkFree(ConnHdl);}
void BtSmpTimeoutCheck(void){SmpCryptoRetryPending();for(int i=0;i<BT_SMP_MAX_LINK;i++){BtSmpLink_t *p=&s_SmpLink[i];if(p->ConnHdl==BT_CONN_HDL_INVALID||p->Ctx.bLocked)continue;if(SmpPairingTimedOut(p)){SmpTimeoutAbort(p);}else if(p->Ctx.State==BT_SMP_STATE_KEYDIST&&(uint32_t)(BtSmpMsTick()-p->Ctx.TmrStart)>=BT_SMP_TIMEOUT_MS){SmpTimeoutAbort(p);}}}

int BtSmpF4SelfTest(void){static const uint8_t U[32]={0xe6,0x9d,0x35,0x0e,0x48,0x01,0x03,0xcc,0xdb,0xfd,0xf4,0xac,0x11,0x91,0xf4,0xef,0xb9,0xa5,0xf9,0xe9,0xa7,0x83,0x2c,0x5e,0x2c,0xbe,0x97,0xf2,0xd2,0x03,0xb0,0x20};static const uint8_t V[32]={0xfd,0xc5,0x7f,0xf4,0x49,0xdd,0x4f,0x6b,0xfb,0x7c,0x9d,0xf1,0xc2,0x9a,0xcb,0x59,0x2a,0xe7,0xd4,0xee,0xfb,0xfc,0x0a,0x90,0x9a,0xbb,0xf6,0x32,0x3d,0x8b,0x18,0x55};static const uint8_t X[16]={0xab,0xae,0x2b,0x71,0xec,0xb2,0xff,0xff,0x3e,0x73,0x77,0xd1,0x54,0x84,0xcb,0xd5};static const uint8_t e[16]={0x2d,0x87,0x74,0xa9,0xbe,0xa1,0xed,0xf1,0x1c,0xbd,0xa9,0x07,0xf1,0x16,0xc9,0xf2};uint8_t o[16];return SmpF4(U,V,X,0,o)&&memcmp(o,e,16)==0?0:-1;}
int BtSmpC1S1SelfTest(void){static const uint8_t k[16]={0};static const uint8_t r[16]={0xe0,0x2e,0x70,0xc6,0x4e,0x27,0x88,0x63,0x0e,0x6f,0xad,0x56,0x21,0xd5,0x83,0x57};static const uint8_t preq[7]={1,1,0,0,16,7,7},pres[7]={2,3,0,0,8,0,5},ia[6]={0xa6,0xa5,0xa4,0xa3,0xa2,0xa1},ra[6]={0xb6,0xb5,0xb4,0xb3,0xb2,0xb1},ce[16]={0x86,0x3b,0xf1,0xbe,0xc5,0x4d,0xa7,0xd2,0xea,0x88,0x89,0x87,0xef,0x3f,0x1e,0x1e};uint8_t c[16];if(!SmpC1(k,r,preq,pres,1,ia,0,ra,c)||memcmp(c,ce,16))return -1;static const uint8_t r1[16]={0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11,9,10,11,12,13,14,15,0},r2[16]={0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11,8,7,6,5,4,3,2,1},se[16]={0x84,0x0c,0xa8,0x42,0x3e,0x54,0x3c,0x82,0x53,0x37,0x65,0x88,0xed,0x47,0xbb,0x42};uint8_t s[16];return SmpS1(k,r1,r2,s)&&memcmp(s,se,16)==0?0:-1;}

static bool SmpIrkPresent(const uint8_t Irk[16]){for(int i=0;i<16;i++)if(Irk[i])return true;return false;}
static void SmpAh(const uint8_t Irk[16],const uint8_t PrandLe[3],uint8_t HashLe[3]){uint8_t key[16],rp[16]={0},out[16];for(int i=0;i<16;i++)key[i]=Irk[15-i];rp[13]=PrandLe[2];rp[14]=PrandLe[1];rp[15]=PrandLe[0];SmpAes(key,rp,out);HashLe[0]=out[15];HashLe[1]=out[14];HashLe[2]=out[13];}
bool BtSmpRpaResolve(const uint8_t Irk[16],const uint8_t Rpa[6]){if(!Irk||!Rpa||!SmpIrkPresent(Irk))return false;uint8_t h[3];SmpAh(Irk,Rpa+3,h);return h[0]==Rpa[0]&&h[1]==Rpa[1]&&h[2]==Rpa[2];}
bool BtSmpRpaGen(const uint8_t Irk[16],uint8_t Rpa[6]){if(!Irk||!Rpa||!SmpIrkPresent(Irk)||!BtSmpCryptoRand(Rpa+3,3))return false;Rpa[5]=(uint8_t)((Rpa[5]&0x3f)|0x40);SmpAh(Irk,Rpa+3,Rpa);return true;}
#pragma pack(push,1)
typedef struct __Bt_Smp_Local_Id_Record{uint32_t Magic;uint16_t Version;uint16_t Len;uint8_t Irk[16];uint32_t Crc;}BtSmpLocalIdRecord_t;
#pragma pack(pop)
#define BT_SMP_LOCALID_MAGIC 0x49504D53U
#define BT_SMP_LOCALID_VERSION 1U
static struct{bool bValid;uint8_t Irk[16];}s_SmpLocalId;
__attribute__((weak)) void BtSmpLocalIdSave(void){}
size_t BtSmpLocalIdRecordSize(void){return sizeof(BtSmpLocalIdRecord_t);}
size_t BtSmpLocalIdSerialize(void *pBuff,size_t BuffLen){if(!s_SmpLocalId.bValid||!pBuff||BuffLen<sizeof(BtSmpLocalIdRecord_t))return 0;BtSmpLocalIdRecord_t r;r.Magic=BT_SMP_LOCALID_MAGIC;r.Version=BT_SMP_LOCALID_VERSION;r.Len=sizeof(r);memcpy(r.Irk,s_SmpLocalId.Irk,16);r.Crc=crc32_ieee((uint8_t*)&r,offsetof(BtSmpLocalIdRecord_t,Crc));memcpy(pBuff,&r,sizeof(r));CryptoSecureWipe(&r,sizeof(r));return sizeof(BtSmpLocalIdRecord_t);}
bool BtSmpLocalIdRestore(const void *pRec,size_t Len){if(!pRec||Len!=sizeof(BtSmpLocalIdRecord_t))return false;BtSmpLocalIdRecord_t r;memcpy(&r,pRec,sizeof(r));if(r.Magic!=BT_SMP_LOCALID_MAGIC||r.Version!=BT_SMP_LOCALID_VERSION||r.Len!=sizeof(r)||r.Crc!=crc32_ieee((uint8_t*)&r,offsetof(BtSmpLocalIdRecord_t,Crc))){CryptoSecureWipe(&r,sizeof(r));return false;}memcpy(s_SmpLocalId.Irk,r.Irk,16);s_SmpLocalId.bValid=true;CryptoSecureWipe(&r,sizeof(r));return true;}
bool BtSmpLocalIrkGet(uint8_t Irk[16]){if(!Irk)return false;if(!s_SmpLocalId.bValid){if(!BtSmpCryptoRand(s_SmpLocalId.Irk,16))return false;s_SmpLocalId.bValid=true;BtSmpLocalIdSave();}memcpy(Irk,s_SmpLocalId.Irk,16);return true;}
int BtSmpRpaSelfTest(void){static const uint8_t irk[16]={0x9b,0x7d,0x39,0x0a,0xa6,0x10,0x10,0x34,0x05,0xad,0xc8,0x57,0xa3,0x34,0x02,0xec},rpa[6]={0xaa,0xfb,0x0d,0x94,0x81,0x70};if(!BtSmpRpaResolve(irk,rpa))return -1;uint8_t g[6];if(!BtSmpRpaGen(irk,g))return -2;if((g[5]&0xc0)!=0x40)return -3;return BtSmpRpaResolve(irk,g)?0:-4;}
void BtSmpSignMac(const uint8_t Csrk[16],const uint8_t *pMsg,size_t Len,uint8_t Mac[8]){uint8_t key[16],mac[16];for(int i=0;i<16;i++)key[i]=Csrk[15-i];SmpAesCmac(key,pMsg,Len,mac);for(int i=0;i<8;i++)Mac[i]=mac[7-i];}
int BtSmpSignSelfTest(void){static const uint8_t c[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16},m[8]={0xd2,3,0,0x55,0,0,0,0},e[8]={0x28,0xda,0x5d,0x44,0x8a,0xce,0x8e,0xd5};uint8_t o[8];BtSmpSignMac(c,m,8,o);return memcmp(o,e,8)==0?0:-1;}
