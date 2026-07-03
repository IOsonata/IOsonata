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
#include "crypto/crypto.h"
#include "coredev/rng.h"

// SMP handshake trace. Set to 1 to print every SMP PDU in/out and the link
// state over SysLog. Leave at 0 for release builds.
#if 1
#include "syslog.h"
#define SMP_TRACE(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)

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

#define SMP_TRACE_PDU(dir, code, state) \
		SMP_TRACE("SMP " dir " %s state=%d\r\n", SmpCodeName(code), state)
#else
#define SMP_TRACE(...)
#define SMP_TRACE_PDU(dir, code, state)
#endif

#ifndef BT_SMP_MAX_LINK
#define BT_SMP_MAX_LINK		4
#endif

#ifndef BT_SMP_LOCAL_IOCAPS
#define BT_SMP_LOCAL_IOCAPS		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#endif

#ifndef BT_SMP_LOCAL_AUTHREQ
#define BT_SMP_LOCAL_AUTHREQ	(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC)
#endif

typedef struct __Bt_Smp_Link {
	uint16_t    ConnHdl;
	BtSmpCtx_t  Ctx;
	BtSmpKeys_t Keys;
} BtSmpLink_t;

static BtSmpLink_t s_SmpLink[BT_SMP_MAX_LINK];

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
			s_SmpLink[i].ConnHdl = ConnHdl;
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
		memset(p, 0, sizeof(BtSmpLink_t));
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
	uint16_t hdl    = pLink->ConnHdl;
	uint8_t  fc     = pLink->Ctx.FailCount;
	bool     locked = pLink->Ctx.bLocked;

	memset(&pLink->Ctx, 0, sizeof(pLink->Ctx));
	memset(&pLink->Keys, 0, sizeof(pLink->Keys));

	pLink->ConnHdl       = hdl;
	pLink->Ctx.FailCount = fc;
	pLink->Ctx.bLocked   = locked;
	pLink->Ctx.State     = BT_SMP_STATE_IDLE;
}

//-----------------------------------------------------------------------------
// Pairing timeout (Core Vol 3 Part H 3.4)
//-----------------------------------------------------------------------------

static void SmpSendFailed(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint8_t Reason);

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
	pLink->Ctx.State = BT_SMP_STATE_IDLE;
	pLink->Ctx.bLocked = true;
	BtSmpPairingComplete(ConnHdl, false, nullptr);
}

//-----------------------------------------------------------------------------
// Outbound packet helpers
//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------
// Crypto primitives
//-----------------------------------------------------------------------------

static BtHciDevice_t *s_pSmpActiveDev = nullptr;

// Runtime association configuration. Defaults mirror the compile-time
// BT_SMP_LOCAL_* values, i.e. Just Works, until BtSmpAuthConfig overrides them.
static uint8_t s_SmpIoCaps  = BT_SMP_LOCAL_IOCAPS;
static uint8_t s_SmpAuthReq = BT_SMP_LOCAL_AUTHREQ;

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
static void SmpC1(const uint8_t k[16], const uint8_t r[16],
				  const uint8_t preq[7], const uint8_t pres[7],
				  uint8_t iat, const uint8_t ia[6],
				  uint8_t rat, const uint8_t ra[6], uint8_t out[16])
{
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
}

// s1(k, r1, r2)
static void SmpS1(const uint8_t k[16], const uint8_t r1[16],
				  const uint8_t r2[16], uint8_t out[16])
{
	uint8_t r[16];

	memcpy(&r[0], &r2[0], 8);
	memcpy(&r[8], &r1[0], 8);
	SmpAes(k, r, out);
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
static void SmpF4(const uint8_t u[32], const uint8_t v[32],
				  const uint8_t x[16], uint8_t z, uint8_t out[16])
{
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
static void SmpF5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16],
				  uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
				  uint8_t mackey[16], uint8_t ltk[16])
{
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
}

// f6 -> DHKey check value.
//
// Inputs and output are in SMP byte order.
static void SmpF6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16],
				  const uint8_t r[16], const uint8_t iocap[3],
				  uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
				  uint8_t out[16])
{
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
}

//-----------------------------------------------------------------------------
// Pairing feature negotiation
//-----------------------------------------------------------------------------

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
static uint32_t SmpNumericValue(BtSmpLink_t *pLink)
{
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

	return SmpG2(pkaX, pkbX, na, nb) % 1000000;
}

static void SmpBuildPairingRsp(BtSmpLink_t *pLink, BtSmpPairingRsp_t *pRsp)
{
	pRsp->Code = BT_SMP_CODE_PAIRING_RSP;
	pRsp->IOCaps = s_SmpIoCaps;
	pRsp->OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
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
static void SmpApplyKeySize(BtSmpKeys_t *pKeys)
{
	int ks = pKeys->EncKeySize;
	if (ks < BT_SMP_MIN_ENC_KEY_SIZE)
	{
		ks = BT_SMP_MIN_ENC_KEY_SIZE;	// defensive; negotiation already enforced the floor
	}
	for (int i = ks; i < 16; i++)
	{
		pKeys->Ltk[i] = 0;
	}
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
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	memcpy(pLink->Ctx.PReq, pReq, 7);
	pLink->Ctx.PeerAuthReq = pReq->AuthReq;
	pLink->Ctx.bInitiator = false;
	pLink->Ctx.bSc = (pReq->AuthReq & BT_SMP_AUTHREQ_SC) &&
					 (BT_SMP_LOCAL_AUTHREQ & BT_SMP_AUTHREQ_SC);

	// This build requires LE Secure Connections. If the peer requests legacy
	// pairing (no SC bit), reject with Authentication Requirements rather than
	// entering the legacy flow. A central that opens with legacy (e.g. nRF
	// Connect Desktop) then retries with SC. Accepting legacy here and failing
	// later at the confirm/random step leaves such a central wedged.
	if ((BT_SMP_LOCAL_AUTHREQ & BT_SMP_AUTHREQ_SC) &&
		!(pReq->AuthReq & BT_SMP_AUTHREQ_SC))
	{
		SMP_TRACE("SMP reject legacy (peer auth=0x%02x), require SC\r\n",
				  pReq->AuthReq);
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	pLink->Keys.EncKeySize = pReq->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ?
							 pReq->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;

	// Select the association model now both IO capabilities are known: the peer
	// is the initiator, the local device is the responder. MITM applies when
	// either side sets the flag. Only Just Works is wired so far; any MITM model
	// fails closed rather than continuing to an unauthenticated link the peer
	// would treat as authenticated.
	bool mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) ||
				(pReq->AuthReq & BT_SMP_AUTHREQ_MITM);
	pLink->Ctx.Model = SmpSelectModel(pReq->IOCaps, s_SmpIoCaps, mitm, false);
	SMP_TRACE("SMP model=%d init_io=%d resp_io=%d mitm=%d\r\n",
			  pLink->Ctx.Model, pReq->IOCaps, s_SmpIoCaps, mitm ? 1 : 0);
	if (pLink->Ctx.Model != BT_SMP_MODEL_JUST_WORKS &&
		pLink->Ctx.Model != BT_SMP_MODEL_NUMERIC_COMPARISON &&
		pLink->Ctx.Model != BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		// OOB is not wired yet; fail closed rather than continuing to a link
		// the peer would treat as authenticated.
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	BtSmpPairingRsp_t rsp;
	SmpBuildPairingRsp(pLink, &rsp);
	SmpSend(pDev, ConnHdl, &rsp, sizeof(rsp));

	if (pLink->Ctx.bSc)
	{
		pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
		int rc = BtSmpCryptoP256KeyGen(pDev, pLink->Ctx.LocalPubKey);
		SMP_TRACE("SMP P256KeyGen rc=%d\r\n", rc);
		if (rc == BT_SMP_CRYPTO_FAIL)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
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

	pLink->Ctx.State = BT_SMP_STATE_DHKEY_WAIT;
	int rc = BtSmpCryptoEcdh(pDev, pLink->Ctx.PeerPubKey, pLink->Ctx.DhKey);
	if (rc == BT_SMP_CRYPTO_OK)
	{
		BtSmpDhKeyReady(pDev, 0, pLink->Ctx.DhKey);
	}
	else if (rc == BT_SMP_CRYPTO_FAIL)
	{
		return false;
	}

	return true;
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
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	// LE Secure Connections only build.
	if (!(pRsp->AuthReq & BT_SMP_AUTHREQ_SC))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	memcpy(pLink->Ctx.PRsp, pRsp, 7);
	pLink->Ctx.PeerAuthReq = pRsp->AuthReq;
	pLink->Ctx.bSc = true;
	pLink->Keys.EncKeySize = pRsp->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ?
							 pRsp->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;

	// Select the association model now both IO capabilities are known: the local
	// device is the initiator, the peer is the responder. MITM applies when
	// either side sets the flag. Only Just Works is wired so far; any MITM model
	// fails closed rather than continuing to an unauthenticated link the peer
	// would treat as authenticated.
	bool mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) ||
				(pRsp->AuthReq & BT_SMP_AUTHREQ_MITM);
	pLink->Ctx.Model = SmpSelectModel(s_SmpIoCaps, pRsp->IOCaps, mitm, false);
	SMP_TRACE("SMP model=%d init_io=%d resp_io=%d mitm=%d\r\n",
			  pLink->Ctx.Model, s_SmpIoCaps, pRsp->IOCaps, mitm ? 1 : 0);
	if (pLink->Ctx.Model != BT_SMP_MODEL_JUST_WORKS &&
		pLink->Ctx.Model != BT_SMP_MODEL_NUMERIC_COMPARISON &&
		pLink->Ctx.Model != BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		// OOB is not wired yet; fail closed rather than continuing to a link
		// the peer would treat as authenticated.
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_AUTHEN_REQUIREMENTS);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
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
		pLink->Ctx.State = BT_SMP_STATE_DHKEY_WAIT;
		int rc = BtSmpCryptoEcdh(pDev, pLink->Ctx.PeerPubKey, pLink->Ctx.DhKey);
		if (rc == BT_SMP_CRYPTO_OK)
		{
			BtSmpDhKeyReady(pDev, 0, pLink->Ctx.DhKey);
		}
		else if (rc == BT_SMP_CRYPTO_FAIL)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
		}
		return;
	}

	if (!SmpTryStartDhKey(pDev, pLink, ConnHdl))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
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

// Fill the 16 octet r value for the DHKey Check f6. Zero for Just Works and
// Numeric Comparison; the passkey (SMP little-endian, low octet first) for
// Passkey Entry. f6 reverses r internally to big-endian.
static void SmpDhKeyCheckR(const BtSmpLink_t *pLink, uint8_t r[16])
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

	BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);					// Nai
	uint8_t ra = SmpPasskeyRa(pLink->Ctx.Passkey, pLink->Ctx.PkRound);

	BtSmpPairingConfirm_t cf;
	cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	SmpF4(localX, peerX, pLink->Ctx.LocalRand, ra, cf.Value);
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

	BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);					// Nbi
	uint8_t rb = SmpPasskeyRa(pLink->Ctx.Passkey, pLink->Ctx.PkRound);

	BtSmpPairingConfirm_t cf;
	cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	SmpF4(localX, peerX, pLink->Ctx.LocalRand, rb, cf.Value);
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

	SmpF5(dhKeySmp, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  localAddrType, localAddr, PeerAddrType, PeerAddr,
		  pLink->Ctx.Mackey, pLink->Ctx.Ltk);

	memcpy(pLink->Keys.Ltk, pLink->Ctx.Ltk, 16);
	pLink->Keys.bSc = true;
	pLink->Keys.bValid = true;
	pLink->Keys.bAuthenticated = true;
	SmpApplyKeySize(&pLink->Keys);

	uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
	uint8_t rChk[16];
	SmpDhKeyCheckR(pLink, rChk);
	uint8_t ea[16];
	SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  rChk, iocapA,
		  localAddrType, localAddr, PeerAddrType, PeerAddr, ea);

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

	SmpF5(dhKeySmp, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
		  PeerAddrType, PeerAddr, localAddrType, localAddr,
		  pLink->Ctx.Mackey, pLink->Ctx.Ltk);

	memcpy(pLink->Keys.Ltk, pLink->Ctx.Ltk, 16);
	pLink->Keys.bSc = true;
	pLink->Keys.bValid = true;
	pLink->Keys.bAuthenticated = true;
	SmpApplyKeySize(&pLink->Keys);

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
		SmpF4(peerX, localX, pLink->Ctx.PeerRand, r, cb);
		if (!SmpEqualCT(cb, pLink->Ctx.PeerConfirm, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
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
	SmpF4(peerX, localX, pLink->Ctx.PeerRand, r, ca);
	if (!SmpEqualCT(ca, pLink->Ctx.PeerConfirm, 16))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
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
		BtSmpCryptoRand(rnd, 4);
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

	BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);

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

	SmpC1(pLink->Ctx.Tk, pLink->Ctx.LocalRand,
		  pLink->Ctx.PReq, pLink->Ctx.PRsp,
		  iat, ia, rat, ra, pLink->Ctx.LocalConfirm);

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

	if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
	{
		SmpPasskeyHandleRandom(pDev, pLink, ConnHdl, peerAddr, peerAddrType);
		return;
	}

	if (pLink->Ctx.bInitiator)
	{
		// Initiator (SC Just Works): received responder nonce Nb. First verify
		// the responder Confirm Cb = f4(PKbx, PKax, Nb, 0).
		uint8_t localX[32];
		uint8_t peerX[32];
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);	// PKax
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);		// PKbx

		uint8_t cb[16];
		SmpF4(peerX, localX, pLink->Ctx.PeerRand, 0, cb);
		if (!SmpEqualCT(cb, pLink->Ctx.PeerConfirm, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			return;
		}

		uint8_t localAddr[6];
		uint8_t localAddrType = 0;
		BtSmpLocalAddrGet(&localAddrType, localAddr);

		uint8_t dhKeySmp[32];
		SmpReverse32(pLink->Ctx.DhKey, dhKeySmp);

		// f5(DHKey, Na, Nb, A1=initiator(local), A2=responder(peer)).
		SmpF5(dhKeySmp, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  localAddrType, localAddr, peerAddrType, peerAddr,
			  pLink->Ctx.Mackey, pLink->Ctx.Ltk);

		memcpy(pLink->Keys.Ltk, pLink->Ctx.Ltk, 16);
		pLink->Keys.bSc = true;
		pLink->Keys.bValid = true;
		SmpApplyKeySize(&pLink->Keys);

		if (pLink->Ctx.Model == BT_SMP_MODEL_NUMERIC_COMPARISON)
		{
			// Hold the initiator DHKey Check (Ea) and have the application
			// display the value for the user to confirm. The flow resumes from
			// BtSmpNumericComparisonReply.
			uint32_t v = SmpNumericValue(pLink);
			pLink->Ctx.State = BT_SMP_STATE_NUMERIC_WAIT;
			BtSmpNumericComparison(ConnHdl, v);
			return;
		}

		// Ea = f6(MacKey, Na, Nb, 0, IOcapA, A=initiator(local), B=responder(peer)).
		uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
		uint8_t zeroR[16] = {0};
		uint8_t ea[16];
		SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  zeroR, iocapA,
			  localAddrType, localAddr, peerAddrType, peerAddr, ea);

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

		SmpF5(dhKeySmp, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  peerAddrType, peerAddr, localAddrType, localAddr,
			  pLink->Ctx.Mackey, pLink->Ctx.Ltk);

		memcpy(pLink->Keys.Ltk, pLink->Ctx.Ltk, 16);
		pLink->Keys.bSc = true;
		pLink->Keys.bValid = true;
		SmpApplyKeySize(&pLink->Keys);
		// Park in DHKEY_CHECK_WAIT before the user callback. A synchronous
		// reject inside BtSmpNumericComparison takes the reply path and sets
		// the state to IDLE; do not overwrite it afterward.
		pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
		if (pLink->Ctx.Model == BT_SMP_MODEL_NUMERIC_COMPARISON)
		{
			// Both nonces are known: display the value for the user. The
			// responder DHKey Check (Eb) is held in the DHKey Check handler
			// until the user confirms through BtSmpNumericComparisonReply.
			uint32_t v = SmpNumericValue(pLink);
			BtSmpNumericComparison(ConnHdl, v);
		}
		return;
	}

	uint8_t calc[16];
	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	SmpC1(pLink->Ctx.Tk, pLink->Ctx.PeerRand,
		  pLink->Ctx.PReq, pLink->Ctx.PRsp,
		  peerAddrType, peerAddr, localAddrType, localAddr, calc);

	if (!SmpEqualCT(calc, pLink->Ctx.PeerConfirm, 16))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_CONFIRM_VALUE_FAILED);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	BtSmpPairingRandom_t rnd;
	rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
	memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);
	SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));

	SmpS1(pLink->Ctx.Tk, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand, pLink->Ctx.Ltk);
	memcpy(pLink->Keys.Ltk, pLink->Ctx.Ltk, 16);
	pLink->Keys.Rand = 0;
	pLink->Keys.Ediv = 0;
	pLink->Keys.bSc = false;
	pLink->Keys.bValid = true;
	SmpApplyKeySize(&pLink->Keys);

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
		SmpDhKeyCheckR(pLink, rChk);
		uint8_t eb[16];
		SmpF6(pLink->Ctx.Mackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  rChk, iocapB,
			  peerAddrType, peerAddr, localAddrType, localAddr, eb);

		if (!SmpEqualCT(eb, pChk->Value, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			return;
		}

		// Authentication complete. As the central, start link encryption from
		// the derived LTK (SC: Rand and EDIV are zero).
		pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
		BtSmpHciEnableEncryption(pDev, ConnHdl, 0, 0, pLink->Keys.Ltk);
		return;
	}

	uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
	uint8_t iocapB[3] = { pLink->Ctx.PRsp[1], pLink->Ctx.PRsp[2], pLink->Ctx.PRsp[3] };
	uint8_t rChk[16];
	SmpDhKeyCheckR(pLink, rChk);

	uint8_t ea[16];
	SmpF6(pLink->Ctx.Mackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
		  rChk, iocapA,
		  peerAddrType, peerAddr, localAddrType, localAddr, ea);

	if (!SmpEqualCT(ea, pChk->Value, 16))
	{
		// The Confirm stage already passed, so ECDH produced the same point as
		// the peer. If Ea fails here, the most common remaining issue is DHKey
		// byte order at f5. Try the other provider order before failing.
		uint8_t altMackey[16];
		uint8_t altLtk[16];
		uint8_t altEa[16];

		SmpF5(pLink->Ctx.DhKey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  peerAddrType, peerAddr, localAddrType, localAddr,
			  altMackey, altLtk);

		SmpF6(altMackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  rChk, iocapA,
			  peerAddrType, peerAddr, localAddrType, localAddr, altEa);

		SMP_TRACE("Ea mismatch cur0=%02x alt0=%02x peer0=%02x\r\n",
				  ea[0], altEa[0], pChk->Value[0]);

		if (!SmpEqualCT(altEa, pChk->Value, 16))
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			return;
		}

		memcpy(pLink->Ctx.Mackey, altMackey, 16);
		memcpy(pLink->Ctx.Ltk, altLtk, 16);
		memcpy(pLink->Keys.Ltk, altLtk, 16);
		SmpApplyKeySize(&pLink->Keys);
		memcpy(ea, altEa, 16);
		SMP_TRACE("Ea matched with raw DHKey f5 input\r\n");
	}

	if (pLink->Ctx.Model == BT_SMP_MODEL_NUMERIC_COMPARISON &&
		!pLink->Keys.bAuthenticated)
	{
		// Ea verified. Hold the responder DHKey Check (Eb) until the user
		// confirms the displayed value through BtSmpNumericComparisonReply.
		pLink->Ctx.State = BT_SMP_STATE_NUMERIC_WAIT;
		return;
	}

	uint8_t eb[16];
	SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  rChk, iocapB,
		  localAddrType, localAddr, peerAddrType, peerAddr, eb);

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
			const BtSmpPairingFailed_t *pFail = (const BtSmpPairingFailed_t*)pSmp;
			if (Len >= sizeof(BtSmpPairingFailed_t))
			{
				SMP_TRACE("SMP RX Failed reason=0x%02x\r\n", pFail->Reason);
			}
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			BtSmpPairingComplete(ConnHdl, false, nullptr);

			// H5 repeated attempts (Core Vol 3 Part H 2.3.6): count the failure
			// and, once too many have occurred on this connection, lock the link
			// so further pairing is refused until it disconnects. Otherwise keep
			// the slot (with the counter) so the next attempt is still counted.
			pLink->Ctx.FailCount++;
			if (pLink->Ctx.FailCount >= BT_SMP_MAX_PAIR_ATTEMPTS)
			{
				pLink->Ctx.bLocked = true;
			}
			else
			{
				SmpLinkResetKeepCount(pLink);
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

	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		BtSmpLink_t *pLink = &s_SmpLink[i];

		if (pLink->ConnHdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}

		if (pLink->Ctx.State != BT_SMP_STATE_PUBKEY_WAIT &&
			pLink->Ctx.State != BT_SMP_STATE_PUBKEY_LOCAL_WAIT)
		{
			continue;
		}

		if (Status != 0)
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
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
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
		}
	}
}

void BtSmpDhKeyReady(BtHciDevice_t * const pDev, uint8_t Status, const uint8_t *pDhKey)
{
	s_pSmpActiveDev = pDev;
	SMP_TRACE("SMP DhKeyReady status=%d\r\n", Status);

	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		BtSmpLink_t *pLink = &s_SmpLink[i];

		if (pLink->ConnHdl == BT_CONN_HDL_INVALID ||
			pLink->Ctx.State != BT_SMP_STATE_DHKEY_WAIT)
		{
			continue;
		}

		if (Status != 0)
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			continue;
		}

		memcpy(pLink->Ctx.DhKey, pDhKey, 32);

		if (pLink->Ctx.Model == BT_SMP_MODEL_PASSKEY_ENTRY)
		{
			SmpPasskeyBegin(pDev, pLink, pLink->ConnHdl);
			continue;
		}

		if (pLink->Ctx.bInitiator)
		{
			// Initiator (SC Just Works): generate our nonce Na and wait for the
			// responder Confirm Cb. The initiator does not send a Confirm.
			BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);
			pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
			continue;
		}

		// LE Secure Connections responder:
		// Cb = f4(PKbx, PKax, Nb, 0)
		// Public keys are stored big-endian for ECDH; f4 uses the SMP
		// little-endian X-coordinate byte order.
		BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);

		uint8_t localX[32];
		uint8_t peerX[32];
		SmpP256CoordBeToSmpLe(&pLink->Ctx.LocalPubKey[0], localX);
		SmpP256CoordBeToSmpLe(&pLink->Ctx.PeerPubKey[0], peerX);

		BtSmpPairingConfirm_t cf;
		cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
		SmpF4(localX, peerX, pLink->Ctx.LocalRand, 0, cf.Value);
		memcpy(pLink->Ctx.LocalConfirm, cf.Value, 16);

		{
			uint8_t cb2[16];
			SmpF4(localX, peerX, pLink->Ctx.LocalRand, 0, cb2);
			bool stable = (memcmp(cb2, cf.Value, 16) == 0);
			SMP_TRACE("Cb stable=%d firstbyte=%02x Nb0=%02x\r\n",
					  stable ? 1 : 0, cf.Value[0], pLink->Ctx.LocalRand[0]);
		}

		SmpSend(pDev, pLink->ConnHdl, &cf, sizeof(cf));

		// After responder Confirm, SC Just Works waits for initiator Random Na.
		pLink->Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
	}
}

void BtSmpProcessLtkRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint64_t Rand, uint16_t Ediv)
{
	uint8_t key[16];
	bool have = false;

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);

	if (pLink != nullptr && pLink->Keys.bValid &&
		(pLink->Ctx.State == BT_SMP_STATE_LTK_WAIT || Ediv == 0))
	{
		memcpy(key, pLink->Keys.Ltk, 16);
		have = true;
	}
	else
	{
		extern bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
									   uint16_t Ediv, uint8_t Ltk[16]);
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

void BtSmpEncryptionChanged(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint8_t Status, uint8_t Enabled)
{
	SMP_TRACE("SMP EncChange status=%d enabled=%d\r\n", Status, Enabled);

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer != nullptr)
	{
		pPeer->bSecure = (Status == 0) && (Enabled != 0);

		// Produce the generic security state the ATT permission checks consume.
		// An authenticated association model (numeric comparison; passkey / OOB
		// when added) raises the level above ENC_UNAUTH. Authenticated Secure
		// Connections is LESC_AUTH; authenticated legacy would be ENC_AUTH.
		BtConnSec_t sec;
		memset(&sec, 0, sizeof(sec));
		if (pPeer->bSecure)
		{
			BtSmpLink_t *pSecLink = SmpLinkFind(ConnHdl);
			sec.KeySize = (pSecLink != nullptr) ? pSecLink->Keys.EncKeySize
												: BT_SMP_MAX_ENC_KEY_SIZE;

			bool authd = (pSecLink != nullptr) && pSecLink->Keys.bAuthenticated;
			bool sc    = (pSecLink != nullptr) && pSecLink->Keys.bSc;

			if (authd)
			{
				sec.Level = sc ? BT_GAP_SEC_LEVEL_LESC_AUTH
							   : BT_GAP_SEC_LEVEL_ENC_AUTH;
			}
			else
			{
				sec.Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
			}
			if (sc)
			{
				sec.Flags |= BT_GAP_SEC_FLAG_SC;
			}
			if (BtSmpBonded(ConnHdl))
			{
				sec.Flags |= BT_GAP_SEC_FLAG_BONDED;
			}
		}
		BtGapConnSecSet(ConnHdl, &sec);
	}

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);

	if (Status != 0 || Enabled == 0)
	{
		if (pLink != nullptr)
		{
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
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
			BtSmpCryptoRand(irk, 16);
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
		}

		if (localKeyDist & BT_SMP_KEYDIST_SIGNKEY)
		{
			uint8_t csrk[16];
			BtSmpCryptoRand(csrk, 16);
			BtSmpSigningInfo_t si;
			si.Code = BT_SMP_CODE_PAIRING_SIGNING_INFO;
			memcpy(si.Csrk, csrk, 16);
			SmpSend(pDev, ConnHdl, &si, sizeof(si));
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
// Crypto composition (three capability slots: ECDH, AES, RNG)
//
// SMP needs three primitives. The slots are
// filled by whatever chips a board has), bt_smp holds three CryptoDev_t slots
// and routes each operation to the engine in that slot. The application fills
// the slots via BtSmpInit from whatever engines its target provides: one engine
// may fill all three (mbedtls / CC310), or they compose from several (uECC for
// ECDH + controller for AES + hardware RNG on a BLE-only part with no
// CryptoCell). Each engine advertises only the capabilities it has; BtSmpInit
// validates each slot before use. The five BtSmpCrypto* wrappers keep their
// existing signatures so the state machine is unchanged; they translate
// CRYPTO_STATUS to the BT_SMP_CRYPTO_* codes the state machine expects.
//-----------------------------------------------------------------------------

static CryptoDev_t *s_pCryptoEcdh = nullptr;	// CRYPTO_CAP_ECDH_P256
static CryptoDev_t *s_pCryptoAes  = nullptr;	// CRYPTO_CAP_AES128_ECB

static int CryptoStatusToSmp(CRYPTO_STATUS st)
{
	switch (st)
	{
	case CRYPTO_STATUS_OK:      return BT_SMP_CRYPTO_OK;
	case CRYPTO_STATUS_PENDING: return BT_SMP_CRYPTO_PENDING;
	default:                    return BT_SMP_CRYPTO_FAIL;
	}
}

void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
								  const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	(void)pDev;		// the engine holds whatever handle it needs in pDevData
	if (CryptoAes128Ecb(s_pCryptoAes, Key, In, Out, nullptr) != CRYPTO_STATUS_OK)
	{
		memset(Out, 0, 16);		// fail loud: confirm check will mismatch
	}
}

int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64])
{
	(void)pDev;
	return CryptoStatusToSmp(CryptoEcdhP256KeyGen(s_pCryptoEcdh, nullptr, pPubKey, nullptr));
}

int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32])
{
	(void)pDev;
	return CryptoStatusToSmp(CryptoEcdhP256(s_pCryptoEcdh, nullptr, pPeerPubKey, pDhKey, nullptr));
}

void BtSmpCryptoRand(uint8_t *pBuf, size_t Len)
{
	// RNG is a target utility (coredev/rng.h), not a crypto engine: hardware
	// where the MCU has an RNG peripheral, weak software default otherwise.
	RngGet(pBuf, Len);
}

int BtSmpCryptoSelfTest(void)
{
	// Run the ECDH engine's self-test (the security-critical path). AES/RNG
	// engines may have their own; the ECDH known-answer vector is the one that
	// matters for SC pairing.
	return CryptoSelfTest(s_pCryptoEcdh);
}

// LTK Request Reply hooks. The active implementation overrides these to route the
// reply through its real HCI command channel (e.g. the SDC command function).
// The weak default uses the generic HCI command builder, which is correct for
// transports where the HCI command and ACL data share one sink.

// Central-only: start link encryption from a (just-derived or stored) LTK via
// HCI LE Enable Encryption. The peripheral instead waits for the controller's
// LTK request and answers BtSmpHciLtkReply. For SC the Rand/Ediv are zero. The
// LTK is little-endian (HCI order). Weak: the active stack overrides this to use
// its real HCI command channel (the SDC command function), same as LtkReply.
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

void BtSmpInit(CryptoDev_t *pEcdh, CryptoDev_t *pAes)
{
	// Compose the crypto from the engines the target provides. Each slot must
	// advertise the capability it is being used for; a mismatch leaves the slot
	// null so the corresponding BtSmpCrypto* wrapper fails loud. RNG is not a
	// slot - it is the target RngGet utility, called directly by BtSmpCryptoRand.
	s_pCryptoEcdh = CryptoIsCapable(pEcdh, CRYPTO_CAP_ECDH_P256) ? pEcdh : nullptr;
	s_pCryptoAes  = CryptoIsCapable(pAes,  CRYPTO_CAP_AES128_ECB) ? pAes  : nullptr;

	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		s_SmpLink[i].ConnHdl = BT_CONN_HDL_INVALID;
	}

	// Repopulate the RAM bond table from non-volatile storage. The default
	// BtSmpBondLoad is a weak no-op (RAM-only); a flash-backed platform
	// overrides it and calls BtSmpBondRestore for each persisted slot.
	BtSmpBondLoad();
}

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
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	// Values matched: the link is MITM authenticated.
	pLink->Keys.bAuthenticated = true;

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
		SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  zeroR, iocapA,
			  localAddrType, localAddr, peerAddrType, peerAddr, ea);

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
		SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
			  zeroR, iocapB,
			  localAddrType, localAddr, peerAddrType, peerAddr, eb);

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
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
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

	// Bonded reconnect: re-encrypt from a stored SC bond, matched by the peer
	// address (SC bonds carry EDIV/Rand zero), instead of pairing again. No key
	// distribution runs on reconnect, so go straight to DONE before enabling
	// encryption; BtSmpEncryptionChanged completes it.
	extern bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
								   uint16_t Ediv, uint8_t Ltk[16]);
	uint8_t ltk[16];
	if (BtSmpBondLtkLookup(ConnHdl, 0, 0, ltk))
	{
		memcpy(pLink->Keys.Ltk, ltk, 16);
		pLink->Keys.bValid = true;
		pLink->Keys.bSc = true;
		pLink->Ctx.bInitiator = true;
		pLink->Ctx.State = BT_SMP_STATE_DONE;
		SMP_TRACE("SMP central reconnect, encrypt from bond\r\n");
		BtSmpHciEnableEncryption(pDev, ConnHdl, 0, 0, ltk);
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
	req.OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
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
	int rc = BtSmpCryptoP256KeyGen(pDev, pLink->Ctx.LocalPubKey);
	if (rc == BT_SMP_CRYPTO_FAIL)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
	}
}

void BtSmpDisconnected(uint16_t ConnHdl)
{
	SmpLinkFree(ConnHdl);
}

void BtSmpTimeoutCheck(void)
{
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
	SmpF4(U, V, X, 0, out);
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
