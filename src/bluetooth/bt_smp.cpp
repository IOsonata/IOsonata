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
	pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

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
	uint8_t buf[sizeof(BtHciCmdPacketHdr_t) + 255];
	BtHciCmd_Packet_t *c = (BtHciCmd_Packet_t*)buf;

	c->Hdr.OpCode = OpCode;
	c->Hdr.ParamLen = ParamLen;
	if (ParamLen > 0 && pParam != nullptr)
	{
		memcpy(c->Param, pParam, ParamLen);
	}

	pDev->SendData((uint8_t*)c, sizeof(BtHciCmdPacketHdr_t) + ParamLen);
}

//-----------------------------------------------------------------------------
// Crypto primitives
//-----------------------------------------------------------------------------

static BtHciDevice_t *s_pSmpActiveDev = nullptr;

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

static void SmpBuildPairingRsp(BtSmpLink_t *pLink, BtSmpPairingRsp_t *pRsp)
{
	pRsp->Code = BT_SMP_CODE_PAIRING_RSP;
	pRsp->IOCaps = BT_SMP_LOCAL_IOCAPS;
	pRsp->OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
	pRsp->AuthReq = (uint8_t)(BT_SMP_LOCAL_AUTHREQ & ~BT_SMP_AUTHREQ_KEYPRESS);
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

	if (pReq->MaxKeySize < BT_SMP_MIN_ENC_KEY_SIZE ||
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

	if (!SmpTryStartDhKey(pDev, pLink, ConnHdl))
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
	}
}

static void SmpHandlePairingConfirm(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
									uint16_t ConnHdl, const BtSmpPairingConfirm_t *pC)
{
	memcpy(pLink->Ctx.PeerConfirm, pC->Value, 16);

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
		pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
		return;
	}

	uint8_t calc[16];
	uint8_t localAddr[6];
	uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	SmpC1(pLink->Ctx.Tk, pLink->Ctx.PeerRand,
		  pLink->Ctx.PReq, pLink->Ctx.PRsp,
		  peerAddrType, peerAddr, localAddrType, localAddr, calc);

	if (memcmp(calc, pLink->Ctx.PeerConfirm, 16) != 0)
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

	uint8_t iocapA[3] = { pLink->Ctx.PReq[1], pLink->Ctx.PReq[2], pLink->Ctx.PReq[3] };
	uint8_t iocapB[3] = { pLink->Ctx.PRsp[1], pLink->Ctx.PRsp[2], pLink->Ctx.PRsp[3] };
	uint8_t zeroR[16] = {0};

	uint8_t ea[16];
	SmpF6(pLink->Ctx.Mackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
		  zeroR, iocapA,
		  peerAddrType, peerAddr, localAddrType, localAddr, ea);

	if (memcmp(ea, pChk->Value, 16) != 0)
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
			  zeroR, iocapA,
			  peerAddrType, peerAddr, localAddrType, localAddr, altEa);

		SMP_TRACE("Ea mismatch cur0=%02x alt0=%02x peer0=%02x\r\n",
				  ea[0], altEa[0], pChk->Value[0]);

		if (memcmp(altEa, pChk->Value, 16) != 0)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			return;
		}

		memcpy(pLink->Ctx.Mackey, altMackey, 16);
		memcpy(pLink->Ctx.Ltk, altLtk, 16);
		memcpy(pLink->Keys.Ltk, altLtk, 16);
		memcpy(ea, altEa, 16);
		SMP_TRACE("Ea matched with raw DHKey f5 input\r\n");
	}

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
	// Peer identity now known. Refresh the stored bond so it carries the peer
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

	// Validate the PDU carries enough bytes for its code before any handler
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

	switch (pSmp->Code)
	{
		case BT_SMP_CODE_PAIRING_REQ:
			SmpHandlePairingReq(pDev, pLink, ConnHdl, (const BtSmpPairingReq_t*)pSmp);
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
			break;

		case BT_SMP_CODE_PAIRING_ID_INFO:
			SmpHandleIdInfo(pLink, (const BtSmpIdInfo_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_ID_ADDR_INFO:
			SmpHandleIdAddrInfo(pLink, (const BtSmpIdAddrInfo_t*)pSmp);
			break;

		case BT_SMP_CODE_PAIRING_SIGNING_INFO:
			SmpHandleSigningInfo(pLink, (const BtSmpSigningInfo_t*)pSmp);
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
			SmpLinkFree(ConnHdl);
			break;
		}

		case BT_SMP_CODE_PAIRING_SECURITY_REQ:
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

		if (pLink->Ctx.State != BT_SMP_STATE_PUBKEY_WAIT)
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
		// backend it must go through sdc_hci_cmd_le_long_term_key_request_reply,
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
	}

	BtSmpLink_t *pLink = SmpLinkFind(ConnHdl);
	if (pLink == nullptr)
	{
		return;
	}

	if (Status != 0 || Enabled == 0)
	{
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		BtSmpPairingComplete(ConnHdl, false, nullptr);
		return;
	}

	if (pLink->Ctx.State == BT_SMP_STATE_LTK_WAIT)
	{
		// Encrypted. Run the key distribution phase the peer negotiated
		// (ResponderKeyDist = PRsp[6]). For SC the LTK is derived (EncKey is
		// never distributed); we send IRK + identity address (IDKEY) and/or
		// CSRK (SIGNKEY) only if negotiated. The peer's pairing procedure does
		// not complete until it has received the responder keys it expects.
		uint8_t respKeyDist = pLink->Ctx.PRsp[6] &
							  (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY);
		SMP_TRACE("SMP encrypted, distribute rk=%02x\r\n", respKeyDist);

		if (respKeyDist & BT_SMP_KEYDIST_IDKEY)
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

		if (respKeyDist & BT_SMP_KEYDIST_SIGNKEY)
		{
			uint8_t csrk[16];
			BtSmpCryptoRand(csrk, 16);
			BtSmpSigningInfo_t si;
			si.Code = BT_SMP_CODE_PAIRING_SIGNING_INFO;
			memcpy(si.Csrk, csrk, 16);
			SmpSend(pDev, ConnHdl, &si, sizeof(si));
		}

		pLink->Ctx.State = BT_SMP_STATE_DONE;
		BtSmpBondAdd(ConnHdl, &pLink->Keys);
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

extern "C" void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
								  const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	(void)pDev;		// the engine holds whatever handle it needs in pDevData
	if (CryptoAes128Ecb(s_pCryptoAes, Key, In, Out, nullptr) != CRYPTO_STATUS_OK)
	{
		memset(Out, 0, 16);		// fail loud: confirm check will mismatch
	}
}

extern "C" int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64])
{
	(void)pDev;
	return CryptoStatusToSmp(CryptoEcdhP256KeyGen(s_pCryptoEcdh, pPubKey, nullptr));
}

extern "C" int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
							   const uint8_t pPeerPubKey[64], uint8_t pDhKey[32])
{
	(void)pDev;
	return CryptoStatusToSmp(CryptoEcdhP256(s_pCryptoEcdh, pPeerPubKey, pDhKey, nullptr));
}

extern "C" void BtSmpCryptoRand(uint8_t *pBuf, size_t Len)
{
	// RNG is a target utility (coredev/rng.h), not a crypto engine: hardware
	// where the MCU has an RNG peripheral, weak software default otherwise.
	RngGet(pBuf, Len);
}

extern "C" int BtSmpCryptoSelfTest(void)
{
	// Run the ECDH engine's self-test (the security-critical path). AES/RNG
	// engines may have their own; the ECDH known-answer vector is the one that
	// matters for SC pairing.
	return CryptoSelfTest(s_pCryptoEcdh);
}

// LTK Request Reply hooks. The active backend overrides these to route the
// reply through its real HCI command channel (e.g. the SDC command function).
// The weak default uses the generic HCI command builder, which is correct for
// transports where the HCI command and ACL data share one sink.
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

extern "C" void BtSmpInit(CryptoDev_t *pEcdh, CryptoDev_t *pAes)
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

// Peripheral-initiated security. Sends an SMP Security Request to prompt the
// central to either encrypt with an existing bond or start pairing. This is
// the standard way a peripheral signals that it wants a secure/bonded link;
// without it a central has no indication to pair (and apps like nRF Connect
// show no security/bond action), and any pairing the user forces has no
// natural begin/end in the central security flow.
extern "C" void BtSmpRequestSecurity(uint16_t ConnHdl)
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
	req.AuthReq = (uint8_t)(BT_SMP_LOCAL_AUTHREQ & ~BT_SMP_AUTHREQ_KEYPRESS);
	SMP_TRACE("SMP TX SecurityReq auth=0x%02x\r\n", req.AuthReq);
	SmpSend(pDev, ConnHdl, &req, sizeof(req));
}

extern "C" void BtSmpDisconnected(uint16_t ConnHdl)
{
	SmpLinkFree(ConnHdl);
}

// f4 self-test against the BLE spec worked example (Vol 3 Part H, D.2).
// Verifies SmpF4 + AES-CMAC + AES are correct on the target. Returns 0 on PASS.
extern "C" int BtSmpF4SelfTest(void)
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
