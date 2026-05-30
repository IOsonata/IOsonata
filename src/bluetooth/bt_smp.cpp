/**-------------------------------------------------------------------------
@file	bt_smp.cpp

@brief	Bluetooth Security Manager Protocol (SMP)

Generic implementation of Bluetooth Security Manager Protocol. Responder
(peripheral) role is implemented end to end for Just Works pairing over both
the legacy and LE Secure Connections paths, plus the controller Long Term
Key request reply that starts link encryption (the path bonded reconnection
relies on).

Crypto primitives are taken from the controller through HCI:
  AES-128 single block -> LE Encrypt command (CTLR_ENCRYPT)
  P-256 public key     -> LE Read Local P-256 Public Key
  DHKey                -> LE Generate DHKey
The SC f4/f5/f6/g2 functions are built on AES-CMAC over LE Encrypt and are
marked where they sit; the only piece intentionally left to the chip layer
is the controller round-trip plumbing for P-256/DHKey, which differs per
backend.

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
#include "bluetooth/bt_smp_crypto.h"

// SMP handshake trace. Set to 1 to print every SMP PDU in/out and the link
// state over SysLog (same pattern as bt_hci_host.cpp DEBUG_PRINTF). Leave at
// 0 for release builds - it compiles out entirely.
#if 1
#include "syslog.h"
#define SMP_TRACE(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)

// Human-readable SMP opcode name for the trace (only compiled when tracing on).
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
// SMP_TRACE that also resolves the opcode name.
#define SMP_TRACE_PDU(dir, code, state) \
		SMP_TRACE("SMP " dir " %s state=%d\r\n", SmpCodeName(code), state)
#else
#define SMP_TRACE(...)
#define SMP_TRACE_PDU(dir, code, state)
#endif

#ifndef BT_SMP_MAX_LINK
// One SMP context per concurrent link. Sized small for a peripheral; a
// central raises it to match its peer pool. Kept here, not in BtDevice_t,
// so the peer record stays untouched and the transient pairing buffers
// (which dwarf the persistent key set) do not bloat every bonded record.
#define BT_SMP_MAX_LINK		4
#endif

// Local IO capability and authentication policy. A headless peripheral is
// Just Works only (no MITM). Override at build time to advertise passkey or
// numeric comparison once an IO backend exists.
#ifndef BT_SMP_LOCAL_IOCAPS
#define BT_SMP_LOCAL_IOCAPS		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#endif
#ifndef BT_SMP_LOCAL_AUTHREQ
#define BT_SMP_LOCAL_AUTHREQ	(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC)
#endif

typedef struct __Bt_Smp_Link {
	uint16_t    ConnHdl;		//!< Owning connection handle, BT_CONN_HDL_INVALID when free
	BtSmpCtx_t  Ctx;			//!< Transient pairing state
	BtSmpKeys_t Keys;			//!< Working key set, copied to the peer on completion
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

// Build and send an SMP PDU over the link (L2CAP CID 6, one ACL fragment).
// Mirrors the ATT response path in bt_hci_host.cpp so the framing stays
// consistent across the stack.
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

// Send a raw HCI command through the same data path the controller layer
// uses. The command opcode/param layout matches Core Vol 4, Part E.
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
//
// All crypto goes through the provider hooks in bt_smp_crypto.h. This file
// names no provider: BtSmpCryptoAes128 / P256KeyGen / Ecdh are weak hooks,
// each target links exactly one provider impl (HCI offload by default,
// mbedTLS as the portable fallback).
//
// The AES toolbox (c1/s1/f4/f5/f6 and AES-CMAC) treats the AES hook as a
// pure synchronous function and needs the HCI device to reach a controller-
// offload provider. Rather than thread pDev through every toolbox function,
// the active device is parked at file scope for the duration of one SMP
// event (the stack runs SMP to completion, single threaded), and SmpAes()
// wraps the hook with it.
//-----------------------------------------------------------------------------

static BtHciDevice_t *s_pSmpActiveDev = nullptr;

static inline void SmpAes(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16])
{
	BtSmpCryptoAes128(s_pSmpActiveDev, Key, In, Out);
}

// AES-CMAC (RFC 4493) over the provider AES. Used by the SC f-functions.
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

	uint8_t x[16] = {0};
	uint8_t y[16];

	for (size_t i = 0; i < n - 1; i++)
	{
		for (int j = 0; j < 16; j++) { y[j] = x[j] ^ pMsg[i*16 + j]; }
		SmpAes(Key, y, x);
	}

	uint8_t last[16];
	size_t rem = Len - (n - 1) * 16;
	if (lastComplete)
	{
		for (int j = 0; j < 16; j++) { last[j] = pMsg[(n-1)*16 + j] ^ k1[j]; }
	}
	else
	{
		for (size_t j = 0; j < 16; j++)
		{
			uint8_t mb = (j < rem) ? pMsg[(n-1)*16 + j] : (j == rem ? 0x80 : 0x00);
			last[j] = mb ^ k2[j];
		}
	}
	for (int j = 0; j < 16; j++) { y[j] = x[j] ^ last[j]; }
	SmpAes(Key, y, Mac);
}

// --- Legacy toolbox (Vol 3, Part H, 2.2.3) -------------------------------
// c1(k, r, preq, pres, iat, ia, rat, ra)
static void SmpC1(const uint8_t k[16], const uint8_t r[16],
				  const uint8_t preq[7], const uint8_t pres[7],
				  uint8_t iat, const uint8_t ia[6],
				  uint8_t rat, const uint8_t ra[6], uint8_t out[16])
{
	uint8_t p1[16], p2[16], tmp[16];

	// p1 = pres || preq || rat || iat
	p1[0] = iat;
	p1[1] = rat;
	memcpy(&p1[2], preq, 7);
	memcpy(&p1[9], pres, 7);

	// p2 = padding || ia || ra
	memset(p2, 0, 16);
	memcpy(&p2[0], ra, 6);
	memcpy(&p2[6], ia, 6);

	for (int i = 0; i < 16; i++) { tmp[i] = r[i] ^ p1[i]; }
	SmpAes(k, tmp, tmp);
	for (int i = 0; i < 16; i++) { tmp[i] ^= p2[i]; }
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

// --- SC toolbox (Vol 3, Part H, 2.2.5 onward) ----------------------------
// f4(U, V, X, Z) -> confirm
static void SmpF4(const uint8_t u[32], const uint8_t v[32],
				  const uint8_t x[16], uint8_t z, uint8_t out[16])
{
	uint8_t m[65];
	memcpy(&m[0], u, 32);
	memcpy(&m[32], v, 32);
	m[64] = z;
	SmpAesCmac(x, m, sizeof(m), out);
}

// f5 -> MacKey || LTK. Both halves returned via mackey[16], ltk[16].
static void SmpF5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16],
				  uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
				  uint8_t mackey[16], uint8_t ltk[16])
{
	static const uint8_t salt[16] = {
		0x6C,0x88,0x83,0x91,0xAA,0xF5,0xA5,0x38,
		0x60,0x37,0x0B,0xDB,0x5A,0x60,0x83,0xBE };
	uint8_t t[16];
	SmpAesCmac(salt, w, 32, t);

	uint8_t m[53];
	// counter || keyID("btle") || N1 || N2 || A1 || A2 || length(256)
	m[0] = 0;					// counter, filled below
	m[1] = 0x62; m[2] = 0x74; m[3] = 0x6C; m[4] = 0x65;	// "btle"
	memcpy(&m[5], n1, 16);
	memcpy(&m[21], n2, 16);
	m[37] = a1t; memcpy(&m[38], a1, 6);
	m[44] = a2t; memcpy(&m[45], a2, 6);
	m[51] = 0x01; m[52] = 0x00;	// length = 256 bits, big endian

	m[0] = 0; SmpAesCmac(t, m, sizeof(m), mackey);
	m[0] = 1; SmpAesCmac(t, m, sizeof(m), ltk);
}

// f6 -> check value (Ea / Eb)
static void SmpF6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16],
				  const uint8_t r[16], const uint8_t iocap[3],
				  uint8_t a1t, const uint8_t a1[6], uint8_t a2t, const uint8_t a2[6],
				  uint8_t out[16])
{
	uint8_t m[65];
	memcpy(&m[0], n1, 16);
	memcpy(&m[16], n2, 16);
	memcpy(&m[32], r, 16);
	memcpy(&m[48], iocap, 3);
	m[51] = a1t; memcpy(&m[52], a1, 6);
	m[58] = a2t; memcpy(&m[59], a2, 6);
	SmpAesCmac(w, m, sizeof(m), out);
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

	// Key distribution: the responder MUST NOT set any bit the initiator did
	// not request (Vol 3, Part H, 3.6.1). The initiator's Pairing Request
	// carries the two key-dist octets it is willing to do; we mask our
	// preferred set against them. Setting bits the central never offered is
	// what a strict central rejects as SMP_ERR_INVALID_PARAMS.
	const BtSmpPairingReq_t *pReq = (const BtSmpPairingReq_t*)pLink->Ctx.PReq;
	uint8_t reqInitKeyDist = pReq->InitiatorKeyDist;
	uint8_t reqRespKeyDist = pReq->ResponderKeyDist;

	// What we are willing to handle. For SC the LTK is derived, not
	// distributed, so EncKey is never set on the SC path.
	uint8_t wantKeyDist = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;
	if (!pLink->Ctx.bSc)
	{
		// Legacy: the LTK is distributed via EncKey if the peer asked for it.
		wantKeyDist |= BT_SMP_KEYDIST_ENCKEY;
	}

	pRsp->InitiatorKeyDist = reqInitKeyDist & wantKeyDist;
	pRsp->ResponderKeyDist = reqRespKeyDist & wantKeyDist;

	pLink->Ctx.IoCaps = pRsp->IOCaps;
	pLink->Ctx.AuthReq = pRsp->AuthReq;
	memcpy(pLink->Ctx.PRsp, pRsp, 7);
}

//-----------------------------------------------------------------------------
// Inbound SMP PDU handling (responder role)
//-----------------------------------------------------------------------------

static void SmpHandlePairingReq(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								uint16_t ConnHdl, const BtSmpPairingReq_t *pReq)
{
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
	// SC iff both sides set the SC bit.
	pLink->Ctx.bSc = (pReq->AuthReq & BT_SMP_AUTHREQ_SC) &&
					 (BT_SMP_LOCAL_AUTHREQ & BT_SMP_AUTHREQ_SC);
	pLink->Keys.EncKeySize = pReq->MaxKeySize < BT_SMP_MAX_ENC_KEY_SIZE ?
							 pReq->MaxKeySize : BT_SMP_MAX_ENC_KEY_SIZE;

	BtSmpPairingRsp_t rsp;
	SmpBuildPairingRsp(pLink, &rsp);
	SmpSend(pDev, ConnHdl, &rsp, sizeof(rsp));

	if (pLink->Ctx.bSc)
	{
		// SC: peer sends its public key next; we also need our own. Ask the
		// provider. A software provider returns OK immediately; the HCI
		// offload provider returns PENDING and BtSmpLocalPubKeyReady resumes.
		pLink->Ctx.State = BT_SMP_STATE_PUBKEY_WAIT;
		int rc = BtSmpCryptoP256KeyGen(pDev, pLink->Ctx.LocalPubKey);
		SMP_TRACE("SMP P256KeyGen rc=%d\r\n", rc);
		if (rc == BT_SMP_CRYPTO_FAIL)
		{
			SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_UNSPECIFIED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
		}
		// On OK the local key is in LocalPubKey now; SmpHandlePublicKey will
		// find it populated when the peer key arrives. On PENDING it is filled
		// later by BtSmpLocalPubKeyReady. Both converge in SmpTryStartDhKey.
	}
	else
	{
		// Legacy Just Works: TK is all zeros, wait for peer confirm.
		memset(pLink->Ctx.Tk, 0, 16);
		pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
	}
}

// Once both public keys are present, send ours (if not already sent) and
// kick off ECDH. Idempotent guard via the DHKEY_WAIT state. Returns false on
// a hard provider failure (caller fails the pairing).
static bool SmpTryStartDhKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
							 uint16_t ConnHdl)
{
	bool localIn = (pLink->Ctx.LocalPubKey[0] | pLink->Ctx.LocalPubKey[1]) != 0;
	bool peerIn  = (pLink->Ctx.PeerPubKey[0]  | pLink->Ctx.PeerPubKey[1])  != 0;
	if (!localIn || !peerIn)
	{
		return true;	// not ready yet, no error
	}

	// Send our public key to the peer. Internal is big-endian; the air PDU is
	// little-endian, so reverse each 32-byte coordinate.
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
		// Software provider: DHKey is ready now. Drive the same path the
		// async completion would.
		BtSmpDhKeyReady(pDev, 0, pLink->Ctx.DhKey);
	}
	else if (rc == BT_SMP_CRYPTO_FAIL)
	{
		return false;
	}
	// PENDING: BtSmpDhKeyReady arrives via the controller event.
	return true;
}

static void SmpHandlePublicKey(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
							   uint16_t ConnHdl, const BtSmpPublicKey_t *pPk)
{
	// The Public Key PDU carries the coordinates little-endian on air (SMP is
	// a host-layer protocol, multi-octet fields are little-endian). The f4/f5
	// toolbox and ECDH use them big-endian, so reverse each 32-byte coord.
	for (int i = 0; i < 32; i++)
	{
		pLink->Ctx.PeerPubKey[i]      = pPk->KeyX[31 - i];
		pLink->Ctx.PeerPubKey[32 + i] = pPk->KeyY[31 - i];
	}

	// Converge: if our local key is also in, this sends ours and starts ECDH;
	// otherwise it is a no-op until BtSmpLocalPubKeyReady fills the local key.
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
		// SC Just Works / numeric comparison: the initiator does NOT send a
		// Pairing Confirm, so reaching here is unexpected for those methods
		// (only passkey entry has an initiator confirm, not yet supported as
		// responder). Store it but take no action: do not regenerate the
		// nonce and do not send a random here. Our Nb was committed in
		// BtSmpDhKeyReady and is sent when Na (Pairing Random) arrives.
		return;
	}

	// Legacy: responder generates Srand, computes Sconfirm = c1(TK, Srand,...)
	// and sends Sconfirm.
	BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	uint8_t ra[6] = {0}, ia[6] = {0};
	uint8_t rat = 0, iat = 0;
	if (pPeer != nullptr)
	{
		// Responder address = local; initiator address = peer.
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
	uint8_t peerAddr[6] = {0}; uint8_t peerAddrType = 0;
	if (pPeer != nullptr)
	{
		memcpy(peerAddr, pPeer->Conn.PeerAddr, 6);
		peerAddrType = pPeer->Conn.PeerAddrType;
	}

	if (pLink->Ctx.bSc)
	{
		// SC Just Works / numeric comparison: the INITIATOR does not send a
		// Pairing Confirm (only the responder does). So there is no peer
		// confirm to verify here - the arriving Pairing Random is Na.
		//
		// Sequence (Vol 3, Part H, 2.3.5.6.2): we already sent our
		// Cb = f4(PKbx, PKax, Nb, 0) in BtSmpDhKeyReady, committing
		// pLink->Ctx.LocalRand as Nb. Now that Na arrived, reply with that
		// SAME Nb - regenerating it makes the initiator's check of Cb fail
		// (this was the CONFIRM_VALUE bug). PeerRand already holds Na.
		BtSmpPairingRandom_t rnd;
		rnd.Code = BT_SMP_CODE_PAIRING_RANDOM;
		memcpy(rnd.Value, pLink->Ctx.LocalRand, 16);	// Nb, unchanged
		SmpSend(pDev, ConnHdl, &rnd, sizeof(rnd));

		// Derive MacKey + LTK with f5(DHKey, Na, Nb, A_init, A_resp).
		// A_init = peer (central), A_resp = local.
		uint8_t localAddr[6]; uint8_t localAddrType = 0;
		BtSmpLocalAddrGet(&localAddrType, localAddr);
		SmpF5(pLink->Ctx.DhKey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
			  peerAddrType, peerAddr, localAddrType, localAddr,
			  pLink->Ctx.Mackey, pLink->Ctx.Ltk);

		memcpy(pLink->Keys.Ltk, pLink->Ctx.Ltk, 16);
		pLink->Keys.bSc = true;
		pLink->Keys.bValid = true;
		pLink->Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
		// Wait for peer DHKey Check (Ea), then send ours (Eb).
		return;
	}

	// Legacy: verify Mconfirm == c1(TK, Mrand, preq, pres, iat, ia, rat, ra).
	// ia/iat = initiator (peer/central), ra/rat = responder (us). Our own
	// address and type must match what the central saw, or c1 mismatches.
	uint8_t calc[16];
	uint8_t localAddr[6]; uint8_t localAddrType = 0;
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

	// Send Srand and compute STK = s1(TK, Srand, Mrand).
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

	// Legacy STK encrypts the link via the controller LTK request that the
	// central triggers. Wait for it.
	pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
}

static void SmpHandleDhKeyCheck(BtHciDevice_t * const pDev, BtSmpLink_t *pLink,
								uint16_t ConnHdl, const BtSmpDhKeyCheck_t *pChk)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	uint8_t peerAddr[6] = {0}; uint8_t peerAddrType = 0;
	if (pPeer != nullptr)
	{
		memcpy(peerAddr, pPeer->Conn.PeerAddr, 6);
		peerAddrType = pPeer->Conn.PeerAddrType;
	}
	uint8_t localAddr[6]; uint8_t localAddrType = 0;
	BtSmpLocalAddrGet(&localAddrType, localAddr);

	// IOcap of the initiator (peer): AuthReq, OOB, IOCaps from cached PReq.
	uint8_t iocapA[3] = { pLink->Ctx.PReq[3], pLink->Ctx.PReq[2], pLink->Ctx.PReq[1] };
	uint8_t iocapB[3] = { pLink->Ctx.PRsp[3], pLink->Ctx.PRsp[2], pLink->Ctx.PRsp[1] };

	// Ea = f6(MacKey, Mrand, Srand, 0, IOcapA, A_init, A_resp). Just Works: ra=0.
	uint8_t zeroR[16] = {0};
	uint8_t ea[16];
	SmpF6(pLink->Ctx.Mackey, pLink->Ctx.PeerRand, pLink->Ctx.LocalRand,
		  zeroR, iocapA,
		  peerAddrType, peerAddr, localAddrType, localAddr, ea);

	if (memcmp(ea, pChk->Value, 16) != 0)
	{
		SmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
		pLink->Ctx.State = BT_SMP_STATE_IDLE;
		return;
	}

	// Eb = f6(MacKey, Srand, Mrand, 0, IOcapB, A_resp, A_init).
	uint8_t eb[16];
	SmpF6(pLink->Ctx.Mackey, pLink->Ctx.LocalRand, pLink->Ctx.PeerRand,
		  zeroR, iocapB,
		  localAddrType, localAddr, peerAddrType, peerAddr, eb);

	BtSmpDhKeyCheck_t chk;
	chk.Code = BT_SMP_CODE_PAIRING_DHKEY_CHECK;
	memcpy(chk.Value, eb, 16);
	SmpSend(pDev, ConnHdl, &chk, sizeof(chk));

	// LTK is ready; central will request encryption start.
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
}

static void SmpHandleSigningInfo(BtSmpLink_t *pLink, const BtSmpSigningInfo_t *pInfo)
{
	memcpy(pLink->Keys.Csrk, pInfo->Csrk, 16);
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

	s_pSmpActiveDev = pDev;	// park for the AES toolbox during this event
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
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
			BtSmpPairingComplete(ConnHdl, false, nullptr);
			SmpLinkFree(ConnHdl);
			break;
		case BT_SMP_CODE_PAIRING_SECURITY_REQ:
			// Peripheral does not act on its own Security Request echo.
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

	// The controller key is link-agnostic; apply it to any link awaiting it.
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

		// Converge: starts ECDH iff the peer key is also in.
		if (!SmpTryStartDhKey(pDev, pLink, pLink->ConnHdl))
		{
			SmpSendFailed(pDev, pLink->ConnHdl, BT_SMP_ERR_DHKEY_CHECK_FAILED);
			pLink->Ctx.State = BT_SMP_STATE_IDLE;
		}
	}
}

void BtSmpDhKeyReady(BtHciDevice_t * const pDev, uint8_t Status, const uint8_t *pDhKey)
{
	s_pSmpActiveDev = pDev;	// park for the f4 AES toolbox call below
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

		// SC Just Works: responder picks Srand, sends Cb = f4(PKbx, PKax, Srand, 0).
		BtSmpCryptoRand(pLink->Ctx.LocalRand, 16);
		BtSmpPairingConfirm_t cf;
		cf.Code = BT_SMP_CODE_PAIRING_CONFIRM;
		SmpF4(&pLink->Ctx.LocalPubKey[0], &pLink->Ctx.PeerPubKey[0],
			  pLink->Ctx.LocalRand, 0, cf.Value);
		memcpy(pLink->Ctx.LocalConfirm, cf.Value, 16);
		// Compact self-check: recompute Cb and confirm it is stable, and
		// verify our public key corresponds to our DHKey path. Print short.
		{
			uint8_t cb2[16];
			SmpF4(&pLink->Ctx.LocalPubKey[0], &pLink->Ctx.PeerPubKey[0],
				  pLink->Ctx.LocalRand, 0, cb2);
			bool stable = (memcmp(cb2, cf.Value, 16) == 0);
			SMP_TRACE("Cb stable=%d firstbyte=%02x Nb0=%02x\r\n",
					  stable ? 1 : 0, cf.Value[0], pLink->Ctx.LocalRand[0]);
		}
		SmpSend(pDev, pLink->ConnHdl, &cf, sizeof(cf));
		pLink->Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
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
		// Fresh pairing: SC LTK ignores Rand/EDIV (both 0); legacy STK too.
		memcpy(key, pLink->Keys.Ltk, 16);
		have = true;
	}
	else
	{
		// Reconnection: look up the bonded peer's stored LTK. The peer record
		// carries it after the app reloaded the bond. Match on Rand/EDIV for
		// legacy, accept any for SC (Rand/EDIV are 0).
		BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
		(void)pPeer;
		// Application-supplied bond lookup. Override to read from storage.
		extern bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
									   uint16_t Ediv, uint8_t Ltk[16]);
		have = BtSmpBondLtkLookup(ConnHdl, Rand, Ediv, key);
	}

	if (have)
	{
		// LE Long Term Key Request Reply: ConnHdl(2) || LTK(16).
		uint8_t param[18];
		param[0] = (uint8_t)(ConnHdl & 0xFF);
		param[1] = (uint8_t)(ConnHdl >> 8);
		memcpy(&param[2], key, 16);
		SmpSendHciCmd(pDev, BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_REPLY, param, sizeof(param));
	}
	else
	{
		uint8_t param[2];
		param[0] = (uint8_t)(ConnHdl & 0xFF);
		param[1] = (uint8_t)(ConnHdl >> 8);
		SmpSendHciCmd(pDev, BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_NEG_REPLY, param, sizeof(param));
	}
}

void BtSmpEncryptionChanged(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint8_t Status, uint8_t Enabled)
{
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
		// Link is now encrypted. Responder distributes its IRK/CSRK if asked,
		// then pairing is complete. Key distribution PDUs from the peer arrive
		// after this point and land in the per-link key set.
		pLink->Ctx.State = BT_SMP_STATE_DONE;
		BtSmpPairingComplete(ConnHdl, true, &pLink->Keys);
	}
}

//-----------------------------------------------------------------------------
// Weak application hooks
//-----------------------------------------------------------------------------

__attribute__((weak))
void BtSmpPairingComplete(uint16_t ConnHdl, bool Success, const BtSmpKeys_t *pKeys)
{
	(void)ConnHdl; (void)Success; (void)pKeys;
}

// Weak default: public, all-zero. The active backend MUST override this with
// the real on-air address + type, or every confirm/check value mismatches.
__attribute__((weak))
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = 0;
	memset(pAddr, 0, 6);
}

// Default bond lookup: no persisted bonds. Override to read LTK from NVM.
__attribute__((weak))
bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv, uint8_t Ltk[16])
{
	(void)ConnHdl; (void)Rand; (void)Ediv; (void)Ltk;
	return false;
}

// Weak default for the optional provider self-test: no-op PASS. The SDC
// provider overrides this with the real P-256 DH known-answer test.
__attribute__((weak))
int BtSmpCryptoSelfTest(void)
{
	return 0;
}

// Init / link teardown. Call BtSmpInit once after BtPeerInit; call
// BtSmpDisconnected from the Disconnected handler to release the link slot.
extern "C" void BtSmpInit(void)
{
	for (int i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		s_SmpLink[i].ConnHdl = BT_CONN_HDL_INVALID;
	}
}

extern "C" void BtSmpDisconnected(uint16_t ConnHdl)
{
	SmpLinkFree(ConnHdl);
}
