/**-------------------------------------------------------------------------
@file	bt_smp_pubkey_test.cpp

@brief	Peer public key validation for LE Secure Connections.

Core Vol 3 Part H 2.3.5.6.1. Before the ECDH the responder has to refuse a
peer public key that would let the peer predict the confirm values. The
reflection case is a peer that echoes the local key back; f4 consumes only the
X coordinate (Vol 3 Part H 2.2.6), so the test that catches it has to compare
the X coordinate and nothing else.

Same harness as bt_smp_keydist_test and bt_smp_pairing_policy_test: the pairing
state and the key material are file-static, so the source is included to reach
them. Unlike those two this one binds real crypto engines - software P-256, AES
and RNG - so a legitimate peer key really completes the key agreement. That is
what makes the control case worth anything: a refusal has to be the validation
firing, not the absence of an engine.

@author	Hoang Nguyen Hoan
@date	Aug. 13, 2026

@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softrng.h"
#include "crypto/crypto_uecc.h"

#include "bt_smp_link_test_stubs.h"

#include "../../../src/bluetooth/bt_smp.cpp"

namespace {

int s_Checks = 0;
int s_Failures = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

constexpr uint16_t kConnHdl = 0x0046;

BtHciDevice_t s_Dev = {};
BtDevice_t s_Peer = {};
uint8_t s_Role = BT_CONN_ROLE_PERIPHERAL;

// CryptoUecc refuses to generate a key from an RNG that does not claim to be
// secure, and CryptoSoftRng correctly does not claim it. A host test has no
// entropy source and does not need one - the keys here only have to be valid
// points, not unpredictable - so the claim is made here, in the test, where it
// is visible.
class TestRng : public CryptoSoftRng {
public:
	bool IsSecure() const override { return true; }
};

alignas(uint64_t) uint8_t s_EcdhMem[CRYPTO_UECC_MEMSIZE];
alignas(uint64_t) uint8_t s_AesMem[CRYPTO_SOFTAES_MEMSIZE];
TestRng s_Rng;
KeyAgreeEngine *s_pEcdh = nullptr;
CipherEngine *s_pAes = nullptr;

// P-256 field prime, big endian. Vol 3 Part H 2.3.5.6.1 names the curve; the
// value is FIPS 186-4 D.1.2.3.
const uint8_t kP256Prime[32] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

// Out = kP256Prime - In, big endian. For a point (x, y) on the curve, (x, p-y)
// is the negation and is also on the curve: y^2 and (p-y)^2 are the same
// residue. So it passes an on-curve check while being a different 64 octet
// key.
void FieldNegate(const uint8_t In[32], uint8_t Out[32])
{
	int borrow = 0;
	for (int i = 31; i >= 0; i--)
	{
		int d = (int)kP256Prime[i] - (int)In[i] - borrow;
		borrow = d < 0;
		Out[i] = (uint8_t)(d + (borrow ? 256 : 0));
	}
}

bool CryptoBringUp()
{
	s_pEcdh = CryptoUeccCreate(s_EcdhMem, sizeof(s_EcdhMem), &s_Rng);
	s_pAes = CryptoSoftAesCreate(s_AesMem, sizeof(s_AesMem));
	return s_pEcdh != nullptr && s_pAes != nullptr &&
		BtSmpInit(s_pEcdh, s_pAes, &s_Rng);
}

void ResetLink()
{
	// BtSmpInit clears every slot and puts the invalid handle back, which is
	// what SmpLinkAlloc reads as free. Re-running it per case also drops any
	// crypto request left in flight by the previous one.
	CHECK(BtSmpInit(s_pEcdh, s_pAes, &s_Rng));

	std::memset(&s_Peer, 0, sizeof(s_Peer));
	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.bSecure = false;

	s_Role = BT_CONN_ROLE_PERIPHERAL;
	BtSmpAuthConfig(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	SmpTxReset();
}

// Drive the responder up to the point where it is waiting for the peer's
// public key: feed a Pairing Request as a central would send it, which makes
// this side answer Pairing Response and generate its own P-256 key.
void FeedPairingReq()
{
	BtSmpPairingReq_t req = {};
	req.Code = BT_SMP_CODE_PAIRING_REQ;
	req.IOCaps = BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT;
	req.OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
	req.AuthReq = BT_SMP_AUTHREQ_SC;
	req.MaxKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	req.InitiatorKeyDist = BT_SMP_KEYDIST_IDKEY;
	req.ResponderKeyDist = BT_SMP_KEYDIST_IDKEY;

	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)&req, sizeof(req));
}

// Feed a Pairing Public Key PDU. X and Y arrive big endian here and go out on
// the wire little endian, which is the order SmpHandlePublicKey undoes.
void FeedPublicKey(const uint8_t X[32], const uint8_t Y[32])
{
	BtSmpPublicKey_t pk = {};
	pk.Code = BT_SMP_CODE_PAIRING_PUBLIC_KEY;
	for (int i = 0; i < 32; i++)
	{
		pk.KeyX[i] = X[31 - i];
		pk.KeyY[i] = Y[31 - i];
	}
	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)&pk, sizeof(pk));
}

bool SentPairingFailed(uint8_t Reason)
{
	for (size_t i = 0; i < g_SmpTx.Count && i < SMP_TX_CAPTURE_MAX; i++)
	{
		const SmpTxPdu &p = g_SmpTx.Pdu[i];
		if (p.Len >= 2 && p.Data[0] == BT_SMP_CODE_PAIRING_FAILED &&
			p.Data[1] == Reason)
		{
			return true;
		}
	}
	return false;
}

bool SentAnyPairingFailed()
{
	for (size_t i = 0; i < g_SmpTx.Count && i < SMP_TX_CAPTURE_MAX; i++)
	{
		const SmpTxPdu &p = g_SmpTx.Pdu[i];
		if (p.Len >= 1 && p.Data[0] == BT_SMP_CODE_PAIRING_FAILED)
		{
			return true;
		}
	}
	return false;
}

// The local key the responder generated for this pairing, X then Y, big
// endian - the order Ctx.LocalPubKey holds.
BtSmpLink_t *ActiveLink()
{
	for (size_t i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		if (s_SmpLink[i].ConnHdl == kConnHdl)
		{
			return &s_SmpLink[i];
		}
	}
	return nullptr;
}

// Control. A different, valid peer key is accepted and the key agreement runs,
// so nothing here answers Pairing Failed. Without this the refusals below
// would pass just as well against a stack that refused every key.
void TestDistinctKeyIsAccepted()
{
	ResetLink();
	FeedPairingReq();

	BtSmpLink_t *pLink = ActiveLink();
	CHECK(pLink != nullptr);
	if (pLink == nullptr)
	{
		return;
	}
	CHECK(SmpKeyPresent(pLink->Ctx.LocalPubKey, 64));

	// A second key pair from the same engine, so the point is genuinely on the
	// curve rather than assumed to be.
	alignas(8) uint8_t peerCtx[CRYPTO_KEYCTX_MAX] = {};
	uint8_t peerPub[64] = {};
	CHECK(s_pEcdh->KeyGen(CRYPTO_CURVE_P256, peerCtx, peerPub) ==
		CRYPTO_STATUS_OK);
	CHECK(std::memcmp(peerPub, pLink->Ctx.LocalPubKey, 32) != 0);

	SmpTxReset();
	FeedPublicKey(peerPub, peerPub + 32);

	CHECK(SentAnyPairingFailed() == false);
	CHECK(SmpKeyPresent(pLink->Ctx.DhKey, sizeof(pLink->Ctx.DhKey)));
}

// The plain reflection: the peer echoes both coordinates back.
void TestReflectedKeyIsRefused()
{
	ResetLink();
	FeedPairingReq();

	BtSmpLink_t *pLink = ActiveLink();
	CHECK(pLink != nullptr);
	if (pLink == nullptr)
	{
		return;
	}

	uint8_t x[32], y[32];
	std::memcpy(x, pLink->Ctx.LocalPubKey, 32);
	std::memcpy(y, pLink->Ctx.LocalPubKey + 32, 32);

	SmpTxReset();
	FeedPublicKey(x, y);

	CHECK(SentPairingFailed(BT_SMP_ERR_DHKEY_CHECK_FAILED));
}

// The reflection that a 64 octet compare misses. (x, p-y) is a distinct,
// valid, on-curve point, so an on-curve check passes it and a whole-key
// compare does not match. But f4 consumes only X, so every confirm value in a
// Passkey Entry round becomes computable by the peer from public material,
// and X(d * -P) equals X(d * P) so the shared secret is the reflected one as
// well. Vol 3 Part H 2.2.6 for f4; the check has to be on X alone, which is
// what Zephyr's smp.c compares (BT_PUB_KEY_COORD_LEN).
void TestNegatedYReflectionIsRefused()
{
	ResetLink();
	FeedPairingReq();

	BtSmpLink_t *pLink = ActiveLink();
	CHECK(pLink != nullptr);
	if (pLink == nullptr)
	{
		return;
	}

	uint8_t x[32], y[32], negY[32];
	std::memcpy(x, pLink->Ctx.LocalPubKey, 32);
	std::memcpy(y, pLink->Ctx.LocalPubKey + 32, 32);
	FieldNegate(y, negY);

	// The premise: a different 64 octet key, same X.
	CHECK(std::memcmp(negY, y, 32) != 0);

	// And it really is on the curve, so nothing downstream would have caught
	// it. Proven with the engine's own validation rather than asserted.
	uint8_t negPub[64];
	std::memcpy(negPub, x, 32);
	std::memcpy(negPub + 32, negY, 32);
	alignas(8) uint8_t probeCtx[CRYPTO_KEYCTX_MAX] = {};
	uint8_t probePub[64] = {};
	uint8_t shared[32] = {};
	CHECK(s_pEcdh->KeyGen(CRYPTO_CURVE_P256, probeCtx, probePub) ==
		CRYPTO_STATUS_OK);
	CHECK(s_pEcdh->Agree(CRYPTO_CURVE_P256, probeCtx, negPub, shared, false) ==
		CRYPTO_STATUS_OK);

	SmpTxReset();
	FeedPublicKey(x, negY);

	CHECK(SentPairingFailed(BT_SMP_ERR_DHKEY_CHECK_FAILED));
	CHECK(SmpKeyPresent(pLink->Ctx.DhKey, sizeof(pLink->Ctx.DhKey)) == false);
}

// The same refusal on the other call site. SmpStartDhKey is reached from two
// places in SmpHandlePublicKey - the initiator branch, which starts the key
// agreement on the responder's key, and the responder branch through
// SmpTryStartDhKey. They share the function, so one fix covers both, but this
// repository has twice had a fix land on only one of two call sites. Drive the
// central role and check.
void TestNegatedYReflectionIsRefusedAsInitiator()
{
	ResetLink();
	s_Role = BT_CONN_ROLE_CENTRAL;
	s_Peer.pHciDev = &s_Dev;

	// Central: send Pairing Request and generate the local key.
	BtSmpStartPairing(kConnHdl);

	BtSmpLink_t *pLink = ActiveLink();
	CHECK(pLink != nullptr);
	if (pLink == nullptr)
	{
		return;
	}

	// The responder answers, which makes this side send its public key and
	// wait for the peer's.
	BtSmpPairingRsp_t rsp = {};
	rsp.Code = BT_SMP_CODE_PAIRING_RSP;
	rsp.IOCaps = BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT;
	rsp.OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
	rsp.AuthReq = BT_SMP_AUTHREQ_SC;
	rsp.MaxKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	rsp.InitiatorKeyDist = BT_SMP_KEYDIST_IDKEY;
	rsp.ResponderKeyDist = BT_SMP_KEYDIST_IDKEY;
	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)&rsp, sizeof(rsp));

	CHECK(pLink->Ctx.bInitiator);
	CHECK(SmpKeyPresent(pLink->Ctx.LocalPubKey, 64));

	uint8_t x[32], y[32], negY[32];
	std::memcpy(x, pLink->Ctx.LocalPubKey, 32);
	std::memcpy(y, pLink->Ctx.LocalPubKey + 32, 32);
	FieldNegate(y, negY);

	SmpTxReset();
	FeedPublicKey(x, negY);

	CHECK(SentPairingFailed(BT_SMP_ERR_DHKEY_CHECK_FAILED));
}

} // namespace

extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return Hdl == kConnHdl ? &s_Peer : nullptr;
}

uint8_t BtPeerRole(uint16_t)
{
	return s_Role;
}

void BtGapConnSecSet(uint16_t, const BtConnSec_t *)
{
}

void BtGattCccdRestoreBonded(uint16_t)
{
}

bool BtSmpBondKeysLookup(uint16_t, uint64_t, uint16_t, BtSmpKeys_t *)
{
	return false;
}

void BtSmpBondLoad(void)
{
}

bool BtSmpBonded(uint16_t)
{
	return false;
}

} // extern "C"

int main()
{
	if (!CryptoBringUp())
	{
		std::printf("SMP public key tests: FAIL, crypto engines did not "
					"come up\n");
		return 1;
	}

	TestDistinctKeyIsAccepted();
	TestReflectedKeyIsRefused();
	TestNegatedYReflectionIsRefused();
	TestNegatedYReflectionIsRefusedAsInitiator();

	if (s_Failures != 0)
	{
		std::printf("SMP public key tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SMP public key tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
