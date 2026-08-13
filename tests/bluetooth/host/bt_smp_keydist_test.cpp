// Phase 3 key distribution gate.
//
// Core Vol 3 Part H 3.6.1: the keys to be distributed are the ones named in
// the Key Distribution fields of the Pairing Request and Pairing Response, and
// in LE Secure Connections on the LE transport the EncKey field shall be
// ignored, with EDIV and Rand set to zero and not distributed. A key the peer
// never negotiated is therefore a PDU the server must not act on.
//
// The pairing state machine and its key store are file-static, so this test
// includes the source. Driving a whole Secure Connections pairing to reach the
// key-distribution phase would need the f4/f6 helpers, which are static too;
// including the source is what lets the gate be exercised directly, one PDU at
// a time, which is the level the rule lives at.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

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

constexpr uint16_t kConnHdl = 0x0031;

BtHciDevice_t s_Dev = {};
BtDevice_t s_Peer = {};

// Put a link in the state a completed Secure Connections phase 2 leaves it in:
// encrypted, key material valid, waiting for the peer keys named by KeyDist.
BtSmpLink_t *ArmKeyDist(uint8_t KeyDist)
{
	std::memset(s_SmpLink, 0, sizeof(s_SmpLink));
	std::memset(&s_Peer, 0, sizeof(s_Peer));

	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.bSecure = true;

	BtSmpLink_t *pLink = &s_SmpLink[0];
	pLink->ConnHdl = kConnHdl;
	pLink->Ctx.State = BT_SMP_STATE_KEYDIST;
	pLink->Ctx.bSc = true;
	pLink->Ctx.bInitiator = false;
	pLink->Ctx.KeyDistExp = KeyDist;
	pLink->Ctx.TmrStart = 0;
	pLink->Keys.bValid = true;
	pLink->Keys.bSc = true;
	std::memset(pLink->Keys.Ltk, 0x11, sizeof(pLink->Keys.Ltk));

	SmpTxReset();
	return pLink;
}

void Feed(const void *pPdu, size_t Len)
{
	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)pPdu, Len);
}

bool SentPairingFailed(void)
{
	return g_SmpTx.Count == 1 &&
		   g_SmpTx.Pdu[0].Len >= sizeof(BtSmpPairingFailed_t) &&
		   g_SmpTx.Pdu[0].Data[0] == BT_SMP_CODE_PAIRING_FAILED;
}

// An LTK offered when EncKey was not negotiated must not reach the key record.
// Without the gate the Encryption Information handler writes it straight over
// the LTK that phase 2 derived, and the next Identity Address Information
// persists the replacement to the bond store.
void TestEncryptInfoRejectedWhenNotNegotiated()
{
	BtSmpLink_t *pLink = ArmKeyDist(BT_SMP_KEYDIST_IDKEY);

	BtSmpEncryptInfo_t ei = {};
	ei.Code = BT_SMP_CODE_PAIRING_ENCRYP_INFO;
	std::memset(ei.Ltk, 0xAA, sizeof(ei.Ltk));
	Feed(&ei, sizeof(ei));

	CHECK(SentPairingFailed());
	for (size_t i = 0; i < sizeof(pLink->Keys.Ltk); i++)
	{
		CHECK(pLink->Keys.Ltk[i] == 0x11);
	}
	// The link stays in key distribution, still owing the IRK it negotiated.
	CHECK(pLink->Ctx.State == BT_SMP_STATE_KEYDIST);
	CHECK(pLink->Ctx.KeyDistExp == BT_SMP_KEYDIST_IDKEY);
}

// Central Identification carries EDIV and Rand, which LE Secure Connections
// does not distribute at all.
void TestCentralIdRejectedWhenNotNegotiated()
{
	BtSmpLink_t *pLink = ArmKeyDist(BT_SMP_KEYDIST_IDKEY);

	BtSmpCentralId_t ci = {};
	ci.Code = BT_SMP_CODE_PAIRING_CENTRAL_ID;
	ci.Ediv = 0x1234;
	ci.Rand = 0x0123456789ABCDEFULL;
	Feed(&ci, sizeof(ci));

	CHECK(SentPairingFailed());
	CHECK(pLink->Keys.Ediv == 0);
	CHECK(pLink->Keys.Rand == 0);
	CHECK(pLink->Ctx.State == BT_SMP_STATE_KEYDIST);
}

// A CSRK grants signed writes. Installing one the peer never negotiated hands
// it a capability that was not agreed.
void TestSigningInfoRejectedWhenNotNegotiated()
{
	BtSmpLink_t *pLink = ArmKeyDist(BT_SMP_KEYDIST_IDKEY);

	BtSmpSigningInfo_t si = {};
	si.Code = BT_SMP_CODE_PAIRING_SIGNING_INFO;
	std::memset(si.Csrk, 0xBB, sizeof(si.Csrk));
	Feed(&si, sizeof(si));

	CHECK(SentPairingFailed());
	for (size_t i = 0; i < sizeof(pLink->Keys.Csrk); i++)
	{
		CHECK(pLink->Keys.Csrk[i] == 0);
	}
	CHECK(pLink->Ctx.KeyDistExp == BT_SMP_KEYDIST_IDKEY);
}

// The negotiated set still goes through, and completes the phase.
void TestNegotiatedKeysAccepted()
{
	BtSmpLink_t *pLink = ArmKeyDist(BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY);

	BtSmpIdInfo_t ii = {};
	ii.Code = BT_SMP_CODE_PAIRING_ID_INFO;
	std::memset(ii.Irk, 0xC1, sizeof(ii.Irk));
	Feed(&ii, sizeof(ii));
	CHECK(g_SmpTx.Count == 0);
	CHECK(pLink->Keys.Irk[0] == 0xC1);

	BtSmpIdAddrInfo_t ia = {};
	ia.Code = BT_SMP_CODE_PAIRING_ID_ADDR_INFO;
	ia.AddrType = 0;
	std::memset(ia.Addr, 0xD2, sizeof(ia.Addr));
	Feed(&ia, sizeof(ia));
	CHECK(g_SmpTx.Count == 0);
	CHECK(pLink->Keys.IdAddr[0] == 0xD2);
	CHECK(pLink->Ctx.KeyDistExp == BT_SMP_KEYDIST_SIGNKEY);
	CHECK(pLink->Ctx.State == BT_SMP_STATE_KEYDIST);

	BtSmpSigningInfo_t si = {};
	si.Code = BT_SMP_CODE_PAIRING_SIGNING_INFO;
	std::memset(si.Csrk, 0xE3, sizeof(si.Csrk));
	Feed(&si, sizeof(si));
	CHECK(g_SmpTx.Count == 0);
	CHECK(pLink->Keys.Csrk[0] == 0xE3);
	CHECK(pLink->Ctx.KeyDistExp == 0);
	CHECK(pLink->Ctx.State == BT_SMP_STATE_DONE);
}

// Each negotiated key is distributed once. A second copy arrives after its bit
// has been consumed, so the same gate refuses it and a late substitution
// cannot overwrite the key already accepted.
void TestRepeatedKeyRejected()
{
	BtSmpLink_t *pLink = ArmKeyDist(BT_SMP_KEYDIST_IDKEY);

	BtSmpIdInfo_t ii = {};
	ii.Code = BT_SMP_CODE_PAIRING_ID_INFO;
	std::memset(ii.Irk, 0xC1, sizeof(ii.Irk));
	Feed(&ii, sizeof(ii));

	BtSmpIdAddrInfo_t ia = {};
	ia.Code = BT_SMP_CODE_PAIRING_ID_ADDR_INFO;
	std::memset(ia.Addr, 0xD2, sizeof(ia.Addr));
	Feed(&ia, sizeof(ia));
	CHECK(g_SmpTx.Count == 0);
	CHECK(pLink->Ctx.State == BT_SMP_STATE_DONE);

	SmpTxReset();
	std::memset(ii.Irk, 0x77, sizeof(ii.Irk));
	Feed(&ii, sizeof(ii));
	CHECK(SentPairingFailed());
	CHECK(pLink->Keys.Irk[0] == 0xC1);
}

} // namespace

//-----------------------------------------------------------------------------
// Externals bt_smp.cpp needs that the tested behaviour does not exercise.
//-----------------------------------------------------------------------------

extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return Hdl == kConnHdl ? &s_Peer : nullptr;
}

uint8_t BtPeerRole(uint16_t)
{
	return BT_CONN_ROLE_PERIPHERAL;
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

void CryptoSecureWipe(void *pData, size_t Len)
{
	std::memset(pData, 0, Len);
}

} // extern "C"

int main()
{
	TestEncryptInfoRejectedWhenNotNegotiated();
	TestCentralIdRejectedWhenNotNegotiated();
	TestSigningInfoRejectedWhenNotNegotiated();
	TestNegotiatedKeysAccepted();
	TestRepeatedKeyRejected();

	if (s_Failures != 0)
	{
		std::printf("SMP key distribution tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SMP key distribution tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
