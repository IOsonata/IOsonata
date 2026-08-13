// Pairing policy the local device sets, checked against what the negotiation
// actually produced.
//
// Feature negotiation picks an association model from the two IO capability
// sets. The model decides whether the resulting key carries the Authenticated
// MITM protection security property, so a device that asked for MITM has to
// look at the model it got, not just at the flag it sent.
//
// Same harness as bt_smp_keydist_test: the pairing state is file-static, so
// the source is included to reach it.

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

constexpr uint16_t kConnHdl = 0x0044;

BtHciDevice_t s_Dev = {};
BtDevice_t s_Peer = {};

void ResetLink(uint8_t LocalIoCaps, uint8_t LocalAuthReq)
{
	std::memset(s_SmpLink, 0, sizeof(s_SmpLink));
	// SmpLinkAlloc takes a slot only when it reads as free, which is the
	// invalid handle and not zero.
	for (size_t i = 0; i < BT_SMP_MAX_LINK; i++)
	{
		s_SmpLink[i].ConnHdl = BT_CONN_HDL_INVALID;
	}
	std::memset(&s_Peer, 0, sizeof(s_Peer));

	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.bSecure = false;

	BtSmpAuthConfig(LocalIoCaps, LocalAuthReq);
	SmpTxReset();
}

// A Pairing Request as a central would send it: Secure Connections, full key
// size, no OOB, distributing the identity key.
void FeedPairingReq(uint8_t PeerIoCaps, uint8_t PeerAuthReq)
{
	BtSmpPairingReq_t req = {};
	req.Code = BT_SMP_CODE_PAIRING_REQ;
	req.IOCaps = PeerIoCaps;
	req.OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
	req.AuthReq = (uint8_t)(PeerAuthReq | BT_SMP_AUTHREQ_SC);
	req.MaxKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	req.InitiatorKeyDist = BT_SMP_KEYDIST_IDKEY;
	req.ResponderKeyDist = BT_SMP_KEYDIST_IDKEY;

	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)&req, sizeof(req));
}

bool FirstIsPairingFailed(uint8_t Reason)
{
	return g_SmpTx.Count >= 1 &&
		   g_SmpTx.Pdu[0].Len >= sizeof(BtSmpPairingFailed_t) &&
		   g_SmpTx.Pdu[0].Data[0] == BT_SMP_CODE_PAIRING_FAILED &&
		   g_SmpTx.Pdu[0].Data[1] == Reason;
}

bool FirstIsPairingRsp(void)
{
	return g_SmpTx.Count >= 1 &&
		   g_SmpTx.Pdu[0].Data[0] == BT_SMP_CODE_PAIRING_RSP;
}

// Core Vol 3 Part H 2.3.5.1, after Table 2.8: "If the key generation method
// does not result in a key that provides sufficient Security Properties (see
// Section 2.3.1) then the device shall send the Pairing Failed command with
// the error code Authentication Requirements."
//
// A peer with no input and no output maps to Just Works whatever this device
// can do, and Just Works gives Unauthenticated no MITM protection. Asking for
// MITM and accepting that pairing leaves the application believing it holds an
// authenticated bond.
void TestMitmRequiredRefusesJustWorks()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingFailed(BT_SMP_ERR_AUTHEN_REQUIREMENTS));
	CHECK(FirstIsPairingRsp() == false);
	CHECK(s_SmpLink[0].Ctx.State == BT_SMP_STATE_IDLE);
}

// The local requirement is what this device enforces. A peer that never asked
// for MITM does not change that: the model still comes out Just Works and the
// local key would still be Unauthenticated.
void TestMitmRequiredRefusesWhenOnlyLocalAsks()
{
	ResetLink(BT_SMP_IOCAPS_KEYBOARD_DISPLAY, BT_SMP_AUTHREQ_MITM);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);

	CHECK(FirstIsPairingFailed(BT_SMP_ERR_AUTHEN_REQUIREMENTS));
	CHECK(FirstIsPairingRsp() == false);
}

// Which model each IO capability pair produces, so the cases below say what
// they mean. Ctx.Model cannot be read after the fact: the local key generation
// that follows the Pairing Response has no crypto engine in this harness, and
// the abort path wipes Ctx.
void TestModelSelectionUnderMitm()
{
	CHECK(SmpSelectModel(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
						 BT_SMP_IOCAPS_DISPLAY_YESNO, true, false) ==
		  BT_SMP_MODEL_JUST_WORKS);
	CHECK(SmpSelectModel(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
						 BT_SMP_IOCAPS_KEYBOARD_DISPLAY, true, false) ==
		  BT_SMP_MODEL_JUST_WORKS);
	CHECK(SmpSelectModel(BT_SMP_IOCAPS_DISPLAY_YESNO,
						 BT_SMP_IOCAPS_DISPLAY_YESNO, true, false) ==
		  BT_SMP_MODEL_NUMERIC_COMPARISON);
	CHECK(SmpSelectModel(BT_SMP_IOCAPS_DISPLAY_ONLY,
						 BT_SMP_IOCAPS_KEYBOARD_ONLY, true, false) ==
		  BT_SMP_MODEL_PASSKEY_ENTRY);
}

// An IO capability pair that does reach an authenticated model is answered
// normally. The Pairing Response is the first PDU out; what follows depends on
// the crypto engine, which this harness does not provide.
void TestMitmRequiredAcceptsNumericComparison()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);
	FeedPairingReq(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingRsp());
}

void TestMitmRequiredAcceptsPasskeyEntry()
{
	ResetLink(BT_SMP_IOCAPS_KEYBOARD_ONLY, BT_SMP_AUTHREQ_MITM);
	FeedPairingReq(BT_SMP_IOCAPS_DISPLAY_ONLY, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingRsp());
}

// A device that did not ask for MITM is entitled to Just Works, so the gate
// must not close on an open configuration.
void TestNoMitmStillPairsJustWorks()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);

	CHECK(FirstIsPairingRsp());
}

// The peer asking for MITM does not make it this device's requirement. The
// peer enforces its own policy on its side, and refusing here would break a
// legitimate pairing this device is content with.
void TestPeerMitmAloneDoesNotRefuse()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);
	FeedPairingReq(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingRsp());
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
	TestMitmRequiredRefusesJustWorks();
	TestMitmRequiredRefusesWhenOnlyLocalAsks();
	TestModelSelectionUnderMitm();
	TestMitmRequiredAcceptsNumericComparison();
	TestMitmRequiredAcceptsPasskeyEntry();
	TestNoMitmStillPairsJustWorks();
	TestPeerMitmAloneDoesNotRefuse();

	if (s_Failures != 0)
	{
		std::printf("SMP pairing policy tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SMP pairing policy tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
