/**-------------------------------------------------------------------------
@file	bt_smp_keydist_test.cpp

@brief	Phase 3 key distribution follows the negotiated set, host tests

The key distribution fields are negotiated across two PDUs: the initiator
names a set in the Pairing Request and the responder answers in the Pairing
Response with the subset it accepts. What each side then distributes is the
two octets of its field intersected. Reading the local set from the Pairing
Request alone made the responder's answer advisory.

bt_smp.cpp is included rather than linked so a case can put a link straight
into the state that follows a completed pairing. Its transport and platform
hooks are weak definitions, replaced from bt_smp_link_test_stubs.cpp, which is
a separate object for that reason.

@author	Hoang Nguyen Hoan
@date	Aug. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include <cstring>

#include "crypto/crypto_softrng.h"

#include "bt_test_harness.h"
#include "bt_smp_link_test_stubs.h"

#include "../../../src/bluetooth/bt_smp.cpp"

namespace {

// The CSRK is drawn through BtSmpCryptoRand, which refuses an engine that
// reports IsSecure() false so a deterministic stream can never reach a key.
// These cases are about which PDUs go out, not about the values in them, so
// the software PRNG is presented as secure to let the distribution path run.
// Nothing outside this test does that.
class TestRng : public CryptoSoftRng {
public:
	bool IsSecure() const override { return true; }
};

TestRng s_Rng;

const uint16_t kConnHdl = 0x0040;

// Pairing Request and Pairing Response are the same seven octet layout:
// Code, IO Capability, OOB flag, AuthReq, Max Encryption Key Size, Initiator
// Key Distribution, Responder Key Distribution.
const int kInitKeyDist = 5;
const int kRespKeyDist = 6;

BtHciDevice_t s_HciDev;

// Put a link into the state BtSmpEncryptionChanged sees after a Secure
// Connections pairing has produced an LTK and the controller has reported the
// link encrypted, with the two pairing PDUs the exchange left behind.
void ArmLink(bool bInitiator, uint8_t ReqInit, uint8_t ReqResp,
			 uint8_t RspInit, uint8_t RspResp)
{
	BtSmpTestCaptureReset();

	// A zeroed slot reads as connection handle 0, which SmpLinkFind would match.
	// Only slot 0 belongs to this case; the rest are marked free the way
	// SmpLinkFree leaves them, so a later case that looks up another handle does
	// not find a slot this one left behind.
	memset(s_SmpLink, 0, sizeof(s_SmpLink));
	for (int i = 1; i < BT_SMP_MAX_LINK; i++)
	{
		s_SmpLink[i].ConnHdl = BT_CONN_HDL_INVALID;
	}

	BtSmpLink_t *pLink = &s_SmpLink[0];

	pLink->ConnHdl = kConnHdl;
	pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
	pLink->Ctx.bInitiator = bInitiator;
	pLink->Ctx.bSc = true;
	pLink->Ctx.EncKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	pLink->Ctx.Model = BT_SMP_MODEL_JUST_WORKS;

	pLink->Ctx.PReq[0] = BT_SMP_CODE_PAIRING_REQ;
	pLink->Ctx.PReq[1] = BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT;
	pLink->Ctx.PReq[2] = BT_SMP_OOB_AUTH_NOT_PRESENT;
	pLink->Ctx.PReq[3] = BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC;
	pLink->Ctx.PReq[4] = BT_SMP_MAX_ENC_KEY_SIZE;
	pLink->Ctx.PReq[kInitKeyDist] = ReqInit;
	pLink->Ctx.PReq[kRespKeyDist] = ReqResp;

	memcpy(pLink->Ctx.PRsp, pLink->Ctx.PReq, sizeof(pLink->Ctx.PRsp));
	pLink->Ctx.PRsp[0] = BT_SMP_CODE_PAIRING_RSP;
	pLink->Ctx.PRsp[kInitKeyDist] = RspInit;
	pLink->Ctx.PRsp[kRespKeyDist] = RspResp;

	// The identity IRK is drawn from the secure RBG on first use and this test
	// binds no crypto engines. Seeding it is what a device has after first
	// boot, and without it the distribution path fails the pairing before it
	// sends anything.
	s_SmpLocalId.bValid = true;
	memset(s_SmpLocalId.Irk, 0x5A, sizeof(s_SmpLocalId.Irk));

	s_Rng.Enable();
	s_pCryptoRng = &s_Rng;
	s_pSmpActiveDev = &s_HciDev;
}

int IdInfoCount(void)
{
	return BtSmpTestCaptureCount(BT_SMP_CODE_PAIRING_ID_INFO);
}

int IdAddrCount(void)
{
	return BtSmpTestCaptureCount(BT_SMP_CODE_PAIRING_ID_ADDR_INFO);
}

int SigningCount(void)
{
	return BtSmpTestCaptureCount(BT_SMP_CODE_PAIRING_SIGNING_INFO);
}

// Control. Both sides asked for both keys and neither trimmed, so the
// initiator distributes both and expects both back.
void TestInitiatorSendsTheWholeNegotiatedSet(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, both, both);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, IdInfoCount() == 1);
	BT_CHECK(ctx, IdAddrCount() == 1);
	BT_CHECK(ctx, SigningCount() == 1);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.KeyDistExp == both);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_KEYDIST);
	// The LTK bond is stored once, before the peer keys arrive, so a controller
	// LTK request on an immediate reconnect is answered.
	BT_CHECK(ctx, BtSmpTestBondAddCount() == 1);
}

// A bond store that refuses, the table being full being the ordinary reason,
// used to be swallowed: the call returned nothing and the pairing completed as
// if the record were kept. The peer then holds a bond this device does not.
// The link is encrypted and usable, so pairing still completes; what changes
// is that the application is told.
void TestBondStoreRefusalIsReported(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, both, both);
	BtSmpTestBondAddResultSet(false);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, BtSmpTestBondAddCount() == 1);
	BT_CHECK(ctx, BtSmpTestBondStoreFailedCount() == 1);

	// The key distribution still runs and the link still reaches the phase it
	// should. Losing the bond does not lose the session.
	BT_CHECK(ctx, IdInfoCount() == 1);
	BT_CHECK(ctx, SigningCount() == 1);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_KEYDIST);
}

// The control: a store that accepts reports nothing.
void TestBondStoreSuccessIsSilent(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, both, both);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, BtSmpTestBondAddCount() == 1);
	BT_CHECK(ctx, BtSmpTestBondStoreFailedCount() == 0);
}

// A responder that clears SignKey from the Initiator Key Distribution field of
// its Pairing Response has refused the CSRK. Taking the local set from the
// request alone sent Signing Information anyway. Phones do clear it.
void TestInitiatorHonoursATrimmedSignKey(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, BT_SMP_KEYDIST_IDKEY, both);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, IdInfoCount() == 1);
	BT_CHECK(ctx, IdAddrCount() == 1);
	BT_CHECK(ctx, SigningCount() == 0);
	// The Responder field is untouched, so what the peer owes is unchanged.
	BT_CHECK(ctx, s_SmpLink[0].Ctx.KeyDistExp == both);
}

// The same with IdKey cleared: no IRK and no identity address go out.
void TestInitiatorHonoursATrimmedIdKey(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, BT_SMP_KEYDIST_SIGNKEY, both);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, IdInfoCount() == 0);
	BT_CHECK(ctx, IdAddrCount() == 0);
	BT_CHECK(ctx, SigningCount() == 1);
}

// A responder cleared both, so the initiator distributes nothing. It still
// expects the Responder field back, so the link waits in KEYDIST.
void TestInitiatorDistributesNothingWhenBothCleared(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, 0, both);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, IdInfoCount() == 0);
	BT_CHECK(ctx, IdAddrCount() == 0);
	BT_CHECK(ctx, SigningCount() == 0);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.KeyDistExp == both);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_KEYDIST);
}

// Nothing owed either way ends the pairing without entering KEYDIST.
void TestNoKeysAtAllCompletesImmediately(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, 0, 0);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, g_BtSmpTestCapture.Count == 0);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.KeyDistExp == 0);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_DONE);
}

// The responder distributes the Responder field and expects the Initiator
// field. It builds its own response, so this side already agreed with itself;
// the case pins that the shared expression did not disturb it.
void TestResponderUsesTheResponderField(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(false, both, both, BT_SMP_KEYDIST_IDKEY, BT_SMP_KEYDIST_SIGNKEY);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, IdInfoCount() == 0);
	BT_CHECK(ctx, IdAddrCount() == 0);
	BT_CHECK(ctx, SigningCount() == 1);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.KeyDistExp == BT_SMP_KEYDIST_IDKEY);
}

// A peer answering with a bit the request never offered has not negotiated it.
// Intersecting the two octets keeps that bit out whichever field it lands in.
void TestAResponseCannotAddAKeyTheRequestDidNotOffer(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, BT_SMP_KEYDIST_IDKEY, BT_SMP_KEYDIST_IDKEY, both, both);
	BtSmpEncryptionChanged(&s_HciDev, kConnHdl, 0, 1);

	BT_CHECK(ctx, IdInfoCount() == 1);
	BT_CHECK(ctx, IdAddrCount() == 1);
	BT_CHECK(ctx, SigningCount() == 0);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.KeyDistExp == BT_SMP_KEYDIST_IDKEY);
}

// Sending Pairing Failed ends the pairing for the peer. Keeping this side's
// context alive left the DHKey, the MacKey and the derived LTK in the link
// with no exchange left to use them, reclaimed only by a pairing timeout that
// cannot fire at all on a port leaving BtSmpMsTick at its weak zero.
void TestShortPduDropsTheAttempt(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, both, both);
	s_SmpLink[0].Ctx.State = BT_SMP_STATE_DHKEY_CHECK_WAIT;
	memset(s_SmpLink[0].Ctx.DhKey, 0xD1, sizeof(s_SmpLink[0].Ctx.DhKey));
	memset(s_SmpLink[0].Ctx.Mackey, 0xA4, sizeof(s_SmpLink[0].Ctx.Mackey));
	memset(s_SmpLink[0].Ctx.Ltk, 0x77, sizeof(s_SmpLink[0].Ctx.Ltk));

	// One octet, which is shorter than every code's minimum.
	BtL2CapSmp_t smp = {};
	smp.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	BtProcessSmpData(&s_HciDev, kConnHdl, &smp, 1);

	BT_CHECK(ctx, BtSmpTestCaptureCount(BT_SMP_CODE_PAIRING_FAILED) == 1);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_IDLE);
	BT_CHECK(ctx, SmpIsAllZero(s_SmpLink[0].Ctx.DhKey,
							   sizeof(s_SmpLink[0].Ctx.DhKey)));
	BT_CHECK(ctx, SmpIsAllZero(s_SmpLink[0].Ctx.Mackey,
							   sizeof(s_SmpLink[0].Ctx.Mackey)));
	BT_CHECK(ctx, SmpIsAllZero(s_SmpLink[0].Ctx.Ltk,
							   sizeof(s_SmpLink[0].Ctx.Ltk)));
}

// A link that already finished pairing keeps its record. A stray short PDU
// must not turn into a lost bond: BtSmpProcessLtkRequest answers a reconnect
// from exactly this state.
void TestShortPduLeavesACompletedPairingAlone(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(true, both, both, both, both);
	s_SmpLink[0].Ctx.State = BT_SMP_STATE_DONE;
	s_SmpLink[0].Keys.bValid = true;
	memset(s_SmpLink[0].Keys.Ltk, 0x5C, sizeof(s_SmpLink[0].Keys.Ltk));

	BtL2CapSmp_t smp = {};
	smp.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	BtProcessSmpData(&s_HciDev, kConnHdl, &smp, 1);

	BT_CHECK(ctx, BtSmpTestCaptureCount(BT_SMP_CODE_PAIRING_FAILED) == 1);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_DONE);
	BT_CHECK(ctx, s_SmpLink[0].Keys.bValid);
	BT_CHECK(ctx, s_SmpLink[0].Keys.Ltk[0] == 0x5C);
}

// Put the pending OOB set under this link's reservation, the way SmpOobCtxLoad
// leaves it once the feature exchange has selected the OOB model.
void ArmOobReservation(void)
{
	memset(&s_SmpOob, 0, sizeof(s_SmpOob));
	s_SmpOob.bLocalValid = true;
	s_SmpOob.bPeerValid = true;
	s_SmpOob.bReserved = true;
	s_SmpOob.ConnHdl = s_SmpLink[0].ConnHdl;
	s_SmpOob.Generation = s_SmpLink[0].Generation;
	memset(s_SmpOob.LocalRand, 0x11, sizeof(s_SmpOob.LocalRand));
	memset(s_SmpOob.PeerRand, 0x22, sizeof(s_SmpOob.PeerRand));
	memset(s_SmpOob.PeerConfirm, 0x33, sizeof(s_SmpOob.PeerConfirm));
}

// Sending Pairing Failed used to wipe the link's OOB material as a side
// effect, so any PDU this stack answers without ending the pairing took the
// randoms and confirms with it. The pairing then ran on and failed later at
// the DHKey check. A peer could do it with one unrecognised code.
void TestAnswerablePduKeepsOobMaterial(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(false, both, both, both, both);
	s_SmpLink[0].Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
	s_SmpLink[0].Ctx.Model = BT_SMP_MODEL_OOB;
	ArmOobReservation();

	// A Pairing Request arriving mid-pairing. The phase gate answers it and
	// deliberately leaves the running attempt alone, which is what makes it the
	// right shape for this case. A reserved code would no longer do: those are
	// ignored outright now, so nothing is sent and nothing is released.
	uint8_t pdu[7] = { BT_SMP_CODE_PAIRING_REQ, BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
					   BT_SMP_OOB_AUTH_NOT_PRESENT,
					   BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC,
					   BT_SMP_MAX_ENC_KEY_SIZE, both, both };
	BtProcessSmpData(&s_HciDev, kConnHdl, (BtL2CapSmp_t *)pdu, sizeof(pdu));

	BT_CHECK(ctx, BtSmpTestCaptureCount(BT_SMP_CODE_PAIRING_FAILED) == 1);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_RANDOM_WAIT);
	BT_CHECK(ctx, s_SmpOob.bReserved);
	BT_CHECK(ctx, s_SmpOob.bLocalValid);
	BT_CHECK(ctx, s_SmpOob.bPeerValid);
	BT_CHECK(ctx, s_SmpOob.PeerConfirm[0] == 0x33);
}

// The other half of the same rule: a path that does end the pairing still
// releases, because releasing lives in SmpAbortPairing rather than in the
// transmission.
void TestEndingTheAttemptStillReleasesOob(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(false, both, both, both, both);
	s_SmpLink[0].Ctx.State = BT_SMP_STATE_RANDOM_WAIT;
	s_SmpLink[0].Ctx.Model = BT_SMP_MODEL_OOB;
	ArmOobReservation();

	// A one octet PDU, which drops the attempt.
	BtL2CapSmp_t smp = {};
	smp.Code = BT_SMP_CODE_PAIRING_CONFIRM;
	BtProcessSmpData(&s_HciDev, kConnHdl, &smp, 1);

	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_IDLE);
	BT_CHECK(ctx, s_SmpOob.bReserved == false);
	BT_CHECK(ctx, s_SmpOob.bLocalValid == false);
	BT_CHECK(ctx, SmpIsAllZero(s_SmpOob.PeerConfirm,
							   sizeof(s_SmpOob.PeerConfirm)));
}

// Vol 3 Part H 3.3: "If a packet is received with a Code that is reserved for
// future use it shall be ignored." Table 3.3 defines 0x01 to 0x0E and reserves
// the rest. Answering Command Not Supported put a PDU on air the spec forbids.
void TestReservedCodeIsIgnored(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;
	static const uint8_t reserved[] = { 0x00, 0x0F, 0x10, 0x7F, 0xFF };

	for (size_t i = 0; i < sizeof(reserved) / sizeof(reserved[0]); i++)
	{
		ArmLink(false, both, both, both, both);
		s_SmpLink[0].Ctx.State = BT_SMP_STATE_RANDOM_WAIT;

		BtL2CapSmp_t smp = {};
		smp.Code = reserved[i];
		BtProcessSmpData(&s_HciDev, kConnHdl, &smp, sizeof(smp));

		// Nothing on air at all, and the pairing is left where it was.
		BT_CHECK(ctx, g_BtSmpTestCapture.Count == 0);
		BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_RANDOM_WAIT);
	}
}

// Vol 3 Part H 3.5.8 defines Keypress Notification and 3.3 requires every
// command to be supported once pairing is. It used to reach the default and be
// answered with Pairing Failed, ending a Passkey Entry pairing with a peer
// doing nothing wrong. 3.4 asks for the Security Manager Timer to be reset on
// reception, because the PDU draws no reply.
void TestKeypressNotificationIsAcceptedAndResetsTheTimer(bttest::Context &ctx)
{
	const uint8_t both = BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY;

	ArmLink(false, both, both, both, both);
	s_SmpLink[0].Ctx.State = BT_SMP_STATE_CONFIRM_WAIT;
	s_SmpLink[0].Ctx.Model = BT_SMP_MODEL_PASSKEY_ENTRY;
	// BtSmpMsTick is the weak default and answers zero, so a start stamp is
	// only recent if the unsigned difference from zero is small. 0x1234 would
	// wrap to about four billion and read as an expired pairing, which is the
	// same trap the timeout itself has on a port with no clock.
	s_SmpLink[0].Ctx.TmrStart = 0xFFFFFFF0;

	uint8_t pdu[2] = { BT_SMP_CODE_PAIRING_KEYPRESS_NOTIF, 1 };
	BtProcessSmpData(&s_HciDev, kConnHdl, (BtL2CapSmp_t *)pdu, sizeof(pdu));

	BT_CHECK(ctx, g_BtSmpTestCapture.Count == 0);
	BT_CHECK(ctx, s_SmpLink[0].Ctx.State == BT_SMP_STATE_CONFIRM_WAIT);
	// The reset is visible as the seeded stamp being replaced by the tick.
	BT_CHECK(ctx, s_SmpLink[0].Ctx.TmrStart == 0);
}

} // namespace

int main()
{
	bttest::Context ctx("SMP key distribution tests");

	ctx.Run("initiator sends the whole negotiated set",
			[&] { TestInitiatorSendsTheWholeNegotiatedSet(ctx); });
	ctx.Run("initiator honours a trimmed SignKey",
			[&] { TestInitiatorHonoursATrimmedSignKey(ctx); });
	ctx.Run("initiator honours a trimmed IdKey",
			[&] { TestInitiatorHonoursATrimmedIdKey(ctx); });
	ctx.Run("initiator distributes nothing when both cleared",
			[&] { TestInitiatorDistributesNothingWhenBothCleared(ctx); });
	ctx.Run("no keys at all completes immediately",
			[&] { TestNoKeysAtAllCompletesImmediately(ctx); });
	ctx.Run("responder uses the responder field",
			[&] { TestResponderUsesTheResponderField(ctx); });
	ctx.Run("a response cannot add a key the request did not offer",
			[&] { TestAResponseCannotAddAKeyTheRequestDidNotOffer(ctx); });
	ctx.Run("bond store refusal is reported",
			[&] { TestBondStoreRefusalIsReported(ctx); });
	ctx.Run("bond store success is silent",
			[&] { TestBondStoreSuccessIsSilent(ctx); });
	ctx.Run("short pdu drops the attempt",
			[&] { TestShortPduDropsTheAttempt(ctx); });
	ctx.Run("short pdu leaves a completed pairing alone",
			[&] { TestShortPduLeavesACompletedPairingAlone(ctx); });
	ctx.Run("an answerable pdu keeps oob material",
			[&] { TestAnswerablePduKeepsOobMaterial(ctx); });
	ctx.Run("ending the attempt still releases oob",
			[&] { TestEndingTheAttemptStillReleasesOob(ctx); });
	ctx.Run("a reserved code is ignored",
			[&] { TestReservedCodeIsIgnored(ctx); });
	ctx.Run("keypress notification is accepted and resets the timer",
			[&] { TestKeypressNotificationIsAcceptedAndResetsTheTimer(ctx); });

	return ctx.Finish();
}
