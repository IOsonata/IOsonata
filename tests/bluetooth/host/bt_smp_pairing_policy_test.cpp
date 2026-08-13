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

// The bond this device already holds for the peer, as BtSmpBondKeysLookup
// would return it. bValid false means no stored bond.
BtSmpKeys_t s_StoredBond = {};

uint8_t s_Role = BT_CONN_ROLE_PERIPHERAL;

// HCI commands SMP issued. BtHciCommand dispatches through pDev->Command, so
// capturing it needs no extra link dependency.
uint16_t s_HciOpCode = 0;
int s_HciCount = 0;

uint8_t TestHciCommand(BtHciDevice_t * const, uint16_t OpCode, const void *,
					   uint8_t, void *, uint8_t)
{
	s_HciOpCode = OpCode;
	s_HciCount++;
	return 0;
}

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

	std::memset(&s_StoredBond, 0, sizeof(s_StoredBond));
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = TestHciCommand;
	s_Peer.pHciDev = &s_Dev;
	s_HciOpCode = 0;
	s_HciCount = 0;
	// The responder cases are this device as peripheral. The Security Request
	// cases set the central role themselves.
	s_Role = BT_CONN_ROLE_PERIPHERAL;

	BtSmpAuthConfig(LocalIoCaps, LocalAuthReq);
	SmpTxReset();
}

// Record a stored bond at the strength the cases below compare against.
void StoreBond(uint8_t EncKeySize, bool bSc, bool bAuthenticated)
{
	std::memset(&s_StoredBond, 0, sizeof(s_StoredBond));
	s_StoredBond.EncKeySize = EncKeySize;
	s_StoredBond.bSc = bSc;
	s_StoredBond.bAuthenticated = bAuthenticated;
	s_StoredBond.bValid = true;
}

// A Pairing Request as a central would send it: Secure Connections, full key
// size, no OOB, distributing the identity key.
void FeedPairingReqKeySize(uint8_t PeerIoCaps, uint8_t PeerAuthReq,
						   uint8_t MaxKeySize)
{
	BtSmpPairingReq_t req = {};
	req.Code = BT_SMP_CODE_PAIRING_REQ;
	req.IOCaps = PeerIoCaps;
	req.OOBFlag = BT_SMP_OOB_AUTH_NOT_PRESENT;
	req.AuthReq = (uint8_t)(PeerAuthReq | BT_SMP_AUTHREQ_SC);
	req.MaxKeySize = MaxKeySize;
	req.InitiatorKeyDist = BT_SMP_KEYDIST_IDKEY;
	req.ResponderKeyDist = BT_SMP_KEYDIST_IDKEY;

	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)&req, sizeof(req));
}

void FeedPairingReq(uint8_t PeerIoCaps, uint8_t PeerAuthReq)
{
	FeedPairingReqKeySize(PeerIoCaps, PeerAuthReq, BT_SMP_MAX_ENC_KEY_SIZE);
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


//-----------------------------------------------------------------------------
// Bond strength. A stored bond must not be replaced by a pairing that would
// end at a lower security level. Three properties decide it: the encryption
// key length, whether Secure Connections produced the key, and whether the
// association model authenticated the peer. Equal strength passes so a
// legitimate re-pair still works, and raising any property is an upgrade.
//
// The re-pair guard in SmpHandlePairingReq only fires once the link is already
// encrypted, so on a fresh connection nothing else consults the stored record
// before BtSmpBondAdd overwrites it.
//-----------------------------------------------------------------------------

// The attack: a Numeric Comparison bond holding a 16 octet Secure Connections
// key, replaced by Just Works from anyone who can present the address.
void TestBondDowngradeRefusesUnauthenticatedModel()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, 0);
	StoreBond(16, true, true);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);

	CHECK(FirstIsPairingFailed(BT_SMP_ERR_AUTHEN_REQUIREMENTS));
	CHECK(FirstIsPairingRsp() == false);
	CHECK(s_SmpLink[0].Ctx.State == BT_SMP_STATE_IDLE);
}

// The same bond, kept authenticated but shrunk to the spec floor.
void TestBondDowngradeRefusesShorterKey()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, 0);
	StoreBond(16, true, true);
	FeedPairingReqKeySize(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM,
						  BT_SMP_MIN_ENC_KEY_SIZE);

	CHECK(FirstIsPairingFailed(BT_SMP_ERR_AUTHEN_REQUIREMENTS));
	CHECK(FirstIsPairingRsp() == false);
}

// Key size is compared, not assumed: one octet down is still down.
void TestBondDowngradeRefusesOneOctetShorter()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, 0);
	StoreBond(16, true, true);
	FeedPairingReqKeySize(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM, 15);

	CHECK(FirstIsPairingFailed(BT_SMP_ERR_AUTHEN_REQUIREMENTS));
}

// Re-pairing at the strength already held has to keep working. This is the
// case that makes the rule a downgrade check rather than an overwrite ban.
void TestBondEqualStrengthIsAccepted()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, 0);
	StoreBond(16, true, true);
	FeedPairingReq(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingRsp());
}

// Raising the model from Just Works to Numeric Comparison is an upgrade.
void TestBondUpgradeToAuthenticatedIsAccepted()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, 0);
	StoreBond(16, true, false);
	FeedPairingReq(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingRsp());
}

// Raising the key length is an upgrade, and an unauthenticated stored bond
// does not require the new pairing to be authenticated.
void TestBondUpgradeKeySizeIsAccepted()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);
	StoreBond(7, true, false);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);

	CHECK(FirstIsPairingRsp());
}

// A legacy stored bond replaced by a Secure Connections one is an upgrade. The
// harness always negotiates SC, so this pins the direction of the bSc test:
// reading it backwards would refuse the pairing that should be encouraged.
void TestBondLegacyReplacedByScIsAccepted()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);
	StoreBond(16, false, false);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);

	CHECK(FirstIsPairingRsp());
}

// First pairing with this peer. Nothing stored, nothing to weaken.
void TestNoStoredBondPairsFreely()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);
	FeedPairingReqKeySize(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0,
						  BT_SMP_MIN_ENC_KEY_SIZE);

	CHECK(FirstIsPairingRsp());
}

// A slot that was found but never populated must not be read as a strong bond
// and lock the peer out.
void TestInvalidStoredBondPairsFreely()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0);
	StoreBond(16, true, true);
	s_StoredBond.bValid = false;
	FeedPairingReqKeySize(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, 0,
						  BT_SMP_MIN_ENC_KEY_SIZE);

	CHECK(FirstIsPairingRsp());
}

// The rule judges the model, not Ctx.bAuthenticated, which is still false
// while the negotiation runs and only becomes true once the model's own check
// has passed. Reading the flag instead would refuse every re-pair of an
// authenticated bond.
void TestRuleJudgesModelNotAuthenticatedFlag()
{
	ResetLink(BT_SMP_IOCAPS_DISPLAY_YESNO, 0);
	StoreBond(16, true, true);
	FeedPairingReq(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingRsp());
	CHECK(s_SmpLink[0].Ctx.bAuthenticated == false);
}


//-----------------------------------------------------------------------------
// Security Request. Vol 3 Part H 2.4.6: "After receiving a Security Request,
// the Central shall first check whether it has the required security
// information to enable encryption... If this information is missing or does
// not meet the security properties requested by the Peripheral, then the
// Central shall initiate the pairing procedure." Figure 2.7 draws the same
// decision: no LTK, or an LTK below the requested level, goes to pair.
//
// The requested level comes from the AuthReq octet. Vol 3 Part H 2.3.1 names
// the Security Properties as LE Secure Connections pairing and Authenticated
// MITM protection, and Vol 3 Part C 10.2.1 defines LE security mode 1 level 4
// as authenticated LE Secure Connections pairing with encryption using a
// 128-bit strength encryption key, so asking for both properties also asks for
// that key length.
//-----------------------------------------------------------------------------

void FeedSecurityReq(uint8_t PeerAuthReq)
{
	BtSmpSecurityReq_t req = {};
	req.Code = BT_SMP_CODE_PAIRING_SECURITY_REQ;
	req.AuthReq = PeerAuthReq;

	BtProcessSmpData(&s_Dev, kConnHdl, (BtL2CapSmp_t*)&req, sizeof(req));
}

bool SentEnableEncryption(void)
{
	return s_HciCount > 0 && s_HciOpCode == BT_HCI_CMD_CTLR_ENABLE_ENCRYPTION;
}

bool SentPairingReq(void)
{
	return g_SmpTx.Count >= 1 &&
		   g_SmpTx.Pdu[0].Data[0] == BT_SMP_CODE_PAIRING_REQ;
}

void ResetCentralLink(uint8_t LocalIoCaps, uint8_t LocalAuthReq)
{
	ResetLink(LocalIoCaps, LocalAuthReq);
	s_Role = BT_CONN_ROLE_CENTRAL;
}

// The stored key meets everything asked for, so encryption setup stands. This
// is the case that must not regress into pairing on every reconnect.
void TestSecurityReqMetByBond()
{
	ResetCentralLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MAX_ENC_KEY_SIZE, true, true);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM | BT_SMP_AUTHREQ_SC);

	CHECK(SentEnableEncryption());
	CHECK(SentPairingReq() == false);
}

// Authenticated MITM protection asked for over a bond that does not have it.
void TestSecurityReqMitmOverUnauthenticatedBondPairs()
{
	ResetCentralLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MAX_ENC_KEY_SIZE, true, false);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM | BT_SMP_AUTHREQ_SC);

	CHECK(SentPairingReq());
	CHECK(SentEnableEncryption() == false);
}

// LE Secure Connections pairing asked for over a legacy bond.
void TestSecurityReqScOverLegacyBondPairs()
{
	ResetCentralLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MAX_ENC_KEY_SIZE, false, false);
	FeedSecurityReq(BT_SMP_AUTHREQ_SC);

	CHECK(SentPairingReq());
	CHECK(SentEnableEncryption() == false);
}

// Both properties asked for is LE security mode 1 level 4, which Vol 3 Part C
// 10.2.1 defines with a 128-bit strength encryption key. A bond that is
// authenticated and from Secure Connections but holds a shorter key does not
// reach that level.
void TestSecurityReqLevel4OverShortKeyPairs()
{
	ResetCentralLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MIN_ENC_KEY_SIZE, true, true);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM | BT_SMP_AUTHREQ_SC);

	CHECK(SentPairingReq());
	CHECK(SentEnableEncryption() == false);
}

// One octet short of 128-bit is still short of it, so the check is a
// comparison and not a test for the floor value.
void TestSecurityReqLevel4OverFifteenOctetKeyPairs()
{
	ResetCentralLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MAX_ENC_KEY_SIZE - 1, true, true);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM | BT_SMP_AUTHREQ_SC);

	CHECK(SentPairingReq());
	CHECK(SentEnableEncryption() == false);
}

// The key length belongs to level 4, so it is not applied when only one of the
// two properties is asked for. MITM alone is level 3, which 10.2.1 states
// without a key length.
void TestSecurityReqMitmOnlyIgnoresKeySize()
{
	ResetCentralLink(BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MIN_ENC_KEY_SIZE, true, true);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM);

	CHECK(SentEnableEncryption());
	CHECK(SentPairingReq() == false);
}

// Nothing asked for beyond what the bond holds. An unauthenticated bond and a
// request with no MITM bit must not pair, or every reconnect would.
void TestSecurityReqNoExtraRequirementEncrypts()
{
	ResetCentralLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MAX_ENC_KEY_SIZE, true, false);
	FeedSecurityReq(BT_SMP_AUTHREQ_SC);

	CHECK(SentEnableEncryption());
	CHECK(SentPairingReq() == false);
}

// A bare Security Request names no property, so whatever is stored meets it.
void TestSecurityReqBareEncrypts()
{
	ResetCentralLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MIN_ENC_KEY_SIZE, false, false);
	FeedSecurityReq(0);

	CHECK(SentEnableEncryption());
	CHECK(SentPairingReq() == false);
}

// "If this information is missing... the Central shall initiate the pairing
// procedure."
void TestSecurityReqNoBondPairs()
{
	ResetCentralLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM | BT_SMP_AUTHREQ_SC);

	CHECK(SentPairingReq());
	CHECK(SentEnableEncryption() == false);
}

// A Security Request travels peripheral to central only, Vol 3 Part H 3.6.7.
// Reading AuthReq must not have moved that gate.
void TestSecurityReqRefusedOnPeripheral()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	StoreBond(BT_SMP_MAX_ENC_KEY_SIZE, true, true);
	FeedSecurityReq(BT_SMP_AUTHREQ_MITM);

	CHECK(FirstIsPairingFailed(BT_SMP_ERR_CMD_NOT_SUPPORTED));
	CHECK(SentEnableEncryption() == false);
}


//-----------------------------------------------------------------------------
// Bonding flags. Vol 3 Part C 9.4.4.2, the bonding procedure: the central
// pairs "with the Bonding_Flags set to Bonding as defined in [Vol 3] Part H,
// Section 3.5.1. If the peer device is in the bondable mode, the devices shall
// exchange and store the bonding information in the security database." So it
// takes both sides. 9.4.2.2 states the other half for a device in non-bondable
// mode: it "shall set the Bonding_Flags to 'No Bonding' ... and bonding
// information shall not be exchanged or stored."
//
// Two consequences, tested separately: the transport keys are not distributed,
// and no record is written to the security database.
//-----------------------------------------------------------------------------

// Key distribution fields of the Pairing Response, which is the first PDU out.
// Byte 5 is InitiatorKeyDist and byte 6 is ResponderKeyDist (Figure 3.4).
bool RspKeyDist(uint8_t *pInit, uint8_t *pResp)
{
	if (!FirstIsPairingRsp() || g_SmpTx.Pdu[0].Len < 7)
	{
		return false;
	}
	*pInit = g_SmpTx.Pdu[0].Data[5];
	*pResp = g_SmpTx.Pdu[0].Data[6];
	return true;
}

// Control. Both sides bondable, so the keys the initiator offered come back in
// the response and phase 3 will run.
void TestBondingBothSidesDistributesKeys()
{
	// Bonding_Flags is the low two bits of AuthReq, so a local configuration
	// that names only SC is No Bonding and would clear the set on its own.
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
			  BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_BONDING);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
				   BT_SMP_AUTHREQ_BONDING_FLAG_BONDING);

	uint8_t ik = 0, rk = 0;
	CHECK(RspKeyDist(&ik, &rk));
	CHECK((ik & BT_SMP_KEYDIST_IDKEY) != 0);
	CHECK((rk & BT_SMP_KEYDIST_IDKEY) != 0);
}

// The peer is non-bondable. Nothing may be exchanged, so the negotiated set
// comes back with the transported keys cleared.
void TestPeerNoBondingClearsKeyDistribution()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_AUTHREQ_SC);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
				   BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING);

	uint8_t ik = 0, rk = 0;
	CHECK(RspKeyDist(&ik, &rk));
	CHECK((ik & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY)) == 0);
	CHECK((rk & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY)) == 0);

	// Still a Pairing Response, not a refusal. Non-bondable is a legitimate
	// pairing, it just produces no bond.
	CHECK(FirstIsPairingRsp());
}

// This device is non-bondable and the peer is not. Same outcome: it takes both.
void TestLocalNoBondingClearsKeyDistribution()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
			  BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
				   BT_SMP_AUTHREQ_BONDING_FLAG_BONDING);

	uint8_t ik = 0, rk = 0;
	CHECK(RspKeyDist(&ik, &rk));
	CHECK((ik & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY)) == 0);
	CHECK((rk & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY)) == 0);
}

void TestNeitherSideBondingClearsKeyDistribution()
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
			  BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING);
	FeedPairingReq(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
				   BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING);

	uint8_t ik = 0, rk = 0;
	CHECK(RspKeyDist(&ik, &rk));
	CHECK((ik & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY)) == 0);
	CHECK((rk & (BT_SMP_KEYDIST_IDKEY | BT_SMP_KEYDIST_SIGNKEY)) == 0);
}

// The second half of the clause, "or stored". Drive the link to the point
// where encryption has just come up on a fresh pairing, which is where the
// record is written, and count the writes. The harness has no crypto engine so
// the pairing cannot be run end to end; the state the encryption event acts on
// is set directly instead.
void DriveEncryptionChanged(uint8_t LocalAuthReq, uint8_t PeerAuthReq)
{
	ResetLink(BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, LocalAuthReq);
	SmpBondAddReset();

	// The identity IRK is drawn from the secure RBG on first use, and this
	// harness binds no crypto engines. Seed it so BtSmpLocalIrkGet succeeds and
	// the distribution path runs, which is what a device has after first boot.
	// Without this the IRK fetch fails, the pairing is aborted before the
	// record is written, and the no-bonding cases below would pass for the
	// wrong reason.
	std::memset(s_SmpLocalId.Irk, 0x5A, sizeof(s_SmpLocalId.Irk));
	s_SmpLocalId.bValid = true;

	BtSmpLink_t *pLink = SmpLinkAlloc(kConnHdl);
	if (pLink == nullptr)
	{
		return;
	}
	pLink->Ctx.State = BT_SMP_STATE_LTK_WAIT;
	pLink->Ctx.bInitiator = false;
	pLink->Ctx.bSc = true;
	pLink->Ctx.PeerAuthReq = PeerAuthReq;
	// Negotiated key distribution as SmpBuildPairingRsp would have left it for
	// a bondable pairing: byte 5 initiator, byte 6 responder.
	pLink->Ctx.PRsp[5] = BT_SMP_KEYDIST_IDKEY;
	pLink->Ctx.PRsp[6] = BT_SMP_KEYDIST_IDKEY;
	pLink->Keys.bValid = true;
	pLink->Keys.EncKeySize = BT_SMP_MAX_ENC_KEY_SIZE;
	pLink->Keys.bSc = true;

	BtSmpEncryptionChanged(&s_Dev, kConnHdl, 0, 1);
}

void TestBondingStoresTheRecord()
{
	DriveEncryptionChanged(BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_BONDING,
						   BT_SMP_AUTHREQ_BONDING_FLAG_BONDING);

	CHECK(g_SmpBondAddCount == 1);
}

void TestPeerNoBondingStoresNothing()
{
	DriveEncryptionChanged(BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_BONDING,
						   BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING);

	CHECK(g_SmpBondAddCount == 0);
}

void TestLocalNoBondingStoresNothing()
{
	DriveEncryptionChanged(BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING,
						   BT_SMP_AUTHREQ_BONDING_FLAG_BONDING);

	CHECK(g_SmpBondAddCount == 0);
}

// A non-bondable pairing distributes nothing, so the identity IRK and the CSRK
// do not leave this device either.
void TestNoBondingSendsNoKeyPdus()
{
	DriveEncryptionChanged(BT_SMP_AUTHREQ_SC | BT_SMP_AUTHREQ_BONDING_FLAG_BONDING,
						   BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING);

	for (size_t i = 0; i < g_SmpTx.Count && i < SMP_TX_CAPTURE_MAX; i++)
	{
		uint8_t code = g_SmpTx.Pdu[i].Data[0];
		CHECK(code != BT_SMP_CODE_PAIRING_ID_INFO);
		CHECK(code != BT_SMP_CODE_PAIRING_ID_ADDR_INFO);
		CHECK(code != BT_SMP_CODE_PAIRING_SIGNING_INFO);
	}
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
	return s_Role;
}

void BtGapConnSecSet(uint16_t, const BtConnSec_t *)
{
}

void BtGattCccdRestoreBonded(uint16_t)
{
}

bool BtSmpBondKeysLookup(uint16_t, uint64_t, uint16_t, BtSmpKeys_t *pKeys)
{
	if (pKeys == nullptr || !s_StoredBond.bValid)
	{
		return false;
	}
	std::memcpy(pKeys, &s_StoredBond, sizeof(*pKeys));
	return true;
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
	TestBondDowngradeRefusesUnauthenticatedModel();
	TestBondDowngradeRefusesShorterKey();
	TestBondDowngradeRefusesOneOctetShorter();
	TestBondEqualStrengthIsAccepted();
	TestBondUpgradeToAuthenticatedIsAccepted();
	TestBondUpgradeKeySizeIsAccepted();
	TestBondLegacyReplacedByScIsAccepted();
	TestNoStoredBondPairsFreely();
	TestInvalidStoredBondPairsFreely();
	TestRuleJudgesModelNotAuthenticatedFlag();
	TestSecurityReqMetByBond();
	TestSecurityReqMitmOverUnauthenticatedBondPairs();
	TestSecurityReqScOverLegacyBondPairs();
	TestSecurityReqLevel4OverShortKeyPairs();
	TestSecurityReqLevel4OverFifteenOctetKeyPairs();
	TestSecurityReqMitmOnlyIgnoresKeySize();
	TestSecurityReqNoExtraRequirementEncrypts();
	TestSecurityReqBareEncrypts();
	TestSecurityReqNoBondPairs();
	TestSecurityReqRefusedOnPeripheral();
	TestBondingBothSidesDistributesKeys();
	TestPeerNoBondingClearsKeyDistribution();
	TestLocalNoBondingClearsKeyDistribution();
	TestNeitherSideBondingClearsKeyDistribution();
	TestBondingStoresTheRecord();
	TestPeerNoBondingStoresNothing();
	TestLocalNoBondingStoresNothing();
	TestNoBondingSendsNoKeyPdus();

	if (s_Failures != 0)
	{
		std::printf("SMP pairing policy tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SMP pairing policy tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
