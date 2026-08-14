// Coverage for the host controlled Link Layer FeatureSet claim, Core Vol 4
// Part E 7.8.115 and Vol 6 Part B Table 4.7.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci_cap.h"

// Supported_Commands octet 44 bit 1 is HCI_LE_Set_Host_Feature, Core Vol 4
// Part E 6.27.
static_assert(BT_HCI_CAP_CMD_LE_SET_HOST_FEATURE == 353U);

// FeatureSet bit 40 is Advertising Coding Selection, bit 41 its Host Support
// companion, Core Vol 6 Part B Table 4.7.
static_assert(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION == 40U);
static_assert(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST == 41U);

namespace {

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

struct CapturedCmd {
	uint16_t OpCode;
	uint8_t Param[8];
	uint8_t ParamLen;
};

CapturedCmd s_Cmd;
int s_CmdCount = 0;
int s_OtherCmdCount = 0;
uint8_t s_CommandStatus = 0;

BtHciCapabilities_t s_Cap;
BtHciDevice_t s_Dev;

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode,
	const void *pParam, uint8_t ParamLen, void *, uint8_t)
{
	if (OpCode != BT_HCI_CMD_CTLR_SET_HOST_FEATURE)
	{
		s_OtherCmdCount++;
		return 1;
	}

	s_CmdCount++;
	s_Cmd.OpCode = OpCode;
	s_Cmd.ParamLen = ParamLen;
	std::memset(s_Cmd.Param, 0, sizeof(s_Cmd.Param));
	if (pParam != nullptr && ParamLen <= sizeof(s_Cmd.Param))
	{
		std::memcpy(s_Cmd.Param, pParam, ParamLen);
	}

	return s_CommandStatus;
}

void SetCommandBit(uint16_t CommandBit)
{
	s_Cap.SupportedCommands[CommandBit >> 3] |=
		static_cast<uint8_t>(1U << (CommandBit & 7U));
}

void ClearCommandBit(uint16_t CommandBit)
{
	s_Cap.SupportedCommands[CommandBit >> 3] &=
		static_cast<uint8_t>(~(1U << (CommandBit & 7U)));
}

void SetFeatureBit(uint8_t FeatureBit)
{
	s_Cap.LeFeatures[FeatureBit >> 3] |=
		static_cast<uint8_t>(1U << (FeatureBit & 7U));
}

void ClearFeatureBit(uint8_t FeatureBit)
{
	s_Cap.LeFeatures[FeatureBit >> 3] &=
		static_cast<uint8_t>(~(1U << (FeatureBit & 7U)));
}

bool FeatureBitSet(uint8_t FeatureBit)
{
	return (s_Cap.LeFeatures[FeatureBit >> 3] &
		(1U << (FeatureBit & 7U))) != 0;
}

// A controller that reports everything the claim needs and nothing the claim
// produces: bit 41 stays clear until LE Set Host Feature succeeds.
void Setup()
{
	std::memset(&s_Cmd, 0, sizeof(s_Cmd));
	s_CmdCount = 0;
	s_OtherCmdCount = 0;
	s_CommandStatus = 0;

	std::memset(&s_Cap, 0, sizeof(s_Cap));
	s_Cap.Valid = BT_HCI_CAP_VALID_COMMANDS | BT_HCI_CAP_VALID_LE_FEATURES;
	SetCommandBit(BT_HCI_CAP_CMD_LE_SET_HOST_FEATURE);
	SetCommandBit(BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS_V2);
	SetFeatureBit(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION);

	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = CaptureCommand;
}

void TestInvalidArguments()
{
	Setup();
	CHECK(BtHciHostFeaturesClaim(nullptr, &s_Cap) == false);
	CHECK(BtHciHostFeaturesClaim(&s_Dev, nullptr) == false);

	s_Dev.Command = nullptr;
	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap) == false);

	CHECK(s_CmdCount == 0);
	CHECK(FeatureBitSet(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST)
		== false);
}

void TestClaimIssuesTheCommand()
{
	Setup();
	CHECK(BtHciCapabilitiesAdvertisingCodingSelectionSupported(&s_Cap)
		== false);

	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap));
	CHECK(s_CmdCount == 1);
	CHECK(s_OtherCmdCount == 0);
	CHECK(s_Cmd.OpCode == BT_HCI_CMD_CTLR_SET_HOST_FEATURE);
	CHECK(s_Cmd.ParamLen == 2);
	CHECK(s_Cmd.Param[0] == 41);
	CHECK(s_Cmd.Param[1] == 1);

	// The discovered record has to agree with the FeatureSet the controller
	// now holds, because every coding decision is taken against this copy.
	CHECK(FeatureBitSet(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST));
	CHECK(BtHciCapabilitiesAdvertisingCodingSelectionSupported(&s_Cap));
}

void TestClaimNeedsTheCommand()
{
	Setup();
	ClearCommandBit(BT_HCI_CAP_CMD_LE_SET_HOST_FEATURE);

	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap) == false);
	CHECK(s_CmdCount == 0);
	CHECK(FeatureBitSet(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST)
		== false);
	CHECK(BtHciCapabilitiesAdvertisingCodingSelectionSupported(&s_Cap)
		== false);
}

// 7.8.115 answers a claim whose controller side is missing with Unsupported
// Feature or Parameter Value, so a controller without bit 40 is not asked.
void TestClaimNeedsTheControllerFeature()
{
	Setup();
	ClearFeatureBit(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION);

	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap) == false);
	CHECK(s_CmdCount == 0);
	CHECK(FeatureBitSet(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST)
		== false);
}

// Nothing was read from the controller, so nothing is known. An unread record
// is all zeroes, which must not be taken as a controller that answered.
void TestClaimNeedsDiscoveredCapabilities()
{
	Setup();
	s_Cap.Valid &= ~BT_HCI_CAP_VALID_LE_FEATURES;
	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap) == false);
	CHECK(s_CmdCount == 0);

	Setup();
	s_Cap.Valid &= ~BT_HCI_CAP_VALID_COMMANDS;
	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap) == false);
	CHECK(s_CmdCount == 0);
}

// A refusal is not fatal, it only means the coding stays off air. The record
// must not claim a bit the controller did not set: Command Disallowed is what
// 7.8.115 returns when a connection already exists.
void TestRefusalLeavesTheBitClear()
{
	const uint8_t status[] = { 0x0C, 0x11, 0x01 };

	for (unsigned i = 0; i < sizeof(status); i++)
	{
		Setup();
		s_CommandStatus = status[i];

		CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap) == false);
		CHECK(s_CmdCount == 1);
		CHECK(FeatureBitSet(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST)
			== false);
		CHECK(BtHciCapabilitiesAdvertisingCodingSelectionSupported(&s_Cap)
			== false);
	}
}

// The v2 form of LE Set Extended Advertising Parameters is the only command
// with the PHY options fields, so a successful claim on a controller without
// it still leaves coding unusable.
void TestClaimAloneDoesNotEnableCoding()
{
	Setup();
	ClearCommandBit(BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS_V2);

	CHECK(BtHciHostFeaturesClaim(&s_Dev, &s_Cap));
	CHECK(FeatureBitSet(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION_HOST));
	CHECK(BtHciCapabilitiesAdvertisingCodingSelectionSupported(&s_Cap)
		== false);
}

void TestFeatureMarkBounds()
{
	Setup();
	uint8_t before[8];
	std::memcpy(before, s_Cap.LeFeatures, sizeof(before));

	BtHciCapabilitiesLeFeatureMark(nullptr, 41);
	BtHciCapabilitiesLeFeatureMark(&s_Cap, 64);
	BtHciCapabilitiesLeFeatureMark(&s_Cap, 255);
	CHECK(std::memcmp(before, s_Cap.LeFeatures, sizeof(before)) == 0);

	BtHciCapabilitiesLeFeatureMark(&s_Cap, 63);
	CHECK(FeatureBitSet(63));
	CHECK(s_Cap.LeFeatures[7] == 0x80);
}

} // namespace

int main()
{
	TestInvalidArguments();
	TestClaimIssuesTheCommand();
	TestClaimNeedsTheCommand();
	TestClaimNeedsTheControllerFeature();
	TestClaimNeedsDiscoveredCapabilities();
	TestRefusalLeavesTheBitClear();
	TestClaimAloneDoesNotEnableCoding();
	TestFeatureMarkBounds();

	if (s_Failures == 0)
	{
		std::printf("PASS: %d host feature checks\n", s_Checks);
	}
	else
	{
		std::printf("FAIL: %d of %d host feature checks failed\n",
			s_Failures, s_Checks);
	}
	return s_Failures == 0 ? 0 : 1;
}
