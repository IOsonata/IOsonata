// Command-level coverage for the advertising HCI path: capability-based
// command selection, own-address validation, advertising state, extended-data
// fragmentation, and manufacturer company identifier byte order. BtHciCommand
// dispatches through the device Command pointer, so a capture device records
// every opcode and packed parameter block. No controller is involved.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_adv.h"
#include "convutil.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"

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
	uint8_t  Param[264];
	uint16_t ParamLen;
};

CapturedCmd s_Cmds[64];
int s_CmdCount = 0;

uint8_t s_LocalType = 0;
uint8_t s_LocalAddr[6] = {};

uint16_t s_FailOpCode = 0;
uint8_t  s_FailEnable = 0xFF;

enum CapabilityMode {
	CAP_EXTENDED,
	CAP_EXTENDED_NO_2M,
	CAP_LEGACY,
	CAP_LEGACY_MISSING_DATA,
	CAP_EXT_LIMIT_SMALL,
	CAP_EXT_ZERO_SETS,
	CAP_EXT_CODING_REFUSED,		// bit 40, and the claim of bit 41 is refused
	CAP_EXT_CODING_HOST,		// bit 40, and the claim of bit 41 succeeds
	CAP_EXT_CODING_NO_FEATURE,
};

CapabilityMode s_CapabilityMode = CAP_EXTENDED;

void SetCommandBit(uint8_t commands[64], uint16_t bit)
{
	commands[bit >> 3] |= static_cast<uint8_t>(1U << (bit & 7U));
}

void CopyReturn(void *pRet, uint8_t RetLen, const void *pData, size_t DataLen)
{
	if (pRet != nullptr && RetLen > 0)
	{
		size_t len = RetLen < DataLen ? RetLen : DataLen;
		std::memcpy(pRet, pData, len);
	}
}

bool CapabilityCommand(uint16_t OpCode, void *pRet, uint8_t RetLen,
	uint8_t *pStatus)
{
	uint8_t data[64] = {};
	*pStatus = 0;

	switch (OpCode)
	{
		case BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO:
		{
			const uint8_t value[8] = { 0x0D, 0, 0, 0x0D, 0x59, 0, 0, 0 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			return true;
		}

		case BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS:
			SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS);
			SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_ADV_PARAMETERS);
			if (s_CapabilityMode != CAP_LEGACY_MISSING_DATA)
			{
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_ADV_DATA);
			}
			SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_SCAN_RESPONSE_DATA);
			SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_ADV_ENABLE);

			if (s_CapabilityMode != CAP_LEGACY &&
				s_CapabilityMode != CAP_LEGACY_MISSING_DATA)
			{
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_READ_MAX_ADV_DATA_LENGTH);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_READ_SUPPORTED_ADV_SETS);
			}

			if (s_CapabilityMode == CAP_EXT_CODING_REFUSED ||
				s_CapabilityMode == CAP_EXT_CODING_HOST ||
				s_CapabilityMode == CAP_EXT_CODING_NO_FEATURE)
			{
				SetCommandBit(data,
					BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS_V2);
				SetCommandBit(data, BT_HCI_CAP_CMD_LE_SET_HOST_FEATURE);
			}

			CopyReturn(pRet, RetLen, data, sizeof(data));
			return true;

		case BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES:
			if (s_CapabilityMode != CAP_LEGACY &&
				s_CapabilityMode != CAP_LEGACY_MISSING_DATA)
			{
				data[BT_HCI_CAP_LE_FEATURE_EXT_ADV >> 3] |=
					static_cast<uint8_t>(1U <<
					(BT_HCI_CAP_LE_FEATURE_EXT_ADV & 7U));
			}
			if (s_CapabilityMode != CAP_EXTENDED_NO_2M &&
				s_CapabilityMode != CAP_LEGACY &&
				s_CapabilityMode != CAP_LEGACY_MISSING_DATA)
			{
				data[BT_HCI_CAP_LE_FEATURE_PHY_2M >> 3] |=
					static_cast<uint8_t>(1U <<
					(BT_HCI_CAP_LE_FEATURE_PHY_2M & 7U));
			}
			if (s_CapabilityMode == CAP_EXT_CODING_REFUSED ||
				s_CapabilityMode == CAP_EXT_CODING_HOST ||
				s_CapabilityMode == CAP_EXT_CODING_NO_FEATURE)
			{
				data[BT_HCI_CAP_LE_FEATURE_CODED_PHY >> 3] |=
					static_cast<uint8_t>(1U <<
					(BT_HCI_CAP_LE_FEATURE_CODED_PHY & 7U));
			}
			// CAP_EXT_CODING_NO_FEATURE has the v2 command and the coded PHY
			// but not the Advertising Coding Selection feature itself.
			if (s_CapabilityMode == CAP_EXT_CODING_REFUSED ||
				s_CapabilityMode == CAP_EXT_CODING_HOST)
			{
				data[BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION >> 3] |=
					static_cast<uint8_t>(1U <<
					(BT_HCI_CAP_LE_FEATURE_ADV_CODING_SELECTION & 7U));
			}
			// Bit 41 is never reported here. Vol 6 Part B Table 4.7 makes it
			// host controlled, so it reads as clear until LE Set Host Feature
			// sets it, and the read that fills this record runs before the
			// claim does.
			CopyReturn(pRet, RetLen, data, 8);
			return true;

		// Core Vol 4 Part E 7.8.115. CAP_EXT_CODING_REFUSED is the controller
		// that reports the feature and refuses the claim, which is what a
		// controller with a connection already up answers.
		case BT_HCI_CMD_CTLR_SET_HOST_FEATURE:
			if (s_CapabilityMode == CAP_EXT_CODING_REFUSED)
			{
				*pStatus = BT_HCI_ERR_COMMAND_DISALLOWED;
			}
			return true;

		case BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES:
			CopyReturn(pRet, RetLen, data, 8);
			return true;

		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT:
		{
			const uint8_t value[6] = { 251, 0, 3, 0, 0, 0 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			return true;
		}

		case BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN:
		{
			const uint8_t value[8] = { 251, 0, 0x48, 0x08, 251, 0, 0x48, 0x08 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			return true;
		}

		case BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE:
			data[0] = 8;
			CopyReturn(pRet, RetLen, data, 1);
			return true;

		case BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN:
			if (s_CapabilityMode == CAP_LEGACY ||
				s_CapabilityMode == CAP_LEGACY_MISSING_DATA)
			{
				*pStatus = BT_HCI_ERR_UNKNOWN_COMMAND;
				return true;
			}
			if (s_CapabilityMode == CAP_EXT_LIMIT_SMALL)
			{
				data[0] = 8;
				data[1] = 0;
			}
			else
			{
				// Core maximum Host Advertising Data length: 1650 = 0x0672.
				data[0] = 0x72;
				data[1] = 0x06;
			}
			CopyReturn(pRet, RetLen, data, 2);
			return true;

		case BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS:
			if (s_CapabilityMode == CAP_LEGACY ||
				s_CapabilityMode == CAP_LEGACY_MISSING_DATA)
			{
				*pStatus = BT_HCI_ERR_UNKNOWN_COMMAND;
				return true;
			}
			data[0] = s_CapabilityMode == CAP_EXT_ZERO_SETS ? 0 : 1;
			CopyReturn(pRet, RetLen, data, 1);
			return true;

		case BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE:
			*pStatus = BT_HCI_ERR_UNKNOWN_COMMAND;
			return true;

		default:
			return false;
	}
}

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
	uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (s_CmdCount < (int)(sizeof(s_Cmds) / sizeof(s_Cmds[0])))
	{
		CapturedCmd &c = s_Cmds[s_CmdCount++];
		c.OpCode = OpCode;
		c.ParamLen = ParamLen;
		if (pParam != nullptr)
		{
			std::memcpy(c.Param, pParam, ParamLen);
		}
	}

	uint8_t status;
	if (CapabilityCommand(OpCode, pRet, RetLen, &status))
	{
		return status;
	}

	if (s_FailOpCode != 0 && OpCode == s_FailOpCode)
	{
		const uint8_t *p = (const uint8_t *)pParam;
		if (s_FailEnable == 0xFF ||
			(p != nullptr && ParamLen >= 1 && p[0] == s_FailEnable))
		{
			return BT_HCI_ERR_COMMAND_DISALLOWED;
		}
	}

	return BT_HCI_SUCCESS;
}

BtHciDevice_t s_Dev;

void ClearCommands()
{
	s_CmdCount = 0;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
}

void Setup(uint8_t LocalType, const uint8_t LocalAddr[6],
	CapabilityMode Mode = CAP_EXTENDED)
{
	ClearCommands();
	s_FailOpCode = 0;
	s_FailEnable = 0xFF;
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = CaptureCommand;
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	g_BtAppData.State = BTAPP_STATE_IDLE;
	s_LocalType = LocalType;
	std::memcpy(s_LocalAddr, LocalAddr, 6);
	s_CapabilityMode = Mode;
}

const CapturedCmd *FindCmd(uint16_t OpCode)
{
	for (int i = 0; i < s_CmdCount; i++)
	{
		if (s_Cmds[i].OpCode == OpCode)
		{
			return &s_Cmds[i];
		}
	}
	return nullptr;
}

const CapturedCmd *FindLastCmd(uint16_t OpCode)
{
	for (int i = s_CmdCount - 1; i >= 0; i--)
	{
		if (s_Cmds[i].OpCode == OpCode)
		{
			return &s_Cmds[i];
		}
	}
	return nullptr;
}

int FindCmdIndex(uint16_t OpCode, uint8_t FirstParam = 0xFF)
{
	for (int i = 0; i < s_CmdCount; i++)
	{
		if (s_Cmds[i].OpCode == OpCode &&
			(FirstParam == 0xFF ||
			 (s_Cmds[i].ParamLen > 0 && s_Cmds[i].Param[0] == FirstParam)))
		{
			return i;
		}
	}
	return -1;
}

const uint8_t *FindExtAdvDataRecord(const CapturedCmd *pCommand,
	uint8_t Type, uint8_t *pPayloadLen)
{
	if (pCommand == nullptr || pCommand->ParamLen < 4 || pPayloadLen == nullptr)
	{
		return nullptr;
	}

	int remaining = pCommand->Param[3];
	int offset = 4;
	if (remaining > pCommand->ParamLen - offset)
	{
		return nullptr;
	}

	while (remaining > 0)
	{
		uint8_t len = pCommand->Param[offset];
		int recordLen = len + 1;
		if (len == 0 || recordLen > remaining || offset + recordLen > pCommand->ParamLen)
		{
			return nullptr;
		}
		if (pCommand->Param[offset + 1] == Type)
		{
			*pPayloadLen = len - 1;
			return &pCommand->Param[offset + 2];
		}
		offset += recordLen;
		remaining -= recordLen;
	}

	return nullptr;
}

bool HasAdvertisingCommand()
{
	return FindCmd(BT_HCI_CMD_CTLR_SET_ADV_PARAM) != nullptr ||
		FindCmd(BT_HCI_CMD_CTLR_SET_ADV_DATA) != nullptr ||
		FindCmd(BT_HCI_CMD_CTLR_SET_ADV_ENABLE) != nullptr ||
		FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM) != nullptr ||
		FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA) != nullptr ||
		FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE) != nullptr;
}

BtAppCfg_t MakePeripheralCfg(const char *pName = "Test")
{
	BtAppCfg_t cfg;
	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Role = BTAPP_ROLE_PERIPHERAL;
	cfg.pDevName = pName;
	cfg.Appearance = BT_APPEAR_UNKNOWN_GENERIC;
	cfg.AdvInterval = 100;
	cfg.AdvTimeout = 0;
	cfg.VendorId = 0x1234;
	return cfg;
}

constexpr size_t kExtAdvOwnAddr = 10;
constexpr size_t kExtAdvPrimPhy = 20;
constexpr size_t kExtAdvSecPhy = 22;
constexpr size_t kExtAdvPrimPhyOptions = 25;
constexpr size_t kExtAdvSecPhyOptions = 26;
constexpr size_t kLegacyAdvOwnAddr = 5;

// LE Set Extended Advertising Parameters, Core Vol 4 Part E 7.8.53: handle,
// then the 2-octet event properties, then the two 3-octet primary intervals.
constexpr size_t kExtAdvPrimIntervalMin = 3;
constexpr size_t kExtAdvPrimIntervalMax = 6;

uint32_t Rd24(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

// An interval past what 16 bits of 0.625 ms units can hold. 41000 ms is
// 65600 units, inside the 0x000020 to 0xFFFFFF that 7.8.53 allows for an
// extended set. Converting through a float and casting to uint16_t made this
// undefined: it produced 64 units, a 40 ms interval, and reported success.
// A fractional millisecond that is a whole number of 0.625 ms units has to
// survive the conversion. 7.5 ms is 12 units exactly, and is the value the
// specification uses for the shortest connection interval, so it is the one
// most likely to be written into a configuration by hand.
void TestAdvIntervalFractionalMilliseconds()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	cfg.AdvInterval = 152;				// the field is whole milliseconds
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		// 152 ms is 243.2 units, and the controller takes whole units.
		CHECK(Rd24(&ap->Param[kExtAdvPrimIntervalMin]) == 243U);
	}

	// The conversion itself keeps a value that lands on a unit boundary,
	// which the millisecond field above cannot express but callers passing a
	// float can. 7.5 ms is 12 units, not 11.
	CHECK(mSecTo0_625(7.5F) == 12U);
	CHECK(mSecTo0_625(11.25F) == 18U);
	CHECK(mSecTo0_625(152.5F) == 244U);
}

void TestAdvIntervalBeyond16Bits()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	cfg.AdvInterval = 41000;
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(Rd24(&ap->Param[kExtAdvPrimIntervalMin]) == 65600U);
		CHECK(Rd24(&ap->Param[kExtAdvPrimIntervalMax]) == 65680U);
	}
}

// Well up the extended range, which 16 bits could not reach at all.
void TestAdvIntervalNearExtendedMaximum()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	cfg.AdvInterval = 10000000;			// 16,000,000 units
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(Rd24(&ap->Param[kExtAdvPrimIntervalMin]) == 16000000U);
	}
}

// Past 0xFFFFFF the command cannot express it, so the request is refused
// rather than wrapped. This check could never fail while the conversion
// truncated to 16 bits, because the value could not reach the bound.
void TestAdvIntervalPastExtendedMaximumRefused()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	cfg.AdvInterval = 11000000;			// 17,600,000 units, over 0xFFFFFF
	CHECK(BtAppAdvInit(&cfg) == false);
}

void TestAdvPublicIdentity()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kExtAdvOwnAddr] == BTADDR_TYPE_PUBLIC);
	}
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR) == nullptr);
}

void TestAdvRandomIdentity()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kExtAdvOwnAddr] == BTADDR_TYPE_RAND);
	}
	const CapturedCmd *ra = FindCmd(BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR);
	CHECK(ra != nullptr);
	if (ra != nullptr)
	{
		CHECK(std::memcmp(ra->Param + 1, addr, 6) == 0);
	}
}

void TestAdvInvalidRandomIdentity()
{
	uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == false);
}

void TestAdvDegenerateRandomIdentity()
{
	uint8_t allZero[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, allZero);
	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == false);

	uint8_t allOne[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	Setup(BTADDR_TYPE_RANDOM_STATIC, allOne);
	cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == false);

	uint8_t oneBitSet[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC1 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, oneBitSet);
	cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
}

void TestLegacyCommandSelection()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_LEGACY);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM) == nullptr);
	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kLegacyAdvOwnAddr] == BTADDR_TYPE_PUBLIC);
	}
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_ADV_DATA) != nullptr);

	BtAppAdvStart();
	CHECK(FindCmdIndex(BT_HCI_CMD_CTLR_SET_ADV_ENABLE, 1) >= 0);
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);
}

void TestLegacyMissingCommandRefused()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_LEGACY_MISSING_DATA);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == false);
	CHECK(HasAdvertisingCommand() == false);
}

void TestExtendedPayloadNeedsExtendedCommands()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_LEGACY);

	BtAppCfg_t cfg = MakePeripheralCfg(
		"This name is long enough to require extended advertising data");
	CHECK(BtAppAdvInit(&cfg) == false);
	CHECK(HasAdvertisingCommand() == false);
}

void TestExtendedLimitFallsBackToLegacy()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_LIMIT_SMALL);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_ADV_PARAM) != nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM) == nullptr);
}

void TestExtendedLimitRejectsExtendedPayload()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_LIMIT_SMALL);

	BtAppCfg_t cfg = MakePeripheralCfg(
		"This name is long enough to require extended advertising data");
	CHECK(BtAppAdvInit(&cfg) == false);
	CHECK(HasAdvertisingCommand() == false);
}

void TestZeroAdvertisingSetsFallsBackToLegacy()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_ZERO_SETS);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_ADV_PARAM) != nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM) == nullptr);
}

void TestExtendedSecondaryPhySelection()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXTENDED_NO_2M);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kExtAdvSecPhy] == BTADV_EXTADV_PHY_1M);
	}
}

void TestAdvertisingCodingSelectionArguments()
{
	uint8_t prim = 0xFF;
	uint8_t sec = 0xFF;

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_REQUIRE_S8,
		BT_HCI_ADV_PHY_OPT_PREFER_S2));
	BtAdvCodingSelectionGet(&prim, &sec);
	CHECK(prim == BT_HCI_ADV_PHY_OPT_REQUIRE_S8);
	CHECK(sec == BT_HCI_ADV_PHY_OPT_PREFER_S2);

	// Out of range on either argument changes nothing.
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_MAX + 1,
		BT_HCI_ADV_PHY_OPT_NONE) == false);
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_MAX + 1) == false);
	BtAdvCodingSelectionGet(&prim, &sec);
	CHECK(prim == BT_HCI_ADV_PHY_OPT_REQUIRE_S8);
	CHECK(sec == BT_HCI_ADV_PHY_OPT_PREFER_S2);

	// A NULL output is skipped, not written through.
	BtAdvCodingSelectionGet(nullptr, nullptr);

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_NONE));
}

// The controller has Advertising Coding Selection and the v2 command, but it
// refuses the host support bit. Vol 6 Part B Table 4.7 makes bit 41 host
// controlled and Vol 4 Part E 7.8.115 is the only way to set it, so with the
// claim refused the controller puts no coding on air whatever the parameters
// say. Sending it anyway is the state this branch shipped in.
void TestAdvertisingCodingSelectionNeedsTheHostBit()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_CODING_REFUSED);
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_REQUIRE_S8,
		BT_HCI_ADV_PHY_OPT_PREFER_S2));

	BtAppCfg_t cfg = MakePeripheralCfg(
		"This name is long enough to require extended advertising data");
	CHECK(BtAppAdvInit(&cfg) == true);

	// The claim was attempted and answered Command Disallowed.
	const CapturedCmd *hf = FindCmd(BT_HCI_CMD_CTLR_SET_HOST_FEATURE);
	CHECK(hf != nullptr);

	// No coding to carry, so the v1 command goes out and both PHYs stay off
	// the coded one.
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM_V2) == nullptr);
	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kExtAdvPrimPhy] != BTADV_EXTADV_PHY_CODED);
		CHECK(ap->Param[kExtAdvSecPhy] != BTADV_EXTADV_PHY_CODED);
	}

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_NONE));
}

void TestAdvertisingCodingSelectionIgnoredOnLegacySet()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_CODING_HOST);
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_REQUIRE_S8,
		BT_HCI_ADV_PHY_OPT_REQUIRE_S2));

	BtAppCfg_t cfg = MakePeripheralCfg();		// short name, legacy PDUs
	CHECK(BtAppAdvInit(&cfg) == true);

	// No coding to carry, so the v1 command is the one that goes out.
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM_V2) == nullptr);
	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK((ap->Param[1] & BTADV_EXTADV_EVT_PROP_LEGACY) != 0);
		CHECK(ap->Param[kExtAdvPrimPhy] == BTADV_EXTADV_PHY_1M);
	}

	// The request is remembered, since the getter reports what was asked for
	// rather than what the set was able to use.
	uint8_t prim = 0;
	uint8_t sec = 0;
	BtAdvCodingSelectionGet(&prim, &sec);
	CHECK(prim == BT_HCI_ADV_PHY_OPT_REQUIRE_S8);
	CHECK(sec == BT_HCI_ADV_PHY_OPT_REQUIRE_S2);

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_NONE));
}

void TestAdvertisingCodingSelectionApplied()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_CODING_HOST);
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_REQUIRE_S8,
		BT_HCI_ADV_PHY_OPT_PREFER_S2));

	// The payload has to force extended PDUs. A coding cannot apply to a set
	// using legacy ones, so driving this from the short default name tested
	// nothing the specification allows.
	BtAppCfg_t cfg = MakePeripheralCfg(
		"This name is long enough to require extended advertising data");
	CHECK(BtAppAdvInit(&cfg) == true);

	// Bit 41 is not reported by the controller read, so the coding below only
	// reaches the air because the claim ran and succeeded first. Core Vol 4
	// Part E 7.8.115 fixes the two parameters.
	const CapturedCmd *hf = FindCmd(BT_HCI_CMD_CTLR_SET_HOST_FEATURE);
	CHECK(hf != nullptr);
	if (hf != nullptr)
	{
		CHECK(hf->ParamLen == 2);
		CHECK(hf->Param[0] == 41);
		CHECK(hf->Param[1] == 1);
	}

	// The v2 command is the only one with the coding fields, so the
	// v1 form must not be the one that went out.
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM) == nullptr);
	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM_V2);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->ParamLen == 27);
		CHECK(ap->Param[kExtAdvPrimPhyOptions] ==
			BT_HCI_ADV_PHY_OPT_REQUIRE_S8);
		CHECK(ap->Param[kExtAdvSecPhyOptions] ==
			BT_HCI_ADV_PHY_OPT_PREFER_S2);
		// A coding is meaningless unless the channel is on the coded PHY.
		CHECK(ap->Param[kExtAdvPrimPhy] == BTADV_EXTADV_PHY_CODED);
		CHECK(ap->Param[kExtAdvSecPhy] == BTADV_EXTADV_PHY_CODED);
	}

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_NONE));
}

void TestAdvertisingCodingSelectionSecondaryOnly()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_CODING_HOST);
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_REQUIRE_S2));

	BtAppCfg_t cfg = MakePeripheralCfg(
		"This name is long enough to require extended advertising data");
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM_V2);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		// Only the channel that was asked for moves to the coded PHY.
		CHECK(ap->Param[kExtAdvPrimPhy] == BTADV_EXTADV_PHY_1M);
		CHECK(ap->Param[kExtAdvPrimPhyOptions] == BT_HCI_ADV_PHY_OPT_NONE);
		CHECK(ap->Param[kExtAdvSecPhy] == BTADV_EXTADV_PHY_CODED);
		CHECK(ap->Param[kExtAdvSecPhyOptions] ==
			BT_HCI_ADV_PHY_OPT_REQUIRE_S2);
	}

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_NONE));
}

void TestAdvertisingCodingSelectionWithoutFeature()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_EXT_CODING_NO_FEATURE);
	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_REQUIRE_S8,
		BT_HCI_ADV_PHY_OPT_REQUIRE_S8));

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	// A controller without the feature answers any non zero option with
	// Unsupported Feature or Parameter Value, so the request is dropped and
	// the v1 command goes out unchanged. 7.8.115 answers a claim whose
	// controller side is missing the same way, so it is not asked either.
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_HOST_FEATURE) == nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM_V2) == nullptr);
	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->ParamLen == 25);
		CHECK(ap->Param[kExtAdvPrimPhy] == BTADV_EXTADV_PHY_1M);
	}

	CHECK(BtAdvCodingSelectionSet(BT_HCI_ADV_PHY_OPT_NONE,
		BT_HCI_ADV_PHY_OPT_NONE));
}

void TestExtendedAdvertisingDataFragmentation()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	char name[251];
	std::memset(name, 'N', sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';

	uint8_t manData[250];
	for (size_t i = 0; i < sizeof(manData); i++)
	{
		manData[i] = static_cast<uint8_t>(i);
	}

	BtAppCfg_t cfg = MakePeripheralCfg(name);
	cfg.pAdvManData = manData;
	cfg.AdvManDataLen = sizeof(manData);
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *fragment[3] = {};
	int fragmentCount = 0;
	for (int i = 0; i < s_CmdCount; i++)
	{
		if (s_Cmds[i].OpCode == BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA &&
			fragmentCount < 3)
		{
			fragment[fragmentCount++] = &s_Cmds[i];
		}
	}

	CHECK(fragmentCount == 3);
	if (fragmentCount == 3)
	{
		CHECK(fragment[0]->Param[1] == 0x01); // First
		CHECK(fragment[1]->Param[1] == 0x00); // Intermediate
		CHECK(fragment[2]->Param[1] == 0x02); // Last
		CHECK(fragment[0]->Param[2] == 0x01);
		CHECK(fragment[1]->Param[2] == 0x01);
		CHECK(fragment[2]->Param[2] == 0x01);
		CHECK(fragment[0]->Param[3] == 251);
		CHECK(fragment[1]->Param[3] == 251);
		CHECK(fragment[2]->Param[3] == 7);
		CHECK(fragment[0]->ParamLen == 255);
		CHECK(fragment[1]->ParamLen == 255);
		CHECK(fragment[2]->ParamLen == 11);
		CHECK((int)fragment[0]->Param[3] + fragment[1]->Param[3] +
			fragment[2]->Param[3] == 509);
	}
}

void TestAdvStopKeepsStateOnFailure()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	s_FailOpCode = BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE;
	s_FailEnable = 0;
	BtAppAdvStop();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	s_FailOpCode = 0;
	BtAppAdvStop();
	CHECK(g_BtAppData.State == BTAPP_STATE_IDLE);
}

void TestAdvManDataSetReportsRestartFailure()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	uint8_t data[4] = { 1, 2, 3, 4 };
	CHECK(BtAppAdvManDataSet(data, sizeof(data), nullptr, 0) == true);
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	s_FailOpCode = BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE;
	s_FailEnable = 1;
	CHECK(BtAppAdvManDataSet(data, sizeof(data), nullptr, 0) == false);
	CHECK(g_BtAppData.State == BTAPP_STATE_IDLE);
}

void TestAdvManDataSetRestartsAfterWriteFailure()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);
	ClearCommands();

	s_FailOpCode = BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA;
	uint8_t data[2] = { 0xA1, 0xB2 };
	CHECK(BtAppAdvManDataSet(data, sizeof(data), nullptr, 0) == false);
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	int disable = FindCmdIndex(BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE, 0);
	int failedWrite = FindCmdIndex(BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA);
	int enable = FindCmdIndex(BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE, 1);
	CHECK(disable >= 0);
	CHECK(failedWrite > disable);
	CHECK(enable > failedWrite);
}

void TestAdvManDataSetRollsBackAfterScanResponseFailure()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_LEGACY);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	const CapturedCmd *initial = FindCmd(BT_HCI_CMD_CTLR_SET_ADV_DATA);
	CHECK(initial != nullptr);
	CapturedCmd oldAdv = {};
	if (initial != nullptr)
	{
		oldAdv = *initial;
	}

	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);
	ClearCommands();

	s_FailOpCode = BT_HCI_CMD_CTLR_SET_SCAN_RESP_DATA;
	uint8_t advData[2] = { 0x11, 0x22 };
	uint8_t srData[2] = { 0x33, 0x44 };
	CHECK(BtAppAdvManDataSet(advData, sizeof(advData),
		srData, sizeof(srData)) == false);
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	int advWriteCount = 0;
	for (int i = 0; i < s_CmdCount; i++)
	{
		if (s_Cmds[i].OpCode == BT_HCI_CMD_CTLR_SET_ADV_DATA)
		{
			advWriteCount++;
		}
	}
	CHECK(advWriteCount == 2);

	const CapturedCmd *rollback = FindLastCmd(BT_HCI_CMD_CTLR_SET_ADV_DATA);
	CHECK(rollback != nullptr);
	if (rollback != nullptr && initial != nullptr)
	{
		CHECK(rollback->ParamLen == oldAdv.ParamLen);
		CHECK(std::memcmp(rollback->Param, oldAdv.Param,
			oldAdv.ParamLen) == 0);
	}
}

void TestLegacyDataUpdateOrder()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr, CAP_LEGACY);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);
	ClearCommands();

	uint8_t data[2] = { 0x11, 0x22 };
	CHECK(BtAppAdvManDataSet(data, sizeof(data), nullptr, 0) == true);
	int disable = FindCmdIndex(BT_HCI_CMD_CTLR_SET_ADV_ENABLE, 0);
	int write = FindCmdIndex(BT_HCI_CMD_CTLR_SET_ADV_DATA);
	int enable = FindCmdIndex(BT_HCI_CMD_CTLR_SET_ADV_ENABLE, 1);
	CHECK(disable >= 0);
	CHECK(write > disable);
	CHECK(enable > write);
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);
}

void TestManDataCompanyIdLittleEndian()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	g_BtAppData.AppDevice.VendorId = 0x1234;

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
	ClearCommands();

	uint8_t payload[3] = { 0xAA, 0xBB, 0xCC };
	CHECK(BtAppAdvManDataSet(payload, sizeof(payload), nullptr, 0) == true);

	const CapturedCmd *ad = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA);
	CHECK(ad != nullptr);
	uint8_t len = 0;
	const uint8_t *p = FindExtAdvDataRecord(ad,
		BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, &len);
	CHECK(p != nullptr);
	if (p != nullptr)
	{
		CHECK(len == 5);
		CHECK(p[0] == 0x34);
		CHECK(p[1] == 0x12);
		CHECK(p[2] == 0xAA);
	}
}

} // namespace

extern "C" {

BtAppData_t g_BtAppData;

void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	*pType = s_LocalType;
	std::memcpy(pAddr, s_LocalAddr, 6);
}

} // extern "C"

int main()
{
	TestManDataCompanyIdLittleEndian();
	TestAdvIntervalFractionalMilliseconds();
	TestAdvIntervalBeyond16Bits();
	TestAdvIntervalNearExtendedMaximum();
	TestAdvIntervalPastExtendedMaximumRefused();
	TestAdvPublicIdentity();
	TestAdvRandomIdentity();
	TestAdvInvalidRandomIdentity();
	TestAdvDegenerateRandomIdentity();
	TestLegacyCommandSelection();
	TestLegacyMissingCommandRefused();
	TestExtendedPayloadNeedsExtendedCommands();
	TestExtendedLimitFallsBackToLegacy();
	TestExtendedLimitRejectsExtendedPayload();
	TestZeroAdvertisingSetsFallsBackToLegacy();
	TestExtendedSecondaryPhySelection();
	TestAdvertisingCodingSelectionArguments();
	TestAdvertisingCodingSelectionNeedsTheHostBit();
	TestAdvertisingCodingSelectionIgnoredOnLegacySet();
	TestAdvertisingCodingSelectionApplied();
	TestAdvertisingCodingSelectionSecondaryOnly();
	TestAdvertisingCodingSelectionWithoutFeature();
	TestExtendedAdvertisingDataFragmentation();
	TestAdvStopKeepsStateOnFailure();
	TestAdvManDataSetReportsRestartFailure();
	TestAdvManDataSetRestartsAfterWriteFailure();
	TestAdvManDataSetRollsBackAfterScanResponseFailure();
	TestLegacyDataUpdateOrder();

	if (s_Failures != 0)
	{
		std::printf("Adv HCI host tests: %d failure(s), %d checks\n",
			s_Failures, s_Checks);
		return 1;
	}

	std::printf("Adv HCI host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
