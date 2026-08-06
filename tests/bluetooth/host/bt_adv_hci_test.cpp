// Command-level coverage for the advertising HCI path: capability-based
// command selection, own-address validation, advertising state, and
// manufacturer company identifier byte order. BtHciCommand dispatches through
// the device Command pointer, so a capture device records every opcode and
// packed parameter block. No controller is involved.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_adv.h"
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
			CopyReturn(pRet, RetLen, data, 8);
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
			data[0] = s_CapabilityMode == CAP_EXT_LIMIT_SMALL ? 8 : 251;
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
constexpr size_t kExtAdvSecPhy = 22;
constexpr size_t kLegacyAdvOwnAddr = 5;

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
	TestAdvStopKeepsStateOnFailure();
	TestAdvManDataSetReportsRestartFailure();
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
