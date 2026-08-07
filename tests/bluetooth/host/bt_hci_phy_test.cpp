// Command-level coverage for capability-aware LE PHY control.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci_cap.h"

static_assert(BT_HCI_CAP_CMD_LE_READ_PHY == 281U);
static_assert(BT_HCI_CAP_CMD_LE_SET_DEFAULT_PHY == 282U);
static_assert(BT_HCI_CAP_CMD_LE_SET_PHY == 283U);

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
	uint8_t Param[16];
	uint8_t ParamLen;
};

CapturedCmd s_PhyCmd;
int s_PhyCmdCount = 0;
int s_DiscoveryCount = 0;
bool s_DiscoveryOk = true;
uint8_t s_PhyCommandStatus = 0;
uint8_t s_ReadPhyResult[4] = {};
uint8_t s_SupportedCommands[64] = {};
uint8_t s_LeFeatures[8] = {};

void CopyReturn(void *pRet, uint8_t RetLen, const uint8_t *pData,
	size_t DataLen)
{
	if (pRet != nullptr && RetLen != 0)
	{
		size_t len = RetLen < DataLen ? RetLen : DataLen;
		std::memcpy(pRet, pData, len);
	}
}

void SetCommand(uint16_t CommandBit)
{
	s_SupportedCommands[CommandBit >> 3] |=
		static_cast<uint8_t>(1U << (CommandBit & 7U));
}

void ClearCommand(uint16_t CommandBit)
{
	s_SupportedCommands[CommandBit >> 3] &=
		static_cast<uint8_t>(~(1U << (CommandBit & 7U)));
}

void SetFeature(uint8_t FeatureBit)
{
	s_LeFeatures[FeatureBit >> 3] |=
		static_cast<uint8_t>(1U << (FeatureBit & 7U));
}

void ClearFeature(uint8_t FeatureBit)
{
	s_LeFeatures[FeatureBit >> 3] &=
		static_cast<uint8_t>(~(1U << (FeatureBit & 7U)));
}

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode,
	const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	uint8_t data[64] = {};

	switch (OpCode)
	{
		case BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO:
		{
			s_DiscoveryCount++;
			if (!s_DiscoveryOk)
			{
				return 1;
			}
			const uint8_t value[8] = { 0x0D, 0, 0, 0x0D, 0, 0, 0, 0 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			return 0;
		}

		case BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS:
			CopyReturn(pRet, RetLen, s_SupportedCommands,
				sizeof(s_SupportedCommands));
			return 0;

		case BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES:
			CopyReturn(pRet, RetLen, s_LeFeatures, sizeof(s_LeFeatures));
			return 0;

		case BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES:
			CopyReturn(pRet, RetLen, data, 8);
			return 0;

		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT:
		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE:
		case BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN:
		case BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE:
		case BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN:
		case BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS:
		case BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE:
			return 1;

		case BT_HCI_CMD_CTLR_READ_PHY:
		case BT_HCI_CMD_CTLR_SET_DEFAULT_PHY:
		case BT_HCI_CMD_CTLR_SET_PHY:
			s_PhyCmdCount++;
			s_PhyCmd.OpCode = OpCode;
			s_PhyCmd.ParamLen = ParamLen;
			std::memset(s_PhyCmd.Param, 0, sizeof(s_PhyCmd.Param));
			if (pParam != nullptr && ParamLen <= sizeof(s_PhyCmd.Param))
			{
				std::memcpy(s_PhyCmd.Param, pParam, ParamLen);
			}
			if (OpCode == BT_HCI_CMD_CTLR_READ_PHY &&
				pRet != nullptr && RetLen > 0)
			{
				CopyReturn(pRet, RetLen, s_ReadPhyResult,
					sizeof(s_ReadPhyResult));
			}
			return s_PhyCommandStatus;

		default:
			return 1;
	}
}

BtHciDevice_t s_Dev;

void Setup()
{
	std::memset(&s_PhyCmd, 0, sizeof(s_PhyCmd));
	s_PhyCmdCount = 0;
	s_DiscoveryCount = 0;
	s_DiscoveryOk = true;
	s_PhyCommandStatus = 0;
	std::memset(s_ReadPhyResult, 0, sizeof(s_ReadPhyResult));
	std::memset(s_SupportedCommands, 0, sizeof(s_SupportedCommands));
	std::memset(s_LeFeatures, 0, sizeof(s_LeFeatures));
	SetCommand(BT_HCI_CAP_CMD_LE_READ_PHY);
	SetCommand(BT_HCI_CAP_CMD_LE_SET_DEFAULT_PHY);
	SetCommand(BT_HCI_CAP_CMD_LE_SET_PHY);
	SetFeature(BT_HCI_CAP_LE_FEATURE_PHY_2M);
	SetFeature(BT_HCI_CAP_LE_FEATURE_CODED_PHY);
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = CaptureCommand;
}

void SetReadPhyResult(uint16_t ConnHdl, uint8_t TxPhy, uint8_t RxPhy)
{
	s_ReadPhyResult[0] = static_cast<uint8_t>(ConnHdl & 0xFFU);
	s_ReadPhyResult[1] = static_cast<uint8_t>(ConnHdl >> 8);
	s_ReadPhyResult[2] = TxPhy;
	s_ReadPhyResult[3] = RxPhy;
}

void TestInvalidArguments()
{
	Setup();
	uint8_t tx = 0;
	uint8_t rx = 0;

	CHECK(BtHciReadPhy(nullptr, 1, &tx, &rx) == false);
	CHECK(BtHciReadPhy(&s_Dev, 1, nullptr, &rx) == false);
	CHECK(BtHciReadPhy(&s_Dev, 1, &tx, nullptr) == false);
	CHECK(BtHciReadPhy(&s_Dev, BT_HCI_LE_CONN_HANDLE_MAX + 1U,
		&tx, &rx) == false);
	CHECK(BtHciSetDefaultPhy(nullptr, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M) == false);
	CHECK(BtHciSetPhy(&s_Dev, BT_HCI_LE_CONN_HANDLE_MAX + 1U,
		BT_HCI_PHY_1M, BT_HCI_PHY_1M, BT_HCI_PHY_CODED_ANY) == false);
	CHECK(s_DiscoveryCount == 0);
	CHECK(s_PhyCmdCount == 0);
}

void TestReadPhy()
{
	Setup();
	SetReadPhyResult(0x0042, 0x01, 0x02);
	uint8_t tx = 0;
	uint8_t rx = 0;

	CHECK(BtHciReadPhy(&s_Dev, 0x0042, &tx, &rx));
	CHECK(s_DiscoveryCount == 1);
	CHECK(s_PhyCmdCount == 1);
	CHECK(s_PhyCmd.OpCode == BT_HCI_CMD_CTLR_READ_PHY);
	CHECK(s_PhyCmd.ParamLen == 2);
	CHECK(s_PhyCmd.Param[0] == 0x42);
	CHECK(s_PhyCmd.Param[1] == 0x00);
	CHECK(tx == BT_HCI_PHY_1M);
	CHECK(rx == BT_HCI_PHY_2M);

	Setup();
	SetReadPhyResult(0x0123, 0x03, 0x03);
	CHECK(BtHciReadPhy(&s_Dev, 0x0123, &tx, &rx));
	CHECK(tx == BT_HCI_PHY_CODED);
	CHECK(rx == BT_HCI_PHY_CODED);
}

void TestReadPhyValidation()
{
	Setup();
	ClearCommand(BT_HCI_CAP_CMD_LE_READ_PHY);
	uint8_t tx = 0;
	uint8_t rx = 0;
	CHECK(BtHciReadPhy(&s_Dev, 1, &tx, &rx) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	SetReadPhyResult(2, 0x01, 0x01);
	CHECK(BtHciReadPhy(&s_Dev, 1, &tx, &rx) == false);
	CHECK(s_PhyCmdCount == 1);

	Setup();
	SetReadPhyResult(1, 0x04, 0x01);
	CHECK(BtHciReadPhy(&s_Dev, 1, &tx, &rx) == false);

	Setup();
	ClearFeature(BT_HCI_CAP_LE_FEATURE_PHY_2M);
	SetReadPhyResult(1, 0x02, 0x01);
	CHECK(BtHciReadPhy(&s_Dev, 1, &tx, &rx) == false);

	Setup();
	SetReadPhyResult(1, 0x01, 0x01);
	s_PhyCommandStatus = 0x0C;
	CHECK(BtHciReadPhy(&s_Dev, 1, &tx, &rx) == false);
}

void TestSetDefaultPhy()
{
	Setup();
	CHECK(BtHciSetDefaultPhy(&s_Dev, BT_HCI_PHY_1M,
		BT_HCI_PHY_CODED));
	CHECK(s_PhyCmdCount == 1);
	CHECK(s_PhyCmd.OpCode == BT_HCI_CMD_CTLR_SET_DEFAULT_PHY);
	CHECK(s_PhyCmd.ParamLen == 3);
	CHECK(s_PhyCmd.Param[0] == 0x00);
	CHECK(s_PhyCmd.Param[1] == BT_HCI_PHY_1M);
	CHECK(s_PhyCmd.Param[2] == BT_HCI_PHY_CODED);

	Setup();
	CHECK(BtHciSetDefaultPhy(&s_Dev, 0, 0));
	CHECK(s_PhyCmd.Param[0] == 0x03);
	CHECK(s_PhyCmd.Param[1] == 0);
	CHECK(s_PhyCmd.Param[2] == 0);
}

void TestSetDefaultPhyValidation()
{
	Setup();
	ClearFeature(BT_HCI_CAP_LE_FEATURE_PHY_2M);
	CHECK(BtHciSetDefaultPhy(&s_Dev, BT_HCI_PHY_2M,
		BT_HCI_PHY_1M) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	ClearFeature(BT_HCI_CAP_LE_FEATURE_CODED_PHY);
	CHECK(BtHciSetDefaultPhy(&s_Dev, BT_HCI_PHY_CODED,
		BT_HCI_PHY_1M) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	CHECK(BtHciSetDefaultPhy(&s_Dev, 0x08, BT_HCI_PHY_1M) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_DEFAULT_PHY);
	CHECK(BtHciSetDefaultPhy(&s_Dev, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M) == false);
	CHECK(s_PhyCmdCount == 0);
}

void TestSetPhy()
{
	Setup();
	CHECK(BtHciSetPhy(&s_Dev, 0x0042, BT_HCI_PHY_2M,
		BT_HCI_PHY_CODED, BT_HCI_PHY_CODED_S2));
	CHECK(s_PhyCmdCount == 1);
	CHECK(s_PhyCmd.OpCode == BT_HCI_CMD_CTLR_SET_PHY);
	CHECK(s_PhyCmd.ParamLen == 7);
	CHECK(s_PhyCmd.Param[0] == 0x42);
	CHECK(s_PhyCmd.Param[1] == 0x00);
	CHECK(s_PhyCmd.Param[2] == 0x00);
	CHECK(s_PhyCmd.Param[3] == BT_HCI_PHY_2M);
	CHECK(s_PhyCmd.Param[4] == BT_HCI_PHY_CODED);
	CHECK(s_PhyCmd.Param[5] == BT_HCI_PHY_CODED_S2);
	CHECK(s_PhyCmd.Param[6] == 0x00);

	Setup();
	CHECK(BtHciSetPhy(&s_Dev, 0x0001, 0, 0,
		BT_HCI_PHY_CODED_ANY));
	CHECK(s_PhyCmd.Param[2] == 0x03);

	// PHY_Options remains valid when Coded PHY is not preferred. The
	// controller may override the preferred PHY and use the coding request.
	Setup();
	CHECK(BtHciSetPhy(&s_Dev, 0x0001, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M, BT_HCI_PHY_CODED_S8));
	CHECK(s_PhyCmd.Param[5] == BT_HCI_PHY_CODED_S8);
}

void TestSetPhyValidation()
{
	Setup();
	CHECK(BtHciSetPhy(&s_Dev, 1, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M, 3) == false);
	CHECK(s_DiscoveryCount == 0);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	ClearFeature(BT_HCI_CAP_LE_FEATURE_PHY_2M);
	CHECK(BtHciSetPhy(&s_Dev, 1, BT_HCI_PHY_2M,
		BT_HCI_PHY_1M, BT_HCI_PHY_CODED_ANY) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	ClearFeature(BT_HCI_CAP_LE_FEATURE_CODED_PHY);
	CHECK(BtHciSetPhy(&s_Dev, 1, BT_HCI_PHY_1M,
		BT_HCI_PHY_CODED, BT_HCI_PHY_CODED_ANY) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_PHY);
	CHECK(BtHciSetPhy(&s_Dev, 1, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M, BT_HCI_PHY_CODED_ANY) == false);
	CHECK(s_PhyCmdCount == 0);

	Setup();
	s_PhyCommandStatus = 0x0C;
	CHECK(BtHciSetPhy(&s_Dev, 1, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M, BT_HCI_PHY_CODED_ANY) == false);
	CHECK(s_PhyCmdCount == 1);
}

void TestDiscoveryFailure()
{
	Setup();
	s_DiscoveryOk = false;
	CHECK(BtHciSetDefaultPhy(&s_Dev, BT_HCI_PHY_1M,
		BT_HCI_PHY_1M) == false);
	CHECK(s_DiscoveryCount == 1);
	CHECK(s_PhyCmdCount == 0);
}

} // namespace

int main()
{
	TestInvalidArguments();
	TestReadPhy();
	TestReadPhyValidation();
	TestSetDefaultPhy();
	TestSetDefaultPhyValidation();
	TestSetPhy();
	TestSetPhyValidation();
	TestDiscoveryFailure();

	if (s_Failures == 0)
	{
		std::printf("PASS: %d PHY checks\n", s_Checks);
	}
	else
	{
		std::printf("FAIL: %d of %d PHY checks failed\n",
			s_Failures, s_Checks);
	}
	return s_Failures == 0 ? 0 : 1;
}
