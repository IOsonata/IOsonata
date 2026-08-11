/**-------------------------------------------------------------------------
@file	bt_adv_periodic_test.cpp

@brief	Periodic advertising over standard HCI.

Core Vol 4 Part E 7.8.61 to 7.8.63. A train rides on an extended advertising
set that must not be scannable, connectable, legacy or anonymous, and does not
go out until that set is advertising.

@author	Hoang Nguyen Hoan
@date	Aug. 11, 2026

@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_adv.h"

namespace {

int s_Checks;
int s_Failures;

#define CHECK(expr) do { \
	s_Checks++; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		s_Failures++; \
	} \
} while (0)

struct CapturedCmd {
	uint16_t OpCode;
	uint8_t  Param[264];
	uint16_t ParamLen;
};

CapturedCmd s_Cmds[64];
int s_CmdCount;
uint16_t s_FailOpCode;
int s_FailAfter;
BtHciCapabilities_t s_Caps;
bool s_CapsValid = true;

uint8_t TestCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
	uint8_t ParamLen, void *, uint8_t)
{
	if (s_CmdCount < (int)(sizeof(s_Cmds) / sizeof(s_Cmds[0])))
	{
		CapturedCmd *c = &s_Cmds[s_CmdCount++];
		c->OpCode = OpCode;
		c->ParamLen = ParamLen;
		std::memset(c->Param, 0, sizeof(c->Param));
		if (pParam != nullptr && ParamLen > 0)
		{
			std::memcpy(c->Param, pParam, ParamLen);
		}
	}

	if (OpCode == s_FailOpCode)
	{
		if (s_FailAfter <= 0)
		{
			return 0x0C;
		}
		s_FailAfter--;
	}

	return 0;
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

int CountCmd(uint16_t OpCode)
{
	int n = 0;
	for (int i = 0; i < s_CmdCount; i++)
	{
		if (s_Cmds[i].OpCode == OpCode)
		{
			n++;
		}
	}
	return n;
}

void SetCommandBit(uint8_t commands[64], uint16_t bit)
{
	commands[bit >> 3] |= static_cast<uint8_t>(1U << (bit & 7U));
}

void SetFeatureBit(uint8_t features[8], uint8_t bit)
{
	features[bit >> 3] |= static_cast<uint8_t>(1U << (bit & 7U));
}

void ResetCaps(uint8_t AdvSetCount = 4)
{
	std::memset(&s_Caps, 0, sizeof(s_Caps));
	s_Caps.Valid = BT_HCI_CAP_VALID_COMMANDS | BT_HCI_CAP_VALID_LE_FEATURES |
		BT_HCI_CAP_VALID_ADV_SET_COUNT;
	s_Caps.AdvSetCount = AdvSetCount;
	SetFeatureBit(s_Caps.LeFeatures, BT_HCI_CAP_LE_FEATURE_EXT_ADV);
	SetFeatureBit(s_Caps.LeFeatures, BT_HCI_CAP_LE_FEATURE_PERIODIC_ADV);
	SetFeatureBit(s_Caps.LeFeatures, BT_HCI_CAP_LE_FEATURE_PHY_2M);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_PARAMETERS);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_DATA);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_ENABLE);
	s_CapsValid = true;
}

BtHciDevice_t s_Dev;

void Reset(uint8_t AdvSetCount = 4)
{
	s_CmdCount = 0;
	s_FailOpCode = 0;
	s_FailAfter = 0;
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = TestCommand;
	ResetCaps(AdvSetCount);
}

BtAdvPeriodicCfg_t MakeCfg()
{
	BtAdvPeriodicCfg_t cfg = {};
	cfg.IntervalMin = 0x0060;
	cfg.IntervalMax = 0x0080;
	cfg.OwnAddrType = BTADDR_TYPE_RAND;
	cfg.Sid = 2;
	cfg.IncludeTxPower = false;
	return cfg;
}

// The set the train rides on must be non connectable, non scannable, non
// legacy and non anonymous, or 7.8.61 refuses the parameters.
void TestExtendedSetProperties()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));

	const CapturedCmd *e = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(e != nullptr);
	if (e != nullptr)
	{
		CHECK(e->ParamLen == 25);
		CHECK(e->Param[0] == BT_ADV_PERIODIC_ADV_HANDLE);
		// Event properties, octets 1 and 2, every bit clear.
		CHECK(e->Param[1] == 0);
		CHECK(e->Param[2] == 0);
		CHECK(e->Param[20] == BTADV_EXTADV_PHY_1M);
		CHECK(e->Param[22] == BTADV_EXTADV_PHY_2M);
		CHECK(e->Param[23] == 2);
	}

	const CapturedCmd *p = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM);
	CHECK(p != nullptr);
	if (p != nullptr)
	{
		CHECK(p->ParamLen == 7);
		CHECK(p->Param[0] == BT_ADV_PERIODIC_ADV_HANDLE);
		CHECK(p->Param[1] == 0x60 && p->Param[2] == 0x00);
		CHECK(p->Param[3] == 0x80 && p->Param[4] == 0x00);
		CHECK(p->Param[5] == 0 && p->Param[6] == 0);
	}
}

void TestTxPowerProperty()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	cfg.IncludeTxPower = true;
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));

	const CapturedCmd *p = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM);
	CHECK(p != nullptr);
	if (p != nullptr)
	{
		CHECK(p->Param[5] == (BT_ADV_PERIODIC_PROP_INCLUDE_TXPOWER & 0xFF));
		CHECK(p->Param[6] == 0);
	}
}

void TestIntervalValidation()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();

	cfg.IntervalMin = BT_ADV_PERIODIC_INTERVAL_MIN - 1;
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	cfg = MakeCfg();
	cfg.IntervalMin = 0x0100;
	cfg.IntervalMax = 0x00FF;
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	CHECK(BtAdvPeriodicInit(nullptr, &cfg) == false);
	CHECK(BtAdvPeriodicInit(&s_Dev, nullptr) == false);
}

// A controller with one advertising set has none to spare for the train.
void TestSecondAdvertisingSetRequired()
{
	Reset(1);
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset(2);
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
}

void TestCapabilityGating()
{
	// No LE Periodic Advertising feature.
	Reset();
	s_Caps.LeFeatures[BT_HCI_CAP_LE_FEATURE_PERIODIC_ADV >> 3] &=
		static_cast<uint8_t>(~(1U << (BT_HCI_CAP_LE_FEATURE_PERIODIC_ADV & 7U)));
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(s_CmdCount == 0);

	// Feature present but the enable command missing.
	Reset();
	s_Caps.SupportedCommands[BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_ENABLE >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_ENABLE & 7U)));
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(s_CmdCount == 0);

	// No capability record at all.
	Reset();
	s_CapsValid = false;
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
}

void TestDataFragmentation()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
	s_CmdCount = 0;

	// Two full fragments and a remainder.
	uint8_t data[BT_ADV_PERIODIC_FRAGMENT_MAX * 2 + 10];
	for (size_t i = 0; i < sizeof(data); i++)
	{
		data[i] = static_cast<uint8_t>(i);
	}
	CHECK(BtAdvPeriodicDataSet(data, sizeof(data)));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 3);
	CHECK(s_Cmds[0].Param[1] == 0x01);	// First
	CHECK(s_Cmds[1].Param[1] == 0x00);	// Intermediate
	CHECK(s_Cmds[2].Param[1] == 0x02);	// Last
	CHECK(s_Cmds[0].Param[2] == BT_ADV_PERIODIC_FRAGMENT_MAX);
	CHECK(s_Cmds[2].Param[2] == 10);
	// Only the octets in use are sent, header is handle, operation, length.
	CHECK(s_Cmds[2].ParamLen == 3 + 10);

	uint8_t back[sizeof(data)] = {};
	CHECK(BtAdvPeriodicDataGet(back, sizeof(back)) == sizeof(data));
	CHECK(std::memcmp(back, data, sizeof(data)) == 0);

	// A payload that fits in one command is sent Complete, not First and Last.
	Reset();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
	s_CmdCount = 0;
	CHECK(BtAdvPeriodicDataSet(data, 20));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 1);
	CHECK(s_Cmds[0].Param[1] == 0x03);	// Complete
}

// A fragment run that fails part way leaves the train holding partial data,
// which the controller refuses to advertise. The retained length goes to zero
// so nothing later reports data that is not really there.
void TestPartialDataRunClearsLength()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));

	uint8_t data[64];
	std::memset(data, 0x5A, sizeof(data));
	CHECK(BtAdvPeriodicDataSet(data, sizeof(data)));
	CHECK(BtAdvPeriodicDataGet(nullptr, 0) == sizeof(data));

	// Fail the second fragment of a three fragment run.
	s_CmdCount = 0;
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA;
	s_FailAfter = 1;
	uint8_t big[BT_ADV_PERIODIC_FRAGMENT_MAX * 2 + 4];
	std::memset(big, 0xA5, sizeof(big));
	CHECK(BtAdvPeriodicDataSet(big, sizeof(big)) == false);
	CHECK(BtAdvPeriodicDataGet(nullptr, 0) == 0);
}

void TestDataArguments()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));

	uint8_t data[4] = {};
	CHECK(BtAdvPeriodicDataSet(nullptr, 4) == false);
	CHECK(BtAdvPeriodicDataSet(data, BT_ADV_PERIODIC_DATA_MAX + 1) == false);
	// Zero length is a legitimate way to clear the train's data.
	CHECK(BtAdvPeriodicDataSet(nullptr, 0));
}

// Both the train and the set it rides on have to be enabled, in that order.
void TestStartEnablesBoth()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
	s_CmdCount = 0;

	CHECK(BtAdvPeriodicStart());
	CHECK(s_CmdCount == 2);
	CHECK(s_Cmds[0].OpCode == BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE);
	CHECK(s_Cmds[0].Param[0] == 1);
	CHECK(s_Cmds[0].Param[1] == BT_ADV_PERIODIC_ADV_HANDLE);
	CHECK(s_Cmds[1].OpCode == BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE);
	CHECK(s_Cmds[1].Param[0] == 1);
	CHECK(s_Cmds[1].Param[1] == 1);
	CHECK(s_Cmds[1].Param[2] == BT_ADV_PERIODIC_ADV_HANDLE);
	CHECK(BtAdvPeriodicIsRunning());
}

// If the set refuses, the train must not be left running with nothing
// advertising it.
void TestStartRollsBackOnSetFailure()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
	s_CmdCount = 0;
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE;

	CHECK(BtAdvPeriodicStart() == false);
	CHECK(BtAdvPeriodicIsRunning() == false);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE) == 2);
	// The last periodic enable turns it back off.
	CHECK(s_Cmds[s_CmdCount - 1].OpCode ==
		BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE);
	CHECK(s_Cmds[s_CmdCount - 1].Param[0] == 0);
}

// Disabling the set does not stop a running train, so stop does both, and the
// set is stopped even when the train refuses.
void TestStopDisablesBoth()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
	CHECK(BtAdvPeriodicStart());
	s_CmdCount = 0;

	CHECK(BtAdvPeriodicStop());
	CHECK(s_CmdCount == 2);
	CHECK(s_Cmds[0].OpCode == BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE);
	CHECK(s_Cmds[0].Param[0] == 0);
	CHECK(s_Cmds[1].OpCode == BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE);
	CHECK(s_Cmds[1].Param[0] == 0);
	CHECK(BtAdvPeriodicIsRunning() == false);

	Reset();
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg));
	CHECK(BtAdvPeriodicStart());
	s_CmdCount = 0;
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE;
	CHECK(BtAdvPeriodicStop() == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE) != nullptr);
}

// Nothing works before a successful init, and a failed init leaves nothing
// half configured.
void TestOperationsBeforeInit()
{
	Reset();
	uint8_t data[4] = {};
	BtAdvPeriodicCfg_t cfg = MakeCfg();

	Reset(1);
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(BtAdvPeriodicDataSet(data, sizeof(data)) == false);
	CHECK(BtAdvPeriodicStart() == false);
	CHECK(BtAdvPeriodicIsRunning() == false);

	// The periodic parameters failing leaves init refused as a whole.
	Reset();
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM;
	CHECK(BtAdvPeriodicInit(&s_Dev, &cfg) == false);
	CHECK(BtAdvPeriodicStart() == false);
}

} // namespace

// The module reads capabilities through this. The real one matches the device
// against the active controller; here the test owns the record.
extern "C" const BtHciCapabilities_t *BtHciCapabilitiesForDeviceGet(
	const BtHciDevice_t *pDev)
{
	(void)pDev;
	return s_CapsValid ? &s_Caps : nullptr;
}

int main()
{
	TestExtendedSetProperties();
	TestTxPowerProperty();
	TestIntervalValidation();
	TestSecondAdvertisingSetRequired();
	TestCapabilityGating();
	TestDataFragmentation();
	TestPartialDataRunClearsLength();
	TestDataArguments();
	TestStartEnablesBoth();
	TestStartRollsBackOnSetFailure();
	TestStopDisablesBoth();
	TestOperationsBeforeInit();

	if (s_Failures == 0)
	{
		std::printf("Periodic advertising host tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Periodic advertising host tests: FAIL (%d failures, %d checks)\n",
		s_Failures, s_Checks);
	return 1;
}
