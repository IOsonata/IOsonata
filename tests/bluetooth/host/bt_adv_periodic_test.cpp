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
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_PERIODIC_ADV_CREATE_SYNC);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_PERIODIC_ADV_CREATE_SYNC_CANCEL);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_PERIODIC_ADV_TERMINATE_SYNC);
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

// --- Sync side ---

int s_EstCount, s_LostCount, s_ReportCount;
uint8_t s_EstStatus;
uint16_t s_EstHdl, s_LostHdl, s_ReportHdl;
size_t s_ReportLen;
uint8_t s_ReportData[BT_ADV_PERIODIC_DATA_MAX];
int8_t s_ReportRssi;

void ResetSyncCallbacks()
{
	s_EstCount = s_LostCount = s_ReportCount = 0;
	s_EstStatus = 0xFF;
	s_EstHdl = s_LostHdl = s_ReportHdl = 0xFFFF;
	s_ReportLen = 0;
	s_ReportRssi = 0;
}

BtAdvPeriodicSyncCfg_t MakeSyncCfg()
{
	BtAdvPeriodicSyncCfg_t cfg = {};
	cfg.AdvSid = 3;
	cfg.AdvAddrType = 1;
	for (int i = 0; i < 6; i++)
	{
		cfg.AdvAddr[i] = static_cast<uint8_t>(0xA0 + i);
	}
	cfg.Skip = 4;
	cfg.SyncTimeout = 0x0100;
	cfg.DisableReporting = false;
	return cfg;
}

void TestSyncCreateParameters()
{
	Reset();
	ResetSyncCallbacks();
	BtAdvPeriodicSyncCfg_t cfg = MakeSyncCfg();
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg));

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 14);
		CHECK(c->Param[0] == 0);
		CHECK(c->Param[1] == 3);
		CHECK(c->Param[2] == 1);
		CHECK(c->Param[3] == 0xA0 && c->Param[8] == 0xA5);
		CHECK(c->Param[9] == 4 && c->Param[10] == 0);
		CHECK(c->Param[11] == 0x00 && c->Param[12] == 0x01);
		CHECK(c->Param[13] == 0);
	}

	Reset();
	cfg.DisableReporting = true;
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg));
	c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Param[0] == 0x02);
	}
}

void TestSyncCreateValidation()
{
	Reset();
	BtAdvPeriodicSyncCfg_t cfg = MakeSyncCfg();

	cfg.AdvSid = BT_ADV_PERIODIC_SID_MAX + 1;
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg) == false);
	cfg = MakeSyncCfg();
	cfg.AdvAddrType = 2;
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg) == false);
	cfg = MakeSyncCfg();
	cfg.Skip = BT_ADV_PERIODIC_SKIP_MAX + 1;
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg) == false);
	cfg = MakeSyncCfg();
	cfg.SyncTimeout = BT_ADV_PERIODIC_SYNC_TIMEOUT_MIN - 1;
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg) == false);
	cfg = MakeSyncCfg();
	cfg.SyncTimeout = BT_ADV_PERIODIC_SYNC_TIMEOUT_MAX + 1;
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg) == false);
	CHECK(BtAdvPeriodicSyncCreate(nullptr, &cfg) == false);
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, nullptr) == false);
	CHECK(s_CmdCount == 0);

	// Synchronising is gated on the three sync commands, not the three the
	// advertiser side needs.
	Reset();
	s_Caps.SupportedCommands[BT_HCI_CAP_CMD_LE_PERIODIC_ADV_TERMINATE_SYNC >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_PERIODIC_ADV_TERMINATE_SYNC & 7U)));
	cfg = MakeSyncCfg();
	CHECK(BtAdvPeriodicSyncCreate(&s_Dev, &cfg) == false);
	CHECK(s_CmdCount == 0);
}

void TestSyncCancelAndTerminate()
{
	Reset();
	CHECK(BtAdvPeriodicSyncCancel(&s_Dev));
	const CapturedCmd *c =
		FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 0);
	}
	CHECK(BtAdvPeriodicSyncCancel(nullptr) == false);

	Reset();
	CHECK(BtAdvPeriodicSyncTerminate(&s_Dev, 0x0123));
	c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 2);
		CHECK(c->Param[0] == 0x23 && c->Param[1] == 0x01);
	}

	Reset();
	s_FailOpCode = BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC;
	CHECK(BtAdvPeriodicSyncTerminate(&s_Dev, 0x0123) == false);
}

void TestReportReassembly()
{
	Reset();
	ResetSyncCallbacks();
	uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	BtAdvPeriodicSyncEstablishedEvt(0, 0x0007, 3, 1, addr, 1, 0x0080);
	CHECK(s_EstCount == 1);
	CHECK(s_EstStatus == 0);
	CHECK(s_EstHdl == 0x0007);

	const uint8_t a[4] = { 0x11, 0x22, 0x33, 0x44 };
	const uint8_t b[3] = { 0x55, 0x66, 0x77 };
	BtAdvPeriodicReportFragment(0x0007, -10, -40,
		BT_ADV_PERIODIC_DATA_MORE, sizeof(a), a);
	CHECK(s_ReportCount == 0);
	BtAdvPeriodicReportFragment(0x0007, -10, -42,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(b), b);
	CHECK(s_ReportCount == 1);
	CHECK(s_ReportLen == 7);
	CHECK(s_ReportData[0] == 0x11 && s_ReportData[6] == 0x77);
	CHECK(s_ReportRssi == -42);

	// A report for a handle this module is not synced to is ignored.
	BtAdvPeriodicReportFragment(0x0009, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 1);
}

// Truncated means the rest is never coming, so what is held is a piece of a
// longer payload and must not be delivered as a whole one.
void TestTruncatedAndFailedReports()
{
	Reset();
	ResetSyncCallbacks();
	uint8_t addr[6] = {};
	BtAdvPeriodicSyncEstablishedEvt(0, 0x0007, 3, 1, addr, 1, 0x0080);

	const uint8_t a[4] = { 1, 2, 3, 4 };
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_MORE, sizeof(a), a);
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_TRUNCATED, sizeof(a), a);
	CHECK(s_ReportCount == 0);

	// The next payload starts clean rather than continuing the abandoned one.
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 1);
	CHECK(s_ReportLen == 4);

	// A failed receive has no payload and abandons what was accumulating.
	ResetSyncCallbacks();
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_MORE, sizeof(a), a);
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_RX_FAILED, 0, nullptr);
	CHECK(s_ReportCount == 0);
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 1);
	CHECK(s_ReportLen == 4);
}

// More than a whole payload can hold is dropped, not delivered as a prefix.
void TestReportOverflowDropped()
{
	Reset();
	ResetSyncCallbacks();
	uint8_t addr[6] = {};
	BtAdvPeriodicSyncEstablishedEvt(0, 0x0007, 3, 1, addr, 1, 0x0080);

	uint8_t chunk[248];
	std::memset(chunk, 0x5A, sizeof(chunk));
	size_t sent = 0;
	while (sent + sizeof(chunk) <= BT_ADV_PERIODIC_DATA_MAX)
	{
		BtAdvPeriodicReportFragment(0x0007, 0, 0,
			BT_ADV_PERIODIC_DATA_MORE, sizeof(chunk), chunk);
		sent += sizeof(chunk);
	}
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_MORE, sizeof(chunk), chunk);
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, 0, nullptr);
	CHECK(s_ReportCount == 0);

	// And the next payload is unaffected.
	const uint8_t a[2] = { 9, 9 };
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 1);
	CHECK(s_ReportLen == 2);
}

void TestSyncLostAndFailedEstablish()
{
	Reset();
	ResetSyncCallbacks();
	uint8_t addr[6] = {};
	BtAdvPeriodicSyncEstablishedEvt(0, 0x0007, 3, 1, addr, 1, 0x0080);
	BtAdvPeriodicSyncLostEvt(0x0007);
	CHECK(s_LostCount == 1);
	CHECK(s_LostHdl == 0x0007);

	// After the loss nothing is reassembled for that handle any more.
	const uint8_t a[2] = { 1, 2 };
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 0);

	// A failed establish reports the status and joins no train.
	ResetSyncCallbacks();
	BtAdvPeriodicSyncEstablishedEvt(0x3E, 0x0000, 3, 1, addr, 1, 0);
	CHECK(s_EstCount == 1);
	CHECK(s_EstStatus == 0x3E);
	BtAdvPeriodicReportFragment(0x0000, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 0);
}

// A terminate drops the state without waiting for a Sync Lost that never comes.
void TestTerminateDropsReassembly()
{
	Reset();
	ResetSyncCallbacks();
	uint8_t addr[6] = {};
	BtAdvPeriodicSyncEstablishedEvt(0, 0x0007, 3, 1, addr, 1, 0x0080);
	CHECK(BtAdvPeriodicSyncTerminate(&s_Dev, 0x0007));

	const uint8_t a[2] = { 1, 2 };
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 0);
}

} // namespace

// Strong overrides of the application facing callbacks.
void BtAdvPeriodicSyncEstablished(uint8_t Status, uint16_t SyncHdl,
	uint8_t, uint8_t, const uint8_t[6], uint8_t, uint16_t)
{
	s_EstCount++;
	s_EstStatus = Status;
	s_EstHdl = SyncHdl;
}

void BtAdvPeriodicSyncReport(uint16_t SyncHdl, int8_t, int8_t Rssi,
	size_t Len, const uint8_t *pData)
{
	s_ReportCount++;
	s_ReportHdl = SyncHdl;
	s_ReportRssi = Rssi;
	s_ReportLen = Len;
	if (Len > 0 && pData != nullptr && Len <= sizeof(s_ReportData))
	{
		std::memcpy(s_ReportData, pData, Len);
	}
}

void BtAdvPeriodicSyncLost(uint16_t SyncHdl)
{
	s_LostCount++;
	s_LostHdl = SyncHdl;
}

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
	TestSyncCreateParameters();
	TestSyncCreateValidation();
	TestSyncCancelAndTerminate();
	TestReportReassembly();
	TestTruncatedAndFailedReports();
	TestReportOverflowDropped();
	TestSyncLostAndFailedEstablish();
	TestTerminateDropsReassembly();

	if (s_Failures == 0)
	{
		std::printf("Periodic advertising host tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Periodic advertising host tests: FAIL (%d failures, %d checks)\n",
		s_Failures, s_Checks);
	return 1;
}
