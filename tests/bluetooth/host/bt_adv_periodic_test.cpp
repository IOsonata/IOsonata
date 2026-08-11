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
#include "bluetooth/bt_app.h"

// The record bt_adv_hci reads the local HCI device out of, the same
// way bt_adv_hci and bt_gap_hci do. Defined out here rather than in the
// anonymous namespace below, which would give it internal linkage and leave
// the module under test with an undefined reference. Only pHciDev is used, so
// the rest stays zeroed.
BtAppData_t g_BtAppData;

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
	SetFeatureBit(s_Caps.LeFeatures, BT_HCI_CAP_LE_FEATURE_PAWR_ADVERTISER);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_PARAMETERS_V2);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_SUBEVENT_DATA);
	SetFeatureBit(s_Caps.LeFeatures, BT_HCI_CAP_LE_FEATURE_PAWR_SCANNER);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_SYNC_SUBEVENT);
	SetCommandBit(s_Caps.SupportedCommands, BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_RESPONSE_DATA);
	s_CapsValid = true;
}

BtHciDevice_t s_Dev;

// bt_adv_hci reads the local HCI device out of the shared app record,
// the same way bt_adv_hci and bt_gap_hci do. Supplying the record here keeps
// the test linking two files. A port with no HCI device leaves the field NULL,
// which is what the null device cases below model.

void Reset(uint8_t AdvSetCount = 4)
{
	s_CmdCount = 0;
	s_FailOpCode = 0;
	s_FailAfter = 0;
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
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
	CHECK(BtAdvPeriodicInit(&cfg));

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
	CHECK(BtAdvPeriodicInit(&cfg));

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
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	cfg = MakeCfg();
	cfg.IntervalMin = 0x0100;
	cfg.IntervalMax = 0x00FF;
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	g_BtAppData.AppDevice.pHciDev = nullptr;
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	CHECK(BtAdvPeriodicInit(nullptr) == false);
}

// A controller with one advertising set has none to spare for the train.
void TestSecondAdvertisingSetRequired()
{
	Reset(1);
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset(2);
	CHECK(BtAdvPeriodicInit(&cfg));
}

void TestCapabilityGating()
{
	// No LE Periodic Advertising feature.
	Reset();
	s_Caps.LeFeatures[BT_HCI_CAP_LE_FEATURE_PERIODIC_ADV >> 3] &=
		static_cast<uint8_t>(~(1U << (BT_HCI_CAP_LE_FEATURE_PERIODIC_ADV & 7U)));
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	// Feature present but the enable command missing.
	Reset();
	s_Caps.SupportedCommands[BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_ENABLE >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_ENABLE & 7U)));
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	// No capability record at all.
	Reset();
	s_CapsValid = false;
	CHECK(BtAdvPeriodicInit(&cfg) == false);
}

void TestDataFragmentation()
{
	Reset();
	BtAdvPeriodicCfg_t cfg = MakeCfg();
	CHECK(BtAdvPeriodicInit(&cfg));
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
	CHECK(BtAdvPeriodicInit(&cfg));
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
	CHECK(BtAdvPeriodicInit(&cfg));

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
	CHECK(BtAdvPeriodicInit(&cfg));

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
	CHECK(BtAdvPeriodicInit(&cfg));
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
	CHECK(BtAdvPeriodicInit(&cfg));
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
	CHECK(BtAdvPeriodicInit(&cfg));
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
	CHECK(BtAdvPeriodicInit(&cfg));
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
	CHECK(BtAdvPeriodicInit(&cfg) == false);
	CHECK(BtAdvPeriodicDataSet(data, sizeof(data)) == false);
	CHECK(BtAdvPeriodicStart() == false);
	CHECK(BtAdvPeriodicIsRunning() == false);

	// The periodic parameters failing leaves init refused as a whole.
	Reset();
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM;
	CHECK(BtAdvPeriodicInit(&cfg) == false);
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
	CHECK(BtAdvPeriodicSyncCreate(&cfg));

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
	CHECK(BtAdvPeriodicSyncCreate(&cfg));
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
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	cfg = MakeSyncCfg();
	cfg.AdvAddrType = 2;
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	cfg = MakeSyncCfg();
	cfg.Skip = BT_ADV_PERIODIC_SKIP_MAX + 1;
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	cfg = MakeSyncCfg();
	cfg.SyncTimeout = BT_ADV_PERIODIC_SYNC_TIMEOUT_MIN - 1;
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	cfg = MakeSyncCfg();
	cfg.SyncTimeout = BT_ADV_PERIODIC_SYNC_TIMEOUT_MAX + 1;
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	g_BtAppData.AppDevice.pHciDev = nullptr;
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	CHECK(BtAdvPeriodicSyncCreate(nullptr) == false);
	CHECK(s_CmdCount == 0);

	// Synchronising is gated on the three sync commands, not the three the
	// advertiser side needs.
	Reset();
	s_Caps.SupportedCommands[BT_HCI_CAP_CMD_LE_PERIODIC_ADV_TERMINATE_SYNC >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_PERIODIC_ADV_TERMINATE_SYNC & 7U)));
	cfg = MakeSyncCfg();
	CHECK(BtAdvPeriodicSyncCreate(&cfg) == false);
	CHECK(s_CmdCount == 0);
}

void TestSyncCancelAndTerminate()
{
	Reset();
	CHECK(BtAdvPeriodicSyncCancel());
	const CapturedCmd *c =
		FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 0);
	}
	g_BtAppData.AppDevice.pHciDev = nullptr;
	CHECK(BtAdvPeriodicSyncCancel() == false);
	g_BtAppData.AppDevice.pHciDev = &s_Dev;

	Reset();
	CHECK(BtAdvPeriodicSyncTerminate(0x0123));
	c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 2);
		CHECK(c->Param[0] == 0x23 && c->Param[1] == 0x01);
	}

	Reset();
	s_FailOpCode = BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC;
	CHECK(BtAdvPeriodicSyncTerminate(0x0123) == false);
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
	CHECK(BtAdvPeriodicSyncTerminate(0x0007));

	const uint8_t a[2] = { 1, 2 };
	BtAdvPeriodicReportFragment(0x0007, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_ReportCount == 0);
}

// --- PAwR advertiser ---

int s_ReqCount, s_RspCount;
uint8_t s_ReqStart, s_ReqCount2, s_RspSubevent, s_RspSlot;
size_t s_RspLen;
uint8_t s_RspData[BT_ADV_PAWR_SUBEVENT_DATA_MAX];

void ResetPawrCallbacks()
{
	s_ReqCount = s_RspCount = 0;
	s_ReqStart = s_ReqCount2 = s_RspSubevent = s_RspSlot = 0xFF;
	s_RspLen = 0;
}

BtAdvPawrCfg_t MakePawrCfg()
{
	BtAdvPawrCfg_t cfg = {};
	cfg.IntervalMin = 0x0060;
	cfg.IntervalMax = 0x0080;
	cfg.OwnAddrType = BTADDR_TYPE_RAND;
	cfg.Sid = 1;
	cfg.NumSubevents = 4;
	cfg.SubeventInterval = 0x10;
	cfg.ResponseSlotDelay = 0x04;
	cfg.ResponseSlotSpacing = 0x04;
	cfg.NumResponseSlots = 8;
	return cfg;
}

// A PAwR train uses the v2 parameters, the only form with the subevent fields.
void TestPawrParametersV2()
{
	Reset();
	BtAdvPawrCfg_t cfg = MakePawrCfg();
	CHECK(BtAdvPawrInit(&cfg));

	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM) == nullptr);
	const CapturedCmd *p = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM_V2);
	CHECK(p != nullptr);
	if (p != nullptr)
	{
		CHECK(p->ParamLen == 12);
		CHECK(p->Param[0] == BT_ADV_PERIODIC_ADV_HANDLE);
		CHECK(p->Param[7] == 4);		// num subevents
		CHECK(p->Param[8] == 0x10);		// subevent interval
		CHECK(p->Param[9] == 0x04);		// response slot delay
		CHECK(p->Param[10] == 0x04);	// response slot spacing
		CHECK(p->Param[11] == 8);		// num response slots
	}

	// The set it rides on is created the same way as for a plain train.
	const CapturedCmd *e = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(e != nullptr);
	if (e != nullptr)
	{
		CHECK(e->Param[0] == BT_ADV_PERIODIC_ADV_HANDLE);
		CHECK(e->Param[1] == 0 && e->Param[2] == 0);
	}
}

void TestPawrValidation()
{
	Reset();
	BtAdvPawrCfg_t cfg = MakePawrCfg();

	// Zero subevents is a plain train, not a PAwR one.
	cfg.NumSubevents = 0;
	CHECK(BtAdvPawrInit(&cfg) == false);
	cfg = MakePawrCfg();
	cfg.NumSubevents = BT_ADV_PAWR_SUBEVENT_MAX + 1;
	CHECK(BtAdvPawrInit(&cfg) == false);
	cfg = MakePawrCfg();
	cfg.SubeventInterval = BT_ADV_PAWR_SUBEVENT_INTERVAL_MIN - 1;
	CHECK(BtAdvPawrInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	// Response slots are all or nothing.
	cfg = MakePawrCfg();
	cfg.ResponseSlotDelay = 0;
	CHECK(BtAdvPawrInit(&cfg) == false);
	cfg = MakePawrCfg();
	cfg.ResponseSlotSpacing = BT_ADV_PAWR_RSP_SLOT_SPACING_MIN - 1;
	CHECK(BtAdvPawrInit(&cfg) == false);
	cfg = MakePawrCfg();
	cfg.NumResponseSlots = 0;
	CHECK(BtAdvPawrInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	// A train nobody answers is legitimate: every slot parameter zero.
	Reset();
	cfg = MakePawrCfg();
	cfg.ResponseSlotDelay = 0;
	cfg.ResponseSlotSpacing = 0;
	cfg.NumResponseSlots = 0;
	CHECK(BtAdvPawrInit(&cfg));

	g_BtAppData.AppDevice.pHciDev = nullptr;
	CHECK(BtAdvPawrInit(&cfg) == false);
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	CHECK(BtAdvPawrInit(nullptr) == false);
}

void TestPawrCapabilityGating()
{
	Reset();
	s_Caps.LeFeatures[BT_HCI_CAP_LE_FEATURE_PAWR_ADVERTISER >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_LE_FEATURE_PAWR_ADVERTISER & 7U)));
	BtAdvPawrCfg_t cfg = MakePawrCfg();
	CHECK(BtAdvPawrInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	s_Caps.SupportedCommands[BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_PARAMETERS_V2 >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_SET_PERIODIC_ADV_PARAMETERS_V2 & 7U)));
	CHECK(BtAdvPawrInit(&cfg) == false);
	CHECK(s_CmdCount == 0);
}

void TestSubeventData()
{
	Reset();
	BtAdvPawrCfg_t cfg = MakePawrCfg();
	CHECK(BtAdvPawrInit(&cfg));
	s_CmdCount = 0;

	const uint8_t data[5] = { 1, 2, 3, 4, 5 };
	CHECK(BtAdvPawrSubeventDataSet(2, 0, 8, data, sizeof(data)));
	const CapturedCmd *c =
		FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SUBEVENT_DATA);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 6 + 5);
		CHECK(c->Param[0] == BT_ADV_PERIODIC_ADV_HANDLE);
		CHECK(c->Param[1] == 1);		// one subevent per command
		CHECK(c->Param[2] == 2);		// subevent
		CHECK(c->Param[3] == 0);		// response slot start
		CHECK(c->Param[4] == 8);		// response slot count
		CHECK(c->Param[5] == 5);		// data length
		CHECK(c->Param[6] == 1 && c->Param[10] == 5);
	}

	// A subevent at or past the configured count is refused.
	s_CmdCount = 0;
	CHECK(BtAdvPawrSubeventDataSet(4, 0, 8, data, sizeof(data)) == false);
	CHECK(BtAdvPawrSubeventDataSet(0, 0, 8, data,
		BT_ADV_PAWR_SUBEVENT_DATA_MAX + 1) == false);
	CHECK(BtAdvPawrSubeventDataSet(0, 0, 8, nullptr, 4) == false);
	CHECK(s_CmdCount == 0);

	// Zero length is legitimate, a subevent that only opens response slots.
	CHECK(BtAdvPawrSubeventDataSet(1, 2, 3, nullptr, 0));
	c = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SUBEVENT_DATA);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 6);
		CHECK(c->Param[5] == 0);
	}

	// Subevent data on a plain train has no subevent to go in.
	Reset();
	BtAdvPeriodicCfg_t plain = MakeCfg();
	CHECK(BtAdvPeriodicInit(&plain));
	CHECK(BtAdvPawrSubeventDataSet(0, 0, 8, data, sizeof(data)) == false);
}

void TestSubeventDataRequestEvent()
{
	Reset();
	ResetPawrCallbacks();
	BtAdvPawrSubeventDataRequestEvt(BT_ADV_PERIODIC_ADV_HANDLE, 3, 2);
	CHECK(s_ReqCount == 1);
	CHECK(s_ReqStart == 3);
	CHECK(s_ReqCount2 == 2);

	// A request for another advertising set is not this module's.
	BtAdvPawrSubeventDataRequestEvt(0, 1, 1);
	CHECK(s_ReqCount == 1);
}

void TestResponseReport()
{
	Reset();
	ResetPawrCallbacks();

	// Two responses. Each is TxPower, RSSI, CTE type, slot, data status,
	// length, then data.
	const uint8_t responses[] = {
		0x00, 0xD6, 0x00, 0x02, 0x00, 0x03, 0xAA, 0xBB, 0xCC,
		0x00, 0xE0, 0x00, 0x05, 0x00, 0x01, 0x11,
	};
	BtAdvPawrResponseReportEvt(BT_ADV_PERIODIC_ADV_HANDLE, 1, 0, 2,
		responses, sizeof(responses));
	CHECK(s_RspCount == 2);
	CHECK(s_RspSubevent == 1);
	CHECK(s_RspSlot == 5);
	CHECK(s_RspLen == 1);
	CHECK(s_RspData[0] == 0x11);

	// An incomplete response is a piece of an answer with no second report to
	// finish it, so it is not handed up.
	ResetPawrCallbacks();
	const uint8_t partial[] = {
		0x00, 0xD6, 0x00, 0x02, 0x01, 0x02, 0xAA, 0xBB,
	};
	BtAdvPawrResponseReportEvt(BT_ADV_PERIODIC_ADV_HANDLE, 1, 0, 1,
		partial, sizeof(partial));
	CHECK(s_RspCount == 0);

	// A count larger than the event delivered stops at the end rather than
	// reading past it.
	ResetPawrCallbacks();
	BtAdvPawrResponseReportEvt(BT_ADV_PERIODIC_ADV_HANDLE, 1, 0, 8,
		responses, sizeof(responses));
	CHECK(s_RspCount == 2);

	// A length that runs past the event is refused rather than copied.
	ResetPawrCallbacks();
	const uint8_t overrun[] = { 0x00, 0xD6, 0x00, 0x02, 0x00, 0x40, 0xAA };
	BtAdvPawrResponseReportEvt(BT_ADV_PERIODIC_ADV_HANDLE, 1, 0, 1,
		overrun, sizeof(overrun));
	CHECK(s_RspCount == 0);

	// Another advertising set is not this module's.
	ResetPawrCallbacks();
	BtAdvPawrResponseReportEvt(0, 1, 0, 2, responses, sizeof(responses));
	CHECK(s_RspCount == 0);
	BtAdvPawrResponseReportEvt(BT_ADV_PERIODIC_ADV_HANDLE, 1, 0, 2,
		nullptr, 0);
	CHECK(s_RspCount == 0);
}

// --- PAwR scanner ---

int s_SubRptCount;
uint16_t s_SubRptEvent;
uint8_t s_SubRptSubevent;
size_t s_SubRptLen;
uint8_t s_SubRptData[BT_ADV_PERIODIC_DATA_MAX];

void ResetScannerCallbacks()
{
	s_SubRptCount = 0;
	s_SubRptEvent = 0xFFFF;
	s_SubRptSubevent = 0xFF;
	s_SubRptLen = 0;
}

void TestSyncSubeventSet()
{
	Reset();
	const uint8_t subevents[3] = { 0, 2, 5 };
	CHECK(BtAdvPawrSyncSubeventSet(0x0007, subevents,
		sizeof(subevents)));

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_SYNC_SUBEVENT);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 5 + 3);
		CHECK(c->Param[0] == 0x07 && c->Param[1] == 0x00);
		CHECK(c->Param[2] == 0 && c->Param[3] == 0);	// properties
		CHECK(c->Param[4] == 3);
		CHECK(c->Param[5] == 0 && c->Param[6] == 2 && c->Param[7] == 5);
	}

	// Validation.
	Reset();
	g_BtAppData.AppDevice.pHciDev = nullptr;
	CHECK(BtAdvPawrSyncSubeventSet(0x0007, subevents, 3) == false);
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	CHECK(BtAdvPawrSyncSubeventSet(0x0007, nullptr, 3) == false);
	CHECK(BtAdvPawrSyncSubeventSet(0x0007, subevents, 0) == false);
	const uint8_t bad[2] = { 0, BT_ADV_PAWR_SUBEVENT_NUM_MAX + 1 };
	CHECK(BtAdvPawrSyncSubeventSet(0x0007, bad, 2) == false);
	CHECK(s_CmdCount == 0);

	// Gated on its own command.
	Reset();
	s_Caps.SupportedCommands[BT_HCI_CAP_CMD_LE_SET_PERIODIC_SYNC_SUBEVENT >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_SET_PERIODIC_SYNC_SUBEVENT & 7U)));
	CHECK(BtAdvPawrSyncSubeventSet(0x0007, subevents, 3) == false);
	CHECK(s_CmdCount == 0);
}

void TestResponseDataSet()
{
	Reset();
	const uint8_t data[3] = { 0xDE, 0xAD, 0xBE };
	CHECK(BtAdvPawrResponseDataSet(0x0007, 0x1234, 2, 3, 4,
		data, sizeof(data)));

	const CapturedCmd *c =
		FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_RESPONSE_DATA);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 8 + 3);
		CHECK(c->Param[0] == 0x07 && c->Param[1] == 0x00);
		CHECK(c->Param[2] == 0x34 && c->Param[3] == 0x12);	// request event
		CHECK(c->Param[4] == 2);		// request subevent
		CHECK(c->Param[5] == 3);		// response subevent
		CHECK(c->Param[6] == 4);		// response slot
		CHECK(c->Param[7] == 3);		// length
		CHECK(c->Param[8] == 0xDE && c->Param[10] == 0xBE);
	}

	Reset();
	g_BtAppData.AppDevice.pHciDev = nullptr;
	CHECK(BtAdvPawrResponseDataSet(0x0007, 0, 0, 0, 0, data, 3) == false);
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	CHECK(BtAdvPawrResponseDataSet(0x0007, 0, 0, 0, 0, nullptr, 3) == false);
	CHECK(BtAdvPawrResponseDataSet(0x0007, 0, 0, 0, 0, data,
		BT_ADV_PAWR_RESPONSE_DATA_MAX + 1) == false);
	CHECK(BtAdvPawrResponseDataSet(0x0007, 0,
		BT_ADV_PAWR_SUBEVENT_NUM_MAX + 1, 0, 0, data, 3) == false);
	CHECK(s_CmdCount == 0);

	// Zero length is an empty answer, still an answer.
	CHECK(BtAdvPawrResponseDataSet(0x0007, 1, 0, 0, 0, nullptr, 0));
}

// The v2 report is not the v1 layout with a tail: the event counter and
// subevent sit in the middle, and both have to reach the application because
// a reply quotes them back.
void TestSubeventReportReassembly()
{
	Reset();
	ResetSyncCallbacks();
	ResetScannerCallbacks();
	uint8_t addr[6] = {};
	BtAdvPeriodicSyncEstablishedEvt(0, 0x0007, 3, 1, addr, 1, 0x0080);

	const uint8_t a[3] = { 1, 2, 3 };
	const uint8_t b[2] = { 4, 5 };
	BtAdvPawrSubeventReportEvt(0x0007, 0x00AB, 2, 0, -50,
		BT_ADV_PERIODIC_DATA_MORE, sizeof(a), a);
	CHECK(s_SubRptCount == 0);
	// A later part reporting a different event does not move the answer target.
	BtAdvPawrSubeventReportEvt(0x0007, 0x00AC, 3, 0, -51,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(b), b);
	CHECK(s_SubRptCount == 1);
	CHECK(s_SubRptEvent == 0x00AB);
	CHECK(s_SubRptSubevent == 2);
	CHECK(s_SubRptLen == 5);
	CHECK(s_SubRptData[0] == 1 && s_SubRptData[4] == 5);

	// Truncated is dropped, like the plain report path.
	ResetScannerCallbacks();
	BtAdvPawrSubeventReportEvt(0x0007, 0x00B0, 1, 0, 0,
		BT_ADV_PERIODIC_DATA_TRUNCATED, sizeof(a), a);
	CHECK(s_SubRptCount == 0);

	// A report for a train this module is not synced to is ignored.
	BtAdvPawrSubeventReportEvt(0x0009, 0x00B1, 1, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_SubRptCount == 0);

	// And the next payload starts clean.
	BtAdvPawrSubeventReportEvt(0x0007, 0x00B2, 6, 0, 0,
		BT_ADV_PERIODIC_DATA_COMPLETE, sizeof(a), a);
	CHECK(s_SubRptCount == 1);
	CHECK(s_SubRptEvent == 0x00B2);
	CHECK(s_SubRptSubevent == 6);
	CHECK(s_SubRptLen == 3);
}

} // namespace

void BtAdvPawrSubeventDataRequest(uint8_t, uint8_t SubeventStart,
	uint8_t SubeventDataCount)
{
	s_ReqCount++;
	s_ReqStart = SubeventStart;
	s_ReqCount2 = SubeventDataCount;
}

void BtAdvPawrSubeventReport(uint16_t, uint16_t EventCounter,
	uint8_t Subevent, int8_t, size_t Len, const uint8_t *pData)
{
	s_SubRptCount++;
	s_SubRptEvent = EventCounter;
	s_SubRptSubevent = Subevent;
	s_SubRptLen = Len;
	if (Len > 0 && pData != nullptr && Len <= sizeof(s_SubRptData))
	{
		std::memcpy(s_SubRptData, pData, Len);
	}
}

void BtAdvPawrResponse(uint8_t, uint8_t Subevent, uint8_t ResponseSlot,
	int8_t, size_t Len, const uint8_t *pData)
{
	s_RspCount++;
	s_RspSubevent = Subevent;
	s_RspSlot = ResponseSlot;
	s_RspLen = Len;
	if (Len > 0 && pData != nullptr && Len <= sizeof(s_RspData))
	{
		std::memcpy(s_RspData, pData, Len);
	}
}

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
	TestPawrParametersV2();
	TestPawrValidation();
	TestPawrCapabilityGating();
	TestSubeventData();
	TestSubeventDataRequestEvent();
	TestResponseReport();
	TestSyncSubeventSet();
	TestResponseDataSet();
	TestSubeventReportReassembly();

	if (s_Failures == 0)
	{
		std::printf("Periodic advertising host tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Periodic advertising host tests: FAIL (%d failures, %d checks)\n",
		s_Failures, s_Checks);
	return 1;
}
