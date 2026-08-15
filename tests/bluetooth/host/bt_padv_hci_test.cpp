// Command-level coverage for the periodic advertising path. BtHciCommand
// dispatches through the device Command pointer, so a capture device records
// the opcode and packed parameter bytes each call emits. No controller is
// involved, which is the point: what is checked is the wire layout of Core
// spec Vol 4 Part E sections 7.8.61 to 7.8.63 and the order the commands go
// out in, both of which a controller would only report as a status byte.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_padv.h"

// bt_app.cpp is not linked here. It brings the whole application state machine
// in for one field, the HCI device the command helper reaches the controller
// through, so the definition lives here instead.
BtAppData_t g_BtAppData;

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

CapturedCmd s_Cmds[16];
int s_CmdCount = 0;

// Opcode the stub controller rejects, and which occurrence of it. 0 rejects
// every call, n rejects the nth. Lets a case drive a failure in the middle of
// a fragment sequence.
uint16_t s_FailOpCode = 0;
int s_FailNth = 0;
int s_OpCodeSeen = 0;

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
					   uint8_t ParamLen, void *, uint8_t)
{
	if (s_CmdCount < (int)(sizeof(s_Cmds) / sizeof(s_Cmds[0])))
	{
		CapturedCmd &c = s_Cmds[s_CmdCount++];
		c.OpCode   = OpCode;
		c.ParamLen = ParamLen;
		if (pParam != nullptr)
		{
			std::memcpy(c.Param, pParam, ParamLen);
		}
	}

	if (s_FailOpCode != 0 && OpCode == s_FailOpCode)
	{
		s_OpCodeSeen++;
		if (s_FailNth == 0 || s_OpCodeSeen == s_FailNth)
		{
			return 0x0C;		// Command Disallowed
		}
	}

	return 0;		// HCI success
}

BtHciDevice_t s_Dev;

const uint8_t kAdvHdl = 0;

void Reset(void)
{
	s_CmdCount = 0;
	s_FailOpCode = 0;
	s_FailNth = 0;
	s_OpCodeSeen = 0;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = CaptureCommand;
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
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

const CapturedCmd *NthCmd(uint16_t OpCode, int N)
{
	int n = 0;

	for (int i = 0; i < s_CmdCount; i++)
	{
		if (s_Cmds[i].OpCode == OpCode && ++n == N)
		{
			return &s_Cmds[i];
		}
	}

	return nullptr;
}

BtPadvCfg_t MakeCfg(void)
{
	BtPadvCfg_t cfg;

	std::memset(&cfg, 0, sizeof(cfg));
	cfg.AdvHdl = kAdvHdl;
	cfg.IntervalMin = 0x0060;		// 120 ms
	cfg.IntervalMax = 0x0080;		// 160 ms
	cfg.Properties = BTPADV_PROP_TXPWR;

	return cfg;
}

// Bring a train up to configured so the data and enable cases start from a
// handle this layer accepts.
bool Configure(void)
{
	BtPadvCfg_t cfg = MakeCfg();

	return BtPadvInit(&cfg);
}

// The opcodes these commands go out under have to be the ones the controller
// answers. The SoftDevice Controller names them in its own header as
// SDC_HCI_OPCODE_CMD_LE_SET_PERIODIC_ADV_PARAMS 0x203e, _DATA 0x203f and
// _ENABLE 0x2040, and bt_hci_ctlr_sdc.cpp dispatches on the constants below,
// so a mismatch would send a periodic advertising command under some other
// opcode and never show up as anything but a refusal on hardware.
void TestOpcodeValues(void)
{
	CHECK(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM == 0x203E);
	CHECK(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA == 0x203F);
	CHECK(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE == 0x2040);

	// The sync side, for the same reason, ahead of the code that uses them.
	CHECK(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC == 0x2044);
	CHECK(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL == 0x2045);
	CHECK(BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC == 0x2046);
}

// LE Set Periodic Advertising Parameters, Vol 4 Part E 7.8.61: Advertising_-
// Handle(1) Interval_Min(2) Interval_Max(2) Properties(2), little endian.
void TestParametersLayout(void)
{
	Reset();

	CHECK(Configure());
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM) == 1);

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM, 1);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	CHECK(c->ParamLen == 7);
	CHECK(c->Param[0] == kAdvHdl);
	CHECK(c->Param[1] == 0x60 && c->Param[2] == 0x00);
	CHECK(c->Param[3] == 0x80 && c->Param[4] == 0x00);
	CHECK(c->Param[5] == BTPADV_PROP_TXPWR && c->Param[6] == 0x00);
}

// The interval range is checked before the command goes out, because a
// controller reports a violation as one status byte that does not say which
// of the two parameters was wrong.
void TestIntervalRangeRefusedBeforeTheCommand(void)
{
	BtPadvCfg_t cfg;

	Reset();
	cfg = MakeCfg();
	cfg.IntervalMin = BTPADV_INTERVAL_MIN - 1;
	CHECK(BtPadvInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	cfg = MakeCfg();
	cfg.IntervalMin = 0x0100;
	cfg.IntervalMax = 0x0080;			// max below min
	CHECK(BtPadvInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Reset();
	cfg = MakeCfg();
	cfg.AdvHdl = BTPADV_ADV_HDL_MAX + 1;
	CHECK(BtPadvInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	// The boundary values themselves are legal.
	Reset();
	cfg = MakeCfg();
	cfg.IntervalMin = BTPADV_INTERVAL_MIN;
	cfg.IntervalMax = BTPADV_INTERVAL_MAX;
	CHECK(BtPadvInit(&cfg));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM) == 1);
}

// A refused command is reported as a failure. What the controller does with
// its own set table is its business: this layer does not mirror it, so a
// refusal is passed up rather than turned into local state.
void TestARefusedCommandIsReported(void)
{
	Reset();
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM;

	CHECK(Configure() == false);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM) == 1);
	CHECK(BtPadvIsEnabled(kAdvHdl) == false);

	Reset();
	CHECK(Configure());
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA;

	const uint8_t data[4] = { 1, 2, 3, 4 };
	CHECK(BtPadvDataSet(kAdvHdl, data, sizeof(data)) == false);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 1);
}

// The Advertising_Handle range is 0x00 to 0xEF (Vol 4 Part E 7.8.61). That is
// a property of the parameter rather than of the controller state, so it is
// checked here and no command goes out.
void TestHandleRangeIsCheckedLocally(void)
{
	const uint8_t bad = BTPADV_ADV_HDL_MAX + 1;
	const uint8_t data[4] = { 1, 2, 3, 4 };

	Reset();
	CHECK(BtPadvDataSet(bad, data, sizeof(data)) == false);
	CHECK(BtPadvDataRefresh(bad) == false);
	CHECK(BtPadvStart(bad) == false);
	CHECK(BtPadvStop(bad) == false);
	CHECK(s_CmdCount == 0);

	// The top of the range is legal.
	Reset();
	CHECK(BtPadvStart(BTPADV_ADV_HDL_MAX));
	CHECK(BtPadvIsEnabled(BTPADV_ADV_HDL_MAX));
	CHECK(BtPadvStop(BTPADV_ADV_HDL_MAX));
}

// The enable state is per handle. A train enabled on one set must not make the
// data path treat another set as enabled, which would refuse a fragment
// sequence that is legal there.
void TestTheEnableStateIsPerHandle(void)
{
	const size_t len = BTPADV_DATA_FRAG_MAX + 1;
	static uint8_t data[BTPADV_DATA_FRAG_MAX + 1];

	std::memset(data, 0x33, sizeof(data));

	Reset();
	CHECK(BtPadvStart(1));
	CHECK(BtPadvIsEnabled(1));
	CHECK(BtPadvIsEnabled(2) == false);

	// Handle 2 is not enabled, so fragmenting on it is allowed.
	CHECK(BtPadvDataSet(2, data, len));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 2);

	// Handle 1 is enabled, so it is not.
	CHECK(BtPadvDataSet(1, data, len) == false);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 2);
}

// Data that fits one command goes out as the complete operation, 0x03.
void TestDataThatFitsUsesOneCompleteOperation(void)
{
	uint8_t data[BTPADV_DATA_FRAG_MAX];

	for (int i = 0; i < BTPADV_DATA_FRAG_MAX; i++)
	{
		data[i] = (uint8_t)i;
	}

	Reset();
	CHECK(Configure());
	CHECK(BtPadvDataSet(kAdvHdl, data, sizeof(data)));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 1);

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, 1);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	// Advertising_Handle(1) Operation(1) Advertising_Data_Length(1) then data.
	CHECK(c->Param[0] == kAdvHdl);
	CHECK(c->Param[1] == 0x03);
	CHECK(c->Param[2] == BTPADV_DATA_FRAG_MAX);
	CHECK(c->ParamLen == 3 + BTPADV_DATA_FRAG_MAX);
	CHECK(std::memcmp(&c->Param[3], data, sizeof(data)) == 0);

	// Zero length clears, and is still the complete operation.
	Reset();
	CHECK(Configure());
	CHECK(BtPadvDataSet(kAdvHdl, nullptr, 0));
	c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, 1);
	CHECK(c != nullptr && c->Param[1] == 0x03 && c->Param[2] == 0 &&
		  c->ParamLen == 3);
}

// Longer data is fragmented first / intermediate / last, 0x01 / 0x00 / 0x02,
// and the fragments reassemble to the original.
void TestLongerDataIsFragmented(void)
{
	const size_t len = BTPADV_DATA_FRAG_MAX * 2 + 10;
	static uint8_t data[BTPADV_DATA_FRAG_MAX * 2 + 10];

	for (size_t i = 0; i < len; i++)
	{
		data[i] = (uint8_t)(i * 7);
	}

	Reset();
	CHECK(Configure());
	CHECK(BtPadvDataSet(kAdvHdl, data, len));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 3);

	const uint8_t expectOp[3] = { 0x01, 0x00, 0x02 };
	const uint8_t expectLen[3] = { BTPADV_DATA_FRAG_MAX, BTPADV_DATA_FRAG_MAX, 10 };
	size_t off = 0;

	for (int i = 0; i < 3; i++)
	{
		const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, i + 1);
		CHECK(c != nullptr);
		if (c == nullptr)
		{
			return;
		}

		CHECK(c->Param[0] == kAdvHdl);
		CHECK(c->Param[1] == expectOp[i]);
		CHECK(c->Param[2] == expectLen[i]);
		CHECK(c->ParamLen == 3 + expectLen[i]);
		CHECK(std::memcmp(&c->Param[3], data + off, expectLen[i]) == 0);
		off += expectLen[i];
	}

	CHECK(off == len);
}

// Vol 4 Part E 7.8.62: with the train enabled the controller answers any
// operation other than complete or unchanged with Command Disallowed. Sending
// the first fragment anyway has it discard the data it holds and then refuse
// the rest, which leaves the train with nothing to advertise.
void TestFragmentedDataIsRefusedWhileEnabled(void)
{
	const size_t len = BTPADV_DATA_FRAG_MAX + 1;
	static uint8_t data[BTPADV_DATA_FRAG_MAX + 1];

	std::memset(data, 0xA5, sizeof(data));

	Reset();
	CHECK(Configure());
	CHECK(BtPadvStart(kAdvHdl));
	CHECK(BtPadvIsEnabled(kAdvHdl));

	int before = CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA);
	CHECK(BtPadvDataSet(kAdvHdl, data, len) == false);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == before);

	// Data that fits one command is the complete operation, which stays legal.
	CHECK(BtPadvDataSet(kAdvHdl, data, BTPADV_DATA_FRAG_MAX));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == before + 1);
}

// A fragment sequence that fails partway leaves the controller holding a
// partial set, which it refuses to enable. Clearing it is what lets the caller
// retry.
void TestAFailedFragmentClearsThePartialSet(void)
{
	const size_t len = BTPADV_DATA_FRAG_MAX * 2;
	static uint8_t data[BTPADV_DATA_FRAG_MAX * 2];

	std::memset(data, 0x5A, sizeof(data));

	Reset();
	CHECK(Configure());
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA;
	s_FailNth = 2;					// first fragment lands, second is refused

	CHECK(BtPadvDataSet(kAdvHdl, data, len) == false);

	// First fragment, refused second, then the clear.
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 3);

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, 3);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	CHECK(c->Param[1] == 0x03);		// complete
	CHECK(c->Param[2] == 0);		// with no data, which clears
}

// LE Set Periodic Advertising Enable, Vol 4 Part E 7.8.63: Enable(1) then
// Advertising_Handle(1). Bit 1 asks for the ADI field and needs a feature a
// controller may not have, so only bit 0 is set.
void TestEnableLayoutAndState(void)
{
	Reset();
	CHECK(Configure());
	CHECK(BtPadvIsEnabled(kAdvHdl) == false);

	CHECK(BtPadvStart(kAdvHdl));

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE, 1);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	CHECK(c->ParamLen == 2);
	CHECK(c->Param[0] == 0x01);
	CHECK(c->Param[1] == kAdvHdl);
	CHECK(BtPadvIsEnabled(kAdvHdl));

	CHECK(BtPadvStop(kAdvHdl));
	c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE, 2);
	CHECK(c != nullptr && c->ParamLen == 2 && c->Param[0] == 0x00 &&
		  c->Param[1] == kAdvHdl);
	CHECK(BtPadvIsEnabled(kAdvHdl) == false);
}

// A refused enable or disable must not move the recorded state. Recording a
// refused disable as stopped would make a later start look like a no-op while
// the train stays on air.
void TestARefusedEnableDoesNotMoveTheState(void)
{
	Reset();
	CHECK(Configure());
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE;

	CHECK(BtPadvStart(kAdvHdl) == false);
	CHECK(BtPadvIsEnabled(kAdvHdl) == false);

	Reset();
	CHECK(Configure());
	CHECK(BtPadvStart(kAdvHdl));
	CHECK(BtPadvIsEnabled(kAdvHdl));

	s_FailOpCode = BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE;
	CHECK(BtPadvStop(kAdvHdl) == false);
	CHECK(BtPadvIsEnabled(kAdvHdl));
}

// Vol 4 Part E 7.8.62 answers the unchanged operation with Invalid HCI Command
// Parameters when the train is disabled, so it is refused here instead.
void TestDataRefreshNeedsAnEnabledTrain(void)
{
	Reset();
	CHECK(Configure());
	CHECK(BtPadvDataRefresh(kAdvHdl) == false);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA) == 0);

	CHECK(BtPadvStart(kAdvHdl));
	CHECK(BtPadvDataRefresh(kAdvHdl));

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, 1);
	CHECK(c != nullptr && c->Param[1] == 0x04 && c->Param[2] == 0 &&
		  c->ParamLen == 3);
}

// No HCI device wired is refused rather than dereferenced.
void TestNoDeviceIsRefused(void)
{
	Reset();
	g_BtAppData.AppDevice.pHciDev = nullptr;

	BtPadvCfg_t cfg = MakeCfg();
	CHECK(BtPadvInit(&cfg) == false);
	CHECK(BtPadvInit(nullptr) == false);
	CHECK(s_CmdCount == 0);
}

} // namespace

int main()
{
	TestOpcodeValues();
	TestParametersLayout();
	TestIntervalRangeRefusedBeforeTheCommand();
	TestARefusedCommandIsReported();
	TestHandleRangeIsCheckedLocally();
	TestTheEnableStateIsPerHandle();
	TestDataThatFitsUsesOneCompleteOperation();
	TestLongerDataIsFragmented();
	TestFragmentedDataIsRefusedWhileEnabled();
	TestAFailedFragmentClearsThePartialSet();
	TestEnableLayoutAndState();
	TestARefusedEnableDoesNotMoveTheState();
	TestDataRefreshNeedsAnEnabledTrain();
	TestNoDeviceIsRefused();

	if (s_Failures == 0)
	{
		std::printf("Periodic advertising HCI tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Periodic advertising HCI tests: %d failure(s), %d checks\n",
				s_Failures, s_Checks);

	return 1;
}
