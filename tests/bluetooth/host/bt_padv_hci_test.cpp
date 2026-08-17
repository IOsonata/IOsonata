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


// --- Periodic Advertising with Responses ---

// 7.8.61 [v2] is a separate opcode from [v1], not a version parameter, and it
// appends Num_Subevents(1) Subevent_Interval(1) Response_Slot_Delay(1)
// Response_Slot_Spacing(1) Num_Response_Slots(1).
void TestPawrUsesTheV2Opcode(void)
{
	Reset();

	BtPadvCfg_t cfg = MakeCfg();
	cfg.NbSubevents = 4;
	cfg.SubeventInterval = 0x18;
	cfg.RspSlotDelay = 0x08;
	cfg.RspSlotSpacing = 0x10;
	cfg.NbRspSlots = 4;

	CHECK(BtPadvInit(&cfg));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM) == 0);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM_V2) == 1);

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM_V2, 1);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	CHECK(c->ParamLen == 12);
	CHECK(c->Param[0] == kAdvHdl);
	CHECK(c->Param[7] == 4);
	CHECK(c->Param[8] == 0x18);
	CHECK(c->Param[9] == 0x08);
	CHECK(c->Param[10] == 0x10);
	CHECK(c->Param[11] == 4);
	CHECK(BtPadvNbSubevents() == 4);

	// Zero subevents is a plain train and keeps the [v1] opcode.
	Reset();
	cfg = MakeCfg();
	CHECK(BtPadvInit(&cfg));
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM) == 1);
	CHECK(CountCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM_V2) == 0);
	CHECK(BtPadvNbSubevents() == 0);
}

// 7.8.61 states the relations between the four PAwR parameters. A controller
// answers all of them with one status byte, so they are checked before the
// command goes out.
void TestPawrParameterRelationsRefusedLocally(void)
{
	BtPadvCfg_t cfg;

	Reset();
	// Subevent_Interval x Num_Subevents must fit Interval_Min. Interval_Min is
	// 0x0060, so four subevents of 0x19 do not fit.
	cfg = MakeCfg();
	cfg.NbSubevents = 4;
	cfg.SubeventInterval = 0x19;
	cfg.RspSlotDelay = 0x08;
	cfg.RspSlotSpacing = 0x10;
	cfg.NbRspSlots = 4;
	CHECK(BtPadvInit(&cfg) == false);

	// Response_Slot_Delay must be less than Subevent_Interval.
	cfg = MakeCfg();
	cfg.NbSubevents = 4;
	cfg.SubeventInterval = 0x18;
	cfg.RspSlotDelay = 0x18;
	cfg.RspSlotSpacing = 0x10;
	cfg.NbRspSlots = 4;
	CHECK(BtPadvInit(&cfg) == false);

	// Response_Slot_Spacing x Num_Response_Slots must fit
	// 10 x (Subevent_Interval - Response_Slot_Delay). That is 160 here, so
	// four slots of 0x29 do not fit and four of 0x28 do.
	cfg = MakeCfg();
	cfg.NbSubevents = 4;
	cfg.SubeventInterval = 0x18;
	cfg.RspSlotDelay = 0x08;
	cfg.RspSlotSpacing = 0x29;
	cfg.NbRspSlots = 4;
	CHECK(BtPadvInit(&cfg) == false);

	cfg.NbSubevents = 0x81;
	CHECK(BtPadvInit(&cfg) == false);

	CHECK(s_CmdCount == 0);

	cfg = MakeCfg();
	cfg.NbSubevents = 4;
	cfg.SubeventInterval = 0x18;
	cfg.RspSlotDelay = 0x08;
	cfg.RspSlotSpacing = 0x28;
	cfg.NbRspSlots = 4;
	CHECK(BtPadvInit(&cfg));

	// A single response slot ignores the spacing, so a value that would not
	// fit four slots is accepted for one.
	cfg.NbRspSlots = 1;
	cfg.RspSlotSpacing = 0xFF;
	CHECK(BtPadvInit(&cfg));
	CHECK(s_CmdCount == 2);
}

// 7.8.125, and Vol 4 Part E 5.4.1 for the ordering: the arrayed parameters
// interleave one record per subevent. Encoding them as five parallel blocks
// gives a buffer of the right length whose every octet after the first two is
// in the wrong place.
void TestSubeventDataInterleaves(void)
{
	Reset();

	const uint8_t a[3] = { 0xA0, 0xA1, 0xA2 };
	const uint8_t b[2] = { 0xB0, 0xB1 };
	BtPadvSubeventData_t se[2];

	se[0].Subevent = 1;
	se[0].RspSlotStart = 0;
	se[0].RspSlotCount = 2;
	se[0].DataLen = sizeof(a);
	se[0].pData = a;
	se[1].Subevent = 2;
	se[1].RspSlotStart = 2;
	se[1].RspSlotCount = 3;
	se[1].DataLen = sizeof(b);
	se[1].pData = b;

	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 2));

	const CapturedCmd *c = NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SUBEVENT_DATA, 1);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	// AdvHdl, NbSubevents, then {1,0,2,3,A0,A1,A2}, then {2,2,3,2,B0,B1}.
	CHECK(c->ParamLen == 2 + 4 + 3 + 4 + 2);
	CHECK(c->Param[0] == kAdvHdl);
	CHECK(c->Param[1] == 2);
	CHECK(c->Param[2] == 1 && c->Param[3] == 0 && c->Param[4] == 2);
	CHECK(c->Param[5] == 3);
	CHECK(std::memcmp(&c->Param[6], a, sizeof(a)) == 0);
	CHECK(c->Param[9] == 2 && c->Param[10] == 2 && c->Param[11] == 3);
	CHECK(c->Param[12] == 2);
	CHECK(std::memcmp(&c->Param[13], b, sizeof(b)) == 0);
}

void TestSubeventDataRangesRefusedLocally(void)
{
	const uint8_t a[3] = { 1, 2, 3 };
	BtPadvSubeventData_t se[1];

	Reset();
	se[0].Subevent = 0;
	se[0].RspSlotStart = 0;
	se[0].RspSlotCount = 1;
	se[0].DataLen = sizeof(a);
	se[0].pData = a;

	CHECK(BtPadvSubeventDataSet(BTPADV_ADV_HDL_MAX + 1, se, 1) == false);
	CHECK(BtPadvSubeventDataSet(kAdvHdl, nullptr, 1) == false);
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 0) == false);
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, BTPADV_SUBEVENT_DATA_MAX + 1) == false);

	// Subevent index range is 0x00 to 0x7F.
	se[0].Subevent = BTPADV_SUBEVENT_MAX + 1;
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 1) == false);

	// Data with no buffer behind it.
	se[0].Subevent = 0;
	se[0].pData = nullptr;
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 1) == false);

	CHECK(s_CmdCount == 0);

	// Zero length with a null buffer is legal, it clears the subevent.
	se[0].DataLen = 0;
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 1));
	CHECK(s_CmdCount == 1);
}

// More than one HCI command packet can hold is refused whole. The controller
// discards the entire set of a command it refuses, so sending part of it
// would leave the train advertising some subevents and not others.
void TestSubeventDataTooLongIsRefusedWhole(void)
{
	static uint8_t big[BTPADV_SUBEVENT_DATA_LEN_MAX];
	BtPadvSubeventData_t se[2];

	std::memset(big, 0x5A, sizeof(big));

	Reset();
	for (int i = 0; i < 2; i++)
	{
		se[i].Subevent = (uint8_t)i;
		se[i].RspSlotStart = 0;
		se[i].RspSlotCount = 1;
		se[i].DataLen = sizeof(big);
		se[i].pData = big;
	}

	// Two subevents of 251 octets is 502 plus headers, past the 255 octet
	// Parameter_Total_Length.
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 2) == false);
	CHECK(s_CmdCount == 0);

	// One of them does not fit either. 7.8.125 gives Subevent_Data_Length a
	// range up to 251, but Parameter_Total_Length is one octet and the
	// command spends 2 on the header and 4 on the record, so the effective
	// maximum for a single subevent is 249. Vol 4 Part E 5.4.1 names this
	// case: the specified maximum of an arrayed parameter can exceed what the
	// packet holds, and the effective maximum is then lower.
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 1) == false);
	CHECK(s_CmdCount == 0);

	se[0].DataLen = 250;
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 1) == false);

	se[0].DataLen = 249;
	CHECK(BtPadvSubeventDataSet(kAdvHdl, se, 1));
	CHECK(s_CmdCount == 1);
	CHECK(NthCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SUBEVENT_DATA, 1)->ParamLen == 255);
}

// 7.7.65.36: the subevent numbers wrap from one less than the number of
// subevents back to zero, so a request is not simply Start to Start + Count.
void TestDataRequestSubeventsWrap(void)
{
	// Six subevents, asked for three starting at four: 4, 5, 0.
	CHECK(BtPadvSubeventOfRequest(4, 0, 6) == 4);
	CHECK(BtPadvSubeventOfRequest(4, 1, 6) == 5);
	CHECK(BtPadvSubeventOfRequest(4, 2, 6) == 0);

	// No wrap when it does not reach the end.
	CHECK(BtPadvSubeventOfRequest(0, 2, 6) == 2);

	// A train with no subevents has no answer to give.
	CHECK(BtPadvSubeventOfRequest(0, 0, 0) == 0xFF);
}

} // namespace

int main()
{
	TestOpcodeValues();
	TestPawrUsesTheV2Opcode();
	TestPawrParameterRelationsRefusedLocally();
	TestSubeventDataInterleaves();
	TestSubeventDataRangesRefusedLocally();
	TestSubeventDataTooLongIsRefusedWhole();
	TestDataRequestSubeventsWrap();
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
