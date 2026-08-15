// Periodic advertising synchronization: the command wire layouts and the
// report reassembly. BtHciCommand dispatches through the device Command
// pointer, so a capture device records what each call emits. The event side
// is driven by handing the handlers the payload an LE meta event would carry,
// which is what the HCI dispatcher does after bounding it.
//
// Core spec Vol 4 Part E 7.8.67 to 7.8.73, 7.8.88 and 7.7.65.14 to 7.7.65.16.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_psync.h"

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
	uint8_t  Param[64];
	uint16_t ParamLen;
};

CapturedCmd s_Cmds[16];
int s_CmdCount = 0;
uint16_t s_FailOpCode = 0;
uint8_t s_ListSize = 0;

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
					   uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (s_CmdCount < (int)(sizeof(s_Cmds) / sizeof(s_Cmds[0])))
	{
		CapturedCmd &c = s_Cmds[s_CmdCount++];
		c.OpCode   = OpCode;
		c.ParamLen = ParamLen;
		if (pParam != nullptr && ParamLen <= sizeof(c.Param))
		{
			std::memcpy(c.Param, pParam, ParamLen);
		}
	}

	if (s_FailOpCode != 0 && OpCode == s_FailOpCode)
	{
		return 0x0C;		// Command Disallowed
	}

	if (OpCode == BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE &&
		pRet != nullptr && RetLen >= 1)
	{
		*(uint8_t*)pRet = s_ListSize;
	}

	return 0;
}

BtHciDevice_t s_Dev;

// What the application hooks were handed.
struct {
	int EstablishedCount;
	BtPsyncInfo_t Info;
	int ReportCount;
	uint16_t ReportHdl;
	int8_t ReportTxPwr;
	int8_t ReportRssi;
	uint8_t ReportCteType;
	uint16_t ReportLen;
	uint8_t ReportData[BTPSYNC_REPORT_MAX];
	int LostCount;
	uint16_t LostHdl;
} s_App;

const uint8_t kAddr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

void Reset(void)
{
	s_CmdCount = 0;
	s_FailOpCode = 0;
	s_ListSize = 0;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	std::memset(&s_App, 0, sizeof(s_App));
	s_Dev.Command = CaptureCommand;
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	// Any train state a previous case left behind.
	for (uint16_t h = 0; h < 8; h++)
	{
		BtPsyncReasmReset(h);
	}
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

BtPsyncCfg_t MakeCfg(void)
{
	BtPsyncCfg_t cfg;

	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Options = 0;
	cfg.AdvSid = 3;
	cfg.AdvAddrType = BTPSYNC_ADDR_RANDOM;
	std::memcpy(cfg.AdvAddr, kAddr, 6);
	cfg.Skip = 0x0010;
	cfg.SyncTimeout = 0x0100;
	cfg.SyncCteType = 0;

	return cfg;
}

// --- commands ------------------------------------------------------------

// 7.8.67: Options(1) Advertising_SID(1) Advertiser_Address_Type(1)
// Advertiser_Address(6) Skip(2) Sync_Timeout(2) Sync_CTE_Type(1).
void TestCreateSyncLayout(void)
{
	Reset();

	BtPsyncCfg_t cfg = MakeCfg();
	CHECK(BtPsyncCreate(&cfg));

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC);
	CHECK(c != nullptr);
	if (c == nullptr)
	{
		return;
	}

	CHECK(c->ParamLen == 14);
	CHECK(c->Param[0] == 0);
	CHECK(c->Param[1] == 3);
	CHECK(c->Param[2] == BTPSYNC_ADDR_RANDOM);
	CHECK(std::memcmp(&c->Param[3], kAddr, 6) == 0);
	CHECK(c->Param[9] == 0x10 && c->Param[10] == 0x00);
	CHECK(c->Param[11] == 0x00 && c->Param[12] == 0x01);
	CHECK(c->Param[13] == 0);
}

// Every range 7.8.67 gives is checked before the command goes out, because
// the controller answers a violation with one status byte covering seven
// parameters, and Create Sync answers Command Status rather than Command
// Complete, so there is even less to work back from.
void TestCreateSyncRangesRefusedLocally(void)
{
	BtPsyncCfg_t cfg;

	Reset();
	cfg = MakeCfg();
	cfg.Options = 0x08;					// reserved bit
	CHECK(BtPsyncCreate(&cfg) == false);

	cfg = MakeCfg();
	cfg.AdvSid = BTPSYNC_SID_MAX + 1;
	CHECK(BtPsyncCreate(&cfg) == false);

	cfg = MakeCfg();
	cfg.AdvAddrType = 2;				// only public and random on the command
	CHECK(BtPsyncCreate(&cfg) == false);

	cfg = MakeCfg();
	cfg.Skip = BTPSYNC_SKIP_MAX + 1;
	CHECK(BtPsyncCreate(&cfg) == false);

	cfg = MakeCfg();
	cfg.SyncTimeout = BTPSYNC_TIMEOUT_MIN - 1;
	CHECK(BtPsyncCreate(&cfg) == false);

	cfg = MakeCfg();
	cfg.SyncTimeout = BTPSYNC_TIMEOUT_MAX + 1;
	CHECK(BtPsyncCreate(&cfg) == false);

	// Refusing every Constant Tone Extension kind leaves nothing to sync to,
	// which the controller answers Command Disallowed.
	cfg = MakeCfg();
	cfg.SyncCteType = BTPSYNC_CTE_ALL;
	CHECK(BtPsyncCreate(&cfg) == false);

	cfg = MakeCfg();
	cfg.SyncCteType = 0x20;				// reserved bit
	CHECK(BtPsyncCreate(&cfg) == false);

	CHECK(s_CmdCount == 0);

	// The boundaries themselves are legal.
	cfg = MakeCfg();
	cfg.AdvSid = BTPSYNC_SID_MAX;
	cfg.Skip = BTPSYNC_SKIP_MAX;
	cfg.SyncTimeout = BTPSYNC_TIMEOUT_MIN;
	CHECK(BtPsyncCreate(&cfg));
	cfg.SyncTimeout = BTPSYNC_TIMEOUT_MAX;
	CHECK(BtPsyncCreate(&cfg));
	CHECK(s_CmdCount == 2);
}

// With the Periodic Advertiser List in use the controller ignores the SID and
// address, so values that would otherwise be refused are passed through.
void TestListModeIgnoresTheAdvertiserName(void)
{
	Reset();

	BtPsyncCfg_t cfg = MakeCfg();
	cfg.Options = BTPSYNC_OPT_USE_LIST;
	cfg.AdvSid = 0xFF;
	cfg.AdvAddrType = 0xFF;

	CHECK(BtPsyncCreate(&cfg));
	CHECK(s_CmdCount == 1);
}

void TestCancelAndTerminate(void)
{
	Reset();

	CHECK(BtPsyncCreateCancel());
	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL);
	CHECK(c != nullptr && c->ParamLen == 0);

	Reset();
	CHECK(BtPsyncTerminate(0x0123));
	c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 2);
		CHECK(c->Param[0] == 0x23 && c->Param[1] == 0x01);
	}

	// Sync_Handle is 12 bits meaningful, 0x0000 to 0x0EFF.
	Reset();
	CHECK(BtPsyncTerminate(BTPSYNC_HDL_MAX + 1) == false);
	CHECK(BtPsyncReceiveEnable(BTPSYNC_HDL_MAX + 1, true) == false);
	CHECK(s_CmdCount == 0);
}

void TestListCommands(void)
{
	Reset();

	CHECK(BtPsyncListAdd(BTPSYNC_ADDR_PUBLIC, kAddr, 5));
	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_ADD_DEV);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		// 7.8.70: Address_Type(1) Address(6) SID(1).
		CHECK(c->ParamLen == 8);
		CHECK(c->Param[0] == BTPSYNC_ADDR_PUBLIC);
		CHECK(std::memcmp(&c->Param[1], kAddr, 6) == 0);
		CHECK(c->Param[7] == 5);
	}

	Reset();
	CHECK(BtPsyncListRemove(BTPSYNC_ADDR_RANDOM, kAddr, 0));
	CHECK(FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_REMOVE_DEV) != nullptr);

	Reset();
	CHECK(BtPsyncListClear());
	c = FindCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_CLEAR);
	CHECK(c != nullptr && c->ParamLen == 0);

	Reset();
	s_ListSize = 8;
	uint8_t size = 0;
	CHECK(BtPsyncListSizeGet(&size));
	CHECK(size == 8);

	// The same name rules as Create Sync apply.
	Reset();
	CHECK(BtPsyncListAdd(2, kAddr, 0) == false);
	CHECK(BtPsyncListAdd(BTPSYNC_ADDR_PUBLIC, kAddr, BTPSYNC_SID_MAX + 1) == false);
	CHECK(BtPsyncListAdd(BTPSYNC_ADDR_PUBLIC, nullptr, 0) == false);
	CHECK(s_CmdCount == 0);
}

void TestReceiveEnableLayout(void)
{
	Reset();

	CHECK(BtPsyncReceiveEnable(0x0002, true));
	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_RECEIVE_ENABLE);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		// 7.8.88: Sync_Handle(2) Enable(1).
		CHECK(c->ParamLen == 3);
		CHECK(c->Param[0] == 0x02 && c->Param[1] == 0x00);
		CHECK(c->Param[2] == 1);
	}

	Reset();
	CHECK(BtPsyncReceiveEnable(0x0002, false));
	c = FindCmd(BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_RECEIVE_ENABLE);
	CHECK(c != nullptr && c->Param[2] == 0);
}

void TestARefusedCommandIsReported(void)
{
	Reset();
	s_FailOpCode = BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC;

	BtPsyncCfg_t cfg = MakeCfg();
	CHECK(BtPsyncCreate(&cfg) == false);
	CHECK(s_CmdCount == 1);

	Reset();
	g_BtAppData.AppDevice.pHciDev = nullptr;
	cfg = MakeCfg();
	CHECK(BtPsyncCreate(&cfg) == false);
	CHECK(BtPsyncCreateCancel() == false);
	CHECK(BtPsyncListClear() == false);
	CHECK(s_CmdCount == 0);
}

// --- events --------------------------------------------------------------

// 7.7.65.14 V1: Status(1) Sync_Handle(2) SID(1) Addr_Type(1) Addr(6) PHY(1)
// Interval(2) Clock_Accuracy(1).
void BuildEstablished(uint8_t *p, uint8_t Status, uint16_t Hdl)
{
	p[0] = Status;
	p[1] = (uint8_t)(Hdl & 0xFF);
	p[2] = (uint8_t)(Hdl >> 8);
	p[3] = 7;							// SID
	p[4] = 1;							// random address
	std::memcpy(&p[5], kAddr, 6);
	p[11] = 2;							// PHY
	p[12] = 0x50;						// interval low
	p[13] = 0x00;
	p[14] = 1;							// clock accuracy
}

void TestEstablishedIsParsed(void)
{
	uint8_t evt[19];

	Reset();
	std::memset(evt, 0, sizeof(evt));
	BuildEstablished(evt, 0, 0x0004);

	BtPsyncEvtEstablished(evt, 15, false);

	CHECK(s_App.EstablishedCount == 1);
	CHECK(s_App.Info.Status == 0);
	CHECK(s_App.Info.SyncHdl == 0x0004);
	CHECK(s_App.Info.AdvSid == 7);
	CHECK(s_App.Info.AdvAddrType == 1);
	CHECK(std::memcmp(s_App.Info.AdvAddr, kAddr, 6) == 0);
	CHECK(s_App.Info.AdvPhy == 2);
	CHECK(s_App.Info.Interval == 0x0050);
	CHECK(s_App.Info.ClockAccuracy == 1);

	// A truncated event is ignored rather than parsed past its end.
	Reset();
	BtPsyncEvtEstablished(evt, 14, false);
	CHECK(s_App.EstablishedCount == 0);

	// The V2 form needs its four extra octets before it can be trusted.
	Reset();
	BtPsyncEvtEstablished(evt, 18, true);
	CHECK(s_App.EstablishedCount == 0);
	BtPsyncEvtEstablished(evt, 19, true);
	CHECK(s_App.EstablishedCount == 1);

	// A cancelled attempt reports its status and nothing else meaningful.
	// Vol 4 Part E 7.8.68 has the controller send this after a successful
	// Create Sync Cancel, with Operation Cancelled by Host.
	Reset();
	BuildEstablished(evt, 0x44, 0);
	BtPsyncEvtEstablished(evt, 15, false);
	CHECK(s_App.EstablishedCount == 1);
	CHECK(s_App.Info.Status == 0x44);
}

// 7.7.65.15 V1: Sync_Handle(2) TX_Power(1) RSSI(1) CTE_Type(1) Data_Status(1)
// Data_Length(1) Data. V2 inserts Periodic_Event_Counter(2) and Subevent(1)
// before Data_Status.
int BuildReport(uint8_t *p, bool bV2, uint16_t Hdl, int8_t TxPwr, int8_t Rssi,
				uint8_t CteType, uint8_t DataStatus, const uint8_t *pData,
				uint8_t Len)
{
	int n = 0;

	p[n++] = (uint8_t)(Hdl & 0xFF);
	p[n++] = (uint8_t)(Hdl >> 8);
	p[n++] = (uint8_t)TxPwr;
	p[n++] = (uint8_t)Rssi;
	p[n++] = CteType;
	if (bV2)
	{
		p[n++] = 0x34;					// periodic event counter low
		p[n++] = 0x12;
		p[n++] = 0xFF;					// subevent, none
	}
	p[n++] = DataStatus;
	p[n++] = Len;
	if (Len > 0)
	{
		std::memcpy(&p[n], pData, Len);
	}

	return n + Len;
}

void TestCompleteReportIsDelivered(void)
{
	uint8_t evt[64];
	const uint8_t ad[5] = { 1, 2, 3, 4, 5 };

	Reset();
	int len = BuildReport(evt, false, 0x0001, -20, -60, 0xFF,
						  BTPSYNC_DATA_COMPLETE, ad, sizeof(ad));
	BtPsyncEvtReport(evt, len, false);

	CHECK(s_App.ReportCount == 1);
	CHECK(s_App.ReportHdl == 0x0001);
	CHECK(s_App.ReportTxPwr == -20);
	CHECK(s_App.ReportRssi == -60);
	CHECK(s_App.ReportCteType == 0xFF);
	CHECK(s_App.ReportLen == sizeof(ad));
	CHECK(std::memcmp(s_App.ReportData, ad, sizeof(ad)) == 0);

	// The V2 layout puts Data_Status three octets further along. Parsing it as
	// V1 would read the event counter as the status and deliver nothing.
	Reset();
	len = BuildReport(evt, true, 0x0001, -20, -60, 0xFF,
					  BTPSYNC_DATA_COMPLETE, ad, sizeof(ad));
	BtPsyncEvtReport(evt, len, true);
	CHECK(s_App.ReportCount == 1);
	CHECK(s_App.ReportLen == sizeof(ad));
	CHECK(std::memcmp(s_App.ReportData, ad, sizeof(ad)) == 0);
}

void TestFragmentedReportIsReassembled(void)
{
	uint8_t evt[64];
	const uint8_t a[4] = { 0xA0, 0xA1, 0xA2, 0xA3 };
	const uint8_t b[3] = { 0xB0, 0xB1, 0xB2 };
	const uint8_t c[2] = { 0xC0, 0xC1 };

	Reset();
	// Transmit power belongs to the first packet and may read 0x7F on the
	// later ones; the RSSI is the last packet received.
	BtPsyncEvtReport(evt, BuildReport(evt, false, 2, -10, -50, 0x00,
									  BTPSYNC_DATA_MORE, a, sizeof(a)), false);
	CHECK(s_App.ReportCount == 0);
	BtPsyncEvtReport(evt, BuildReport(evt, false, 2, 0x7F, -55, 0x00,
									  BTPSYNC_DATA_MORE, b, sizeof(b)), false);
	CHECK(s_App.ReportCount == 0);
	BtPsyncEvtReport(evt, BuildReport(evt, false, 2, 0x7F, -58, 0x00,
									  BTPSYNC_DATA_COMPLETE, c, sizeof(c)), false);

	CHECK(s_App.ReportCount == 1);
	CHECK(s_App.ReportLen == sizeof(a) + sizeof(b) + sizeof(c));
	CHECK(std::memcmp(s_App.ReportData, a, sizeof(a)) == 0);
	CHECK(std::memcmp(s_App.ReportData + sizeof(a), b, sizeof(b)) == 0);
	CHECK(std::memcmp(s_App.ReportData + sizeof(a) + sizeof(b), c, sizeof(c)) == 0);
	CHECK(s_App.ReportTxPwr == -10);
	CHECK(s_App.ReportRssi == -58);
}

// The terminating fragment of an abandoned reassembly is indistinguishable
// from a complete single-fragment report. Delivering it hands the application
// a truncated suffix as if it were whole advertising data.
void TestAnAbandonedReassemblyDoesNotDeliverItsTail(void)
{
	uint8_t evt[64];
	const uint8_t frag[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

	Reset();
	// Three trains start a fragmented report; only two contexts exist, so the
	// third is abandoned before it holds anything.
	for (uint16_t h = 1; h <= 3; h++)
	{
		BtPsyncEvtReport(evt, BuildReport(evt, false, h, -10, -50, 0,
										  BTPSYNC_DATA_MORE, frag, sizeof(frag)),
						 false);
	}
	CHECK(s_App.ReportCount == 0);

	// The abandoned train finishes. Nothing may be delivered for it.
	BtPsyncEvtReport(evt, BuildReport(evt, false, 3, -10, -50, 0,
									  BTPSYNC_DATA_COMPLETE, frag, sizeof(frag)),
					 false);
	CHECK(s_App.ReportCount == 0);

	// The two that kept a context still complete normally.
	BtPsyncEvtReport(evt, BuildReport(evt, false, 1, -10, -50, 0,
									  BTPSYNC_DATA_COMPLETE, frag, sizeof(frag)),
					 false);
	CHECK(s_App.ReportCount == 1);
	CHECK(s_App.ReportLen == sizeof(frag) * 2);

	// Once the abandoned train is forgotten, a fresh standalone report from it
	// is delivered again.
	BtPsyncEvtReport(evt, BuildReport(evt, false, 3, -10, -50, 0,
									  BTPSYNC_DATA_COMPLETE, frag, sizeof(frag)),
					 false);
	CHECK(s_App.ReportCount == 2);
	CHECK(s_App.ReportLen == sizeof(frag));
}

void TestTruncatedAndOverlongReportsAreDropped(void)
{
	uint8_t evt[300];
	uint8_t big[200];

	std::memset(big, 0x5A, sizeof(big));

	// Data_Status truncated: no more data is coming and what arrived is
	// incomplete, so nothing is delivered.
	Reset();
	BtPsyncEvtReport(evt, BuildReport(evt, false, 1, -10, -50, 0,
									  BTPSYNC_DATA_MORE, big, 100), false);
	BtPsyncEvtReport(evt, BuildReport(evt, false, 1, -10, -50, 0,
									  BTPSYNC_DATA_TRUNCATED, big, 10), false);
	CHECK(s_App.ReportCount == 0);

	// More than the reassembly buffer holds is dropped rather than delivered
	// as a truncation.
	Reset();
	for (int i = 0; i < 3; i++)
	{
		BtPsyncEvtReport(evt, BuildReport(evt, false, 1, -10, -50, 0,
										  BTPSYNC_DATA_MORE, big, sizeof(big)),
						 false);
	}
	BtPsyncEvtReport(evt, BuildReport(evt, false, 1, -10, -50, 0,
									  BTPSYNC_DATA_COMPLETE, big, 10), false);
	CHECK(s_App.ReportCount == 0);

	// Data_Length is controller supplied. A report claiming more than the
	// event holds must not be read past its end.
	Reset();
	int len = BuildReport(evt, false, 1, -10, -50, 0, BTPSYNC_DATA_COMPLETE,
						  big, 100);
	BtPsyncEvtReport(evt, len - 10, false);
	CHECK(s_App.ReportCount == 0);

	// A header shorter than the fixed part is ignored.
	Reset();
	BtPsyncEvtReport(evt, 6, false);
	BtPsyncEvtReport(evt, 9, true);
	CHECK(s_App.ReportCount == 0);
}

void TestSyncLostClearsTheTrain(void)
{
	uint8_t evt[64];
	const uint8_t frag[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

	Reset();
	BtPsyncEvtReport(evt, BuildReport(evt, false, 5, -10, -50, 0,
									  BTPSYNC_DATA_MORE, frag, sizeof(frag)),
					 false);

	uint8_t lost[2] = { 5, 0 };
	BtPsyncEvtLost(lost, sizeof(lost));

	CHECK(s_App.LostCount == 1);
	CHECK(s_App.LostHdl == 5);

	// A later train given the same handle does not inherit the fragment.
	BtPsyncEvtReport(evt, BuildReport(evt, false, 5, -20, -60, 0,
									  BTPSYNC_DATA_COMPLETE, frag, sizeof(frag)),
					 false);
	CHECK(s_App.ReportCount == 1);
	CHECK(s_App.ReportLen == sizeof(frag));

	// A short event is ignored.
	Reset();
	BtPsyncEvtLost(lost, 1);
	CHECK(s_App.LostCount == 0);
}

// A successful Terminate Sync destroys the handle, so anything held for it
// goes too.
void TestTerminateClearsTheTrain(void)
{
	uint8_t evt[64];
	const uint8_t frag[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

	Reset();
	BtPsyncEvtReport(evt, BuildReport(evt, false, 6, -10, -50, 0,
									  BTPSYNC_DATA_MORE, frag, sizeof(frag)),
					 false);
	CHECK(BtPsyncTerminate(6));

	BtPsyncEvtReport(evt, BuildReport(evt, false, 6, -20, -60, 0,
									  BTPSYNC_DATA_COMPLETE, frag, sizeof(frag)),
					 false);
	CHECK(s_App.ReportCount == 1);
	CHECK(s_App.ReportLen == sizeof(frag));
}

} // namespace

// Strong definitions beat the weak defaults in bt_psync.cpp.
void BtPsyncEstablished(const BtPsyncInfo_t * const pInfo)
{
	s_App.EstablishedCount++;
	if (pInfo != nullptr)
	{
		s_App.Info = *pInfo;
	}
}

void BtPsyncReport(uint16_t SyncHdl, int8_t TxPwr, int8_t Rssi, uint8_t CteType,
				   const uint8_t *pData, uint16_t Len)
{
	s_App.ReportCount++;
	s_App.ReportHdl = SyncHdl;
	s_App.ReportTxPwr = TxPwr;
	s_App.ReportRssi = Rssi;
	s_App.ReportCteType = CteType;
	s_App.ReportLen = Len;
	if (pData != nullptr && Len <= sizeof(s_App.ReportData))
	{
		std::memcpy(s_App.ReportData, pData, Len);
	}
}

void BtPsyncLost(uint16_t SyncHdl)
{
	s_App.LostCount++;
	s_App.LostHdl = SyncHdl;
}

int main()
{
	TestCreateSyncLayout();
	TestCreateSyncRangesRefusedLocally();
	TestListModeIgnoresTheAdvertiserName();
	TestCancelAndTerminate();
	TestListCommands();
	TestReceiveEnableLayout();
	TestARefusedCommandIsReported();
	TestEstablishedIsParsed();
	TestCompleteReportIsDelivered();
	TestFragmentedReportIsReassembled();
	TestAnAbandonedReassemblyDoesNotDeliverItsTail();
	TestTruncatedAndOverlongReportsAreDropped();
	TestSyncLostClearsTheTrain();
	TestTerminateClearsTheTrain();

	if (s_Failures == 0)
	{
		std::printf("Periodic sync host tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Periodic sync host tests: %d failure(s), %d checks\n",
				s_Failures, s_Checks);

	return 1;
}
