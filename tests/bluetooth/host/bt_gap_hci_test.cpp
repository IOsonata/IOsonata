// Command-level coverage for the GAP HCI scan/initiate path: the own-address
// resolution (G1), the static random address validity check, and the scan
// interval/window clamping (G4). BtHciCommand dispatches through the device's
// Command function pointer, so a capture device records the opcode and the
// packed parameter bytes each call emits. No controller is involved; the tests
// assert on what would be sent on air.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_app.h"

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
	uint8_t  ParamLen;
};

CapturedCmd s_Cmds[8];
int s_CmdCount = 0;

// Controlled local identity returned by the BtSmpLocalAddrGet override below.
uint8_t s_LocalType = 0;
uint8_t s_LocalAddr[6] = {};

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
					   uint8_t ParamLen, void *, uint8_t)
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
	return 0;		// HCI success
}

BtHciDevice_t s_Dev;

void Setup(uint8_t LocalType, const uint8_t LocalAddr[6])
{
	s_CmdCount = 0;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = CaptureCommand;
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	s_LocalType = LocalType;
	std::memcpy(s_LocalAddr, LocalAddr, 6);
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

uint16_t GetLe16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

BtGapScanCfg_t MakeScanCfg(uint32_t Interval, uint32_t Duration)
{
	BtGapScanCfg_t cfg;
	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Type          = BTSCAN_TYPE_PASSIVE;
	cfg.Param.Phy     = BT_GAP_PHY_1MBITS;
	cfg.Param.Interval = Interval;
	cfg.Param.Duration = Duration;
	return cfg;
}

BtGapConnParams_t MakeConnParams()
{
	BtGapConnParams_t params = {};
	params.IntervalMin = 30.0f;
	params.IntervalMax = 50.0f;
	params.Latency = 0;
	params.Timeout = 4000;
	return params;
}

// Offsets into the SET_EXT_SCAN_PARAM parameter block (single PHY):
// OwnAddrType(1) FilterPolicy(1) ScanPhys(1) ScanType(1) Interval(2) Window(2).
constexpr size_t kScanOwnAddr = 0;
constexpr size_t kScanPhys = 2;
constexpr size_t kScanInterval = 4;
constexpr size_t kScanWindow = 6;

void TestInvalidArguments()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	CHECK(BtGapScanInit(nullptr) == false);
	CHECK(s_CmdCount == 0);

	BtGapPeerAddr_t peer = {};
	BtGapConnParams_t params = MakeConnParams();
	CHECK(BtGapConnect(nullptr, &params) == false);
	CHECK(BtGapConnect(&peer, nullptr) == false);
	CHECK(s_CmdCount == 0);
}

void TestScanPhyValidation()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	cfg.Param.Phy = 0;
	CHECK(BtGapScanInit(&cfg) == false);
	cfg.Param.Phy = BT_GAP_PHY_2MBITS;
	CHECK(BtGapScanInit(&cfg) == false);
	cfg.Param.Phy = BT_GAP_PHY_1MBITS | BT_GAP_PHY_CODED;
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	cfg.Param.Phy = BT_GAP_PHY_CODED;
	CHECK(BtGapScanInit(&cfg) == true);
	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(sp->ParamLen == 8);
		CHECK(sp->Param[kScanPhys] == BT_GAP_PHY_CODED);
	}
}

// ---- G1: own-address resolution ------------------------------------------

void TestScanPublicIdentity()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(sp->Param[kScanOwnAddr] == BTADDR_TYPE_PUBLIC);
	}
	// A public identity needs no controller random address.
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_RANDOM_ADDR) == nullptr);
}

void TestScanRandomIdentity()
{
	// Static random address: two most significant bits of the MSB are 1.
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);

	// The controller random address is programmed before scanning.
	const CapturedCmd *ra = FindCmd(BT_HCI_CMD_CTLR_SET_RANDOM_ADDR);
	CHECK(ra != nullptr);
	if (ra != nullptr)
	{
		CHECK(ra->ParamLen == 6);
		CHECK(std::memcmp(ra->Param, addr, 6) == 0);
	}
	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(sp->Param[kScanOwnAddr] == BTADDR_TYPE_RAND);
	}
}

void TestScanInvalidRandomIdentity()
{
	// MSB 0x06: top two bits are not 0b11, so not a valid static random address.
	uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	// Scan init fails rather than scanning as Random with a bad address.
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM) == nullptr);
}

// The two type bits are not the whole rule: the 46 bit random part must hold
// at least one 0 and at least one 1 (Vol 6 Part B 1.3.2.1). An identity left at
// the all-zero default with only the type bits forced on, or one left all ones,
// passes the bit pattern check but is not a usable address.
void TestScanDegenerateRandomIdentity()
{
	uint8_t allZero[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, allZero);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM) == nullptr);

	uint8_t allOne[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	Setup(BTADDR_TYPE_RANDOM_STATIC, allOne);
	cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM) == nullptr);

	// One bit away from each degenerate case is valid again.
	uint8_t oneBitSet[6] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0xC0 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, oneBitSet);
	cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);

	uint8_t oneBitClear[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
	Setup(BTADDR_TYPE_RANDOM_STATIC, oneBitClear);
	cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);
}

// ---- G4: interval/window clamping ----------------------------------------

void TestScanClampLower()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	// 1 ms encodes to ~1 (0.625 ms units), below the 0x0004 minimum.
	BtGapScanCfg_t cfg = MakeScanCfg(1, 1);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(GetLe16(sp->Param + kScanInterval) == 0x0004);
		CHECK(GetLe16(sp->Param + kScanWindow) == 0x0004);
	}
}

void TestScanClampUpper()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	// 12000 ms encodes to 19200 (0x4B00), above the 0x4000 maximum.
	BtGapScanCfg_t cfg = MakeScanCfg(12000, 12000);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(GetLe16(sp->Param + kScanInterval) == 0x4000);
		CHECK(GetLe16(sp->Param + kScanWindow) == 0x4000);
	}
}

void TestScanClampWindowLeInterval()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	// Interval 100 ms -> 160, window 200 ms -> 320: window is clamped down to
	// the interval.
	BtGapScanCfg_t cfg = MakeScanCfg(100, 200);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		uint16_t interval = GetLe16(sp->Param + kScanInterval);
		uint16_t window   = GetLe16(sp->Param + kScanWindow);
		CHECK(interval == 160);
		CHECK(window == 160);
		CHECK(window <= interval);
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
	TestInvalidArguments();
	TestScanPhyValidation();
	TestScanPublicIdentity();
	TestScanRandomIdentity();
	TestScanInvalidRandomIdentity();
	TestScanDegenerateRandomIdentity();
	TestScanClampLower();
	TestScanClampUpper();
	TestScanClampWindowLeInterval();

	if (s_Failures != 0)
	{
		std::printf("GAP HCI host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GAP HCI host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
