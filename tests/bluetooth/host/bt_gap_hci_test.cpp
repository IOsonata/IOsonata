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

// Return parameters the next captured command answers with, and the HCI
// status it answers. A command that reads something back needs both, and a
// caller has to be seen refusing a controller error as well as a bad value.
uint8_t s_Ret[8] = {};
uint8_t s_RetLen = 0;
uint8_t s_CmdStatus = 0;

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

	if (pRet != nullptr && RetLen > 0 && s_RetLen > 0)
	{
		std::memcpy(pRet, s_Ret, RetLen < s_RetLen ? RetLen : s_RetLen);
	}

	return s_CmdStatus;
}

BtHciDevice_t s_Dev;

void Setup(uint8_t LocalType, const uint8_t LocalAddr[6])
{
	s_CmdCount = 0;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	std::memset(s_Ret, 0, sizeof(s_Ret));
	s_RetLen = 0;
	s_CmdStatus = 0;
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

// --- Link procedures on an established connection ---
//
// LE Read PHY 7.8.47, LE Set PHY 7.8.49 and LE Set Data Length 7.8.33. The
// same capture device records what would go on air.

// Offsets into the SET_PHY parameter block:
// ConnHdl(2) AllPhys(1) TxPhys(1) RxPhys(1) PhyOptions(2).
constexpr size_t kPhyConnHdl = 0;
constexpr size_t kPhyAllPhys = 2;
constexpr size_t kPhyTxPhys = 3;
constexpr size_t kPhyRxPhys = 4;
constexpr size_t kPhyOptions = 5;

// Offsets into the SET_DATA_LEN parameter block:
// ConnHdl(2) TxOctets(2) TxTime(2).
constexpr size_t kDataLenConnHdl = 0;
constexpr size_t kDataLenOctets = 2;
constexpr size_t kDataLenTime = 4;

void SetReadPhyReturn(uint16_t ConnHdl, uint8_t TxPhy, uint8_t RxPhy)
{
	s_Ret[0] = (uint8_t)(ConnHdl & 0xFF);
	s_Ret[1] = (uint8_t)(ConnHdl >> 8);
	s_Ret[2] = TxPhy;
	s_Ret[3] = RxPhy;
	s_RetLen = 4;
}

// 7.8.47 enumerates the PHY 1, 2, 3 while the API uses the bit mask LE Set PHY
// takes. A caller comparing a read against a mask it set would see every read
// as a mismatch if the two encodings were mixed.
void TestReadPhyEnumToMask()
{
	uint8_t addr[6] = {};
	uint8_t tx = 0;
	uint8_t rx = 0;

	Setup(BTADDR_TYPE_PUBLIC, addr);
	SetReadPhyReturn(0x0010, 1, 3);

	CHECK(BtGapReadPhy(0x0010, &tx, &rx) == true);
	CHECK(tx == BT_GAP_PHY_1MBITS);
	CHECK(rx == BT_GAP_PHY_CODED);

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_READ_PHY);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 2);
		CHECK(GetLe16(c->Param) == 0x0010);
	}

	Setup(BTADDR_TYPE_PUBLIC, addr);
	SetReadPhyReturn(0x0010, 2, 2);
	CHECK(BtGapReadPhy(0x0010, &tx, &rx) == true);
	CHECK(tx == BT_GAP_PHY_2MBITS);
	CHECK(rx == BT_GAP_PHY_2MBITS);
}

// A value outside the enumeration would map to a mask bit that means a
// different PHY or to none at all, so it is refused rather than converted.
void TestReadPhyRejectsUnknownValue()
{
	uint8_t addr[6] = {};
	uint8_t tx = 0xAA;
	uint8_t rx = 0xAA;

	Setup(BTADDR_TYPE_PUBLIC, addr);
	SetReadPhyReturn(0x0010, 0, 1);
	CHECK(BtGapReadPhy(0x0010, &tx, &rx) == false);

	Setup(BTADDR_TYPE_PUBLIC, addr);
	SetReadPhyReturn(0x0010, 1, 4);
	CHECK(BtGapReadPhy(0x0010, &tx, &rx) == false);

	// Nothing was written on either refusal.
	CHECK(tx == 0xAA);
	CHECK(rx == 0xAA);
}

void TestReadPhyRefusesBadArguments()
{
	uint8_t addr[6] = {};
	uint8_t phy = 0;

	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapReadPhy(0x0010, nullptr, &phy) == false);
	CHECK(BtGapReadPhy(0x0010, &phy, nullptr) == false);
	CHECK(s_CmdCount == 0);

	// A controller error is a refusal, not a PHY of whatever the buffer held.
	Setup(BTADDR_TYPE_PUBLIC, addr);
	SetReadPhyReturn(0x0010, 1, 1);
	s_CmdStatus = 0x12;
	uint8_t tx = 0;
	uint8_t rx = 0;
	CHECK(BtGapReadPhy(0x0010, &tx, &rx) == false);
}

// An empty mask is no preference, which 7.8.49 spells in ALL_PHYS. Sent in the
// mask instead it would be a request for no PHY at all.
void TestSetPhyEmptyMaskGoesToAllPhys()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	CHECK(BtGapSetPhy(0x0021, 0, 0, BT_GAP_PHY_OPT_NONE) == true);

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_SET_PHY);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 7);
		CHECK(GetLe16(c->Param + kPhyConnHdl) == 0x0021);
		CHECK(c->Param[kPhyAllPhys] == 0x03);
		CHECK(c->Param[kPhyTxPhys] == 0);
		CHECK(c->Param[kPhyRxPhys] == 0);
	}

	// One direction named, the other left to the controller.
	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapSetPhy(0x0021, BT_GAP_PHY_2MBITS, 0, BT_GAP_PHY_OPT_NONE) == true);
	c = FindCmd(BT_HCI_CMD_CTLR_SET_PHY);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Param[kPhyAllPhys] == 0x02);
		CHECK(c->Param[kPhyTxPhys] == BT_GAP_PHY_2MBITS);
	}
}

void TestSetPhyEmitsMaskAndOptions()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	CHECK(BtGapSetPhy(0x0034, BT_GAP_PHY_1MBITS | BT_GAP_PHY_CODED,
					  BT_GAP_PHY_CODED, BT_GAP_PHY_OPT_REQUIRE_S8) == true);

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_SET_PHY);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Param[kPhyAllPhys] == 0);
		CHECK(c->Param[kPhyTxPhys] ==
			  (BT_GAP_PHY_1MBITS | BT_GAP_PHY_CODED));
		CHECK(c->Param[kPhyRxPhys] == BT_GAP_PHY_CODED);
		CHECK(GetLe16(c->Param + kPhyOptions) == BT_GAP_PHY_OPT_REQUIRE_S8);
	}
}

void TestSetPhyRefusesOutOfRange()
{
	uint8_t addr[6] = {};

	// A bit outside the three PHYs 7.8.49 defines.
	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapSetPhy(0x0034, 0x08, BT_GAP_PHY_1MBITS,
					  BT_GAP_PHY_OPT_NONE) == false);
	CHECK(s_CmdCount == 0);

	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapSetPhy(0x0034, BT_GAP_PHY_1MBITS, 0x80,
					  BT_GAP_PHY_OPT_NONE) == false);
	CHECK(s_CmdCount == 0);

	// A coding option past the last one defined.
	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapSetPhy(0x0034, BT_GAP_PHY_CODED, BT_GAP_PHY_CODED,
					  BT_GAP_PHY_OPT_MAX + 1) == false);
	CHECK(s_CmdCount == 0);

	// A controller that refuses the request is reported as a refusal.
	Setup(BTADDR_TYPE_PUBLIC, addr);
	s_CmdStatus = 0x0C;
	CHECK(BtGapSetPhy(0x0034, BT_GAP_PHY_2MBITS, BT_GAP_PHY_2MBITS,
					  BT_GAP_PHY_OPT_NONE) == false);
}

void TestDataLenEmitsRequest()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	CHECK(BtGapSetDataLength(0x0042, 251, 2120) == true);

	const CapturedCmd *c = FindCmd(BT_HCI_CMD_CTLR_SET_DATA_LEN);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->ParamLen == 6);
		CHECK(GetLe16(c->Param + kDataLenConnHdl) == 0x0042);
		CHECK(GetLe16(c->Param + kDataLenOctets) == 251);
		CHECK(GetLe16(c->Param + kDataLenTime) == 2120);
	}
}

// 7.8.33 gives both parameters a closed range. A value outside it reaches the
// controller as Invalid HCI Command Parameters, which a caller then has to map
// back to a parameter it already knew was wrong.
void TestDataLenRefusesOutOfRange()
{
	uint8_t addr[6] = {};

	const struct {
		uint16_t Octets;
		uint16_t Time;
	} bad[] = {
		{ BT_GAP_DATA_LEN_OCTETS_MIN - 1, BT_GAP_DATA_LEN_TIME_MIN },
		{ BT_GAP_DATA_LEN_OCTETS_MAX + 1, BT_GAP_DATA_LEN_TIME_MIN },
		{ BT_GAP_DATA_LEN_OCTETS_MIN, BT_GAP_DATA_LEN_TIME_MIN - 1 },
		{ BT_GAP_DATA_LEN_OCTETS_MIN, BT_GAP_DATA_LEN_TIME_MAX + 1 },
		{ 0, 0 },
	};

	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
	{
		Setup(BTADDR_TYPE_PUBLIC, addr);
		CHECK(BtGapSetDataLength(0x0042, bad[i].Octets, bad[i].Time) == false);
		CHECK(s_CmdCount == 0);
	}

	// Both ends of the range are accepted.
	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapSetDataLength(0x0042, BT_GAP_DATA_LEN_OCTETS_MIN,
							 BT_GAP_DATA_LEN_TIME_MIN) == true);
	Setup(BTADDR_TYPE_PUBLIC, addr);
	CHECK(BtGapSetDataLength(0x0042, BT_GAP_DATA_LEN_OCTETS_MAX,
							 BT_GAP_DATA_LEN_TIME_MAX) == true);
}

// Every one of the three refuses without an HCI device rather than following a
// null pointer.
void TestLinkProceduresWithoutDevice()
{
	uint8_t addr[6] = {};
	uint8_t tx = 0;
	uint8_t rx = 0;

	Setup(BTADDR_TYPE_PUBLIC, addr);
	g_BtAppData.AppDevice.pHciDev = nullptr;

	CHECK(BtGapReadPhy(0x0010, &tx, &rx) == false);
	CHECK(BtGapSetPhy(0x0010, BT_GAP_PHY_1MBITS, BT_GAP_PHY_1MBITS,
					  BT_GAP_PHY_OPT_NONE) == false);
	CHECK(BtGapSetDataLength(0x0010, 251, 2120) == false);
	CHECK(s_CmdCount == 0);
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
	TestReadPhyEnumToMask();
	TestReadPhyRejectsUnknownValue();
	TestReadPhyRefusesBadArguments();
	TestSetPhyEmptyMaskGoesToAllPhys();
	TestSetPhyEmitsMaskAndOptions();
	TestSetPhyRefusesOutOfRange();
	TestDataLenEmitsRequest();
	TestDataLenRefusesOutOfRange();
	TestLinkProceduresWithoutDevice();

	if (s_Failures != 0)
	{
		std::printf("GAP HCI host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GAP HCI host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
