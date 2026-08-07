// Command-level coverage for the GAP HCI scan/initiate path. The capture
// device records the exact standard HCI commands and parameter bytes selected
// from a reported controller capability set.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
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

CapturedCmd s_Cmds[16];
int s_CmdCount = 0;
uint8_t s_SupportedCommands[64];
uint8_t s_LeFeatures[8];

// Controlled local identity returned by the BtSmpLocalAddrGet override below.
uint8_t s_LocalType = 0;
uint8_t s_LocalAddr[6] = {};

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

void SetDefaultCapabilities()
{
	std::memset(s_SupportedCommands, 0, sizeof(s_SupportedCommands));
	std::memset(s_LeFeatures, 0, sizeof(s_LeFeatures));

	SetCommand(BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS);
	SetCommand(BT_HCI_CAP_CMD_LE_SET_SCAN_PARAMETERS);
	SetCommand(BT_HCI_CAP_CMD_LE_SET_SCAN_ENABLE);
	SetCommand(BT_HCI_CAP_CMD_LE_CREATE_CONNECTION);
	SetCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS);
	SetCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE);
	SetCommand(BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION);

	s_LeFeatures[BT_HCI_CAP_LE_FEATURE_CODED_PHY >> 3] |=
		static_cast<uint8_t>(1U << (BT_HCI_CAP_LE_FEATURE_CODED_PHY & 7U));
}

void ClearCaptured()
{
	s_CmdCount = 0;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
}

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
	ClearCaptured();
	SetDefaultCapabilities();
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
	cfg.Type           = BTSCAN_TYPE_PASSIVE;
	cfg.Param.Phy      = BT_GAP_PHY_1MBITS;
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

BtGapPeerAddr_t MakePeer()
{
	BtGapPeerAddr_t peer = {};
	peer.Type = BTADDR_TYPE_PUBLIC;
	uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	std::memcpy(peer.Addr, addr, sizeof(addr));
	return peer;
}

// Offsets into SET_EXT_SCAN_PARAM (single PHY): OwnAddrType(1),
// FilterPolicy(1), ScanPhys(1), ScanType(1), Interval(2), Window(2).
constexpr size_t kExtScanOwnAddr = 0;
constexpr size_t kExtScanPhys = 2;
constexpr size_t kExtScanInterval = 4;
constexpr size_t kExtScanWindow = 6;

// Offsets into SET_SCAN_PARAM: ScanType(1), Interval(2), Window(2),
// OwnAddrType(1), FilterPolicy(1).
constexpr size_t kLegacyScanType = 0;
constexpr size_t kLegacyScanInterval = 1;
constexpr size_t kLegacyScanWindow = 3;
constexpr size_t kLegacyScanOwnAddr = 5;

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
		CHECK(sp->Param[kExtScanPhys] == BT_GAP_PHY_CODED);
	}
}

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
		CHECK(sp->Param[kExtScanOwnAddr] == BTADDR_TYPE_PUBLIC);
	}
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_RANDOM_ADDR) == nullptr);
}

void TestScanRandomIdentity()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);

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
		CHECK(sp->Param[kExtScanOwnAddr] == BTADDR_TYPE_RAND);
	}
}

void TestScanInvalidRandomIdentity()
{
	uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM) == nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_SCAN_PARAM) == nullptr);
}

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

	uint8_t oneBitSet[6] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0xC0 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, oneBitSet);
	cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);

	uint8_t oneBitClear[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
	Setup(BTADDR_TYPE_RANDOM_STATIC, oneBitClear);
	cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == true);
}

void TestScanClampLower()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(1, 1);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(GetLe16(sp->Param + kExtScanInterval) == 0x0004);
		CHECK(GetLe16(sp->Param + kExtScanWindow) == 0x0004);
	}
}

void TestScanClampUpper()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(12000, 12000);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		CHECK(GetLe16(sp->Param + kExtScanInterval) == 0x4000);
		CHECK(GetLe16(sp->Param + kExtScanWindow) == 0x4000);
	}
}

void TestScanClampWindowLeInterval()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 200);
	CHECK(BtGapScanInit(&cfg) == true);

	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM);
	CHECK(sp != nullptr);
	if (sp != nullptr)
	{
		uint16_t interval = GetLe16(sp->Param + kExtScanInterval);
		uint16_t window   = GetLe16(sp->Param + kExtScanWindow);
		CHECK(interval == 160);
		CHECK(window == 160);
		CHECK(window <= interval);
	}
}

void TestLegacyScanFallback()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS);
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 50);
	cfg.Type = BTSCAN_TYPE_ACTIVE;
	CHECK(BtGapScanInit(&cfg));
	const CapturedCmd *sp = FindCmd(BT_HCI_CMD_CTLR_SET_SCAN_PARAM);
	CHECK(sp != nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM) == nullptr);
	if (sp != nullptr)
	{
		CHECK(sp->ParamLen == 7);
		CHECK(sp->Param[kLegacyScanType] == 1);
		CHECK(GetLe16(sp->Param + kLegacyScanInterval) == 160);
		CHECK(GetLe16(sp->Param + kLegacyScanWindow) == 80);
		CHECK(sp->Param[kLegacyScanOwnAddr] == BTADDR_TYPE_PUBLIC);
	}

	ClearCaptured();
	CHECK(BtGapScanStart(nullptr, 0));
	const CapturedCmd *se = FindCmd(BT_HCI_CMD_CTLR_SET_SCAN_ENABLE);
	CHECK(se != nullptr);
	if (se != nullptr)
	{
		CHECK(se->ParamLen == 2);
		CHECK(se->Param[0] == 1);
		CHECK(se->Param[1] == 1);
	}

	ClearCaptured();
	BtGapScanStop();
	se = FindCmd(BT_HCI_CMD_CTLR_SET_SCAN_ENABLE);
	CHECK(se != nullptr);
	if (se != nullptr)
	{
		CHECK(se->Param[0] == 0);
		CHECK(se->Param[1] == 0);
	}
}

void TestScanCommandRefusal()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS);
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE);
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_SCAN_ENABLE);

	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_SCAN_PARAM) == nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM) == nullptr);
}

void TestCodedScanChecks()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	cfg.Param.Phy = BT_GAP_PHY_CODED;

	s_LeFeatures[BT_HCI_CAP_LE_FEATURE_CODED_PHY >> 3] &=
		static_cast<uint8_t>(~(1U << (BT_HCI_CAP_LE_FEATURE_CODED_PHY & 7U)));
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(s_CmdCount == 0);

	Setup(BTADDR_TYPE_PUBLIC, addr);
	ClearCommand(BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE);
	CHECK(BtGapScanInit(&cfg) == false);
	CHECK(s_CmdCount == 0);
}

void TestExtendedScanEnable()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg));

	ClearCaptured();
	CHECK(BtGapScanStart(nullptr, 0));
	const CapturedCmd *se = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE);
	CHECK(se != nullptr);
	if (se != nullptr)
	{
		CHECK(se->ParamLen == 6);
		CHECK(se->Param[0] == 1);
		CHECK(se->Param[1] == 1);
	}

	ClearCaptured();
	BtGapScanStop();
	se = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE);
	CHECK(se != nullptr);
	if (se != nullptr)
	{
		CHECK(se->Param[0] == 0);
	}
}

void TestExtendedCreateConnection1M()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 50);
	CHECK(BtGapScanInit(&cfg));
	ClearCaptured();

	BtGapPeerAddr_t peer = MakePeer();
	BtGapConnParams_t params = MakeConnParams();
	CHECK(BtGapConnect(&peer, &params));

	const CapturedCmd *cc = FindCmd(BT_HCI_CMD_CTLR_EXT_CREATE_CONN);
	CHECK(cc != nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_CREATE_CONN) == nullptr);
	if (cc != nullptr)
	{
		CHECK(cc->ParamLen == 26);
		CHECK(cc->Param[0] == 0);
		CHECK(cc->Param[1] == BTADDR_TYPE_PUBLIC);
		CHECK(cc->Param[2] == peer.Type);
		CHECK(std::memcmp(cc->Param + 3, peer.Addr, 6) == 0);
		CHECK(cc->Param[9] == BT_GAP_PHY_1MBITS);
		CHECK(GetLe16(cc->Param + 10) == 160);
		CHECK(GetLe16(cc->Param + 12) == 80);
		CHECK(GetLe16(cc->Param + 14) == 24);
		CHECK(GetLe16(cc->Param + 16) == 40);
		CHECK(GetLe16(cc->Param + 18) == 0);
		CHECK(GetLe16(cc->Param + 20) == 400);
	}
}

void TestLegacyCreateConnectionFallback()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	ClearCommand(BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 50);
	CHECK(BtGapScanInit(&cfg));
	ClearCaptured();

	BtGapPeerAddr_t peer = MakePeer();
	BtGapConnParams_t params = MakeConnParams();
	CHECK(BtGapConnect(&peer, &params));
	const CapturedCmd *cc = FindCmd(BT_HCI_CMD_CTLR_CREATE_CONN);
	CHECK(cc != nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_EXT_CREATE_CONN) == nullptr);
	if (cc != nullptr)
	{
		CHECK(cc->ParamLen == 25);
		CHECK(GetLe16(cc->Param) == 160);
		CHECK(GetLe16(cc->Param + 2) == 80);
	}
}

void TestCodedCreateConnection()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	cfg.Param.Phy = BT_GAP_PHY_CODED;
	CHECK(BtGapScanInit(&cfg));
	ClearCaptured();

	BtGapPeerAddr_t peer = MakePeer();
	BtGapConnParams_t params = MakeConnParams();
	CHECK(BtGapConnect(&peer, &params));
	const CapturedCmd *cc = FindCmd(BT_HCI_CMD_CTLR_EXT_CREATE_CONN);
	CHECK(cc != nullptr);
	if (cc != nullptr)
	{
		CHECK(cc->ParamLen == 26);
		CHECK(cc->Param[9] == BT_GAP_PHY_CODED);
	}
}

void TestCodedCreateConnectionRefusalBeforeRandomAddress()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	cfg.Param.Phy = BT_GAP_PHY_CODED;
	CHECK(BtGapScanInit(&cfg));
	ClearCaptured();
	ClearCommand(BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION);

	BtGapPeerAddr_t peer = MakePeer();
	BtGapConnParams_t params = MakeConnParams();
	CHECK(BtGapConnect(&peer, &params) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_RANDOM_ADDR) == nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_CREATE_CONN) == nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_EXT_CREATE_CONN) == nullptr);
}

void TestCreateConnectionCommandRefusal()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg));
	ClearCaptured();
	ClearCommand(BT_HCI_CAP_CMD_LE_CREATE_CONNECTION);
	ClearCommand(BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION);

	BtGapPeerAddr_t peer = MakePeer();
	BtGapConnParams_t params = MakeConnParams();
	CHECK(BtGapConnect(&peer, &params) == false);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_CREATE_CONN) == nullptr);
	CHECK(FindCmd(BT_HCI_CMD_CTLR_EXT_CREATE_CONN) == nullptr);
}

void TestInvalidConnectionParametersBeforeCommands()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	BtGapScanCfg_t cfg = MakeScanCfg(100, 100);
	CHECK(BtGapScanInit(&cfg));
	ClearCaptured();

	BtGapPeerAddr_t peer = MakePeer();
	BtGapConnParams_t params = MakeConnParams();
	params.Timeout = 10;
	CHECK(BtGapConnect(&peer, &params) == false);
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

bool BtHciCapabilitiesRead(BtHciDevice_t * const pDev,
	BtHciCapabilities_t * const pCapabilities)
{
	if (pDev == nullptr || pDev->Command == nullptr || pCapabilities == nullptr)
	{
		return false;
	}

	std::memset(pCapabilities, 0, sizeof(*pCapabilities));
	pCapabilities->Valid = BT_HCI_CAP_VALID_COMMANDS |
		BT_HCI_CAP_VALID_LE_FEATURES;
	std::memcpy(pCapabilities->SupportedCommands, s_SupportedCommands,
		sizeof(s_SupportedCommands));
	std::memcpy(pCapabilities->LeFeatures, s_LeFeatures,
		sizeof(s_LeFeatures));
	return true;
}

const BtHciCapabilities_t *BtHciCapabilitiesForDeviceGet(
	const BtHciDevice_t *)
{
	return nullptr;
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
	TestLegacyScanFallback();
	TestScanCommandRefusal();
	TestCodedScanChecks();
	TestExtendedScanEnable();
	TestExtendedCreateConnection1M();
	TestLegacyCreateConnectionFallback();
	TestCodedCreateConnection();
	TestCodedCreateConnectionRefusalBeforeRandomAddress();
	TestCreateConnectionCommandRefusal();
	TestInvalidConnectionParametersBeforeCommands();

	if (s_Failures != 0)
	{
		std::printf("GAP HCI host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GAP HCI host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
