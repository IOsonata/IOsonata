// Command-level coverage for the advertising HCI path: the own-address type
// selection and static random address validation (G2), and the manufacturer
// company identifier byte order (G5). BtHciCommand dispatches through the
// device Command pointer, so a capture device records the opcode and packed
// parameter bytes each call emits. No controller is involved.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"

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

uint8_t s_LocalType = 0;
uint8_t s_LocalAddr[6] = {};

// Opcode the stub controller rejects, and the Enable byte it rejects it for
// (0 disable, 1 enable, 0xFF either). Lets a test drive the paths where a
// command comes back with an error status.
uint16_t s_FailOpCode = 0;
uint8_t  s_FailEnable = 0xFF;

uint8_t CaptureCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
					   uint8_t ParamLen, void *, uint8_t)
{
	if (s_CmdCount < (int)(sizeof(s_Cmds) / sizeof(s_Cmds[0])))
	{
		CapturedCmd &c = s_Cmds[s_CmdCount++];
		c.OpCode   = OpCode;
		c.ParamLen = ParamLen;
		// ParamLen is uint8_t (max 255) and Param is 264 bytes, so any
		// parameter block fits; no size check needed.
		if (pParam != nullptr)
		{
			std::memcpy(c.Param, pParam, ParamLen);
		}
	}

	if (s_FailOpCode != 0 && OpCode == s_FailOpCode)
	{
		// LE Set Extended Advertising Enable starts with the Enable octet.
		const uint8_t *p = (const uint8_t *)pParam;
		if (s_FailEnable == 0xFF || (p != nullptr && ParamLen >= 1 && p[0] == s_FailEnable))
		{
			return 0x0C;		// Command Disallowed
		}
	}

	return 0;		// HCI success
}

BtHciDevice_t s_Dev;

void Setup(uint8_t LocalType, const uint8_t LocalAddr[6])
{
	s_CmdCount = 0;
	s_FailOpCode = 0;
	s_FailEnable = 0xFF;
	std::memset(s_Cmds, 0, sizeof(s_Cmds));
	std::memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = CaptureCommand;
	g_BtAppData.AppDevice.pHciDev = &s_Dev;
	g_BtAppData.State = BTAPP_STATE_IDLE;
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

BtAppCfg_t MakePeripheralCfg()
{
	BtAppCfg_t cfg;
	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Role        = BTAPP_ROLE_PERIPHERAL;
	cfg.pDevName    = "Test";
	cfg.Appearance  = BT_APPEAR_UNKNOWN_GENERIC;
	cfg.AdvInterval = 100;
	cfg.AdvTimeout  = 0;
	cfg.VendorId    = 0x1234;
	return cfg;
}

// SET_EXT_ADV_PARAM parameter layout: AdvHandle(1) EvtProp(2) PrimIntMin(3)
// PrimIntMax(3) PrimChanMap(1) OwnAddrType(1) ...
constexpr size_t kAdvOwnAddr = 10;

// ---- G2: own-address type and static random validation -------------------

void TestAdvPublicIdentity()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kAdvOwnAddr] == BTADDR_TYPE_PUBLIC);
	}
	// A public identity advertises with no per-set random address.
	CHECK(FindCmd(BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR) == nullptr);
}

void TestAdvRandomIdentity()
{
	// Static random address: two most significant bits of the MSB are 1.
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	const CapturedCmd *ap = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM);
	CHECK(ap != nullptr);
	if (ap != nullptr)
	{
		CHECK(ap->Param[kAdvOwnAddr] == BTADDR_TYPE_RAND);
	}
	const CapturedCmd *ra = FindCmd(BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR);
	CHECK(ra != nullptr);
	if (ra != nullptr)
	{
		// AdvHandle(1) then the 6-octet random address.
		CHECK(std::memcmp(ra->Param + 1, addr, 6) == 0);
	}
}

void TestAdvInvalidRandomIdentity()
{
	// MSB 0x06: top two bits are not 0b11, not a valid static random address.
	uint8_t addr[6] = { 1, 2, 3, 4, 5, 6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	// Advertising init fails rather than loading an invalid random address.
	CHECK(BtAppAdvInit(&cfg) == false);
}

// The random part must hold at least one 0 and at least one 1 as well as the
// type bits (Vol 6 Part B 1.3.2.1), so an identity left all zero or all one
// with only the type bits forced on is still rejected.
void TestAdvDegenerateRandomIdentity()
{
	uint8_t allZero[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, allZero);
	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == false);

	uint8_t allOne[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	Setup(BTADDR_TYPE_RANDOM_STATIC, allOne);
	cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == false);

	// A single differing bit makes it valid.
	uint8_t oneBitSet[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC1 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, oneBitSet);
	cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);
}

// ---- F11: the advertising state follows the controller -------------------

// BtAppAdvStop used to report idle whatever the controller answered. If the
// disable is rejected the set is still on air, and a later BtAppAdvStart would
// see IDLE, send an enable and treat the device as freshly advertising when it
// never stopped.
void TestAdvStopKeepsStateOnFailure()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	// The controller rejects the disable: the device is still advertising.
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE;
	s_FailEnable = 0;
	BtAppAdvStop();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	// Once the disable is accepted the state follows.
	s_FailOpCode = 0;
	BtAppAdvStop();
	CHECK(g_BtAppData.State == BTAPP_STATE_IDLE);
}

// Updating the manufacturer data while advertising stops and restarts the set.
// If the restart is rejected the device is off air, so the call reports failure
// instead of leaving the state claiming it still advertises.
void TestAdvManDataSetReportsRestartFailure()
{
	uint8_t addr[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC6 };
	Setup(BTADDR_TYPE_RANDOM_STATIC, addr);

	BtAppCfg_t cfg = MakePeripheralCfg();
	CHECK(BtAppAdvInit(&cfg) == true);

	BtAppAdvStart();
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	uint8_t data[4] = { 1, 2, 3, 4 };

	// A successful update keeps the device advertising.
	CHECK(BtAppAdvManDataSet(data, sizeof(data), nullptr, 0) == true);
	CHECK(g_BtAppData.State == BTAPP_STATE_ADVERTISING);

	// The re-enable is rejected: the device is off air with the new data
	// loaded, and the state says so.
	s_FailOpCode = BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE;
	s_FailEnable = 1;
	CHECK(BtAppAdvManDataSet(data, sizeof(data), nullptr, 0) == false);
	CHECK(g_BtAppData.State == BTAPP_STATE_IDLE);
}

// ---- G5: manufacturer company identifier byte order ----------------------

void TestManDataCompanyIdLittleEndian()
{
	uint8_t addr[6] = {};
	Setup(BTADDR_TYPE_PUBLIC, addr);
	g_BtAppData.AppDevice.VendorId = 0x1234;

	uint8_t payload[3] = { 0xAA, 0xBB, 0xCC };
	CHECK(BtAppAdvManDataSet(payload, sizeof(payload), nullptr, 0) == true);

	const CapturedCmd *ad = FindCmd(BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA);
	CHECK(ad != nullptr);
	if (ad != nullptr)
	{
		// AdvHandle(1) Operation(1) FragPref(1) DataLen(1) then the AD list:
		// Length, Type(0xFF manuf), CompanyId_lo, CompanyId_hi, payload...
		CHECK(ad->Param[5] == 0xFF);
		CHECK(ad->Param[6] == 0x34);		// 0x1234 low octet first
		CHECK(ad->Param[7] == 0x12);
		CHECK(ad->Param[8] == 0xAA);		// payload follows the company id
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
	TestManDataCompanyIdLittleEndian();
	TestAdvPublicIdentity();
	TestAdvRandomIdentity();
	TestAdvInvalidRandomIdentity();
	TestAdvDegenerateRandomIdentity();
	TestAdvStopKeepsStateOnFailure();
	TestAdvManDataSetReportsRestartFailure();

	if (s_Failures != 0)
	{
		std::printf("Adv HCI host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("Adv HCI host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
