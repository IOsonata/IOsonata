#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bluetooth/bt_app.h"
#include "bluetooth/services/bt_dis.h"
#include "bt_test_harness.h"

#define NRF_SUCCESS					0U
#define NRF_ERROR_INTERNAL			3U
#define BLE_GAP_ADV_SET_HANDLE_NOT_SET	0xFFU
#define BLE_GAP_TX_POWER_ROLE_ADV		1U
#define BLE_GAP_TX_POWER_ROLE_SCAN_INIT	2U
#define GATT_MTU_SIZE_DEFAULT			23U
#define DEBUG_PRINTF(...)

struct ble_conn_params_evt {};

namespace {

enum FailPoint {
	FAIL_NONE,
	FAIL_EVENT_QUEUE,
	FAIL_PEER_POOL,
	FAIL_STACK,
	FAIL_DIS_SERVICE,
	FAIL_DIS_VALUE,
	FAIL_SECURITY,
	FAIL_ADVERTISING,
};

enum CallId {
	CALL_EVENT_QUEUE,
	CALL_PEER_POOL,
	CALL_LONG_WRITE,
	CALL_STACK,
	CALL_APPEARANCE,
	CALL_GAP,
	CALL_NAME,
	CALL_CONN_PARAMS,
	CALL_USER_SERVICES,
	CALL_DIS_SERVICE,
	CALL_DIS_VALUE,
	CALL_USER_DATA,
	CALL_SECURITY,
	CALL_ADVERTISING,
	CALL_TX_POWER,
};

static FailPoint s_FailPoint;
static int s_FailDisValue;
static CallId s_Calls[64];
static int s_CallCount;
static int s_DisValueCount;
static int s_ZeroLenCount;
static bool s_ZeroLenPointerValid;

static void Record(CallId Id)
{
	if (s_CallCount < (int)(sizeof(s_Calls) / sizeof(s_Calls[0])))
	{
		s_Calls[s_CallCount++] = Id;
	}
}

static void ResetHarness()
{
	s_FailPoint = FAIL_NONE;
	s_FailDisValue = -1;
	s_CallCount = 0;
	s_DisValueCount = 0;
	s_ZeroLenCount = 0;
	s_ZeroLenPointerValid = true;
	memset(&g_BtAppData, 0, sizeof(g_BtAppData));
	g_BtAppData.State = BTAPP_STATE_UNKNOWN;
	g_BtAppData.AdvHdl = 0xFF;
	g_BtAppData.ConnLedPort = -1;
	g_BtAppData.ConnLedPin = -1;
}

static BtAppCfg_t MakeCfg(bool Secure = true, bool DeviceInfo = true)
{
	static const BtAppDevInfo_t devInfo = {
		"Model-54", "I-SYST", "SN-54", "1.0.0", "A1"
	};
	static uint8_t evtMem[64];
	static uint8_t peerMem[256];
	static uint8_t longWriteMem[256];

	BtAppCfg_t cfg = {};
	cfg.Role = BTAPP_ROLE_PERIPHERAL;
	cfg.PeriLinkCount = 1;
	cfg.pDevName = "BM init test";
	cfg.VendorId = 0x1234;
	cfg.ProductId = 0x5678;
	cfg.ProductVer = 0x0100;
	cfg.pDevInfo = DeviceInfo ? &devInfo : nullptr;
	cfg.SecType = Secure ? BTGAP_SECTYPE_LESC_MITM : BTGAP_SECTYPE_NONE;
	cfg.SecExchg = BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO;
	cfg.AdvInterval = 100;
	cfg.ConnIntervalMin = 15.0F;
	cfg.ConnIntervalMax = 30.0F;
	cfg.ConnLedPort = -1;
	cfg.ConnLedPin = -1;
	cfg.MaxMtu = 247;
	cfg.pEvtHandlerQueMem = evtMem;
	cfg.EvtHandlerQueMemSize = sizeof(evtMem);
	cfg.pPeerPoolMem = peerMem;
	cfg.PeerPoolMemSize = sizeof(peerMem);
	cfg.pLongWrPoolMem = longWriteMem;
	cfg.LongWrPoolMemSize = sizeof(longWriteMem);
	return cfg;
}

static bool CallSeen(CallId Id)
{
	for (int i = 0; i < s_CallCount; i++)
	{
		if (s_Calls[i] == Id)
		{
			return true;
		}
	}
	return false;
}

} // namespace

BtAppData_t g_BtAppData = {};

extern "C" {

bool AppEvtHandlerInit(uint8_t *, size_t)
{
	Record(CALL_EVENT_QUEUE);
	return s_FailPoint != FAIL_EVENT_QUEUE;
}

bool BtPeerInit(uint8_t *, size_t)
{
	Record(CALL_PEER_POOL);
	return s_FailPoint != FAIL_PEER_POOL;
}

void BtPeerLongWrInit(uint8_t *, size_t)
{
	Record(CALL_LONG_WRITE);
}

void BtAppConnLedOff(void) {}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}

bool BtAppStackInit(const BtAppCfg_t *)
{
	Record(CALL_STACK);
	return s_FailPoint != FAIL_STACK;
}

uint32_t sd_ble_gap_appearance_set(uint16_t)
{
	Record(CALL_APPEARANCE);
	return NRF_SUCCESS;
}

void BtGapInit(const BtGapCfg_t *)
{
	Record(CALL_GAP);
}

void BtGapSetDevName(const char *)
{
	Record(CALL_NAME);
}

void ble_conn_params_evt_handler_set(void (*)(const struct ble_conn_params_evt *))
{
	Record(CALL_CONN_PARAMS);
}

void BtAppInitUserServices(void)
{
	Record(CALL_USER_SERVICES);
}

void BtAppInitUserData(void)
{
	Record(CALL_USER_DATA);
}

bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	Record(CALL_DIS_SERVICE);
	if (s_FailPoint == FAIL_DIS_SERVICE || pSrvc == nullptr)
	{
		return false;
	}
	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		pSrvc->pCharArray[i].ValHdl = (uint16_t)(0x20 + i);
	}
	return true;
}

bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	Record(CALL_DIS_VALUE);
	const int index = s_DisValueCount++;
	if (pChar == nullptr || pVal == nullptr || Len > pChar->MaxDataLen ||
		pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}
	if (Len == 0)
	{
		s_ZeroLenCount++;
		s_ZeroLenPointerValid = s_ZeroLenPointerValid && pVal != nullptr;
	}
	if (s_FailPoint == FAIL_DIS_VALUE && index == s_FailDisValue)
	{
		return false;
	}
	pChar->pValue = (uint8_t *)pVal;
	pChar->ValueLen = (uint16_t)Len;
	return true;
}

bool BtAppAdvInit(const BtAppCfg_t *)
{
	Record(CALL_ADVERTISING);
	return s_FailPoint != FAIL_ADVERTISING;
}

uint32_t sd_ble_gap_tx_power_set(uint8_t, uint8_t, int8_t)
{
	Record(CALL_TX_POWER);
	return NRF_SUCCESS;
}

int8_t GetValidTxPower(int Value)
{
	return (int8_t)Value;
}

} // extern "C"

static void on_conn_params_evt(const struct ble_conn_params_evt *) {}

static uint32_t BtAppPeerMngrInit(BTGAP_SECTYPE, uint8_t, bool)
{
	Record(CALL_SECURITY);
	return s_FailPoint == FAIL_SECURITY ? NRF_ERROR_INTERNAL : NRF_SUCCESS;
}

#include "generated/bt_app_bm_init.inc"

int main()
{
	bttest::Context ctx("nRF54 BM BtAppInit contract tests");

	ctx.Run("full secure initialization", [&]() {
		ResetHarness();
		BtAppCfg_t cfg = MakeCfg();
		BT_CHECK(ctx, BtAppInit(&cfg));
		BT_CHECK(ctx, g_BtAppData.State == BTAPP_STATE_INITIALIZED);
		BT_CHECK(ctx, CallSeen(CALL_DIS_SERVICE));
		BT_CHECK(ctx, s_DisValueCount == 7);
		BT_CHECK(ctx, s_ZeroLenCount == 1);
		BT_CHECK(ctx, s_ZeroLenPointerValid);
		BT_CHECK(ctx, CallSeen(CALL_SECURITY));
		BT_CHECK(ctx, CallSeen(CALL_ADVERTISING));
		BT_CHECK(ctx, CallSeen(CALL_TX_POWER));
	});

	ctx.Run("open initialization skips security", [&]() {
		ResetHarness();
		BtAppCfg_t cfg = MakeCfg(false, true);
		BT_CHECK(ctx, BtAppInit(&cfg));
		BT_CHECK(ctx, !CallSeen(CALL_SECURITY));
		BT_CHECK(ctx, CallSeen(CALL_ADVERTISING));
	});

	ctx.Run("fail fast before stack", [&]() {
		ResetHarness();
		s_FailPoint = FAIL_EVENT_QUEUE;
		BtAppCfg_t cfg = MakeCfg();
		BT_CHECK(ctx, !BtAppInit(&cfg));
		BT_CHECK(ctx, s_CallCount == 1);

		ResetHarness();
		s_FailPoint = FAIL_PEER_POOL;
		BT_CHECK(ctx, !BtAppInit(&cfg));
		BT_CHECK(ctx, CallSeen(CALL_EVENT_QUEUE));
		BT_CHECK(ctx, CallSeen(CALL_PEER_POOL));
		BT_CHECK(ctx, !CallSeen(CALL_STACK));
	});

	ctx.Run("stack failure stops service setup", [&]() {
		ResetHarness();
		s_FailPoint = FAIL_STACK;
		BtAppCfg_t cfg = MakeCfg();
		BT_CHECK(ctx, !BtAppInit(&cfg));
		BT_CHECK(ctx, !CallSeen(CALL_USER_SERVICES));
		BT_CHECK(ctx, !CallSeen(CALL_DIS_SERVICE));
	});

	ctx.Run("DIS registration failure reaches AppInit", [&]() {
		ResetHarness();
		s_FailPoint = FAIL_DIS_SERVICE;
		BtAppCfg_t cfg = MakeCfg();
		BT_CHECK(ctx, !BtAppInit(&cfg));
		BT_CHECK(ctx, !CallSeen(CALL_SECURITY));
		BT_CHECK(ctx, !CallSeen(CALL_ADVERTISING));
	});

	ctx.Run("every DIS value failure reaches AppInit", [&]() {
		for (int i = 0; i < 7; i++)
		{
			ResetHarness();
			s_FailPoint = FAIL_DIS_VALUE;
			s_FailDisValue = i;
			BtAppCfg_t cfg = MakeCfg();
			BT_CHECK(ctx, !BtAppInit(&cfg));
			BT_CHECK(ctx, !CallSeen(CALL_SECURITY));
			BT_CHECK(ctx, !CallSeen(CALL_ADVERTISING));
		}
	});

	ctx.Run("security and advertising failures propagate", [&]() {
		ResetHarness();
		s_FailPoint = FAIL_SECURITY;
		BtAppCfg_t cfg = MakeCfg();
		BT_CHECK(ctx, !BtAppInit(&cfg));
		BT_CHECK(ctx, !CallSeen(CALL_ADVERTISING));

		ResetHarness();
		s_FailPoint = FAIL_ADVERTISING;
		BT_CHECK(ctx, !BtAppInit(&cfg));
		BT_CHECK(ctx, CallSeen(CALL_SECURITY));
		BT_CHECK(ctx, !CallSeen(CALL_TX_POWER));
	});

	return ctx.Finish();
}
