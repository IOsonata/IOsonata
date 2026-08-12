#include <cstdint>
#include <cstring>

#include "bluetooth/bt_app.h"
#include "bluetooth/services/bt_bas.h"
#include "bluetooth/services/bt_dis.h"
#include "bt_test_harness.h"

namespace {

struct ValueCapture {
	BtGattChar_t *pChar;
	size_t Len;
	uint8_t Data[64];
};

static BtGattSrvc_t *s_Services[4];
static int s_ServiceCount;
static ValueCapture s_Values[16];
static int s_ValueCount;
static uint16_t s_NotifyConn;
static BtGattChar_t *s_NotifyChar;
static uint8_t s_NotifyData[8];
static size_t s_NotifyLen;
static int s_NotifyCount;

static ValueCapture *CaptureFor(BtGattChar_t *pChar)
{
	for (int i = 0; i < s_ValueCount; ++i)
	{
		if (s_Values[i].pChar == pChar)
		{
			return &s_Values[i];
		}
	}
	if (s_ValueCount >= (int)(sizeof(s_Values) / sizeof(s_Values[0])))
	{
		return nullptr;
	}
	s_Values[s_ValueCount].pChar = pChar;
	return &s_Values[s_ValueCount++];
}

static BtGattChar_t *FindChar(BtGattSrvc_t *pSrvc, uint16_t Uuid)
{
	if (pSrvc == nullptr)
	{
		return nullptr;
	}
	for (int i = 0; i < pSrvc->NbChar; ++i)
	{
		if (pSrvc->pCharArray[i].Uuid == Uuid)
		{
			return &pSrvc->pCharArray[i];
		}
	}
	return nullptr;
}

static bool ValueEquals(BtGattChar_t *pChar, const void *pData, size_t Len)
{
	ValueCapture *p = CaptureFor(pChar);
	return p != nullptr && p->Len == Len &&
		   std::memcmp(p->Data, pData, Len) == 0;
}

} // namespace

extern "C" {

bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	if (pSrvc == nullptr)
	{
		return false;
	}

	// Every port accepts a service it already holds and adds nothing, so
	// reinitialization refreshes values without a second registration.
	for (int i = 0; i < s_ServiceCount; i++)
	{
		if (s_Services[i] == pSrvc)
		{
			return true;
		}
	}

	if (s_ServiceCount >= 4)
	{
		return false;
	}
	s_Services[s_ServiceCount++] = pSrvc;
	return true;
}

bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	// Model strict vendor APIs: the buffer itself must be valid even when the
	// semantic value length is zero.
	if (pChar == nullptr || pVal == nullptr || Len > 64)
	{
		return false;
	}
	ValueCapture *p = CaptureFor(pChar);
	if (p == nullptr)
	{
		return false;
	}
	p->Len = Len;
	if (Len > 0)
	{
		std::memcpy(p->Data, pVal, Len);
	}
	pChar->pValue = pVal;
	pChar->ValueLen = (uint16_t)Len;
	return true;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar,
						  void * const pVal, size_t Len)
{
	if (pChar == nullptr || (Len > 0 && pVal == nullptr) || Len > sizeof(s_NotifyData))
	{
		return false;
	}
	s_NotifyConn = ConnHdl;
	s_NotifyChar = pChar;
	s_NotifyLen = Len;
	if (Len > 0)
	{
		std::memcpy(s_NotifyData, pVal, Len);
	}
	++s_NotifyCount;
	return true;
}

} // extern "C"

int main()
{
	bttest::Context ctx("Bluetooth standard service tests");

	ctx.Run("generic DIS", [&]() {
		BT_CHECK(ctx, BtDisInit(nullptr) == false);

		BtAppDevInfo_t info = {
			"Model-54", "I-SYST", "SN-42", "1.2.3", "A1", "9.8.7"
		};
		BtAppCfg_t cfg = {};
		cfg.pDevInfo = &info;
		cfg.VendorId = 0x1234;
		cfg.ProductId = 0x5678;
		cfg.ProductVer = 0x0102;

		cfg.Role = BTAPP_ROLE_CENTRAL;
		BT_CHECK(ctx, BtDisInit(&cfg));
		BT_CHECK(ctx, s_ServiceCount == 0);

		cfg.Role = BTAPP_ROLE_PERIPHERAL;
		BT_CHECK(ctx, BtDisInit(&cfg));
		BT_CHECK(ctx, s_ServiceCount == 1);
		BtGattSrvc_t *pDis = s_Services[0];
		BT_CHECK(ctx, pDis->UuidSrvc == BT_UUID_GATT_SERVICE_DEVICE_INFORMATION);
		// Seven because this config carries a software revision; a build
		// without one registers six and no Software Revision characteristic.
		BT_CHECK(ctx, pDis->NbChar == 7);

		BtGattChar_t *pManuf = FindChar(pDis,
			BT_UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING);
		BtGattChar_t *pModel = FindChar(pDis,
			BT_UUID_CHARACTERISTIC_MODEL_NUMBER_STRING);
		BtGattChar_t *pSw = FindChar(pDis,
			BT_UUID_CHARACTERISTIC_SOFTWARE_REVISION_STRING);
		BtGattChar_t *pPnp = FindChar(pDis,
			BT_UUID_CHARACTERISTIC_PNP_ID);
		BT_CHECK(ctx, pManuf != nullptr);
		BT_CHECK(ctx, pModel != nullptr);
		BT_CHECK(ctx, pSw != nullptr);
		BT_CHECK(ctx, pPnp != nullptr);
		BT_CHECK(ctx, ValueEquals(pManuf, "I-SYST", 6));
		BT_CHECK(ctx, ValueEquals(pModel, "Model-54", 8));

		BT_CHECK(ctx, ValueEquals(pSw, "9.8.7", 5));

		// An absent software revision leaves the characteristic registered
		// with an empty value rather than keeping the previous one.
		info.pSwVerStr = nullptr;
		BT_CHECK(ctx, BtDisInit(&cfg));
		ValueCapture *pSwCap = CaptureFor(pSw);
		BT_CHECK(ctx, pSwCap != nullptr && pSwCap->Len == 0);
		BT_CHECK(ctx, pSw != nullptr && pSw->pValue != nullptr);

		BtDisPnpId_t pnp = {};
		ValueCapture *pCap = CaptureFor(pPnp);
		BT_CHECK(ctx, pCap != nullptr);
		if (pCap != nullptr && pCap->Len == sizeof(pnp))
		{
			std::memcpy(&pnp, pCap->Data, sizeof(pnp));
		}
		BT_CHECK(ctx, pCap != nullptr && pCap->Len == sizeof(pnp));
		BT_CHECK(ctx, pnp.VendorIdSrc == BT_DIS_PNP_VENDOR_ID_SRC_BT_SIG);
		BT_CHECK(ctx, pnp.VendorId == cfg.VendorId);
		BT_CHECK(ctx, pnp.ProductId == cfg.ProductId);
		BT_CHECK(ctx, pnp.ProductVer == cfg.ProductVer);
	});

	ctx.Run("generic BAS", [&]() {
		BT_CHECK(ctx, BtBasInit(87));
		BT_CHECK(ctx, s_ServiceCount == 2);
		BtGattSrvc_t *pBas = s_Services[1];
		BT_CHECK(ctx, pBas->UuidSrvc == BT_UUID_GATT_SERVICE_BATTERY);
		BT_CHECK(ctx, pBas->NbChar == 1);
		BtGattChar_t *pLevel = BtBasBatteryLevelChar();
		BT_CHECK(ctx, pLevel == FindChar(pBas,
			BT_UUID_CHARACTERISTIC_BATTERY_LEVEL));
		uint8_t level = 87;
		BT_CHECK(ctx, ValueEquals(pLevel, &level, 1));
		BT_CHECK(ctx, BtBasGetLevel() == 87);

		BT_CHECK(ctx, BtBasSetLevel(0x0042, 120, true));
		BT_CHECK(ctx, BtBasGetLevel() == 100);
		BT_CHECK(ctx, s_NotifyCount == 1);
		BT_CHECK(ctx, s_NotifyConn == 0x0042);
		BT_CHECK(ctx, s_NotifyChar == pLevel);
		BT_CHECK(ctx, s_NotifyLen == 1 && s_NotifyData[0] == 100);

		BT_CHECK(ctx, BtBasInit(50));
		BT_CHECK(ctx, s_ServiceCount == 2);
		BT_CHECK(ctx, BtBasGetLevel() == 50);
		BT_CHECK(ctx, BtBasSetLevel(0x0042, 25, false));
		BT_CHECK(ctx, s_NotifyCount == 1);
	});

	return ctx.Finish();
}
