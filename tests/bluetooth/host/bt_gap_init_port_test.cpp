/**-------------------------------------------------------------------------
@file	bt_gap_init_port_test.cpp

@brief	Host test for BtGapInit on a port whose stack owns the GAP service.

		The SoftDevice, sdk-nrf-bm and CubeWBA each create 0x1800 and 0x1801
		themselves and refuse to have them registered again, so all three
		ports short-circuit those two services in BtGattSrvcAdd and return
		success without assigning a handle or a value pointer. This test
		reproduces that shape with a strong BtGattSrvcAdd and drives BtGapInit
		through it.

		What it has to end up doing is fixed by Core Vol 3 Part C Table 12.2,
		which lists the LE GATT Security Levels characteristic as Optional for
		both the LE Peripheral and the LE Central role. A device that does not
		expose it conforms. So a port that cannot host 0x2BF5 has to finish
		initialising, not fail.

@author	Hoang Nguyen Hoan
@date	Aug. 13, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/
#include <cstring>

#include "bt_test_harness.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gatt_init.h"
#include "bluetooth/bt_uuid.h"

// BtGapInit publishes the configured security type here. The real definition
// lives in bt_app.cpp, which this test does not link.
BtAppData_t g_BtAppData;

namespace {

bttest::Context g_Ctx("bt_gap_init_port_test");

#define CHECK(expr)	BT_CHECK(g_Ctx, expr)

// Services the stand-in port declined, so a test can tell that BtGapInit
// really did try to register them.
int g_DeclinedCount;

// Turned off to check the other half: a port that does register the GAP
// service still fails init when the value write fails.
bool g_bDeclineNativeServices = true;
bool g_bFailValueSet;

BtGapCfg_t MakeCfg(uint8_t Role, uint8_t SecType)
{
	BtGapCfg_t cfg = {};

	cfg.Role = Role;
	cfg.SecType = SecType;
	cfg.AdvInterval = 64;
	cfg.ConnIntervalMin = 8;
	cfg.ConnIntervalMax = 40;
	cfg.SupTimeout = 400;
	return cfg;
}

void Reset(void)
{
	g_DeclinedCount = 0;
	g_bDeclineNativeServices = true;
	g_bFailValueSet = false;
	BtAttDBInit(2048);
}

//-----------------------------------------------------------------------------

// The case that blocks three ports. A peripheral whose stack owns 0x1800 has
// nowhere to put the optional LE GATT Security Levels characteristic, and
// Table 12.2 says it does not have to. Initialisation completes.
void TestPeripheralInitCompletesWithoutTheCharacteristic()
{
	Reset();
	BtGapCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL,
							 BT_GAP_SECTYPE_LESC_MITM);

	BtGapInit(&cfg);

	// The GAP service was offered and declined, so this is the port shape the
	// test means to cover and not an accident of registration order.
	CHECK(g_DeclinedCount >= 1);
	CHECK(BtGattInitStatusErrorGet() == BT_GATT_INIT_ERROR_NONE);
	CHECK(BtGattInitStatusComplete());
}

// The same for a central, which Table 12.2 also lists as Optional.
void TestCentralInitCompletesWithoutTheCharacteristic()
{
	Reset();
	BtGapCfg_t cfg = MakeCfg(BT_GAP_ROLE_CENTRAL, BT_GAP_SECTYPE_LESC_MITM);

	BtGapInit(&cfg);

	CHECK(BtGattInitStatusErrorGet() == BT_GATT_INIT_ERROR_NONE);
	CHECK(BtGattInitStatusComplete());
}

// An open device on the same port. Nothing to write either way, and the
// result must be the same.
void TestOpenPeripheralInitCompletes()
{
	Reset();
	BtGapCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL, BT_GAP_SECTYPE_NONE);

	BtGapInit(&cfg);

	CHECK(BtGattInitStatusErrorGet() == BT_GATT_INIT_ERROR_NONE);
	CHECK(BtGattInitStatusComplete());
}

// A characteristic the port never registered has no value, so an application
// asking for the requirements gets nothing rather than a stale answer, and
// setting them reports that it did not happen.
void TestSecurityLevelsUnavailableOnSuchAPort()
{
	Reset();
	BtGapCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL,
							 BT_GAP_SECTYPE_STATICKEY_MITM);
	BtGapInit(&cfg);

	uint8_t buff[BT_GAP_SEC_LEVEL_VALUE_MAX] = {};
	CHECK(BtGapGetSecurityLevels(buff, sizeof(buff)) == 0);

	const uint8_t req[2] = {
		BT_GAP_SEC_MODE_1, BT_GAP_SEC_MODE1_LEVEL_ENC_AUTH,
	};
	CHECK(BtGapSetSecurityLevels(req, 1) == false);
}

// The other half, so the relaxation above cannot hide a real fault: on a port
// that does register the GAP service, a failing value write is still an
// initialisation failure.
void TestRegisteredCharacteristicValueSetFailureStillFails()
{
	Reset();
	g_bDeclineNativeServices = false;
	g_bFailValueSet = true;

	BtGapCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL,
							 BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	CHECK(g_DeclinedCount == 0);
	CHECK(BtGattInitStatusErrorGet() == BT_GATT_INIT_ERROR_VALUE_SET);
	CHECK(BtGattInitStatusComplete() == false);
}

// And that the same port, with the write working, writes what the security
// type amounts to. This is the control for the case above.
void TestRegisteredCharacteristicIsWritten()
{
	Reset();
	g_bDeclineNativeServices = false;

	BtGapCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL,
							 BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	uint8_t buff[BT_GAP_SEC_LEVEL_VALUE_MAX] = {};
	CHECK(BtGapGetSecurityLevels(buff, sizeof(buff)) == 2);
	CHECK(buff[0] == BT_GAP_SEC_MODE_1);
	CHECK(buff[1] == BT_GAP_SEC_MODE1_LEVEL_LESC);
	CHECK(BtGattInitStatusErrorGet() == BT_GATT_INIT_ERROR_NONE);
}

} // namespace

// Stand-in port. Declines 0x1800 and 0x1801 exactly as the three vendor ports
// do: reports success, assigns no handle and no value pointer.
bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr ||
		pSrvc->NbChar <= 0)
	{
		return false;
	}

	if (g_bDeclineNativeServices && !pSrvc->bCustom &&
		(pSrvc->UuidSrvc == BT_UUID_GATT_SERVICE_GENERIC_ACCESS ||
		 pSrvc->UuidSrvc == BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE))
	{
		g_DeclinedCount++;
		return true;
	}

	// Otherwise behave like a port that really does register: give every
	// characteristic a handle and somewhere to hold its value.
	static uint8_t s_ValueStore[8][248];
	static uint16_t s_NextHdl = 1;

	pSrvc->Hdl = s_NextHdl++;

	for (int i = 0; i < pSrvc->NbChar && i < 8; i++)
	{
		BtGattChar_t *pChar = &pSrvc->pCharArray[i];

		pChar->pSrvc = pSrvc;
		pChar->Hdl = s_NextHdl++;
		pChar->ValHdl = s_NextHdl++;
		pChar->pValue = s_ValueStore[i];
		pChar->ValueLen = 0;
	}

	BtGattInsertSrvcList(pSrvc);
	return true;
}

// Value writes go through the port on every stack, so the stand-in owns this
// too. Refuses a characteristic with no handle or no storage, as the nRF52
// and nRF54 ports do, and as the generic version does through pValue.
bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr || (Len > 0 && pVal == nullptr) ||
		Len > pChar->MaxDataLen || pChar->pValue == nullptr ||
		pChar->ValHdl == 0 || pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}
	if (g_bFailValueSet)
	{
		return false;
	}

	if (Len > 0)
	{
		std::memcpy(pChar->pValue, pVal, Len);
	}
	pChar->ValueLen = (uint16_t)Len;
	return true;
}

int main(void)
{
	g_Ctx.Run("peripheral init completes without the characteristic",
			  TestPeripheralInitCompletesWithoutTheCharacteristic);
	g_Ctx.Run("central init completes without the characteristic",
			  TestCentralInitCompletesWithoutTheCharacteristic);
	g_Ctx.Run("open peripheral init completes",
			  TestOpenPeripheralInitCompletes);
	g_Ctx.Run("security levels unavailable on such a port",
			  TestSecurityLevelsUnavailableOnSuchAPort);
	g_Ctx.Run("registered characteristic value set failure still fails",
			  TestRegisteredCharacteristicValueSetFailureStillFails);
	g_Ctx.Run("registered characteristic is written",
			  TestRegisteredCharacteristicIsWritten);

	return g_Ctx.Finish();
}
