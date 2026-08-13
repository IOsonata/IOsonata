/**-------------------------------------------------------------------------
@file	bt_gap_sec_levels_test.cpp

@brief	LE GATT Security Levels characteristic, Core 5.4.

Core Vol 3 Part C, Section 12.7. The characteristic reports the highest
security requirements of the GATT server as one or more Security Level
Requirements, each a mode octet followed by a level octet. Section 10.2 defines
four levels for mode 1 and two for mode 2.

@author	Hoang Nguyen Hoan
@date	Aug. 11, 2026

@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_uuid.h"

// BtGapInit publishes the configured security type here for the GATT layer to
// resolve against. The real definition lives in bt_app.cpp, which this test
// does not link.
BtAppData_t g_BtAppData;

namespace {

int s_Checks;
int s_Failures;

#define CHECK(expr) do { \
	s_Checks++; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		s_Failures++; \
	} \
} while (0)

BtGapCfg_t MakeCfg(uint8_t SecType)
{
	BtGapCfg_t cfg = {};
	cfg.Role = BT_GAP_ROLE_PERIPHERAL;
	cfg.SecType = SecType;
	cfg.AdvInterval = 64;
	cfg.ConnIntervalMin = 8;
	cfg.ConnIntervalMax = 40;
	cfg.SupTimeout = 400;
	return cfg;
}

// Every security type maps to the one Security Level Requirement it amounts
// to, per Core Vol 3 Part C 10.2.1 and 10.2.2.
void TestDefaultFromSecurityType()
{
	struct {
		uint8_t SecType;
		uint8_t Mode;
		uint8_t Level;
	} cases[] = {
		{ BT_GAP_SECTYPE_NONE, BT_GAP_SEC_MODE_1, BT_GAP_SEC_MODE1_LEVEL_NONE },
		{ BT_GAP_SECTYPE_STATICKEY_NO_MITM, BT_GAP_SEC_MODE_1,
		  BT_GAP_SEC_MODE1_LEVEL_ENC_NO_AUTH },
		{ BT_GAP_SECTYPE_STATICKEY_MITM, BT_GAP_SEC_MODE_1,
		  BT_GAP_SEC_MODE1_LEVEL_ENC_AUTH },
		{ BT_GAP_SECTYPE_LESC_MITM, BT_GAP_SEC_MODE_1,
		  BT_GAP_SEC_MODE1_LEVEL_LESC },
		{ BT_GAP_SECTYPE_SIGNED_NO_MITM, BT_GAP_SEC_MODE_2,
		  BT_GAP_SEC_MODE2_LEVEL_SIGN_NO_AUTH },
		{ BT_GAP_SECTYPE_SIGNED_MITM, BT_GAP_SEC_MODE_2,
		  BT_GAP_SEC_MODE2_LEVEL_SIGN_AUTH },
	};

	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		BtGapCfg_t cfg = MakeCfg(cases[i].SecType);
		BtGapInit(&cfg);

		uint8_t buff[BT_GAP_SEC_LEVEL_VALUE_MAX] = {};
		size_t len = BtGapGetSecurityLevels(buff, sizeof(buff));
		CHECK(len == 2);
		if (len == 2)
		{
			CHECK(buff[0] == cases[i].Mode);
			CHECK(buff[1] == cases[i].Level);
		}
	}
}

// A server that accepts either of two requirements says so with two entries.
void TestMultipleRequirements()
{
	BtGapCfg_t cfg = MakeCfg(BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	const uint8_t req[4] = {
		BT_GAP_SEC_MODE_1, BT_GAP_SEC_MODE1_LEVEL_ENC_AUTH,
		BT_GAP_SEC_MODE_2, BT_GAP_SEC_MODE2_LEVEL_SIGN_AUTH,
	};
	CHECK(BtGapSetSecurityLevels(req, 2));

	uint8_t buff[BT_GAP_SEC_LEVEL_VALUE_MAX] = {};
	CHECK(BtGapGetSecurityLevels(buff, sizeof(buff)) == 4);
	CHECK(std::memcmp(buff, req, sizeof(req)) == 0);
}

void TestArgumentValidation()
{
	BtGapCfg_t cfg = MakeCfg(BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	const uint8_t good[2] = { BT_GAP_SEC_MODE_1, BT_GAP_SEC_MODE1_LEVEL_LESC };
	CHECK(BtGapSetSecurityLevels(good, 1));

	CHECK(BtGapSetSecurityLevels(nullptr, 1) == false);
	CHECK(BtGapSetSecurityLevels(good, 0) == false);

	// One requirement past what the characteristic holds.
	uint8_t many[(BT_GAP_SEC_LEVEL_REQ_MAX + 1) * 2];
	for (size_t i = 0; i < sizeof(many) / 2; i++)
	{
		many[i * 2] = BT_GAP_SEC_MODE_1;
		many[i * 2 + 1] = BT_GAP_SEC_MODE1_LEVEL_LESC;
	}
	CHECK(BtGapSetSecurityLevels(many, BT_GAP_SEC_LEVEL_REQ_MAX + 1) == false);

	// Mode 3 is BR/EDR only and has no meaning here; mode 0 does not exist.
	const uint8_t mode3[2] = { 3, 1 };
	CHECK(BtGapSetSecurityLevels(mode3, 1) == false);
	const uint8_t mode0[2] = { 0, 1 };
	CHECK(BtGapSetSecurityLevels(mode0, 1) == false);

	// Mode 1 has four levels, mode 2 has two. Level 0 exists in neither.
	const uint8_t mode1Level5[2] = { BT_GAP_SEC_MODE_1, 5 };
	CHECK(BtGapSetSecurityLevels(mode1Level5, 1) == false);
	const uint8_t mode1Level0[2] = { BT_GAP_SEC_MODE_1, 0 };
	CHECK(BtGapSetSecurityLevels(mode1Level0, 1) == false);
	const uint8_t mode2Level3[2] = { BT_GAP_SEC_MODE_2, 3 };
	CHECK(BtGapSetSecurityLevels(mode2Level3, 1) == false);

	// Every refusal above left the good value in place.
	uint8_t buff[BT_GAP_SEC_LEVEL_VALUE_MAX] = {};
	CHECK(BtGapGetSecurityLevels(buff, sizeof(buff)) == 2);
	CHECK(buff[0] == BT_GAP_SEC_MODE_1);
	CHECK(buff[1] == BT_GAP_SEC_MODE1_LEVEL_LESC);
}

// A bad entry anywhere in the list refuses the whole set, so the
// characteristic never holds half an update.
void TestPartialUpdateRefused()
{
	BtGapCfg_t cfg = MakeCfg(BT_GAP_SECTYPE_NONE);
	BtGapInit(&cfg);

	const uint8_t mixed[4] = {
		BT_GAP_SEC_MODE_1, BT_GAP_SEC_MODE1_LEVEL_LESC,
		BT_GAP_SEC_MODE_2, 9,
	};
	CHECK(BtGapSetSecurityLevels(mixed, 2) == false);

	uint8_t buff[BT_GAP_SEC_LEVEL_VALUE_MAX] = {};
	CHECK(BtGapGetSecurityLevels(buff, sizeof(buff)) == 2);
	CHECK(buff[0] == BT_GAP_SEC_MODE_1);
	CHECK(buff[1] == BT_GAP_SEC_MODE1_LEVEL_NONE);
}

void TestGetArguments()
{
	BtGapCfg_t cfg = MakeCfg(BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	// A length query copies nothing.
	CHECK(BtGapGetSecurityLevels(nullptr, 0) == 2);

	// A destination too small is refused rather than truncated into.
	uint8_t small[1] = { 0xA5 };
	CHECK(BtGapGetSecurityLevels(small, sizeof(small)) == 2);
	CHECK(small[0] == 0xA5);
}

// BtGapInit is the one path every port takes, so it is where the configured
// security type is published for the GATT layer. Without this the field keeps
// its zero initialiser, BT_GAP_SECTYPE_NONE, on every port that does not set
// it itself.
void TestApplicationSecurityTypePublished()
{
	const uint8_t types[] = {
		BT_GAP_SECTYPE_STATICKEY_NO_MITM,
		BT_GAP_SECTYPE_STATICKEY_MITM,
		BT_GAP_SECTYPE_LESC_MITM,
		BT_GAP_SECTYPE_SIGNED_NO_MITM,
		BT_GAP_SECTYPE_SIGNED_MITM,
		BT_GAP_SECTYPE_NONE,
	};

	for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); i++)
	{
		g_BtAppData.SecType = BTGAP_SECTYPE_OPEN;
		BtGapCfg_t cfg = MakeCfg(types[i]);
		BtGapInit(&cfg);
		CHECK(g_BtAppData.SecType == types[i]);
	}

	// A role with no peripheral bit still configures the application, because
	// a central enforces the same requirement on the server it talks to.
	g_BtAppData.SecType = BTGAP_SECTYPE_OPEN;
	BtGapCfg_t central = MakeCfg(BT_GAP_SECTYPE_LESC_MITM);
	central.Role = BT_GAP_ROLE_CENTRAL;
	BtGapInit(&central);
	CHECK(g_BtAppData.SecType == BT_GAP_SECTYPE_LESC_MITM);

	// A rejected configuration must not leave a stale requirement behind for
	// the services that follow. BTGAP_SECTYPE_OPEN is the sentinel here
	// because it is a valid enumerator that none of the cases configures.
	g_BtAppData.SecType = BTGAP_SECTYPE_OPEN;
	BtGapInit(nullptr);
	CHECK(g_BtAppData.SecType == BT_GAP_SECTYPE_NONE);
}

// The resolution rule every port shares: characteristic, then service, then
// application. A service written to inherit the application policy names
// nothing itself, so the walk has to reach a real value.
void TestInheritedSecurityResolvesToApplication()
{
	BtGapCfg_t cfg = MakeCfg(BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	BtGattChar_t chr = {};
	BtGattSrvc_t srvc = {};
	chr.Uuid = 0xFFF1;
	chr.SecType = BT_GAP_SECTYPE_NONE;
	srvc.SecType = BT_GAP_SECTYPE_NONE;

	uint8_t sec = BtGattSecTypeGet(&srvc, &chr, g_BtAppData.SecType);
	CHECK(sec == BT_GAP_SECTYPE_LESC_MITM);
	CHECK(BtGattSecTypeIsOpen(sec) == false);

	// The service still overrides the application, and the characteristic
	// still overrides the service.
	srvc.SecType = BT_GAP_SECTYPE_STATICKEY_NO_MITM;
	CHECK(BtGattSecTypeGet(&srvc, &chr, g_BtAppData.SecType) ==
		BT_GAP_SECTYPE_STATICKEY_NO_MITM);

	chr.SecType = BT_GAP_SECTYPE_OPEN;
	CHECK(BtGattSecTypeGet(&srvc, &chr, g_BtAppData.SecType) ==
		BT_GAP_SECTYPE_OPEN);
	CHECK(BtGattSecTypeIsOpen(
		BtGattSecTypeGet(&srvc, &chr, g_BtAppData.SecType)) == true);
}

// The GAP and GATT service characteristics are read before a client has met
// any requirement, so they name OPEN rather than inheriting. Publishing the
// application type must not change that.
void TestStandardServicesStayOpen()
{
	BtGapCfg_t cfg = MakeCfg(BT_GAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	for (BtGattSrvc_t *p = BtGattGetSrvcList(); p != nullptr; p = p->pNext)
	{
		if (p->UuidSrvc != BT_UUID_GATT_SERVICE_GENERIC_ACCESS &&
			p->UuidSrvc != BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE)
		{
			continue;
		}
		for (int i = 0; i < p->NbChar; i++)
		{
			CHECK(BtGattSecTypeIsOpen(BtGattSecTypeGet(p,
				&p->pCharArray[i], g_BtAppData.SecType)) == true);
		}
	}
}

} // namespace

int main()
{
	TestDefaultFromSecurityType();
	TestMultipleRequirements();
	TestArgumentValidation();
	TestPartialUpdateRefused();
	TestGetArguments();
	TestApplicationSecurityTypePublished();
	TestInheritedSecurityResolvesToApplication();
	TestStandardServicesStayOpen();

	if (s_Failures == 0)
	{
		std::printf("GAP security levels tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("GAP security levels tests: FAIL (%d failures, %d checks)\n",
		s_Failures, s_Checks);
	return 1;
}
