// The GAP service bt_gap.cpp defines, and the LE GATT Security Levels
// characteristic in particular. Core 5.4 Vol 3 Part C 12.7.
//
// Nothing linked bt_gap.cpp on the host before this, so the service
// definition itself was never checked. BtGattSrvcAdd is stubbed here, which
// is what lets a case see the characteristic values as they are at
// registration: 12.7 requires the value to be static during a connection, and
// a port that copies the characteristic into a vendor database when the
// service is registered would take whatever was there at that moment.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_uuid.h"

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

// What BtGattSrvcAdd saw, captured at registration rather than read after.
struct {
	int Count;
	BtGattSrvc_t *pSrvc[4];
	int NbChar[4];
	uint16_t SecLevelUuid;
	uint8_t SecLevelVal[BT_GAP_SEC_LEVELS_MAX_LEN];
	int SecLevelLen;
	uint8_t SecLevelProp;
	bool bFail;
} s_Reg;

void ResetReg(void)
{
	std::memset(&s_Reg, 0, sizeof(s_Reg));
}

BtGapCfg_t MakeCfg(uint32_t SecType)
{
	BtGapCfg_t cfg;

	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Role = BT_GAP_ROLE_PERIPHERAL;
	cfg.SecType = SecType;

	return cfg;
}

// The characteristic the security levels live in, found by UUID rather than
// by index so a later insertion in the array does not silently move it.
const BtGattChar_t *SecLevelChar(void)
{
	for (int i = 0; i < s_Reg.Count; i++)
	{
		BtGattSrvc_t *p = s_Reg.pSrvc[i];

		if (p == nullptr || p->UuidSrvc != BT_UUID_GATT_SERVICE_GENERIC_ACCESS)
		{
			continue;
		}

		for (int j = 0; j < p->NbChar; j++)
		{
			if (p->pCharArray[j].Uuid ==
				BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS)
			{
				return &p->pCharArray[j];
			}
		}
	}

	return nullptr;
}

// 12.7 puts the characteristic in the GAP service, read only, with no
// encryption, authentication or authorization, because a client reads it to
// learn what security to establish and cannot have established it yet.
void TestTheCharacteristicIsInTheGapService(void)
{
	ResetReg();

	BtGapCfg_t cfg = MakeCfg(BTGAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	const BtGattChar_t *p = SecLevelChar();
	CHECK(p != nullptr);
	if (p == nullptr)
	{
		return;
	}

	CHECK(p->Property == BT_GATT_CHAR_PROP_READ);
	CHECK((p->Property & BT_GATT_CHAR_PROP_WRITE) == 0);
	CHECK((p->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED) == 0);
	CHECK(p->SecType == BT_GAP_SECTYPE_NONE);
}

// The value is a sequence of Security Level Requirements, each a Security
// Mode octet then a Security Level octet, written as the numbers used in
// their definitions.
void TestTheDefaultFollowsTheConfiguredSecurity(void)
{
	struct { uint32_t SecType; uint8_t Mode; uint8_t Level; } kMap[] = {
		{ BTGAP_SECTYPE_NONE,               1, 1 },
		{ BTGAP_SECTYPE_STATICKEY_NO_MITM,  1, 2 },
		{ BTGAP_SECTYPE_STATICKEY_MITM,     1, 3 },
		{ BTGAP_SECTYPE_LESC_MITM,          1, 4 },
		{ BTGAP_SECTYPE_SIGNED_NO_MITM,     2, 1 },
		{ BTGAP_SECTYPE_SIGNED_MITM,        2, 2 },
	};

	for (size_t i = 0; i < sizeof(kMap) / sizeof(kMap[0]); i++)
	{
		ResetReg();

		BtGapCfg_t cfg = MakeCfg(kMap[i].SecType);
		BtGapInit(&cfg);

		const BtGattChar_t *p = SecLevelChar();
		CHECK(p != nullptr);
		if (p == nullptr)
		{
			continue;
		}

		CHECK(p->ValueLen == 2);
		if (p->ValueLen == 2)
		{
			const uint8_t *v = (const uint8_t*)p->pValue;
			CHECK(v[0] == kMap[i].Mode);
			CHECK(v[1] == kMap[i].Level);
		}
	}
}

// The value has to be in place before the service is registered. A port that
// copies the characteristic into a vendor database at registration would
// otherwise take an empty one, and 12.7 requires the value to be static for
// the whole connection.
void TestTheValueIsSetBeforeRegistration(void)
{
	ResetReg();

	BtGapCfg_t cfg = MakeCfg(BTGAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	CHECK(s_Reg.SecLevelUuid == BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS);
	CHECK(s_Reg.SecLevelLen == 2);
	CHECK(s_Reg.SecLevelVal[0] == 1);
	CHECK(s_Reg.SecLevelVal[1] == 4);
	CHECK(s_Reg.SecLevelProp == BT_GATT_CHAR_PROP_READ);
}

// An application whose stack refuses legacy pairing replaces the value, since
// the default answers level 3 for an authenticated configuration and level 3
// tells a client that authenticated legacy pairing is enough.
void TestAnApplicationCanReplaceTheValue(void)
{
	ResetReg();

	BtGapCfg_t cfg = MakeCfg(BTGAP_SECTYPE_STATICKEY_MITM);
	BtGapInit(&cfg);

	const BtGattChar_t *p = SecLevelChar();
	CHECK(p != nullptr);
	if (p == nullptr)
	{
		return;
	}

	CHECK(((const uint8_t*)p->pValue)[1] == 3);

	const uint8_t sc[2] = { 1, 4 };
	CHECK(BtGapSecLevelsSet(sc, sizeof(sc)));
	CHECK(p->ValueLen == 2);
	CHECK(((const uint8_t*)p->pValue)[0] == 1);
	CHECK(((const uint8_t*)p->pValue)[1] == 4);

	// Several requirements for one mode are legal, and mean any of them is
	// enough rather than all of them.
	const uint8_t two[4] = { 1, 2, 2, 1 };
	CHECK(BtGapSecLevelsSet(two, sizeof(two)));
	CHECK(p->ValueLen == 4);
	CHECK(std::memcmp(p->pValue, two, sizeof(two)) == 0);
}

// Each requirement is two octets, so an odd length is not a sequence of them.
void TestMalformedValuesAreRefused(void)
{
	ResetReg();

	BtGapCfg_t cfg = MakeCfg(BTGAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	const uint8_t v[BT_GAP_SEC_LEVELS_MAX_LEN + 2] = { 1, 4 };

	CHECK(BtGapSecLevelsSet(nullptr, 2) == false);
	CHECK(BtGapSecLevelsSet(v, 0) == false);
	CHECK(BtGapSecLevelsSet(v, 1) == false);
	CHECK(BtGapSecLevelsSet(v, 3) == false);
	CHECK(BtGapSecLevelsSet(v, BT_GAP_SEC_LEVELS_MAX_LEN + 2) == false);

	// The value the refusals left alone is still the one from init.
	const BtGattChar_t *p = SecLevelChar();
	CHECK(p != nullptr);
	if (p != nullptr)
	{
		CHECK(p->ValueLen == 2);
		CHECK(((const uint8_t*)p->pValue)[1] == 4);
	}

	// The largest legal value is accepted.
	CHECK(BtGapSecLevelsSet(v, BT_GAP_SEC_LEVELS_MAX_LEN));
}

// A device with no peripheral role registers no GAP service, so there is
// nothing to put the characteristic in.
void TestACentralOnlyDeviceRegistersNothing(void)
{
	ResetReg();

	BtGapCfg_t cfg = MakeCfg(BTGAP_SECTYPE_LESC_MITM);
	cfg.Role = BT_GAP_ROLE_CENTRAL;
	BtGapInit(&cfg);

	CHECK(s_Reg.Count == 0);
	CHECK(SecLevelChar() == nullptr);
}

} // namespace

// --- stubs ---------------------------------------------------------------

// The real one lives in bt_gatt.cpp, which brings the whole attribute
// database with it. What these cases need is only that a value set through it
// lands on the characteristic.
bool BtGattCharSetValue(BtGattChar_t *pChar, void *pVal, size_t Len)
{
	if (pChar == nullptr || pVal == nullptr || Len > pChar->MaxDataLen)
	{
		return false;
	}

	if (pChar->pValue == nullptr)
	{
		static uint8_t s_ValBuf[8][64];
		static int s_ValNext = 0;

		if (s_ValNext >= 8 || Len > sizeof(s_ValBuf[0]))
		{
			return false;
		}
		pChar->pValue = s_ValBuf[s_ValNext++];
	}

	std::memcpy(pChar->pValue, pVal, Len);
	pChar->ValueLen = (uint16_t)Len;

	return true;
}

bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	if (s_Reg.bFail)
	{
		return false;
	}

	if (pSrvc != nullptr &&
		s_Reg.Count < (int)(sizeof(s_Reg.pSrvc) / sizeof(s_Reg.pSrvc[0])))
	{
		s_Reg.pSrvc[s_Reg.Count] = pSrvc;
		s_Reg.NbChar[s_Reg.Count] = pSrvc->NbChar;
		s_Reg.Count++;

		for (int i = 0; i < pSrvc->NbChar; i++)
		{
			BtGattChar_t *p = &pSrvc->pCharArray[i];

			if (p->Uuid != BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS)
			{
				continue;
			}

			s_Reg.SecLevelUuid = p->Uuid;
			s_Reg.SecLevelProp = p->Property;
			s_Reg.SecLevelLen = (int)p->ValueLen;
			if (p->pValue != nullptr &&
				p->ValueLen <= sizeof(s_Reg.SecLevelVal))
			{
				std::memcpy(s_Reg.SecLevelVal, p->pValue, p->ValueLen);
			}
		}
	}

	return true;
}

int main()
{
	TestTheCharacteristicIsInTheGapService();
	TestTheDefaultFollowsTheConfiguredSecurity();
	TestTheValueIsSetBeforeRegistration();
	TestAnApplicationCanReplaceTheValue();
	TestMalformedValuesAreRefused();
	TestACentralOnlyDeviceRegistersNothing();

	if (s_Failures == 0)
	{
		std::printf("GAP service host tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("GAP service host tests: %d failure(s), %d checks\n",
				s_Failures, s_Checks);

	return 1;
}
