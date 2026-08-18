// BtGapInit against the real GATT server, not a stub.
//
// bt_gap_srvc_test.cpp checks what BtGattSrvcAdd is handed, and supplies its
// own BtGattCharSetValue that allocates storage when a characteristic has
// none. The real one in bt_gatt.cpp refuses exactly that case, because
// storage and the value handle are what registration hands out. No test
// linked the two real files together, so a peripheral that could not
// initialise looked healthy here: the default security levels value was set
// before the service was registered, the set failed, BT_GATT_INIT_ERROR_VALUE_SET
// was latched, and BtAppAdvInit refuses on a failed init.
//
// This file exists to keep those two together. Everything it checks is
// observable through the public interface after BtGapInit returns.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"
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

uint8_t s_AttPool[8192];

void ResetDb(void)
{
	std::memset(s_AttPool, 0, sizeof(s_AttPool));
	BtAttDBInit(sizeof(s_AttPool));
}

BtGapCfg_t Cfg(uint8_t Role, uint32_t SecType)
{
	BtGapCfg_t cfg;

	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Role = Role;
	cfg.SecType = SecType;

	return cfg;
}

// A peripheral has to come out of BtGapInit ready to advertise. This is the
// case that was failing: a port refuses to bring advertising up when this
// reports the services did not register.
void TestPeripheralInitLeavesNoError(void)
{
	ResetDb();

	BtGapCfg_t cfg = Cfg(BT_GAP_ROLE_PERIPHERAL, BTGAP_SECTYPE_LESC_MITM);
	CHECK(BtGapInit(&cfg));
}

// Every SecType, because the default value is derived from it and a failure
// in any one of them latches the same error.
void TestEverySecTypeInitialises(void)
{
	const uint32_t types[] = {
		BTGAP_SECTYPE_NONE,
		BTGAP_SECTYPE_STATICKEY_NO_MITM,
		BTGAP_SECTYPE_STATICKEY_MITM,
		BTGAP_SECTYPE_LESC_MITM,
		BTGAP_SECTYPE_SIGNED_NO_MITM,
		BTGAP_SECTYPE_SIGNED_MITM,
	};

	for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); i++)
	{
		ResetDb();

		BtGapCfg_t cfg = Cfg(BT_GAP_ROLE_PERIPHERAL, types[i]);
		CHECK(BtGapInit(&cfg));
	}
}

// The value has to be there for a client to read, not merely registered. The
// setter refuses a characteristic without storage or a handle, so a set that
// succeeds after init is what says registration gave it both.
void TestSecurityLevelsValueIsWritable(void)
{
	ResetDb();

	BtGapCfg_t cfg = Cfg(BT_GAP_ROLE_PERIPHERAL, BTGAP_SECTYPE_LESC_MITM);
	BtGapInit(&cfg);

	uint8_t req[2] = { 1, 4 };
	CHECK(BtGapSecLevelsSet(req, sizeof(req)));

	// And the shape rules still hold: pairs of octets, never odd, never empty,
	// never longer than the characteristic.
	uint8_t odd[3] = { 1, 4, 1 };
	CHECK(BtGapSecLevelsSet(odd, sizeof(odd)) == false);
	CHECK(BtGapSecLevelsSet(req, 0) == false);
	CHECK(BtGapSecLevelsSet(nullptr, 2) == false);

	uint8_t big[BT_GAP_SEC_LEVELS_MAX_LEN + 2];
	std::memset(big, 1, sizeof(big));
	CHECK(BtGapSecLevelsSet(big, sizeof(big)) == false);
}

// The registered security levels characteristic, found the way a client would
// reach it: through the service list the server publishes.
const BtGattChar_t *FindSecLevelsChar(void)
{
	for (const BtGattSrvc_t *s = BtGattGetSrvcList(); s != nullptr; s = s->pNext)
	{
		for (int i = 0; i < s->NbChar; i++)
		{
			const BtGattChar_t *c = &s->pCharArray[i];

			if (c->Uuid == BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS)
			{
				return c;
			}
		}
	}

	return nullptr;
}

// The value the server holds has to match the configured security type, and
// has to be right on a second init as well. The generic server moves pValue to
// its own database entry when it registers, so a second init that staged from
// the old pointer would read from a database that has since been reset.
void TestRegisteredValueMatchesSecType(void)
{
	struct { uint32_t SecType; uint8_t Mode; uint8_t Level; } cases[] = {
		{ BTGAP_SECTYPE_NONE, 1, 1 },
		{ BTGAP_SECTYPE_STATICKEY_NO_MITM, 1, 2 },
		{ BTGAP_SECTYPE_STATICKEY_MITM, 1, 3 },
		{ BTGAP_SECTYPE_LESC_MITM, 1, 4 },
		{ BTGAP_SECTYPE_SIGNED_NO_MITM, 2, 1 },
		{ BTGAP_SECTYPE_SIGNED_MITM, 2, 2 },
	};

	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		ResetDb();

		BtGapCfg_t cfg = Cfg(BT_GAP_ROLE_PERIPHERAL, cases[i].SecType);
		BtGapInit(&cfg);

		const BtGattChar_t *c = FindSecLevelsChar();
		CHECK(c != nullptr);

		if (c == nullptr)
		{
			continue;
		}

		const uint8_t *val = (const uint8_t *)c->pValue;
		CHECK(val != nullptr);
		CHECK(c->ValueLen == 2);

		if (val == nullptr || c->ValueLen != 2)
		{
			continue;
		}

		CHECK(val[0] == cases[i].Mode);
		CHECK(val[1] == cases[i].Level);
	}
}

// A central or an observer registers no GAP service, and must not be left with
// an error from a service it never added.
void TestNonPeripheralRolesAreClean(void)
{
	const uint8_t roles[] = {
		BT_GAP_ROLE_CENTRAL,
		BT_GAP_ROLE_OBSERVER,
		BT_GAP_ROLE_BROADCASTER,
	};

	for (size_t i = 0; i < sizeof(roles) / sizeof(roles[0]); i++)
	{
		ResetDb();

		BtGapCfg_t cfg = Cfg(roles[i], BTGAP_SECTYPE_NONE);

		CHECK(BtGapInit(&cfg));
	}
}

// A null configuration is the one case that should report an error, so the
// checks above are known to be able to fail.
void TestNullConfigIsReported(void)
{
	ResetDb();

	CHECK(BtGapInit(nullptr) == false);
}

}	// namespace

// Stubs for what bt_gatt.cpp reaches beyond the GATT server itself. None is
// called by BtGapInit; they exist so the two real files can be linked.
void BtGattSrvcEvtHandler(BtGattSrvc_t * const, uint32_t, void * const)
{
}

uint32_t BtHciSendAcl(BtHciDevice_t * const, BtHciACLDataPacket_t * const)
{
	return 0;
}

uint16_t BtPeerCount(void)
{
	return 0;
}

BtDevice_t *BtPeerFindByHdl(uint16_t)
{
	return nullptr;
}

BtDevice_t *BtPeerSlot(uint16_t)
{
	return nullptr;
}

size_t BtPeerGetConnectedHandles(uint16_t *, size_t)
{
	return 0;
}

uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t)
{
	return 0;
}

void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t)
{
}

bool BtSmpBonded(uint16_t)
{
	return false;
}

bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *)
{
	return false;
}

int main(void)
{
	TestPeripheralInitLeavesNoError();
	TestEverySecTypeInitialises();
	TestSecurityLevelsValueIsWritable();
	TestRegisteredValueMatchesSecType();
	TestNonPeripheralRolesAreClean();
	TestNullConfigIsReported();

	if (s_Failures != 0)
	{
		std::printf("GAP with the real GATT server: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GAP with the real GATT server: PASS (%d checks)\n", s_Checks);

	return 0;
}
