/**-------------------------------------------------------------------------
@file	ba414ep_test.cpp

@brief	On-target validation for the Silex BA414EP P-256 engine.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cracen_intrf.h"
#include "crypto/ba414ep.h"
#include "crypto/crypto_uecc.h"
#include "crypto_rng_nrf.h"

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

static const uint8_t privA[32] = {
	0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
	0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
static const uint8_t pubA[64] = {
	0x20,0xb0,0x03,0xd2,0xf2,0x97,0xbe,0x2c,0x5e,0x2c,0x83,0xa7,0xe9,0xf9,0xa5,0xb9,
	0xef,0xf4,0x91,0x11,0xac,0xf4,0xfd,0xdb,0xcc,0x03,0x01,0x48,0x0e,0x35,0x9d,0xe6,
	0xdc,0x80,0x9c,0x49,0x65,0x2a,0xeb,0x6d,0x63,0x32,0x9a,0xbf,0x5a,0x52,0x15,0x5c,
	0x76,0x63,0x45,0xc2,0x8f,0xed,0x30,0x24,0x74,0x1c,0x8e,0xd0,0x15,0x89,0xd2,0x8b };

int main(void)
{
	printf("BA414EP hardware P-256 validation\n");
	RngEngine *rng = CryptoRngNrfInstance();
	check("hardware RNG instance", rng != nullptr);
	if (rng == nullptr || !rng->Enable()) return 1;

	static Ba414ep hardwareEngine;
	Ba414ep *hardware = hardwareEngine.Init(CracenIntrfInstance(), rng) ?
		&hardwareEngine : nullptr;
	alignas(CryptoUecc) static uint8_t swMem[CRYPTO_UECC_MEMSIZE];
	CryptoUecc *software = CryptoUeccCreate(swMem, sizeof(swMem), rng);
	check("hardware and software engines construct",
		hardware != nullptr && software != nullptr);
	if (hardware == nullptr || software == nullptr) return 1;

	alignas(Ba414ep::KeyCtx) uint8_t hwCtx[64];
	alignas(CryptoUecc::KeyCtx) uint8_t swCtx[64];
	uint8_t hwPub[64], swPub[64], hwShared[32], swShared[32];
	bool generated = hardware->KeyGen(CRYPTO_CURVE_P256, hwCtx, hwPub) ==
		CRYPTO_STATUS_OK && software->KeyGen(CRYPTO_CURVE_P256, swCtx, swPub) ==
		CRYPTO_STATUS_OK;
	check("hardware and software KeyGen", generated);
	bool agreed = generated &&
		hardware->Agree(CRYPTO_CURVE_P256, hwCtx, swPub, hwShared) ==
			CRYPTO_STATUS_OK &&
		software->Agree(CRYPTO_CURVE_P256, swCtx, hwPub, swShared) ==
			CRYPTO_STATUS_OK;
	check("cross ECDH produces identical shared X",
		agreed && memcmp(hwShared, swShared, sizeof(hwShared)) == 0);

	alignas(Ba414ep::KeyCtx) uint8_t fixedHw[64];
	alignas(CryptoUecc::KeyCtx) uint8_t fixedSw[64];
	hardware->KeyReset(fixedHw);
	software->KeyReset(fixedSw);
	memcpy(((Ba414ep::KeyCtx *)fixedHw)->PrivKey, privA, sizeof(privA));
	((Ba414ep::KeyCtx *)fixedHw)->bKeyValid = true;
	memcpy(((CryptoUecc::KeyCtx *)fixedSw)->PrivKey, privA, sizeof(privA));
	((CryptoUecc::KeyCtx *)fixedSw)->bKeyValid = true;
	uint8_t fixedHwSecret[32], fixedSwSecret[32];
	check("fixed BLE vector hardware equals software",
		hardware->Agree(CRYPTO_CURVE_P256, fixedHw, pubA, fixedHwSecret) ==
			CRYPTO_STATUS_OK &&
		software->Agree(CRYPTO_CURVE_P256, fixedSw, pubA, fixedSwSecret) ==
			CRYPTO_STATUS_OK &&
		memcmp(fixedHwSecret, fixedSwSecret, sizeof(fixedHwSecret)) == 0);

	alignas(Ba414ep::KeyCtx) uint8_t invalidCtx[64];
	uint8_t generatedPub[64], badPoint[64], ignored[32];
	memset(badPoint, 0xAB, sizeof(badPoint));
	check("off-curve peer point rejected",
		hardware->KeyGen(CRYPTO_CURVE_P256, invalidCtx, generatedPub) ==
			CRYPTO_STATUS_OK &&
		hardware->Agree(CRYPTO_CURVE_P256, invalidCtx, badPoint, ignored) ==
			CRYPTO_STATUS_FAIL);

	alignas(Ba414ep::KeyCtx) uint8_t resetCtx[64];
	check("KeyGen before explicit reset",
		hardware->KeyGen(CRYPTO_CURVE_P256, resetCtx, generatedPub) ==
			CRYPTO_STATUS_OK);
	hardware->KeyReset(resetCtx);
	check("Agree fails after explicit KeyReset",
		hardware->Agree(CRYPTO_CURVE_P256, resetCtx, pubA, ignored) ==
			CRYPTO_STATUS_FAIL);

	alignas(Ba414ep::KeyCtx) uint8_t singleCtx[64];
	check("single-use key rejects second Agree",
		hardware->KeyGen(CRYPTO_CURVE_P256, singleCtx, generatedPub) ==
			CRYPTO_STATUS_OK &&
		hardware->Agree(CRYPTO_CURVE_P256, singleCtx, pubA, ignored) ==
			CRYPTO_STATUS_OK &&
		hardware->Agree(CRYPTO_CURVE_P256, singleCtx, pubA, ignored) ==
			CRYPTO_STATUS_FAIL);

	// CRACEN contention. Take the operation hold the way another engine
	// would, then confirm every crypto path fails cleanly, the foreign hold
	// is neither stolen nor released, and normal operation resumes after
	// release. The engines draw their entropy before taking the public-key
	// hold, so under a whole-core hold the failure surfaces at the entropy
	// draw; the observable fail-closed outcome is the same.
	CracenIntrf *cracen = CracenIntrfInstance();
	alignas(Ba414ep::KeyCtx) uint8_t contCtx[64];
	hardware->KeyReset(contCtx);
	memcpy(((Ba414ep::KeyCtx *)contCtx)->PrivKey, privA, sizeof(privA));
	((Ba414ep::KeyCtx *)contCtx)->bKeyValid = true;
	bool held = cracen->ModuleHold(CRACEN_MODULE_RNG);
	check("interface accepts one owner and rejects a second",
		held && !cracen->ModuleHold(CRACEN_MODULE_PKEIKG));
	uint8_t entropy[16];
	check("entropy draw fails while CRACEN is held",
		held && rng->Random(entropy, sizeof(entropy)) != CRYPTO_STATUS_OK);
	check("Agree fails while CRACEN is held",
		held && hardware->Agree(CRYPTO_CURVE_P256, contCtx, pubA, ignored) !=
			CRYPTO_STATUS_OK);
	check("KeyGen fails while CRACEN is held",
		held && hardware->KeyGen(CRYPTO_CURVE_P256, contCtx, generatedPub) ==
			CRYPTO_STATUS_FAIL);
	check("foreign hold survives the rejected operations",
		held && !cracen->ModuleHold(CRACEN_MODULE_RNG));
	if (held)
	{
		cracen->ModuleRelease();
	}
	check("KeyGen recovers after release",
		hardware->KeyGen(CRYPTO_CURVE_P256, contCtx, generatedPub) ==
			CRYPTO_STATUS_OK);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
