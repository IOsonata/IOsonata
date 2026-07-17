/**-------------------------------------------------------------------------
@file	crypto_uecc_test.cpp

@brief	Host validation for CryptoUecc through its OO facets.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/crypto_uecc.h"
#include "crypto/crypto_softrng.h"

class TestSecureRng : public CryptoSoftRng {
public:
	bool IsSecure() const override { return true; }
};

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

alignas(CryptoUecc::KeyCtx) static uint8_t g_ctxA[64];
alignas(CryptoUecc::KeyCtx) static uint8_t g_ctxB[64];

int main(void)
{
	printf("CryptoUecc OO engine validation\n");

	alignas(TestSecureRng) static uint8_t rngMem[sizeof(TestSecureRng)];
	TestSecureRng *secureRng = new (rngMem) TestSecureRng();
	secureRng->Enable();

	alignas(CryptoUecc) static uint8_t memA[CRYPTO_UECC_MEMSIZE];
	alignas(CryptoUecc) static uint8_t memB[CRYPTO_UECC_MEMSIZE];
	CryptoUecc *engineA = CryptoUeccCreate(memA, sizeof(memA), secureRng);
	CryptoUecc *engineB = CryptoUeccCreate(memB, sizeof(memB), secureRng);
	check("aligned factories construct engines", engineA != nullptr && engineB != nullptr);
	if (engineA == nullptr || engineB == nullptr) return 1;

	KeyAgreeEngine *agreeA = engineA;
	KeyAgreeEngine *agreeB = engineB;
	SignEngine *signA = engineA;
	check("one object resolves both facets", agreeA != nullptr && signA != nullptr);
	check("BLE P-256 known-answer self-test", engineA->SelfTest() == 0);
	check("key context size fits caller storage",
		agreeA->KeyCtxSize() > 0U && agreeA->KeyCtxSize() <= sizeof(g_ctxA));

	uint8_t pubA[64], pubB[64], sharedA[32], sharedB[32];
	CRYPTO_STATUS keyA = agreeA->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA);
	CRYPTO_STATUS keyB = agreeB->KeyGen(CRYPTO_CURVE_P256, g_ctxB, pubB);
	CRYPTO_STATUS dhA = agreeA->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, sharedA);
	CRYPTO_STATUS dhB = agreeB->Agree(CRYPTO_CURVE_P256, g_ctxB, pubA, sharedB);
	check("two generated key pairs derive the same secret",
		keyA == CRYPTO_STATUS_OK && keyB == CRYPTO_STATUS_OK &&
		dhA == CRYPTO_STATUS_OK && dhB == CRYPTO_STATUS_OK &&
		memcmp(sharedA, sharedB, sizeof(sharedA)) == 0);

	uint8_t badPeer[64]; memset(badPeer, 0xAB, sizeof(badPeer));
	check("invalid curve point is rejected",
		agreeA->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA) == CRYPTO_STATUS_OK &&
		agreeA->Agree(CRYPTO_CURVE_P256, g_ctxA, badPeer, sharedA) ==
			CRYPTO_STATUS_FAIL);

	check("single-use key rejects a second Agree",
		agreeA->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA) == CRYPTO_STATUS_OK &&
		agreeA->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, sharedA) ==
			CRYPTO_STATUS_OK &&
		agreeA->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, sharedA) ==
			CRYPTO_STATUS_FAIL);

	check("explicit KeyReset cancels an exchange",
		agreeA->KeyGen(CRYPTO_CURVE_P256, g_ctxA, pubA) == CRYPTO_STATUS_OK);
	agreeA->KeyReset(g_ctxA);
	check("Agree fails after explicit KeyReset",
		agreeA->Agree(CRYPTO_CURVE_P256, g_ctxA, pubB, sharedA) ==
			CRYPTO_STATUS_FAIL);

	static const uint8_t privateKey[32] = {
		0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,
		0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd };
	static const uint8_t publicKey[64] = {
		0x20,0xb0,0x03,0xd2,0xf2,0x97,0xbe,0x2c,0x5e,0x2c,0x83,0xa7,0xe9,0xf9,0xa5,0xb9,
		0xef,0xf4,0x91,0x11,0xac,0xf4,0xfd,0xdb,0xcc,0x03,0x01,0x48,0x0e,0x35,0x9d,0xe6,
		0xdc,0x80,0x9c,0x49,0x65,0x2a,0xeb,0x6d,0x63,0x32,0x9a,0xbf,0x5a,0x52,0x15,0x5c,
		0x76,0x63,0x45,0xc2,0x8f,0xed,0x30,0x24,0x74,0x1c,0x8e,0xd0,0x15,0x89,0xd2,0x8b };
	uint8_t hash[32];
	for (int i = 0; i < 32; i++) hash[i] = (uint8_t)(i * 7 + 1);
	CryptoKey signingKey{CRYPTO_KEY_ECC_P256, CRYPTO_KEY_LOC_PLAIN,
						 CRYPTO_KEY_USE_SIGN, {}};
	signingKey.Plain.pData = privateKey;
	signingKey.Plain.Len = sizeof(privateKey);
	uint8_t signature[64];
	check("ECDSA sign succeeds",
		signA->Sign(CRYPTO_CURVE_P256, signingKey, hash, sizeof(hash), signature) ==
			CRYPTO_STATUS_OK);
	check("ECDSA verify accepts valid signature",
		signA->Verify(CRYPTO_CURVE_P256, publicKey, hash, sizeof(hash), signature) ==
			CRYPTO_STATUS_OK);
	signature[0] ^= 1U;
	check("ECDSA verify rejects modified signature",
		signA->Verify(CRYPTO_CURVE_P256, publicKey, hash, sizeof(hash), signature) ==
			CRYPTO_STATUS_FAIL);

	alignas(CryptoSoftRng) static uint8_t prngMem[sizeof(CryptoSoftRng)];
	CryptoSoftRng *prng = CryptoSoftRngCreate(prngMem, sizeof(prngMem));
	alignas(CryptoUecc) static uint8_t weakMem[CRYPTO_UECC_MEMSIZE];
	CryptoUecc *weakEngine = CryptoUeccCreate(weakMem, sizeof(weakMem), prng);
	alignas(CryptoUecc::KeyCtx) uint8_t weakCtx[64];
	check("key generation refuses a non-secure PRNG",
		weakEngine != nullptr && weakEngine->KeyGen(CRYPTO_CURVE_P256,
			weakCtx, pubA) == CRYPTO_STATUS_UNSUPPORTED);

	// Agree runs the private scalar against a caller-supplied point. The
	// engine binds its RNG so micro-ecc applies the initial-Z randomization,
	// and refuses to run without a security-grade source.
	CryptoUecc::KeyCtx *weakKey = (CryptoUecc::KeyCtx *)weakCtx;
	memcpy(weakKey->PrivKey, privateKey, sizeof(privateKey));
	weakKey->bKeyValid = true;
	check("Agree refuses a non-secure PRNG",
		weakEngine != nullptr &&
		weakEngine->Agree(CRYPTO_CURVE_P256, weakCtx, publicKey, sharedA) ==
			CRYPTO_STATUS_FAIL);
	check("refused Agree wipes the retained key", weakKey->bKeyValid == false);

	alignas(CryptoUecc) static uint8_t noRngMem[CRYPTO_UECC_MEMSIZE];
	CryptoUecc *noRngEngine = CryptoUeccCreate(noRngMem, sizeof(noRngMem),
											   nullptr);
	alignas(CryptoUecc::KeyCtx) uint8_t noRngCtx[64];
	memset(noRngCtx, 0, sizeof(noRngCtx));
	CryptoUecc::KeyCtx *noRngKey = (CryptoUecc::KeyCtx *)noRngCtx;
	memcpy(noRngKey->PrivKey, privateKey, sizeof(privateKey));
	noRngKey->bKeyValid = true;
	check("Agree refuses without an RNG",
		noRngEngine != nullptr &&
		noRngEngine->Agree(CRYPTO_CURVE_P256, noRngCtx, publicKey, sharedA) ==
			CRYPTO_STATUS_FAIL);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
