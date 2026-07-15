/**-------------------------------------------------------------------------
@file	crypto_softaes_test.cpp

@brief	Host validation harness for the OO CryptoSoftAes engine.

		Exercises CryptoSoftAes through the CipherEngine and MacEngine facet
		interfaces. Includes only crypto_softaes.h. Runs on the build host.

		Tests:
		  1. FIPS-197 AES-128 ECB known-answer.
		  2. RFC 4493 AES-CMAC known-answers (all four example message lengths:
		     0, 16, 40, 64 bytes).
		  3. AES-CTR encrypt then decrypt round trip (CTR is its own inverse).
		  4. AES-CBC self-consistency across a multi-block message.
		  5. Self-test (FIPS-197 + RFC 4493 empty CMAC).
		  6. Delegation over override: a subclass that overrides Cipher with a
		     call-counting wrapper proves the INHERITED software CMAC routes its
		     AES blocks through the override, not a private path. This is the
		     core of the base-software / hardware-override design.

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softaes.h"

static int s_pass = 0, s_fail = 0;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

// RFC 4493 example key (used for all CMAC vectors).
static const uint8_t kCmacKey[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
	0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
// RFC 4493 example message (first 64 bytes; sub-lengths reuse the prefix).
static const uint8_t kMsg[64] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
	0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
	0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
	0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
// RFC 4493 expected CMACs for lengths 0, 16, 40, 64.
static const uint8_t kMac0[16] = {
	0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };
static const uint8_t kMac16[16] = {
	0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c };
static const uint8_t kMac40[16] = {
	0xdf,0xa6,0x67,0x47,0xde,0x9a,0xe6,0x30,0x30,0xca,0x32,0x61,0x14,0x97,0xc8,0x27 };
static const uint8_t kMac64[16] = {
	0x51,0xf0,0xbe,0xbf,0x7e,0x3b,0x9d,0x92,0xfc,0x49,0x74,0x17,0x79,0x36,0x3c,0xfe };

// A hardware-simulating subclass: overrides Cipher and counts calls, so the
// test can prove the inherited MacEngine::Mac routes through the override.
class CountingAes : public CryptoSoftAes {
public:
	int CipherCalls = 0;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key, const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut) override
	{
		CipherCalls++;
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen,
									 pIn, Len, pOut);
	}
};

int main(void)
{
	printf("CryptoSoftAes OO engine validation\n");

	static uint8_t mem[128];
	CryptoSoftAes *e = CryptoSoftAesCreate(mem, sizeof(mem));
	check("factory constructs engine", e != nullptr);
	if (e == nullptr) { printf("abort\n"); return 1; }

	CipherEngine *ce = e;
	MacEngine    *me = e;
	check("Cipher + Mac facet pointers resolve", ce != nullptr && me != nullptr);

	// Test 1: FIPS-197 AES-128 ECB known-answer.
	{
		static const uint8_t key[16] = {
			0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
			0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
		static const uint8_t in[16] = {
			0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
			0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
		static const uint8_t exp[16] = {
			0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
			0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_ENCRYPT, {} };
		k.Plain.pData = key; k.Plain.Len = 16;
		uint8_t out[16];
		CRYPTO_STATUS r = ce->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, out);
		check("FIPS-197 AES-128 ECB known-answer",
			  r == CRYPTO_STATUS_OK && memcmp(out, exp, 16) == 0);
	}

	// Test 2: RFC 4493 CMAC at lengths 0, 16, 40, 64.
	{
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {} };
		k.Plain.pData = kCmacKey; k.Plain.Len = 16;
		uint8_t mac[16];
		CRYPTO_STATUS r0 = me->Mac(CRYPTO_MAC_CMAC, k, nullptr, 0, mac, 16);
		check("RFC 4493 CMAC len 0",  r0 == CRYPTO_STATUS_OK && memcmp(mac, kMac0, 16) == 0);
		CRYPTO_STATUS r1 = me->Mac(CRYPTO_MAC_CMAC, k, kMsg, 16, mac, 16);
		check("RFC 4493 CMAC len 16", r1 == CRYPTO_STATUS_OK && memcmp(mac, kMac16, 16) == 0);
		CRYPTO_STATUS r2 = me->Mac(CRYPTO_MAC_CMAC, k, kMsg, 40, mac, 16);
		check("RFC 4493 CMAC len 40", r2 == CRYPTO_STATUS_OK && memcmp(mac, kMac40, 16) == 0);
		CRYPTO_STATUS r3 = me->Mac(CRYPTO_MAC_CMAC, k, kMsg, 64, mac, 16);
		check("RFC 4493 CMAC len 64", r3 == CRYPTO_STATUS_OK && memcmp(mac, kMac64, 16) == 0);
	}

	// Test 3: AES-CTR encrypt then decrypt round trip.
	{
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_ENCRYPT, {} };
		k.Plain.pData = kCmacKey; k.Plain.Len = 16;
		uint8_t iv[16]; memset(iv, 0x24, 16);
		uint8_t ct[40], pt[40];
		CRYPTO_STATUS re = ce->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, kMsg, 40, ct);
		CRYPTO_STATUS rd = ce->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, ct, 40, pt);
		check("AES-CTR encrypt/decrypt round trip",
			  re == CRYPTO_STATUS_OK && rd == CRYPTO_STATUS_OK &&
			  memcmp(pt, kMsg, 40) == 0 && memcmp(ct, kMsg, 40) != 0);
	}

	// Test 4: AES-CBC produces block-chained output distinct from ECB.
	{
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_ENCRYPT, {} };
		k.Plain.pData = kCmacKey; k.Plain.Len = 16;
		uint8_t iv[16]; memset(iv, 0, 16);
		uint8_t cbc[32], ecb[32];
		// Two identical plaintext blocks: ECB gives identical ciphertext blocks,
		// CBC gives different ones. Proves chaining is active.
		uint8_t twoSame[32];
		memcpy(twoSame, kMsg, 16); memcpy(twoSame + 16, kMsg, 16);
		CRYPTO_STATUS rc = ce->Cipher(CRYPTO_CIPHER_CBC, 1, k, iv, 16, twoSame, 32, cbc);
		CRYPTO_STATUS rz = ce->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, twoSame, 32, ecb);
		bool ecbEqualBlocks = memcmp(ecb, ecb + 16, 16) == 0;
		bool cbcDiffBlocks  = memcmp(cbc, cbc + 16, 16) != 0;
		check("AES-CBC chaining active (blocks differ where ECB repeats)",
			  rc == CRYPTO_STATUS_OK && rz == CRYPTO_STATUS_OK &&
			  ecbEqualBlocks && cbcDiffBlocks);
	}

	// Test 5: engine self-test.
	check("SelfTest (FIPS-197 + RFC 4493 empty CMAC)", e->SelfTest() == 0);

	// Test 6: delegation over override. The inherited software CMAC must route
	// its AES blocks through the overridden Cipher. For a 16-byte message CMAC
	// performs two AES blocks (subkey L, plus the single message block), so the
	// override must be called at least twice and the result must still match the
	// RFC 4493 vector, proving correctness through the override path.
	{
		static uint8_t cmem[128];
		CountingAes *ca = new (cmem) CountingAes();
		ca->Enable();
		MacEngine *cme = ca;
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {} };
		k.Plain.pData = kCmacKey; k.Plain.Len = 16;
		uint8_t mac[16];
		ca->CipherCalls = 0;
		CRYPTO_STATUS r = cme->Mac(CRYPTO_MAC_CMAC, k, kMsg, 16, mac, 16);
		check("inherited CMAC routes through overridden Cipher",
			  r == CRYPTO_STATUS_OK && ca->CipherCalls >= 2 &&
			  memcmp(mac, kMac16, 16) == 0);
		printf("       (Cipher override invoked %d times for a 16-byte CMAC)\n",
			   ca->CipherCalls);
	}

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
