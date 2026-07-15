/**-------------------------------------------------------------------------
@file	cryptomaster_test.cpp

@brief	On-target validation harness for the CryptoMaster hardware AES engine.

		Runs on an nRF54 target (needs the CRACEN block). Validates the hardware
		AES through the CipherEngine facet, the inherited CMAC through the
		MacEngine facet, and cross-checks the hardware output against the
		software CryptoSoftAes engine so any register or descriptor error shows
		up as a mismatch rather than a wrong-but-plausible value.

		Same shape as the other crypto examples: int main and printf, retargeted
		to the board console by the target build. Build and flash it as they are.

		Tests:
		  1. Engine constructs and reports the AES sub-engine present.
		  2. Hardware AES-128 ECB matches the FIPS-197 known-answer.
		  3. Hardware AES equals the software engine on random blocks.
		  4. Inherited CMAC over hardware AES matches the RFC 4493 vectors.
		  5. Hardware CTR encrypt/decrypt round trip.

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crypto/cryptomaster.h"
#include "crypto/crypto_softaes.h"

static int s_pass = 0, s_fail = 0;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

// RFC 4493 vectors (key, message prefix, expected CMACs at 0/16/40/64).
static const uint8_t kCmacKey[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
static const uint8_t kMsg[64] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
	0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
	0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
	0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
static const uint8_t kMac0[16]  = {
	0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };
static const uint8_t kMac16[16] = {
	0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c };
static const uint8_t kMac40[16] = {
	0xdf,0xa6,0x67,0x47,0xde,0x9a,0xe6,0x30,0x30,0xca,0x32,0x61,0x14,0x97,0xc8,0x27 };
static const uint8_t kMac64[16] = {
	0x51,0xf0,0xbe,0xbf,0x7e,0x3b,0x9d,0x92,0xfc,0x49,0x74,0x17,0x79,0x36,0x3c,0xfe };

int main(void)
{
	printf("CryptoMaster hardware AES validation\n");

	static uint8_t hwMem[128], swMem[128];
	CryptoMaster *hw = CryptoMasterCreate(hwMem, sizeof(hwMem));
	CryptoSoftAes *sw = CryptoSoftAesCreate(swMem, sizeof(swMem));
	check("engine constructs, AES sub-engine present", hw != nullptr);
	if (hw == nullptr) { printf("abort: no CryptoMaster\n"); return 1; }

	CipherEngine *hce = hw;
	MacEngine    *hme = hw;

	// Test 2: hardware AES-128 ECB vs FIPS-197.
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
		CRYPTO_STATUS r = hce->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, out);
		check("hardware AES-128 ECB == FIPS-197",
			  r == CRYPTO_STATUS_OK && memcmp(out, exp, 16) == 0);
	}

	// Test 3: hardware AES == software AES on varied blocks.
	if (sw != nullptr)
	{
		bool allEq = true;
		for (int t = 0; t < 8 && allEq; t++)
		{
			uint8_t key[16], in[16], ho[16], so[16];
			for (int j = 0; j < 16; j++) { key[j] = (uint8_t)(t*16+j); in[j] = (uint8_t)(255-(t*16+j)); }
			CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
						 CRYPTO_KEY_USE_ENCRYPT, {} };
			k.Plain.pData = key; k.Plain.Len = 16;
			CRYPTO_STATUS rh = hce->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, ho);
			CRYPTO_STATUS rs = ((CipherEngine *)sw)->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, so);
			if (rh != CRYPTO_STATUS_OK || rs != CRYPTO_STATUS_OK ||
				memcmp(ho, so, 16) != 0)
			{
				allEq = false;
			}
		}
		check("hardware AES == software AES (8 random blocks)", allEq);
	}

	// Test 4: inherited CMAC over hardware AES vs RFC 4493.
	{
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {} };
		k.Plain.pData = kCmacKey; k.Plain.Len = 16;
		uint8_t mac[16];
		bool ok0 = hme->Mac(CRYPTO_MAC_CMAC, k, nullptr, 0, mac, 16) == CRYPTO_STATUS_OK && memcmp(mac, kMac0, 16) == 0;
		bool ok1 = hme->Mac(CRYPTO_MAC_CMAC, k, kMsg, 16, mac, 16) == CRYPTO_STATUS_OK && memcmp(mac, kMac16, 16) == 0;
		bool ok2 = hme->Mac(CRYPTO_MAC_CMAC, k, kMsg, 40, mac, 16) == CRYPTO_STATUS_OK && memcmp(mac, kMac40, 16) == 0;
		bool ok3 = hme->Mac(CRYPTO_MAC_CMAC, k, kMsg, 64, mac, 16) == CRYPTO_STATUS_OK && memcmp(mac, kMac64, 16) == 0;
		check("inherited CMAC over HW AES == RFC 4493 (len 0/16/40/64)",
			  ok0 && ok1 && ok2 && ok3);
	}

	// Test 5: hardware CTR round trip.
	{
		CryptoKey k{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_ENCRYPT, {} };
		k.Plain.pData = kCmacKey; k.Plain.Len = 16;
		uint8_t iv[16]; memset(iv, 0x24, 16);
		uint8_t ct[40], pt[40];
		CRYPTO_STATUS re = hce->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, kMsg, 40, ct);
		CRYPTO_STATUS rd = hce->Cipher(CRYPTO_CIPHER_CTR, 1, k, iv, 16, ct, 40, pt);
		check("hardware AES-CTR encrypt/decrypt round trip",
			  re == CRYPTO_STATUS_OK && rd == CRYPTO_STATUS_OK &&
			  memcmp(pt, kMsg, 40) == 0 && memcmp(ct, kMsg, 40) != 0);
	}

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
