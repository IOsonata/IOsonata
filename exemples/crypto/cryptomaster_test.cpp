/**-------------------------------------------------------------------------
@file	cryptomaster_test.cpp

@brief	On-target validation for the CryptoMaster hardware AES engine.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cracen_intrf.h"
#include "crypto/cryptomaster.h"
#include "crypto/crypto_softaes.h"

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

static const uint8_t keyBytes[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
static const uint8_t message[64] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
	0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
	0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
	0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
static const uint8_t mac0[16] = {
	0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };
static const uint8_t mac16[16] = {
	0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c };
static const uint8_t mac40[16] = {
	0xdf,0xa6,0x67,0x47,0xde,0x9a,0xe6,0x30,0x30,0xca,0x32,0x61,0x14,0x97,0xc8,0x27 };
static const uint8_t mac64[16] = {
	0x51,0xf0,0xbe,0xbf,0x7e,0x3b,0x9d,0x92,0xfc,0x49,0x74,0x17,0x79,0x36,0x3c,0xfe };

int main(void)
{
	printf("CryptoMaster hardware AES validation\n");
	static CryptoMaster hardwareEngine;
	CryptoMaster *hardware = hardwareEngine.Init(CracenIntrfInstance()) ?
		&hardwareEngine : nullptr;
	alignas(CryptoSoftAes) static uint8_t swMem[CRYPTO_SOFTAES_MEMSIZE];
	CryptoSoftAes *software = CryptoSoftAesCreate(swMem, sizeof(swMem));
	check("hardware and software engines construct",
		hardware != nullptr && software != nullptr);
	if (hardware == nullptr || software == nullptr) return 1;

	CipherEngine *cipher = hardware;
	MacEngine *macEngine = hardware;
	static const uint8_t katKey[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t katIn[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
	static const uint8_t katExpected[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };
	CryptoKey kat{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
				  CRYPTO_KEY_USE_ENCRYPT, {}};
	kat.Plain.pData = katKey;
	kat.Plain.Len = sizeof(katKey);
	uint8_t result[16];
	check("hardware ECB matches FIPS-197",
		cipher->Cipher(CRYPTO_CIPHER_ECB, 1, kat, nullptr, 0,
					   katIn, sizeof(katIn), result) == CRYPTO_STATUS_OK &&
		memcmp(result, katExpected, sizeof(result)) == 0);

	bool equal = true;
	for (int test = 0; test < 8 && equal; test++)
	{
		uint8_t key[16], input[16], hw[16], sw[16];
		for (int i = 0; i < 16; i++)
		{
			key[i] = (uint8_t)(test * 16 + i);
			input[i] = (uint8_t)(255 - test * 16 - i);
		}
		CryptoKey descriptor{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
							 CRYPTO_KEY_USE_ENCRYPT, {}};
		descriptor.Plain.pData = key;
		descriptor.Plain.Len = sizeof(key);
		equal = cipher->Cipher(CRYPTO_CIPHER_ECB, 1, descriptor, nullptr, 0,
							 input, sizeof(input), hw) == CRYPTO_STATUS_OK &&
			software->Cipher(CRYPTO_CIPHER_ECB, 1, descriptor, nullptr, 0,
							 input, sizeof(input), sw) == CRYPTO_STATUS_OK &&
			memcmp(hw, sw, sizeof(hw)) == 0;
	}
	check("hardware ECB equals software ECB", equal);

	CryptoKey signKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {}};
	signKey.Plain.pData = keyBytes;
	signKey.Plain.Len = sizeof(keyBytes);
	uint8_t tag[16];
	bool cmac = macEngine->Mac(CRYPTO_MAC_CMAC, signKey, nullptr, 0,
							 tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, mac0, 16) == 0 &&
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, message, 16,
							 tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, mac16, 16) == 0 &&
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, message, 40,
							 tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, mac40, 16) == 0 &&
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, message, 64,
							 tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, mac64, 16) == 0;
	check("inherited CMAC over hardware AES", cmac);

	CryptoKey ctrKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	ctrKey.Plain.pData = keyBytes;
	ctrKey.Plain.Len = sizeof(keyBytes);
	uint8_t iv[16]; memset(iv, 0x24, sizeof(iv));
	uint8_t encrypted[40], plain[40];
	check("hardware CTR encrypt/decrypt round trip",
		cipher->Cipher(CRYPTO_CIPHER_CTR, 1, ctrKey, iv, 16,
					   message, 40, encrypted) == CRYPTO_STATUS_OK &&
		cipher->Cipher(CRYPTO_CIPHER_CTR, 0, ctrKey, iv, 16,
					   encrypted, 40, plain) == CRYPTO_STATUS_OK &&
		memcmp(plain, message, 40) == 0);

	CryptoKey denied = kat;
	denied.Usage = CRYPTO_KEY_USE_DERIVE;
	check("hardware cipher rejects disallowed key usage",
		cipher->Cipher(CRYPTO_CIPHER_ECB, 1, denied, nullptr, 0,
					   katIn, 16, result) == CRYPTO_STATUS_UNSUPPORTED);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
