/**-------------------------------------------------------------------------
@file	crypto_softaes_test.cpp

@brief	Host validation for the OO CryptoSoftAes engine.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softaes.h"

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

static const uint8_t kCmacKey[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
	0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
static const uint8_t kMsg[64] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
	0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
	0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
	0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
static const uint8_t kMac0[16] = {
	0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };
static const uint8_t kMac16[16] = {
	0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c };
static const uint8_t kMac40[16] = {
	0xdf,0xa6,0x67,0x47,0xde,0x9a,0xe6,0x30,0x30,0xca,0x32,0x61,0x14,0x97,0xc8,0x27 };
static const uint8_t kMac64[16] = {
	0x51,0xf0,0xbe,0xbf,0x7e,0x3b,0x9d,0x92,0xfc,0x49,0x74,0x17,0x79,0x36,0x3c,0xfe };

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

	alignas(CryptoSoftAes) static uint8_t mem[CRYPTO_SOFTAES_MEMSIZE];
	CryptoSoftAes *engine = CryptoSoftAesCreate(mem, sizeof(mem));
	check("factory constructs aligned engine", engine != nullptr);
	if (engine == nullptr) return 1;
	CipherEngine *cipher = engine;
	MacEngine *macEngine = engine;

	static const uint8_t aesKey[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t aesIn[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
	static const uint8_t aesExpected[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };

	CryptoKey encryptKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
						CRYPTO_KEY_USE_ENCRYPT, {}};
	encryptKey.Plain.pData = aesKey;
	encryptKey.Plain.Len = sizeof(aesKey);
	uint8_t out[16];
	check("FIPS-197 AES-128 ECB known-answer",
		cipher->Cipher(CRYPTO_CIPHER_ECB, 1, encryptKey, nullptr, 0,
					   aesIn, sizeof(aesIn), out) == CRYPTO_STATUS_OK &&
		memcmp(out, aesExpected, sizeof(out)) == 0);

	CryptoKey signKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {}};
	signKey.Plain.pData = kCmacKey;
	signKey.Plain.Len = sizeof(kCmacKey);
	uint8_t tag[16];
	check("RFC 4493 CMAC len 0",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, nullptr, 0, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac0, 16) == 0);
	check("RFC 4493 CMAC len 16",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 16, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac16, 16) == 0);
	check("RFC 4493 CMAC len 40",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 40, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac40, 16) == 0);
	check("RFC 4493 CMAC len 64",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 64, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac64, 16) == 0);

	CryptoKey ctrKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	ctrKey.Plain.pData = kCmacKey;
	ctrKey.Plain.Len = sizeof(kCmacKey);
	uint8_t iv[16]; memset(iv, 0x24, sizeof(iv));
	uint8_t encrypted[40], plain[40];
	CRYPTO_STATUS enc = cipher->Cipher(CRYPTO_CIPHER_CTR, 1, ctrKey, iv, 16,
									kMsg, 40, encrypted);
	CRYPTO_STATUS dec = cipher->Cipher(CRYPTO_CIPHER_CTR, 0, ctrKey, iv, 16,
									encrypted, 40, plain);
	check("AES-CTR encrypt/decrypt round trip",
		enc == CRYPTO_STATUS_OK && dec == CRYPTO_STATUS_OK &&
		memcmp(plain, kMsg, 40) == 0);

	CryptoKey denied = encryptKey;
	denied.Usage = CRYPTO_KEY_USE_DERIVE;
	check("cipher rejects key without encrypt usage",
		cipher->Cipher(CRYPTO_CIPHER_ECB, 1, denied, nullptr, 0,
					   aesIn, 16, out) == CRYPTO_STATUS_UNSUPPORTED);
	check("CMAC rejects key without sign usage",
		macEngine->Mac(CRYPTO_MAC_CMAC, denied, nullptr, 0, tag, 16) ==
		CRYPTO_STATUS_UNSUPPORTED);
	check("CMAC rejects zero-length tag",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, nullptr, 0, tag, 0) ==
		CRYPTO_STATUS_UNSUPPORTED);

	check("engine self-test", engine->SelfTest() == 0);

	alignas(CountingAes) static uint8_t countingMem[sizeof(CountingAes)];
	CountingAes *counting = new (countingMem) CountingAes();
	counting->Enable();
	counting->CipherCalls = 0;
	check("inherited CMAC routes through overridden Cipher",
		counting->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 16, tag, 16) ==
			CRYPTO_STATUS_OK && counting->CipherCalls >= 2 &&
		memcmp(tag, kMac16, 16) == 0);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
