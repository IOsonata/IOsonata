/**-------------------------------------------------------------------------
@file	cracen_aes_test.cpp

@brief	nRF54 CRACEN AES and RNG acceptance test.

		Hardware acceptance test for crypto_cracen_bm.cpp. The test runs without
		Bluetooth or the SoftDevice so the CRACEN driver is exercised directly.

		The test checks provider selection and properties, AES-128 ECB known-answer
		vectors, in-place encryption, repeated CryptoMaster reset/start cycles,
		Cryptor forwarding, CMAC and GCM derived over the hardware AES primitive,
		rejection of unsupported P-256 requests, and the CRACEN-backed RNG path.

		Usage: build this file as a standalone nRF54 application linked with the
		nRF54 IOsonata library containing crypto_cracen_bm.cpp. Inspect
		g_CracenTestResult in the debugger: 1 is PASS; a negative value identifies
		the failed stage. Test scaffolding, not part of the library build.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See crypto.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "crypto/crypto.h"

// Result codes. One means complete success; a negative value identifies the
// first failed check and remains available after the CPU enters WFE.
enum {
	CRACEN_TEST_PASS                 = 1,
	CRACEN_TEST_ERR_HW_INIT          = -1,
	CRACEN_TEST_ERR_NAME             = -2,
	CRACEN_TEST_ERR_PROPERTY         = -3,
	CRACEN_TEST_ERR_CAPABILITY       = -4,
	CRACEN_TEST_ERR_SELFTEST         = -5,
	CRACEN_TEST_ERR_AES_KAT_1        = -6,
	CRACEN_TEST_ERR_AES_KAT_2        = -7,
	CRACEN_TEST_ERR_AES_IN_PLACE     = -8,
	CRACEN_TEST_ERR_AES_REPEAT       = -9,
	CRACEN_TEST_ERR_CRYPTOR_INIT     = -10,
	CRACEN_TEST_ERR_CRYPTOR_AES      = -11,
	CRACEN_TEST_ERR_CMAC             = -12,
	CRACEN_TEST_ERR_GCM_ENCRYPT      = -13,
	CRACEN_TEST_ERR_GCM_DECRYPT      = -14,
	CRACEN_TEST_ERR_GCM_TAG          = -15,
	CRACEN_TEST_ERR_UNSUPPORTED_ECDH = -16,
	CRACEN_TEST_ERR_RNG_INIT         = -17,
	CRACEN_TEST_ERR_RNG_GET          = -18,
	CRACEN_TEST_ERR_RNG_OUTPUT       = -19,
	CRACEN_TEST_ERR_AES_AFTER_RNG    = -20,
};

volatile int g_CracenTestResult;
volatile uint32_t g_CracenTestPassMask;
volatile uint8_t g_CracenTestLastAes[16];
volatile uint8_t g_CracenTestLastRng[32];

#define CRACEN_TEST_MARK(Bit) \
	do { g_CracenTestPassMask |= (1UL << (Bit)); } while (0)

static bool AllZero(const uint8_t *pData, size_t Len)
{
	uint8_t acc = 0;

	for (size_t i = 0; i < Len; i++)
	{
		acc |= pData[i];
	}

	return acc == 0;
}

static bool CracenTestFail(int Result)
{
	g_CracenTestResult = Result;
	return false;
}

bool CracenAesTest(void)
{
	static const uint8_t key1[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
	};
	static const uint8_t plain1[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff
	};
	static const uint8_t cipher1[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a
	};

	static const uint8_t key2[16] = {
		0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
		0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
	};
	static const uint8_t plain2[16] = {
		0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
		0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a
	};
	static const uint8_t cipher2[16] = {
		0x3a,0xd7,0x7b,0xb4,0x0d,0x7a,0x36,0x60,
		0xa8,0x9e,0xca,0xf3,0x24,0x66,0xef,0x97
	};

	// RFC 4493 example 1: AES-CMAC over an empty message.
	static const uint8_t cmacEmpty[16] = {
		0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,
		0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46
	};

	// NIST GCM test case: zero key, zero 96-bit IV, one zero block, no AAD.
	static const uint8_t zeroKey[16] = { 0 };
	static const uint8_t zeroIv[12] = { 0 };
	static const uint8_t zeroPlain[16] = { 0 };
	static const uint8_t gcmCipherExpected[16] = {
		0x03,0x88,0xda,0xce,0x60,0xb6,0xa3,0x92,
		0xf3,0x28,0xc2,0xb9,0x71,0xb2,0xfe,0x78
	};
	static const uint8_t gcmTagExpected[16] = {
		0xab,0x6e,0x47,0xd4,0x2c,0xec,0x13,0xbd,
		0xf5,0x3a,0x67,0xb2,0x12,0x57,0xbd,0xdf
	};

	CryptoDev_t hwDev;
	Cryptor_t cryptor;
	CryptoCfg_t hwCfg;
	CryptoCfg_t cryptorCfg;
	CryptoCfg_t unsupportedCfg;
	CryptoDev_t *pUse;
	uint8_t out[16];
	uint8_t inPlace[16];
	uint8_t cmac[16];
	uint8_t gcmCipher[16];
	uint8_t gcmPlain[16];
	uint8_t gcmTag[16];
	uint8_t badTag[16];
	uint8_t rng1[32];
	uint8_t rng2[32];

	g_CracenTestResult = 0;
	g_CracenTestPassMask = 0;
	memset((void *)g_CracenTestLastAes, 0, sizeof(g_CracenTestLastAes));
	memset((void *)g_CracenTestLastRng, 0, sizeof(g_CracenTestLastRng));
	memset(&hwDev, 0, sizeof(hwDev));
	memset(&cryptor, 0, sizeof(cryptor));
	memset(&hwCfg, 0, sizeof(hwCfg));
	memset(&cryptorCfg, 0, sizeof(cryptorCfg));
	memset(&unsupportedCfg, 0, sizeof(unsupportedCfg));
	memset(out, 0, sizeof(out));
	memset(inPlace, 0, sizeof(inPlace));
	memset(cmac, 0, sizeof(cmac));
	memset(gcmCipher, 0, sizeof(gcmCipher));
	memset(gcmPlain, 0, sizeof(gcmPlain));
	memset(gcmTag, 0, sizeof(gcmTag));
	memset(badTag, 0, sizeof(badTag));
	memset(rng1, 0, sizeof(rng1));
	memset(rng2, 0, sizeof(rng2));

	hwCfg.DevNo = 0;
	hwCfg.Provider = CRYPTO_PROVIDER_HW;
	hwCfg.ReqCaps = CRYPTO_CAP_AES128_ECB;
	hwCfg.Flags = CRYPTO_FLAG_SYNC |
				  CRYPTO_FLAG_SELFTEST |
				  CRYPTO_FLAG_NO_FALLBACK;

	if (!CryptoHwInit(&hwDev, &hwCfg))
	{
		return CracenTestFail(CRACEN_TEST_ERR_HW_INIT);
	}
	CRACEN_TEST_MARK(0);

	if (strcmp(CryptoName(&hwDev), "cracen-aes") != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_NAME);
	}
	CRACEN_TEST_MARK(1);

	if (!CryptoHasProp(&hwDev, CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC) ||
		CryptoHasProp(&hwDev, CRYPTO_PROP_PLAIN_KEYCTX))
	{
		return CracenTestFail(CRACEN_TEST_ERR_PROPERTY);
	}
	CRACEN_TEST_MARK(2);

	if (!CryptoIsCapable(&hwDev, CRYPTO_CAP_AES128_ECB |
								   CRYPTO_CAP_AES_CMAC |
								   CRYPTO_CAP_AES_CCM |
								   CRYPTO_CAP_AES_GCM) ||
		CryptoIsCapable(&hwDev, CRYPTO_CAP_ECDH_P256))
	{
		return CracenTestFail(CRACEN_TEST_ERR_CAPABILITY);
	}
	CRACEN_TEST_MARK(3);

	if (CryptoSelfTest(&hwDev) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_SELFTEST);
	}
	CRACEN_TEST_MARK(4);

	if (CryptoAes128Ecb(&hwDev, key1, plain1, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher1, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_AES_KAT_1);
	}
	memcpy((void *)g_CracenTestLastAes, out, sizeof(out));
	CRACEN_TEST_MARK(5);

	if (CryptoAes128Ecb(&hwDev, key2, plain2, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher2, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_AES_KAT_2);
	}
	CRACEN_TEST_MARK(6);

	memcpy(inPlace, plain1, sizeof(inPlace));
	if (CryptoAes128Ecb(&hwDev, key1, inPlace, inPlace, NULL) != CRYPTO_STATUS_OK ||
		memcmp(inPlace, cipher1, sizeof(inPlace)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_AES_IN_PLACE);
	}
	CRACEN_TEST_MARK(7);

	// Exercise repeated CryptoMaster reset, descriptor setup, start and release.
	for (unsigned i = 0; i < 128; i++)
	{
		if (CryptoAes128Ecb(&hwDev, key1, plain1, out, NULL) != CRYPTO_STATUS_OK ||
			memcmp(out, cipher1, sizeof(out)) != 0)
		{
			return CracenTestFail(CRACEN_TEST_ERR_AES_REPEAT);
		}
	}
	CRACEN_TEST_MARK(8);

	cryptorCfg.ReqCaps = CRYPTO_CAP_AES128_ECB |
						 CRYPTO_CAP_AES_CMAC |
						 CRYPTO_CAP_AES_GCM;
	if (!CryptorInit(&cryptor, &cryptorCfg, &hwDev))
	{
		return CracenTestFail(CRACEN_TEST_ERR_CRYPTOR_INIT);
	}
	pUse = CryptorHandle(&cryptor);
	if (pUse == NULL ||
		CryptoAes128Ecb(pUse, key2, plain2, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher2, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_CRYPTOR_AES);
	}
	CRACEN_TEST_MARK(9);

	if (CryptoCmac(pUse, key2, NULL, 0, cmac, NULL) != CRYPTO_STATUS_OK ||
		memcmp(cmac, cmacEmpty, sizeof(cmac)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_CMAC);
	}
	CRACEN_TEST_MARK(10);

	if (CryptoGcmEncrypt(pUse, zeroKey, zeroIv, sizeof(zeroIv), NULL, 0,
					 zeroPlain, sizeof(zeroPlain), gcmCipher, gcmTag,
					 sizeof(gcmTag), NULL) != CRYPTO_STATUS_OK ||
		memcmp(gcmCipher, gcmCipherExpected, sizeof(gcmCipher)) != 0 ||
		memcmp(gcmTag, gcmTagExpected, sizeof(gcmTag)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_GCM_ENCRYPT);
	}
	CRACEN_TEST_MARK(11);

	if (CryptoGcmDecrypt(pUse, zeroKey, zeroIv, sizeof(zeroIv), NULL, 0,
					 gcmCipher, sizeof(gcmCipher), gcmPlain, gcmTag,
					 sizeof(gcmTag), NULL) != CRYPTO_STATUS_OK ||
		memcmp(gcmPlain, zeroPlain, sizeof(gcmPlain)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_GCM_DECRYPT);
	}
	CRACEN_TEST_MARK(12);

	memcpy(badTag, gcmTag, sizeof(badTag));
	badTag[0] ^= 1;
	memset(gcmPlain, 0xa5, sizeof(gcmPlain));
	if (CryptoGcmDecrypt(pUse, zeroKey, zeroIv, sizeof(zeroIv), NULL, 0,
					 gcmCipher, sizeof(gcmCipher), gcmPlain, badTag,
					 sizeof(badTag), NULL) == CRYPTO_STATUS_OK ||
		!AllZero(gcmPlain, sizeof(gcmPlain)))
	{
		return CracenTestFail(CRACEN_TEST_ERR_GCM_TAG);
	}
	CRACEN_TEST_MARK(13);

	unsupportedCfg.DevNo = 0;
	unsupportedCfg.Provider = CRYPTO_PROVIDER_HW;
	unsupportedCfg.ReqCaps = CRYPTO_CAP_ECDH_P256;
	if (CryptoHwInit(&hwDev, &unsupportedCfg))
	{
		return CracenTestFail(CRACEN_TEST_ERR_UNSUPPORTED_ECDH);
	}
	CRACEN_TEST_MARK(14);

	// Reinitialize the AES handle after the deliberate unsupported Init attempt.
	if (!CryptoHwInit(&hwDev, &hwCfg))
	{
		return CracenTestFail(CRACEN_TEST_ERR_HW_INIT);
	}

	if (!RngInit())
	{
		return CracenTestFail(CRACEN_TEST_ERR_RNG_INIT);
	}
	if (!RngGet(rng1, sizeof(rng1)) || !RngGet(rng2, sizeof(rng2)))
	{
		return CracenTestFail(CRACEN_TEST_ERR_RNG_GET);
	}
	if (AllZero(rng1, sizeof(rng1)) || AllZero(rng2, sizeof(rng2)) ||
		memcmp(rng1, rng2, sizeof(rng1)) == 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_RNG_OUTPUT);
	}
	memcpy((void *)g_CracenTestLastRng, rng2, sizeof(rng2));
	CRACEN_TEST_MARK(15);

	// Confirm the shared CRACEN lock is released by the RNG path and AES remains
	// operational after two DRBG requests.
	if (CryptoAes128Ecb(&hwDev, key1, plain1, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher1, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_AES_AFTER_RNG);
	}
	CRACEN_TEST_MARK(16);

	CryptoSecureWipe(out, sizeof(out));
	CryptoSecureWipe(inPlace, sizeof(inPlace));
	CryptoSecureWipe(cmac, sizeof(cmac));
	CryptoSecureWipe(gcmCipher, sizeof(gcmCipher));
	CryptoSecureWipe(gcmPlain, sizeof(gcmPlain));
	CryptoSecureWipe(gcmTag, sizeof(gcmTag));
	CryptoSecureWipe(badTag, sizeof(badTag));
	CryptoSecureWipe(rng1, sizeof(rng1));
	CryptoSecureWipe(rng2, sizeof(rng2));

	g_CracenTestResult = CRACEN_TEST_PASS;
	return true;
}

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	(void)CracenAesTest();

	while (true)
	{
		__WFE();
	}
}
