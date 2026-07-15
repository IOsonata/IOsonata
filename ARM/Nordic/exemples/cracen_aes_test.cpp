/**-------------------------------------------------------------------------
@example	cracen_aes_test.cpp

@brief	nRF54 CRACEN AES, P-256 ECDH and RNG acceptance test.

		Hardware acceptance test for crypto_cracen.cpp. The test runs without
		Bluetooth or the SoftDevice so the CRACEN driver is exercised directly.

		The test checks the provider self-test, AES-128 ECB, Cryptor forwarding,
		CMAC and GCM derived over hardware AES, CRACEN/uECC P-256 cross-
		derivation, invalid peer-point rejection, single-use key consumption,
		and the CRACEN-backed RNG path.

		Usage: build this file as a standalone nRF54 application linked with the
		nRF54 IOsonata library containing crypto_cracen.cpp. Inspect
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

enum {
	CRACEN_TEST_PASS                 = 1,
	CRACEN_TEST_ERR_HW_INIT          = -1,
	CRACEN_TEST_ERR_NAME             = -2,
	CRACEN_TEST_ERR_PROPERTY         = -3,
	CRACEN_TEST_ERR_CAPABILITY       = -4,
	CRACEN_TEST_ERR_SELFTEST         = -5,
	CRACEN_TEST_ERR_AES_KAT          = -6,
	CRACEN_TEST_ERR_AES_REPEAT       = -7,
	CRACEN_TEST_ERR_CRYPTOR_INIT     = -8,
	CRACEN_TEST_ERR_CRYPTOR_AES      = -9,
	CRACEN_TEST_ERR_CMAC             = -10,
	CRACEN_TEST_ERR_GCM_ENCRYPT      = -11,
	CRACEN_TEST_ERR_GCM_DECRYPT      = -12,
	CRACEN_TEST_ERR_GCM_TAG          = -13,
	CRACEN_TEST_ERR_UECC_INIT        = -14,
	CRACEN_TEST_ERR_CRACEN_KEYGEN    = -15,
	CRACEN_TEST_ERR_UECC_KEYGEN      = -16,
	CRACEN_TEST_ERR_CRACEN_DH        = -17,
	CRACEN_TEST_ERR_UECC_DH          = -18,
	CRACEN_TEST_ERR_DH_MISMATCH      = -19,
	CRACEN_TEST_ERR_KEY_REUSE        = -20,
	CRACEN_TEST_ERR_INVALID_POINT    = -21,
	CRACEN_TEST_ERR_INVALID_CONSUME  = -22,
	CRACEN_TEST_ERR_RNG_INIT         = -23,
	CRACEN_TEST_ERR_RNG_GET          = -24,
	CRACEN_TEST_ERR_RNG_OUTPUT       = -25,
	CRACEN_TEST_ERR_AES_AFTER_RNG    = -26,
};

volatile int g_CracenTestResult;
volatile uint32_t g_CracenTestPassMask;
volatile uint8_t g_CracenTestLastAes[16];
volatile uint8_t g_CracenTestLastDh[32];
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
	static const uint8_t key[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
	};
	static const uint8_t plain[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff
	};
	static const uint8_t cipher[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a
	};
	// AES-CMAC over an empty message with key 000102...0f.
	static const uint8_t cmacEmpty[16] = {
		0x97,0xdd,0x6e,0x5a,0x88,0x2c,0xbd,0x56,
		0x4c,0x39,0xae,0x7d,0x1c,0x5a,0x31,0xaa
	};
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

	CryptoDev_t hwDev = {};
	CryptoDev_t ueccDev = {};
	Cryptor_t cryptor = {};
	CryptoCfg_t hwCfg = {};
	CryptoCfg_t ueccCfg = {};
	CryptoCfg_t cryptorCfg = {};
	alignas(uint32_t) static uint8_t hwArena[CRYPTO_MEMSIZE_HW];
	alignas(uint32_t) static uint8_t ueccArena[CRYPTO_MEMSIZE_UECC];
	uint8_t out[16] = {};
	uint8_t cmac[16] = {};
	uint8_t gcmCipher[16] = {};
	uint8_t gcmPlain[16] = {};
	uint8_t gcmTag[16] = {};
	uint8_t badTag[16] = {};
	uint8_t cracenPub[64] = {};
	uint8_t ueccPub[64] = {};
	uint8_t dhCracen[32] = {};
	uint8_t dhUecc[32] = {};
	uint8_t invalidPoint[64] = {};
	uint8_t rng1[32] = {};
	uint8_t rng2[32] = {};

	g_CracenTestResult = 0;
	g_CracenTestPassMask = 0;
	memset((void *)g_CracenTestLastAes, 0, sizeof(g_CracenTestLastAes));
	memset((void *)g_CracenTestLastDh, 0, sizeof(g_CracenTestLastDh));
	memset((void *)g_CracenTestLastRng, 0, sizeof(g_CracenTestLastRng));

	hwCfg.DevNo = 0;
	hwCfg.Provider = CRYPTO_PROVIDER_HW;
	hwCfg.ReqCaps = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256;
	hwCfg.Flags = CRYPTO_FLAG_SYNC | CRYPTO_FLAG_SELFTEST |
				  CRYPTO_FLAG_NO_FALLBACK;
	hwCfg.pMem = hwArena;
	hwCfg.MemSize = sizeof(hwArena);
	if (!CryptoHwInit(&hwDev, &hwCfg))
	{
		return CracenTestFail(CRACEN_TEST_ERR_HW_INIT);
	}
	CRACEN_TEST_MARK(0);

	if (strcmp(CryptoName(&hwDev), "cracen-hw") != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_NAME);
	}
	if (!CryptoHasProp(&hwDev, CRYPTO_PROP_PLAIN_KEYCTX |
								 CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC))
	{
		return CracenTestFail(CRACEN_TEST_ERR_PROPERTY);
	}
	if (!CryptoIsCapable(&hwDev, CRYPTO_CAP_AES128_ECB |
								  CRYPTO_CAP_ECDH_P256 |
								  CRYPTO_CAP_AES_CMAC |
								  CRYPTO_CAP_AES_CCM |
								  CRYPTO_CAP_AES_GCM))
	{
		return CracenTestFail(CRACEN_TEST_ERR_CAPABILITY);
	}
	if (CryptoSelfTest(&hwDev) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_SELFTEST);
	}
	CRACEN_TEST_MARK(1);

	if (CryptoAes128Ecb(&hwDev, key, plain, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_AES_KAT);
	}
	memcpy((void *)g_CracenTestLastAes, out, sizeof(out));
	for (unsigned i = 0; i < 128; i++)
	{
		if (CryptoAes128Ecb(&hwDev, key, plain, out, NULL) != CRYPTO_STATUS_OK ||
			memcmp(out, cipher, sizeof(out)) != 0)
		{
			return CracenTestFail(CRACEN_TEST_ERR_AES_REPEAT);
		}
	}
	CRACEN_TEST_MARK(2);

	cryptorCfg.ReqCaps = CRYPTO_CAP_AES128_ECB |
						 CRYPTO_CAP_AES_CMAC | CRYPTO_CAP_AES_GCM;
	if (!CryptorInit(&cryptor, &cryptorCfg, &hwDev))
	{
		return CracenTestFail(CRACEN_TEST_ERR_CRYPTOR_INIT);
	}
	CryptoDev_t *pUse = CryptorHandle(&cryptor);
	if (pUse == NULL ||
		CryptoAes128Ecb(pUse, key, plain, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_CRYPTOR_AES);
	}
	if (CryptoCmac(pUse, key, NULL, 0, cmac, NULL) != CRYPTO_STATUS_OK ||
		memcmp(cmac, cmacEmpty, sizeof(cmac)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_CMAC);
	}
	if (CryptoGcmEncrypt(pUse, zeroKey, zeroIv, sizeof(zeroIv), NULL, 0,
					 zeroPlain, sizeof(zeroPlain), gcmCipher, gcmTag,
					 sizeof(gcmTag), NULL) != CRYPTO_STATUS_OK ||
		memcmp(gcmCipher, gcmCipherExpected, sizeof(gcmCipher)) != 0 ||
		memcmp(gcmTag, gcmTagExpected, sizeof(gcmTag)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_GCM_ENCRYPT);
	}
	if (CryptoGcmDecrypt(pUse, zeroKey, zeroIv, sizeof(zeroIv), NULL, 0,
					 gcmCipher, sizeof(gcmCipher), gcmPlain, gcmTag,
					 sizeof(gcmTag), NULL) != CRYPTO_STATUS_OK ||
		memcmp(gcmPlain, zeroPlain, sizeof(gcmPlain)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_GCM_DECRYPT);
	}
	memcpy(badTag, gcmTag, sizeof(badTag));
	badTag[0] ^= 1U;
	memset(gcmPlain, 0xA5, sizeof(gcmPlain));
	if (CryptoGcmDecrypt(pUse, zeroKey, zeroIv, sizeof(zeroIv), NULL, 0,
					 gcmCipher, sizeof(gcmCipher), gcmPlain, badTag,
					 sizeof(badTag), NULL) == CRYPTO_STATUS_OK ||
		!AllZero(gcmPlain, sizeof(gcmPlain)))
	{
		return CracenTestFail(CRACEN_TEST_ERR_GCM_TAG);
	}
	CRACEN_TEST_MARK(3);

	ueccCfg.Provider = CRYPTO_PROVIDER_UECC;
	ueccCfg.ReqCaps = CRYPTO_CAP_ECDH_P256;
	ueccCfg.pMem = ueccArena;
	ueccCfg.MemSize = sizeof(ueccArena);
	if (!CryptoUeccInit(&ueccDev, &ueccCfg))
	{
		return CracenTestFail(CRACEN_TEST_ERR_UECC_INIT);
	}
	if (CryptoEcdhP256KeyGen(&hwDev, NULL, cracenPub, NULL) != CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_CRACEN_KEYGEN);
	}
	if (CryptoEcdhP256KeyGen(&ueccDev, NULL, ueccPub, NULL) != CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_UECC_KEYGEN);
	}
	if (CryptoEcdhP256(&hwDev, NULL, ueccPub, dhCracen, NULL) != CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_CRACEN_DH);
	}
	if (CryptoEcdhP256(&ueccDev, NULL, cracenPub, dhUecc, NULL) != CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_UECC_DH);
	}
	if (memcmp(dhCracen, dhUecc, sizeof(dhCracen)) != 0 ||
		AllZero(dhCracen, sizeof(dhCracen)))
	{
		return CracenTestFail(CRACEN_TEST_ERR_DH_MISMATCH);
	}
	memcpy((void *)g_CracenTestLastDh, dhCracen, sizeof(dhCracen));
	if (CryptoEcdhP256(&hwDev, NULL, ueccPub, dhCracen, NULL) == CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_KEY_REUSE);
	}
	CRACEN_TEST_MARK(4);

	if (CryptoEcdhP256KeyGen(&hwDev, NULL, cracenPub, NULL) != CRYPTO_STATUS_OK ||
		CryptoEcdhP256(&hwDev, NULL, invalidPoint, dhCracen, NULL) == CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_INVALID_POINT);
	}
	if (CryptoEcdhP256(&hwDev, NULL, ueccPub, dhCracen, NULL) == CRYPTO_STATUS_OK)
	{
		return CracenTestFail(CRACEN_TEST_ERR_INVALID_CONSUME);
	}
	CRACEN_TEST_MARK(5);

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
	if (CryptoAes128Ecb(&hwDev, key, plain, out, NULL) != CRYPTO_STATUS_OK ||
		memcmp(out, cipher, sizeof(out)) != 0)
	{
		return CracenTestFail(CRACEN_TEST_ERR_AES_AFTER_RNG);
	}
	CRACEN_TEST_MARK(6);

	CryptoSecureWipe(out, sizeof(out));
	CryptoSecureWipe(cmac, sizeof(cmac));
	CryptoSecureWipe(gcmCipher, sizeof(gcmCipher));
	CryptoSecureWipe(gcmPlain, sizeof(gcmPlain));
	CryptoSecureWipe(gcmTag, sizeof(gcmTag));
	CryptoSecureWipe(badTag, sizeof(badTag));
	CryptoSecureWipe(cracenPub, sizeof(cracenPub));
	CryptoSecureWipe(ueccPub, sizeof(ueccPub));
	CryptoSecureWipe(dhCracen, sizeof(dhCracen));
	CryptoSecureWipe(dhUecc, sizeof(dhUecc));
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
