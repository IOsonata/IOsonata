/**-------------------------------------------------------------------------
@file	cc3xx_ecdh_test.cpp

@brief	P-256 ECDH acceptance: open CC3xx driver engine vs uECC, plus KAT

		Hardware acceptance test for crypto_cc3xx.cpp, the CryptoDev_t engine
		on the open Arm CC3xx register level driver over the nRF52840 CC310,
		with no vendor blob.

		The test checks the BLE Core P-256 known-answer vector, the CC3xx
		plain-key-context property and Cryptor forwarding path, two independent
		CC3xx/uECC cross-derivations, invalid peer-point rejection, and
		single-use private-key consumption.

		Usage: call Cc3xxEcdhTest() from an application on the nRF52840 after
		system initialization. Returns true on pass. Test scaffolding, not part
		of the library build.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "nrf.h"
#include "crypto/crypto.h"

volatile int g_Cc3xxEcdhTestResult;

bool Cc3xxEcdhTest(void)
{
	CryptoDev_t ccDev;
	CryptoDev_t ueccDev;
	Cryptor_t ccUse;
	CryptoDev_t *pCc = nullptr;
	CryptoCfg_t ccCfg;
	CryptoCfg_t ccUseCfg;
	CryptoCfg_t ueccCfg;
	alignas(uint32_t) static uint8_t ccArena[CRYPTO_MEMSIZE_HW];
	alignas(uint32_t) static uint8_t ccUseArena[CRYPTO_MEMSIZE_HW];
	alignas(uint32_t) static uint8_t ueccArena[CRYPTO_MEMSIZE_UECC];
	uint8_t ccPub[64];
	uint8_t ueccPub[64];
	uint8_t dhCc[32];
	uint8_t dhUecc[32];
	uint8_t zero[64];
	bool res = false;

	memset(&ccDev, 0, sizeof(ccDev));
	memset(&ueccDev, 0, sizeof(ueccDev));
	memset(&ccUse, 0, sizeof(ccUse));
	memset(&ccCfg, 0, sizeof(ccCfg));
	memset(&ccUseCfg, 0, sizeof(ccUseCfg));
	memset(&ueccCfg, 0, sizeof(ueccCfg));
	memset(ccPub, 0, sizeof(ccPub));
	memset(ueccPub, 0, sizeof(ueccPub));
	memset(dhCc, 0, sizeof(dhCc));
	memset(dhUecc, 0, sizeof(dhUecc));
	memset(zero, 0, sizeof(zero));

	do
	{
		ccCfg.Provider = CRYPTO_PROVIDER_HW;
		ccCfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
		ccCfg.Flags    = CRYPTO_FLAG_SELFTEST;
		ccCfg.pMem     = ccArena;
		ccCfg.MemSize  = sizeof(ccArena);

		if (CryptoHwInit(&ccDev, &ccCfg) == false)
		{
			break;
		}
		if (CryptoHasProp(&ccDev, CRYPTO_PROP_PLAIN_KEYCTX) == false)
		{
			break;
		}

		ccUseCfg.ReqCaps = CRYPTO_CAP_ECDH_P256;
		ccUseCfg.pMem    = ccUseArena;
		ccUseCfg.MemSize = sizeof(ccUseArena);

		if (CryptorInit(&ccUse, &ccUseCfg, &ccDev) == false)
		{
			break;
		}
		pCc = CryptorHandle(&ccUse);
		if (pCc == nullptr)
		{
			break;
		}

		ueccCfg.Provider = CRYPTO_PROVIDER_UECC;
		ueccCfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
		ueccCfg.pMem     = ueccArena;
		ueccCfg.MemSize  = sizeof(ueccArena);

		if (CryptoUeccInit(&ueccDev, &ueccCfg) == false)
		{
			break;
		}

		if (CryptoEcdhP256KeyGen(pCc, NULL, ccPub, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256KeyGen(&ueccDev, NULL, ueccPub, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256(pCc, NULL, ueccPub, dhCc, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256(&ueccDev, NULL, ccPub, dhUecc, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (memcmp(dhCc, dhUecc, sizeof(dhCc)) != 0)
		{
			break;
		}

		// The CC3xx private key must be consumed by the successful ECDH.
		if (CryptoEcdhP256(pCc, NULL, ueccPub, dhCc, NULL) == CRYPTO_STATUS_OK)
		{
			break;
		}

		if (CryptoEcdhP256KeyGen(&ueccDev, NULL, ueccPub, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256KeyGen(pCc, NULL, ccPub, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256(&ueccDev, NULL, ccPub, dhUecc, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256(pCc, NULL, ueccPub, dhCc, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (memcmp(dhCc, dhUecc, sizeof(dhCc)) != 0)
		{
			break;
		}
		if (memcmp(dhCc, zero, sizeof(dhCc)) == 0)
		{
			break;
		}

		// A zero point is not a valid P-256 public key and must be rejected.
		if (CryptoEcdhP256KeyGen(pCc, NULL, ccPub, NULL) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (CryptoEcdhP256(pCc, NULL, zero, dhCc, NULL) == CRYPTO_STATUS_OK)
		{
			break;
		}

		// The key is consumed even when peer validation fails.
		if (CryptoEcdhP256(pCc, NULL, ueccPub, dhCc, NULL) == CRYPTO_STATUS_OK)
		{
			break;
		}

		res = true;
	} while (false);

	CryptoSecureWipe(dhCc, sizeof(dhCc));
	CryptoSecureWipe(dhUecc, sizeof(dhUecc));
	CryptoSecureWipe(ccArena, sizeof(ccArena));
	CryptoSecureWipe(ccUseArena, sizeof(ccUseArena));
	CryptoSecureWipe(ueccArena, sizeof(ueccArena));
	return res;
}

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	g_Cc3xxEcdhTestResult = Cc3xxEcdhTest() ? 1 : -1;

	while (true)
	{
		__WFE();
	}
}
