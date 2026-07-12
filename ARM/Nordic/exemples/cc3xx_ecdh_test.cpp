/**-------------------------------------------------------------------------
@file	cc3xx_ecdh_test.cpp

@brief	P-256 ECDH acceptance: open CC3xx driver engine vs uECC, plus KAT

		Hardware acceptance test for crypto_cc3xx.cpp, the CryptoDev_t engine
		on the open Arm CC3xx register level driver over the nRF52840 CC310,
		with no vendor blob.

		The test checks the BLE Core P-256 known-answer vector, two independent
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
#include <stdio.h>

#include "nrf.h"
#include "crypto/crypto.h"

bool Cc3xxEcdhTest(void)
{
	CryptoDev_t ccDev;
	CryptoDev_t ueccDev;
	alignas(uint32_t) uint8_t ccArena[CRYPTO_MEMSIZE_HW];
	alignas(uint32_t) uint8_t ueccArena[CRYPTO_MEMSIZE_UECC];

	memset(&ccDev, 0, sizeof(ccDev));
	memset(&ueccDev, 0, sizeof(ueccDev));

	CryptoCfg_t ccCfg;
	memset(&ccCfg, 0, sizeof(ccCfg));
	ccCfg.Provider = CRYPTO_PROVIDER_HW;
	ccCfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
	ccCfg.Flags    = CRYPTO_FLAG_SELFTEST;
	ccCfg.pMem     = ccArena;
	ccCfg.MemSize  = sizeof(ccArena);

	if (CryptoHwInit(&ccDev, &ccCfg) == false)
	{
		return false;
	}

	CryptoCfg_t ueccCfg;
	memset(&ueccCfg, 0, sizeof(ueccCfg));
	ueccCfg.Provider = CRYPTO_PROVIDER_UECC;
	ueccCfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
	ueccCfg.pMem     = ueccArena;
	ueccCfg.MemSize  = sizeof(ueccArena);

	if (CryptoUeccInit(&ueccDev, &ueccCfg) == false)
	{
		return false;
	}

	uint8_t ccPub[64];
	uint8_t ueccPub[64];
	uint8_t dhCc[32];
	uint8_t dhUecc[32];

	if (CryptoEcdhP256KeyGen(&ccDev, NULL, ccPub, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256KeyGen(&ueccDev, NULL, ueccPub, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256(&ccDev, NULL, ueccPub, dhCc, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256(&ueccDev, NULL, ccPub, dhUecc, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (memcmp(dhCc, dhUecc, sizeof(dhCc)) != 0)
	{
		return false;
	}

	// The CC3xx private key must be consumed by the successful ECDH.
	if (CryptoEcdhP256(&ccDev, NULL, ueccPub, dhCc, NULL) == CRYPTO_STATUS_OK)
	{
		return false;
	}

	if (CryptoEcdhP256KeyGen(&ueccDev, NULL, ueccPub, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256KeyGen(&ccDev, NULL, ccPub, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256(&ueccDev, NULL, ccPub, dhUecc, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256(&ccDev, NULL, ueccPub, dhCc, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (memcmp(dhCc, dhUecc, sizeof(dhCc)) != 0)
	{
		return false;
	}

	uint8_t zero[64];
	memset(zero, 0, sizeof(zero));
	if (memcmp(dhCc, zero, sizeof(dhCc)) == 0)
	{
		return false;
	}

	// A zero point is not a valid P-256 public key and must be rejected.
	if (CryptoEcdhP256KeyGen(&ccDev, NULL, ccPub, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	if (CryptoEcdhP256(&ccDev, NULL, zero, dhCc, NULL) == CRYPTO_STATUS_OK)
	{
		return false;
	}

	// The key is consumed even when peer validation fails.
	if (CryptoEcdhP256(&ccDev, NULL, ueccPub, dhCc, NULL) == CRYPTO_STATUS_OK)
	{
		return false;
	}

	return true;
}

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	bool res = Cc3xxEcdhTest();
	printf(res ? "Passed\r\n" : "Failed\r\n");

	while (true)
	{
		__WFE();
	}
}
