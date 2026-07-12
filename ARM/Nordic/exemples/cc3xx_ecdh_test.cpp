/**-------------------------------------------------------------------------
@file	cc3xx_ecdh_test.cpp

@brief	P-256 ECDH acceptance: open CC3xx driver engine vs uECC, plus KAT

		Hardware acceptance test for crypto_cc3xx.cpp, the CryptoDev_t engine
		on the open Arm CC3xx register level driver over the nRF52840 CC310,
		with no vendor blob.

		Two independent checks:
		1. The engine self test (BLE Core spec P-256 known-answer vector),
		   run through CryptoInit with CRYPTO_FLAG_SELFTEST. This validates
		   the CC310 PKA path against a fixed spec vector, so a byte-order or
		   PKA setup fault fails deterministically, independent of any second
		   engine.
		2. A cross-derivation against the uECC software engine in both
		   directions: each engine generates a key pair and derives the
		   shared secret from the other's public key; both must agree. This
		   catches point-format or interface mismatches that a single-engine
		   KAT cannot.

		If the TRNG entropy startup health test fails on a device, CryptoInit
		returns false at the genkey step (the private key cannot be drawn),
		which points at the ring oscillator or subsampling parameters in
		cc3xx_config.h rather than at the EC math.

		Usage: call Cc3xxEcdhTest() from an application on the nRF52840 SDC
		configuration after system init. Returns true on pass. Test
		scaffolding, not part of the library build.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "crypto/crypto.h"

bool Cc3xxEcdhTest(void)
{
	CryptoDev_t ccDev;
	CryptoDev_t ueccDev;
	// Arenas hold engine word-typed key state; CryptoCfg_t pMem requires
	// uint32_t alignment.
	alignas(uint32_t) uint8_t ccArena[CRYPTO_MEMSIZE_HW];
	alignas(uint32_t) uint8_t ueccArena[CRYPTO_MEMSIZE_UECC];

	memset(&ccDev, 0, sizeof(ccDev));
	memset(&ueccDev, 0, sizeof(ueccDev));

	// CC3xx engine with the KAT self test requested at init. A KAT failure
	// makes CryptoInit return false here.
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

	// Direction 1: each engine generates, then each derives from the other.
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

	// Direction 2: fresh keys, roles repeated, to catch state left behind by
	// the single-use key handling.
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

	// Guard against a degenerate all-zero secret even when both agree.
	uint8_t zero[32];
	memset(zero, 0, sizeof(zero));
	if (memcmp(dhCc, zero, sizeof(zero)) == 0)
	{
		return false;
	}

	return true;
}

int main(int argc, char **argv)
{
	bool res = Cc3xxEcdhTest();

	if (res == false)
	{
		printf("failed\r\n");
	}
	else
	{
		printf("Passed\r\n");
	}
}

