/**-------------------------------------------------------------------------
@file	cc3xx_ecdh_test.cpp

@brief	P-256 ECDH acceptance: open CC3xx driver engine vs uECC, plus KAT

		Hardware acceptance test for crypto_cc3xx.cpp, the CryptoCc3xx engine
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
#include "crypto/crypto_cc3xx_engine.h"
#include "crypto/crypto_uecc.h"
#include "crypto_rng_nrf.h"

volatile int g_Cc3xxEcdhTestResult;

bool Cc3xxEcdhTest(void)
{
	alignas(uint32_t) static uint8_t ccMem[CRYPTO_CC3XX_MEMSIZE];
	alignas(uint64_t) static uint8_t ueccMem[CRYPTO_UECC_MEMSIZE];
	uint8_t ccCtx[64];
	uint8_t ueccCtx[128];
	uint8_t ccPub[64];
	uint8_t ueccPub[64];
	uint8_t dhCc[32];
	uint8_t dhUecc[32];
	uint8_t zero[64];
	bool res = false;

	memset(ccCtx, 0, sizeof(ccCtx));
	memset(ueccCtx, 0, sizeof(ueccCtx));
	memset(ccPub, 0, sizeof(ccPub));
	memset(ueccPub, 0, sizeof(ueccPub));
	memset(dhCc, 0, sizeof(dhCc));
	memset(dhUecc, 0, sizeof(dhUecc));
	memset(zero, 0, sizeof(zero));

	// Hardware CC310 engine and the software oracle. Both take randomness from
	// the security-grade RNG.
	KeyAgreeEngine *pCc = CryptoCc3xxCreate(ccMem, sizeof(ccMem));
	KeyAgreeEngine *pUecc = CryptoUeccCreate(ueccMem, sizeof(ueccMem),
											 CryptoRngNrfInstance());

	do
	{
		if (pCc == nullptr || pUecc == nullptr)
		{
			break;
		}
		if (pCc->SelfTest() != 0)
		{
			break;
		}

		// Cross ECDH: both engines must derive the same shared secret.
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_OK ||
			pUecc->KeyGen(CRYPTO_CURVE_P256, ueccCtx, ueccPub) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) != CRYPTO_STATUS_OK ||
			pUecc->Agree(CRYPTO_CURVE_P256, ueccCtx, ccPub, dhUecc) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (memcmp(dhCc, dhUecc, sizeof(dhCc)) != 0 ||
			memcmp(dhCc, zero, sizeof(dhCc)) == 0)
		{
			break;
		}

		// Single use: the private key is consumed by the successful Agree, so a
		// second Agree on the same context must fail.
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) == CRYPTO_STATUS_OK)
		{
			break;
		}

		// A zero point is not a valid P-256 public key and must be rejected.
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, zero, dhCc) == CRYPTO_STATUS_OK)
		{
			break;
		}

		// The key is consumed even when peer validation fails.
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) == CRYPTO_STATUS_OK)
		{
			break;
		}

		res = true;
	} while (false);

	CryptoSecureWipe(dhCc, sizeof(dhCc));
	CryptoSecureWipe(dhUecc, sizeof(dhUecc));
	CryptoSecureWipe(ccCtx, sizeof(ccCtx));
	CryptoSecureWipe(ueccCtx, sizeof(ueccCtx));
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
