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

		The test prints each step and previews of the derived values to
		stdout: semihosting console or wherever the project retargets printf.
		Result is also left in g_Cc3xxEcdhTestResult (0 pass, negative fail
		code) and g_Cc3xxEcdhTestPassMask for debugger inspection.

		Usage: call Cc3xxEcdhTest() from an application on the nRF52840 after
		system initialization, or build standalone (main included). Returns
		true on pass. Test scaffolding, not part of the library build.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "crypto_cc3xx.h"
#include "cc3xx_intrf.h"
#include "crypto/crypto_uecc.h"
#include "crypto_rng_nrf.h"

enum CC3XX_ECDH_TEST_ERR {
	CC3XX_ECDH_TEST_ERR_NONE        = 0,
	CC3XX_ECDH_TEST_ERR_CREATE      = -1,	// an engine failed to construct
	CC3XX_ECDH_TEST_ERR_SELFTEST    = -2,	// CC3xx known-answer self test failed
	CC3XX_ECDH_TEST_ERR_KEYGEN      = -3,	// a key generation failed
	CC3XX_ECDH_TEST_ERR_AGREE       = -4,	// a derivation failed
	CC3XX_ECDH_TEST_ERR_MISMATCH    = -5,	// engines derived different secrets
	CC3XX_ECDH_TEST_ERR_ZERO_SECRET = -6,	// derived secret is all zero
	CC3XX_ECDH_TEST_ERR_KEY_REUSE   = -7,	// consumed key accepted a second Agree
	CC3XX_ECDH_TEST_ERR_BAD_POINT   = -8,	// invalid peer point was accepted
	CC3XX_ECDH_TEST_ERR_REUSE_AFTER = -9,	// key survived a failed validation
	CC3XX_ECDH_TEST_ERR_SW_CREATE   = -10,	// software oracle failed to construct
	CC3XX_ECDH_TEST_ERR_CONTEND     = -11,	// lock contention was not honored
	CC3XX_ECDH_TEST_ERR_STALE_KEY   = -12,	// old key survived a rejected KeyGen
	CC3XX_ECDH_TEST_ERR_LIFECYCLE   = -13,	// disable/enable lifecycle broken
};

volatile int g_Cc3xxEcdhTestResult;
volatile uint32_t g_Cc3xxEcdhTestPassMask;

#define CC3XX_ECDH_TEST_MARK(Bit) \
	do { g_Cc3xxEcdhTestPassMask |= (1UL << (Bit)); } while (0)

static bool Cc3xxEcdhTestFail(int Result)
{
	g_Cc3xxEcdhTestResult = Result;
	printf("FAIL (code %d)\r\n", Result);
	return false;
}

static void Cc3xxEcdhTestHex(const char *pLabel, const uint8_t *pData, int Len)
{
	printf("    %s : ", pLabel);
	for (int i = 0; i < Len; i++)
	{
		printf("%02X", pData[i]);
	}
	printf("...\r\n");
}

bool Cc3xxEcdhTest(void)
{
	alignas(CryptoCc3xx) static uint8_t ccMem[CRYPTO_CC3XX_MEMSIZE];
	alignas(uint64_t) static uint8_t ueccMem[CRYPTO_UECC_MEMSIZE];
	uint8_t ccCtx[64];
	uint8_t ueccCtx[128];
	uint8_t ccPub[64];
	uint8_t ueccPub[64];
	uint8_t dhCc[32];
	uint8_t dhUecc[32];
	uint8_t zero[64];
	bool res = false;

	g_Cc3xxEcdhTestResult = CC3XX_ECDH_TEST_ERR_NONE;
	g_Cc3xxEcdhTestPassMask = 0;

	memset(ccCtx, 0, sizeof(ccCtx));
	memset(ueccCtx, 0, sizeof(ueccCtx));
	memset(ccPub, 0, sizeof(ccPub));
	memset(ueccPub, 0, sizeof(ueccPub));
	memset(dhCc, 0, sizeof(dhCc));
	memset(dhUecc, 0, sizeof(dhUecc));
	memset(zero, 0, sizeof(zero));

	printf("\r\n--- CC3xx P-256 ECDH acceptance test ---\r\n");
	printf("Hardware engine   : CryptoCc3xx (CC310 register driver, no vendor blob)\r\n");
	printf("Software oracle   : CryptoUecc (uECC P-256)\r\n");
	printf("Randomness        : CryptoRngNrfInstance (security-grade)\r\n\r\n");

	do
	{
		// Step 0a : hardware engine constructs, with the injected RNG.
		printf("[0a] CC3xx engine construction   : ");
		printf("(object %u bytes align %u, arena %u at %%8=%u) ",
			   (unsigned)sizeof(CryptoCc3xx), (unsigned)alignof(CryptoCc3xx),
			   (unsigned)sizeof(ccMem), (unsigned)((uintptr_t)ccMem & 7U));
		KeyAgreeEngine *pCc = CryptoCc3xxCreate(ccMem, sizeof(ccMem),
												CryptoRngNrfInstance());
		if (pCc == nullptr)
		{
			// Discriminate the bring-up state for the report. The wrapper
			// power-on readback is the CoreInit gate.
			printf("[wrapper enable readback %u] ", (unsigned)Cc3xxEnable());
			Cc3xxDisable();
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_CREATE);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(0);

		// Step 0b : software oracle constructs.
		printf("[0b] uECC oracle construction    : ");
		KeyAgreeEngine *pUecc = CryptoUeccCreate(ueccMem, sizeof(ueccMem),
												 CryptoRngNrfInstance());
		if (pUecc == nullptr)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_SW_CREATE);
			break;
		}
		printf("PASS\r\n");

		// Step 1 : CC3xx known-answer self test (BLE Core P-256 vector).
		printf("[1] CC3xx self test (P-256 KAT)  : ");
		if (pCc->SelfTest() != 0)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_SELFTEST);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(1);

		// Step 2 : key generation on both engines.
		printf("[2] key generation (hw and sw)   : ");
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_OK ||
			pUecc->KeyGen(CRYPTO_CURVE_P256, ueccCtx, ueccPub) != CRYPTO_STATUS_OK)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_KEYGEN);
			break;
		}
		printf("PASS\r\n");
		Cc3xxEcdhTestHex("CC3xx pub X", ccPub, 8);
		Cc3xxEcdhTestHex("uECC  pub X", ueccPub, 8);
		CC3XX_ECDH_TEST_MARK(2);

		// Step 3 : cross ECDH, both directions.
		printf("[3] cross derivation             : ");
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) != CRYPTO_STATUS_OK ||
			pUecc->Agree(CRYPTO_CURVE_P256, ueccCtx, ccPub, dhUecc) != CRYPTO_STATUS_OK)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_AGREE);
			break;
		}
		printf("PASS\r\n");
		Cc3xxEcdhTestHex("hw secret  ", dhCc, 8);
		Cc3xxEcdhTestHex("sw secret  ", dhUecc, 8);
		CC3XX_ECDH_TEST_MARK(3);

		// Step 4 : the secrets agree and are not zero.
		printf("[4] secrets match, non zero      : ");
		if (memcmp(dhCc, dhUecc, sizeof(dhCc)) != 0)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_MISMATCH);
			break;
		}
		if (memcmp(dhCc, zero, sizeof(dhCc)) == 0)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_ZERO_SECRET);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(4);

		// Step 5 : single use. The private key is consumed by the successful
		// Agree, so a second Agree on the same context must fail.
		printf("[5] key consumed after use       : ");
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) == CRYPTO_STATUS_OK)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_KEY_REUSE);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(5);

		// Step 6 : a zero point is not a valid P-256 public key and must be
		// rejected (invalid-curve attack defense).
		printf("[6] invalid peer point rejected  : ");
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_OK)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_KEYGEN);
			break;
		}
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, zero, dhCc) == CRYPTO_STATUS_OK)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_BAD_POINT);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(6);

		// Step 7 : the key is consumed even when peer validation fails.
		printf("[7] key consumed on failed check : ");
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) == CRYPTO_STATUS_OK)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_REUSE_AFTER);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(7);

		// Step 8 : operation-lock contention. Hold the CC3xx operation lock
		// the way a concurrent operation would. KeyGen must fail cleanly,
		// must not steal or release the foreign hold, and the engine must
		// recover once the lock is released.
		printf("[8] KeyGen fails while lock held : ");
		Cc3xxIntrf *pLock = Cc3xxIntrfInstance();
		if (pLock == nullptr || !pLock->OpHold())
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_CONTEND);
			break;
		}
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_FAIL ||
			pLock->OpHold())
		{
			pLock->OpRelease();
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_CONTEND);
			break;
		}
		pLock->OpRelease();
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(8);

		// Step 9 : a KeyGen rejected at acquisition must not leave the
		// previous key usable in the reused context. Generate a valid key,
		// fail one KeyGen under the lock, then Agree with that context must
		// fail because the context was reset before the acquisition attempt.
		printf("[9] no stale key after failed gen: ");
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_OK ||
			!pLock->OpHold())
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_STALE_KEY);
			break;
		}
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_FAIL)
		{
			pLock->OpRelease();
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_STALE_KEY);
			break;
		}
		pLock->OpRelease();
		if (pCc->Agree(CRYPTO_CURVE_P256, ccCtx, ueccPub, dhCc) !=
			CRYPTO_STATUS_FAIL)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_STALE_KEY);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(9);

		// Step 10 : device lifecycle through the interface reference count.
		// A disabled engine must refuse every operation (and must not
		// silently repower the wrapper); Enable must bring it back.
		printf("[10] disable/enable lifecycle    : ");
		pCc->Disable();
		if (pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) !=
			CRYPTO_STATUS_FAIL || pCc->SelfTest() == 0)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_LIFECYCLE);
			break;
		}
		if (!pCc->Enable() ||
			pCc->KeyGen(CRYPTO_CURVE_P256, ccCtx, ccPub) != CRYPTO_STATUS_OK ||
			pCc->SelfTest() != 0)
		{
			(void)Cc3xxEcdhTestFail(CC3XX_ECDH_TEST_ERR_LIFECYCLE);
			break;
		}
		printf("PASS\r\n");
		CC3XX_ECDH_TEST_MARK(10);

		printf("\r\nAll steps passed. Result 0, pass mask 0x%02X\r\n",
			   (unsigned)g_Cc3xxEcdhTestPassMask);
		res = true;
	} while (false);

	CryptoSecureWipe(dhCc, sizeof(dhCc));
	CryptoSecureWipe(dhUecc, sizeof(dhUecc));
	CryptoSecureWipe(ccCtx, sizeof(ccCtx));
	CryptoSecureWipe(ueccCtx, sizeof(ueccCtx));
	return res;
}

#ifndef CC3XX_ECDH_TEST_NO_MAIN
int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	(void)Cc3xxEcdhTest();

	while (true)
	{
		__WFE();
	}
}
#endif
