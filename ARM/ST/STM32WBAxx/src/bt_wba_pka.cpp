/**-------------------------------------------------------------------------
@file	bt_wba_pka.cpp

@brief	STM32WBA BLE stack P-256 key operations on the IOsonata crypto layer.

The Cube BLE stack asks for two P-256 operations through bleplat.c, which
forwards them to the PKACTRL_* functions of the Cube PKA controller: the public
key of a private key the link layer generated, and the Diffie-Hellman shared
secret of a private key and a peer public key. This file provides the same
PKACTRL_* functions on the IOsonata CryptoWbaPka engine (the PKA peripheral),
so the Cube pka_ctrl.c, hw_pka.c and hw_pka_p256.c are not built. Define
BT_WBA_SOFT_PKA to use the CryptoUecc software engine instead.

The operation runs from the sequencer task and reports the end through
PKACTRL_CB_Complete, as the PKA controller does. The engine is polled. The
random numbers the engines ask for come from the RNG of bt_wba_platform.cpp,
which keeps the RNG clock and start state in one place.

Known limits:
- The operation runs in the event queue context. A multiplication takes about
  31 ms on the PKA and about 190 ms on CryptoUecc at the DK1 clock.
- The hardware engine has run in the crypto engine test on the DK1 only. The
  Bluetooth use of it is not tested on hardware.

@author	Hoang Nguyen Hoan
@date	Oct. 16, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/icrypto.h"
#ifdef BT_WBA_SOFT_PKA
#include "crypto/crypto_uecc.h"
#else
#include "crypto/crypto_stm32wba.h"
#endif
#include "bt_wba_platform.h"

// Bit of the task in the sequencer mask. The link layer task uses the bit its
// Target file defines, which is below this one.
#define BT_WBA_SEQ_PKA			31U

// Return values of the PKACTRL_* functions, aligned with BLEPLAT_*
#define BT_WBA_PKA_OK			0
#define BT_WBA_PKA_BUSY			-2
#define BT_WBA_PKA_EOF			-3

#define BT_WBA_PKA_NONE			0U
#define BT_WBA_PKA_KEYGEN		1U
#define BT_WBA_PKA_DHKEY		2U

#define BT_WBA_PKA_WORDS		24U

extern "C" void UTIL_SEQ_RegTask(uint32_t TaskBm, uint32_t Flags, void (*Task)(void));
extern "C" void UTIL_SEQ_SetTask(uint32_t TaskBm, uint32_t Prio);
extern "C" void HW_RNG_Get(uint8_t n, uint32_t *pVal);
extern "C" void PKACTRL_CB_Complete(void);

// Random source of the blinding. It reads the RNG of the platform layer.
class BtWbaPkaRng : public RngEngine {
public:
	BtWbaPkaRng() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override
	{
		while (Len > 0U)
		{
			uint32_t w;
			size_t n = Len < sizeof(w) ? Len : sizeof(w);

			HW_RNG_Get(1, &w);
			memcpy(pOut, &w, n);
			pOut += n;
			Len -= n;
		}

		return CRYPTO_STATUS_OK;
	}

	bool IsSecure() const override { return true; }
};

#ifdef BT_WBA_SOFT_PKA
typedef CryptoUecc BtWbaPkaEng;
#else
typedef CryptoWbaPka BtWbaPkaEng;
#endif

static uint64_t s_EngMem[(sizeof(BtWbaPkaEng) + 7U) / 8U];
static uint64_t s_RngMem[(sizeof(BtWbaPkaRng) + 7U) / 8U];
static BtWbaPkaEng *s_pEng;

static volatile uint8_t s_PkaOp;
static volatile int s_PkaErr = 1;
static volatile bool s_PkaTaskReg;
static uint32_t s_PkaBuf[BT_WBA_PKA_WORDS];

// The engine is built on first use so no static constructor is needed. Null
// when the engine does not start; the next operation tries again.
static BtWbaPkaEng *BtWbaPkaEngine(void)
{
	if (s_pEng == nullptr)
	{
		BtWbaPkaRng *pRng = new (s_RngMem) BtWbaPkaRng();
#ifdef BT_WBA_SOFT_PKA
		CryptoUecc *pEng = new (s_EngMem) CryptoUecc();

		pEng->SetRng(pRng);
		pEng->Enable();
#else
		CryptoWbaPka *pEng = CryptoWbaPkaCreate(s_EngMem, sizeof(s_EngMem), pRng);
#endif
		s_pEng = pEng;
	}

	return s_pEng;
}

// The stack words are little endian numbers, the engine takes big endian
static void BtWbaPkaRev(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1U - i];
	}
}

static bool BtWbaPkaKeyGen(void)
{
	BtWbaPkaEng *pEng = BtWbaPkaEngine();
	BtWbaPkaEng::KeyCtx ctx;
	uint8_t priv[P256_BYTES];
	uint8_t pub[2U * P256_BYTES];
	bool bOk;

	if (pEng == nullptr)
	{
		return false;
	}
	BtWbaPkaRev(priv, (const uint8_t *)s_PkaBuf, P256_BYTES);
	bOk = pEng->KeyImport(CRYPTO_CURVE_P256, &ctx, priv, pub) ==
		CRYPTO_STATUS_OK;
	if (bOk)
	{
		BtWbaPkaRev((uint8_t *)s_PkaBuf, pub, P256_BYTES);
		BtWbaPkaRev((uint8_t *)s_PkaBuf + P256_BYTES, pub + P256_BYTES,
					P256_BYTES);
	}
	pEng->KeyReset(&ctx);
	CryptoSecureWipe(priv, sizeof(priv));

	return bOk;
}

static bool BtWbaPkaDhKey(void)
{
	BtWbaPkaEng *pEng = BtWbaPkaEngine();
	BtWbaPkaEng::KeyCtx ctx;
	uint8_t priv[P256_BYTES];
	uint8_t peer[2U * P256_BYTES];
	uint8_t secret[P256_BYTES];
	const uint8_t *pRaw = (const uint8_t *)s_PkaBuf;
	bool bOk;

	if (pEng == nullptr)
	{
		return false;
	}
	BtWbaPkaRev(priv, pRaw, P256_BYTES);
	BtWbaPkaRev(peer, pRaw + P256_BYTES, P256_BYTES);
	BtWbaPkaRev(peer + P256_BYTES, pRaw + 2U * P256_BYTES, P256_BYTES);

	// The engine refuses a peer key that is not on the curve
	bOk = pEng->KeyImport(CRYPTO_CURVE_P256, &ctx, priv, nullptr) ==
			CRYPTO_STATUS_OK &&
		pEng->Agree(CRYPTO_CURVE_P256, &ctx, peer, secret) ==
			CRYPTO_STATUS_OK;
	if (bOk)
	{
		BtWbaPkaRev((uint8_t *)s_PkaBuf, secret, P256_BYTES);
	}
	pEng->KeyReset(&ctx);
	CryptoSecureWipe(priv, sizeof(priv));
	CryptoSecureWipe(secret, sizeof(secret));

	return bOk;
}

// Sequencer task. The stack is told the end in every case, and the read
// functions tell it whether the result is valid.
static void BtWbaPkaTask(void)
{
	bool bOk = false;

	if (s_PkaOp == BT_WBA_PKA_KEYGEN)
	{
		bOk = BtWbaPkaKeyGen();
	}
	else if (s_PkaOp == BT_WBA_PKA_DHKEY)
	{
		bOk = BtWbaPkaDhKey();
	}
	else
	{
		return;
	}

	s_PkaErr = bOk ? 0 : 1;
	s_PkaOp = BT_WBA_PKA_NONE;
	PKACTRL_CB_Complete();
}

static void BtWbaPkaQueue(void)
{
	if (s_PkaTaskReg == false)
	{
		UTIL_SEQ_RegTask(1UL << BT_WBA_SEQ_PKA, 0, BtWbaPkaTask);
		s_PkaTaskReg = true;
	}
	UTIL_SEQ_SetTask(1UL << BT_WBA_SEQ_PKA, 0);
}

extern "C" void PKACTRL_Reset(void)
{
	s_PkaOp = BT_WBA_PKA_NONE;
	s_PkaErr = 1;
	CryptoSecureWipe(s_PkaBuf, sizeof(s_PkaBuf));
}

// local_private_key : 8 words, little endian
extern "C" int PKACTRL_StartP256Key(const uint32_t *pLocalPrivateKey)
{
	if (s_PkaOp != BT_WBA_PKA_NONE)
	{
		return BT_WBA_PKA_BUSY;
	}

	memcpy(s_PkaBuf, pLocalPrivateKey, P256_BYTES);
	s_PkaErr = 1;
	s_PkaOp = BT_WBA_PKA_KEYGEN;
	BtWbaPkaQueue();

	return BT_WBA_PKA_OK;
}

// local_public_key : X then Y, 16 words, little endian
extern "C" int PKACTRL_ReadP256Key(uint32_t *pLocalPublicKey)
{
	if (s_PkaErr != 0)
	{
		return BT_WBA_PKA_EOF;
	}

	memcpy(pLocalPublicKey, s_PkaBuf, 2U * P256_BYTES);

	return BT_WBA_PKA_OK;
}

// local_private_key : 8 words, remote_public_key : X then Y, 16 words
extern "C" int PKACTRL_StartDhKey(const uint32_t *pLocalPrivateKey,
								  const uint32_t *pRemotePublicKey)
{
	if (s_PkaOp != BT_WBA_PKA_NONE)
	{
		return BT_WBA_PKA_BUSY;
	}

	memcpy(s_PkaBuf, pLocalPrivateKey, P256_BYTES);
	memcpy((uint8_t *)s_PkaBuf + P256_BYTES, pRemotePublicKey,
		   2U * P256_BYTES);
	s_PkaErr = 1;
	s_PkaOp = BT_WBA_PKA_DHKEY;
	BtWbaPkaQueue();

	return BT_WBA_PKA_OK;
}

// dh_key : 8 words, little endian
extern "C" int PKACTRL_ReadDhKey(uint32_t *pDhKey)
{
	if (s_PkaErr != 0)
	{
		return BT_WBA_PKA_EOF;
	}

	memcpy(pDhKey, s_PkaBuf, P256_BYTES);
	CryptoSecureWipe(s_PkaBuf, sizeof(s_PkaBuf));

	return BT_WBA_PKA_OK;
}
