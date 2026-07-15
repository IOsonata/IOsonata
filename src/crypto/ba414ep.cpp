/**-------------------------------------------------------------------------
@file	ba414ep.cpp

@brief	Silex BA414EP hardware P-256 engine on the OO engine tree.

		Implements KeyAgreeEngine with the BA414EP scalar point multiply. All
		access is raw register and crypto-RAM read/write at the offsets in
		ba414ep.h, reached through the injected base and the vendor hooks; no
		vendor HAL and no architecture register name here.

		The core is fixed function for P-256: the curve is selected by a command
		bit and its parameters are built into the hardware, so no curve constants
		and no microcode are loaded. KeyGen multiplies the built-in generator by
		a fresh private scalar; Agree multiplies the peer point by the retained
		scalar. Both enable scalar blinding as a differential-power-analysis
		countermeasure and retry on a non-invertible intermediate. An off-curve
		peer point is rejected by the core (point-error status), closing the
		invalid-curve attack (CVE-2018-5383).

		The private scalar is single use: wiped on Agree and on every failure.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/ba414ep.h"

#ifndef __DMB
#define __DMB()		((void)0)
#endif

#define P256_SZ				32U
#define BA414EP_POLL_LIMIT	10000000U		// point multiply is long; bounded
#define BA414EP_RETRY_COUNT	10U

static void PkWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

static inline volatile uint32_t *PkReg(void)
{
	return (volatile uint32_t *)((uintptr_t)Ba414epBase());
}

// Address of a 32-byte P-256 operand inside its slot. Big-endian operands are
// placed at the end of the slot, so the value occupies the last P256_SZ bytes.
static inline volatile uint8_t *PkOperand(uint32_t Slot)
{
	uintptr_t base = (uintptr_t)Ba414epOperandRam();
	return (volatile uint8_t *)(base + Slot * BA414EP_SLOT_SIZE +
								BA414EP_SLOT_SIZE - P256_SZ);
}

static void PkWrite(volatile uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[i];
	}
}

static void PkRead(uint8_t *pDst, const volatile uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[i];
	}
}

static void PkClearSlot(uint32_t Slot)
{
	volatile uint8_t *p = PkOperand(Slot);
	for (size_t i = 0; i < P256_SZ; i++)
	{
		p[i] = 0;
	}
}

// Wait for the core to leave busy. Returns false on timeout. Does not treat a
// lingering error bit from a previous operation as a failure: the current
// operation's status is read after it runs, in PkWaitResult.
static bool PkWaitNotBusy(void)
{
	volatile uint32_t *reg = PkReg();
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		if ((reg[BA414EP_REG_STATUS / sizeof(uint32_t)] & BA414EP_STATUS_BUSY) == 0U)
		{
			return true;
		}
	}
	return false;
}

// Wait for the core to leave busy, then return the masked error bits of the
// completed operation (0 = success).
static bool PkWaitIdle(uint32_t *pStatus)
{
	volatile uint32_t *reg = PkReg();
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		uint32_t status = reg[BA414EP_REG_STATUS / sizeof(uint32_t)];
		if ((status & BA414EP_STATUS_BUSY) == 0U)
		{
			*pStatus = status & BA414EP_STATUS_ERROR_MASK;
			return true;
		}
	}
	*pStatus = BA414EP_STATUS_BUSY;
	return false;
}

// Power the core and wait until it is not busy. A stale error bit from a prior
// operation is not a bring-up failure; only readiness (not busy) matters here.
static bool PkPrepare(void)
{
	Ba414epModuleEnable();
	return PkWaitNotBusy();
}

// Clear the operand slots the multiply touched, clear the completion flag, and
// power the core down.
static void PkCleanup(void)
{
	PkClearSlot(BA414EP_SLOT_SCALAR);
	PkClearSlot(BA414EP_SLOT_RESULT_X);
	PkClearSlot(BA414EP_SLOT_RESULT_Y);
	PkClearSlot(BA414EP_SLOT_POINT_X);
	PkClearSlot(BA414EP_SLOT_POINT_Y);
	__DMB();

	volatile uint32_t *reg = PkReg();
	reg[BA414EP_REG_CONTROL / sizeof(uint32_t)] = BA414EP_CONTROL_CLEAR_IRQ;
	Ba414epModuleDisable();
}

// Result = Scalar * Point on P-256, with scalar blinding. The scalar must be in
// range; the point is supplied by the caller. The core rejects an off-curve
// point with a point-error status. A non-invertible intermediate is retried
// with a fresh blinding factor. Returns true and a valid result on success.
static bool PkPointMultiply(const uint8_t Point[64], const uint8_t Scalar[32],
							uint8_t Result[64], RngEngine *pRng)
{
	if (Point == nullptr || Scalar == nullptr || Result == nullptr ||
		pRng == nullptr || !pRng->IsSecure() || !P256ScalarInRange(Scalar))
	{
		return false;
	}

	memset(Result, 0, 64U);
	for (uint32_t attempt = 0; attempt < BA414EP_RETRY_COUNT; attempt++)
	{
		if (!Ba414epTryAcquire())
		{
			return false;
		}

		bool prepared = PkPrepare();
		uint32_t status = BA414EP_STATUS_BUSY;

		if (prepared)
		{
			volatile uint32_t *reg = PkReg();

			// Order matches the reference driver: command (with operand size),
			// then operands, then the slot-pointer config, then start.
			reg[BA414EP_REG_COMMAND / sizeof(uint32_t)] = BA414EP_CMD_P256_PTMUL;

			PkWrite(PkOperand(BA414EP_SLOT_SCALAR),  Scalar,     P256_SZ);
			PkWrite(PkOperand(BA414EP_SLOT_POINT_X), &Point[0],  P256_SZ);
			PkWrite(PkOperand(BA414EP_SLOT_POINT_Y), &Point[32], P256_SZ);

			reg[BA414EP_REG_CONFIG / sizeof(uint32_t)] = BA414EP_CONFIG_PTMUL;

			__DMB();
			reg[BA414EP_REG_CONTROL / sizeof(uint32_t)] =
				BA414EP_CONTROL_START | BA414EP_CONTROL_CLEAR_IRQ;
			(void)PkWaitIdle(&status);
			__DMB();

			if (status == 0U)
			{
				PkRead(&Result[0],  PkOperand(BA414EP_SLOT_RESULT_X), P256_SZ);
				PkRead(&Result[32], PkOperand(BA414EP_SLOT_RESULT_Y), P256_SZ);
			}
		}

		PkCleanup();
		Ba414epRelease();

		if (prepared && status == 0U &&
			!P256IsZero(&Result[0], P256_SZ) &&
			!P256IsZero(&Result[32], P256_SZ))
		{
			return true;
		}
		memset(Result, 0, 64U);

		// Only a non-invertible intermediate is retryable; any other failure
		// (off-curve point, timeout, bring-up) is terminal.
		if (!prepared || status != BA414EP_STATUS_NOT_INVERTIBLE)
		{
			return false;
		}
	}
	return false;
}

// P-256 generator (SEC1 uncompressed X||Y, big-endian). Public constant.
static const uint8_t s_P256Generator[64] = {
	0x6B,0x17,0xD1,0xF2,0xE1,0x2C,0x42,0x47,0xF8,0xBC,0xE6,0xE5,0x63,0xA4,0x40,0xF2,
	0x77,0x03,0x7D,0x81,0x2D,0xEB,0x33,0xA0,0xF4,0xA1,0x39,0x45,0xD8,0x98,0xC2,0x96,
	0x4F,0xE3,0x42,0xE2,0xFE,0x1A,0x7F,0x9B,0x8E,0xE7,0xEB,0x4A,0x7C,0x0F,0x9E,0x16,
	0x2B,0xCE,0x33,0x57,0x6B,0x31,0x5E,0xCE,0xCB,0xB6,0x40,0x68,0x37,0xBF,0x51,0xF5 };

bool Ba414ep::Enable()
{
	if (!Ba414epTryAcquire())
	{
		return false;
	}
	bool ok = PkPrepare();
	if (ok)
	{
		PkCleanup();
	}
	else
	{
		Ba414epModuleDisable();
	}
	Ba414epRelease();
	vbValid = ok;
	return ok;
}

CRYPTO_STATUS Ba414ep::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;		// fail closed on a weak RNG
	}
	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	pk->bKeyValid = false;
	PkWipe(pk->PrivKey, sizeof(pk->PrivKey));

	// Public key = generator * private scalar.
	if (P256RandomScalar(pk->PrivKey) &&
		PkPointMultiply(s_P256Generator, pk->PrivKey, pPubKey, vpRng))
	{
		pk->bKeyValid = true;
		return CRYPTO_STATUS_OK;
	}

	PkWipe(pk->PrivKey, sizeof(pk->PrivKey));
	memset(pPubKey, 0, 64U);
	return CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS Ba414ep::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
							 const uint8_t *pPeerPubKey, uint8_t *pSharedX)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
		pPeerPubKey == nullptr || pSharedX == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	KeyCtx *pk = (KeyCtx *)pKeyCtx;

	// Fail closed: Agree before KeyGen, or a second Agree after the single-use
	// key was consumed and wiped.
	if (!pk->bKeyValid || vpRng == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Shared secret = peer point * private scalar. The core rejects an off-curve
	// peer point (CVE-2018-5383) as a point error, surfaced as failure here.
	uint8_t point[64];
	bool ok = PkPointMultiply(pPeerPubKey, pk->PrivKey, point, vpRng);

	// Single use: wipe the private scalar on every exit.
	PkWipe(pk->PrivKey, sizeof(pk->PrivKey));
	pk->bKeyValid = false;

	if (ok)
	{
		memcpy(pSharedX, point, P256_SZ);		// shared X coordinate
	}
	else
	{
		memset(pSharedX, 0, P256_SZ);
	}
	PkWipe(point, sizeof(point));
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

Ba414ep *Ba414epCreate(void *pMem, size_t MemSize, RngEngine *pRng)
{
	if (pMem == nullptr || MemSize < sizeof(Ba414ep))
	{
		return nullptr;
	}
	Ba414ep *p = new (pMem) Ba414ep();
	p->SetRng(pRng);
	if (!p->Enable())
	{
		return nullptr;		// core absent or no P-256 support
	}
	return p;
}
