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

#include "cracen_intrf.h"
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

// Register and operand access uses the interface direct accessors inside a held
// operation. Offsets are the Silex IP layout; the interface adds the base.
static uint32_t PkRegRead(CracenIntrf *pIntrf, uint32_t Offset)
{
	return pIntrf->RegRead(Offset);
}

static void PkRegWrite(CracenIntrf *pIntrf, uint32_t Offset, uint32_t Value)
{
	pIntrf->RegWrite(Offset, Value);
}

// Byte offset of a 32-byte P-256 operand inside its slot, relative to the
// operand memory base. Big-endian operands sit at the end of the slot.
static uint32_t PkOperandOffset(uint32_t Slot)
{
	return Slot * BA414EP_SLOT_SIZE + BA414EP_SLOT_SIZE - P256_SZ;
}

static void PkWriteOperand(CracenIntrf *pIntrf, uint32_t Slot,
						   const uint8_t *pSrc, size_t Len)
{
	pIntrf->MemWrite(PkOperandOffset(Slot), pSrc, Len);
}

static void PkReadOperand(CracenIntrf *pIntrf, uint32_t Slot, uint8_t *pDst,
						  size_t Len)
{
	pIntrf->MemRead(PkOperandOffset(Slot), pDst, Len);
}

static void PkClearSlot(CracenIntrf *pIntrf, uint32_t Slot)
{
	uint8_t zero[P256_SZ];
	memset(zero, 0, sizeof(zero));
	PkWriteOperand(pIntrf, Slot, zero, P256_SZ);
}

// Wait for the core to leave busy. Returns false on timeout. Does not treat a
// lingering error bit from a previous operation as a failure: the current
// operation's status is read after it runs, in PkWaitResult.
static bool PkWaitNotBusy(CracenIntrf *pIntrf)
{
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		if ((PkRegRead(pIntrf, BA414EP_REG_STATUS) & BA414EP_STATUS_BUSY) == 0U)
		{
			return true;
		}
	}
	return false;
}

// Wait for the core to leave busy, then return the masked error bits of the
// completed operation (0 = success).
static bool PkWaitIdle(CracenIntrf *pIntrf, uint32_t *pStatus)
{
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		uint32_t status = PkRegRead(pIntrf, BA414EP_REG_STATUS);
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
static bool PkPrepare(CracenIntrf *pIntrf)
{
	// Enable the public-key module for the whole operation. It stays enabled
	// across every register and operand access until PkCleanup disables it,
	// matching the reference driver that enables once and disables once.
	pIntrf->ModuleHold(CRACEN_MODULE_PKEIKG);
	return PkWaitNotBusy(pIntrf);
}

// Clear the operand slots the multiply touched, clear the completion flag, and
// power the core down.
static void PkCleanup(CracenIntrf *pIntrf)
{
	PkClearSlot(pIntrf, BA414EP_SLOT_SCALAR);
	PkClearSlot(pIntrf, BA414EP_SLOT_RESULT_X);
	PkClearSlot(pIntrf, BA414EP_SLOT_RESULT_Y);
	PkClearSlot(pIntrf, BA414EP_SLOT_POINT_X);
	PkClearSlot(pIntrf, BA414EP_SLOT_POINT_Y);
	__DMB();

	PkRegWrite(pIntrf, BA414EP_REG_CONTROL, BA414EP_CONTROL_CLEAR_IRQ);
	// Power the module down at the end of the operation.
	pIntrf->ModuleRelease();
}

// Result = Scalar * Point on P-256, with scalar blinding. The scalar must be in
// range; the point is supplied by the caller. The core rejects an off-curve
// point with a point-error status. A non-invertible intermediate is retried
// with a fresh blinding factor. Returns true and a valid result on success.
static bool PkPointMultiply(CracenIntrf *pIntrf,
							const uint8_t Point[64], const uint8_t Scalar[32],
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
		bool prepared = PkPrepare(pIntrf);
		uint32_t status = BA414EP_STATUS_BUSY;

		if (prepared)
		{
			// Order matches the reference driver: command (with operand size),
			// then operands, then the slot-pointer config, then start.
			PkRegWrite(pIntrf, BA414EP_REG_COMMAND, BA414EP_CMD_P256_PTMUL);

			PkWriteOperand(pIntrf, BA414EP_SLOT_SCALAR,  Scalar,     P256_SZ);
			PkWriteOperand(pIntrf, BA414EP_SLOT_POINT_X, &Point[0],  P256_SZ);
			PkWriteOperand(pIntrf, BA414EP_SLOT_POINT_Y, &Point[32], P256_SZ);

			PkRegWrite(pIntrf, BA414EP_REG_CONFIG, BA414EP_CONFIG_PTMUL);

			__DMB();
			PkRegWrite(pIntrf, BA414EP_REG_CONTROL,
					   BA414EP_CONTROL_START | BA414EP_CONTROL_CLEAR_IRQ);
			(void)PkWaitIdle(pIntrf, &status);
			__DMB();

			if (status == 0U)
			{
				PkReadOperand(pIntrf, BA414EP_SLOT_RESULT_X, &Result[0],  P256_SZ);
				PkReadOperand(pIntrf, BA414EP_SLOT_RESULT_Y, &Result[32], P256_SZ);
			}
		}

		PkCleanup(pIntrf);

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

bool Ba414ep::Init(DeviceIntrf * const pIntrf, RngEngine *pRng)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	Interface(pIntrf);
	vpRng = pRng;
	return Enable();
}

bool Ba414ep::Enable()
{
	// Probe the core: a prepare/cleanup round confirms the public-key module
	// responds. Register and operand access run through the bound interface.
	bool ok = PkPrepare((CracenIntrf *)Interface());
	if (ok)
	{
		PkCleanup((CracenIntrf *)Interface());
	}
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
		PkPointMultiply((CracenIntrf *)Interface(), s_P256Generator, pk->PrivKey, pPubKey, vpRng))
	{
		pk->bKeyValid = true;
		return CRYPTO_STATUS_OK;
	}

	PkWipe(pk->PrivKey, sizeof(pk->PrivKey));
	memset(pPubKey, 0, 64U);
	return CRYPTO_STATUS_FAIL;
}

CRYPTO_STATUS Ba414ep::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
							 const uint8_t *pPeerPubKey, uint8_t *pSharedX,
							 bool bKeepKey)
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
	bool ok = PkPointMultiply((CracenIntrf *)Interface(), pPeerPubKey, pk->PrivKey, point, vpRng);

	// Wipe the private scalar unless the caller asked to keep it after a success
	// (bKeepKey), for one ephemeral key pair against several peers. A failure
	// always wipes.
	if (!bKeepKey || !ok)
	{
		PkWipe(pk->PrivKey, sizeof(pk->PrivKey));
		pk->bKeyValid = false;
	}

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
