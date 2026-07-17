/**-------------------------------------------------------------------------
@file	ba414ep.cpp

@brief	Silex BA414EP hardware P-256 engine on the OO engine tree.

		Implements KeyAgreeEngine with the BA414EP scalar point multiply. All
		register and crypto-RAM access is made through the injected CracenIntrf.
		KeyGen multiplies the built-in generator by a fresh private scalar; Agree
		multiplies the peer point by the retained scalar. Hardware scalar and
		projective randomization are enabled for every multiply.

		The private scalar is wiped on failure and is single-use unless Agree is
		explicitly called with bKeepKey.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

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

#include "cracen_intrf.h"
#include "crypto/ba414ep.h"

#ifndef __DMB
#define __DMB()		((void)0)
#endif

#define P256_SZ				32U
#define BA414EP_POLL_LIMIT	10000000U
#define BA414EP_RETRY_COUNT	10U
#define BA414EP_BLIND_SIZE	8U

static void PkWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U)
	{
		*p++ = 0U;
	}
}

static uint32_t PkRegRead(Device *pDev, uint32_t Offset)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CRACEN_ADDR_REG);
	return pDev->Read32(off, 4);
}

static void PkRegWrite(Device *pDev, uint32_t Offset, uint32_t Value)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CRACEN_ADDR_REG);
	pDev->Write32(off, 4, Value);
}

static uint32_t PkOperandOffset(uint32_t Slot, size_t Len)
{
	return Slot * BA414EP_SLOT_SIZE + BA414EP_SLOT_SIZE - (uint32_t)Len;
}

static void PkWriteOperand(Device *pDev, uint32_t Slot, const uint8_t *pSrc,
						   size_t Len)
{
	uint32_t o = PkOperandOffset(Slot, Len);
	uint8_t off[4] = { (uint8_t)o, (uint8_t)(o >> 8),
					   (uint8_t)(o >> 16), (uint8_t)(o >> 24) };
	pDev->DeviceAddress(CRACEN_ADDR_MEM);
	pDev->Write(off, 4, pSrc, (int)Len);
}

static void PkReadOperand(Device *pDev, uint32_t Slot, uint8_t *pDst, size_t Len)
{
	uint32_t o = PkOperandOffset(Slot, Len);
	uint8_t off[4] = { (uint8_t)o, (uint8_t)(o >> 8),
					   (uint8_t)(o >> 16), (uint8_t)(o >> 24) };
	pDev->DeviceAddress(CRACEN_ADDR_MEM);
	pDev->Read(off, 4, pDst, (int)Len);
}

static void PkClearSlot(Device *pDev, uint32_t Slot, size_t Len)
{
	uint8_t zero[P256_SZ] = {};
	PkWriteOperand(pDev, Slot, zero, Len);
}

static bool PkWaitNotBusy(Device *pDev)
{
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		if ((PkRegRead(pDev, BA414EP_REG_STATUS) & BA414EP_STATUS_BUSY) == 0U)
		{
			return true;
		}
	}
	return false;
}

static bool PkWaitIdle(Device *pDev, uint32_t *pStatus)
{
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		uint32_t status = PkRegRead(pDev, BA414EP_REG_STATUS);
		if ((status & BA414EP_STATUS_BUSY) == 0U)
		{
			*pStatus = status & BA414EP_STATUS_ERROR_MASK;
			return true;
		}
	}
	*pStatus = BA414EP_STATUS_BUSY;
	return false;
}

// A timed-out operation left the engine possibly still executing. Crypto RAM
// must not be touched until the engine is confirmed idle again; the flag
// defers the operand wipe to the next successful PkPrepare.
static bool s_bPkAborted = false;

// Abort a still-busy operation: no register or operand access, only the
// module release. Dropping the module enable stops the engine; the next hold
// re-enables it fresh, and the next PkPrepare wipes the operand RAM the
// aborted operation left behind.
static void PkAbort(CracenIntrf *pIntrf)
{
	s_bPkAborted = true;
	pIntrf->ModuleRelease();
}

// Acquire the shared CRACEN operation hold before any register or operand
// access. OK with the hold owned and the engine idle; BUSY when the hold or
// the engine is owned elsewhere (retryable); FAIL only for a missing
// interface. Callers only call PkCleanup after OK.
static CRYPTO_STATUS PkPrepare(Device *pDev, CracenIntrf *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!pIntrf->ModuleHold(CRACEN_MODULE_PKEIKG))
	{
		return CRYPTO_STATUS_BUSY;
	}
	if (!PkWaitNotBusy(pDev))
	{
		pIntrf->ModuleRelease();
		return CRYPTO_STATUS_BUSY;
	}
	if (s_bPkAborted)
	{
		// The engine is idle again after an abort: wipe the operand RAM the
		// aborted operation left, then proceed normally.
		PkClearSlot(pDev, BA414EP_SLOT_SCALAR, P256_SZ);
		PkClearSlot(pDev, BA414EP_SLOT_RESULT_X, P256_SZ);
		PkClearSlot(pDev, BA414EP_SLOT_RESULT_Y, P256_SZ);
		PkClearSlot(pDev, BA414EP_SLOT_POINT_X, P256_SZ);
		PkClearSlot(pDev, BA414EP_SLOT_POINT_Y, P256_SZ);
		PkClearSlot(pDev, BA414EP_SLOT_BLIND, P256_SZ);
		__DMB();
		s_bPkAborted = false;
	}
	return CRYPTO_STATUS_OK;
}

static void PkCleanup(Device *pDev, CracenIntrf *pIntrf)
{
	PkClearSlot(pDev, BA414EP_SLOT_SCALAR, P256_SZ);
	PkClearSlot(pDev, BA414EP_SLOT_RESULT_X, P256_SZ);
	PkClearSlot(pDev, BA414EP_SLOT_RESULT_Y, P256_SZ);
	PkClearSlot(pDev, BA414EP_SLOT_POINT_X, P256_SZ);
	PkClearSlot(pDev, BA414EP_SLOT_POINT_Y, P256_SZ);
	PkClearSlot(pDev, BA414EP_SLOT_BLIND, P256_SZ);
	__DMB();

	PkRegWrite(pDev, BA414EP_REG_CONTROL, BA414EP_CONTROL_CLEAR_IRQ);
	pIntrf->ModuleRelease();
}

static CRYPTO_STATUS PkPointMultiply(Device *pDev, CracenIntrf *pIntrf,
							const uint8_t Point[64], const uint8_t Scalar[32],
							uint8_t Result[64], RngEngine *pRng)
{
	if (pDev == nullptr || pIntrf == nullptr || Point == nullptr ||
		Scalar == nullptr || Result == nullptr || pRng == nullptr ||
		!pRng->IsSecure() || !P256ScalarInRange(Scalar))
	{
		return CRYPTO_STATUS_FAIL;
	}

	memset(Result, 0, 64U);
	for (uint32_t attempt = 0; attempt < BA414EP_RETRY_COUNT; attempt++)
	{
		// Draw before taking CRACEN: the injected RNG may itself use the same
		// interface. Drawing while the PKE hold is owned would deadlock or fail.
		uint8_t blind[BA414EP_BLIND_SIZE];
		if (pRng->Random(blind, sizeof(blind)) != CRYPTO_STATUS_OK)
		{
			PkWipe(blind, sizeof(blind));
			return CRYPTO_STATUS_FAIL;
		}

		// BA414e countermeasure factor requirements in big-endian mode.
		blind[0] = (uint8_t)((blind[0] & 0x3FU) | 0x20U);
		blind[sizeof(blind) - 1U] |= 1U;

		CRYPTO_STATUS ready = PkPrepare(pDev, pIntrf);
		if (ready != CRYPTO_STATUS_OK)
		{
			PkWipe(blind, sizeof(blind));
			return ready;
		}

		uint32_t status = BA414EP_STATUS_BUSY;
		PkRegWrite(pDev, BA414EP_REG_COMMAND, BA414EP_CMD_P256_PTMUL);
		PkWriteOperand(pDev, BA414EP_SLOT_SCALAR, Scalar, P256_SZ);
		PkWriteOperand(pDev, BA414EP_SLOT_POINT_X, &Point[0], P256_SZ);
		PkWriteOperand(pDev, BA414EP_SLOT_POINT_Y, &Point[32], P256_SZ);

		// The countermeasure factor is one full 32 byte operand in slot 15
		// with the 8 byte value at the big-endian tail. Zero the window first
		// so power-on crypto RAM content cannot enter the factor.
		PkClearSlot(pDev, BA414EP_SLOT_BLIND, P256_SZ);
		PkWriteOperand(pDev, BA414EP_SLOT_BLIND, blind, sizeof(blind));
		PkRegWrite(pDev, BA414EP_REG_CONFIG, BA414EP_CONFIG_PTMUL);

		__DMB();
		PkRegWrite(pDev, BA414EP_REG_CONTROL,
				   BA414EP_CONTROL_START | BA414EP_CONTROL_CLEAR_IRQ);
		if (!PkWaitIdle(pDev, &status))
		{
			// The operation is still executing past the poll limit: abort
			// without touching crypto RAM. The scalar is wiped from RAM by
			// the deferred cleanup at the next successful PkPrepare.
			PkAbort(pIntrf);
			PkWipe(blind, sizeof(blind));
			return CRYPTO_STATUS_FAIL;
		}
		__DMB();

		if (status == 0U)
		{
			PkReadOperand(pDev, BA414EP_SLOT_RESULT_X, &Result[0], P256_SZ);
			PkReadOperand(pDev, BA414EP_SLOT_RESULT_Y, &Result[32], P256_SZ);
		}

		PkCleanup(pDev, pIntrf);
		PkWipe(blind, sizeof(blind));

		if (status == 0U && !P256IsZero(&Result[0], P256_SZ) &&
			!P256IsZero(&Result[32], P256_SZ))
		{
			return CRYPTO_STATUS_OK;
		}
		memset(Result, 0, 64U);

		if (status != BA414EP_STATUS_NOT_INVERTIBLE)
		{
			return CRYPTO_STATUS_FAIL;
		}
	}
	return CRYPTO_STATUS_FAIL;
}

static const uint8_t s_P256Generator[64] = {
	0x6B,0x17,0xD1,0xF2,0xE1,0x2C,0x42,0x47,0xF8,0xBC,0xE6,0xE5,0x63,0xA4,0x40,0xF2,
	0x77,0x03,0x7D,0x81,0x2D,0xEB,0x33,0xA0,0xF4,0xA1,0x39,0x45,0xD8,0x98,0xC2,0x96,
	0x4F,0xE3,0x42,0xE2,0xFE,0x1A,0x7F,0x9B,0x8E,0xE7,0xEB,0x4A,0x7C,0x0F,0x9E,0x16,
	0x2B,0xCE,0x33,0x57,0x6B,0x31,0x5E,0xCE,0xCB,0xB6,0x40,0x68,0x37,0xBF,0x51,0xF5
};

bool Ba414ep::Init(CracenIntrf * const pIntrf, RngEngine *pRng)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	vpCracen = pIntrf;
	Interface(pIntrf);
	vpRng = pRng;
	return Enable();
}

bool Ba414ep::Enable()
{
	bool ok = PkPrepare(this, vpCracen) == CRYPTO_STATUS_OK;
	if (ok)
	{
		PkCleanup(this, vpCracen);
	}
	vbValid = ok;
	return ok;
}

void Ba414ep::KeyReset(void *pKeyCtx)
{
	if (pKeyCtx != nullptr)
	{
		PkWipe(pKeyCtx, sizeof(KeyCtx));
	}
}

CRYPTO_STATUS Ba414ep::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx, uint8_t *pPubKey)
{
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	KeyCtx *pk = (KeyCtx *)pKeyCtx;
	KeyReset(pk);
	memset(pPubKey, 0, 64U);

	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	if (!P256RandomScalar(vpRng, pk->PrivKey))
	{
		KeyReset(pk);
		return CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS st = PkPointMultiply(this, vpCracen, s_P256Generator,
									   pk->PrivKey, pPubKey, vpRng);
	if (st == CRYPTO_STATUS_OK)
	{
		pk->bKeyValid = true;
		return CRYPTO_STATUS_OK;
	}

	KeyReset(pk);
	memset(pPubKey, 0, 64U);
	return st;
}

// Known-answer self-test: the Bluetooth LESC debug key pair (Core spec Vol 3
// Part H 2.3.5.6.1). The generator multiplied by the debug private scalar
// must yield the debug public key, exercising the full blinded hardware
// multiply path.
int Ba414ep::SelfTest()
{
	static const uint8_t priv[32] = {
		0x3F,0x49,0xF6,0xD4,0xA3,0xC5,0x5F,0x38,0x74,0xC9,0xB3,0xE3,0xD2,0x10,0x3F,0x50,
		0x4A,0xFF,0x60,0x7B,0xEB,0x40,0xB7,0x99,0x58,0x99,0xB8,0xA6,0xCD,0x3C,0x1A,0xBD,
	};
	static const uint8_t pub[64] = {
		0x1E,0xA1,0xF0,0xF0,0x1F,0xAF,0x1D,0x96,0x09,0x59,0x22,0x84,0xF1,0x9E,0x4C,0x00,
		0x47,0xB5,0x8A,0xFD,0x86,0x15,0xA6,0x9F,0x55,0x90,0x77,0xB2,0x2F,0xAA,0xA1,0x90,
		0x4C,0x55,0xF3,0x3E,0x42,0x9D,0xAD,0x37,0x73,0x56,0x70,0x3A,0x9A,0xB8,0x51,0x60,
		0x47,0x2D,0x11,0x30,0xE2,0x8E,0x36,0x76,0x5F,0x89,0xAF,0xF9,0x15,0xB1,0x21,0x4A,
	};
	if (!vbValid)
	{
		return -1;
	}
	uint8_t result[64];
	CRYPTO_STATUS st = PkPointMultiply(this, vpCracen, s_P256Generator, priv,
									   result, vpRng);
	int rc = (st == CRYPTO_STATUS_OK &&
			  memcmp(result, pub, sizeof(pub)) == 0) ? 0 : -2;
	PkWipe(result, sizeof(result));
	return rc;
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
	if (!pk->bKeyValid || vpRng == nullptr || !vpRng->IsSecure())
	{
		memset(pSharedX, 0, P256_SZ);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t point[64];
	CRYPTO_STATUS st = PkPointMultiply(this, vpCracen, pPeerPubKey,
									   pk->PrivKey, point, vpRng);

	// BUSY is transient contention: the shared secret is cleared and the
	// single-use key survives for retry. Every other failure consumes it.
	if (st == CRYPTO_STATUS_OK)
	{
		if (!bKeepKey)
		{
			KeyReset(pk);
		}
		memcpy(pSharedX, point, P256_SZ);
	}
	else
	{
		if (st != CRYPTO_STATUS_BUSY)
		{
			KeyReset(pk);
		}
		memset(pSharedX, 0, P256_SZ);
	}
	PkWipe(point, sizeof(point));
	return st;
}
