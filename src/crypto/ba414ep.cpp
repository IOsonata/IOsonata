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

// Acquire the shared CRACEN operation hold before any register or operand
// access. If the core does not become ready, release immediately; callers must
// only call PkCleanup after this function returns true.
static bool PkPrepare(Device *pDev, CracenIntrf *pIntrf)
{
	if (pIntrf == nullptr || !pIntrf->ModuleHold(CRACEN_MODULE_PKEIKG))
	{
		return false;
	}
	if (!PkWaitNotBusy(pDev))
	{
		pIntrf->ModuleRelease();
		return false;
	}
	return true;
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

static bool PkPointMultiply(Device *pDev, CracenIntrf *pIntrf,
							const uint8_t Point[64], const uint8_t Scalar[32],
							uint8_t Result[64], RngEngine *pRng)
{
	if (pDev == nullptr || pIntrf == nullptr || Point == nullptr ||
		Scalar == nullptr || Result == nullptr || pRng == nullptr ||
		!pRng->IsSecure() || !P256ScalarInRange(Scalar))
	{
		return false;
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
			return false;
		}

		// BA414e countermeasure factor requirements in big-endian mode.
		blind[0] = (uint8_t)((blind[0] & 0x3FU) | 0x20U);
		blind[sizeof(blind) - 1U] |= 1U;

		if (!PkPrepare(pDev, pIntrf))
		{
			PkWipe(blind, sizeof(blind));
			return false;
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
		(void)PkWaitIdle(pDev, &status);
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
			return true;
		}
		memset(Result, 0, 64U);

		if (status != BA414EP_STATUS_NOT_INVERTIBLE)
		{
			return false;
		}
	}
	return false;
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
	bool ok = PkPrepare(this, vpCracen);
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

	if (P256RandomScalar(vpRng, pk->PrivKey) &&
		PkPointMultiply(this, vpCracen, s_P256Generator, pk->PrivKey,
						pPubKey, vpRng))
	{
		pk->bKeyValid = true;
		return CRYPTO_STATUS_OK;
	}

	KeyReset(pk);
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
	if (!pk->bKeyValid || vpRng == nullptr || !vpRng->IsSecure())
	{
		memset(pSharedX, 0, P256_SZ);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t point[64];
	bool ok = PkPointMultiply(this, vpCracen, pPeerPubKey, pk->PrivKey,
							 point, vpRng);

	if (!bKeepKey || !ok)
	{
		KeyReset(pk);
	}

	if (ok)
	{
		memcpy(pSharedX, point, P256_SZ);
	}
	else
	{
		memset(pSharedX, 0, P256_SZ);
	}
	PkWipe(point, sizeof(point));
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}
