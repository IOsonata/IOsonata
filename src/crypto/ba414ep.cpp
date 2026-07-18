/**-------------------------------------------------------------------------
@file	ba414ep.cpp

@brief	Silex BA414EP hardware P-256 engine on the OO engine tree.

		Implements KeyAgreeEngine with the BA414EP scalar point multiply. All
		register and crypto-RAM access is made through the injected DeviceIntrf.
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

#include "crypto/ba414ep.h"

// Diagnostic tracing. Define BA414EP_TRACE_ENABLE to trace every operation.
//#define BA414EP_TRACE_ENABLE
#if defined(BA414EP_TRACE_ENABLE)
#include <stdio.h>
#define BA414EP_TRACE(...)	printf(__VA_ARGS__)
#else
#define BA414EP_TRACE(...)
#endif

#ifndef __DMB
#define __DMB()		((void)0)
#endif

#define P256_SZ				32U
#define BA414EP_POLL_LIMIT	10000000U
#define BA414EP_RETRY_COUNT	10U
#define BA414EP_BLIND_SIZE	8U

// A failed selected-module reset quarantines the engine. No later operation may
// enter while the public-key engine can still be active with private operands.
static bool s_bPkQuarantined = false;

// Operand slot stride in CryptoRAM, derived from the hardware geometry at the
// first Enable. The default matches the 4096-bit configuration until HWCONFIG
// has been read.
static uint32_t s_PkSlotSize = BA414EP_OPERAND_SLOT_4096;

// NIST P-256 domain parameters and generator, big endian.
static const uint8_t s_P256Prime[32] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};
static const uint8_t s_P256Order[32] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xBC,0xE6,0xFA,0xAD,0xA7,0x17,0x9E,0x84,0xF3,0xB9,0xCA,0xC2,0xFC,0x63,0x25,0x51,
};
static const uint8_t s_P256CoeffA[32] = {
	0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,
};
static const uint8_t s_P256CoeffB[32] = {
	0x5A,0xC6,0x35,0xD8,0xAA,0x3A,0x93,0xE7,0xB3,0xEB,0xBD,0x55,0x76,0x98,0x86,0xBC,
	0x65,0x1D,0x06,0xB0,0xCC,0x53,0xB0,0xF6,0x3B,0xCE,0x3C,0x3E,0x27,0xD2,0x60,0x4B,
};
static const uint8_t s_P256Generator[64] = {
	0x6B,0x17,0xD1,0xF2,0xE1,0x2C,0x42,0x47,0xF8,0xBC,0xE6,0xE5,0x63,0xA4,0x40,0xF2,
	0x77,0x03,0x7D,0x81,0x2D,0xEB,0x33,0xA0,0xF4,0xA1,0x39,0x45,0xD8,0x98,0xC2,0x96,
	0x4F,0xE3,0x42,0xE2,0xFE,0x1A,0x7F,0x9B,0x8E,0xE7,0xEB,0x4A,0x7C,0x0F,0x9E,0x16,
	0x2B,0xCE,0x33,0x57,0x6B,0x31,0x5E,0xCE,0xCB,0xB6,0x40,0x68,0x37,0xBF,0x51,0xF5
};

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
	pDev->DeviceAddress(BA414EP_ADDR_REG);
	return pDev->Read32(off, 4);
}

static void PkRegWrite(Device *pDev, uint32_t Offset, uint32_t Value)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(BA414EP_ADDR_REG);
	pDev->Write32(off, 4, Value);
}

static uint32_t PkOperandOffset(uint32_t Slot, size_t Len)
{
	return Slot * s_PkSlotSize + s_PkSlotSize - (uint32_t)Len;
}

static void PkWriteOperand(Device *pDev, uint32_t Slot, const uint8_t *pSrc,
						   size_t Len)
{
	uint32_t o = PkOperandOffset(Slot, Len);
	uint8_t off[4] = { (uint8_t)o, (uint8_t)(o >> 8),
					   (uint8_t)(o >> 16), (uint8_t)(o >> 24) };
	pDev->DeviceAddress(BA414EP_ADDR_MEM);
	pDev->Write(off, 4, pSrc, (int)Len);
}

static void PkReadOperand(Device *pDev, uint32_t Slot, uint8_t *pDst, size_t Len)
{
	uint32_t o = PkOperandOffset(Slot, Len);
	uint8_t off[4] = { (uint8_t)o, (uint8_t)(o >> 8),
					   (uint8_t)(o >> 16), (uint8_t)(o >> 24) };
	pDev->DeviceAddress(BA414EP_ADDR_MEM);
	pDev->Read(off, 4, pDst, (int)Len);
}

static void PkClearSlot(Device *pDev, uint32_t Slot, size_t Len)
{
	uint8_t zero[P256_SZ] = {};
	PkWriteOperand(pDev, Slot, zero, Len);
}

// Read and validate the hardware geometry from HWCONFIG: bits [11:0] hold the
// maximum prime field operand size in bytes, bit 16 prime field support, bit 31
// countermeasures disabled. P-256 with the blinding this driver always applies
// needs prime field support, at least a 32 byte operand, and the
// countermeasures present. The slot stride follows the operand size the same
// way the Silex driver sizes it: 4096-bit slots unless the maximum operand
// exceeds them. Reject a configuration that fails these requirements instead
// of computing CryptoRAM offsets from an unverified geometry.
static bool PkGeometryValid(Device *pDev)
{
	uint32_t cfg = PkRegRead(pDev, BA414EP_REG_HWCONFIG);
	uint32_t maxOpSize = cfg & BA414EP_HWCONFIG_MAX_OPSIZE_Msk;

	if (maxOpSize < P256_SZ ||
		(cfg & BA414EP_HWCONFIG_PRIME_FIELD_Msk) == 0U ||
		(cfg & BA414EP_HWCONFIG_CM_DISABLED_Msk) != 0U)
	{
		BA414EP_TRACE("Ba414ep geometry rejected: hwconfig=%08x\r\n",
					  (unsigned)cfg);
		return false;
	}
	s_PkSlotSize = maxOpSize > BA414EP_OPERAND_SLOT_4096 ?
		BA414EP_OPERAND_SLOT_8192 : BA414EP_OPERAND_SLOT_4096;
	return true;
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

CRYPTO_STATUS Ba414ep::PkPrepare()
{
	if (!vbValid || !vbIntrfEnabled || Interface() == nullptr || s_bPkQuarantined)
	{
		BA414EP_TRACE("Ba414ep PkPrepare: valid=%d enabled=%d intrf=%p quarantined=%d -> FAIL\r\n",
					  (int)vbValid, (int)vbIntrfEnabled, (void *)Interface(),
					  (int)s_bPkQuarantined);
		return CRYPTO_STATUS_FAIL;
	}
	if (!OpAcquire())
	{
		BA414EP_TRACE("Ba414ep PkPrepare: OpAcquire failed -> BUSY\r\n");
		return CRYPTO_STATUS_BUSY;
	}
	if (!PkWaitNotBusy(this))
	{
		BA414EP_TRACE("Ba414ep PkPrepare: PkWaitNotBusy timeout -> BUSY\r\n");
		OpRelease();
		return CRYPTO_STATUS_BUSY;
	}
	return CRYPTO_STATUS_OK;
}

void Ba414ep::PkCleanup()
{
	PkClearSlot(this, BA414EP_SLOT_SCALAR_GEN, P256_SZ);
	PkClearSlot(this, BA414EP_SLOT_RESULT_GX, P256_SZ);
	PkClearSlot(this, BA414EP_SLOT_RESULT_GY, P256_SZ);
	PkClearSlot(this, BA414EP_SLOT_BLIND, P256_SZ);
	__DMB();

	PkRegWrite(this, BA414EP_REG_CONTROL, BA414EP_CONTROL_CLEAR_IRQ);
	OpRelease();
}

bool Ba414ep::Recover()
{
	DeviceIntrf *pIntrf = Interface();

	vbReady = false;
	if (pIntrf == nullptr)
	{
		return false;
	}

	// Reset through the interface with the PKE selected. The StartTx/StopTx
	// pair serializes the reset against other engines sharing the transport
	// and latches the module an address-selective interface reset acts on.
	// A transport that cannot even open a transfer is unusable; the engine
	// stays down rather than resetting an unselected module.
	if (!pIntrf->StartTx(BA414EP_ADDR_REG))
	{
		return false;
	}
	pIntrf->Reset();
	pIntrf->StopTx();

	if (!WaitIkIdle() || !HandoverProbe() || !PkWaitNotBusy(this))
	{
		return false;
	}

	PkClearSlot(this, BA414EP_SLOT_SCALAR_GEN, P256_SZ);
	PkClearSlot(this, BA414EP_SLOT_RESULT_GX, P256_SZ);
	PkClearSlot(this, BA414EP_SLOT_RESULT_GY, P256_SZ);
	PkClearSlot(this, BA414EP_SLOT_BLIND, P256_SZ);
	PkRegWrite(this, BA414EP_REG_CONTROL, BA414EP_CONTROL_CLEAR_IRQ);
	__DMB();
	vbReady = true;
	s_bPkQuarantined = false;
	return true;
}

bool Ba414ep::PkAbortAndReset()
{
	if (Interface() == nullptr || !Recover())
	{
		vbValid = false;
		s_bPkQuarantined = true;
		return false;
	}
	OpRelease();
	return true;
}

CRYPTO_STATUS Ba414ep::PkPointMultiply(const uint8_t Point[64],
							const uint8_t Scalar[32], uint8_t Result[64],
							bool bValidatePoint)
{
	const bool argsOk = Point != nullptr && Scalar != nullptr && Result != nullptr;
	const bool rngOk = vpRng != nullptr && vpRng->IsSecure();
	const bool scalarOk = Scalar != nullptr && P256ScalarInRange(Scalar);
	if (!argsOk || !rngOk || !scalarOk)
	{
		BA414EP_TRACE("Ba414ep PkMul: args=%d rng=%p secure=%d scalarok=%d -> FAIL\r\n",
					  (int)argsOk, (void *)vpRng, (int)rngOk, (int)scalarOk);
		if (Result != nullptr)
		{
			memset(Result, 0, 64U);
		}
		return CRYPTO_STATUS_FAIL;
	}

	memset(Result, 0, 64U);
	for (uint32_t attempt = 0; attempt < BA414EP_RETRY_COUNT; attempt++)
	{
		uint8_t blind[BA414EP_BLIND_SIZE];
		CRYPTO_STATUS rngStatus = vpRng->Random(blind, sizeof(blind));
		if (rngStatus != CRYPTO_STATUS_OK)
		{
			BA414EP_TRACE("Ba414ep PkMul: blinding RNG draw failed st=%d\r\n",
						  (int)rngStatus);
			PkWipe(blind, sizeof(blind));
			return rngStatus;
		}

		blind[0] = (uint8_t)((blind[0] & 0x3FU) | 0x20U);
		blind[sizeof(blind) - 1U] |= 1U;

		CRYPTO_STATUS ready = PkPrepare();
		if (ready != CRYPTO_STATUS_OK)
		{
			PkWipe(blind, sizeof(blind));
			return ready;
		}

		uint32_t status = BA414EP_STATUS_BUSY;
		PkWriteOperand(this, BA414EP_SLOT_P, s_P256Prime, P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_N, s_P256Order, P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_GX, &Point[0], P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_GY, &Point[32], P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_A, s_P256CoeffA, P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_B, s_P256CoeffB, P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_SCALAR_GEN, Scalar, P256_SZ);

		PkClearSlot(this, BA414EP_SLOT_BLIND, P256_SZ);
		PkWriteOperand(this, BA414EP_SLOT_BLIND, blind, sizeof(blind));

		PkRegWrite(this, BA414EP_REG_CONFIG, BA414EP_CONFIG_PTMUL_GEN);
		__DMB();

		if (bValidatePoint)
		{
			PkRegWrite(this, BA414EP_REG_COMMAND, BA414EP_CMD_P256_PTONCURVE);
			__DMB();
			PkRegWrite(this, BA414EP_REG_CONTROL,
					   BA414EP_CONTROL_START | BA414EP_CONTROL_CLEAR_IRQ);
			if (!PkWaitIdle(this, &status))
			{
				BA414EP_TRACE("Ba414ep PkMul: on-curve check timeout -> FAIL\r\n");
				(void)PkAbortAndReset();
				PkWipe(blind, sizeof(blind));
				return CRYPTO_STATUS_FAIL;
			}
			if (status != 0U)
			{
				BA414EP_TRACE("Ba414ep PkMul: point rejected status=0x%x -> FAIL\r\n",
							  (unsigned)status);
				PkCleanup();
				PkWipe(blind, sizeof(blind));
				return CRYPTO_STATUS_FAIL;
			}
		}

		PkRegWrite(this, BA414EP_REG_COMMAND, BA414EP_CMD_P256_PTMUL_GEN);
		__DMB();
#if defined(BA414EP_TRACE_ENABLE)
		BA414EP_TRACE("Ba414ep PkMul: rd cmd=0x%x cfg=0x%x (cfg want 0x060e02)\r\n",
					  (unsigned)PkRegRead(this, BA414EP_REG_COMMAND),
					  (unsigned)PkRegRead(this, BA414EP_REG_CONFIG));
#endif
		PkRegWrite(this, BA414EP_REG_CONTROL,
				   BA414EP_CONTROL_START | BA414EP_CONTROL_CLEAR_IRQ);
#if defined(BA414EP_TRACE_ENABLE)
		BA414EP_TRACE("Ba414ep PkMul: post-START status=0x%x ctrl=0x%x\r\n",
					  (unsigned)PkRegRead(this, BA414EP_REG_STATUS),
					  (unsigned)PkRegRead(this, BA414EP_REG_CONTROL));
#endif
		if (!PkWaitIdle(this, &status))
		{
			BA414EP_TRACE("Ba414ep PkMul: PkWaitIdle timeout, status=0x%x -> FAIL\r\n",
						  (unsigned)status);
			(void)PkAbortAndReset();
			PkWipe(blind, sizeof(blind));
			return CRYPTO_STATUS_FAIL;
		}
		__DMB();
		BA414EP_TRACE("Ba414ep PkMul: attempt %u hw status=0x%x\r\n",
					  (unsigned)attempt, (unsigned)status);

		if (status == 0U)
		{
			PkReadOperand(this, BA414EP_SLOT_RESULT_GX, &Result[0], P256_SZ);
			PkReadOperand(this, BA414EP_SLOT_RESULT_GY, &Result[32], P256_SZ);
#if defined(BA414EP_TRACE_ENABLE)
			{
				uint8_t p12[P256_SZ], p13[P256_SZ];
				PkReadOperand(this, BA414EP_SLOT_GX, p12, P256_SZ);
				PkReadOperand(this, BA414EP_SLOT_GY, p13, P256_SZ);
				BA414EP_TRACE("Ba414ep PkMul: rX %02x%02x%02x%02x rY %02x%02x%02x%02x bX %02x%02x%02x%02x bY %02x%02x%02x%02x\r\n",
							  Result[0], Result[1], Result[2], Result[3],
							  Result[32], Result[33], Result[34], Result[35],
							  p12[0], p12[1], p12[2], p12[3],
							  p13[0], p13[1], p13[2], p13[3]);
			}
#endif
		}

		PkCleanup();
		PkWipe(blind, sizeof(blind));

		if (status == 0U && !P256IsZero(Result, 64U))
		{
			return CRYPTO_STATUS_OK;
		}
		BA414EP_TRACE("Ba414ep PkMul: reject status=0x%x point_zero=%d\r\n",
					  (unsigned)status, (int)P256IsZero(Result, 64U));
		memset(Result, 0, 64U);

		if (status != BA414EP_STATUS_NOT_INVERTIBLE)
		{
			return CRYPTO_STATUS_FAIL;
		}
	}
	BA414EP_TRACE("Ba414ep PkMul: all %u attempts exhausted -> FAIL\r\n",
				  (unsigned)BA414EP_RETRY_COUNT);
	return CRYPTO_STATUS_FAIL;
}

Ba414ep::Ba414ep()
{
	vbValid = false;
	vpRng = nullptr;
	vbReady = false;
	vbIntrfEnabled = false;
	atomic_flag_clear(&vOpBusy);
}

bool Ba414ep::OpAcquire()
{
	return atomic_flag_test_and_set(&vOpBusy) == false;
}

void Ba414ep::OpRelease()
{
	atomic_flag_clear(&vOpBusy);
}

bool Ba414ep::WaitIkIdle()
{
	for (uint32_t i = 0; i < BA414EP_POLL_LIMIT; i++)
	{
		if ((PkRegRead(this, BA414EP_REG_STATUS) & BA414EP_STATUS_BUSY) == 0U &&
			(PkRegRead(this, BA414EP_IK_REG_PK_STATUS) &
			 BA414EP_IK_PK_BUSY_MASK) == 0U)
		{
			return true;
		}
	}
	return false;
}

bool Ba414ep::HandoverProbe()
{
	static const uint8_t mod[4] = { 0, 0, 0, 13 };
	static const uint8_t opa[4] = { 0, 0, 0, 7 };
	static const uint8_t opb[4] = { 0, 0, 0, 8 };

	for (uint32_t attempt = 0; attempt < 2U; attempt++)
	{
		PkWriteOperand(this, 0U, mod, sizeof(mod));
		PkWriteOperand(this, 6U, opa, sizeof(opa));
		PkWriteOperand(this, 8U, opb, sizeof(opb));
		PkRegWrite(this, BA414EP_REG_CONFIG, BA414EP_CONFIG_PTRS(6U, 8U, 10U));
		PkRegWrite(this, BA414EP_REG_COMMAND,
				   BA414EP_CMD_MOD_ADD | BA414EP_CMD_OPSIZE(4U) |
				   BA414EP_CMD_BIG_ENDIAN | BA414EP_CMD_RESQUARE);
		__DMB();
		PkRegWrite(this, BA414EP_REG_CONTROL,
				   BA414EP_CONTROL_START | BA414EP_CONTROL_CLEAR_IRQ);

		uint32_t status;
		bool idle = PkWaitIdle(this, &status);
		PkRegWrite(this, BA414EP_REG_CONTROL, BA414EP_CONTROL_CLEAR_IRQ);

		uint8_t res[4];
		PkReadOperand(this, 10U, res, sizeof(res));
		if (idle && status == 0U && res[0] == 0U && res[1] == 0U &&
			res[2] == 0U && res[3] == 2U)
		{
			return true;
		}
	}
	return false;
}

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
	if (Interface() == nullptr || s_bPkQuarantined)
	{
		vbValid = false;
		return false;
	}
	if (vbIntrfEnabled && vbValid)
	{
		return true;
	}

	bool acquiredRef = false;
	if (!vbIntrfEnabled)
	{
		Interface()->Enable();
		vbIntrfEnabled = true;
		acquiredRef = true;
	}

	if (!OpAcquire())
	{
		vbValid = false;
		if (acquiredRef)
		{
			Interface()->Disable();
			vbIntrfEnabled = false;
		}
		return false;
	}

	const bool ok = vbReady ||
		(PkGeometryValid(this) && WaitIkIdle() && HandoverProbe());
	OpRelease();
	if (!ok)
	{
		vbValid = false;
		vbReady = false;
		if (acquiredRef)
		{
			Interface()->Disable();
			vbIntrfEnabled = false;
		}
		return false;
	}

	vbReady = true;
	vbValid = true;
	return true;
}

void Ba414ep::Disable()
{
	if (!vbIntrfEnabled || Interface() == nullptr)
	{
		vbValid = false;
		return;
	}
	if (!OpAcquire())
	{
		return;
	}
	vbValid = false;
	Interface()->Disable();
	vbIntrfEnabled = false;
	OpRelease();
}

void Ba414ep::Reset()
{
	if (Interface() == nullptr || !vbIntrfEnabled || !OpAcquire())
	{
		return;
	}
	vbValid = false;
	if (Recover())
	{
		vbValid = true;
		OpRelease();
	}
	else
	{
		s_bPkQuarantined = true;
	}
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

	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (vpRng == nullptr || !vpRng->IsSecure())
	{
		BA414EP_TRACE("Ba414ep KeyGen: rng=%p secure=%d -> UNSUPPORTED\r\n",
					  (void *)vpRng, vpRng != nullptr ? (int)vpRng->IsSecure() : -1);
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	CRYPTO_STATUS rngStatus = P256RandomScalarStatus(vpRng, pk->PrivKey);
	if (rngStatus != CRYPTO_STATUS_OK)
	{
		BA414EP_TRACE("Ba414ep KeyGen: private key draw failed st=%d\r\n",
					  (int)rngStatus);
		KeyReset(pk);
		return rngStatus;
	}
	CRYPTO_STATUS st = PkPointMultiply(s_P256Generator, pk->PrivKey, pPubKey,
									   false);
	if (st == CRYPTO_STATUS_OK)
	{
		pk->bKeyValid = true;
		return CRYPTO_STATUS_OK;
	}

	BA414EP_TRACE("Ba414ep KeyGen: PkPointMultiply -> st=%d\r\n", (int)st);
	KeyReset(pk);
	memset(pPubKey, 0, 64U);
	return st;
}

int Ba414ep::SelfTest()
{
	static const uint8_t priv[32] = {
		0x3F,0x49,0xF6,0xD4,0xA3,0xC5,0x5F,0x38,0x74,0xC9,0xB3,0xE3,0xD2,0x10,0x3F,0x50,
		0x4A,0xFF,0x60,0x7B,0xEB,0x40,0xB7,0x99,0x58,0x99,0xB8,0xA6,0xCD,0x3C,0x1A,0xBD,
	};
	static const uint8_t pub[64] = {
		0x20,0xB0,0x03,0xD2,0xF2,0x97,0xBE,0x2C,0x5E,0x2C,0x83,0xA7,0xE9,0xF9,0xA5,0xB9,
		0xEF,0xF4,0x91,0x11,0xAC,0xF4,0xFD,0xDB,0xCC,0x03,0x01,0x48,0x0E,0x35,0x9D,0xE6,
		0xDC,0x80,0x9C,0x49,0x65,0x2A,0xEB,0x6D,0x63,0x32,0x9A,0xBF,0x5A,0x52,0x15,0x5C,
		0x76,0x63,0x45,0xC2,0x8F,0xED,0x30,0x24,0x74,0x1C,0x8E,0xD0,0x15,0x89,0xD2,0x8B,
	};
	if (!vbValid)
	{
		return -1;
	}
	uint8_t result[64];
	CRYPTO_STATUS st = PkPointMultiply(s_P256Generator, priv, result, false);
	int rc = 0;
	if (st == CRYPTO_STATUS_BUSY)
	{
		rc = -6;
	}
	else if (st != CRYPTO_STATUS_OK)
	{
		rc = -2;
	}
	else if (memcmp(&result[0], &pub[0], P256_SZ) != 0)
	{
		rc = -3;
	}
	else if (memcmp(&result[32], &pub[32], P256_SZ) != 0)
	{
		rc = -4;
	}
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
	if (!vbValid || !pk->bKeyValid || vpRng == nullptr || !vpRng->IsSecure())
	{
		BA414EP_TRACE("Ba414ep Agree: valid=%d keyvalid=%d rng=%p secure=%d -> FAIL\r\n",
					  (int)vbValid, (int)pk->bKeyValid, (void *)vpRng,
					  (int)(vpRng != nullptr && vpRng->IsSecure()));
		KeyReset(pk);
		memset(pSharedX, 0, P256_SZ);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t point[64];
	CRYPTO_STATUS st = PkPointMultiply(pPeerPubKey, pk->PrivKey, point, true);

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
