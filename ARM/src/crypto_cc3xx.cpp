/**-------------------------------------------------------------------------
@file	crypto_cc3xx.cpp

@brief	IOsonata P-256 engine for Arm CryptoCell CC3xx PKA.

		Implements P-256 key generation, public-key validation, and ECDH
		directly on the CC3xx public-key accelerator. The implementation is
		self-contained and does not depend on PSA, tf-psa-crypto-drivers, a
		vendor binary, dynamic allocation, or a generic multi-curve framework.

		The vendor implements the cc3xx_intrf.h surface with the CC3xx register base
		and the target-specific enable and disable operations. All PKA register
		handling and P-256 arithmetic remain private to this translation unit.

@author	Hoang Nguyen Hoan
@date	Jul. 12, 2026

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
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include "crypto/icrypto.h"
#include "cmsis_compiler.h"

#include "crypto_cc3xx.h"
#include "cc3xx_intrf.h"
#include "coredev/interrupt.h"
#include <new>

#ifndef CC3XX_PKA_WAIT_SPINS
#define CC3XX_PKA_WAIT_SPINS		10000000U
#endif

#ifndef CC3XX_ACQUIRE_SPINS
#define CC3XX_ACQUIRE_SPINS		100000U
#endif

// Internal PKA driver: file-static helpers and constants (like ba414ep.cpp).

constexpr size_t P256_WORDS = P256_BYTES / sizeof(uint32_t);
constexpr uint32_t PKA_REG_BYTES = 48U;
constexpr uint32_t PKA_REG_WORDS = PKA_REG_BYTES / sizeof(uint32_t);
constexpr uint32_t PKA_PHYS_REG_COUNT = 32U;

static_assert(PKA_PHYS_REG_COUNT * PKA_REG_BYTES <= CC3XX_PKA_SRAM_SIZE,
			  "CC3xx PKA SRAM is too small for the fixed P-256 register map");
typedef uint8_t PkaReg_t;

enum : PkaReg_t {
	PKA_N = 0,
	PKA_NP = 1,
	PKA_N_MASK = 2,
	P256_P_X = 3,
	P256_P_Y = 4,
	P256_P_Z = 5,
	P256_R_X = 6,
	P256_R_Y = 7,
	P256_R_Z = 8,
	P256_D_X = 9,
	P256_D_Y = 10,
	P256_D_Z = 11,
	P256_A_X = 12,
	P256_A_Y = 13,
	P256_A_Z = 14,
	P256_CONST_A = 15,
	P256_CONST_B = 16,
	P256_T0 = 17,
	P256_T1 = 18,
	P256_T2 = 19,
	P256_T3 = 20,
	P256_T4 = 21,
	P256_T5 = 22,
	P256_T6 = 23,
	P256_T7 = 24,
	P256_T8 = 25,
	P256_T9 = 26,
	P256_T10 = 27,
	P256_T11 = 28,
	P256_T12 = 29,
};

struct PkaPoint {
	PkaReg_t X;
	PkaReg_t Y;
	PkaReg_t Z;
};

constexpr PkaPoint POINT_P = {P256_P_X, P256_P_Y, P256_P_Z};
constexpr PkaPoint POINT_R = {P256_R_X, P256_R_Y, P256_R_Z};
constexpr PkaPoint POINT_D = {P256_D_X, P256_D_Y, P256_D_Z};
constexpr PkaPoint POINT_A = {P256_A_X, P256_A_Y, P256_A_Z};
constexpr PkaPoint POINT_ADD_DOUBLE = {P256_T6, P256_T7, P256_T8};
constexpr PkaPoint POINT_INFINITY = {P256_T9, P256_T10, P256_T11};

enum : uint32_t {
	REG_PKA_MEMORY_MAP = 0x000U,
	REG_PKA_OPCODE = 0x080U,
	REG_PKA_STATUS = 0x088U,
	REG_PKA_SW_RESET = 0x08CU,
	REG_PKA_L_N = 0x090U,
	REG_PKA_L_REGISTER = 0x094U,
	REG_PKA_PIPE_READY = 0x0B0U,
	REG_PKA_DONE = 0x0B4U,
	REG_PKA_SRAM_ADDR = 0x0D4U,
	REG_PKA_SRAM_WRITE = 0x0D8U,
	REG_PKA_SRAM_READ = 0x0DCU,
	REG_PKA_SRAM_READ_ADDR = 0x0E4U,
	REG_PKA_CLOCK_ENABLE = 0x81CU,
	REG_DMA_CLOCK_ENABLE = 0x820U,
	REG_CRYPTO_CTL = 0x900U,
	REG_HOST_ENDIAN = 0xA0CU,
	REG_AHB_SINGLE = 0xB00U,
	REG_AHB_HPROT = 0xB04U,
	REG_AHB_HNONSEC = 0xB0CU,
};

enum : uint32_t {
	PKA_OP_ADD = 0x04U,
	PKA_OP_SUB = 0x05U,
	PKA_OP_MOD_ADD = 0x06U,
	PKA_OP_MOD_SUB = 0x07U,
	PKA_OP_AND = 0x08U,
	PKA_OP_COPY = 0x09U,
	PKA_OP_XOR_COMPARE = 0x0AU,
	PKA_OP_MOD_MUL = 0x11U,
	PKA_OP_MOD_EXP = 0x13U,
};

enum : uint32_t {
	PKA_SIZE_N = 0U,
	PKA_SIZE_REGISTER = 1U,
	PKA_STATUS_SIGN = (1U << 8),
	PKA_STATUS_ZERO = (1U << 12),
};

static const uint32_t s_P256Field[P256_WORDS] = {
	0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0x00000000U,
	0x00000000U, 0x00000000U, 0x00000001U, 0xFFFFFFFFU,
};

static const uint32_t s_P256FieldBarrett[10] = {
	0xFFFFFFFFU, 0x0000007FU, 0x00000080U, 0x00000000U,
	0x00000000U, 0xFFFFFFFFU, 0x0000007FU, 0x00000080U,
	0x00000000U, 0x00000000U,
};

static const uint32_t s_P256A[P256_WORDS] = {
	0xFFFFFFFCU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0x00000000U,
	0x00000000U, 0x00000000U, 0x00000001U, 0xFFFFFFFFU,
};

static const uint32_t s_P256B[P256_WORDS] = {
	0x27D2604BU, 0x3BCE3C3EU, 0xCC53B0F6U, 0x651D06B0U,
	0x769886BCU, 0xB3EBBD55U, 0xAA3A93E7U, 0x5AC635D8U,
};

static bool s_bCcInit;
static bool s_bPkaFault;

static bool CoreInit(Device *pDev);

static uint32_t CcRegRead(Device *pDev, uint32_t Offset)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CC3XX_ADDR_REG);
	return pDev->Read32(off, 4);
}

static void CcRegWrite(Device *pDev, uint32_t Offset, uint32_t Value)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CC3XX_ADDR_REG);
	pDev->Write32(off, 4, Value);
}

static inline uint32_t PkaMap(Device *pDev, uint32_t Reg)
{
	return CcRegRead(pDev, REG_PKA_MEMORY_MAP + Reg * sizeof(uint32_t));
}

static inline void PkaMapWrite(Device *pDev, uint32_t Reg, uint32_t Value)
{
	CcRegWrite(pDev, REG_PKA_MEMORY_MAP + Reg * sizeof(uint32_t), Value);
}

static void PkaFault(Device *pDev)
{
	s_bPkaFault = true;
	CcRegWrite(pDev, REG_PKA_CLOCK_ENABLE, 0U);
	// Fault kill: unconditional power-off through the interface PowerOff
	// escape hatch. The user count is intentionally untouched; counted users
	// re-assert power on their next operation through EnsureCc3xx, which
	// reruns the core bring-up because s_bCcInit is cleared here.
	Cc3xxIntrf *pIntrf = Cc3xxIntrfInstance();
	if (pIntrf != nullptr)
	{
		pIntrf->PowerOff();
	}
	s_bCcInit = false;
}

static bool PkaWait(Device *pDev, uint32_t Offset)
{
	if (s_bPkaFault)
	{
		return false;
	}

	for (uint32_t spin = 0U; spin < CC3XX_PKA_WAIT_SPINS; spin++)
	{
		if (CcRegRead(pDev, Offset) != 0U)
		{
			return true;
		}
	}

	PkaFault(pDev);
	return false;
}

static inline bool PkaWaitDone(Device *pDev)
{
	return PkaWait(pDev, REG_PKA_DONE);
}

static inline bool PkaWaitPipe(Device *pDev)
{
	return PkaWait(pDev, REG_PKA_PIPE_READY);
}

static inline bool PkaRegValid(PkaReg_t Reg)
{
	return Reg <= P256_T12;
}

static uint32_t PkaOpcode(Device *pDev, uint32_t Op, uint32_t Size,
						 bool AImmediate, uint32_t A,
						 bool BImmediate, uint32_t B,
						 bool Discard, PkaReg_t Result)
{
	assert(AImmediate || PkaRegValid((PkaReg_t)A));
	assert(BImmediate || PkaRegValid((PkaReg_t)B));
	assert(Discard || PkaRegValid(Result));

	uint32_t opcode = Discard ? (1U << 11) : (((uint32_t)Result & 0x1FU) << 6);
	opcode |= (BImmediate ? (1U << 17) : 0U) | ((B & 0x1FU) << 12);
	opcode |= (AImmediate ? (1U << 23) : 0U) | ((A & 0x1FU) << 18);
	opcode |= (Size & 0x7U) << 24;
	opcode |= (Op & 0x1FU) << 27;
	(void)PkaWaitPipe(pDev);
	return opcode;
}

static inline void PkaSubmit(Device *pDev, uint32_t Op, uint32_t Size,
							 bool AImmediate, uint32_t A,
							 bool BImmediate, uint32_t B,
							 bool Discard, PkaReg_t Result)
{
	if (s_bPkaFault)
	{
		return;
	}
	const uint32_t opcode = PkaOpcode(pDev, Op, Size, AImmediate, A,
										 BImmediate, B, Discard, Result);
	if (!s_bPkaFault)
	{
		CcRegWrite(pDev, REG_PKA_OPCODE, opcode);
	}
}

static inline uint32_t LoadBe32(const uint8_t *pData)
{
	return ((uint32_t)pData[0] << 24) | ((uint32_t)pData[1] << 16) |
		   ((uint32_t)pData[2] << 8) | (uint32_t)pData[3];
}

static inline void StoreBe32(uint8_t *pData, uint32_t Value)
{
	pData[0] = (uint8_t)(Value >> 24);
	pData[1] = (uint8_t)(Value >> 16);
	pData[2] = (uint8_t)(Value >> 8);
	pData[3] = (uint8_t)Value;
}

static void PkaClear(Device *pDev, PkaReg_t Reg)
{
	assert(PkaRegValid(Reg));
	if (!PkaWaitDone(pDev))
	{
		return;
	}
	CcRegWrite(pDev, REG_PKA_SRAM_ADDR, PkaMap(pDev, Reg));
	for (uint32_t idx = 0U; idx < PKA_REG_WORDS; idx++)
	{
		CcRegWrite(pDev, REG_PKA_SRAM_WRITE, 0U);
		if (!PkaWaitDone(pDev))
		{
			return;
		}
	}
}

static void PkaWrite(Device *pDev, PkaReg_t Reg, const uint32_t *pData, size_t Len)
{
	assert(PkaRegValid(Reg));
	assert(pData != nullptr);
	assert(Len <= PKA_REG_BYTES && (Len & 3U) == 0U);
	PkaClear(pDev, Reg);
	if (s_bPkaFault)
	{
		return;
	}
	CcRegWrite(pDev, REG_PKA_SRAM_ADDR, PkaMap(pDev, Reg));
	for (size_t idx = 0U; idx < Len / sizeof(uint32_t); idx++)
	{
		CcRegWrite(pDev, REG_PKA_SRAM_WRITE, pData[idx]);
		if (!PkaWaitDone(pDev))
		{
			return;
		}
	}
}

static void PkaWriteBe(Device *pDev, PkaReg_t Reg, const uint8_t *pData, size_t Len)
{
	assert(PkaRegValid(Reg));
	assert(pData != nullptr);
	assert(Len <= PKA_REG_BYTES && (Len & 3U) == 0U);
	PkaClear(pDev, Reg);
	if (s_bPkaFault)
	{
		return;
	}
	CcRegWrite(pDev, REG_PKA_SRAM_ADDR, PkaMap(pDev, Reg));
	for (size_t idx = 0U; idx < Len / sizeof(uint32_t); idx++)
	{
		const size_t offset = Len - (idx + 1U) * sizeof(uint32_t);
		CcRegWrite(pDev, REG_PKA_SRAM_WRITE, LoadBe32(&pData[offset]));
		if (!PkaWaitDone(pDev))
		{
			return;
		}
	}
}

static void PkaReadBe(Device *pDev, PkaReg_t Reg, uint8_t *pData, size_t Len)
{
	assert(PkaRegValid(Reg));
	assert(pData != nullptr);
	assert(Len <= PKA_REG_BYTES && (Len & 3U) == 0U);
	memset(pData, 0, Len);
	if (!PkaWaitDone(pDev))
	{
		return;
	}
	for (size_t idx = 0U; idx < Len / sizeof(uint32_t); idx++)
	{
		CcRegWrite(pDev, REG_PKA_SRAM_READ_ADDR, PkaMap(pDev, Reg) + (uint32_t)idx);
		if (!PkaWaitDone(pDev))
		{
			memset(pData, 0, Len);
			return;
		}
		const size_t offset = Len - (idx + 1U) * sizeof(uint32_t);
		StoreBe32(&pData[offset], CcRegRead(pDev, REG_PKA_SRAM_READ));
	}
}

static inline void PkaCopy(Device *pDev, PkaReg_t Src, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_COPY, PKA_SIZE_REGISTER, false, Src, true, 0U, false, Dst);
}

static inline void PkaAddImm(Device *pDev, PkaReg_t A, int32_t Value, PkaReg_t Dst)
{
	assert(Value >= -16 && Value <= 15);
	PkaSubmit(pDev, PKA_OP_ADD, PKA_SIZE_REGISTER, false, A, true,
			  (uint32_t)Value, false, Dst);
}

static inline void PkaSubImm(Device *pDev, PkaReg_t A, int32_t Value, PkaReg_t Dst)
{
	assert(Value >= -16 && Value <= 15);
	PkaSubmit(pDev, PKA_OP_SUB, PKA_SIZE_REGISTER, false, A, true,
			  (uint32_t)Value, false, Dst);
}

static inline void PkaModAdd(Device *pDev, PkaReg_t A, PkaReg_t B, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_MOD_ADD, PKA_SIZE_REGISTER, false, A, false, B, false, Dst);
}

static inline void PkaModSub(Device *pDev, PkaReg_t A, PkaReg_t B, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_MOD_SUB, PKA_SIZE_REGISTER, false, A, false, B, false, Dst);
}

static inline void PkaAnd(Device *pDev, PkaReg_t A, PkaReg_t B, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_AND, PKA_SIZE_REGISTER, false, A, false, B, false, Dst);
}

static inline void PkaXor(Device *pDev, PkaReg_t A, PkaReg_t B, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_XOR_COMPARE, PKA_SIZE_REGISTER,
			  false, A, false, B, false, Dst);
}

static inline void PkaMask(Device *pDev, PkaReg_t Reg)
{
	PkaAnd(pDev, Reg, PKA_N_MASK, Reg);
}

static inline void PkaModMul(Device *pDev, PkaReg_t A, PkaReg_t B, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_MOD_MUL, PKA_SIZE_N, false, A, false, B, false, Dst);
	PkaMask(pDev, Dst);
}

static inline void PkaModExp(Device *pDev, PkaReg_t A, PkaReg_t Exp, PkaReg_t Dst)
{
	PkaSubmit(pDev, PKA_OP_MOD_EXP, PKA_SIZE_N, false, A, false, Exp, false, Dst);
	PkaMask(pDev, Dst);
}

static inline bool PkaEqual(Device *pDev, PkaReg_t A, PkaReg_t B)
{
	PkaSubmit(pDev, PKA_OP_XOR_COMPARE, PKA_SIZE_REGISTER,
			  false, A, false, B, true, 0U);
	if (!PkaWaitDone(pDev))
	{
		return false;
	}
	return (CcRegRead(pDev, REG_PKA_STATUS) & PKA_STATUS_ZERO) != 0U;
}

static inline bool PkaEqualImm(Device *pDev, PkaReg_t A, int32_t Value)
{
	assert(Value >= -16 && Value <= 15);
	PkaSubmit(pDev, PKA_OP_XOR_COMPARE, PKA_SIZE_REGISTER,
			  false, A, true, (uint32_t)Value, true, 0U);
	if (!PkaWaitDone(pDev))
	{
		return false;
	}
	return (CcRegRead(pDev, REG_PKA_STATUS) & PKA_STATUS_ZERO) != 0U;
}

static inline bool PkaLess(Device *pDev, PkaReg_t A, PkaReg_t B)
{
	PkaSubmit(pDev, PKA_OP_SUB, PKA_SIZE_REGISTER, false, A, false, B, true, 0U);
	if (!PkaWaitDone(pDev))
	{
		return false;
	}
	return (CcRegRead(pDev, REG_PKA_STATUS) & PKA_STATUS_SIGN) != 0U;
}

static void PkaSetOne(Device *pDev, PkaReg_t Reg)
{
	PkaClear(pDev, Reg);
	PkaAddImm(pDev, Reg, 1, Reg);
}

static void PkaSetFieldModulus(Device *pDev)
{
	static const uint32_t mask[12] = {
		0xFFFFFFFFU,0xFFFFFFFFU,0xFFFFFFFFU,0xFFFFFFFFU,
		0xFFFFFFFFU,0xFFFFFFFFU,0xFFFFFFFFU,0xFFFFFFFFU,
		0U,0U,0U,0U,
	};
	PkaWrite(pDev, PKA_N, s_P256Field, sizeof(s_P256Field));
	PkaWrite(pDev, PKA_NP, s_P256FieldBarrett, sizeof(s_P256FieldBarrett));
	PkaWrite(pDev, PKA_N_MASK, mask, sizeof(mask));
	if (!s_bPkaFault)
	{
		CcRegWrite(pDev, REG_PKA_L_N, 256U);
	}
}

static bool PkaInit(Device *pDev)
{
	s_bPkaFault = false;

	if (s_bCcInit == false)
	{
		s_bCcInit = CoreInit(pDev);
		if (s_bCcInit == false)
		{
			return false;
		}
	}

	CcRegWrite(pDev, REG_PKA_CLOCK_ENABLE, 1U);
	CcRegWrite(pDev, REG_PKA_SW_RESET, 1U);
	if (!PkaWaitDone(pDev))
	{
		return false;
	}
	CcRegWrite(pDev, REG_PKA_L_REGISTER, PKA_REG_BYTES * 8U);
	CcRegWrite(pDev, REG_PKA_L_N, 256U);
	for (uint32_t reg = 0U; reg < PKA_PHYS_REG_COUNT; reg++)
	{
		PkaMapWrite(pDev, reg, reg * PKA_REG_WORDS);
		if (!PkaWaitDone(pDev))
		{
			return false;
		}
	}
	PkaSetFieldModulus(pDev);
	PkaWrite(pDev, P256_CONST_A, s_P256A, sizeof(s_P256A));
	PkaWrite(pDev, P256_CONST_B, s_P256B, sizeof(s_P256B));
	return !s_bPkaFault && CcRegRead(pDev, REG_PKA_CLOCK_ENABLE) != 0U;
}

static void PkaUninit(Device *pDev)
{
	if (s_bPkaFault || !PkaWaitDone(pDev))
	{
		return;
	}
	for (uint32_t slot = 0U; slot < PKA_PHYS_REG_COUNT; slot++)
	{
		CcRegWrite(pDev, REG_PKA_SRAM_ADDR, slot * PKA_REG_WORDS);
		for (uint32_t idx = 0U; idx < PKA_REG_WORDS; idx++)
		{
			CcRegWrite(pDev, REG_PKA_SRAM_WRITE, 0U);
			if (!PkaWaitDone(pDev))
			{
				return;
			}
		}
	}
	CcRegWrite(pDev, REG_PKA_CLOCK_ENABLE, 0U);
}

static void PointCopy(Device *pDev, const PkaPoint &Src, const PkaPoint &Dst)
{
	PkaCopy(pDev, Src.X, Dst.X);
	PkaCopy(pDev, Src.Y, Dst.Y);
	PkaCopy(pDev, Src.Z, Dst.Z);
}

static void PointInfinity(Device *pDev, const PkaPoint &P)
{
	PkaClear(pDev, P.X);
	PkaSetOne(pDev, P.Y);
	PkaClear(pDev, P.Z);
}

static bool PointIsInfinity(Device *pDev, const PkaPoint &P)
{
	return PkaEqualImm(pDev, P.Z, 0);
}

static void PointDouble(Device *pDev, const PkaPoint &P, const PkaPoint &R)
{
	const PkaReg_t t0 = P256_T0;
	const PkaReg_t t1 = P256_T1;
	const PkaReg_t t2 = P256_T2;
	const PkaReg_t t3 = P256_T3;

	PkaModMul(pDev, P.Y, P.Y, t0);
	PkaModMul(pDev, P.Z, P.Z, t1);
	PkaModMul(pDev, P.X, P.X, t2);
	PkaModMul(pDev, t0, t0, t3);
	PkaModAdd(pDev, P.Y, P.Z, R.Z);
	PkaModMul(pDev, R.Z, R.Z, R.Z);
	PkaModSub(pDev, R.Z, t0, R.Z);
	PkaModSub(pDev, R.Z, t1, R.Z);
	PkaModAdd(pDev, P.X, t0, t0);
	PkaModMul(pDev, t0, t0, t0);
	PkaModSub(pDev, t0, t2, t0);
	PkaModSub(pDev, t0, t3, t0);
	PkaModAdd(pDev, t0, t0, t0);
	PkaModMul(pDev, t1, t1, t1);
	PkaModMul(pDev, P256_CONST_A, t1, t1);
	PkaModAdd(pDev, t1, t2, t1);
	PkaModAdd(pDev, t1, t2, t1);
	PkaModAdd(pDev, t1, t2, t1);
	PkaModMul(pDev, t1, t1, R.X);
	PkaModSub(pDev, R.X, t0, R.X);
	PkaModSub(pDev, R.X, t0, R.X);
	PkaModSub(pDev, t0, R.X, R.Y);
	PkaModMul(pDev, t1, R.Y, R.Y);
	PkaSetOne(pDev, t2);
	PkaModAdd(pDev, t2, t2, t2);
	PkaModAdd(pDev, t2, t2, t2);
	PkaModAdd(pDev, t2, t2, t2);
	PkaModMul(pDev, t2, t3, t3);
	PkaModSub(pDev, R.Y, t3, R.Y);
}

// Select one of two projective points through fixed PKA registers. The private
// bit changes only the value loaded into the fixed mask register. The opcode
// stream and all source and destination register numbers remain unchanged.
static void PointSelectCopy(Device *pDev, const PkaPoint &Zero, const PkaPoint &One,
							uint32_t Bit, const PkaPoint &Dst)
{
	uint32_t mask[PKA_REG_WORDS];
	const uint32_t value = 0U - (Bit & 1U);

	for (size_t idx = 0U; idx < P256_WORDS; idx++)
	{
		mask[idx] = value;
	}
	for (size_t idx = P256_WORDS; idx < PKA_REG_WORDS; idx++)
	{
		mask[idx] = 0U;
	}
	PkaWrite(pDev, P256_T12, mask, sizeof(mask));

	const PkaReg_t zeroReg[3] = {Zero.X, Zero.Y, Zero.Z};
	const PkaReg_t oneReg[3] = {One.X, One.Y, One.Z};
	const PkaReg_t dstReg[3] = {Dst.X, Dst.Y, Dst.Z};

	for (size_t coord = 0U; coord < 3U; coord++)
	{
		PkaXor(pDev, zeroReg[coord], oneReg[coord], P256_T0);
		PkaAnd(pDev, P256_T0, P256_T12, P256_T0);
		PkaXor(pDev, zeroReg[coord], P256_T0, dstReg[coord]);
	}

	CryptoSecureWipe(mask, sizeof(mask));
}

static void PointAdd(Device *pDev, const PkaPoint &P, const PkaPoint &Q, const PkaPoint &R)
{
	const PkaReg_t t0 = P256_T0;
	const PkaReg_t t1 = P256_T1;
	const PkaReg_t u1 = P256_T2;
	const PkaReg_t u2 = P256_T3;
	const PkaReg_t s1 = P256_T4;
	const PkaReg_t s2 = P256_T5;

	const uint32_t pInfinity = (uint32_t)PointIsInfinity(pDev, P);
	const uint32_t qInfinity = (uint32_t)PointIsInfinity(pDev, Q);

	PkaModMul(pDev, P.Z, P.Z, t0);
	PkaModMul(pDev, Q.Z, Q.Z, t1);
	PkaModMul(pDev, P.X, t1, u1);
	PkaModMul(pDev, Q.X, t0, u2);
	PkaModMul(pDev, P.Y, Q.Z, s1);
	PkaModMul(pDev, s1, t1, s1);
	PkaModMul(pDev, Q.Y, P.Z, s2);
	PkaModMul(pDev, s2, t0, s2);

	const uint32_t sameX = (uint32_t)PkaEqual(pDev, u1, u2);
	const uint32_t sameY = (uint32_t)PkaEqual(pDev, s1, s2);

	PkaModSub(pDev, u2, u1, u2);
	PkaModAdd(pDev, P.Z, Q.Z, R.Z);
	PkaModMul(pDev, R.Z, R.Z, R.Z);
	PkaModSub(pDev, R.Z, t0, R.Z);
	PkaModSub(pDev, R.Z, t1, R.Z);
	PkaModMul(pDev, R.Z, u2, R.Z);
	PkaModAdd(pDev, u2, u2, t1);
	PkaModMul(pDev, t1, t1, t1);
	PkaModMul(pDev, u1, t1, u1);
	PkaModMul(pDev, u2, t1, t1);
	PkaModSub(pDev, s2, s1, u2);
	PkaModAdd(pDev, u2, u2, u2);
	PkaModMul(pDev, u2, u2, R.X);
	PkaModSub(pDev, R.X, t1, R.X);
	PkaModSub(pDev, R.X, u1, R.X);
	PkaModSub(pDev, R.X, u1, R.X);
	PkaModMul(pDev, t1, s1, t1);
	PkaModAdd(pDev, t1, t1, t1);
	PkaModSub(pDev, u1, R.X, R.Y);
	PkaModMul(pDev, R.Y, u2, R.Y);
	PkaModSub(pDev, R.Y, t1, R.Y);

	PointDouble(pDev, P, POINT_ADD_DOUBLE);
	PointInfinity(pDev, POINT_INFINITY);

	const uint32_t samePoint = sameX & sameY;
	const uint32_t inversePoint = sameX & (1U ^ sameY);
	PointSelectCopy(pDev, R, POINT_ADD_DOUBLE, samePoint, R);
	PointSelectCopy(pDev, R, POINT_INFINITY, inversePoint, R);
	PointSelectCopy(pDev, R, P, qInfinity, R);
	PointSelectCopy(pDev, R, Q, pInfinity, R);
}

static bool PointValidate(Device *pDev, PkaReg_t X, PkaReg_t Y)
{
	if (!PkaLess(pDev, X, PKA_N) || !PkaLess(pDev, Y, PKA_N))
	{
		return false;
	}

	PkaModMul(pDev, Y, Y, P256_T0);
	PkaModMul(pDev, X, X, P256_T1);
	PkaModMul(pDev, P256_T1, X, P256_T1);
	PkaModMul(pDev, P256_CONST_A, X, P256_T2);
	PkaModAdd(pDev, P256_T1, P256_T2, P256_T1);
	PkaModAdd(pDev, P256_T1, P256_CONST_B, P256_T1);
	return !s_bPkaFault && PkaEqual(pDev, P256_T0, P256_T1);
}

static bool RandomizePoint(Device *pDev, RngEngine *pRng, const PkaPoint &P)
{
	uint8_t z[P256_BYTES];
	bool valid = false;
	for (uint32_t attempt = 0U; attempt < 100U; attempt++)
	{
		if (pRng == nullptr || !pRng->IsSecure() ||
			pRng->Random(z, sizeof(z)) != CRYPTO_STATUS_OK)
		{
			break;
		}
		if (P256NonzeroFieldElement(z))
		{
			valid = true;
			break;
		}
	}
	if (!valid)
	{
		CryptoSecureWipe(z, sizeof(z));
		return false;
	}

	PkaWriteBe(pDev, P.Z, z, sizeof(z));
	PkaModMul(pDev, P.Z, P.Z, P256_T0);
	PkaModMul(pDev, P.X, P256_T0, P.X);
	PkaModMul(pDev, P256_T0, P.Z, P256_T0);
	PkaModMul(pDev, P.Y, P256_T0, P.Y);
	CryptoSecureWipe(z, sizeof(z));
	return !s_bPkaFault;
}

static bool PointToAffine(Device *pDev, const PkaPoint &P, uint8_t Out[64])
{
	const bool infinity = PointIsInfinity(pDev, P);
	PkaSubImm(pDev, PKA_N, 2, P256_T2);
	PkaModExp(pDev, P.Z, P256_T2, P256_T0);
	PkaModMul(pDev, P256_T0, P256_T0, P256_T1);
	PkaModMul(pDev, P.X, P256_T1, P256_T3);
	PkaModMul(pDev, P256_T1, P256_T0, P256_T1);
	PkaModMul(pDev, P.Y, P256_T1, P256_T4);
	PkaReadBe(pDev, P256_T3, &Out[0], P256_BYTES);
	PkaReadBe(pDev, P256_T4, &Out[32], P256_BYTES);
	return !infinity && !s_bPkaFault;
}

static const uint8_t s_P256Generator[64] = {
	0x6B,0x17,0xD1,0xF2,0xE1,0x2C,0x42,0x47,0xF8,0xBC,0xE6,0xE5,0x63,0xA4,0x40,0xF2,
	0x77,0x03,0x7D,0x81,0x2D,0xEB,0x33,0xA0,0xF4,0xA1,0x39,0x45,0xD8,0x98,0xC2,0x96,
	0x4F,0xE3,0x42,0xE2,0xFE,0x1A,0x7F,0x9B,0x8E,0xE7,0xEB,0x4A,0x7C,0x0F,0x9E,0x16,
	0x2B,0xCE,0x33,0x57,0x6B,0x31,0x5E,0xCE,0xCB,0xB6,0x40,0x68,0x37,0xBF,0x51,0xF5,
};

static bool P256Multiply(Device *pDev, RngEngine *pRng, const uint8_t Point[64],
						 const uint8_t Scalar[32], uint8_t Result[64],
						 bool Randomize)
{
	if (Point == nullptr || Scalar == nullptr || Result == nullptr ||
		!P256ScalarInRange(Scalar))
	{
		return false;
	}
	memset(Result, 0, 64U);
	if (!PkaInit(pDev))
	{
		return false;
	}

	bool ok = false;
	uint8_t regular[P256_BYTES + 1U] = {};
	PkaWriteBe(pDev, POINT_P.X, &Point[0], P256_BYTES);
	PkaWriteBe(pDev, POINT_P.Y, &Point[32], P256_BYTES);
	PkaSetOne(pDev, POINT_P.Z);

	if (!PointValidate(pDev, POINT_P.X, POINT_P.Y))
	{
		goto out;
	}
	if (Randomize && !RandomizePoint(pDev, pRng, POINT_P))
	{
		goto out;
	}

	P256RegularizeScalar(Scalar, regular);
	PointCopy(pDev, POINT_P, POINT_R);

	for (int bit = 255; bit >= 0; bit--)
	{
		PointDouble(pDev, POINT_R, POINT_D);
		PointAdd(pDev, POINT_D, POINT_P, POINT_A);
		PointSelectCopy(pDev, POINT_D, POINT_A,
						 P256RegularBit(regular, (uint32_t)bit), POINT_R);
		if (s_bPkaFault)
		{
			break;
		}
	}

	if (!s_bPkaFault)
	{
		ok = PointToAffine(pDev, POINT_R, Result);
	}
out:
	CryptoSecureWipe(regular, sizeof(regular));
	PkaUninit(pDev);
	if (!ok)
	{
		memset(Result, 0, 64U);
	}
	return ok;
}

static bool CoreInit(Device *pDev)
{
	if (Cc3xxEnable() == false)
	{
		return false;
	}

	CcRegWrite(pDev, REG_HOST_ENDIAN, 0U);
	CcRegWrite(pDev, REG_CRYPTO_CTL, 0U);
	CcRegWrite(pDev, REG_DMA_CLOCK_ENABLE, 1U);
	CcRegWrite(pDev, REG_AHB_HPROT, (1U << 0) | (1U << 1) | (1U << 3));
	CcRegWrite(pDev, REG_AHB_SINGLE, 0U);
	CcRegWrite(pDev, REG_AHB_HNONSEC, 0U);
	CcRegWrite(pDev, REG_DMA_CLOCK_ENABLE, 0U);
	return true;
}

// The per-instance key context is CryptoCc3xx::KeyCtx (declared with the class
// below); the private scalar lives there, single use by default.

static bool Cc3xxAcquire(Device *pDev, Cc3xxIntrf *pIntrf)
{
	uint32_t attempts = __get_IPSR() == 0U ? CC3XX_ACQUIRE_SPINS : 1U;
	while (attempts-- > 0U)
	{
		bool acquired = pIntrf->OpHold(pDev);

		if (acquired)
		{
			return true;
		}
		__asm volatile("" ::: "memory");
	}
	return false;
}

static void Cc3xxRelease(Device *pDev, Cc3xxIntrf *pIntrf)
{
	(void)pIntrf->OpRelease(pDev);
}

static CRYPTO_STATUS EnsureCc3xx(Device *pDev, Cc3xxIntrf *pIntrf)
{
	if (!Cc3xxAcquire(pDev, pIntrf))
	{
		// The operation lock is owned elsewhere: transient, retryable.
		return CRYPTO_STATUS_BUSY;
	}

	if (s_bCcInit == false)
	{
		// The caller is a counted user of the interface. A fault kill or a
		// sibling engine Disable can have dropped wrapper power or the core
		// setup since; re-assert power for the owned count (idempotent) and
		// gate a wrapper that does not come up, then rerun the bring-up.
		if (!Cc3xxEnable())
		{
			Cc3xxRelease(pDev, pIntrf);
			return CRYPTO_STATUS_FAIL;
		}
		s_bCcInit = CoreInit(pDev);
	}
	bool initialized = s_bCcInit;
	Cc3xxRelease(pDev, pIntrf);
	return initialized ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

// @brief	nRF52840 CryptoCell CC310 P-256 ECDH as a KeyAgreeEngine. The
//			private scalar lives in caller key context (single use by default,
//			multi-peer with bKeepKey). The class is declared in the engine
//			header; its methods are defined below, after the anonymous namespace
//			closes, and still reach the internal PKA helpers in this same file.


bool CryptoCc3xx::Init(DeviceIntrf * const pIntrf, RngEngine *pRng)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	Interface(pIntrf);
	vpRng = pRng;
	return Enable();
}

bool CryptoCc3xx::Enable()
{
	Device *pDev = this;
	Cc3xxIntrf *pIntrf = (Cc3xxIntrf *)Interface();
	if (pIntrf == nullptr)
	{
		vbValid = false;
		return false;
	}

	// Count this engine as one interface user, owned through vbIntrfEnabled
	// and nothing else: vbValid can go false on a failed re-Enable while the
	// count is still owned, and the count must be released exactly once. On
	// a first bring-up failure the fresh count is released so the wrapper
	// does not stay powered for a dead engine; on a failed re-Enable the
	// original count is kept for the eventual Disable.
	bool fresh = !vbIntrfEnabled;
	if (fresh)
	{
		pIntrf->Enable();
		vbIntrfEnabled = true;
	}
	bool ok = (EnsureCc3xx(pDev, pIntrf) == CRYPTO_STATUS_OK);
	if (!ok && fresh)
	{
		pIntrf->Disable();
		vbIntrfEnabled = false;
	}
	vbValid = ok;
	return ok;
}

void CryptoCc3xx::Disable()
{
	vbValid = false;
	if (!vbIntrfEnabled)
	{
		return;
	}
	vbIntrfEnabled = false;
	// Force the next user through the core bring-up: if this release powers
	// the wrapper off, the core setup is lost. A sibling engine still
	// counted re-establishes it through EnsureCc3xx at its next operation.
	s_bCcInit = false;
	DeviceIntrf *pIntrf = Interface();
	if (pIntrf != nullptr)
	{
		pIntrf->Disable();
	}
}

void CryptoCc3xx::Reset()
{
	// Power-cycle recovery: release this engine's count (last user powers
	// the wrapper off), then bring it back up through the normal path.
	Disable();
	(void)Enable();
}

CRYPTO_STATUS CryptoCc3xx::KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
								  uint8_t *pPubKey)
{
	Device *pDev = this;
	Cc3xxIntrf *pIntrf = (Cc3xxIntrf *)Interface();
	if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Reset the context and output before any gate, including the lifecycle
	// gate. A disabled engine, a failed bring-up or a busy accelerator must
	// not leave a previous private key usable in a reused context.
	KeyCtx *pk = (KeyCtx *)pKeyCtx;
	KeyReset(pk);
	memset(pPubKey, 0, 64U);

	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS ready = EnsureCc3xx(pDev, pIntrf);
	if (ready != CRYPTO_STATUS_OK)
	{
		return ready;
	}
	if (!Cc3xxAcquire(pDev, pIntrf))
	{
		return CRYPTO_STATUS_BUSY;
	}

	CRYPTO_STATUS status = CRYPTO_STATUS_FAIL;
	if (P256RandomScalar(vpRng, pk->PrivKey) &&
		P256Multiply(pDev, vpRng, s_P256Generator, pk->PrivKey, pPubKey, true))
	{
		pk->bKeyValid = true;
		status = CRYPTO_STATUS_OK;
	}
	else
	{
		KeyReset(pk);
		memset(pPubKey, 0, 64U);
	}
	Cc3xxRelease(pDev, pIntrf);
	return status;
}

CRYPTO_STATUS CryptoCc3xx::Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
								 const uint8_t *pPeerPubKey, uint8_t *pSharedX,
								 bool bKeepKey)
{
	Device *pDev = this;
	Cc3xxIntrf *pIntrf = (Cc3xxIntrf *)Interface();
	if (pKeyCtx == nullptr || pPeerPubKey == nullptr || pSharedX == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	// Disposition on every path: the shared secret is cleared up front and
	// only written on success. A transient contention returns BUSY and
	// preserves the single-use key for retry; any other failure consumes it.
	memset(pSharedX, 0, P256_BYTES);
	KeyCtx *pk = (KeyCtx *)pKeyCtx;
	if (Curve != CRYPTO_CURVE_P256 || !vbValid)
	{
		KeyReset(pk);
		return CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS ready = EnsureCc3xx(pDev, pIntrf);
	if (ready == CRYPTO_STATUS_BUSY)
	{
		return CRYPTO_STATUS_BUSY;
	}
	if (ready != CRYPTO_STATUS_OK)
	{
		KeyReset(pk);
		return CRYPTO_STATUS_FAIL;
	}
	if (!Cc3xxAcquire(pDev, pIntrf))
	{
		return CRYPTO_STATUS_BUSY;
	}
	if (!pk->bKeyValid)
	{
		Cc3xxRelease(pDev, pIntrf);
		KeyReset(pk);
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t point[64];
	const bool ok = P256Multiply(pDev, vpRng, pPeerPubKey, pk->PrivKey, point, true);

	// Wipe the private scalar unless the caller asked to keep it after a success
	// (bKeepKey), for one ephemeral key pair against several peers. A failure
	// always wipes.
	if (!bKeepKey || !ok)
	{
		KeyReset(pk);
	}

	if (ok)
	{
		memcpy(pSharedX, point, P256_BYTES);
	}
	else
	{
		memset(pSharedX, 0, P256_BYTES);
	}
	CryptoSecureWipe(point, sizeof(point));
	Cc3xxRelease(pDev, pIntrf);
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

int CryptoCc3xx::SelfTest()
{
	Device *pDev = this;
	Cc3xxIntrf *pIntrf = (Cc3xxIntrf *)Interface();
	if (!vbValid)
	{
		return -1;
	}
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
	static const uint8_t expected[32] = {
		0xEC,0x02,0x34,0xA3,0x57,0xC8,0xAD,0x05,0x34,0x10,0x10,0xA6,0x0A,0x39,0x7D,0x9B,
		0x99,0x79,0x6B,0x13,0xB4,0xF8,0x66,0xF1,0x86,0x8D,0x34,0xF3,0x73,0xBF,0xA6,0x98,
	};

	if (EnsureCc3xx(pDev, pIntrf) != CRYPTO_STATUS_OK || !Cc3xxAcquire(pDev, pIntrf))
	{
		return -1;
	}
	uint8_t result[64];
	const bool ok = P256Multiply(pDev, nullptr, pub, priv, result, false);
	Cc3xxRelease(pDev, pIntrf);
	const int status = ok && memcmp(result, expected, sizeof(expected)) == 0 ? 0 : -1;
	CryptoSecureWipe(result, sizeof(result));
	return status;
}


CryptoCc3xx *CryptoCc3xxCreate(void *pMem, size_t MemSize, RngEngine *pRng)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoCc3xx) ||
		((uintptr_t)pMem & (alignof(CryptoCc3xx) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoCc3xx *p = new (pMem) CryptoCc3xx();
	if (!p->Init(Cc3xxIntrfInstance(), pRng))
	{
		return nullptr;
	}
	return p;
}
