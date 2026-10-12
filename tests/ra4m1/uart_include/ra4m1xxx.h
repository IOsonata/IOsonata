/**-------------------------------------------------------------------------
@file	ra4m1xxx.h

@brief	Host test replacement of ra4m1xxx.h for uart_ra4m1.cpp.

		The SCI, DMAC, ICU.DELSR, PRCR, MSTPCRA and MSTPCRB registers are
		objects whose reads and writes call the model of uart_test.cpp. Bit
		definitions are the CMSIS ones, copied from R7FA4M1AB.h into
		sci_bits.h by run_uart_validation.py.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __RA4M1XXX_H__
#define __RA4M1XXX_H__

#include <stdint.h>

#include "sci_bits.h"

typedef int IRQn_Type;

#define __NVIC_PRIO_BITS	4

static inline void __DSB(void) {}
static inline void __NOP(void) {}

// Register ids of the model
enum {
	SCIREG_SMR, SCIREG_BRR, SCIREG_SCR, SCIREG_TDR, SCIREG_SSR, SCIREG_RDR,
	SCIREG_SCMR, SCIREG_SEMR, SCIREG_SNFR, SCIREG_SIMR1, SCIREG_SPMR,
	SCIREG_FTDRL, SCIREG_FRDRHL, SCIREG_MDDR, SCIREG_DCCR, SCIREG_FCR,
	SCIREG_FDR, SCIREG_SPTR,
	SYSREG_PRCR, SYSREG_MSTPCRA, SYSREG_MSTPCRB,
	DMAREG_SAR, DMAREG_DAR, DMAREG_CRA, DMAREG_CRB, DMAREG_TMD, DMAREG_INT, DMAREG_AMD,
	DMAREG_OFR, DMAREG_CNT, DMAREG_STS, DMAREG_DMAST,
	ICUREG_DELSR,
};

// pReg is the register object, the model finds its unit from the address
uint32_t TestRegRead(const void *pReg, int Id);
void TestRegWrite(const void *pReg, int Id, uint32_t Val);

template <typename T, int Id> struct TestReg {
	operator T() const { return (T)TestRegRead(this, Id); }
	TestReg &operator=(uint32_t Val) { TestRegWrite(this, Id, (T)Val); return *this; }
	TestReg &operator|=(uint32_t Val) { TestRegWrite(this, Id, (T)(TestRegRead(this, Id) | Val)); return *this; }
	TestReg &operator&=(uint32_t Val) { TestRegWrite(this, Id, (T)(TestRegRead(this, Id) & Val)); return *this; }
};

typedef struct {
	TestReg<uint8_t, SCIREG_SMR> SMR;
	TestReg<uint8_t, SCIREG_BRR> BRR;
	TestReg<uint8_t, SCIREG_SCR> SCR;
	TestReg<uint8_t, SCIREG_TDR> TDR;
	TestReg<uint8_t, SCIREG_SSR> SSR;
	TestReg<uint8_t, SCIREG_SSR> SSR_FIFO;
	TestReg<uint8_t, SCIREG_RDR> RDR;
	TestReg<uint8_t, SCIREG_SCMR> SCMR;
	TestReg<uint8_t, SCIREG_SEMR> SEMR;
	TestReg<uint8_t, SCIREG_SNFR> SNFR;
	TestReg<uint8_t, SCIREG_SIMR1> SIMR1;
	TestReg<uint8_t, SCIREG_SPMR> SPMR;
	TestReg<uint8_t, SCIREG_FTDRL> FTDRL;
	TestReg<uint16_t, SCIREG_FRDRHL> FRDRHL;
	TestReg<uint8_t, SCIREG_MDDR> MDDR;
	TestReg<uint8_t, SCIREG_DCCR> DCCR;
	TestReg<uint16_t, SCIREG_FCR> FCR;
	TestReg<uint16_t, SCIREG_FDR> FDR;
	TestReg<uint8_t, SCIREG_SPTR> SPTR;
} R_SCI0_Type;

typedef struct {
	TestReg<uint16_t, SYSREG_PRCR> PRCR;
} R_SYSTEM_Type;

typedef struct {
	TestReg<uint32_t, SYSREG_MSTPCRA> MSTPCRA;
	TestReg<uint32_t, SYSREG_MSTPCRB> MSTPCRB;
} R_MSTP_Type;

typedef struct {
	TestReg<uint32_t, DMAREG_SAR> DMSAR;
	TestReg<uint32_t, DMAREG_DAR> DMDAR;
	TestReg<uint32_t, DMAREG_CRA> DMCRA;
	TestReg<uint32_t, DMAREG_CRB> DMCRB;
	TestReg<uint16_t, DMAREG_TMD> DMTMD;
	TestReg<uint8_t, DMAREG_INT> DMINT;
	TestReg<uint16_t, DMAREG_AMD> DMAMD;
	TestReg<uint32_t, DMAREG_OFR> DMOFR;
	TestReg<uint8_t, DMAREG_CNT> DMCNT;
	TestReg<uint8_t, DMAREG_STS> DMSTS;
} R_DMAC0_Type;

typedef struct {
	TestReg<uint8_t, DMAREG_DMAST> DMAST;
} R_DMA_Type;

typedef struct {
	TestReg<uint32_t, ICUREG_DELSR> DELSR[4];
} R_ICU_Type;

extern R_SCI0_Type g_TestSci[4];
extern R_SYSTEM_Type g_TestSystem;
extern R_MSTP_Type g_TestMstp;
extern R_DMAC0_Type g_TestDmac[4];
extern R_DMA_Type g_TestDma;
extern R_ICU_Type g_TestIcu;

#define R_SCI0		(&g_TestSci[0])
#define R_SCI1		(&g_TestSci[1])
#define R_SCI2		(&g_TestSci[2])
#define R_SCI9		(&g_TestSci[3])
#define R_SYSTEM	(&g_TestSystem)
#define R_MSTP		(&g_TestMstp)
#define R_DMAC0		(&g_TestDmac[0])
#define R_DMAC1		(&g_TestDmac[1])
#define R_DMAC2		(&g_TestDmac[2])
#define R_DMAC3		(&g_TestDmac[3])
#define R_DMA		(&g_TestDma)
#define R_ICU		(&g_TestIcu)

extern "C" uint32_t SystemCoreClock;
extern "C" uint32_t SystemPeriphClockGet(int Idx);

#endif
