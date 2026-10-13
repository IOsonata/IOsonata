/**-------------------------------------------------------------------------
@file	ra4m1xxx.h

@brief	Host test replacement of ra4m1xxx.h for i2c_ra4m1.cpp.

		The IIC, PRCR and MSTPCRB registers are objects whose reads and
		writes call the model of i2c_test.cpp. Bit definitions of PRCR are the
		CMSIS ones, copied into sys_bits.h by run_i2c_validation.py.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <stdbool.h>

#include "sys_bits.h"

typedef int IRQn_Type;

static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __NOP(void) {}

enum {
	IICREG_ICCR1, IICREG_ICCR2, IICREG_ICMR1, IICREG_ICMR2, IICREG_ICMR3,
	IICREG_ICFER, IICREG_ICSER, IICREG_ICIER, IICREG_ICSR1, IICREG_ICSR2,
	IICREG_SARL, IICREG_SARU, IICREG_ICBRL, IICREG_ICBRH, IICREG_ICDRT,
	IICREG_ICDRR,
	SYSREG_PRCR, SYSREG_MSTPCRB, SYSREG_SCKDIVCR,
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
	TestReg<uint8_t, IICREG_SARL> L;
	TestReg<uint8_t, IICREG_SARU> U;
} TestIicSar_t;

// Same member names and order as R_IIC0_Type of the CMSIS header
typedef struct {
	TestReg<uint8_t, IICREG_ICCR1> ICCR1;
	TestReg<uint8_t, IICREG_ICCR2> ICCR2;
	TestReg<uint8_t, IICREG_ICMR1> ICMR1;
	TestReg<uint8_t, IICREG_ICMR2> ICMR2;
	TestReg<uint8_t, IICREG_ICMR3> ICMR3;
	TestReg<uint8_t, IICREG_ICFER> ICFER;
	TestReg<uint8_t, IICREG_ICSER> ICSER;
	TestReg<uint8_t, IICREG_ICIER> ICIER;
	TestReg<uint8_t, IICREG_ICSR1> ICSR1;
	TestReg<uint8_t, IICREG_ICSR2> ICSR2;
	TestIicSar_t SAR[3];
	TestReg<uint8_t, IICREG_ICBRL> ICBRL;
	TestReg<uint8_t, IICREG_ICBRH> ICBRH;
	TestReg<uint8_t, IICREG_ICDRT> ICDRT;
	TestReg<uint8_t, IICREG_ICDRR> ICDRR;
} R_IIC0_Type;

typedef struct {
	TestReg<uint16_t, SYSREG_PRCR> PRCR;
	TestReg<uint32_t, SYSREG_SCKDIVCR> SCKDIVCR;
} R_SYSTEM_Type;

typedef struct {
	TestReg<uint32_t, SYSREG_MSTPCRB> MSTPCRB;
} R_MSTP_Type;

extern R_IIC0_Type g_TestIic[2];
extern R_SYSTEM_Type g_TestSystem;
extern R_MSTP_Type g_TestMstp;

#define R_IIC0		(&g_TestIic[0])
#define R_IIC1		(&g_TestIic[1])
#define R_SYSTEM	(&g_TestSystem)
#define R_MSTP		(&g_TestMstp)

#endif
