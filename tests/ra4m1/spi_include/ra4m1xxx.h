/**-------------------------------------------------------------------------
@file	ra4m1xxx.h

@brief	Host test replacement of ra4m1xxx.h for spi_ra4m1.cpp.

		The RSPI, PRCR and MSTPCRB registers are objects whose reads and
		writes call the model of spi_test.cpp. Bit definitions are the CMSIS
		ones, copied into spi_bits.h by run_spi_validation.py.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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

#include "spi_bits.h"

typedef int IRQn_Type;

static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __NOP(void) {}

enum {
	SPIREG_SPCR, SPIREG_SSLP, SPIREG_SPPCR, SPIREG_SPSR, SPIREG_SPDR_HA,
	SPIREG_SPDR_BY, SPIREG_SPBR, SPIREG_SPDCR, SPIREG_SPCKD, SPIREG_SSLND,
	SPIREG_SPND, SPIREG_SPCR2, SPIREG_SPCMD,
	SYSREG_PRCR, SYSREG_MSTPCRB,
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

// Same member names as R_SPI0_Type of the CMSIS header
typedef struct {
	TestReg<uint8_t, SPIREG_SPCR> SPCR;
	TestReg<uint8_t, SPIREG_SSLP> SSLP;
	TestReg<uint8_t, SPIREG_SPPCR> SPPCR;
	TestReg<uint8_t, SPIREG_SPSR> SPSR;
	TestReg<uint16_t, SPIREG_SPDR_HA> SPDR_HA;
	TestReg<uint8_t, SPIREG_SPDR_BY> SPDR_BY;
	TestReg<uint8_t, SPIREG_SPBR> SPBR;
	TestReg<uint8_t, SPIREG_SPDCR> SPDCR;
	TestReg<uint8_t, SPIREG_SPCKD> SPCKD;
	TestReg<uint8_t, SPIREG_SSLND> SSLND;
	TestReg<uint8_t, SPIREG_SPND> SPND;
	TestReg<uint8_t, SPIREG_SPCR2> SPCR2;
	TestReg<uint16_t, SPIREG_SPCMD> SPCMD[1];
} R_SPI0_Type;

typedef struct {
	TestReg<uint16_t, SYSREG_PRCR> PRCR;
} R_SYSTEM_Type;

typedef struct {
	TestReg<uint32_t, SYSREG_MSTPCRB> MSTPCRB;
} R_MSTP_Type;

extern R_SPI0_Type g_TestSpi[2];
extern R_SYSTEM_Type g_TestSystem;
extern R_MSTP_Type g_TestMstp;

#define R_SPI0		(&g_TestSpi[0])
#define R_SPI1		(&g_TestSpi[1])
#define R_SYSTEM	(&g_TestSystem)
#define R_MSTP		(&g_TestMstp)

#endif
