/**-------------------------------------------------------------------------
@file	i2c_lpc_target.h

@brief	LPC17xx vendor I2C register access for the shared polling master.

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
#ifndef IOSONATA_I2C_LPC_TARGET_H
#define IOSONATA_I2C_LPC_TARGET_H

#include "LPC17xx.h"

typedef LPC_I2C_TypeDef LpcI2CReg_t;

static inline uint32_t LpcI2CControl(LpcI2CReg_t *pReg)
{
	return pReg->I2CONSET;
}

static inline void LpcI2CSetControl(LpcI2CReg_t *pReg, uint32_t Value)
{
	pReg->I2CONSET = Value;
}

static inline void LpcI2CClearControl(LpcI2CReg_t *pReg, uint32_t Value)
{
	pReg->I2CONCLR = Value;
}

static inline uint32_t LpcI2CStatus(LpcI2CReg_t *pReg)
{
	return pReg->I2STAT & 0xF8U;
}

static inline uint8_t LpcI2CReadData(LpcI2CReg_t *pReg)
{
	return (uint8_t)pReg->I2DAT;
}

static inline void LpcI2CWriteData(LpcI2CReg_t *pReg, uint8_t Value)
{
	pReg->I2DAT = Value;
}

static inline void LpcI2CTiming(LpcI2CReg_t *pReg, uint32_t Low, uint32_t High)
{
	pReg->I2SCLL = Low;
	pReg->I2SCLH = High;
}

#endif
