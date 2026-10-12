/**-------------------------------------------------------------------------
@file	i2c_lpc11uxx.cpp

@brief	LPC11Uxx I2C clock, reset and pin validation for the shared polling master.

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
#include "i2c_lpc_master.h"
#include "coredev/interrupt.h"

#define LPC11_I2C_CLOCK	(1U << 5)
#define LPC11_I2C_RESET	(1U << 1)

static LpcI2CMaster_t s_I2C = {
	.pReg = LPC_I2C,
	.MaxRate = 400000U,
};

void LpcI2CMasterClock(LpcI2CMaster_t *pData, bool Enable)
{
	(void)pData;
	uint32_t state = DisableInterrupt();

	if (Enable)
	{
		LPC_SYSCON->SYSAHBCLKCTRL |= LPC11_I2C_CLOCK;
	}
	else
	{
		LPC_SYSCON->SYSAHBCLKCTRL &= ~LPC11_I2C_CLOCK;
	}
	(void)LPC_SYSCON->SYSAHBCLKCTRL;
	EnableInterrupt(state);
}

void LpcI2CMasterReset(LpcI2CMaster_t *pData)
{
	(void)pData;
	uint32_t state = DisableInterrupt();

	LPC_SYSCON->PRESETCTRL &= ~LPC11_I2C_RESET;
	(void)LPC_SYSCON->PRESETCTRL;
	LPC_SYSCON->PRESETCTRL |= LPC11_I2C_RESET;
	(void)LPC_SYSCON->PRESETCTRL;
	NVIC_DisableIRQ(I2C_IRQn);
	NVIC_ClearPendingIRQ(I2C_IRQn);
	EnableInterrupt(state);
}

bool I2CInit(I2CDev_t *pDev, const I2CCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 ||
		pCfg->Type != I2CTYPE_STANDARD || pCfg->Mode != I2CMODE_MASTER ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL || pCfg->bIntEn || pCfg->bDmaEn ||
		pCfg->Rate == 0 || pCfg->MaxRetry < 0 || SystemCoreClock == 0 ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 2)
	{
		return false;
	}
	// Dedicated open-drain SDA=P0.5, SCL=P0.4, function 1.
	const IOPinCfg_t *pins = pCfg->pIOPinMap;

	if (pins[0].PortNo != 0 || pins[0].PinNo != 5 || pins[0].PinOp != IOPINOP_FUNC1 ||
		pins[1].PortNo != 0 || pins[1].PinNo != 4 || pins[1].PinOp != IOPINOP_FUNC1)
	{
		return false;
	}
	return LpcI2CMasterInit(&s_I2C, pDev, pCfg, SystemCoreClock);
}
