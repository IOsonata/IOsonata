/*--------------------------------------------------------------------------
File   : i2c_lpc17xx.c

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : LPC17xx clocks and pins for the shared LPC I2C polling master

Copyright (c) 2011, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include "i2c_lpc_master.h"
#include "coredev/interrupt.h"

#define LPC17_I2C_COUNT	3

static LpcI2CMaster_t s_I2C[LPC17_I2C_COUNT] = {
	{ .pReg = LPC_I2C0, .MaxRate = 400000U, },
	{ .pReg = LPC_I2C1, .MaxRate = 400000U, },
	{ .pReg = LPC_I2C2, .MaxRate = 400000U, },
};

static const uint32_t s_ClockMask[LPC17_I2C_COUNT] = {
	1U << 7, 1U << 19, 1U << 26,
};

static const IRQn_Type s_Irq[LPC17_I2C_COUNT] = {
	I2C0_IRQn, I2C1_IRQn, I2C2_IRQn,
};

void LpcI2CMasterClock(LpcI2CMaster_t *pData, bool Enable)
{
	int devno = (int)(pData - s_I2C);
	uint32_t state = DisableInterrupt();

	if (Enable)
	{
		LPC_SC->PCONP |= s_ClockMask[devno];
		// Keep the previous LPC17xx driver's PCLK = CCLK/4 selection.
		if (devno == 0)
		{
			LPC_SC->PCLKSEL0 &= ~(3U << 14);
			// Standard/fast mode: slew control and glitch filtering enabled.
			LPC_PINCON->I2CPADCFG &= ~15U;
		}
		else
		{
			uint32_t shift = devno == 1 ? 6U : 20U;

			LPC_SC->PCLKSEL1 &= ~(3U << shift);
		}
	}
	else
	{
		LPC_SC->PCONP &= ~s_ClockMask[devno];
	}
	(void)LPC_SC->PCONP;
	EnableInterrupt(state);
}

void LpcI2CMasterReset(LpcI2CMaster_t *pData)
{
	int devno = (int)(pData - s_I2C);

	NVIC_DisableIRQ(s_Irq[devno]);
	// Clearing I2EN resets the bus state and forces STO to zero (UM10360).
	LpcI2CClearControl(pData->pReg, LPC_I2C_CLEAR);
	(void)LpcI2CControl(pData->pReg);
	pData->pReg->MMCTRL = 0;
	NVIC_ClearPendingIRQ(s_Irq[devno]);
}

static bool Lpc17I2CPinValid(int DevNo, int Index, const IOPinCfg_t *pPin)
{
	if (pPin->PortNo != 0 ||
		(pPin->Res != IOPINRES_NONE && pPin->Res != IOPINRES_PULLUP))
	{
		return false;
	}
	if (DevNo == 0)
	{
		// I2C0 has dedicated open-drain pads.
		return pPin->PinNo == 27 + Index && pPin->PinOp == IOPINOP_FUNC1;
	}
	if (pPin->Type != IOPINTYPE_OPENDRAIN)
	{
		return false;
	}
	if (DevNo == 1)
	{
		return (pPin->PinNo == Index || pPin->PinNo == 19 + Index) &&
			pPin->PinOp == IOPINOP_FUNC3;
	}
	return pPin->PinNo == 10 + Index && pPin->PinOp == IOPINOP_FUNC2;
}

bool I2CInit(I2CDev_t *pDev, const I2CCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= LPC17_I2C_COUNT ||
		pCfg->Type != I2CTYPE_STANDARD || pCfg->Mode != I2CMODE_MASTER ||
		pCfg->AddrType != I2CADDR_TYPE_NORMAL || pCfg->bIntEn || pCfg->bDmaEn ||
		pCfg->Rate == 0 || pCfg->MaxRetry < 0 || SystemCoreClock < 4U ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 2)
	{
		return false;
	}
	for (int i = 0; i < 2; i++)
	{
		if (!Lpc17I2CPinValid(pCfg->DevNo, i, &pCfg->pIOPinMap[i]))
		{
			return false;
		}
	}
	uint32_t state = DisableInterrupt();

	for (int i = 0; i < LPC17_I2C_COUNT; i++)
	{
		if (i != pCfg->DevNo && s_I2C[i].pDev == pDev)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	LpcI2CMaster_t *data = &s_I2C[pCfg->DevNo];

	bool result = LpcI2CMasterInit(data, pDev, pCfg, SystemCoreClock / 4U);

	EnableInterrupt(state);
	return result;
}
