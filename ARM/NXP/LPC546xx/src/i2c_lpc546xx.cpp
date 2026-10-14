/**-------------------------------------------------------------------------
@file	i2c_lpc546xx.cpp

@brief	LPC546xx Flexcomm I2C master implementation

		DevNo is the Flexcomm number, 0 to 9. The function clock is the
		12 MHz FRO. Master mode with polling, 7 bit addresses. Slave mode,
		interrupt and DMA configurations fail I2CInit.

		StartTx and StartRx send the START, or the repeated START after an
		earlier phase, with the address and return false when it is not
		acknowledged. TxData and RxData move the data bytes, StopTx and
		StopRx send the STOP. A read ends with a NACK on its last byte, sent
		with the STOP or the next START.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "LPC546xx.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "coredev/i2c.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"

#define LPC546XX_I2C_WAIT_MAX		1000000UL	// STAT reads before a wait gives up
#define LPC546XX_I2C_ADDR_MAX		0x7FU		// 7 bit address
#define LPC546XX_I2C_READ			1U			// Address R/W bit for a read
#define LPC546XX_I2C_DIV_MAX		0x10000U	// CLKDIV DIVVAL + 1
#define LPC546XX_I2C_SCL_CLK_MIN	2U			// MSTSCLLOW and MSTSCLHIGH + 2
#define LPC546XX_I2C_SCL_CLK_MAX	9U

// MSTSTATE values
#define LPC546XX_I2C_MST_IDLE		0
#define LPC546XX_I2C_MST_RXRDY		1
#define LPC546XX_I2C_MST_TXRDY		2
#define LPC546XX_I2C_MST_NACKADR	3
#define LPC546XX_I2C_MST_NACKDAT	4

// Bus errors, cleared by writing 1
#define LPC546XX_I2C_STAT_ERR		(I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK)

#pragma pack(push, 4)

typedef struct __Lpc546xx_I2C_Dev {
	I2C_Type *pReg;					//!< Flexcomm I2C registers
	int DevNo;						//!< Flexcomm number
	uint32_t FClk;					//!< Function clock frequency
	I2CDev_t *pI2cDev;				//!< Generic I2C device data
	bool bActive;					//!< Address acknowledged, transfer in progress
	bool bRxData;					//!< A received byte is waiting in MSTDAT
} Lpc546xxI2CDev_t;

#pragma pack(pop)

static Lpc546xxI2CDev_t s_Lpc546xxI2CDev[] = {
	{ .pReg = I2C0, .DevNo = 0, },
	{ .pReg = I2C1, .DevNo = 1, },
	{ .pReg = I2C2, .DevNo = 2, },
	{ .pReg = I2C3, .DevNo = 3, },
	{ .pReg = I2C4, .DevNo = 4, },
	{ .pReg = I2C5, .DevNo = 5, },
	{ .pReg = I2C6, .DevNo = 6, },
	{ .pReg = I2C7, .DevNo = 7, },
	{ .pReg = I2C8, .DevNo = 8, },
	{ .pReg = I2C9, .DevNo = 9, },
};

static const int s_NbI2CDev = sizeof(s_Lpc546xxI2CDev) / sizeof(Lpc546xxI2CDev_t);

static int Lpc546xxI2CWait(Lpc546xxI2CDev_t * const pDev);
static bool Lpc546xxI2CStart(DevIntrf_t * const pDev, uint32_t DevAddr, uint32_t Rw);
static void Lpc546xxI2CStop(DevIntrf_t * const pDev);

static uint32_t Lpc546xxI2CGetRate(DevIntrf_t * const pDev);
static uint32_t Lpc546xxI2CSetRate(DevIntrf_t * const pDev, uint32_t Rate);
static bool Lpc546xxI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Lpc546xxI2CRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen);
static bool Lpc546xxI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr);
static int Lpc546xxI2CTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen);
static void Lpc546xxI2CDisable(DevIntrf_t * const pDev);
static void Lpc546xxI2CEnable(DevIntrf_t * const pDev);
static void Lpc546xxI2CPowerOff(DevIntrf_t * const pDev);
static void Lpc546xxI2CReset(DevIntrf_t * const pDev);
static void *Lpc546xxI2CGetHandle(DevIntrf_t * const pDev);

// Wait until the master needs service. Returns the master state, or -1 on
// timeout, arbitration loss or START/STOP error.
static int Lpc546xxI2CWait(Lpc546xxI2CDev_t * const pDev)
{
	I2C_Type *reg = pDev->pReg;

	for (uint32_t n = LPC546XX_I2C_WAIT_MAX; n > 0; n--)
	{
		uint32_t stat = reg->STAT;

		if (stat & LPC546XX_I2C_STAT_ERR)
		{
			reg->STAT = stat & LPC546XX_I2C_STAT_ERR;

			return -1;
		}
		if (stat & I2C_STAT_MSTPENDING_MASK)
		{
			return (int)((stat & I2C_STAT_MSTSTATE_MASK) >> I2C_STAT_MSTSTATE_SHIFT);
		}
	}

	return -1;
}

// START or repeated START with the address. On NACK the bus is released.
static bool Lpc546xxI2CStart(DevIntrf_t * const pDev, uint32_t DevAddr, uint32_t Rw)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	I2C_Type *reg = dev->pReg;

	dev->bRxData = false;

	if (DevAddr > LPC546XX_I2C_ADDR_MAX || Lpc546xxI2CWait(dev) < 0)
	{
		dev->bActive = false;

		return false;
	}

	reg->MSTDAT = (DevAddr << 1) | Rw;
	reg->MSTCTL = I2C_MSTCTL_MSTSTART_MASK;

	int state = Lpc546xxI2CWait(dev);

	if (state == (Rw == LPC546XX_I2C_READ ? LPC546XX_I2C_MST_RXRDY : LPC546XX_I2C_MST_TXRDY))
	{
		dev->bActive = true;
		dev->bRxData = Rw == LPC546XX_I2C_READ;

		return true;
	}

	if (state == LPC546XX_I2C_MST_NACKADR || state == LPC546XX_I2C_MST_NACKDAT)
	{
		reg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
		(void)Lpc546xxI2CWait(dev);
	}
	dev->bActive = false;

	return false;
}

// STOP when the master holds the bus. After a read it is preceded by the
// NACK of the last byte.
static void Lpc546xxI2CStop(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	if (Lpc546xxI2CWait(dev) > LPC546XX_I2C_MST_IDLE)
	{
		dev->pReg->MSTCTL = I2C_MSTCTL_MSTSTOP_MASK;
		(void)Lpc546xxI2CWait(dev);
	}

	dev->bActive = false;
	dev->bRxData = false;
}

static uint32_t Lpc546xxI2CGetRate(DevIntrf_t * const pDev)
{
	return ((Lpc546xxI2CDev_t *)pDev->pDevData)->pI2cDev->Cfg.Rate;
}

// Closest SCL rate with the SCL low and high times of the bus mode of the
// requested rate: standard mode up to 100 kHz, fast mode up to 400 kHz,
// fast mode plus above.
static uint32_t Lpc546xxI2CSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	uint32_t fclk = dev->FClk;

	if (Rate == 0 || fclk == 0)
	{
		return 0;
	}

	uint32_t tlow = I2C_SCL_TLOW_FAST_MODE_PLUS_MIN;
	uint32_t thigh = I2C_SCL_THIGH_FAST_MODE_PLUS_MIN;

	if (Rate <= I2C_SCL_STD_MODE_MAX_SPEED * 1000U)
	{
		tlow = I2C_SCL_TLOW_STD_MODE_MIN;
		thigh = I2C_SCL_THIGH_STD_MODE_MIN;
	}
	else if (Rate <= I2C_SCL_FAST_MODE_MAX_SPEED * 1000U)
	{
		tlow = I2C_SCL_TLOW_FAST_MODE_MIN;
		thigh = I2C_SCL_THIGH_FAST_MODE_MIN;
	}

	uint32_t bestdiv = 0;
	uint32_t bestlow = LPC546XX_I2C_SCL_CLK_MAX;
	uint32_t besthigh = LPC546XX_I2C_SCL_CLK_MAX;
	uint32_t bestrate = 0;
	uint32_t besterr = UINT32_MAX;

	// SCL period is 4 to 18 function clocks after the divider
	uint64_t divmin = (uint64_t)fclk / ((uint64_t)Rate * 2U * LPC546XX_I2C_SCL_CLK_MAX);
	uint64_t divmax = (uint64_t)fclk / ((uint64_t)Rate * 2U * LPC546XX_I2C_SCL_CLK_MIN) + 1U;

	if (divmin < 1U)
	{
		divmin = 1U;
	}
	if (divmax > LPC546XX_I2C_DIV_MAX)
	{
		divmax = LPC546XX_I2C_DIV_MAX;
	}

	for (uint32_t div = (uint32_t)divmin; div <= (uint32_t)divmax; div++)
	{
		uint32_t f = fclk / div;
		uint32_t clocks = (f + Rate / 2U) / Rate;
		uint32_t lmin = (uint32_t)(((uint64_t)tlow * f + 999999999U) / 1000000000U);
		uint32_t hmin = (uint32_t)(((uint64_t)thigh * f + 999999999U) / 1000000000U);

		if (lmin < LPC546XX_I2C_SCL_CLK_MIN)
		{
			lmin = LPC546XX_I2C_SCL_CLK_MIN;
		}
		if (hmin < LPC546XX_I2C_SCL_CLK_MIN)
		{
			hmin = LPC546XX_I2C_SCL_CLK_MIN;
		}
		if (lmin > LPC546XX_I2C_SCL_CLK_MAX || hmin > LPC546XX_I2C_SCL_CLK_MAX)
		{
			continue;
		}
		if (clocks < lmin + hmin)
		{
			clocks = lmin + hmin;
		}
		if (clocks > 2U * LPC546XX_I2C_SCL_CLK_MAX)
		{
			clocks = 2U * LPC546XX_I2C_SCL_CLK_MAX;
		}

		// Extra clocks go to the low time first
		uint32_t low = clocks - hmin;

		if (low > LPC546XX_I2C_SCL_CLK_MAX)
		{
			low = LPC546XX_I2C_SCL_CLK_MAX;
		}

		uint32_t high = clocks - low;
		uint32_t rate = fclk / (div * clocks);
		uint32_t err = rate > Rate ? rate - Rate : Rate - rate;

		if (err < besterr)
		{
			besterr = err;
			bestdiv = div;
			bestlow = low;
			besthigh = high;
			bestrate = rate;
		}
	}

	if (bestdiv == 0)
	{
		return 0;
	}

	dev->pReg->CLKDIV = I2C_CLKDIV_DIVVAL(bestdiv - 1U);
	dev->pReg->MSTTIME = I2C_MSTTIME_MSTSCLLOW(bestlow - LPC546XX_I2C_SCL_CLK_MIN) |
						 I2C_MSTTIME_MSTSCLHIGH(besthigh - LPC546XX_I2C_SCL_CLK_MIN);

	dev->pI2cDev->Cfg.Rate = bestrate;

	return bestrate;
}

static bool Lpc546xxI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return Lpc546xxI2CStart(pDev, DevAddr, LPC546XX_I2C_READ);
}

// Bytes received are acknowledged when the next one is requested. The last
// one is left for the NACK sent with StopRx or the next start.
static int Lpc546xxI2CRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	I2C_Type *reg = dev->pReg;
	int cnt = 0;

	if (dev->bActive == false || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	while (cnt < BuffLen)
	{
		if (dev->bRxData == false)
		{
			reg->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;
		}

		if (Lpc546xxI2CWait(dev) != LPC546XX_I2C_MST_RXRDY)
		{
			dev->bRxData = false;
			break;
		}

		pBuff[cnt++] = (uint8_t)reg->MSTDAT;
		dev->bRxData = false;
	}

	return cnt;
}

static bool Lpc546xxI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return Lpc546xxI2CStart(pDev, DevAddr, 0);
}

// Returns the number of bytes acknowledged
static int Lpc546xxI2CTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;
	I2C_Type *reg = dev->pReg;
	int cnt = 0;

	if (dev->bActive == false || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	int state = Lpc546xxI2CWait(dev);

	while (cnt < DataLen && state == LPC546XX_I2C_MST_TXRDY)
	{
		reg->MSTDAT = pData[cnt++];
		reg->MSTCTL = I2C_MSTCTL_MSTCONTINUE_MASK;
		state = Lpc546xxI2CWait(dev);
	}

	// The last byte written was not acknowledged
	if (state != LPC546XX_I2C_MST_TXRDY && cnt > 0)
	{
		cnt--;
	}

	return cnt;
}

static void Lpc546xxI2CDisable(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	dev->pReg->CFG = 0;
	dev->bActive = false;
	dev->bRxData = false;
	Lpc546xxFlexcommClock(dev->DevNo, false);
}

static void Lpc546xxI2CEnable(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	Lpc546xxFlexcommClock(dev->DevNo, true);
	IOPinCfg(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
	dev->pReg->STAT = LPC546XX_I2C_STAT_ERR;
	dev->pReg->CFG = I2C_CFG_MSTEN_MASK;
}

static void Lpc546xxI2CPowerOff(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	Lpc546xxI2CDisable(pDev);
	IOPinDis(dev->pI2cDev->Cfg.pIOPinMap, dev->pI2cDev->Cfg.NbIOPins);
}

// Bus recovery with clock pulses on SCL, then Enable restores the pins
static void Lpc546xxI2CReset(DevIntrf_t * const pDev)
{
	Lpc546xxI2CDev_t *dev = (Lpc546xxI2CDev_t *)pDev->pDevData;

	I2CBusReset(dev->pI2cDev);
}

static void *Lpc546xxI2CGetHandle(DevIntrf_t * const pDev)
{
	return ((Lpc546xxI2CDev_t *)pDev->pDevData)->pI2cDev;
}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL || pCfgData->DevNo < 0 || pCfgData->DevNo >= s_NbI2CDev ||
		pCfgData->pIOPinMap == NULL || pCfgData->NbIOPins < 2 || pCfgData->Rate == 0)
	{
		return false;
	}

	// Polling master, 7 bit address
	if (pCfgData->Mode != I2CMODE_MASTER || pCfgData->Type != I2CTYPE_STANDARD ||
		pCfgData->AddrType != I2CADDR_TYPE_NORMAL || pCfgData->bDmaEn || pCfgData->bIntEn)
	{
		return false;
	}

	Lpc546xxI2CDev_t *dev = &s_Lpc546xxI2CDev[pCfgData->DevNo];

	// Bus clock on, Flexcomm reset with the I2C function selected. Fails if
	// the Flexcomm is locked to the USART or SPI.
	dev->FClk = Lpc546xxFlexcommSelect(dev->DevNo, LPC546XX_FLEXCOMM_I2C);
	if (dev->FClk == 0)
	{
		return false;
	}

	NVIC_DisableIRQ(Lpc546xxFlexcommIrqNo(dev->DevNo));
	SharedIntrfSetIrqHandler(dev->DevNo, NULL, NULL);

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));
	if (pDev->Cfg.MaxRetry <= 0)
	{
		pDev->Cfg.MaxRetry = I2C_MAX_RETRY;
	}

	dev->pI2cDev = pDev;
	dev->bActive = false;
	dev->bRxData = false;

	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.bIntEn = false;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.GetHandle = Lpc546xxI2CGetHandle;
	pDev->DevIntrf.Disable = Lpc546xxI2CDisable;
	pDev->DevIntrf.Enable = Lpc546xxI2CEnable;
	pDev->DevIntrf.PowerOff = Lpc546xxI2CPowerOff;
	pDev->DevIntrf.Reset = Lpc546xxI2CReset;
	pDev->DevIntrf.GetRate = Lpc546xxI2CGetRate;
	pDev->DevIntrf.SetRate = Lpc546xxI2CSetRate;
	pDev->DevIntrf.StartRx = Lpc546xxI2CStartRx;
	pDev->DevIntrf.RxData = Lpc546xxI2CRxData;
	pDev->DevIntrf.StopRx = Lpc546xxI2CStop;
	pDev->DevIntrf.StartTx = Lpc546xxI2CStartTx;
	pDev->DevIntrf.TxData = Lpc546xxI2CTxData;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.StopTx = Lpc546xxI2CStop;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	if (Lpc546xxI2CSetRate(&pDev->DevIntrf, pCfgData->Rate) == 0)
	{
		Lpc546xxFlexcommClock(dev->DevNo, false);

		return false;
	}

	dev->pReg->CFG = I2C_CFG_MSTEN_MASK;

	return true;
}
