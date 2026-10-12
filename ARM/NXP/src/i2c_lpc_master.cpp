/**-------------------------------------------------------------------------
@file	i2c_lpc_master.cpp

@brief	Shared LPC state-machine I2C polling master; 7-bit addresses, up to 400 kHz.

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
#include <stddef.h>
#include "i2c_lpc_master.h"
#include "coredev/interrupt.h"

static void LpcI2CStop(DevIntrf_t *pIntrf);

static LpcI2CMaster_t *LpcI2CData(DevIntrf_t *pIntrf)
{
	return (LpcI2CMaster_t *)pIntrf->pDevData;
}

static int LpcI2CWait(LpcI2CMaster_t *pData)
{
	// Allow a byte at the slowest divider as well as bounded clock stretching.
	uint32_t limit = LPC_I2C_WAIT_MAX + 32U * (pData->Low + pData->High);

	for (uint32_t n = limit; n > 0; n--)
	{
		if (LpcI2CControl(pData->pReg) & LPC_I2C_SI)
		{
			return (int)LpcI2CStatus(pData->pReg);
		}
	}
	return -1;
}

// STOP completes when hardware clears STO; it does not raise SI.
static void LpcI2CRelease(LpcI2CMaster_t *pData)
{
	if (pData->bOwned)
	{
		LpcI2CClearControl(pData->pReg, LPC_I2C_STA | LPC_I2C_AA);
		LpcI2CSetControl(pData->pReg, LPC_I2C_STO);
		LpcI2CClearControl(pData->pReg, LPC_I2C_SI);
		uint32_t n = LPC_I2C_WAIT_MAX + 32U * (pData->Low + pData->High);

		while ((LpcI2CControl(pData->pReg) & LPC_I2C_STO) && n > 0)
		{
			n--;
		}
		if (n == 0)
		{
			LpcI2CMasterReset(pData);
			LpcI2CTiming(pData->pReg, pData->Low, pData->High);
			LpcI2CSetControl(pData->pReg, LPC_I2C_EN);
		}
	}
	pData->bOwned = false;
}

static void LpcI2CFail(LpcI2CMaster_t *pData, int Status)
{
	if (Status == I2CSTATUS_ARB_LOST)
	{
		// The winning master owns the bus. Never generate STOP here.
		pData->bOwned = false;
		LpcI2CClearControl(pData->pReg, LPC_I2C_STA | LPC_I2C_AA | LPC_I2C_SI);
	}
	else if (pData->bOwned)
	{
		LpcI2CRelease(pData);
	}
	else
	{
		// Cancel a START that could not finish, without clocking the bus.
		LpcI2CMasterReset(pData);
		LpcI2CTiming(pData->pReg, pData->Low, pData->High);
		LpcI2CSetControl(pData->pReg, LPC_I2C_EN);
	}
	// Keep a failed command phase failed until its matching Stop call.
	pData->bError = pData->bTransfer;
}

static bool LpcI2CStart(DevIntrf_t *pIntrf, uint32_t Address, bool Read)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);

	if (!data->bEnabled || data->bError || Address > 0x7FU)
	{
		if (data->bEnabled && data->bTransfer)
		{
			LpcI2CFail(data, -1);
		}
		return false;
	}
	LpcI2CSetControl(data->pReg, LPC_I2C_STA);
	LpcI2CClearControl(data->pReg, LPC_I2C_SI);
	int status = LpcI2CWait(data);

	if (status != I2CSTATUS_START_COND && status != I2CSTATUS_RESTART_COND)
	{
		LpcI2CFail(data, status);
		return false;
	}
	data->bOwned = true;
	LpcI2CWriteData(data->pReg, (uint8_t)((Address << 1) | (Read ? 1U : 0U)));
	LpcI2CClearControl(data->pReg, LPC_I2C_STA | LPC_I2C_SI);
	status = LpcI2CWait(data);
	if (status != (Read ? I2CSTATUS_SLAR_ACK : I2CSTATUS_SLAW_ACK))
	{
		LpcI2CFail(data, status);
		return false;
	}
	data->bTransfer = true;
	data->bRead = Read;
	data->bReadDone = false;
	return true;
}

static bool LpcI2CStartRx(DevIntrf_t *pIntrf, uint32_t Address)
{
	return LpcI2CStart(pIntrf, Address, true);
}

static bool LpcI2CStartTx(DevIntrf_t *pIntrf, uint32_t Address)
{
	return LpcI2CStart(pIntrf, Address, false);
}

static int LpcI2CTxData(DevIntrf_t *pIntrf, const uint8_t *pBuffer, int Length)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);
	int count = 0;

	if (!data->bEnabled || !data->bTransfer || data->bError || data->bRead || pBuffer == NULL || Length <= 0)
	{
		return 0;
	}
	while (count < Length)
	{
		LpcI2CWriteData(data->pReg, pBuffer[count]);
		LpcI2CClearControl(data->pReg, LPC_I2C_SI);
		int status = LpcI2CWait(data);

		if (status != I2CSTATUS_M_TXDAT_ACK)
		{
			LpcI2CFail(data, status);
			break;
		}
		count++;
	}
	return count;
}

static int LpcI2CRxData(DevIntrf_t *pIntrf, uint8_t *pBuffer, int Length)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);
	int count = 0;

	if (!data->bEnabled || !data->bTransfer || data->bError || !data->bRead ||
		data->bReadDone || pBuffer == NULL || Length <= 0)
	{
		return 0;
	}
	while (count < Length)
	{
		bool last = count == Length - 1;

		if (last)
		{
			LpcI2CClearControl(data->pReg, LPC_I2C_AA);
		}
		else
		{
			LpcI2CSetControl(data->pReg, LPC_I2C_AA);
		}
		LpcI2CClearControl(data->pReg, LPC_I2C_SI);
		int status = LpcI2CWait(data);

		if (status != (last ? I2CSTATUS_RXDATA_NACK : I2CSTATUS_RXDATA_ACK))
		{
			LpcI2CFail(data, status);
			break;
		}
		pBuffer[count] = LpcI2CReadData(data->pReg);
		count++;
		data->bReadDone = last;
	}
	return count;
}

static void LpcI2CStop(DevIntrf_t *pIntrf)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);

	if (data->bEnabled)
	{
		LpcI2CRelease(data);
	}
	data->bTransfer = false;
	data->bReadDone = false;
	data->bError = false;
}

static uint32_t LpcI2CGetRate(DevIntrf_t *pIntrf)
{
	return LpcI2CData(pIntrf)->pDev->Cfg.Rate;
}

static uint32_t LpcI2CSetRate(DevIntrf_t *pIntrf, uint32_t Rate)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);
	uint32_t clock = data->Clock;

	if (Rate == 0 || data->bTransfer || clock == 0)
	{
		return LpcI2CGetRate(pIntrf);
	}
	if (Rate > data->MaxRate)
	{
		Rate = data->MaxRate;
	}
	uint32_t lowNs = Rate <= 100000U ? 4700U : 1300U;
	uint32_t highNs = Rate <= 100000U ? 4000U : 600U;
	uint32_t low = (uint32_t)(((uint64_t)clock * lowNs + 999999999U) / 1000000000U);
	uint32_t high = (uint32_t)(((uint64_t)clock * highNs + 999999999U) / 1000000000U);

	if (low < 4U)
	{
		low = 4U;
	}
	if (high < 4U)
	{
		high = 4U;
	}
	uint32_t minimum = low + high;
	uint32_t modeMax = Rate <= 100000U ? 100000U : data->MaxRate;
	uint32_t modeMin = (clock + modeMax - 1U) / modeMax;

	if (minimum < modeMin)
	{
		minimum = modeMin;
	}
	uint32_t divisor = clock / Rate;

	if (divisor < minimum)
	{
		divisor = minimum;
	}
	if (divisor > 131070U)
	{
		divisor = 131070U;
	}
	// Compare actual Hz errors of the adjacent divisors without truncation.
	if (divisor < 131070U && (uint64_t)Rate * divisor <= clock)
	{
		uint64_t error = clock - (uint64_t)Rate * divisor;
		uint64_t next = (uint64_t)Rate * (divisor + 1U) - clock;

		if (next * divisor < error * (divisor + 1U))
		{
			divisor++;
		}
	}
	uint32_t desiredLow = divisor / 2U;

	if (desiredLow < low)
	{
		desiredLow = low;
	}
	if (desiredLow > divisor - high)
	{
		desiredLow = divisor - high;
	}
	if (desiredLow > 65535U)
	{
		desiredLow = 65535U;
	}
	if (divisor - desiredLow > 65535U)
	{
		desiredLow = divisor - 65535U;
	}
	data->Low = desiredLow;
	data->High = divisor - desiredLow;
	if (data->bEnabled)
	{
		LpcI2CTiming(data->pReg, data->Low, data->High);
	}
	data->pDev->Cfg.Rate = clock / divisor;
	return data->pDev->Cfg.Rate;
}

static void LpcI2CDisable(DevIntrf_t *pIntrf)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);

	if (data->bEnabled)
	{
		LpcI2CStop(pIntrf);
		LpcI2CClearControl(data->pReg, LPC_I2C_CLEAR);
		data->bEnabled = false;
		LpcI2CMasterClock(data, false);
	}
}

static void LpcI2CEnable(DevIntrf_t *pIntrf)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);

	LpcI2CMasterClock(data, true);
	IOPinCfg(data->pDev->Cfg.pIOPinMap, data->pDev->Cfg.NbIOPins);
	LpcI2CMasterReset(data);
	LpcI2CTiming(data->pReg, data->Low, data->High);
	LpcI2CSetControl(data->pReg, LPC_I2C_EN);
	data->bEnabled = true;
	data->bTransfer = false;
	data->bOwned = false;
	data->bError = false;
}

static void LpcI2CPowerOff(DevIntrf_t *pIntrf)
{
	LpcI2CDisable(pIntrf);
	// PowerOff is terminal for existing enable references, as for LPC SSP.
	pIntrf->EnCnt = 0;
}

static void LpcI2CReset(DevIntrf_t *pIntrf)
{
	LpcI2CMaster_t *data = LpcI2CData(pIntrf);

	if (data->bEnabled)
	{
		LpcI2CStop(pIntrf);
		LpcI2CEnable(pIntrf);
	}
}

static void *LpcI2CGetHandle(DevIntrf_t *pIntrf)
{
	return LpcI2CData(pIntrf)->pDev;
}

bool LpcI2CMasterInit(LpcI2CMaster_t *pData, I2CDev_t *pDev, const I2CCfg_t *pCfg, uint32_t Clock)
{
	uint32_t state = DisableInterrupt();

	if (pData->bTransfer || (pData->pDev != NULL && pData->pDev != pDev))
	{
		EnableInterrupt(state);
		return false;
	}
	pData->Clock = Clock;
	pData->pDev = pDev;
	pData->bEnabled = false;
	pData->bError = false;
	pData->bOwned = false;
	pDev->Cfg = *pCfg;
	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = pData;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->bIntEn = false;
	intrf->bDma = false;
	intrf->bNoStop = false;
	intrf->bTxReady = true;
	intrf->GetHandle = LpcI2CGetHandle;
	intrf->Enable = LpcI2CEnable;
	intrf->Disable = LpcI2CDisable;
	intrf->PowerOff = LpcI2CPowerOff;
	intrf->Reset = LpcI2CReset;
	intrf->GetRate = LpcI2CGetRate;
	intrf->SetRate = LpcI2CSetRate;
	intrf->StartRx = LpcI2CStartRx;
	intrf->StartTx = LpcI2CStartTx;
	intrf->RxData = LpcI2CRxData;
	intrf->TxData = LpcI2CTxData;
	intrf->TxSrData = NULL;
	intrf->StopRx = LpcI2CStop;
	intrf->StopTx = LpcI2CStop;
	intrf->EnCnt = 1;
	atomic_flag_clear(&intrf->bBusy);
	LpcI2CSetRate(intrf, pCfg->Rate);
	LpcI2CEnable(intrf);
	EnableInterrupt(state);
	return true;
}
