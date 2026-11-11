/**-------------------------------------------------------------------------
@file	i2c_sam4e.cpp

@brief	I2C implementation on SAM4E series MCU

@author	Hoang Nguyen Hoan
@date	July 12, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#include <math.h>

#include "sam4e.h"

#include "istddef.h"
#include "coredev/i2c.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "idelay.h"

#define SAM4E_I2C_MAXDEV        2
#define SAM4E_I2C_MAXSLAVE		1

#define SAM4E_I2C_TRBUFF_SIZE	4

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	uint32_t SamDevId;
	I2CDev_t *pI2cDev;
	Sam4eTwi *pReg;
	Sam4ePdc *pPdc;
	uint8_t TRData[SAM4E_I2C_TRBUFF_SIZE];
} SAM4E_I2CDEV;
#pragma pack(pop)

static SAM4E_I2CDEV s_Sam4eI2CDev[] = {
	{
		0, 17, NULL, SAM4E_TWI0, SAM4E_PDC_TWI0,
	},
	{
		1, 18, NULL, SAM4E_TWI1, SAM4E_PDC_TWI1,
	},
};

static const int s_NbSam4eI2cDev = sizeof(s_Sam4eI2CDev) / sizeof(SAM4E_I2CDEV);

bool Sam4eI2CWaitRx(SAM4E_I2CDEV * const pDev, int Timeout)
{
    do {
    	uint32_t sr = pDev->pReg->TWI_SR;
    	if (sr & TWI_SR_NACK)
    	{
    		return false;
    	}
        if (sr & TWI_SR_OVRE)
        {
            return false;
        }
        if (pDev->pI2cDev->DevIntrf.bDma)
        {
			if (sr & TWI_SR_ENDRX)
			{
				return true;
			}
        }
        else
        {
            if (sr & TWI_SR_RXRDY)
            {
                return true;
            }
        }
    } while (Timeout-- >  0);

    return false;
}

bool Sam4eI2CWaitTx(SAM4E_I2CDEV * const pDev, int Timeout)
{
    do {
    	uint32_t sr = pDev->pReg->TWI_SR;

    	if (sr & TWI_SR_NACK)
    	{
    		return false;
    	}
    	if (sr & TWI_SR_TXRDY)
    	{
    		return true;
    	}
    } while (Timeout-- >  0);

    return false;
}

bool Sam4eI2CWaitStop(SAM4E_I2CDEV * const pDev, int Timeout)
{
	do {
		if (pDev->pReg->TWI_CR & TWI_SR_TXCOMP)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

void Sam4eI2CDisable(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

	dev->pReg->TWI_IDR = 0xFFFFFFFFUL;
	dev->pReg->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVDIS;
}

void Sam4eI2CEnable(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

    if (dev->pI2cDev->DevIntrf.bDma)
    {
    }
	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		dev->pReg->TWI_CR = TWI_CR_SVEN;
	}
	else
	{
		dev->pReg->TWI_CR = TWI_CR_MSEN;
	}
}

void Sam4eI2CPowerOff(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;
}

uint32_t Sam4eI2CGetRate(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

	return dev->pI2cDev->Cfg.Rate;
}

// TWI clock (TWI_CWGR): low period (CLDIV * 2^CKDIV + 4) MCK cycles, high
// period (CHDIV * 2^CKDIV + 4) MCK cycles, SCL = MCK / (((CLDIV + CHDIV) <<
// CKDIV) + 8). The rate closest to the request, at most 100 kHz up to a
// 100 kHz request, at most 400 kHz above, each period at least the I2C
// minimum of the mode. Out of the range of the dividers, the closest end.
uint32_t Sam4eI2CSetRate(DevIntrf_t * const pDev, uint32_t RateHz)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;
	uint32_t mck = SystemPeriphClockGet(0);

	if (RateHz == 0 || mck == 0)
	{
		return 0;
	}
	if (RateHz > I2C_SCL_FAST_MODE_MAX_SPEED * 1000U)
	{
		RateHz = I2C_SCL_FAST_MODE_MAX_SPEED * 1000U;
	}

	bool fast = RateHz > I2C_SCL_STD_MODE_MAX_SPEED * 1000U;
	uint32_t top = (fast ? I2C_SCL_FAST_MODE_MAX_SPEED : I2C_SCL_STD_MODE_MAX_SPEED) * 1000U;

	// Minimum low, high and SCL periods in MCK cycles
	uint32_t lowmin = (uint32_t)(((uint64_t)mck * (fast ? I2C_SCL_TLOW_FAST_MODE_MIN :
						I2C_SCL_TLOW_STD_MODE_MIN) + 999999999ULL) / 1000000000ULL);
	uint32_t highmin = (uint32_t)(((uint64_t)mck * (fast ? I2C_SCL_THIGH_FAST_MODE_MIN :
						I2C_SCL_THIGH_STD_MODE_MIN) + 999999999ULL) / 1000000000ULL);
	uint32_t permin = (mck + top - 1) / top;
	uint32_t req = mck / RateHz;
	uint32_t bestper = 0;
	uint64_t besterr = 0;
	uint32_t bestsum = 510;
	uint32_t bestck = 7;
	uint32_t clmin = 0;
	uint32_t chmin = 0;

	for (uint32_t ck = 0; ck < 8; ck++)
	{
		uint32_t up = (1U << ck) - 1;
		uint32_t cl = lowmin > 4 ? (lowmin - 4 + up) >> ck : 0;
		uint32_t ch = highmin > 4 ? (highmin - 4 + up) >> ck : 0;

		// Range of CLDIV + CHDIV
		uint32_t summin = permin > 8 ? (permin - 8 + up) >> ck : 0;

		if (summin < cl + ch)
		{
			summin = cl + ch;
		}
		if (cl > 255 || ch > 255 || summin > 510)
		{
			continue;
		}

		// The sums around the request: the first at or above it, the next
		// below it
		uint32_t sum = req > 8 ? (req - 8) >> ck : 0;

		for (uint32_t i = 0; i < 2; i++, sum++)
		{
			uint32_t s = sum < summin ? summin : (sum > 510 ? 510 : sum);
			uint32_t per = (s << ck) + 8;
			uint64_t rp = (uint64_t)RateHz * per;

			// Error of MCK / per to RateHz, scaled by per
			uint64_t err = rp > mck ? rp - mck : mck - rp;

			if (bestper == 0 || err * bestper < besterr * per)
			{
				bestper = per;
				besterr = err;
				bestsum = s;
				bestck = ck;
				clmin = cl;
				chmin = ch;
			}
		}
	}

	if (bestper == 0)
	{
		// Slowest
		bestper = (510U << 7) + 8;
		clmin = 0;
		chmin = 0;
	}

	// The low period takes the larger half
	uint32_t chdiv = bestsum / 2;

	if (chdiv < chmin)
	{
		chdiv = chmin;
	}
	if (bestsum - chdiv < clmin)
	{
		chdiv = bestsum - clmin;
	}

	dev->pReg->TWI_CWGR = TWI_CWGR_CLDIV(bestsum - chdiv) | TWI_CWGR_CHDIV(chdiv) | TWI_CWGR_CKDIV(bestck);
	dev->pI2cDev->Cfg.Rate = mck / bestper;

	return dev->pI2cDev->Cfg.Rate;
}

bool Sam4eI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

	if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
	{

	}
	else
	{
		dev->pReg->TWI_MMR |= TWI_MMR_MREAD | TWI_MMR_DADR(DevAddr);
		dev->pReg->TWI_CR = TWI_CR_START;
	}

	return true;
}

// Receive Data only, no Start/Stop condition
int Sam4eI2CRxData(DevIntrf_t *pDev, uint8_t *pBuff, int BuffLen)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;
	int cnt = 0;

	if (pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	while (BuffLen > 0)
	{
		if (Sam4eI2CWaitRx(dev, 100000) == false)
		{
			break;
		}

		*pBuff = dev->pReg->TWI_RHR;

		BuffLen--;
		pBuff++;
		cnt++;
	}

	return cnt;
}

void Sam4eI2CStopRx(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

	dev->pReg->TWI_CR = TWI_CR_STOP;
    if (dev->pI2cDev->DevIntrf.bDma == false)
    {
        // must read dummy last byte to generate NACK & STOP condition
    	Sam4eI2CWaitRx(dev, 100000);
    	uint8_t d __attribute__((unused)) = dev->pReg->TWI_RHR;
    }
    Sam4eI2CWaitStop(dev, 1000);
    dev->pReg->TWI_IADR = 0;
}

bool Sam4eI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

//	dev->pReg->TWI_MMR = 0;
	dev->pReg->TWI_MMR = TWI_MMR_DADR(DevAddr);
	dev->pReg->TWI_IADR = 0;

	return true;
}

int Sam4eI2CTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;
	uint32_t d;
	int cnt = 0;

	while (DataLen > 0)
	{
		dev->pReg->TWI_THR = *pData;

		if (Sam4eI2CWaitTx(dev, 10000) == false)
		{
			break;
		}

		DataLen--;
		pData++;
		cnt++;
	}
	return cnt;
}

int Sam4eI2CTxSrData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;
	uint32_t d = 0;
	int cnt = 1;

	dev->pReg->TWI_MMR |= (DataLen << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk;

	for (int i = 0; i < DataLen; i++)
	{
		d |= (uint32_t)pData[DataLen-i-1] << (8UL * i);
		cnt++;
	}

	dev->pReg->TWI_IADR = d;//twi_mk_addr(pData, DataLen);

	return cnt;
}

void Sam4eI2CStopTx(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

	//Sam4eI2CWaitTx(dev, 10000);
	dev->pReg->TWI_CR = TWI_CR_STOP;

	Sam4eI2CWaitStop(dev, 10000);
}

void Sam4eI2CReset(DevIntrf_t * const pDev)
{
	SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

	dev->pReg->TWI_IDR = 0xFFFFFFFFUL;
	(void)dev->pReg->TWI_SR;

	// reset
	dev->pReg->TWI_CR = TWI_CR_SWRST;
	dev->pReg->TWI_RHR;
}

void I2CIrqHandler(int DevNo, DevIntrf_t * const pDev)
{
    SAM4E_I2CDEV *dev = (SAM4E_I2CDEV*)pDev->pDevData;

    if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
    {
    	// Slave mode
    }
    else
    {
    	// Master mode
    	// TODO: implement interrupt handling for master mode
    }

}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

    if (pCfgData->DevNo < 0 || pCfgData->DevNo >= s_NbSam4eI2cDev)
	{
		return false;
	}

    int devno = pCfgData->DevNo;

	// Get the correct register map
	Sam4eTwi *reg = s_Sam4eI2CDev[devno].pReg;

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));


	if (s_Sam4eI2CDev[devno].SamDevId < 32)
	{
		SAM4E_PMC->PMC_PCER0 |= 1 << s_Sam4eI2CDev[devno].SamDevId;
	}
	else
	{
		SAM4E_PMC->PMC_PCER1 |= 1 << (s_Sam4eI2CDev[devno].SamDevId - 32);
	}

	// Configure I/O pins
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
    IOPinSet(pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PortNo, pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PinNo);
    IOPinSet(pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PortNo, pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PinNo);

	s_Sam4eI2CDev[pCfgData->DevNo].pI2cDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_Sam4eI2CDev[pCfgData->DevNo];

	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.Disable = Sam4eI2CDisable;
	pDev->DevIntrf.Enable = Sam4eI2CEnable;
	pDev->DevIntrf.PowerOff = Sam4eI2CPowerOff;
	pDev->DevIntrf.GetRate = Sam4eI2CGetRate;
	pDev->DevIntrf.SetRate = Sam4eI2CSetRate;
	pDev->DevIntrf.StartRx = Sam4eI2CStartRx;
	pDev->DevIntrf.StopRx = Sam4eI2CStopRx;
	pDev->DevIntrf.StartTx = Sam4eI2CStartTx;
	pDev->DevIntrf.TxSrData = Sam4eI2CTxSrData;
	if (pDev->DevIntrf.bDma)
	{
		//pDev->DevIntrf.RxData = Sam4eI2CRxDataDMA;
		//pDev->DevIntrf.TxData = Sam4eI2CTxDataDMA;
	}
	else
	{
		pDev->DevIntrf.RxData = Sam4eI2CRxData;
		pDev->DevIntrf.TxData = Sam4eI2CTxData;
	}
	pDev->DevIntrf.StopTx = Sam4eI2CStopTx;
	pDev->DevIntrf.Reset = Sam4eI2CReset;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	Sam4eI2CReset(&pDev->DevIntrf);

    usDelay(1000);

	reg->TWI_CR = TWI_CR_MSDIS | TWI_CR_SVDIS;
	reg->TWI_IADR = 0;
	reg->TWI_MMR = 0;
	reg->TWI_SMR = 0;

	Sam4eI2CSetRate(&pDev->DevIntrf, pCfgData->Rate);

    if (pDev->DevIntrf.bDma)
    {
    }

    uint32_t inten = TWI_IER_TXCOMP;

    if (pCfgData->Mode == I2CMODE_SLAVE)
    {
        pDev->Cfg.NbSlaveAddr = 1;	// Only 1 slave address
    	pDev->Cfg.SlaveAddr[0] = pCfgData->SlaveAddr[0];
    	reg->TWI_SMR = TWI_SMR_SADR(pDev->Cfg.SlaveAddr[0]);

    	reg->TWI_CR = TWI_CR_SVEN;
    	inten = TWI_IER_SVACC | TWI_IER_EOSACC;
    }
    else
    {
    	reg->TWI_CR = TWI_CR_MSEN;
    }

    if (pCfgData->bIntEn)
    {
    	if (pCfgData->DevNo == 0)
    	{
    		NVIC_ClearPendingIRQ(TWI0_IRQn);
    		NVIC_SetPriority(TWI0_IRQn, pCfgData->IntPrio);
    		NVIC_EnableIRQ(TWI0_IRQn);
    	}
    	else
    	{
    		NVIC_ClearPendingIRQ(TWI1_IRQn);
    		NVIC_SetPriority(TWI1_IRQn, pCfgData->IntPrio);
    		NVIC_EnableIRQ(TWI1_IRQn);
    	}

    	reg->TWI_IER = inten;
    }

//    reg->ENABLE = enval;

	return true;
}

extern "C" void TWI0_Handler(void)
{
	I2CIrqHandler(0, &s_Sam4eI2CDev[0].pI2cDev->DevIntrf);
}

extern "C" void TWI1_Handler(void)
{
	I2CIrqHandler(1, &s_Sam4eI2CDev[1].pI2cDev->DevIntrf);
}
