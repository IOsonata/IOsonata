/**-------------------------------------------------------------------------
@file	i2c_stm32l4xx.cpp

@brief	I2C implementation on STM32L4xx series MCU

@author	Hoang Nguyen Hoan
@date	Sep. 5, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include "stm32l4xx.h"

#include "istddef.h"
#include "coredev/i2c.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "diskio_flash.h"

#ifdef STM32L4S9xx
#define STM32L4XX_I2C_MAXDEV		4
#else
#define STM32L4XX_I2C_MAXDEV		3
#endif

#define RCC_CCIPR_I2CSEL_PCLK		0
#define RCC_CCIPR_I2CSEL_SYSCLK		1
#define RCC_CCIPR_I2CSEL_HSI16		2

#define I2CCLK_HZ					16000000

#pragma pack(push, 4)

typedef struct {
	int DevNo;
	I2CDev_t *pI2cDev;
	I2C_TypeDef	*pReg;
} STM32L4XX_I2CDEV;

#pragma pack(pop)

static STM32L4XX_I2CDEV s_STM32L4xxI2CDev[STM32L4XX_I2C_MAXDEV] = {
	{
		0, NULL, .pReg = I2C1
	},
	{
		1, NULL, .pReg = I2C2
	},
	{
		2, NULL, .pReg = I2C3
	},
#ifdef STM32L4S9xx
	{
		2, NULL, .pReg = I2C4
	},
#endif
};

static bool STM32L4xxI2CWaitBusy(STM32L4XX_I2CDEV *pDev, int Timeout)
{
	while (Timeout-- > 0)
	{
		if ((pDev->pReg->ISR & I2C_ISR_BUSY) == 0)
		{
			return true;
		}
	}

	return false;
}

static bool STM32L4xxI2CWaitStop(STM32L4XX_I2CDEV *pDev, int Timeout)
{
	while (Timeout-- > 0)
	{
		if ((pDev->pReg->ISR & I2C_ISR_STOPF))
		{
			return true;
		}
	}

	return false;
}

static bool STM32L4xxI2CWaitTxComplete(STM32L4XX_I2CDEV *pDev, int Timeout)
{
	while (Timeout-- > 0)
	{
		if ((pDev->pReg->ISR & (I2C_ISR_TC | I2C_ISR_TCR)))
		{
			return true;
		}
	}

	return false;
}

static bool STM32L4xxI2CWaitTxRdy(STM32L4XX_I2CDEV *pDev, int Timeout)
{
	while (Timeout-- > 0)
	{
		if (pDev->pReg->ISR & (/*I2C_ISR_TXIS |*/ I2C_ISR_TXE))
		{
			return true;
		}
		usDelay(1);
	}

	return false;
}

static bool STM32L4xxI2CWaitRx(STM32L4XX_I2CDEV *pDev, int Timeout)
{
	while (Timeout-- > 0)
	{
		if (pDev->pReg->ISR & I2C_ISR_RXNE)
		{
			return true;
		}
	}

	return false;
}

static void STM32L4xxI2CReset(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	dev->pReg->CR1 &= ~I2C_CR1_PE;

	if (dev->DevNo < 3)
	{
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST << dev->DevNo;
		usDelay(100);
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C1RST << dev->DevNo);
	}
#ifdef STM32L4S9xx
	else
	{
		RCC->APB1RSTR2 |= RCC_APB1RSTR2_I2C4RST;
		usDelay(100);
		RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_I2C4RST;
	}
#endif
}

static uint32_t STM32L4xxI2CGetRate(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	return dev->pI2cDev->Cfg.Rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
// Calculation
// SCL period = tsync1 + tsync2 + ((SCLL+1) + (SCLH+1)) * (PRESC+1)) * ti2cclk
//
static uint32_t STM32L4xxI2CSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;
	uint32_t iclk = I2CCLK_HZ;	// default i2cclk 8MHz for fast mode

	// Get PCLK1
	uint32_t clk = SystemPeriphClockGet(0);

	if (Rate <= 100000)
	{
		// use lower clock rate for standard mode
		iclk = 4000000; // 4Mhz
	}

	// Select clock source. We want to use 8MHz as i2cclk (I2CCLK_HZ)

	if (clk < iclk)
	{
		// Get SYSCLK
		clk = SystemCoreClockGet();

		if (dev->DevNo < 3)
		{
			RCC->CCIPR &= ~(RCC_CCIPR_I2C1SEL_Msk << (dev->DevNo << 1));
			RCC->CCIPR |= RCC_CCIPR_I2CSEL_SYSCLK << (RCC_CCIPR_I2C1SEL_Pos + (dev->DevNo << 1));
		}
#ifdef STM32L4S9xx
		else
		{
			RCC->CCIPR2 &= ~RCC_CCIPR2_I2C4SEL_Msk;
			RCC->CCIPR2 |= RCC_CCIPR_I2CSEL_SYSCLK << RCC_CCIPR2_I2C4SEL_Pos;
		}
#endif
	}
	else
	{
		// PCLK1
		if (dev->DevNo < 3)
		{
			RCC->CCIPR &= ~(RCC_CCIPR_I2C1SEL_Msk << (dev->DevNo << 1));
			RCC->CCIPR |= RCC_CCIPR_I2CSEL_PCLK << (RCC_CCIPR_I2C1SEL_Pos + (dev->DevNo << 1));
		}
#ifdef STM32L4S9xx
		else
		{
			RCC->CCIPR2 &= ~RCC_CCIPR2_I2C4SEL_Msk;
			RCC->CCIPR2 |= RCC_CCIPR_I2CSEL_PCLK << RCC_CCIPR2_I2C4SEL_Pos;
		}
#endif
	}

	uint32_t presc = (clk / iclk - 1) & 0xf;
	iclk = clk / (presc + 1);
	uint32_t period = 1000000000ULL / iclk;
	uint32_t scldel;
	uint32_t sdadel;
	uint32_t dnf = ((dev->pReg->CR1 & I2C_CR1_DNF_Msk) >> I2C_CR1_DNF_Pos) * period;
	uint32_t af = (1 - ((dev->pReg->CR1 & I2C_CR1_ANFOFF_Msk) >> I2C_CR1_ANFOFF_Pos)) * 50;

	if (Rate <= 100000)
	{
		scldel = ((I2C_TR_STDMODE_MAX + I2C_TSUDAT_STDMODE_MIN + period * 2) / period - 1) & 0xf;
		sdadel = ((I2C_TF_STDMODE_MAX - af - dnf - period * 2) / period - 1) & 0xf;
	}
	else if (Rate <= 400000)
	{
		scldel = ((I2C_TR_FASTMODE_MAX + I2C_TSUDAT_FASTMODE_MIN + period * 2) / period - 1) & 0xf;
		sdadel = ((I2C_TF_FASTMODE_MAX - af - dnf  - period * 2) / period - 1) & 0xf;
	}
	else
	{
		scldel = ((I2C_TR_FASTMODEPLUS_MAX + I2C_TSUDAT_FASTMODEPLUS_MIN + period * 2) / period - 1) & 0xf;
		sdadel = ((I2C_TF_FASTMODEPLUS_MAX - af - dnf - period * 2) / period - 1) & 0xf;
	}
	uint32_t scll = ((iclk / (Rate << 1)) -1);//+ (scldel + 1) - 1) & 0xff;
	uint32_t sclh = ((iclk / (Rate << 1)) -1);// (sdadel + 1) - 1) & 0xff;

	uint32_t timingr = (presc << 28) | (scldel << 20) | (sdadel << 16) |
						 (sclh << 8) | scll;
	dev->pReg->TIMINGR = timingr;
/*
	printf("%d timingr: %x\n", clk, timingr);
	uint32_t r = 1000000000 / ((scll + sclh) * period);
	printf("presc:%x, scldel:%x, sdadel:%x\n", presc, scldel, sdadel);
	printf("%d, %d, %x, %x, r = %d\n", iclk, Rate, scll, sclh, r);
*/
	return Rate;
}

void STM32L4xxI2CDisable(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	dev->pReg->CR1 &= ~I2C_CR1_PE;

	if (dev->DevNo < 3)
	{
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_I2C1EN << dev->DevNo);
	}
#ifdef STM32L4S9xx
	else
	{
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_I2C4EN;
	}
#endif
}

static void STM32L4xxI2CEnable(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	if (dev->DevNo < 3)
	{
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN << dev->DevNo;
	}
#ifdef STM32L4S9xx
	else
	{
		RCC->APB1ENR2 |= RCC_APB1ENR2_I2C4EN;
	}
#endif
}

static void STM32L4xxI2CPowerOff(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	if (dev->DevNo < 3)
	{
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_I2C1EN << dev->DevNo);
	}
#ifdef STM32L4S9xx
	else
	{
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_I2C4EN;
	}
#endif
}

// Initial receive
static bool STM32L4xxI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	dev->pReg->CR2 = ((DevAddr << 1) & I2C_CR2_SADD_Msk) | I2C_CR2_RD_WRN;
	dev->pReg->CR2 |= I2C_CR2_START;

	return true;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxI2CRxDataDma(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxI2CRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;
    int cnt = 0;
    uint16_t d = 0;

    while (BuffLen > 0)
    {
        int len = min(BuffLen, 255);

        uint32_t cr2 = dev->pReg->CR2 & ~I2C_CR2_NBYTES_Msk;
    	cr2 |= (len << I2C_CR2_NBYTES_Pos);
    	dev->pReg->CR2 = cr2;

    	while (len > 0)
    	{
			if (STM32L4xxI2CWaitRx(dev, 1000000) == false)
			{
				return cnt;
			}
			*pBuff = dev->pReg->RXDR;
			pBuff++;
			cnt++;
			BuffLen--;
			len--;
    	}
    }

    return cnt;
}

// Stop receive
static void STM32L4xxI2CStopRx(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;

	dev->pReg->CR2 |= I2C_CR2_STOP;
	STM32L4xxI2CWaitStop(dev, 100000);
}

// Initiate transmit
static bool STM32L4xxI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;

	dev->pReg->CR2 = ((DevAddr << 1) & I2C_CR2_SADD_Msk) | I2C_CR2_RELOAD;
	dev->pReg->CR2 |= I2C_CR2_START;

	return true;
}

// Transmit Data only, no Start/Stop condition
static int STM32L4xxI2CTxDataDma(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Send Data only, no Start/Stop condition
static int STM32L4xxI2CTxData(DevIntrf_t *pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV*)pDev->pDevData;
    int cnt = 0;

    while (DataLen > 0)
    {
    	int len = min(DataLen, 255);
        uint32_t cr2 = dev->pReg->CR2 & ~I2C_CR2_NBYTES_Msk;

        cr2 |= (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD;
		dev->pReg->CR2 = cr2;

		while (len > 0)
		{
			if (STM32L4xxI2CWaitTxRdy(dev, 1000000) == false)
			{
				return cnt;
			}
			dev->pReg->TXDR = *pData;
			pData++;
			len--;
			cnt++;
			DataLen--;
		}
    }

    return cnt;
}

// Stop transmit
static void STM32L4xxI2CStopTx(DevIntrf_t * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;

	STM32L4xxI2CWaitTxComplete(dev, 100000);
	dev->pReg->CR2 |= I2C_CR2_STOP;
	STM32L4xxI2CWaitStop(dev, 1000000);
	dev->pReg->ICR = dev->pReg->ISR;
}

void I2CIrqHandler(int DevNo)
{
	STM32L4XX_I2CDEV *dev = &s_STM32L4xxI2CDev[DevNo];
}


bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	I2C_TypeDef *reg;
	uint32_t cr1reg = 0;
	uint32_t tmp = 0;
	bool retval = false;

	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo >= STM32L4XX_I2C_MAXDEV)
	{
		return false;
	}

	// Save config data
	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));

	s_STM32L4xxI2CDev[pCfgData->DevNo].pI2cDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_STM32L4xxI2CDev[pCfgData->DevNo];

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Reset = STM32L4xxI2CReset;
	pDev->DevIntrf.Disable = STM32L4xxI2CDisable;
	pDev->DevIntrf.Enable = STM32L4xxI2CEnable;
	pDev->DevIntrf.GetRate = STM32L4xxI2CGetRate;
	pDev->DevIntrf.SetRate = STM32L4xxI2CSetRate;
	pDev->DevIntrf.StartRx = STM32L4xxI2CStartRx;
	pDev->DevIntrf.RxData = STM32L4xxI2CRxData;
	pDev->DevIntrf.StopRx = STM32L4xxI2CStopRx;
	pDev->DevIntrf.StartTx = STM32L4xxI2CStartTx;
	pDev->DevIntrf.TxData = STM32L4xxI2CTxData;
	pDev->DevIntrf.StopTx = STM32L4xxI2CStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = STM32L4xxI2CPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	if (pDev->Cfg.DevNo < 3)
	{
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN << pDev->Cfg.DevNo;
	}
#ifdef STM32L4S9xx
	else
	{
		RCC->APB1ENR2 |= RCC_APB1ENR2_I2C4EN;
	}
#endif

	// Reset I2C engine
	STM32L4xxI2CReset(&pDev->DevIntrf);

	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);

	for (int i = 0; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSetSpeed(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo, IOPINSPEED_TURBO);
	}

	// Get the correct register map
	reg = s_STM32L4xxI2CDev[pCfgData->DevNo].pReg;


	// Note : this function call will modify CR1 register
	STM32L4xxI2CSetRate(&pDev->DevIntrf, pCfgData->Rate);

	// Enable fast mode
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C1_FMP << pDev->Cfg.DevNo;

	uint32_t cr1 = 0;

	if (pDev->Cfg.Mode == I2CMODE_SLAVE)
	{
		pDev->Cfg.bIntEn = true;	// Force interrupt in slave mode

		if (pDev->Cfg.bClkStretch == false)
		{
			// Clock stretching enable
			cr1 |= I2C_CR1_NOSTRETCH;
		}


	}
	else
	{
		reg->OAR1 = 0x3FF;
		reg->OAR2 = 0;
	}

	if (pDev->Cfg.bDmaEn)
	{
		// DMA enable
		cr1 |= I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN;
	}

    if (pCfgData->bIntEn && pCfgData->Mode == I2CMODE_SLAVE)
    {
    	cr1 |= I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE |
    		   I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE;

    	switch (pCfgData->DevNo)
    	{
    		case 0:
                NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
                NVIC_SetPriority(I2C1_EV_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(I2C1_EV_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
                NVIC_SetPriority(I2C2_EV_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(I2C2_EV_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(I2C3_EV_IRQn);
                NVIC_SetPriority(I2C3_EV_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(I2C3_EV_IRQn);
                break;
    	}
    }

    reg->CR1 = cr1 | I2C_CR1_PE | I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXIE;
	reg->ICR = 0x3F38;

	return true;
}

extern "C" void I2C1_EV_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
}

extern "C" void I2C1_ER_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
}

extern "C" void I2C2_EV_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
}

extern "C" void I2C2_ER_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(I2C2_ER_IRQn);
}

extern "C" void I2C3_EV_IRQHandler()
{
    NVIC_ClearPendingIRQ(I2C3_EV_IRQn);
}

extern "C" void I2C3_ER_IRQHandler()
{
    NVIC_ClearPendingIRQ(I2C3_ER_IRQn);
}

