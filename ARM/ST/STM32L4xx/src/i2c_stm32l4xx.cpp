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
#include "system_core_clock.h"
#include "idelay.h"
#include "diskio_flash.h"

#define STM32L4XX_I2C_MAXDEV		3

#pragma pack(push, 4)

typedef struct {
	int DevNo;
	I2CDEV *pI2cDev;
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
};

static uint32_t STM32L4xxI2CGetRate(DEVINTRF * const pDev)
{
	int rate = 0;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
static uint32_t STM32L4xxI2CSetRate(DEVINTRF * const pDev, uint32_t DataRate)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	uint32_t pclk = SystemPeriphClockGet();
	uint32_t div = (pclk + (DataRate >> 1)) / DataRate;

	return DataRate;
}

void STM32L4xxI2CDisable(DEVINTRF * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;
	int32_t timout = 100000;
}

static void STM32L4xxI2CEnable(DEVINTRF * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

}

static void STM32L4xxI2CPowerOff(DEVINTRF * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

}

// Initial receive
static bool STM32L4xxI2CStartRx(DEVINTRF * const pDev, uint32_t DevAddr)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev->pDevData;

	return true;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxI2CRxDataDma(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Receive Data only, no Start/Stop condition
static int STM32L4xxI2CRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;
    int cnt = 0;
    uint16_t d = 0;

    return cnt;
}

// Stop receive
static void STM32L4xxI2CStopRx(DEVINTRF * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;

}

// Initiate transmit
static bool STM32L4xxI2CStartTx(DEVINTRF * const pDev, uint32_t DevAddr)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;

	return true;
}

// Transmit Data only, no Start/Stop condition
static int STM32L4xxI2CTxDataDma(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;
	int cnt = 0;

	return cnt;
}

// Send Data only, no Start/Stop condition
static int STM32L4xxI2CTxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV*)pDev->pDevData;
    int cnt = 0;
    uint16_t d;


    return cnt;
}

// Stop transmit
static void STM32L4xxI2CStopTx(DEVINTRF * const pDev)
{
	STM32L4XX_I2CDEV *dev = (STM32L4XX_I2CDEV *)pDev-> pDevData;

}

void I2CIrqHandler(int DevNo)
{
	STM32L4XX_I2CDEV *dev = &s_STM32L4xxI2CDev[DevNo];
}


bool I2CInit(I2CDEV * const pDev, const I2CCFG *pCfgData)
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

	pDev->Mode = pCfgData->Mode;
	s_STM32L4xxI2CDev[pCfgData->DevNo].pI2cDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_STM32L4xxI2CDev[pCfgData->DevNo];

	// Configure I/O pins
	memcpy(pDev->Pins, pCfgData->Pins, sizeof(pDev->Pins));

	IOPinCfg(pCfgData->Pins, I2C_MAX_NB_IOPIN);

	for (int i = 0; i < I2C_MAX_NB_IOPIN; i++)
	{
		IOPinSetSpeed(pCfgData->Pins[i].PortNo, pCfgData->Pins[i].PinNo, IOPINSPEED_TURBO);
	}

	// Get the correct register map
	reg = s_STM32L4xxI2CDev[pCfgData->DevNo].pReg;


	// Note : this function call will modify CR1 register
	STM32L4xxI2CSetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
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

    if (pCfgData->bIntEn && pCfgData->Mode == I2CMODE_SLAVE)
    {
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

	return true;
}

void QSPIIrqHandler()
{

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

