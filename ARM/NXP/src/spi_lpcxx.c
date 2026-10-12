/*--------------------------------------------------------------------------
File   : spi_lpcxx.c

Author : Hoang Nguyen Hoan          Dec. 25, 2011

Desc   : Synchronous Serial Port(SSP) implementation on LPC series MCU

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
Modified by         Date            Description
Hoan				Feb. 20, 2015	New EHAL
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "coredev/iopincfg.h"
#include "spi_lpcxx.h"
#include "coredev/interrupt.h"

extern uint32_t SystemCoreClock;

uint32_t LpcSSPGetRate(DevIntrf_t *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((SSPDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// SCK = PCLK / (CPSDVSR * (SCR + 1)). CPSDVSR is even, 2..254,
// and SCR is 0..255. For each prescaler, only the two divisors bracketing
// the requested rate can be nearest. Zero selects the minimum rate.
uint32_t LpcSSPSetRate(DevIntrf_t *pDev, uint32_t DataRate)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return 0;
	}
	SSPDEV *dev = (SSPDEV *)pDev->pDevData;

	if (dev->pSpiDev == NULL || dev->pSspReg == NULL || dev->PClkFreq == 0)
	{
		return 0;
	}
	uint32_t bestDiv = 0;
	uint32_t bestPrescale = 2;
	uint32_t bestSerial = 1;
	uint64_t bestError = 0;

	for (uint32_t prescale = 2; prescale <= 254; prescale += 2)
	{
		uint64_t ideal = DataRate == 0 ? 256 :
			dev->PClkFreq / ((uint64_t)DataRate * prescale);

		for (uint32_t next = 0; next < 2; next++)
		{
			uint64_t serial = ideal + next;

			if (serial < 1)
			{
				serial = 1;
			}
			if (serial > 256)
			{
				serial = 256;
			}
			uint32_t div = prescale * (uint32_t)serial;
			uint64_t target = (uint64_t)DataRate * div;
			uint64_t error = target > dev->PClkFreq ?
				target - dev->PClkFreq : dev->PClkFreq - target;

			// Compare exact errors before rounding to integer Hz. Both
			// divisors are below 65536, so these products fit in 64 bits.
			if (bestDiv == 0 || error * bestDiv < bestError * div)
			{
				bestDiv = div;
				bestError = error;
				bestPrescale = prescale;
				bestSerial = (uint32_t)serial;
			}
		}
	}

	uint32_t state = DisableInterrupt();

	// Keep an in-flight frame and any queued TX frames at their old rate.
	if ((dev->pSspReg->SR & (LPCSSP_SR_BSY | LPCSSP_SR_TFE)) != LPCSSP_SR_TFE)
	{
		EnableInterrupt(state);
		return dev->pSpiDev->Cfg.Rate;
	}
	uint32_t cr1 = dev->pSspReg->CR1;

	dev->pSspReg->CR1 = cr1 & ~LPCSSP_CR1_SSP_EN;
	dev->pSspReg->CPSR = bestPrescale;
	dev->pSspReg->CR0 = (dev->pSspReg->CR0 & ~LPCSSP_CR0_SCR_MASK) |
		((bestSerial - 1U) << 8);
	dev->pSspReg->CR1 = cr1;
	dev->pSpiDev->Cfg.Rate = (uint32_t)(((uint64_t)dev->PClkFreq + bestDiv / 2U) / bestDiv);
	EnableInterrupt(state);

	return dev->pSpiDev->Cfg.Rate;
}

// Initial receive
bool LpcSSPStartRx(DevIntrf_t *pDev, uint32_t DevAddr)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;

	LpcSSPWaitBusy(dev, 100000);

	// Clear interrupt status
	dev->pSspReg->ICR = 3;

	return true;
}

// Receive Data only, no Start/Stop condition
int LpcSSPRxData(DevIntrf_t *pDev, uint8_t *pBuff, int BuffLen)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;
	int cnt = 0;
	bool bw = true;
	//uint16_t *p16 = (uint16_t *)pBuff;
	uint16_t d = 0xff;

	if (dev->pSspReg->CR0 & 0x8)
	{
		// 9-16 bits data
		bw = false;
		d = 0xffff;
	}

	while (BuffLen > cnt) 
	{
		dev->pSspReg->DR = d;
		//for (int i = 10; i > 0; i--)
		//	__NOP();
		if (LpcSSPWaitRxFifo(dev, 100000))
		{
			if (bw) 
			{
				*pBuff = dev->pSspReg->DR & 0xff;
				pBuff++;
				cnt++;
			} 
			else
			{
				d = dev->pSspReg->DR & LPCSSP_DR_MASK;
				pBuff[0] = d & 0xff;
				pBuff[1] = (d >> 8) & 0xff;
				cnt+=2;
				pBuff+=2;
			}
		}
	}

	return cnt;
}

// Stop receive
void LpcSSPStopRx(DevIntrf_t *pDev)
{

}

// Receive stream
int LpcSSPRx(DevIntrf_t *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
{
	int cnt = 0;

	if (LpcSSPStartRx(pDev, DevAddr))
	{
		cnt = LpcSSPRxData(pDev, pBuff, BuffLen);
		LpcSSPStopRx(pDev);
	}

	return cnt;
}


// Initiate transmit
bool LpcSSPStartTx(DevIntrf_t *pDev, uint32_t DevAddr)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;

	LpcSSPWaitBusy(dev, 100000);
	// Clear interrupt status
	dev->pSspReg->ICR = 3;

	return true;
}

// Transmit Data only, no Start/Stop condition
int LpcSSPTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;
	int cnt = 0;
	bool bw = true;	// byte width

	if (dev->pSspReg->CR0 & 0x8)
	{
		// 9-16 bits data
		bw = false;
	}

	while (DataLen > 0)//cnt)
	{
		if (LpcSSPWaitTxFifo(dev, 100000))
		{
			if (bw) 
			{
				dev->pSspReg->DR = *pData & 0xff;
				pData++;
				cnt++;
				DataLen--;

			}
			else
			{
				dev->pSspReg->DR = *(uint16_t *)pData & LPCSSP_DR_MASK;
				pData += 2;
				cnt += 2;
				DataLen -= 2;
			}
			//if (LpcSSPWaitRxFifo(dev, 100000))
			{
				dev->pSspReg->DR;
			}
		}
	}

	return cnt;
}

// Stop transmit
void LpcSSPStopTx(DevIntrf_t *pDev)
{

}

// Transmit stream
int LpcSSPTx(DevIntrf_t *pDev, uint32_t DevAddr, uint8_t *pData, int DataLen)
{
	int cnt = 0;

	if (LpcSSPStartTx(pDev, DevAddr))
	{
		cnt = LpcSSPTxData(pDev, pData, DataLen);
		LpcSSPStopTx(pDev);
	}

	return cnt;
}

bool LpcSSPWaitBusy(SSPDEV *pDev, int TimeoutCnt)
{
	do {
		if ((pDev->pSspReg->SR & LPCSSP_SR_BSY) == 0)
			return true;
	} while (--TimeoutCnt > 0);

	return false;
}

bool LpcSSPWaitRxFifo(SSPDEV *pDev, int TimeoutCnt)
{
	do
	{
		if (pDev->pSspReg->SR & LPCSSP_SR_RNE)
			return true;
	} while (--TimeoutCnt > 0);

	return false;
}

bool LpcSSPWaitTxFifo(SSPDEV *pDev, int TimeoutCnt)
{
	do
	{
		if (pDev->pSspReg->SR & LPCSSP_SR_TNF)
			return true;
	} while (--TimeoutCnt > 0);

	return false;
}



