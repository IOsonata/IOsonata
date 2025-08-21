/**-------------------------------------------------------------------------
@file	i2c_nrfx.cpp

@brief	I2C implementation on nRFx series MCU

Note: I2C device are shared with other device such as SPI therefore be careful
not to use the same device number on an other device.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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
#include "nrf.h"
#include "nrf_peripherals.h"


#include "istddef.h"
#include "cfifo.h"
#include "coredev/i2c.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "coredev/interrupt.h"
#include "coredev/shared_intrf.h"

#ifdef TWIM_PRESENT
#define NRFX_I2C_MAXDEV				TWIM_COUNT
#else
#define NRFX_I2C_MAXDEV				TWI_COUNT
#endif

#ifdef TWIS_PRESENT
#define NRFX_I2CSLAVE_MAXDEV		TWIS_COUNT
#else
#define NRFX_I2CSLAVE_MAXDEV		0
#endif

#define NRFX_I2C_DMA_MAXCNT			((1<<TWIS0_EASYDMA_MAXCNT_SIZE)-1)

#define NRFX_I2C_TRBUFF_SIZE	4

// Default fifo size if one is not provided is not provided in the config.
#define NRFX_I2C_BUFF_SIZE			8

#define NRFX_I2C_CFIFO_SIZE			CFIFO_MEMSIZE(NRFX_I2C_BUFF_SIZE)

//void I2CIrqHandler(int DevNo, DevIntrf_t * const pDev);

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	I2CDev_t *pI2cDev;
	union {
#ifdef TWI_PRESENT
		NRF_TWI_Type *pReg;		// Master register map
#endif
#ifdef TWIM_PRESENT
		NRF_TWIM_Type *pDmaReg;	// Master DMA register map
		NRF_TWIS_Type *pDmaSReg;// Slave DMA register map
#endif
	};
	int RxBufflen;
	int RxIdx;
	uint8_t *pRxBuff;
	int TxDatalen;
	int TxIdx;
	uint8_t *pTxData;
} nRFTwiDev_t;

typedef struct {
	uint32_t Freq;
	uint32_t RegVal;
} nRFTwiFreq_t;

#pragma pack(pop)

static const nRFTwiFreq_t s_nRFxI2CFreq[] = {
#ifdef TWIM_PRESENT
	{100000, TWIM_FREQUENCY_FREQUENCY_K100},
	{250000, TWIM_FREQUENCY_FREQUENCY_K250},
	{400000, TWIM_FREQUENCY_FREQUENCY_K400},
#else
	{100000, TWI_FREQUENCY_FREQUENCY_K100},
	{250000, TWI_FREQUENCY_FREQUENCY_K250},
	{400000, TWI_FREQUENCY_FREQUENCY_K400},
#endif
};

static const int s_NbI2CFreq = sizeof(s_nRFxI2CFreq) / sizeof(nRFTwiFreq_t);

alignas(4) static nRFTwiDev_t s_nRFxI2CDev[NRFX_I2C_MAXDEV] = {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	{
		0, NULL, (NRF_TWIM_Type *)NRF_TWIM0_NS_BASE,
	},
#else
	{
		0, NULL, (NRF_TWIM_Type *)NRF_TWIM0_S_BASE,
	},
	{
		1, NULL, (NRF_TWIM_Type *)NRF_TWIM1_S_BASE,
	},
	{
		2, NULL, (NRF_TWIM_Type *)NRF_TWIM2_S_BASE,
	},
	{
		3, NULL, (NRF_TWIM_Type *)NRF_TWIM3_S_BASE,
	},
#endif
#else
	{
		0, NULL, (NRF_TWI_Type *)NRF_TWI0_BASE,
	},
#if NRFX_I2C_MAXDEV > 1
	{
		1, NULL, (NRF_TWI_Type *)NRF_TWI1_BASE,
	},
#endif
#endif
};

bool nRFxI2CWaitStop(nRFTwiDev_t * const pDev, int Timeout)
{
#ifdef TWIM_PRESENT
	NRF_TWIM_Type *reg = pDev->pDmaReg;
#else
	NRF_TWI_Type *reg = pDev->pReg;
#endif

    do {
        if (reg->EVENTS_ERROR)
        {
            // Abort in case error
            reg->ERRORSRC = reg->ERRORSRC;
            reg->EVENTS_ERROR = 0;
            reg->TASKS_RESUME = 1;
            reg->TASKS_STOP = 1;
            while( !reg->EVENTS_STOPPED );

            return false;
        }
        if (reg->EVENTS_STOPPED)
        {
            // Must wait for stop, other wise DMA count would
            // not be updated with correct value
            reg->EVENTS_STOPPED = 0;

#ifdef TWIM_PRESENT
            pDev->pDmaReg->EVENTS_TXSTARTED = 0;
            pDev->pDmaReg->EVENTS_RXSTARTED = 0;
#endif
            return true;
        }
    } while (Timeout-- >  0);

    return false;
}

bool nRFxI2CWaitRxComplete(nRFTwiDev_t * const pDev, int Timeout)
{
#ifdef TWIM_PRESENT
	NRF_TWIM_Type *reg = pDev->pDmaReg;
#else
	NRF_TWI_Type *reg = pDev->pReg;
#endif

	do {
        if (reg->EVENTS_ERROR)
        {
            while ( !nRFxI2CWaitStop( pDev, Timeout ) );

            return false;
        }
#ifdef TWIM_PRESENT
        if (pDev->pI2cDev->DevIntrf.bDma)
        {
			if (pDev->pDmaReg->EVENTS_LASTRX)
			{
				// Must wait for last DMA then issue a stop
				pDev->pDmaReg->EVENTS_LASTRX = 0;

				return true;
			}
        }
        else
#endif
        {
#ifdef TWI_PRESENT
            if (pDev->pReg->EVENTS_RXDREADY)
            {
            	pDev->pReg->EVENTS_RXDREADY = 0;

                return true;
            }
#endif
        }
    } while (Timeout-- >  0);

    return false;
}

bool nRFxI2CWaitTxComplete(nRFTwiDev_t * const pDev, int Timeout)
{
#ifdef TWIM_PRESENT
	NRF_TWIM_Type *reg = pDev->pDmaReg;
#else
	NRF_TWI_Type *reg = pDev->pReg;
#endif

	do {
        if (reg->EVENTS_ERROR)
        {
            while ( !nRFxI2CWaitStop( pDev, Timeout ) );

            return false;
        }

#ifdef TWIM_PRESENT
        if (pDev->pI2cDev->DevIntrf.bDma)
        {
			if (pDev->pDmaReg->EVENTS_LASTTX)
			{
				// Must wait for last DMA then issue a stop
				pDev->pDmaReg->EVENTS_LASTTX = 0;

				return true;
			}
        }
        else
#endif
        {
#ifdef TWI_PRESENT
            if (pDev->pReg->EVENTS_TXDSENT)
            {
                pDev->pReg->EVENTS_TXDSENT = 0;

                return true;
            }
#endif
        }
    } while (Timeout-- >  0);

    return false;
}

void nRFxI2CDisable(DevIntrf_t * const pDev)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;

#ifdef TWIM_PRESENT
	dev->pDmaReg->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
#else
	dev->pReg->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);
#endif
}

void nRFxI2CEnable(DevIntrf_t * const pDev)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;

#ifdef TWIM_PRESENT
    if (dev->pI2cDev->DevIntrf.bDma)
    {
		if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
		{
			dev->pDmaReg->ENABLE = (TWIS_ENABLE_ENABLE_Enabled << TWIS_ENABLE_ENABLE_Pos);
		}
		else
		{
			dev->pDmaReg->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
		}
    }
    else
#endif
	{
#ifdef TWI_PRESENT
    	dev->pReg->ENABLE = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);
#endif
    }
}

void nRFxI2CPowerOff(DevIntrf_t * const pDev)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;

	// Undocumented Power down I2C.  Nordic Bug with DMA causing high current consumption
#ifdef TWIM_PRESENT
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC) = 0;
#else
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 0;
#endif


	IOPinDisable(dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PortNo, dev->pI2cDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PinNo);
	IOPinDisable(dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PortNo, dev->pI2cDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PinNo);
}

uint32_t nRFxI2CGetRate(DevIntrf_t * const pDev)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;

	return dev->pI2cDev->Cfg.Rate;
}

uint32_t nRFxI2CSetRate(DevIntrf_t * const pDev, uint32_t RateHz)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
	uint32_t regval = 0;

	for (int i = 0; i < s_NbI2CFreq; i++)
	{
		if (s_nRFxI2CFreq[i].Freq <= RateHz)
		{
			regval =  s_nRFxI2CFreq[i].RegVal;
			dev->pI2cDev->Cfg.Rate = s_nRFxI2CFreq[i].Freq;
		}
	}

#ifdef TWIM_PRESENT
	dev->pDmaReg->FREQUENCY = regval;
#else
	dev->pReg->FREQUENCY = regval;
#endif

	return dev->pI2cDev->Cfg.Rate;
}

bool nRFxI2CStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;

#ifdef TWI_PRESENT
	dev->pReg->ADDRESS = DevAddr;
	//dev->pReg->INTENCLR = 0xFFFFFFFF;
#else
	dev->pDmaReg->ADDRESS = DevAddr;
	//dev->pDmaReg->INTENCLR = 0xFFFFFFFF;
#endif

	return true;
}

// Receive Data only, no Start/Stop condition
int nRFxI2CRxDataDMA(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
	uint32_t d;
	int cnt = 0;
/*
	if (pDev->bCmdMode)
	{
		int Timeout = 100000;
		while (pDev->bTxComplete == false && --Timeout > 0);
		pDev->bTxComplete = false;
	}

	pDev->bCmdMode = false;*/

#ifdef TWIM_PRESENT
	while (BuffLen > 0)
	{
		int l = min(BuffLen, NRFX_I2C_DMA_MAXCNT);
		dev->pDmaReg->EVENTS_ERROR = 0;
		dev->pDmaReg->EVENTS_STOPPED = 0;
		dev->pDmaReg->RXD.PTR = (uint32_t)pBuff;
		dev->pDmaReg->RXD.MAXCNT = l;
		dev->pDmaReg->RXD.LIST = 0;
		dev->pDmaReg->SHORTS = TWIM_SHORTS_LASTRX_STOP_Msk;
		dev->pDmaReg->EVENTS_SUSPENDED = 0;
		dev->pDmaReg->TASKS_RESUME = 1;
		dev->pDmaReg->TASKS_STARTRX = 1;

		if (pDev->bIntEn == true)
		{
			return -1;
		}
		if (nRFxI2CWaitRxComplete(dev, 1000000) == false)
		{
			break;
		}
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}
#endif
	return cnt;
}

// Receive Data only, no Start/Stop condition
int nRFxI2CRxData(DevIntrf_t *pDev, uint8_t *pBuff, int Bufflen)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
	int cnt = 0;

	if (pBuff == NULL || Bufflen <= 0)
	{
		return 0;
	}

#ifdef TWI_PRESENT
	if (pDev->bIntEn == true)
	{
		dev->RxBufflen = Bufflen;
		dev->RxIdx = 0;
		dev->pRxBuff = pBuff;

		dev->pReg->SHORTS = TWI_SHORTS_BB_SUSPEND_Msk;
		dev->pReg->EVENTS_SUSPENDED = 0;
		dev->pReg->EVENTS_STOPPED = 0;
		dev->pReg->TASKS_STARTRX = 1;

		cnt = -1;
	}
	else
	{
		dev->pReg->SHORTS = 0;
		dev->pReg->EVENTS_STOPPED = 0;
		dev->pReg->TASKS_STARTRX = 1;

		while (Bufflen > 0)
		{
			if (nRFxI2CWaitRxComplete(dev, 100000) == false)
			{
				break;
			}

			*pBuff = dev->pReg->RXD;

			Bufflen--;
			pBuff++;
			cnt++;

			if (Bufflen == 1)
			{
			    dev->pReg->TASKS_STOP = 1;
			}
		}
	}
#endif

	return cnt;
}

void nRFxI2CStopRx(DevIntrf_t * const pDev)
{
    nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
#ifdef TWIM_PRESENT
	NRF_TWIM_Type *reg = dev->pDmaReg;
#else
	NRF_TWI_Type *reg = dev->pReg;
#endif

//	dev->pReg->SHORTS = TWI_SHORTS_BB_STOP_Msk;
	reg->TASKS_RESUME = 1;
    reg->TASKS_STOP = 1;

	if (pDev->bIntEn == false)
	{
		nRFxI2CWaitStop(dev, 1000000);
	}
}

bool nRFxI2CStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;

#ifdef TWI_PRESENT
	dev->pReg->ADDRESS = DevAddr;
	NRF_TWI_Type *reg = dev->pReg;
#else
	dev->pDmaReg->ADDRESS = DevAddr;
	NRF_TWIM_Type *reg = dev->pDmaReg;
#endif
    if (reg->EVENTS_ERROR)
    {
    	reg->ERRORSRC = reg->ERRORSRC;
    	reg->EVENTS_ERROR = 0;
    }
	return true;
}

// Send Data only, no Start/Stop condition
int nRFxI2CTxDataDMA(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
	uint32_t d;
	int cnt = 0;

#ifdef TWIM_PRESENT
	while (DataLen > 0)
	{
		int l = min(DataLen, NRFX_I2C_DMA_MAXCNT);

		pDev->bTxReady = false;

		dev->pDmaReg->EVENTS_ERROR = 0;
		dev->pDmaReg->EVENTS_STOPPED = 0;
	    if (dev->pI2cDev->DevIntrf.bDma)
	    {
			dev->pDmaReg->TXD.PTR = (uint32_t)pData;
			dev->pDmaReg->TXD.MAXCNT = l;
			dev->pDmaReg->TXD.LIST = 0;
	    }
		dev->pDmaReg->SHORTS = (TWIM_SHORTS_LASTTX_SUSPEND_Enabled << TWIM_SHORTS_LASTTX_SUSPEND_Pos);
		dev->pDmaReg->EVENTS_SUSPENDED = 0;
		dev->pDmaReg->TASKS_RESUME = 1;
		dev->pDmaReg->TASKS_STARTTX = 1;

		if (pDev->bIntEn == false || pDev->bNoStop)
		{
			if (nRFxI2CWaitTxComplete(dev, 100000) == false)
				break;
		}
		else
		{
			return -1;
		}
		DataLen -= l;
		pData += l;
		cnt += l;
	}
#endif
	return cnt;
}

int nRFxI2CTxData(DevIntrf_t * const pDev, uint8_t *pData, int Datalen)
{
	nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
	uint32_t d;
	int cnt = 0;
	int rtry = pDev->MaxRetry;

	//atomic_store(&pDev->bTxComplete, false);
#ifdef TWI_PRESENT
	dev->pReg->SHORTS = 0;
	dev->pReg->EVENTS_STOPPED = 0;
	dev->pReg->TASKS_STARTTX = 1;

	if (pDev->bIntEn == true)
	{
		dev->TxDatalen = Datalen;
		dev->TxIdx = 0;
		dev->pTxData = pData;

		//g_Uart.printf("TxData %d\r\n", Datalen);
		if (pDev->bTxReady == true)
		{
			//g_Uart.printf("iTx %x %d\r\n", dev->pTxData[dev->TxIdx], dev->pReg->EVENTS_RXDREADY);
			pDev->bTxReady = false;
			dev->pReg->TXD = dev->pTxData[dev->TxIdx];

			if (pDev->bNoStop == true)
			{
				//if (nRFxI2CWaitTxComplete(dev, 100000) == false)
				{
				//	break;
				}
			}
		}

		return -1;
	}
	else
	{

		pDev->bTxReady = false;
		while (Datalen > 0 && rtry > 0)
		{
//			g_Uart.printf("Tx %d %x\r\n", Datalen, *pData);
			dev->pReg->TXD = *pData;
			if (nRFxI2CWaitTxComplete(dev, 100000) == false)
			{
				//g_Uart.printf("Tx Failed\r\n");
				rtry--;
				continue;
			}
			Datalen--;
			cnt++;
			pData++;
		}
		pDev->bTxReady = true;
	}
#endif
	return cnt;
}

void nRFxI2CStopTx(DevIntrf_t * const pDev)
{
    nRFTwiDev_t *dev = (nRFTwiDev_t*)pDev->pDevData;
#ifdef TWIM_PRESENT
	NRF_TWIM_Type *reg = dev->pDmaReg;

	if (dev->pI2cDev->DevIntrf.bDma)
    {
		if (reg->EVENTS_LASTTX == 1)
		{
			reg->EVENTS_LASTTX = 0;
		}
    }
#else
	NRF_TWI_Type *reg = dev->pReg;
#endif

	reg->EVENTS_SUSPENDED = 0;
	reg->TASKS_RESUME = 1;
    reg->TASKS_STOP = 1;

    if (pDev->bIntEn == false)
    {
    	nRFxI2CWaitStop(dev, 1000000);

    }
}

void nRFxI2CReset(DevIntrf_t * const pDev)
{
	I2CDev_t *dev = (I2CDev_t*)((nRFTwiDev_t*)pDev->pDevData)->pI2cDev;

    I2CBusReset(dev);

#if 0
    nRFxI2CDisable(pDev);

    IOPinConfig(0, dev->pReg->PSELSCL, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    IOPinConfig(0, dev->pReg->PSELSDA, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    IOPinSet(0, dev->pReg->PSELSDA);

    for (int i = 0; i < 10; i++)
    {
        IOPinSet(0, dev->pReg->PSELSCL);
        usDelay(5);
        IOPinClear(0, dev->pReg->PSELSCL);
        usDelay(5);
    }
    IOPinConfig(0, dev->pReg->PSELSDA, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    IOPinClear(0, dev->pReg->PSELSDA);
    usDelay(5);
    IOPinSet(0, dev->pReg->PSELSCL);
    usDelay(2);
    IOPinSet(0, dev->pReg->PSELSDA);

    nRFxI2CEnable(pDev);
#endif
}

void *nRFxI2CGetHandle(DevIntrf_t * const pDev)
{
	I2CDev_t *dev = (I2CDev_t*)((nRFTwiDev_t*)pDev->pDevData)->pI2cDev;

	return dev;
}

void I2CSetReadRqstData(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen) {
	if (SlaveIdx < 0 || SlaveIdx >= I2C_SLAVEMODE_MAX_ADDR || pDev == NULL)
		return;

	pDev->pRRData[SlaveIdx] = pData;
	pDev->RRDataLen[SlaveIdx] = DataLen;

    nRFTwiDev_t *nrfdev = (nRFTwiDev_t*)pDev->DevIntrf.pDevData;

#ifdef TWIM_PRESENT
    nrfdev->pDmaSReg->TXD.PTR = (uint32_t)pDev->pRRData[SlaveIdx];
    nrfdev->pDmaSReg->TXD.MAXCNT = pDev->RRDataLen[SlaveIdx] & 0xFF;
    nrfdev->pDmaSReg->TASKS_PREPARETX = 1;
    nrfdev->pDmaSReg->TASKS_RESUME = 1;
#endif
}

void I2C_IRQHandler(int DevNo, DevIntrf_t * const pDev)
//extern "C" void I2C_IRQHandler(int DevNo)
{
    nRFTwiDev_t *dev = &s_nRFxI2CDev[DevNo];//(nRFTwiDev_t*)&pDev->DevIntrf.pDevData;

    if (dev->pI2cDev->Cfg.Mode == I2CMODE_SLAVE)
    {
    	// Slave mode
#ifdef TWIM_PRESENT
    	if (dev->pDmaSReg->EVENTS_READ)
    	{
    		// Read command received
    	    int cnt = 0;

    		if (dev->pI2cDev->DevIntrf.EvtCB)
    		{
    			int len = dev->pDmaSReg->EVENTS_RXSTARTED ? dev->pDmaSReg->RXD.AMOUNT : 0;

    			cnt = dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_READ_RQST, NULL, len);
    		}
    		dev->pDmaSReg->EVENTS_RXSTARTED = 0;
    		dev->pDmaSReg->EVENTS_READ = 0;
/*
    		if (cnt > 0)
    		{
				dev->pDmaSReg->TXD.PTR = (uint32_t)dev->pI2cDev->pRRData[dev->pDmaSReg->MATCH];
				dev->pDmaSReg->TXD.MAXCNT = dev->pI2cDev->RRDataLen[dev->pDmaSReg->MATCH] & 0xFF;
				dev->pDmaSReg->TASKS_PREPARETX = 1;
				dev->pDmaSReg->TASKS_RESUME = 1;
    		}*/
    	}

    	if (dev->pDmaSReg->EVENTS_WRITE)
    	{
    		// Write command received

    		if (dev->pI2cDev->DevIntrf.EvtCB)
    		{
    			dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_WRITE_RQST, NULL, 0);
    		}
    		dev->pDmaSReg->EVENTS_WRITE = 0;
    		dev->pDmaSReg->RXD.PTR = (uint32_t)dev->pI2cDev->pTRBuff[dev->pDmaSReg->MATCH];
    		dev->pDmaSReg->RXD.MAXCNT = dev->pI2cDev->TRBuffLen[dev->pDmaSReg->MATCH];
    		//dev->pDmaSReg->SHORTS = 0;
    		dev->pDmaSReg->TASKS_PREPARERX = 1;
    		dev->pDmaSReg->TASKS_RESUME = 1;
    	}

    	if (dev->pDmaSReg->EVENTS_STOPPED)
    	{
    		int len = 0;

    		if (dev->pDmaSReg->EVENTS_RXSTARTED)
    		{
    			len = dev->pDmaSReg->RXD.AMOUNT;
    			dev->pDmaSReg->EVENTS_RXSTARTED = 0;
    		}
    		if (dev->pDmaSReg->EVENTS_TXSTARTED)
    		{
    			len = dev->pDmaSReg->TXD.AMOUNT;
    			dev->pDmaSReg->EVENTS_TXSTARTED = 0;
    		}
    		dev->pDmaSReg->EVENTS_STOPPED = 0;
    		if (dev->pI2cDev->DevIntrf.EvtCB)
    		{
    			dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_COMPLETED, NULL, len);
    		}
    	}
    	if (dev->pDmaSReg->EVENTS_ERROR)
    	{
    		dev->pDmaSReg->EVENTS_ERROR = 0;
    	}

#endif
    }
    else
    {
    	// Master mode
    	// TODO: implement interrupt handling for master mode
    	int len = 0;

#ifdef TWIM_PRESENT
    	NRF_TWIM_Type *reg = dev->pDmaReg;
#else
    	NRF_TWI_Type *reg = dev->pReg;
#endif

        if (reg->EVENTS_ERROR)
        {
            // Abort in case error
            printf("error %x\r\n", reg->ERRORSRC);
            reg->ERRORSRC = reg->ERRORSRC;
            reg->EVENTS_ERROR = 0;
            reg->TASKS_RESUME = 1;
            reg->TASKS_STOP = 1;
            while( !reg->EVENTS_STOPPED );
        }

        if (reg->EVENTS_STOPPED)
        {
            reg->EVENTS_STOPPED = 0;

			len = dev->RxIdx;
#ifdef TWIM_PRESENT

            reg->EVENTS_TXSTARTED = 0;
			if (reg->EVENTS_RXSTARTED)
			{
				len = dev->pDmaReg->RXD.AMOUNT;
				//DeviceIntrfRxComplete(pDev);
	            reg->EVENTS_RXSTARTED = 0;
			}
#endif
    		if (dev->pI2cDev->DevIntrf.EvtCB)
    		{
    			dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_COMPLETED, NULL, len);
    		}
        }

#ifdef TWIM_PRESENT
        if (pDev->bDma)
        {
			if (reg->EVENTS_LASTTX)
			{
				// Must wait for last DMA then issue a stop
				//DeviceIntrfTxComplete(pDev);
				reg->EVENTS_LASTTX = 0;

				DeviceIntrfTxComplete(pDev);
/*
				if (pDev->bNoStop == false)
				{
					DeviceIntrfStopTx(pDev);
					atomic_store(&pDev->bTxComplete, true);

//					pDev->bTxComplete = true;
				}
*/
				if (dev->pI2cDev->DevIntrf.EvtCB)
				{
					dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_TX_READY, NULL, len);
				}
/*
				if (pDev->bCmdMode == false)
				{
					reg->EVENTS_SUSPENDED = 0;
					reg->TASKS_RESUME = 1;
				    reg->TASKS_STOP = 1;
				}
				else
				{

				}*/
			}
			if (reg->EVENTS_LASTRX)
			{
				// Must wait for last DMA then issue a stop
				reg->EVENTS_LASTRX = 0;
				//DeviceIntrfRxComplete(pDev);

				if (pDev->bNoStop == false)
				{
					//printf("Stop RX\r\n");
					DeviceIntrfStopRx(pDev);
				}
				else
				{

				}
			}
        }
        else
#endif
        {
#ifdef TWI_PRESENT

            if (dev->pReg->EVENTS_TXDSENT)
            {
				dev->pReg->EVENTS_TXDSENT = 0;
				dev->TxIdx++;
				dev->TxDatalen--;

				if (dev->TxDatalen > 0)
				{
					dev->pReg->TXD = dev->pTxData[dev->TxIdx];
				}
				else
				{
					if (dev->pI2cDev->DevIntrf.EvtCB)
					{
						dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_TX_FIFO_EMPTY, NULL, dev->TxIdx);
					}
					if (pDev->bNoStop == false)
					{
						//g_Uart.printf("TxStop\r\n");
						DeviceIntrfStopTx(pDev);
					}
					atomic_store(&pDev->bTxReady, true);
				}
				//if (dev->pI2cDev->DevIntrf.EvtCB)
				//{
				//	dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_TX_READY, NULL, dev->TxIdx - 1);
				//}
            }
            if (dev->pReg->EVENTS_RXDREADY)
            {
				dev->pReg->EVENTS_RXDREADY = 0;
				dev->pRxBuff[dev->RxIdx] = dev->pReg->RXD;

           		dev->RxBufflen--;
           		dev->RxIdx++;

           		//while (dev->pReg->EVENTS_SUSPENDED == 0 && dev->pReg->EVENTS_STOPPED == 0);

           		dev->pReg->EVENTS_SUSPENDED = 0;
				if (dev->RxBufflen > 1)
        		{
					dev->pReg->EVENTS_STOPPED = 0;
					dev->pReg->TASKS_RESUME = 1;
        		}
				else if (dev->RxBufflen > 0)
        		{
    				if (pDev->bNoStop == false)
    				{
    					//g_Uart.printf("StopRx\r\n");
    					//dev->bRxReady = false;
    					DeviceIntrfStopRx(pDev);
    				}
        		}
				if (dev->pI2cDev->DevIntrf.EvtCB)
				{
					//uint32_t d = dev->pReg->RXD;
					//int cnt = CFifoUsed(dev->hRxFifo);
					//dev->pI2cDev->DevIntrf.EvtCB(&dev->pI2cDev->DevIntrf, DEVINTRF_EVT_RX_DATA, (uint8_t*)&d, dev->RxIdx);
				}
            }
#endif
        }
    }

}

bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

    if (pCfgData->Mode == I2CMODE_SLAVE && pCfgData->DevNo >= NRFX_I2CSLAVE_MAXDEV)
    {
    	return false;
    }

    if (pCfgData->DevNo < 0 || pCfgData->DevNo >= NRFX_I2C_MAXDEV)
	{
		return false;
	}

	// Get the correct register map
#ifdef TWIM_PRESENT
	NRF_TWIM_Type *reg = s_nRFxI2CDev[pCfgData->DevNo].pDmaReg;
#else
	NRF_TWI_Type *reg = s_nRFxI2CDev[pCfgData->DevNo].pReg;
#endif

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;

	memcpy(&pDev->Cfg, pCfgData, sizeof(I2CCfg_t));

	// Configure I/O pins
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);
    //IOPinSet(pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PortNo, pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PinNo);
    //IOPinSet(pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PortNo, pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PinNo);

#ifdef TWIM_PRESENT
    reg->PSEL.SCL = (pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PortNo << 5);
    reg->PSEL.SDA = (pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PortNo << 5);
#else
    reg->PSELSCL = (pDev->Cfg.pIOPinMap[I2C_SCL_IOPIN_IDX].PinNo & 0x1f);
    reg->PSELSDA = (pDev->Cfg.pIOPinMap[I2C_SDA_IOPIN_IDX].PinNo & 0x1f);
#endif

	s_nRFxI2CDev[pCfgData->DevNo].pI2cDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_nRFxI2CDev[pCfgData->DevNo];


	nRFxI2CSetRate(&pDev->DevIntrf, pCfgData->Rate);

	s_nRFxI2CDev[pCfgData->DevNo].RxBufflen = 0;
	s_nRFxI2CDev[pCfgData->DevNo].RxIdx = 0;
	s_nRFxI2CDev[pCfgData->DevNo].pRxBuff = NULL;
	s_nRFxI2CDev[pCfgData->DevNo].TxDatalen = 0;
	s_nRFxI2CDev[pCfgData->DevNo].TxIdx = 0;
	s_nRFxI2CDev[pCfgData->DevNo].pTxData = NULL;
	//s_nRFxI2CDev[pCfgData->DevNo].bTxReady = true;
	pDev->DevIntrf.MaxTrxLen = NRFX_I2C_DMA_MAXCNT;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.bIntEn = pCfgData->bIntEn;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Disable = nRFxI2CDisable;
	pDev->DevIntrf.Enable = nRFxI2CEnable;
	pDev->DevIntrf.PowerOff = nRFxI2CPowerOff;
	pDev->DevIntrf.GetRate = nRFxI2CGetRate;
	pDev->DevIntrf.SetRate = nRFxI2CSetRate;
	pDev->DevIntrf.StartRx = nRFxI2CStartRx;
	pDev->DevIntrf.StopRx = nRFxI2CStopRx;
	pDev->DevIntrf.StartTx = nRFxI2CStartTx;

	if (pDev->DevIntrf.bDma)
	{
		pDev->DevIntrf.RxData = nRFxI2CRxDataDMA;
		pDev->DevIntrf.TxData = nRFxI2CTxDataDMA;
	}
	else
	{
		pDev->DevIntrf.RxData = nRFxI2CRxData;
		pDev->DevIntrf.TxData = nRFxI2CTxData;
	}
	pDev->DevIntrf.StopTx = nRFxI2CStopTx;
	pDev->DevIntrf.Reset = nRFxI2CReset;
	pDev->DevIntrf.GetHandle = nRFxI2CGetHandle;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;


	//s_nRFxI2CDev[pCfgData->DevNo].hRxFifo = CFifoInit(s_nRFxI2CDev[pCfgData->DevNo].RxFifoMem, NRFX_I2C_CFIFO_SIZE, 1, true);
	//s_nRFxI2CDev[pCfgData->DevNo].hTxFifo = CFifoInit(s_nRFxI2CDev[pCfgData->DevNo].TxFifoMem, NRFX_I2C_CFIFO_SIZE, 1, true);

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->SHORTS = 0;

	// Clear all errors
    if (reg->EVENTS_ERROR)
    {
        reg->ERRORSRC = reg->ERRORSRC;
        reg->EVENTS_ERROR = 0;
        reg->TASKS_RESUME = 1;
        reg->TASKS_STOP = 1;
    }

    usDelay(1000);

#ifdef TWIM_PRESENT
    reg->EVENTS_LASTRX = 0;
    reg->EVENTS_LASTTX = 0;
    reg->EVENTS_RXSTARTED = 0;
    reg->EVENTS_TXSTARTED = 0;
#endif
    reg->EVENTS_SUSPENDED = 0;
    reg->EVENTS_STOPPED = 0;

    uint32_t enval = 0;
    uint32_t inten = 0;

#ifdef TWIS_PRESENT
    if (pCfgData->Mode == I2CMODE_SLAVE)
    {
    	NRF_TWIS_Type *sreg = s_nRFxI2CDev[pCfgData->DevNo].pDmaSReg;
        pDev->Cfg.NbSlaveAddr = min(pCfgData->NbSlaveAddr, NRFX_I2CSLAVE_MAXDEV);

        sreg->CONFIG = 0;
        sreg->ORC = 0xff;

        for (int i = 0; i < pDev->Cfg.NbSlaveAddr; i++)
        {
        	sreg->ADDRESS[i] = (uint32_t)pDev->Cfg.SlaveAddr[i];
        	if (pDev->Cfg.SlaveAddr[i] != 0)
        	{
        		sreg->CONFIG |= 1<<i;
        	}
        }

        // Slave mode only avail in DMA mode
        pDev->DevIntrf.bDma = true;
		pDev->DevIntrf.RxData = nRFxI2CRxDataDMA;
		pDev->DevIntrf.TxData = nRFxI2CTxDataDMA;

		sreg->SHORTS = TWIS_SHORTS_READ_SUSPEND_Msk | TWIS_SHORTS_WRITE_SUSPEND_Msk;
        sreg->EVENTS_READ = 0;
        sreg->EVENTS_WRITE = 0;
        sreg->EVENTS_ERROR = 0;
        sreg->EVENTS_RXSTARTED = 0;
        sreg->EVENTS_TXSTARTED = 0;
        sreg->EVENTS_STOPPED = 0;

        enval = TWIS_ENABLE_ENABLE_Enabled << TWIS_ENABLE_ENABLE_Pos;

        // NOTE: Do not enable RXSTARTED interrupt
        inten = (TWIS_INTEN_READ_Enabled << TWIS_INTEN_READ_Pos) |
        		(TWIS_INTEN_WRITE_Enabled << TWIS_INTEN_WRITE_Pos) |
//				(TWIS_INTEN_RXSTARTED_Enabled << TWIS_INTEN_RXSTARTED_Pos) |
//				(TWIS_INTEN_TXSTARTED_Enabled << TWIS_INTEN_TXSTARTED_Pos) |
				(TWIS_INTEN_ERROR_Enabled << TWIS_INTEN_ERROR_Pos) |
        		(TWIS_INTEN_STOPPED_Enabled << TWIS_INTEN_STOPPED_Pos);
	    sreg->ENABLE = enval;
    }
    else
#endif
    {
#ifdef TWIM_PRESENT

        reg->EVENTS_RXSTARTED = 0;
        reg->EVENTS_TXSTARTED = 0;
        reg->EVENTS_STOPPED = 0;

        if (pDev->DevIntrf.bDma)
		{
			if (pDev->DevIntrf.bIntEn)
			{
				inten = TWIM_INTEN_LASTTX_Msk | TWIM_INTEN_LASTRX_Msk |
						TWIM_INTEN_ERROR_Msk | TWIM_INTEN_STOPPED_Msk;

				//reg->ENABLE = enval;
			}

			enval = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
		    reg->ENABLE = enval;
		}
		else
#endif
		{
#ifdef TWI_PRESENT
			enval = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);
			s_nRFxI2CDev[pCfgData->DevNo].pReg->EVENTS_RXDREADY = 0;
			s_nRFxI2CDev[pCfgData->DevNo].pReg->ENABLE = enval;
			// Interrupt not available in non DMA
//			pDev->DevIntrf.bIntEn = false;
			if (pDev->DevIntrf.bIntEn)
			{
				inten = TWI_INTENSET_TXDSENT_Msk | TWI_INTENSET_RXDREADY_Msk |
						TWI_INTENSET_ERROR_Msk | TWI_INTENSET_STOPPED_Msk | TWI_INTENSET_SUSPENDED_Msk;

				//reg->ENABLE = enval;
			}
#endif
		}

    }

    if (inten != 0)
    {
    	SharedIntrfSetIrqHandler(pCfgData->DevNo, &pDev->DevIntrf, I2C_IRQHandler);

    	switch (pCfgData->DevNo)
    	{
#ifdef NRF91_SERIES
    		case 0:
                NVIC_ClearPendingIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
                NVIC_SetPriority(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
                NVIC_SetPriority(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
                NVIC_SetPriority(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
                break;
    	    case 3:
                NVIC_ClearPendingIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
                NVIC_SetPriority(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
                break;
#elif defined(NRF53_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                break;
#ifdef NRF5340_XXAA_APPLICATION
    	    case 1:
                NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                NVIC_SetPriority(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                break;
    	    case 3:
                NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                NVIC_SetPriority(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                break;
#endif
#elif defined(NRF52805_XXAA) || defined(NRF52810_XXAA)
    	    case 0:
				NVIC_ClearPendingIRQ(TWIM0_TWIS0_TWI0_IRQn);
				NVIC_SetPriority(TWIM0_TWIS0_TWI0_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(TWIM0_TWIS0_TWI0_IRQn);
				break;
#elif defined(NRF52_SERIES)
    	    case 0:
				NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
				NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
				break;
    	    case 1:
				NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
				NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
				break;
#endif
    	}
    	reg->INTENSET = inten;
    }

//    reg->ENABLE = enval;

	return true;
}
