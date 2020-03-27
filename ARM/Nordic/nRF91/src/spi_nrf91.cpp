/**-------------------------------------------------------------------------
@file	spi_nrf91.cpp

@brief	SPI implementation on nRF91 series MCU

Note : UART/I2C/SPI are shared.

@author	Hoang Nguyen Hoan
@date	Mar. 26, 2020

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
#include "nrf.h"

#include "istddef.h"
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "coredev/shared_irq.h"

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	union {
		NRF_SPIM_Type *pReg;	// Master DMA register map
		NRF_SPIS_Type *pSReg;// Slave DMA register map
	};
} NRF91_SPIDEV;
#pragma pack(pop)

#define NRF91_SPI_MAXDEV			4
#define NRF91_SPISLAVE_MAXDEV		4
#define NRF91_SPI_DMA_MAXCNT		((1<<12)-1)

static NRF91_SPIDEV s_nRF91SPIDev[NRF91_SPI_MAXDEV] = {
	{
		0, NULL, (NRF_SPIM_Type*)NRF_SPIM0_S_BASE
	},
	{
		1, NULL, (NRF_SPIM_Type*)NRF_SPIM1_S_BASE
	},
	{
		2, NULL, (NRF_SPIM_Type*)NRF_SPIM2_S_BASE
	},
	{
		3, NULL, (NRF_SPIM_Type*)NRF_SPIM3_S_BASE
	},
};

bool nRF52SPIWaitDMA(NRF91_SPIDEV * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pReg->EVENTS_END)
		{
			pDev->pReg->EVENTS_END = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRF91SPIWaitRX(NRF91_SPIDEV * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pReg->EVENTS_ENDRX)
		{
			pDev->pReg->EVENTS_ENDRX = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

int nRF91SPIGetRate(DEVINTRF * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NRF91_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
int nRF91SPISetRate(DEVINTRF * const pDev, int DataRate)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev->pDevData;

	if (DataRate < 250000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K125;
		dev->pSpiDev->Cfg.Rate = 125000;
	}
	else if (DataRate < 500000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K250;
		dev->pSpiDev->Cfg.Rate = 250000;
	}
	else if (DataRate < 1000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K500;
		dev->pSpiDev->Cfg.Rate = 500000;
	}
	else if (DataRate < 2000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M1;
		dev->pSpiDev->Cfg.Rate = 1000000;
	}
	else if (DataRate < 4000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M2;
		dev->pSpiDev->Cfg.Rate = 2000000;
	}
	else if (DataRate < 8000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M4;
		dev->pSpiDev->Cfg.Rate = 4000000;
	}
	else
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
		dev->pSpiDev->Cfg.Rate = 8000000;
	}

	return dev->pSpiDev->Cfg.Rate;
}

void nRF91SPIDisable(DEVINTRF * const pDev)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Type == SPITYPE_SLAVE)
	{
		dev->pSReg->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
	}
	else
	{
		dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
	}
}

void nRF91SPIEnable(DEVINTRF * const pDev)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Type == SPITYPE_SLAVE)
	{
		dev->pSReg->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
	}
	else
	{
		dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	}
}

void nRF91SPIPowerOff(DEVINTRF * const pDev)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev->pDevData;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 0;
}

// Initial receive
bool nRF91SPIStartRx(DEVINTRF * const pDev, int DevCs)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
	{
        dev->pReg->PSEL.MISO = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		dev->pReg->PSEL.MOSI = -1;
	}

	return true;
}

// Receive Data only, no Start/Stop condition
int nRF91SPIRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

    dev->pReg->TXD.PTR = 0;
    dev->pReg->TXD.MAXCNT = 0;
    dev->pReg->TXD.LIST = 0;
    dev->pReg->RXD.PTR = (uint32_t)pBuff;
    dev->pReg->RXD.LIST = SPIM_RXD_LIST_LIST_ArrayList << SPIM_RXD_LIST_LIST_Pos;

    while (BuffLen > 0)
	{
		int l = min(BuffLen, NRF91_SPI_DMA_MAXCNT);

		dev->pReg->RXD.MAXCNT = l;
		dev->pReg->EVENTS_END = 0;
		dev->pReg->EVENTS_ENDRX = 0;
		dev->pReg->TASKS_START = 1;

		if (nRF91SPIWaitRX(dev, 100000) == false)
			break;

        l = dev->pReg->RXD.AMOUNT;
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}

	return cnt;
}

// Stop receive
void nRF91SPIStopRx(DEVINTRF * const pDev)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
	{
//		dev->pReg->PSEL.MOSI = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
	}
}

// Initiate transmit
bool nRF91SPIStartTx(DEVINTRF * const pDev, int DevCs)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

    if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
    {
        dev->pReg->PSEL.MOSI = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        dev->pReg->PSEL.MISO = -1;
    }
	return true;
}

// Transmit Data only, no Start/Stop condition
int nRF91SPITxData(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	dev->pReg->RXD.PTR = 0;
    dev->pReg->RXD.MAXCNT = 0;
    dev->pReg->RXD.LIST = 0;
    dev->pReg->TXD.PTR = (uint32_t)pData;
    dev->pReg->TXD.LIST = SPIM_TXD_LIST_LIST_ArrayList << SPIM_TXD_LIST_LIST_Pos;

    while (DataLen > 0)
	{
		int l = min(DataLen, NRF91_SPI_DMA_MAXCNT);
		dev->pReg->TXD.MAXCNT = l;
		dev->pReg->EVENTS_END = 0;
		dev->pReg->EVENTS_ENDTX = 0;
		dev->pReg->TASKS_START = 1;

		if (nRF52SPIWaitDMA(dev, 100000) == false)
		{
		    break;
		}

		l = dev->pReg->TXD.AMOUNT;
		DataLen -= l;
		pData += l;
		cnt += l;
	}

	return cnt;
}

// Stop transmit
void nRF91SPIStopTx(DEVINTRF * const pDev)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
    if (dev->pSpiDev->Cfg.Mode == SPIMODE_3WIRE)
    {
        dev->pReg->PSEL.MOSI = -1;
        dev->pReg->PSEL.MISO = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
    }
}

void SPIIrqHandler(int DevNo, DEVINTRF * const pDev)
{
	NRF91_SPIDEV *dev = (NRF91_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.Type == SPITYPE_SLAVE)
	{
		if (dev->pSReg->EVENTS_ENDRX)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_RX_FIFO_FULL, NULL, 0);
			}
			dev->pSReg->EVENTS_ENDRX = 0;
			dev->pSReg->STATUS = dev->pSReg->STATUS;
		}

		if (dev->pSReg->EVENTS_END)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_COMPLETED, (uint8_t*)dev->pSReg->RXD.PTR, dev->pSReg->RXD.AMOUNT);
			}
			dev->pSReg->EVENTS_END = 0;
		}

		if (dev->pSReg->EVENTS_ACQUIRED)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_STATECHG, NULL, 0);
			}
			dev->pSReg->STATUS = dev->pSReg->STATUS;

			dev->pSReg->RXD.PTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
			dev->pSReg->RXD.MAXCNT = dev->pSpiDev->RxBuffLen[0];
			dev->pSReg->TXD.PTR = (uint32_t)dev->pSpiDev->pTxData[0];
			dev->pSReg->TXD.MAXCNT = dev->pSpiDev->TxDataLen[0];
			dev->pSReg->EVENTS_ACQUIRED = 0;
			dev->pSReg->TASKS_RELEASE = 1;
		}
	}
	else
	{
		// Master
		if (dev->pSReg->EVENTS_ENDRX)
		{
//			dev->pSReg->EVENTS_ENDRX = 0;
//			dev->pSReg->STATUS = dev->pSReg->STATUS;
		}
	}
}

bool SPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	NRF_SPIM_Type *reg;
	uint32_t cfgreg = 0;

	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->Type == SPITYPE_SLAVE && pCfgData->DevNo >= NRF91_SPISLAVE_MAXDEV)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= NRF91_SPI_MAXDEV || pCfgData->NbIOPins < 3)
	{
		return false;
	}

	// Get the correct register map
	reg = s_nRF91SPIDev[pCfgData->DevNo].pReg;

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)s_nRF91SPIDev[pCfgData->DevNo].pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)s_nRF91SPIDev[pCfgData->DevNo].pReg + 0xFFC) = 1;

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfgreg |= SPIM_CONFIG_ORDER_LsbFirst;
	}
	else
	{
		cfgreg |= SPIM_CONFIG_ORDER_MsbFirst;
	}

	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfgreg |= (SPIM_CONFIG_CPHA_Trailing   << SPIM_CONFIG_CPHA_Pos);
	}
	else
	{
		cfgreg |= (SPIM_CONFIG_CPHA_Leading    << SPIM_CONFIG_CPHA_Pos);
	}

	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cfgreg |= (SPIM_CONFIG_CPOL_ActiveLow  << SPIM_CONFIG_CPOL_Pos);
		IOPinSet(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}
	else
	{
		cfgreg |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos);
		IOPinClear(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}

	reg->CONFIG = cfgreg;

	pDev->Cfg = *pCfgData;
	s_nRF91SPIDev[pCfgData->DevNo].pSpiDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_nRF91SPIDev[pCfgData->DevNo];

	nRF91SPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = nRF91SPIDisable;
	pDev->DevIntrf.Enable = nRF91SPIEnable;
	pDev->DevIntrf.GetRate = nRF91SPIGetRate;
	pDev->DevIntrf.SetRate = nRF91SPISetRate;
	pDev->DevIntrf.StartRx = nRF91SPIStartRx;
	pDev->DevIntrf.RxData = nRF91SPIRxData;
	pDev->DevIntrf.StopRx = nRF91SPIStopRx;
	pDev->DevIntrf.StartTx = nRF91SPIStartTx;
	pDev->DevIntrf.TxData = nRF91SPITxData;
	pDev->DevIntrf.StopTx = nRF91SPIStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = nRF91SPIPowerOff;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	// Only DMA mode avail
	pDev->DevIntrf.bDma = true;

	uint32_t inten = 0;

    if (pCfgData->Type == SPITYPE_SLAVE)
	{
		NRF_SPIS_Type *sreg = s_nRF91SPIDev[pCfgData->DevNo].pSReg;

		sreg->PSEL.SCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
        sreg->PSEL.MISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        sreg->PSEL.MOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);
		sreg->PSEL.CSN = (pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX].PortNo << 5);
		sreg->ORC = 0xFF;
		sreg->STATUS = sreg->STATUS;
		sreg->EVENTS_ENDRX = 0;
		sreg->EVENTS_END = 0;
		sreg->EVENTS_ACQUIRED = 0;
		sreg->DEF = 0xFF;
		sreg->SHORTS = (SPIS_SHORTS_END_ACQUIRE_Enabled << SPIS_SHORTS_END_ACQUIRE_Pos);

		inten = (SPIS_INTENSET_ACQUIRED_Enabled << SPIS_INTENSET_ACQUIRED_Pos) |
				(SPIS_INTENSET_ENDRX_Enabled << SPIS_INTENSET_ENDRX_Pos) |
				(SPIS_INTENSET_END_Enabled << SPIS_INTENSET_END_Pos);
		reg->ENABLE =  (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
		sreg->TASKS_ACQUIRE = 1;	// Active event to update rx/tx buffer
	}
	else
	{
		reg->PSEL.SCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
		reg->PSEL.MISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		reg->PSEL.MOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);

		if (pDev->DevIntrf.bDma == true)
		{
			s_nRF91SPIDev[pCfgData->DevNo].pReg->ORC = 0xFF;
			s_nRF91SPIDev[pCfgData->DevNo].pReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
		}

        //s_nRF91SPIDev[pCfgData->DevNo].pReg->EVENTS_READY = 0;
	}

    if (pCfgData->bIntEn)
    {
    	SetSharedIntHandler(pCfgData->DevNo, &pDev->DevIntrf, SPIIrqHandler);

    	switch (pCfgData->DevNo)
    	{
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
    	}

    	reg->INTENSET = inten;
    }

	return true;
}




