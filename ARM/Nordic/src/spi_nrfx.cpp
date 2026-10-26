/**-------------------------------------------------------------------------
@file	spi_nrfx.cpp

@brief	SPI implementation on nRFx series MCU

Note: SPI device are shared with other device such as I2C therefore be careful
not to use the same device number on an other device.

For 3 wire support : Set both MISO & MOSI to the same pin.
For QSPI support where available is the last device of the list.

Master with EasyDMA and bIntEn : TxData and RxData start the first DMA block and
return -1. The interrupt handler starts the following blocks, releases the bus
and reports DEVINTRF_EVT_COMPLETED with the number of bytes moved (the receive
buffer for a receive). A transmit that keeps the bus for a following receive
(bNoStop) only sets bTxReady.

Master without EasyDMA and bIntEn : the interrupt handler moves one byte per
READY event and ends the transfer the same way.

A transmit of up to NRFX_SPI_TXBUF_SIZE bytes is sent from a copy, so the
caller's buffer may be released as soon as TxData returns. A longer transmit
waits for completion or cancellation before TxData returns. Each DMA block
uses the same owned buffer. If the wait expires, no more caller bytes are read.
The transfer ends with DEVINTRF_EVT_TX_TIMEOUT after STOPPED (READY without EasyDMA).
Until that event the interface and its copy buffer remain busy.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2016

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
#if defined(NRF5340_XXAA_NETWORK)
#define NRF53_SERIAL0_IRQ SERIAL0_IRQn
#else
#define NRF53_SERIAL0_IRQ SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn
#endif

#include "nrf.h"

#include "istddef.h"
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "coredev/shared_intrf.h"

#include "spi_nrfx.h"
#include "nrf_erratas.h"

#ifdef NRF54L_SERIES
#define NRFX_SPI_RX(reg) ((reg)->DMA.RX)
#define NRFX_SPI_TX(reg) ((reg)->DMA.TX)
#define NRFX_SPI_ENDRX(reg) ((reg)->EVENTS_DMA.RX.END)
#define NRFX_SPI_ENDTX(reg) ((reg)->EVENTS_DMA.TX.END)
#else
#define NRFX_SPI_RX(reg) ((reg)->RXD)
#define NRFX_SPI_TX(reg) ((reg)->TXD)
#define NRFX_SPI_ENDRX(reg) ((reg)->EVENTS_ENDRX)
#define NRFX_SPI_ENDTX(reg) ((reg)->EVENTS_ENDTX)
#endif

#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
#define NRFX_SPI_RX_LIST 0
#define NRFX_SPI_TX_LIST 0
#else
#define NRFX_SPI_RX_LIST (SPIM_RXD_LIST_LIST_ArrayList << SPIM_RXD_LIST_LIST_Pos)
#define NRFX_SPI_TX_LIST (SPIM_TXD_LIST_LIST_ArrayList << SPIM_TXD_LIST_LIST_Pos)
#endif
#ifdef NRF54L_SERIES
#define NRFX_SPIS_RX_INT SPIS_INTENSET_DMARXEND_Msk
#else
#define NRFX_SPIS_RX_INT SPIS_INTENSET_ENDRX_Msk
#endif

#define NRFSPI_TIMEOUT			100000

bool nRFxQSPIInit(SPIDev_t * const pDev);
//void SPIIrqHandler(int DevNo, DevIntrf_t * const pDev);

#ifndef NRF54L_SERIES
alignas(4) static const nRFSpiFreq_t s_nRFxSPIFreq[] = {
#ifdef SPIM_PRESENT
		{125000, SPIM_FREQUENCY_FREQUENCY_K125},
		{250000, SPIM_FREQUENCY_FREQUENCY_K250},
		{500000, SPIM_FREQUENCY_FREQUENCY_K500},
		{1000000, SPIM_FREQUENCY_FREQUENCY_M1},
		{2000000, SPIM_FREQUENCY_FREQUENCY_M2},
		{4000000, SPIM_FREQUENCY_FREQUENCY_M4},
		{8000000, SPIM_FREQUENCY_FREQUENCY_M8},
#else
		{125000, SPI_FREQUENCY_FREQUENCY_K125},
		{250000, SPI_FREQUENCY_FREQUENCY_K250},
		{500000, SPI_FREQUENCY_FREQUENCY_K500},
		{1000000, SPI_FREQUENCY_FREQUENCY_M1},
		{2000000, SPI_FREQUENCY_FREQUENCY_M2},
		{4000000, SPI_FREQUENCY_FREQUENCY_M4},
		{8000000, SPI_FREQUENCY_FREQUENCY_M8},
#endif
#if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
		{16000000, SPIM_FREQUENCY_FREQUENCY_M16},
		{32000000, SPIM_FREQUENCY_FREQUENCY_M32},
#endif
};

static const int s_NbnRFxSPIFreq = sizeof(s_nRFxSPIFreq) / sizeof(nRFSpiFreq_t);

#endif

alignas(4) nRFSpiDev_t g_nRFxSPIDev[NRFX_SPI_MAXDEV] = {
#ifdef NRF54L_SERIES
	{ 0, NULL, NRF_SPIM30_S },
	{ 1, NULL, NRF_SPIM20_S },
	{ 2, NULL, NRF_SPIM21_S },
	{ 3, NULL, NRF_SPIM22_S },
	{ 4, NULL, NRF_SPIM00_S },
#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	{ 5, NULL, NRF_SPIM23_S },
	{ 6, NULL, NRF_SPIM24_S },
#endif
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	{
		0, NULL, (NRF_SPIM_Type*)NRF_SPIM0_NS_BASE,
	},
#else
	{
		0, NULL, (NRF_SPIM_Type*)NRF_SPIM0_S_BASE,
	},
	{
		1, NULL, (NRF_SPIM_Type*)NRF_SPIM1_S_BASE,
	},
	{
		2, NULL, (NRF_SPIM_Type*)NRF_SPIM2_S_BASE,
	},
	{
		3, NULL, (NRF_SPIM_Type*)NRF_SPIM3_S_BASE,
	},
#ifdef NRF5340_XXAA_APPLICATION
	{
		4, NULL, (NRF_SPIM_Type *)NRF_SPIM4_S_BASE,
	},
	{
		5, NULL, {.pQSpiReg = (NRF_QSPI_Type*)NRF_QSPI_S_BASE},
	}
#endif
#endif
#else
	{
		0, NULL, (NRF_SPI_Type*)NRF_SPI0_BASE,
	},
#if NRFX_SPI_MAXDEV > 1
	{
		1, NULL, (NRF_SPI_Type*)NRF_SPI1_BASE,
	},
#endif
#if NRFX_SPI_MAXDEV > 2
	{
		2, NULL, {.pDmaReg = (NRF_SPIM_Type*)NRF_SPIM2_BASE},
	},
#endif
#if NRFX_SPI_MAXDEV > 3
	{
		3, NULL, {.pDmaReg = (NRF_SPIM_Type*)NRF_SPIM3_BASE},
	},
#endif
#if NRFX_SPI_MAXDEV > 4
	{
		4, NULL, {.pQSpiReg = (NRF_QSPI_Type*)NRF_QSPI_BASE},
	}
#endif
#endif
};

static const int s_NbnRFxSPIDev = sizeof(g_nRFxSPIDev) / sizeof(nRFSpiDev_t);

static uint32_t nRFxSPIPinSelect(const IOPinCfg_t &Pin)
{
	return Pin.PortNo < 0 || Pin.PinNo < 0 ? UINT32_MAX :
		((uint32_t)Pin.PortNo << 5) | (uint32_t)Pin.PinNo;
}

// Allow at least one CPU cycle per polling iteration for the wire time.
static uint32_t nRFxSPIDmaTimeout(int Length, uint32_t Rate)
{
	uint64_t cycles = (uint64_t)SystemCoreClock * Length * 8 / Rate + NRFSPI_TIMEOUT;
	return cycles > INT32_MAX ? INT32_MAX : (uint32_t)cycles;
}

// Start an interrupt driven transmit. Up to NRFX_SPI_TXBUF_SIZE bytes are
// sent from a copy and the call returns at once. A longer transmit keeps its
// source until completion or cancellation; every DMA block uses TxBuf.
static const uint8_t *nRFxSPIAsyncTxBuff(nRFSpiDev_t * const pDev, const uint8_t *pData, int DataLen)
{
	if (DataLen <= NRFX_SPI_TXBUF_SIZE)
	{
		memcpy(pDev->TxBuf, pData, DataLen);

		return pDev->TxBuf;
	}

	return pData;
}

static void nRFxSPIAsyncTxWait(nRFSpiDev_t * const pDev, int DataLen, volatile bool &Done)
{
	if (DataLen <= NRFX_SPI_TXBUF_SIZE)
	{
		return;
	}

	uint32_t wait = nRFxSPIDmaTimeout(DataLen, pDev->pSpiDev->Cfg.Rate);
	while (Done == false && --wait > 0);

	uint32_t irq = __get_PRIMASK();
	__disable_irq();

	// Completion may have started another transfer from its callback.
	if (Done == false)
	{
		// DMA only uses TxBuf. Detach the caller before returning. A late
		// interrupt may finish the abort, but cannot send another block.
		pDev->bTxAbort = true;
		pDev->bCmdFailed = pDev->pSpiDev->DevIntrf.bNoStop;
		pDev->pTxData = NULL;
		pDev->pTxWait = NULL;
		Done = true;

#ifdef SPIM_PRESENT
		if (pDev->pSpiDev->DevIntrf.bDma)
		{
			pDev->pDmaReg->INTENSET = SPIM_INTENSET_STOPPED_Msk;
			pDev->pDmaReg->TASKS_STOP = 1;
		}
#endif
		// Without DMA, READY ends the byte already in the shift register.
	}

	__set_PRIMASK(irq);
}
#ifdef SPIM_PRESENT
bool nRFxSPIWaitDMA(nRFSpiDev_t * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pDmaReg->EVENTS_END)
		{
			pDev->pDmaReg->EVENTS_END = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
	// Finish or abort DMA before the caller can reuse its buffer.
	pDev->pDmaReg->EVENTS_STOPPED = 0;
	pDev->pDmaReg->TASKS_STOP = 1;
	Timeout = NRFSPI_TIMEOUT;
	while (!pDev->pDmaReg->EVENTS_STOPPED && Timeout-- > 0);
	if (!pDev->pDmaReg->EVENTS_STOPPED)
		pDev->pDmaReg->ENABLE = 0;
#endif

	return false;
}
#endif

#ifdef SPI_PRESENT
bool nRFxSPIWaitReady(nRFSpiDev_t * const pDev, uint32_t Timeout)
{
	do {
        if (pDev->pReg->EVENTS_READY)
        {
            pDev->pReg->EVENTS_READY = 0; // clear event
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}
#endif

#ifdef SPIM_PRESENT
bool nRFxSPIWaitRX(nRFSpiDev_t * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (NRFX_SPI_ENDRX(pDev->pDmaReg))
		{
			NRFX_SPI_ENDRX(pDev->pDmaReg) = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}
#endif

static uint32_t nRFxSPIGetRate(DevIntrf_t * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((nRFSpiDev_t*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
uint32_t nRFxSPISetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;
#ifdef NRF54L_SERIES
	const uint32_t clock = (dev->DevNo == 4 ? SPIM00_CORE_FREQUENCY :
		SPIM20_CORE_FREQUENCY) * 1000000U;
	const uint32_t first = dev->DevNo == 4 ? SPIM00_PRESCALER_DIVISOR_RANGE_MIN :
		SPIM20_PRESCALER_DIVISOR_RANGE_MIN;
	uint32_t divisor = first;
	uint32_t error = UINT32_MAX;
	for (uint32_t d = first; d <= SPIM20_PRESCALER_DIVISOR_RANGE_MAX; d += 2)
	{
		uint32_t freq = clock / d;
		uint32_t diff = freq > Rate ? freq - Rate : Rate - freq;
		if (diff < error)
		{
			error = diff;
			divisor = d;
		}
	}
	dev->pSpiDev->Cfg.Rate = clock / divisor;
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_MASTER)
		dev->pDmaReg->PRESCALER = divisor;
#else
#ifdef NRF5340_XXAA_APPLICATION
	const IOPinCfg_t *pins = dev->pSpiDev->Cfg.pIOPinMap;
	uint32_t limit = dev->DevNo == 4 ? 16000000U : 8000000U;
	bool highSpeed = dev->DevNo == 4 && SystemCoreClock == 128000000U &&
		dev->pSpiDev->Cfg.Phy == SPIPHY_NORMAL &&
		pins[SPI_SCK_IOPIN_IDX].PortNo == 0 && pins[SPI_SCK_IOPIN_IDX].PinNo == 8 &&
		pins[SPI_MOSI_IOPIN_IDX].PortNo == 0 && pins[SPI_MOSI_IOPIN_IDX].PinNo == 9 &&
		pins[SPI_MISO_IOPIN_IDX].PinNo == -1;
	if (highSpeed)
		limit = 32000000U;
	uint32_t error = UINT32_MAX;
	uint32_t regval = s_nRFxSPIFreq[0].RegVal;
	for (int i = 0; i < s_NbnRFxSPIFreq && s_nRFxSPIFreq[i].Freq <= limit; ++i)
	{
		uint32_t freq = s_nRFxSPIFreq[i].Freq;
		uint32_t diff = freq > Rate ? freq - Rate : Rate - freq;
		if (diff < error)
		{
			error = diff;
			regval = s_nRFxSPIFreq[i].RegVal;
			dev->pSpiDev->Cfg.Rate = freq;
		}
	}
	if (dev->DevNo == 4)
	{
		// Only the fixed SCK/MOSI pins use the dedicated 32-Mbps path.
		// CS remains software-controlled by the generic SPI transaction.
		for (int i = SPI_SCK_IOPIN_IDX; i <= SPI_MOSI_IOPIN_IDX; i += 2)
		{
			int pin = pins[i].PinNo;
			if (pins[i].PortNo == 0 && pin == (i == SPI_SCK_IOPIN_IDX ? 8 : 9))
			{
				uint32_t cnf = NRF_P0_S->PIN_CNF[pin] & ~GPIO_PIN_CNF_MCUSEL_Msk;
				if (dev->pSpiDev->Cfg.Rate == 32000000U)
					{
					cnf &= ~GPIO_PIN_CNF_DRIVE_Msk;
					cnf |= (GPIO_PIN_CNF_MCUSEL_Peripheral << GPIO_PIN_CNF_MCUSEL_Pos) |
						(GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
				}
				NRF_P0_S->PIN_CNF[pin] = cnf;
			}
		}
	}
#else
	uint32_t regval = 0;

	for (int i = 0; i < s_NbnRFxSPIFreq; i++)
	{
		if (s_nRFxSPIFreq[i].Freq <= Rate)
		{
			regval =  s_nRFxSPIFreq[i].RegVal;
			dev->pSpiDev->Cfg.Rate = s_nRFxSPIFreq[i].Freq;
		}
	}

#endif
#ifdef SPI_PRESENT
	dev->pReg->FREQUENCY = regval;
#else
	dev->pDmaReg->FREQUENCY = regval;
#endif

#endif

	return dev->pSpiDev->Cfg.Rate;
}

void nRFxSPIDisable(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pDmaSReg->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
	}
#ifdef SPIM_PRESENT
	else if (pDev->bDma)
	{
		dev->pDmaReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
	}
#endif
#ifdef SPI_PRESENT
	else
	{
        dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
	}
#endif
}

void nRFxSPIEnable(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pDmaSReg->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
	}
#ifdef SPIM_PRESENT
	else if (pDev->bDma)
	{
		dev->pDmaReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	}
#endif
#ifdef SPI_PRESENT
	else
	{
        dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
	}
#endif
}

void nRFxSPIPowerOff(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
	nRFxSPIDisable(pDev);
#else
#ifdef SPIM_PRESENT
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC) = 0;
#else
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 0;
#endif
#endif

//	if (dev->pSpiDev->Cfg.NbIOPins > 3)
	{
		for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
		{
			if (dev->pSpiDev->Cfg.pIOPinMap[i].PortNo != -1 && dev->pSpiDev->Cfg.pIOPinMap[i].PinNo != -1)
			{
				IOPinDisable(dev->pSpiDev->Cfg.pIOPinMap[i].PortNo, dev->pSpiDev->Cfg.pIOPinMap[i].PinNo);
			}
		}
	}
}

// Initial receive
bool nRFxSPIStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

	if (dev->bTxAbort || dev->bCmdFailed)
	{
		return false;
	}

	if (dev->pSpiDev->Cfg.ChipSel != SPICSEL_MAN &&
		DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;

	if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
	{
#ifdef SPIM_PRESENT
        dev->pDmaReg->PSEL.MISO = nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
		dev->pDmaReg->PSEL.MOSI = -1;
#else
        dev->pReg->PSELMISO = nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
		dev->pReg->PSELMOSI = -1;
#endif
	}

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	return true;
}

// Receive Data only, no Start/Stop condition
int nRFxSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;
	int cnt = 0;

	if (dev->bTxAbort || dev->bCmdFailed)
	{
		uint32_t irq = __get_PRIMASK();
		__disable_irq();
		bool pending = dev->bTxAbort;

		dev->bCmdFailed = false;
		__set_PRIMASK(irq);
		return pending ? -1 : 0;
	}

#ifdef SPI_PRESENT
	if (pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	dev->pReg->EVENTS_READY = 0;

	if (pDev->bIntEn == true)
	{
		// The interrupt handler takes each byte, releases the bus and reports
		// DEVINTRF_EVT_COMPLETED.
		dev->pRxBuff = pBuff;
		dev->RxIdx = 0;
		dev->RxBufflen = BuffLen;
		dev->TxDatalen = 0;
		dev->TxIdx = 0;
		cnt = -1;
		dev->pReg->TXD = dev->pSpiDev->Cfg.DummyByte;
	}
	else
	{
		while (BuffLen > 0)
		{
			dev->pReg->TXD = dev->pSpiDev->Cfg.DummyByte;

			if (nRFxSPIWaitReady(dev, NRFSPI_TIMEOUT) == false)
				break;

			*pBuff = dev->pReg->RXD;

			BuffLen--;
			pBuff++;
			cnt++;
		}
    }
#endif
    return cnt;
}

#ifdef SPIM_PRESENT
// Start the next DMA block of the pending interrupt driven master transfer.
// The receive state is used when RxBufflen is not zero, the transmit state
// otherwise. The pointer is loaded for every block.
static void nRFxSPIDmaNextBlock(nRFSpiDev_t * const pDev)
{
	if (pDev->RxBufflen > 0)
	{
		NRFX_SPI_RX(pDev->pDmaReg).PTR = (uint32_t)&pDev->pRxBuff[pDev->RxIdx];
		NRFX_SPI_RX(pDev->pDmaReg).MAXCNT = min(pDev->RxBufflen, NRFX_SPI_DMA_MAXCNT);
	}
	else
	{
		int l = min(pDev->TxDatalen, min(NRFX_SPI_DMA_MAXCNT, NRFX_SPI_TXBUF_SIZE));

		memmove(pDev->TxBuf, &pDev->pTxData[pDev->TxIdx], l);
		NRFX_SPI_TX(pDev->pDmaReg).PTR = (uint32_t)pDev->TxBuf;
		NRFX_SPI_TX(pDev->pDmaReg).MAXCNT = l;
	}

	pDev->pDmaReg->EVENTS_END = 0;
	pDev->pDmaReg->EVENTS_STOPPED = 0;
	NRFX_SPI_ENDRX(pDev->pDmaReg) = 0;
	NRFX_SPI_ENDTX(pDev->pDmaReg) = 0;
	pDev->pDmaReg->TASKS_START = 1;
}

// DMA block done in the master interrupt. Start the next block, or release
// the bus and report DEVINTRF_EVT_COMPLETED with the number of bytes moved.
static void nRFxSPIDmaEnd(DevIntrf_t * const pIntrf, nRFSpiDev_t * const pDev)
{
	bool rx = pDev->RxBufflen > 0;
	int l = 0;
	int left = 0;

	pDev->pDmaReg->EVENTS_END = 0;
	NRFX_SPI_ENDRX(pDev->pDmaReg) = 0;
	NRFX_SPI_ENDTX(pDev->pDmaReg) = 0;

	if (rx)
	{
		l = NRFX_SPI_RX(pDev->pDmaReg).AMOUNT;
		pDev->RxIdx += l;
		pDev->RxBufflen -= l;
		left = pDev->RxBufflen;
	}
	else if (pDev->TxDatalen > 0)
	{
		l = NRFX_SPI_TX(pDev->pDmaReg).AMOUNT;
		pDev->TxIdx += l;
		pDev->TxDatalen -= l;
		left = pDev->TxDatalen;
	}
	else
	{
		// No transfer pending
		return;
	}

	if (l > 0 && left > 0)
	{
		nRFxSPIDmaNextBlock(pDev);
		return;
	}

	uint8_t *buff = NULL;
	bool report = true;
	int count;

	// The generic stop helpers own the busy flag. Release it before the callback.
	if (rx)
	{
		buff = pDev->pRxBuff;
		count = pDev->RxIdx;
		pDev->RxBufflen = 0;
		DeviceIntrfStopRx(pIntrf);
	}
	else
	{
		count = pDev->TxIdx;
		pDev->TxDatalen = 0;
		// A command phase that keeps the bus is not a completion, the
		// data phase that follows reports it.
		report = pIntrf->bNoStop == false;
		if (pDev->pTxWait)
		{
			volatile bool *done = pDev->pTxWait;

			pDev->pTxWait = NULL;
			*done = true;
		}
		DeviceIntrfTxComplete(pIntrf);
	}

	if (report && pIntrf->EvtCB)
	{
		pIntrf->EvtCB(pIntrf, DEVINTRF_EVT_COMPLETED, buff, count);
	}
}
#endif

#if defined(NRF52_SERIES) && defined(SPI_PRESENT)
// nRF52832 anomaly 58: with RXD.MAXCNT = 1 and TXD.MAXCNT <= 1 the SPIM clocks
// out an additional byte. As the errata proposes, a one byte receive is done
// by the SPI, the same instance without EasyDMA, then the SPIM is enabled
// again. The byte is returned at once, also with bIntEn.
static int nRFxSPIRxByteNoDma(nRFSpiDev_t * const pDev, uint8_t *pBuff)
{
	int cnt = 0;

	pDev->pDmaReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
	pDev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
	pDev->pReg->EVENTS_READY = 0;
	pDev->pReg->TXD = pDev->pSpiDev->Cfg.DummyByte;

	if (nRFxSPIWaitReady(pDev, nRFxSPIDmaTimeout(1, pDev->pSpiDev->Cfg.Rate)))
	{
		*pBuff = pDev->pReg->RXD;
		cnt = 1;
	}

	pDev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
	pDev->pDmaReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);

	return cnt;
}
#endif

// Receive Data only, no Start/Stop condition
int nRFxSPIRxDataDma(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;
	int cnt = 0;

	if (dev->bTxAbort || dev->bCmdFailed)
	{
		uint32_t irq = __get_PRIMASK();
		__disable_irq();
		bool pending = dev->bTxAbort;

		dev->bCmdFailed = false;
		__set_PRIMASK(irq);
		return pending ? -1 : 0;
	}

	if (pBuff == NULL || BuffLen <= 0)
		return 0;

#ifdef NRF5340_XXAA_APPLICATION
	if (dev->pSpiDev->Cfg.Mode == SPIMODE_MASTER &&
		nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]) == UINT32_MAX)
		return 0;
#endif

#ifdef SPIM_PRESENT
	NRFX_SPI_TX(dev->pDmaReg).PTR = 0;
	NRFX_SPI_TX(dev->pDmaReg).MAXCNT = 0;
	NRFX_SPI_TX(dev->pDmaReg).LIST = 0;
#if defined(NRF52_SERIES) && defined(SPI_PRESENT)
	if (BuffLen == 1 && nrf52_errata_58())
	{
		return nRFxSPIRxByteNoDma(dev, pBuff);
	}
#endif

	NRFX_SPI_RX(dev->pDmaReg).PTR = (uint32_t)pBuff;
	NRFX_SPI_RX(dev->pDmaReg).LIST = NRFX_SPI_RX_LIST;

	if (pDev->bIntEn == true)
	{
		// The interrupt handler moves through the DMA blocks, releases the
		// bus and reports DEVINTRF_EVT_COMPLETED.
		dev->pRxBuff = pBuff;
		dev->RxIdx = 0;
		dev->TxDatalen = 0;
		dev->RxBufflen = BuffLen;
		nRFxSPIDmaNextBlock(dev);

		return -1;
	}

	while (BuffLen > 0)
	{
		int l = min(BuffLen, NRFX_SPI_DMA_MAXCNT);
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
		uint32_t timeout = nRFxSPIDmaTimeout(l, dev->pSpiDev->Cfg.Rate);
#else
		uint32_t timeout = NRFSPI_TIMEOUT;
#endif

#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
		NRFX_SPI_RX(dev->pDmaReg).PTR = (uint32_t)pBuff;
#endif
		NRFX_SPI_RX(dev->pDmaReg).MAXCNT = l;
		dev->pDmaReg->EVENTS_END = 0;
		NRFX_SPI_ENDRX(dev->pDmaReg) = 0;
		dev->pDmaReg->TASKS_START = 1;

#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
		if (nRFxSPIWaitDMA(dev, timeout) == false)
#else
		if (nRFxSPIWaitRX(dev, timeout) == false)
#endif
			break;

		l = NRFX_SPI_RX(dev->pDmaReg).AMOUNT;
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
		if (l <= 0)
			break;
#endif
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}
#endif

	return cnt;
}

// Stop receive
void nRFxSPIStopRx(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;

	dev->bCmdFailed = false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
bool nRFxSPIStartTx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;

	if (dev->bTxAbort || dev->bCmdFailed)
	{
		return false;
	}

	if (dev->pSpiDev->Cfg.ChipSel != SPICSEL_MAN &&
		DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;

    if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
    {
#ifdef SPIM_PRESENT
        dev->pDmaReg->PSEL.MOSI = nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
        dev->pDmaReg->PSEL.MISO = -1;
#else
        dev->pReg->PSELMOSI = nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
        dev->pReg->PSELMISO = -1;
#endif
    }

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	return true;
}

// Send Data only, no Start/Stop condition
int nRFxSPITxData(DevIntrf_t *pDev, const uint8_t *pData, int DataLen)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t*)pDev->pDevData;
	int cnt = 0;

#ifdef SPI_PRESENT
	if (pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	pDev->bTxReady = false;

	if (pDev->bIntEn)
	{
		// The interrupt handler writes each byte. At the end it sets
		// bTxReady, then releases the bus and reports DEVINTRF_EVT_COMPLETED,
		// or keeps the bus when bNoStop is set.
		dev->pTxData = nRFxSPIAsyncTxBuff(dev, pData, DataLen);
		volatile bool done = false;
		dev->pTxWait = DataLen > NRFX_SPI_TXBUF_SIZE ? &done : NULL;
		dev->bTxAbort = false;
		dev->TxDatalen = DataLen;
		dev->TxIdx = 0;
		dev->RxBufflen = 0;
		dev->pReg->EVENTS_READY = 0;
		dev->pReg->TXD = dev->pTxData[0];
		dev->TxIdx++;
		dev->TxDatalen--;
		nRFxSPIAsyncTxWait(dev, DataLen, done);

		return -1;
	}
    else
    {
		while (DataLen > 0)
		{
			dev->pReg->TXD = *pData;

			if (nRFxSPIWaitReady(dev, NRFSPI_TIMEOUT) == false)
			{
				break;
			}

			int d = dev->pReg->RXD;

			DataLen--;
			pData++;
			cnt++;
		}
    }
#endif

    return cnt;
}

// Transmit Data only, no Start/Stop condition
int nRFxSPITxDataDma(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;
	int cnt = 0;

	if (pData == NULL || DataLen <= 0)
		return 0;

#ifdef SPIM_PRESENT
	NRFX_SPI_RX(dev->pDmaReg).PTR = 0;
	NRFX_SPI_RX(dev->pDmaReg).MAXCNT = 0;
	NRFX_SPI_RX(dev->pDmaReg).LIST = 0;

	NRFX_SPI_TX(dev->pDmaReg).PTR = (uint32_t)pData;
	NRFX_SPI_TX(dev->pDmaReg).LIST = NRFX_SPI_TX_LIST;

	if (pDev->bIntEn == true && DataLen > 0)
	{
		// The interrupt handler moves through the DMA blocks, releases the
		// bus and reports DEVINTRF_EVT_COMPLETED.
		pDev->bTxReady = false;
		dev->pTxData = nRFxSPIAsyncTxBuff(dev, pData, DataLen);
		volatile bool done = false;
		dev->pTxWait = DataLen > NRFX_SPI_TXBUF_SIZE ? &done : NULL;
		dev->bTxAbort = false;
		dev->TxIdx = 0;
		dev->RxBufflen = 0;
		dev->TxDatalen = DataLen;
		nRFxSPIDmaNextBlock(dev);
		nRFxSPIAsyncTxWait(dev, DataLen, done);

		return -1;
	}

	while (DataLen > 0)
	{
		int l = min(DataLen, NRFX_SPI_DMA_MAXCNT);
#ifdef NRF5340_XXAA_APPLICATION
		l = min(l, NRFX_SPI_TXBUF_SIZE);
		memcpy(dev->TxBuf, pData, l);
#endif
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
		uint32_t timeout = nRFxSPIDmaTimeout(l, dev->pSpiDev->Cfg.Rate);
#else
		uint32_t timeout = NRFSPI_TIMEOUT;
#endif
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
#ifdef NRF5340_XXAA_APPLICATION
		NRFX_SPI_TX(dev->pDmaReg).PTR = (uint32_t)dev->TxBuf;
#else
		NRFX_SPI_TX(dev->pDmaReg).PTR = (uint32_t)pData;
#endif
#endif
		NRFX_SPI_TX(dev->pDmaReg).MAXCNT = l;
		dev->pDmaReg->EVENTS_END = 0;
		NRFX_SPI_ENDTX(dev->pDmaReg) = 0;
		dev->pDmaReg->TASKS_START = 1;

		if (nRFxSPIWaitDMA(dev, timeout) == false)
		{
			break;
		}

		l = NRFX_SPI_TX(dev->pDmaReg).AMOUNT;
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
		if (l <= 0)
			break;
#endif
		DataLen -= l;
		pData += l;
		cnt += l;
	}
#endif

	return cnt;
}

// Stop transmit
void nRFxSPIStopTx(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
    if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
    {
#ifdef SPIM_PRESENT
        dev->pDmaReg->PSEL.MISO = nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
        dev->pDmaReg->PSEL.MOSI = -1;
#else
        dev->pReg->PSELMOSI = -1;
        dev->pReg->PSELMISO = nRFxSPIPinSelect(dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
#endif
    }
}

void nRFxSPIReset(DevIntrf_t * const pDev)
{

}

void *nRFxSPIGetHandle(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;

	return dev->pSpiDev;
}

static void nRFxSPIAsyncTxAbortEnd(DevIntrf_t * const pIntrf, nRFSpiDev_t * const pDev, int Count)
{
	pDev->TxDatalen = 0;
	pDev->TxIdx = 0;
	pDev->bTxAbort = false;
	if (pDev->bCmdFailed)
	{
		// The interrupted read still owns the bus until its rejected receive
		// phase runs. Do not let the timeout callback replace it before then.
		atomic_store(&pIntrf->bTxReady, true);
	}
	else
	{
		DeviceIntrfTxComplete(pIntrf);
	}
	if (pIntrf->EvtCB)
	{
		pIntrf->EvtCB(pIntrf, DEVINTRF_EVT_TX_TIMEOUT, NULL, Count);
	}
}

void SPI_IRQHandler(int DevNo, DevIntrf_t * const pDev)//DevIntrf_t * const pDev)
//extern "C" void SPI_IRQHandler(int DevNo)
{
	nRFSpiDev_t *dev = &g_nRFxSPIDev[DevNo];//(NrfSpiDev_t *)pDev->DevIntrf.pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		if (NRFX_SPI_ENDRX(dev->pDmaSReg))
		{
			if (dev->pSpiDev->DevIntrf.EvtCB)
			{
				dev->pSpiDev->DevIntrf.EvtCB(&dev->pSpiDev->DevIntrf, DEVINTRF_EVT_RX_FIFO_FULL, NULL, 0);
			}
			NRFX_SPI_ENDRX(dev->pDmaSReg) = 0;
			dev->pDmaSReg->STATUS = dev->pDmaSReg->STATUS;
		}

		if (dev->pDmaSReg->EVENTS_END)
		{
			if (dev->pSpiDev->DevIntrf.EvtCB)
			{
#ifdef SPIM_PRESENT
				dev->pSpiDev->DevIntrf.EvtCB(&dev->pSpiDev->DevIntrf, DEVINTRF_EVT_COMPLETED, (uint8_t*)NRFX_SPI_RX(dev->pDmaSReg).PTR, NRFX_SPI_RX(dev->pDmaSReg).AMOUNT);
#else
				dev->pSpiDev->DevIntrf.EvtCB(&dev->pSpiDev->DevIntrf, DEVINTRF_EVT_COMPLETED, (uint8_t*)dev->pDmaSReg->RXDPTR, dev->pDmaSReg->AMOUNTRX);
#endif
			}
			dev->pDmaSReg->EVENTS_END = 0;
		}

		if (dev->pDmaSReg->EVENTS_ACQUIRED)
		{
			if (dev->pSpiDev->DevIntrf.EvtCB)
			{
				dev->pSpiDev->DevIntrf.EvtCB(&dev->pSpiDev->DevIntrf, DEVINTRF_EVT_STATECHG, NULL, 0);
			}
			dev->pDmaSReg->STATUS = dev->pDmaSReg->STATUS;

#ifdef SPIM_PRESENT
			NRFX_SPI_RX(dev->pDmaSReg).PTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
#ifdef NRF54L_SERIES
			NRFX_SPI_RX(dev->pDmaSReg).MAXCNT = min(max(dev->pSpiDev->RxBuffLen[0], 0), NRFX_SPI_DMA_MAXCNT);
#else
			NRFX_SPI_RX(dev->pDmaSReg).MAXCNT = dev->pSpiDev->RxBuffLen[0];
#endif
			NRFX_SPI_TX(dev->pDmaSReg).PTR = (uint32_t)dev->pSpiDev->pTxData[0];
#ifdef NRF54L_SERIES
			NRFX_SPI_TX(dev->pDmaSReg).MAXCNT = min(max(dev->pSpiDev->TxDataLen[0], 0), NRFX_SPI_DMA_MAXCNT);
#else
			NRFX_SPI_TX(dev->pDmaSReg).MAXCNT = dev->pSpiDev->TxDataLen[0];
#endif
#else
			dev->pDmaSReg->RXDPTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
			dev->pDmaSReg->MAXRX = dev->pSpiDev->RxBuffLen[0];
			dev->pDmaSReg->TXDPTR = (uint32_t)dev->pSpiDev->pTxData[0];
			dev->pDmaSReg->MAXTX = dev->pSpiDev->TxDataLen[0];
#endif
			dev->pDmaSReg->EVENTS_ACQUIRED = 0;
			dev->pDmaSReg->TASKS_RELEASE = 1;
		}
	}
	else
	{
		// Master mode

#ifdef SPIM_PRESENT
		if (pDev->bDma)
		{
			if (dev->bTxAbort)
			{
				dev->pDmaReg->EVENTS_END = 0;
				NRFX_SPI_ENDRX(dev->pDmaReg) = 0;
				NRFX_SPI_ENDTX(dev->pDmaReg) = 0;
				if (dev->pDmaReg->EVENTS_STOPPED)
				{
					int count = dev->TxIdx + NRFX_SPI_TX(dev->pDmaReg).AMOUNT;

					dev->pDmaReg->EVENTS_STOPPED = 0;
					dev->pDmaReg->INTENCLR = SPIM_INTENCLR_STOPPED_Msk;
					NRFX_SPI_TX(dev->pDmaReg).MAXCNT = 0;
					NRFX_SPI_TX(dev->pDmaReg).PTR = 0;
					nRFxSPIAsyncTxAbortEnd(pDev, dev, count);
				}
				return;
			}

			if (dev->pDmaReg->EVENTS_END)
			{
				nRFxSPIDmaEnd(pDev, dev);
			}
		}
		else
#endif
		{
#ifdef SPI_PRESENT
			// Without EasyDMA, one byte per READY event
			if (dev->pReg->EVENTS_READY)
			{
				dev->pReg->EVENTS_READY = 0;

				uint8_t d = dev->pReg->RXD;

				if (dev->bTxAbort)
				{
					nRFxSPIAsyncTxAbortEnd(pDev, dev, dev->TxIdx);
					return;
				}

				if (dev->RxBufflen > 0)
				{
					dev->pRxBuff[dev->RxIdx] = d;
					dev->RxIdx++;
					dev->RxBufflen--;

					if (dev->RxBufflen > 0)
					{
						dev->pReg->TXD = dev->pSpiDev->Cfg.DummyByte;
					}
					else
					{
						// The generic stop helpers own the busy flag.
						// Release it before the callback.
						DeviceIntrfStopRx(pDev);
						if (pDev->EvtCB)
						{
							pDev->EvtCB(pDev, DEVINTRF_EVT_COMPLETED, dev->pRxBuff, dev->RxIdx);
						}
					}
				}
				else if (dev->TxIdx > 0)
				{
					if (dev->TxDatalen > 0)
					{
						dev->pReg->TXD = dev->pTxData[dev->TxIdx];
						dev->TxIdx++;
						dev->TxDatalen--;
					}
					else
					{
						int count = dev->TxIdx;
						// A command phase that keeps the bus is not a
						// completion, the data phase that follows reports it.
						bool report = pDev->bNoStop == false;

						dev->TxIdx = 0;
						if (dev->pTxWait)
						{
							volatile bool *done = dev->pTxWait;

							dev->pTxWait = NULL;
							*done = true;
						}
						DeviceIntrfTxComplete(pDev);
						if (report && pDev->EvtCB)
						{
							pDev->EvtCB(pDev, DEVINTRF_EVT_COMPLETED, NULL, count);
						}
					}
				}
			}
#endif
		}
	}
}

static bool nRFxSPIInit(SPIDev_t * const pDev)
{
	uint32_t cfgreg = 0;

	if (pDev->Cfg.Phy >= SPIPHY_DUAL)
	{
		return false;
	}

	// Get the correct register map
#ifdef SPIM_PRESENT
	NRF_SPIM_Type *reg = g_nRFxSPIDev[pDev->Cfg.DevNo].pDmaReg;
#else
	NRF_SPI_Type *reg = g_nRFxSPIDev[pDev->Cfg.DevNo].pReg;
#endif

	// Force power on in case it was powered off previously
#if !defined(NRF54L_SERIES) && !defined(NRF5340_XXAA_APPLICATION)
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
#else
	reg->ENABLE = 0;
#endif
	// No interrupt from a previous configuration. SPI and SPIM are one block:
	// READY left enabled by an interrupt SPI configuration would also fire
	// for the SPIM, whose handler does not clear it.
	reg->INTENCLR = 0xFFFFFFFF;
#ifdef NRF5340_XXAA_APPLICATION
	reg->EVENTS_END = 0;
	reg->EVENTS_STOPPED = 0;
	reg->EVENTS_ENDRX = 0;
	reg->EVENTS_ENDTX = 0;
	reg->SHORTS = 0;
#endif

	// Configure I/O pins
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);

	for (int i = SPI_CS_IOPIN_IDX; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSet(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo);
	}

	if (pDev->Cfg.BitOrder == SPIDATABIT_LSB)
	{
#ifdef SPIM_PRESENT
		cfgreg |= SPIM_CONFIG_ORDER_LsbFirst;
#else
		cfgreg |= SPI_CONFIG_ORDER_LsbFirst;
#endif
	}
	else
	{
#ifdef SPIM_PRESENT
		cfgreg |= SPIM_CONFIG_ORDER_MsbFirst;
#else
		cfgreg |= SPI_CONFIG_ORDER_MsbFirst;
#endif
	}

	if (pDev->Cfg.DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPHA_Trailing << SPIM_CONFIG_CPHA_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos);
#endif
	}
	else
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPHA_Leading << SPIM_CONFIG_CPHA_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos);
#endif
	}

	if (pDev->Cfg.ClkPol == SPICLKPOL_LOW)
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPOL_ActiveLow << SPIM_CONFIG_CPOL_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
#endif
		IOPinSet(pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo, pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}
	else
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
#endif
		IOPinClear(pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo, pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}

	reg->CONFIG = cfgreg;

	nRFxSPISetRate(&pDev->DevIntrf, pDev->Cfg.Rate);

	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = nRFxSPIDisable;
	pDev->DevIntrf.Enable = nRFxSPIEnable;
	pDev->DevIntrf.GetRate = nRFxSPIGetRate;
	pDev->DevIntrf.SetRate = nRFxSPISetRate;
	pDev->DevIntrf.StartRx = nRFxSPIStartRx;
	pDev->DevIntrf.RxData = nRFxSPIRxData;
	pDev->DevIntrf.StopRx = nRFxSPIStopRx;
	pDev->DevIntrf.StartTx = nRFxSPIStartTx;
	pDev->DevIntrf.TxData = nRFxSPITxData;
	pDev->DevIntrf.StopTx = nRFxSPIStopTx;
	pDev->DevIntrf.IntPrio = pDev->Cfg.IntPrio;
	pDev->DevIntrf.EvtCB = pDev->Cfg.EvtCB;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.bDma = pDev->Cfg.bDmaEn;
	pDev->DevIntrf.bIntEn = pDev->Cfg.bIntEn;
	pDev->DevIntrf.PowerOff = nRFxSPIPowerOff;
	pDev->DevIntrf.Reset = nRFxSPIReset;
	pDev->DevIntrf.GetHandle = nRFxSPIGetHandle;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	if (pDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		// Force DMA for slave mode
		pDev->DevIntrf.bDma = true;
	}

#ifndef SPI_PRESENT
	// only DMA mode avail.  Force DMA
	pDev->DevIntrf.bDma = true;
#endif

	if (pDev->DevIntrf.bDma == true)
	{
		pDev->DevIntrf.RxData = nRFxSPIRxDataDma;
		pDev->DevIntrf.TxData = nRFxSPITxDataDma;
	}

	uint32_t inten = 0;

    if (pDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		NRF_SPIS_Type *sreg = g_nRFxSPIDev[pDev->Cfg.DevNo].pDmaSReg;

#ifdef SPIM_PRESENT
        sreg->PSEL.SCK = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX]);
        sreg->PSEL.MISO = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
        sreg->PSEL.MOSI = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MOSI_IOPIN_IDX]);
		sreg->PSEL.CSN = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX]);
#else
        sreg->PSELSCK = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX]);
        sreg->PSELMISO = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
        sreg->PSELMOSI = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MOSI_IOPIN_IDX]);
		sreg->PSELCSN = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_CS_IOPIN_IDX]);
#endif
		sreg->ORC = 0xFF;
		sreg->STATUS = sreg->STATUS;
		NRFX_SPI_ENDRX(sreg) = 0;
		sreg->EVENTS_END = 0;
		sreg->EVENTS_ACQUIRED = 0;
		sreg->DEF = 0xFF;
		sreg->SHORTS = (SPIS_SHORTS_END_ACQUIRE_Enabled << SPIS_SHORTS_END_ACQUIRE_Pos);

		inten = (SPIS_INTENSET_ACQUIRED_Enabled << SPIS_INTENSET_ACQUIRED_Pos) |
				NRFX_SPIS_RX_INT |
				(SPIS_INTENSET_END_Enabled << SPIS_INTENSET_END_Pos);
		reg->ENABLE =  (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
		sreg->TASKS_ACQUIRE = 1;	// Active event to update rx/tx buffer
	}
	else
	{
#ifdef SPIM_PRESENT
		reg->PSEL.SCK = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX]);
		reg->PSEL.MISO = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
		reg->PSEL.MOSI = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MOSI_IOPIN_IDX]);

		if (pDev->DevIntrf.bDma)
		{
#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
			reg->ORC = pDev->Cfg.DummyByte;
#else
			reg->ORC = 0xFF;
#endif
			reg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	        if (pDev->DevIntrf.bIntEn == true)
	        {
	        	// Only END is used: it follows both the RX and TX DMA blocks.
	        	inten = SPIM_INTENSET_END_Msk;
	        }
		}
		else
		{
#ifdef SPI_PRESENT
			g_nRFxSPIDev[pDev->Cfg.DevNo].pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
			g_nRFxSPIDev[pDev->Cfg.DevNo].pReg->EVENTS_READY = 0;
	        if (pDev->DevIntrf.bIntEn == true)
	        {
	        	inten = reg->INTENSET = SPI_INTENSET_READY_Msk;
	        }
#endif
		}
#else
		reg->PSELSCK = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX]);
		reg->PSELMISO = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX]);
		reg->PSELMOSI = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[SPI_MOSI_IOPIN_IDX]);

		reg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
        reg->EVENTS_READY = 0;
        if (pDev->DevIntrf.bIntEn == true)
        {
        	inten = reg->INTENSET = SPI_INTENSET_READY_Msk;
        }
#endif

	}

	reg->INTENSET = inten;

	return true;
}

bool SPIInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData)
{
	bool retval = false;

	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->Mode == SPIMODE_SLAVE && pCfgData->DevNo >= NRFX_SPISLAVE_MAXDEV)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= NRFX_SPI_MAXDEV || pCfgData->NbIOPins < 3)
	{
		return false;
	}
	if (g_nRFxSPIDev[pCfgData->DevNo].bTxAbort || g_nRFxSPIDev[pCfgData->DevNo].bCmdFailed)
	{
		// TxBuf may still be in use by the cancelled DMA block.
		return false;
	}

#if defined(NRF54L_SERIES) || defined(NRF5340_XXAA_APPLICATION)
	if (pCfgData->pIOPinMap == NULL || pCfgData->DataSize != 8 ||
		((pCfgData->Mode == SPIMODE_SLAVE || pCfgData->ChipSel == SPICSEL_AUTO) && pCfgData->NbIOPins < 4))
		return false;
#endif

#ifdef NRF5340_XXAA_APPLICATION
	if (pCfgData->DevNo == 4 && pCfgData->Mode != SPIMODE_MASTER)
		return false;
#endif

	pDev->Cfg = *pCfgData;
	g_nRFxSPIDev[pCfgData->DevNo].pSpiDev  = pDev;
	g_nRFxSPIDev[pCfgData->DevNo].RxBufflen = 0;
	g_nRFxSPIDev[pCfgData->DevNo].RxIdx = 0;
	g_nRFxSPIDev[pCfgData->DevNo].pRxBuff = NULL;
	g_nRFxSPIDev[pCfgData->DevNo].TxDatalen = 0;
	g_nRFxSPIDev[pCfgData->DevNo].TxIdx = 0;
	g_nRFxSPIDev[pCfgData->DevNo].pTxData = NULL;
	g_nRFxSPIDev[pCfgData->DevNo].pTxWait = NULL;
	g_nRFxSPIDev[pCfgData->DevNo].bTxAbort = false;
	g_nRFxSPIDev[pCfgData->DevNo].bCmdFailed = false;

	pDev->DevIntrf.pDevData = (void*)&g_nRFxSPIDev[pCfgData->DevNo];

#if defined(NRF52840) || defined(NRF5340_XXAA_APPLICATION)
	if (pCfgData->DevNo < NRFX_SPI_MAXDEV - 1)
#else
	if (pCfgData->DevNo < NRFX_SPI_MAXDEV)
#endif
	{
		// SPI
		retval = nRFxSPIInit(pDev);
	}
#if defined(NRF52840) || defined(NRF5340_XXAA_APPLICATION)
	else
	{
		// Quad SPI
		retval = nRFxQSPIInit(pDev);
	}
#endif

	if (retval == false)
	{
		return false;
	}


    if (pCfgData->bIntEn || pCfgData->Mode == SPIMODE_SLAVE)
    {
#ifdef NRF5340_XXAA_APPLICATION
		// SPIM4 and QSPI have dedicated vectors, not shared serial instances.
		if (pCfgData->DevNo < 4)
#endif
		SharedIntrfSetIrqHandler(pCfgData->DevNo, &pDev->DevIntrf, SPI_IRQHandler);

    	switch (pCfgData->DevNo)
    	{
#ifdef NRF54L_SERIES
			case 0:
				NVIC_ClearPendingIRQ(SERIAL30_IRQn);
				NVIC_SetPriority(SERIAL30_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL30_IRQn);
				break;
			case 1:
				NVIC_ClearPendingIRQ(SERIAL20_IRQn);
				NVIC_SetPriority(SERIAL20_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL20_IRQn);
				break;
			case 2:
				NVIC_ClearPendingIRQ(SERIAL21_IRQn);
				NVIC_SetPriority(SERIAL21_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL21_IRQn);
				break;
			case 3:
				NVIC_ClearPendingIRQ(SERIAL22_IRQn);
				NVIC_SetPriority(SERIAL22_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL22_IRQn);
				break;
			case 4:
				NVIC_ClearPendingIRQ(SERIAL00_IRQn);
				NVIC_SetPriority(SERIAL00_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL00_IRQn);
				break;
#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
			case 5:
				NVIC_ClearPendingIRQ(SERIAL23_IRQn);
				NVIC_SetPriority(SERIAL23_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL23_IRQn);
				break;
			case 6:
				NVIC_ClearPendingIRQ(SERIAL24_IRQn);
				NVIC_SetPriority(SERIAL24_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SERIAL24_IRQn);
				break;
#endif
#elif defined(NRF52805_XXAA) || defined(NRF52810_XXAA)
			case 0:
				NVIC_ClearPendingIRQ(SPIM0_SPIS0_SPI0_IRQn);
				NVIC_SetPriority(SPIM0_SPIS0_SPI0_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(SPIM0_SPIS0_SPI0_IRQn);
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
    	    case 2:
                NVIC_ClearPendingIRQ(SPIM2_SPIS2_SPI2_IRQn);
                NVIC_SetPriority(SPIM2_SPIS2_SPI2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM2_SPIS2_SPI2_IRQn);
                break;
#ifdef NRF52840_XXAA
    	    case 3:
                NVIC_ClearPendingIRQ(SPIM3_IRQn);
                NVIC_SetPriority(SPIM3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM3_IRQn);
                break;
    	    case 4:
                NVIC_ClearPendingIRQ(QSPI_IRQn);
                NVIC_SetPriority(QSPI_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(QSPI_IRQn);
    	    	break;
#endif
#elif defined(NRF91_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                break;
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
#elif defined(NRF53_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(NRF53_SERIAL0_IRQ);
                NVIC_SetPriority(NRF53_SERIAL0_IRQ, pCfgData->IntPrio);
                NVIC_EnableIRQ(NRF53_SERIAL0_IRQ);
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
    	    case 4:
                NVIC_ClearPendingIRQ(SPIM4_IRQn);
                NVIC_SetPriority(SPIM4_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM4_IRQn);
                break;
			case 5:
				NVIC_ClearPendingIRQ(QSPI_IRQn);
				NVIC_SetPriority(QSPI_IRQn, pCfgData->IntPrio);
				NVIC_EnableIRQ(QSPI_IRQn);
				break;
#endif
#else
    		case 0:
                NVIC_ClearPendingIRQ(SPI0_TWI0_IRQn);
                NVIC_SetPriority(SPI0_TWI0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI0_TWI0_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(SPI1_TWI1_IRQn);
                NVIC_SetPriority(SPI1_TWI1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI1_TWI1_IRQn);
                break;
#endif
    	}

    	//reg->INTENSET = inten;
    }

	return true;
}

#ifdef NRF52_SERIES
#if !defined(NRF52805_XXAA) && !defined(NRF52810_XXAA)
extern "C" void SPIM2_SPIS2_SPI2_IRQHandler(void)
{
	SPI_IRQHandler(2, &g_nRFxSPIDev[2].pSpiDev->DevIntrf);
    NVIC_ClearPendingIRQ(SPIM2_SPIS2_SPI2_IRQn);
}
#endif
#ifdef NRF52840_XXAA
extern "C" void SPIM3_IRQHandler(void)
{
	SPI_IRQHandler(3, &g_nRFxSPIDev[3].pSpiDev->DevIntrf);
    NVIC_ClearPendingIRQ(SPIM3_IRQn);
}
#endif
#endif

#ifdef NRF5340_XXAA_APPLICATION
// SPIM4 has its own interrupt, like SPIM3 on nRF52840. It is not part of
// the UART/TWI/SPI shared serial dispatcher (instances 0 through 3).
extern "C" void SPIM4_IRQHandler(void)
{
	SPIDev_t *dev = g_nRFxSPIDev[4].pSpiDev;

	if (dev != NULL)
		SPI_IRQHandler(4, &dev->DevIntrf);
}
#endif
