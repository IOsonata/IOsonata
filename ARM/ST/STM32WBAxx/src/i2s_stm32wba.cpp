/**-------------------------------------------------------------------------
@file	i2s_stm32wba.cpp

@brief	STM32WBA SAI1 I2S transport


@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#include <limits.h>
#include <string.h>
#include "stm32wbaxx.h"
#include "coredev/i2s.h"
#include "gpdma_stm32wba.h"
#include "sai_stm32wba.h"
#include "iopinctrl.h"

#define I2S_DMA_FRAMES 32
#define I2S_STOP_POLLS 10000
#define I2S_DATA_POLLS 1000000
#define I2S_ERRORS (SAI_xSR_OVRUDR | SAI_xSR_WCKCFG | SAI_xSR_AFSDET | SAI_xSR_LFSDET)

// Caller FIFOs contain complete frames. 24-bit samples use three little-endian
// bytes; the internal DMA buffers use the hardware's four-byte data access.
// Only the ISR produces RX frames or consumes TX frames when interrupts are on.
typedef struct __I2s_Dma_Buffer
{
	alignas(4) uint8_t Data[2][I2S_DMA_FRAMES * 2 * 4];
	int Channel;
	unsigned Index;
	unsigned Bytes[2];
} I2sDmaBuffer_t;

typedef struct __I2s_Pin_Route
{
	unsigned Index;
	int Port;
	int Pin;
} I2sPinRoute_t;

// DS14736 table 25: A supplies SCK/FS/MCLK and transmit; B receives.
static const I2sPinRoute_t s_PinRoutes[] = {
	{ I2S_SCK_IOPIN_IDX, IOPORTA, 7 },
	{ I2S_SCK_IOPIN_IDX, IOPORTE, 5 },
	{ I2S_SCK_IOPIN_IDX, IOPORTG, 9 },
	{ I2S_WS_IOPIN_IDX, IOPORTA, 8 },
	{ I2S_WS_IOPIN_IDX, IOPORTC, 4 },
	{ I2S_WS_IOPIN_IDX, IOPORTE, 4 },
	{ I2S_WS_IOPIN_IDX, IOPORTG, 10 },
	{ I2S_SDI_IOPIN_IDX, IOPORTB, 7 },
	{ I2S_SDI_IOPIN_IDX, IOPORTC, 5 },
	{ I2S_SDI_IOPIN_IDX, IOPORTE, 3 },
	{ I2S_SDI_IOPIN_IDX, IOPORTG, 5 },
	{ I2S_SDO_IOPIN_IDX, IOPORTB, 12 },
	{ I2S_SDO_IOPIN_IDX, IOPORTB, 14 },
	{ I2S_SDO_IOPIN_IDX, IOPORTC, 1 },
	{ I2S_SDO_IOPIN_IDX, IOPORTC, 3 },
	{ I2S_SDO_IOPIN_IDX, IOPORTD, 5 },
	{ I2S_SDO_IOPIN_IDX, IOPORTE, 6 },
	{ I2S_SDO_IOPIN_IDX, IOPORTG, 12 },
	{ I2S_MCK_IOPIN_IDX, IOPORTA, 6 },
	{ I2S_MCK_IOPIN_IDX, IOPORTE, 2 },
	{ I2S_MCK_IOPIN_IDX, IOPORTG, 7 },
	{ I2S_MCK_IOPIN_IDX, IOPORTG, 11 }
};

static I2SDev_t *s_pDev;
static I2sDmaBuffer_t s_Rx;
static I2sDmaBuffer_t s_Tx;
static bool s_HasRx;
static bool s_HasTx;
static volatile bool s_Enabled;
static volatile bool s_Failed;
static uint32_t s_OldClock;
static uint32_t s_Divider;
static uint32_t s_KernelClock;
static unsigned s_Channels;
static unsigned s_FrameBytes;
static unsigned s_AccessBytes;
static unsigned s_RxSample;
static uint8_t s_RxFrame[6];
static unsigned s_TxSample;
static uint8_t s_TxFrame[6];

static void I2sSaiHandler();
static void I2sDmaHandler(int Ch, void *pCtx);
static void I2sEnable(DevIntrf_t *pDev);
static void I2sDisable(DevIntrf_t *pDev);

static bool I2sOwned(DevIntrf_t *pDev)
{
	return s_pDev && pDev == &s_pDev->DevIntrf;
}

static void I2sNotify(DEVINTRF_EVT Evt, int Count)
{
	if (s_pDev->DevIntrf.EvtCB)
	{
		s_pDev->DevIntrf.EvtCB(&s_pDev->DevIntrf, Evt, nullptr, Count);
	}
}

static uint32_t I2sUnpack(const uint8_t *pData)
{
	uint32_t value = 0;

	for (unsigned i = 0; i < s_pDev->SampleWidth; i++)
	{
		value |= uint32_t(pData[i]) << (i * 8);
	}
	return value;
}

static void I2sPack(uint8_t *pData, uint32_t Value)
{
	for (unsigned i = 0; i < s_pDev->SampleWidth; i++)
	{
		pData[i] = Value >> (i * 8);
	}
}

static uint32_t I2sLoad(const uint8_t *pData)
{
	uint32_t value = 0;

	memcpy(&value, pData, s_AccessBytes);
	return value;
}

static void I2sStore(uint8_t *pData, uint32_t Value)
{
	memcpy(pData, &Value, s_AccessBytes);
}

static void I2sHalt()
{
	s_Enabled = false;
	SAI1_Block_A->IMR = 0;
	SAI1_Block_B->IMR = 0;
	SAI1_Block_A->CR1 &= ~SAI_xCR1_DMAEN;
	SAI1_Block_B->CR1 &= ~SAI_xCR1_DMAEN;
	// Stop the synchronized receiver while its clock generator is still running.
	SAI1_Block_B->CR1 &= ~SAI_xCR1_SAIEN;
	for (unsigned i = 0; i < I2S_STOP_POLLS && (SAI1_Block_B->CR1 & SAI_xCR1_SAIEN); i++)
	{
	}
	SAI1_Block_A->CR1 &= ~(SAI_xCR1_SAIEN | SAI_xCR1_MCKEN);
	for (unsigned i = 0; i < I2S_STOP_POLLS && (SAI1_Block_A->CR1 & SAI_xCR1_SAIEN); i++)
	{
	}
	if (s_Rx.Channel >= 0)
	{
		WbaGpdmaStop(s_Rx.Channel);
	}
	if (s_Tx.Channel >= 0)
	{
		WbaGpdmaStop(s_Tx.Channel);
	}
	if ((SAI1_Block_A->CR1 | SAI1_Block_B->CR1) & SAI_xCR1_SAIEN)
	{
		RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
	}
	SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;
	SAI1_Block_B->CR2 |= SAI_xCR2_FFLUSH;
	SAI1_Block_A->CLRFR = I2S_ERRORS;
	SAI1_Block_B->CLRFR = I2S_ERRORS;
	NVIC_ClearPendingIRQ(SAI1_IRQn);
}

static void I2sFail(DEVINTRF_EVT Evt)
{
	if (s_Enabled)
	{
		s_Failed = true;
		I2sHalt();
		I2sNotify(Evt, 0);
	}
}

static bool I2sPublish(const uint8_t *pFrame)
{
	uint8_t *dest = CFifoResv(s_pDev->hRxFifo);

	if (!dest)
	{
		I2sFail(DEVINTRF_EVT_RX_FIFO_FULL);
		return false;
	}
	memcpy(dest, pFrame, s_FrameBytes);
	CFifoPut(s_pDev->hRxFifo);
	return true;
}

static unsigned I2sTxBlock(uint8_t *pRaw)
{
	unsigned count = 0;

	if (s_AccessBytes == s_pDev->SampleWidth)
	{
		while (count < I2S_DMA_FRAMES)
		{
			int frames = I2S_DMA_FRAMES - count;
			const uint8_t *data = CFifoPeekMultiple(s_pDev->hTxFifo, &frames);

			if (!data)
			{
				break;
			}
			memcpy(pRaw + count * s_FrameBytes, data, frames * s_FrameBytes);
			CFifoRelease(s_pDev->hTxFifo, frames);
			count += frames;
		}
		memset(pRaw + count * s_FrameBytes, 0, (I2S_DMA_FRAMES - count) * s_FrameBytes);
		return count * s_FrameBytes;
	}
	for (unsigned frame = 0; frame < I2S_DMA_FRAMES; frame++)
	{
		const uint8_t *data = CFifoPeek(s_pDev->hTxFifo);

		for (unsigned sample = 0; sample < s_Channels; sample++)
		{
			uint32_t value = data ? I2sUnpack(data + sample * s_pDev->SampleWidth) : 0;

			I2sStore(pRaw, value);
			pRaw += s_AccessBytes;
		}
		if (data)
		{
			CFifoRelease(s_pDev->hTxFifo, 1);
			count += s_FrameBytes;
		}
	}
	return count;
}

static bool I2sRxBlock(const uint8_t *pRaw)
{
	if (s_AccessBytes == s_pDev->SampleWidth)
	{
		unsigned count = 0;

		while (count < I2S_DMA_FRAMES)
		{
			int frames = I2S_DMA_FRAMES - count;
			uint8_t *dest = CFifoResvMultiple(s_pDev->hRxFifo, &frames);

			if (!dest)
			{
				I2sFail(DEVINTRF_EVT_RX_FIFO_FULL);
				return false;
			}
			memcpy(dest, pRaw + count * s_FrameBytes, frames * s_FrameBytes);
			CFifoPutMultiple(s_pDev->hRxFifo, &frames);
			count += frames;
		}
		return true;
	}
	for (unsigned frame = 0; frame < I2S_DMA_FRAMES; frame++)
	{
		uint8_t packed[6];

		for (unsigned sample = 0; sample < s_Channels; sample++)
		{
			I2sPack(packed + sample * s_pDev->SampleWidth, I2sLoad(pRaw));
			pRaw += s_AccessBytes;
		}
		if (!I2sPublish(packed))
		{
			return false;
		}
	}
	return true;
}

static void I2sServiceDma(int Ch)
{
	if (!s_Enabled)
	{
		WbaGpdmaClear(Ch);
		return;
	}
	uint32_t rxFlags = s_Rx.Channel >= 0 ? WbaGpdmaFlags(s_Rx.Channel) : 0;
	uint32_t txFlags = s_Tx.Channel >= 0 ? WbaGpdmaFlags(s_Tx.Channel) : 0;

	if ((rxFlags | txFlags) & WBA_GPDMA_FLAG_ERR)
	{
		I2sFail(DEVINTRF_EVT_STATECHG);
		return;
	}
	unsigned rxIndex = s_Rx.Index;
	unsigned txIndex = s_Tx.Index;
	unsigned bytes = I2S_DMA_FRAMES * s_Channels * s_AccessBytes;

	// Restart every completed channel before copying or converting frames.
	// TX's alternate buffer was prepared during the previous completion.
	if (rxFlags & WBA_GPDMA_FLAG_TC)
	{
		WbaGpdmaClear(s_Rx.Channel);
		s_Rx.Index ^= 1;
		WbaGpdmaStart(s_Rx.Channel, s_Rx.Data[s_Rx.Index], bytes);
	}
	if (txFlags & WBA_GPDMA_FLAG_TC)
	{
		WbaGpdmaClear(s_Tx.Channel);
		s_Tx.Index ^= 1;
		WbaGpdmaStart(s_Tx.Channel, s_Tx.Data[s_Tx.Index], bytes);
	}
	if (txFlags & WBA_GPDMA_FLAG_TC)
	{
		unsigned completed = s_Tx.Bytes[txIndex];

		s_Tx.Bytes[txIndex] = I2sTxBlock(s_Tx.Data[txIndex]);
		if (completed && !s_Tx.Bytes[0] && !s_Tx.Bytes[1])
		{
			I2sNotify(DEVINTRF_EVT_TX_FIFO_EMPTY, 0);
		}
		if (completed)
		{
			I2sNotify(DEVINTRF_EVT_TX_READY, CFifoUsed(s_pDev->hTxFifo) * s_FrameBytes);
		}
	}
	if ((rxFlags & WBA_GPDMA_FLAG_TC) && s_Enabled && I2sRxBlock(s_Rx.Data[rxIndex]))
	{
		I2sNotify(DEVINTRF_EVT_RX_DATA, CFifoUsed(s_pDev->hRxFifo) * s_FrameBytes);
	}
}

static void I2sDmaHandler(int Ch, void *pCtx)
{
	if (s_pDev == pCtx)
	{
		I2sServiceDma(Ch);
	}
}

static void I2sServiceFifo()
{
	bool received = false;

	if (!s_Enabled)
	{
		return;
	}
	if ((SAI1_Block_A->SR & (I2S_ERRORS & ~SAI_xSR_OVRUDR)) || (s_HasRx && (SAI1_Block_B->SR & I2S_ERRORS)))
	{
		I2sFail(DEVINTRF_EVT_STATECHG);
		return;
	}
	if (s_pDev->DevIntrf.bDma)
	{
		return;
	}
	// At most one hardware FIFO per service, keeping interrupt work bounded.
	for (unsigned i = 0; s_pDev->hRxFifo && i < 8 && (SAI1_Block_B->SR & SAI_xSR_FLVL); i++)
	{
		I2sPack(s_RxFrame + s_RxSample * s_pDev->SampleWidth, SAI1_Block_B->DR);
		if (++s_RxSample == s_Channels)
		{
			s_RxSample = 0;
			if (!I2sPublish(s_RxFrame))
			{
				return;
			}
			received = true;
		}
	}
	for (unsigned i = 0; s_pDev->hTxFifo && i < 8 && (SAI1_Block_A->SR & SAI_xSR_FREQ); i++)
	{
		if (!s_TxSample)
		{
			uint8_t *data = CFifoPeek(s_pDev->hTxFifo);

			if (data)
			{
				memcpy(s_TxFrame, data, s_FrameBytes);
				CFifoRelease(s_pDev->hTxFifo, 1);
			}
			else
			{
				memset(s_TxFrame, 0, s_FrameBytes);
			}
		}
		SAI1_Block_A->DR = I2sUnpack(s_TxFrame + s_TxSample * s_pDev->SampleWidth);
		s_TxSample = (s_TxSample + 1) % s_Channels;
	}
	SAI1_Block_A->CLRFR = SAI_xCLRFR_COVRUDR;
	if (received)
	{
		I2sNotify(DEVINTRF_EVT_RX_DATA, CFifoUsed(s_pDev->hRxFifo) * s_FrameBytes);
	}
}

static void I2sSaiHandler()
{
	if (s_Enabled)
	{
		I2sServiceFifo();
	}
	else
	{
		SAI1_Block_A->IMR = 0;
		SAI1_Block_B->IMR = 0;
		SAI1_Block_A->CLRFR = I2S_ERRORS;
		SAI1_Block_B->CLRFR = I2S_ERRORS;
	}
}

static void I2sConfigure()
{
	unsigned slotBits = s_pDev->SampleWidth == 3 ? 32 : s_pDev->SampleWidth * 8;
	uint32_t ds = s_pDev->SampleWidth == 1 ? SAI_xCR1_DS_1 : (s_pDev->SampleWidth == 2 ? SAI_xCR1_DS_2 : SAI_xCR1_DS_2 | SAI_xCR1_DS_1);
	uint32_t frame = (slotBits * 2 - 1) | ((slotBits - 1) << SAI_xFRCR_FSALL_Pos) | SAI_xFRCR_FSDEF;
	uint32_t slot = SAI_xSLOTR_NBSLOT_0;
	uint32_t active = s_pDev->Chan == I2SCHAN_STEREO ? 3 : (s_pDev->Chan == I2SCHAN_LEFT ? 1 : 2);
	uint32_t cr1 = ds | SAI_xCR1_CKSTR;

	if (s_pDev->Format == I2SFMT_NORMAL)
	{
		frame |= SAI_xFRCR_FSOFF;
	}
	else
	{
		frame |= SAI_xFRCR_FSPOL;
	}
	if (s_pDev->SampleWidth == 3)
	{
		slot |= SAI_xSLOTR_SLOTSZ_1;
		if (s_pDev->Format == I2SFMT_ALIGN_RIGHT)
		{
			slot |= 8U << SAI_xSLOTR_FBOFF_Pos;
		}
	}
	slot |= active << SAI_xSLOTR_SLOTEN_Pos;
	if (s_pDev->Mode == I2SMODE_SLAVE)
	{
		cr1 |= SAI_xCR1_MODE_1;
	}
	else
	{
		cr1 |= s_Divider << SAI_xCR1_MCKDIV_Pos;
		if (s_pDev->NbIOPins > I2S_MCK_IOPIN_IDX && s_pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PortNo >= 0)
		{
			cr1 |= SAI_xCR1_MCKEN;
		}
	}
	SAI1->GCR = 0;
	SAI1->PDMCR = 0;
	SAI1_Block_A->CR1 = cr1;
	SAI1_Block_B->CR1 = ds | SAI_xCR1_CKSTR | SAI_xCR1_MODE | SAI_xCR1_SYNCEN_0;
	SAI1_Block_A->CR2 = SAI_xCR2_FTH_1;
	SAI1_Block_B->CR2 = SAI_xCR2_FTH_1;
	SAI1_Block_A->FRCR = frame;
	SAI1_Block_B->FRCR = frame;
	SAI1_Block_A->SLOTR = slot;
	SAI1_Block_B->SLOTR = slot;
}

static void I2sEnable(DevIntrf_t *pDev)
{
	if (!I2sOwned(pDev) || s_Enabled || s_Failed)
	{
		return;
	}
	I2sConfigure();
	s_RxSample = 0;
	s_TxSample = 0;
	s_Rx.Index = 0;
	s_Tx.Index = 0;
	if (s_Rx.Channel >= 0)
	{
		WbaGpdmaSetup(s_Rx.Channel, WBA_GPDMA_REQ_SAI1_B, false, &SAI1_Block_B->DR, s_AccessBytes == 4 ? 2 : s_AccessBytes - 1, pDev->bIntEn);
		WbaGpdmaStart(s_Rx.Channel, s_Rx.Data[0], I2S_DMA_FRAMES * s_Channels * s_AccessBytes);
		SAI1_Block_B->CR1 |= SAI_xCR1_DMAEN;
	}
	if (s_Tx.Channel >= 0)
	{
		s_Tx.Bytes[0] = I2sTxBlock(s_Tx.Data[0]);
		s_Tx.Bytes[1] = I2sTxBlock(s_Tx.Data[1]);
		WbaGpdmaSetup(s_Tx.Channel, WBA_GPDMA_REQ_SAI1_A, true, &SAI1_Block_A->DR, s_AccessBytes == 4 ? 2 : s_AccessBytes - 1, pDev->bIntEn);
		WbaGpdmaStart(s_Tx.Channel, s_Tx.Data[0], I2S_DMA_FRAMES * s_Channels * s_AccessBytes);
		SAI1_Block_A->CR1 |= SAI_xCR1_DMAEN;
	}
	SAI1_Block_A->CLRFR = I2S_ERRORS;
	SAI1_Block_B->CLRFR = I2S_ERRORS;
	SAI1_Block_A->IMR = pDev->bIntEn ? I2S_ERRORS & ~SAI_xIMR_OVRUDRIE : 0;
	SAI1_Block_B->IMR = pDev->bIntEn && s_HasRx ? I2S_ERRORS : 0;
	if (pDev->bIntEn && !pDev->bDma)
	{
		if (s_pDev->hRxFifo)
		{
			SAI1_Block_B->IMR |= SAI_xIMR_FREQIE;
		}
		if (s_pDev->hTxFifo)
		{
			SAI1_Block_A->IMR |= SAI_xIMR_FREQIE;
		}
	}
	s_Enabled = true;
	if (s_HasRx)
	{
		SAI1_Block_B->CR1 |= SAI_xCR1_SAIEN;
	}
	// Empty transmit slots send silence while block A supplies capture clocks.
	if (s_Tx.Channel < 0)
	{
		SAI1_Block_A->DR = 0;
	}
	SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
}

static void I2sDisable(DevIntrf_t *pDev)
{
	if (I2sOwned(pDev))
	{
		I2sHalt();
		if (s_pDev->hRxFifo)
		{
			CFifoFlush(s_pDev->hRxFifo);
		}
		if (s_pDev->hTxFifo)
		{
			CFifoFlush(s_pDev->hTxFifo);
		}
	}
}

static void I2sPowerOff(DevIntrf_t *pDev)
{
	if (!I2sOwned(pDev))
	{
		return;
	}
	I2sDisable(pDev);
	WbaGpdmaFree(s_Rx.Channel);
	WbaGpdmaFree(s_Tx.Channel);
	RCC->CCIPR2 = (RCC->CCIPR2 & ~RCC_CCIPR2_SAI1SEL) | s_OldClock;
	RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
	WbaSaiRelease(s_pDev);
	s_pDev = nullptr;
	atomic_store(&pDev->EnCnt, 0);
}

static uint32_t I2sClockSource(uint32_t &Selection)
{
	Selection = RCC_CCIPR2_SAI1SEL_2;
	if (!(RCC->CR & RCC_CR_HSERDY))
	{
		return 16000000;
	}
	uint32_t source = 32000000U >> ((RCC->CR & RCC_CR_HSEPRE) >> RCC_CR_HSEPRE_Pos);
	uint32_t sws = RCC->CFGR1 & RCC_CFGR1_SWS;
	uint32_t cfgr = RCC->PLL1CFGR;

	if (sws == RCC_CFGR1_SWS_1)
	{
		Selection = RCC_CCIPR2_SAI1SEL_1;
		return source;
	}
	if (sws == (RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0) &&
		(RCC->CR & RCC_CR_PLL1RDY) &&
		(cfgr & RCC_PLL1CFGR_PLL1SRC) == (RCC_PLL1CFGR_PLL1SRC_1 | RCC_PLL1CFGR_PLL1SRC_0))
	{
		uint32_t divr = RCC->PLL1DIVR;
		uint32_t m = ((cfgr & RCC_PLL1CFGR_PLL1M) >> RCC_PLL1CFGR_PLL1M_Pos) + 1;
		uint32_t n = ((divr & RCC_PLL1DIVR_PLL1N) >> RCC_PLL1DIVR_PLL1N_Pos) + 1;
		uint32_t r = ((divr & RCC_PLL1DIVR_PLL1R) >> RCC_PLL1DIVR_PLL1R_Pos) + 1;
		uint32_t fraction = cfgr & RCC_PLL1CFGR_PLL1FRACEN ?
			(RCC->PLL1FRACR & RCC_PLL1FRACR_PLL1FRACN) >> RCC_PLL1FRACR_PLL1FRACN_Pos : 0;

		Selection = RCC_CCIPR2_SAI1SEL_1;
		return uint64_t(source) * (n * 8192U + fraction) / (uint64_t(m) * r * 8192U);
	}
	return 16000000;
}

static uint32_t I2sGetRate(DevIntrf_t *pDev)
{
	return I2sOwned(pDev) ? s_pDev->Freq : 0;
}

static uint32_t I2sSetRate(DevIntrf_t *pDev, uint32_t Rate)
{
	uint64_t difference = UINT64_MAX;
	uint32_t divider = 1;
	bool enabled;

	if (!I2sOwned(pDev))
	{
		return 0;
	}
	if (!Rate)
	{
		Rate = 16000;
	}
	for (uint32_t d = 1; d <= 63; d++)
	{
		uint32_t actual = s_KernelClock / (256U * d);
		uint64_t diff = actual > Rate ? uint64_t(actual) - Rate : uint64_t(Rate) - actual;

		if (diff < difference)
		{
			difference = diff;
			divider = d;
		}
	}
	enabled = s_Enabled;
	I2sDisable(pDev);
	s_Divider = divider;
	s_pDev->MClkFreq = s_pDev->Mode == I2SMODE_MASTER ? s_KernelClock / divider : 0;
	s_pDev->Freq = s_pDev->Mode == I2SMODE_MASTER ? s_KernelClock / (256U * divider) : Rate;
	if (enabled)
	{
		I2sEnable(pDev);
	}
	return s_pDev->Freq;
}

static bool I2sStartRx(DevIntrf_t *pDev, uint32_t DevAddr)
{
	(void)DevAddr;
	return I2sOwned(pDev) && s_Enabled && !s_Failed && s_HasRx;
}

static bool I2sStartTx(DevIntrf_t *pDev, uint32_t DevAddr)
{
	(void)DevAddr;
	return I2sOwned(pDev) && s_Enabled && !s_Failed && s_HasTx;
}

static void I2sStop(DevIntrf_t *pDev)
{
	(void)pDev;
}

static void I2sPoll()
{
	I2sServiceFifo();
	if (s_Enabled && s_pDev->DevIntrf.bDma)
	{
		if (s_Rx.Channel >= 0)
		{
			I2sServiceDma(s_Rx.Channel);
		}
		if (s_Enabled && s_Tx.Channel >= 0)
		{
			I2sServiceDma(s_Tx.Channel);
		}
	}
}

static int I2sRxData(DevIntrf_t *pDev, uint8_t *pBuff, int BuffLen)
{
	int count = 0;

	if (!I2sStartRx(pDev, 0) || !pBuff || BuffLen <= 0 || unsigned(BuffLen) % s_FrameBytes)
	{
		return 0;
	}
	if (!pDev->bIntEn)
	{
		I2sPoll();
	}
	if (s_Failed)
	{
		return 0;
	}
	if (s_pDev->hRxFifo)
	{
		while (count < BuffLen)
		{
			uint8_t *data = CFifoPeek(s_pDev->hRxFifo);

			if (!data)
			{
				break;
			}
			memcpy(pBuff + count, data, s_FrameBytes);
			CFifoRelease(s_pDev->hRxFifo, 1);
			count += s_FrameBytes;
		}
		return count;
	}
	while (count < BuffLen)
	{
		for (unsigned sample = 0; sample < s_Channels; sample++)
		{
			unsigned retry = 0;

			while (!(SAI1_Block_B->SR & SAI_xSR_FLVL) && retry++ < I2S_DATA_POLLS)
			{
			}
			if (!(SAI1_Block_B->SR & SAI_xSR_FLVL))
			{
				I2sFail(DEVINTRF_EVT_RX_TIMEOUT);
				return count;
			}
			I2sPack(pBuff + count + sample * s_pDev->SampleWidth, SAI1_Block_B->DR);
		}
		count += s_FrameBytes;
	}
	return count;
}

static int I2sTxData(DevIntrf_t *pDev, const uint8_t *pData, int DataLen)
{
	int count = 0;

	if (!I2sStartTx(pDev, 0) || !pData || DataLen <= 0 || unsigned(DataLen) % s_FrameBytes)
	{
		return 0;
	}
	if (!pDev->bIntEn)
	{
		I2sPoll();
	}
	while (count < DataLen && s_Enabled)
	{
		if (s_pDev->hTxFifo)
		{
			uint8_t *data = CFifoResv(s_pDev->hTxFifo);

			if (!data)
			{
				break;
			}
			memcpy(data, pData + count, s_FrameBytes);
			CFifoPut(s_pDev->hTxFifo);
		}
		else
		{
			for (unsigned sample = 0; sample < s_Channels; sample++)
			{
				unsigned retry = 0;

				while ((SAI1_Block_A->SR & SAI_xSR_FLVL) == (SAI_xSR_FLVL_2 | SAI_xSR_FLVL_0) && retry++ < I2S_DATA_POLLS)
				{
				}
				if ((SAI1_Block_A->SR & SAI_xSR_FLVL) == (SAI_xSR_FLVL_2 | SAI_xSR_FLVL_0))
				{
					I2sFail(DEVINTRF_EVT_TX_TIMEOUT);
					return count;
				}
				SAI1_Block_A->DR = I2sUnpack(pData + count + sample * s_pDev->SampleWidth);
			}
		}
		count += s_FrameBytes;
	}
	return count;
}

static void I2sReset(DevIntrf_t *pDev)
{
	if (!I2sOwned(pDev))
	{
		return;
	}
	I2sDisable(pDev);
	s_Failed = false;
	if (atomic_load(&pDev->EnCnt) > 0)
	{
		I2sEnable(pDev);
	}
}

static void *I2sGetHandle(DevIntrf_t *pDev)
{
	return I2sOwned(pDev) ? s_pDev : nullptr;
}

static bool I2sPin(const IOPinCfg_t &Pin, unsigned Index)
{
	if (Pin.PinOp != IOPINOP_FUNC13)
	{
		return false;
	}
	for (const auto &route : s_PinRoutes)
	{
		if (Index == route.Index && Pin.PortNo == route.Port && Pin.PinNo == route.Pin)
		{
			return true;
		}
	}
	return false;
}

bool I2SInit(I2SDev_t *const pDev, const I2SCfg_t *const pCfg)
{
	bool hasRx;
	bool hasTx;
	unsigned frameBytes;

	if (!pDev || !pCfg || pCfg->DevNo != 0 || !pCfg->pIOPinMap ||
		pCfg->NbIOPins < 3 || pCfg->NbIOPins > I2S_MAX_NB_IOPIN ||
		(pCfg->Mode != I2SMODE_MASTER && pCfg->Mode != I2SMODE_SLAVE) ||
		(pCfg->Format != I2SFMT_NORMAL && pCfg->Format != I2SFMT_ALIGN_LEFT && pCfg->Format != I2SFMT_ALIGN_RIGHT) ||
		(pCfg->SampleWidth != I2SSWIDTH_8BITS && pCfg->SampleWidth != I2SSWIDTH_16BITS && pCfg->SampleWidth != I2SSWIDTH_24BITS) ||
		(pCfg->Chan != I2SCHAN_STEREO && pCfg->Chan != I2SCHAN_LEFT && pCfg->Chan != I2SCHAN_RIGHT) ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}
	const IOPinCfg_t *pins = pCfg->pIOPinMap;

	hasRx = pins[I2S_SDI_IOPIN_IDX].PortNo >= 0;
	hasTx = pCfg->NbIOPins > I2S_SDO_IOPIN_IDX && pins[I2S_SDO_IOPIN_IDX].PortNo >= 0;
	if (!hasRx && !hasTx)
	{
		return false;
	}
	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		if ((i < 2 || pins[i].PortNo >= 0) && !I2sPin(pins[i], i))
		{
			return false;
		}
		if (pins[i].PortNo >= 0)
		{
			for (int j = 0; j < i; j++)
			{
				if (pins[i].PortNo == pins[j].PortNo && pins[i].PinNo == pins[j].PinNo)
				{
					return false;
				}
			}
		}
	}
	if (pCfg->pRxFifoMem && pCfg->pTxFifoMem &&
		pCfg->RxFifoMemSize > 0 && pCfg->TxFifoMemSize > 0 &&
		uint64_t(uintptr_t(pCfg->pRxFifoMem)) < uint64_t(uintptr_t(pCfg->pTxFifoMem)) + pCfg->TxFifoMemSize &&
		uint64_t(uintptr_t(pCfg->pTxFifoMem)) < uint64_t(uintptr_t(pCfg->pRxFifoMem)) + pCfg->RxFifoMemSize)
	{
		return false;
	}
	frameBytes = (pCfg->SampleWidth / 8) * (pCfg->Chan == I2SCHAN_STEREO ? 2 : 1);
	if ((hasRx && (pCfg->bIntEn || pCfg->bDmaEn) && !pCfg->pRxFifoMem) ||
		(pCfg->pRxFifoMem && (!hasRx || (uintptr_t(pCfg->pRxFifoMem) & 3U) || pCfg->RxFifoMemSize < int(sizeof(CFifo_t) + frameBytes * (pCfg->bDmaEn ? I2S_DMA_FRAMES : 1)))) ||
		(pCfg->pTxFifoMem && (!hasTx || (uintptr_t(pCfg->pTxFifoMem) & 3U) || pCfg->TxFifoMemSize < int(sizeof(CFifo_t) + frameBytes))) ||
		!WbaSaiClaim(pDev, I2sSaiHandler))
	{
		return false;
	}
	s_Rx.Channel = -1;
	s_Tx.Channel = -1;
	if (pCfg->bDmaEn)
	{
		if (hasRx)
		{
			s_Rx.Channel = WbaGpdmaAlloc(I2sDmaHandler, pDev, pCfg->IntPrio, pCfg->bIntEn);
		}
		if (pCfg->pTxFifoMem)
		{
			s_Tx.Channel = WbaGpdmaAlloc(I2sDmaHandler, pDev, pCfg->IntPrio, pCfg->bIntEn);
		}
		if ((hasRx && s_Rx.Channel < 0) || (pCfg->pTxFifoMem && s_Tx.Channel < 0))
		{
			WbaGpdmaFree(s_Rx.Channel);
			WbaGpdmaFree(s_Tx.Channel);
			WbaSaiRelease(pDev);
			return false;
		}
	}
	uint32_t selection;

	s_KernelClock = I2sClockSource(selection);
	if (selection == RCC_CCIPR2_SAI1SEL_2)
	{
		RCC->CR |= RCC_CR_HSION;
		for (unsigned i = 0; i < I2S_STOP_POLLS && !(RCC->CR & RCC_CR_HSIRDY); i++)
		{
		}
	}
	if (selection == RCC_CCIPR2_SAI1SEL_2 && !(RCC->CR & RCC_CR_HSIRDY))
	{
		WbaGpdmaFree(s_Rx.Channel);
		WbaGpdmaFree(s_Tx.Channel);
		WbaSaiRelease(pDev);
		return false;
	}
	s_pDev = pDev;
	s_HasRx = hasRx;
	s_HasTx = hasTx;
	s_Channels = pCfg->Chan == I2SCHAN_STEREO ? 2 : 1;
	s_FrameBytes = frameBytes;
	s_AccessBytes = pCfg->SampleWidth == I2SSWIDTH_24BITS ? 4 : pCfg->SampleWidth / 8;
	s_Enabled = false;
	s_Failed = false;
	pDev->Mode = pCfg->Mode;
	pDev->Format = pCfg->Format;
	pDev->SampleWidth = pCfg->SampleWidth / 8;
	pDev->Chan = pCfg->Chan;
	pDev->pIOPinMap = pins;
	pDev->NbIOPins = pCfg->NbIOPins;
	pDev->hRxFifo = pCfg->pRxFifoMem ? CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, frameBytes, true) : nullptr;
	pDev->hTxFifo = pCfg->pTxFifoMem ? CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, frameBytes, true) : nullptr;
	pDev->EvtCB = pCfg->EvtCB;
	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = pDev;
	intrf->Type = DEVINTRF_TYPE_I2S;
	intrf->bDma = pCfg->bDmaEn;
	intrf->bIntEn = pCfg->bIntEn;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = I2sEnable;
	intrf->Disable = I2sDisable;
	intrf->PowerOff = I2sPowerOff;
	intrf->Reset = I2sReset;
	intrf->GetRate = I2sGetRate;
	intrf->SetRate = I2sSetRate;
	intrf->StartRx = I2sStartRx;
	intrf->StartTx = I2sStartTx;
	intrf->StopRx = I2sStop;
	intrf->StopTx = I2sStop;
	intrf->RxData = I2sRxData;
	intrf->TxData = I2sTxData;
	intrf->TxSrData = nullptr;
	intrf->GetHandle = I2sGetHandle;
	atomic_flag_clear(&intrf->bBusy);
	atomic_store(&intrf->EnCnt, 1);
	atomic_store(&intrf->bTxReady, true);
	atomic_store(&intrf->bNoStop, false);
	s_OldClock = RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL;
	RCC->CCIPR2 = (RCC->CCIPR2 & ~RCC_CCIPR2_SAI1SEL) | selection;
	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
	(void)RCC->APB2ENR;
	RCC->APB2RSTR |= RCC_APB2RSTR_SAI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SAI1RST;
	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		if (pins[i].PortNo >= 0)
		{
			IOPinCfg(&pins[i], 1);
		}
	}
	I2sSetRate(intrf, pCfg->Freq);
	NVIC_ClearPendingIRQ(SAI1_IRQn);
	NVIC_SetPriority(SAI1_IRQn, pCfg->IntPrio);
	if (pCfg->bIntEn)
	{
		NVIC_EnableIRQ(SAI1_IRQn);
	}
	I2sEnable(intrf);
	return true;
}
