/**-------------------------------------------------------------------------
@file	i2s_lpc546xx.cpp

@brief	LPC546xx I2S on Flexcomm 6 and 7

See i2s_lpc546xx.h. Register use as the NXP SDK fsl_i2s.c and fsl_i2s_dma.c:
classic I2S mode, normal master or slave (CFG1 MSTSLVCFG 3 or 0), 16 bit
data in a 32 bit frame, one FIFO entry a stereo frame with the left channel
in bits 15:0 and the right channel in bits 31:16, the TX FIFO sending 0 when
it runs empty (FIFOCFG TXI2SE0). The DMA moves 32 bit FIFO entries, with the
Flexcomm requests.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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

#include "cfifo.h"
#include "coredev/iopincfg.h"
#include "coredev/i2s.h"
#include "coredev/interrupt.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"
#include "dma_lpc546xx.h"
#include "clock_lpc546xx.h"
#include "i2s_lpc546xx.h"

#ifndef LPC546XX_I2S_DATA_POLLS
// FIFOSTAT reads before a polling transfer gives up
#define LPC546XX_I2S_DATA_POLLS		1000000U
#endif

#define LPC546XX_I2S_FC0			6			// I2S0
#define LPC546XX_I2S_FC1			7			// I2S1
#define LPC546XX_I2S_FIFO_DEPTH		8U
#define LPC546XX_I2S_TXLVL			4U			// TX interrupt and DMA with 4 entries or less
#define LPC546XX_I2S_RXLVL			3U			// RX interrupt with more than 3 entries
#define LPC546XX_I2S_DMA_FRAMES		32U
#define LPC546XX_I2S_DEFAULT_RATE	16000U
#define LPC546XX_I2S_MCLK_FS		256U		// MCLK / Fs
#define LPC546XX_I2S_SCK_FS			32U			// SCK / Fs, 2 x 16 bits
#define LPC546XX_I2S_PLL_MIN		12000000U	// Lowest audio PLL output asked
#define LPC546XX_I2S_K_MAX			256U		// MCLKDIV DIV + 1
#define LPC546XX_FCLKSEL_AUDPLL		2U
#define LPC546XX_FCLKSEL_NONE		7U
#define LPC546XX_MCLKCLKSEL_AUDPLL	1U
#define LPC546XX_MCLKCLKSEL_NONE	7U
#define LPC546XX_MCLKIO_OUT			1U
#define LPC546XX_I2S_MSTSLV_SLAVE	0U
#define LPC546XX_I2S_MSTSLV_MASTER	3U

typedef struct {
	alignas(4) uint32_t Data[2][LPC546XX_I2S_DMA_FRAMES];
	int Ch;						// DMA channel, -1 without DMA
	unsigned Index;				// Buffer the DMA moves
	unsigned Bytes[2];			// TX: CFIFO bytes in each buffer
} Lpc546xxI2sDma_t;

typedef struct {
	I2SDev_t *pDev;
	int TxFc;					// Sending Flexcomm, -1 none
	int RxFc;					// Receiving Flexcomm, -1 none
	bool bMaster;				// TxFc, or RxFc alone, makes SCK and WS
	bool bMclk;					// MCLK pin used
	volatile bool bEnabled;
	unsigned FrameBytes;		// 4 stereo, 2 mono
	uint32_t K;					// Audio PLL / MCLK
	Lpc546xxI2sDma_t Rx;
	Lpc546xxI2sDma_t Tx;
} Lpc546xxI2s_t;

static Lpc546xxI2s_t s_Lpc546xxI2s;

static I2S_Type *Lpc546xxI2sReg(int FcNo)
{
	return FcNo == LPC546XX_I2S_FC0 ? I2S0 : I2S1;
}

static bool Lpc546xxI2sOwned(DevIntrf_t * const pDev)
{
	return s_Lpc546xxI2s.pDev != NULL && pDev == &s_Lpc546xxI2s.pDev->DevIntrf;
}

static void Lpc546xxI2sNotify(DEVINTRF_EVT Evt, int Count)
{
	DevIntrf_t *intrf = &s_Lpc546xxI2s.pDev->DevIntrf;

	if (intrf->EvtCB != NULL)
	{
		intrf->EvtCB(intrf, Evt, NULL, Count);
	}
}

// CFIFO frame to a FIFO entry, left in bits 15:0
static uint32_t Lpc546xxI2sEntry(const uint8_t *pFrame)
{
	uint16_t s[2] = { 0, 0 };

	if (s_Lpc546xxI2s.FrameBytes == 4U)
	{
		memcpy(s, pFrame, 4);
	}
	else
	{
		memcpy(&s[s_Lpc546xxI2s.pDev->Chan == I2SCHAN_RIGHT ? 1 : 0], pFrame, 2);
	}

	return (uint32_t)s[0] | ((uint32_t)s[1] << 16);
}

// FIFO entry to a CFIFO frame
static void Lpc546xxI2sFrame(uint32_t Entry, uint8_t *pFrame)
{
	uint16_t s[2] = { (uint16_t)Entry, (uint16_t)(Entry >> 16) };

	if (s_Lpc546xxI2s.FrameBytes == 4U)
	{
		memcpy(pFrame, s, 4);
	}
	else
	{
		memcpy(pFrame, &s[s_Lpc546xxI2s.pDev->Chan == I2SCHAN_RIGHT ? 1 : 0], 2);
	}
}

// Stop both Flexcomms and the DMA, empty the hardware FIFOs
static void Lpc546xxI2sHalt(void)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	int fc[2] = { dev->TxFc, dev->RxFc };

	dev->bEnabled = false;

	// The master first, a slave stops on its own frame boundary
	for (int i = 0; i < 2; i++)
	{
		if (fc[i] >= 0)
		{
			I2S_Type *reg = Lpc546xxI2sReg(fc[i]);

			reg->FIFOINTENCLR = I2S_FIFOINTENSET_TXERR_MASK | I2S_FIFOINTENSET_RXERR_MASK |
								I2S_FIFOINTENSET_TXLVL_MASK | I2S_FIFOINTENSET_RXLVL_MASK;
			reg->CFG1 &= ~I2S_CFG1_MAINENABLE_MASK;
		}
	}
	if (dev->Rx.Ch >= 0)
	{
		Lpc546xxDmaChStop(dev->Rx.Ch);
	}
	if (dev->Tx.Ch >= 0)
	{
		Lpc546xxDmaChStop(dev->Tx.Ch);
	}
	for (int i = 0; i < 2; i++)
	{
		if (fc[i] >= 0)
		{
			I2S_Type *reg = Lpc546xxI2sReg(fc[i]);

			reg->FIFOCFG = (reg->FIFOCFG & ~(I2S_FIFOCFG_DMATX_MASK | I2S_FIFOCFG_DMARX_MASK)) |
						   I2S_FIFOCFG_EMPTYTX_MASK | I2S_FIFOCFG_EMPTYRX_MASK;
			reg->FIFOSTAT = I2S_FIFOSTAT_TXERR_MASK | I2S_FIFOSTAT_RXERR_MASK;
			NVIC_ClearPendingIRQ(Lpc546xxFlexcommIrqNo(fc[i]));
		}
	}
}

static void Lpc546xxI2sFail(DEVINTRF_EVT Evt)
{
	if (s_Lpc546xxI2s.bEnabled)
	{
		Lpc546xxI2sHalt();
		Lpc546xxI2sNotify(Evt, 0);
	}
}

static bool Lpc546xxI2sPublish(uint32_t Entry)
{
	uint8_t *dest = CFifoResv(s_Lpc546xxI2s.pDev->hRxFifo);

	if (dest == NULL)
	{
		Lpc546xxI2sFail(DEVINTRF_EVT_RX_FIFO_FULL);

		return false;
	}
	Lpc546xxI2sFrame(Entry, dest);
	CFifoPut(s_Lpc546xxI2s.pDev->hRxFifo);

	return true;
}

// The next TX entry: a CFIFO frame, silence when it has none. pCnt counts
// the CFIFO bytes.
static uint32_t Lpc546xxI2sNextTx(unsigned *pCnt)
{
	uint8_t *data = CFifoPeek(s_Lpc546xxI2s.pDev->hTxFifo);

	if (data == NULL)
	{
		return 0;
	}

	uint32_t e = Lpc546xxI2sEntry(data);

	CFifoRelease(s_Lpc546xxI2s.pDev->hTxFifo, 1);
	*pCnt += s_Lpc546xxI2s.FrameBytes;

	return e;
}

// A receive FIFO overrun with the interrupts or the DMA moving the frames
// stops the stream. Without them, frames lost between the calls are not an
// error.
static bool Lpc546xxI2sRxError(I2S_Type *pReg)
{
	if ((pReg->FIFOSTAT & I2S_FIFOSTAT_RXERR_MASK) == 0U ||
		(s_Lpc546xxI2s.pDev->DevIntrf.bIntEn == false && s_Lpc546xxI2s.Rx.Ch < 0))
	{
		return false;
	}
	Lpc546xxI2sFail(DEVINTRF_EVT_STATECHG);

	return true;
}

// Frames between the hardware FIFOs and the CFIFOs, at most one hardware
// FIFO each call, keeping the interrupt work bounded
static void Lpc546xxI2sServiceFifo(void)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	bool received = false;

	if (dev->bEnabled == false)
	{
		return;
	}
	if (dev->RxFc >= 0 && Lpc546xxI2sRxError(Lpc546xxI2sReg(dev->RxFc)))
	{
		return;
	}
	if (dev->RxFc >= 0 && dev->pDev->hRxFifo != NULL && dev->Rx.Ch < 0)
	{
		I2S_Type *reg = Lpc546xxI2sReg(dev->RxFc);

		for (unsigned i = 0; i < LPC546XX_I2S_FIFO_DEPTH && (reg->FIFOSTAT & I2S_FIFOSTAT_RXNOTEMPTY_MASK); i++)
		{
			if (Lpc546xxI2sPublish(reg->FIFORD) == false)
			{
				return;
			}
			received = true;
		}
	}
	if (dev->TxFc >= 0 && dev->pDev->hTxFifo != NULL && dev->Tx.Ch < 0)
	{
		I2S_Type *reg = Lpc546xxI2sReg(dev->TxFc);
		unsigned cnt = 0;

		for (unsigned i = 0; i < LPC546XX_I2S_FIFO_DEPTH && (reg->FIFOSTAT & I2S_FIFOSTAT_TXNOTFULL_MASK); i++)
		{
			reg->FIFOWR = Lpc546xxI2sNextTx(&cnt);
		}
		if (cnt > 0U)
		{
			Lpc546xxI2sNotify(DEVINTRF_EVT_TX_READY, CFifoUsed(dev->pDev->hTxFifo) * (int)dev->FrameBytes);
		}
	}
	if (received)
	{
		Lpc546xxI2sNotify(DEVINTRF_EVT_RX_DATA, CFifoUsed(dev->pDev->hRxFifo) * (int)dev->FrameBytes);
	}
}

// A TX DMA buffer from the CFIFO, silence after its last frame. Returns the
// CFIFO bytes taken.
static unsigned Lpc546xxI2sTxBlock(uint32_t *pBuf)
{
	unsigned cnt = 0;

	for (unsigned i = 0; i < LPC546XX_I2S_DMA_FRAMES; i++)
	{
		pBuf[i] = Lpc546xxI2sNextTx(&cnt);
	}

	return cnt;
}

static bool Lpc546xxI2sRxBlock(const uint32_t *pBuf)
{
	for (unsigned i = 0; i < LPC546XX_I2S_DMA_FRAMES; i++)
	{
		if (Lpc546xxI2sPublish(pBuf[i]) == false)
		{
			return false;
		}
	}

	return true;
}

// End of DMA blocks: each channel done restarts on its other buffer before
// the frames of the finished one are moved
static void Lpc546xxI2sServiceDma(void)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	uint32_t rxf = dev->Rx.Ch >= 0 ? Lpc546xxDmaChFlags(dev->Rx.Ch) : 0U;
	uint32_t txf = dev->Tx.Ch >= 0 ? Lpc546xxDmaChFlags(dev->Tx.Ch) : 0U;

	if (dev->bEnabled == false)
	{
		return;
	}
	if ((rxf | txf) & LPC546XX_DMA_FLAG_ERR)
	{
		Lpc546xxI2sFail(DEVINTRF_EVT_STATECHG);

		return;
	}

	unsigned rxi = dev->Rx.Index;
	unsigned txi = dev->Tx.Index;

	if (rxf & LPC546XX_DMA_FLAG_DONE)
	{
		dev->Rx.Index ^= 1U;
		Lpc546xxDmaChStart(dev->Rx.Ch, dev->Rx.Data[dev->Rx.Index], sizeof(dev->Rx.Data[0]));
	}
	if (txf & LPC546XX_DMA_FLAG_DONE)
	{
		dev->Tx.Index ^= 1U;
		Lpc546xxDmaChStart(dev->Tx.Ch, dev->Tx.Data[dev->Tx.Index], sizeof(dev->Tx.Data[0]));

		unsigned done = dev->Tx.Bytes[txi];

		dev->Tx.Bytes[txi] = Lpc546xxI2sTxBlock(dev->Tx.Data[txi]);
		if (done > 0U)
		{
			if (dev->Tx.Bytes[0] == 0U && dev->Tx.Bytes[1] == 0U)
			{
				Lpc546xxI2sNotify(DEVINTRF_EVT_TX_FIFO_EMPTY, 0);
			}
			Lpc546xxI2sNotify(DEVINTRF_EVT_TX_READY, CFifoUsed(dev->pDev->hTxFifo) * (int)dev->FrameBytes);
		}
	}
	if ((rxf & LPC546XX_DMA_FLAG_DONE) && Lpc546xxI2sRxBlock(dev->Rx.Data[rxi]))
	{
		Lpc546xxI2sNotify(DEVINTRF_EVT_RX_DATA, CFifoUsed(dev->pDev->hRxFifo) * (int)dev->FrameBytes);
	}
}

static void Lpc546xxI2sDmaHandler(int Ch, void *pCtx)
{
	(void)Ch;
	(void)pCtx;

	Lpc546xxI2sServiceDma();
}

static void Lpc546xxI2sIrqHandler(int FcNo, DevIntrf_t * const pDev)
{
	(void)FcNo;

	if (Lpc546xxI2sOwned(pDev))
	{
		Lpc546xxI2sServiceFifo();
	}
}

// Transfers without interrupt: the frames move when the application calls
static void Lpc546xxI2sPoll(void)
{
	Lpc546xxI2sServiceFifo();
	if (s_Lpc546xxI2s.bEnabled && (s_Lpc546xxI2s.Rx.Ch >= 0 || s_Lpc546xxI2s.Tx.Ch >= 0))
	{
		Lpc546xxI2sServiceDma();
	}
}

// Configuration of one Flexcomm, not enabled
static void Lpc546xxI2sConfig(int FcNo, bool bTx, bool bMaster, uint32_t Div)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	I2S_Type *reg = Lpc546xxI2sReg(FcNo);
	DevIntrf_t *intrf = &dev->pDev->DevIntrf;
	bool dma = bTx ? dev->Tx.Ch >= 0 : dev->Rx.Ch >= 0;
	bool cfifo = (bTx ? dev->pDev->hTxFifo : dev->pDev->hRxFifo) != NULL;

	reg->CFG1 = I2S_CFG1_MSTSLVCFG(bMaster ? LPC546XX_I2S_MSTSLV_MASTER : LPC546XX_I2S_MSTSLV_SLAVE) |
				I2S_CFG1_MODE(0) | I2S_CFG1_DATALEN(15);
	reg->CFG2 = I2S_CFG2_FRAMELEN(LPC546XX_I2S_SCK_FS - 1U) | I2S_CFG2_POSITION(0);
	reg->DIV = I2S_DIV_DIV(Div - 1U);
	if (bTx)
	{
		reg->FIFOCFG = I2S_FIFOCFG_ENABLETX_MASK | I2S_FIFOCFG_EMPTYTX_MASK | I2S_FIFOCFG_TXI2SE0_MASK |
					   (dma ? I2S_FIFOCFG_DMATX_MASK : 0U);
		reg->FIFOTRIG = I2S_FIFOTRIG_TXLVLENA_MASK | I2S_FIFOTRIG_TXLVL(LPC546XX_I2S_TXLVL);
	}
	else
	{
		reg->FIFOCFG = I2S_FIFOCFG_ENABLERX_MASK | I2S_FIFOCFG_EMPTYRX_MASK | (dma ? I2S_FIFOCFG_DMARX_MASK : 0U);
		reg->FIFOTRIG = I2S_FIFOTRIG_RXLVLENA_MASK | I2S_FIFOTRIG_RXLVL(LPC546XX_I2S_RXLVL);
	}
	reg->FIFOSTAT = I2S_FIFOSTAT_TXERR_MASK | I2S_FIFOSTAT_RXERR_MASK;
	if (intrf->bIntEn)
	{
		uint32_t inten = 0;

		if (bTx == false && cfifo)
		{
			inten |= I2S_FIFOINTENSET_RXERR_MASK | (dma ? 0U : I2S_FIFOINTENSET_RXLVL_MASK);
		}
		if (bTx && cfifo && dma == false)
		{
			inten |= I2S_FIFOINTENSET_TXLVL_MASK;
		}
		reg->FIFOINTENSET = inten;
	}
}

static void Lpc546xxI2sEnable(DevIntrf_t * const pDev)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;

	if (Lpc546xxI2sOwned(pDev) == false || dev->bEnabled)
	{
		return;
	}

	uint32_t div = LPC546XX_I2S_MCLK_FS * dev->K / LPC546XX_I2S_SCK_FS;

	if (dev->RxFc >= 0)
	{
		Lpc546xxI2sConfig(dev->RxFc, false, dev->bMaster && dev->TxFc < 0, div);
		dev->Rx.Index = 0;
		if (dev->Rx.Ch >= 0)
		{
			Lpc546xxDmaChSetup(dev->Rx.Ch, false, &Lpc546xxI2sReg(dev->RxFc)->FIFORD, 2);
			Lpc546xxDmaChStart(dev->Rx.Ch, dev->Rx.Data[0], sizeof(dev->Rx.Data[0]));
		}
	}
	if (dev->TxFc >= 0)
	{
		I2S_Type *reg = Lpc546xxI2sReg(dev->TxFc);

		Lpc546xxI2sConfig(dev->TxFc, true, dev->bMaster, div);
		dev->Tx.Index = 0;
		if (dev->Tx.Ch >= 0)
		{
			dev->Tx.Bytes[0] = Lpc546xxI2sTxBlock(dev->Tx.Data[0]);
			dev->Tx.Bytes[1] = Lpc546xxI2sTxBlock(dev->Tx.Data[1]);
			Lpc546xxDmaChSetup(dev->Tx.Ch, true, &reg->FIFOWR, 2);
			Lpc546xxDmaChStart(dev->Tx.Ch, dev->Tx.Data[0], sizeof(dev->Tx.Data[0]));
		}
		else
		{
			// Silence first, the frames follow
			for (unsigned i = 0; i < LPC546XX_I2S_FIFO_DEPTH; i++)
			{
				reg->FIFOWR = 0U;
			}
		}
	}

	dev->bEnabled = true;

	// The slave first, then the master starts SCK and WS
	if (dev->RxFc >= 0 && dev->TxFc >= 0)
	{
		Lpc546xxI2sReg(dev->RxFc)->CFG1 |= I2S_CFG1_MAINENABLE_MASK;
		Lpc546xxI2sReg(dev->TxFc)->CFG1 |= I2S_CFG1_MAINENABLE_MASK;
	}
	else
	{
		Lpc546xxI2sReg(dev->TxFc >= 0 ? dev->TxFc : dev->RxFc)->CFG1 |= I2S_CFG1_MAINENABLE_MASK;
	}
}

static void Lpc546xxI2sDisable(DevIntrf_t * const pDev)
{
	if (Lpc546xxI2sOwned(pDev))
	{
		Lpc546xxI2sHalt();
		if (s_Lpc546xxI2s.pDev->hRxFifo != NULL)
		{
			CFifoFlush(s_Lpc546xxI2s.pDev->hRxFifo);
		}
		if (s_Lpc546xxI2s.pDev->hTxFifo != NULL)
		{
			CFifoFlush(s_Lpc546xxI2s.pDev->hTxFifo);
		}
	}
}

// The audio PLL for Freq, the MCLK divider and the rate the Flexcomms make
static uint32_t Lpc546xxI2sClock(uint32_t Freq)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	uint64_t mclk = (uint64_t)Freq * LPC546XX_I2S_MCLK_FS;
	uint32_t k = (uint32_t)((LPC546XX_I2S_PLL_MIN + mclk - 1U) / mclk);

	if (k > LPC546XX_I2S_K_MAX)
	{
		k = LPC546XX_I2S_K_MAX;
	}

	// The PLL gives its closest output, 0 when it failed
	uint32_t pll = Lpc546xxAudioPllStart(mclk * k > 0xFFFFFFFFU ? 0xFFFFFFFFU : (uint32_t)(mclk * k));

	dev->K = k;
	if (dev->bMclk)
	{
		SYSCON->MCLKCLKSEL = SYSCON_MCLKCLKSEL_SEL(LPC546XX_MCLKCLKSEL_AUDPLL);
		SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(k - 1U);
		SYSCON->MCLKIO = SYSCON_MCLKIO_DIR(LPC546XX_MCLKIO_OUT);
	}
	dev->pDev->MClkFreq = dev->bMaster ? pll / k : 0U;

	return pll / (LPC546XX_I2S_MCLK_FS * k);
}

static uint32_t Lpc546xxI2sGetRate(DevIntrf_t * const pDev)
{
	return Lpc546xxI2sOwned(pDev) ? s_Lpc546xxI2s.pDev->Freq : 0U;
}

// The closest rate the audio PLL makes. The stream stops while the clocks
// change.
static uint32_t Lpc546xxI2sSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	if (Lpc546xxI2sOwned(pDev) == false)
	{
		return 0;
	}
	if (Rate == 0U)
	{
		Rate = LPC546XX_I2S_DEFAULT_RATE;
	}

	bool enabled = s_Lpc546xxI2s.bEnabled;

	Lpc546xxI2sHalt();

	uint32_t f = Lpc546xxI2sClock(Rate);

	if (f == 0U)
	{
		return 0;
	}
	s_Lpc546xxI2s.pDev->Freq = f;
	if (enabled)
	{
		Lpc546xxI2sEnable(pDev);
	}

	return f;
}

static bool Lpc546xxI2sStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)DevAddr;

	return Lpc546xxI2sOwned(pDev) && s_Lpc546xxI2s.bEnabled && s_Lpc546xxI2s.RxFc >= 0;
}

static bool Lpc546xxI2sStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)DevAddr;

	return Lpc546xxI2sOwned(pDev) && s_Lpc546xxI2s.bEnabled && s_Lpc546xxI2s.TxFc >= 0;
}

static void Lpc546xxI2sStop(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static int Lpc546xxI2sRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	int count = 0;

	if (Lpc546xxI2sStartRx(pDev, 0) == false || pBuff == NULL || BuffLen <= 0 ||
		(unsigned)BuffLen % dev->FrameBytes != 0U)
	{
		return 0;
	}
	if (pDev->bIntEn == false)
	{
		Lpc546xxI2sPoll();
	}
	if (dev->bEnabled == false)
	{
		return 0;
	}
	if (dev->pDev->hRxFifo != NULL)
	{
		while (count < BuffLen)
		{
			uint8_t *data = CFifoPeek(dev->pDev->hRxFifo);

			if (data == NULL)
			{
				break;
			}
			memcpy(pBuff + count, data, dev->FrameBytes);
			CFifoRelease(dev->pDev->hRxFifo, 1);
			count += (int)dev->FrameBytes;
		}

		return count;
	}

	I2S_Type *reg = Lpc546xxI2sReg(dev->RxFc);

	while (count < BuffLen)
	{
		uint32_t n = 0;

		while ((reg->FIFOSTAT & I2S_FIFOSTAT_RXNOTEMPTY_MASK) == 0U && ++n < LPC546XX_I2S_DATA_POLLS);
		if (n >= LPC546XX_I2S_DATA_POLLS)
		{
			Lpc546xxI2sFail(DEVINTRF_EVT_RX_TIMEOUT);

			return count;
		}
		Lpc546xxI2sFrame(reg->FIFORD, pBuff + count);
		count += (int)dev->FrameBytes;
	}

	return count;
}

static int Lpc546xxI2sTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	int count = 0;

	if (Lpc546xxI2sStartTx(pDev, 0) == false || pData == NULL || DataLen <= 0 ||
		(unsigned)DataLen % dev->FrameBytes != 0U)
	{
		return 0;
	}
	if (pDev->bIntEn == false)
	{
		Lpc546xxI2sPoll();
	}
	if (dev->pDev->hTxFifo != NULL)
	{
		while (count < DataLen)
		{
			uint8_t *data = CFifoResv(dev->pDev->hTxFifo);

			if (data == NULL)
			{
				break;
			}
			memcpy(data, pData + count, dev->FrameBytes);
			CFifoPut(dev->pDev->hTxFifo);
			count += (int)dev->FrameBytes;
		}

		return count;
	}

	I2S_Type *reg = Lpc546xxI2sReg(dev->TxFc);

	while (count < DataLen)
	{
		uint32_t n = 0;

		while ((reg->FIFOSTAT & I2S_FIFOSTAT_TXNOTFULL_MASK) == 0U && ++n < LPC546XX_I2S_DATA_POLLS);
		if (n >= LPC546XX_I2S_DATA_POLLS)
		{
			Lpc546xxI2sFail(DEVINTRF_EVT_TX_TIMEOUT);

			return count;
		}
		reg->FIFOWR = Lpc546xxI2sEntry(pData + count);
		count += (int)dev->FrameBytes;
	}

	return count;
}

static void Lpc546xxI2sReset(DevIntrf_t * const pDev)
{
	if (Lpc546xxI2sOwned(pDev) == false)
	{
		return;
	}
	Lpc546xxI2sDisable(pDev);
	if (atomic_load(&pDev->EnCnt) > 0)
	{
		Lpc546xxI2sEnable(pDev);
	}
}

static void *Lpc546xxI2sGetHandle(DevIntrf_t * const pDev)
{
	return Lpc546xxI2sOwned(pDev) ? s_Lpc546xxI2s.pDev : NULL;
}

static void Lpc546xxI2sRelease(void)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;
	int fc[2] = { dev->TxFc, dev->RxFc };

	for (int i = 0; i < 2; i++)
	{
		if (fc[i] >= 0 && fc[i] < Lpc546xxFlexcommCount())
		{
			NVIC_DisableIRQ(Lpc546xxFlexcommIrqNo(fc[i]));
			SharedIntrfSetIrqHandler(fc[i], NULL, NULL);
			SYSCON->FCLKSEL[fc[i]] = SYSCON_FCLKSEL_SEL(LPC546XX_FCLKSEL_NONE);
			Lpc546xxFlexcommClock(fc[i], false);
		}
	}
	if (dev->Rx.Ch >= 0)
	{
		Lpc546xxDmaChFree(dev->Rx.Ch);
	}
	if (dev->Tx.Ch >= 0)
	{
		Lpc546xxDmaChFree(dev->Tx.Ch);
	}
	if (dev->bMclk)
	{
		SYSCON->MCLKIO = 0;
		SYSCON->MCLKCLKSEL = SYSCON_MCLKCLKSEL_SEL(LPC546XX_MCLKCLKSEL_NONE);
	}
	Lpc546xxAudioPllStop();
	dev->pDev = NULL;
	dev->TxFc = -1;
	dev->RxFc = -1;
	dev->Rx.Ch = -1;
	dev->Tx.Ch = -1;
}

static void Lpc546xxI2sPowerOff(DevIntrf_t * const pDev)
{
	if (Lpc546xxI2sOwned(pDev) == false)
	{
		return;
	}
	Lpc546xxI2sDisable(pDev);
	Lpc546xxI2sRelease();
}

static bool Lpc546xxI2sPinUsed(const IOPinCfg_t *pPins, int NbPins, int Idx)
{
	return Idx < NbPins && pPins[Idx].PortNo >= 0;
}

bool I2SInit(I2SDev_t * const pDev, const I2SCfg_t * const pCfg)
{
	Lpc546xxI2s_t *dev = &s_Lpc546xxI2s;

	if (pDev == NULL || pCfg == NULL || pCfg->pIOPinMap == NULL || pCfg->NbIOPins > LPC546XX_I2S_MAX_NB_IOPIN ||
		(pCfg->DevNo != LPC546XX_I2S_FC0 && pCfg->DevNo != LPC546XX_I2S_FC1) ||
		(pCfg->Mode != I2SMODE_MASTER && pCfg->Mode != I2SMODE_SLAVE) || pCfg->Format != I2SFMT_NORMAL ||
		pCfg->SampleWidth != I2SSWIDTH_16BITS ||
		(pCfg->Chan != I2SCHAN_STEREO && pCfg->Chan != I2SCHAN_LEFT && pCfg->Chan != I2SCHAN_RIGHT) ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}

	// A device still set up, the same one set up again
	if (dev->pDev != NULL)
	{
		if (dev->pDev != pDev)
		{
			return false;
		}
		Lpc546xxI2sPowerOff(&pDev->DevIntrf);
	}

	const IOPinCfg_t *pins = pCfg->pIOPinMap;
	bool rx = Lpc546xxI2sPinUsed(pins, pCfg->NbIOPins, I2S_SDI_IOPIN_IDX);
	bool tx = Lpc546xxI2sPinUsed(pins, pCfg->NbIOPins, I2S_SDO_IOPIN_IDX);
	unsigned frame = pCfg->Chan == I2SCHAN_STEREO ? 4U : 2U;

	if ((rx == false && tx == false) || pins[I2S_SCK_IOPIN_IDX].PortNo < 0 || pins[I2S_WS_IOPIN_IDX].PortNo < 0 ||
		(rx && tx && (Lpc546xxI2sPinUsed(pins, pCfg->NbIOPins, LPC546XX_I2S_RXSCK_IOPIN_IDX) == false ||
					  Lpc546xxI2sPinUsed(pins, pCfg->NbIOPins, LPC546XX_I2S_RXWS_IOPIN_IDX) == false)))
	{
		return false;
	}
	// FIFO memory: 4 byte aligned, a frame or more, for a direction used,
	// needed with interrupts or DMA, not overlapping. A DMA block of received
	// frames goes to the CFIFO at once: it must hold one.
	if ((rx && (pCfg->bIntEn || pCfg->bDmaEn) && pCfg->pRxFifoMem == NULL) ||
		(rx && pCfg->bDmaEn && pCfg->RxFifoMemSize < (int)(sizeof(CFifo_t) + LPC546XX_I2S_DMA_FRAMES * frame)) ||
		(tx && (pCfg->bIntEn || pCfg->bDmaEn) && pCfg->pTxFifoMem == NULL) ||
		(pCfg->pRxFifoMem != NULL && (rx == false || ((uintptr_t)pCfg->pRxFifoMem & 3U) ||
									  pCfg->RxFifoMemSize < (int)(sizeof(CFifo_t) + frame))) ||
		(pCfg->pTxFifoMem != NULL && (tx == false || ((uintptr_t)pCfg->pTxFifoMem & 3U) ||
									  pCfg->TxFifoMemSize < (int)(sizeof(CFifo_t) + frame))) ||
		(pCfg->pRxFifoMem != NULL && pCfg->pTxFifoMem != NULL &&
		 (uintptr_t)pCfg->pRxFifoMem < (uintptr_t)pCfg->pTxFifoMem + (uintptr_t)pCfg->TxFifoMemSize &&
		 (uintptr_t)pCfg->pTxFifoMem < (uintptr_t)pCfg->pRxFifoMem + (uintptr_t)pCfg->RxFifoMemSize))
	{
		return false;
	}

	int txfc = tx ? pCfg->DevNo : -1;
	int rxfc = rx ? (tx ? LPC546XX_I2S_FC0 + LPC546XX_I2S_FC1 - pCfg->DevNo : pCfg->DevNo) : -1;

	// The Flexcomms in their I2S function, clocked by the audio PLL
	if ((txfc >= 0 && Lpc546xxFlexcommSelect(txfc, LPC546XX_FLEXCOMM_I2S_TX) == 0U) ||
		(rxfc >= 0 && Lpc546xxFlexcommSelect(rxfc, LPC546XX_FLEXCOMM_I2S_RX) == 0U))
	{
		if (txfc >= 0)
		{
			Lpc546xxFlexcommClock(txfc, false);
		}
		if (rxfc >= 0)
		{
			Lpc546xxFlexcommClock(rxfc, false);
		}

		return false;
	}

	memset(dev, 0, sizeof(*dev));
	dev->pDev = pDev;
	dev->TxFc = txfc;
	dev->RxFc = rxfc;
	dev->bMaster = pCfg->Mode == I2SMODE_MASTER;
	dev->bMclk = Lpc546xxI2sPinUsed(pins, pCfg->NbIOPins, I2S_MCK_IOPIN_IDX);
	dev->FrameBytes = frame;
	dev->K = 1;
	dev->Rx.Ch = -1;
	dev->Tx.Ch = -1;

	pDev->Mode = pCfg->Mode;
	pDev->Format = pCfg->Format;
	pDev->SampleWidth = 2;
	pDev->Chan = pCfg->Chan;
	pDev->pIOPinMap = pins;
	pDev->NbIOPins = pCfg->NbIOPins;
	pDev->hRxFifo = pCfg->pRxFifoMem != NULL ? CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, frame, true) : NULL;
	pDev->hTxFifo = pCfg->pTxFifoMem != NULL ? CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, frame, true) : NULL;
	pDev->EvtCB = pCfg->EvtCB;

	DevIntrf_t *intrf = &pDev->DevIntrf;

	intrf->pDevData = pDev;
	intrf->Type = DEVINTRF_TYPE_I2S;
	intrf->bDma = pCfg->bDmaEn;
	intrf->bIntEn = pCfg->bIntEn;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->Enable = Lpc546xxI2sEnable;
	intrf->Disable = Lpc546xxI2sDisable;
	intrf->PowerOff = Lpc546xxI2sPowerOff;
	intrf->Reset = Lpc546xxI2sReset;
	intrf->GetRate = Lpc546xxI2sGetRate;
	intrf->SetRate = Lpc546xxI2sSetRate;
	intrf->StartRx = Lpc546xxI2sStartRx;
	intrf->StartTx = Lpc546xxI2sStartTx;
	intrf->StopRx = Lpc546xxI2sStop;
	intrf->StopTx = Lpc546xxI2sStop;
	intrf->RxData = Lpc546xxI2sRxData;
	intrf->TxData = Lpc546xxI2sTxData;
	intrf->TxSrData = NULL;
	intrf->GetHandle = Lpc546xxI2sGetHandle;
	atomic_flag_clear(&intrf->bBusy);
	atomic_store(&intrf->EnCnt, 1);
	atomic_store(&intrf->bTxReady, true);
	atomic_store(&intrf->bNoStop, false);

	// DMA on each direction, with the FIFO memory checked above
	if (pCfg->bDmaEn)
	{
		if (rxfc >= 0)
		{
			if (Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_RX(rxfc), Lpc546xxI2sDmaHandler, pDev, pCfg->IntPrio,
								   pCfg->bIntEn) == false)
			{
				Lpc546xxI2sRelease();

				return false;
			}
			dev->Rx.Ch = LPC546XX_DMA_CH_FC_RX(rxfc);
		}
		if (txfc >= 0)
		{
			if (Lpc546xxDmaChClaim(LPC546XX_DMA_CH_FC_TX(txfc), Lpc546xxI2sDmaHandler, pDev, pCfg->IntPrio,
								   pCfg->bIntEn) == false)
			{
				Lpc546xxI2sRelease();

				return false;
			}
			dev->Tx.Ch = LPC546XX_DMA_CH_FC_TX(txfc);
		}
	}

	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		if (pins[i].PortNo >= 0)
		{
			IOPinCfg(&pins[i], 1);
		}
	}

	int fc[2] = { txfc, rxfc };

	for (int i = 0; i < 2; i++)
	{
		if (fc[i] >= 0)
		{
			IRQn_Type irq = Lpc546xxFlexcommIrqNo(fc[i]);

			SYSCON->FCLKSEL[fc[i]] = SYSCON_FCLKSEL_SEL(LPC546XX_FCLKSEL_AUDPLL);
			SharedIntrfSetIrqHandler(fc[i], intrf, Lpc546xxI2sIrqHandler);
			NVIC_ClearPendingIRQ(irq);
			NVIC_SetPriority(irq, pCfg->IntPrio);
			if (pCfg->bIntEn)
			{
				NVIC_EnableIRQ(irq);
			}
		}
	}

	if (Lpc546xxI2sSetRate(intrf, pCfg->Freq) == 0U)
	{
		Lpc546xxI2sRelease();

		return false;
	}
	Lpc546xxI2sEnable(intrf);

	return true;
}
