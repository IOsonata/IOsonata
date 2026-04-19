/**-------------------------------------------------------------------------
@file	uart_sam4l.cpp

@brief	SAM4L USART implementation

All four SAM4L serial ports are USART instances (USART0..USART3).  There is
no separate UART peripheral as on the SAM4E.  DMA is driven by the single
PDCA (Peripheral DMA Controller A) selected via a per-channel peripheral id,
not by a dedicated PDC block per USART.

Channel allocation for optional TX DMA:
  USART0 TX -> PDCA channel 0, PID 18
  USART1 TX -> PDCA channel 1, PID 19
  USART2 TX -> PDCA channel 2, PID 20
  USART3 TX -> PDCA channel 3, PID 21

RX is always interrupt driven.

@author Hoang Nguyen Hoan
@date	July. 7, 2020

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
#include <assert.h>
#include <string.h>

#include "sam4lxxx.h"
#include "component/component_pdca.h"
#include "component/component_usart.h"
#include "component/component_pm.h"

#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "cfifo.h"

#define SAM4L_UART_CFIFO_SIZE		16
#define SAM4L_UART_CFIFO_MEMSIZE	CFIFO_MEMSIZE(SAM4L_UART_CFIFO_SIZE)

// USART WPMR key - "USA" in ASCII (0x55 0x53 0x41)
#define SAM4L_US_WPKEY			0x555341u

// PDCA peripheral ids for USART TX (datasheet Table 16-2)
#define SAM4L_PDCA_PID_USART_TX(n)	(18u + (n))
// PDCA channel assignment : TX channel N for USART N
#define SAM4L_PDCA_TX_CHAN(n)		(n)

#pragma pack(push, 4)

typedef struct _Sam_Uart_Dev {
	int DevNo;			//!< UART device index 0..3
	uint32_t PbaMask;		//!< PM_PBAMASK_USARTn bit
	IRQn_Type IrqNo;		//!< USARTn_IRQn
	Usart *pReg;			//!< USART register block
	UARTDev_t *pUartDev;		//!< Generic UART device data
	uint8_t RxFifoMem[SAM4L_UART_CFIFO_MEMSIZE];
	uint8_t TxFifoMem[SAM4L_UART_CFIFO_MEMSIZE];
	uint8_t PdcTxBuff[SAM4L_UART_CFIFO_SIZE];
} SAM4L_UARTDEV;

#pragma pack(pop)

static SAM4L_UARTDEV s_Sam4lUartDev[] = {
	{
		.DevNo    = 0,
		.PbaMask  = PM_PBAMASK_USART0,
		.IrqNo    = USART0_IRQn,
		.pReg     = SAM4L_USART0,
	},
	{
		.DevNo    = 1,
		.PbaMask  = PM_PBAMASK_USART1,
		.IrqNo    = USART1_IRQn,
		.pReg     = SAM4L_USART1,
	},
	{
		.DevNo    = 2,
		.PbaMask  = PM_PBAMASK_USART2,
		.IrqNo    = USART2_IRQn,
		.pReg     = SAM4L_USART2,
	},
	{
		.DevNo    = 3,
		.PbaMask  = PM_PBAMASK_USART3,
		.IrqNo    = USART3_IRQn,
		.pReg     = SAM4L_USART3,
	},
};

static const int s_NbSam4lUartDev = sizeof(s_Sam4lUartDev) / sizeof(SAM4L_UARTDEV);

UARTDev_t const * const UARTGetInstance(int DevNo)
{
	if (DevNo < 0 || DevNo >= s_NbSam4lUartDev)
	{
		return NULL;
	}

	return s_Sam4lUartDev[DevNo].pUartDev;
}

/**
 * @brief Unlock and write a PM register.
 *
 * The PM block requires an unlock key to be written to PM_UNLOCK immediately
 * before any write to a protected register.
 */
static inline void Sam4lPmWrite(volatile uint32_t *pReg, uint32_t Value)
{
	uint32_t offs = (uint32_t)pReg - (uint32_t)SAM4L_PM;
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu) | PM_UNLOCK_ADDR(offs);
	*pReg = Value;
}

static void Sam4lUartClockEnable(SAM4L_UARTDEV *pDev)
{
	Sam4lPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK | pDev->PbaMask);
}

static void Sam4lPdcaClockEnable(void)
{
	// PDCA is clocked on HSB (bus master side) and PBB (register side)
	Sam4lPmWrite(&SAM4L_PM->PM_HSBMASK, SAM4L_PM->PM_HSBMASK | PM_HSBMASK_PDCA);
	Sam4lPmWrite(&SAM4L_PM->PM_PBBMASK, SAM4L_PM->PM_PBBMASK | PM_PBBMASK_PDCA);
}

static inline void Sam4lUsartUnlockWpmr(Usart *pReg)
{
	pReg->US_WPMR = US_WPMR_WPKEY(SAM4L_US_WPKEY);
}

void Sam4lUsartIrqHandler(SAM4L_UARTDEV *pDev)
{
	Usart *reg = pDev->pReg;
	uint32_t csr = reg->US_CSR;
	uint32_t imr = reg->US_IMR;
	bool err = false;
	int cnt = 10;

	if ((csr & US_CSR_RXRDY) && (imr & US_IER_RXRDY))
	{
		do
		{
			uint8_t *p = CFifoPut(pDev->pUartDev->hRxFifo);
			if (p == NULL)
			{
				pDev->pUartDev->RxDropCnt++;
				// discard to clear RXRDY
				(void)reg->US_RHR;
				break;
			}

			*p = reg->US_RHR & 0xFFu;
			csr = reg->US_CSR;

		} while ((csr & US_CSR_RXRDY) && (cnt-- > 0));

		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, NULL, 0);
		}
	}

	if (pDev->pUartDev->DevIntrf.bDma == true)
	{
		// TX DMA uses the dedicated PDCA channel; "transfer complete" is
		// signalled by channel transfer-counter-reached-zero (TRC).
		PdcaChannel *chan = &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_PDCA_TX_CHAN(pDev->DevNo)];

		if (chan->PDCA_ISR & PDCA_ISR_TRC)
		{
			int l = SAM4L_UART_CFIFO_SIZE;
			uint8_t *p = CFifoGetMultiple(pDev->pUartDev->hTxFifo, &l);

			if (p)
			{
				memcpy(pDev->PdcTxBuff, p, l);
				chan->PDCA_MAR = (uint32_t)pDev->PdcTxBuff;
				chan->PDCA_TCR = l;
				chan->PDCA_CR  = PDCA_CR_TEN;
			}
			else
			{
				// Nothing to send; disable TX DMA, disable its interrupt,
				// drop back to TXEMPTY polling on the next push.
				chan->PDCA_CR = PDCA_CR_TDIS;
				chan->PDCA_IDR = PDCA_IER_TRC;	// disable transfer-complete interrupt
				pDev->pUartDev->bTxReady = true;
			}

			if (pDev->pUartDev->EvtCallback)
			{
				pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}
	else if ((csr & US_CSR_TXRDY) && (imr & US_IER_TXRDY))
	{
		cnt = 10;
		do
		{
			uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);
			if (p == NULL)
			{
				pDev->pUartDev->bTxReady = true;
				reg->US_IDR = US_IER_TXRDY;
				break;
			}
			reg->US_THR = US_THR_TXCHR(*p);
			csr = reg->US_CSR;
		} while ((csr & US_CSR_TXRDY) && (cnt-- > 0));

		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
		}
	}

	if (csr & US_CSR_OVRE)
	{
		pDev->pUartDev->RxOvrErrCnt++;
		err = true;
	}
	if (csr & US_CSR_FRAME)
	{
		pDev->pUartDev->FramErrCnt++;
		err = true;
	}
	if (csr & US_CSR_PARE)
	{
		pDev->pUartDev->ParErrCnt++;
		err = true;
	}
	if (err)
	{
		reg->US_CR = US_CR_RSTSTA;
	}
}

extern "C" void USART0_Handler(void)
{
	Sam4lUsartIrqHandler(&s_Sam4lUartDev[0]);
}

extern "C" void USART1_Handler(void)
{
	Sam4lUsartIrqHandler(&s_Sam4lUartDev[1]);
}

extern "C" void USART2_Handler(void)
{
	Sam4lUsartIrqHandler(&s_Sam4lUartDev[2]);
}

extern "C" void USART3_Handler(void)
{
	Sam4lUsartIrqHandler(&s_Sam4lUartDev[3]);
}

static inline uint32_t Sam4lUartGetRate(DevIntrf_t * const pDev)
{
	return ((SAM4L_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

/**
 * @brief  Configure the USART baud rate generator.
 *
 * Async mode formula (SAM4L datasheet 24.7.1):
 *   OVER = 0  :  baud = f_PBA / (16 * (CD + FP/8))
 *   OVER = 1  :  baud = f_PBA / ( 8 * (CD + FP/8))
 *
 * Picks OVER=0 when the divisor permits, otherwise OVER=1 for low ratios.
 */
static uint32_t Sam4lUartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	SAM4L_UARTDEV *dev = (SAM4L_UARTDEV *)pDev->pDevData;
	uint32_t pclk = SystemPeriphClockGet(0);	// PBA clock
	uint32_t over = 0;
	uint32_t cd, fp;

	if (Rate == 0)
	{
		return 0;
	}

	// Try OVER = 0 (16x) first.  cd_x8 = pclk * 8 / (16 * baud) = pclk / (2 * baud)
	uint32_t cd_x8 = (pclk + Rate) / (Rate << 1);	// rounded

	if (cd_x8 < 8)
	{
		// Too small for 16x; switch to 8x oversampling.
		over = US_MR_OVER;
		cd_x8 = (pclk + (Rate >> 1)) / Rate;		// cd_x8 = pclk * 8 / (8 * baud)
	}

	cd = cd_x8 >> 3;
	fp = cd_x8 & 0x7u;

	if (cd == 0)
	{
		return 0;	// clock too low for requested baud
	}
	if (cd > 0xFFFFu)
	{
		cd = 0xFFFFu;
		fp = 0;
	}

	Sam4lUsartUnlockWpmr(dev->pReg);
	if (over)
	{
		dev->pReg->US_MR |= US_MR_OVER;
	}
	else
	{
		dev->pReg->US_MR &= ~US_MR_OVER;
	}

	dev->pReg->US_BRGR = US_BRGR_CD(cd) | US_BRGR_FP(fp);

	// Actual rate achieved, rounded.
	uint32_t divisor = (over ? 8u : 16u) * ((cd << 3) + fp);
	uint32_t actual = (divisor == 0) ? 0 : ((pclk << 3) / divisor);

	dev->pUartDev->Rate = actual;
	return actual;
}

static inline bool Sam4lUartStartRx(DevIntrf_t * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int Sam4lUartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	SAM4L_UARTDEV *dev = (SAM4L_UARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p == NULL)
		{
			break;
		}
		memcpy(pBuff, p, l);
		cnt += l;
		pBuff += l;
		Bufflen -= l;
	}
	EnableInterrupt(state);

	// If the ISR drained nothing since last call and a byte is sitting in RHR,
	// pull it directly so tight polling loops can make progress.
	if (dev->pUartDev->bRxReady)
	{
		if (dev->pReg->US_CSR & US_CSR_RXRDY)
		{
			uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
			if (p)
			{
				*p = dev->pReg->US_RHR & 0xFFu;
				dev->pUartDev->bRxReady = false;
			}
		}
	}

	return cnt;
}

static inline void Sam4lUartStopRx(DevIntrf_t * const pDev)
{
}

static inline bool Sam4lUartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return true;
}

static int Sam4lUartTxData(DevIntrf_t * const pDev, const uint8_t *pData, int Datalen)
{
	SAM4L_UARTDEV *dev = (SAM4L_UARTDEV *)pDev->pDevData;
	int cnt = 0;
	int rtry = pDev->MaxRetry > 0 ? pDev->MaxRetry : 5;

	while (Datalen > 0 && rtry-- > 0)
	{
		uint32_t state = DisableInterrupt();

		while (Datalen > 0)
		{
			int l = Datalen;
			uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
			if (p == NULL)
			{
				break;
			}
			memcpy(p, pData, l);
			Datalen -= l;
			pData += l;
			cnt += l;
		}
		EnableInterrupt(state);

		if (dev->pUartDev->bTxReady)
		{
			if (dev->pUartDev->DevIntrf.bDma == true)
			{
				// Kick off a new PDCA TX burst only when the previous burst
				// has completed (bTxReady set by the ISR).
				PdcaChannel *chan = &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_PDCA_TX_CHAN(dev->DevNo)];

				int l = SAM4L_UART_CFIFO_SIZE;
				uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
				if (p)
				{
					dev->pUartDev->bTxReady = false;
					memcpy(dev->PdcTxBuff, p, l);

					chan->PDCA_MAR = (uint32_t)dev->PdcTxBuff;
					chan->PDCA_TCR = l;
					chan->PDCA_CR  = PDCA_CR_TEN;
					chan->PDCA_IER = PDCA_IER_TRC;	// fire IRQ on transfer complete
				}
			}
			else
			{
				if (dev->pReg->US_CSR & US_CSR_TXRDY)
				{
					uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
					if (p)
					{
						dev->pUartDev->bTxReady = false;
						dev->pReg->US_THR = US_THR_TXCHR(*p);
						dev->pReg->US_IER = US_IER_TXRDY;
					}
				}
			}
		}
	}
	return cnt;
}

void Sam4lUartStopTx(DevIntrf_t * const pDev)
{
}

void Sam4lUartDisable(DevIntrf_t * const pDev)
{
	SAM4L_UARTDEV *dev = (SAM4L_UARTDEV *)pDev->pDevData;

	dev->pReg->US_IDR = 0xFFFFFFFFu;
	dev->pReg->US_CR  = US_CR_RXDIS | US_CR_TXDIS;

	if (dev->pUartDev->DevIntrf.bDma)
	{
		PdcaChannel *chan = &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_PDCA_TX_CHAN(dev->DevNo)];
		chan->PDCA_CR = PDCA_CR_TDIS;
		chan->PDCA_IDR = 0xFFFFFFFFu;
	}

	// Gate the peripheral clock off.
	Sam4lPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK & ~dev->PbaMask);
}

void Sam4lUartEnable(DevIntrf_t * const pDev)
{
	SAM4L_UARTDEV *dev = (SAM4L_UARTDEV *)pDev->pDevData;

	Sam4lUartClockEnable(dev);

	CFifoFlush(dev->pUartDev->hTxFifo);
	CFifoFlush(dev->pUartDev->hRxFifo);

	dev->pReg->US_CR  = US_CR_RSTSTA | US_CR_RSTRX | US_CR_RSTTX;
	dev->pReg->US_CR  = US_CR_RXEN | US_CR_TXEN;
	dev->pReg->US_IER = US_IER_RXRDY;

	dev->pUartDev->bRxReady = false;
	dev->pUartDev->bTxReady = true;
}

void Sam4lUartPowerOff(DevIntrf_t * const pDev)
{
	Sam4lUartDisable(pDev);
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
	// SAM4L USART has no modem control lines mapped to generic line state.
	pDev->LineState = LineState;
}

static void Sam4lPdcaTxChannelInit(int DevNo)
{
	PdcaChannel *chan = &SAM4L_PDCA->PDCA_CHANNEL[SAM4L_PDCA_TX_CHAN(DevNo)];

	chan->PDCA_CR  = PDCA_CR_TDIS | PDCA_CR_ECLR;
	chan->PDCA_PSR = PDCA_PSR_PID(SAM4L_PDCA_PID_USART_TX(DevNo));
	chan->PDCA_MR  = PDCA_MR_SIZE_BYTE;
	chan->PDCA_MAR = 0;
	chan->PDCA_TCR = 0;
	chan->PDCA_IDR = 0xFFFFFFFFu;
}

bool UARTInit(UARTDev_t * const pDev, const UARTCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbSam4lUartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;
	SAM4L_UARTDEV *dev = &s_Sam4lUartDev[devno];

	Sam4lUartClockEnable(dev);

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, SAM4L_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(dev->TxFifoMem, SAM4L_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;

	// Pin muxing.  Caller supplies RX, TX, and optionally RTS/CTS.
	if (pCfg->NbIOPins > 2 && pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, pCfg->NbIOPins);
	}
	else
	{
		IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, 2);
	}

	Usart *reg = dev->pReg;

	// Disable and reset the USART before reconfiguring.
	Sam4lUsartUnlockWpmr(reg);
	reg->US_IDR = 0xFFFFFFFFu;
	reg->US_CR  = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
	reg->US_RTOR = 0;
	reg->US_TTGR = 0;

	// Build mode register.
	uint32_t mr = US_MR_CHMODE_NORMAL;

	switch (pCfg->Parity)
	{
		case UART_PARITY_ODD:
			mr |= US_MR_PAR_ODD;
			break;
		case UART_PARITY_EVEN:
			mr |= US_MR_PAR_EVEN;
			break;
		case UART_PARITY_MARK:
			mr |= US_MR_PAR_MARK;
			break;
		case UART_PARITY_SPACE:
			mr |= US_MR_PAR_SPACE;
			break;
		case UART_PARITY_NONE:
		default:
			mr |= US_MR_PAR_NONE;
			break;
	}

	if (pCfg->StopBits == 2)
	{
		mr |= US_MR_NBSTOP_2;
	}
	else
	{
		mr |= US_MR_NBSTOP_1;
	}

	switch (pCfg->DataBits)
	{
		case 5:  mr |= US_MR_CHRL_5; break;
		case 6:  mr |= US_MR_CHRL_6; break;
		case 7:  mr |= US_MR_CHRL_7; break;
		default: mr |= US_MR_CHRL_8; break;
	}

	Sam4lUsartUnlockWpmr(reg);
	reg->US_MR = mr;

	// Baud rate generator (also unlocks WPMR internally).
	pDev->Rate = Sam4lUartSetRate(&pDev->DevIntrf, pCfg->Rate);

	// DMA (PDCA) setup for TX.  RX is always interrupt driven.
	if (pCfg->bDMAMode)
	{
		Sam4lPdcaClockEnable();
		Sam4lPdcaTxChannelInit(devno);
		pDev->DevIntrf.bDma = true;
	}
	else
	{
		pDev->DevIntrf.bDma = false;
	}

	reg->US_CR  = US_CR_RXEN | US_CR_TXEN;
	reg->US_IER = US_IER_RXRDY;
	reg->US_CR  = US_CR_RSTSTA;

	pDev->bRxReady = false;
	pDev->bTxReady = true;

	pDev->DevIntrf.Type     = DEVINTRF_TYPE_UART;
	pDev->DataBits          = pCfg->DataBits;
	pDev->FlowControl       = pCfg->FlowControl;
	pDev->StopBits          = pCfg->StopBits;
	pDev->bIrDAFixPulse     = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert       = pCfg->bIrDAInvert;
	pDev->bIrDAMode         = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv      = pCfg->IrDAPulseDiv;
	pDev->Parity            = pCfg->Parity;
	pDev->EvtCallback       = pCfg->EvtCallback;
	pDev->DevIntrf.Disable  = Sam4lUartDisable;
	pDev->DevIntrf.Enable   = Sam4lUartEnable;
	pDev->DevIntrf.GetRate  = Sam4lUartGetRate;
	pDev->DevIntrf.SetRate  = Sam4lUartSetRate;
	pDev->DevIntrf.StartRx  = Sam4lUartStartRx;
	pDev->DevIntrf.RxData   = Sam4lUartRxData;
	pDev->DevIntrf.StopRx   = Sam4lUartStopRx;
	pDev->DevIntrf.StartTx  = Sam4lUartStartTx;
	pDev->DevIntrf.TxData   = Sam4lUartTxData;
	pDev->DevIntrf.StopTx   = Sam4lUartStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = Sam4lUartPowerOff;
	pDev->DevIntrf.EnCnt    = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	NVIC_ClearPendingIRQ(dev->IrqNo);
	NVIC_SetPriority(dev->IrqNo, pCfg->IntPrio);
	NVIC_EnableIRQ(dev->IrqNo);

	return true;
}
