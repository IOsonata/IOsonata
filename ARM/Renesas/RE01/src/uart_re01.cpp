/**-------------------------------------------------------------------------
@file	uart_re01.cpp

@brief	Renesas RE01 UART implementation


@author Hoang Nguyen Hoan
@date	Jan. 29, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#include <stdlib.h>

#include "re01xxx.h"

#include "interrupt.h"
#include "interrupt_re01.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "cfifo.h"

#define RE01_UART_CFIFO_SIZE		16
#define RE01_UART_CFIFO_MEMSIZE		CFIFO_MEMSIZE(RE01_UART_CFIFO_SIZE)

#pragma pack(push, 4)

typedef struct __Re01_Uart_Dev {
	int DevNo;				// UART interface number
	union {
		SCI0_Type *pUartReg0;
		SCI2_Type *pUartReg2;
	};
	UARTDEV	*pUartDev;		// Pointer to generic UART dev. data
	uint8_t RxFifoMem[RE01_UART_CFIFO_MEMSIZE];
	uint8_t TxFifoMem[RE01_UART_CFIFO_MEMSIZE];
	IRQn_Type IrqRxi;
	IRQn_Type IrqTxi;
	IRQn_Type IrqTei;
	IRQn_Type IrqEri;
	IRQn_Type IrqAm;
} Re01UartDev_t;

#pragma pack(pop)

static Re01UartDev_t s_Re01UartDev[] = {
	{
		.DevNo = 0,
		.pUartReg0 = SCI0,
	},
	{
		.DevNo = 1,
		.pUartReg0 = SCI1,
	},
	{
		.DevNo = 2,
		.pUartReg2 = SCI2,
	},
	{
		.DevNo = 3,
		.pUartReg2 = SCI3,
	},
	{
		.DevNo = 4,
		.pUartReg2 = SCI4,
	},
	{
		.DevNo = 5,
		.pUartReg2 = SCI5,
	},
	{
		.DevNo = 6,
		.pUartReg2 = SCI9,
	},
};

static const int s_NbRe01UartDev = sizeof(s_Re01UartDev) / sizeof(Re01UartDev_t);

UARTDEV * const UARTGetInstance(int DevNo)
{
	if (DevNo < 0 || DevNo >= s_NbRe01UartDev)
	{
		return NULL;
	}

	return s_Re01UartDev[DevNo].pUartDev;
}

void Re01UartIRQHandlerFifo(int IntNo, void *pCtx)
{
	Re01UartDev_t *dev = (Re01UartDev_t*)pCtx;
	uint8_t status = dev->pUartReg0->SSR_FIFO;
	bool err = false;
	int cnt = 10;

	if (status & SCI0_SSR_FIFO_DR_Msk)
	{
		do
		{
			uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
			if (p == NULL)
			{
				break;
			}

			*p = dev->pUartReg0->FRDRL;
			status = dev->pUartReg0->SSR_FIFO;

		} while ((status & SCI0_SSR_FIFO_DR_Msk) && (cnt-- > 0));

		dev->pUartReg0->SSR_FIFO_b.DR = 0;

		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, 0);
		}
	}
	cnt = 16;

	if (dev->pUartDev->DevIntrf.bDma == true)
	{
/*		if (status & UART_IER_ENDTX)
		{
			int l = SAM4_UART_CFIFO_SIZE;
			uint8_t *p = CFifoGetMultiple(pDev->pUartDev->hTxFifo, &l);
			if (p)
			{
				memcpy(pDev->PdcTxBuff, p, l);
				pDev->pPdc->PERIPH_TPR = (uint32_t)pDev->PdcTxBuff;
				pDev->pPdc->PERIPH_TCR = l;
				pDev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
			}
			else
			{
				pDev->pPdc->PERIPH_TCR = 0;
				pDev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
				pDev->pUartReg->UART_IDR = UART_IER_ENDTX;
			}

			if (pDev->pUartDev->EvtCallback)
			{
				pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
			}
		}*/
	}
	else if (status & SCI0_SSR_FIFO_TEND_Msk)
	{
		do
		{
			dev->pUartReg0->SSR_FIFO_b.TEND = 0;
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
			if (p == NULL)
			{
				dev->pUartDev->bTxReady = true;
				dev->pUartReg0->SSR_FIFO_b.TEND = 0;
				break;
			}
			dev->pUartReg0->FTDRL = *p;
			status = dev->pUartReg0->SSR_FIFO;
		} while ((status & SCI0_SSR_FIFO_TDFE_Msk) && (cnt-- > 0));

		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, 0);
		}
	}

	if (status & SCI2_SSR_ORER_Msk)
	{
		// Overrun
		dev->pUartDev->RxOvrErrCnt++;
	}

	if (status & SCI2_SSR_FER_Msk)
	{
		dev->pUartDev->FramErrCnt++;
	}

	if (status & SCI2_SSR_PER_Msk)
	{
		dev->pUartDev->ParErrCnt++;
	}
}

void Re01UartIRQHandler(int IntNo, void *pCtx)
{
	Re01UartDev_t *dev = (Re01UartDev_t*)pCtx;
	uint8_t status = dev->pUartReg2->SSR;
	int cnt = 10;

	if (status & SCI2_SSR_RDRF_Msk)
	{
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p != NULL)
		{
			*p = dev->pUartReg2->RDR;
		}
		else
		{
			dev->pUartDev->RxDropCnt++;
		}
		dev->pUartReg2->SSR_b.RDRF = 0;

		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, 0);
		}

	}

	if (status & SCI2_SSR_TDRE_Msk)
	{
		uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
		if (p == NULL)
		{
			dev->pUartDev->bTxReady = true;
			//dev->pUartReg2->SSR_b.TDRE = 0;
			if (dev->pUartDev->EvtCallback)
			{
				dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, 0);
			}
		}
		else
		{
			dev->pUartReg2->TDR = *p;
		}
	}

	if (status & SCI2_SSR_ORER_Msk)
	{
		// Overrun
		dev->pUartDev->RxOvrErrCnt++;
	}

	if (status & SCI2_SSR_FER_Msk)
	{
		dev->pUartDev->FramErrCnt++;
	}

	if (status & SCI2_SSR_PER_Msk)
	{
		dev->pUartDev->ParErrCnt++;
	}
}


static inline uint32_t Re01UARTGetRate(DEVINTRF * const pDev) {
	return ((Re01UartDev_t*)pDev->pDevData)->pUartDev->Rate;
}

// Data sheet formula
// N = PCLK / (div x (2^(2n-1)) x (256/M) × Rate) - 1
// where
// div =
// 	64 if BGDM, ABCS, ABCSE = 000
//	32 if BGDM, ABCS, ABCSE = 100
//	16 if BGDM, ABCS, ABCSE = 110
//	12 if BGDM, ABCS, ABCSE = xx1
// 128 <= M <= 255 is MDDR
// 0 <= N <= 255 is BRR
// n : CKS => PCLK divider
//
// Simplification
// 	1/2^(2n-1) => 2/(2^(2n)) => 2/(1<<(n<<1))
// N = 2 * PCLK /  (div x (1<<(n<<1)) x (256/M) × Rate) - 1
// Hence
// N = PCLK /  (div x (1<<(n<<1)) x (256/M) × Rate) - 1
// where div =
// 	32 if BGDM, ABCS, ABCSE = 000
//	16 if BGDM, ABCS, ABCSE = 100
//	8 if BGDM, ABCS, ABCSE = 110
//	6 if BGDM, ABCS, ABCSE = xx1

static const struct __SemrDiv {
	uint32_t div;
	uint32_t regval;
} s_Re01SemrDivTbl[] = {
	{32, 0},
	{16, SCI0_SEMR_BGDM_Msk},
	{8, SCI0_SEMR_BGDM_Msk | SCI0_SEMR_ABCS_Msk},
	{6, SCI0_SEMR_ABCSE_Msk}
};

static uint32_t Re01UARTSetRate(DEVINTRF * const pDev, uint32_t Rate)
{
	Re01UartDev_t *dev = (Re01UartDev_t *)pDev->pDevData;
	uint32_t pclk = SystemPeriphClockGet(0);
	uint32_t mddr = 0;
	uint32_t brr = 0;
	uint32_t cks = 0;
	uint32_t semr = 0;
	uint32_t diff = -1;
	uint32_t baud = 0;

	// Try to find most suitable registers values
	for (int i = Rate > 250000 ? 1 : 0; i < sizeof(s_Re01SemrDivTbl) / sizeof(struct __SemrDiv) - 1; i++)
	{
		for (int n = 0; n < 4; n++)	// CKS
		{
			for (uint32_t m = 128; m < 256; m++) // MDDR
			{
				uint32_t t = s_Re01SemrDivTbl[i].div * (1<<(n<<1)) * (256 / m);
				uint32_t div = pclk / (t * Rate);
				uint32_t r = pclk / (t * div);
				uint32_t d = abs((int)r - (int)Rate);

				if (d < diff && div > 0 && div < 256)
				{
					brr = div - 1;
					mddr = m;
					cks = n;
					semr = s_Re01SemrDivTbl[i].regval;
					diff = d;
					baud = r;
				}
			}
		}
	}
	for (uint32_t m = 128; m < 256; m++) // MDDR
	{
		uint32_t t = s_Re01SemrDivTbl[3].div * (1<<(0<<1)) * (256 / m);
		uint32_t div = 1;//pclk / (t * Rate);
		uint32_t r = pclk / (t * div);
		uint32_t d = abs((int)r - (int)Rate);

		if (d < diff && div > 0 && div < 256)
		{
			brr = div - 1;
			mddr = m;
			cks = 0;
			semr = s_Re01SemrDivTbl[3].regval;
			diff = d;
			baud = r;
		}
	}

	if (mddr > 0)
	{
		dev->pUartReg0->SEMR &= ~(SCI0_SEMR_BGDM_Msk | SCI0_SEMR_ABCS_Msk | SCI0_SEMR_ABCSE_Msk);
		dev->pUartReg0->SEMR |= semr | SCI0_SEMR_BRME_Msk;
		dev->pUartReg0->BRR = brr;
		dev->pUartReg0->MDDR = mddr;
		dev->pUartReg0->SMR_b.CKS = cks;
	}

	return baud;	// return actual baudrate
}

static inline bool Re01UARTStartRx(DEVINTRF * const pSerDev, uint32_t DevAddr) {
	return true;
}

static int Re01UARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	Re01UartDev_t *dev = (Re01UartDev_t *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l  = Bufflen;
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

	if (dev->pUartDev->bRxReady)
	{
		bool rdy = false;
		if (dev->DevNo < 2)
		{
			rdy = dev->pUartReg0->SSR & SCI0_SSR_FIFO_RDF_Msk;
		}
		else
		{
			rdy = dev->pUartReg2->SSR & SCI2_SSR_RDRF_Msk;
		}
		
		if (rdy == true)
		{
			uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
			if (p)
			{
				if (dev->DevNo < 2)
				{
					*p = dev->pUartReg0->FRDRHL & 0xFF;
				}
				else
				{
					*p = dev->pUartReg2->RDR;
				}
				dev->pUartDev->bRxReady = false;
			}
		}
	}

	return cnt;
}

static inline void Re01UARTStopRx(DEVINTRF * const pDev) {
}

static inline bool Re01UARTStartTx(DEVINTRF * const pDev, uint32_t DevAddr) {
	return true;
}

static int Re01UARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	Re01UartDev_t *dev = (Re01UartDev_t *)pDev->pDevData;
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
			bool rdy = false;
			
			if (dev->DevNo < 2)
			{
				rdy = dev->pUartReg0->SSR_FIFO & SCI0_SSR_FIFO_TDFE_Msk;
			}
			else
			{
				rdy = dev->pUartReg2->SSR & SCI2_SSR_TDRE_Msk;
			}
			if (rdy == true)
			{
				if (dev->pUartDev->DevIntrf.bDma == true)
				{
					int l = RE01_UART_CFIFO_SIZE;
					uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
					if (p)
					{
						//memcpy(dev->PdcTxBuff, p, l);
						//dev->pPdc->PERIPH_TPR = (uint32_t)dev->PdcTxBuff;
						//dev->pPdc->PERIPH_TCR = l;
						//dev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
						if (dev->DevNo < 2)
						{
						//	dev->pUartReg->UART_IER = UART_IER_ENDTX;
						}
						else
						{
						//	dev->pUSartReg->US_IER = US_IER_ENDTX;
						}
					}
				}
				else
				{
					uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
					if (p)
					{
						dev->pUartDev->bTxReady = false;

						if (dev->DevNo < 2)
						{
							dev->pUartReg0->FTDRHL_b.TDAT = *p;
						}
						else
						{
							dev->pUartReg2->TDR = *p;
						}
					}
				}
			}
		}
	}
	return cnt;
}

void Re01UARTStopTx(DEVINTRF * const pDev)
{
}

void Re01UARTDisable(DEVINTRF * const pDev)
{
	Re01UartDev_t *dev = (Re01UartDev_t *)pDev->pDevData;

	if (dev->DevNo < 2)
	{
		dev->pUartReg0->SCR &= ~(SCI0_SCR_RE_Msk | SCI0_SCR_TE_Msk);
	}
	else
	{
		dev->pUartReg2->SCR &= ~(SCI2_SCR_RE_Msk | SCI2_SCR_TE_Msk);
	}
}

void Re01UARTEnable(DEVINTRF * const pDev)
{
	Re01UartDev_t *dev = (Re01UartDev_t *)pDev->pDevData;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	if (dev->DevNo < 2)
	{
		dev->pUartReg0->SCR |= (SCI0_SCR_RE_Msk | SCI0_SCR_TE_Msk);
	}
	else
	{
		dev->pUartReg2->SCR |= (SCI2_SCR_RE_Msk | SCI2_SCR_TE_Msk);
	}
}

void Re01UARTPowerOff(DEVINTRF * const pDev)
{
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{

}

bool UARTInit(UARTDEV * const pDev, const UARTCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 2)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbRe01UartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;
	
	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_Re01UartDev[devno].RxFifoMem, RE01_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_Re01UartDev[devno].TxFifoMem, RE01_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	pDev->DevIntrf.pDevData = &s_Re01UartDev[devno];
	s_Re01UartDev[devno].pUartDev = pDev;

	MSTP->MSTPCRB &= ~(0x80000000 >> devno);

	IOPINCFG *iopins = (IOPINCFG*)pCfg->pIOPinMap;
	
	uint32_t ctse = 0;

	if (pCfg->NbIOPins > 2 && pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, pCfg->NbIOPins);
		ctse = 1;
	}
	else
	{
		IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, 2);
	}

	uint32_t smr = 0;
	uint32_t chr1 = 0;

	switch (pCfg->Parity)
	{
		case UART_PARITY_ODD:
			smr |= SCI0_SMR_PE_Msk | SCI0_SMR_PM_Msk;
			break;
		case UART_PARITY_EVEN:
			smr |= SCI0_SMR_PE_Msk;
			break;
		default:
			break;
	}

	if (pCfg->StopBits == 2)
	{
		smr |= SCI0_SMR_STOP_Msk;
	}

	switch (pCfg->DataBits)
	{
		case 7:
			chr1 = 1;
		case 9:
			smr |= SCI0_SMR_CHR_Msk;
			break;
		default:
			chr1 = 1;
			break;
	}

	if (devno < 2)
	{
		s_Re01UartDev[devno].pUartReg0->SMR = smr;
		s_Re01UartDev[devno].pUartReg0->SCMR_b.CHR1 = chr1;

		// Enable fifo. Avail only on SCI0 & SCI1
		s_Re01UartDev[devno].pUartReg0->FCR = SCI0_FCR_FM_Msk | SCI0_FCR_RFRST_Msk | SCI0_FCR_TFRST_Msk |
											  (15 << SCI0_FCR_TTRG_Pos) | (8 << SCI0_FCR_RTRG_Pos) |
											  (15 << SCI0_FCR_RSTRG_Pos);
		s_Re01UartDev[devno].pUartReg0->SPMR_b.CTSE = ctse;
	}
	else
	{
		s_Re01UartDev[devno].pUartReg2->SMR = smr;
		s_Re01UartDev[devno].pUartReg2->SCMR_b.CHR1 = chr1;
		s_Re01UartDev[devno].pUartReg2->SPMR_b.CTSE = ctse;
	}

	pDev->Rate = Re01UARTSetRate(&pDev->DevIntrf, pCfg->Rate);

#if 0
	if (pCfg->bDMAMode == true)
	{
		s_Re01UartDev[devno].pUartDev->DevIntrf.bDma = true;
		s_Re01UartDev[devno].pUartReg->UART_PTCR = UART_PTCR_TXTEN;
		s_Re01UartDev[devno].pUartReg->UART_IER = UART_IER_RXRDY;// | UART_IER_ENDTX;//UART_IER_TXBUFE;
		s_Re01UartDev[devno].pPdc->PERIPH_TPR = (uint32_t)s_Re01UartDev[devno].PdcTxBuff;
		s_Re01UartDev[devno].pPdc->PERIPH_TCR = 0;
		s_Re01UartDev[devno].pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
	}
	else
	{
		s_Re01UartDev[devno].pUartDev->DevIntrf.bDma = false;
		s_Re01UartDev[devno].pUartReg->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
		s_Re01UartDev[devno].pUartReg->UART_IER = UART_IER_RXRDY;// | UART_IER_TXEMPTY;//UART_IER_TXRDY;
	}
#endif

	s_Re01UartDev[devno].pUartDev->bRxReady = false;
	s_Re01UartDev[devno].pUartDev->bTxReady = true;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->bIntMode = pCfg->bIntMode;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->DevIntrf.Disable = Re01UARTDisable;
	pDev->DevIntrf.Enable = Re01UARTEnable;
	pDev->DevIntrf.GetRate = Re01UARTGetRate;
	pDev->DevIntrf.SetRate = Re01UARTSetRate;
	pDev->DevIntrf.StartRx = Re01UARTStartRx;
	pDev->DevIntrf.RxData = Re01UARTRxData;
	pDev->DevIntrf.StopRx = Re01UARTStopRx;
	pDev->DevIntrf.StartTx = Re01UARTStartTx;
	pDev->DevIntrf.TxData = Re01UARTTxData;
	pDev->DevIntrf.StopTx = Re01UARTStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = Re01UARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	// Enable UART must be done before enabling interrupt as per specs
	Re01UARTEnable(&s_Re01UartDev[devno].pUartDev->DevIntrf);

	if (devno < 2)
	{
		s_Re01UartDev[devno].pUartReg0->SCR_b.RIE = 1;
		s_Re01UartDev[devno].pUartReg0->SCR_b.TEIE = 1;
		s_Re01UartDev[devno].pUartReg0->SCR_b.TIE = 1;
	}
	else
	{
		s_Re01UartDev[devno].pUartReg2->SCR_b.RIE = 1;
		s_Re01UartDev[devno].pUartReg2->SCR_b.TEIE = 1;
		s_Re01UartDev[devno].pUartReg2->SCR_b.TIE = 1;
	}

	switch (devno)
	{
		case 0:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI0_SCI0_RXI,
																 pCfg->IntPrio, Re01UartIRQHandlerFifo,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI0_SCI0_TXI,
																 pCfg->IntPrio, Re01UartIRQHandlerFifo,
																 &s_Re01UartDev[devno]);
			break;
		case 1:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI1_SCI1_RXI,
																 pCfg->IntPrio, Re01UartIRQHandlerFifo,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI1_SCI1_TXI,
																 pCfg->IntPrio, Re01UartIRQHandlerFifo,
																 &s_Re01UartDev[devno]);
			break;
		case 2:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI2_SCI2_RXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI2_SCI2_TXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			break;
		case 3:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI3_SCI3_RXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI3_SCI3_TXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			break;
		case 4:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI4_SCI4_RXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI4_SCI4_TXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			break;
		case 5:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI5_SCI5_RXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI5_SCI5_TXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			break;
		case 6:
			s_Re01UartDev[devno].IrqRxi = Re01RegisterIntHandler(RE01_EVTID_SCI9_SCI9_RXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			s_Re01UartDev[devno].IrqTxi = Re01RegisterIntHandler(RE01_EVTID_SCI9_SCI9_TXI,
																 pCfg->IntPrio, Re01UartIRQHandler,
																 &s_Re01UartDev[devno]);
			break;
	}

	return true;
}
