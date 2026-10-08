/**-------------------------------------------------------------------------
@file	uart_re01.cpp

@brief	Renesas RE01 UART implementation

NOTE:	This chip does not have a precision baudrate clock divider some rate work
		with some PCLK freq some don't.  Make sure the actual calculated baudreate
		is within 0.2% tolerance.  Try a different baudrate otherwise


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
#include <string.h>

#include "re01xxx.h"

#include "coredev/interrupt.h"
#include "interrupt_re01.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "cfifo.h"

#define RE01_UART_CFIFO_SIZE		16
#define RE01_UART_CFIFO_MEMSIZE		CFIFO_MEMSIZE(RE01_UART_CFIFO_SIZE)

#pragma pack(push, 4)
typedef struct {
	int DevNo;				// SCI0-5, SCI9 (logical device 6)
	union {
		SCI0_Type *pUartReg0;
		SCI2_Type *pUartReg2;
	};
	UARTDev_t *pUartDev;
	uint8_t RxFifoMem[RE01_UART_CFIFO_MEMSIZE];
	uint8_t TxFifoMem[RE01_UART_CFIFO_MEMSIZE];
	IRQn_Type IrqRxi;
	IRQn_Type IrqTxi;
	IRQn_Type IrqEri;
} Re01UartDev_t;
#pragma pack(pop)

static Re01UartDev_t s_Re01UartDev[] = {
	{.DevNo = 0, .pUartReg0 = SCI0, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
	{.DevNo = 1, .pUartReg0 = SCI1, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
	{.DevNo = 2, .pUartReg2 = SCI2, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
	{.DevNo = 3, .pUartReg2 = SCI3, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
	{.DevNo = 4, .pUartReg2 = SCI4, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
	{.DevNo = 5, .pUartReg2 = SCI5, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
	{.DevNo = 6, .pUartReg2 = SCI9, .IrqRxi = (IRQn_Type)-1, .IrqTxi = (IRQn_Type)-1, .IrqEri = (IRQn_Type)-1},
};
static const int s_NbRe01UartDev = sizeof(s_Re01UartDev) / sizeof(s_Re01UartDev[0]);

static const uint32_t s_Re01UartStopMask[] = {
	MSTP_MSTPCRB_MSTPB31_Msk, MSTP_MSTPCRB_MSTPB30_Msk,
	MSTP_MSTPCRB_MSTPB29_Msk, MSTP_MSTPCRB_MSTPB28_Msk,
	MSTP_MSTPCRB_MSTPB27_Msk, MSTP_MSTPCRB_MSTPB26_Msk,
	MSTP_MSTPCRB_MSTPB22_Msk,
};

static const uint8_t s_Re01UartRxEvent[] = {
	RE01_EVTID_SCI0_SCI0_RXI, RE01_EVTID_SCI1_SCI1_RXI,
	RE01_EVTID_SCI2_SCI2_RXI, RE01_EVTID_SCI3_SCI3_RXI,
	RE01_EVTID_SCI4_SCI4_RXI, RE01_EVTID_SCI5_SCI5_RXI,
	RE01_EVTID_SCI9_SCI9_RXI,
};

UARTDEV const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < s_NbRe01UartDev ? s_Re01UartDev[DevNo].pUartDev : NULL;
}

static void Re01UartReleaseIRQs(Re01UartDev_t *dev)
{
	Re01UnregisterIntHandler(dev->IrqRxi);
	Re01UnregisterIntHandler(dev->IrqTxi);
	Re01UnregisterIntHandler(dev->IrqEri);
	dev->IrqRxi = dev->IrqTxi = dev->IrqEri = (IRQn_Type)-1;
}

static void Re01UartClearErrors(Re01UartDev_t *dev, uint8_t Status)
{
	uint8_t errors = Status & (SCI2_SSR_ORER_Msk | SCI2_SSR_FER_Msk | SCI2_SSR_PER_Msk);
	if (errors & SCI2_SSR_ORER_Msk)
		dev->pUartDev->RxOvrErrCnt++;
	if (errors & SCI2_SSR_FER_Msk)
		dev->pUartDev->FramErrCnt++;
	if (errors & SCI2_SSR_PER_Msk)
		dev->pUartDev->ParErrCnt++;
	if (errors)
	{
		// SSR and SSR_FIFO share the error-bit positions and byte offset.
		dev->pUartReg2->SSR &= ~errors;
	}
}

// Called with interrupts masked or from the ISR. At most one hardware FIFO.
static int Re01UartFillTx(Re01UartDev_t *dev)
{
	int sent = 0;
	if (dev->DevNo < 2)
	{
		int space = RE01_UART_CFIFO_SIZE - dev->pUartReg0->FDR_b.T;
		for (int i = 0; i < space && i < RE01_UART_CFIFO_SIZE; i++)
		{
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
			if (p == NULL)
				break;
			dev->pUartReg0->FTDRL = *p;
			sent++;
		}
		if (sent)
		{
			(void)dev->pUartReg0->SSR_FIFO;
			dev->pUartReg0->SSR_FIFO_b.TDFE = 0;
		}
	}
	else if (dev->pUartReg2->SSR & SCI2_SSR_TDRE_Msk)
	{
		uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
		if (p)
		{
			dev->pUartReg2->TDR = *p;
			dev->pUartReg2->SSR_b.TDRE = 0;
			sent = 1;
		}
	}
	bool pending = CFifoUsed(dev->pUartDev->hTxFifo) > 0;
	dev->pUartDev->bTxReady = !pending;
	if (pending)
		dev->pUartReg2->SCR |= SCI2_SCR_TIE_Msk;
	else
		dev->pUartReg2->SCR &= ~SCI2_SCR_TIE_Msk;
	return sent;
}

static void Re01UartIRQHandler(int IntNo, void *pCtx)
{
	Re01UartDev_t *dev = (Re01UartDev_t*)pCtx;
	uint8_t status = dev->pUartReg2->SSR;
	int received = 0;
	int sent = 0;
	Re01UartClearErrors(dev, status);
	if (dev->DevNo < 2)
	{
		int count = dev->pUartReg0->FDR_b.R;
		for (int i = 0; i < count && i < RE01_UART_CFIFO_SIZE; i++)
		{
			uint8_t data = dev->pUartReg0->FRDRL;
			uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
			if (p)
			{
				*p = data;
				received++;
			}
			else
				dev->pUartDev->RxDropCnt++;
		}
		if (count || (status & (SCI0_SSR_FIFO_RDF_Msk | SCI0_SSR_FIFO_DR_Msk)))
			dev->pUartReg0->SSR_FIFO &= ~(SCI0_SSR_FIFO_RDF_Msk | SCI0_SSR_FIFO_DR_Msk);
	}
	else if (status & SCI2_SSR_RDRF_Msk)
	{
		// Always read RDR, even if the software FIFO is full.
		uint8_t data = dev->pUartReg2->RDR;
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p)
		{
			*p = data;
			received = 1;
		}
		else
			dev->pUartDev->RxDropCnt++;
		dev->pUartReg2->SSR_b.RDRF = 0;
	}
	if ((dev->pUartReg2->SCR & SCI2_SCR_TIE_Msk) && (status & SCI2_SSR_TDRE_Msk))
		sent = Re01UartFillTx(dev);
	if (dev->pUartDev->EvtCallback)
	{
		if (received)
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->pUartDev->hRxFifo));
		if (sent)
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, CFifoAvail(dev->pUartDev->hTxFifo));
	}
}

static uint32_t Re01UARTGetRate(DevIntrf_t * const pDev)
{
	return ((Re01UartDev_t*)pDev->pDevData)->pUartDev->Rate;
}

// Baud = PCLK * M / (div * 4^CKS * 256 * (BRR + 1)).
// M = 256 means modulation disabled; otherwise MDDR is 128..255.
static const struct {
	uint32_t Div;
	uint8_t Semr;
} s_Re01SemrDivTbl[] = {
	{32, 0},
	{16, SCI0_SEMR_BGDM_Msk},
	{8, SCI0_SEMR_BGDM_Msk | SCI0_SEMR_ABCS_Msk},
	{6, SCI0_SEMR_ABCSE_Msk},
};

static uint32_t Re01UARTSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	Re01UartDev_t *dev = (Re01UartDev_t*)pDev->pDevData;
	uint32_t pclk = SystemPeriphClockGet(dev->DevNo < 2 ? 0 : 1);
	if (Rate == 0 || pclk == 0 || Rate > pclk / 6 ||
		(uint64_t)Rate * 32 * 64 * 256 * 256 < (uint64_t)pclk * 128)
		return 0;
	uint32_t diff = UINT32_MAX;
	uint32_t baud = 0;
	uint8_t bestBrr = 0, bestCks = 0, bestSemr = 0, bestM = 255;
	for (unsigned i = 0; i < sizeof(s_Re01SemrDivTbl) / sizeof(s_Re01SemrDivTbl[0]); i++)
	{
		for (unsigned n = 0; n < 4; n++)
		{
			uint64_t factor = (uint64_t)s_Re01SemrDivTbl[i].Div * (1UL << (n * 2)) * 256;
			for (unsigned m = 256; m >= 128; m--)
			{
				uint64_t numerator = (uint64_t)pclk * m;
				uint32_t count = numerator / (factor * Rate);
				// Both neighbours of the ideal BRR+1; include BRR=255.
				for (unsigned j = 0; j < 2; j++)
				{
					uint32_t divisor = count + j;
					if (divisor < 1 || divisor > 256)
						continue;
					uint64_t denominator = factor * divisor;
					uint32_t actual = (numerator + denominator / 2) / denominator;
					uint32_t error = actual > Rate ? actual - Rate : Rate - actual;
					bool modulation = m < 256;
					if (error < diff || (error == diff && !modulation && (bestSemr & SCI0_SEMR_BRME_Msk)))
					{
						diff = error;
						baud = actual;
						bestBrr = divisor - 1;
						bestCks = n;
						bestM = modulation ? m : 255;
						bestSemr = s_Re01SemrDivTbl[i].Semr | (modulation ? SCI0_SEMR_BRME_Msk : 0);
					}
				}
			}
		}
	}
	if (baud == 0)
		return 0;
	uint32_t state = DisableInterrupt();
	uint8_t scr = dev->pUartReg2->SCR;
	dev->pUartReg2->SCR = 0;
	dev->pUartReg2->SEMR = (dev->pUartReg2->SEMR & ~(SCI0_SEMR_BGDM_Msk | SCI0_SEMR_ABCS_Msk |
			SCI0_SEMR_ABCSE_Msk | SCI0_SEMR_BRME_Msk)) | bestSemr;
	dev->pUartReg2->SMR_b.CKS = bestCks;
	dev->pUartReg2->BRR = bestBrr;
	dev->pUartReg2->MDDR = bestM;
	dev->pUartDev->Rate = baud;
	// Let the baud generator settle before restoring TE/RE.
	for (uint32_t wait = SystemCoreClock / baud + 1; wait > 0; wait--)
	{
		__NOP();
	}
	dev->pUartReg2->SCR = scr;
	EnableInterrupt(state);
	return baud;
}

static bool Re01UARTStartRx(DevIntrf_t * const pDev, uint32_t DevAddr) { return true; }
static void Re01UARTStopRx(DevIntrf_t * const pDev) {}
static bool Re01UARTStartTx(DevIntrf_t * const pDev, uint32_t DevAddr) { return true; }
static void Re01UARTStopTx(DevIntrf_t * const pDev) {}

static int Re01UARTRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int Bufflen)
{
	if (pBuff == NULL || Bufflen <= 0)
		return 0;
	Re01UartDev_t *dev = (Re01UartDev_t*)pDev->pDevData;
	int cnt = 0;
	uint32_t state = DisableInterrupt();
	if (pDev->bIntEn)
	{
		while (cnt < Bufflen)
		{
			int len = Bufflen - cnt;
			uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &len);
			if (p == NULL)
				break;
			memcpy(pBuff + cnt, p, len);
			cnt += len;
		}
	}
	else
	{
		uint8_t status = dev->pUartReg2->SSR;
		Re01UartClearErrors(dev, status);
		if (dev->DevNo < 2)
		{
			int count = dev->pUartReg0->FDR_b.R;
			while (cnt < Bufflen && cnt < count && cnt < RE01_UART_CFIFO_SIZE)
				pBuff[cnt++] = dev->pUartReg0->FRDRL;
			if (cnt)
				dev->pUartReg0->SSR_FIFO &= ~(SCI0_SSR_FIFO_RDF_Msk | SCI0_SSR_FIFO_DR_Msk);
		}
		else if (status & SCI2_SSR_RDRF_Msk)
		{
			pBuff[cnt++] = dev->pUartReg2->RDR;
			dev->pUartReg2->SSR_b.RDRF = 0;
		}
	}
	EnableInterrupt(state);
	return cnt;
}

static int Re01UARTTxData(DevIntrf_t * const pDev, const uint8_t *pData, int Datalen)
{
	if (pData == NULL || Datalen <= 0)
		return 0;
	Re01UartDev_t *dev = (Re01UartDev_t*)pDev->pDevData;
	int cnt = 0;
	int retry = pDev->MaxRetry > 0 ? pDev->MaxRetry : 5;
	while (cnt < Datalen && retry-- > 0)
	{
		uint32_t state = DisableInterrupt();
		if (pDev->bIntEn)
		{
			int len = Datalen - cnt;
			uint8_t *p = CFifoResvMultiple(dev->pUartDev->hTxFifo, &len);
			if (p)
			{
				memcpy(p, pData + cnt, len);
				CFifoPutMultiple(dev->pUartDev->hTxFifo, &len);
				cnt += len;
			}
			Re01UartFillTx(dev);
		}
		else if (dev->DevNo < 2)
		{
			int space = RE01_UART_CFIFO_SIZE - dev->pUartReg0->FDR_b.T;
			int sent = 0;
			while (cnt < Datalen && sent < space && sent < RE01_UART_CFIFO_SIZE)
			{
				dev->pUartReg0->FTDRL = pData[cnt++];
				sent++;
			}
			if (sent)
			{
				(void)dev->pUartReg0->SSR_FIFO;
				dev->pUartReg0->SSR_FIFO_b.TDFE = 0;
			}
		}
		else if (dev->pUartReg2->SSR & SCI2_SSR_TDRE_Msk)
		{
			dev->pUartReg2->TDR = pData[cnt++];
			dev->pUartReg2->SSR_b.TDRE = 0;
		}
		EnableInterrupt(state);
	}
	return cnt;
}

static void Re01UARTDisable(DevIntrf_t * const pDev)
{
	((Re01UartDev_t*)pDev->pDevData)->pUartReg2->SCR = 0;
}

static void Re01UARTEnable(DevIntrf_t * const pDev)
{
	Re01UartDev_t *dev = (Re01UartDev_t*)pDev->pDevData;
	uint32_t state = DisableInterrupt();
	dev->pUartReg2->SCR = SCI2_SCR_RE_Msk | SCI2_SCR_TE_Msk;
	if (pDev->bIntEn)
	{
		dev->pUartReg2->SCR |= SCI2_SCR_RIE_Msk;
		Re01UartFillTx(dev);
	}
	EnableInterrupt(state);
}

static void Re01UARTPowerOff(DevIntrf_t * const pDev)
{
	Re01UARTDisable(pDev);
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState) {}

bool UARTInit(UARTDEV * const pDev, const UARTCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->pIOPinMap == NULL || pCfg->NbIOPins < 2 ||
		pCfg->DevNo < 0 || pCfg->DevNo >= s_NbRe01UartDev || pCfg->Rate <= 0 || pCfg->bDMAMode ||
		(pCfg->pRxMem && pCfg->RxMemSize < (int)CFIFO_MEMSIZE(1)) ||
		(pCfg->pTxMem && pCfg->TxMemSize < (int)CFIFO_MEMSIZE(1)))
		return false;
	Re01UartDev_t *dev = &s_Re01UartDev[pCfg->DevNo];
	uint32_t state = DisableInterrupt();
	if (dev->pUartDev && dev->pUartDev != pDev)
	{
		EnableInterrupt(state);
		return false;
	}
	MSTP->MSTPCRB &= ~s_Re01UartStopMask[pCfg->DevNo];
	dev->pUartReg2->SCR = 0;
	Re01UartReleaseIRQs(dev);
	dev->pUartDev = pDev;
	pDev->DevIntrf.pDevData = dev;
	pDev->DevIntrf.EnCnt = 0;
	pDev->hRxFifo = CFifoInit(pCfg->pRxMem ? pCfg->pRxMem : dev->RxFifoMem,
			pCfg->pRxMem ? pCfg->RxMemSize : RE01_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	pDev->hTxFifo = CFifoInit(pCfg->pTxMem ? pCfg->pTxMem : dev->TxFifoMem,
			pCfg->pTxMem ? pCfg->TxMemSize : RE01_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	if (pDev->hRxFifo == NULL || pDev->hTxFifo == NULL)
	{
		dev->pUartDev = NULL;
		EnableInterrupt(state);
		return false;
	}
	uint8_t smr = 0;
	if (pCfg->Parity != UART_PARITY_NONE)
	{
		smr |= SCI2_SMR_PE_Msk;
		if (pCfg->Parity == UART_PARITY_ODD)
			smr |= SCI2_SMR_PM_Msk;
	}
	if (pCfg->StopBits == 2)
		smr |= SCI2_SMR_STOP_Msk;
	if (pCfg->DataBits == 7 || pCfg->DataBits == 9)
		smr |= SCI2_SMR_CHR_Msk;
	dev->pUartReg2->SMR = smr;
	dev->pUartReg2->SCMR_b.CHR1 = pCfg->DataBits == 9 ? 0 : 1;
	dev->pUartReg2->SPMR_b.CTSE = pCfg->FlowControl == UART_FLWCTRL_HW && pCfg->NbIOPins > 2;
	if (dev->DevNo < 2)
	{
		dev->pUartReg0->FCR = SCI0_FCR_FM_Msk | SCI0_FCR_RFRST_Msk | SCI0_FCR_TFRST_Msk;
		dev->pUartReg0->FCR = SCI0_FCR_FM_Msk | (15UL << SCI0_FCR_TTRG_Pos) |
				(15UL << SCI0_FCR_RSTRG_Pos);
	}
	if (Re01UARTSetRate(&pDev->DevIntrf, pCfg->Rate) == 0)
	{
		dev->pUartDev = NULL;
		EnableInterrupt(state);
		return false;
	}
	pDev->Mode = pCfg->Mode;
	pDev->Duplex = pCfg->Duplex;
	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->Parity = pCfg->Parity;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->bRxReady = false;
	pDev->bTxReady = true;
	pDev->RxOvrErrCnt = pDev->ParErrCnt = pDev->FramErrCnt = pDev->RxDropCnt = pDev->TxDropCnt = 0;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
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
	pDev->DevIntrf.PowerOff = Re01UARTPowerOff;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	if (pCfg->bIntMode)
	{
		uint8_t evt = s_Re01UartRxEvent[dev->DevNo];
		dev->IrqRxi = Re01RegisterIntHandler(evt, pCfg->IntPrio, Re01UartIRQHandler, dev);
		dev->IrqTxi = Re01RegisterIntHandler(evt + 1, pCfg->IntPrio, Re01UartIRQHandler, dev);
		dev->IrqEri = Re01RegisterIntHandler(evt + 3, pCfg->IntPrio, Re01UartIRQHandler, dev);
		if (dev->IrqRxi == (IRQn_Type)-1 || dev->IrqTxi == (IRQn_Type)-1 || dev->IrqEri == (IRQn_Type)-1)
		{
			Re01UartReleaseIRQs(dev);
			dev->pUartDev = NULL;
			EnableInterrupt(state);
			return false;
		}
	}
	IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, pCfg->FlowControl == UART_FLWCTRL_HW ? pCfg->NbIOPins : 2);
	pDev->DevIntrf.EnCnt = 1;
	Re01UARTEnable(&pDev->DevIntrf);
	EnableInterrupt(state);
	return true;
}
