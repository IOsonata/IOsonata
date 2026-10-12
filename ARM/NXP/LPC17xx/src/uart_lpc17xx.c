/*--------------------------------------------------------------------------
File   : lpc17xx_uart.c

Author : Hoang Nguyen Hoan          Jan. 16, 2014

Desc   : LPC17xx UART implementation

Copyright (c) 2014, I-SYST inc., all rights reserved

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
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include "uart_lpcxx.h"
#include "uart_lpc_target.h"

// PCONP and PCLKSEL fields, in public UART device-number order.
typedef struct {
	uint32_t PowerMask;
	volatile uint32_t *pClockSel;
	uint32_t ClockShift;
	IRQn_Type IrqNo;
} Lpc17Uart_t;

static const Lpc17Uart_t s_Lpc17Uart[] = {
	{1U << 3, &LPC_SC->PCLKSEL0, 6, UART0_IRQn},
	{1U << 4, &LPC_SC->PCLKSEL0, 8, UART1_IRQn},
	{1U << 24, &LPC_SC->PCLKSEL1, 16, UART2_IRQn},
	{1U << 25, &LPC_SC->PCLKSEL1, 18, UART3_IRQn}
};
static const uint8_t s_Lpc17ClockDiv[] = {4, 1, 2, 8};

bool LpcUARTConfigValid(const UARTCfg_t *pCfg)
{
	return (pCfg->FlowControl != UART_FLWCTRL_HW || (pCfg->DevNo == 1 && pCfg->NbIOPins >= 4)) &&
		(!pCfg->bIrDAMode || (pCfg->DevNo != 1 && pCfg->IrDAPulseDiv >= 0 && pCfg->IrDAPulseDiv <= 7));
}

void LpcUARTClock(int DevNo, bool bOn)
{
	if (bOn)
	{
		LPC_SC->PCONP |= s_Lpc17Uart[DevNo].PowerMask;
	}
	else
	{
		LPC_SC->PCONP &= ~s_Lpc17Uart[DevNo].PowerMask;
	}
}

void LpcUARTPrepare(int DevNo)
{
	const Lpc17Uart_t *dev = &s_Lpc17Uart[DevNo];

	LpcUARTClock(DevNo, true);
	*dev->pClockSel = (*dev->pClockSel & ~(3U << dev->ClockShift)) | (1U << dev->ClockShift);
}

uint32_t LpcUARTClockFreq(int DevNo)
{
	const Lpc17Uart_t *dev = &s_Lpc17Uart[DevNo];
	uint32_t sel = (*dev->pClockSel >> dev->ClockShift) & 3U;

	return SystemCoreClock / s_Lpc17ClockDiv[sel];
}

int LpcUARTIrq(int DevNo)
{
	return s_Lpc17Uart[DevNo].IrqNo;
}

void LpcUARTConfigure(int DevNo, const UARTCfg_t *pCfg)
{
	if (DevNo == 1)
	{
		LPC_UART1->MCR = pCfg->FlowControl == UART_FLWCTRL_HW ?
			LPCUART_MCR_RTSEN | LPCUART_MCR_CTSEN : 0;
		LPC_UART1->RS485CTRL = 0;
	}
	if (DevNo != 1)
	{
		uint32_t icr = 0;

		if (pCfg->bIrDAMode)
		{
			icr = LPCUART_ICR_IRDAEN;
			if (pCfg->bIrDAInvert)
			{
				icr |= LPCUART_ICR_IRDAINV;
			}
			if (pCfg->bIrDAFixPulse)
			{
				icr |= LPCUART_ICR_IRDA_FIXPULSE | ((uint32_t)pCfg->IrDAPulseDiv << 3);
			}
		}
		if (DevNo == 0)
		{
			LPC_UART0->ICR = icr;
		}
		else if (DevNo == 2)
		{
			LPC_UART2->ICR = icr;
		}
		else
		{
			LPC_UART3->ICR = icr;
		}
	}
}

uint32_t LpcUARTModemGet(int DevNo)
{
	if (DevNo != 1)
	{
		return 0;
	}
	uint32_t msr = LPC_UART1->MSR;
	uint32_t line = 0;

	if (msr & LPCUART_MSR_CTS)
	{
		line |= UART_LINESTATE_CTS;
	}
	if (msr & LPCUART_MSR_DSR)
	{
		line |= UART_LINESTATE_DSR;
	}
	if (msr & LPCUART_MSR_RI)
	{
		line |= UART_LINESTATE_RI;
	}
	if (msr & LPCUART_MSR_DCD)
	{
		line |= UART_LINESTATE_DCD;
	}

	return line;
}

void LpcUARTModemSet(int DevNo, uint32_t LineState)
{
	if (DevNo != 1)
	{
		return;
	}
	uint32_t mcr = LPC_UART1->MCR & ~(LPCUART_MCR_DTRCTRL | LPCUART_MCR_RTSCTRL);

	if (LineState & UART_LINESTATE_DTR)
	{
		mcr |= LPCUART_MCR_DTRCTRL;
	}
	if (LineState & UART_LINESTATE_RTS)
	{
		mcr |= LPCUART_MCR_RTSCTRL;
	}
	LPC_UART1->MCR = mcr;
}

void UART0_IRQHandler(void)
{
	LpcUARTIrqHandler(0);
}

void UART1_IRQHandler(void)
{
	LpcUARTIrqHandler(1);
}

void UART2_IRQHandler(void)
{
	LpcUARTIrqHandler(2);
}

void UART3_IRQHandler(void)
{
	LpcUARTIrqHandler(3);
}
