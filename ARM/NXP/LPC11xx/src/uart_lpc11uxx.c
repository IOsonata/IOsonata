/*--------------------------------------------------------------------------
File   : uart_lpc11uxx.c

Author : Hoang Nguyen Hoan          Oct. 26, 2014

Desc   : LPC11Uxx UART implementation

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

#define LPC11U_UART_CLOCK		(1U << 12)

extern uint32_t SystemMainClkFreq;

// UM10462 chapter 7. Keep pin routing in the family adapter.
static bool Lpc11UARTSignalPin(int Signal, const IOPinCfg_t *pPin)
{
	switch (Signal)
	{
		case UARTPIN_RX_IDX:
			return (pPin->PortNo == 0 && pPin->PinNo == 18 && pPin->PinOp == IOPINOP_FUNC1) ||
				(pPin->PortNo == 1 &&
				((pPin->PinNo == 14 && pPin->PinOp == IOPINOP_FUNC3) ||
				((pPin->PinNo == 17 || pPin->PinNo == 26) && pPin->PinOp == IOPINOP_FUNC2)));
		case UARTPIN_TX_IDX:
			return (pPin->PortNo == 0 && pPin->PinNo == 19 && pPin->PinOp == IOPINOP_FUNC1) ||
				(pPin->PortNo == 1 &&
				((pPin->PinNo == 13 && pPin->PinOp == IOPINOP_FUNC3) ||
				((pPin->PinNo == 18 || pPin->PinNo == 27) && pPin->PinOp == IOPINOP_FUNC2)));
		case UARTPIN_CTS_IDX:
			return pPin->PortNo == 0 && pPin->PinNo == 7 && pPin->PinOp == IOPINOP_FUNC1;
		case UARTPIN_RTS_IDX:
			return pPin->PortNo == 0 && pPin->PinNo == 17 && pPin->PinOp == IOPINOP_FUNC1;
		case UARTPIN_DCD_IDX:
			return pPin->PortNo == 1 && (pPin->PinNo == 15 || pPin->PinNo == 21) &&
				pPin->PinOp == IOPINOP_FUNC1;
		case UARTPIN_DTE_IDX:
			// The generic DTE slot holds the UART's DSR input.
			return pPin->PortNo == 1 && (pPin->PinNo == 14 || pPin->PinNo == 20) &&
				pPin->PinOp == IOPINOP_FUNC1;
		case UARTPIN_DTR_IDX:
			return pPin->PortNo == 1 && (pPin->PinNo == 13 || pPin->PinNo == 19) &&
				pPin->PinOp == IOPINOP_FUNC1;
		case UARTPIN_RI_IDX:
			return pPin->PortNo == 1 && (pPin->PinNo == 16 || pPin->PinNo == 22) &&
				pPin->PinOp == IOPINOP_FUNC1;
	}
	return false;
}

bool LpcUARTConfigValid(const UARTCfg_t *pCfg)
{
	if (pCfg->NbIOPins > UART_NB_PINS ||
		(pCfg->bIrDAMode && (pCfg->IrDAPulseDiv < 0 || pCfg->IrDAPulseDiv > 7)) ||
		(pCfg->FlowControl == UART_FLWCTRL_HW && pCfg->NbIOPins < 4))
	{
		return false;
	}
	const IOPinCfg_t *pins = (const IOPinCfg_t *)pCfg->pIOPinMap;
	uint32_t used[2] = {0, 0};

	for (int i = 0; i < pCfg->NbIOPins; i++)
	{
		const IOPinCfg_t *pin = &pins[i];

		// IOPinConfig ignores negative port/pin entries (unconnected pads).
		if (pin->PortNo < 0 || pin->PinNo < 0)
		{
			if (pCfg->FlowControl == UART_FLWCTRL_HW &&
				(i == UARTPIN_CTS_IDX || i == UARTPIN_RTS_IDX))
			{
				return false;
			}
			continue;
		}
		if (!Lpc11UARTSignalPin(i, pin))
		{
			return false;
		}
		uint32_t mask = 1UL << pin->PinNo;

		// Alternate RX/TX routes share pads with some modem signals.
		if (used[pin->PortNo] & mask)
		{
			return false;
		}
		used[pin->PortNo] |= mask;
	}
	return true;
}

void LpcUARTClock(int DevNo, bool bOn)
{
	(void)DevNo;
	if (bOn)
	{
		LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_UART_CLOCK;
	}
	else
	{
		LPC_SYSCON->SYSAHBCLKCTRL &= ~LPC11U_UART_CLOCK;
	}
}

void LpcUARTPrepare(int DevNo)
{
	LpcUARTClock(DevNo, true);
	// The main clock allows an exact 1 Mbaud at the normal 48 MHz setting.
	LPC_SYSCON->UARTCLKDIV = 1;
}

uint32_t LpcUARTClockFreq(int DevNo)
{
	(void)DevNo;
	uint32_t div = LPC_SYSCON->UARTCLKDIV & 0xFFU;

	return div != 0 ? SystemMainClkFreq / div : 0;
}

int LpcUARTIrq(int DevNo)
{
	(void)DevNo;
	return UART_IRQn;
}

void LpcUARTConfigure(int DevNo, const UARTCfg_t *pCfg)
{
	(void)DevNo;
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
	LPC_USART->ICR = icr;
	LPC_USART->OSR = 0xF0U;
	LPC_USART->HDEN = 0;
	LPC_USART->SYNCCTRL = 0;
	LPC_USART->SCICTRL = 0;
	LPC_USART->RS485CTRL = 0;
	LPC_USART->MCR = pCfg->FlowControl == UART_FLWCTRL_HW ?
		LPCUART_MCR_RTSEN | LPCUART_MCR_CTSEN : 0;
}

uint32_t LpcUARTModemGet(int DevNo)
{
	(void)DevNo;
	uint32_t msr = LPC_USART->MSR;
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
	(void)DevNo;
	uint32_t mcr = LPC_USART->MCR & ~(LPCUART_MCR_DTRCTRL | LPCUART_MCR_RTSCTRL);

	if (LineState & UART_LINESTATE_DTR)
	{
		mcr |= LPCUART_MCR_DTRCTRL;
	}
	if (LineState & UART_LINESTATE_RTS)
	{
		mcr |= LPCUART_MCR_RTSCTRL;
	}
	LPC_USART->MCR = mcr;
}

void UART_IRQHandler(void)
{
	LpcUARTIrqHandler(0);
}
