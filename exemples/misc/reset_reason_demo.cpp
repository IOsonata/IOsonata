/**-------------------------------------------------------------------------
@file	reset_reason_demo.cpp

@brief	Report the MCU reset reason and exercise a software reset.

Send 'r' to request a software reset. An external RESET press can
be used to verify the pin-reset reason. The reset is not a wake from
Standby, and this example does not configure low-power modes.

MIT License
Copyright (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "board.h"

#ifndef UART_BAUDRATE
#define UART_BAUDRATE	115200
#endif

static const IOPinCfg_t s_UartPins[] = {
	{ UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = false,
	.IntPrio = 2,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = false,
};

static UART s_Uart;

int main(void)
{
	uint32_t reason;
	uint8_t command;

	reason = SystemResetReason();
	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}

	s_Uart.printf("ResetReasonDemo\r\n");
	s_Uart.printf("Reset reason 0x%08x\r\n", (unsigned)reason);
	s_Uart.printf("Power-on %u, pin %u, watchdog %u, software %u\r\n",
		(reason & SYSTEM_RESET_POWERON) != 0U,
		(reason & SYSTEM_RESET_PIN) != 0U,
		(reason & SYSTEM_RESET_WDT) != 0U,
		(reason & SYSTEM_RESET_SOFT) != 0U);
	s_Uart.printf("Send r to issue software reset; use board RESET for pin reset\r\n");

	while (1)
	{
		if (s_Uart.Rx(&command, 1) == 1 && command == 'r')
		{
			s_Uart.printf("Software reset\r\n");
			msDelay(250);
			NVIC_SystemReset();
		}
	}

	return 0;
}
