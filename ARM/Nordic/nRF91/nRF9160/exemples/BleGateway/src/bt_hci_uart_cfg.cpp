/**-------------------------------------------------------------------------
@file	bt_hci_uart_cfg.cpp

@brief	UART to the Bluetooth controller of the nRF91 port

The nRF91 Bluetooth port runs the host over this UART to an nRF5x running
HciController. The FIFO memory is left to the port.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "bluetooth/bt_hci_uart.h"

#include "board.h"

static const IOPinCfg_t s_HciUartPins[] = HCI_UART_PINS;

const UARTCfg_t g_BtHciUartCfg = {
	.DevNo = HCI_UART_DEVNO,
	.pIOPinMap = s_HciUartPins,
	.NbIOPins = sizeof(s_HciUartPins) / sizeof(IOPinCfg_t),
	.Rate = HCI_UART_RATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_HW,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = nullptr,				// Replaced by the HCI transport
	.bFifoBlocking = true,
	.RxMemSize = 0,						// FIFO memory from the port
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
	.bIrDAMode = false,
	.bIrDAInvert = false,
	.bIrDAFixPulse = false,
	.IrDAPulseDiv = 0,
	.Duplex = UART_DUPLEX_FULL,
	.Mode = UART_MODE_UART,
};
