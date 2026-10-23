/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_uart.cpp

@brief	UART transport of the HCI host to a controller behind an H4 link

Opens a BtHciUart (bt_hci_uart.h) on the UART the application gives in
g_BtHciUartCfg and hands its H4 layer to the controller port of the host
(bt_hci_h4_ctlr.cpp). This is the transport of an MCU without a radio: an
nRF91, an STM32 or any other MCU wired to an nRF5x running the HciController
firmware.

When the application leaves pRxMem or pTxMem of g_BtHciUartCfg NULL, this
transport supplies the FIFO memory. The RX FIFO holds what arrives while the
event queue is busy, 2 KB is 20 ms at 1 Mbaud. There is no HCI controller to
host flow control, so the FIFO has to be sized for the traffic; hardware flow
control on the UART is recommended.

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
#include <stdint.h>

#include "cfifo.h"
#include "bluetooth/bt_hci_uart.h"
#include "bluetooth/bt_hci_h4_ctlr.h"

// UART FIFOs used when the application gives none
#ifndef BT_HCI_CTLR_UART_RXFIFO_SIZE
#define BT_HCI_CTLR_UART_RXFIFO_SIZE	2048
#endif

#ifndef BT_HCI_CTLR_UART_TXFIFO_SIZE
#define BT_HCI_CTLR_UART_TXFIFO_SIZE	1024
#endif

static_assert(BT_HCI_CTLR_UART_TXFIFO_SIZE >= BT_HCI_UART_TXFIFO_MIN,
			  "TX FIFO must hold one whole HCI packet");

// HCI transport to the controller
static BtHciUartDev_t s_BtHciUart;

alignas(4) static uint8_t s_BtHciCtlrUartRxMem[CFIFO_MEMSIZE(BT_HCI_CTLR_UART_RXFIFO_SIZE)];
alignas(4) static uint8_t s_BtHciCtlrUartTxMem[CFIFO_MEMSIZE(BT_HCI_CTLR_UART_TXFIFO_SIZE)];

// The link callbacks, forwarded to the controller port
static void BtHciCtlrUartRx(BtHciUartDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)pDev;

	BtHciH4CtlrRx(Type, pPkt, Len);
}

static void BtHciCtlrUartCmdRsp(BtHciUartDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	(void)pDev;

	BtHciH4CtlrCmdRsp(Type, pPkt, Len);
}

static void BtHciCtlrUartRxReady(BtHciUartDev_t * const pDev)
{
	(void)pDev;

	BtHciH4CtlrRxReady();
}

BtHciH4Dev_t *BtHciCtlrH4Open(uint8_t *pPktMem, uint32_t PktMemSize)
{
	UARTCfg_t ucfg = g_BtHciUartCfg;

	if (ucfg.pRxMem == nullptr)
	{
		ucfg.pRxMem = s_BtHciCtlrUartRxMem;
		ucfg.RxMemSize = sizeof(s_BtHciCtlrUartRxMem);
	}
	if (ucfg.pTxMem == nullptr)
	{
		ucfg.pTxMem = s_BtHciCtlrUartTxMem;
		ucfg.TxMemSize = sizeof(s_BtHciCtlrUartTxMem);
	}

	BtHciUartCfg_t hcicfg = {};
	hcicfg.pUartCfg = &ucfg;
	hcicfg.pPktMem = pPktMem;
	hcicfg.PktMemSize = PktMemSize;
	hcicfg.RxHandler = BtHciCtlrUartRx;
	hcicfg.CmdRspHandler = BtHciCtlrUartCmdRsp;
	hcicfg.RxReady = BtHciCtlrUartRxReady;

	if (BtHciUartInit(&s_BtHciUart, &hcicfg) == false)
	{
		return nullptr;
	}

	return &s_BtHciUart.H4;
}
