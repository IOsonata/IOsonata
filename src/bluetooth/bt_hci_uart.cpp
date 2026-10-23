/**-------------------------------------------------------------------------
@file	bt_hci_uart.cpp

@brief	Bluetooth HCI UART transport (H4)

See bt_hci_uart.h for the receive and command model.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include <string.h>

#include "bluetooth/bt_hci_uart.h"

// The core finds the transport it belongs to through its owner data
static inline BtHciUartDev_t *BtHciUartOwner(BtHciH4Dev_t * const pH4)
{
	return (BtHciUartDev_t*)pH4->pCtx;
}

static void BtHciUartH4Rx(BtHciH4Dev_t * const pH4, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	BtHciUartDev_t *dev = BtHciUartOwner(pH4);

	if (dev->RxHandler != nullptr)
	{
		dev->RxHandler(dev, Type, pPkt, Len);
	}
}

static void BtHciUartH4CmdRsp(BtHciH4Dev_t * const pH4, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	BtHciUartDev_t *dev = BtHciUartOwner(pH4);

	if (dev->CmdRspHandler != nullptr)
	{
		dev->CmdRspHandler(dev, Type, pPkt, Len);
	}
}

static void BtHciUartH4RxReady(BtHciH4Dev_t * const pH4)
{
	BtHciUartDev_t *dev = BtHciUartOwner(pH4);

	if (dev->RxReady != nullptr)
	{
		dev->RxReady(dev);
	}
}

static int BtHciUartH4TxRoom(BtHciH4Dev_t * const pH4)
{
	return CFifoAvail(BtHciUartOwner(pH4)->Uart.hTxFifo);
}

// UART event handler, in the UART interrupt. Only reports that bytes arrived,
// the framing runs in the owner context.
static int BtHciUartEvtHandler(UARTDev_t * const pUart, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pBuffer;
	(void)BufferLen;

	// Uart is the first member of the transport
	BtHciUartDev_t *dev = (BtHciUartDev_t *)pUart;

	if (EvtId == UART_EVT_RXDATA || EvtId == UART_EVT_RXTIMEOUT)
	{
		BtHciH4RxNotify(&dev->H4);
	}

	return 0;
}

bool BtHciUartInit(BtHciUartDev_t * const pDev, const BtHciUartCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pUartCfg == nullptr)
	{
		return false;
	}

	memset((void*)pDev, 0, sizeof(BtHciUartDev_t));

	pDev->RxHandler = pCfg->RxHandler;
	pDev->CmdRspHandler = pCfg->CmdRspHandler;
	pDev->RxReady = pCfg->RxReady;
	pDev->pCtx = pCfg->pCtx;

	// The core checks the packet memory. The UART is not touched by it, only
	// its interface address is kept.
	BtHciH4Cfg_t h4cfg = {};

	h4cfg.pPhyIntrf = &pDev->Uart.DevIntrf;
	h4cfg.pPktMem = pCfg->pPktMem;
	h4cfg.PktMemSize = pCfg->PktMemSize;
	h4cfg.RxHandler = BtHciUartH4Rx;
	h4cfg.CmdRspHandler = BtHciUartH4CmdRsp;
	h4cfg.RxReady = BtHciUartH4RxReady;
	h4cfg.TxRoom = BtHciUartH4TxRoom;
	h4cfg.TimeoutMs = pCfg->TimeoutMs;
	h4cfg.pCtx = pDev;

	if (BtHciH4Init(&pDev->H4, &h4cfg) == false)
	{
		return false;
	}

	// A FIFO that pushes out old bytes when full would cut packets in the
	// middle, so both FIFOs refuse instead. The transport needs the UART
	// events, so interrupt mode is not optional either.
	UARTCfg_t ucfg = *pCfg->pUartCfg;
	ucfg.bIntMode = true;
	ucfg.bFifoBlocking = true;
	ucfg.EvtCallback = BtHciUartEvtHandler;

	if (UARTInit(&pDev->Uart, &ucfg) == false)
	{
		return false;
	}

	if (CFifoAvail(pDev->Uart.hTxFifo) < (int)BT_HCI_UART_TXFIFO_MIN)
	{
		// A packet that does not fit the TX FIFO could never be sent whole
		UARTDisable(&pDev->Uart);

		return false;
	}

	// Reads are polled until the UART has nothing. Retrying an empty read the
	// default number of times only delays the wait loops, and a write is made
	// only when the TX FIFO has room for all of it.
	pDev->Uart.DevIntrf.MaxRetry = 1;

	return true;
}

void BtHciUartFlush(BtHciUartDev_t * const pDev)
{
	if (pDev != nullptr)
	{
		BtHciH4Flush(&pDev->H4);
	}
}

bool BtHciUartSend(BtHciUartDev_t * const pDev, uint8_t Type, const void *pData, uint32_t Len)
{
	return pDev != nullptr && BtHciH4Send(&pDev->H4, Type, pData, Len);
}

void BtHciUartPoll(BtHciUartDev_t * const pDev)
{
	if (pDev != nullptr)
	{
		BtHciH4Poll(&pDev->H4);
	}
}

bool BtHciUartProcess(BtHciUartDev_t * const pDev, int MaxPkt)
{
	return pDev != nullptr && BtHciH4Process(&pDev->H4, MaxPkt);
}

uint8_t BtHciUartCommand(BtHciUartDev_t * const pDev, BtHciDevice_t * const pHci, uint16_t OpCode,
						 const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (pDev == nullptr)
	{
		return BT_HCI_ERR_INVALID_COMMAND_PARAM;
	}

	return BtHciH4Command(&pDev->H4, pHci, OpCode, pParam, ParamLen, pRet, RetLen);
}
