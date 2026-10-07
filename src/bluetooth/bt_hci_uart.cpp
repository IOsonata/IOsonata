/**-------------------------------------------------------------------------
@file	bt_hci_uart.cpp

@brief	Bluetooth HCI UART transport (H4)

See bt_hci_uart.h for the receive and command model.

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
#include <stddef.h>
#include <string.h>

#include "idelay.h"
#include "bluetooth/bt_hci_uart.h"

// Wait step of the command and TX room loops, in usec. A wait of TimeoutMs
// runs at least TimeoutMs * 1000 / BT_HCI_UART_POLL_US steps.
#define BT_HCI_UART_POLL_US				20U

// Packet header lengths, after the type byte
#define BT_HCI_UART_HDRLEN_CMD			3U		// opcode, parameter length
#define BT_HCI_UART_HDRLEN_ACL			4U		// handle and flags, data length
#define BT_HCI_UART_HDRLEN_SCO			3U		// handle and flags, data length
#define BT_HCI_UART_HDRLEN_EVT			2U		// event code, parameter length
#define BT_HCI_UART_HDRLEN_ISO			4U		// handle and flags, 14 bit data length

// Framing state
enum {
	BT_HCI_UART_STATE_TYPE,		// waiting for a packet type byte
	BT_HCI_UART_STATE_HDR,		// receiving the packet header
	BT_HCI_UART_STATE_BODY,		// receiving the packet parameters or data
	BT_HCI_UART_STATE_SKIP,		// dropping a packet too long for a slot
};

static uint8_t BtHciUartHdrLen(uint8_t Type)
{
	switch (Type)
	{
		case BT_HCI_UART_PKT_CMD:
			return BT_HCI_UART_HDRLEN_CMD;
		case BT_HCI_UART_PKT_ACL:
			return BT_HCI_UART_HDRLEN_ACL;
		case BT_HCI_UART_PKT_SCO:
			return BT_HCI_UART_HDRLEN_SCO;
		case BT_HCI_UART_PKT_EVT:
			return BT_HCI_UART_HDRLEN_EVT;
		case BT_HCI_UART_PKT_ISO:
			return BT_HCI_UART_HDRLEN_ISO;
		default:
			return 0;
	}
}

// Length of what follows the header, read from the header
static uint16_t BtHciUartBodyLen(uint8_t Type, const uint8_t *pHdr)
{
	switch (Type)
	{
		case BT_HCI_UART_PKT_CMD:
		case BT_HCI_UART_PKT_SCO:
			return pHdr[2];
		case BT_HCI_UART_PKT_ACL:
			return (uint16_t)(pHdr[2] | (pHdr[3] << 8));
		case BT_HCI_UART_PKT_ISO:
			return (uint16_t)((pHdr[2] | (pHdr[3] << 8)) & 0x3FFFU);
		case BT_HCI_UART_PKT_EVT:
		default:
			return pHdr[1];
	}
}

// Wait steps of TimeoutMs
static uint32_t BtHciUartWaitCount(BtHciUartDev_t * const pDev)
{
	return pDev->TimeoutMs * (1000U / BT_HCI_UART_POLL_US);
}

// UART event handler, in the UART interrupt. Only reports that bytes arrived,
// the framing runs in the owner context.
static int BtHciUartEvtHandler(UARTDev_t * const pUart, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pBuffer;
	(void)BufferLen;

	// Uart is the first member of the transport
	BtHciUartDev_t *dev = (BtHciUartDev_t *)pUart;

	if ((EvtId == UART_EVT_RXDATA || EvtId == UART_EVT_RXTIMEOUT) && dev->RxReady != nullptr)
	{
		dev->RxReady(dev);
	}

	return 0;
}

bool BtHciUartInit(BtHciUartDev_t * const pDev, const BtHciUartCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pUartCfg == nullptr ||
		pCfg->pPktMem == nullptr)
	{
		return false;
	}

	if (((uintptr_t)pCfg->pPktMem & (alignof(CFifo_t) - 1U)) != 0 ||
		pCfg->PktMemSize < BT_HCI_UART_PKTMEM_SIZE(1))
	{
		return false;
	}

	memset((void*)pDev, 0, sizeof(BtHciUartDev_t));

	pDev->hPktFifo = CFifoInit(pCfg->pPktMem, pCfg->PktMemSize, sizeof(BtHciUartPkt_t), true);
	if (pDev->hPktFifo == nullptr)
	{
		return false;
	}

	pDev->RxHandler = pCfg->RxHandler;
	pDev->CmdRspHandler = pCfg->CmdRspHandler;
	pDev->RxReady = pCfg->RxReady;
	pDev->pCtx = pCfg->pCtx;
	pDev->TimeoutMs = pCfg->TimeoutMs > 0 ? pCfg->TimeoutMs : BT_HCI_UART_TIMEOUT_DEFAULT;
	pDev->State = BT_HCI_UART_STATE_TYPE;

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
	if (pDev == nullptr || pDev->bPolling || pDev->bProcessing)
	{
		return;
	}

	uint8_t buf[sizeof(pDev->Stage)];

	while (UARTRx(&pDev->Uart, buf, (int)sizeof(buf)) > 0)
	{
	}

	CFifoFlush(pDev->hPktFifo);
	pDev->pAsm = nullptr;
	pDev->State = BT_HCI_UART_STATE_TYPE;
	pDev->SkipCnt = 0;
	pDev->StageIdx = 0;
	pDev->StageCnt = 0;
	pDev->bSpareHeld = false;
}

bool BtHciUartSend(BtHciUartDev_t * const pDev, uint8_t Type, const void *pData, uint32_t Len)
{
	if (pDev == nullptr || pData == nullptr || Len == 0 || Len > BT_HCI_UART_PKT_MAXLEN)
	{
		return false;
	}

	// Wait until the whole packet fits. Writing part of it now and the rest
	// later is no option: another packet written in between would be framed
	// by the other side as the rest of this one.
	uint32_t n = BtHciUartWaitCount(pDev);

	while (CFifoAvail(pDev->Uart.hTxFifo) < (int)(Len + 1))
	{
		if (n == 0)
		{
			pDev->TxFailCnt++;

			return false;
		}
		n--;
		usDelay(BT_HCI_UART_POLL_US);
	}

	// One write: the UART takes it whole into its FIFO, or not at all when
	// the interface is busy in another context.
	uint8_t pkt[1 + BT_HCI_UART_PKT_MAXLEN];

	pkt[0] = Type;
	memcpy(&pkt[1], pData, Len);

	if (UARTTx(&pDev->Uart, pkt, (int)(Len + 1)) != (int)(Len + 1))
	{
		pDev->TxFailCnt++;

		return false;
	}

	return true;
}

// The packet in the framing slot is complete. Command Complete and Command
// Status go to CmdRspHandler and the slot is free for the next packet.
// Everything else is published to the queue, or held in the spare slot when
// it was framed there.
static void BtHciUartPktDone(BtHciUartDev_t * const pDev)
{
	BtHciUartPkt_t *p = pDev->pAsm;

	p->Len = pDev->AsmLen;
	pDev->pAsm = nullptr;
	pDev->State = BT_HCI_UART_STATE_TYPE;

	if (p->Type == BT_HCI_UART_PKT_EVT && pDev->CmdRspHandler != nullptr &&
		(p->Data[0] == BT_HCI_EVT_COMMAND_COMPLETE || p->Data[0] == BT_HCI_EVT_COMMAND_STATUS))
	{
		pDev->CmdRspHandler(pDev, p->Type, p->Data, p->Len);
	}
	else if (p == &pDev->Spare)
	{
		pDev->bSpareHeld = true;
	}
	else
	{
		CFifoPut(pDev->hPktFifo);
	}
}

// Move a packet held in the spare slot to the queue. Returns false when it
// is still held: nothing more is framed until it is in the queue, it is
// older than anything behind it.
static bool BtHciUartSpareFlush(BtHciUartDev_t * const pDev)
{
	if (pDev->bSpareHeld == false)
	{
		return true;
	}

	BtHciUartPkt_t *p = (BtHciUartPkt_t*)CFifoResv(pDev->hPktFifo);
	if (p == nullptr)
	{
		return false;
	}

	memcpy(p, &pDev->Spare, offsetof(BtHciUartPkt_t, Data) + pDev->Spare.Len);
	CFifoPut(pDev->hPktFifo);
	pDev->bSpareHeld = false;

	return true;
}

// Frame the bytes in Stage. Returns false when a packet has to start and
// neither the queue nor the spare slot can take it, the byte stays in Stage.
static bool BtHciUartFrame(BtHciUartDev_t * const pDev)
{
	while (pDev->StageIdx < pDev->StageCnt)
	{
		switch (pDev->State)
		{
			case BT_HCI_UART_STATE_TYPE:
				{
					// A packet held in the spare slot goes to the queue before
					// a new one starts, it is the older one.
					if (BtHciUartSpareFlush(pDev) == false)
					{
						return false;
					}

					BtHciUartPkt_t *p = (BtHciUartPkt_t*)CFifoResv(pDev->hPktFifo);
					if (p == nullptr)
					{
						p = &pDev->Spare;
					}

					uint8_t type = pDev->Stage[pDev->StageIdx++];
					uint8_t hdrlen = BtHciUartHdrLen(type);
					if (hdrlen == 0)
					{
						// Not a packet type: line noise, or a byte left from a
						// packet the UART lost part of. Skip until one is found.
						pDev->SyncErrCnt++;
						break;
					}

					p->Type = type;
					pDev->pAsm = p;
					pDev->HdrLen = hdrlen;
					pDev->AsmIdx = 0;
					pDev->State = BT_HCI_UART_STATE_HDR;
				}
				break;

			case BT_HCI_UART_STATE_HDR:
				pDev->pAsm->Data[pDev->AsmIdx++] = pDev->Stage[pDev->StageIdx++];
				if (pDev->AsmIdx == pDev->HdrLen)
				{
					uint16_t bodylen = BtHciUartBodyLen(pDev->pAsm->Type, pDev->pAsm->Data);
					uint32_t total = (uint32_t)pDev->HdrLen + bodylen;

					if (total > BT_HCI_UART_PKT_MAXLEN)
					{
						// Too long for a slot. Drop it whole so the next byte
						// read is the type of the next packet.
						pDev->DropCnt++;
						pDev->pAsm = nullptr;
						pDev->SkipCnt = bodylen;
						pDev->State = BT_HCI_UART_STATE_SKIP;
					}
					else
					{
						pDev->AsmLen = (uint16_t)total;
						if (bodylen == 0)
						{
							BtHciUartPktDone(pDev);
						}
						else
						{
							pDev->State = BT_HCI_UART_STATE_BODY;
						}
					}
				}
				break;

			case BT_HCI_UART_STATE_BODY:
				{
					uint32_t l = (uint32_t)(pDev->StageCnt - pDev->StageIdx);
					uint32_t left = (uint32_t)(pDev->AsmLen - pDev->AsmIdx);
					if (l > left)
					{
						l = left;
					}
					memcpy(&pDev->pAsm->Data[pDev->AsmIdx], &pDev->Stage[pDev->StageIdx], l);
					pDev->AsmIdx = (uint16_t)(pDev->AsmIdx + l);
					pDev->StageIdx = (uint8_t)(pDev->StageIdx + l);
					if (pDev->AsmIdx == pDev->AsmLen)
					{
						BtHciUartPktDone(pDev);
					}
				}
				break;

			case BT_HCI_UART_STATE_SKIP:
			default:
				{
					uint32_t l = (uint32_t)(pDev->StageCnt - pDev->StageIdx);
					if (l > pDev->SkipCnt)
					{
						l = pDev->SkipCnt;
					}
					pDev->StageIdx = (uint8_t)(pDev->StageIdx + l);
					pDev->SkipCnt = (uint16_t)(pDev->SkipCnt - l);
					if (pDev->SkipCnt == 0)
					{
						pDev->State = BT_HCI_UART_STATE_TYPE;
					}
				}
				break;
		}
	}

	return true;
}

void BtHciUartPoll(BtHciUartDev_t * const pDev)
{
	if (pDev == nullptr || pDev->bPolling)
	{
		// Called again from CmdRspHandler. The slot being handled must not be
		// framed into.
		return;
	}

	pDev->bPolling = true;

	while (true)
	{
		if (BtHciUartSpareFlush(pDev) == false)
		{
			// No slot for the next packet. Its bytes wait in the UART.
			break;
		}

		if (pDev->StageIdx >= pDev->StageCnt)
		{
			int l = UARTRx(&pDev->Uart, pDev->Stage, (int)sizeof(pDev->Stage));
			if (l <= 0)
			{
				break;
			}
			pDev->StageIdx = 0;
			pDev->StageCnt = (uint8_t)l;
		}

		if (BtHciUartFrame(pDev) == false)
		{
			break;
		}
	}

	pDev->bPolling = false;
}

bool BtHciUartProcess(BtHciUartDev_t * const pDev, int MaxPkt)
{
	if (pDev == nullptr || pDev->bProcessing)
	{
		// Called again from RxHandler. The packet being handled is still at
		// the head of the queue and would be handled twice.
		return false;
	}

	pDev->bProcessing = true;

	bool bMore = false;
	int cnt = 0;

	while (true)
	{
		BtHciUartPoll(pDev);

		// The packet stays in the queue while it is handled. A command sent
		// by the handler frames new packets behind it, never over it.
		BtHciUartPkt_t *p = (BtHciUartPkt_t*)CFifoPeek(pDev->hPktFifo);
		if (p == nullptr)
		{
			break;
		}

		if (MaxPkt > 0 && cnt >= MaxPkt)
		{
			bMore = true;
			break;
		}

		if (pDev->RxHandler != nullptr)
		{
			pDev->RxHandler(pDev, p->Type, p->Data, p->Len);
		}

		CFifoGet(pDev->hPktFifo);
		cnt++;
	}

	pDev->bProcessing = false;

	return bMore;
}

uint8_t BtHciUartCommand(BtHciUartDev_t * const pDev, BtHciDevice_t * const pHci, uint16_t OpCode,
						 const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (pDev == nullptr || pHci == nullptr || (ParamLen > 0 && pParam == nullptr))
	{
		return BT_HCI_ERR_INVALID_COMMAND_PARAM;
	}

	if (pDev->bCmdActive)
	{
		// Sent from CmdRspHandler while another command waits. One command
		// at a time: its response could not be told apart from this one.
		return BT_HCI_ERR_COMMAND_DISALLOWED;
	}

	pDev->bCmdActive = true;

	// Command flow control. Num_HCI_Command_Packets of the last Command
	// Complete or Command Status says whether the controller takes one more.
	uint32_t n = BtHciUartWaitCount(pDev);

	while (pHci->CmdCredit <= 0)
	{
		if (n == 0)
		{
			// No credit returned. Count one so a lost response does not stop
			// every command after this one.
			pHci->CmdCredit = 1;
			pDev->bCmdActive = false;

			return BT_HCI_UART_ERR_TIMEOUT;
		}
		n--;
		BtHciUartPoll(pDev);
		usDelay(BT_HCI_UART_POLL_US);
	}

	uint8_t pkt[BT_HCI_UART_HDRLEN_CMD + 255];

	pkt[0] = (uint8_t)(OpCode & 0xFF);
	pkt[1] = (uint8_t)(OpCode >> 8);
	pkt[2] = ParamLen;
	if (ParamLen > 0)
	{
		memcpy(&pkt[BT_HCI_UART_HDRLEN_CMD], pParam, ParamLen);
	}

	// Response matching, filled by BtHciProcessEvent through CmdRspHandler. A
	// Command Complete without a status byte leaves CmdStatus at 0.
	pHci->CmdStatus = 0;
	pHci->pCmdRet = (uint8_t*)pRet;
	pHci->CmdRetLen = pRet != nullptr ? RetLen : 0;
	pHci->CmdDone = false;
	pHci->CmdOpCode = OpCode;

	uint8_t status = BT_HCI_UART_ERR_TIMEOUT;

	if (BtHciUartSend(pDev, BT_HCI_UART_PKT_CMD, pkt, BT_HCI_UART_HDRLEN_CMD + ParamLen))
	{
		pHci->CmdCredit--;

		n = BtHciUartWaitCount(pDev);

		while (pHci->CmdDone == false && n > 0)
		{
			n--;
			BtHciUartPoll(pDev);
			if (pHci->CmdDone == false)
			{
				usDelay(BT_HCI_UART_POLL_US);
			}
		}

		if (pHci->CmdDone)
		{
			status = pHci->CmdStatus;
		}
		else
		{
			// The response may still come. It matches no opcode then and only
			// sets the credit count.
			pHci->CmdCredit = 1;
		}
	}

	pHci->CmdOpCode = 0;
	pHci->pCmdRet = nullptr;
	pHci->CmdRetLen = 0;
	pDev->bCmdActive = false;

	return status;
}
