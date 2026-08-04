/**-------------------------------------------------------------------------
@file	bt_intrf.cpp

@brief	Implementation allow the creation of generic serial interface of
a custom Bluetooth service with multiple user defined characteristics.

@author	Hoang Nguyen Hoan
@date	Feb. 6, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include <string.h>

#include "istddef.h"
#include "cfifo.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_peer.h"
#include "coredev/interrupt.h"

#define BTINTRF_PACKET_SIZE		20
#define BTINTRF_CFIFO_SIZE		BTINTRF_CFIFO_TOTAL_MEMSIZE(2, BTINTRF_PACKET_SIZE)

alignas(CFifo_t) static uint8_t s_BtDevIntrfRxFifoMem[BTINTRF_CFIFO_SIZE];
alignas(CFifo_t) static uint8_t s_BtDevIntrfTxFifoMem[BTINTRF_CFIFO_SIZE];
static BtDevIntrf_t *s_pDefaultRxOwner;
static BtDevIntrf_t *s_pDefaultTxOwner;

static bool BtIntrfFifoMemValid(const uint8_t *pMem, int MemSize,
		uint32_t BlockSize)
{
	if (pMem == nullptr || MemSize <= 0 ||
		((uintptr_t)pMem % alignof(CFifo_t)) != 0)
	{
		return false;
	}

	return (uint32_t)MemSize >= sizeof(CFifo_t) + BlockSize;
}

static uint16_t BtIntrfNotifyHandle(BtGattChar_t *pChar)
{
	if (pChar == nullptr)
	{
		return BT_CONN_HDL_INVALID;
	}

	for (uint16_t i = 0; i < BtPeerCount(); i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer != nullptr && pPeer->Conn.Hdl != BT_CONN_HDL_INVALID &&
			BtGattCharNotifyEnabled(pPeer->Conn.Hdl, pChar))
		{
			return pPeer->Conn.Hdl;
		}
	}

	return BT_CONN_HDL_INVALID;
}

void BtIntrfDisable(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

void BtIntrfEnable(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

uint32_t BtIntrfGetRate(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
	return 0;
}

uint32_t BtIntrfSetRate(DevIntrf_t *pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;
	return 0;
}

bool BtIntrfStartRx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	(void)DevAddr;
	return pDevIntrf != nullptr && pDevIntrf->pDevData != nullptr;
}

int BtIntrfRxData(DevIntrf_t *pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr ||
		pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}

	BtDevIntrf_t *pIntrf = (BtDevIntrf_t *)pDevIntrf->pDevData;
	if (pIntrf->hRxFifo == nullptr)
	{
		return 0;
	}

	int count = 0;
	uint32_t state = DisableInterrupt();
	BtIntrfPkt_t *pPkt = (BtIntrfPkt_t *)CFifoPeek(pIntrf->hRxFifo);
	if (pPkt != nullptr)
	{
		int payloadMax = pIntrf->PacketSize - (int)BTINTRF_PKHDR_LEN;
		if (pPkt->Len > payloadMax)
		{
			// Malformed blocks cannot be delivered. Remove this block so it
			// cannot permanently block the packets behind it.
			(void)CFifoGet(pIntrf->hRxFifo);
			pIntrf->RxDropCnt++;
		}
		else if (BuffLen >= (int)pPkt->Len)
		{
			// Consume only after the caller has provided enough room for the
			// whole packet. The old code consumed first, copied BuffLen bytes,
			// and silently discarded the unread tail.
			pPkt = (BtIntrfPkt_t *)CFifoGet(pIntrf->hRxFifo);
			if (pPkt != nullptr)
			{
				count = (int)pPkt->Len;
				memcpy(pBuff, pPkt->Data, (size_t)count);
			}
		}
	}
	EnableInterrupt(state);

	return count;
}

void BtIntrfStopRx(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

bool BtIntrfStartTx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	(void)DevAddr;
	return pDevIntrf != nullptr && pDevIntrf->pDevData != nullptr;
}

bool BtIntrfNotify(BtDevIntrf_t *pIntrf)
{
	if (pIntrf == nullptr || pIntrf->pSrvc == nullptr ||
		pIntrf->pSrvc->pCharArray == nullptr ||
		pIntrf->TxCharIdx < 0 || pIntrf->TxCharIdx >= pIntrf->pSrvc->NbChar ||
		pIntrf->hTxFifo == nullptr)
	{
		return false;
	}

	BtGattChar_t *pChar = &pIntrf->pSrvc->pCharArray[pIntrf->TxCharIdx];
	uint16_t connHdl = BtIntrfNotifyHandle(pChar);
	if (connHdl == BT_CONN_HDL_INVALID)
	{
		return false;
	}

	if (pIntrf->TransBuffLen > 0)
	{
		if (!BtGattCharNotify(connHdl, pChar, pIntrf->TransBuff,
				(size_t)pIntrf->TransBuffLen))
		{
			return false;
		}
		pIntrf->TransBuffLen = 0;
	}

	for (;;)
	{
		uint32_t state = DisableInterrupt();
		BtIntrfPkt_t *pPkt = (BtIntrfPkt_t *)CFifoGet(pIntrf->hTxFifo);
		if (pPkt == nullptr)
		{
			EnableInterrupt(state);
			return true;
		}

		int payloadMax = pIntrf->PacketSize - (int)BTINTRF_PKHDR_LEN;
		if (pPkt->Len > payloadMax || pPkt->Len > sizeof(pIntrf->TransBuff))
		{
			pIntrf->TxDropCnt++;
			EnableInterrupt(state);
			continue;
		}

		memcpy(pIntrf->TransBuff, pPkt->Data, pPkt->Len);
		pIntrf->TransBuffLen = pPkt->Len;
		EnableInterrupt(state);

		if (!BtGattCharNotify(connHdl, pChar, pIntrf->TransBuff,
				(size_t)pIntrf->TransBuffLen))
		{
			return false;
		}
		pIntrf->TransBuffLen = 0;
	}
}

int BtIntrfTxData(DevIntrf_t *pDevIntrf, const uint8_t *pData, int DataLen)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr ||
		pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	BtDevIntrf_t *pIntrf = (BtDevIntrf_t *)pDevIntrf->pDevData;
	if (pIntrf->hTxFifo == nullptr)
	{
		return 0;
	}

	int payloadMax = pIntrf->PacketSize - (int)BTINTRF_PKHDR_LEN;
	if (payloadMax <= 0)
	{
		return 0;
	}

	int count = 0;
	while (DataLen > 0)
	{
		int len = min(DataLen, payloadMax);
		uint32_t state = DisableInterrupt();
		BtIntrfPkt_t *pPkt = (BtIntrfPkt_t *)CFifoPut(pIntrf->hTxFifo);
		if (pPkt != nullptr)
		{
			memcpy(pPkt->Data, pData, (size_t)len);
			pPkt->Len = (uint16_t)len;
		}
		EnableInterrupt(state);

		if (pPkt == nullptr)
		{
			pIntrf->TxDropCnt++;
			break;
		}

		DataLen -= len;
		pData += len;
		count += len;
	}

	BtIntrfNotify(pIntrf);
	return count;
}

void BtIntrfStopTx(DevIntrf_t *pDevIntrf)
{
	(void)pDevIntrf;
}

void BtIntrfReset(DevIntrf_t *pDevIntrf)
{
	if (pDevIntrf == nullptr || pDevIntrf->pDevData == nullptr)
	{
		return;
	}

	BtDevIntrf_t *pIntrf = (BtDevIntrf_t *)pDevIntrf->pDevData;
	uint32_t state = DisableInterrupt();
	CFifoFlush(pIntrf->hRxFifo);
	CFifoFlush(pIntrf->hTxFifo);
	pIntrf->TransBuffLen = 0;
	EnableInterrupt(state);
}

void *BtIntrfGetHandle(DevIntrf_t *pDevIntrf)
{
	return pDevIntrf != nullptr ? pDevIntrf->pDevData : nullptr;
}

void BtIntrfTxComplete(BtGattChar_t *pChar, int CharIdx)
{
	(void)CharIdx;
	if (pChar != nullptr && pChar->pSrvc != nullptr)
	{
		BtIntrfNotify((BtDevIntrf_t *)pChar->pSrvc->pContext);
	}
}

void BtIntrfRxWrCB(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{
	(void)Offset;
	if (pChar == nullptr || pChar->pSrvc == nullptr ||
		pChar->pSrvc->pContext == nullptr || pData == nullptr || Len <= 0)
	{
		return;
	}

	BtDevIntrf_t *pIntrf = (BtDevIntrf_t *)pChar->pSrvc->pContext;
	if (pIntrf->hRxFifo == nullptr)
	{
		return;
	}

	int payloadMax = pIntrf->PacketSize - (int)BTINTRF_PKHDR_LEN;
	while (Len > 0)
	{
		int len = min(payloadMax, Len);
		uint32_t state = DisableInterrupt();
		BtIntrfPkt_t *pPkt = (BtIntrfPkt_t *)CFifoPut(pIntrf->hRxFifo);
		if (pPkt != nullptr)
		{
			memcpy(pPkt->Data, pData, (size_t)len);
			pPkt->Len = (uint16_t)len;
		}
		EnableInterrupt(state);

		if (pPkt == nullptr)
		{
			pIntrf->RxDropCnt++;
			break;
		}

		Len -= len;
		pData += len;
	}

	if (CFifoUsed(pIntrf->hRxFifo) > 0 && pIntrf->DevIntrf.EvtCB != nullptr)
	{
		pIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,
			DEVINTRF_EVT_RX_DATA, nullptr, 0);
	}
}

bool BtIntrfInit(BtDevIntrf_t *pIntrf, const BtIntrfCfg_t *pCfg)
{
	if (pIntrf == nullptr || pCfg == nullptr || pCfg->pSrvc == nullptr ||
		pCfg->pSrvc->pCharArray == nullptr || pCfg->pSrvc->NbChar <= 0 ||
		pCfg->RxCharIdx < -1 || pCfg->RxCharIdx >= pCfg->pSrvc->NbChar ||
		pCfg->TxCharIdx < -1 || pCfg->TxCharIdx >= pCfg->pSrvc->NbChar)
	{
		return false;
	}

	int payloadSize = pCfg->PacketSize > 0 ? pCfg->PacketSize :
		BTINTRF_PACKET_SIZE;
	if (payloadSize <= 0 || payloadSize > BTINTRF_TRANSBUFF_MAXLEN)
	{
		return false;
	}
	if (pCfg->TxCharIdx >= 0 &&
		payloadSize > pCfg->pSrvc->pCharArray[pCfg->TxCharIdx].MaxDataLen)
	{
		return false;
	}

	uint32_t blockSize = (uint32_t)payloadSize + BTINTRF_PKHDR_LEN;
	uint8_t *pRxMem = pCfg->pRxFifoMem;
	int rxMemSize = pCfg->RxFifoMemSize;
	uint8_t *pTxMem = pCfg->pTxFifoMem;
	int txMemSize = pCfg->TxFifoMemSize;
	bool useDefaultRx = pRxMem == nullptr;
	bool useDefaultTx = pTxMem == nullptr;

	if (useDefaultRx)
	{
		if ((s_pDefaultRxOwner != nullptr && s_pDefaultRxOwner != pIntrf) ||
			blockSize > BTINTRF_PACKET_SIZE + BTINTRF_PKHDR_LEN)
		{
			return false;
		}
		pRxMem = s_BtDevIntrfRxFifoMem;
		rxMemSize = sizeof(s_BtDevIntrfRxFifoMem);
	}
	if (useDefaultTx)
	{
		if ((s_pDefaultTxOwner != nullptr && s_pDefaultTxOwner != pIntrf) ||
			blockSize > BTINTRF_PACKET_SIZE + BTINTRF_PKHDR_LEN)
		{
			return false;
		}
		pTxMem = s_BtDevIntrfTxFifoMem;
		txMemSize = sizeof(s_BtDevIntrfTxFifoMem);
	}

	if (!BtIntrfFifoMemValid(pRxMem, rxMemSize, blockSize) ||
		!BtIntrfFifoMemValid(pTxMem, txMemSize, blockSize))
	{
		return false;
	}

	hCFifo_t rxFifo = CFifoInit(pRxMem, (uint32_t)rxMemSize,
		blockSize, pCfg->bBlocking);
	hCFifo_t txFifo = CFifoInit(pTxMem, (uint32_t)txMemSize,
		blockSize, pCfg->bBlocking);
	if (rxFifo == nullptr || txFifo == nullptr)
	{
		return false;
	}

	if (useDefaultRx)
	{
		s_pDefaultRxOwner = pIntrf;
	}
	if (useDefaultTx)
	{
		s_pDefaultTxOwner = pIntrf;
	}

	pIntrf->DevIntrf.pDevData = pIntrf;
	pIntrf->pSrvc = pCfg->pSrvc;
	pIntrf->pSrvc->pContext = pIntrf;
	pIntrf->RxCharIdx = pCfg->RxCharIdx;
	pIntrf->TxCharIdx = pCfg->TxCharIdx;
	pIntrf->PacketSize = (int)blockSize;
	pIntrf->hRxFifo = rxFifo;
	pIntrf->hTxFifo = txFifo;

	if (pIntrf->RxCharIdx >= 0)
	{
		pIntrf->pSrvc->pCharArray[pIntrf->RxCharIdx].WrCB = BtIntrfRxWrCB;
	}
	if (pIntrf->TxCharIdx >= 0)
	{
		pIntrf->pSrvc->pCharArray[pIntrf->TxCharIdx].TxCompleteCB =
			BtIntrfTxComplete;
	}

	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_BT;
	pIntrf->DevIntrf.Enable = BtIntrfEnable;
	pIntrf->DevIntrf.Disable = BtIntrfDisable;
	pIntrf->DevIntrf.GetRate = BtIntrfGetRate;
	pIntrf->DevIntrf.SetRate = BtIntrfSetRate;
	pIntrf->DevIntrf.StartRx = BtIntrfStartRx;
	pIntrf->DevIntrf.RxData = BtIntrfRxData;
	pIntrf->DevIntrf.StopRx = BtIntrfStopRx;
	pIntrf->DevIntrf.StartTx = BtIntrfStartTx;
	pIntrf->DevIntrf.TxData = BtIntrfTxData;
	pIntrf->DevIntrf.TxSrData = BtIntrfTxData;
	pIntrf->DevIntrf.StopTx = BtIntrfStopTx;
	pIntrf->DevIntrf.Reset = BtIntrfReset;
	pIntrf->DevIntrf.PowerOff = BtIntrfDisable;
	pIntrf->DevIntrf.GetHandle = BtIntrfGetHandle;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->DevIntrf.bDma = false;
	pIntrf->DevIntrf.bIntEn = true;
	pIntrf->TransBuffLen = 0;
	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);
	atomic_store(&pIntrf->DevIntrf.EnCnt, 0);
	atomic_store(&pIntrf->DevIntrf.bTxReady, true);
	atomic_store(&pIntrf->DevIntrf.bNoStop, false);

	DeviceIntrfEnable(&pIntrf->DevIntrf);
	return true;
}

bool BtIntrf::Init(const BtIntrfCfg_t &Cfg)
{
	return BtIntrfInit(&vBtDevIntrf, &Cfg);
}

bool BtIntrf::RequestToSend(int NbBytes)
{
	if (NbBytes < 0 || vBtDevIntrf.hTxFifo == nullptr)
	{
		return false;
	}
	if (NbBytes == 0)
	{
		return true;
	}

	BtIntrfNotify(&vBtDevIntrf);
	int avail = CFifoAvail(vBtDevIntrf.hTxFifo);
	int payloadSize = vBtDevIntrf.PacketSize - (int)BTINTRF_PKHDR_LEN;
	return (int64_t)avail * payloadSize >= NbBytes;
}
