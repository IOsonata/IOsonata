/**-------------------------------------------------------------------------
@file	bt_hci_ipc.cpp

@brief	Bluetooth HCI transport over IpcIntrf, one packet per slot

See bt_hci_ipc.h.

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

#include "bluetooth/bt_hci_ipc.h"

static inline BtHciIpcDev_t *BtHciIpcOwner(BtHciH4Dev_t * const pH4)
{
	return (BtHciIpcDev_t*)pH4->pCtx;
}

static void BtHciIpcH4Rx(BtHciH4Dev_t * const pH4, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	BtHciIpcDev_t *dev = BtHciIpcOwner(pH4);

	if (dev->RxHandler != nullptr)
	{
		dev->RxHandler(dev, Type, pPkt, Len);
	}
}

static void BtHciIpcH4CmdRsp(BtHciH4Dev_t * const pH4, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	BtHciIpcDev_t *dev = BtHciIpcOwner(pH4);

	if (dev->CmdRspHandler != nullptr)
	{
		dev->CmdRspHandler(dev, Type, pPkt, Len);
	}
}

static void BtHciIpcH4RxReady(BtHciH4Dev_t * const pH4)
{
	BtHciIpcDev_t *dev = BtHciIpcOwner(pH4);

	if (dev->RxReady != nullptr)
	{
		dev->RxReady(dev);
	}
}

// A slot holds one whole packet, so room is a free slot
static int BtHciIpcH4TxRoom(BtHciH4Dev_t * const pH4)
{
	return IpcIntrfTxRoom(&BtHciIpcOwner(pH4)->Ipc) > 0 ? (int)BT_HCI_H4_TXROOM_MIN : 0;
}

// IPC event handler, in the IPC interrupt. Only reports that packets arrived.
static int BtHciIpcEvtHandler(DevIntrf_t * const pIntrf, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pBuffer;
	(void)BufferLen;

	// Ipc is the first member of the transport and is the device data of the interface
	BtHciIpcDev_t *dev = (BtHciIpcDev_t *)pIntrf->pDevData;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		BtHciH4RxNotify(&dev->H4);
	}

	return 0;
}

bool BtHciIpcInit(BtHciIpcDev_t * const pDev, const BtHciIpcCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pMemBlk == nullptr)
	{
		return false;
	}

	if (pCfg->bOwner && (pCfg->NbPkt == 0 || pCfg->MemSize < BT_HCI_IPC_MEMSIZE(pCfg->NbPkt)))
	{
		return false;
	}

	memset((void*)pDev, 0, sizeof(BtHciIpcDev_t));

	pDev->RxHandler = pCfg->RxHandler;
	pDev->CmdRspHandler = pCfg->CmdRspHandler;
	pDev->RxReady = pCfg->RxReady;
	pDev->pCtx = pCfg->pCtx;

	BtHciH4Cfg_t h4cfg = {};

	h4cfg.pPhyIntrf = &pDev->Ipc.DevIntrf;
	h4cfg.pPktMem = pCfg->pPktMem;
	h4cfg.PktMemSize = pCfg->PktMemSize;
	h4cfg.RxHandler = BtHciIpcH4Rx;
	h4cfg.CmdRspHandler = BtHciIpcH4CmdRsp;
	h4cfg.RxReady = BtHciIpcH4RxReady;
	h4cfg.TxRoom = BtHciIpcH4TxRoom;
	h4cfg.bPktPhy = true;
	h4cfg.TimeoutMs = pCfg->TimeoutMs;
	h4cfg.pCtx = pDev;

	if (BtHciH4Init(&pDev->H4, &h4cfg) == false)
	{
		return false;
	}

	IpcIntrfCfg_t icfg = {};

	icfg.pMemBlk = pCfg->pMemBlk;
	icfg.MemSize = pCfg->MemSize;
	icfg.bOwner = pCfg->bOwner;
	icfg.TxNbBlk = pCfg->NbPkt;
	icfg.TxBlkSize = BT_HCI_IPC_BLKSIZE;
	icfg.RxNbBlk = pCfg->NbPkt;
	icfg.RxBlkSize = BT_HCI_IPC_BLKSIZE;
	icfg.TxChan = pCfg->TxChan;
	icfg.RxChan = pCfg->RxChan;
	icfg.IntPrio = pCfg->IntPrio;
	icfg.EvtCB = BtHciIpcEvtHandler;

	if (IpcIntrfInit(&pDev->Ipc, &icfg) == false)
	{
		return false;
	}

	// Reads are polled until the ring has nothing
	pDev->Ipc.DevIntrf.MaxRetry = 1;

	return true;
}

bool BtHciIpcIsUp(BtHciIpcDev_t * const pDev)
{
	return pDev != nullptr && IpcIntrfIsUp(&pDev->Ipc);
}

bool BtHciIpcSend(BtHciIpcDev_t * const pDev, uint8_t Type, const void *pData, uint32_t Len)
{
	return pDev != nullptr && BtHciH4Send(&pDev->H4, Type, pData, Len);
}

void BtHciIpcFlush(BtHciIpcDev_t * const pDev)
{
	if (pDev != nullptr)
	{
		BtHciH4Flush(&pDev->H4);
	}
}

void BtHciIpcPoll(BtHciIpcDev_t * const pDev)
{
	if (pDev != nullptr)
	{
		BtHciH4Poll(&pDev->H4);
	}
}

bool BtHciIpcProcess(BtHciIpcDev_t * const pDev, int MaxPkt)
{
	return pDev != nullptr && BtHciH4Process(&pDev->H4, MaxPkt);
}

uint8_t BtHciIpcCommand(BtHciIpcDev_t * const pDev, BtHciDevice_t * const pHci, uint16_t OpCode,
						const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (pDev == nullptr)
	{
		return BT_HCI_ERR_INVALID_COMMAND_PARAM;
	}

	return BtHciH4Command(&pDev->H4, pHci, OpCode, pParam, ParamLen, pRet, RetLen);
}

// ---- Packet DevIntrf face ------------------------------------------------

static inline BtHciIpcIntrfDev_t *BtHciIpcIntrfOwner(DevIntrf_t * const pIntrf)
{
	return (BtHciIpcIntrfDev_t*)pIntrf->pDevData;
}

static int BtHciIpcIntrfEvt(DevIntrf_t * const pIpcIntrf, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	// Ipc is the first member and the device data of the IPC interface
	BtHciIpcIntrfDev_t *dev = (BtHciIpcIntrfDev_t *)pIpcIntrf->pDevData;

	if (dev->EvtCB != nullptr)
	{
		return dev->EvtCB(&dev->DevIntrf, EvtId, pBuffer, BufferLen);
	}

	return 0;
}

// Read the next slot into Held unless one is waiting. A slot that is not a
// whole packet is dropped and the next one is read.
static bool BtHciIpcIntrfFill(BtHciIpcIntrfDev_t * const pDev)
{
	while (pDev->HeldLen == 0)
	{
		int l = DeviceIntrfRx(&pDev->Ipc.DevIntrf, 0, pDev->Held, (int)sizeof(pDev->Held));

		if (l <= 0)
		{
			return false;
		}

		if (BtHciH4PktCheck(pDev->Held[0], &pDev->Held[1], (uint32_t)(l - 1), BT_HCI_IPC_PKT_MAXLEN))
		{
			pDev->HeldLen = (uint16_t)l;
		}
		else
		{
			pDev->SyncErrCnt++;
		}
	}

	return true;
}

static bool BtHciIpcIntrfStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	BtHciIpcIntrfDev_t *dev = BtHciIpcIntrfOwner(pIntrf);

	return BtHciIpcIntrfFill(dev) && dev->Held[0] == (uint8_t)DevAddr;
}

static int BtHciIpcIntrfRxData(DevIntrf_t * const pIntrf, uint8_t *pBuffer, int BufferLen)
{
	BtHciIpcIntrfDev_t *dev = BtHciIpcIntrfOwner(pIntrf);

	if (dev->HeldLen == 0 || pBuffer == nullptr)
	{
		return 0;
	}

	int l = (int)dev->HeldLen - 1;

	dev->HeldLen = 0;

	if (l > BufferLen)
	{
		// Left in place it would block everything behind it for good
		dev->RxDropCnt++;

		return 0;
	}

	memcpy(pBuffer, &dev->Held[1], (size_t)l);

	return l;
}

static void BtHciIpcIntrfStopRx(DevIntrf_t * const)
{
}

static bool BtHciIpcIntrfStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	BtHciIpcIntrfDev_t *dev = BtHciIpcIntrfOwner(pIntrf);

	if (DevAddr < BT_HCI_IPC_PKT_CMD || DevAddr > BT_HCI_IPC_PKT_ISO || IpcIntrfTxRoom(&dev->Ipc) <= 0)
	{
		return false;
	}

	dev->TxType = (uint8_t)DevAddr;

	return true;
}

static int BtHciIpcIntrfTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	BtHciIpcIntrfDev_t *dev = BtHciIpcIntrfOwner(pIntrf);

	if (pData == nullptr || DataLen <= 0 || DataLen > (int)BT_HCI_IPC_PKT_MAXLEN)
	{
		return 0;
	}

	uint8_t pkt[1 + BT_HCI_IPC_PKT_MAXLEN];

	pkt[0] = dev->TxType;
	memcpy(&pkt[1], pData, (size_t)DataLen);

	return DeviceIntrfTx(&dev->Ipc.DevIntrf, 0, pkt, DataLen + 1) == DataLen + 1 ? DataLen : 0;
}

static void BtHciIpcIntrfStopTx(DevIntrf_t * const)
{
}

static void BtHciIpcIntrfDisable(DevIntrf_t * const pIntrf)
{
	DeviceIntrfDisable(&BtHciIpcIntrfOwner(pIntrf)->Ipc.DevIntrf);
}

static void BtHciIpcIntrfEnable(DevIntrf_t * const pIntrf)
{
	DeviceIntrfEnable(&BtHciIpcIntrfOwner(pIntrf)->Ipc.DevIntrf);
}

static uint32_t BtHciIpcIntrfRate(DevIntrf_t * const)
{
	return 0;
}

static uint32_t BtHciIpcIntrfSetRate(DevIntrf_t * const, uint32_t)
{
	return 0;
}

static void BtHciIpcIntrfReset(DevIntrf_t * const pIntrf)
{
	BtHciIpcIntrfDev_t *dev = BtHciIpcIntrfOwner(pIntrf);

	dev->HeldLen = 0;
}

static void BtHciIpcIntrfPowerOff(DevIntrf_t * const)
{
}

static void *BtHciIpcIntrfHandle(DevIntrf_t * const pIntrf)
{
	return BtHciIpcIntrfOwner(pIntrf);
}

bool BtHciIpcIntrfInit(BtHciIpcIntrfDev_t * const pDev, const BtHciIpcIntrfCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pMemBlk == nullptr)
	{
		return false;
	}

	if (pCfg->bOwner && (pCfg->NbPkt == 0 || pCfg->MemSize < BT_HCI_IPC_MEMSIZE(pCfg->NbPkt)))
	{
		return false;
	}

	memset((void*)pDev, 0, sizeof(BtHciIpcIntrfDev_t));

	pDev->EvtCB = pCfg->EvtCB;

	IpcIntrfCfg_t icfg = {};

	icfg.pMemBlk = pCfg->pMemBlk;
	icfg.MemSize = pCfg->MemSize;
	icfg.bOwner = pCfg->bOwner;
	icfg.TxNbBlk = pCfg->NbPkt;
	icfg.TxBlkSize = BT_HCI_IPC_BLKSIZE;
	icfg.RxNbBlk = pCfg->NbPkt;
	icfg.RxBlkSize = BT_HCI_IPC_BLKSIZE;
	icfg.TxChan = pCfg->TxChan;
	icfg.RxChan = pCfg->RxChan;
	icfg.IntPrio = pCfg->IntPrio;
	icfg.EvtCB = BtHciIpcIntrfEvt;

	if (IpcIntrfInit(&pDev->Ipc, &icfg) == false)
	{
		return false;
	}

	pDev->Ipc.DevIntrf.MaxRetry = 1;

	DevIntrf_t *d = &pDev->DevIntrf;

	d->pDevData = pDev;
	d->IntPrio = pCfg->IntPrio;
	d->EvtCB = pCfg->EvtCB;
	d->Type = DEVINTRF_TYPE_BT;
	d->bDma = false;
	d->bIntEn = true;
	d->MaxRetry = 1;
	atomic_flag_clear(&d->bBusy);
	atomic_store(&d->EnCnt, 0);
	atomic_store(&d->bTxReady, true);
	atomic_store(&d->bNoStop, false);
	d->Disable = BtHciIpcIntrfDisable;
	d->Enable = BtHciIpcIntrfEnable;
	d->GetRate = BtHciIpcIntrfRate;
	d->SetRate = BtHciIpcIntrfSetRate;
	d->StartRx = BtHciIpcIntrfStartRx;
	d->RxData = BtHciIpcIntrfRxData;
	d->StopRx = BtHciIpcIntrfStopRx;
	d->StartTx = BtHciIpcIntrfStartTx;
	d->TxData = BtHciIpcIntrfTxData;
	d->TxSrData = BtHciIpcIntrfTxData;
	d->StopTx = BtHciIpcIntrfStopTx;
	d->Reset = BtHciIpcIntrfReset;
	d->PowerOff = BtHciIpcIntrfPowerOff;
	d->GetHandle = BtHciIpcIntrfHandle;

	return true;
}

bool BtHciIpcIntrfIsUp(BtHciIpcIntrfDev_t * const pDev)
{
	return pDev != nullptr && IpcIntrfIsUp(&pDev->Ipc);
}
