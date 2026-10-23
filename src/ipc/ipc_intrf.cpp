/**-------------------------------------------------------------------------
@file	ipc_intrf.cpp

@brief	Inter-core DeviceIntrf over shared RAM, generic implementation.

See ipc_intrf.h for the design. This file has no hardware access: the signal
between the cores goes through the port seam IpcCtrlrInit, IpcCtrlrSend and
IpcCtrlrRxEnable.

Memory order. A producer publishes the bytes with a release store of the ring
put index, and only then raises the channel. The consumer takes the channel
signal, then reads the put index with acquire, so the bytes are there. A
consumer releases bytes with a release store of the get index. The wait flag
and the ring indices are linked by a full barrier on both sides: the producer
sets its wait flag and then looks at the room, the consumer frees room and
then looks at the wait flag, so one of them always sees the other.

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
#include <string.h>

#include "ipc/ipc_intrf.h"

#define IPC_INTRF_HDR_SIZE		((uint32_t)sizeof(IpcIntrfHdr_t))

static inline bool IpcIntrfIsPkt(uint32_t Blk)
{
	return Blk > 1U;
}

// Valid block size of a ring: byte stream, or a slot with room for a payload
static inline bool IpcIntrfBlkValid(uint32_t BlkSize)
{
	return BlkSize <= 1U || (BlkSize > IPC_INTRF_PKT_HDR_SIZE && BlkSize <= IPC_INTRF_PKT_BLK_MAX);
}

// Memory of a ring, word rounded, 64 bit so a bad count cannot wrap
static inline uint64_t IpcIntrfRingMem(uint32_t NbBlk, uint32_t Blk)
{
	return (static_cast<uint64_t>(NbBlk) * Blk + sizeof(CFifo_t) + 3U) & ~3ULL;
}

static inline __attribute__((always_inline))
void IpcIntrfBarrier(void)
{
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
}

// bUp and Epoch are touched from the interrupt and from the transfer
// functions of the same core, with the same values.
static inline __attribute__((always_inline))
bool IpcIntrfGetUp(const IpcIntrfDev_t *pDev)
{
	return __atomic_load_n(&pDev->bUp, __ATOMIC_ACQUIRE);
}

static inline __attribute__((always_inline))
void IpcIntrfSetUp(IpcIntrfDev_t *pDev, bool bUp)
{
	__atomic_store_n(&pDev->bUp, bUp, __ATOMIC_RELEASE);
}

// Buffer-less event, same shape as the other interfaces: Len is the count.
static void IpcIntrfNotify(IpcIntrfDev_t *pDev, DEVINTRF_EVT Event, int Length)
{
	if (pDev->DevIntrf.EvtCB != nullptr)
	{
		pDev->DevIntrf.EvtCB(&pDev->DevIntrf, Event, nullptr, Length);
	}
}

// Owner side: partition the block into device data and two rings, format the
// rings, then publish the device data. The magic goes down first and up last,
// so the peer never sees a half formatted block.
static bool IpcIntrfFormat(IpcIntrfDev_t *pDev)
{
	IpcIntrfHdr_t *pHdr = pDev->pHdr;
	const IpcIntrfCfg_t *pCfg = &pDev->Cfg;
	uint8_t *pBase = reinterpret_cast<uint8_t *>(pHdr);
	uint32_t txBlk = IPC_INTRF_BLK(pCfg->TxBlkSize);
	uint32_t rxBlk = IPC_INTRF_BLK(pCfg->RxBlkSize);
	uint32_t txOff = IPC_INTRF_HDR_SIZE;
	uint32_t rxOff = txOff + static_cast<uint32_t>(IpcIntrfRingMem(pCfg->TxNbBlk, txBlk));

	IpcIntrfSetUp(pDev, false);
	__atomic_store_n(&pHdr->Magic, 0U, __ATOMIC_RELEASE);

	// The exact byte count is given to CFifoInit, so the ring holds NbBlk
	// blocks and not what the word rounding would add.
	pDev->hTxFifo = CFifoInit(pBase + txOff, static_cast<uint32_t>(CFIFO_TOTAL_MEMSIZE(pCfg->TxNbBlk, txBlk)), txBlk, true);
	pDev->hRxFifo = CFifoInit(pBase + rxOff, static_cast<uint32_t>(CFIFO_TOTAL_MEMSIZE(pCfg->RxNbBlk, rxBlk)), rxBlk, true);
	if (pDev->hTxFifo == nullptr || pDev->hRxFifo == nullptr)
	{
		return false;
	}

	uint32_t epoch = __atomic_load_n(&pHdr->Epoch, __ATOMIC_RELAXED) + 1U;

	pHdr->Version = IPC_INTRF_VERSION;
	pHdr->Wait[0] = 0U;
	pHdr->Wait[1] = 0U;
	pHdr->Ring[0].Off = txOff;
	pHdr->Ring[0].NbBlk = pCfg->TxNbBlk;
	pHdr->Ring[0].BlkSize = txBlk;
	pHdr->Ring[1].Off = rxOff;
	pHdr->Ring[1].NbBlk = pCfg->RxNbBlk;
	pHdr->Ring[1].BlkSize = rxBlk;
	__atomic_store_n(&pHdr->Epoch, epoch, __ATOMIC_RELAXED);
	__atomic_store_n(&pHdr->Magic, IPC_INTRF_MAGIC, __ATOMIC_RELEASE);

	pDev->TxBlk = txBlk;
	pDev->RxBlk = rxBlk;
	__atomic_store_n(&pDev->Epoch, epoch, __ATOMIC_RELAXED);
	IpcIntrfSetUp(pDev, true);

	// A peer that is already running learns about the new block.
	IpcCtrlrSend(pDev->TxChan);

	return true;
}

// Check one ring written by the owner against the block of this core.
static bool IpcIntrfRingFits(const IpcIntrfRing_t *pRing, uint32_t MemSize)
{
	if (!IpcIntrfBlkValid(pRing->BlkSize) || pRing->BlkSize == 0U || pRing->NbBlk == 0U ||
		(pRing->Off & 3U) != 0U || pRing->Off < IPC_INTRF_HDR_SIZE)
	{
		return false;
	}

	return pRing->Off + IpcIntrfRingMem(pRing->NbBlk, pRing->BlkSize) <= MemSize;
}

// Peer side: read the geometry the owner wrote and take the two rings where it
// put them. The ring the owner writes is this core's Rx. The link is up when
// the owner has formatted the block and the geometry fits the block of this
// core. The owner formatting again changes the epoch, and the geometry is read
// again; the link is down until the new device data is valid.
static bool IpcIntrfPeerUp(IpcIntrfDev_t *pDev)
{
	IpcIntrfHdr_t *pHdr = pDev->pHdr;

	if (__atomic_load_n(&pHdr->Magic, __ATOMIC_ACQUIRE) != IPC_INTRF_MAGIC)
	{
		IpcIntrfSetUp(pDev, false);
		return false;
	}

	uint32_t epoch = __atomic_load_n(&pHdr->Epoch, __ATOMIC_RELAXED);

	if (IpcIntrfGetUp(pDev) && epoch == __atomic_load_n(&pDev->Epoch, __ATOMIC_RELAXED))
	{
		return true;
	}

	if (pHdr->Version != IPC_INTRF_VERSION ||
		!IpcIntrfRingFits(&pHdr->Ring[0], pDev->MemSize) ||
		!IpcIntrfRingFits(&pHdr->Ring[1], pDev->MemSize))
	{
		IpcIntrfSetUp(pDev, false);
		return false;
	}

	uint8_t *pBase = reinterpret_cast<uint8_t *>(pHdr);

	pDev->hRxFifo = reinterpret_cast<hCFifo_t>(pBase + pHdr->Ring[0].Off);
	pDev->hTxFifo = reinterpret_cast<hCFifo_t>(pBase + pHdr->Ring[1].Off);
	pDev->RxBlk = pHdr->Ring[0].BlkSize;
	pDev->TxBlk = pHdr->Ring[1].BlkSize;

	__atomic_store_n(&pDev->Epoch, epoch, __ATOMIC_RELAXED);
	IpcIntrfSetUp(pDev, true);

	return true;
}

bool IpcIntrfIsUp(IpcIntrfDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}

	if (pDev->bOwner)
	{
		return IpcIntrfGetUp(pDev);
	}

	return IpcIntrfPeerUp(pDev);
}

static void IpcIntrfNoop(DevIntrf_t * const)
{
}

static void IpcIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	IpcCtrlrRxEnable(static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData), false);
}

static void IpcIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	IpcCtrlrRxEnable(static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData), true);
}

// No transfer rate: the cores move data as fast as they run.
static uint32_t IpcIntrfGetRate(DevIntrf_t * const)
{
	return 0;
}

static uint32_t IpcIntrfSetRate(DevIntrf_t * const, uint32_t)
{
	return 0;
}

// The framework busy flag is taken and released around the transfer by the
// callers of these hooks; there is nothing else to open or close.
static bool IpcIntrfStart(DevIntrf_t * const, uint32_t)
{
	return true;
}

// Reserve, fill, then publish, so the other core never reads a byte that is
// not written yet. A short write means the ring is full: the wait flag asks
// the consumer for a signal when it frees room, and the room is looked at
// again after the flag is set so a free that happened in between is not lost.
static int IpcIntrfTxBytes(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen)
{
	IpcIntrfDev_t *pDev = static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData);

	if (DataLen <= 0 || pData == nullptr || !IpcIntrfIsUp(pDev))
	{
		return 0;
	}

	uint32_t *pWait = &pDev->pHdr->Wait[pDev->Me];
	int cnt = 0;
	bool bWait = false;

	for (;;)
	{
		while (DataLen > 0)
		{
			int n = DataLen;
			uint8_t *p = CFifoResvMultiple(pDev->hTxFifo, &n);

			if (p == nullptr || n <= 0)
			{
				break;
			}

			memcpy(p, pData, static_cast<size_t>(n));
			CFifoPutMultiple(pDev->hTxFifo, &n);

			pData += n;
			DataLen -= n;
			cnt += n;
		}

		if (DataLen == 0)
		{
			break;
		}

		// Not ready before the wait flag is up, so a ready store of the
		// interrupt that follows is never overwritten by this one.
		atomic_store_explicit(&pDevIntrf->bTxReady, false, memory_order_release);
		__atomic_store_n(pWait, 1U, __ATOMIC_RELEASE);
		bWait = true;
		IpcIntrfBarrier();

		if (CFifoAvail(pDev->hTxFifo) == 0)
		{
			break;
		}
	}

	if (DataLen == 0 && bWait)
	{
		__atomic_store_n(pWait, 0U, __ATOMIC_RELEASE);
		atomic_store_explicit(&pDevIntrf->bTxReady, true, memory_order_release);
	}

	if (cnt > 0)
	{
		IpcCtrlrSend(pDev->TxChan);
	}

	return cnt;
}

// Arm the room request. Not ready before the flag is up, so a ready store of
// the interrupt that follows is never overwritten by this one. The caller
// looks at the room again afterwards, so a free in between is not lost.
static void IpcIntrfWaitArm(IpcIntrfDev_t *pDev)
{
	atomic_store_explicit(&pDev->DevIntrf.bTxReady, false, memory_order_release);
	__atomic_store_n(&pDev->pHdr->Wait[pDev->Me], 1U, __ATOMIC_RELEASE);
	IpcIntrfBarrier();
}

// Packet stream write: one slot, length first, taken whole or not at all.
static int IpcIntrfTxPacket(IpcIntrfDev_t *pDev, const uint8_t *pData, int DataLen)
{
	if (static_cast<uint32_t>(DataLen) > IPC_INTRF_PKT_PAYLOAD(pDev->TxBlk))
	{
		pDev->TxOversizeCnt++;

		return 0;
	}

	uint8_t *p = CFifoResv(pDev->hTxFifo);

	if (p == nullptr)
	{
		IpcIntrfWaitArm(pDev);
		p = CFifoResv(pDev->hTxFifo);

		if (p == nullptr)
		{
			return 0;
		}
	}

	p[0] = static_cast<uint8_t>(DataLen);
	p[1] = static_cast<uint8_t>(DataLen >> 8);
	memcpy(p + IPC_INTRF_PKT_HDR_SIZE, pData, static_cast<size_t>(DataLen));
	CFifoPut(pDev->hTxFifo);

	// Room was found: a flag set above is stale, take it back. A consumer
	// that saw it meanwhile only raised one extra TX_READY.
	if (__atomic_load_n(&pDev->pHdr->Wait[pDev->Me], __ATOMIC_RELAXED) != 0U)
	{
		__atomic_store_n(&pDev->pHdr->Wait[pDev->Me], 0U, __ATOMIC_RELEASE);
		atomic_store_explicit(&pDev->DevIntrf.bTxReady, true, memory_order_release);
	}

	IpcCtrlrSend(pDev->TxChan);

	return DataLen;
}

static int IpcIntrfTxData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen)
{
	IpcIntrfDev_t *pDev = static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData);

	if (DataLen <= 0 || pData == nullptr || !IpcIntrfIsUp(pDev))
	{
		return 0;
	}

	if (IpcIntrfIsPkt(pDev->TxBlk))
	{
		return IpcIntrfTxPacket(pDev, pData, DataLen);
	}

	return IpcIntrfTxBytes(pDevIntrf, pData, DataLen);
}

// Peek, copy, then release, so the producer cannot reuse a byte that is
// still being read. Room that came back is signalled only to a producer that
// said it was waiting.
static int IpcIntrfRxBytes(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	IpcIntrfDev_t *pDev = static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData);

	if (BuffLen <= 0 || pBuff == nullptr || !IpcIntrfIsUp(pDev))
	{
		return 0;
	}

	int cnt = 0;

	while (BuffLen > 0)
	{
		int n = BuffLen;
		uint8_t *p = CFifoPeekMultiple(pDev->hRxFifo, &n);

		if (p == nullptr || n <= 0)
		{
			break;
		}

		memcpy(pBuff, p, static_cast<size_t>(n));
		CFifoRelease(pDev->hRxFifo, n);

		pBuff += n;
		BuffLen -= n;
		cnt += n;
	}

	if (cnt > 0)
	{
		IpcIntrfBarrier();

		if (__atomic_load_n(&pDev->pHdr->Wait[1U - pDev->Me], __ATOMIC_ACQUIRE) != 0U)
		{
			IpcCtrlrSend(pDev->TxChan);
		}
	}

	return cnt;
}

// Room that came back is signalled only to a producer that said it was
// waiting.
static void IpcIntrfRoomBack(IpcIntrfDev_t *pDev)
{
	IpcIntrfBarrier();

	if (__atomic_load_n(&pDev->pHdr->Wait[1U - pDev->Me], __ATOMIC_ACQUIRE) != 0U)
	{
		IpcCtrlrSend(pDev->TxChan);
	}
}

// Packet stream read: one slot per call. A slot that does not fit the buffer
// or has a bad length is released, so it cannot block the ring.
static int IpcIntrfRxPacket(IpcIntrfDev_t *pDev, uint8_t *pBuff, int BuffLen)
{
	uint8_t *p = CFifoPeek(pDev->hRxFifo);

	if (p == nullptr)
	{
		return 0;
	}

	uint32_t len = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8);
	int cnt = 0;

	if (len <= IPC_INTRF_PKT_PAYLOAD(pDev->RxBlk) && len <= static_cast<uint32_t>(BuffLen))
	{
		memcpy(pBuff, p + IPC_INTRF_PKT_HDR_SIZE, len);
		cnt = static_cast<int>(len);
	}
	else
	{
		pDev->RxDropCnt++;
	}

	CFifoRelease(pDev->hRxFifo, 1);
	IpcIntrfRoomBack(pDev);

	return cnt;
}

static int IpcIntrfRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	IpcIntrfDev_t *pDev = static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData);

	if (BuffLen <= 0 || pBuff == nullptr || !IpcIntrfIsUp(pDev))
	{
		return 0;
	}

	if (IpcIntrfIsPkt(pDev->RxBlk))
	{
		return IpcIntrfRxPacket(pDev, pBuff, BuffLen);
	}

	return IpcIntrfRxBytes(pDevIntrf, pBuff, BuffLen);
}

// The owner formats again and the peer follows through the epoch. Nothing to
// do on the peer: it owns no state of the link.
static void IpcIntrfReset(DevIntrf_t * const pDevIntrf)
{
	IpcIntrfDev_t *pDev = static_cast<IpcIntrfDev_t *>(pDevIntrf->pDevData);

	if (pDev->bOwner)
	{
		(void)IpcIntrfFormat(pDev);
	}
}

static void *IpcIntrfGetHandle(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf->pDevData;
}

int IpcIntrfTxRoom(IpcIntrfDev_t * const pDev)
{
	if (pDev == nullptr || !IpcIntrfIsUp(pDev))
	{
		return 0;
	}

	return CFifoAvail(pDev->hTxFifo);
}

void IpcIntrfIrqHandler(IpcIntrfDev_t * const pDev)
{
	if (pDev == nullptr || !IpcIntrfIsUp(pDev))
	{
		return;
	}

	int used = CFifoUsed(pDev->hRxFifo);

	if (used > 0)
	{
		IpcIntrfNotify(pDev, DEVINTRF_EVT_RX_DATA, used);
	}

	uint32_t *pWait = &pDev->pHdr->Wait[pDev->Me];

	if (__atomic_load_n(pWait, __ATOMIC_ACQUIRE) != 0U && CFifoAvail(pDev->hTxFifo) > 0)
	{
		__atomic_store_n(pWait, 0U, __ATOMIC_RELEASE);
		atomic_store_explicit(&pDev->DevIntrf.bTxReady, true, memory_order_release);
		IpcIntrfNotify(pDev, DEVINTRF_EVT_TX_READY, 0);
	}
}

bool IpcIntrfInit(IpcIntrfDev_t * const pDev, const IpcIntrfCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pMemBlk == nullptr ||
		(reinterpret_cast<uintptr_t>(pCfg->pMemBlk) & 3U) != 0U ||
		pCfg->MemSize < IPC_INTRF_HDR_SIZE)
	{
		return false;
	}

	if (pCfg->bOwner)
	{
		if (!IpcIntrfBlkValid(pCfg->TxBlkSize) || !IpcIntrfBlkValid(pCfg->RxBlkSize) ||
			pCfg->TxNbBlk == 0U || pCfg->RxNbBlk == 0U)
		{
			return false;
		}

		uint64_t need = IPC_INTRF_HDR_SIZE +
						IpcIntrfRingMem(pCfg->TxNbBlk, IPC_INTRF_BLK(pCfg->TxBlkSize)) +
						IpcIntrfRingMem(pCfg->RxNbBlk, IPC_INTRF_BLK(pCfg->RxBlkSize));

		if (pCfg->MemSize < need)
		{
			return false;
		}
	}

	memset(static_cast<void *>(pDev), 0, sizeof(*pDev));

	pDev->Cfg = *pCfg;
	pDev->pHdr = reinterpret_cast<IpcIntrfHdr_t *>(pCfg->pMemBlk);
	pDev->MemSize = pCfg->MemSize;
	pDev->TxBlk = 1U;
	pDev->RxBlk = 1U;
	pDev->TxChan = pCfg->TxChan;
	pDev->RxChan = pCfg->RxChan;
	pDev->bOwner = pCfg->bOwner;
	pDev->Me = pCfg->bOwner ? 0U : 1U;

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.IntPrio = pCfg->IntPrio;
	pDev->DevIntrf.EvtCB = pCfg->EvtCB;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = true;
	pDev->DevIntrf.Disable = IpcIntrfDisable;
	pDev->DevIntrf.Enable = IpcIntrfEnable;
	pDev->DevIntrf.GetRate = IpcIntrfGetRate;
	pDev->DevIntrf.SetRate = IpcIntrfSetRate;
	pDev->DevIntrf.StartRx = IpcIntrfStart;
	pDev->DevIntrf.RxData = IpcIntrfRxData;
	pDev->DevIntrf.StopRx = IpcIntrfNoop;
	pDev->DevIntrf.StartTx = IpcIntrfStart;
	pDev->DevIntrf.TxData = IpcIntrfTxData;
	pDev->DevIntrf.TxSrData = IpcIntrfTxData;
	pDev->DevIntrf.StopTx = IpcIntrfNoop;
	pDev->DevIntrf.Reset = IpcIntrfReset;
	pDev->DevIntrf.PowerOff = IpcIntrfNoop;
	pDev->DevIntrf.GetHandle = IpcIntrfGetHandle;

	atomic_store_explicit(&pDev->DevIntrf.bTxReady, true, memory_order_relaxed);

	if (!IpcCtrlrInit(pDev))
	{
		return false;
	}

	if (pCfg->bOwner)
	{
		return IpcIntrfFormat(pDev);
	}

	return true;
}
