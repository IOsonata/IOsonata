/**--------------------------------------------------------------------------
@file 	cfifo.c

@brief	Implementation of an overly simple circular FIFO buffer.

There is no queuing implementation and non blocking to be able to be use in
interrupt. User must ensure thread safety when used in a threaded environment.

New Index scheme
------------
PutIdx and GetIdx are monotonic unsigned 32-bit counters. They never reset.

    empty   : PutIdx == GetIdx
    full    : (PutIdx - GetIdx) >= MaxIdxCnt
    used    : PutIdx - GetIdx
    avail   : MaxIdxCnt - used

@author Hoang Nguyen Hoan
@date 	Jan. 3, 2014

@license

MIT License

Copyright (c) 2014 I-SYST inc. All rights reserved.

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
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "cfifo.h"

#define CFIFO_ATOMIC_STORE(p, v, order) \
	__atomic_store_n((p), (v), (order))
#define CFIFO_ATOMIC_LOAD(p, order) \
	__atomic_load_n((p), (order))
#define CFIFO_ATOMIC_CAS_WEAK(p, expected, desired, success, failure) \
	__atomic_compare_exchange_n((p), (expected), (desired), true, \
		(success), (failure))
#define CFIFO_ATOMIC_FETCH_ADD(p, v, order) \
	__atomic_fetch_add((p), (v), (order))

static inline uint32_t cfifo_slot(const CFifo_t *pFifo, uint32_t Idx)
{
	return pFifo->Mask ? (Idx & pFifo->Mask) :
		(uint32_t)(Idx % (uint32_t)pFifo->MaxIdxCnt);
}

static inline uint8_t *cfifo_addr(const CFifo_t *pFifo, uint32_t Slot)
{
	return pFifo->pMemStart + Slot * pFifo->BlkSize;
}

hCFifo_t CFifoInit(uint8_t * const pMemBlk, uint32_t TotalMemSize,
		uint32_t BlkSize, bool bBlocking)
{
	if (pMemBlk == NULL || BlkSize == 0 ||
		TotalMemSize < sizeof(CFifo_t) + BlkSize)
	{
		return NULL;
	}

	uint32_t count = (TotalMemSize - sizeof(CFifo_t)) / BlkSize;
	if (count == 0 || count > INT32_MAX)
	{
		return NULL;
	}

	CFifo_t *pFifo = (CFifo_t *)pMemBlk;
	CFIFO_ATOMIC_STORE(&pFifo->PutIdx, 0U, __ATOMIC_RELAXED);
	CFIFO_ATOMIC_STORE(&pFifo->GetIdx, 0U, __ATOMIC_RELAXED);
	pFifo->BlkSize = BlkSize;
	pFifo->pMemStart = pMemBlk + sizeof(CFifo_t);
	pFifo->MaxIdxCnt = (int32_t)count;
	pFifo->bBlocking = bBlocking;
	CFIFO_ATOMIC_STORE(&pFifo->DropCnt, 0U, __ATOMIC_RELAXED);
	pFifo->Mask = (count & (count - 1U)) == 0U ? count - 1U : 0U;

	return pFifo;
}

uint8_t *CFifoGet(hCFifo_t const pFifo)
{
	if (pFifo == NULL)
	{
		return NULL;
	}

	uint32_t getIdx;
	uint32_t putIdx;
	do {
		getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_RELAXED);
		putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_ACQUIRE);
		if (getIdx == putIdx)
		{
			return NULL;
		}
	} while (!CFIFO_ATOMIC_CAS_WEAK(&pFifo->GetIdx,
		&getIdx, getIdx + 1U, __ATOMIC_RELEASE, __ATOMIC_RELAXED));

	return cfifo_addr(pFifo, cfifo_slot(pFifo, getIdx));
}

uint8_t *CFifoGetMultiple(hCFifo_t const pFifo, int *pCnt)
{
	if (pCnt == NULL)
	{
		return CFifoGet(pFifo);
	}
	if (pFifo == NULL || *pCnt <= 0)
	{
		*pCnt = 0;
		return NULL;
	}

	uint32_t max = (uint32_t)pFifo->MaxIdxCnt;
	uint32_t getIdx;
	uint32_t putIdx;
	uint32_t used;
	uint32_t count;

	do {
		getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_RELAXED);
		putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_ACQUIRE);
		used = putIdx - getIdx;
		if (used == 0U)
		{
			*pCnt = 0;
			return NULL;
		}

		count = (uint32_t)*pCnt;
		if (count > used)
		{
			count = used;
		}

		uint32_t slot = cfifo_slot(pFifo, getIdx);
		uint32_t contiguous = max - slot;
		if (count > contiguous)
		{
			count = contiguous;
		}
	} while (!CFIFO_ATOMIC_CAS_WEAK(&pFifo->GetIdx,
		&getIdx, getIdx + count, __ATOMIC_RELEASE, __ATOMIC_RELAXED));

	*pCnt = (int)count;
	return cfifo_addr(pFifo, cfifo_slot(pFifo, getIdx));
}

uint8_t *CFifoPut(hCFifo_t const pFifo)
{
	if (pFifo == NULL)
	{
		return NULL;
	}

	uint32_t max = (uint32_t)pFifo->MaxIdxCnt;
	uint32_t putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_RELAXED);
	uint32_t getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_ACQUIRE);
	uint32_t used = putIdx - getIdx;

	if (used >= max)
	{
		if (pFifo->bBlocking)
		{
			return NULL;
		}

		uint32_t oldGet = getIdx;
		while (!CFIFO_ATOMIC_CAS_WEAK(&pFifo->GetIdx,
			&oldGet, oldGet + 1U, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
		{
		}
		CFIFO_ATOMIC_FETCH_ADD(&pFifo->DropCnt, 1U, __ATOMIC_RELAXED);
	}

	uint32_t slot = cfifo_slot(pFifo, putIdx);
	CFIFO_ATOMIC_STORE(&pFifo->PutIdx, putIdx + 1U, __ATOMIC_RELEASE);
	return cfifo_addr(pFifo, slot);
}

uint8_t *CFifoPutMultiple(hCFifo_t const pFifo, int *pCnt)
{
	if (pCnt == NULL)
	{
		return CFifoPut(pFifo);
	}
	if (pFifo == NULL || *pCnt <= 0)
	{
		*pCnt = 0;
		return NULL;
	}

	uint32_t max = (uint32_t)pFifo->MaxIdxCnt;
	uint32_t putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_RELAXED);
	uint32_t getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_ACQUIRE);
	uint32_t used = putIdx - getIdx;
	uint32_t avail = used < max ? max - used : 0U;

	if (avail == 0U)
	{
		if (pFifo->bBlocking)
		{
			*pCnt = 0;
			return NULL;
		}

		int drop = *pCnt;
		if ((uint32_t)drop > max)
		{
			drop = (int)max;
		}
		CFifoGetMultiple(pFifo, &drop);
		CFIFO_ATOMIC_FETCH_ADD(&pFifo->DropCnt,
			(uint32_t)drop, __ATOMIC_RELAXED);
		getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_ACQUIRE);
		used = putIdx - getIdx;
		avail = used < max ? max - used : 0U;
	}

	uint32_t count = (uint32_t)*pCnt;
	if (count > avail)
	{
		count = avail;
	}

	uint32_t slot = cfifo_slot(pFifo, putIdx);
	uint32_t contiguous = max - slot;
	if (count > contiguous)
	{
		count = contiguous;
	}
	if (count == 0U)
	{
		*pCnt = 0;
		return NULL;
	}

	CFIFO_ATOMIC_STORE(&pFifo->PutIdx, putIdx + count, __ATOMIC_RELEASE);
	*pCnt = (int)count;
	return cfifo_addr(pFifo, slot);
}

void CFifoFlush(hCFifo_t const pFifo)
{
	if (pFifo == NULL)
	{
		return;
	}

	uint32_t putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_RELAXED);
	CFIFO_ATOMIC_STORE(&pFifo->GetIdx, putIdx, __ATOMIC_RELEASE);
}

int CFifoAvail(hCFifo_t const pFifo)
{
	if (pFifo == NULL)
	{
		return 0;
	}

	uint32_t putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_RELAXED);
	uint32_t getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_RELAXED);
	uint32_t used = putIdx - getIdx;
	uint32_t max = (uint32_t)pFifo->MaxIdxCnt;
	return used < max ? (int)(max - used) : 0;
}

int CFifoUsed(hCFifo_t const pFifo)
{
	if (pFifo == NULL)
	{
		return 0;
	}

	uint32_t putIdx = CFIFO_ATOMIC_LOAD(&pFifo->PutIdx, __ATOMIC_RELAXED);
	uint32_t getIdx = CFIFO_ATOMIC_LOAD(&pFifo->GetIdx, __ATOMIC_RELAXED);
	uint32_t used = putIdx - getIdx;
	uint32_t max = (uint32_t)pFifo->MaxIdxCnt;
	return (int)(used < max ? used : max);
}

int CFifoRead(hCFifo_t const pFifo, uint8_t *pBuff, int BuffLen)
{
	if (pFifo == NULL || pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	int count = 0;
	while (BuffLen > 0)
	{
		uint32_t blockSize = pFifo->BlkSize;
		int blocks = (int)(((uint32_t)BuffLen + blockSize - 1U) / blockSize);
		uint8_t *pData = CFifoGetMultiple(pFifo, &blocks);
		if (pData == NULL || blocks <= 0)
		{
			break;
		}

		uint32_t capacity = (uint32_t)blocks * blockSize;
		int bytes = BuffLen < (int)capacity ? BuffLen : (int)capacity;
		memcpy(pBuff, pData, (size_t)bytes);
		pBuff += bytes;
		BuffLen -= bytes;
		count += bytes;
	}

	return count;
}

int CFifoWrite(hCFifo_t const pFifo, uint8_t *pData, int DataLen)
{
	if (pFifo == NULL || pData == NULL || DataLen <= 0)
	{
		return 0;
	}

	int count = 0;
	while (DataLen > 0)
	{
		uint32_t blockSize = pFifo->BlkSize;
		int blocks = (int)(((uint32_t)DataLen + blockSize - 1U) / blockSize);
		uint8_t *pDest = CFifoPutMultiple(pFifo, &blocks);
		if (pDest == NULL || blocks <= 0)
		{
			break;
		}

		uint32_t capacity = (uint32_t)blocks * blockSize;
		int bytes = DataLen < (int)capacity ? DataLen : (int)capacity;
		memcpy(pDest, pData, (size_t)bytes);
		if ((uint32_t)bytes < capacity)
		{
			memset(pDest + bytes, 0, capacity - (uint32_t)bytes);
		}
		pData += bytes;
		DataLen -= bytes;
		count += bytes;
	}

	return count;
}
