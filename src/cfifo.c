/**--------------------------------------------------------------------------
@file 	cfifo.c

@brief	Implementation of an overly simple circular FIFO buffer.

There is no queuing implementation and non blocking to be able to be use in
interrupt. User must ensure thread safety when used in a threaded environment.

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
#include <stdint.h>
#include <string.h>

#ifndef WIN32
#include <stdatomic.h>
#else
#include <signal.h>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

typedef sig_atomic_t		atomic_int;

static inline void atomic_store(atomic_int *pVar, sig_atomic_t NewVal) {
	InterlockedExchange(pVar, NewVal);
}

#endif

#include "cfifo.h"

HCFIFO const CFifoInit(uint8_t * const pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize, bool bBlocking)
{
	if (pMemBlk == NULL)
		return NULL;

	CFifo_t *hdr = (CFifo_t *)pMemBlk;
	hdr->bBlocking = bBlocking;
	hdr->DropCnt = 0;
	hdr->PutIdx = 0;
	hdr->GetIdx = -1;
	hdr->BlkSize = BlkSize;
	hdr->MemSize = TotalMemSize;
	hdr->MaxIdxCnt = (TotalMemSize - sizeof(CFifo_t)) / BlkSize;
	hdr->pMemStart = (uint8_t*)(pMemBlk + sizeof(CFifo_t));

	return hdr;
}

uint8_t *CFifoGet(HCFIFO const pFifo)
{
	if (pFifo == NULL || pFifo->GetIdx < 0)
		return NULL;

	int32_t idx = pFifo->GetIdx;
	int32_t getidx = idx + 1;

	if (getidx >= pFifo->MaxIdxCnt)
		getidx = 0;
	if (getidx == pFifo->PutIdx)
		getidx = -1;

	atomic_store((atomic_int *)&pFifo->GetIdx, getidx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoGetMultiple(HCFIFO const pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoGet(pFifo);

	if (pFifo == NULL || pFifo->GetIdx < 0 || *pCnt == 0)
	{
		*pCnt = 0;
		return NULL;
	}

	int32_t cnt = *pCnt;
	int32_t putidx = pFifo->PutIdx;
	int32_t idx = pFifo->GetIdx;
	int32_t getidx = idx + cnt;

	if (idx < putidx)
	{
		if (getidx >= putidx)
		{
			getidx = -1;
			cnt = putidx- idx;
		}
	}
	else
	{
		if (getidx >= pFifo->MaxIdxCnt)
		{
			getidx = putidx == 0 ? -1 : 0;
			cnt = pFifo->MaxIdxCnt - idx;
		}
	}

	atomic_store((atomic_int *)&pFifo->GetIdx, getidx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;
	*pCnt = cnt;

	return p;
}

uint8_t *CFifoPut(HCFIFO const pFifo)
{
	if (pFifo == NULL)
		return NULL;

    if (pFifo->PutIdx == pFifo->GetIdx)
    {
        if (pFifo->bBlocking == true)
            return NULL;
        // drop data
        int32_t gidx = pFifo->GetIdx + 1;
        if (gidx >= pFifo->MaxIdxCnt)
            gidx = 0;
        atomic_store((atomic_int *)&pFifo->GetIdx, gidx);

        pFifo->DropCnt++;
    }
	int32_t idx = pFifo->PutIdx;
	int32_t putidx = idx + 1;
	if (putidx >= pFifo->MaxIdxCnt)
		putidx = 0;
	atomic_store((atomic_int *)&pFifo->PutIdx, putidx);

	if (pFifo->GetIdx < 0)
	{
		atomic_store((atomic_int *)&pFifo->GetIdx, idx);
	}

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoPutMultiple(HCFIFO const pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoPut(pFifo);

	if (pFifo == NULL || *pCnt == 0)
	{
		*pCnt = 0;
		return NULL;
	}

	if (pFifo->PutIdx == pFifo->GetIdx)
    {
	    if (pFifo->bBlocking == true)
	        return NULL;
	    // Drop
	    int l = *pCnt;
	    CFifoGetMultiple(pFifo, &l);
        pFifo->DropCnt += l;
    }
	int32_t cnt = *pCnt;
	int32_t idx = pFifo->PutIdx;
	int32_t getidx = pFifo->GetIdx;
	int32_t putidx = idx + cnt;

	if (idx > getidx)
	{
		if (putidx >= pFifo->MaxIdxCnt)
		{
			cnt = pFifo->MaxIdxCnt - idx;
			putidx = 0;
		}
	}
	else
	{
		if (putidx >= getidx)
		{
			cnt = getidx - idx;
			putidx = getidx;
		}
	}

	atomic_store((atomic_int *)&pFifo->PutIdx, putidx);

	if (getidx < 0)
	{
		atomic_store((atomic_int *)&pFifo->GetIdx, idx);
	}

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	*pCnt = cnt;

	return p;
}

void CFifoFlush(HCFIFO const pFifo)
{
	atomic_store((atomic_int *)&pFifo->GetIdx, -1);

	atomic_store((atomic_int *)&pFifo->PutIdx, 0);
}

int CFifoAvail(HCFIFO const pFifo)
{
	int len = 0;

	if (pFifo->GetIdx < 0)
		return pFifo->MaxIdxCnt;

	if (pFifo->PutIdx > pFifo->GetIdx)
	{
		len = pFifo->MaxIdxCnt - pFifo->PutIdx + pFifo->GetIdx;
	}
	else if (pFifo->PutIdx < pFifo->GetIdx)
	{
		len = pFifo->GetIdx - pFifo->PutIdx;
	}

	return len;
}

int CFifoUsed(HCFIFO const pFifo)
{
	int len = 0;

	if (pFifo->GetIdx < 0)
		return 0;

	if (pFifo->GetIdx < pFifo->PutIdx)
	{
		len = pFifo->PutIdx - pFifo->GetIdx;
	}
	else
	{
		len = pFifo->MaxIdxCnt - pFifo->GetIdx + pFifo->PutIdx;
	}

	return len;
}

int CFifoRead(HCFIFO const pFifo, uint8_t *pBuff, int BuffLen)
{
	if (pFifo == NULL || pFifo->GetIdx < 0 || pBuff == NULL)
		return 0;

	int cnt = 0;

	if (BuffLen <= (int)pFifo->BlkSize)
	{
		// Single block
		uint8_t *p = CFifoGet(pFifo);
		if (p)
		{
			memcpy(pBuff, p, BuffLen);

			return BuffLen;
		}
	}
	else
	{
		// Span multiple blocks
		while (BuffLen > 0)
		{
			int l = BuffLen / pFifo->BlkSize;
			if ((BuffLen % pFifo->BlkSize) > 0)
				l++;
			uint8_t *p = CFifoGetMultiple(pFifo, &l);
			if (p == NULL)
				break;
			l *= pFifo->BlkSize;
			memcpy(pBuff, p, l);
			pBuff += l;
			BuffLen -= l;
			cnt += l;
		}
	}

	return cnt;
}

int CFifoWrite(HCFIFO const pFifo, uint8_t *pData, int DataLen)
{
	if (pFifo == NULL || pData == NULL)
		return 0;

    if (pFifo->PutIdx == pFifo->GetIdx)
    {
        if (pFifo->bBlocking == true)
            return 0;
        int l = DataLen;
        CFifoGetMultiple(pFifo, &l);
        pFifo->DropCnt += l;
    }

	int cnt = 0;

	if (DataLen <= (int)pFifo->BlkSize)
	{
		// Single block
		uint8_t *p = CFifoPut(pFifo);
		if (p)
		{
			memcpy(p, pData, DataLen);

			return DataLen;
		}
	}
	else
	{
		// Span multiple blocks
		while (DataLen > 0)
		{
			int l = DataLen / pFifo->BlkSize;
			if ((DataLen % pFifo->BlkSize) > 0)
				l++;
			uint8_t *p = CFifoPutMultiple(pFifo, &l);
			if (p == NULL)
				break;
			l *= pFifo->BlkSize;
			memcpy(p, pData, l);
			pData += l;
			DataLen -= l;
			cnt += l;
		}
	}

	return cnt;
}

