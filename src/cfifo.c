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

#include "cfifo.h"

/**
 * @brief	Initialize FIFO.
 *
 * This function must be called first to initialize FIFO before any other functions
 * can be used.
 *
 * @param	pMemBlk 		: Pointer to memory block to be used for FIFO
 * @param	TotalMemSize	: Total memory size in byte
 * @param	BlkSize 		: Block size in bytes
 * @param   bBlocking  		: Behavior when FIFO is full.\n
 *                    			false - Old data will be pushed out to make place
 *                            			for new data. Put always succeed\n
 *                    			true  - New data will not be pushed in. Put will
 *                    					return fail.
 *
 * 	@return CFifo Handle
 */
hCFifo_t CFifoInit(uint8_t * const pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize, bool bBlocking)
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

uint8_t *CFifoGet(hCFifo_t const pFifo)
{
	if (pFifo == NULL || atomic_load(&pFifo->GetIdx) < 0)
		return NULL;

	int32_t idx = atomic_load(&pFifo->GetIdx);
	int32_t getidx = idx + 1;

	if (getidx >= pFifo->MaxIdxCnt)
		getidx = 0;

	int32_t putidx = atomic_load(&pFifo->PutIdx);
	if (getidx == putidx)
		getidx = -1;

	atomic_store(&pFifo->GetIdx, getidx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoGetMultiple(hCFifo_t const pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoGet(pFifo);

	if (pFifo == NULL || atomic_load(&pFifo->GetIdx) < 0 || *pCnt == 0)
	{
		*pCnt = 0;
		return NULL;
	}

	int32_t cnt = *pCnt;
	int32_t putidx = atomic_load(&pFifo->PutIdx);
	int32_t idx    = atomic_load(&pFifo->GetIdx);
	int32_t getidx = idx + cnt;

	if (idx < putidx)
	{
		if (getidx >= putidx)
		{
			getidx = -1;
			cnt = putidx - idx;
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

	atomic_store(&pFifo->GetIdx, getidx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;
	*pCnt = cnt;

	return p;
}

uint8_t *CFifoPut(hCFifo_t const pFifo)
{
	if (pFifo == NULL)
		return NULL;

    if (atomic_load(&pFifo->PutIdx) == atomic_load(&pFifo->GetIdx))
    {
        if (pFifo->bBlocking == true)
            return NULL;
        // drop data
        int32_t gidx = atomic_load(&pFifo->GetIdx) + 1;
        if (gidx >= pFifo->MaxIdxCnt)
            gidx = 0;
        atomic_store(&pFifo->GetIdx, gidx);

        pFifo->DropCnt++;
    }

	int32_t idx = atomic_load(&pFifo->PutIdx);
	int32_t putidx = idx + 1;
	if (putidx >= pFifo->MaxIdxCnt)
		putidx = 0;
	atomic_store(&pFifo->PutIdx, putidx);

	if (atomic_load(&pFifo->GetIdx) < 0)
	{
		atomic_store(&pFifo->GetIdx, idx);
	}

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoPutMultiple(hCFifo_t const pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoPut(pFifo);

	if (pFifo == NULL || *pCnt == 0)
	{
		*pCnt = 0;
		return NULL;
	}

	if (atomic_load(&pFifo->PutIdx) ==
	    atomic_load(&pFifo->GetIdx))
    {
	    if (pFifo->bBlocking == true)
	        return NULL;
	    // Drop
	    int l = *pCnt;
	    CFifoGetMultiple(pFifo, &l);
        pFifo->DropCnt += l;
    }

	int32_t cnt    = *pCnt;
	int32_t idx    = atomic_load(&pFifo->PutIdx);
	int32_t getidx = atomic_load(&pFifo->GetIdx);
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

	atomic_store(&pFifo->PutIdx, putidx);

	if (getidx < 0)
	{
		atomic_store(&pFifo->GetIdx, idx);
	}

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	*pCnt = cnt;

	return p;
}

void CFifoFlush(hCFifo_t const pFifo)
{
	atomic_store(&pFifo->GetIdx, -1);
	atomic_store(&pFifo->PutIdx, 0);
}

int CFifoAvail(hCFifo_t const pFifo)
{
	int len = 0;

	if (atomic_load(&pFifo->GetIdx) < 0)
		return pFifo->MaxIdxCnt;

	int gi = atomic_load(&pFifo->GetIdx);
	int pi = atomic_load(&pFifo->PutIdx);

	if (pi > gi)
	{
		len = pFifo->MaxIdxCnt - pi + gi;
	}
	else if (pi < gi)
	{
		len = gi - pi;
	}

	return len;
}

int CFifoUsed(hCFifo_t const pFifo)
{
	int len = 0;

	if (atomic_load((atomic_int *)&pFifo->GetIdx) < 0)
		return 0;

	int gi = atomic_load((atomic_int *)&pFifo->GetIdx);
	int pi = atomic_load((atomic_int *)&pFifo->PutIdx);

	if (gi < pi)
	{
		len = pi - gi;
	}
	else
	{
		len = pFifo->MaxIdxCnt - gi + pi;
	}

	return len;
}

int CFifoRead(hCFifo_t const pFifo, uint8_t *pBuff, int BuffLen)
{
	if (pFifo == NULL || atomic_load((atomic_int *)&pFifo->GetIdx) < 0 || pBuff == NULL)
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

int CFifoWrite(hCFifo_t const pFifo, uint8_t *pData, int DataLen)
{
	if (pFifo == NULL || pData == NULL)
		return 0;

    if (atomic_load(&pFifo->PutIdx) == atomic_load(&pFifo->GetIdx))
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
