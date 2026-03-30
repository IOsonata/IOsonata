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
#include <stdint.h>
#include <string.h>

#include "cfifo.h"

// Mask != 0: pow2 path — single AND, no branch.
// Mask == 0: non-pow2 — modulo
static inline uint32_t cfifo_slot(const CFifo_t *pFifo, uint32_t Idx)
{
    return pFifo->Mask ? (Idx & pFifo->Mask) : (uint32_t)(Idx % (uint32_t)pFifo->MaxIdxCnt);
}

// Struct layout places BlkSize at [8] and pMemStart at [12].
// Adjacent fields → compiler emits LDRD to load both in one cycle.
static inline uint8_t *cfifo_addr(const CFifo_t *pFifo, uint32_t Slot)
{
    return pFifo->pMemStart + Slot * pFifo->BlkSize;
}

/**
 * @brief	Initialize FIFO.
 *
 * This function must be called first to initialize FIFO before any other functions
 * can be used.
 *
 * @param	pMemBlk 		: Pointer to memory block to be used for FIFO.
 * 							  NOTE : This memory block must be word aligned
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
	if (pMemBlk == NULL || BlkSize == 0 || TotalMemSize <= sizeof(CFifo_t))
		return NULL;

	CFifo_t *hdr = (CFifo_t *)pMemBlk;

	/* Monotonic indices start at zero. Empty = (PutIdx == GetIdx). */
	atomic_store_explicit(&hdr->PutIdx, 0u, __ATOMIC_RELAXED);
	atomic_store_explicit(&hdr->GetIdx, 0u, __ATOMIC_RELAXED);

	hdr->BlkSize    = BlkSize;
	hdr->pMemStart  = pMemBlk + sizeof(CFifo_t);
	hdr->MaxIdxCnt  = (int32_t)((TotalMemSize - sizeof(CFifo_t)) / BlkSize);
	hdr->bBlocking  = bBlocking;
	atomic_store_explicit(&hdr->DropCnt, 0u, __ATOMIC_RELAXED);

	/* Precompute Mask for the AND-wrap fast path.
	 * pow2 check: n > 0 && (n & (n-1)) == 0. */
	uint32_t n = (uint32_t)hdr->MaxIdxCnt;
	hdr->Mask = (n > 0u && (n & (n - 1u)) == 0u) ? (n - 1u) : 0u;

	return hdr;
}

uint8_t *CFifoGet(hCFifo_t const pFifo)
{
    if (pFifo == NULL)
    {
    	return NULL;
    }

    uint32_t gi, pi;

    do {
        /* gi loaded RELAXED: the CAS below revalidates it on every attempt,
         * so a stale value is harmless — it just causes a retry.
         * pi loaded ACQUIRE: pairs with the RELEASE store in CFifoPut,
         * ensuring we see the data written to the slot before PutIdx advanced.
         * On Cortex-M4 load-load reordering does not occur in hardware, but
         * the ACQUIRE barrier is required for C11 conformance and for M7/M33. */
        gi = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_RELAXED);
        pi = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_ACQUIRE);

        if (gi == pi)
            return NULL;   /* empty */

    } while (!atomic_compare_exchange_weak_explicit(
                 &pFifo->GetIdx, &gi, gi + 1u,
                 __ATOMIC_RELEASE, __ATOMIC_RELAXED));

    return cfifo_addr(pFifo, cfifo_slot(pFifo, gi));
}

uint8_t *CFifoGetMultiple(hCFifo_t const pFifo, int *pCnt)
{
    if (pCnt == NULL)
    {
    	return CFifoGet(pFifo);
    }

    if (pFifo == NULL || *pCnt <= 0)
    {
        if (pCnt) *pCnt = 0;
        return NULL;
    }

    uint32_t max = (uint32_t)pFifo->MaxIdxCnt;
    uint32_t gi, pi, used, cnt;

    do {
        gi   = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_RELAXED);
        pi   = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_ACQUIRE);
        used = pi - gi;

        if (used == 0u) {
            *pCnt = 0;
            return NULL;
        }

        cnt = (uint32_t)*pCnt;
        if (cnt > used) cnt = used;

        /* Limit to slots that are physically contiguous before ring wrap. */
        uint32_t slot  = cfifo_slot(pFifo, gi);
        uint32_t avail_before_wrap = max - slot;

        if (cnt > avail_before_wrap)
        {
        	cnt = avail_before_wrap;
        }

    } while (!atomic_compare_exchange_weak_explicit(
                 &pFifo->GetIdx, &gi, gi + cnt,
                 __ATOMIC_RELEASE, __ATOMIC_RELAXED));

    *pCnt = (int)cnt;

    return cfifo_addr(pFifo, cfifo_slot(pFifo, gi));
}

uint8_t *CFifoPut(hCFifo_t const pFifo)
{
    if (pFifo == NULL)
    {
    	return NULL;
    }

    /* Single-producer contract: pi is loaded RELAXED because only one caller
     * advances PutIdx. If two contexts call CFifoPut concurrently they will
     * load the same pi, claim the same slot, and corrupt each other's data.
     * The caller (e.g. Queue::Send) MUST hold a critical section or otherwise
     * guarantee mutual exclusion across all CFifoPut call sites. */
    uint32_t max = (uint32_t)pFifo->MaxIdxCnt;

    uint32_t pi   = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_RELAXED);
    uint32_t gi   = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_ACQUIRE);
    uint32_t used = pi - gi;

    if (used >= max)
    {
        if (pFifo->bBlocking)
        {
        	return NULL;
        }

        /* Drop oldest: advance GetIdx by 1.
         * CAS needed here because a concurrent consumer may race. */
        uint32_t old_gi = gi;

        while (!atomic_compare_exchange_weak_explicit(
                   &pFifo->GetIdx, &old_gi, old_gi + 1u,
                   __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
            ;

        atomic_fetch_add_explicit(&pFifo->DropCnt, 1u, __ATOMIC_RELAXED);
    }

    /* Advance PutIdx with RELEASE so the consumer sees our data write.
     * No CAS required: single-producer contract (see above). */
    uint32_t slot = cfifo_slot(pFifo, pi);
    atomic_store_explicit(&pFifo->PutIdx, pi + 1u, __ATOMIC_RELEASE);

    return cfifo_addr(pFifo, slot);
}

uint8_t *CFifoPutMultiple(hCFifo_t const pFifo, int *pCnt)
{
    if (pCnt == NULL)
    {
    	return CFifoPut(pFifo);
    }

    /* Covers both pFifo==NULL and *pCnt<=0. */
    if (pFifo == NULL || *pCnt <= 0)
    {
        *pCnt = 0;
        return NULL;
    }

    /* Single-producer contract applies here for the same reason as CFifoPut. */
    uint32_t max = (uint32_t)pFifo->MaxIdxCnt;

    uint32_t pi    = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_RELAXED);
    uint32_t gi    = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_ACQUIRE);
    uint32_t used  = pi - gi;
    uint32_t avail = max - used;

    if (avail == 0u)
    {
        if (pFifo->bBlocking)
        {
            *pCnt = 0;
            return NULL;
        }

        /* Drop enough entries to make room, then re-read GetIdx. */
        int drop = *pCnt;

        CFifoGetMultiple(pFifo, &drop);
        atomic_fetch_add_explicit(&pFifo->DropCnt, (uint32_t)drop, __ATOMIC_RELAXED);
        gi    = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_ACQUIRE);
        avail = max - (pi - gi);
    }

    uint32_t cnt = (uint32_t)*pCnt;

    if (cnt > avail)
    {
    	cnt = avail;
    }

    /* Limit to physically contiguous slots before the ring wraps. */
    uint32_t slot              = cfifo_slot(pFifo, pi);
    uint32_t avail_before_wrap = max - slot;

    if (cnt > avail_before_wrap)
    {
    	cnt = avail_before_wrap;
    }

    atomic_store_explicit(&pFifo->PutIdx, pi + cnt, __ATOMIC_RELEASE);

    *pCnt = (int)cnt;

    return cfifo_addr(pFifo, slot);
}

void CFifoFlush(hCFifo_t const pFifo)
{
    if (pFifo == NULL)
    {
    	return;
    }

    /* Precondition: no concurrent CFifoPut may be in progress.
     * If a producer increments PutIdx between our RELAXED load and the
     * RELEASE store below, GetIdx is set to a stale value and the FIFO
     * will permanently report phantom used-items. The caller must ensure
     * all producers are quiesced (e.g. inside a critical section) before
     * calling CFifoFlush. */
    uint32_t pi = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_RELAXED);
    atomic_store_explicit(&pFifo->GetIdx, pi, __ATOMIC_RELEASE);
}

int CFifoAvail(hCFifo_t const pFifo)
{
    if (pFifo == NULL)
    {
    	return 0;
    }

    uint32_t pi = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_RELAXED);
    uint32_t gi = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_RELAXED);

    return (int)((uint32_t)pFifo->MaxIdxCnt - (pi - gi));
}

int CFifoUsed(hCFifo_t const pFifo)
{
    if (pFifo == NULL)
    {
    	return 0;
    }

    uint32_t pi = atomic_load_explicit(&pFifo->PutIdx, __ATOMIC_RELAXED);
    uint32_t gi = atomic_load_explicit(&pFifo->GetIdx, __ATOMIC_RELAXED);

    return (int)(pi - gi);
}

int CFifoRead(hCFifo_t const pFifo, uint8_t *pBuff, int BuffLen)
{
    if (pFifo == NULL || pBuff == NULL || BuffLen <= 0)
    {
    	return 0;
    }

    int cnt = 0;

    if (BuffLen <= (int)pFifo->BlkSize)
    {
        uint8_t *p = CFifoGet(pFifo);

        if (p)
        {
        	memcpy(pBuff, p, (size_t)BuffLen); cnt = BuffLen;
        }
    }
    else
    {
        while (BuffLen > 0)
        {
            int l = BuffLen / (int)pFifo->BlkSize;

            if (BuffLen % (int)pFifo->BlkSize)
            {
            	l++;
            }

            uint8_t *p = CFifoGetMultiple(pFifo, &l);

            if (!p)
            {
            	break;
            }

            int bytes = l * (int)pFifo->BlkSize;

            memcpy(pBuff, p, (size_t)bytes);
            pBuff += bytes; BuffLen -= bytes; cnt += bytes;
        }
    }

    return cnt;
}

int CFifoWrite(hCFifo_t const pFifo, uint8_t *pData, int DataLen)
{
    if (pFifo == NULL || pData == NULL || DataLen <= 0)
        return 0;

    int cnt = 0;

    if (DataLen <= (int)pFifo->BlkSize)
    {
        uint8_t *p = CFifoPut(pFifo);
        if (p)
        {
        	memcpy(p, pData, (size_t)DataLen); cnt = DataLen;
        }
    }
    else
    {
        while (DataLen > 0)
        {
            int l = DataLen / (int)pFifo->BlkSize;

            if (DataLen % (int)pFifo->BlkSize)
            {
            	l++;
            }

            uint8_t *p = CFifoPutMultiple(pFifo, &l);

            if (!p)
            {
            	break;
            }

            int bytes = l * (int)pFifo->BlkSize;

            memcpy(p, pData, (size_t)bytes);
            pData += bytes; DataLen -= bytes; cnt += bytes;
        }
    }

    return cnt;
}
