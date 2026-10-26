/**-------------------------------------------------------------------------
@file nrf70_osal_memory.h
@brief Allocation and memory callbacks for the Nordic nRF Wi-Fi OSAL.
@license MIT License
Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#ifndef __NRF70_OSAL_MEMORY_H__
#define __NRF70_OSAL_MEMORY_H__

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** One caller-owned fixed block. Used is maintained by the OSAL. */
typedef struct __Nrf70_Osal_Memory_Block
{
	void *pMem;
	size_t Size;
	bool Used;
} Nrf70OsalMemBlock_t;

/**
 * Install fixed blocks before Nordic host initialization. Blocks must be
 * non-overlapping, aligned for max_align_t, and separate from the table.
 * The table and storage must remain alive and unchanged until detached.
 * Initial Used values are ignored. Reconfiguration is refused while any
 * block is allocated; invalid configuration preserves the installed table.
 * Pass NULL, 0 after host shutdown and all frees to detach the table.
 *
 * Single foreground owner only, like the nRF70 bare-metal callbacks. Do not
 * call from an ISR, another core or a concurrent RTOS thread.
 */
bool Nrf70OsalMemInit(Nrf70OsalMemBlock_t *pBlocks, size_t Count);

/** Smallest available fitting block, NULL for zero size or exhaustion.
 * No fallback allocation. Allocations are aligned for max_align_t.
 */
void *Nrf70OsalMemAlloc(size_t Size);
void *Nrf70OsalMemZalloc(size_t Size);
void Nrf70OsalMemFree(void *pMem);
void *Nrf70OsalMemCopy(void *pDst, const void *pSrc, size_t Size);
void *Nrf70OsalMemSet(void *pDst, int Value, size_t Size);
int Nrf70OsalMemCompare(const void *pLeft, const void *pRight, size_t Size);

/*
 * Compile with IO_ENABLE_NRF70_BM and the exact pinned Nordic nrf_wifi
 * headers to populate the corresponding fields of the OSAL operations table.
 * The full table must be initialized by the host port before driver startup.
 */
#ifdef IO_ENABLE_NRF70_BM
struct nrf_wifi_osal_ops;
void Nrf70OsalInstallMemory(struct nrf_wifi_osal_ops *pOps);
#endif

#ifdef __cplusplus
}
#endif
#endif
