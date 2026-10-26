/**-------------------------------------------------------------------------
@file nrf70_osal_memory.cpp
@brief Nordic nRF Wi-Fi host memory operations for IOsonata.
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
#include <stdint.h>
#include <string.h>
#include "net/nrf70_osal_memory.h"

#ifdef IO_ENABLE_NRF70_BM
#include "osal_ops.h"
#endif

static Nrf70OsalMemBlock_t *s_pBlocks;
static size_t s_BlockCount;
static size_t s_UsedCount;

extern "C" bool Nrf70OsalMemInit(Nrf70OsalMemBlock_t *pBlocks, size_t Count)
{
	if (s_UsedCount != 0 || (pBlocks == nullptr) != (Count == 0) ||
		Count > SIZE_MAX / sizeof(*pBlocks))
	{
		return false;
	}

	uintptr_t table = (uintptr_t)pBlocks;
	size_t tableSize = Count * sizeof(*pBlocks);
	if (tableSize > UINTPTR_MAX - table)
	{
		return false;
	}

	// Validate the entire configuration before changing any ownership flags.
	for (size_t i = 0; i < Count; i++)
	{
		uintptr_t start = (uintptr_t)pBlocks[i].pMem;
		size_t size = pBlocks[i].Size;
		if (start == 0 || start % alignof(max_align_t) != 0 || size == 0 ||
			size > UINTPTR_MAX - start ||
			(start < table + tableSize && table < start + size))
		{
			return false;
		}
		for (size_t j = 0; j < i; j++)
		{
			uintptr_t other = (uintptr_t)pBlocks[j].pMem;
			if (start < other + pBlocks[j].Size && other < start + size)
			{
				return false;
			}
		}
	}

	for (size_t i = 0; i < Count; i++)
	{
		pBlocks[i].Used = false;
	}
	s_pBlocks = pBlocks;
	s_BlockCount = Count;
	return true;
}

extern "C" void *Nrf70OsalMemAlloc(size_t Size)
{
	if (Size == 0)
	{
		return nullptr;
	}

	Nrf70OsalMemBlock_t *best = nullptr;
	for (size_t i = 0; i < s_BlockCount; i++)
	{
		Nrf70OsalMemBlock_t *block = &s_pBlocks[i];
		if (!block->Used && block->Size >= Size &&
			(best == nullptr || block->Size < best->Size))
		{
			best = block;
		}
	}
	if (best == nullptr)
	{
		return nullptr;
	}
	best->Used = true;
	s_UsedCount++;
	return best->pMem;
}

extern "C" void *Nrf70OsalMemZalloc(size_t Size)
{
	void *pMem = Nrf70OsalMemAlloc(Size);
	if (pMem != nullptr)
	{
		memset(pMem, 0, Size);
	}
	return pMem;
}

extern "C" void Nrf70OsalMemFree(void *pMem)
{
	for (size_t i = 0; i < s_BlockCount; i++)
	{
		if (s_pBlocks[i].pMem == pMem && s_pBlocks[i].Used)
		{
			s_pBlocks[i].Used = false;
			s_UsedCount--;
			return;
		}
	}
}

extern "C" void *Nrf70OsalMemCopy(void *pDst, const void *pSrc, size_t Size)
{
	return memcpy(pDst, pSrc, Size);
}

extern "C" void *Nrf70OsalMemSet(void *pDst, int Value, size_t Size)
{
	return memset(pDst, Value, Size);
}

extern "C" int Nrf70OsalMemCompare(const void *pLeft, const void *pRight, size_t Size)
{
	return memcmp(pLeft, pRight, Size);
}

#ifdef IO_ENABLE_NRF70_BM
extern "C" void Nrf70OsalInstallMemory(struct nrf_wifi_osal_ops *pOps)
{
	if (pOps == nullptr)
		return;
	pOps->mem_alloc = Nrf70OsalMemAlloc;
	pOps->mem_zalloc = Nrf70OsalMemZalloc;
	pOps->mem_free = Nrf70OsalMemFree;
	pOps->mem_cpy = Nrf70OsalMemCopy;
	pOps->mem_set = Nrf70OsalMemSet;
	pOps->mem_cmp = Nrf70OsalMemCompare;
}
#endif
