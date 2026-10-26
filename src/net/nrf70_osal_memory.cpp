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
#include <stdlib.h>
#include <string.h>
#include "net/nrf70_osal_memory.h"

#ifdef IO_ENABLE_NRF70_BM
#include "osal_ops.h"
#endif

extern "C" void *Nrf70OsalMemAlloc(size_t Size)
{
	return malloc(Size);
}

extern "C" void *Nrf70OsalMemZalloc(size_t Size)
{
	return calloc(1, Size);
}

extern "C" void Nrf70OsalMemFree(void *pMem)
{
	free(pMem);
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
