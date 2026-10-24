/**-------------------------------------------------------------------------
@file nrf70_osal_memory.cpp
@brief Nordic nRF Wi-Fi host memory operations for IOsonata.
@license MIT License
Copyright (c) 2026 I-SYST inc.
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
