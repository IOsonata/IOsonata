/**-------------------------------------------------------------------------
@file	dfu_store_nvm.cpp

@brief	A DFU store over an Nvm, for an application whose memory a radio
		stack arbitrates. The Nvm's NvmIntrf asks the stack for each
		operation; a refusal for the moment is retried here.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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
#include <errno.h>

#include "storage/nvm.h"
#include "dfu/dfu_writer.h"

/** @addtogroup DFU
  * @{
  */

// Tries at a memory operation the radio arbitration refused for the moment.
#define DFU_STORE_NVM_RETRY		50

static bool DfuNvmStRead(const DfuStore_t *pSt, uint32_t Off, void *pBuf,
						 uint32_t Len)
{
	Nvm *nvm = (Nvm *)pSt->pCtx;
	int res = -EBUSY;

	for (int i = 0; i < DFU_STORE_NVM_RETRY && res == -EBUSY; i++)
	{
		res = nvm->Read(Off, pBuf, Len);
	}

	return res == (int)Len;
}

static bool DfuNvmStWrite(const DfuStore_t *pSt, uint32_t Off,
						  const void *pData, uint32_t Len)
{
	Nvm *nvm = (Nvm *)pSt->pCtx;
	int res = -EBUSY;

	for (int i = 0; i < DFU_STORE_NVM_RETRY && res == -EBUSY; i++)
	{
		res = nvm->Write(Off, pData, Len);
	}

	return res >= 0;
}

static bool DfuNvmStErase(const DfuStore_t *pSt, uint32_t Off, uint32_t Len)
{
	Nvm *nvm = (Nvm *)pSt->pCtx;
	uint32_t e = pSt->EraseSize;

	if (e == 0)
	{
		return true;
	}

	// Whole units covering the range.
	uint32_t start = Off - Off % e;
	uint32_t end = (Off + Len + e - 1) / e * e;
	int res = -EBUSY;

	for (int i = 0; i < DFU_STORE_NVM_RETRY && res == -EBUSY; i++)
	{
		res = nvm->Erase(start, end - start);
	}

	return res >= 0;
}

bool DfuStoreNvm(DfuStore_t *pSt, Nvm *pNvm)
{
	if (pSt == nullptr || pNvm == nullptr || pNvm->WriteGran() == 0 ||
		pNvm->WriteGran() > DFU_TGT_WRITE_UNIT_MAX)
	{
		return false;
	}

	pSt->Size = (uint32_t)pNvm->Size();
	pSt->EraseSize = pNvm->EraseSize();
	pSt->WriteGran = pNvm->WriteGran();
	pSt->Addr = 0;
	pSt->Read = DfuNvmStRead;
	pSt->Write = DfuNvmStWrite;
	pSt->Erase = DfuNvmStErase;
	pSt->UnitAt = nullptr;
	pSt->pCtx = pNvm;

	return true;
}

/** @} */
