/**-------------------------------------------------------------------------
@file	nvm_mcu.cpp

@brief	MCU internal flash / NVM driver on the NvmIO base.

@author	Hoang Nguyen Hoan
@date	Jul. 22, 2026

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

#include "storage/nvm_mcu.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

NvmMcu::NvmMcu()
{
	vBaseAddr = 0;
	vDevSize = 0;
	vPageSize = 0;
	vWrGran = 4;
	vMaxProgWords = 0;
	vProgram = nullptr;
	vErasePage = nullptr;
	vSetProtect = nullptr;
	vpCtx = nullptr;
}

bool NvmMcu::Init(const NvmCfg_t &Cfg, const NvmMcuOp_t &Op,
				  uint64_t RegionOff, uint64_t RegionSize)
{
	vBaseAddr = Cfg.BaseAddr;
	vDevSize = Cfg.TotalSize;
	vPageSize = Cfg.EraseSize;
	vWrGran = (Cfg.WriteGran != 0) ? Cfg.WriteGran : 4;
	vMaxProgWords = Op.MaxProgWords;
	vProgram = Op.Program;
	vErasePage = Op.ErasePage;
	vSetProtect = Op.SetProtect;
	vpCtx = Op.pCtx;

	// The program and erase operations are required; there is no generic way
	// to reach the memory controller without them.
	if (vProgram == nullptr || vErasePage == nullptr)
	{
		DEBUG_PRINTF("NvmMcu missing program or erase operation\r\n");
		return false;
	}

	// Geometry must be present and the page a whole number of write units.
	if (vDevSize == 0 || vPageSize == 0 || vWrGran == 0 ||
		(vPageSize % vWrGran) != 0)
	{
		DEBUG_PRINTF("NvmMcu geometry invalid: size=%lu page=%lu gran=%lu\r\n",
					 (unsigned long)vDevSize, (unsigned long)vPageSize,
					 (unsigned long)vWrGran);
		return false;
	}

	// The base holds the mode and callbacks.
	SetMode(Cfg.bIntEn, Cfg.EvtHandler, Cfg.pWaitCB);

	uint64_t rsize = (RegionSize != 0) ? RegionSize : (vDevSize - RegionOff);
	if (RegionOff + rsize > vDevSize)
	{
		return false;
	}
	Region(RegionOff, rsize);

	Valid(true);

	return true;
}

int NvmMcu::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	// The memory is mapped; read it directly.
	const void *src = (const void *)(uintptr_t)(vBaseAddr + RegionOffset() + Off);
	memcpy(pBuf, src, Len);

	return (int)Len;
}

int NvmMcu::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}
	// The memory programs whole words only.
	if ((Off % vWrGran) != 0 || (Len % vWrGran) != 0)
	{
		return -EINVAL;
	}

	uintptr_t addr = (uintptr_t)(vBaseAddr + RegionOffset() + Off);
	const uint32_t *p = (const uint32_t *)pData;
	uint32_t words = Len / vWrGran;

	// Split the request to the largest word count the target program operation
	// accepts. This is a limit of the stack performing the write, not a page
	// boundary; a mapped memory has no address auto increment window.
	while (words > 0)
	{
		uint32_t n = words;
		if (vMaxProgWords != 0 && n > vMaxProgWords)
		{
			n = vMaxProgWords;
		}

		int res = vProgram(addr, p, n, vpCtx);
		if (res < 0)
		{
			DEBUG_PRINTF("NvmMcu program failed at 0x%lx res=%d\r\n",
						 (unsigned long)addr, res);
			return res;
		}

		addr += n * vWrGran;
		p += n;
		words -= n;
	}

	return (int)Len;
}

int NvmMcu::Erase(uint64_t Off, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	// Off and Len must be page aligned.
	if ((Off % vPageSize) != 0 || (Len % vPageSize) != 0 || Len == 0)
	{
		return -EINVAL;
	}

	uintptr_t addr = (uintptr_t)(vBaseAddr + RegionOffset() + Off);
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		int res = vErasePage(addr, vpCtx);
		if (res < 0)
		{
			DEBUG_PRINTF("NvmMcu erase failed at 0x%lx res=%d\r\n",
						 (unsigned long)addr, res);
			return res;
		}
		addr += vPageSize;
		cnt -= vPageSize;
	}

	return 0;
}

int NvmMcu::SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable)
{
	if (vSetProtect == nullptr)
	{
		return -ENOTSUP;
	}
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}

	uintptr_t addr = (uintptr_t)(vBaseAddr + RegionOffset() + Off);

	return vSetProtect(addr, Len, bEnable, vpCtx);
}
