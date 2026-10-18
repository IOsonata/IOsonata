/**-------------------------------------------------------------------------
@file	bt_pds_stm32wba.cpp

@brief	Mount the bt_pds record store on the STM32WBA on chip flash.

		The store runs on the same Nvm driver every other target uses, over
		the WBA memory controller interface (nvm_stm32wba.cpp). Where the
		store lives comes from the linker script: the project reserves an
		NVM0 region on a flash page boundary, sized a whole number of pages
		and at least two of them, and this file asks nvm_region for it. On a
		dual bank part put the region in the bank the code does not run
		from, so a program or erase does not stall the core that the ST
		link layer's interrupts run on.

		Layering:
		    BLEPLAT_NvmStore / restore (bt_smp_bond_stm32wba.cpp)
		      -> bt_pds.cpp (log structured KV over a region)
		        -> Nvm (src/storage/nvm.cpp)
		          -> NvmIntrf (nvm_stm32wba.cpp, the WBA flash controller)

@author	Hoang Nguyen Hoan
@date	July 29, 2026

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
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "storage/nvm.h"
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"
#include "bluetooth/bt_pds.h"

// Store trace, unconditional for the same reason the Nordic stores' is: a
// release build is where bond data has to survive a reset, and a mount that
// silently did not happen looks exactly like data that was written and not
// found again.
#define BT_PDS_WBA_TRACE

#ifdef BT_PDS_WBA_TRACE
#include "syslog.h"
#define PDS_PRINTF(...)			SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define PDS_PRINTF(...)
#endif

// Which linker region holds the store.
#ifndef BT_PDS_WBA_REGION_NO
#define BT_PDS_WBA_REGION_NO	0
#endif

static NvmIntrf s_PdsIntrf;
static Nvm s_PdsMem;
static bool s_bPdsMounted;

// Bring up the memory and mount the store on it. Safe to call more than
// once; the first caller does the work.
extern "C" int BtPdsWbaInit(void)
{
	if (s_bPdsMounted)
	{
		return 0;
	}

	uintptr_t base = NvmRegionAddr(BT_PDS_WBA_REGION_NO);
	size_t size = NvmRegionSize(BT_PDS_WBA_REGION_NO);

	PDS_PRINTF("PDS: linker NVM%d at %08lX size %08lX\r\n",
			   BT_PDS_WBA_REGION_NO, (unsigned long)base,
			   (unsigned long)size);

	if (base == 0 || size == 0)
	{
		PDS_PRINTF("PDS: no NVM%d region in the linker script, "
				   "bonds will not persist\r\n", BT_PDS_WBA_REGION_NO);

		return -ENODEV;
	}

	if (s_PdsIntrf.Init() == false)
	{
		PDS_PRINTF("PDS: memory interface init failed\r\n");

		return -EIO;
	}

	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	NvmMcuCfg(cfg);

	// The region the linker set aside is the device: base and size come
	// from it, and the geometry from the part.
	cfg.BaseAddr = base;
	cfg.TotalSize = size;

	if (s_PdsMem.Init(cfg, &s_PdsIntrf, 0, 0) == false)
	{
		PDS_PRINTF("PDS: Nvm init failed\r\n");

		return -EIO;
	}

	PDS_PRINTF("PDS: Nvm ok, size %lu, sector %lu, wr %lu\r\n",
			   (unsigned long)s_PdsMem.Size(),
			   (unsigned long)s_PdsMem.LogicalSectorSize(),
			   (unsigned long)s_PdsMem.WriteGran());

	int r = BtPdsInit(&s_PdsMem);
	if (r != 0)
	{
		PDS_PRINTF("PDS: store mount failed %d\r\n", r);

		return r;
	}

	PDS_PRINTF("PDS: store mounted\r\n");

	s_bPdsMounted = true;

	return 0;
}
