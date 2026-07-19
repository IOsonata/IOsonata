/**-------------------------------------------------------------------------
@file	diskio_flash.cpp

@brief	Generic flash disk I/O driver class


@author	Hoang Nguyen Hoan
@date	Aug. 30, 2016


@license

MIT License

Copyright (c) 2016 I-SYST inc. All rights reserved.

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
#include "istddef.h"
#include "storage/diskio_flash.h"

FlashDiskIO::FlashDiskIO() : DiskIO()
{
	DiskIO::vType = DISKIO_TYPE_ERASEWR;
	vBlkSize = 0;
}

bool FlashDiskIO::Init(const FlashCfg_t &Cfg, DeviceIntrf * const pInterf,
					   DiskIOCache_t * const pCacheBlk, int NbCacheBlk)
{
	if (vFlash.Init(Cfg, pInterf) == false)
	{
		return false;
	}

	vBlkSize = Cfg.BlkSize;

	if (pCacheBlk && NbCacheBlk > 0)
	{
		SetCache(pCacheBlk, NbCacheBlk);
	}

	return true;
}

void FlashDiskIO::Reset(void)
{
	vFlash.Reset();
	DiskIO::Reset();
}
