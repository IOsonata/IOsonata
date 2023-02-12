/**-------------------------------------------------------------------------
@file	diskio_impl.cpp

@brief	Generic Disk I/O driver class

@author	Hoang Nguyen Hoan
@date	Mar. 1, 2015

@license

MIT License

Copyright (c) 2015, I-SYST, all rights reserved

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
#include <stdio.h>

#include "istddef.h"
#include "crc.h"
#include "storage/diskio.h"

DiskIO::DiskIO() : vLastIdx(-1), vNbCache(0), vpCacheSect(NULL)
{
}

void DiskIO::SetCache(DiskIOCache_t * const pCacheBlk, int NbCacheBlk)
{
	if (pCacheBlk == NULL || NbCacheBlk <= 0)
		return;

	vNbCache = NbCacheBlk;

	vpCacheSect = pCacheBlk;

	for (int i = 0; i < vNbCache; i++)
	{
		vpCacheSect[i].UseCnt = 0;
		vpCacheSect[i].SectNo = -1;
	}
}

void DiskIO::Reset()
{
	for (int i = 0; i < vNbCache; i++)
	{
		vpCacheSect[i].UseCnt = 0;
		vpCacheSect[i].SectNo = -1;
	}
}

int	DiskIO::GetCacheSect(uint32_t SectNo, bool bLock)
{
    // Try to find sector in cache
	for (int i = 0; i < vNbCache; i++)
	{
		// Grab first cache
		vpCacheSect[i].UseCnt++;
		if (vpCacheSect[i].SectNo == SectNo)
		{
			return i;
		}
		// Not requested sector release it
		vpCacheSect[i].UseCnt--;
	}

	// Not in cache, try to pick unused cache
	int i = vNbCache;

	while (i > 0)
	{
		vLastIdx++;

		if (vLastIdx >= vNbCache)
			vLastIdx = 0;

		if ((vpCacheSect[vLastIdx].UseCnt & ~DISKIO_CACHE_DIRTY_BIT) == 0)
		{
			// Got unused cache

		    // Flush cache is dirty
		    if (vpCacheSect[vLastIdx].UseCnt & DISKIO_CACHE_DIRTY_BIT)
		        SectWrite(vpCacheSect[vLastIdx].SectNo, vpCacheSect[vLastIdx].pSectData);

	        vpCacheSect[vLastIdx].UseCnt = 1;

	        // Fill cache
			SectRead(SectNo, vpCacheSect[vLastIdx].pSectData);

			vpCacheSect[vLastIdx].SectNo = SectNo;
			return vLastIdx;
		}
		i--;
	}

	// No Cache avail
	return -1;
}

int DiskIO::Read(uint32_t SectNo, uint32_t SectOffset, uint8_t *pBuff, uint32_t Len)
{
	if (pBuff == NULL)
		return -1;

	uint32_t sectsize = GetSectSize();
	int l = min(Len, (sectsize - SectOffset));

	int idx = GetCacheSect(SectNo);
	if (idx < 0)
	{
	    // No cache, do physical read
	    uint8_t d[sectsize];
	    SectRead(SectNo, d);
	    memcpy(pBuff, d + SectOffset, l);
	}
	else
	{
	    // Get it from cache
	    memcpy(pBuff, vpCacheSect[idx].pSectData + SectOffset, l);

	    // Done with cache sector, release it
	    vpCacheSect[idx].UseCnt--;
	}

	return l;
}

int DiskIO::Read(uint64_t Offset, uint8_t *pBuff, uint32_t Len)
{
	uint32_t sectsize = GetSectSize();
	uint64_t sectno = Offset / sectsize;
	uint32_t sectoff = Offset % sectsize;

	uint32_t retval = 0;

	while (Len > 0)
	{
		int l = Read(sectno, sectoff, pBuff, Len);
		if (l <= 0)
			break;
		pBuff += l;
		Len -= l;
		retval += l;
		sectoff += l;
		if (sectoff >= sectsize)
		{
			sectno++;
			sectoff = 0;
		}
	}

	return retval;
}

int DiskIO::Write(uint32_t SectNo, uint32_t SectOffset, uint8_t *pData, uint32_t Len)
{
	if (pData == NULL)
		return -1;

	uint32_t sectsize = GetSectSize();
	uint32_t l = min(Len, sectsize - SectOffset);

	int idx = GetCacheSect(SectNo, true);
	if (idx < 0)
	{
	    // No cache, do physical write
	    uint8_t d[sectsize];
	    SectRead(SectNo, d);
	    memcpy(d + SectOffset, pData, l);
	    SectWrite(SectNo, d);
	}
	else
	{
	    // Write to cache
        memcpy(vpCacheSect[idx].pSectData + SectOffset, pData, l);

        // Done with cache sector, release it
        vpCacheSect[idx].UseCnt |= DISKIO_CACHE_DIRTY_BIT;
        vpCacheSect[idx].UseCnt--;
	}

	return l;
}

int DiskIO::Write(uint64_t Offset, uint8_t *pData, uint32_t Len)
{
	uint32_t sectsize = GetSectSize();
	uint64_t sectno = Offset / sectsize;
	uint32_t sectoff = Offset % sectsize;

	uint32_t retval = 0;

	while (Len > 0)
	{
		int l = Write(sectno, sectoff, pData, Len);
		if (l < 0)
			break;
		pData += l;
		Len -= l;
		retval += l;
		sectoff += l;
		if (sectoff >= sectsize)
		{
			sectno++;
			sectoff = 0;
		}
	}

	return retval;
}

void DiskIO::Flush()
{
    for (int i = 0; i < vNbCache; i++)
    {
        if (vpCacheSect[i].UseCnt & DISKIO_CACHE_DIRTY_BIT)
        {
            SectWrite(vpCacheSect[i].SectNo, vpCacheSect[i].pSectData);
            vpCacheSect[i].UseCnt &= ~DISKIO_CACHE_DIRTY_BIT;
        }
    }
}
