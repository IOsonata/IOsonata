/**-------------------------------------------------------------------------
@file	stddev.c

@brief	System functions overload for libc

@author	Hoang Nguyen Hoan
@date	Mar. 4, 2015

@license

MIT License

Copyright (c) 2015, I-SYST inc., all rights reserved

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
#include <stdlib.h>
#include <string.h>

#include "stddev.h"

#define STDDEV_FDIDX_MASK		0xF
#define STDDEV_FDIDX_NBITS		4

StdDev_t *g_DevTable[STDDEV_MAX] = {
	NULL,
};

int InstallBlkDev(StdDev_t * const pDev, int MapId)
{
	int retval = -1;

	switch (MapId)
	{
		case STDIN_FILENO:
		case STDOUT_FILENO:
		case STDERR_FILENO:
		case STDFS_FILENO:
			retval = MapId;
			break;
		default:
			for (int i = STDDEV_USER_FILENO; i < STDDEV_MAX; i++)
			{
				if (g_DevTable[i] == NULL)
				{
					retval = i;
					break;
				}
			}

	}
	if (retval >= 0)
	{
		g_DevTable[retval] = pDev;
	}

	return retval;
}

void RemoveBlkDev(int Idx)
{
	if (Idx >=0 && Idx < STDDEV_MAX)
		g_DevTable[Idx] = NULL;
}

int _open(const char * const pPathName, int Flags, int Mode)
{
//	return g_FatFS.Open((char*)pPathName, Flags, Mode);
	if (pPathName == NULL)
		return -1;

	char *p;
	int retval = -1;

	p = strchr(pPathName, ':');

	if (p == NULL || strncmp(pPathName, "FAT:", 4) == 0)
	{
		retval = g_DevTable[STDFS_FILENO]->Open(g_DevTable[STDFS_FILENO]->pDevObj, pPathName, Flags, Mode);
		if (retval != -1)
		{
			retval = (retval << 4) | STDFS_FILENO;
		}
	}
	else
	{
		// check for named device
		for (int i = STDFS_FILENO; i < STDDEV_MAX; i++)
		{
			if (strncmp(g_DevTable[i]->Name, pPathName, 4) == 0)
			{
				retval = g_DevTable[i]->Open(g_DevTable[i]->pDevObj, pPathName, Flags, Mode);

				if (retval != -1)
				{
					retval = (retval << 4) | i;
				}
			}
		}
	}

	return retval;
}

int _close(int Fd)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx] && g_DevTable[idx]->Close)
		return g_DevTable[idx]->Close(g_DevTable[idx]->pDevObj, Fd);

	return -1;
}

int _lseek(int Fd, int Offset)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx]  && g_DevTable[idx]->Seek)
		return g_DevTable[idx]->Seek(g_DevTable[idx]->pDevObj, Fd, Offset);

	return -1;
}

int _read (int Fd, char *pBuff, size_t Len)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx] && g_DevTable[idx]->Read)
		return g_DevTable[idx]->Read(g_DevTable[idx]->pDevObj, Fd, (uint8_t*)pBuff, Len);

	return -1;
}

int _write (int Fd, char *pBuff, size_t Len)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx] && g_DevTable[idx]->Write)
	{
		return g_DevTable[idx]->Write(g_DevTable[idx]->pDevObj, Fd, (uint8_t*)pBuff, Len);
	}

	return -1;
}
