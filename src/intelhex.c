/**-------------------------------------------------------------------------
@file	intelhex.c

Author : Hoang Nguyen Hoan          Feb. 8, 2015

@brief	Intel Hex parser

@author	Nguyen Hoan Hoang
@date	Feb. 8,2015

@license

MIT License

Copyright (c) 2015 I-SYST inc. All rights reserved.

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
#include "convutil.h"
#include "intelhex.h"

/*
 * Parse Intel Hex record (one line)
 *
 * @param	pRec : Pointer to text line of intel hex record
 * 			pData : Pointer to place holder for parsed record
 *
 * @return	true - Success
 * 			false - Bad in record data
 */
bool IHexParseRecord(char *pRec, IHEXDATA *pData)
{
	if (pRec == NULL || pData == NULL)
		return false;
	
	if (pRec[0] != ':')
		return false;

	char *p = pRec + 1;
	int8_t cs = 0;
	
	pData->Count = (chex2i(*p++) << 4);
	pData->Count += chex2i(*p++);
	pData->Offset = (chex2i(*p++) << 12);
	pData->Offset += (chex2i(*p++) << 8);
	pData->Offset += (chex2i(*p++) << 4);
	pData->Offset += chex2i(*p++);
	pData->Type = (chex2i(*p++) << 4);
	pData->Type += chex2i(*p++);
	
	cs += pData->Count + (pData->Offset & 0xff) + ((pData->Offset >> 8u) & 0xff) + pData->Type;
	
	for (int i = 0; i < pData->Count; i++)
	{
		pData->Data[i] = (chex2i(*p++) << 4);
		pData->Data[i] += chex2i(*p++);
		cs += pData->Data[i];
	}
	
	pData->Checksum = (chex2i(*p++) << 4);
	pData->Checksum += chex2i(*p++);
	cs += pData->Checksum;
	
	return cs == 0;
}
