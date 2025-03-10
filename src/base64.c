/**-------------------------------------------------------------------------
@file	base64.c

@brief	Base64 encode/decode. It is the Base64 binary to ASCII encode/decode

@author	Nguyen Hoan Hoang
@date	Nov. 3, 2012

@license

MIT License

Copyright (c) 2012 I-SYST inc. All rights reserved.

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

#include "istddef.h"
#include "base64.h"

/*
 * Base64 Encode binary to ASCII
 *
 * @param 	pSrc : Pointer to source binary data
 * 			SrcLen	: Source data length in bytes
 * 			pDest	: Pointer to ASCII destination buffer
 * 			DstLen	: Destination buffer length in bytes
 *
 * 	@return	Number of bytes encoded
 */
int Base64Encode(uint8_t *pSrc, int SrcLen, char *pDest, int DstLen)
{
	int idx = 0;
	uint32_t d = 0;
	uint8_t *p = (uint8_t *)&d;
	int len;
	int cnt = 0;

	while (SrcLen > 0)
	{
		d = 0;
		len = min(SrcLen - 1, 2);
		for (int i = 2; i >= 0 && SrcLen > 0; i--)
		{
			p[i] = *pSrc;
			pSrc++;
			SrcLen--;
		}
		len += 1 + idx;
		for (int i = idx + 3; i >= idx && cnt < DstLen - 1; i--)
		{
			if (i > len)
				pDest[i] = '=';
			else {
				pDest[i] = d & 0x3f;
				if (pDest[i] < 26)
					pDest[i] += 'A';
				else if (pDest[i] < 52)
					pDest[i] += 'a' - 26;
				else if (pDest[i] < 62)
					pDest[i] += '0' - 52;
				else if (pDest[i] == 63)
					pDest[i] = '/';
				else
					pDest[i] = '+';
			}
			d >>= 6;
			cnt++;
		}
		idx += 4;
	}

	pDest[cnt] = '\0';

	return cnt;
}

