/**-------------------------------------------------------------------------
@file	dfu_http.cpp

@brief	Image download over HTTP/1.1 into slot 1. See dfu_http.h.

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
#include <string.h>

#include "dfu/dfu_http.h"

/** @addtogroup DFU
  * @{
  */

// Tries at a stream that takes nothing, sending the request.
#define DFU_HTTP_TX_RETRY		0x100000UL

static char DfuHttpLower(char c)
{
	return c >= 'A' && c <= 'Z' ? (char)(c - 'A' + 'a') : c;
}

// Case insensitive prefix match of a header name, colon included.
static const char *DfuHttpHdr(const char *pLine, const char *pName)
{
	size_t n = strlen(pName);

	for (size_t i = 0; i < n; i++)
	{
		if (DfuHttpLower(pLine[i]) != pName[i])
		{
			return nullptr;
		}
	}

	const char *p = pLine + n;
	while (*p == ' ' || *p == '\t')
	{
		p++;
	}

	return p;
}

// Decimal at p, false when there is none or it overflows 32 bits.
static bool DfuHttpNum(const char *&p, uint32_t *pVal)
{
	uint64_t v = 0;
	bool any = false;

	while (*p >= '0' && *p <= '9')
	{
		v = v * 10 + (uint64_t)(*p - '0');
		if (v > 0xFFFFFFFFULL)
		{
			return false;
		}
		p++;
		any = true;
	}

	*pVal = (uint32_t)v;

	return any;
}

// Append a string at p + *pN.
static void DfuHttpPut(char *p, int *pN, const char *s)
{
	size_t l = strlen(s);

	memcpy(p + *pN, s, l);
	*pN += (int)l;
}

static int DfuHttpUtoa(uint32_t v, char *p)
{
	char t[10];
	int n = 0;

	do
	{
		t[n++] = (char)('0' + v % 10);
		v /= 10;
	} while (v != 0);

	for (int i = 0; i < n; i++)
	{
		p[i] = t[n - 1 - i];
	}

	return n;
}

DfuHttp::DfuHttp()
{
	vpWr = nullptr;
	vpStream = nullptr;
	vState = DFU_HTTP_IDLE;
	vErr = DFU_HTTP_ERR_NONE;
	vImgErr = 0;
	vHost[0] = 0;
	vPath[0] = 0;
	vLineLen = 0;
	vbLineLong = false;
	vbStatusSeen = false;
	vStatus = 0;
	vbLen = false;
	vLen = 0;
	vbRange = false;
	vRangeFrom = 0;
	vRangeTotal = 0;
	vbChunked = false;
	vAsked = 0;
	vTotal = 0;
	vBodyLeft = 0;
}

bool DfuHttp::Init(DfuMgr *pMgr)
{
	if (pMgr == nullptr || pMgr->Ready() == false || pMgr->Direct())
	{
		return false;
	}

	vpWr = pMgr;
	vState = DFU_HTTP_IDLE;

	return true;
}

void DfuHttp::Fail(int Err)
{
	vErr = Err;
	vState = DFU_HTTP_FAILED;
	if (vpWr != nullptr)
	{
		vpWr->Abort();
	}
}

bool DfuHttp::Request(uint32_t From)
{
	char req[DFU_HTTP_PATH_MAX + DFU_HTTP_HOST_MAX + 96];
	int n = 0;

	DfuHttpPut(req, &n, "GET ");
	DfuHttpPut(req, &n, vPath);
	DfuHttpPut(req, &n, " HTTP/1.1\r\nHost: ");
	DfuHttpPut(req, &n, vHost);
	DfuHttpPut(req, &n, "\r\n");
	if (From != 0)
	{
		DfuHttpPut(req, &n, "Range: bytes=");
		n += DfuHttpUtoa(From, req + n);
		DfuHttpPut(req, &n, "-\r\n");
	}
	DfuHttpPut(req, &n, "Connection: close\r\n\r\n");

	const uint8_t *p = (const uint8_t *)req;
	uint32_t left = (uint32_t)n;
	uint32_t tries = 0;

	while (left > 0)
	{
		int s = vpStream->Tx(0, p, (int)left);

		if (s > 0)
		{
			p += s;
			left -= (uint32_t)s;
			tries = 0;
		}
		else if (++tries >= DFU_HTTP_TX_RETRY)
		{
			return false;
		}
	}

	vAsked = From;
	vLineLen = 0;
	vbLineLong = false;
	vbStatusSeen = false;
	vStatus = 0;
	vbLen = false;
	vbRange = false;
	vbChunked = false;
	vState = DFU_HTTP_HEADER;

	return true;
}

bool DfuHttp::Get(DeviceIntrf *pStream, const char *pHost, const char *pPath)
{
	if (vpWr == nullptr || pStream == nullptr || pHost == nullptr ||
		pPath == nullptr || pPath[0] != '/' ||
		strlen(pHost) >= sizeof(vHost) || strlen(pPath) >= sizeof(vPath))
	{
		return false;
	}

	// Nothing kept from an earlier image.
	vpWr->Abort();
	strcpy(vHost, pHost);
	strcpy(vPath, pPath);
	vpStream = pStream;
	vTotal = 0;
	vErr = DFU_HTTP_ERR_NONE;
	vImgErr = 0;

	if (Request(0) == false)
	{
		Fail(DFU_HTTP_ERR_SEND);
		return false;
	}

	return true;
}

bool DfuHttp::Resume(DeviceIntrf *pStream)
{
	if (vState != DFU_HTTP_BROKEN || pStream == nullptr)
	{
		return false;
	}

	vpStream = pStream;

	// Nothing written yet: the image starts over, the length check with it.
	uint32_t from = vpWr->Active() ? vpWr->Offset() : 0;

	if (Request(from) == false)
	{
		Fail(DFU_HTTP_ERR_SEND);
		return false;
	}

	return true;
}

void DfuHttp::HeaderLine(void)
{
	vLine[vLineLen] = 0;

	if (vbStatusSeen == false)
	{
		// HTTP/1.x nnn reason
		vbStatusSeen = true;
		const char *p = vLine;
		if (strncmp(p, "HTTP/1.", 7) != 0)
		{
			vStatus = 0;
			return;
		}
		p += 7;
		while (*p != 0 && *p != ' ')
		{
			p++;
		}
		while (*p == ' ')
		{
			p++;
		}
		uint32_t s;
		vStatus = DfuHttpNum(p, &s) ? (int)s : 0;
		return;
	}

	const char *v;
	if ((v = DfuHttpHdr(vLine, "content-length:")) != nullptr)
	{
		vbLen = DfuHttpNum(v, &vLen);
	}
	else if ((v = DfuHttpHdr(vLine, "content-range:")) != nullptr)
	{
		// bytes first-last/total
		uint32_t last;
		if (strncmp(v, "bytes ", 6) == 0)
		{
			v += 6;
			vbRange = DfuHttpNum(v, &vRangeFrom) && *v++ == '-' &&
					  DfuHttpNum(v, &last) && *v++ == '/' &&
					  DfuHttpNum(v, &vRangeTotal) && last < vRangeTotal &&
					  vRangeFrom <= last;
		}
	}
	else if ((v = DfuHttpHdr(vLine, "transfer-encoding:")) != nullptr)
	{
		vbChunked = DfuHttpLower(v[0]) != 'i';		// anything but identity
	}
}

// The blank line: decide what the body is.
bool DfuHttp::HeaderEnd(void)
{
	if (vbChunked || vbLen == false)
	{
		Fail(DFU_HTTP_ERR_HEADER);
		return false;
	}

	if (vStatus == 206)
	{
		if (vbRange == false || vRangeFrom != vAsked ||
			vRangeTotal != vRangeFrom + vLen ||
			(vTotal != 0 && vRangeTotal != vTotal) ||
			vpWr->Active() == false || vpWr->Offset() != vAsked)
		{
			Fail(DFU_HTTP_ERR_RANGE);
			return false;
		}
	}
	else if (vStatus == 200)
	{
		// The whole image, whatever was asked.
		vTotal = vLen;
		int r = vpWr->BeginImage(vLen);
		if (r != DFU_IMG_OK)
		{
			Fail(r == DFU_IMG_ERR_SIZE ? DFU_HTTP_ERR_SIZE : DFU_HTTP_ERR_WRITE);
			return false;
		}
	}
	else
	{
		Fail(DFU_HTTP_ERR_STATUS);
		return false;
	}

	vTotal = vpWr->Length();
	vBodyLeft = vLen;
	vState = DFU_HTTP_BODY;

	return true;
}

void DfuHttp::Body(const uint8_t *p, uint32_t Len)
{
	if (Len > vBodyLeft)
	{
		// More than the length said: the rest is not ours.
		Len = vBodyLeft;
	}

	int r = vpWr->Write(vpWr->Offset(), p, Len);
	if (r != DFU_IMG_OK)
	{
		vImgErr = r;
		Fail(r == DFU_IMG_ERR_READ ? DFU_HTTP_ERR_WRITE : DFU_HTTP_ERR_IMAGE);
		return;
	}
	vBodyLeft -= Len;

	if (vpWr->Offset() == vpWr->Length())
	{
		DfuImgInfo_t info;

		r = vpWr->Finish(&info);
		if (r != DFU_IMG_OK)
		{
			vImgErr = r;
			Fail(DFU_HTTP_ERR_IMAGE);
			return;
		}
		if (vpWr->Commit() == false)
		{
			Fail(DFU_HTTP_ERR_WRITE);
			return;
		}
		vState = DFU_HTTP_DONE;
	}
}

DFU_HTTP_STATE DfuHttp::Poll(void)
{
	uint8_t buf[256];

	while ((vState == DFU_HTTP_HEADER || vState == DFU_HTTP_BODY) &&
		   vpStream != nullptr)
	{
		int n = vpStream->Rx(0, buf, sizeof(buf));

		if (n <= 0)
		{
			break;
		}

		const uint8_t *p = buf;
		uint32_t left = (uint32_t)n;

		while (left > 0 && vState == DFU_HTTP_HEADER)
		{
			char c = (char)*p++;
			left--;

			if (c == '\n')
			{
				if (vLineLen > 0 && vLine[vLineLen - 1] == '\r')
				{
					vLineLen--;
				}
				if (vLineLen == 0 && vbLineLong == false && vbStatusSeen)
				{
					HeaderEnd();
				}
				else if (vbLineLong == false)
				{
					HeaderLine();
				}
				vLineLen = 0;
				vbLineLong = false;
			}
			else if (vLineLen < sizeof(vLine) - 1)
			{
				vLine[vLineLen++] = c;
			}
			else
			{
				// Too long to be one this cares about: skipped.
				vbLineLong = true;
			}
		}

		if (left > 0 && vState == DFU_HTTP_BODY)
		{
			Body(p, left);
		}
	}

	return vState;
}

void DfuHttp::StreamEnded(void)
{
	if (vState == DFU_HTTP_HEADER || vState == DFU_HTTP_BODY)
	{
		vState = DFU_HTTP_BROKEN;
	}
}

/** @} */
