/**-------------------------------------------------------------------------
@file	dfu_serial.cpp

@brief	SMP serial transport. See dfu_serial.h.

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

#include "crc.h"
#include "base64.h"
#include "dfu/dfu_serial.h"

/** @addtogroup DFU
  * @{
  */

// Tries at a full transmit FIFO before a response is given up.
#define DFU_SERIAL_TX_RETRY		0x100000UL

// Value of a base64 character, 64 for padding, -1 for anything else.
static int DfuB64Val(uint8_t c)
{
	if (c >= 'A' && c <= 'Z') return c - 'A';
	if (c >= 'a' && c <= 'z') return c - 'a' + 26;
	if (c >= '0' && c <= '9') return c - '0' + 52;
	if (c == '+') return 62;
	if (c == '/') return 63;
	if (c == '=') return 64;
	return -1;
}

uint16_t DfuSerialCrc16(uint16_t Crc, const uint8_t *p, uint32_t Len)
{
	// The library CRC-16/CCITT, polynomial 0x1021 not reflected: with 0 to
	// start it is the XMODEM form SMP uses.
	return crc16_ccitt((uint8_t *)p, (int)Len, Crc);
}

DfuSerial::DfuSerial()
{
	memset(&vCfg, 0, sizeof(vCfg));
	vbInit = false;
	vbReset = false;
	vState = SER_IDLE;
	vMark = 0;
	vbInPkt = false;
	vQuadLen = 0;
	vPad = 0;
	vLen = 0;
	vDropped = 0;
}

bool DfuSerial::Init(const DfuSerialCfg_t &Cfg)
{
	vbInit = false;

	if (Cfg.pMgr == nullptr || Cfg.pIntrf == nullptr ||
		Cfg.pRxBuf == nullptr || Cfg.RxBufSize < 16 ||
		Cfg.pTxBuf == nullptr || Cfg.TxBufSize < DFUSMP_HDR_SIZE + 36)
	{
		return false;
	}

	vCfg = Cfg;
	if (vCfg.TxRetry == 0)
	{
		vCfg.TxRetry = DFU_SERIAL_TX_RETRY;
	}
	vState = SER_IDLE;
	vbInPkt = false;
	vLen = 0;
	vQuadLen = 0;
	vbReset = false;
	vbInit = true;

	return true;
}

void DfuSerial::ResetCB(void *pCtx)
{
	DfuSerial *s = (DfuSerial *)pCtx;

	if (s != nullptr)
	{
		s->vbReset = true;
	}
}

bool DfuSerial::TxAll(const uint8_t *p, uint32_t Len)
{
	uint32_t tries = 0;

	while (Len > 0)
	{
		int n = vCfg.pIntrf->Tx(0, p, (int)Len);

		if (n > 0)
		{
			p += n;
			Len -= (uint32_t)n;
			tries = 0;
		}
		else if (++tries >= vCfg.TxRetry)
		{
			return false;
		}
	}

	return true;
}

// Response in pTxBuf from offset 2, its length already there: add the CRC,
// then send as base64 in frames.
bool DfuSerial::Send(void)
{
	uint8_t *b = vCfg.pTxBuf;
	uint32_t pl = ((uint32_t)b[0] << 8) | b[1];		// packet + CRC
	uint16_t crc = DfuSerialCrc16(0, b + 2, pl - 2);

	b[pl] = (uint8_t)(crc >> 8);
	b[pl + 1] = (uint8_t)crc;

	uint32_t total = pl + 2;
	// Marker, base64, newline, and the terminator Base64Encode adds.
	char line[DFU_SERIAL_FRAME_MAX + 1];
	bool first = true;

	for (uint32_t i = 0; i < total; )
	{
		// Whole groups of 3 bytes per frame, the rest in the last one.
		uint32_t n = total - i;
		if (n > DFU_SERIAL_B64_MAX / 4 * 3)
		{
			n = DFU_SERIAL_B64_MAX / 4 * 3;
		}

		line[0] = (char)(first ? DFU_SERIAL_START1 : DFU_SERIAL_CONT1);
		line[1] = (char)(first ? DFU_SERIAL_START2 : DFU_SERIAL_CONT2);
		first = false;

		int ll = 2 + Base64Encode(b + i, (int)n, line + 2,
								  (int)sizeof(line) - 2);
		line[ll++] = '\n';
		if (TxAll((const uint8_t *)line, (uint32_t)ll) == false)
		{
			return false;
		}
		i += n;
	}

	return true;
}

// A decoded packet is complete: check it, serve it, answer it.
bool DfuSerial::Serve(void)
{
	uint8_t *r = vCfg.pRxBuf;
	uint32_t pl = ((uint32_t)r[0] << 8) | r[1];

	vbInPkt = false;
	vLen = 0;

	uint16_t crc = DfuSerialCrc16(0, r + 2, pl - 2);
	if ((((uint16_t)r[pl] << 8) | r[pl + 1]) != crc)
	{
		vDropped++;
		return false;
	}

	int n = vCfg.pMgr->Process(r + 2, pl - 2, vCfg.pTxBuf + 2,
							   vCfg.TxBufSize - 4);
	if (n <= 0)
	{
		return false;
	}

	vCfg.pTxBuf[0] = (uint8_t)((n + 2) >> 8);
	vCfg.pTxBuf[1] = (uint8_t)(n + 2);

	bool sent = Send();

	if (vbReset)
	{
		vbReset = false;
		if (vCfg.Reset != nullptr)
		{
			vCfg.Reset();
		}
	}

	return sent;
}

// One received byte. Returns true when it completed a request that was
// served.
bool DfuSerial::Byte(uint8_t b)
{
	switch (vState)
	{
		case SER_IDLE:
			if (b == DFU_SERIAL_START1 || b == DFU_SERIAL_CONT1)
			{
				vMark = b;
				vState = SER_MARK1;
			}
			else if (b != '\n')
			{
				vState = SER_SKIP;
			}
			return false;

		case SER_MARK1:
			if (vMark == DFU_SERIAL_START1 && b == DFU_SERIAL_START2)
			{
				// A new packet, whatever was under way.
				vbInPkt = true;
				vLen = 0;
				vQuadLen = 0;
				vPad = 0;
				vState = SER_LINE;
			}
			else if (vMark == DFU_SERIAL_CONT1 && b == DFU_SERIAL_CONT2 &&
					 vbInPkt)
			{
				vState = SER_LINE;
			}
			else
			{
				vState = b == '\n' ? SER_IDLE : SER_SKIP;
			}
			return false;

		case SER_SKIP:
			// Not ours, to the end of the line.
			if (b == '\n')
			{
				vState = SER_IDLE;
			}
			return false;

		case SER_LINE:
			break;
	}

	if (b == '\n' || b == '\r')
	{
		vState = b == '\n' ? SER_IDLE : SER_LINE;
		if (b == '\r')
		{
			return false;
		}

		// A packet is served at the end of the frame that completes it.
		if (vbInPkt && vLen >= 2)
		{
			uint32_t pl = ((uint32_t)vCfg.pRxBuf[0] << 8) | vCfg.pRxBuf[1];

			if (vLen >= pl + 2)
			{
				return Serve();
			}
		}
		return false;
	}

	int v = DfuB64Val(b);
	if (v < 0 || vbInPkt == false)
	{
		// Garbled: drop the packet, wait for the next start frame.
		if (vbInPkt)
		{
			vDropped++;
		}
		vbInPkt = false;
		vState = SER_SKIP;
		return false;
	}

	vQuad[vQuadLen++] = (uint8_t)v;
	if (vQuadLen < 4)
	{
		return false;
	}
	vQuadLen = 0;

	int pad = (vQuad[2] == 64) + (vQuad[3] == 64);
	uint32_t bits = ((uint32_t)(vQuad[0] & 0x3F) << 18) |
					((uint32_t)(vQuad[1] & 0x3F) << 12) |
					((uint32_t)(vQuad[2] & 0x3F) << 6) |
					(uint32_t)(vQuad[3] & 0x3F);
	uint8_t out[3] = {
		(uint8_t)(bits >> 16), (uint8_t)(bits >> 8), (uint8_t)bits,
	};
	int n = 3 - pad;

	if (vQuad[0] == 64 || vQuad[1] == 64 || (vQuad[2] == 64 && vQuad[3] != 64))
	{
		vDropped++;
		vbInPkt = false;
		vState = SER_SKIP;
		return false;
	}

	for (int i = 0; i < n; i++)
	{
		if (vLen >= vCfg.RxBufSize)
		{
			// Larger than this side takes.
			vDropped++;
			vbInPkt = false;
			vState = SER_SKIP;
			return false;
		}
		vCfg.pRxBuf[vLen++] = out[i];
	}

	if (vLen >= 2)
	{
		uint32_t pl = ((uint32_t)vCfg.pRxBuf[0] << 8) | vCfg.pRxBuf[1];

		if (pl < DFUSMP_HDR_SIZE + 2 || pl + 2 > vCfg.RxBufSize)
		{
			vDropped++;
			vbInPkt = false;
			vState = SER_SKIP;
		}
	}

	return false;
}

bool DfuSerial::Feed(const uint8_t *pData, uint32_t Len)
{
	bool served = false;

	if (vbInit == false || pData == nullptr)
	{
		return false;
	}

	for (uint32_t i = 0; i < Len; i++)
	{
		if (Byte(pData[i]))
		{
			served = true;
		}
	}

	return served;
}

bool DfuSerial::Poll(void)
{
	uint8_t buf[64];
	bool served = false;

	if (vbInit == false)
	{
		return false;
	}

	for (;;)
	{
		int n = vCfg.pIntrf->Rx(0, buf, sizeof(buf));

		if (n <= 0)
		{
			break;
		}
		if (Feed(buf, (uint32_t)n))
		{
			served = true;
		}
	}

	return served;
}

/** @} */
