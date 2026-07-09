/**-------------------------------------------------------------------------
@file	rftag.cpp

@brief	RF tag device class base implementation

Local memory tag behavior of the RFTag facet: memory image access, NDEF
container formats over it, engine binding and frame forwarding, write protect
through the configured board callback.

@author	Hoang Nguyen Hoan
@date	Jul. 7, 2026

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

#include "rftag/rftag.h"

bool RFTag::Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	// A failed Init must not leave a previously initialized object usable.
	Valid(false);

	vCfg = Cfg;
	vpProto = nullptr;

	Interface(pIntrf);

	// The base is a local memory tag. It requires the image and a NDEF area
	// inside it. A bus tag subclass overrides Init and lifts this.
	if (vCfg.pMem == nullptr || vCfg.MemSize == 0)
	{
		return false;
	}

	if (vCfg.NdefAddr >= vCfg.MemSize)
	{
		return false;
	}

	// NdefMaxLen of 0 selects the remainder of the tag memory.
	uint32_t avail = vCfg.MemSize - vCfg.NdefAddr;

	if (vCfg.NdefMaxLen == 0)
	{
		vCfg.NdefMaxLen = avail;
	}

	// Subtraction form, the additive form can wrap on large offsets.
	if (vCfg.NdefMaxLen > avail)
	{
		return false;
	}

	Valid(true);

	return true;
}

void RFTag::Reset()
{
	if (vpProto)
	{
		vpProto->Init(this);
	}
}

bool RFTag::Attach(RFTagProto * const pProto)
{
	if (pProto == nullptr || Valid() == false)
	{
		return false;
	}

	if (pProto->Init(this) == false)
	{
		return false;
	}

	vpProto = pProto;

	return true;
}

int RFTag::ProcessFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (vpProto == nullptr)
	{
		return 0;
	}

	return vpProto->OnFrame(pRx, RxLen, pTx, TxCap);
}

int RFTag::MemRead(uint32_t Addr, uint8_t *pBuff, int Len)
{
	if (pBuff == nullptr || Len <= 0 || Addr >= vCfg.MemSize)
	{
		return 0;
	}

	int l = Len;

	// Addr < MemSize is checked above, so the subtraction cannot wrap.
	uint32_t avail = vCfg.MemSize - Addr;

	if ((uint32_t)l > avail)
	{
		l = (int)avail;
	}

	memcpy(pBuff, &vCfg.pMem[Addr], l);

	return l;
}

int RFTag::MemWrite(uint32_t Addr, const uint8_t *pData, int Len)
{
	if (pData == nullptr || Len <= 0 || Addr >= vCfg.MemSize)
	{
		return 0;
	}

	int l = Len;

	// Addr < MemSize is checked above, so the subtraction cannot wrap.
	uint32_t avail = vCfg.MemSize - Addr;

	if ((uint32_t)l > avail)
	{
		l = (int)avail;
	}

	memcpy(&vCfg.pMem[Addr], pData, l);

	return l;
}

bool RFTag::SetNdefNLen16(const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[2];

	// Length prefix plus payload must fit the NDEF area
	if ((uint32_t)Len + sizeof(hdr) > vCfg.NdefMaxLen)
	{
		return false;
	}

	hdr[0] = (uint8_t)(Len >> 8);
	hdr[1] = (uint8_t)Len;

	if (MemWrite(vCfg.NdefAddr, hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		return false;
	}

	return MemWrite(vCfg.NdefAddr + sizeof(hdr), pNdef, Len) == Len;
}

bool RFTag::SetNdefTlv(const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[4];
	uint32_t addr = vCfg.NdefAddr;
	int hlen = 0;

	// Short form framing is 0x03, length, payload, 0xFE terminator
	if ((uint32_t)(Len + 3) > vCfg.NdefMaxLen)
	{
		return false;
	}

	hdr[hlen++] = 0x03;

	if (Len < 0xFF)
	{
		hdr[hlen++] = (uint8_t)Len;
	}
	else
	{
		// Long form framing adds 0xFF and a 2 byte length ahead of payload
		if ((uint32_t)(Len + 5) > vCfg.NdefMaxLen)
		{
			return false;
		}
		hdr[hlen++] = 0xFF;
		hdr[hlen++] = (uint8_t)(Len >> 8);
		hdr[hlen++] = (uint8_t)Len;
	}

	if (MemWrite(addr, hdr, hlen) != hlen)
	{
		return false;
	}

	addr += hlen;

	if (MemWrite(addr, pNdef, Len) != Len)
	{
		return false;
	}

	addr += Len;

	uint8_t term = 0xFE;

	return MemWrite(addr, &term, 1) == 1;
}

bool RFTag::SetNdef(const uint8_t *pNdef, uint16_t Len)
{
	if (Len > 0 && pNdef == nullptr)
	{
		return false;
	}

	bool res;

	switch (vCfg.NdefFmt)
	{
		case RFTAG_NDEF_FMT_NLEN16:
			res = SetNdefNLen16(pNdef, Len);
			break;

		case RFTAG_NDEF_FMT_TLV:
			res = SetNdefTlv(pNdef, Len);
			break;

		case RFTAG_NDEF_FMT_NONE:
		default:
			if ((uint32_t)Len > vCfg.NdefMaxLen)
			{
				return false;
			}
			res = MemWrite(vCfg.NdefAddr, pNdef, Len) == Len;
			break;
	}

	if (res)
	{
		EvtHandler(RFTAG_EVT_MEM_CHANGED, vCfg.NdefAddr, Len);
	}

	return res;
}

int RFTag::GetNdefNLen16(uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[2];

	if (MemRead(vCfg.NdefAddr, hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		return 0;
	}

	uint16_t nlen = ((uint16_t)hdr[0] << 8) | hdr[1];

	if ((uint32_t)nlen + sizeof(hdr) > vCfg.NdefMaxLen)
	{
		return 0;
	}

	if (Len < nlen)
	{
		nlen = Len;
	}

	return MemRead(vCfg.NdefAddr + sizeof(hdr), pNdef, nlen);
}

int RFTag::GetNdefTlv(uint8_t *pNdef, uint16_t Len)
{
	uint32_t addr = vCfg.NdefAddr;
	uint32_t end = vCfg.NdefAddr + vCfg.NdefMaxLen;
	uint8_t b = 0;

	while (addr < end)
	{
		if (MemRead(addr++, &b, 1) != 1)
		{
			return 0;
		}

		if (b == 0x00)
		{
			// NULL TLV, single byte filler
			continue;
		}

		if (b == 0xFE)
		{
			// Terminator before any NDEF TLV
			return 0;
		}

		if (addr >= end)
		{
			return 0;
		}

		if (b != 0x03)
		{
			// Skip a foreign TLV: 1 byte or 0xFF prefixed 2 byte length
			if (MemRead(addr++, &b, 1) != 1)
			{
				return 0;
			}

			uint16_t skip = b;

			if (b == 0xFF)
			{
				uint8_t lbuf[2];

				if (addr + sizeof(lbuf) > end)
				{
					return 0;
				}

				if (MemRead(addr, lbuf, sizeof(lbuf)) != (int)sizeof(lbuf))
				{
					return 0;
				}
				addr += 2;
				skip = ((uint16_t)lbuf[0] << 8) | lbuf[1];
			}

			if (addr >= end || skip > end - addr)
			{
				return 0;
			}

			addr += skip;
			continue;
		}

		// NDEF message TLV, short or long form length
		if (MemRead(addr++, &b, 1) != 1)
		{
			return 0;
		}

		uint16_t nlen = b;

		if (b == 0xFF)
		{
			uint8_t lbuf[2];

			if (addr + sizeof(lbuf) > end)
			{
				return 0;
			}

			if (MemRead(addr, lbuf, sizeof(lbuf)) != (int)sizeof(lbuf))
			{
				return 0;
			}
			addr += 2;
			nlen = ((uint16_t)lbuf[0] << 8) | lbuf[1];
		}

		if (addr >= end || nlen > end - addr)
		{
			return 0;
		}

		if (Len < nlen)
		{
			nlen = Len;
		}

		return MemRead(addr, pNdef, nlen);
	}

	return 0;
}

int RFTag::GetNdef(uint8_t *pNdef, uint16_t Len)
{
	if (pNdef == nullptr || Len == 0)
	{
		return 0;
	}

	switch (vCfg.NdefFmt)
	{
		case RFTAG_NDEF_FMT_NLEN16:
			return GetNdefNLen16(pNdef, Len);

		case RFTAG_NDEF_FMT_TLV:
			return GetNdefTlv(pNdef, Len);

		case RFTAG_NDEF_FMT_NONE:
		default:
		{
			uint16_t l = Len;

			if ((uint32_t)l > vCfg.NdefMaxLen)
			{
				l = (uint16_t)vCfg.NdefMaxLen;
			}
			return MemRead(vCfg.NdefAddr, pNdef, l);
		}
	}
}

bool RFTag::SetWriteProt(bool bVal)
{
	if (vCfg.WrProtCB)
	{
		return vCfg.WrProtCB(vCfg.pWrProtCtx, bVal);
	}

	// No protection mechanism configured. Report unsupported rather than
	// claiming success on an action that did not happen.
	return false;
}
