/*--------------------------------------------------------------------------
File   : rftag.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : RF tag memory device implementation

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <string.h>
#include <algorithm>
using namespace std;

#include "istddef.h"
#include "idelay.h"
#include "rftag/rftag.h"
#include "iopinctrl.h"

RFTag::RFTag()
{
	memset(&vDevData, 0, sizeof(RFTagDev_t));
}

RFTag::~RFTag()
{
}

static void RFTagAddrEncode(RFTagDev_t * const pDev, uint32_t Addr, uint8_t *pAddr)
{
	for (int i = 0; i < pDev->AddrLen; i++)
	{
		pAddr[i] = (uint8_t)(Addr >> ((pDev->AddrLen - i - 1) << 3));
	}
}

bool RFTagInit(RFTagDev_t * const pDev, const RFTagCfg_t * const pCfg, DevIntrf_t * const pIntrf)
{
	if (pDev == nullptr || pCfg == nullptr || pIntrf == nullptr)
	{
		return false;
	}

	if (pCfg->AddrLen > 4)
	{
		return false;
	}

	if (pCfg->NdefAddr > pCfg->Size)
	{
		return false;
	}

	if (pCfg->NdefMaxLen > 0 && pCfg->NdefMaxLen > (pCfg->Size - pCfg->NdefAddr))
	{
		return false;
	}

	memset(pDev, 0, sizeof(RFTagDev_t));

	pDev->pIntrf = pIntrf;
	pDev->DevAddr = pCfg->DevAddr;
	pDev->PageSize = pCfg->PageSize;
	pDev->AddrLen = pCfg->AddrLen;
	pDev->Size = pCfg->Size;
	pDev->WrDelay = pCfg->WrDelay * 1000;
	pDev->NdefAddr = pCfg->NdefAddr;
	pDev->NdefMaxLen = pCfg->NdefMaxLen ? pCfg->NdefMaxLen : (pCfg->Size - pCfg->NdefAddr);
	pDev->NdefFmt = pCfg->NdefFmt;
	pDev->FdPin = pCfg->FdPin;
	pDev->WrProtPin = pCfg->WrProtPin;
	pDev->pWaitCB = pCfg->pWaitCB;
	pDev->pEvtCB = pCfg->pEvtCB;
	pDev->pCtx = pCfg->pCtx;

	if (pCfg->WrProtPin.PortNo >= 0 && pCfg->WrProtPin.PinNo >= 0)
	{
		IOPinCfg(&pCfg->WrProtPin, 1);
		IOPinClear(pDev->WrProtPin.PortNo, pDev->WrProtPin.PinNo);
	}

	if (pCfg->FdPin.PortNo >= 0 && pCfg->FdPin.PinNo >= 0)
	{
		IOPinCfg(&pCfg->FdPin, 1);
	}

	if (pCfg->pInitCB)
	{
		return pCfg->pInitCB(pCfg->DevAddr, pIntrf);
	}

	return true;
}

int RFTagRead(RFTagDev_t * const pDev, uint32_t Addr, uint8_t *pBuff, int Len)
{
	uint8_t ad[4];
	int count = 0;

	if (pDev == nullptr || pDev->pIntrf == nullptr || pBuff == nullptr || Len <= 0)
	{
		return 0;
	}

	if (pDev->AddrLen > sizeof(ad))
	{
		return 0;
	}

	while (Len > 0 && Addr < pDev->Size)
	{
		int l = min(Len, (int)(pDev->Size - Addr));
		uint8_t devaddr = pDev->DevAddr;

		RFTagAddrEncode(pDev, Addr, ad);

		l = DeviceIntrfRead(pDev->pIntrf, devaddr, ad, pDev->AddrLen, pBuff, l);
		if (l <= 0)
		{
			break;
		}

		count += l;
		Addr += l;
		Len -= l;
		pBuff += l;
	}

	return count;
}

int RFTagWrite(RFTagDev_t * const pDev, uint32_t Addr, const uint8_t *pData, int Len)
{
	uint8_t ad[4];
	int count = 0;

	if (pDev == nullptr || pDev->pIntrf == nullptr || pData == nullptr || Len <= 0)
	{
		return 0;
	}

	if (pDev->AddrLen > sizeof(ad))
	{
		return 0;
	}

	while (Len > 0 && Addr < pDev->Size)
	{
		uint8_t devaddr = pDev->DevAddr;
		int l = min(Len, (int)(pDev->Size - Addr));

		if (pDev->PageSize > 0)
		{
			l = min(l, (int)(pDev->PageSize - (Addr % pDev->PageSize)));
		}

		RFTagAddrEncode(pDev, Addr, ad);

		l = DeviceIntrfWrite(pDev->pIntrf, devaddr, ad, pDev->AddrLen, pData, l);
		if (l <= 0)
		{
			break;
		}

		if (pDev->pWaitCB)
		{
			pDev->pWaitCB(devaddr, pDev->pIntrf);
		}
		else if (pDev->WrDelay > 0)
		{
			usDelay(pDev->WrDelay);
		}

		Addr += l;
		Len -= l;
		pData += l;
		count += l;
	}

	return count;
}

uint32_t RFTagGetSize(RFTagDev_t * const pDev)
{
	return pDev ? pDev->Size : 0;
}

uint16_t RFTagGetPageSize(RFTagDev_t * const pDev)
{
	return pDev ? pDev->PageSize : 0;
}

static bool RFTagSetNdefNLen16(RFTagDev_t * const pDev, const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[2];

	// Length prefix plus payload must fit the NDEF area
	if ((uint32_t)Len + sizeof(hdr) > pDev->NdefMaxLen)
	{
		return false;
	}

	hdr[0] = (uint8_t)(Len >> 8);
	hdr[1] = (uint8_t)Len;

	if (RFTagWrite(pDev, pDev->NdefAddr, hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		return false;
	}

	return RFTagWrite(pDev, pDev->NdefAddr + sizeof(hdr), pNdef, Len) == Len;
}

static bool RFTagSetNdefTlv(RFTagDev_t * const pDev, const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[4];
	uint32_t addr = pDev->NdefAddr;
	int hlen = 0;

	// Short form framing is 0x03, length, payload, 0xFE terminator
	if ((uint32_t)(Len + 3) > pDev->NdefMaxLen)
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
		if ((uint32_t)(Len + 5) > pDev->NdefMaxLen)
		{
			return false;
		}
		hdr[hlen++] = 0xFF;
		hdr[hlen++] = (uint8_t)(Len >> 8);
		hdr[hlen++] = (uint8_t)Len;
	}

	if (RFTagWrite(pDev, addr, hdr, hlen) != hlen)
	{
		return false;
	}

	addr += hlen;

	if (RFTagWrite(pDev, addr, pNdef, Len) != Len)
	{
		return false;
	}

	addr += Len;

	uint8_t term = 0xFE;

	return RFTagWrite(pDev, addr, &term, 1) == 1;
}

bool RFTagSetNdef(RFTagDev_t * const pDev, const uint8_t *pNdef, uint16_t Len)
{
	if (pDev == nullptr)
	{
		return false;
	}

	if (Len > 0 && pNdef == nullptr)
	{
		return false;
	}

	switch (pDev->NdefFmt)
	{
		case RFTAG_NDEF_FMT_NLEN16:
			return RFTagSetNdefNLen16(pDev, pNdef, Len);

		case RFTAG_NDEF_FMT_TLV:
			return RFTagSetNdefTlv(pDev, pNdef, Len);

		case RFTAG_NDEF_FMT_NONE:
		default:
			if ((uint32_t)Len > pDev->NdefMaxLen)
			{
				return false;
			}
			return RFTagWrite(pDev, pDev->NdefAddr, pNdef, Len) == Len;
	}
}

static int RFTagGetNdefNLen16(RFTagDev_t * const pDev, uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[2];

	if (RFTagRead(pDev, pDev->NdefAddr, hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		return 0;
	}

	uint16_t nlen = ((uint16_t)hdr[0] << 8) | hdr[1];

	if ((uint32_t)nlen + sizeof(hdr) > pDev->NdefMaxLen)
	{
		return 0;
	}

	if (Len < nlen)
	{
		nlen = Len;
	}

	return RFTagRead(pDev, pDev->NdefAddr + sizeof(hdr), pNdef, nlen);
}

static int RFTagGetNdefTlv(RFTagDev_t * const pDev, uint8_t *pNdef, uint16_t Len)
{
	uint32_t addr = pDev->NdefAddr;
	uint32_t end = pDev->NdefAddr + pDev->NdefMaxLen;
	uint8_t b = 0;

	while (addr < end)
	{
		if (RFTagRead(pDev, addr++, &b, 1) != 1)
		{
			return 0;
		}

		if (b == 0x00)
		{
			continue;
		}

		if (b == 0xFE)
		{
			return 0;
		}

		if (addr >= end)
		{
			return 0;
		}

		if (b != 0x03)
		{
			if (RFTagRead(pDev, addr++, &b, 1) != 1)
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

				if (RFTagRead(pDev, addr, lbuf, sizeof(lbuf)) != (int)sizeof(lbuf))
				{
					return 0;
				}
				addr += 2;
				skip = ((uint16_t)lbuf[0] << 8) | lbuf[1];
			}

			if (addr + skip > end)
			{
				return 0;
			}

			addr += skip;
			continue;
		}

		if (RFTagRead(pDev, addr++, &b, 1) != 1)
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

			if (RFTagRead(pDev, addr, lbuf, sizeof(lbuf)) != (int)sizeof(lbuf))
			{
				return 0;
			}
			addr += 2;
			nlen = ((uint16_t)lbuf[0] << 8) | lbuf[1];
		}

		if (addr + nlen > end)
		{
			return 0;
		}

		if (Len < nlen)
		{
			nlen = Len;
		}

		return RFTagRead(pDev, addr, pNdef, nlen);
	}

	return 0;
}

int RFTagGetNdef(RFTagDev_t * const pDev, uint8_t *pNdef, uint16_t Len)
{
	if (pDev == nullptr || pNdef == nullptr || Len == 0)
	{
		return 0;
	}

	switch (pDev->NdefFmt)
	{
		case RFTAG_NDEF_FMT_NLEN16:
			return RFTagGetNdefNLen16(pDev, pNdef, Len);

		case RFTAG_NDEF_FMT_TLV:
			return RFTagGetNdefTlv(pDev, pNdef, Len);

		case RFTAG_NDEF_FMT_NONE:
		default:
		{
			uint16_t l = min((uint32_t)Len, pDev->NdefMaxLen);
			return RFTagRead(pDev, pDev->NdefAddr, pNdef, l);
		}
	}
}

void RFTagSetWriteProt(RFTagDev_t * const pDev, bool bVal)
{
	if (pDev == nullptr)
	{
		return;
	}

	if (pDev->WrProtPin.PortNo < 0 || pDev->WrProtPin.PinNo < 0)
	{
		return;
	}

	if (bVal)
	{
		IOPinSet(pDev->WrProtPin.PortNo, pDev->WrProtPin.PinNo);
	}
	else
	{
		IOPinClear(pDev->WrProtPin.PortNo, pDev->WrProtPin.PinNo);
	}
}

void RFTagEvtDispatch(RFTagDev_t * const pDev, RFTAG_EVT Evt, uint32_t Addr, uint32_t Len, uint32_t Flags)
{
	if (pDev == nullptr || pDev->pEvtCB == nullptr)
	{
		return;
	}

	RFTagEvt_t e = {
		.Evt = Evt,
		.Addr = Addr,
		.Len = Len,
		.Flags = Flags,
	};

	pDev->pEvtCB(pDev->pCtx, &e);
}
