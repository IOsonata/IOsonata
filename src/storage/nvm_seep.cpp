/**-------------------------------------------------------------------------
@file	nvm_seep.cpp

@brief	Serial I2C EEPROM driver on the NvmIO base.

@author	Hoang Nguyen Hoan
@date	Jul. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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

#include "idelay.h"
#include "iopinctrl.h"
#include "storage/nvm_seep.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

NvmSeep::NvmSeep()
{
	vpDevIntrf = nullptr;
	vDevSize = 0;
	vPageSize = 0;
	vAddrSize = 1;
	vWrDelayUs = 0;
	vWrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

uint8_t NvmSeep::FrameAddr(uint8_t *pAd, uint32_t Addr)
{
	uint8_t devaddr = (uint8_t)DeviceAddress();

	// Memory address MSB first.
	for (int i = 0; i < vAddrSize; i++)
	{
		pAd[i] = (uint8_t)(Addr >> (8 * (vAddrSize - 1 - i)));
	}

	// For a part that uses device address block select, the address bits above
	// the memory address width select the memory block through the low device
	// address bits.
	uint32_t shift = (uint32_t)vAddrSize << 3;
	uint32_t admask = (shift >= 32) ? 0 : (0xFFFFFFFFU << shift);
	if (Addr & admask)
	{
		devaddr |= (uint8_t)((Addr >> shift) & 7);
	}

	return devaddr;
}

bool NvmSeep::Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
				   uint64_t RegionOff, uint64_t RegionSize)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	vpDevIntrf = *pIntrf;
	Interface(pIntrf);
	DeviceAddress(Cfg.DevNo);

	vDevSize = Cfg.TotalSize;
	vPageSize = Cfg.PageSize;
	vAddrSize = (Cfg.AddrSize != 0) ? Cfg.AddrSize : 1;
	vWrDelayUs = Cfg.WriteDelayUs;
	vWrProtPin = Cfg.WrProtPin;

	if (vDevSize == 0 || vPageSize == 0 || vAddrSize < 1 || vAddrSize > 4)
	{
		DEBUG_PRINTF("NvmSeep geometry invalid: size=%lu page=%lu addr=%d\r\n",
					 (unsigned long)vDevSize, (unsigned long)vPageSize,
					 vAddrSize);
		return false;
	}

	// The base holds the mode and callbacks.
	SetMode(Cfg.bIntEn, Cfg.EvtHandler, Cfg.pWaitCB);

	// Configure the write protect pin and start unprotected.
	if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
	{
		IOPinCfg(&vWrProtPin, 1);
		IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}

	uint64_t rsize = (RegionSize != 0) ? RegionSize : (vDevSize - RegionOff);
	if (RegionOff + rsize > vDevSize)
	{
		return false;
	}
	Region(RegionOff, rsize);

	Valid(true);

	return true;
}

int NvmSeep::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint8_t *pb = (uint8_t*)pBuf;
	uint32_t cnt = Len;

	// Read may span the whole region in one transfer; split only at the block
	// select boundary so the device address stays correct for each block.
	while (cnt > 0)
	{
		uint8_t ad[4];
		uint8_t devaddr = FrameAddr(ad, addr);

		// Bytes remaining before the next block select boundary.
		uint32_t blk = 1u << ((uint32_t)vAddrSize << 3);
		uint32_t r = blk - (addr & (blk - 1));
		uint32_t l = cnt < r ? cnt : r;

		int rd = DeviceIntrfRead(vpDevIntrf, devaddr, ad, vAddrSize, pb, l);
		if (rd <= 0)
		{
			return (cnt == Len) ? -EIO : (int)(Len - cnt);
		}
		addr += rd;
		pb += rd;
		cnt -= rd;
	}

	return (int)Len;
}

int NvmSeep::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	const uint8_t *p = (const uint8_t*)pData;
	uint32_t cnt = Len;

	// Split the write at page boundaries. A write must not cross a page; each
	// page write is followed by the device write cycle time.
	while (cnt > 0)
	{
		uint8_t ad[4];
		uint8_t devaddr = FrameAddr(ad, addr);

		uint32_t r = vPageSize - (addr % vPageSize);
		uint32_t l = cnt < r ? cnt : r;

		int wr = DeviceIntrfWrite(vpDevIntrf, devaddr, ad, vAddrSize,
								  (uint8_t*)p, l);
		if (wr <= 0)
		{
			return (cnt == Len) ? -EIO : (int)(Len - cnt);
		}

		// Wait the write cycle: the cooperative wait callback if set, else the
		// configured delay.
		if (WaitPoll() == false)
		{
			return (int)(Len - cnt);
		}
		if (vWrDelayUs > 0)
		{
			usDelay(vWrDelayUs);
		}

		addr += wr;
		p += wr;
		cnt -= wr;
	}

	return (int)Len;
}

int NvmSeep::SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable)
{
	(void)Off;
	(void)Len;

	if (vWrProtPin.PortNo < 0 || vWrProtPin.PinNo < 0)
	{
		return -ENOTSUP;
	}

	// The pin protects the whole device; a high level enables protection.
	if (bEnable)
	{
		IOPinSet(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}
	else
	{
		IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}

	return 0;
}
