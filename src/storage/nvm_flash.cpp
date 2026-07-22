/**--------------------------------------------------------------------------
@file	nvm_flash.cpp

@brief	Serial NOR and QSPI flash driver on the NvmIO base.

@author	Hoang Nguyen Hoan
@date	Jul. 20, 2026

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
#include "coredev/spi.h"
#include "storage/nvm_flash.h"

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

// Status register write in progress and write enable latch bits.
#define NVM_FLASH_SR_WIP		0x01U
#define NVM_FLASH_SR_WEL		0x02U

// Status register block protect bits. The all set pattern protects the whole
// device; clearing them removes protection. BP0..BP3 cover the common layout.
#define NVM_FLASH_SR_BP_MASK	0x3CU

NvmFlash::NvmFlash()
{
	vpDevIntrf = nullptr;
	vDevSize = 0;
	vSectSize = 0;
	vBlkSize = 0;
	vPageSize = 0;
	vAddrSize = 3;
	vRdCmd = { FLASH_CMD_READ, 0 };
	vWrCmd = { FLASH_CMD_WRITE, 0 };
	vpInitCB = nullptr;
}

void NvmFlash::FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr)
{
	// Command byte then the address in big endian order on the wire.
	pFrame[0] = Cmd;
	for (int i = 0; i < vAddrSize; i++)
	{
		pFrame[1 + i] = (uint8_t)(Addr >> (8 * (vAddrSize - 1 - i)));
	}
}

void NvmFlash::SendSimpleCmd(uint8_t Cmd)
{
	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, Cmd, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		DeviceIntrfTx(vpDevIntrf, DeviceAddress(), &Cmd, 1);
	}
}

uint8_t NvmFlash::ReadStatus(void)
{
	uint8_t sr = 0;

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartRx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_READSTATUS, -1, 0, 1, 0);
		SPIRxData(dev, &sr, 1);
		SPIStopRx(dev);
	}
	else
	{
		uint8_t cmd = FLASH_CMD_READSTATUS;
		DeviceIntrfRead(vpDevIntrf, DeviceAddress(), &cmd, 1, &sr, 1);
	}

	return sr;
}

uint32_t NvmFlash::ReadId(int Len)
{
	uint32_t id = 0;

	if (Len > 4) { Len = 4; }

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartRx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_READID, -1, 0, Len, 0);
		SPIRxData(dev, (uint8_t*)&id, Len);
		SPIStopRx(dev);
	}
	else
	{
		uint8_t cmd = FLASH_CMD_READID;
		DeviceIntrfRead(vpDevIntrf, DeviceAddress(), &cmd, 1, (uint8_t*)&id, Len);
	}

	return id;
}

bool NvmFlash::WaitReady(uint32_t Timeout, uint32_t usRtyDelay)
{
	do {
		uint8_t sr = ReadStatus();
		if ((sr & NVM_FLASH_SR_WIP) == 0)
		{
			return true;
		}
		// A configured wait callback lets the caller do other work during a
		// long wait; otherwise optionally delay between polls.
		if (WaitPoll() == false)
		{
			return false;
		}
		if (usRtyDelay > 0)
		{
			usDelay(usRtyDelay);
		}
	} while (--Timeout > 0);

	return false;
}

void NvmFlash::WriteDisable(void)
{
	SendSimpleCmd(FLASH_CMD_WRDISABLE);
}

bool NvmFlash::WriteEnable(uint32_t Timeout)
{
	WaitReady(Timeout, 0);
	SendSimpleCmd(FLASH_CMD_WRENABLE);

	// Confirm the latch is set.
	do {
		if ((ReadStatus() & NVM_FLASH_SR_WEL) != 0)
		{
			return true;
		}
	} while (--Timeout > 0);

	return false;
}

bool NvmFlash::Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
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
	vSectSize = Cfg.EraseSize;
	vBlkSize = (Cfg.SectorSize != 0) ? Cfg.SectorSize : Cfg.EraseSize;
	vPageSize = Cfg.PageSize;
	vAddrSize = (Cfg.AddrSize != 0) ? Cfg.AddrSize : 3;
	vRdCmd = Cfg.RdCmd;
	vWrCmd = Cfg.WrCmd;
	vpInitCB = Cfg.pInitCB;

	// Validate geometry.
	if (vDevSize == 0 || vSectSize == 0 || vPageSize == 0 ||
		vAddrSize < 1 || vAddrSize > 4)
	{
		DEBUG_PRINTF("NvmFlash geometry invalid: size=%lu sect=%lu page=%lu addr=%d\r\n",
					 (unsigned long)vDevSize, (unsigned long)vSectSize,
					 (unsigned long)vPageSize, vAddrSize);
		return false;
	}

	// The base holds the mode and callbacks.
	SetMode(Cfg.bIntEn, Cfg.EvtHandler, Cfg.pWaitCB);

	// Enable 4 byte address mode where the device uses it.
	if (vAddrSize > 3)
	{
		SendSimpleCmd(FLASH_CMD_EN4B);
	}

	// Probe the JEDEC id where requested.
	if (Cfg.DevId != 0 && Cfg.DevIdSize > 0)
	{
		uint32_t id = ReadId(Cfg.DevIdSize);
		if (id != Cfg.DevId)
		{
			DEBUG_PRINTF("NvmFlash id mismatch: got 0x%lx want 0x%lx\r\n",
						 (unsigned long)id, (unsigned long)Cfg.DevId);
			return false;
		}
	}

	// Chip quirks, such as enabling quad mode.
	if (vpInitCB != nullptr)
	{
		if (vpInitCB(this, pIntrf) == false)
		{
			return false;
		}
	}

	// Set the region window. 0 size means to the end of the device.
	uint64_t rsize = (RegionSize != 0) ? RegionSize : (vDevSize - RegionOff);
	if (RegionOff + rsize > vDevSize)
	{
		return false;
	}
	Region(RegionOff, rsize);

	Valid(true);

	return true;
}

int NvmFlash::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	WaitReady();

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint8_t *pb = (uint8_t*)pBuf;

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartRx(dev, DeviceAddress());
		QuadSPISendCmd(dev, vRdCmd.Cmd, addr, vAddrSize, Len, vRdCmd.DummyCycle);
		SPIRxData(dev, pb, Len);
		SPIStopRx(dev);
	}
	else
	{
		// Command byte plus address as the read command frame, then the data.
		uint8_t frame[5];
		FrameAddr(frame, FLASH_CMD_READ, addr);
		int l = DeviceIntrfRead(vpDevIntrf, DeviceAddress(), frame,
								vAddrSize + 1, pb, Len);
		if (l <= 0)
		{
			return -EIO;
		}
	}

	return (int)Len;
}

int NvmFlash::Program(uint32_t Addr, const uint8_t *pData, uint32_t Len)
{
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, vWrCmd.Cmd, Addr, vAddrSize, Len, vWrCmd.DummyCycle);
		SPITxData(dev, (uint8_t*)pData, Len);
		SPIStopTx(dev);
	}
	else
	{
		uint8_t frame[5];
		FrameAddr(frame, FLASH_CMD_WRITE, Addr);

		DeviceIntrfStartTx(vpDevIntrf, DeviceAddress());
		DeviceIntrfTxData(vpDevIntrf, frame, vAddrSize + 1);
		DeviceIntrfTxData(vpDevIntrf, (uint8_t*)pData, Len);
		DeviceIntrfStopTx(vpDevIntrf);
	}

	// The write is complete only when the write in progress flag clears.
	if (WaitReady() == false)
	{
		return -EIO;
	}

	return (int)Len;
}

int NvmFlash::Write(uint64_t Off, const void *pData, uint32_t Len)
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

	// Split the write at page boundaries. The address counter auto increments
	// only within a page and wraps at the page boundary, so a single program
	// must stay within one page. Each chunk reissues the address for its page.
	while (cnt > 0)
	{
		uint32_t r = vPageSize - (addr % vPageSize);
		uint32_t l = cnt < r ? cnt : r;

		int res = Program(addr, p, l);
		if (res < 0)
		{
			return res;
		}
		cnt -= l;
		addr += l;
		p += l;
	}

	return (int)Len;
}

int NvmFlash::SectorErase(uint32_t Addr)
{
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_SECTOR_ERASE, Addr, vAddrSize, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		uint8_t frame[5];
		FrameAddr(frame, FLASH_CMD_SECTOR_ERASE, Addr);
		DeviceIntrfTx(vpDevIntrf, DeviceAddress(), frame, vAddrSize + 1);
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	return 0;
}

int NvmFlash::Erase(uint64_t Off, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	// Off and Len must be sector aligned.
	if ((Off % vSectSize) != 0 || (Len % vSectSize) != 0 || Len == 0)
	{
		return -EINVAL;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		int res = SectorErase(addr);
		if (res < 0)
		{
			return res;
		}
		addr += vSectSize;
		cnt -= vSectSize;
	}

	return 0;
}

int NvmFlash::SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable)
{
	(void)Off;
	(void)Len;

	// Read the current status, change only the block protect bits, write back.
	uint8_t sr = ReadStatus();
	if (bEnable)
	{
		sr |= NVM_FLASH_SR_BP_MASK;
	}
	else
	{
		sr &= (uint8_t)~NVM_FLASH_SR_BP_MASK;
	}

	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_WRSR, -1, 0, 1, 0);
		DeviceIntrfTxData(vpDevIntrf, &sr, 1);
		SPIStopTx(dev);
	}
	else
	{
		uint8_t frame[2] = { FLASH_CMD_WRSR, sr };
		DeviceIntrfTx(vpDevIntrf, DeviceAddress(), frame, 2);
	}

	if (WaitReady() == false)
	{
		return -EBUSY;
	}
	WriteDisable();

	return 0;
}

int NvmFlash::MassErase(void)
{
	if (RegionOffset() != 0 || Size() != vDevSize)
	{
		// The chip erase command wipes the whole device; only an instance
		// covering the whole device may use it.
		return -EPERM;
	}

	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	SendSimpleCmd(FLASH_CMD_BULK_ERASE);

	// A long wait, poll slowly.
	if (WaitReady((uint32_t)-1, 1000000) == false)
	{
		return -EBUSY;
	}
	WriteDisable();

	return 0;
}

bool NvmFlash::Enable(void)
{
	return true;
}

void NvmFlash::Disable(void)
{
	// No deep power down command is defined here; clear the write latch.
	SendSimpleCmd(FLASH_CMD_WRDISABLE);
}

void NvmFlash::Reset(void)
{
	SendSimpleCmd(FLASH_CMD_WRDISABLE);
}

void NvmFlash::PowerOff(void)
{
	Disable();
}
