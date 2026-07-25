/**-------------------------------------------------------------------------
@file	nvm.cpp

@brief	Serial non volatile memory driver.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

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

#include "idelay.h"
#include "iopinctrl.h"
#include "storage/nvm.h"

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

// Status register bits common to serial flash.
#define NVM_SR_WIP			0x01U		// write in progress
#define NVM_SR_WEL			0x02U		// write enable latch

// Read the device id where the medium has the usual JEDEC command.
#define NVM_CMD_READID		0x9FU

// The serial NOR protocol the JEDEC standard fixed. An erase medium speaks
// all of it; none of it is configuration, because none of it varies.
#define NVM_CMD_READ		0x03U
#define NVM_CMD_WRITE		0x02U
#define NVM_CMD_WRENABLE	0x06U
#define NVM_CMD_WRDISABLE	0x04U
#define NVM_CMD_RDSR		0x05U
#define NVM_CMD_WRSR		0x01U
#define NVM_CMD_SECT_ERASE	0x20U		// 4K granule
#define NVM_CMD_BLK32_ERASE	0x52U		// 32K granule
#define NVM_CMD_BLK64_ERASE	0xD8U		// 64K granule
#define NVM_CMD_CHIP_ERASE	0xC7U
#define NVM_CMD_RESET_EN	0x66U
#define NVM_CMD_RESET		0x99U
#define NVM_CMD_EN4B		0xB7U

int Nvm::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	// Straight through; a memory command is not a sensor register address.
	return Interface()->Read(DeviceAddress(), pCmdAddr, CmdAddrLen,
							 pBuff, BuffLen);
}

Nvm::Nvm()
{
	vRegionOffset = 0;
	vRegionSize = 0;
	vbIntEn = false;
	vEvtHandler = nullptr;
	vpWaitCB = nullptr;
	vDevSize = 0;
	vSectSize = 0;
	vEraseSize = 0;
	vPageSize = 0;
	vWrGran = 1;
	vAddrSize = 3;
	vAddrSpan = 0;
	vWrDelayUs = 0;
	vWrProtMask = 0;
	memset(&vRdCmd, 0, sizeof(vRdCmd));
	memset(&vWrCmd, 0, sizeof(vWrCmd));
	vWrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

int Nvm::FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				   uint32_t *pDevAddr)
{
	int len = 0;
	uint32_t devaddr = vBaseDevAddr;

	// A command byte where the medium has one. An EEPROM has none and the
	// frame is the address alone.
	if (Cmd != 0)
	{
		pFrame[len++] = Cmd;
	}

	// Address MSB first on the wire.
	for (int i = 0; i < vAddrSize; i++)
	{
		pFrame[len++] = (uint8_t)(Addr >> (8 * (vAddrSize - 1 - i)));
	}

	// Bits the address bytes cannot hold go into the device selection, the way
	// a small EEPROM selects a memory block.
	if (vAddrSpan != 0 && Addr >= vAddrSpan)
	{
		devaddr |= (Addr / vAddrSpan) & 7;
	}

	if (pDevAddr != nullptr)
	{
		*pDevAddr = devaddr;
	}

	return len;
}

int Nvm::SendCmd(const NvmCmd_t &Cmd)
{
	if (Cmd.Cmd == 0)
	{
		return 0;
	}

	uint8_t c = Cmd.Cmd;

	// A command only transaction has no payload, so Device::Write's payload
	// count is zero whether it worked or not. Only the full transfer count
	// distinguishes the two.
	return (Interface()->Tx(DeviceAddress(), &c, 1) == 1) ? 0 : -EIO;
}

uint8_t Nvm::ReadStatus(void)
{
	if (vbBare)
	{
		return 0;
	}

	uint8_t cmd = NVM_CMD_RDSR;
	uint8_t sr = 0;

	Read(&cmd, 1, &sr, 1);

	return sr;
}

uint32_t Nvm::ReadId(int Len)
{
	uint32_t id = 0;
	uint8_t cmd = NVM_CMD_READID;

	if (Len > 4) { Len = 4; }

	Read(&cmd, 1, (uint8_t*)&id, Len);

	return id;
}

bool Nvm::WaitReady(uint32_t Timeout)
{
	// A medium with a status register says when it is done. One without takes
	// a known time, which the config gives.
	if (vbBare)
	{
		if (WaitPoll() == false)
		{
			return false;
		}
		if (vWrDelayUs > 0)
		{
			usDelay(vWrDelayUs);
		}
		return true;
	}

	do {
		if ((ReadStatus() & NVM_SR_WIP) == 0)
		{
			return true;
		}
		if (WaitPoll() == false)
		{
			return false;
		}
	} while (--Timeout > 0);

	return false;
}

bool Nvm::WriteEnable(uint32_t Timeout)
{
	// A bare bus has no latch: always writable.
	if (vbBare)
	{
		return true;
	}

	if (WaitReady(Timeout) == false)
	{
		return false;
	}
	if (SendCmd({ NVM_CMD_WRENABLE, 0 }) != 0)
	{
		return false;
	}

	do {
		if ((ReadStatus() & NVM_SR_WEL) != 0)
		{
			return true;
		}
	} while (--Timeout > 0);

	return false;
}

void Nvm::WriteDisable(void)
{
	if (vbBare == false)
	{
		SendCmd({ NVM_CMD_WRDISABLE, 0 });
	}
}

bool Nvm::Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			   uint64_t RegionOff, uint64_t RegionSize)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(Cfg.DevNo);

	vDevSize = Cfg.TotalSize;
	vEraseSize = Cfg.EraseSize;
	vSectSize = Cfg.SectorSize;
	vPageSize = Cfg.PageSize;
	vWrGran = (Cfg.WriteGran != 0) ? Cfg.WriteGran : 1;
	vAddrSize = (Cfg.AddrSize != 0) ? Cfg.AddrSize : 3;
	vBaseDevAddr = (uint32_t)Cfg.DevNo;
	vWrDelayUs = Cfg.WriteDelayUs;
	vWrProtMask = Cfg.WrProtMask;
	// Almost no command is configuration: the bus and the kind decide them.
	// An I2C memory takes a bare address and no command byte, and waits by
	// WriteDelayUs. Everything serial speaks the standard protocol, which
	// SPI EEPROMs and FRAMs speak just as a NOR does, and which the
	// internal memory interface emulates. The one exception is the read and
	// program pair, where a wide bus part has its own command and dummy
	// cycles: zero derives the plain bus standard, set overrides.
	// The buses this driver understands: I2C takes bare addresses, the SPI
	// family speaks the serial protocol, and UNKNOWN is a command emulating
	// adapter such as the internal memory interface. Anything else is not a
	// memory bus.
	switch (pIntrf->Type())
	{
		case DEVINTRF_TYPE_I2C:
		case DEVINTRF_TYPE_SPI:
		case DEVINTRF_TYPE_QSPI:
		case DEVINTRF_TYPE_OSPI:
		case DEVINTRF_TYPE_UNKOWN:
			break;

		default:
			return false;
	}

	// The bus is one of the two facts every command follows from; the kind,
	// from EraseSize, is the other. Nothing else is stored: each operation
	// decides its command where it runs.
	vbBare = (pIntrf->Type() == DEVINTRF_TYPE_I2C);

	if (vbBare)
	{
		vRdCmd = Cfg.RdCmd;
		vWrCmd = Cfg.WrCmd;
	}
	else
	{
		vRdCmd = (Cfg.RdCmd.Cmd != 0) ? Cfg.RdCmd
									  : NvmCmd_t{ NVM_CMD_READ, 0 };
		vWrCmd = (Cfg.WrCmd.Cmd != 0) ? Cfg.WrCmd
									  : NvmCmd_t{ NVM_CMD_WRITE, 0 };
	}
	vWrProtPin = Cfg.WrProtPin;

	if (vDevSize == 0 || vPageSize == 0 || vAddrSize < 1 || vAddrSize > 4)
	{
		DEBUG_PRINTF("Nvm geometry invalid: size=%lu page=%lu addr=%d\r\n",
					 (unsigned long)vDevSize, (unsigned long)vPageSize,
					 vAddrSize);
		return false;
	}

	// Only these standard erase commands are implemented. Accepting another
	// granule would issue a 4K command while advancing by the configured size.
	if (vEraseSize != 0 && vEraseSize != 4 * 1024 &&
		vEraseSize != 32 * 1024 && vEraseSize != 64 * 1024)
	{
		DEBUG_PRINTF("Nvm erase size unsupported: %lu\r\n",
					 (unsigned long)vEraseSize);
		return false;
	}

	// What the address bytes can reach. Anything beyond has to go into the
	// device selection.
	vAddrSpan = (vAddrSize >= 4) ? 0 : (1UL << (8 * vAddrSize));
	if (vAddrSpan != 0 && vDevSize <= vAddrSpan)
	{
		vAddrSpan = 0;			// the address bytes cover the whole device
	}

	vbIntEn = Cfg.bIntEn;
	vEvtHandler = Cfg.EvtHandler;
	vpWaitCB = Cfg.pWaitCB;

	// Configure the write protect pin and start unprotected.
	if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
	{
		IOPinCfg(&vWrProtPin, 1);
		IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}

	// Chip quirks first, such as releasing a power down state or selecting a
	// sector architecture, before the driver talks to the device.
	if (Cfg.pInitCB != nullptr)
	{
		if (Cfg.pInitCB(this, pIntrf) == false)
		{
			return false;
		}
	}

	// Reset an erase medium. The chip does not reset when the MCU does:
	// across a soft reset or a debugger restart it can be left in power
	// down, in a continuous read or quad mode where commands are misread,
	// in 4 byte address mode, or mid operation. Reset is the one command
	// honored from any of those states, so it goes before anything that
	// assumes a sane device.
	if (vEraseSize != 0)
	{
		if (SendCmd({ NVM_CMD_RESET_EN, 0 }) != 0 ||
			SendCmd({ NVM_CMD_RESET, 0 }) != 0)
		{
			return false;
		}
	}

	// Probe the device id where the config asked for it. The retries cover
	// the device's recovery time after the reset above.
	if (Cfg.DevId != 0 && Cfg.DevIdSize > 0)
	{
		int rtry = 5;

		do {
			if (ReadId(Cfg.DevIdSize) == Cfg.DevId)
			{
				break;
			}
		} while (rtry-- > 0);

		if (rtry <= 0)
		{
			DEBUG_PRINTF("Nvm id mismatch\r\n");
			return false;
		}
	}

	// A part larger than 16 MBytes takes its addresses in 4 bytes; the
	// reset above cleared the mode, so it is entered last.
	if (vEraseSize != 0 && vAddrSize > 3)
	{
		if (SendCmd({ NVM_CMD_EN4B, 0 }) != 0)
		{
			return false;
		}
	}

	if (RegionOff > vDevSize)
	{
		return false;
	}

	uint64_t avail = vDevSize - RegionOff;

	if (RegionSize > avail)
	{
		return false;
	}

	uint64_t rsize = (RegionSize != 0) ? RegionSize : avail;

	// The physical placement must respect the write granularity, or a frame
	// starting mid word would silently lose its tail; and an erase region
	// must start on an erase unit.
	if ((RegionOff % vWrGran) != 0 || (vPageSize % vWrGran) != 0)
	{
		return false;
	}
	if (vEraseSize != 0 && (RegionOff % vEraseSize) != 0)
	{
		return false;
	}
	Region(RegionOff, rsize);

	Valid(true);

	return true;
}

int Nvm::Read(uint64_t Off, void *pBuf, uint32_t Len)
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
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		uint8_t frame[8];
		uint32_t devaddr;
		int flen = FrameAddr(frame, vRdCmd.Cmd, addr, &devaddr);

		// Split only where the device selection holds part of the address,
		// so a read does not run past what this selection reaches.
		uint32_t l = cnt;
		if (vAddrSpan != 0)
		{
			uint32_t r = vAddrSpan - (addr % vAddrSpan);
			if (l > r) { l = r; }
		}

		DeviceAddress(devaddr);
		int rd = Read(frame, flen, pb, (int)l);
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

int Nvm::Program(uint32_t Addr, const uint8_t *pData, uint32_t Len)
{
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	uint8_t frame[8];
	uint32_t devaddr;
	int flen = FrameAddr(frame, vWrCmd.Cmd, Addr, &devaddr);

	DeviceAddress(devaddr);
	int wr = Device::Write(frame, flen, pData, (int)Len);
	// The interface reports the whole transfer, frame included; the caller
	// only wants to know how much of its data went.
	if (wr < (int)Len)
	{
		return -EIO;
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	return (int)Len;
}

int Nvm::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}
	// A memory that programs whole words cannot take a part of one.
	if (vWrGran > 1 && ((Off % vWrGran) != 0 || (Len % vWrGran) != 0))
	{
		return -EINVAL;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	const uint8_t *p = (const uint8_t*)pData;
	uint32_t cnt = Len;

	// Split at page boundaries. The address counter auto increments only
	// within a page and wraps at the boundary, so one transfer must stay
	// inside a page. Each chunk reissues the address for its page.
	while (cnt > 0)
	{
		uint32_t r = vPageSize - (addr % vPageSize);
		uint32_t l = cnt < r ? cnt : r;

		int res = Program(addr, p, l);
		if (res < 0)
		{
			return res;
		}
		cnt -= res;
		addr += res;
		p += res;
	}

	return (int)Len;
}

int Nvm::EraseUnit(uint32_t Addr)
{
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	uint8_t op = NVM_CMD_SECT_ERASE;
	if (vEraseSize == 32 * 1024) { op = NVM_CMD_BLK32_ERASE; }
	else if (vEraseSize == 64 * 1024) { op = NVM_CMD_BLK64_ERASE; }

	uint8_t frame[8];
	uint32_t devaddr;
	int flen = FrameAddr(frame, op, Addr, &devaddr);

	DeviceAddress(devaddr);
	if (Interface()->Tx(devaddr, frame, flen) != flen)
	{
		return -EIO;
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	return 0;
}

int Nvm::Erase(uint64_t Off, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}

	// A medium that overwrites directly has nothing to erase.
	if (vEraseSize == 0)
	{
		return 0;
	}

	if ((Off % vEraseSize) != 0 || (Len % vEraseSize) != 0 || Len == 0)
	{
		return -EINVAL;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		int res = EraseUnit(addr);
		if (res < 0)
		{
			return res;
		}
		addr += vEraseSize;
		cnt -= vEraseSize;
	}

	return 0;
}

int Nvm::MassErase(void)
{
	// The internal memory adapter reports UNKNOWN because it emulates the
	// serial command frame. It supports unit erase, but never chip erase of the
	// device executing this code.
	if (vEraseSize == 0 || Interface()->Type() == DEVINTRF_TYPE_UNKOWN)
	{
		return -ENOTSUP;
	}
	if (RegionOffset() != 0 || Size() != vDevSize)
	{
		// The command wipes the whole device, so only an instance covering all
		// of it may issue one.
		return -EPERM;
	}

	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	if (SendCmd({ NVM_CMD_CHIP_ERASE, 0 }) != 0)
	{
		return -EIO;
	}

	if (WaitReady((uint32_t)-1) == false)
	{
		return -EBUSY;
	}
	WriteDisable();

	return 0;
}

int Nvm::SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable)
{
	(void)Off;
	(void)Len;

	// Block protect bits where the medium has them. Checked first because a
	// config left at zero would otherwise look like it had a pin on port 0.
	if (vbBare || vWrProtMask == 0)
	{
		// No status bits, so a pin if one is configured.
		if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
		{
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

		return -ENOTSUP;
	}

	uint8_t sr = ReadStatus();

	if (bEnable)
	{
		sr |= vWrProtMask;
	}
	else
	{
		sr &= (uint8_t)~vWrProtMask;
	}

	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	uint8_t wrsr = NVM_CMD_WRSR;

	Device::Write(&wrsr, 1, &sr, 1);

	if (WaitReady() == false)
	{
		return -EBUSY;
	}
	WriteDisable();

	return 0;
}

void Nvm::Disable(void)
{
	WriteDisable();
}

void Nvm::Reset(void)
{
	WriteDisable();
}
