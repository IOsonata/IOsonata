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
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#define NVM_SR_WIP			0x01U
#define NVM_SR_WEL			0x02U

#define NVM_CMD_READID		0x9FU
#define NVM_CMD_READ		0x03U
#define NVM_CMD_WRITE		0x02U
#define NVM_CMD_WRENABLE	0x06U
#define NVM_CMD_WRDISABLE	0x04U
#define NVM_CMD_RDSR		0x05U
#define NVM_CMD_WRSR		0x01U
#define NVM_CMD_SECT_ERASE	0x20U
#define NVM_CMD_BLK32_ERASE	0x52U
#define NVM_CMD_BLK64_ERASE	0xD8U
#define NVM_CMD_CHIP_ERASE	0xC7U
#define NVM_CMD_RESET_EN	0x66U
#define NVM_CMD_RESET		0x99U
#define NVM_CMD_EN4B		0xB7U

int Nvm::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	return Interface()->Read(DeviceAddress(), pCmdAddr, CmdAddrLen,
							 pBuff, BuffLen);
}

Nvm::Nvm()
{
	Valid(false);
	vRegionOffset = 0;
	vRegionSize = 0;
	vbIntEn = false;
	vEvtHandler = nullptr;
	vpWaitCB = nullptr;
	vInitCB = nullptr;
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
	vbBare = true;
	vbEnabled = false;
	vbIntrfEnabled = false;
	vBaseDevAddr = 0;
	vExpectedDevId = 0;
	vExpectedDevIdSize = 0;
	vWrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

int Nvm::FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				   uint32_t *pDevAddr)
{
	int len = 0;
	uint32_t devaddr = vBaseDevAddr;

	if (Cmd != 0)
	{
		pFrame[len++] = Cmd;
	}

	for (int i = 0; i < vAddrSize; i++)
	{
		pFrame[len++] = (uint8_t)(Addr >> (8 * (vAddrSize - 1 - i)));
	}

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

	return (Interface()->Tx(DeviceAddress(), &c, 1) == 1) ? 0 : -EIO;
}

int Nvm::ReadStatus(uint8_t &Status)
{
	Status = 0;

	if (vbBare)
	{
		return 0;
	}

	uint8_t cmd = NVM_CMD_RDSR;

	return Read(&cmd, 1, &Status, 1) == 1 ? 0 : -EIO;
}

uint8_t Nvm::ReadStatus(void)
{
	uint8_t sr = 0;

	if (Ready())
	{
		(void)ReadStatus(sr);
	}

	return sr;
}

uint32_t Nvm::ReadId(int Len)
{
	uint32_t id = 0;
	uint8_t cmd = NVM_CMD_READID;

	if (Len < 1 || Len > 4 || Interface() == nullptr || !vbEnabled)
	{
		return 0;
	}

	if (Read(&cmd, 1, (uint8_t*)&id, Len) != Len)
	{
		return 0;
	}

	return id;
}

bool Nvm::WaitReady(uint32_t Timeout)
{
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

	while (Timeout-- > 0)
	{
		uint8_t sr;

		if (ReadStatus(sr) != 0)
		{
			return false;
		}
		if ((sr & NVM_SR_WIP) == 0)
		{
			return true;
		}
		if (WaitPoll() == false)
		{
			return false;
		}
	}

	return false;
}

bool Nvm::WriteEnable(uint32_t Timeout)
{
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

	while (Timeout-- > 0)
	{
		uint8_t sr;

		if (ReadStatus(sr) != 0)
		{
			return false;
		}
		if ((sr & NVM_SR_WEL) != 0)
		{
			return true;
		}
		if (WaitPoll() == false)
		{
			return false;
		}
	}

	return false;
}

void Nvm::WriteDisable(void)
{
	if (vbBare == false && Interface() != nullptr && vbEnabled)
	{
		(void)SendCmd({ NVM_CMD_WRDISABLE, 0 });
	}
}

bool Nvm::ConfigureDevice(void)
{
	DeviceAddress(vBaseDevAddr);

	if (vInitCB != nullptr && vInitCB(this, Interface()) == false)
	{
		return false;
	}

	if (vEraseSize != 0)
	{
		if (SendCmd({ NVM_CMD_RESET_EN, 0 }) != 0 ||
			SendCmd({ NVM_CMD_RESET, 0 }) != 0)
		{
			return false;
		}
	}

	if (vExpectedDevId != 0 && vExpectedDevIdSize > 0)
	{
		bool found = false;

		for (int rtry = 0; rtry < 6; rtry++)
		{
			if (ReadId(vExpectedDevIdSize) == vExpectedDevId)
			{
				found = true;
				break;
			}
		}

		if (found == false)
		{
			DEBUG_PRINTF("Nvm id mismatch\r\n");
			return false;
		}
	}

	if (vEraseSize != 0 && vAddrSize > 3)
	{
		if (SendCmd({ NVM_CMD_EN4B, 0 }) != 0)
		{
			return false;
		}
	}

	return true;
}

bool Nvm::FailInit(void)
{
	if (vbEnabled)
	{
		WriteDisable();
	}
	vbEnabled = false;

	if (vbIntrfEnabled && Interface() != nullptr)
	{
		Interface()->Disable();
		vbIntrfEnabled = false;
	}

	Valid(false);
	Region(0, 0);
	return false;
}

bool Nvm::Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			   uint64_t RegionOff, uint64_t RegionSize)
{
	if (vbIntrfEnabled && Interface() != nullptr)
	{
		Interface()->Disable();
	}

	vbIntrfEnabled = false;
	vbEnabled = false;
	Valid(false);
	Region(0, 0);

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
	vExpectedDevId = Cfg.DevId;
	vExpectedDevIdSize = Cfg.DevIdSize;
	vInitCB = Cfg.pInitCB;
	vEvtHandler = Cfg.EvtHandler;
	vpWaitCB = Cfg.pWaitCB;
	vWrProtPin = Cfg.WrProtPin;

	switch (pIntrf->Type())
	{
		case DEVINTRF_TYPE_I2C:
		case DEVINTRF_TYPE_SPI:
		case DEVINTRF_TYPE_UNKOWN:
			break;

		case DEVINTRF_TYPE_QSPI:
		case DEVINTRF_TYPE_OSPI:
		default:
			return false;
	}

	if (Cfg.bIntEn)
	{
		return false;
	}

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

	if (vDevSize == 0 || vPageSize == 0 || vAddrSize < 1 || vAddrSize > 4 ||
		Cfg.DevIdSize > 4)
	{
		DEBUG_PRINTF("Nvm geometry invalid: size=%lu page=%lu addr=%d id=%u\r\n",
					 (unsigned long)vDevSize, (unsigned long)vPageSize,
					 vAddrSize, Cfg.DevIdSize);
		return false;
	}

	if (vEraseSize != 0 && vEraseSize != 4 * 1024 &&
		vEraseSize != 32 * 1024 && vEraseSize != 64 * 1024)
	{
		DEBUG_PRINTF("Nvm erase size unsupported: %lu\r\n",
					 (unsigned long)vEraseSize);
		return false;
	}

	vAddrSpan = (vAddrSize >= 4) ? 0 : (1UL << (8 * vAddrSize));
	if (vAddrSpan != 0 && vDevSize <= vAddrSpan)
	{
		vAddrSpan = 0;
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

	if ((RegionOff % vWrGran) != 0 || (vPageSize % vWrGran) != 0)
	{
		return false;
	}
	if (vEraseSize != 0 && (RegionOff % vEraseSize) != 0)
	{
		return false;
	}

	vbIntEn = false;

	pIntrf->Enable();
	vbIntrfEnabled = true;
	vbEnabled = true;

	if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
	{
		IOPinCfg(&vWrProtPin, 1);
		IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}

	if (ConfigureDevice() == false)
	{
		return FailInit();
	}

	Region(RegionOff, rsize);
	Valid(true);

	return true;
}

int Nvm::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (!Ready())
	{
		return Valid() ? -EACCES : -ENODEV;
	}
	if (!RangeValid(Off, Len) || (Len > 0 && pBuf == nullptr))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint8_t *pb = (uint8_t*)pBuf;
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		uint8_t frame[8];
		uint32_t devaddr;
		int flen = FrameAddr(frame, vRdCmd.Cmd, addr, &devaddr);
		uint32_t l = cnt;

		if (vAddrSpan != 0)
		{
			uint32_t r = vAddrSpan - (addr % vAddrSpan);
			if (l > r) { l = r; }
		}

		DeviceAddress(devaddr);
		int rd = Read(frame, flen, pb, (int)l);
		if (rd != (int)l)
		{
			return -EIO;
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
	if (wr != (int)Len)
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
	if (!Ready())
	{
		return Valid() ? -EACCES : -ENODEV;
	}
	if (!RangeValid(Off, Len) || (Len > 0 && pData == nullptr))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}
	if (vWrGran > 1 && ((Off % vWrGran) != 0 || (Len % vWrGran) != 0))
	{
		return -EINVAL;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	const uint8_t *p = (const uint8_t*)pData;
	uint32_t cnt = Len;

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
	if (!Ready())
	{
		return Valid() ? -EACCES : -ENODEV;
	}
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}

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
	if (!Ready())
	{
		return Valid() ? -EACCES : -ENODEV;
	}
	if (vEraseSize == 0 || Interface()->Type() == DEVINTRF_TYPE_UNKOWN)
	{
		return -ENOTSUP;
	}
	if (RegionOffset() != 0 || Size() != vDevSize)
	{
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
	if (!Ready())
	{
		return Valid() ? -EACCES : -ENODEV;
	}
	if (!RangeValid(Off, Len) || Len == 0)
	{
		return -EINVAL;
	}
	if (Off != 0 || (uint64_t)Len != Size())
	{
		return -ENOTSUP;
	}
	if (RegionOffset() != 0 || Size() != vDevSize)
	{
		return -EPERM;
	}

	if (vbBare || vWrProtMask == 0)
	{
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

	uint8_t sr;
	if (ReadStatus(sr) != 0)
	{
		return -EIO;
	}

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

	if (Device::Write(&wrsr, 1, &sr, 1) != 1)
	{
		WriteDisable();
		return -EIO;
	}

	if (WaitReady() == false)
	{
		WriteDisable();
		return -EBUSY;
	}
	WriteDisable();

	return 0;
}

bool Nvm::Enable(void)
{
	if (!Valid() || Interface() == nullptr)
	{
		return false;
	}
	if (vbEnabled)
	{
		return true;
	}

	Interface()->Enable();
	vbIntrfEnabled = true;
	vbEnabled = true;
	return true;
}

void Nvm::Disable(void)
{
	if (!vbEnabled)
	{
		return;
	}

	(void)Sync();
	WriteDisable();
	vbEnabled = false;

	if (vbIntrfEnabled && Interface() != nullptr)
	{
		Interface()->Disable();
		vbIntrfEnabled = false;
	}
}

void Nvm::Reset(void)
{
	if (!Ready())
	{
		return;
	}

	(void)Sync();
	WriteDisable();
	Valid(false);

	if (ConfigureDevice())
	{
		Valid(true);
	}
}
