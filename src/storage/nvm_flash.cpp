/**-------------------------------------------------------------------------
@file	nvm_flash.cpp

@brief	Serial NOR Flash implementation of the NvmIO device class.

Absorbs the former storage/flash.cpp C driver. Functional differences from
that driver: byte addressed Read/Write over a region window instead of whole
sector transfers, errno error propagation, WriteEnable confirms the latch,
and the Device lifecycle maps Enable/Disable to deep power down.

@author	Hoang Nguyen Hoan
@date	Jul 19, 2026

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
#include <errno.h>
#include <string.h>

#include "istddef.h"
#include "idelay.h"
#include "coredev/spi.h"
#include "storage/nvm_flash.h"

NvmFlash::NvmFlash()
{
	vpDevIntrf = nullptr;
	vDevSize = 0;
	vSectSize = 0;
	vBlkSize = 0;
	vPageSize = 0;
	vAddrSize = 0;
	vRdCmd = {0, 0};
	vWrCmd = {0, 0};
	vpWaitCB = nullptr;
}

bool NvmFlash::Init(const FlashCfg_t &Cfg, DeviceIntrf * const pIntrf,
					uint64_t RegionOffset, uint64_t RegionSize)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	Interface(pIntrf);
	vpDevIntrf = *pIntrf;
	DeviceAddress(Cfg.DevNo);

	if (Cfg.pInitCB)
	{
		if (Cfg.pInitCB(Cfg.DevNo, vpDevIntrf) == false)
		{
			return false;
		}
	}

	vpWaitCB = Cfg.pWaitCB;
	vSectSize = Cfg.SectSize;
	vBlkSize = Cfg.BlkSize;
	vPageSize = Cfg.PageSize;
	vAddrSize = Cfg.AddrSize;
	vRdCmd = Cfg.RdCmd;
	vWrCmd = Cfg.WrCmd;
	vDevSize = (uint64_t)Cfg.TotalSize * 1024ULL;	// TotalSize is in KBytes

	if (RegionSize == 0)
	{
		RegionSize = vDevSize - RegionOffset;
	}
	if (vSectSize == 0 || vPageSize == 0 ||
		RegionOffset + RegionSize > vDevSize ||
		(RegionOffset % vSectSize) != 0 || (RegionSize % vSectSize) != 0)
	{
		return false;
	}
	Region(RegionOffset, RegionSize);

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		QuadSPISetMemSize(SPIGetHandle(vpDevIntrf), Cfg.TotalSize);
	}

	Reset();

	if (Cfg.DevIdSize > 0 && Cfg.DevId != 0)
	{
		int rtry = 5;

		do {
			uint32_t d = ReadId(Cfg.DevIdSize);
			if (d == Cfg.DevId)
			{
				DeviceID(d);
				break;
			}
		} while (rtry-- > 0);

		if (rtry <= 0)
		{
			return false;
		}
	}

	if (vAddrSize > 3)
	{
		// Enable 4 bytes address
		if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
		{
			SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

			SPIStartRx(dev, DeviceAddress());
			QuadSPISendCmd(dev, FLASH_CMD_EN4B, -1, 0, 0, 0);
			SPIStopRx(dev);
		}
		else
		{
			SendSimpleCmd(FLASH_CMD_EN4B);
		}
	}

	Valid(true);

	return true;
}

void NvmFlash::SendSimpleCmd(uint8_t Cmd)
{
	DeviceIntrfTx(vpDevIntrf, DeviceAddress(), &Cmd, 1);
}

uint32_t NvmFlash::ReadId(int Len)
{
	uint32_t id = -1;

	if (Len > 0)
	{
		id = 0;

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

			DeviceIntrfRead(vpDevIntrf, DeviceAddress(), &cmd, 1,
							(uint8_t*)&id, Len);
		}
	}

	return id;
}

uint8_t NvmFlash::ReadStatus(void)
{
	uint8_t d = 0;

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartRx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_READSTATUS, -1, 0, 1, 0);
		SPIRxData(dev, &d, 1);
		SPIStopRx(dev);
	}
	else
	{
		uint8_t cmd = FLASH_CMD_READSTATUS;

		DeviceIntrfRead(vpDevIntrf, DeviceAddress(), &cmd, 1, &d, 1);
	}

	return d;
}

bool NvmFlash::WaitReady(uint32_t Timeout, uint32_t usRtyDelay)
{
	do {
		uint8_t d = ReadStatus();

		if (!(d & FLASH_STATUS_WIP))
		{
			return true;
		}

		if (usRtyDelay > 0)
		{
			if (vpWaitCB)
			{
				vpWaitCB(DeviceAddress(), vpDevIntrf);
			}
			else
			{
				usDelay(usRtyDelay);
			}
		}
	} while (Timeout-- > 0);

	return false;
}

bool NvmFlash::WriteEnable(uint32_t Timeout)
{
	if (WaitReady(Timeout, 0) == false)
	{
		return false;
	}

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_WRENABLE, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		SendSimpleCmd(FLASH_CMD_WRENABLE);
	}

	// Confirm the latch. Some devices need time or reject the command while
	// another operation completes.
	do {
		if (ReadStatus() & FLASH_STATUS_WEL)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

void NvmFlash::WriteDisable(void)
{
	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_WRDISABLE, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		SendSimpleCmd(FLASH_CMD_WRDISABLE);
	}
}

void NvmFlash::Reset(void)
{
	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_RESET_ENABLE, -1, 0, 0, 0);
		SPIStopTx(dev);
		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_RESET_DEVICE, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		SendSimpleCmd(FLASH_CMD_RESET_ENABLE);
		SendSimpleCmd(FLASH_CMD_RESET_DEVICE);
	}
}

bool NvmFlash::Enable(void)
{
	DeviceIntrfEnable(vpDevIntrf);

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_RDPD, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		SendSimpleCmd(FLASH_CMD_RDPD);
	}

	// Deep power down release time
	usDelay(50);

	return true;
}

void NvmFlash::Disable(void)
{
	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_DPD, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		SendSimpleCmd(FLASH_CMD_DPD);
	}

	DeviceIntrfDisable(vpDevIntrf);
}

void NvmFlash::PowerOff(void)
{
	DeviceIntrfPowerOff(vpDevIntrf);
}

int NvmFlash::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (pBuf == nullptr || RangeValid(Off, Len) == false)
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint8_t *p = (uint8_t*)pBuf;
	int cnt = Len;

	// Make sure there is no write access pending
	if (WaitReady(100000, 0) == false)
	{
		return -EBUSY;
	}

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartRx(dev, DeviceAddress());
		QuadSPISendCmd(dev, vRdCmd.Cmd, addr, vAddrSize, Len,
					   vRdCmd.DummyCycle);
		SPIRxData(dev, p, Len);
		SPIStopRx(dev);
	}
	else
	{
		uint8_t d[9];
		uint8_t *pa = (uint8_t*)&addr;

		d[0] = FLASH_CMD_READ;

		while (cnt > 0)
		{
			for (int i = 1; i <= vAddrSize; i++)
			{
				d[i] = pa[vAddrSize - i];
			}

			DeviceIntrfStartRx(vpDevIntrf, DeviceAddress());
			DeviceIntrfTxData(vpDevIntrf, d, vAddrSize + 1);
			int l = DeviceIntrfRxData(vpDevIntrf, p, cnt);
			DeviceIntrfStopRx(vpDevIntrf);
			if (l <= 0)
			{
				return -EIO;
			}
			cnt -= l;
			addr += l;
			p += l;
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
		int cnt = Len;

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, vWrCmd.Cmd, Addr, vAddrSize, Len,
					   vWrCmd.DummyCycle);
		while (cnt > 0)
		{
			int l = SPITxData(dev, pData, cnt);
			if (l <= 0)
			{
				SPIStopTx(dev);
				return -EIO;
			}
			cnt -= l;
			pData += l;
		}
		SPIStopTx(dev);
	}
	else
	{
		uint8_t d[9];
		uint8_t *pa = (uint8_t*)&Addr;
		int cnt = Len;

		d[0] = FLASH_CMD_WRITE;
		for (int i = 1; i <= vAddrSize; i++)
		{
			d[i] = pa[vAddrSize - i];
		}

		DeviceIntrfStartTx(vpDevIntrf, DeviceAddress());
		DeviceIntrfTxData(vpDevIntrf, d, vAddrSize + 1);
		while (cnt > 0)
		{
			int l = DeviceIntrfTxData(vpDevIntrf, pData, cnt);
			if (l <= 0)
			{
				DeviceIntrfStopTx(vpDevIntrf);
				return -EIO;
			}
			cnt -= l;
			pData += l;
		}
		DeviceIntrfStopTx(vpDevIntrf);
	}

	return (int)Len;
}

int NvmFlash::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (pData == nullptr || RangeValid(Off, Len) == false)
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

	// Program page chunk by page chunk in ascending address order. A chunk
	// never crosses a page boundary.
	while (cnt > 0)
	{
		uint32_t r = vPageSize - (addr % vPageSize);
		uint32_t l = cnt < r ? cnt : r;

		int res = Program(addr, p, l);
		if (res < 0)
		{
			WriteDisable();
			return res;
		}
		cnt -= l;
		addr += l;
		p += l;
	}

	WriteDisable();

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
		uint8_t cmd = vAddrSize > 3 ?
					  FLASH_CMD_SECTOR_ERASE_4B : FLASH_CMD_SECTOR_ERASE;

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, cmd, Addr, vAddrSize, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		uint8_t d[8];
		uint8_t *pa = (uint8_t*)&Addr;

		d[0] = vAddrSize > 3 ?
			   FLASH_CMD_SECTOR_ERASE_4B : FLASH_CMD_SECTOR_ERASE;
		for (int i = 1; i <= vAddrSize; i++)
		{
			d[i] = pa[vAddrSize - i];
		}
		if (DeviceIntrfTx(vpDevIntrf, DeviceAddress(), d, vAddrSize + 1) <= 0)
		{
			return -EIO;
		}
	}

	return 0;
}

int NvmFlash::Erase(uint64_t Off, uint32_t Len)
{
	if (Len == 0 || RangeValid(Off, Len) == false ||
		(Off % vSectSize) != 0 || (Len % vSectSize) != 0)
	{
		return -EINVAL;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint32_t cnt = Len / vSectSize;

	for (uint32_t k = 0; k < cnt; k++)
	{
		if (WaitReady(-1, 100) == false)
		{
			return -EBUSY;
		}

		int res = SectorErase(addr);
		if (res != 0)
		{
			return res;
		}
		addr += vSectSize;
	}

	if (WaitReady(-1, 100) == false)
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

	if (vpDevIntrf->Type == DEVINTRF_TYPE_QSPI)
	{
		SPIDev_t *dev = SPIGetHandle(vpDevIntrf);

		SPIStartTx(dev, DeviceAddress());
		QuadSPISendCmd(dev, FLASH_CMD_BULK_ERASE, -1, 0, 0, 0);
		SPIStopTx(dev);
	}
	else
	{
		SendSimpleCmd(FLASH_CMD_BULK_ERASE);
	}

	// This is a long wait, polling every second only
	if (WaitReady(-1, 1000000) == false)
	{
		return -EBUSY;
	}
	WriteDisable();

	return 0;
}
