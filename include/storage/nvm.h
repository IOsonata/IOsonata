/**-------------------------------------------------------------------------
@file	nvm.h

@brief	Serial non volatile memory driver.

One driver for any addressed serial memory: NOR flash over SPI, I2C EEPROM,
FRAM, and anything else that answers to an address on a bus. There is no need
to write code for each family, only to fill a config and pass it to Init.

They are the same device to this driver. Every access is an address framed on
the wire followed by data, split at the page boundary, and a wait until the
memory has taken it. What differs between a flash and an EEPROM the driver
works out for itself, from the bus and from EraseSize:

	                  serial (SPI family)     I2C
	command byte      0x03 read, 0x02 write   none, address only
	write enable      0x06, latch polled      not needed
	wait for commit   status 0x05 polled      WriteDelayUs
	erase             from the granule        EraseSize 0, overwrites
	write protect     status bits or pin      WrProtPin

Addresses too wide for the address bytes are folded into the device address,
the way small EEPROMs select a block. The driver works that out from TotalSize
and AddrSize rather than being told.

Note that port 0 pin 0 is a real pin, so a config that does not use a write
protect pin has to say so with WrProtPin = { -1, -1, }. A designated
initializer leaves it zeroed, which would look like a pin on port 0.

Example of defining device info :

-----
MX25R3235F : 32 Mbits NOR flash, 4K sector, 256 byte page

static const NvmCfg_t s_FlashCfg = {
	.DevNo = 0,
	.TotalSize = 32 * 1024 * 1024 / 8,
	.EraseSize = 4 * 1024,
	.PageSize = 256,
	.AddrSize = 3,
	.WrProtMask = 0x3C,				// BP0..BP3
	.WrProtPin = { -1, -1, },		// no pin on this part
};

Almost no command is configuration, because almost no command varies. Two
things the driver already holds decide them. The bus, from the interface
type: an I2C memory takes a bare address and no command byte, and waits by
WriteDelayUs; everything serial speaks the standard protocol, read 0x03,
program 0x02, the write latch and the status poll, which SPI EEPROMs and
FRAMs speak just as a NOR does, so their busy wait is real. The kind, from
EraseSize: an erase medium adds what the JEDEC standard fixed on top, the
erase opcode chosen by the granule (4K, 32K or 64K), chip erase, block
protect, the reset pair, the id read, and 4 byte address mode past 16
MBytes. At Init the driver resets an erase medium first, because the chip
does not reset when the MCU does and can be left in a state where every
other command is misread; the id probe then retries to cover the recovery
time, and 4 byte mode, which the reset cleared, is entered last. Past 16
MBytes the policy is EN4B mode with the standard opcodes taking 4 address
bytes, not the dedicated 4 byte opcodes the legacy driver issues; both are
JEDEC defined, but no 4 byte part has run in this mode yet.

The one place parts genuinely differ is the read and program pair once the
bus is wider than plain SPI: the fast, dual, quad or octal command and,
above all, its dummy cycles, which are the part's own and can depend on the
operating frequency. That pair is therefore the only command configuration:
zero derives the plain bus standard, set is the part's wide bus command,
the same mechanism that made the legacy flash driver universal across SPI,
DSPI, QSPI and OSPI.

-----
Quad SPI Micron N25Q128A : the quad command and dummy cycles are the part's,
so they are the config; everything else stays derived. The quad transfer
path is not wired yet.

	.RdCmd = { FLASH_CMD_QREAD, 10 },
	.WrCmd = { FLASH_CMD_QWRITE, 0 },

Macronix MX25R3235F in quad mode differs only here :

	.RdCmd = { FLASH_CMD_4READ, 6 },
	.WrCmd = { FLASH_CMD_4WRITE, 0 },

-----
CAT24C32 : 32 Kbits I2C EEPROM, 2 byte address, 32 byte page, 5 ms write

static const NvmCfg_t s_EepCfg = {
	.DevNo = 0x50,					// I2C device address
	.TotalSize = 32 * 1024 / 8,
	.EraseSize = 0,					// overwrites directly
	.PageSize = 32,
	.AddrSize = 2,
	.WrProtPin = { -1, -1, },		// no pin on this part
	.WriteDelayUs = 5000,			// Twr
};

-----
CAT24C02 : 2 Kbits, 1 byte address, 16 byte page

	.DevNo = 0x50,
	.TotalSize = 2048 / 8,
	.EraseSize = 0,
	.PageSize = 16,
	.AddrSize = 1,
	.WriteDelayUs = 5000,

-----
M24C64S with a write protect pin, adds only :

	.WrProtPin = { 0, 12, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },

-----
A chip needing preparation before the generic reset or ID probe puts it in
pInitCB, written against the generic interface. The S25FS has to be told to
use a uniform sector architecture before the normal command sequence :

	.pInitCB = s25fs_init,

bool s25fs_init(Nvm * const pDev, DeviceIntrf * const pIntrf)
{
	uint8_t p[8];

	p[0] = NFLASH_S25FS_CMD_WRAR;
	p[1] = (NFLASH_S25FS_REG_CR3V >> 16) & 0xFF;
	p[2] = (NFLASH_S25FS_REG_CR3V >> 8) & 0xFF;
	p[3] = NFLASH_S25FS_REG_CR3V & 0xFF;
	p[4] = NFLASH_S25FS_REG_CR3NV_20h_NV | NFLASH_S25FS_REG_CR3V_02h_NV;
	pIntrf->Tx(pDev->DeviceAddress(), p, 5);

	return true;
}

-----
Usage :

SPI g_Spi;					// or I2C, already initialized
Nvm g_Nvm;

g_Nvm.Init(s_FlashCfg, &g_Spi);

This is new development on the Nvm API. It does not replace the legacy
flash.h and seep.h drivers, which remain for backward compatibility.

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
#ifndef __NVM_H__
#define __NVM_H__

#include <stdint.h>
#include <errno.h>

#include "device.h"
#include "device_intrf.h"
#include "coredev/iopincfg.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

class Nvm;

typedef enum __Nvm_Evt {
	NVM_EVT_UNKNOWN,
	NVM_EVT_WRITE_DONE,
	NVM_EVT_ERASE_DONE,
	NVM_EVT_READ_DONE,
	NVM_EVT_ERROR
} NVM_EVT;

typedef void (*NvmEvtHandler_t)(Nvm * const pDev, NVM_EVT Evt,
								uint64_t Off, uint32_t Len, int Res);
typedef bool (*NvmWaitCb_t)(Nvm * const pDev);
typedef bool (*NvmInitCb_t)(Nvm * const pDev, DeviceIntrf * const pIntrf);

typedef struct __Nvm_Cmd {
	uint8_t	Cmd;
	uint8_t	DummyCycle;
} NvmCmd_t;

typedef struct __Nvm_Cfg {
	int			DevNo;
	uint64_t	BaseAddr;
	uint64_t	TotalSize;
	uint32_t	EraseSize;
	uint32_t	SectorSize;
	uint32_t	PageSize;
	uint32_t	WriteGran;
	uint8_t		AddrSize;
	uint32_t	DevId;
	uint8_t		DevIdSize;
	NvmCmd_t	RdCmd;
	NvmCmd_t	WrCmd;
	uint8_t		WrProtMask;
	IOPinCfg_t	WrProtPin;
	uint32_t	WriteDelayUs;
	bool			bIntEn;
	NvmEvtHandler_t	EvtHandler;
	NvmWaitCb_t		pWaitCB;
	NvmInitCb_t		pInitCB;
} NvmCfg_t;

class Nvm : virtual public Device {
public:
	Nvm();
	virtual ~Nvm() {}
	Nvm(Nvm&) = delete;

	using Device::Read;
	using Device::Write;

	bool Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			  uint64_t RegionOff = 0, uint64_t RegionSize = 0);

	virtual uint64_t Size(void) const { return vRegionSize; }
	virtual uint32_t EraseSize(void) const { return vEraseSize; }
	virtual uint32_t WriteGran(void) const { return vWrGran; }
	virtual uint32_t LogicalSectorSize(void) const {
		return vSectSize != 0 ? vSectSize : EraseSize();
	}
	virtual uint32_t PageSize(void) const { return vPageSize; }

	virtual int Read(uint64_t Off, void *pBuf, uint32_t Len);
	virtual int Write(uint64_t Off, const void *pData, uint32_t Len);
	virtual int Erase(uint64_t Off, uint32_t Len);

	/**
	 * @brief Set or clear whole-device write protection.
	 *
	 * The current mechanisms, status block-protect bits or a WP pin, affect the
	 * whole physical medium. Off must be zero, Len must equal Size(), and this
	 * instance must cover the whole device. Partial protection is not implied.
	 *
	 * @return 0 on success, -ENOTSUP for a partial range or where the medium has
	 *         no mechanism, -EPERM for a windowed instance.
	 */
	virtual int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable);

	int MassErase(void);

	virtual bool IsBusy(void) const { return false; }
	virtual int Sync(void) { return 0; }

	uint64_t RegionOffset(void) const { return vRegionOffset; }
	uint32_t ReadId(int Len);
	uint8_t ReadStatus(void);

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff,
			 int BuffLen) override;

	bool Enable(void) override;
	void Disable(void) override;
	void Reset(void) override;

protected:
	void Region(uint64_t Offset, uint64_t Size) {
		vRegionOffset = Offset;
		vRegionSize = Size;
	}

	void NotifyDone(NVM_EVT Evt, uint64_t Off, uint32_t Len, int Res) {
		if (vEvtHandler != nullptr) {
			vEvtHandler(this, Evt, Off, Len, Res);
		}
	}

	bool WaitPoll(void) {
		return vpWaitCB != nullptr ? vpWaitCB(this) : true;
	}

	bool IntEn(void) const { return vbIntEn; }

	bool RangeValid(uint64_t Off, uint32_t Len) const {
		return Off <= vRegionSize && Len <= vRegionSize - Off;
	}

	bool WaitReady(uint32_t Timeout = 100000);
	bool WriteEnable(uint32_t Timeout = 100000);
	void WriteDisable(void);

private:
	int FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				  uint32_t *pDevAddr);
	int Program(uint32_t Addr, const uint8_t *pData, uint32_t Len);
	int EraseUnit(uint32_t Addr);
	int SendCmd(const NvmCmd_t &Cmd);
	int ReadStatus(uint8_t &Status);
	bool ConfigureDevice(void);
	bool FailInit(void);
	bool Ready(void) { return Valid() && vbEnabled && Interface() != nullptr; }

	uint64_t		vRegionOffset;
	uint64_t		vRegionSize;
	bool			vbIntEn;
	NvmEvtHandler_t	vEvtHandler;
	NvmWaitCb_t		vpWaitCB;
	NvmInitCb_t		vInitCB;
	uint64_t	vDevSize;
	uint32_t	vEraseSize;
	uint32_t	vSectSize;
	uint32_t	vPageSize;
	uint32_t	vWrGran;
	int			vAddrSize;
	uint32_t	vAddrSpan;
	uint32_t	vWrDelayUs;
	uint8_t		vWrProtMask;
	NvmCmd_t	vRdCmd;
	NvmCmd_t	vWrCmd;
	bool		vbBare;
	bool		vbEnabled;
	bool		vbIntrfEnabled;
	uint32_t	vBaseDevAddr;
	uint32_t	vExpectedDevId;
	uint8_t		vExpectedDevIdSize;
	IOPinCfg_t	vWrProtPin;
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_H__
