/**-------------------------------------------------------------------------
@file	nvm.h

@brief	Serial non volatile memory driver.

One driver for any addressed serial memory: NOR flash over SPI or QSPI, I2C
EEPROM, FRAM, and anything else that answers to an address on a bus. There is
no need to write code for each family, only to fill a config and pass it to
Init.

They are the same device to this driver. Every access is an address framed on
the wire followed by data, split at the page boundary, and a wait until the
memory has taken it. What differs between a flash and an EEPROM is only which
commands exist, and a command of 0 means the medium does not have it:

	                  flash                EEPROM
	command byte      RdCmd, WrCmd         0, address only
	write enable      WrEnCmd              0, not needed
	wait for commit   RdStatusCmd polled   0, WriteDelayUs instead
	erase             EraseCmd             0, overwrites directly
	write protect     WrStatusCmd bits     WrProtPin

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
	.RdCmd = { FLASH_CMD_READ, 0 },
	.WrCmd = { FLASH_CMD_WRITE, 0 },
	.WrEnCmd = { FLASH_CMD_WRENABLE, 0 },
	.WrDisCmd = { FLASH_CMD_WRDISABLE, 0 },
	.EraseCmd = { FLASH_CMD_SECTOR_ERASE, 0 },
	.RdStatusCmd = { FLASH_CMD_READSTATUS, 0 },
	.WrStatusCmd = { FLASH_CMD_WRSR, 0 },
	.WrProtMask = 0x3C,				// BP0..BP3
	.WrProtPin = { -1, -1, },		// no pin on this part
};

-----
Quad SPI Micron N25Q128A : the chip's quad command and dummy cycles are config,
so the same code drives any quad part.

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
A chip needing special setup puts it in pInitCB, written against the generic
interface. The S25FS has to be told to use a uniform sector architecture :

	.pInitCB = s25fs_init,

bool s25fs_init(NvmIO * const pDev, DeviceIntrf * const pIntrf)
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

This is new development on the NvmIO API. It does not replace the legacy
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

#include "storage/nvmio.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Serial non volatile memory on the NvmIO API.
class Nvm : public NvmIO {
public:
	Nvm();
	virtual ~Nvm() {}
	Nvm(Nvm&) = delete;

	/**
	 * @brief	Initialize the memory and set the region window.
	 *
	 * @param	Cfg			: Device configuration
	 * @param	pIntrf		: Interface to reach the device
	 * @param	RegionOff	: Region start on the device in bytes
	 * @param	RegionSize	: Region size in bytes, 0 : to the end of device
	 *
	 * @return	true on success.
	 */
	bool Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			  uint64_t RegionOff = 0, uint64_t RegionSize = 0);

	// *** NvmIO ***

	uint32_t EraseSize(void) const override { return vEraseSize; }
	uint32_t WriteGran(void) const override { return vWrGran; }
	uint32_t PageSize(void) const override { return vPageSize; }
	int Read(uint64_t Off, void *pBuf, uint32_t Len) override;
	int Write(uint64_t Off, const void *pData, uint32_t Len) override;
	int Erase(uint64_t Off, uint32_t Len) override;
	int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable) override;

	// *** Device ***

	/**
	 * @brief	Framed read, without the sensor register convention.
	 *
	 * Device::Read sets the top bit of the command for SPI because that is how
	 * most sensors mark a register read. A memory command is not a register
	 * address, so it has to go out untouched.
	 */
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff,
			 int BuffLen) override;

	bool Enable(void) override { return true; }
	void Disable(void) override;
	void Reset(void) override;

	/**
	 * @brief	Erase the whole device, where the medium offers it. Only an
	 * 			instance whose region covers the whole device may use it.
	 *
	 * @return	0 on success, -EPERM on a windowed instance, -ENOTSUP where the
	 * 			medium has no such command, negative errno on failure.
	 */
	int MassErase(void);

	/**
	 * @brief	Read the device id, where the medium has an id command.
	 *
	 * @param	Len : Number of id bytes to read, max 4
	 *
	 * @return	Device id, 0 when the medium has no id command.
	 */
	uint32_t ReadId(int Len);

	/**
	 * @brief	Read the status register, where the medium has one.
	 *
	 * @return	Status value, 0 when the medium has no status command.
	 */
	uint8_t ReadStatus(void);

protected:
	/**
	 * @brief	Wait until the memory has taken the data.
	 *
	 * Polls the status register where the medium has one, otherwise waits the
	 * configured write time. Calls the cooperative wait callback throughout.
	 *
	 * @param	Timeout : Poll count before giving up
	 *
	 * @return	true when the memory is ready.
	 */
	bool WaitReady(uint32_t Timeout = 100000);

	/**
	 * @brief	Set the write enable latch, where the medium has one.
	 *
	 * @return	true when the medium is ready to be written.
	 */
	bool WriteEnable(uint32_t Timeout = 100000);

	/**
	 * @brief	Clear the write enable latch, where the medium has one.
	 */
	void WriteDisable(void);

private:
	// Frame the command, where there is one, and the address. Returns the
	// frame length and fills pDevAddr with the device selection, which takes
	// the high address bits on a part whose address bytes cannot hold them.
	int FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				  uint32_t *pDevAddr);
	int Program(uint32_t Addr, const uint8_t *pData, uint32_t Len);
	int EraseUnit(uint32_t Addr);
	void SendCmd(const NvmCmd_t &Cmd);

	uint64_t	vDevSize;		//!< Whole device size in bytes
	uint32_t	vEraseSize;		//!< Erase unit, 0 : overwrites directly
	uint32_t	vPageSize;		//!< Address auto increment window
	uint32_t	vWrGran;		//!< Minimum aligned write unit
	int			vAddrSize;		//!< Address bytes on the wire
	uint32_t	vAddrSpan;		//!< What the address bytes can reach
	uint32_t	vWrDelayUs;		//!< Write cycle time where there is no status
	uint8_t		vWrProtMask;	//!< Status bits holding the block protect
	NvmCmd_t	vRdCmd;			//!< Read command
	NvmCmd_t	vWrCmd;			//!< Program command
	NvmCmd_t	vWrEnCmd;		//!< Write enable
	NvmCmd_t	vWrDisCmd;		//!< Write disable
	NvmCmd_t	vEraseCmd;		//!< Erase one unit
	NvmCmd_t	vMassEraseCmd;	//!< Erase the whole device
	NvmCmd_t	vRdStatusCmd;	//!< Read status
	NvmCmd_t	vWrStatusCmd;	//!< Write status
	IOPinCfg_t	vWrProtPin;		//!< Write protect pin, PortNo < 0 if unused
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_H__
