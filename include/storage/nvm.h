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
so they are the config; everything else stays derived and goes through the
same phased path:

	.RdCmd = { FLASH_CMD_QREAD, 10 },
	.WrCmd = { FLASH_CMD_QWRITE, 0 },

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

/// Event notification for interrupt driven operation.
typedef enum __Nvm_Evt {
	NVM_EVT_UNKNOWN,
	NVM_EVT_WRITE_DONE,			//!< An interrupt driven Write completed
	NVM_EVT_ERASE_DONE,			//!< An interrupt driven Erase completed
	NVM_EVT_READ_DONE,			//!< An interrupt driven Read completed
	NVM_EVT_ERROR				//!< The pending operation failed
} NVM_EVT;

/// Completion handler for interrupt driven operation. Res is 0 on success or a
/// negative errno on failure. Off and Len identify the completed request.
typedef void (*NvmEvtHandler_t)(Nvm * const pDev, NVM_EVT Evt,
								uint64_t Off, uint32_t Len, int Res);

/// Cooperative wait callback, called during a long blocking operation when
/// polling so the caller can do other work. Returning false requests the
/// driver to abort the wait where the medium allows it.
typedef bool (*NvmWaitCb_t)(Nvm * const pDev);

/// Custom pre-initialization callback for device-specific preparation needed
/// before the generic reset or device-ID probe. Returns true on success. The
/// transport and configured device address are available when it is called.
typedef bool (*NvmInitCb_t)(Nvm * const pDev, DeviceIntrf * const pIntrf);

/// A command plus dummy cycle pair. A command of 0 means the medium does not
/// have it.
typedef struct __Nvm_Cmd {
	uint8_t	Cmd;			//!< Command byte
	uint8_t	DummyCycle;		//!< Dummy cycles after the address phase
} NvmCmd_t;

/// Device configuration. A given memory uses the fields that apply to it.
typedef struct __Nvm_Cfg {
	// Identity and placement on the transport.
	int			DevNo;			//!< Device index, or I2C address for EEPROM
	uint64_t	BaseAddr;		//!< Reserved for a future memory-mapped transport
	// Geometry. Set by the user, or filled by the driver from a device query
	// (a JEDEC id, or an interface that knows its own memory).
	uint64_t	TotalSize;		//!< Total usable size in bytes
	uint32_t	EraseSize;		//!< Erase unit. 0 : a direct read write
								//!< medium that overwrites in place, EEPROM,
								//!< FRAM, RAM based. This is the one property
								//!< that decides the kind: an erase medium
								//!< speaks the standard serial NOR protocol,
								//!< which is driver knowledge, not config
	uint32_t	SectorSize;		//!< Logical sector. 0 : equals EraseSize.
	uint32_t	PageSize;		//!< Largest bytes one transfer may take
	uint32_t	WriteGran;		//!< Minimum aligned write unit. 0 : treat as 1.
	uint8_t		AddrSize;		//!< Address bytes on the wire (1..4)
	// Device probe.
	uint32_t	DevId;			//!< Expected device id, 0 to skip the check
	uint8_t		DevIdSize;		//!< Id length in bytes, maximum 4
	// The read and program pair, the one command configuration: 0 derives
	// the plain bus standard from the interface type, set is a wide bus
	// part's own command with its dummy cycles, which is
	// how one driver serves a flash that needs commands and an EEPROM that
	// needs none.
	NvmCmd_t	RdCmd;			//!< Read. 0 : derived from the interface
								//!< type, bare address on I2C, 0x03 serial.
								//!< Set for a wide bus part: the fast, quad
								//!< or octal command with its dummy cycles is
								//!< the one place parts genuinely differ
	NvmCmd_t	WrCmd;			//!< Program. 0 : derived, bare on I2C, 0x02
								//!< serial. Set for a wide bus part
	uint8_t		WrProtMask;		//!< Status bits that hold the block protect
	// Write protect. A pin that is not used must say so with { -1, -1, }.
	IOPinCfg_t	WrProtPin;		//!< Write protect pin
	// Timing.
	uint32_t	WriteDelayUs;	//!< Write cycle time where there is no status
	// Operation mode and callbacks.
	bool			bIntEn;			//!< Reserved; must be false in the polling driver
	NvmEvtHandler_t	EvtHandler;	//!< Reserved for asynchronous completion
	NvmWaitCb_t		pWaitCB;	//!< Cooperative wait, used when polling
	NvmInitCb_t		pInitCB;	//!< Pre-initialization callback, may be NULL
} NvmCfg_t;

/// @brief	Non volatile memory device.
///
/// One driver for any addressed memory. The medium is described by the config;
/// the transport is the DeviceIntrf passed to Init.
class Nvm : virtual public Device {
public:
	Nvm();
	virtual ~Nvm() {}
	Nvm(Nvm&) = delete;

	// Keep the Device register access overloads visible alongside the byte
	// addressed overloads declared here.
	using Device::Read;
	using Device::Write;

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

	/**
	 * @brief	Get the region size in bytes.
	 */
	virtual uint64_t Size(void) const { return vRegionSize; }

	/**
	 * @brief	Get the erase unit in bytes.
	 *
	 * @return	0 : the medium overwrites directly and has no erase step.
	 */
	virtual uint32_t EraseSize(void) const { return vEraseSize; }

	/**
	 * @brief	Get the minimum aligned write unit in bytes.
	 */
	virtual uint32_t WriteGran(void) const { return vWrGran; }

	/**
	 * @brief	Get the logical sector size in bytes.
	 *
	 * A log structured consumer partitions the region into sectors. On an
	 * erase write medium that is the erase unit. A medium that overwrites
	 * directly has no erase unit and reports whatever sector the config asked
	 * for. The logical sector must be a whole multiple of EraseSize.
	 */
	virtual uint32_t LogicalSectorSize(void) const {
		// Through EraseSize so a subclass that overrides it is honoured.
		return vSectSize != 0 ? vSectSize : EraseSize();
	}

	/**
	 * @brief	Get the largest bytes one transfer may take.
	 *
	 * On a medium with an address auto increment window this is the page: the
	 * address counter advances only within it and wraps at the boundary, so a
	 * single transfer must stay inside one. On a medium reached through a
	 * controller it is whatever the controller accepts at once. Either way the
	 * driver splits a longer request here.
	 */
	virtual uint32_t PageSize(void) const { return vPageSize; }

	/**
	 * @brief	Read data from the region.
	 *
	 * @return	Len on success, negative errno on failure.
	 */
	virtual int Read(uint64_t Off, void *pBuf, uint32_t Len);

	/**
	 * @brief	Write data to the region.
	 *
	 * On an erase write medium the destination must be erased. Off and Len are
	 * multiples of WriteGran.
	 *
	 * @return	Len on success, negative errno on failure.
	 */
	virtual int Write(uint64_t Off, const void *pData, uint32_t Len);

	/**
	 * @brief	Erase a range. Off and Len are multiples of EraseSize.
	 *
	 * @return	0 on success, negative errno on failure. A medium that
	 * 			overwrites directly returns success without doing anything.
	 */
	virtual int Erase(uint64_t Off, uint32_t Len);

	/**
	 * @brief	Set or clear write protection.
	 *
	 * Uses the status register block protect bits where the medium has them,
	 * otherwise the configured pin. The mechanism is per medium; the caller
	 * sees one verb.
	 *
	 * @return	0 on success, -ENOTSUP where the medium has neither.
	 */
	virtual int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable);

	/**
	 * @brief	Erase the whole device, where the medium offers it. Only an
	 * 			instance whose region covers the whole device may use it.
	 *
	 * @return	0 on success, -EPERM on a windowed instance, -ENOTSUP where the
	 * 			medium has no such command.
	 */
	int MassErase(void);

	/**
	 * @brief	Report whether an operation is in progress.
	 */
	virtual bool IsBusy(void) const { return false; }

	/**
	 * @brief	Flush buffered state, or drain a pending operation.
	 */
	virtual int Sync(void) { return 0; }

	/**
	 * @brief	Get the region offset on the physical medium.
	 */
	uint64_t RegionOffset(void) const { return vRegionOffset; }

	/**
	 * @brief	Read the device id, where the medium has an id command.
	 */
	uint32_t ReadId(int Len);

	/**
	 * @brief	Read the status register, where the medium has one.
	 *
	 * This compatibility helper returns zero when the medium has no status
	 * register or when the status transaction fails. Driver operations use an
	 * internal error-returning form so a bus failure is never treated as ready.
	 */
	uint8_t ReadStatus(void);

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

protected:
	/**
	 * @brief	Set the region window. Init calls this once geometry is known.
	 */
	void Region(uint64_t Offset, uint64_t Size) {
		vRegionOffset = Offset;
		vRegionSize = Size;
	}

	/**
	 * @brief	Report completion of an interrupt driven operation.
	 */
	void NotifyDone(NVM_EVT Evt, uint64_t Off, uint32_t Len, int Res) {
		if (vEvtHandler != nullptr) {
			vEvtHandler(this, Evt, Off, Len, Res);
		}
	}

	/**
	 * @brief	Invoke the cooperative wait during a long operation.
	 *
	 * @return	false when the caller asked to abort.
	 */
	bool WaitPoll(void) {
		return vpWaitCB != nullptr ? vpWaitCB(this) : true;
	}

	/**
	 * @brief	True when the device is configured interrupt driven.
	 */
	bool IntEn(void) const { return vbIntEn; }

	/**
	 * @brief	Validate a region relative range.
	 */
	bool RangeValid(uint64_t Off, uint32_t Len) const {
		return Off <= vRegionSize && Len <= vRegionSize - Off;
	}

	/**
	 * @brief	Wait until the memory has taken the data.
	 *
	 * Polls the status register where the medium has one, otherwise waits the
	 * configured write time.
	 */
	bool WaitReady(uint32_t Timeout = 100000);

	/**
	 * @brief	Set the write enable latch, where the medium has one.
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
	int SendCmd(const NvmCmd_t &Cmd);
	int ReadStatus(uint8_t &Status);

	uint64_t		vRegionOffset;	//!< Absolute region offset on the medium
	uint64_t		vRegionSize;	//!< Region size in bytes
	bool			vbIntEn;		//!< Interrupt driven when true, else polling
	NvmEvtHandler_t	vEvtHandler;	//!< Completion handler for interrupt mode
	NvmWaitCb_t		vpWaitCB;		//!< Cooperative wait for polling mode
	uint64_t	vDevSize;		//!< Whole device size in bytes
	uint32_t	vEraseSize;		//!< Erase unit, 0 : overwrites directly
	uint32_t	vSectSize;		//!< Logical sector where there is no erase unit
	uint32_t	vPageSize;		//!< Largest bytes one transfer may take
	uint32_t	vWrGran;		//!< Minimum aligned write unit
	int			vAddrSize;		//!< Address bytes on the wire
	uint32_t	vAddrSpan;		//!< What the address bytes can reach
	uint32_t	vWrDelayUs;		//!< Write cycle time where there is no status
	uint8_t		vWrProtMask;	//!< Status bits holding the block protect
	NvmCmd_t	vRdCmd;			//!< Read command
	NvmCmd_t	vWrCmd;			//!< Program command
	bool		vbBare;			//!< Bare address bus (I2C): no command
								//!< bytes, no latch, no status register
	bool		vbPhased;		//!< Phased command bus (QSPI, OSPI): command,
								//!< address and dummy cycles go as transaction
								//!< phases through QuadSPISendCmd, not as
								//!< frame bytes
	uint32_t	vBaseDevAddr;	//!< Configured device address, immutable;
								//!< the banked address derives from it per
								//!< frame so bank bits never stick
	IOPinCfg_t	vWrProtPin;		//!< Write protect pin, PortNo < 0 if unused
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_H__
