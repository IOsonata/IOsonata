/**-------------------------------------------------------------------------
@file	nvm_seep.h

@brief	Serial I2C EEPROM driver on the NvmIO base.

One class for a serial I2C EEPROM. The device is described by an NvmCfg_t
(total size, memory address length, page size, and the optional write protect
pin and write cycle delay); a different chip is a different config, not a
subclass. The bus is any DeviceIntrf passed to Init.

An EEPROM has no erase step: a write overwrites the target bytes directly, so
EraseSize() is 0 and Erase() is a no-op success. A write must not cross a page
boundary, so a burst is split at page boundaries like the flash driver. After
each page write the device needs a write cycle time; the driver waits the
configured delay, or calls the wait callback, before the next page.

This is new development on the NvmIO API. It does not replace the legacy
seep.h and Seep driver, which remain for backward compatibility.

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
#ifndef __NVM_SEEP_H__
#define __NVM_SEEP_H__

#include "storage/nvmio.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Serial I2C EEPROM device on the NvmIO API.
class NvmSeep : public NvmIO {
public:
	NvmSeep();
	virtual ~NvmSeep() {}
	NvmSeep(NvmSeep&) = delete;

	/**
	 * @brief	Initialize the EEPROM and set the region window.
	 *
	 * @param	Cfg			: Device configuration. DevNo is the I2C device
	 *						  address, AddrSize the memory address length in
	 *						  bytes (1 or 2), PageSize the write page size.
	 * @param	pIntrf		: I2C interface to access the device
	 * @param	RegionOff	: Region start on the device in bytes
	 * @param	RegionSize	: Region size in bytes, 0 : to the end of device
	 *
	 * @return	true on success.
	 */
	bool Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			  uint64_t RegionOff = 0, uint64_t RegionSize = 0);

	// *** NvmIO ***

	// An EEPROM overwrites directly and has no erase unit.
	uint32_t EraseSize(void) const override { return 0; }
	uint32_t WriteGran(void) const override { return 1; }
	uint32_t PageSize(void) const override { return vPageSize; }
	int Read(uint64_t Off, void *pBuf, uint32_t Len) override;
	int Write(uint64_t Off, const void *pData, uint32_t Len) override;
	// Erase inherits the base no-op success for a direct write medium.
	int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable) override;

	// *** Device ***

	bool Enable(void) override { return true; }
	void Disable(void) override {}
	void Reset(void) override {}

private:
	// Frame the memory address MSB first into pAd and return the I2C device
	// address, folding the high address bits into it for a block select part.
	uint8_t FrameAddr(uint8_t *pAd, uint32_t Addr);

	DevIntrf_t	*vpDevIntrf;	//!< Interface handle for the C helpers
	uint64_t	vDevSize;		//!< Whole device size in bytes
	uint32_t	vPageSize;		//!< Write page size in bytes
	int			vAddrSize;		//!< Memory address length in bytes
	uint32_t	vWrDelayUs;		//!< Write cycle delay in usec
	IOPinCfg_t	vWrProtPin;		//!< Write protect pin, PortNo < 0 if unused
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_SEEP_H__
