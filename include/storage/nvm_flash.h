/**-------------------------------------------------------------------------
@file	nvm_flash.h

@brief	Serial NOR and QSPI flash driver on the NvmIO base.

One class for any serial flash chip over SPI or QSPI. The chip is described by
an NvmCfg_t (geometry, address size, JEDEC id, and the read and write command
and dummy cycle pairs); a different chip is a different config, not a subclass.
The bus is any DeviceIntrf passed to Init; the driver branches at runtime on
the interface type for the QSPI command path. Chip quirks such as enabling
quad mode go in the config init callback.

This is new development on the NvmIO API. It does not replace the legacy
flash.h and Flash driver, which remain for backward compatibility.

@author	Hoang Nguyen Hoan
@date	Jul. 20, 2026

@license

MIT license

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
#ifndef __NVM_FLASH_H__
#define __NVM_FLASH_H__

#include "storage/nvmio.h"
#include "storage/flash.h"		// for the FLASH_CMD_* command definitions

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Serial NOR / QSPI flash device on the NvmIO API.
class NvmFlash : public NvmIO {
public:
	NvmFlash();
	virtual ~NvmFlash() {}
	NvmFlash(NvmFlash&) = delete;

	/**
	 * @brief	Initialize the flash device and set the region window.
	 *
	 * @param	Cfg			: Device configuration
	 * @param	pIntrf		: Interface to access the device (SPI or QSPI)
	 * @param	RegionOff	: Region start on the device in bytes
	 * @param	RegionSize	: Region size in bytes, 0 : to the end of device
	 *
	 * @return	true on success.
	 */
	bool Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			  uint64_t RegionOff = 0, uint64_t RegionSize = 0);

	// *** NvmIO ***

	uint32_t EraseSize(void) const override { return vSectSize; }
	uint32_t WriteGran(void) const override { return 1; }
	uint32_t PageSize(void) const override { return vPageSize; }
	int Read(uint64_t Off, void *pBuf, uint32_t Len) override;
	int Write(uint64_t Off, const void *pData, uint32_t Len) override;
	int Erase(uint64_t Off, uint32_t Len) override;
	int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable) override;

	/**
	 * @brief	Erase the entire device with the chip erase command. Only
	 *			allowed on an instance whose region covers the whole device.
	 *
	 * @return	0 on success, -EPERM on a windowed instance, negative errno on
	 *			failure.
	 */
	int MassErase(void);

	// *** Device ***

	bool Enable(void) override;
	void Disable(void) override;
	void Reset(void) override;
	void PowerOff(void) override;

	/**
	 * @brief	Read the flash JEDEC id.
	 *
	 * @param	Len : Number of id bytes to read (max 4)
	 *
	 * @return	Device id.
	 */
	uint32_t ReadId(int Len);

	/**
	 * @brief	Read the flash status register.
	 *
	 * @return	Status register value.
	 */
	uint8_t ReadStatus(void);

protected:
	/**
	 * @brief	Wait for the write in progress flag to clear. Uses the wait
	 *			callback if the config supplied one, else a poll delay.
	 *
	 * @param	Timeout		: Poll count before giving up
	 * @param	usRtyDelay	: Delay between polls in usec, 0 : no delay
	 *
	 * @return	true when the device is ready.
	 */
	bool WaitReady(uint32_t Timeout = 100000, uint32_t usRtyDelay = 0);

	/**
	 * @brief	Set the write enable latch.
	 *
	 * @param	Timeout : Poll count for the latch to be confirmed set
	 *
	 * @return	true when the latch is confirmed set.
	 */
	bool WriteEnable(uint32_t Timeout = 100000);

	/**
	 * @brief	Clear the write enable latch.
	 */
	void WriteDisable(void);

private:
	int Program(uint32_t Addr, const uint8_t *pData, uint32_t Len);
	int SectorErase(uint32_t Addr);
	void SendSimpleCmd(uint8_t Cmd);
	void FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr);

	DevIntrf_t	*vpDevIntrf;	//!< Interface handle for the C helpers
	uint64_t	vDevSize;		//!< Whole device size in bytes
	uint32_t	vSectSize;		//!< Sector erase size in bytes
	uint32_t	vBlkSize;		//!< Block erase size in bytes
	uint32_t	vPageSize;		//!< Program page size in bytes
	int			vAddrSize;		//!< Address size in bytes
	NvmCmd_t	vRdCmd;			//!< QSPI read command and dummy cycle
	NvmCmd_t	vWrCmd;			//!< QSPI write command and dummy cycle
	NvmInitCb_t	vpInitCB;		//!< Custom init callback
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_FLASH_H__
