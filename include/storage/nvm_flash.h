/**-------------------------------------------------------------------------
@file	nvm_flash.h

@brief	Serial NOR Flash implementation of the NvmIO device class.

Generic driver for SPI and Quad SPI NOR Flash. Device specifics are supplied
by a FlashCfg_t (see storage/flash.h for the structure and the predefined
part configurations); there is no need to write code per device.

The device is an erase-write medium: EraseSize() is the sector erase size,
WriteGran() is 1 and PageSize() bounds a single program operation. Write
programs erased cells page chunk by page chunk in ascending address order.
Erase operates on sector multiples. All operations are synchronous.

An instance covers one region of the device given at Init; several instances
may map disjoint regions of the same part and share the DeviceIntrf.

Usage :

// SPI interface instance, assumed initialized
SPI g_Spi;

static const FlashCfg_t s_FlashCfg = FLASH_MX25R3235F(NULL, NULL);

NvmFlash g_Flash;

g_Flash.Init(s_FlashCfg, &g_Spi);	// whole device
...
g_Flash.Erase(0, g_Flash.EraseSize());
g_Flash.Write(0, data, len);
g_Flash.Read(0, buff, len);

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
#ifndef __NVM_FLASH_H__
#define __NVM_FLASH_H__

#include <stdint.h>

#include "storage/nvmio.h"
#include "storage/flash.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	NOR Flash NvmIO device.
class NvmFlash : public NvmIO {
public:
	NvmFlash();
	virtual ~NvmFlash() {}
	NvmFlash(NvmFlash&) = delete;

	/**
	 * @brief	Initialize the Flash device and set the region window.
	 *
	 * @param	Cfg				: Flash device configuration
	 * @param	pIntrf			: Interface to access the device (SPI/QSPI)
	 * @param	RegionOffset	: Region start on the device in bytes
	 * @param	RegionSize		: Region size in bytes, 0 : to end of device
	 *
	 * @return	true on success.
	 */
	bool Init(const FlashCfg_t &Cfg, DeviceIntrf * const pIntrf,
			  uint64_t RegionOffset = 0, uint64_t RegionSize = 0);

	// *** NvmIO implementation ***

	uint32_t EraseSize(void) const override { return vSectSize; }
	uint32_t WriteGran(void) const override { return 1; }
	uint32_t PageSize(void) const override { return vPageSize; }
	int Read(uint64_t Off, void *pBuf, uint32_t Len) override;
	int Write(uint64_t Off, const void *pData, uint32_t Len) override;
	int Erase(uint64_t Off, uint32_t Len) override;

	/**
	 * @brief	Erase the entire device with the chip erase command.
	 *
	 * Only allowed on an instance whose region covers the whole device.
	 * This can take a long time; use the pWaitCB hook of the configuration
	 * to service other tasks while waiting.
	 *
	 * @return	0 on success, -EPERM on a windowed instance, negative errno
	 * 			on failure.
	 */
	int MassErase(void);

	// *** Device implementation ***

	bool Enable(void) override;
	void Disable(void) override;
	void Reset(void) override;
	void PowerOff(void) override;

	/**
	 * @brief	Read Flash JEDEC id.
	 *
	 * @param	Len : Number of id bytes to read (max 4)
	 *
	 * @return	Device id.
	 */
	uint32_t ReadId(int Len);

	/**
	 * @brief	Read Flash status register.
	 *
	 * @return	Status register value.
	 */
	uint8_t ReadStatus(void);

protected:
	/**
	 * @brief	Wait for the write in progress flag to clear.
	 *
	 * @param	Timeout		: Poll count before giving up
	 * @param	usRtyDelay	: Delay between polls in usec, 0 : no delay.
	 * 						  When the config supplies pWaitCB it is called
	 * 						  instead of delaying.
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

	DevIntrf_t *vpDevIntrf;		//!< Interface handle for the C helpers
	uint64_t vDevSize;			//!< Whole device size in bytes
	uint32_t vSectSize;			//!< Sector erase size in bytes
	uint32_t vBlkSize;			//!< Block erase size in bytes
	uint32_t vPageSize;			//!< Program page size in bytes
	int vAddrSize;				//!< Address size in bytes
	CmdCycle_t vRdCmd;			//!< QSPI read command and dummy cycle
	CmdCycle_t vWrCmd;			//!< QSPI write command and dummy cycle
	FlaskCb_t vpWaitCB;			//!< Long wait callback
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __NVM_FLASH_H__
