/**-------------------------------------------------------------------------
@file	nvmc_intrf.h

@brief	The nRF52 internal memory controller as a DeviceIntrf.

The internal memory is a device like any other: it takes a command, an address
and data. It simply has no wire, so this interface is the adapter between that
and the controller registers. With it the ordinary Nvm driver serves internal
memory with no code of its own, the same way it serves a NOR flash on SPI or an
EEPROM on I2C. Only the commands and the interface differ, as they do between
a SPI and a QSPI flash.

The commands use the same opcodes a serial flash does, so a config reads the
same whichever memory it describes.

Where the work runs is decided here, not by the application:

	no stack running    the controller is driven directly
	link controller     the same work runs inside an MPSL timeslot, and a page
	                    erase is sliced across several of them
	SoftDevice running  the work is submitted to the SoftDevice and the result
	                    arrives as a SoC event

A SoftDevice that is present but stopped arbitrates nothing, so the controller
is driven directly in that case as well.

The long wait is handed to the application through NvmcIntrfSetWait, the same
idea as the wait callback the flash and EEPROM configs take. On a SoftDevice
build the application must also pass SoC events to NvmcIntrfSocEvt.

Example :

	NvmcIntrf g_Nvmc;
	NvmCfg_t cfg;

	g_Nvmc.Init();

	memset(&cfg, 0, sizeof(cfg));
	NvmcIntrfCfg(cfg);				// geometry from the device

	Nvm g_Nvm;
	g_Nvm.Init(cfg, &g_Nvmc, RegionAddr, RegionSize);

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
#ifndef __NVMC_INTRF_H__
#define __NVMC_INTRF_H__

#include "device_intrf.h"
#include "storage/nvm.h"

/** @addtogroup Storage
  * @{
  */

// The command set, using the opcodes a serial flash uses so a config for
// internal memory reads like one for a flash chip.
#define NVMC_CMD_READ			0x03		//!< Read
#define NVMC_CMD_WRITE			0x02		//!< Program words
#define NVMC_CMD_ERASE			0x20		//!< Erase one page

// Address bytes on the frame. The internal memory is 32 bit addressed.
#define NVMC_ADDR_SIZE			4

#ifdef __cplusplus

/// Called repeatedly while an operation is waiting, so the application can run
/// its event dispatch or yield to a scheduler. Returning false asks the
/// interface to give up on the wait.
typedef bool (*NvmcIntrfWait_t)(void);

/// Counts of what the interface did, for a test to show which path ran.
typedef struct __Nvmc_Intrf_Stat {
	uint32_t	Ops;		//!< Operations completed
	uint32_t	Busy;		//!< Refused while the radio held the memory
	uint32_t	Evt;		//!< Results delivered as a SoC event
	uint32_t	Skipped;	//!< Page erases skipped, the page already erased
} NvmcIntrfStat_t;

/// @brief	The nRF52 memory controller as a device interface.
class NvmcIntrf : public DeviceIntrf {
public:
	bool Init(void);

	operator DevIntrf_t * const () override { return &vDevIntrf; }

	uint32_t Rate(uint32_t RateHz) override { (void)RateHz; return 0; }
	uint32_t Rate(void) override { return 0; }

	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vDevIntrf, DevAddr);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vDevIntrf, pBuff, BuffLen);
	}
	void StopRx(void) override { DeviceIntrfStopRx(&vDevIntrf); }

	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vDevIntrf, DevAddr);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vDevIntrf, pData, DataLen);
	}
	void StopTx(void) override { DeviceIntrfStopTx(&vDevIntrf); }

protected:
	DevIntrf_t vDevIntrf;
};

/**
 * @brief	Set the wait callback and the operation timeout.
 *
 * @param	pWait		: Called repeatedly while an operation waits. NULL to
 * \t\t\t\t\t\t  spend the wait in a short delay instead.
 * @param	TimeoutMs	: Give up on one operation after this many msec.
 * \t\t\t\t\t\t  0 keeps the current value.
 */
void NvmcIntrfSetWait(NvmcIntrfWait_t pWait, uint32_t TimeoutMs);

/**
 * @brief	Pass a SoftDevice SoC event to the interface.
 *
 * Needed only on a build running a SoftDevice. Anything that is not a memory
 * result is ignored, so passing every event is safe.
 *
 * @param	SysEvt : SoC event id
 */
void NvmcIntrfSocEvt(uint32_t SysEvt);

/**
 * @brief	Read the operation counts.
 *
 * @param	pStat : Filled with the counts since reset
 */
void NvmcIntrfGetStat(NvmcIntrfStat_t *pStat);

/**
 * @brief	Fill a config for the internal memory.
 *
 * Sets the geometry from the factory information registers and the command
 * set this interface understands. The remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void NvmcIntrfCfg(NvmCfg_t &Cfg);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVMC_INTRF_H__
