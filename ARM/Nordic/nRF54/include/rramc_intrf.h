/**-------------------------------------------------------------------------
@file	rramc_intrf.h

@brief	The nRF54L internal RRAM controller as a DeviceIntrf.

The internal memory is a device like any other: it takes a command, an address
and data. It simply has no wire, so this interface is the adapter between that
and the controller registers. With it the ordinary Nvm driver serves internal
memory with no code of its own, the same way it serves a NOR flash on SPI or an
EEPROM on I2C.

The commands use the same opcodes a serial flash does, so a config reads the
same whichever memory it describes. RRAM has no erase command and rewrites in
place, so the erase writes the erased pattern over the unit; nothing above sees
the difference.

Where the work runs is decided here, not by the application:

	no stack running    the controller is driven directly
	link controller     the same work runs inside an MPSL timeslot
	SoftDevice running  the work is submitted to the SoftDevice and the result
	                    arrives as a SoC event

A SoftDevice that is present but stopped arbitrates nothing, so the controller
is driven directly in that case as well.

The long wait is handed to the application through RramcIntrfSetWait, the same
idea as the wait callback the flash and EEPROM configs take. On a SoftDevice
build the application must also pass SoC events to RramcIntrfSocEvt.

The memory size cannot be read from the device the way the nRF52 page size
comes from FICR, so it is a build value. Override RRAMC_INTRF_TOTAL_SIZE for a
part other than the nRF54L15.

Example :

	RramcIntrf g_Rramc;
	NvmCfg_t cfg;

	g_Rramc.Init();

	memset(&cfg, 0, sizeof(cfg));
	RramcIntrfCfg(cfg);

	Nvm g_Nvm;
	g_Nvm.Init(cfg, &g_Rramc, RegionAddr, RegionSize);

@author	Hoang Nguyen Hoan
@date	July 24, 2026

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
#ifndef __RRAMC_INTRF_H__
#define __RRAMC_INTRF_H__

#include "device_intrf.h"
#include "storage/nvmio.h"

/** @addtogroup Storage
  * @{
  */

// The command set, using the opcodes a serial flash uses so a config for
// internal memory reads like one for a flash chip.
#define RRAMC_CMD_READ			0x03		//!< Read
#define RRAMC_CMD_WRITE			0x02		//!< Program words
#define RRAMC_CMD_ERASE			0x20		//!< Clear one unit

// Address bytes on the frame. The internal memory is 32 bit addressed.
#define RRAMC_ADDR_SIZE			4

#ifdef __cplusplus

/// Called repeatedly while an operation is waiting, so the application can run
/// its event dispatch or yield to a scheduler. Returning false asks the
/// interface to give up on the wait.
typedef bool (*RramcIntrfWait_t)(void);

/// Counts of what the interface did, for a test to show which path ran.
typedef struct __Rramc_Intrf_Stat {
	uint32_t	Ops;		//!< Operations completed
	uint32_t	Busy;		//!< Refused while the radio held the memory
	uint32_t	Evt;		//!< Results delivered as a SoC event
} RramcIntrfStat_t;

/// @brief	The nRF54L RRAM controller as a device interface.
class RramcIntrf : public DeviceIntrf {
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
void RramcIntrfSetWait(RramcIntrfWait_t pWait, uint32_t TimeoutMs);

/**
 * @brief	Pass a SoftDevice SoC event to the interface.
 *
 * Needed only on a build running a SoftDevice. Anything that is not a memory
 * result is ignored, so passing every event is safe.
 *
 * @param	SysEvt : SoC event id
 */
void RramcIntrfSocEvt(uint32_t SysEvt);

/**
 * @brief	Read the operation counts.
 *
 * @param	pStat : Filled with the counts since reset
 */
void RramcIntrfGetStat(RramcIntrfStat_t *pStat);

/**
 * @brief	Fill a config for the internal memory.
 *
 * Sets the geometry and the command set this interface understands. The
 * remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void RramcIntrfCfg(NvmCfg_t &Cfg);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __RRAMC_INTRF_H__
