/**-------------------------------------------------------------------------
@file	nvm_intrf.h

@brief	The MCU internal memory as a DeviceIntrf.

Internal memory is a device like any other: it takes a command, an address and
data. It simply has no wire, so this interface stands in the position a SPI or
an I2C occupies for a flash or an EEPROM chip, and the same Nvm driver serves
it with no code of its own.

There is one class, here, and one implementation per target in its own file
(nvm_nrfx.cpp on the nRF). An application declares the interface the way it
declares a UART or a SPI, and names nothing target specific. The target
implementation owns the controller access and the arbitration with whatever
stack runs.

Usage :

	NvmIntrf g_MemIntrf;
	NvmCfg_t cfg;

	g_MemIntrf.Init();

	memset(&cfg, 0, sizeof(cfg));
	NvmIntrfCfg(cfg);

	Nvm g_Nvm;
	g_Nvm.Init(cfg, &g_MemIntrf, RegionAddr, RegionSize);

The region placement is the application's decision. NvmIntrfCeiling() reports
where the memory the application owns ends, which is not always the device
size: a stack image or a reserved partition can sit at the top, and writing
there destroys it.

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
#ifndef __NVM_INTRF_H__
#define __NVM_INTRF_H__

#include "device_intrf.h"
#include "storage/nvm.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// Called repeatedly while an operation is waiting, so the application can run
/// its event dispatch or yield to a scheduler. Returning false asks the
/// interface to give up on the wait.
typedef bool (*NvmIntrfWait_t)(void);

/// Counts of what the interface did, for a test to show which path ran.
typedef struct __Nvm_Intrf_Stat {
	uint32_t	Ops;		//!< Operations completed
	uint32_t	Busy;		//!< Refused while a stack held the memory
	uint32_t	Evt;		//!< Results delivered as a stack event
	uint32_t	Skipped;	//!< Erases skipped, the unit already erased

	// Which of the three arbitration paths carried the operation. Ops counts
	// them all together, so on its own it cannot tell an operation that went
	// through a stack from one that drove the controller with the radio up,
	// and those two are the pair worth telling apart.
	uint32_t	Sd;			//!< Submitted to the SoftDevice
	uint32_t	Slot;		//!< Run inside an MPSL timeslot
	uint32_t	Direct;		//!< Drove the controller, the memory was ours
} NvmIntrfStat_t;

/// @brief	The MCU memory controller as a device interface.
class NvmIntrf : public DeviceIntrf {
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
 * 						  spend the wait in a short delay instead.
 * @param	TimeoutMs	: Give up on one operation after this many msec.
 * 						  0 keeps the current value.
 */
void NvmIntrfSetWait(NvmIntrfWait_t pWait, uint32_t TimeoutMs);

/**
 * @brief	Read the operation counts.
 *
 * @param	pStat : Filled with the counts since reset
 */
void NvmIntrfGetStat(NvmIntrfStat_t *pStat);

/**
 * @brief	Fill a config for the internal memory.
 *
 * Sets the geometry and the command set the interface understands. The
 * remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void NvmIntrfCfg(NvmCfg_t &Cfg);

/**
 * @brief	Where the memory the application owns ends.
 *
 * On most targets this is the device size. Where a stack image or a reserved
 * partition sits at the top, it is that boundary instead: on the nRF54L the
 * S145 image occupies the top of the RRAM and the application slot ends at
 * the storage partition. C linkage so a target can override the weak default
 * from a C file.
 *
 * @return	First byte past the application owned memory.
 */
extern "C" uint64_t NvmIntrfCeiling(void);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_INTRF_H__
