/**-------------------------------------------------------------------------
@file	sdram.h

@brief	SDRAM on the external memory controller of the MCU

SdramInit sets up the memory controller of the MCU for an SDRAM described by
its organization and the timings of its data sheet, runs the power up
sequence of the part, then maps it in the address space: pBase and Size of
SdramDev_t. Several parts in parallel make the data bus (BusWidth /
DevWidth). The MCU port takes the organizations its controller supports and
refuses the others.

The SDRAM clock is the closest the controller makes to Rate. The timings are
converted to clocks of that rate, rounded up. The mode register gets the CAS
latency and a sequential burst chosen by the port.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __SDRAM_H__
#define __SDRAM_H__

#include <stdint.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"

#pragma pack(push, 4)

/// SDRAM configuration: the part, its timings, the pins of the controller
typedef struct __Sdram_Config {
	int DevNo;						//!< Chip select of the controller for the SDRAM, 0 based
	const IOPinCfg_t *pIOPinMap;	//!< Controller pins: address, data, bank, DQM, RAS, CAS, WE, CLK, CKE, CS
	int NbIOPins;					//!< Number of pins in the map
	uint32_t Rate;					//!< SDRAM clock asked, Hz
	uint8_t BusWidth;				//!< Data bus, bits: 8, 16 or 32
	uint8_t DevWidth;				//!< Data width of one part, bits: 8, 16 or 32
	uint8_t NbBanks;				//!< Banks of a part: 2 or 4
	uint8_t RowBits;				//!< Row address bits of a part
	uint8_t ColBits;				//!< Column address bits of a part
	uint8_t CasLatency;				//!< CAS latency, clocks
	uint8_t TwrClk;					//!< Write recovery time, clock part, added to TwrNs
	uint8_t TmrdClk;				//!< Load mode register to active command, clocks (tMRD)
	uint32_t RefreshNs;				//!< Time between two refresh commands: refresh period / rows
	uint16_t TrcdNs;				//!< Active to read or write command (tRCD)
	uint16_t TrpNs;					//!< Precharge command period (tRP)
	uint16_t TrasNs;				//!< Active to precharge command period (tRAS)
	uint16_t TrcNs;					//!< Active to active command period (tRC)
	uint16_t TrfcNs;				//!< Auto refresh period (tRFC)
	uint16_t TrrdNs;				//!< Active bank A to active bank B (tRRD)
	uint16_t TwrNs;					//!< Write recovery time, ns part (tWR)
	uint16_t TaprNs;				//!< Last data out to active command (tAPR)
	uint16_t TsrexNs;				//!< Self refresh exit time (tSREX)
	uint16_t TxsrNs;				//!< Exit self refresh to active command (tXSR)
} SdramCfg_t;

/// SDRAM data
typedef struct __Sdram_Device {
	SdramCfg_t Cfg;					//!< Configuration
	uint32_t Rate;					//!< SDRAM clock, Hz, 0 when not set up
	void *pBase;					//!< Address of the memory
	uint32_t Size;					//!< Size of the memory, bytes
} SdramDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Set up the memory controller and the SDRAM. MCU port.
 *
 * @param	pDev	: SDRAM data, pBase, Size and Rate set on success
 * @param	pCfg	: Configuration
 *
 * @return	true on success
 */
bool SdramInit(SdramDev_t * const pDev, const SdramCfg_t * const pCfg);

#ifdef __cplusplus
}

/// SDRAM class
class Sdram {
public:
	Sdram() { vDev.Rate = 0; vDev.pBase = nullptr; vDev.Size = 0; }
	virtual ~Sdram() {}
	Sdram(Sdram&) = delete;

	virtual bool Init(const SdramCfg_t &Cfg) { return SdramInit(&vDev, &Cfg); }

	/// SDRAM clock, Hz
	uint32_t Rate(void) const { return vDev.Rate; }
	/// Address of the memory
	void *Base(void) const { return vDev.pBase; }
	/// Size of the memory, bytes
	uint32_t Size(void) const { return vDev.Size; }

	operator SdramDev_t * () { return &vDev; }

private:
	SdramDev_t vDev;
};

#endif	// __cplusplus

#endif	// __SDRAM_H__
