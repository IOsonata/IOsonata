/**-------------------------------------------------------------------------
@file	audiodev_pdm.h

@brief	PDM (Pulse Density Modulation) generic definitions


@author	Nguyen Hoan Hoang
@date	Apr. 4, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#ifndef __AUDIODEV_PDM_H__
#define __AUDIODEV_PDM_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device_intrf.h"
#include "cfifo.h"
#include "coredev/iopincfg.h"

#define PDM_MAX_NB_IOPIN		2

typedef enum __PDM_OpMode {
	PDM_OPMODE_MONO,
	PDM_OPMODE_STEREO
} PDM_OPMODE;

typedef enum __PDM_SamplMode {
	PDM_SMPLMODE_FALING,
	PDM_SMPLMODE_RISING
} PDM_SMPLMODE;

#define PDM_IOPIN_MAXCNT		2

/// PDM pins map index
#define PDM_CLK_IOPIN_IDX		0	//!< Clock
#define PDM_DATA_IOPIN_IDX		1	//!< Data

#pragma pack(push, 4)

/// Configuration data used to initialize device
typedef struct __PDM_Config {
	int DevNo;					//!< Device physical instance number
	IOPinCfg_t * const pIOPinMap;//!< Define I/O pins used by PDM (standard pins : CLK, DATA)
	PDM_OPMODE OpMode;			//!< Mono/Stereo mode
	PDM_SMPLMODE SmplMode;		//!< Sample mode
	uint32_t Freq;				//! sampling frequency in Hz
	int32_t GainLeft;
	int32_t GainRight;
	int RxBlkSize;				//!< Fifo block size, must be multiple of samples
	int RxMemSize;				//!< Memory size in bytes for Rx CFIFO
	uint8_t *pRxMem;			//!< Pointer to memory allocated for RX CFIFO
	bool bIntEn;				//!< true - Interrupt enable
	int IntPrio;				//!< Interrupt priority
	DevIntrfEvtHandler_t EvtCB;	//!< Interrupt based event callback function pointer. Must be set to NULL if not used
} PdmCfg_t;

/// Device driver data require by low level functions
typedef struct __PDM_Device {
	PDM_OPMODE OpMode;			//!< Mono/Stereo mode
	PDM_SMPLMODE SmplMode;		//!< Sample mode
	uint32_t Freq;				//! sampling frequency in mHz
	uint32_t MClkFreq;			//!< Master clock frequency in Hz
	DevIntrf_t DevIntrf;		//!< Device interface instance
	IOPinCfg_t *pIOPinMap;		//!< Define I/O pins used by PDM (standard pins : CLK, DATA)
	int32_t GainLeft;
	int32_t GainRight;
	HCFIFO hRxFifo;				//!< Rx FIFO handle
	int NbSamples;
} PdmDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize PDM hardware interface
 *
 * This is a required implementation.
 *
 * @param	pDev : Pointer to device private data to be initialized
 * @param	pCfg : Pointer to configuration data
 *
 * @return	true - Initialization success
 */
bool PdmInit(PdmDev_t * const pDev, const PdmCfg_t * const pCfg);

static inline void PdmEnable(PdmDev_t *const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void PdmDisable(PdmDev_t *const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }

#ifdef __cplusplus
}

class Pdm : public DeviceIntrf {
public:
	bool Init(const PdmCfg_t &Cfg) { return PdmInit(&vDevData, &Cfg); }
	operator DEVINTRF * const () { return &vDevData.DevIntrf; }
	operator PdmDev_t& () { return vDevData; }
	operator PdmDev_t* const () { return &vDevData; }
	uint32_t Rate(uint32_t RateHz) { return vDevData.DevIntrf.SetRate(&vDevData.DevIntrf, RateHz); }
	uint32_t Rate(void) { return vDevData.DevIntrf.GetRate(&vDevData.DevIntrf); }	// Get rate in Hz
	void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
	void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }

	// DevCs is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartRx(uint32_t DevAddr) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr);
	}
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	// DevAddr is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr);
	}
	// Send Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }

protected:
private:
	PdmDev_t vDevData;
};

#endif // __cplusplus

#endif // __AUDIODEV_PDM_H__
