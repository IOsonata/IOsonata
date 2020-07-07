/**-------------------------------------------------------------------------
@file	eink_inrf.h

@brief	Implementation of E-Ink display serial interface

This driver operates in BSI low mode.  BSI can be hardware pulled down or
connected MCU. If connected to MCU BSI will be force to low by this driver.
This mode allows to support both SPI interface or bit banging

@author	Hoang Nguyen Hoan
@date	Apr. 5, 2020

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

#ifndef __EINK_INTRF_H__
#define __EINK_INTRF_H__

#include "coredev/spi.h"
//#include "device.h"

/// E-Ink interface type
typedef enum __EInk_Intrt_Type {
	EIINTRF_TYPE_SPI,				//1< 3 wire SPI
	EIINTRF_TYPE_BITBANG,			//!< Bit banging
} EIINTRF_TYPE;

/// Pin map array indexes. Pin map array must follow the order indicated by these indexes
#define EIINTRF_DC_PIN_IDX			0	//!< Output Data (1)/Command(0) select (valid only in SPI mode)
#define EIINTRF_BUSY_PIN_IDX		1	//!< Input display busy flag, 0 : busy
#define EIINTRF_RST_PIN_IDX			2	//!< Display reset, active low
#define EIINTRF_BSI_PIN_IDX			3	//!< serial interface select 0 : SPI, 1 : Bitbang
#define EIINTRF_SCL_PIN_IDX			4	//!< Clock
#define EIINTRF_SDA_PIN_IDX			5	//!< Data
#define EIINTRF_CS_PIN_IDX			6	//!< Chip select

/// E-Ink interface configuration data
typedef struct __EInk_Intrf_Cfg {
	EIINTRF_TYPE Type;				//!< Interface type SPI or Bitbanging
	const IOPINCFG *pIOPinMap;		//!< Control pins
	int NbIOPins;					//!< Total number of pin to use
	SPIDEV * const pSpiDev;			//!< Pointer to SPI device data if SPI mode is used
	int SpiCsIdx;					//!< CS index in the SPI driver
} EIINTRF_CFG;

/// E-Ink interface driver data
typedef struct __EInk_Device_Intrf {
	EIINTRF_TYPE Type;
	const IOPINCFG *pIOPinMap;
	int NbIOPins;
	DEVINTRF DevIntrf;
	SPIDEV *pSpiDev;
	int SpiCsIdx;
} EIINTRF_DEV;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	E-Ink interface initialization
 *
 * @param	pDev	 : Pointer E-Ink device data to be initialized
 * @param	pCfgData : Pointer to configuration data.
 *
 * @return	true - success
 */
bool EInkIntrfInit(EIINTRF_DEV * const pDev, const EIINTRF_CFG *pCfgData);

/**
 * @brief	Select data/cmd mode
 *
 * @param	pDev : Pointer to E-Ink device data (this driver data)
 * @param	bDataMode : true - Data mode, false : Command mode
 */
void EInkIntrfSetDataMode(EIINTRF_DEV *pDev, bool bDataMode);
static inline void EInkIntrfReset(EIINTRF_DEV * const pDev) { DeviceIntrfReset(&pDev->DevIntrf);}
static inline int EInkIntrfRx(EIINTRF_DEV * const pDev, uint8_t *pBuff, int BuffLen) {
	return DeviceIntrfRx(&pDev->DevIntrf, pDev->SpiCsIdx, pBuff, BuffLen);
}
static inline int EInkIntrfTx(EIINTRF_DEV * const pDev, uint8_t *pData, int DataLen) {
	return DeviceIntrfTx(&pDev->DevIntrf, pDev->SpiCsIdx, pData, DataLen);
}
int EInkIntrfWrite(EIINTRF_DEV * const pDev, uint8_t *pCmd, int CmdLen, uint8_t *pData, int DataLen);

#ifdef __cplusplus
}

/// E-Ink interface class
class EInkIntrf : public DeviceIntrf {
public:
	EInkIntrf() { memset((void*)&vDevData, 0, (int)sizeof(vDevData));}
	EInkIntrf(EInkIntrf&);	// copy ctor not allowed

	/**
	 * @brief	E-Ink interface initialization
	 *
	 * @param	pCfgData : Pointer to configuration data.
	 *
	 * @return	true - success
	 */
	bool Init(const EIINTRF_CFG &Cfg);
	operator DEVINTRF * const () { return &vDevData.DevIntrf; }
	operator EIINTRF_DEV& () { return vDevData; };			// Get config data
	operator EIINTRF_DEV * const () { return &vDevData; };	// Get pointer to device data
	uint32_t Rate(uint32_t RateHz) { return vDevData.DevIntrf.SetRate(&vDevData.DevIntrf, RateHz); }
	uint32_t Rate(void) { return vDevData.DevIntrf.GetRate(&vDevData.DevIntrf); }	// Get rate in Hz
	void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
	void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }

	/**
	 * @brief	Select data/cmd mode
	 *
	 * @param	pDev : Pointer to E-Ink device data (this driver data)
	 * @param	bDataMode : true - Data mode, false : Command mode
	 */
	void SetDataMode(bool bDataMode) { EInkIntrfSetDataMode(&vDevData, bDataMode); }

	// DevCs is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartRx(uint32_t DevCs) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevCs);
	}

	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }

	// DevAddr is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartTx(uint32_t DevCs) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevCs);
	}

	// Send Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }
	virtual int Write(uint8_t *pCmd, int CmdLen, uint8_t *pData, int DataLen) {
		return EInkIntrfWrite(&vDevData, pCmd, CmdLen, pData, DataLen);
	}

protected:
private:
	EIINTRF_DEV vDevData;	//!< This driver data
};

#endif

#endif // __EINK_INTRF_H__
