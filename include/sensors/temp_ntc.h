/**-------------------------------------------------------------------------
@file	temp_ntc.h

@brief	Generic NTC temperature sensor implementation


Calculation formula

Tkelvin = 1 / (ln(R/R25) / Beta + 1 / T25C); T25C = 298.15 Kelvin

Tcelcius = Tkelvin - 273.15

NTC parameters 
	R25 = Resistor value at 25C
	Beta = given in datasheet

@author	Hoang Nguyen Hoan
@date	Aug. 20, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#ifndef __TEMP_NTC_H__
#define __TEMP_NTC_H__

#include <math.h>

#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

typedef struct __Ntc_Temp_Res {
	int32_t Temp;	//!< Temperature in C * 100
	uint32_t Res;	//!< Resistor value in Ohm
} NtcTempRes_t;

#ifdef __cplusplus

class TempNtc : public TempSensor {
public:

	/**
	 * @brief	Initialize NTC using calculation method
	 *
	 * @param	LoadR	: ADC series load resistor value in Ohm
	 * @param	R25		: Datasheet resistor value at 25C in Ohm
	 * @param	Beta	: Datasheet Beta value
	 * @param	pTimer	: Pointer to timer instance for times stamp (optional)
	 *
	 * @return	None
	 */
	void Init(uint32_t LoadR, uint32_t R25, uint32_t Beta, Timer * const pTimer = NULL);

	/**
	 * @brief	Initialize NTC using lookup table
	 *
	 * @param	LoadR	: ADC series load resistor value in Ohm
	 * @param	pTable	: Pointer to temp/res table
	 * @param	TblSize	: Table size
	 * @param	pTimer	: Pointer to timer instance for times stamp  (optional)
	 *
	 * @return	None
	 */
	void Init(uint32_t LoadR, const NtcTempRes_t *pTable, size_t TblSize, Timer * const pTimer = NULL);

	/**
	 * @brief	Update sensor data with measured voltage
	 *
	 * @param 	mVoltSrc : Source voltage in mV
	 * @param	mVoltNtc : NTC measured voltage in mV
	 *
	 * @return	Temperature in C * 100
	 */
	int32_t UpdateData(int32_t mVoltSrc, int32_t mVoltNtc);

	/**
	 * @brief	Update sensor data with measured NTC resistor
	 *
	 * @param 	Res : Measured NTC resistor in Ohm
	 *
	 * @return	Temperature in C * 100
	 */
	int32_t UdpateData(uint32_t Res);

private:
	virtual bool Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) { return false; }
	virtual bool UpdateDate() { return true; }

	uint32_t vLoadR;				//!< ADC load series resistor value in Ohm
	uint32_t vR25;					//!< Datasheet resistor value at 25C in Ohm
	uint32_t vBeta;					//!< Datasheet Beta value
	const NtcTempRes_t *vpNtcTbl;	//!< Temp/Res lookup table
	size_t vTblSize;				//!< Lookup table size
};

extern "C" {
#endif // __cplusplus

/**
 * @brief	Calculate Temperature based on datasheet R25 & Beta parameters
 * 
 * This function do a linear approximation calculation of the NTC temperature. It does not take
 * into account the coefficient A, B, C, D
 *
 * Calculation is based on the formula :
 * 
 * 	Tk = 1 / (ln(R/R25) / Beta + 1 / T25); T25 = 298.15K
 *
 * 	Tc = Tk - 273.15K
 * 
 * This calculation only have a precision about 2 C. For better precision, use lookup table
 *
 * @param 	R		: Measured resistor value in Ohm
 * @param	R25		: Datasheet resistor value at 25C in Ohm
 * @param	Beta	: Datasheet Beta value
 *  
 * @return	Temperature in degree C * 100
 */
static inline int32_t CalcNtcTemp(uint32_t R, uint32_t R25, uint32_t Beta) {
	return (int32_t)(100.0 / (log((float)R/R25) / (float)Beta + 1.0 / 298.15) - 27315);
}

/**
 * @brief	Get Temperature from lookup table
 *
 * @param 	Res			: Measured resistor value in Ohm
 * @param 	pNtcTbl		: Pointer to lookup temperature/resistor table
 * @param 	TableSize	: Table size
 *
 * @return	Temperature in degree C * 100
 */
int32_t LookupNtcTemp(uint32_t Res, const NtcTempRes_t *pNtcTbl, const size_t TableSize);

#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __TEMP_NTC_H__
