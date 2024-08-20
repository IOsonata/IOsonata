/**-------------------------------------------------------------------------
@file	temp_ntc.cpp

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
#include <math.h>

#include "sensors/temp_ntc.h"

#if 0
/**
 * @brief	Calculate Temperature based on datasheet R25 & Beta parameters
 *
 * Calculation is based on the formula :
 *
 * 	Tk = 1 / (ln(R/R25) / Beta + 1 / T25); T25 = 298.15K
 *
 * 	Tc = Tk - 273.15K
 *
 * @param 	R 		: Measured resistor value in Ohm
 * @param	R25 	: Datasheet resistor value at 25C in Ohm
 * @param	Beta	: Datasheet Beta value
 *
 * @return	Temperature in degree C * 100
 */
int32_t CalcNtcTemp(uint32_t R, uint32_t R25, uint32_t Beta)
{
	int32_t t = (int32_t)(100.0 / (log((float)R/R25) / (float)Beta + 1.0 / 298.15) - 273.15);

	return t;
}
#endif

/**
 * @brief	Get Temperature from lookup table
 *
 * @param 	Res			: Measured resistor value in Ohm
 * @param 	pNtcTbl		: Pointer to lookup temperature/resistor table
 * @param 	TableSize	: Table size
 *
 * @return	Temperature in degree C * 100
 */
int32_t LookupNtcTemp(uint32_t Res, const NtcTempRes_t *pNtcTbl, const size_t TableSize)
{
	int idx = TableSize >> 1;
	int h = TableSize;
	int l = 0;
	int32_t t = 20;

	do {
		if (pNtcTbl[idx].Res >= Res)
		{
			l = idx;
		}
		else
		{
			h = idx;
		}
		idx = ((h + l) >> 1);
	} while (idx != l && idx != h);

	t = pNtcTbl[l].Temp > pNtcTbl[h].Temp ? pNtcTbl[l].Temp : pNtcTbl[h].Temp;

	return t;
}

/**
 * @brief	Initialize NTC using calculation method
 *
 * @param	LoadR	: ADC series load resistor value in Ohm
 * @param	R25		: Datasheet resistor value at 25C in Ohm
 * @param	Beta	: Datasheet Beta value
 * @param	pTimer	: Pointer to timer instance for times stamp
 *
 * @return	None
 */
void TempNtc::Init(uint32_t LoadR, uint32_t R25, uint32_t Beta, Timer * const pTimer)
{
	vLoadR = LoadR;
	vR25 = R25;
	vBeta = Beta;
	vpNtcTbl = nullptr;
	vTblSize = 0;
	vpTimer = pTimer;
}

/**
 * @brief	Initialize NTC using lookup table
 *
 * @param	LoadR	: ADC series load resistor value in Ohm
 * @param	pTable	: Pointer to temp/res table
 * @param	TblSize	: Table size
 * @param	pTimer	: Pointer to timer instance for times stamp
 *
 * @return	None
 */
void TempNtc::Init(uint32_t LoadR, const NtcTempRes_t *pTable, size_t TblSize, Timer * const pTimer)
{
	vLoadR = LoadR;
	vR25 = 0;
	vBeta = 0;
	vpNtcTbl = pTable;
	vTblSize = TblSize;
	vpTimer = pTimer;
}

/**
 * @brief	Update sensor data with measured voltage
 *
 * @param 	mVoltSrc : Source voltage in mV
 * @param	mVoltNtc : NTC measured voltage in mV
 *
 * @return	Temperature in C * 100
 */
int32_t TempNtc::UdpateData(int32_t mVoltSrc, int32_t mVoltNtc)
{
	uint32_t r;

	// Calculate R
	// R = V / I
	// => R = mVoltNtc / I
	// where I = (mVoltSrc - mVoltNtc) / vLoadR
	// R = mVoltNtc * vLoadR / (mVoltSrc - mVoltNtc)
	r = mVoltNtc * vLoadR / (mVoltSrc - mVoltNtc);

	return TempNtc::UdpateData(r);
}

/**
 * @brief	Update sensor data with measured NTC resistor
 *
 * @param 	Res : Measured NTC resistor in Ohm
 *
 * @return	Temperature in C * 100
 */
int32_t TempNtc::UdpateData(uint32_t Res)
{
	if (vpTimer)
	{
		vData.Timestamp = vpTimer->uSecond();
	}

	if (vpNtcTbl)
	{
		vData.Temperature = LookupNtcTemp(Res, vpNtcTbl, vTblSize);
	}
	else
	{
		vData.Temperature = CalcNtcTemp(Res, vR25, vBeta);
	}


	return vData.Temperature;
}
