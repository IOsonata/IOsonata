/**-------------------------------------------------------------------------
@file	mag_bmm150.cpp

@brief	Implementation of BOSCH BMM150 mag sensor

@author	Hoang Nguyen Hoan
@date	Aug. 23, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include <stdint.h>

#include "idelay.h"
#include "sensors/mag_bmm150.h"


/**
 * @brief	Initialize mag sensor.
 *
 * @param 	Cfg		: Configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool MagBmm150::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	MagSensor::Type(SENSOR_TYPE_MAG);

	vData.Range = Range(0x7FFF);

	uint8_t regaddr = BMM150_CTRL1_REG;
	uint8_t d = BMM150_CTRL1_POWER_ON;

	// use this way to allow hook up on the secondary interface of a combo device
	MagSensor::Write(&regaddr, 1, &d, 1);

	Precision(Cfg.Precision);
	SamplingFrequency(Cfg.Freq);

	Enable();

	return true;
}

uint32_t MagBmm150::SamplingFrequency(uint32_t Freq)
{

}

uint8_t MagBmm150::Precision(uint8_t Value)
{

}

bool MagBmm150::Enable()
{
	return true;
}

void MagBmm150::Disable()
{
}

void MagBmm150::Reset()
{
}

/*
uint8_t MagBmm150::Read8(uint8_t *pCmdAddr, int CmdAddrLen)
{
	uint8_t d;

	int l = Read(vDevAddr, pCmdAddr, CmdAddrLen, &d, 1);

	return d;
}

int MagBmm150::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	return Read(vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int MagBmm150::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	return Write(vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int MagBmm150::Read(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
//	return Read((uint8_t)vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int MagBmm150::Write(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	//return Write((uint8_t)vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}
*/
