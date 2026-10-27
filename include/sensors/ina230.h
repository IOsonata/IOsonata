/**-------------------------------------------------------------------------
@file	ina230.h
@brief	TI INA230/INA231 current, voltage and power monitor
@author	Hoang Nguyen Hoan
@date	Oct. 24, 2026
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
#ifndef __INA230_H__
#define __INA230_H__

#include <stdint.h>
#include "device.h"

#define INA230_REG_CONFIG			0x00
#define INA230_REG_SHUNT_VOLTAGE	0x01
#define INA230_REG_BUS_VOLTAGE		0x02
#define INA230_REG_POWER			0x03
#define INA230_REG_CURRENT			0x04
#define INA230_REG_CALIBRATION		0x05
#define INA230_REG_MASK_ENABLE		0x06
#define INA230_REG_ALERT_LIMIT		0x07

#define INA230_CONFIG_RESET			0x8000
#define INA230_CONFIG_AVG_1			(0U << 9)
#define INA230_CONFIG_AVG_4			(1U << 9)
#define INA230_CONFIG_AVG_16		(2U << 9)
#define INA230_CONFIG_AVG_64		(3U << 9)
#define INA230_CONFIG_AVG_128		(4U << 9)
#define INA230_CONFIG_AVG_256		(5U << 9)
#define INA230_CONFIG_AVG_512		(6U << 9)
#define INA230_CONFIG_AVG_1024		(7U << 9)

#define INA230_CONFIG_VBUSCT_140US	(0U << 6)
#define INA230_CONFIG_VBUSCT_204US	(1U << 6)
#define INA230_CONFIG_VBUSCT_332US	(2U << 6)
#define INA230_CONFIG_VBUSCT_588US	(3U << 6)
#define INA230_CONFIG_VBUSCT_1100US	(4U << 6)
#define INA230_CONFIG_VBUSCT_2116US	(5U << 6)
#define INA230_CONFIG_VBUSCT_4156US	(6U << 6)
#define INA230_CONFIG_VBUSCT_8244US	(7U << 6)

#define INA230_CONFIG_VSHCT_140US	(0U << 3)
#define INA230_CONFIG_VSHCT_204US	(1U << 3)
#define INA230_CONFIG_VSHCT_332US	(2U << 3)
#define INA230_CONFIG_VSHCT_588US	(3U << 3)
#define INA230_CONFIG_VSHCT_1100US	(4U << 3)
#define INA230_CONFIG_VSHCT_2116US	(5U << 3)
#define INA230_CONFIG_VSHCT_4156US	(6U << 3)
#define INA230_CONFIG_VSHCT_8244US	(7U << 3)

#define INA230_CONFIG_MODE_POWERDOWN		0U
#define INA230_CONFIG_MODE_SHUNT_TRIG		1U
#define INA230_CONFIG_MODE_BUS_TRIG			2U
#define INA230_CONFIG_MODE_SHUNT_BUS_TRIG	3U
#define INA230_CONFIG_MODE_SHUNT_CONT		5U
#define INA230_CONFIG_MODE_BUS_CONT			6U
#define INA230_CONFIG_MODE_SHUNT_BUS_CONT	7U

typedef struct __Ina230_Config
{
	uint8_t DevAddr;
	uint32_t RShuntMicroOhm;
	uint32_t CurrentLsbMicroAmp;
	uint16_t Config;
} Ina230Cfg_t;

class Ina230 : public Device
{
public:
	bool Init(const Ina230Cfg_t &Cfg, DeviceIntrf *pIntrf);
	bool Enable();
	void Disable();
	void Reset();
	void PowerOff();

	int64_t ShuntVoltageNanoVolt();
	uint32_t BusVoltageMicroVolt();
	int64_t CurrentMicroAmp();
	uint64_t PowerMicroWatt();

	uint16_t Calibration() const
	{
		return vCalibration;
	}
	uint16_t Configuration() const
	{
		return vCfg.Config;
	}

private:
	bool ReadReg(uint8_t Reg, uint16_t &Value);
	bool WriteReg(uint8_t Reg, uint16_t Value);
	bool Program();

	Ina230Cfg_t vCfg{};
	uint16_t vCalibration = 0;
	bool vbEnabled = false;
};

#endif
