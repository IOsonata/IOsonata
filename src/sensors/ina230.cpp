/**-------------------------------------------------------------------------
@file	ina230.cpp
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

#include "sensors/ina230.h"
#include "idelay.h"

bool Ina230::ReadReg(uint8_t Reg, uint16_t &Value)
{
	uint8_t data[2];
	int count;

	count = Interface()->Read(DeviceAddress(), &Reg, 1, data, 2);
	if (count != 2)
	{
		return false;
	}

	Value = (uint16_t(data[0]) << 8) | data[1];

	return true;
}

bool Ina230::WriteReg(uint8_t Reg, uint16_t Value)
{
	uint8_t data[2];
	int count;

	data[0] = uint8_t(Value >> 8);
	data[1] = uint8_t(Value);
	count = Interface()->Write(DeviceAddress(), &Reg, 1, data, 2);

	return count == 2;
}

bool Ina230::Program()
{
	uint16_t value;

	if (!WriteReg(INA230_REG_CALIBRATION, vCalibration))
	{
		return false;
	}
	if (!WriteReg(INA230_REG_CONFIG, vCfg.Config))
	{
		return false;
	}
	if (!ReadReg(INA230_REG_CALIBRATION, value) || value != vCalibration)
	{
		return false;
	}
	if (!ReadReg(INA230_REG_CONFIG, value) || (value & 0x7FFFU) != vCfg.Config)
	{
		return false;
	}

	vbEnabled = true;

	return true;
}

bool Ina230::Init(const Ina230Cfg_t &Cfg, DeviceIntrf *pIntrf)
{
	uint64_t denominator;
	uint64_t calibration;

	Valid(false);
	vbEnabled = false;
	vCalibration = 0;

	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		Cfg.DevAddr > 0x7FU || Cfg.RShuntMicroOhm == 0 ||
		Cfg.CurrentLsbMicroAmp == 0 || (Cfg.Config & INA230_CONFIG_RESET) != 0)
	{
		return false;
	}

	denominator = uint64_t(Cfg.RShuntMicroOhm) * Cfg.CurrentLsbMicroAmp;
	calibration = (5120000000ULL + denominator / 2ULL) / denominator;
	if (calibration == 0 || calibration > 0xFFFFU)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vCfg = Cfg;
	vCalibration = uint16_t(calibration);

	if (!Program())
	{
		return false;
	}

	Valid(true);

	return true;
}

bool Ina230::Enable()
{
	if (!Device::Valid())
	{
		return false;
	}
	if (vbEnabled)
	{
		return true;
	}

	return Program();
}

void Ina230::Disable()
{
	uint16_t config;

	if (!Device::Valid())
	{
		return;
	}

	config = vCfg.Config & ~uint16_t(7U);
	if (!WriteReg(INA230_REG_CONFIG, config))
	{
		Valid(false);
		vbEnabled = false;
		return;
	}

	vbEnabled = false;
}

void Ina230::Reset()
{
	if (!Device::Valid())
	{
		return;
	}

	if (!WriteReg(INA230_REG_CONFIG, INA230_CONFIG_RESET))
	{
		Valid(false);
		vbEnabled = false;
		return;
	}

	msDelay(1);
	if (!Program())
	{
		Valid(false);
		vbEnabled = false;
	}
}

void Ina230::PowerOff()
{
	Disable();
	Valid(false);
	vbEnabled = false;
}

int64_t Ina230::ShuntVoltageNanoVolt()
{
	uint16_t raw;

	if (!Device::Valid() || !vbEnabled ||
		!ReadReg(INA230_REG_SHUNT_VOLTAGE, raw))
	{
		return 0;
	}

	return int64_t(int16_t(raw)) * 2500LL;
}

uint32_t Ina230::BusVoltageMicroVolt()
{
	uint16_t raw;

	if (!Device::Valid() || !vbEnabled ||
		!ReadReg(INA230_REG_BUS_VOLTAGE, raw))
	{
		return 0;
	}

	return uint32_t(raw) * 1250U;
}

int64_t Ina230::CurrentMicroAmp()
{
	uint16_t raw;

	if (!Device::Valid() || !vbEnabled ||
		!ReadReg(INA230_REG_CURRENT, raw))
	{
		return 0;
	}

	return int64_t(int16_t(raw)) * vCfg.CurrentLsbMicroAmp;
}

uint64_t Ina230::PowerMicroWatt()
{
	uint16_t raw;

	if (!Device::Valid() || !vbEnabled ||
		!ReadReg(INA230_REG_POWER, raw))
	{
		return 0;
	}

	return uint64_t(raw) * vCfg.CurrentLsbMicroAmp * 25ULL;
}
