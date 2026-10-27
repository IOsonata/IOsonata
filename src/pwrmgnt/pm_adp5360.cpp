/**-------------------------------------------------------------------------
@file pm_adp5360.cpp

@brief Analog Devices ADP5360 power-management driver.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "pwrmgnt/pm_adp5360.h"

#define ADP5360_DEVICE_ID_REG				0x00U
#define ADP5360_CHARGER_TERM_REG			0x03U
#define ADP5360_CHARGER_CURRENT_REG			0x04U
#define ADP5360_CHARGER_FUNC_REG			0x07U
#define ADP5360_CHARGER_STATUS1_REG			0x08U
#define ADP5360_CHARGER_STATUS2_REG			0x09U
#define ADP5360_BAT_SOC_REG					0x21U
#define ADP5360_VBAT_READ_REG				0x25U
#define ADP5360_BUCK_CFG_REG					0x29U
#define ADP5360_BUCK_OUTPUT_REG				0x2AU
#define ADP5360_BUCKBOOST_CFG_REG			0x2BU
#define ADP5360_BUCKBOOST_OUTPUT_REG			0x2CU
#define ADP5360_FAULT_STATUS_REG				0x2EU
#define ADP5360_PGOOD_STATUS_REG				0x2FU

#define ADP5360_REGULATOR_ENABLE_MASK			0x01U
#define ADP5360_REGULATOR_DISCHARGE_MASK		0x02U
#define ADP5360_BUCK_PWM_MASK					0x08U
#define ADP5360_BUCK_CURRENT_MASK				0x30U
#define ADP5360_BUCKBOOST_CURRENT_MASK			0x38U
#define ADP5360_OUTPUT_VOLTAGE_MASK			0x3FU

#define ADP5360_CHARGER_ENABLE_MASK			0x01U
#define ADP5360_CHARGER_MODE_MASK				0x07U
#define ADP5360_BATTERY_STATE_MASK				0x07U
#define ADP5360_TRICKLE_CURRENT_MASK			0x03U
#define ADP5360_FAST_CURRENT_MASK				0x1FU
#define ADP5360_TERM_VOLTAGE_MASK				0xFCU

PmAdp5360::PmAdp5360() : vbInitialized(false)
{
	vChrgCurr = 0;
	vpEvtHandler = NULL;
}

bool PmAdp5360::ReadReg(uint8_t Reg, uint8_t &Value)
{
	DeviceIntrf *pIntrf;

	pIntrf = Interface();

	if (pIntrf == NULL)
		return false;

	return pIntrf->Read(DeviceAddress(), &Reg, 1, &Value, 1) == 1;
}

bool PmAdp5360::WriteReg(uint8_t Reg, uint8_t Value)
{
	DeviceIntrf *pIntrf;

	pIntrf = Interface();

	if (pIntrf == NULL)
		return false;

	return pIntrf->Write(DeviceAddress(), &Reg, 1, &Value, 1) == 1;
}

bool PmAdp5360::UpdateReg(uint8_t Reg, uint8_t Mask, uint8_t Value)
{
	uint8_t current;

	if (!ReadReg(Reg, current))
		return false;

	current &= (uint8_t)~Mask;
	current |= Value & Mask;

	return WriteReg(Reg, current);
}

bool PmAdp5360::Init(DeviceIntrf * const pIntrf, uint8_t Addr)
{
	uint8_t id;

	vbInitialized = false;
	Valid(false);

	if (pIntrf == NULL || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		Addr != ADP5360_I2C_ADDR)
		return false;

	Interface(pIntrf);
	DeviceAddress(Addr);

	if (!ReadReg(ADP5360_DEVICE_ID_REG, id) || id != ADP5360_DEVICE_ID)
		return false;

	vbInitialized = true;
	Valid(true);

	return true;
}

bool PmAdp5360::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	size_t count;
	size_t i;
	uint8_t addr;
	int32_t actual;

	addr = Cfg.DevAddr == 0 ? ADP5360_I2C_ADDR : (uint8_t)Cfg.DevAddr;

	if (!Init(pIntrf, addr))
		return false;

	vpEvtHandler = Cfg.pEvtHandler;

	count = Cfg.NbVout;

	if (count > ADP5360_VOUT_COUNT)
		count = ADP5360_VOUT_COUNT;

	if (count > 0 && Cfg.pVout == NULL)
		return false;

	for (i = 0; i < count; i++)
	{
		actual = SetVout(i, Cfg.pVout[i].mVout, Cfg.pVout[i].mAlimit);

		if (Cfg.pVout[i].mVout > 0 && actual <= 0)
			return false;
	}

	if (Cfg.ChrgCurr > 0 || Cfg.VEndChrg > 0)
	{
		if (SetCharge(PWRMGNT_CHARGE_TYPE_AUTO, Cfg.VEndChrg,
			Cfg.ChrgCurr) == 0 && Cfg.ChrgCurr > 0)
			return false;
	}

	return true;
}

int32_t PmAdp5360::SetVout(size_t VoutIdx, int32_t mVolt,
	uint32_t mALimit)
{
	uint8_t cfgReg;
	uint8_t outReg;
	uint8_t currentMask;
	uint8_t currentShift;
	uint8_t currentCode;
	uint8_t voltageCode;
	uint32_t maxCurrent;
	int32_t actualVoltage;

	if (!vbInitialized || VoutIdx >= ADP5360_VOUT_COUNT)
		return -1;

	cfgReg = VoutIdx == ADP5360_VOUT_BUCK ?
		ADP5360_BUCK_CFG_REG : ADP5360_BUCKBOOST_CFG_REG;
	outReg = VoutIdx == ADP5360_VOUT_BUCK ?
		ADP5360_BUCK_OUTPUT_REG : ADP5360_BUCKBOOST_OUTPUT_REG;
	currentMask = VoutIdx == ADP5360_VOUT_BUCK ?
		ADP5360_BUCK_CURRENT_MASK : ADP5360_BUCKBOOST_CURRENT_MASK;
	currentShift = VoutIdx == ADP5360_VOUT_BUCK ? 4U : 3U;
	maxCurrent = VoutIdx == ADP5360_VOUT_BUCK ? 400U : 800U;

	if (mVolt <= 0)
	{
		if (!UpdateReg(cfgReg, ADP5360_REGULATOR_ENABLE_MASK, 0))
			return -1;

		return 0;
	}

	if (VoutIdx == ADP5360_VOUT_BUCK)
	{
		if (mVolt < 600 || mVolt > 3750)
			return -1;

		voltageCode = (uint8_t)((mVolt - 600) / 50);
		actualVoltage = 600 + (int32_t)voltageCode * 50;
	}
	else
	{
		if (mVolt < 1800 || mVolt > 5500)
			return -1;

		if (mVolt < 2950)
		{
			voltageCode = (uint8_t)((mVolt - 1800) / 100);
			actualVoltage = 1800 + (int32_t)voltageCode * 100;
		}
		else
		{
			voltageCode = 12U + (uint8_t)((mVolt - 2950) / 50);
			actualVoltage = 2950 + (int32_t)(voltageCode - 12U) * 50;
		}
	}

	if (mALimit == 0 || mALimit > maxCurrent)
		mALimit = maxCurrent;

	if (mALimit < 100)
		mALimit = 100;

	currentCode = (uint8_t)((mALimit - 100) / 100);

	if (!UpdateReg(outReg, ADP5360_OUTPUT_VOLTAGE_MASK, voltageCode))
		return -1;

	if (!UpdateReg(cfgReg, currentMask,
		(uint8_t)(currentCode << currentShift)))
		return -1;

	if (!UpdateReg(cfgReg, ADP5360_REGULATOR_ENABLE_MASK,
		ADP5360_REGULATOR_ENABLE_MASK))
		return -1;

	return actualVoltage;
}

bool PmAdp5360::GetVout(size_t VoutIdx, int32_t &mVolt,
	uint32_t &mALimit, bool &Enabled)
{
	uint8_t cfgReg;
	uint8_t outReg;
	uint8_t cfg;
	uint8_t out;
	uint8_t code;

	if (!vbInitialized || VoutIdx >= ADP5360_VOUT_COUNT)
		return false;

	cfgReg = VoutIdx == ADP5360_VOUT_BUCK ?
		ADP5360_BUCK_CFG_REG : ADP5360_BUCKBOOST_CFG_REG;
	outReg = VoutIdx == ADP5360_VOUT_BUCK ?
		ADP5360_BUCK_OUTPUT_REG : ADP5360_BUCKBOOST_OUTPUT_REG;

	if (!ReadReg(cfgReg, cfg) || !ReadReg(outReg, out))
		return false;

	Enabled = (cfg & ADP5360_REGULATOR_ENABLE_MASK) != 0;
	code = out & ADP5360_OUTPUT_VOLTAGE_MASK;

	if (VoutIdx == ADP5360_VOUT_BUCK)
	{
		mVolt = 600 + (int32_t)code * 50;
		mALimit = 100U + (uint32_t)((cfg & ADP5360_BUCK_CURRENT_MASK) >> 4) * 100U;
	}
	else
	{
		if (code < 12)
			mVolt = 1800 + (int32_t)code * 100;
		else
			mVolt = 2950 + (int32_t)(code - 12U) * 50;

		mALimit = 100U +
			(uint32_t)((cfg & ADP5360_BUCKBOOST_CURRENT_MASK) >> 3) * 100U;
	}

	return true;
}

bool PmAdp5360::SetBuckPwmMode(bool Pwm)
{
	if (!vbInitialized)
		return false;

	return UpdateReg(ADP5360_BUCK_CFG_REG, ADP5360_BUCK_PWM_MASK,
		Pwm ? ADP5360_BUCK_PWM_MASK : 0);
}

bool PmAdp5360::SetActiveDischarge(size_t VoutIdx, bool Enable)
{
	uint8_t reg;

	if (!vbInitialized || VoutIdx >= ADP5360_VOUT_COUNT)
		return false;

	reg = VoutIdx == ADP5360_VOUT_BUCK ?
		ADP5360_BUCK_CFG_REG : ADP5360_BUCKBOOST_CFG_REG;

	return UpdateReg(reg, ADP5360_REGULATOR_DISCHARGE_MASK,
		Enable ? ADP5360_REGULATOR_DISCHARGE_MASK : 0);
}

uint32_t PmAdp5360::SetCharge(PWRMGNT_CHARGE_TYPE Type,
	int32_t mVoltEoC, uint32_t mACurr)
{
	uint8_t voltageCode;
	uint8_t currentCode;
	uint32_t actualCurrent;

	if (!vbInitialized)
		return 0;

	if (Type != PWRMGNT_CHARGE_TYPE_TRICKLE &&
		Type != PWRMGNT_CHARGE_TYPE_NORMAL &&
		Type != PWRMGNT_CHARGE_TYPE_AUTO)
		return 0;

	if (mACurr == 0)
	{
		if (!UpdateReg(ADP5360_CHARGER_FUNC_REG,
			ADP5360_CHARGER_ENABLE_MASK, 0))
			return 0;

		vChrgCurr = 0;
		return 0;
	}

	if (mVoltEoC < 3560 || mVoltEoC > 4660)
		return 0;

	voltageCode = (uint8_t)((mVoltEoC - 3560) / 20);

	if (!UpdateReg(ADP5360_CHARGER_TERM_REG,
		ADP5360_TERM_VOLTAGE_MASK, (uint8_t)(voltageCode << 2)))
		return 0;

	if (Type == PWRMGNT_CHARGE_TYPE_TRICKLE)
	{
		if (mACurr < 5)
		{
			currentCode = 0;
			actualCurrent = 1;
		}
		else if (mACurr < 10)
		{
			currentCode = 2;
			actualCurrent = 5;
		}
		else
		{
			currentCode = 3;
			actualCurrent = 10;
		}

		if (!UpdateReg(ADP5360_CHARGER_TERM_REG,
			ADP5360_TRICKLE_CURRENT_MASK, currentCode))
			return 0;
	}
	else
	{
		if (mACurr < 10)
			mACurr = 10;

		if (mACurr > 320)
			mACurr = 320;

		currentCode = (uint8_t)((mACurr - 10) / 10);
		actualCurrent = 10U + (uint32_t)currentCode * 10U;

		if (!UpdateReg(ADP5360_CHARGER_CURRENT_REG,
			ADP5360_FAST_CURRENT_MASK, currentCode))
			return 0;
	}

	if (!UpdateReg(ADP5360_CHARGER_FUNC_REG,
		ADP5360_CHARGER_ENABLE_MASK, ADP5360_CHARGER_ENABLE_MASK))
		return 0;

	vChrgCurr = actualCurrent;

	return actualCurrent;
}

bool PmAdp5360::ChargerMode(Adp5360ChargerMode_t &Mode)
{
	uint8_t value;

	if (!vbInitialized || !ReadReg(ADP5360_CHARGER_STATUS1_REG, value))
		return false;

	Mode = (Adp5360ChargerMode_t)(value & ADP5360_CHARGER_MODE_MASK);

	return true;
}

bool PmAdp5360::BatteryState(Adp5360BatteryState_t &State)
{
	uint8_t value;
	uint8_t state;

	if (!vbInitialized || !ReadReg(ADP5360_CHARGER_STATUS2_REG, value))
		return false;

	state = value & ADP5360_BATTERY_STATE_MASK;

	if (state > ADP5360_BATTERY_OK)
		return false;

	State = (Adp5360BatteryState_t)state;

	return true;
}

bool PmAdp5360::Charging()
{
	Adp5360ChargerMode_t mode;

	if (!ChargerMode(mode))
		return false;

	return mode == ADP5360_CHARGER_TRICKLE ||
		mode == ADP5360_CHARGER_FAST_CONST_I ||
		mode == ADP5360_CHARGER_FAST_CONST_V;
}

bool PmAdp5360::Battery()
{
	Adp5360BatteryState_t state;

	if (!BatteryState(state))
		return false;

	return state != ADP5360_BATTERY_NOT_PRESENT;
}

bool PmAdp5360::BatteryLevel(uint8_t &Percent)
{
	uint8_t value;

	if (!vbInitialized || !ReadReg(ADP5360_BAT_SOC_REG, value))
		return false;

	Percent = value > 100 ? 100 : value;

	return true;
}

bool PmAdp5360::BatteryVoltage(uint16_t &MilliVolts)
{
	DeviceIntrf *pIntrf;
	uint16_t raw;
	uint8_t reg;
	uint8_t data[2];

	if (!vbInitialized)
		return false;

	pIntrf = Interface();

	if (pIntrf == NULL)
		return false;

	reg = ADP5360_VBAT_READ_REG;

	if (pIntrf->Read(DeviceAddress(), &reg, 1, data, sizeof(data)) != sizeof(data))
		return false;

	raw = ((uint16_t)data[0] << 8) | data[1];
	MilliVolts = raw >> 3;

	return true;
}

bool PmAdp5360::ReadFaultStatus(uint8_t &Status)
{
	return vbInitialized && ReadReg(ADP5360_FAULT_STATUS_REG, Status);
}

bool PmAdp5360::ReadPowerGood(uint8_t &Status)
{
	return vbInitialized && ReadReg(ADP5360_PGOOD_STATUS_REG, Status);
}


bool PmAdp5360::Enable()
{
	return vbInitialized && UpdateReg(ADP5360_CHARGER_FUNC_REG,
		ADP5360_CHARGER_ENABLE_MASK, ADP5360_CHARGER_ENABLE_MASK);
}

void PmAdp5360::Disable()
{
	if (vbInitialized)
		UpdateReg(ADP5360_CHARGER_FUNC_REG, ADP5360_CHARGER_ENABLE_MASK, 0);
}

void PmAdp5360::Reset()
{
	vbInitialized = false;
	Valid(false);
}
