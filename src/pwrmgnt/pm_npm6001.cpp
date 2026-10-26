/**-------------------------------------------------------------------------
@file pm_npm6001.cpp

@brief Nordic nPM6001 power regulator control.

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
#include "pwrmgnt/pm_npm6001.h"

PmnPM6001::PmnPM6001() : vbInitialized(false)
{
}

bool PmnPM6001::ReadReg(uint8_t Reg, uint8_t &Value)
{
	DeviceIntrf *pIntrf;

	pIntrf = Interface();

	if (pIntrf == NULL)
		return false;

	return pIntrf->Read(DeviceAddress(), &Reg, 1, &Value, 1) == 1;
}

bool PmnPM6001::WriteReg(uint8_t Reg, uint8_t Value)
{
	DeviceIntrf *pIntrf;

	pIntrf = Interface();

	if (pIntrf == NULL)
		return false;

	return pIntrf->Write(DeviceAddress(), &Reg, 1, &Value, 1) == 1;
}

bool PmnPM6001::Init(DeviceIntrf * const pI2c, uint8_t Addr)
{
	uint8_t ready;

	vbInitialized = false;
	Valid(false);

	if (pI2c == NULL || Addr != NPM6001_I2C_ADDR ||
		pI2c->Type() != DEVINTRF_TYPE_I2C)
		return false;

	Interface(pI2c);
	DeviceAddress(Addr);

	/*
	 * nPM6001 has no device-ID register. SWREADY is read/write, so an
	 * acknowledged read verifies access without changing any power rail.
	 */
	if (!ReadReg(NPM6001_SWREADY_REG, ready))
		return false;

	vbInitialized = true;
	Valid(true);

	return true;
}

bool PmnPM6001::SetBuck3Enabled(bool Enable)
{
	if (!vbInitialized)
		return false;

	return WriteReg(Enable ? NPM6001_START_BUCK3_REG :
		NPM6001_STOP_BUCK3_REG, 1);
}

bool PmnPM6001::SetBuck3PwmMode(bool ForcePwm)
{
	uint8_t value;

	if (!vbInitialized || !ReadReg(NPM6001_BUCK3_PWM_MODE_REG, value))
		return false;

	if (ForcePwm)
		value |= NPM6001_BUCK3_FORCE_PWM_MASK;
	else
		value &= (uint8_t)~NPM6001_BUCK3_FORCE_PWM_MASK;

	return WriteReg(NPM6001_BUCK3_PWM_MODE_REG, value);
}

bool PmnPM6001::GetBuck3PwmMode(bool &ForcePwm)
{
	uint8_t value;

	if (!vbInitialized || !ReadReg(NPM6001_BUCK3_PWM_MODE_REG, value))
		return false;

	ForcePwm = (value & NPM6001_BUCK3_FORCE_PWM_MASK) != 0;

	return true;
}

bool PmnPM6001::SetBuck3Voltage(uint16_t MilliVolts)
{
	uint8_t code;
	bool previousPwm;
	bool result;

	if (!vbInitialized || MilliVolts < 500 || MilliVolts > 3300 ||
		(MilliVolts - 500) % 25 != 0)
		return false;

	if (!GetBuck3PwmMode(previousPwm))
		return false;

	if (!previousPwm && !SetBuck3PwmMode(true))
		return false;

	code = (uint8_t)((MilliVolts - 500) / 25);
	result = WriteReg(NPM6001_BUCK3_VOUT_REG, code);

	if (result)
		result = WriteReg(NPM6001_BUCK3_DAC_REG,
			NPM6001_BUCK3_DAC_SELECT_MASK);

	if (!previousPwm && !SetBuck3PwmMode(false))
		result = false;

	return result;
}

bool PmnPM6001::GetBuck3Voltage(uint16_t &MilliVolts, bool &InternalDac)
{
	uint8_t code;
	uint8_t select;

	if (!vbInitialized ||
		!ReadReg(NPM6001_BUCK3_DAC_REG, select) ||
		!ReadReg(NPM6001_BUCK3_VOUT_REG, code) || code > 112)
		return false;

	InternalDac = (select & NPM6001_BUCK3_DAC_SELECT_MASK) != 0;
	MilliVolts = 500 + (uint16_t)code * 25;

	return true;
}

bool PmnPM6001::SetSoftwareReady(bool Ready)
{
	if (!vbInitialized)
		return false;

	return WriteReg(NPM6001_SWREADY_REG, Ready ? 1 : 0);
}

bool PmnPM6001::ReadInterruptPending(uint8_t &Pending)
{
	return vbInitialized && ReadReg(NPM6001_INTPEND_REG, Pending);
}

bool PmnPM6001::ReadThermalWarning(bool &Warning)
{
	uint8_t value;

	if (!vbInitialized || !ReadReg(NPM6001_EVENT_THWARN_REG, value))
		return false;

	Warning = (value & 1) != 0;

	return true;
}

bool PmnPM6001::ReadBuck3Overcurrent(bool &Overcurrent)
{
	uint8_t value;

	if (!vbInitialized || !ReadReg(NPM6001_EVENT_BUCK3OC_REG, value))
		return false;

	Overcurrent = (value & 1) != 0;

	return true;
}

bool PmnPM6001::ClearThermalWarning()
{
	return vbInitialized && WriteReg(NPM6001_EVENT_THWARN_REG, 0);
}

bool PmnPM6001::ClearBuck3Overcurrent()
{
	return vbInitialized && WriteReg(NPM6001_EVENT_BUCK3OC_REG, 0);
}

bool PmnPM6001::EnableFaultInterrupts(uint8_t Mask, bool Enable)
{
	if (!vbInitialized || (Mask & (uint8_t)~NPM6001_FAULT_INT_MASK) != 0)
		return false;

	return WriteReg(Enable ? NPM6001_INTENSET_REG :
		NPM6001_INTENCLR_REG, Mask);
}


bool PmnPM6001::Enable()
{
	return SetBuck3Enabled(true);
}

void PmnPM6001::Disable()
{
	SetBuck3Enabled(false);
}

void PmnPM6001::Reset()
{
	vbInitialized = false;
	Valid(false);
}
