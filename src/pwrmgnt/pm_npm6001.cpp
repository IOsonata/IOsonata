/**-------------------------------------------------------------------------
@file pm_npm6001.cpp
@brief Nordic nPM6001 power regulator control.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "pwrmgnt/pm_npm6001.h"

PmnPM6001::PmnPM6001() : vbInitialized(false) {}

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
	if (pI2c == NULL || Addr != NPM6001_I2C_ADDR ||
		pI2c->Type() != DEVINTRF_TYPE_I2C)
		return false;
	Interface(pI2c);
	DeviceAddress(Addr);
	// No device ID register exists. SWREADY is a read/write register.
	// An acknowledged read verifies bus access without modifying supplies.
	if (!ReadReg(NPM6001_SWREADY_REG, ready))
		return false;
	vbInitialized = true;
	return true;
}

bool PmnPM6001::SetBuck3Enabled(bool Enable)
{
	if (!vbInitialized)
		return false;
	return WriteReg(Enable ? NPM6001_START_BUCK3_REG :
		NPM6001_STOP_BUCK3_REG, 1);
}

bool PmnPM6001::SetBuck3Voltage(uint16_t MilliVolts)
{
	uint8_t code;

	if (!vbInitialized || MilliVolts < 500 || MilliVolts > 3300 ||
		(MilliVolts - 500) % 25 != 0)
		return false;
	code = (uint8_t)((MilliVolts - 500) / 25);
	// Write the DAC setting before enabling the internal DAC.
	if (!WriteReg(NPM6001_BUCK3_VOUT_REG, code))
		return false;
	return WriteReg(NPM6001_BUCK3_DAC_REG, 1);
}

bool PmnPM6001::GetBuck3Voltage(uint16_t &MilliVolts, bool &InternalDac)
{
	uint8_t code;
	uint8_t select;

	if (!vbInitialized ||
		!ReadReg(NPM6001_BUCK3_DAC_REG, select) ||
		!ReadReg(NPM6001_BUCK3_VOUT_REG, code) || code > 112)
		return false;
	InternalDac = (select & 1) != 0;
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
	if (!vbInitialized)
		return false;
	return WriteReg(Enable ? NPM6001_INTENSET_REG :
		NPM6001_INTENCLR_REG, Mask);
}
