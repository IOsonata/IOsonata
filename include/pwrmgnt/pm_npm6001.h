/**-------------------------------------------------------------------------
@file pm_npm6001.h

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
#ifndef __PM_NPM6001_H__
#define __PM_NPM6001_H__

#include "device.h"

#define NPM6001_I2C_ADDR					0x70U

#define NPM6001_SWREADY_REG					0x01U
#define NPM6001_START_BUCK3_REG				0x02U
#define NPM6001_STOP_BUCK3_REG				0x08U

#define NPM6001_EVENT_THWARN_REG				0x1EU
#define NPM6001_EVENT_BUCK3OC_REG			0x22U
#define NPM6001_INTENSET_REG					0x2BU
#define NPM6001_INTENCLR_REG					0x2CU
#define NPM6001_INTPEND_REG					0x2DU

#define NPM6001_BUCK3_DAC_REG				0x44U
#define NPM6001_BUCK3_VOUT_REG				0x45U
#define NPM6001_BUCK3_PWM_MODE_REG			0x4DU

#define NPM6001_BUCK3_DAC_SELECT_MASK		0x01U
#define NPM6001_BUCK3_FORCE_PWM_MASK			0x08U
#define NPM6001_INT_THWARN_MASK				0x08U
#define NPM6001_INT_BUCK3OC_MASK				0x80U
#define NPM6001_FAULT_INT_MASK				(NPM6001_INT_THWARN_MASK | NPM6001_INT_BUCK3OC_MASK)

/**
 * nPM6001 controls hardware power rails; it has no battery charger.
 * This driver deliberately does not change startup voltages or rail state
 * during Init().
 *
 * On Thingy:91 X, BUCK3 supplies nRF7002. nPM6001 ENABLE is driven by
 * nPM1300 LDO1 used as a load switch and nPM6001 READY is connected to
 * nPM1300 GPIO4. Board code must establish that power sequence before Init().
 */
class PmnPM6001 : public Device {
public:
	PmnPM6001();

	/** Enable/disable BUCK3; other rails retain their state. */
	bool Enable() override;
	void Disable() override;
	/** Forget initialization; no hardware reset or board power cycle. */
	void Reset() override;

	bool Init(DeviceIntrf * const pI2c, uint8_t Addr = NPM6001_I2C_ADDR);

	bool SetBuck3Enabled(bool Enable);
	bool SetBuck3PwmMode(bool ForcePwm);
	bool GetBuck3PwmMode(bool &ForcePwm);

	/**
	 * Set BUCK3 using the internal DAC.
	 * The supported range is 500 to 3300 mV in 25 mV steps.
	 * The update is performed in forced PWM mode and the previous mode is
	 * restored afterward. No write occurs for an invalid voltage.
	 */
	bool SetBuck3Voltage(uint16_t MilliVolts);
	bool GetBuck3Voltage(uint16_t &MilliVolts, bool &InternalDac);

	bool SetSoftwareReady(bool Ready);

	bool ReadInterruptPending(uint8_t &Pending);
	bool ReadThermalWarning(bool &Warning);
	bool ReadBuck3Overcurrent(bool &Overcurrent);
	bool ClearThermalWarning();
	bool ClearBuck3Overcurrent();

	/** Enable or disable THWARN and BUCK3 overcurrent interrupt sources. */
	bool EnableFaultInterrupts(uint8_t Mask, bool Enable);

	bool Initialized() const
	{
		return vbInitialized;
	}

private:
	bool ReadReg(uint8_t Reg, uint8_t &Value);
	bool WriteReg(uint8_t Reg, uint8_t Value);

	bool vbInitialized;
};

#endif
