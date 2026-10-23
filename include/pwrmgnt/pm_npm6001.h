/**-------------------------------------------------------------------------
@file pm_npm6001.h
@brief Nordic nPM6001 power regulator control.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __PM_NPM6001_H__
#define __PM_NPM6001_H__

#include "device.h"

#define NPM6001_I2C_ADDR              0x70U
#define NPM6001_SWREADY_REG           0x01U
#define NPM6001_START_BUCK3_REG       0x02U
#define NPM6001_STOP_BUCK3_REG        0x08U
#define NPM6001_EVENT_THWARN_REG      0x1EU
#define NPM6001_EVENT_BUCK3OC_REG     0x22U
#define NPM6001_INTENSET_REG          0x2BU
#define NPM6001_INTENCLR_REG          0x2CU
#define NPM6001_INTPEND_REG           0x2DU
#define NPM6001_BUCK3_DAC_REG         0x44U
#define NPM6001_BUCK3_VOUT_REG        0x45U

/**
 * nPM6001 controls hardware power rails; it has no battery charger.
 * This driver deliberately does not change the startup voltages or the
 * state of other rails during Init().
 *
 * On Thingy:91 X, BUCK3 supplies nRF7002; ENABLE and READY are board GPIOs.
 * The board must assert ENABLE, wait for READY, and initialize its I2C bus
 * before calling Init().
 */
class PmnPM6001 : public Device {
public:
	PmnPM6001();
	bool Init(DeviceIntrf * const pI2c, uint8_t Addr = NPM6001_I2C_ADDR);
	bool SetBuck3Enabled(bool Enable);
	/** Set BUCK3 only when the board design uses the internal DAC.
	 * The supported DAC voltage range is 500 to 3300 mV in 25 mV steps.
	 * No write occurs for an invalid voltage. */
	bool SetBuck3Voltage(uint16_t MilliVolts);
	bool GetBuck3Voltage(uint16_t &MilliVolts, bool &InternalDac);
	bool SetSoftwareReady(bool Ready);
	bool ReadInterruptPending(uint8_t &Pending);
	bool ReadThermalWarning(bool &Warning);
	bool ReadBuck3Overcurrent(bool &Overcurrent);
	bool ClearThermalWarning();
	bool ClearBuck3Overcurrent();
	bool EnableFaultInterrupts(uint8_t Mask, bool Enable);
	bool Initialized() const { return vbInitialized; }

private:
	bool ReadReg(uint8_t Reg, uint8_t &Value);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool vbInitialized;
};

#endif
