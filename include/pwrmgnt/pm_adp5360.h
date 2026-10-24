/**-------------------------------------------------------------------------
@file pm_adp5360.h

@brief Analog Devices ADP5360 power-management driver.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __PM_ADP5360_H__
#define __PM_ADP5360_H__

#include "pwrmgnt/pwrmgnt.h"

#define ADP5360_I2C_ADDR					0x46U
#define ADP5360_DEVICE_ID					0x10U

#define ADP5360_VOUT_BUCK					0U
#define ADP5360_VOUT_BUCKBOOST				1U
#define ADP5360_VOUT_COUNT					2U

#define ADP5360_FAULT_TEMP_SHUTDOWN			0x01U
#define ADP5360_FAULT_WATCHDOG				0x04U
#define ADP5360_FAULT_BAT_CHARGE_OV			0x10U
#define ADP5360_FAULT_CHARGE_OC				0x20U
#define ADP5360_FAULT_DISCHARGE_OC			0x40U
#define ADP5360_FAULT_DISCHARGE_UV			0x80U

#define ADP5360_PGOOD_VOUT1					0x01U
#define ADP5360_PGOOD_VOUT2					0x02U
#define ADP5360_PGOOD_BATTERY				0x04U
#define ADP5360_PGOOD_VBUS					0x08U
#define ADP5360_PGOOD_CHARGE_COMPLETE		0x10U
#define ADP5360_PGOOD_RESET					0x20U

typedef enum __Adp5360ChargerMode {
	ADP5360_CHARGER_OFF = 0,
	ADP5360_CHARGER_TRICKLE = 1,
	ADP5360_CHARGER_FAST_CONST_I = 2,
	ADP5360_CHARGER_FAST_CONST_V = 3,
	ADP5360_CHARGER_COMPLETE = 4,
	ADP5360_CHARGER_LDO = 5,
	ADP5360_CHARGER_TIMER_EXPIRED = 6,
	ADP5360_CHARGER_BATTERY_DETECT = 7,
} Adp5360ChargerMode_t;

typedef enum __Adp5360BatteryState {
	ADP5360_BATTERY_NORMAL = 0,
	ADP5360_BATTERY_NOT_PRESENT = 1,
	ADP5360_BATTERY_DEAD = 2,
	ADP5360_BATTERY_WEAK = 3,
	ADP5360_BATTERY_OK = 4,
} Adp5360BatteryState_t;

/**
 * ADP5360 power management.
 *
 * Init(DeviceIntrf...) only verifies the device ID and does not change
 * regulator, charger, fuel-gauge or protection configuration.
 */
class PmAdp5360 : public PowerMgnt {
public:
	PmAdp5360();

	bool Init(DeviceIntrf * const pIntrf, uint8_t Addr = ADP5360_I2C_ADDR);
	bool Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf);

	/**
	 * Vout 0 is BUCK, Vout 1 is BUCKBOOST.
	 * mVolt <= 0 disables the selected output.
	 */
	int32_t SetVout(size_t VoutIdx, int32_t mVolt, uint32_t mALimit);
	bool GetVout(size_t VoutIdx, int32_t &mVolt, uint32_t &mALimit,
		bool &Enabled);

	bool SetBuckPwmMode(bool Pwm);
	bool SetActiveDischarge(size_t VoutIdx, bool Enable);

	/**
	 * Configure charging. NORMAL/AUTO configure fast-charge current.
	 * TRICKLE configures the hardware trickle current; the PMIC selects
	 * trickle operation according to battery voltage.
	 * mACurr == 0 disables charging.
	 */
	uint32_t SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC,
		uint32_t mACurr);

	bool Charging();
	bool Battery();

	bool ChargerMode(Adp5360ChargerMode_t &Mode);
	bool BatteryState(Adp5360BatteryState_t &State);
	bool BatteryLevel(uint8_t &Percent);
	bool BatteryVoltage(uint16_t &MilliVolts);
	bool ReadFaultStatus(uint8_t &Status);
	bool ReadPowerGood(uint8_t &Status);

	bool Initialized() const { return vbInitialized; }

private:
	bool ReadReg(uint8_t Reg, uint8_t &Value);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool UpdateReg(uint8_t Reg, uint8_t Mask, uint8_t Value);

	bool vbInitialized;
};

#endif
