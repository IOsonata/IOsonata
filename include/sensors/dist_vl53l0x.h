/**-------------------------------------------------------------------------
@file	dist_vl53l0x.h

@brief	Definitions for the ST VL53L0X time of flight distance sensor

The VL53L0X measures the distance to the nearest target up to about 2 m with
a 940 nm laser, on I2C. ST publishes no register map: the registers and the
initialization sequence below follow the ST VL53L0X API (STSW-IMG005).
Multiple byte registers are big endian.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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
#ifndef __DIST_VL53L0X_H__
#define __DIST_VL53L0X_H__

#include <stdint.h>

#include "sensors/dist_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define VL53L0X_I2C_DEV_ADDR					0x29	//!< Address at power up

#define VL53L0X_SYSRANGE_START_REG				0x00
#define VL53L0X_SYSRANGE_START_STARTSTOP								(1<<0)	//!< Start a measurement, stop the continuous mode
#define VL53L0X_SYSRANGE_START_BACKTOBACK								(1<<1)	//!< Continuous, back to back
#define VL53L0X_SYSRANGE_START_TIMED									(1<<2)	//!< Continuous, one measurement per period

#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_REG		0x01
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_VHV								(1<<0)	//!< VHV calibration step
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_PHASE							(1<<1)	//!< Phase calibration step
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_MSRC								(1<<2)	//!< Minimum signal rate check
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_DSS								(1<<3)	//!< Dynamic SPAD selection
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_TCC								(1<<4)	//!< Target centre check
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_PRE_RANGE						(1<<6)
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_FINAL_RANGE						(1<<7)
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG_RANGING							0xE8	//!< DSS, pre range and final range

#define VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD_REG	0x04	//!< 32 bits, oscillator periods
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_REG	0x0A
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO_NEW_SAMPLE					0x04	//!< Interrupt on each new sample
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR_REG		0x0B
#define VL53L0X_RESULT_INTERRUPT_STATUS_REG		0x13
#define VL53L0X_RESULT_INTERRUPT_STATUS_MASK							(7<<0)	//!< Not 0 when a sample is ready
#define VL53L0X_RESULT_RANGE_STATUS_REG			0x14	//!< Start of the 12 bytes of result
#define VL53L0X_RESULT_SIZE						12
#define VL53L0X_RESULT_SIGNAL_RATE				6		//!< Offset in the result, Mcps 9.7 fixed point
#define VL53L0X_RESULT_AMBIENT_RATE				8		//!< Offset in the result, Mcps 9.7 fixed point
#define VL53L0X_RESULT_DISTANCE					10		//!< Offset in the result, mm
#define VL53L0X_RANGE_STATUS_DEVICE_MASK								(0xF<<3)	//!< Device range status
#define VL53L0X_RANGE_STATUS_DEVICE_POS									(3)
#define VL53L0X_DEVICE_STATUS_RANGE_COMPLETE							11
#define VL53L0X_DEVICE_STATUS_NONE_CODE				0xFF	//!< Status of a result with device status 0

#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG	0x44	//!< 16 bits, Mcps 9.7 fixed point
#define VL53L0X_SIGNAL_RATE_LIMIT_DEFAULT		32		//!< 0.25 Mcps
#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP_REG	0x46
#define VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD_REG	0x50
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG	0x51	//!< 16 bits, encoded
#define VL53L0X_MSRC_CONFIG_CONTROL_REG			0x60
#define VL53L0X_MSRC_CONFIG_CONTROL_LIMIT_CHECKS_OFF					0x12	//!< No MSRC and pre range signal rate checks
#define VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD_REG	0x70
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI_REG	0x71	//!< 16 bits, encoded
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_REG		0x84
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH_POL								(1<<4)	//!< GPIO1 interrupt active high
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_REG	0x89
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV_2V8					(1<<0)	//!< 2.8 V I/O, else 1.8 V
#define VL53L0X_I2C_SLAVE_DEVICE_ADDRESS_REG	0x8A
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG	0xB0	//!< 6 bytes, 48 reference SPADs
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_SIZE	6
#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT_REG	0xB6
#define VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N_REG	0xBF	//!< 0 holds the device in reset
#define VL53L0X_IDENTIFICATION_MODEL_ID_REG		0xC0
#define VL53L0X_IDENTIFICATION_MODEL_ID											0xEE
#define VL53L0X_IDENTIFICATION_REVISION_ID_REG	0xC2	//!< Reads 0 while in reset
#define VL53L0X_OSC_CALIBRATE_VAL_REG			0xF8	//!< 16 bits, oscillator periods per ms
#define VL53L0X_PAGE_SELECT_REG					0xFF

#define VL53L0X_TIMING_BUDGET_DEFAULT			33000	//!< usec

#ifdef __cplusplus

class DistVl53l0x : public DistSensor {
public:
	DistVl53l0x();

	/**
	 * @brief	Initialize the sensor.
	 *
	 * Resets the device, then runs the initialization and the reference
	 * calibrations of the ST API. Cfg.DevAddr 0 or 0x29 uses the power up
	 * address. Another address is used when a device answers there, else the
	 * device at 0x29 is moved to it, to run several devices on one bus.
	 * Cfg.OpMode SENSOR_OPMODE_SINGLE measures once on each StartSampling,
	 * else continuously, one measurement per period of Cfg.Freq, back to
	 * back when Cfg.Freq is 0. Cfg.TimingBudget is the time of one
	 * measurement, longer is more precise. Cfg.Inter not 0 enables the
	 * interrupt on GPIO1 with the polarity of Cfg.IntPol.
	 */
	virtual bool Init(const DistSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	/**
	 * @brief	Set the rate of the continuous mode.
	 *
	 * Written by Enable.
	 *
	 * @param	Freq : Frequency in mHz, 0 for back to back measurements
	 *
	 * @return	Actual frequency in mHz, from a period in whole ms
	 */
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint32_t SamplingFrequency() { return DistSensor::SamplingFrequency(); }

	/**
	 * @brief	Set the time of one measurement.
	 *
	 * @param	Us : Time in usec, 20000 and above with the default sequence
	 *
	 * @return	Time in effect in usec, unchanged when Us is too short
	 */
	uint32_t TimingBudget(uint32_t Us);
	uint32_t TimingBudget() { return vTimingBudget; }

	/**
	 * @brief	Select the I/O level, before Init.
	 *
	 * @param	bEn : true for 2.8 V I/O (default), false for 1.8 V
	 */
	void Io2v8(bool bEn) { vbIo2v8 = bEn; }

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual bool UpdateData();
	virtual void IntHandler();

private:
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);
	bool DataInit();
	bool SpadInit();
	bool RefCalibration(uint8_t SeqConfig, uint8_t VhvInit);
	void StopVariable(uint8_t Val);
	uint32_t SequenceTime(uint8_t SeqConfig, uint32_t &PreMclks, uint32_t &FinalUs);
	uint8_t ModelId();
	uint16_t Reg16(uint8_t Reg);
	bool Reg16(uint8_t Reg, uint16_t Val);
	bool Reg32(uint8_t Reg, uint32_t Val);

	uint8_t vStopVariable;		//!< Value of the internal stop register, read at init
	bool vbIo2v8;				//!< 2.8 V I/O
	uint32_t vTimingBudget;		//!< usec
	uint32_t vPeriodMs;			//!< Continuous period, 0 back to back
};

extern "C" {
#endif // __cplusplus


#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __DIST_VL53L0X_H__
