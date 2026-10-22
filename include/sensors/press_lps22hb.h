/**-------------------------------------------------------------------------
@file	press_lps22hb.h

@brief	Definitions for the ST LPS22HB pressure and temperature sensor

The LPS22HB measures the absolute pressure from 260 to 1260 hPa and the
temperature, on I2C or SPI. It is calibrated in the factory: the outputs are
in hPa x 4096 and degree C x 100.

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
#ifndef __PRESS_LPS22HB_H__
#define __PRESS_LPS22HB_H__

#include <stdint.h>

#include "sensors/press_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define LPS22HB_I2C_DEV_ADDR0			0x5C	//!< SA0 low
#define LPS22HB_I2C_DEV_ADDR1			0x5D	//!< SA0 high

#define LPS22HB_PRESS_SENSITIVITY		4096	//!< LSB per hPa
#define LPS22HB_TEMP_SENSITIVITY		100		//!< LSB per degree C

#define LPS22HB_INTERRUPT_CFG_REG				0x0B

#define LPS22HB_WHO_AM_I_REG					0x0F
#define LPS22HB_WHO_AM_I_ID												0xB1

#define LPS22HB_CTRL_REG1_REG					0x10
#define LPS22HB_CTRL_REG1_SIM											(1<<0)	//!< SPI 3 wire
#define LPS22HB_CTRL_REG1_BDU											(1<<1)	//!< Output registers not updated until all bytes are read
#define LPS22HB_CTRL_REG1_LPFP_CFG										(1<<2)	//!< Filter bandwidth ODR/20, else ODR/9
#define LPS22HB_CTRL_REG1_EN_LPFP										(1<<3)	//!< Pressure low pass filter on
#define LPS22HB_CTRL_REG1_ODR_MASK										(7<<4)	//!< Output data rate
#define LPS22HB_CTRL_REG1_ODR_ONE_SHOT									(0<<4)	//!< Power down, one shot on request
#define LPS22HB_CTRL_REG1_ODR_1HZ										(1<<4)
#define LPS22HB_CTRL_REG1_ODR_10HZ										(2<<4)
#define LPS22HB_CTRL_REG1_ODR_25HZ										(3<<4)
#define LPS22HB_CTRL_REG1_ODR_50HZ										(4<<4)
#define LPS22HB_CTRL_REG1_ODR_75HZ										(5<<4)

#define LPS22HB_CTRL_REG2_REG					0x11
#define LPS22HB_CTRL_REG2_ONE_SHOT										(1<<0)	//!< Start one measurement, cleared by the device when done
#define LPS22HB_CTRL_REG2_SWRESET										(1<<2)	//!< Registers to their default values, cleared by the device when done
#define LPS22HB_CTRL_REG2_I2C_DIS										(1<<3)	//!< I2C interface off
#define LPS22HB_CTRL_REG2_IF_ADD_INC									(1<<4)	//!< Register address increment on multiple byte access
#define LPS22HB_CTRL_REG2_STOP_ON_FTH									(1<<5)
#define LPS22HB_CTRL_REG2_FIFO_EN										(1<<6)
#define LPS22HB_CTRL_REG2_BOOT											(1<<7)	//!< Reload the trimming, cleared by the device when done

#define LPS22HB_CTRL_REG3_REG					0x12
#define LPS22HB_CTRL_REG3_INT_S_MASK									(3<<0)	//!< INT_DRDY pin signal, 0 for the data signals
#define LPS22HB_CTRL_REG3_DRDY											(1<<2)	//!< Data ready on the INT_DRDY pin
#define LPS22HB_CTRL_REG3_F_OVR											(1<<3)
#define LPS22HB_CTRL_REG3_F_FTH											(1<<4)
#define LPS22HB_CTRL_REG3_F_FSS5										(1<<5)
#define LPS22HB_CTRL_REG3_PP_OD											(1<<6)	//!< INT_DRDY pin open drain, else push-pull
#define LPS22HB_CTRL_REG3_INT_H_L										(1<<7)	//!< INT_DRDY pin active low

#define LPS22HB_FIFO_CTRL_REG					0x14

#define LPS22HB_RES_CONF_REG					0x1A
#define LPS22HB_RES_CONF_LC_EN											(1<<0)	//!< Low current mode, else low noise

#define LPS22HB_INT_SOURCE_REG					0x25
#define LPS22HB_INT_SOURCE_BOOT_STATUS									(1<<7)	//!< Trimming reload running

#define LPS22HB_STATUS_REG						0x27
#define LPS22HB_STATUS_P_DA												(1<<0)	//!< New pressure data
#define LPS22HB_STATUS_T_DA												(1<<1)	//!< New temperature data
#define LPS22HB_STATUS_P_OR												(1<<4)	//!< Pressure data overwritten
#define LPS22HB_STATUS_T_OR												(1<<5)	//!< Temperature data overwritten

// Pressure 24 bits then temperature 16 bits, right after STATUS
#define LPS22HB_PRESS_OUT_XL_REG				0x28
#define LPS22HB_TEMP_OUT_L_REG					0x2B

#define LPS22HB_LPFP_RES_REG					0x33	//!< Reading it resets the low pass filter

#ifdef __cplusplus

class PressLps22hb : public PressSensor, public TempSensor {
public:
	PressLps22hb();

	/**
	 * @brief	Initialize the pressure part.
	 *
	 * Cfg.OpMode SENSOR_OPMODE_SINGLE measures once on each StartSampling,
	 * else the device measures continuously at the rate of Cfg.Freq.
	 * Cfg.FilterCoeff selects the low pass filter of the pressure output by
	 * its bandwidth divider: below 9 no filter (bandwidth ODR/2), 9 to 19
	 * ODR/9, 20 and above ODR/20. The device has no oversampling setting,
	 * Cfg.PresOvrs is not used.
	 */
	virtual bool Init(const PressureSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	/**
	 * @brief	Initialize the temperature part.
	 *
	 * Cfg.Inter not 0 enables the data ready on the INT_DRDY pin with the
	 * polarity of Cfg.IntPol. Cfg.TempOvrs and Cfg.FilterCoeff are not used.
	 */
	virtual bool Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	/**
	 * @brief	Set the output data rate.
	 *
	 * @param	Freq : Frequency in mHz, 1, 10, 25, 50 or 75 Hz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual bool UpdateData();
	virtual void IntHandler();
	virtual void Read(PressureSensorData_t &Data) { PressSensor::Read(Data); }
	virtual void Read(TempSensorData_t &Data) { TempSensor::Read(Data); }

private:
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	uint8_t vOdr;			//!< ODR field of CTRL_REG1 written by Enable
	uint8_t vLpfp;			//!< Low pass filter bits of CTRL_REG1
};

extern "C" {
#endif // __cplusplus


#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __PRESS_LPS22HB_H__
