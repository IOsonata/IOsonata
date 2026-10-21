/**-------------------------------------------------------------------------
@file	ag_lsm6dsl.h

@brief	Definitions for the ST LSM6DSL accel & gyro sensor

The LSM6DSL is a 3 axis accelerometer and 3 axis gyroscope with a temperature
sensor, on I2C or SPI. This implementation reads the output registers on the
data ready status or interrupt, the FIFO is not used.

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
#ifndef __AG_LSM6DSL_H__
#define __AG_LSM6DSL_H__

#include <stdint.h>

#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define LSM6DSL_I2C_DEV_ADDR0			0x6A	// SA0 = 0
#define LSM6DSL_I2C_DEV_ADDR1			0x6B	// SA0 = 1

#define LSM6DSL_DRDY_PULSE_CFG_G_REG			0x0B
#define LSM6DSL_DRDY_PULSE_CFG_G_DRDY_PULSED							(1<<7)	//!< Data ready on the INT pins as 75 us pulses, else latched

#define LSM6DSL_INT1_CTRL_REG					0x0D
#define LSM6DSL_INT1_CTRL_DRDY_XL										(1<<0)	//!< Accelerometer data ready on INT1
#define LSM6DSL_INT1_CTRL_DRDY_G										(1<<1)	//!< Gyroscope data ready on INT1

#define LSM6DSL_INT2_CTRL_REG					0x0E
#define LSM6DSL_INT2_CTRL_DRDY_XL										(1<<0)	//!< Accelerometer data ready on INT2
#define LSM6DSL_INT2_CTRL_DRDY_G										(1<<1)	//!< Gyroscope data ready on INT2
#define LSM6DSL_INT2_CTRL_DRDY_TEMP										(1<<2)	//!< Temperature data ready on INT2

#define LSM6DSL_WHO_AM_I_REG					0x0F
#define LSM6DSL_WHO_AM_I_ID												0x6A

#define LSM6DSL_CTRL1_XL_REG					0x10
#define LSM6DSL_CTRL1_XL_BW0_XL											(1<<0)	//!< Analog chain bandwidth : 0 - 1.5 kHz, 1 - 400 Hz
#define LSM6DSL_CTRL1_XL_LPF1_BW_SEL									(1<<1)	//!< Digital LPF1 bandwidth selection
#define LSM6DSL_CTRL1_XL_FS_XL_MASK										(3<<2)	//!< Full scale
#define LSM6DSL_CTRL1_XL_FS_XL_2G										(0<<2)
#define LSM6DSL_CTRL1_XL_FS_XL_16G										(1<<2)
#define LSM6DSL_CTRL1_XL_FS_XL_4G										(2<<2)
#define LSM6DSL_CTRL1_XL_FS_XL_8G										(3<<2)
#define LSM6DSL_CTRL1_XL_ODR_XL_MASK									(0xF<<4)//!< Output data rate, 0 powers down
#define LSM6DSL_CTRL1_XL_ODR_XL_POS										(4)

#define LSM6DSL_CTRL2_G_REG						0x11
#define LSM6DSL_CTRL2_G_FS_125											(1<<1)	//!< 125 dps full scale, overrides FS_G
#define LSM6DSL_CTRL2_G_FS_G_MASK										(3<<2)	//!< Full scale
#define LSM6DSL_CTRL2_G_FS_G_250										(0<<2)
#define LSM6DSL_CTRL2_G_FS_G_500										(1<<2)
#define LSM6DSL_CTRL2_G_FS_G_1000										(2<<2)
#define LSM6DSL_CTRL2_G_FS_G_2000										(3<<2)
#define LSM6DSL_CTRL2_G_ODR_G_MASK										(0xF<<4)//!< Output data rate, 0 powers down
#define LSM6DSL_CTRL2_G_ODR_G_POS										(4)

// Output data rate codes of ODR_XL and ODR_G
#define LSM6DSL_ODR_POWER_DOWN											0
#define LSM6DSL_ODR_12_5HZ												1
#define LSM6DSL_ODR_26HZ												2
#define LSM6DSL_ODR_52HZ												3
#define LSM6DSL_ODR_104HZ												4
#define LSM6DSL_ODR_208HZ												5
#define LSM6DSL_ODR_416HZ												6
#define LSM6DSL_ODR_833HZ												7
#define LSM6DSL_ODR_1660HZ												8
#define LSM6DSL_ODR_3330HZ												9
#define LSM6DSL_ODR_6660HZ												10

#define LSM6DSL_CTRL3_C_REG						0x12
#define LSM6DSL_CTRL3_C_SW_RESET										(1<<0)	//!< Software reset, cleared by the device when done
#define LSM6DSL_CTRL3_C_BLE												(1<<1)	//!< Big endian data
#define LSM6DSL_CTRL3_C_IF_INC											(1<<2)	//!< Register address increment on multiple byte access
#define LSM6DSL_CTRL3_C_SIM												(1<<3)	//!< SPI 3 wire
#define LSM6DSL_CTRL3_C_PP_OD											(1<<4)	//!< INT pins open drain, else push-pull
#define LSM6DSL_CTRL3_C_H_LACTIVE										(1<<5)	//!< INT pins active low
#define LSM6DSL_CTRL3_C_BDU												(1<<6)	//!< Output registers not updated until both bytes are read
#define LSM6DSL_CTRL3_C_BOOT											(1<<7)	//!< Reboot memory content

#define LSM6DSL_CTRL4_C_REG						0x13
#define LSM6DSL_CTRL4_C_LPF1_SEL_G										(1<<1)	//!< Enable the gyroscope LPF1
#define LSM6DSL_CTRL4_C_I2C_DISABLE										(1<<2)	//!< Disable the I2C interface
#define LSM6DSL_CTRL4_C_DRDY_MASK										(1<<3)	//!< Mask data ready until the filters have settled

#define LSM6DSL_STATUS_REG						0x1E
#define LSM6DSL_STATUS_XLDA												(1<<0)	//!< New accelerometer data
#define LSM6DSL_STATUS_GDA												(1<<1)	//!< New gyroscope data
#define LSM6DSL_STATUS_TDA												(1<<2)	//!< New temperature data

// Temperature, gyroscope X, Y, Z then accelerometer X, Y, Z, 16 bits each
#define LSM6DSL_OUT_TEMP_L_REG					0x20
#define LSM6DSL_OUTX_L_G_REG					0x22
#define LSM6DSL_OUTX_L_XL_REG					0x28

// Raw value of the full scale. The accelerometer sensitivity is 0.061 mg per
// LSB at 2 g. The gyroscope sensitivity is 4.375 mdps per LSB at 125 dps up to
// 70 mdps at 2000 dps, a full scale of 28571 LSB.
#define LSM6DSL_ACCEL_ADC_RANGE			32768
#define LSM6DSL_GYRO_ADC_RANGE			28571

// Temperature : 256 LSB per degree C, 0 at 25 C
#define LSM6DSL_TEMP_SENSITIVITY		256
#define LSM6DSL_TEMP_OFFSET				25

#ifdef __cplusplus

class AccelLsm6dsl : public AccelSensor {
public:
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint16_t Scale(uint16_t Value);
	virtual bool Enable();
	virtual void Disable();

protected:
	uint8_t vOdr;		//!< ODR_XL code written by Enable

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer) = 0;
};

class GyroLsm6dsl : public GyroSensor {
public:
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint16_t Sensitivity(uint16_t Value);
	virtual bool Enable();
	virtual void Disable();

protected:
	uint8_t vOdr;		//!< ODR_G code written by Enable

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer) = 0;
};

class TempLsm6dsl : public TempSensor {
public:
	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	CfgData : Reference to configuration data
	 * @param	pIntrf 	: Pointer to interface to the sensor.
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 * @param	pTimer	: Pointer to timer for retrieval of time stamp
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	virtual bool Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	/**
	 * @brief	Power on or wake up device
	 *
	 * The temperature is measured while the accelerometer or the gyroscope
	 * is enabled.
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * @return	None
	 */
	virtual void Disable();

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer) = 0;
};

class AgLsm6dsl : public AccelLsm6dsl, public GyroLsm6dsl, public TempLsm6dsl {
public:
	AgLsm6dsl();

	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[0] = AccelLsm6dsl::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[0];
	}
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[1] = GyroLsm6dsl::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[1];
	}
	virtual bool Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) {
		vbSensorEnabled[2] = TempLsm6dsl::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[2];
	}

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool Read(AccelSensorRawData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(GyroSensorRawData_t &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return GyroSensor::Read(Data); }
	virtual void Read(TempSensorData_t &Data) { TempSensor::Read(Data); }
	virtual void IntHandler();
	virtual bool StartSampling() { return true; }

	/**
	 * @brief	Read the new data
	 *
	 * Reads the status then the output registers, and updates the data of
	 * each sensor that has a new sample.
	 *
	 * @return	true - New data was read
	 */
	virtual bool UpdateData();

protected:
	bool vbSensorEnabled[3];

private:
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer);
};

extern "C" {
#endif // __cplusplus


#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __AG_LSM6DSL_H__
