/**-------------------------------------------------------------------------
@file	mag_lis3mdl.h

@brief	Definitions for the ST LIS3MDL 3 axis magnetometer

The LIS3MDL measures the magnetic field on 3 axes, 16 bits, with a full scale
of 4, 8, 12 or 16 gauss (100 uT per gauss), on I2C or SPI. It also has a
temperature sensor, read with the field into the Temp member of the data.

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
#ifndef __MAG_LIS3MDL_H__
#define __MAG_LIS3MDL_H__

#include <stdint.h>

#include "sensors/mag_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define LIS3MDL_I2C_DEV_ADDR0			0x1C	//!< SA1 low
#define LIS3MDL_I2C_DEV_ADDR1			0x1E	//!< SA1 high

// Register address flag of a multiple byte access : on I2C, bit 7 increments
// the address. On SPI, bit 7 reads and bit 6 increments.
#define LIS3MDL_I2C_ADDR_INC			(1<<7)
#define LIS3MDL_SPI_READ				(1<<7)
#define LIS3MDL_SPI_ADDR_INC			(1<<6)

#define LIS3MDL_ADC_RANGE				32768
#define LIS3MDL_SENSITIVITY_4G			6842	//!< LSB per gauss at 4 gauss full scale
#define LIS3MDL_SENSITIVITY_8G			3421
#define LIS3MDL_SENSITIVITY_12G			2281
#define LIS3MDL_SENSITIVITY_16G			1711
#define LIS3MDL_TEMP_SENSITIVITY		8		//!< LSB per degree C
#define LIS3MDL_TEMP_OFFSET				25		//!< Degree C at output 0

#define LIS3MDL_WHO_AM_I_REG					0x0F
#define LIS3MDL_WHO_AM_I_ID												0x3D

#define LIS3MDL_CTRL_REG1_REG					0x20
#define LIS3MDL_CTRL_REG1_ST											(1<<0)	//!< Self test
#define LIS3MDL_CTRL_REG1_FAST_ODR										(1<<1)	//!< Rate above 80 Hz, set by the X and Y mode
#define LIS3MDL_CTRL_REG1_DO_MASK										(7<<2)	//!< Output data rate
#define LIS3MDL_CTRL_REG1_DO_POS										(2)		//!< 0.625 Hz << DO, 0.625 to 80 Hz
#define LIS3MDL_CTRL_REG1_OM_MASK										(3<<5)	//!< X and Y axis operating mode
#define LIS3MDL_CTRL_REG1_OM_LP											(0<<5)	//!< Low power
#define LIS3MDL_CTRL_REG1_OM_MP											(1<<5)	//!< Medium performance
#define LIS3MDL_CTRL_REG1_OM_HP											(2<<5)	//!< High performance
#define LIS3MDL_CTRL_REG1_OM_UHP										(3<<5)	//!< Ultra high performance
#define LIS3MDL_CTRL_REG1_TEMP_EN										(1<<7)	//!< Temperature sensor on

#define LIS3MDL_CTRL_REG2_REG					0x21
#define LIS3MDL_CTRL_REG2_SOFT_RST										(1<<2)	//!< Registers to their default values, cleared by the device when done
#define LIS3MDL_CTRL_REG2_REBOOT										(1<<3)	//!< Reload the trimming
#define LIS3MDL_CTRL_REG2_FS_MASK										(3<<5)	//!< Full scale
#define LIS3MDL_CTRL_REG2_FS_4G											(0<<5)
#define LIS3MDL_CTRL_REG2_FS_8G											(1<<5)
#define LIS3MDL_CTRL_REG2_FS_12G										(2<<5)
#define LIS3MDL_CTRL_REG2_FS_16G										(3<<5)

#define LIS3MDL_CTRL_REG3_REG					0x22
#define LIS3MDL_CTRL_REG3_MD_MASK										(3<<0)	//!< Conversion mode
#define LIS3MDL_CTRL_REG3_MD_CONTINUOUS									(0<<0)
#define LIS3MDL_CTRL_REG3_MD_SINGLE										(1<<0)	//!< One conversion, then power down
#define LIS3MDL_CTRL_REG3_MD_POWER_DOWN									(3<<0)	//!< Default value
#define LIS3MDL_CTRL_REG3_SIM											(1<<2)	//!< SPI 3 wire
#define LIS3MDL_CTRL_REG3_LP											(1<<5)	//!< Minimum averaging at 0.625 Hz

#define LIS3MDL_CTRL_REG4_REG					0x23
#define LIS3MDL_CTRL_REG4_BLE											(1<<1)	//!< Big endian outputs
#define LIS3MDL_CTRL_REG4_OMZ_MASK										(3<<2)	//!< Z axis operating mode
#define LIS3MDL_CTRL_REG4_OMZ_LP										(0<<2)
#define LIS3MDL_CTRL_REG4_OMZ_MP										(1<<2)
#define LIS3MDL_CTRL_REG4_OMZ_HP										(2<<2)
#define LIS3MDL_CTRL_REG4_OMZ_UHP										(3<<2)

#define LIS3MDL_CTRL_REG5_REG					0x24
#define LIS3MDL_CTRL_REG5_BDU											(1<<6)	//!< Output registers not updated until both bytes are read
#define LIS3MDL_CTRL_REG5_FAST_READ										(1<<7)	//!< High bytes only

#define LIS3MDL_STATUS_REG						0x27
#define LIS3MDL_STATUS_XDA												(1<<0)
#define LIS3MDL_STATUS_YDA												(1<<1)
#define LIS3MDL_STATUS_ZDA												(1<<2)
#define LIS3MDL_STATUS_ZYXDA											(1<<3)	//!< New data on the 3 axes
#define LIS3MDL_STATUS_ZYXOR											(1<<7)	//!< Data overwritten

// X, Y, Z then temperature, 16 bits each, right after STATUS. A multiple byte
// read returns to OUT_X_L after OUT_Z_H: the temperature needs its own read.
#define LIS3MDL_OUT_X_L_REG						0x28
#define LIS3MDL_TEMP_OUT_L_REG					0x2E

#define LIS3MDL_INT_CFG_REG						0x30
#define LIS3MDL_INT_SRC_REG						0x31

#ifdef __cplusplus

class MagLis3mdl : public MagSensor {
public:
	MagLis3mdl();

	/**
	 * @brief	Initialize the magnetometer.
	 *
	 * Full scale 4 gauss, see Scale. Cfg.Precision MAGSENSOR_PRECISION_HIGH
	 * selects the ultra high performance mode, else the low power mode.
	 * Cfg.OpMode SENSOR_OPMODE_SINGLE converts once on each StartSampling,
	 * else the device converts continuously at the rate of Cfg.Freq. The DRDY
	 * pin of the device is always on, active high: Cfg.Inter not 0 only marks
	 * the interrupt enabled, Cfg.IntPol is not used. The device has no FIFO,
	 * Cfg.bFifoEn is not used.
	 */
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	/**
	 * @brief	Set the output data rate.
	 *
	 * @param	Freq : Frequency in mHz, 625 times a power of 2 up to 80000.
	 * 				   Above 80 Hz, the rate of the mode: 155 Hz in ultra
	 * 				   high performance, 1000 Hz in low power
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint32_t SamplingFrequency() { return MagSensor::SamplingFrequency(); }
	virtual MAGSENSOR_PRECISION Precision(MAGSENSOR_PRECISION Val);
	virtual MAGSENSOR_PRECISION Precision() { return vPrecision; }

	/**
	 * @brief	Set the full scale.
	 *
	 * The gain of the calibration follows the new scale, the offset in uT
	 * stays.
	 *
	 * @param	Gauss : 4, 8, 12 or 16 gauss
	 *
	 * @return	Actual full scale in gauss
	 */
	uint16_t Scale(uint16_t Gauss);
	uint16_t Scale() { return vScale; }

	/**
	 * @brief	Calibration with the exact scale of the device.
	 *
	 * The scale of the device in nT per LSB is not an integer: these use it
	 * in floating point, the Sensitivity of the data is rounded.
	 * Offset is in LSB, as in MagSensor.
	 */
	virtual void SetCalibration(const float (&Gain)[3][3], const float (&Offset)[3]);
	virtual void ClearCalibration();

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual bool UpdateData();
	virtual void IntHandler();
	virtual bool Read(MagSensorRawData_t &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return MagSensor::Read(Data); }

protected:
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen);

private:
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	uint16_t vScale;		//!< Full scale in gauss
	uint16_t vLsbPerGauss;	//!< Scale of the outputs at this full scale
};

extern "C" {
#endif // __cplusplus


#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __MAG_LIS3MDL_H__
