/**-------------------------------------------------------------------------
@file	accel_lisxdh.h

@brief	Implementation of ST LISxDH accel. sensor

Common implementation for LIS2DH & LIS3DH

@author	Hoang Nguyen Hoan
@date	Jan. 17, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#ifndef __ACCEL_LISXDH_H__
#define __ACCEL_LISXDH_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define LISXDH_I2C_DEVADDR0				0x18	//!< 7 bits i2c address for SA0 = 0
#define LISXDH_I2C_DEVADDR1				0x19	//!< 7 bits i2c address for SA0 = 1

#define LISXDH_WHO_AM_I_REG					0x0F
#define LISXDH_WHO_AM_I_ID								0x33

#define LISXDH_STATUS_REG_AUX_REG			0x07
#define LISXDH_STATUS_REG_AUX_1DA							(1<<0)	//!< new 1-axis data avail
#define LISXDH_STATUS_REG_AUX_2DA							(1<<1)	//!< new 2-axis data avail
#define LISXDH_STATUS_REG_TDA_AUX_3DA						(1<<2)	//!< new temperature or 3-axis data avail
#define LISXDH_STATUS_REG_AUX_321DA							(1<<3)	//!< new 321-axis data avail
#define LISXDH_STATUS_REG_AUX_1OR							(1<<4)	//!< 1-axis data overrun
#define LISXDH_STATUS_REG_AUX_2OR							(1<<5)	//!< 1-axis data overrun
#define LISXDH_STATUS_REG_TOR_AUX_3OR						(1<<6)	//!< Temperature or 1-axis data overrun
#define LISXDH_STATUS_REG_AUX_321OR							(1<<7)	//!< 321-axis data overrun

// 10 bits ADC data registers
#define LISXDH_OUT_ADC1_L_REG				0x08
#define LISXDH_OUT_ADC1_H_REG				0x09
#define LISXDH_OUT_ADC2_L_REG				0x0A
#define LISXDH_OUT_ADC2_H_REG				0x0B
// See section 3.7
#define LISXDH_OUT_TEMP_ADC3_L_REG			0x0C
#define LISXDH_OUT_TEMP_ADC3_H_REG			0x0D

#define LISXDH_CTRL_REG0_REG				0x1E
#define LISXDH_CTRL_REG0_SDO_PU_DISC						(1<<7)	//!< Disconnect SDO/SA0 pullup

#define LISXDH_TEMP_CFG_REG					0x1F
#define LISXDH_TEMP_CFG_TEMP_EN								(1<<6)	//!< Temperature sensor enable
#define LISXDH_TEMP_CFG_ADC_EN								(1<<7)	//!< ADC enable

#define LISXDH_CTRL_REG1_REG				0x20
#define LISXDH_CTRL_REG1_XEN								(1<<0)	//!< X axis enable
#define LISXDH_CTRL_REG1_YEN								(1<<1)	//!< Y axis enable
#define LISXDH_CTRL_REG1_ZEN								(1<<2)	//!< Z axis enable
#define LISXDH_CTRL_REG1_LPEN								(1<<3)	//!< Low power enable
#define LISXDH_CTRL_REG1_ODR_MASK							(0xF<<4)
#define LISXDH_CTRL_REG1_ODR_PWRDWN							(0<<4)	//!< Power down
#define LISXDH_CTRL_REG1_ODR_1HZ							(1<<4)	//!< 1 Hz
#define LISXDH_CTRL_REG1_ODR_10HZ							(2<<4)	//!< 10 Hz
#define LISXDH_CTRL_REG1_ODR_25HZ							(3<<4)	//!< 25 Hz
#define LISXDH_CTRL_REG1_ODR_50HZ							(4<<4)	//!< 50 Hz
#define LISXDH_CTRL_REG1_ODR_100HZ							(5<<4)	//!< 100 Hz
#define LISXDH_CTRL_REG1_ODR_200HZ							(6<<4)	//!< 200 Hz
#define LISXDH_CTRL_REG1_ODR_400HZ							(7<<4)	//!< 400 Hz
#define LISXDH_CTRL_REG1_ODR_1620HZ							(8<<4)	//!< Low power mode 1.62 KHz
#define LISXDH_CTRL_REG1_ODR_HR_LP							(9<<4)	//!< HR/ Normal 1.344 KHz, Low power 5.376 KHz

#define LISXDH_CTRL_REG2_REG				0x21
#define LISXDH_CTRL_REG2_HP_IA1								(1<<0)	//!< High pass filter enable for AOI on int1
#define LISXDH_CTRL_REG2_HP_IA2								(1<<1)	//!< High pass filter enable for AOI on int2
#define LISXDH_CTRL_REG2_HP_CLICK							(1<<2)	//!< High pass filter enable on click function
#define LISXDH_CTRL_REG2_FDS								(1<<3)	//!< Filtered data
#define LISXDH_CTRL_REG2_HPCF_MASK							(3<<4)	//!< High pass cut off freq
#define LISXDH_CTRL_REG2_HPM_MASK							(3<<6)	//!< High pass filter mode
#define LISXDH_CTRL_REG2_HPM_NORMALRST						(0<<6)	//!< Normal mode (reset by reading register)
#define LISXDH_CTRL_REG2_HPM_REFSIG							(1<<6)
#define LISXDH_CTRL_REG2_HPM_NORMAL							(2<<6)
#define LISXDH_CTRL_REG2_HPM_AUTORST						(3<<6)	//!< Auto reset on interrupt event

#define LISXDH_CTRL_REG3_REG				0x22
#define LISXDH_CTRL_REG3_I1_OVERRUN							(1<<1)	//!< Enable Fifo overrun on int1
#define LISXDH_CTRL_REG3_I1_WTM								(1<<2)	//!< Enable Fifo watermark on int1
#define LISXDH_CTRL_REG3_I1_321DA							(1<<3)	//!< Enable 321DA on int1
#define LISXDH_CTRL_REG3_I1_ZYXDA							(1<<4)	//!< Enable ZYXDA on int1
#define LISXDH_CTRL_REG3_I1_IA2								(1<<5)	//!< Enable IA2 on int1
#define LISXDH_CTRL_REG3_I1_IA1								(1<<6)	//!< Enable IA1 on int1
#define LISXDH_CTRL_REG3_I1_CLICK							(1<<7)	//!< Enable CLICK on int1

#define LISXDH_CTRL_REG4_REG				0x23
#define LISXDH_CTRL_REG4_SIM_3WIRE							(1<<0)	//!< SPI serial 3 wire mode
#define LISXDH_CTRL_REG4_ST_MASK							(3<<1)	//!< Self test select mask
#define LISXDH_CTRL_REG4_ST_NORMAL							(0<<1)	//!< Normal operation
#define LISXDH_CTRL_REG4_ST_SELFTEST0						(1<<1)
#define LISXDH_CTRL_REG4_ST_SELFTEST1						(2<<1)
#define LISXDH_CTRL_REG4_HR_EN								(1<<3)	//!< HR mode
#define LISXDH_CTRL_REG4_FS_MASK							(3<<4)	//!< Full scale selection mask
#define LISXDH_CTRL_REG4_FS_2G								(0<<4)
#define LISXDH_CTRL_REG4_FS_4G								(1<<4)
#define LISXDH_CTRL_REG4_FS_8G								(2<<4)
#define LISXDH_CTRL_REG4_FS_16G								(3<<4)
#define LISXDH_CTRL_REG4_BLE								(1<<6)	//!< Big endian
#define LISXDH_CTRL_REG4_BDU								(1<<7)	//!< Blocking data update

#define LISXDH_CTRL_REG5_REG				0x24
#define LISXDH_CTRL_REG5_D4D_INT2							(1<<0)	//!< Enable 4D detection on int2
#define LISXDH_CTRL_REG5_LIR_INT2							(1<<1)	//!< Interrupt latched mode on int2
#define LISXDH_CTRL_REG5_D4D_INT1							(1<<2)	//!< Enable 4D detection on int1
#define LISXDH_CTRL_REG5_LIR_INT1							(1<<3)	//!< Interrupt latched mode on int1
#define LISXDH_CTRL_REG5_FIFO_EN							(1<<6)	//!< Enable Fifo
#define LISXDH_CTRL_REG5_BOOT								(1<<7)	//!< Reboot memory content

#define LISXDH_CTRL_REG6_REG				0x25
#define LISXDH_CTRL_REG6_INT_POLARITY_LOW					(1<<1)	//!< Interrupt active low
#define LISXDH_CTRL_REG6_I2_ACT								(1<<3)	//!< Enable activity on int2
#define LISXDH_CTRL_REG6_I2_BOOT							(1<<4)	//!< Enable boot on int2
#define LISXDH_CTRL_REG6_I2_IA2								(1<<5)	//!< Enable interrupt2 functions on int2
#define LISXDH_CTRL_REG6_I2_IA1								(1<<6)	//!< Enable interrupt1 function on int2
#define LISXDH_CTRL_REG6_I2_CLICK							(1<<7)	//!< Enable CLICK on int2

#define LISXDH_REFERENCE_REG				0x26	//!< Reference value for interrupt generation

#define LISXDH_STATUS_REG					0x27
#define LISXDH_STATUS_REG_XDA								(1<<0)	//!< New X-axis data avail
#define LISXDH_STATUS_REG_YDA								(1<<1)	//!< New Y-axis data avail
#define LISXDH_STATUS_REG_ZDA								(1<<2)	//!< New Z-axis data avail
#define LISXDH_STATUS_REG_ZYXDA								(1<<3)	//!< New XYZ-axis data avail
#define LISXDH_STATUS_REG_XOR								(1<<4)	//!< New X-axis data overrun
#define LISXDH_STATUS_REG_YOR								(1<<5)	//!< New Y-axis data overrun
#define LISXDH_STATUS_REG_ZOR								(1<<6)	//!< New Z-axis data overrun
#define LISXDH_STATUS_REG_ZYXOR								(1<<7)	//!< New XYZ-axis data overrun

#define LISXDH_OUT_X_L_REG					0x28
#define LISXDH_OUT_X_H_REG					0x29
#define LISXDH_OUT_Y_L_REG					0x2A
#define LISXDH_OUT_Y_H_REG					0x2B
#define LISXDH_OUT_Z_L_REG					0x2C
#define LISXDH_OUT_Z_H_REG					0x2D

#define LISXDH_FIFO_CTRL_REG				0x2E
#define LISXDH_FIFO_CTRL_FTH_MASK							(0x1F<<0)	//!<
#define LISXDH_FIFO_CTRL_TR_MASK							(1<<5)
#define LISXDH_FIFO_CTRL_TR_INT1							(0<<5)	//!< Trigger event on int1
#define LISXDH_FIFO_CTRL_TR_INT2							(1<<5)	//!< Trigger event on int2
#define LISXDH_FIFO_CTRL_FM_MASK							(3<<6)	//!< Fifo mode selection mask
#define LISXDH_FIFO_CTRL_FM_BYPASS							(0<<6)
#define LISXDH_FIFO_CTRL_FM_FIFO							(1<<6)	//!< Fifo mode
#define LISXDH_FIFO_CTRL_FM_STREAM							(2<<6)	//!< Stream mode
#define LISXDH_FIFO_CTRL_FM_STREAM_FIFO						(3<<6)	//!< Stream to fifo

#define LISXDH_FIFO_SRC_REG					0x2F
#define LISXDH_FIFO_SRC_REG_FSS_MASK						(0x1F<<0)	//!< Nb of samples in fifo
#define LISXDH_FIFO_SRC_REG_EMPTY							(1<<5)	//!< Fifo empty
#define LISXDH_FIFO_SRC_REG_OVRN_FIFO						(1<<6)	//!< Fifo overrun (full)
#define LISXDH_FIFO_SRC_REG_WTM								(1<<7)	//!< Fifo threshold reached

#define LISXDH_INT1_CFG_REG					0x30
#define LISXDH_INT1_CFG_XLIE								(1<<0)	//!< Enable interrupt on XL
#define LISXDH_INT1_CFG_XHIE								(1<<1)	//!< Enable interrupt on XH
#define LISXDH_INT1_CFG_YLIE								(1<<2)	//!< Enable interrupt on YL
#define LISXDH_INT1_CFG_YHIE								(1<<3)	//!< Enable interrupt on YH
#define LISXDH_INT1_CFG_ZLIE								(1<<4)	//!< Enable interrupt on ZL
#define LISXDH_INT1_CFG_ZHIE								(1<<5)	//!< Enable interrupt on ZH
#define LISXDH_INT1_CFG_6D									(1<<6)	//!< Enable 6 direction detection
#define LISXDH_INT1_CFG_AOI_AND								(1<<7)	//!< AND interrupt event

#define LISXDH_INT1_SRC_REG					0x31
#define LISXDH_INT1_SRC_XL									(1<<0)	//!< XL interrupt event
#define LISXDH_INT1_SRC_XH									(1<<1)	//!< XH interrupt event
#define LISXDH_INT1_SRC_YL									(1<<2)	//!< YL interrupt event
#define LISXDH_INT1_SRC_YH									(1<<3)	//!< YH interrupt event
#define LISXDH_INT1_SRC_ZL									(1<<4)	//!< ZL interrupt event
#define LISXDH_INT1_SRC_ZH									(1<<5)	//!< ZH interrupt event
#define LISXDH_INT1_SRC_IA									(1<<6)	//!< Interrupt event

#define LISXDH_INT1_THS_REG					0x32
#define LISXDH_INT1_THS_MASK								(0x7F)

#define LISXDH_INT1_DURATION_REG			0x33
#define LISXDH_INT1_DURATION_MASK							(0x7F)

#define LISXDH_INT2_CFG_REG					0x34
#define LISXDH_INT2_CFG_XLIE								(1<<0)	//!< Enable interrupt on XL
#define LISXDH_INT2_CFG_XHIE								(1<<1)	//!< Enable interrupt on XH
#define LISXDH_INT2_CFG_YLIE								(1<<2)	//!< Enable interrupt on YL
#define LISXDH_INT2_CFG_YHIE								(1<<3)	//!< Enable interrupt on YH
#define LISXDH_INT2_CFG_ZLIE								(1<<4)	//!< Enable interrupt on ZL
#define LISXDH_INT2_CFG_ZHIE								(1<<5)	//!< Enable interrupt on ZH
#define LISXDH_INT2_CFG_6D									(1<<6)	//!< Enable 6 direction detection
#define LISXDH_INT2_CFG_AOI_AND								(1<<7)	//!< AND interrupt event

#define LISXDH_INT2_SRC_REG					0x35
#define LISXDH_INT2_SRC_XL									(1<<0)	//!< XL interrupt event
#define LISXDH_INT2_SRC_XH									(1<<1)	//!< XH interrupt event
#define LISXDH_INT2_SRC_YL									(1<<2)	//!< YL interrupt event
#define LISXDH_INT2_SRC_YH									(1<<3)	//!< YH interrupt event
#define LISXDH_INT2_SRC_ZL									(1<<4)	//!< ZL interrupt event
#define LISXDH_INT2_SRC_ZH									(1<<5)	//!< ZH interrupt event
#define LISXDH_INT2_SRC_IA									(1<<6)	//!< Interrupt event

#define LISXDH_INT2_THS_REG					0x36
#define LISXDH_INT2_THS_MASK								(0x7F)

#define LISXDH_INT2_DURATION_REG			0x37
#define LISXDH_INT2_DURATION_MASK							(0x7F)

#define LISXDH_CLICK_CFG_REG				0x38
#define LISXDH_CLICK_CFG_XS									(1<<0)	//!< Enable single click interrupt on X axis
#define LISXDH_CLICK_CFG_XD									(1<<1)	//!< Enable double click interrupt on X axis
#define LISXDH_CLICK_CFG_YS									(1<<2)	//!< Enable single click interrupt on Y axis
#define LISXDH_CLICK_CFG_YD									(1<<3)	//!< Enable double click interrupt on Y axis
#define LISXDH_CLICK_CFG_ZS									(1<<4)	//!< Enable single click interrupt on Z axis
#define LISXDH_CLICK_CFG_ZD									(1<<5)	//!< Enable double click interrupt on Z axis

#define LISXDH_CLICK_SRC_REG				0x39
#define LISXDH_CLICK_SRC_X									(1<<0)	//!< Click dtected on X axis
#define LISXDH_CLICK_SRC_Y									(1<<1)	//!< Click dtected on Y axis
#define LISXDH_CLICK_SRC_Z									(1<<2)	//!< Click dtected on Z axis
#define LISXDH_CLICK_SRC_SIGN_NEG							(1<<3)	//!< Click sign negative
#define LISXDH_CLICK_SRC_SCLICK								(1<<4)	//!< Enable single click detection
#define LISXDH_CLICK_SRC_DCLICK								(1<<5)	//!< Enable double click detection
#define LISXDH_CLICK_SRC_IA									(1<<6)	//!< Click interrupt flag

#define LISXDH_CLICK_THS_REG				0x3A
#define LISXDH_CLICK_THS_THS_MASK							(0x7F)
#define LISXDH_CLICK_THS_LIR_CLICK							(1<<7)	//!< Interrupt duration

#define LISXDH_TIME_LIMIT_REG				0x3B
#define LISXDH_TIME_LIMIT_MASK								(0x7F)	//!< Click time limit

#define LISXDH_TIME_LATENCY_REG				0x3C	//!< Click time latency

#define LISXDH_TIME_WINDOW_REG				0x3D	//!< Click time window

#define LISXDH_ACT_THS_REG					0x3E
#define LISXDH_ACT_THS_MASK									(0x7F)	//!< Sleep-to-wake activation threshold

// 1 LSb = (8*1[LSb]+1)/ODR
#define LISXDH_ACT_DUR_REG					0x3F	//!< Sleep-to-wake duration

#define LISXDH_TEMP_MAX_C					127

#ifdef __cplusplus

class AccelLisxdh : public AccelSensor, public TempSensor {
public:
	/**
	 * @brief	Initialize accelerometer sensor.
	 *
	 * NOTE: This sensor must be the first to be initialized.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

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
	bool Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	uint16_t Scale(uint16_t Value);
	/**
	 * @brief	Set sampling frequency.
	 *
	 * The sampling frequency is relevant only in continuous mode.
	 *
	 * @return	Frequency in mHz (milliHerz)
	 */
	uint32_t SamplingFrequency(uint32_t Freq);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	uint32_t FilterFreq(uint32_t Freq);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	/**
	 * @brief	Power off the device completely.
	 *
	 * If supported, this will put the device in complete power down.
	 * Full re-intialization is required to re-enable the device.
	 */
	void PowerOff();
	void IntHandler();
	bool UpdateData();

	bool Read(AccelSensorRawData_t &Data) { return AccelSensor::Read(Data); }
	bool Read(AccelSensorData_t &Data) { return AccelSensor::Read(Data); }
	void Read(TempSensorData_t &Data) { TempSensor::Read(Data); }
	bool StartSampling() { return true; }

private:

	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
};

#endif // __cplusplus

/** @} End of group Sensors */

#endif // __ACCEL_LISXDH_H__
