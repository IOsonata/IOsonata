/**-------------------------------------------------------------------------
@file	ag_bmi323.h

@brief	Bosch BMI323 accel gyro implementation

This file implements only accel & gyro part of the BMI323. IMU features are
implemented in imu implementation file.

NOTE: BMI323 read always send a dummy byte first.  Se datasheet for detail.

@author	Hoang Nguyen Hoan
@date	July 18, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#ifndef __AG_BMI323_H__
#define __AG_BMI323_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define BMI323_I2C_7BITS_DEVADDR0							0x68
#define BMI323_I2C_7BITS_DEVADDR1							0x69

#define BMI323_CHIP_ID_REG          	0x0

#define BMI323_CHIP_ID                                      0x43

#define BMI323_ERR_REG					0x1

#define BMI323_ERR_REG_FATAL_ERR							(1<<0)	//!< Fatal error
#define BMI323_ERR_REG_FEAT_ENG_OVRLD						(1<<2)	//!< Overload of the feature engine detected
#define BMI323_ERR_REG_FEAT_ENG_WD							(1<<4)	//!< Watchdog timer of the feature engine triggered
#define BMI323_ERR_REG_ACC_CONF_ERR							(1<<5)	//!< Unsupported acc config
#define BMI323_ERR_REG_GYR_CONF_ERR							(1<<6)	//!< Unsupported gyro config
#define BMI323_ERR_REG_I3C_ERR0								(1<<8)	//!< SDR parity error
#define BMI323_ERR_REG_I3C_ERR3								(1<<11)	//!< S0/S1 error occurred

#define BMI323_STATUS_REG				0x2

#define BMI323_STATUS_POR_DETECTED							(1<<0)	//!< Power on Reset
#define BMI323_STATUS_DRDY_TEMP								(1<<5)	//!< Data ready for temperature
#define BMI323_STATUS_DRDY_GYR								(1<<6)	//!< Data ready for gyroscope
#define BMI323_STATUS_DRDY_ACC								(1<<7)	//!< Data ready for accelerometer

#define BMI323_ACC_DATA_X_REG			0x3
#define BMI323_ACC_DATA_Y_REG			0x4
#define BMI323_ACC_DATA_Z_REG			0x5

#define BMI323_GYR_DATA_X_REG			0x6
#define BMI323_GYR_DATA_Y_REG			0x7
#define BMI323_GYR_DATA_Z_REG			0x8

#define BMI323_TEMP_DATA_REG			0x9		//!< Temperature data : T (degree C) = this / 512 + 23

#define BMI323_SENSOR_TIME_LSW_REG		0xA		//!< Sensor time least significant word
#define BMI323_SENSOR_TIME_MSW_REG		0xB		//!< Sensor time most significant word

#define BMI323_SAT_FLAGS_REG			0xC		//!< Saturation flags
#define BMI323_SAT_FLAGS_SATF_ACC_X							(1<<0)
#define BMI323_SAT_FLAGS_SATF_ACC_Y							(1<<1)
#define BMI323_SAT_FLAGS_SATF_ACC_Z							(1<<2)
#define BMI323_SAT_FLAGS_SATF_GYR_X							(1<<3)
#define BMI323_SAT_FLAGS_SATF_GYR_Y							(1<<4)
#define BMI323_SAT_FLAGS_SATF_GYR_Z							(1<<5)

#define BMI323_INT1_STATUS_REG			0xD
#define BMI323_INT1_STATUS_NO_MOTION						(1<<0)		//!< No motion detection
#define BMI323_INT1_STATUS_ANY_MOTION						(1<<1)		//!< Any motion detection
#define BMI323_INT1_STATUS_FLAT								(1<<2)		//!< Flat detection
#define BMI323_INT1_STATUS_ORIRENTATION						(1<<3)		//!< Orientation detection
#define BMI323_INT1_STATUS_STEP_DETECTOR					(1<<4)		//!< Step detector
#define BMI323_INT1_STATUS_STEP_COUNTER						(1<<5)		//!< Step counter
#define BMI323_INT1_STATUS_SIG_MOTION						(1<<6)		//!< Significant motion detection
#define BMI323_INT1_STATUS_TILT								(1<<7)		//!< Tilt detection
#define BMI323_INT1_STATUS_TAP								(1<<8)		//!< Tap detection
#define BMI323_INT1_STATUS_I3C								(1<<9)		//!< I3C TC sync data ready
#define BMI323_INT1_STATUS_ERR_STATUS						(1<<10)		//!< Feature engine error or status change
#define BMI323_INT1_STATUS_TEMP_DRDY						(1<<11)		//!< Temperature data ready
#define BMI323_INT1_STATUS_GYR_DRDY							(1<<12)		//!< Gyroscope data ready
#define BMI323_INT1_STATUS_ACC_DRDY							(1<<13)		//!< Accelerometer data ready
#define BMI323_INT1_STATUS_FWM								(1<<14)		//!< FIFO watermark interrupt
#define BMI323_INT1_STATUS_FFULL							(1<<15)		//!< FIFO full interrupt

#define BMI323_INT2_STATUS_REG			0xE
#define BMI323_INT2_STATUS_NO_MOTION						(1<<0)		//!< No motion detection
#define BMI323_INT2_STATUS_ANY_MOTION						(1<<1)		//!< Any motion detection
#define BMI323_INT2_STATUS_FLAT								(1<<2)		//!< Flat detection
#define BMI323_INT2_STATUS_ORIRENTATION						(1<<3)		//!< Orientation detection
#define BMI323_INT2_STATUS_STEP_DETECTOR					(1<<4)		//!< Step detector
#define BMI323_INT2_STATUS_STEP_COUNTER						(1<<5)		//!< Step counter
#define BMI323_INT2_STATUS_SIG_MOTION						(1<<6)		//!< Sigmotion detection
#define BMI323_INT2_STATUS_TILT								(1<<7)		//!< Tilt detection
#define BMI323_INT2_STATUS_TAP								(1<<8)		//!< Tap detection
#define BMI323_INT2_STATUS_I3C								(1<<9)		//!< I3C TC sync data ready
#define BMI323_INT2_STATUS_ERR_STATUS						(1<<10)		//!< Feature engine error or status change
#define BMI323_INT2_STATUS_TEMP_DRDY						(1<<11)		//!< Temperature data ready
#define BMI323_INT2_STATUS_GYR_DRDY							(1<<12)		//!< Gyroscope data ready
#define BMI323_INT2_STATUS_ACC_DRDY							(1<<13)		//!< Accelerometer data ready
#define BMI323_INT2_STATUS_FWM								(1<<14)		//!< FIFO watermark interrupt
#define BMI323_INT2_STATUS_FFULL							(1<<15)		//!< FIFO full interrupt

#define BMI323_INT_STATUS_IBI_REG		0xF
#define BMI323_INT_STATUS_IBI_NO_MOTION						(1<<0)		//!< No motion detection
#define BMI323_INT_STATUS_IBI_ANY_MOTION					(1<<1)		//!< Any motion detection
#define BMI323_INT_STATUS_IBI_FLAT							(1<<2)		//!< Flat detection
#define BMI323_INT_STATUS_IBI_ORIRENTATION					(1<<3)		//!< Orientation detection
#define BMI323_INT_STATUS_IBI_STEP_DETECTOR					(1<<4)		//!< Step detector
#define BMI323_INT_STATUS_IBI_STEP_COUNTER					(1<<5)		//!< Step counter
#define BMI323_INT_STATUS_IBI_SIG_MOTION					(1<<6)		//!< Significant motion detection
#define BMI323_INT_STATUS_IBI_TILT							(1<<7)		//!< Tilt detection
#define BMI323_INT_STATUS_IBI_TAP							(1<<8)		//!< Tap detection
#define BMI323_INT_STATUS_IBI_I3C							(1<<9)		//!< I3C TC sync data ready
#define BMI323_INT_STATUS_IBI_ERR_STATUS					(1<<10)		//!< Feature engine error or status change
#define BMI323_INT_STATUS_IBI_TEMP_DRDY						(1<<11)		//!< Temperature data ready
#define BMI323_INT_STATUS_IBI_GYR_DRDY						(1<<12)		//!< Gyroscope data ready
#define BMI323_INT_STATUS_IBI_ACC_DRDY						(1<<13)		//!< Accelerometer data ready
#define BMI323_INT_STATUS_IBI_FWM							(1<<14)		//!< FIFO watermark interrupt
#define BMI323_INT_STATUS_IBI_FFULL							(1<<15)		//!< FIFO full interrupt

#define BMI323_FEATURE_IO0_REG			0x10
#define BMI323_FEATURE_IO0_NO_MOTION_X_EN					(1<<0)		//!< Enable no motion feature for X axis
#define BMI323_FEATURE_IO0_NO_MOTION_Y_EN					(1<<1)		//!< Enable no motion feature for Y axis
#define BMI323_FEATURE_IO0_NO_MOTION_Z_EN					(1<<2)		//!< Enable no motion feature for Z axis
#define BMI323_FEATURE_IO0_ANY_MOTION_X_EN					(1<<3)		//!< Enable any motion feature for X axis
#define BMI323_FEATURE_IO0_ANY_MOTION_Y_EN					(1<<4)		//!< Enable any motion feature for Y axis
#define BMI323_FEATURE_IO0_ANY_MOTION_Z_EN					(1<<5)		//!< Enable any motion feature for Z axis
#define BMI323_FEATURE_IO0_FLAT_EN							(1<<6)		//!< Enable flat feature
#define BMI323_FEATURE_IO0_ORIENTATION_EN					(1<<7)		//!< Enable orientation feature
#define BMI323_FEATURE_IO0_STEP_DETECTOR_EN					(1<<8)		//!< Enable step detector feature
#define BMI323_FEATURE_IO0_STEP_COUNTER_EN					(1<<9)		//!< Enable step counter feature
#define BMI323_FEATURE_IO0_SIG_MOTION_EN					(1<<10)		//!< Enable significant motion feature
#define BMI323_FEATURE_IO0_TILT_EN							(1<<11)		//!< Enable tilt feature
#define BMI323_FEATURE_IO0_TAP_DETECTOR_SINGLE				(1<<12)		//!< Enable single tap detector
#define BMI323_FEATURE_IO0_TAP_DETECTOR_DOUBLE				(1<<13)		//!< Enable double tap detector
#define BMI323_FEATURE_IO0_TAP_DETECTOR_TRIPLE				(1<<14)		//!< Enable triple tap detector
#define BMI323_FEATURE_IO0_I3C_SYNC_EN						(1<<15)		//!< Enable I3C TC sync feature

#define BMI323_FEATURE_IO1_REG			0x11
#define BMI323_FEATURE_IO1_ERR_STATUS_MASK					(0xF<<0)	//!< Error & status information
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_INACTIVE		(0)			//!< Feature engine still inactive
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_ACTIVATED		(1)			//!< Feature engine activated
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_CONFIG_STRING_DWNLD	(3)	//!< Configuration string download failed
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_NO_ERROR		(5)			//!< No error
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_AXIS_MAP_CMD	(6)			//!< Axis map command not processed
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_I3C_TCSYNC_ERR	(8)		//!< I3C TC sync error
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_SELFCALIB_ABORT	(9)		//!< Gyro self calibration aborted
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_SELFCALIB_IGNORE	(10)	//!< Gyro self calibration ignored
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_SELFTEST_IGNORE	(11)	//!< Acc/Gyro self test ignored
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_SELFTEST_ABORT	(12)
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_MODE_CHANGE		(13)
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_I3C_TCSYNC_EN		(14)
#define BMI323_FEATURE_IO1_ERR_STATUS_FEATURE_ILLEGAL_CHANGE	(15)
#define BMI323_FEATURE_IO1_SC_ST_COMPLETE					(1<<4)		//!< Self calibration gyro or self test acc/gyro completed
#define BMI323_FEATURE_IO1_GYR_SELFCALIB_OK					(1<<5)		//!< Gyro self calibration result OK
#define BMI323_FEATURE_IO1_SELFTTEST_OK						(1<<6)		//!< Self test result OK
#define BMI323_FEATURE_IO1_SAMPLE_RATE_ERR					(1<<7)		//!< Insufficient sample rate
#define BMI323_FEATURE_IO1_AXIS_MAP_COMPLETE				(1<<10)		//!< Axis mapping complete
#define BMI323_FEATURE_IO1_STATE_MASK						(3<<11)
#define BMI323_FEATURE_IO1_STATE_FEATURE_MODE				(0<<11)		//!< System in feature mode
#define BMI323_FEATURE_IO1_STATE_SELFCALIB					(1<<11)		//!< Self calibration in progress
#define BMI323_FEATURE_IO1_STATE_SELFTEST					(2<<11)		//!< Self test in progress
#define BMI323_FEATURE_IO1_STATE_ERROR						(3<<11)		//!< System error

#define BMI323_FEATURE_IO2_REG			0x12
#define BMI323_FEATURE_IO3_REG			0x13

#define BMI323_FEATURE_IO_STATUS_REG	0x14	//!< Feature IO synchronization
#define BMI323_FEATURE_IO_STATUS_DATA_WRITTEN				(1<<0)		//!< Data has been written by feature engine

#define BMI323_FIFO_FILL_LEVEL_REG		0x15
#define BMI323_FIFO_FILL_LEVEL_MASK							(0x7F<<0)	//!< Current fill level of fifo buffer

#define BMI323_FIFO_DATA_REG			0x16

#define BMI323_ACC_CONFIG_REG			0x20
#define BMI323_ACC_CONFIG_ODR_MASK							(0xF<<0)	//!< Acc sample rate in Hz
#define BMI323_ACC_CONFIG_ODR_0_78125HZ						(1<<0)		//!< 0.78125 Hz
#define BMI323_ACC_CONFIG_ODR_1_5625HZ						(2<<0)		//!< 1.5625 Hz
#define BMI323_ACC_CONFIG_ODR_3_125HZ						(3<<0)		//!< 3.125 Hz
#define BMI323_ACC_CONFIG_ODR_6_25HZ						(4<<0)		//!< 6.25 Hz
#define BMI323_ACC_CONFIG_ODR_12_5HZ						(5<<0)		//!< 12.5 Hz
#define BMI323_ACC_CONFIG_ODR_25HZ							(6<<0)		//!< 25 Hz
#define BMI323_ACC_CONFIG_ODR_50HZ							(7<<0)		//!< 50 Hz
#define BMI323_ACC_CONFIG_ODR_100HZ							(8<<0)		//!< 100 Hz
#define BMI323_ACC_CONFIG_ODR_200HZ							(9<<0)		//!< 200 Hz
#define BMI323_ACC_CONFIG_ODR_400HZ							(10<<0)		//!< 400 Hz
#define BMI323_ACC_CONFIG_ODR_800HZ							(11<<0)		//!< 800 Hz
#define BMI323_ACC_CONFIG_ODR_1600HZ						(12<<0)		//!< 1.6 KHz
#define BMI323_ACC_CONFIG_ODR_3200HZ						(13<<0)		//!< 3.2 KHz
#define BMI323_ACC_CONFIG_ODR_6400HZ 						(14<<0)		//!< 6.4 KHz
#define BMI323_ACC_CONFIG_RANGE_MASK						(7<<4)		//!< Acc Full scale resolution
#define BMI323_ACC_CONFIG_RANGE_2G							(0<<4)
#define BMI323_ACC_CONFIG_RANGE_4G							(1<<4)
#define BMI323_ACC_CONFIG_RANGE_8G							(2<<4)
#define BMI323_ACC_CONFIG_RANGE_16G							(3<<4)
#define BMI323_ACC_CONFIG_BW_QUARTER						(1<<7)		//!< -3dB cutoff freq at acc_odr/4
#define BMI323_ACC_CONFIG_AVG_NUM_MASK						(1<<8)		//!< Number of samples to be average
#define BMI323_ACC_CONFIG_AVG_NUM_NONE						(0<<8)		//!< No averaging
#define BMI323_ACC_CONFIG_AVG_NUM_2							(1<<8)		//!< averaging 2 samples
#define BMI323_ACC_CONFIG_AVG_NUM_4							(2<<8)		//!< averaging 4 samples
#define BMI323_ACC_CONFIG_AVG_NUM_8							(3<<8)		//!< averaging 8 samples
#define BMI323_ACC_CONFIG_AVG_NUM_16						(4<<8)		//!< averaging 16 samples
#define BMI323_ACC_CONFIG_AVG_NUM_32						(5<<8)		//!< averaging 32 samples
#define BMI323_ACC_CONFIG_AVG_NUM_64						(6<<8)		//!< averaging 64 samples
#define BMI323_ACC_CONFIG_MODE_MASK							(3<<12)		//!< Operation mode for acc
#define BMI323_ACC_CONFIG_MODE_DISABLE						(0<<12)		//!< Disable acc
#define BMI323_ACC_CONFIG_MODE_DUTYCYCLE_EN					(3<<12)		//!< Enable acc with sensing duty cycle
#define BMI323_ACC_CONFIG_MODE_CONT_EN						(4<<12)		//!< Enable acc in continuous mode with reduce current
#define BMI323_ACC_CONFIG_MODE_HIGHPERF_EN					(7<<12)		//!< Enable acc in high performance mode

#define BMI323_GYR_CONFIG_REG			0x21
#define BMI323_GYR_CONFIG_ODR_MASK							(0xF<<0)	//!< Gyro sample rate in Hz
#define BMI323_GYR_CONFIG_RANGE_MASK						(7<<4)		//!< Gyro Full scale resolution
#define BMI323_GYR_CONFIG_RANGE_125							(0<<4)		//!< 125 degree/s
#define BMI323_GYR_CONFIG_RANGE_250							(1<<4)
#define BMI323_GYR_CONFIG_RANGE_500							(2<<4)
#define BMI323_GYR_CONFIG_RANGE_1000						(3<<4)
#define BMI323_GYR_CONFIG_RANGE_2000						(4<<4)
#define BMI323_GYR_CONFIG_BW_QUARTER						(1<<7)		//!< -3dB cutoff freq at acc_odr/4
#define BMI323_GYR_CONFIG_AVG_NUM_MASK						(1<<8)		//!< Number of samples to be average
#define BMI323_GYR_CONFIG_AVG_NUM_NONE						(0<<8)		//!< No averaging
#define BMI323_GYR_CONFIG_AVG_NUM_2							(1<<8)		//!< averaging 2 samples
#define BMI323_GYR_CONFIG_AVG_NUM_4							(2<<8)		//!< averaging 4 samples
#define BMI323_GYR_CONFIG_AVG_NUM_8							(3<<8)		//!< averaging 8 samples
#define BMI323_GYR_CONFIG_AVG_NUM_16						(4<<8)		//!< averaging 16 samples
#define BMI323_GYR_CONFIG_AVG_NUM_32						(5<<8)		//!< averaging 32 samples
#define BMI323_GYR_CONFIG_AVG_NUM_64						(6<<8)		//!< averaging 64 samples
#define BMI323_GYR_CONFIG_MODE_MASK							(3<<12)		//!< Operation mode for gyro
#define BMI323_GYR_CONFIG_MODE_DISABLE						(0<<12)		//!< Disable gyro
#define BMI323_GYR_CONFIG_MODE_DUTYCYCLE_EN					(3<<12)		//!< Enable gyro with sensing duty cycle
#define BMI323_GYR_CONFIG_MODE_CONT_EN						(4<<12)		//!< Enable gyro in continuous mode with reduce current
#define BMI323_GYR_CONFIG_MODE_HIGHPERF_EN					(7<<12)		//!< Enable gyro in high performance mode

#define BMI323_ALT_ACC_CONFIG_REG		0x28
#define BMI323_ALT_GYR_CONFIG_REG		0x29
#define BMI323_ALT_CONFIG_REG			0x2A
#define BMI323_ALT_CONFIG_ALT_ACC_EN						(1<<0)		//!<
#define BMI323_ALT_CONFIG_ALT_GYR_EN						(1<<4)		//!<
#define BMI323_ALT_CONFIG_ALT_RST_CONF_WRITE_EN				(1<<8)		//!<

#define BMI323_ALT_STATUS_REG			0x2B
#define BMI323_ALT_STATUS_ACC_ACTIVE						(1<<0)
#define BMI323_ALT_STATUS_GYR_ACTIVE						(1<<1)

#define BMI323_FIFO_WATERMARK_REG		0x35
#define BMI323_FIFO_WATERMARK_MASK							(0x3F<<0)

#define BMI323_FIFO_CONFIG_REG			0x36
#define BMI323_FIFO_CONFIG_STOP_ON_FULL						(1<<0)		//!<
#define BMI323_FIFO_CONFIG_TIME_EN							(1<<8)		//!< Write sensor time to fifo
#define BMI323_FIFO_CONFIG_ACC_EN							(1<<9)		//!<
#define BMI323_FIFO_CONFIG_GYR_EN							(1<<10)
#define BMI323_FIFO_CONFIG_TEMP_EN							(1<<11)		//!< Temperature

#define BMI323_FIFO_CTRL_REG			0x37
#define BMI323_FIFO_CTRL_FLUSH								(1<<0)		//!< clear fifo

#define BMI323_IO_CTRL_REG				0x38
#define BMI323_IO_CTRL_INT1_ACTIVE_LOW						(0<<0)		//!<
#define BMI323_IO_CTRL_INT1_ACTIVE_HIGH						(1<<0)		//!<
#define BMI323_IO_CTRL_INT1_PUSHPULL						(0<<1)
#define BMI323_IO_CTRL_INT1_OPENDRAIN						(1<<1)
#define BMI323_IO_CTRL_INT1_OUTPUT_EN						(1<<2)		//!< Enable output INT1
#define BMI323_IO_CTRL_INT2_ACTIVE_LOW						(0<<8)		//!<
#define BMI323_IO_CTRL_INT2_ACTIVE_HIGH						(1<<8)		//!<
#define BMI323_IO_CTRL_INT2_PUSHPULL						(0<<9)
#define BMI323_IO_CTRL_INT2_OPENDRAIN						(1<<9)
#define BMI323_IO_CTRL_INT2_OUTPUT_EN						(1<<10)		//!< Enable output INT1

#define BMI323_INT_CONFIG_REG			0x39
#define BMI323_INT_CONFIG_LATCHED							(1<<0)

#define BMI323_INT_MAP1_REG				0x3A
#define BMI323_INT_MAP1_NO_MOTION_MASK						(3<<0)		//!<
#define BMI323_INT_MAP1_NO_MOTION_DIS						(0<<0)		//!< Int disable
#define BMI323_INT_MAP1_NO_MOTION_INT1						(1<<0)		//!< Mapped to Int1
#define BMI323_INT_MAP1_NO_MOTION_INT2						(2<<0)		//!< Mapped to Int2
#define BMI323_INT_MAP1_NO_MOTION_IBI						(3<<0)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_ANY_MOTION_MASK						(3<<2)
#define BMI323_INT_MAP1_ANY_MOTION_MOTION_DIS				(0<<2)		//!< Int disable
#define BMI323_INT_MAP1_ANY_MOTION_MOTION_INT1				(1<<2)		//!< Mapped to Int1
#define BMI323_INT_MAP1_ANY_MOTION_MOTION_INT2				(2<<2)		//!< Mapped to Int2
#define BMI323_INT_MAP1_ANY_MOTION_MOTION_IBI				(3<<2)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_FLAT_MASK							(3<<4)
#define BMI323_INT_MAP1_FLAT_MOTION_DIS						(0<<4)		//!< Int disable
#define BMI323_INT_MAP1_FLAT_MOTION_INT1					(1<<4)		//!< Mapped to Int1
#define BMI323_INT_MAP1_FLAT_MOTION_INT2					(2<<4)		//!< Mapped to Int2
#define BMI323_INT_MAP1_FLAT_MOTION_IBI						(3<<4)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_ORIENTATION_MASK					(3<<6)
#define BMI323_INT_MAP1_ORIENTATION_DIS						(0<<6)		//!< Int disable
#define BMI323_INT_MAP1_ORIENTATION_INT1					(1<<6)		//!< Mapped to Int1
#define BMI323_INT_MAP1_ORIENTATION_INT2					(2<<6)		//!< Mapped to Int2
#define BMI323_INT_MAP1_ORIENTATION_IBI						(3<<6)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_STEP_DETECTOR_MASK					(3<<8)
#define BMI323_INT_MAP1_STEP_DETECTOR_DIS					(0<<8)		//!< Int disable
#define BMI323_INT_MAP1_STEP_DETECTOR_INT1					(1<<8)		//!< Mapped to Int1
#define BMI323_INT_MAP1_STEP_DETECTOR_INT2					(2<<8)		//!< Mapped to Int2
#define BMI323_INT_MAP1_STEP_DETECTOR_IBI					(3<<8)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_STEP_COUNTER_MASK					(3<<10)
#define BMI323_INT_MAP1_STEP_COUNTER_DIS					(0<<10)		//!< Int disable
#define BMI323_INT_MAP1_STEP_COUNTER_INT1					(1<<10)		//!< Mapped to Int1
#define BMI323_INT_MAP1_STEP_COUNTER_INT2					(2<<10)		//!< Mapped to Int2
#define BMI323_INT_MAP1_STEP_COUNTER_IBI					(3<<10)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_SIG_MOTION_MASK						(3<<12)
#define BMI323_INT_MAP1_SIG_MOTION_DIS						(0<<12)		//!< Int disable
#define BMI323_INT_MAP1_SIG_MOTION_INT1						(1<<12)		//!< Mapped to Int1
#define BMI323_INT_MAP1_SIG_MOTION_INT2						(2<<12)		//!< Mapped to Int2
#define BMI323_INT_MAP1_SIG_MOTION_IBI						(3<<12)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP1_TITL_MASK							(3<<14)
#define BMI323_INT_MAP1_TITL_DIS							(0<<14)		//!< Int disable
#define BMI323_INT_MAP1_TITL_INT1							(1<<14)		//!< Mapped to Int1
#define BMI323_INT_MAP1_TITL_INT2							(2<<14)		//!< Mapped to Int2
#define BMI323_INT_MAP1_TITL_IBI							(3<<14)		//!< Mapped to I3C IBI

#define BMI323_INT_MAP2_REG				0x3B
#define BMI323_INT_MAP2_TAP_MASK							(3<<0)
#define BMI323_INT_MAP2_TAP_DIS								(0<<0)		//!< Int disable
#define BMI323_INT_MAP2_TAP_INT1							(1<<0)		//!< Mapped to Int1
#define BMI323_INT_MAP2_TAP_INT2							(2<<0)		//!< Mapped to Int2
#define BMI323_INT_MAP2_TAP_IBI								(3<<0)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_I3C_MASK							(3<<2)
#define BMI323_INT_MAP2_I3C_DIS								(0<<2)		//!< Int disable
#define BMI323_INT_MAP2_I3C_INT1							(1<<2)		//!< Mapped to Int1
#define BMI323_INT_MAP2_I3C_INT2							(2<<2)		//!< Mapped to Int2
#define BMI323_INT_MAP2_I3C_IBI								(3<<2)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_ERR_STATUS_MASK						(3<<4)
#define BMI323_INT_MAP2_ERR_STATUS_DIS						(0<<4)		//!< Int disable
#define BMI323_INT_MAP2_ERR_STATUS_INT1						(1<<4)		//!< Mapped to Int1
#define BMI323_INT_MAP2_ERR_STATUS_INT2						(2<<4)		//!< Mapped to Int2
#define BMI323_INT_MAP2_ERR_STATUS_IBI						(3<<4)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_TEMP_DRDY_MASK						(3<<6)
#define BMI323_INT_MAP2_TEMP_DRDY_DIS						(0<<6)		//!< Int disable
#define BMI323_INT_MAP2_TEMP_DRDY_INT1						(1<<6)		//!< Mapped to Int1
#define BMI323_INT_MAP2_TEMP_DRDY_INT2						(2<<6)		//!< Mapped to Int2
#define BMI323_INT_MAP2_TEMP_DRDY_IBI						(3<<6)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_GYR_DRDY_MASK						(3<<8)
#define BMI323_INT_MAP2_GYR_DRDY_DIS						(0<<8)		//!< Int disable
#define BMI323_INT_MAP2_GYR_DRDY_INT1						(1<<8)		//!< Mapped to Int1
#define BMI323_INT_MAP2_GYR_DRDY_INT2						(2<<8)		//!< Mapped to Int2
#define BMI323_INT_MAP2_GYR_DRDY_IBI						(3<<8)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_ACC_DRDY_MASK						(3<<10)
#define BMI323_INT_MAP2_ACC_DRDY_DIS						(0<<10)		//!< Int disable
#define BMI323_INT_MAP2_ACC_DRDY_INT1						(1<<10)		//!< Mapped to Int1
#define BMI323_INT_MAP2_ACC_DRDY_INT2						(2<<10)		//!< Mapped to Int2
#define BMI323_INT_MAP2_ACC_DRDY_IBI						(3<<10)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_FIFO_WATERMARK_MASK					(3<<12)
#define BMI323_INT_MAP2_FIFO_WATERMARK_DIS					(0<<12)		//!< Int disable
#define BMI323_INT_MAP2_FIFO_WATERMARK_INT1					(1<<12)		//!< Mapped to Int1
#define BMI323_INT_MAP2_FIFO_WATERMARK_INT2					(2<<12)		//!< Mapped to Int2
#define BMI323_INT_MAP2_FIFO_WATERMARK_IBI					(3<<12)		//!< Mapped to I3C IBI
#define BMI323_INT_MAP2_FIFO_FULL_MASK						(3<<14)
#define BMI323_INT_MAP2_FIFO_FULL_DIS						(0<<14)		//!< Int disable
#define BMI323_INT_MAP2_FIFO_FULL_INT1						(1<<14)		//!< Mapped to Int1
#define BMI323_INT_MAP2_FIFO_FULL_INT2						(2<<14)		//!< Mapped to Int2
#define BMI323_INT_MAP2_FIFO_FULL_IBI						(3<<14)		//!< Mapped to I3C IBI

#define BMI323_FEATURE_CTRL_REG			0x40
#define BMI323_FEATURE_CTRL_ENGINE_EN						(1<<0)		//!< Enable feature engine

#define BMI323_FEATURE_DATA_ADDR_REG	0x41
#define BMI323_FEATURE_DATA_ADDR_MASK						(0x7F)

#define BMI323_FEATURE_DATA_TX_REG		0x42
#define BMI323_FEATURE_DATA_STATUS_REG	0x43
#define BMI323_FEATURE_DATA_STATUS_OUTBOUND_ERR				(1<<0)		//!< Too much data written or read from feature engine
#define BMI323_FEATURE_DATA_STATUS_TX_READY					(1<<1)		//!<

#define BMI323_FEATURE_ENGINE_STATUS_REG	0x45
#define BMI323_FEATURE_ENGINE_STATUS_SLEEP					(1<<0)		//!< Feature engine halted or sleep
#define BMI323_FEATURE_ENGINE_STATUS_OVERLOAD				(1<<1)		//!< Data transfer on going
#define BMI323_FEATURE_ENGINE_STATUS_DATA_TX_ACTIVE			(1<<3)
#define BMI323_FEATURE_ENGINE_STATUS_DISABLED_BY_HOST		(1<<4)
#define BMI323_FEATURE_ENGINE_STATUS_WATCHDOG_NOT_ACK		(1<<5)

#define BMI323_FEATURE_EVENT_EXT_REG		0x47
#define BMI323_FEATURE_EVENT_EXT_ORIENT_MASK					(3<<0)
#define BMI323_FEATURE_EVENT_EXT_ORIENT_PORTRAIT				(0<<0)
#define BMI323_FEATURE_EVENT_EXT_ORIENT_LANDSCAPE				(1<<0)
#define BMI323_FEATURE_EVENT_EXT_ORIENT_PORTRAIT_UPSIDE_DWN		(2<<0)
#define BMI323_FEATURE_EVENT_EXT_ORIENT_LANDSCAPE_RIGHT			(3<<0)
#define BMI323_FEATURE_EVENT_EXT_ORIENT_FACEUP					(0<<2)
#define BMI323_FEATURE_EVENT_EXT_ORIENT_FACEDWN					(1<<2)
#define BMI323_FEATURE_EVENT_EXT_SINGLE_TAP						(1<<3)		//!< Single tap detected
#define BMI323_FEATURE_EVENT_EXT_DOUBLE_TAP						(1<<4)		//!< Double tap detected
#define BMI323_FEATURE_EVENT_EXT_TRIPLE_TAP						(1<<5)		//!< Triple tap detected

#define BMI323_IO_PDN_CTRL_REG				0x4F
#define BMI323_IO_PDN_CTRL_ANAIO_PDN_DIS						(1<<0)		//!< Disable pulldown on PIN2 & PIN3

#define BMI323_IO_SPI_IF_REG				0x50
#define BMI323_IO_SPI_IF_3WIRE_EN								(1<<0)		//!< Enable 3 wire SPI

#define BMI323_IO_PAD_STRENGTH_REG			0x51
#define BMI323_IO_PAD_STRENGTH_IF_DRV_MASK						(7<<0)
#define BMI323_IO_PAD_STRENGTH_IF_I2C_BOOST						(1<<3)		//!< Enable drive strength SDX

#define BMI323_IO_I2C_IF_REG				0x52
#define BMI323_IO_I2C_IF_WATCHDOG_TIMER_SEL						(1<<0)		//!<
#define BMI323_IO_I2C_IF_WATCHDOG_TIMER_EN						(1<<1)

#define BMI323_IO_ODR_DEVIATION_REG			0x53
#define BMI323_IO_ODR_DEVIATION_MASK							(0x1F<<0)

#define BMI323_ACC_DP_OFF_X_REG				0x60
#define BMI323_ACC_DP_OFF_X_MASK								(0x3FFF)

#define BMI323_ACC_DP_DGAIN_X_REG			0x61
#define BMI323_ACC_DP_DGAIN_X_MASK								(0xFF)

#define BMI323_ACC_DP_OFF_Y_REG				0x62
#define BMI323_ACC_DP_OFF_Y_MASK								(0x3FFF)

#define BMI323_ACC_DP_DGAIN_Y_REG			0x63
#define BMI323_ACC_DP_DGAIN_Y_MASK								(0xFF)

#define BMI323_ACC_DP_OFF_Z_REG				0x64
#define BMI323_ACC_DP_OFF_Z_MASK								(0x3FFF)

#define BMI323_ACC_DP_DGAIN_Z_REG			0x65
#define BMI323_ACC_DP_DGAIN_Z_MASK								(0xFF)

#define BMI323_GYR_DP_OFF_X_REG				0x66
#define BMI323_GYR_DP_OFF_X_MASK								(0x3FFF)

#define BMI323_GYR_DP_DGAIN_X_REG			0x67
#define BMI323_GYR_DP_DGAIN_X_MASK								(0xFF)

#define BMI323_GYR_DP_OFF_Y_REG				0x68
#define BMI323_GYR_DP_OFF_Y_MASK								(0x3FFF)

#define BMI323_GYR_DP_DGAIN_Y_REG			0x69
#define BMI323_GYR_DP_DGAIN_Y_MASK								(0xFF)

#define BMI323_GYR_DP_OFF_Z_REG				0x6A
#define BMI323_GYR_DP_OFF_Z_MASK								(0x3FFF)

#define BMI323_GYR_DP_DGAIN_Z_REG			0x6B
#define BMI323_GYR_DP_DGAIN_Z_MASK								(0xFF)

#define BMI323_I3C_TC_SYNC_TPH_REG			0x70
#define BMI323_I3C_TC_SYNC_TU_REG			0x71
#define BMI323_I3C_TC_SYNC_TU_MASK								(0xFF)

#define BMI323_I3C_TC_SYNC_ODR_REG			0x72
#define BMI323_I3C_TC_SYNC_ODR_MASK								(0xFF)

#define BMI323_CMD_REG						0x7E
#define BMI323_CMD_TRIG_SELFTEST								(0x100)
#define BMI323_CMD_TRIG_SELFCALIB								(0x101)
#define BMI323_CMD_ABORT										(0x200)
#define BMI323_CMD_I3C_UPDATE_CONFIG							(0x201)
#define BMI323_CMD_AXIS_MAP_UPDATE								(0x300)
#define BMI323_CMD_SOFTRESET									(0xDEAF)

#define BMI323_CFG_RES_REG					0x7F
#define BMI323_CFG_RES_VALUE_ONE_MASK							(0x1F)
#define BMI323_CFG_RES_VALUE_TWO_MASK							(0x3<<14)

#define BMI323_ADC_RANGE				0x7FFF		// 16 Bits
#define BMI323_ACC_DUMMY_X				0x7F01
#define BMI323_GYR_DUMMY_X				0x7F02
#define BMI323_TEMP_DUMMY				-32768	//0x8000

#define BMI323_FIFO_DATA_FLAG_ACC					(1<<0)	//!< Fifo contains Acc data
#define BMI323_FIFO_DATA_FLAG_GYR					(1<<1)	//!< Fifo contains Gyr data
#define BMI323_FIFO_DATA_FLAG_TEMP					(1<<2)	//!< Fifo contains temperature data
#define BMI323_FIFO_DATA_FLAG_TIME					(1<<3)	//!< Fifo contains timer data

#define BMI323_ACCEL_IDX		0
#define BMI323_GYRO_IDX			1
#define BMI323_TEMP_IDX			2
#define BMI323_NB_SENSOR		3

#ifdef __cplusplus

class AccelBmi323 : public AccelSensor {
public:
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint8_t Scale(uint8_t Value);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t FilterFreq(uint32_t Freq);
	virtual bool Enable();
	virtual void Disable();

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	virtual uint8_t FifoDataFlag() = 0;
	virtual void FifoDataFlagSet(uint8_t Flag) = 0;
	virtual void FifoDataFlagClr(uint8_t Flag) = 0;
};

class GyroBmi323 : public GyroSensor {
public:
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint32_t Sensitivity(uint32_t Value);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t FilterFreq(uint32_t Freq);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset() {}

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	virtual uint8_t FifoDataFlag() = 0;
	virtual void FifoDataFlagSet(uint8_t Flag) = 0;
	virtual void FifoDataFlagClr(uint8_t Flag) = 0;
};

class TempBmi323 : public TempSensor {
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
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	virtual uint8_t FifoDataFlag() = 0;
	virtual void FifoDataFlagSet(uint8_t Flag) = 0;
	virtual void FifoDataFlagClr(uint8_t Flag) = 0;
};

class AgBmi323 : public AccelBmi323, public GyroBmi323, public TempBmi323 {
public:
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[BMI323_ACCEL_IDX] = AccelBmi323::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[BMI323_ACCEL_IDX];
	}
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[BMI323_GYRO_IDX] = GyroBmi323::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[BMI323_GYRO_IDX];
	}

	virtual bool Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) {
		vbSensorEnabled[BMI323_TEMP_IDX] = TempBmi323::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[BMI323_TEMP_IDX];
	}

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool Read(AccelSensorRawData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(GyroSensorRawData_t &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return GyroSensor::Read(Data); }
	virtual void IntHandler();
	virtual bool StartSampling() { return true; }

	bool UpdateData();
protected:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen) {
		return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	bool vbInitialized;
	bool vbSensorEnabled[BMI323_NB_SENSOR];

private:
	virtual uint8_t FifoDataFlag() { return vFifoDataFlag; }
	virtual void FifoDataFlagSet(uint8_t Flag);//  { vFifoDataFlag = (vFifoDataFlag & ~Flag) | Flag; }
	virtual void FifoDataFlagClr(uint8_t Flag);//  { vFifoDataFlag = (vFifoDataFlag & ~Flag); }

	uint8_t vFifoDataFlag;	// Fifo frame is dependent on enabled features
	size_t vFifoFrameSize;	// Data word count
	uint16_t vPrevTime;
	uint64_t vRollover;
};

#endif // __cplusplus

/** @} End of group Sensors */

#endif	// __AG_BMI323_H__
