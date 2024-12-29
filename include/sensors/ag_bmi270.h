/**-------------------------------------------------------------------------
@file	ag_bmi270.h

@brief	Bosch BMI270 accel gyro implementation

This file implements only accel & gyro part of the BMI270. IMU features are
implemented in imu implementation file.


@author	Hoang Nguyen Hoan
@date	Dec. 26, 2024

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
#ifndef __AG_BMI270_H__
#define __AG_BMI270_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define BMI270_I2C_7BITS_DEVADDR0		0x68
#define BMI270_I2C_7BITS_DEVADDR1		0x69

#define BMI270_CHIP_ID_REG     				     	0x0

// Datasheet is 0x24, but reading from the chip is 0x21
#define BMI270_CHIP_ID          		                            0x21 // Datasheet shows 0x24

#define BMI270_ERR_REG								0x2
#define BMI270_ERR_FATAL											(1<<0)
#define BMI270_ERR_INTERNAL_MASK									(0x1E<<1)
#define BMI270_ERR_FIFO												(1<<6)
#define BMI270_ERR_AUX												(1<<7)

#define BMI270_STATUS_REG							0x3
#define BMI270_STATUS_AUX_BUSY										(1<<2)
#define BMI270_STATUS_CMD_RDY										(1<<4)
#define BMI270_STATUS_DRDY_AUX										(1<<5)
#define BMI270_STATUS_DRDY_GYR										(1<<6)	//!< Gyro data ready
#define BMI270_STATUS_DRDY_ACC										(1<<7)	//!< Accel data ready

#define BMI270_AUX_X_LSB_REG						0x4
#define BMI270_AUX_X_MSB_REG						0x5
#define BMI270_AUX_Y_LSB_REG						0x6
#define BMI270_AUX_Y_MSB_REG						0x7
#define BMI270_AUX_Z_LSB_REG						0x8
#define BMI270_AUX_Z_MSB_REG						0x9
#define BMI270_AUX_R_LSB_REG						0xA
#define BMI270_AUX_R_MSB_REG						0xB

#define BMI270_ACC_X_LSB_REG						0xC
#define BMI270_ACC_X_MSB_REG						0xD
#define BMI270_ACC_Y_LSB_REG						0xE
#define BMI270_ACC_Y_MSB_REG						0xF
#define BMI270_ACC_Z_LSB_REG						0x10
#define BMI270_ACC_Z_MSB_REG						0x11

#define BMI270_GYR_X_LSB_REG						0x12
#define BMI270_GYR_X_MSB_REG						0x13
#define BMI270_GYR_Y_LSB_REG						0x14
#define BMI270_GYR_Y_MSB_REG						0x15
#define BMI270_GYR_Z_LSB_REG						0x16
#define BMI270_GYR_Z_MSB_REG						0x17

#define BMI270_SENSOR_TIME_REG						0x18	//!< Sensor time register start.  24bits
#define BMI270_SENSOR_TIME0_REG						0x18	//!< Bit 7:0
#define BMI270_SENSOR_TIME1_REG						0x19	//!< Bit 15:8
#define BMI270_SENSOR_TIME2_REG						0x1A	//!< Bit 23:16

#define BMI270_EVENT_REG							0x1B
#define BMI270_EVENT_POR_DETECTED									(1<<0)	//!< 1 - After device power up or soft-reset, 0 - After status read
#define BMI270_EVENT_ERROR_CODE_MASK								(7<<2)	//!< Error codes for persistent errors
#define BMI270_EVENT_ERROR_CODE_NONE								(0<<2)	//!< No error
#define BMI270_EVENT_ERROR_CODE_ACC_ERR								(1<<2)	//!< Error in register ACC_CONF
#define BMI270_EVENT_ERROR_CODE_GYR_ERR								(2<<2)	//!< Error in register GYR_CONF
#define BMI270_EVENT_ERROR_CODE_ACC_GYR_ERR							(3<<2)	//!< Error in registers ACC_CONF & GYR_CONF

#define BMI270_INT_STATUS0_REG						0x1C
#define BMI270_INT_STATUS0_SIG_MOTION_OUT							(1<<0)	//!< Sigmotion ouput
#define BMI270_INT_STATUS0_STEP_COUNTER_OUT							(1<<1)	//!< Step counter watermark or step detector output
#define BMI270_INT_STATUS0_ACTIVITY_OUT								(1<<2)	//!< Step activity output
#define BMI270_INT_STATUS0_WRIST_WEAR_WAKEUP_OUT					(1<<3)	//!< Wrist wear wakeup output
#define BMI270_INT_STATUS0_WRIST_GESTURE_OUT						(1<<4)	//!< Wrist gesture output
#define BMI270_INT_STATUS0_NO_MOTION_OUT							(1<<5)	//!< No motion detection output
#define BMI270_INT_STATUS0_ANY_MOTION_OUT							(1<<6)	//!< Any motion detection output

#define BMI270_INT_STATUS1_REG						0x1D
#define BMI270_INT_STATUS1_FIFO_FULL								(1<<0)	//!< FIFO full interrupt
#define BMI270_INT_STATUS1_FIFO_WATERMARK							(1<<1)	//!< FIFO watermark interrupt
#define BMI270_INT_STATUS1_ERR										(1<<2)	//!< Error interrupt
#define BMI270_INT_STATUS1_AUX_DRDY									(1<<5)	//!< Auxiliary data ready interrupt
#define BMI270_INT_STATUS1_GYR_DRDY									(1<<6)	//!< Gyroscope data ready interrupt
#define BMI270_INT_STATUS1_ACC_DRDY									(1<<7)	//!< Accelerometer data ready interrupt

#define BMI270_STEP_COUNT_LSB_REG					0x1E
#define BMI270_STEP_COUNT_MSB_REG					0x1F

// Wrist gesture and activity detection output
#define BMI270_WR_GEST_ACT_REG						0x20
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_MASK							(7<<0)	//!< Output value of wrist gesture mask
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_UNKNOWN						(0<<0)	//!< Unknown gesture
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_PUSH_ARM_DOWM				(1<<0)	//!< Push arm down gesture
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_PIVOT_UP						(2<<0)	//!< Pivot up gesture
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_WRIST_SHAKE_JIGGLE			(3<<0)	//!< Wrist shake/jiggle gesture
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_FLICK_IN						(4<<0)	//!< Arm flick in gesture
#define BMI270_WR_GEST_ACT_WR_GEST_OUT_FLICK_OUT					(5<<0)	//!< Arm flick out gesture
#define BMI270_WR_GEST_ACT_ACT_OUT_MASK								(3<<3)	//!< Output value of the activity detection feature mask
#define BMI270_WR_GEST_ACT_ACT_OUT_STILL							(0<<3)	//!< User stationary
#define BMI270_WR_GEST_ACT_ACT_OUT_WALKING							(1<<3)	//!< User walking
#define BMI270_WR_GEST_ACT_ACT_OUT_RUNNING							(2<<3)	//!< User running
#define BMI270_WR_GEST_ACT_ACT_OUT_UNKNOWN       					(3<<3)	//!< Unknown state

#define BMI270_INTERNAL_STATUS_REG					0x21
#define BMI270_INTERNAL_STATUS_MESSAGE_MASK							(0xF<<0)
#define BMI270_INTERNAL_STATUS_MESSAGE_NO_INIT						(0<<0)	//!< ASIC is not initialized
#define BMI270_INTERNAL_STATUS_MESSAGE_INIT_OK						(1<<0)	//!< ASIC initialized
#define BMI270_INTERNAL_STATUS_MESSAGE_INIT_ERR						(2<<0)	//!< Initialization error
#define BMI270_INTERNAL_STATUS_MESSAGE_DRV_ERR						(3<<0)	//!< Invalid driver
#define BMI270_INTERNAL_STATUS_MESSAGE_SNS_STOP						(4<<0)	//!< Sensor stopped
#define BMI270_INTERNAL_STATUS_MESSAGE_NVM_ERR						(5<<0)	//!< Internal error while accessing NVM
#define BMI270_INTERNAL_STATUS_MESSAGE_STARTUP_ERR					(6<<0)	//!< Internal error while accessing NVM and initialization error
#define BMI270_INTERNAL_STATUS_MESSAGE_COMPAT_ERR					(7<<0)	//!< Compatibility error
#define BMI270_INTERNAL_STATUS_AXES_REMAP_ERR						(1<<5)	//!< Incorrect axes remapping
#define BMI270_INTERNAL_STATUS_ODR_50HZ_ERR							(1<<6)	//!< The minimum 50Hz bandwidth conditions are not respected

#define BMI270_TEMPERATURE_LSB_REG					0x22
#define BMI270_TEMPERATURE_MSB_REG					0x23

#define BMI270_FIFO_LENGTH_LSB_REG					0x24
#define BMI270_FIFO_LENGTH_MSB_REG					0x25
#define BMI270_FIFO_DATA_REG						0x26

#define BMI270_FEAT_PAGE_REG						0x2F
#define BMI270_FEAT_PAGE_MASK										(7<<0)

// Input registers for feature config. Output registers for feature results
#define BMI270_FEATURES_REG							0x30	//!< Features[16]

// Page 0
#define BMI270_FEATURES_SC_OUT_REG					0x30	//!< 4 bytes register for step counter
#define BMI270_FEATURES_SC_OUT0_REG					0x30	//!< Step count bit 7:0
#define BMI270_FEATURES_SC_OUT1_REG					0x31	//!< Step count bit 15:8
#define BMI270_FEATURES_SC_OUT2_REG					0x32	//!< Step count bit 23:16
#define BMI270_FEATURES_SC_OUT3_REG					0x33	//!< Step count bit 31:24

#define BMI270_FEATURES_ACT_OUT_REG					0x34	//!< Activity output
#define BMI270_FEATURES_ACT_OUT_MASK								(3<<0)
#define BMI270_FEATURES_ACT_OUT_STILL								(0<<0)	//!< User stationary
#define BMI270_FEATURES_ACT_OUT_WALKING								(1<<0)	//!< User walking
#define BMI270_FEATURES_ACT_OUT_RUNNING								(2<<0)	//!< User running
#define BMI270_FEATURES_ACT_OUT_UNKNOWN								(3<<0)	//!< Unknown state

#define BMI270_FEATURES_WR_GESTURE_OUT_REG			0x36
#define BMI270_FEATURES_WR_GESTURE_OUT_MASK							(7<<0)
#define BMI270_FEATURES_WR_GESTURE_OUT_UNKNOWN						(0<<0)	//!< Unknown gesture
#define BMI270_FEATURES_WR_GESTURE_OUT_PUSH_ARM_DOWN				(1<<0)	//!< Push arm down gesture
#define BMI270_FEATURES_WR_GESTURE_OUT_PIVOT_UP						(2<<0)	//!< Pivot up gesture
#define BMI270_FEATURES_WR_GESTURE_OUT_WRIST_SHAKE_JIGGLE			(3<<0)	//!< Wrist shake/jiggle gesture
#define BMI270_FEATURES_WR_GESTURE_OUT_FLICK_IN						(4<<0)	//!< Arm flick in gesture
#define BMI270_FEATURES_WR_GESTURE_OUT_FLICK_OUT					(5<<0)	//!< Arm flick out gesture

#define BMI270_FEATURES_GYR_GAIN_STATUS_REG			0x38
#define BMI270_FEATURES_GYR_GAIN_STATUS_SAT_X						(1<<0)	//!< X saturated
#define BMI270_FEATURES_GYR_GAIN_STATUS_SAT_Y						(1<<1)	//!< Y saturated
#define BMI270_FEATURES_GYR_GAIN_STATUS_SAT_Z						(1<<2)	//!< Z saturated
#define BMI270_FEATURES_GYR_GAIN_STATUS_G_TRIG_STATUS_MASK			(7<<3)	//!< Status of gyro mask
#define BMI270_FEATURES_GYR_GAIN_STATUS_G_TRIG_STATUS_NOERR			(0<<3)	//!< No error
#define BMI270_FEATURES_GYR_GAIN_STATUS_G_TRIG_STATUS_PRECON_ERR	(1<<3)	//!< Cmd aborted, precondition error
#define BMI270_FEATURES_GYR_GAIN_STATUS_G_TRIG_STATUS_DL_ERR		(2<<3)	//!< Cmd aborted, download error
#define BMI270_FEATURES_GYR_GAIN_STATUS_G_TRIG_STATUS_ABORT_ERR		(3<<3)	//!< Cmd aborted by host or due to motion detection

#define BMI270_FEATURES_GYR_CAS_REG					0x3C
#define BMI270_FEATURES_GYR_CAS_FACTOR_ZX_MASK						(0x7F)

// Page 1
#define BMI270_FEATURES_G_TRIG1_REG					0x32
#define BMI270_FEATURES_G_TRIG1_MAX_BURST_LEN_MASK					(0xF<<0)//!< Max burst length
#define BMI270_FEATURES_G_TRIG1_SELECT								(1<<8)	//!< Select
#define BMI270_FEATURES_G_TRIG1_BLOCK								(1<<9)	//!< Block feature with next G_TRIGGER cmd

#define BMI270_FEATURES_GEN_SET1_REG				0x34	// 16 bits
#define BMI270_FEATURES_GEN_SET1_MAP_X_AXIS_MASK					(3<<0)
#define BMI270_FEATURES_GEN_SET1_MAP_X_AXIS_X						(0<<0)	//!< Map to X
#define BMI270_FEATURES_GEN_SET1_MAP_X_AXIS_Y						(1<<0)	//!< Map to Y
#define BMI270_FEATURES_GEN_SET1_MAP_X_AXIS_Z						(2<<0)	//!< Map to Z
#define BMI270_FEATURES_GEN_SET1_MAP_X_AXIS_SIGN					(1<<2)	//!< Invert sign of X axis
#define BMI270_FEATURES_GEN_SET1_MAP_Y_AXIS_MASK					(3<<3)
#define BMI270_FEATURES_GEN_SET1_MAP_Y_AXIS_X						(0<<3)	//!< Map to X
#define BMI270_FEATURES_GEN_SET1_MAP_Y_AXIS_Y						(1<<3)	//!< Map to Y
#define BMI270_FEATURES_GEN_SET1_MAP_Y_AXIS_Z						(2<<3)	//!< Map to Z
#define BMI270_FEATURES_GEN_SET1_MAP_Y_AXIS_SIGN					(1<<5)	//!< Invert sign of Y axis
#define BMI270_FEATURES_GEN_SET1_MAP_Z_AXIS_MASK					(3<<6)
#define BMI270_FEATURES_GEN_SET1_MAP_Z_AXIS_X						(0<<6)	//!< Map to X
#define BMI270_FEATURES_GEN_SET1_MAP_Z_AXIS_Y						(1<<6)	//!< Map to Y
#define BMI270_FEATURES_GEN_SET1_MAP_Z_AXIS_Z						(2<<6)	//!< Map to Z
#define BMI270_FEATURES_GEN_SET1_MAP_Z_AXIS_SIGN					(1<<8)	//!< Invert sign of Z axis
#define BMI270_FEATURES_GEN_SET1_GYR_SELF_OFFSET					(1<<9)	//!< Enable self offset correction
#define BMI270_FEATURES_GEN_SET1_NVM_PROG_PREP						(1<<10)	//!< Prepare the system for NVM programming

#define BMI270_FEATURES_GYR_GAIN_UPD1_REG			0x36	// 16 bits
#define BMI270_FEATURES_GYR_GAIN_UPD1_RATIO_X_MASK					(0x7FF<<0)

#define BMI270_FEATURES_GYR_GAIN_UPD2_REG			0x38	// 16 bits
#define BMI270_FEATURES_GYR_GAIN_UPD2_RATIO_Y_MASK					(0x7FF<<0)

#define BMI270_FEATURES_GYR_GAIN_UPD3_REG			0x3A	// 16 bits
#define BMI270_FEATURES_GYR_GAIN_UPD3_RATIO_Z_MASK					(0x7FF<<0)
#define BMI270_FEATURES_GYR_GAIN_UPD3_ENABLE						(1<<11)	//!< Enable gyroscope gain update

#define BMI270_FEATURES_ANYMO1_REG					0x3C	// 16 bits
#define BMI270_FEATURES_ANYMO1_DURATION_MASK						(0x1FFF)
#define BMI270_FEATURES_ANYMO1_SELECT_X								(1<<13)	//!< Select the feature on a per axis basis
#define BMI270_FEATURES_ANYMO1_SELECT_Y								(1<<14)	//!< Select the feature on a per axis basis
#define BMI270_FEATURES_ANYMO1_SELECT_Z								(1<<15)	//!< Select the feature on a per axis basis

#define BMI270_FEATURES_ANYMO2_REG					0x3E	// 16 bits
#define BMI270_FEATURES_ANYMO2_THRESHOLD_MASK						(0x7FF<<0)	//!< Slope threshold value
#define BMI270_FEATURES_ANYMO2_OUT_CONF_MASK						(0xF<<11)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_ANYMO2_OUT_CONF_DISABLE						(0<<11)	//!< Disable interrupt assignment
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT0						(1<<11)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT1						(2<<11)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT2						(3<<11)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT3						(4<<11)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT4						(5<<11)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT5						(6<<11)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT6						(7<<11)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_ANYMO2_OUT_CONF_BIT7						(8<<11)	//!< Output assign to interrupt bit 7
#define BMI270_FEATURES_ANYMO2_ENABLE								(1<<15)	//!< Enable the feature

// Page 2
#define BMI270_FEATURES_MONO1_REG					0x30	// No motion detection config
#define BMI270_FEATURES_MONO1_DURATION_MASK							(0x1FFF<<0)
#define BMI270_FEATURES_MONO1_SELECT_X								(1<<13)	//!< Select the feature on a per axis basis
#define BMI270_FEATURES_MONO1_SELECT_Y								(1<<14)	//!< Select the feature on a per axis basis
#define BMI270_FEATURES_MONO1_SELECT_Z								(1<<1)	//!< Select the feature on a per axis basis

#define BMI270_FEATURES_MONO2_REG					0x32	// No motion detection config
#define BMI270_FEATURES_MONO2_THRESHOLD_MASK						(0x7FFF<<0)
#define BMI270_FEATURES_NOMO2_OUT_CONF_MASK							(0xF<<11)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_NOMO2_OUT_CONF_DISABLE						(0<<11)	//!< Disable interrupt assignment
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT0							(1<<11)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT1							(2<<11)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT2							(3<<11)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT3							(4<<11)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT4							(5<<11)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT5							(6<<11)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_NOMO2_OUT_CONF_BIT6							(7<<11)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_NOMO2_ENABLE								(1<<15)	//!< Enable the feature

#define BMI270_FEATURES_SIGMO1_REG					0x34	// 16 bits
#define BMI270_FEATURES_SIGMO1_BLOCK_SIZE_MASK						(0xFFFF<<0)

#define BMI270_FEATURES_SIGMO2_REG					0x3E	// 16 bits
#define BMI270_FEATURES_SIGMO2_ENABLE								(1<<0)	//!< Enable the feature
#define BMI270_FEATURES_SIGMO2_OUT_CONF_MASK						(0xF<<1)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_SIGMO2_OUT_CONF_DISABLE						(0<<1)	//!< Disable interrupt assignment
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT0						(1<<1)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT1						(2<<1)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT2						(3<<1)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT3						(4<<1)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT4						(5<<1)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT5						(6<<1)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT6						(7<<1)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_SIGMO2_OUT_CONF_BIT7						(8<<1)	//!< Output assign to interrupt bit 7

// Page 3
#define BMI270_FEATURES_SC1_REG						0x30	// 16 bits
#define BMI270_FEATURES_SC1_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC2_REG						0x32	// 16 bits
#define BMI270_FEATURES_SC2_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC3_REG						0x34	// 16 bits
#define BMI270_FEATURES_SC3_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC4_REG						0x36	// 16 bits
#define BMI270_FEATURES_SC4_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC5_REG						0x38	// 16 bits
#define BMI270_FEATURES_SC5_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC6_REG						0x3A	// 16 bits
#define BMI270_FEATURES_SC6_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC7_REG						0x3C	// 16 bits
#define BMI270_FEATURES_SC7_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC8_REG						0x3E	// 16 bits
#define BMI270_FEATURES_SC8_PARAM_MASK								(0xFFFF<<0)

// Page 4
#define BMI270_FEATURES_SC9_REG						0x30	// 16 bits
#define BMI270_FEATURES_SC9_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC10_REG					0x32	// 16 bits
#define BMI270_FEATURES_SC10_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC11_REG					0x34	// 16 bits
#define BMI270_FEATURES_SC11_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC12_REG					0x36	// 16 bits
#define BMI270_FEATURES_SC12_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC13_REG					0x38	// 16 bits
#define BMI270_FEATURES_SC13_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC14_REG					0x3A	// 16 bits
#define BMI270_FEATURES_SC14_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC15_REG					0x3C	// 16 bits
#define BMI270_FEATURES_SC15_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC16_REG					0x3E	// 16 bits
#define BMI270_FEATURES_SC16_PARAM_MASK								(0xFFFF<<0)

// Page 5
#define BMI270_FEATURES_SC17_REG					0x30	// 16 bits
#define BMI270_FEATURES_SC17_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC18_REG					0x32	// 16 bits
#define BMI270_FEATURES_SC18_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC19_REG					0x34	// 16 bits
#define BMI270_FEATURES_SC19_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC20_REG					0x36	// 16 bits
#define BMI270_FEATURES_SC20_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC21_REG					0x38	// 16 bits
#define BMI270_FEATURES_SC21_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC22_REG					0x3A	// 16 bits
#define BMI270_FEATURES_SC22_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC23_REG					0x3C	// 16 bits
#define BMI270_FEATURES_SC23_PARAM_MASK								(0xFFFF<<0)
#define BMI270_FEATURES_SC24_REG					0x3E	// 16 bits
#define BMI270_FEATURES_SC24_PARAM_MASK								(0xFFFF<<0)

// Page 6
#define BMI270_FEATURES_SC25_REG					0x30	// 16 bits
#define BMI270_FEATURES_SC25_PARAM_MASK								(0xFFFF<<0)

#define BMI270_FEATURES_SC26_REG					0x32	// 16 bits
#define BMI270_FEATURES_SC26_WATERMARK_LEVEL_MASK					(0x2FF<<0)
#define BMI270_FEATURES_SC26_RESET_COUNTER							(1<<10)
#define BMI270_FEATURES_SC26_EN_DETECTOR							(1<<11)
#define BMI270_FEATURES_SC26_EN_COUNTER								(1<<12)
#define BMI270_FEATURES_SC26_EN_ACTIVITY							(1<<13)

#define BMI270_FEATURES_SC27_REG					0x34	//
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_MASK			(0xF<<0)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT0			(1<<0)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT1			(2<<0)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT2			(3<<0)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT3			(4<<0)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT4			(5<<0)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT5			(6<<0)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT6			(7<<0)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_SC27_OUT_CONF_STEP_DETECTOR_BIT7			(8<<0)	//!< Output assign to interrupt bit 7
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_MASK		(0xF<<4)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT0		(1<<4)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT1		(2<<4)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT2		(3<<4)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT3		(4<<4)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT4		(5<<4)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT5		(6<<4)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT6		(7<<4)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_SC27_OUT_CONF_ACTIVITY_DETECTOR_BIT7		(8<<4)	//!< Output assign to interrupt bit 7

#define BMI270_FEATURES_WR_GEST1_REG				0x36
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_MASK						(0xF<<0)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT0						(1<<0)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT1						(2<<0)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT2						(3<<0)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT3						(4<<0)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT4						(5<<0)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT5						(6<<0)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT6						(7<<0)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_WR_GEST1_OUT_CONF_BIT7						(8<<0)	//!< Output assign to interrupt bit 7
#define BMI270_FEATURES_WR_GEST1_WEARABLE_ARM_LEFT					(0<<4)
#define BMI270_FEATURES_WR_GEST1_WEARABLE_ARM_RIGHT					(1<<4)
#define BMI270_FEATURES_WR_GEST1_ENABLE								(1<<5)

#define BMI270_FEATURES_WR_GEST2_REG				0x38
#define BMI270_FEATURES_WR_GEST2_MIN_FLICK_PEAK_MASK				(0xFFFF<<0)

#define BMI270_FEATURES_WR_GEST3_REG				0x3A
#define BMI270_FEATURES_WR_GEST3_MIN_FLICK_SAMPLES_MASK				(0xFFFF<<0)

#define BMI270_FEATURES_WR_GEST4_REG				0x3C
#define BMI270_FEATURES_WR_GEST4_MAX_DURATION_MASK					(0xFFFF<<0)

// Page 7
#define BMI270_FEATURES_WR_WAKEUP1_REG				0x30
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_MASK					(0xF<<0)	//!< Enable interrupt bit assignment
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT0					(1<<0)	//!< Output assign to interrupt bit 0
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT1					(2<<0)	//!< Output assign to interrupt bit 1
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT2					(3<<0)	//!< Output assign to interrupt bit 2
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT3					(4<<0)	//!< Output assign to interrupt bit 3
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT4					(5<<0)	//!< Output assign to interrupt bit 4
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT5					(6<<0)	//!< Output assign to interrupt bit 5
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT6					(7<<0)	//!< Output assign to interrupt bit 6
#define BMI270_FEATURES_WR_WAKEUP1_OUT_CONF_BIT7					(8<<0)	//!< Output assign to interrupt bit 7
#define BMI270_FEATURES_WR_WAKEUP1_ENABLE							(1<<4)

#define BMI270_FEATURES_WR_WAKEUP2_REG				0x32
#define BMI270_FEATURES_WR_WAKEUP2_MIN_ANGLE_FOCUS_MASK				(0xFFFF<<0)

#define BMI270_FEATURES_WR_WAKEUP3_REG				0x34
#define BMI270_FEATURES_WR_WAKEUP3_MIN_ANGLE_NONFOCUS_MASK			(0xFFFF<<0)

#define BMI270_FEATURES_WR_WAKEUP4_REG				0x36
#define BMI270_FEATURES_WR_WAKEUP4_MAX_TILT_LR_MASK					(0xFFFF<<0)

#define BMI270_FEATURES_WR_WAKEUP5_REG				0x38
#define BMI270_FEATURES_WR_WAKEUP5_MAX_TILT_LL_MASK					(0xFFFF<<0)

#define BMI270_FEATURES_WR_WAKEUP6_REG				0x3A
#define BMI270_FEATURES_WR_WAKEUP6_MAX_TILT_PD_MASK					(0xFFFF<<0)

#define BMI270_FEATURES_WR_WAKEUP7_REG				0x3C
#define BMI270_FEATURES_WR_WAKEUP7_MAX_TILT_PU_MASK					(0xFFFF<<0)

#define BMI270_ACC_CONF_REG							0x40
#define BMI270_ACC_CONF_ODR_MASK									(0xF<<0)
#define BMI270_ACC_CONF_ODR_0P78									(1<<0)	//!< 25/32 = 0.78125
#define BMI270_ACC_CONF_ODR_1P5										(2<<0)	//!< 25/16 = 1.5625
#define BMI270_ACC_CONF_ODR_3P1										(3<<0)	//!< 25/8 = 3.125
#define BMI270_ACC_CONF_ODR_6P25									(4<<0)	//!< 25/4 = 6.25
#define BMI270_ACC_CONF_ODR_12P5									(5<<0)	//!< 25/2 = 12.5
#define BMI270_ACC_CONF_ODR_25										(6<<0)	//!< 25
#define BMI270_ACC_CONF_ODR_50										(7<<0)	//!< 50
#define BMI270_ACC_CONF_ODR_100										(8<<0)	//!< 100
#define BMI270_ACC_CONF_ODR_200										(9<<0)	//!< 200
#define BMI270_ACC_CONF_ODR_400										(0xA<<0)//!< 400
#define BMI270_ACC_CONF_ODR_800										(0xB<<0)//!< 800
#define BMI270_ACC_CONF_ODR_1600									(0xC<<0)//!< 1600
#define BMI270_ACC_CONF_ODR_3200									(0xD<<0)//!< 3200
#define BMI270_ACC_CONF_ODR_6400									(0xE<<0)//!< 6400
#define BMI270_ACC_CONF_ODR_12800									(0xF<<0)//!< 12800
#define BMI270_ACC_CONF_ACC_BWP_MASK								(7<<4)	//!< Bandwidth parameter
#define BMI270_ACC_CONF_ACC_BWP_OSR4_AVG1							(0<<4)	//!< acc_filt_perf = 1 -> OSR4
#define BMI270_ACC_CONF_ACC_BWP_OSR2_AVG2							(1<<4)	//!< acc_filt_perf = 1 -> OSR2
#define BMI270_ACC_CONF_ACC_BWP_NORM_AVG4							(2<<4)	//!< acc_filt_perf = 1 -> Normal mode
#define BMI270_ACC_CONF_ACC_BWP_CIC_AVG8							(3<<4)	//!< acc_filt_perf = 1 -> CIC mode
#define BMI270_ACC_CONF_ACC_BWP_RES_AVG16							(4<<4)	//!< acc_filt_perf = 1 -> Reserved
#define BMI270_ACC_CONF_ACC_BWP_RES_AVG32							(5<<4)	//!< acc_filt_perf = 1 -> Reserved
#define BMI270_ACC_CONF_ACC_BWP_RES_AVG64							(6<<4)	//!< acc_filt_perf = 1 -> Reserved
#define BMI270_ACC_CONF_ACC_BWP_RES_AVG128							(7<<4)	//!< acc_filt_perf = 1 -> Reserved
#define BMI270_ACC_CONF_ACC_FILTER_PERF_ULP							(0<<7)	//!< Select acc filter performance mode power optimized
#define BMI270_ACC_CONF_ACC_FILTER_PERF_HP							(1<<7)	//!< Select acc filter performance mode performance optimized

#define BMI270_ACC_RANGE_REG						0x41
#define BMI270_ACC_RANGE_MASK										(3<<0)
#define BMI270_ACC_RANGE_2G											(0<<0)	//!< +-2g
#define BMI270_ACC_RANGE_4G											(1<<0)	//!< +-4g
#define BMI270_ACC_RANGE_8G											(2<<0)	//!< +-8g
#define BMI270_ACC_RANGE_16G										(3<<0)	//!< +-16g

#define BMI270_GYR_CONF_REG							0x42
#define BMI270_GYR_CONF_ODR_MASK									(0xF<<0)
#define BMI270_GYR_CONF_ODR_0P78									(1<<0)
#define BMI270_GYR_CONF_ODR_1P5										(2<<0)
#define BMI270_GYR_CONF_ODR_3P1										(3<<0)
#define BMI270_GYR_CONF_ODR_6P25									(4<<0)
#define BMI270_GYR_CONF_ODR_12P5									(5<<0)
#define BMI270_GYR_CONF_ODR_25										(6<<0)
#define BMI270_GYR_CONF_ODR_50										(7<<0)
#define BMI270_GYR_CONF_ODR_100										(8<<0)
#define BMI270_GYR_CONF_ODR_200										(9<<0)
#define BMI270_GYR_CONF_ODR_400										(0xA<<0)
#define BMI270_GYR_CONF_ODR_800										(0xB<<0)
#define BMI270_GYR_CONF_ODR_1600									(0xC<<0)
#define BMI270_GYR_CONF_ODR_3200									(0xD<<0)
#define BMI270_GYR_CONF_ODR_6400									(0xE<<0)
#define BMI270_GYR_CONF_ODR_12800									(0xF<<0)
#define BMI270_GYR_CONF_BWP_MASK									(3<<4)
#define BMI270_GYR_CONF_BWP_OSR4									(0<<4)
#define BMI270_GYR_CONF_BWP_OSR2									(1<<4)
#define BMI270_GYR_CONF_BWP_NORM									(2<<4)
#define BMI270_GYR_CONF_BWP_RES										(3<<4)
#define BMI270_GYR_CONF_NOISE_PERF_ULP								(0<<6)	//!< Power opt
#define BMI270_GYR_CONF_NOISE_PERF_HP								(1<<6)	//!< Performance opt
#define BMI270_GYR_CONF_FILTER_PERF_ULP								(0<<7)	//!< Power opt
#define BMI270_GYR_CONF_FILTER_PERF_HP								(1<<7)	//!< Performance opt

#define BMI270_GYR_RANGE_REG						0x43
#define BMI270_GYR_RANGE_GYR_RANGE_MASK								(7<<0)
#define BMI270_GYR_RANGE_GYR_RANGE_2000								(0<<0)	//!< +-2000 dps, 16.4 lsb/dps
#define BMI270_GYR_RANGE_GYR_RANGE_1000								(1<<0)	//!< +-1000 dps, 32.8 lsb/dps
#define BMI270_GYR_RANGE_GYR_RANGE_500								(2<<0)	//!< +-500 dps, 65.6 lsb/dps
#define BMI270_GYR_RANGE_GYR_RANGE_250								(3<<0)	//!< +-250 dps, 131.2 lsb/dps
#define BMI270_GYR_RANGE_GYR_RANGE_125								(4<<0)	//!< +-125 dps, 262.4 lsb/dps
#define BMI270_GYR_RANGE_OIS_RANGE_250								(0<<3)	//!< +-250 dps, 131.2 LSB/dps
#define BMI270_GYR_RANGE_OIS_RANGE_2000								(1<<3)	//!< +-2000 dps, 16.4 LSB/dps

#define BMI270_AUX_CONF_REG							0x44
#define BMI270_AUX_CONF_ODR_MASK									(0xF<<0)
#define BMI270_AUX_CONF_ODR_0P78									(1<<0)	//!< 25/32 = 0.78125
#define BMI270_AUX_CONF_ODR_1P5										(2<<0)	//!< 25/16 = 1.5625
#define BMI270_AUX_CONF_ODR_3P1										(3<<0)	//!< 25/8 = 3.125
#define BMI270_AUX_CONF_ODR_6P25									(4<<0)	//!< 25/4 = 6.25
#define BMI270_AUX_CONF_ODR_12P5									(5<<0)	//!< 25/2 = 12.5
#define BMI270_AUX_CONF_ODR_25										(6<<0)	//!< 25
#define BMI270_AUX_CONF_ODR_50										(7<<0)	//!< 50
#define BMI270_AUX_CONF_ODR_100										(8<<0)	//!< 100
#define BMI270_AUX_CONF_ODR_200										(9<<0)	//!< 200
#define BMI270_AUX_CONF_ODR_400										(0xA<<0)//!< 400
#define BMI270_AUX_CONF_ODR_800										(0xB<<0)//!< 800
#define BMI270_AUX_CONF_ODR_1600									(0xC<<0)//!< 1600
#define BMI270_AUX_CONF_ODR_3200									(0xD<<0)//!< 3200
#define BMI270_AUX_CONF_ODR_6400									(0xE<<0)//!< 6400
#define BMI270_AUX_CONF_ODR_12800									(0xF<<0)//!< 12800
#define BMI270_AUX_CONF_OFFSET_MASK									(0xF<<4)//!< Trigger readout offset in units of 2.5 ms

// Configure Gyroscope and accelerometer down sampling rates for FIFO
#define BMI270_FIFO_DOWNS_REG						0x45
#define BMI270_FIFO_DOWNS_GYR_FIFO_DOWNS_MASK						(7<<0)	//!< Down sampling for gyro
#define BMI270_FIFO_DOWNS_GYR_FIFO_FILT_DATA						(1<<3)	//!< Enable filtered data gyro
#define BMI270_FIFO_DOWNS_ACC_FIFO_DOWNS_MASK						(7<<4)	//!< Down sampling for acc
#define BMI270_FIFO_DOWNS_ACC_FIFO_FILT_DATA						(1<<7)	//!< Enable filtered data acc

// FIFO watermark level
#define BMI270_FIFO_WATERMARK_REG					0x46
#define BMI270_FIFO_WTM_LSB_REG						0x46
#define BMI270_FIFO_WTM_LSB_MASK									(0xFF<<0)
#define BMI270_FIFO_WTM_MSB_REG						0x47
#define BMI270_FIFO_WTM_MSB_MASK									(0x1F<<0)

#define BMI270_FIFO_CONFIG0_REG						0x48
#define BMI270_FIFO_CONFIG0_FIFO_STOP_ON_FULL						(1<<0)	//!< Stop writing sample into FIFO when full
#define BMI270_FIFO_CONFIG0_FIFO_TIME_EN							(1<<1)	//!< Return sensor time frame after the last valid data frame

#define BMI270_FIFO_CONFIG1_REG						0x49
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT1_EN_MASK					(3<<0)
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT1_EN_INT_EDGE				(0<<0)	//!< Enable tag on rising edge of int pin
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT1_EN_INT_LEVEL				(1<<0)	//!< Enable tag on rising edge of int pin
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT1_EN_ACC_SAT				(2<<0)	//!< Enable tag on saturation acc
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT1_EN_GYR_SAT				(3<<0)	//!< Enable tag on saturation gyro
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT2_EN_MASK					(3<<2)
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT2_EN_INT_EDGE				(0<<2)	//!< Enable tag on rising edge of int pin
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT2_EN_INT_LEVEL				(1<<2)	//!< Enable tag on rising edge of int pin
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT2_EN_ACC_SAT				(2<<2)	//!< Enable tag on saturation acc
#define BMI270_FIFO_CONFIG1_FIFO_TAG_INT2_EN_GYR_SAT				(3<<2)	//!< Enable tag on saturation gyro
#define BMI270_FIFO_CONFIG1_FIFO_HEADER_EN							(1<<4)	//!< Store header
#define BMI270_FIFO_CONFIG1_FIFO_AUX_EN								(1<<5)	//!< Auxiliary sensor data is stored
#define BMI270_FIFO_CONFIG1_FIFO_ACC_EN								(1<<6)	//!< Accel data is stored
#define BMI270_FIFO_CONFIG1_FIFO_GYR_EN								(1<<7)	//!< Gyro data is stored

#define BMI270_SATURATION_REG						0x4A
#define BMI270_SATURATION_ACC_X										(1<<0)	//!< Accel X axis raw data saturation flag
#define BMI270_SATURATION_ACC_Y										(1<<1)	//!< Accel Y axis raw data saturation flag
#define BMI270_SATURATION_ACC_Z										(1<<2)	//!< Accel Z axis raw data saturation flag
#define BMI270_SATURATION_GYR_X										(1<<3)	//!< Gyro X axis raw data saturation flag
#define BMI270_SATURATION_GYR_Y										(1<<4)	//!< Gyro Y axis raw data saturation flag
#define BMI270_SATURATION_GYR_Z										(1<<5)	//!< Gyro Z axis raw data saturation flag

#define BMI270_AUX_DEV_ID_REG						0x4B
#define BMI270_AUX_DEV_ID_MASK										(0x7F<<1)//!< I2C device address of auxiliary sensor

#define BMI270_AUX_IF_CONF_REG						0x4C
#define BMI270_AUX_IF_CONF_AUX_RD_BURST_MASK						(3<<0)	//!< Burst data length
#define BMI270_AUX_IF_CONF_AUX_RD_BURST_1							(1<<0)	//!< Data length 1
#define BMI270_AUX_IF_CONF_AUX_RD_BURST_2							(1<<0)	//!< Data length 2
#define BMI270_AUX_IF_CONF_AUX_RD_BURST_6							(1<<0)	//!< Data length 6
#define BMI270_AUX_IF_CONF_AUX_RD_BURST_8							(1<<0)	//!< Data length 8
#define BMI270_AUX_IF_CONF_MAN_RD_BURST_MASK						(3<<2)	//!< Manual burst data length
#define BMI270_AUX_IF_CONF_MAN_RD_BURST_1							(1<<2)	//!< Data length 1
#define BMI270_AUX_IF_CONF_MAN_RD_BURST_2							(1<<2)	//!< Data length 2
#define BMI270_AUX_IF_CONF_MAN_RD_BURST_6							(1<<2)	//!< Data length 6
#define BMI270_AUX_IF_CONF_MAN_RD_BURST_8							(1<<2)	//!< Data length 8
#define BMI270_AUX_IF_CONF_AUX_FCU_WRITE_EN							(1<<6)	//!< Enable FCU write command on AUX IF
#define BMI270_AUX_IF_CONF_AUX_MANUAL_EN							(1<<7)	//!< Switches auxiliary interface between automatic and manual

#define BMI270_AUX_RD_ADDR_REG						0x4D	// Auxiliary interface read address
#define BMI270_AUX_RD_ADDR_MASK										(0xFF<<0)

#define BMI270_AUX_WR_ADDR_REG						0x4E	// Auxiliary interface write address
#define BMI270_AUX_WR_ADDR_MASK										(0xFF<<0)

#define BMI270_AUX_WR_DATA_REG						0x4D	// Auxiliary interface write data
#define BMI270_AUX_WR_DATA_MASK										(0xFF<<0)

// Defines which error flag will trigger the error interrupt
#define BMI270_ERR_REG_MSK_REG						0x52
#define BMI270_ERR_REG_MSK_FATAL_ERR								(1<<0)
#define BMI270_ERR_REG_MSK_INTERNAL_ERR_MASK						(0xF<<1)
#define BMI270_ERR_REG_MSK_FIFO_ERR									(1<<6)
#define BMI270_ERR_REG_MSK_AUX_ERR									(1<<7)

#define BMI270_INT1_IO_CTRL_REG						0x53
#define BMI270_INT1_IO_CTRL_LVL_ACT_LOW								(0<<1)	//!< Configure output level of INT1 pin active low
#define BMI270_INT1_IO_CTRL_LVL_ACT_HIGH							(1<<1)	//!< Configure output level of INT1 pin active high
#define BMI270_INT1_IO_CTRL_OD_PUSH_PULL							(0<<2)	//!< Configure output behavior of INT1 pin push pull
#define BMI270_INT1_IO_CTRL_OD_OPEN_DRAIN							(1<<2)	//!< Configure output behavior of INT1 pin open drain
#define BMI270_INT1_IO_CTRL_OUTPUT_EN								(1<<3)	//!< Output enable for INT1 pin
#define BMI270_INT1_IO_CTRL_INPUT_EN								(1<<4)	//!< input enable for INT1 pin

#define BMI270_INT2_IO_CTRL_REG						0x54
#define BMI270_INT2_IO_CTRL_LVL_ACT_LOW								(0<<1)	//!< Configure output level of INT2 pin active low
#define BMI270_INT2_IO_CTRL_LVL_ACT_HIGH							(1<<1)	//!< Configure output level of INT2 pin active high
#define BMI270_INT2_IO_CTRL_OD_PUSH_PULL							(0<<2)	//!< Configure output behavior of INT2 pin push pull
#define BMI270_INT2_IO_CTRL_OD_OPEN_DRAIN							(1<<2)	//!< Configure output behavior of INT2 pin open drain
#define BMI270_INT2_IO_CTRL_OUTPUT_EN								(1<<3)	//!< Output enable for INT2 pin
#define BMI270_INT2_IO_CTRL_INPUT_EN								(1<<4)	//!< input enable for INT2 pin

#define BMI270_INT_LATCH_REG						0x55
#define BMI270_INT_LATCH_NON										(0<<0)	//!< Non latch
#define BMI270_INT_LATCH_PERM										(1<<0)	//!< Latched

#define BMI270_INT1_MAP_FEAT_REG					0x56
#define BMI270_INT1_MAP_FEAT_SIG_MOTION_OUT							(1<<0)	//!< Sigmotion output
#define BMI270_INT1_MAP_FEAT_STEP_COUNTER_OUT						(1<<1)	//!< Step counter watermark or step detector output
#define BMI270_INT1_MAP_FEAT_ACTIVITY_OUT							(1<<2)	//!< Step activity output
#define BMI270_INT1_MAP_FEAT_WRIST_WEAR_WAKEUP_OUT					(1<<3)	//!< Wrist wear wakeup output
#define BMI270_INT1_MAP_FEAT_WRIST_GESTURE_OUT						(1<<4)	//!< Wrist gesture output
#define BMI270_INT1_MAP_FEAT_NO_MOTION_OUT							(1<<5)	//!< No motion detection output
#define BMI270_INT1_MAP_FEAT_ANY_MOTION_OUT							(1<<6)	//!< Any motion detection output

#define BMI270_INT2_MAP_FEAT_REG					0x57
#define BMI270_INT2_MAP_FEAT_SIG_MOTION_OUT							(1<<0)	//!< Sigmotion output
#define BMI270_INT2_MAP_FEAT_STEP_COUNTER_OUT						(1<<1)	//!< Step counter watermark or step detector output
#define BMI270_INT2_MAP_FEAT_ACTIVITY_OUT							(1<<2)	//!< Step activity output
#define BMI270_INT2_MAP_FEAT_WRIST_WEAR_WAKEUP_OUT					(1<<3)	//!< Wrist wear wakeup output
#define BMI270_INT2_MAP_FEAT_WRIST_GESTURE_OUT						(1<<4)	//!< Wrist gesture output
#define BMI270_INT2_MAP_FEAT_NO_MOTION_OUT							(1<<5)	//!< No motion detection output
#define BMI270_INT2_MAP_FEAT_ANY_MOTION_OUT							(1<<6)	//!< Any motion detection output

#define BMI270_INT_MAP_DATA_REG						0x58
#define BMI270_INT_MAP_DATA_FFULL_INT1								(1<<0)	//!< FIFO full interrupt mapped to INT1
#define BMI270_INT_MAP_DATA_FWM_INT1								(1<<1)	//!< FIFO watermark interrupt mapped to INT1
#define BMI270_INT_MAP_DATA_DRDY_INT1								(1<<2)	//!< Data ready interrupt mapped to INT1
#define BMI270_INT_MAP_ERR_INT1										(1<<3)	//!< Error interrupt mapped to INT1
#define BMI270_INT_MAP_DATA_FFULL_INT2								(1<<4)	//!< FIFO full interrupt mapped to INT2
#define BMI270_INT_MAP_DATA_FWM_INT2								(1<<5)	//!< FIFO watermark interrupt mapped to INT2
#define BMI270_INT_MAP_DATA_DRDY_INT2								(1<<6)	//!< Data ready interrupt mapped to INT2
#define BMI270_INT_MAP_ERR_INT2										(1<<7)	//!< Error interrupt mapped to INT2

#define BMI270_INIT_CTRL_REG						0x59

#define BMI270_INIT_ADDR0_REG						0x5B
#define BMI270_INIT_ADDR0_MASK										(0xF<<0)	//!< Bit 0..3 of the base address for initialization data

#define BMI270_INIT_ADDR1_REG						0x5C
#define BMI270_INIT_ADDR1_MASK										(0xFF<<0)	//!< Bit 11..4 of the base address for initialization data

#define BMI270_INIT_DATA_REG						0x5E

#define BMI270_INTERNAL_ERROR_REG					0x5F
#define BMI270_INTERNAL_ERROR_INT_ERR_1								(1<<1)	//!< Long processing time
#define BMI270_INTERNAL_ERROR_INT_ERR_2								(1<<2)	//!< Fatal error
#define BMI270_INTERNAL_ERROR_FEAT_ENG_DISABLED						(1<<4)	//!< Feature engine has been disabled

#define BMI270_AUX_IF_TRIM_REG						0x68
#define BMI270_AUX_IF_TRIM_ASDA_PUPSEL_MASK							(3<<0)	//!< Pullup configuration for ASDA
#define BMI270_AUX_IF_TRIM_ASDA_PUPSEL_RES_OFF						(0<<0)	//!< Pullup off
#define BMI270_AUX_IF_TRIM_ASDA_PUPSEL_RES_40K						(0<<0)	//!< Pullup 40K
#define BMI270_AUX_IF_TRIM_ASDA_PUPSEL_RES_10K						(0<<0)	//!< Pullup 10K
#define BMI270_AUX_IF_TRIM_ASDA_PUPSEL_RES_2K						(0<<0)	//!< Pullup 2K

#define BMI270_GYR_CRT_CONF_REG						0x69
#define BMI270_GYR_CRT_CONF_CRT_RUNNING_EN							(1<<2)	//!< Indicate that CRT is currently running
#define BMI270_GYR_CRT_CONF_RDY_FOR_DL								(1<<3)	//!< Download complete

#define BMI270_NVM_CONF_REG							0x6A
#define BMI270_NVM_CONF_NVM_PROG_EN									(1<<1)	//!< Enable NVM programming

#define BMI270_IF_CONF_REG							0x6B
#define BMI270_IF_CONF_SPI3_EN										(1<<0)	//!< Enable primary SPI 3 wires mode
#define BMI270_IF_CONF_SPI3_OIS_EN									(1<<1)	//!< Enable OIS SPI interface 3 wires mode
#define BMI270_IF_CONF_OIS_EN										(1<<4)	//!< Interface configuration OIS enable
#define BMI270_IF_CONF_AUX_EN										(1<<5)	//!< Interface configuration AUX enable

#define BMI270_DRV_REG								0x6C
#define BMI270_DRV_IO_PAD_DRV1_MASK									(7<<0)
#define BMI270_DRV_IO_PAD_I2C_B1									(1<<3)	//!<
#define BMI270_DRV_IO_PAD_DRV2_MASK									(7<<4)
#define BMI270_DRV_IO_PAD_I2C_B2									(1<<7)	//!<

#define BMI270_ACC_SELF_TEST_REG					0x6D
#define BMI270_ACC_SELF_TEST_EN										(1<<0)	//!< Enable acc self test
#define BMI270_ACC_SELF_TEST_SIGN_POS								(1<<2)	//!< Acc self test sign positive
#define BMI270_ACC_SELF_TEST_AMP_LOW								(0<<3)	//!<
#define BMI270_ACC_SELF_TEST_AMP_HIGH								(1<<3)	//!<

#define BMI270_GYR_SELF_TEST_AXES_REG				0x6E
#define BMI270_GYR_SELF_TEST_AXES_ST_AXES_DONE						(1<<0)	//!< Self test completed
#define BMI270_GYR_SELF_TEST_AXES_X_OK								(1<<1)
#define BMI270_GYR_SELF_TEST_AXES_Y_OK								(1<<2)
#define BMI270_GYR_SELF_TEST_AXES_Z_OK								(1<<3)

#define BMI270_NV_CONF_REG							0x70
#define BMI270_NV_CONF_I2C_EN										(0<<0)	//!< Disable SPI, enable I2C
#define BMI270_NV_CONF_SPI_EN										(1<<0)	//!< Enable SPI, disable I2C
#define BMI270_NV_CONF_I2C_WDT_SEL_1P25MS							(0<<1)	//!< WDT 1.25 ms period for I2C
#define BMI270_NV_CONF_I2C_WDT_SEL_40MS								(1<<1)	//!< WDT 40 ms period for I2C
#define BMI270_NV_CONF_I2C_WDT_EN									(1<<2)	//!< Enable WDT for I2C
#define BMI270_NV_CONF_ACC_OFF_EN									(1<<3)	//!< Enable offset for acc

#define BMI270_ACC_OFFSET_X_REG							0x71
#define BMI270_ACC_OFFSET_Y_REG							0x72
#define BMI270_ACC_OFFSET_Z_REG							0x73

#define BMI270_GYR_OFFSET_X_LSB_REG						0x74
#define BMI270_GYR_OFFSET_Y_LSB_REG						0x75
#define BMI270_GYR_OFFSET_Z_LSB_REG						0x76

#define BMI270_OFFSET6_REG								0x77
#define BMI270_OFFSET6_GYR_OFF_X_MSB_MASK							(3<<0)
#define BMI270_OFFSET6_GYR_OFF_Y_MSB_MASK							(3<<2)
#define BMI270_OFFSET6_GYR_OFF_Z_MSB_MASK							(3<<4)
#define BMI270_OFFSET6_GYR_OFF_EN									(1<<6)
#define BMI270_OFFSET6_GYR_GAIN_EN									(1<<7)

#define BMI270_PWR_CONF_REG								0x7C
#define BMI270_PWR_CONF_ADV_POWER_SAVE_EN							(1<<0)
#define BMI270_PWR_CONF_FIFO_SELF_WAKE_UP_EN						(1<<1)
#define BMI270_PWR_CONF_FUP_EN										(1<<2)	//!< Fast power up enable

#define BMI270_PWR_CTRL_REG								0x7D
#define BMI270_PWR_CTRL_AUX_EN										(1<<0)	//!< Enable auxiliary sensor
#define BMI270_PWR_CTRL_GYR_EN										(1<<1)
#define BMI270_PWR_CTRL_ACC_EN										(1<<2)
#define BMI270_PWR_CTRL_TEMP_EN										(1<<3)

#define BMI270_CMD_REG									0x7E
#define BMI270_CMD_G_TRIGGER										(2<<0)	//!< Trigger special gyro operations
#define BMI270_CMD_USR_GAIN											(3<<0)	//!< Applies new gyro gain value
#define BMI270_CMD_NVM_PROG											(0xA0<<0)	//!< Write the NVM backed register into NVM
#define BMI270_CMD_FIFO_FLUSH										(0xB0<<0)	//!< Clears FIFO content
#define BMI270_CMD_SOFTRESET										(0xB6<<0)	//!< Trigger reset

#define BMI270_ADC_RANGE				0x7FFF		// 16 Bits
#define BMI270_ACC_DUMMY_X				0x7F01
#define BMI270_GYR_DUMMY_X				0x7F02
#define BMI270_TEMP_DUMMY				-32768	//0x8000

#define BMI270_FIFO_DATA_FLAG_ACC					(1<<0)	//!< Fifo contains Acc data
#define BMI270_FIFO_DATA_FLAG_GYR					(1<<1)	//!< Fifo contains Gyr data
#define BMI270_FIFO_DATA_FLAG_TEMP					(1<<2)	//!< Fifo contains temperature data
#define BMI270_FIFO_DATA_FLAG_TIME					(1<<3)	//!< Fifo contains timer data

#ifdef __cplusplus

class AccelBmi270 : public AccelSensor {
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

class GyroBmi270 : public GyroSensor {
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

class TempBmi270 : public TempSensor {
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

class AgBmi270 : public AccelBmi270, public GyroBmi270, public TempBmi270 {
public:
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[0] = AccelBmi270::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[0];
	}
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[1] = GyroBmi270::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[1];
	}

	virtual bool Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) {
		vbSensorEnabled[2] = TempBmi270::Init(Cfg, pIntrf, pTimer); return vbSensorEnabled[2];
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
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	bool vbInitialized;
	bool vbSensorEnabled[3];

private:
	virtual uint8_t FifoDataFlag() { return vFifoDataFlag; }
	virtual void FifoDataFlagSet(uint8_t Flag);//  { vFifoDataFlag = (vFifoDataFlag & ~Flag) | Flag; }
	virtual void FifoDataFlagClr(uint8_t Flag);//  { vFifoDataFlag = (vFifoDataFlag & ~Flag); }

	uint8_t vFifoDataFlag;	// Fifo frame is dependent on enabled features
	size_t vFifoFrameSize;	// Data word count
	uint16_t vPrevTime;
	uint64_t vRollover;
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __AG_BMI270_H__
