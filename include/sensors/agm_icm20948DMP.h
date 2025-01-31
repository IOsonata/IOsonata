/**-------------------------------------------------------------------------
@file	agm_icm20948DMP.h

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

This file contain DMP definitions. Raw low level sensor does not seem to work
properly without DMP. Therefore DMP is need for raw sensor driver
DMP is undocumented

@author	Hoang Nguyen Hoan
@date	Nov. 5, 2018

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#ifndef __AGM_ICM20948DMP_H__
#define __AGM_ICM20948DMP_H__


//#define CFG_FIFO_SIZE                   (4222)

// data output control
#define ICM20948_DMP_DATA_OUT_CTL1_REG					(4 * 16)
#define ICM20948_DMP_DATA_OUT_CTL1_ACCEL_SET					(0x8000)	//!< calibrated accel if accel calibrated, raw accel otherwise
#define ICM20948_DMP_DATA_OUT_CTL1_GYRO_SET						(0x4000)	//!< Raw gyro
#define ICM20948_DMP_DATA_OUT_CTL1_CPASS_SET					(0x2000)	//!< Raw magnetic
#define ICM20948_DMP_DATA_OUT_CTL1_ALS_SET						(0x1000)	//!< ALS/proximity
#define ICM20948_DMP_DATA_OUT_CTL1_QUAT6_SET					(0x0800)	//!< Game rotation vector
#define ICM20948_DMP_DATA_OUT_CTL1_QUAT9_SET					(0x0400)	//!< Rotation vector with heading accuracy
#define ICM20948_DMP_DATA_OUT_CTL1_PQUAT6_SET					(0x0200)	//!< Truncated game rotation vector for batching
#define ICM20948_DMP_DATA_OUT_CTL1_GEOMAG_SET					(0x0100)	//!< Geomag rotation vector with heading accuracy
#define ICM20948_DMP_DATA_OUT_CTL1_PRESSURE_SET					(0x0080)	//!< Pressure
#define ICM20948_DMP_DATA_OUT_CTL1_GYRO_CALIB_SET				(0x0040)	//!< Calibrated gyro
#define ICM20948_DMP_DATA_OUT_CTL1_CPASS_CLIB_SET				(0x0020)	//!< Calibrated mag
#define ICM20948_DMP_DATA_OUT_CTL1_PED_STEPDET_SET				(0x0010)	//!< timestamp when each step is detected
#define ICM20948_DMP_DATA_OUT_CTL1_HEADER2_SET					(0x0008)	//!< Enable/disable data output in data output control register 2
#define ICM20948_DMP_DATA_OUT_CTL1_PED_STEPIND_SET				(0x0007)	//!< number of steps detected will be attached to the 3 least significant bits of header

#define ICM20948_DMP_DATA_OUT_CTL2_REG					(4 * 16 + 2)
#define ICM20948_DMP_DATA_OUT_CTL2_ACCEL_ACCURACY_SET			(0x4000)	//!< accel accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
#define ICM20948_DMP_DATA_OUT_CTL2_GYRO_ACCURACY_SET			(0x2000)	//!< gyro accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
#define ICM20948_DMP_DATA_OUT_CTL2_CPASS_ACCURACY_SET			(0x1000)	//!< compass accuracy when changes, HEADER2_SET also needs to be set in data output control regsiter 1
#define ICM20948_DMP_DATA_OUT_CTL2_BATCH_MODE_EN				(0x0100)	//!< enable batching

#define ICM20948_DMP_DATA_INTR_CTL_REG					(4 * 16 + 12)
#define ICM20948_DMP_DATA_INTR_CTL_ACCEL_SET					(0x8000)	//!< calibrated accel if accel calibrated, raw accel otherwise
#define ICM20948_DMP_DATA_INTR_CTL_GYRO_SET						(0x4000)	//!< raw gyro
#define ICM20948_DMP_DATA_INTR_CTL_CPASS_SET					(0x2000)	//!< raw magnetic
#define ICM20948_DMP_DATA_INTR_CTL_ALS_SET						(0x1000)	//!< ALS/proximity
#define ICM20948_DMP_DATA_INTR_CTL_QUAT6_SET					(0x0800)	//!< game rotation vector
#define ICM20948_DMP_DATA_INTR_CTL_QUAT9_SET					(0x0400)	//!< rotation vector with heading accuracy
#define ICM20948_DMP_DATA_INTR_CTL_PQUAT6_SET					(0x0200)	//!< truncated game rotation vector for batching
#define ICM20948_DMP_DATA_INTR_CTL_GEOMAG_SET					(0x0100)	//!< geomag rotation vector with heading accuracy
#define ICM20948_DMP_DATA_INTR_CTL_PRESSURE_SET					(0x0080)	//!< pressure
#define ICM20948_DMP_DATA_INTR_CTL_GYRO_CALIBR_SET				(0x0040)	//!< calibrated gyro
#define ICM20948_DMP_DATA_INTR_CTL_CPASS_CALIBR_SET				(0x0020)	//!< calibrated magnetic
#define ICM20948_DMP_DATA_INTR_CTL_PED_STEPDET_SET				(0x0010)	//!< timestamp when each step is detected
#define ICM20948_DMP_DATA_INTR_CTL_HEADER2_SET					(0x0008)	//!< data output defined in data output control register 2
#define ICM20948_DMP_DATA_INTR_CTL_PED_STEPIND_SET				(0x0007)	//!< number of steps detected will be attached to the 3 least significant bits of header

#define ICM20948_DMP_FIFO_WATERMARK_REG					(31 * 16 + 14)

// motion event control
#define ICM20948_DMP_MOTION_EVENT_CTL_REG				(4 * 16 + 14)
#define ICM20948_DMP_MOTION_EVENT_CTL_BAC_WEAR_EN        		(0x8000)	//!< change BAC behavior for wearable platform
#define ICM20948_DMP_MOTION_EVENT_CTL_PEDOMETER_EN				(0x4000)	//!< pedometer engine
#define ICM20948_DMP_MOTION_EVENT_CTL_PEDOMETER_INT_EN			(0x2000)	//!< pedometer step detector interrupt
#define ICM20948_DMP_MOTION_EVENT_CTL_SMD_EN					(0x0800)	//!< significant motion detection interrupt
#define ICM20948_DMP_MOTION_EVENT_CTL_ACCEL_CAL_EN				(0x0200)	//!< accel calibration
#define ICM20948_DMP_MOTION_EVENT_CTL_GYRO_CAL_EN				(0x0100)	//!< gyro calibration
#define ICM20948_DMP_MOTION_EVENT_CTL_COMPASS_CAL_EN			(0x0080)	//!< compass calibration
#define ICM20948_DMP_MOTION_EVENT_CTL_NINE_AXIS_EN				(0x0040)	//!< 9-axis algorithm execution
#define ICM20948_DMP_MOTION_EVENT_CTL_GEOMAG_EN					(0x0008)	//!< Geomag algorithm execution
#define ICM20948_DMP_MOTION_EVENT_CTL_BTS_LTS_EN          		(0x0004)	//!< bring & look to see
#define ICM20948_DMP_MOTION_EVENT_CTL_BAC_ACCEL_ONLY_EN   		(0x0002)	//!< run BAC as accel only

// indicates to DMP which sensors are available
/*	1: gyro samples available
2: accel samples available
8: secondary samples available	*/
#define ICM20948_DMP_DATA_RDY_STATUS_REG				(8 * 16 + 10)

// batch mode
#define ICM20948_DMP_BM_BATCH_CNTR						(27 * 16)
#define ICM20948_DMP_BM_BATCH_THLD						(19 * 16 + 12)
#define ICM20948_DMP_BM_BATCH_MASK						(21 * 16 + 14)

// sensor output data rate divider
// desired ODR = base engine rate/(divider + 1)
#define ICM20948_DMP_ODR_ACCEL							(11 * 16 + 14)
#define ICM20948_DMP_ODR_GYRO							(11 * 16 + 10)
#define ICM20948_DMP_ODR_CPASS							(11 * 16 +  6)
#define ICM20948_DMP_ODR_ALS							(11 * 16 +  2)
#define ICM20948_DMP_ODR_QUAT6							(10 * 16 + 12)
#define ICM20948_DMP_ODR_QUAT9							(10 * 16 +  8)
#define ICM20948_DMP_ODR_PQUAT6							(10 * 16 +  4)
#define ICM20948_DMP_ODR_GEOMAG							(10 * 16 +  0)
#define ICM20948_DMP_ODR_PRESSURE						(11 * 16 + 12)
#define ICM20948_DMP_ODR_GYRO_CALIBR					(11 * 16 +  8)
#define ICM20948_DMP_ODR_CPASS_CALIBR					(11 * 16 +  4)

// sensor output data rate counter
#define ICM20948_DMP_ODR_CNTR_ACCEL						(9 * 16 + 14)
#define ICM20948_DMP_ODR_CNTR_GYRO						(9 * 16 + 10)
#define ICM20948_DMP_ODR_CNTR_CPASS						(9 * 16 +  6)
#define ICM20948_DMP_ODR_CNTR_ALS						(9 * 16 +  2)
#define ICM20948_DMP_ODR_CNTR_QUAT6						(8 * 16 + 12)
#define ICM20948_DMP_ODR_CNTR_QUAT9						(8 * 16 +  8)
#define ICM20948_DMP_ODR_CNTR_PQUAT6					(8 * 16 +  4)
#define ICM20948_DMP_ODR_CNTR_GEOMAG					(8 * 16 +  0)
#define ICM20948_DMP_ODR_CNTR_PRESSURE					(9 * 16 + 12)
#define ICM20948_DMP_ODR_CNTR_GYRO_CALIBR				(9 * 16 +  8)
#define ICM20948_DMP_ODR_CNTR_CPASS_CALIBR				(9 * 16 +  4)

// mounting matrix
#define ICM20948_DMP_CPASS_MTX_00						(23 * 16)
#define ICM20948_DMP_CPASS_MTX_01						(23 * 16 + 4)
#define ICM20948_DMP_CPASS_MTX_02						(23 * 16 + 8)
#define ICM20948_DMP_CPASS_MTX_10						(23 * 16 + 12)
#define ICM20948_DMP_CPASS_MTX_11						(24 * 16)
#define ICM20948_DMP_CPASS_MTX_12						(24 * 16 + 4)
#define ICM20948_DMP_CPASS_MTX_20						(24 * 16 + 8)
#define ICM20948_DMP_CPASS_MTX_21						(24 * 16 + 12)
#define ICM20948_DMP_CPASS_MTX_22						(25 * 16)

#define ICM20948_DMP_GYRO_SF							(19 * 16)
#define ICM20948_DMP_ACCEL_FB_GAIN						(34 * 16)
#define ICM20948_DMP_ACCEL_ONLY_GAIN					(16 * 16 + 12)

// bias calibration
#define ICM20948_DMP_GYRO_BIAS_X						(139 * 16 +  4)
#define ICM20948_DMP_GYRO_BIAS_Y						(139 * 16 +  8)
#define ICM20948_DMP_GYRO_BIAS_Z						(139 * 16 + 12)
#define ICM20948_DMP_GYRO_ACCURACY						(138 * 16 +  2)
#define ICM20948_DMP_GYRO_BIAS_SET						(138 * 16 +  6)
#define ICM20948_DMP_GYRO_LAST_TEMPR					(134 * 16)
#define ICM20948_DMP_GYRO_SLOPE_X						( 78 * 16 +  4)
#define ICM20948_DMP_GYRO_SLOPE_Y						( 78 * 16 +  8)
#define ICM20948_DMP_GYRO_SLOPE_Z						( 78 * 16 + 12)

#define ICM20948_DMP_ACCEL_BIAS_X						(110 * 16 +  4)
#define ICM20948_DMP_ACCEL_BIAS_Y						(110 * 16 +  8)
#define ICM20948_DMP_ACCEL_BIAS_Z						(110 * 16 + 12)
#define ICM20948_DMP_ACCEL_ACCURACY						(97 * 16)
#define ICM20948_DMP_ACCEL_CAL_RESET					(77 * 16)
#define ICM20948_DMP_ACCEL_VARIANCE_THRESH				(93 * 16)
#define ICM20948_DMP_ACCEL_CAL_RATE						(94 * 16 + 4)
#define ICM20948_DMP_ACCEL_PRE_SENSOR_DATA				(97 * 16 + 4)
#define ICM20948_DMP_ACCEL_COVARIANCE					(101 * 16 + 8)
#define ICM20948_DMP_ACCEL_ALPHA_VAR					(91 * 16)
#define ICM20948_DMP_ACCEL_A_VAR						(92 * 16)
#define ICM20948_DMP_ACCEL_CAL_INIT						(94 * 16 + 2)
#define ICM20948_DMP_ACCEL_CAL_SCALE_COVQ_IN_RANGE		(194 * 16)
#define ICM20948_DMP_ACCEL_CAL_SCALE_COVQ_OUT_RANGE		(195 * 16)
#define ICM20948_DMP_ACCEL_CAL_TEMPERATURE_SENSITIVITY	(194 * 16 + 4)
#define ICM20948_DMP_ACCEL_CAL_TEMPERATURE_OFFSET_TRIM	(194 * 16 + 12)

#define ICM20948_DMP_CPASS_BIAS_X						(126 * 16 +  4)
#define ICM20948_DMP_CPASS_BIAS_Y						(126 * 16 +  8)
#define ICM20948_DMP_CPASS_BIAS_Z						(126 * 16 + 12)
#define ICM20948_DMP_CPASS_ACCURACY						(37 * 16)
#define ICM20948_DMP_CPASS_BIAS_SET						(34 * 16 + 14)
#define ICM20948_DMP_MAR_MODE							(37 * 16 + 2)
#define ICM20948_DMP_CPASS_COVARIANCE					(115 * 16)
#define ICM20948_DMP_CPASS_COVARIANCE_CUR				(118 * 16 +  8)
#define ICM20948_DMP_CPASS_REF_MAG_3D					(122 * 16)
#define ICM20948_DMP_CPASS_CAL_INIT						(114 * 16)
#define ICM20948_DMP_CPASS_EST_FIRST_BIAS				(113 * 16)
#define ICM20948_DMP_MAG_DISTURB_STATE					(113 * 16 + 2)
#define ICM20948_DMP_CPASS_VAR_COUNT					(112 * 16 + 6)
#define ICM20948_DMP_CPASS_COUNT_7						( 87 * 16 + 2)
#define ICM20948_DMP_CPASS_MAX_INNO						(124 * 16)
#define ICM20948_DMP_CPASS_BIAS_OFFSET					(113 * 16 + 4)
#define ICM20948_DMP_CPASS_CUR_BIAS_OFFSET				(114 * 16 + 4)
#define ICM20948_DMP_CPASS_PRE_SENSOR_DATA				( 87 * 16 + 4)

// Compass Cal params to be adjusted according to sampling rate
#define ICM20948_DMP_CPASS_TIME_BUFFER					(112 * 16 + 14)
#define ICM20948_DMP_CPASS_RADIUS_3D_THRESH_ANOMALY		(112 * 16 + 8)

#define ICM20948_DMP_CPASS_STATUS_CHK					(25 * 16 + 12)

// 9-axis
#define ICM20948_DMP_MAGN_THR_9X						(80 * 16)
#define ICM20948_DMP_MAGN_LPF_THR_9X					(80 * 16 +  8)
#define ICM20948_DMP_QFB_THR_9X							(80 * 16 + 12)

// DMP running counter
#define ICM20948_DMP_DMPRATE_CNTR						(18 * 16 + 4)

// pedometer
#define ICM20948_DMP_PEDSTD_BP_B						(49 * 16 + 12)
#define ICM20948_DMP_PEDSTD_BP_A4						(52 * 16)
#define ICM20948_DMP_PEDSTD_BP_A3						(52 * 16 +  4)
#define ICM20948_DMP_PEDSTD_BP_A2						(52 * 16 +  8)
#define ICM20948_DMP_PEDSTD_BP_A1						(52 * 16 + 12)
#define ICM20948_DMP_PEDSTD_SB							(50 * 16 +  8)
#define ICM20948_DMP_PEDSTD_SB_TIME						(50 * 16 + 12)
#define ICM20948_DMP_PEDSTD_PEAKTHRSH					(57 * 16 +  8)
#define ICM20948_DMP_PEDSTD_TIML						(50 * 16 + 10)
#define ICM20948_DMP_PEDSTD_TIMH						(50 * 16 + 14)
#define ICM20948_DMP_PEDSTD_PEAK						(57 * 16 +  4)
#define ICM20948_DMP_PEDSTD_STEPCTR						(54 * 16)
#define ICM20948_DMP_PEDSTD_STEPCTR2					(58 * 16 +  8)
#define ICM20948_DMP_PEDSTD_TIMECTR						(60 * 16 +  4)
#define ICM20948_DMP_PEDSTD_DECI						(58 * 16)
#define ICM20948_DMP_PEDSTD_SB2							(60 * 16 + 14)
#define ICM20948_DMP_STPDET_TIMESTAMP					(18 * 16 +  8)
#define ICM20948_DMP_PEDSTEP_IND						(19 * 16 +  4)
#define ICM20948_DMP_PED_Y_RATIO						(17 * 16 +  0)

// SMD
#define ICM20948_DMP_SMD_VAR_TH							(141 * 16 + 12)
#define ICM20948_DMP_SMD_VAR_TH_DRIVE					(143 * 16 + 12)
#define ICM20948_DMP_SMD_DRIVE_TIMER_TH					(143 * 16 +  8)
#define ICM20948_DMP_SMD_TILT_ANGLE_TH					(179 * 16 + 12)
#define ICM20948_DMP_BAC_SMD_ST_TH						(179 * 16 +  8)
#define ICM20948_DMP_BAC_ST_ALPHA4						(180 * 16 + 12)
#define ICM20948_DMP_BAC_ST_ALPHA4A						(176 * 16 + 12)

// Wake on Motion
#define ICM20948_DMP_WOM_ENABLE							(64 * 16 + 14)
#define ICM20948_DMP_WOM_STATUS							(64 * 16 + 6)
#define ICM20948_DMP_WOM_THRESHOLD						(64 * 16)
#define ICM20948_DMP_WOM_CNTR_TH						(64 * 16 + 12)

// Activity Recognition
#define ICM20948_DMP_BAC_RATE							(48  * 16 + 10)
#define ICM20948_DMP_BAC_STATE							(179 * 16 +  0)
#define ICM20948_DMP_BAC_STATE_PREV						(179 * 16 +  4)
#define ICM20948_DMP_BAC_ACT_ON							(182 * 16 +  0)
#define ICM20948_DMP_BAC_ACT_OFF						(183 * 16 +  0)
#define ICM20948_DMP_BAC_STILL_S_F						(177 * 16 +  0)
#define ICM20948_DMP_BAC_RUN_S_F						(177 * 16 +  4)
#define ICM20948_DMP_BAC_DRIVE_S_F						(178 * 16 +  0)
#define ICM20948_DMP_BAC_WALK_S_F						(178 * 16 +  4)
#define ICM20948_DMP_BAC_SMD_S_F						(178 * 16 +  8)
#define ICM20948_DMP_BAC_BIKE_S_F						(178 * 16 + 12)
#define ICM20948_DMP_BAC_E1_SHORT						(146 * 16 +  0)
#define ICM20948_DMP_BAC_E2_SHORT						(146 * 16 +  4)
#define ICM20948_DMP_BAC_E3_SHORT						(146 * 16 +  8)
#define ICM20948_DMP_BAC_VAR_RUN						(148 * 16 + 12)
#define ICM20948_DMP_BAC_TILT_INIT						(181 * 16 +  0)
#define ICM20948_DMP_BAC_MAG_ON							(225 * 16 +  0)
#define ICM20948_DMP_BAC_PS_ON							(74  * 16 +  0)
#define ICM20948_DMP_BAC_BIKE_PREFERENCE				(173 * 16 +  8)
#define ICM20948_DMP_BAC_MAG_I2C_ADDR					(229 * 16 +  8)
#define ICM20948_DMP_BAC_PS_I2C_ADDR					(75  * 16 +  4)
#define ICM20948_DMP_BAC_DRIVE_CONFIDENCE				(144 * 16 +  0)
#define ICM20948_DMP_BAC_WALK_CONFIDENCE				(144 * 16 +  4)
#define ICM20948_DMP_BAC_SMD_CONFIDENCE					(144 * 16 +  8)
#define ICM20948_DMP_BAC_BIKE_CONFIDENCE				(144 * 16 + 12)
#define ICM20948_DMP_BAC_STILL_CONFIDENCE				(145 * 16 +  0)
#define ICM20948_DMP_BAC_RUN_CONFIDENCE					(145 * 16 +  4)
#define ICM20948_DMP_BAC_MODE_CNTR						(150 * 16)
#define ICM20948_DMP_BAC_STATE_T_PREV					(185 * 16 +  4)
#define ICM20948_DMP_BAC_ACT_T_ON						(184 * 16 +  0)
#define ICM20948_DMP_BAC_ACT_T_OFF						(184 * 16 +  4)
#define ICM20948_DMP_BAC_STATE_WRDBS_PREV				(185 * 16 +  8)
#define ICM20948_DMP_BAC_ACT_WRDBS_ON					(184 * 16 +  8)
#define ICM20948_DMP_BAC_ACT_WRDBS_OFF					(184 * 16 + 12)
#define ICM20948_DMP_BAC_ACT_ON_OFF						(190 * 16 +  2)
#define ICM20948_DMP_PREV_BAC_ACT_ON_OFF				(188 * 16 +  2)
#define ICM20948_DMP_BAC_CNTR							(48  * 16 +  2)

// Flip/Pick-up
#define ICM20948_DMP_FP_VAR_ALPHA						(245 * 16 +  8)
#define ICM20948_DMP_FP_STILL_TH						(246 * 16 +  4)
#define ICM20948_DMP_FP_MID_STILL_TH					(244 * 16 +  8)
#define ICM20948_DMP_FP_NOT_STILL_TH					(246 * 16 +  8)
#define ICM20948_DMP_FP_VIB_REJ_TH						(241 * 16 +  8)
#define ICM20948_DMP_FP_MAX_PICKUP_T_TH					(244 * 16 + 12)
#define ICM20948_DMP_FP_PICKUP_TIMEOUT_TH				(248 * 16 +  8)
#define ICM20948_DMP_FP_STILL_CONST_TH					(246 * 16 + 12)
#define ICM20948_DMP_FP_MOTION_CONST_TH					(240 * 16 +  8)
#define ICM20948_DMP_FP_VIB_COUNT_TH					(242 * 16 +  8)
#define ICM20948_DMP_FP_STEADY_TILT_TH					(247 * 16 +  8)
#define ICM20948_DMP_FP_STEADY_TILT_UP_TH				(242 * 16 + 12)
#define ICM20948_DMP_FP_Z_FLAT_TH_MINUS					(243 * 16 +  8)
#define ICM20948_DMP_FP_Z_FLAT_TH_PLUS					(243 * 16 + 12)
#define ICM20948_DMP_FP_DEV_IN_POCKET_TH				(76  * 16 + 12)
#define ICM20948_DMP_FP_PICKUP_CNTR						(247 * 16 +  4)
#define ICM20948_DMP_FP_RATE							(240 * 16 + 12)

// Gyro FSR
#define ICM20948_DMP_GYRO_FULLSCALE						(72 * 16 + 12)

// Accel FSR
#define ICM20948_DMP_ACC_SCALE							(30 * 16 + 0)
#define ICM20948_DMP_ACC_SCALE2							(79 * 16 + 4)

// EIS authentication
#define ICM20948_DMP_EIS_AUTH_INPUT						(160 * 16 +   4)
#define ICM20948_DMP_EIS_AUTH_OUTPUT					(160 * 16 +   0)

// B2S
#define ICM20948_DMP_B2S_RATE							(48  * 16 +   8)
// mounting matrix
#define ICM20948_DMP_B2S_MTX_00							(208 * 16)
#define ICM20948_DMP_B2S_MTX_01							(208 * 16 + 4)
#define ICM20948_DMP_B2S_MTX_02							(208 * 16 + 8)
#define ICM20948_DMP_B2S_MTX_10							(208 * 16 + 12)
#define ICM20948_DMP_B2S_MTX_11							(209 * 16)
#define ICM20948_DMP_B2S_MTX_12							(209 * 16 + 4)
#define ICM20948_DMP_B2S_MTX_20							(209 * 16 + 8)
#define ICM20948_DMP_B2S_MTX_21							(209 * 16 + 12)
#define ICM20948_DMP_B2S_MTX_22							(210 * 16)

// Dmp3 orientation parameters (Q30) initialization
#define ICM20948_DMP_Q0_QUAT6							(33 * 16 + 0)
#define ICM20948_DMP_Q1_QUAT6							(33 * 16 + 4)
#define ICM20948_DMP_Q2_QUAT6							(33 * 16 + 8)
#define ICM20948_DMP_Q3_QUAT6							(33 * 16 + 12)

#define ICM20948_DMP_PROG_START_ADDR   					(0x1000U)
#define ICM20948_DMP_MEM_BANK_SIZE   					256
#define ICM20948_DMP_LOAD_MEM_START_ADDR				0x90

#define ICM20948_DMP_CODE_SIZE 							14301

#define ICM20948_DMP_ACCEL_SET							0x0080 //!< 0x8000 - calibrated accel if accel calibrated, raw accel otherwise
#define ICM20948_DMP_GYRO_SET							0x0040 //!< 0x4000 - raw gyro
#define ICM20948_DMP_CPASS_SET							0x0020 //!< 0x2000 - raw magnetic
#define ICM20948_DMP_ALS_SET							0x0010 //!< 0x1000 - ALS/proximity
#define ICM20948_DMP_QUAT6_SET							0x0008 //!< 0x0800 - game rotation vector
#define ICM20948_DMP_QUAT9_SET							0x0004 //!< 0x0400 - rotation vector with heading accuracy
#define ICM20948_DMP_PQUAT6_SET							0x0002 //!< 0x0200 - truncated game rotation vector for batching
#define ICM20948_DMP_GEOMAG_SET							0x0001 //!< 0x0100 - geomag rotation vector with heading accuracy
#define ICM20948_DMP_PRESSURE_SET						0x8000 //!< 0x0080 - pressure
#define ICM20948_DMP_GYRO_CALIBR_SET					0x4000 //!< 0x0040 - calibrated gyro
#define ICM20948_DMP_CPASS_CALIBR_SET					0x2000 //!< 0x0020 - calibrated magnetic
#define ICM20948_DMP_PED_STEPDET_SET					0x1000 //!< 0x0010 - timestamp when each step is detected
#define ICM20948_DMP_HEADER2_SET						0x0800 //!< 0x0008 - enable/disable data output in data output control register 2
#define ICM20948_DMP_PED_STEPIND_SET					0x0700 //!< 0x0007 - number of steps detected will be attached to the 3 least significant bits of header


#endif // __AGM_ICM20948DMP_H__

