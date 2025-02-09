/**-------------------------------------------------------------------------
@file	agm_icm20948.h

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

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

#ifndef __AGM_ICM20948_H__
#define __AGM_ICM20948_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_ak09916.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define ICM20948_I2C_DEV_ADDR0			0x68		// AD0 low
#define ICM20948_I2C_DEV_ADDR1			0x69		// AD0 high

#define ICM20948_REG_BANK_SEL_REG			127

#define ICM20948_REG_BANK_SEL_USER_BANK_BITPOS		(4)
#define ICM20948_REG_BANK_SEL_USER_BANK_MASK		(3<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_0			(0<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_1			(1<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_2			(2<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_3			(3<<4)

// concatenate bank select to second byte of register address making
// register address a 16 bits value
#define ICM20948_REG_BANK0				(0<<7)
#define ICM20948_REG_BANK1				(1<<7)
#define ICM20948_REG_BANK2				(2<<7)
#define ICM20948_REG_BANK3				(3<<7)

//*** Register Bank 0

#define ICM20948_WHO_AM_I_REG				(ICM20948_REG_BANK0 | 0)

#define ICM20948_WHO_AM_I_ID						(0xEA)

#define ICM20948_USER_CTRL_REG				(ICM20948_REG_BANK0 | 3)

#define ICM20948_USER_CTRL_I2C_MST_RST				(1<<1)	// Reset I2C Master module
#define ICM20948_USER_CTRL_SRAM_RST					(1<<2)	// Reset SRAM module
#define ICM20948_USER_CTRL_DMP_RST					(1<<3)	// Reset DMP module
#define ICM20948_USER_CTRL_I2C_IF_DIS				(1<<4)	// Reset and disable I2C Slave module
#define ICM20948_USER_CTRL_I2C_MST_EN				(1<<5)	// Enable I2C master module
#define ICM20948_USER_CTRL_FIFO_EN					(1<<6)	// Enable FIFO operation mode
#define ICM20948_USER_CTRL_DMP_EN					(1<<7)	// Enable DMP feature

#define ICM20948_LP_CONFIG_REG				(ICM20948_REG_BANK0 | 5)

#define ICM20948_LP_CONFIG_GYRO_CYCLE				(1<<4)	// Operate Gyro in duty cycled mode
#define ICM20948_LP_CONFIG_ACCEL_CYCLE				(1<<5)	// Operate Accel in duty cycled mode
#define ICM20948_LP_CONFIG_I2C_MST_CYCLE			(1<<4)	// Operate I2C master in duty cycled mode

#define ICM20948_PWR_MGMT_1_REG				(ICM20948_REG_BANK0 | 6)

#define ICM20948_PWR_MGMT_1_CLKSEL_MASK				(7<<0)	// Clock source
#define ICM20948_PWR_MGMT_1_CLKSEL_BITPOS			(0)
#define ICM20948_PWR_MGMT_1_CLKSEL_INTERN_20MHZ		(0<<0)
#define ICM20948_PWR_MGMT_1_CLKSEL_AUTO				(1<<0)
#define ICM20948_PWR_MGMT_1_CLKSEL_STOP				(7<<0)
#define ICM20948_PWR_MGMT_1_TEMP_DIS				(1<<3)	// Disable temperature sensor
#define ICM20948_PWR_MGMT_1_LP_EN					(1<<5)	// Low Power enable
#define ICM20948_PWR_MGMT_1_SLEEP					(1<<6)	// Enter sleep
#define ICM20948_PWR_MGMT_1_DEVICE_RESET			(1<<7)	// Reset to default settings

#define ICM20948_PWR_MGMT_2_REG				(ICM20948_REG_BANK0 | 7)

#define ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK		(7<<0)	// 0 : Gyro on, 7 : Gyro off
#define ICM20948_PWR_MGMT_2_DISABLE_GYRO_BITPOS		(0)
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK		(7<<3)	// 0 : Accel on, 7 Accel off
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL_BITPOS	(3)
#define ICM20948_PWR_MGMT_2_DISABLE_PRESSURE_MASK	(3<<6)	// Undoc - 1 : off
#define ICM20948_PWR_MGMT_2_DISABLE_PRESSURE_BITPOS	(6)
#define ICM20948_PWR_MGMT_2_DISABLE_ALL				(ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK | ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK | ICM20948_PWR_MGMT_2_DISABLE_PRESSURE_MASK)


#define ICM20948_INT_PIN_CFG_REG			(ICM20948_REG_BANK0 | 15)

#define ICM20948_INT_PIN_CFG_BYPASS_EN				(1<<1)	// I2C master in bypass mode
#define ICM20948_INT_PIN_CFG_FSYNC_INT_MODE_EN		(1<<2)	// FSYNC pin used as interrupt
#define ICM20948_INT_PIN_CFG_ATCL_FSYNC				(1<<3)	// Logic level of FSYNC Interrupt. 1 : Active low, 0 : active high.
#define ICM20948_INT_PIN_CFG_INT_ANYRD_2CLEAR		(1<<4)	// Clear interrupt status
#define ICM20948_INT_PIN_CFG_INT1_LATCH_EN			(1<<5)	// Latch INT1
#define ICM20948_INT_PIN_CFG_INT1_OPEN				(1<<6)	// Open drain
#define ICM20948_INT_PIN_CFG_INT1_ACTL				(1<<7)	// Logic level.  1 : active low, 0 : active high

#define ICM20948_INT_ENABLE_REG				(ICM20948_REG_BANK0 | 16)

#define ICM20948_INT_ENABLE_I2C_MST_INT_EN			(1<<0)	// Enable I2C master interrupt on pin 1
#define ICM20948_INT_ENABLE_DMP_INT1_EN				(1<<1)	// Enable DMP interrupt on pin 1
#define ICM20948_INT_ENABLE_PLL_RDY_EN				(1<<2)	// Enable PLL ready interrupt on pin 1
#define ICM20948_INT_ENABLE_WOM_INT_EN				(1<<3)	// Enable wake on motion interrupt on pin 1
#define ICM20948_INT_ENABLE_WOF_EN				(1<<7)	// Enable wake on FSYNC interrupt

#define ICM20948_INT_ENABLE_1_REG			(ICM20948_REG_BANK0 | 17)

#define ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN		(1<<0)	// Enable raw data ready interrupt on pin 1

#define ICM20948_INT_ENABLE_2_REG			(ICM20948_REG_BANK0 | 18)

#define ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN_MASK	(0x1f<<0)// Enable FIFO overflow interrupt on pin 1
#define ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN		(1<<0)

#define ICM20948_INT_ENABLE_3_REG			(ICM20948_REG_BANK0 | 19)

#define ICM20948_INT_ENABLE_3_FIFO_WM_EN_MASK		(0x1f<<0)// Enable FIFO watermark interrupt on pin 1
#define ICM20948_INT_ENABLE_3_FIFO_WM_EN			(1<<0)

#define ICM20948_I2C_MST_STATUS_REG			(ICM20948_REG_BANK0 | 23)	// 0x17

#define ICM20948_I2C_MST_STATUS_I2C_SLV0_NACK		(1<<0)	// Slave 0 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV1_NACK		(1<<1)	// Slave 1 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV2_NACK		(1<<2)	// Slave 2 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV3_NACK		(1<<3)	// Slave 3 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV4_NACK		(1<<4)	// Slave 4 NACK
#define ICM20948_I2C_MST_STATUS_I2C_LOST_ARB		(1<<5)	// Lost arbitration
#define ICM20948_I2C_MST_STATUS_I2C_SLV4_DONE		(1<<6)	// Slave 4 transfer complete
#define ICM20948_I2C_MST_STATUS_PASS_THROUGH		(1<<7)	// FSYNC interrupt flag

#define ICM20948_DMP_INT_STATUS_REG			(ICM20948_REG_BANK0 | 24)	// 0x18
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0		(1<<0)	// CI Command
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_1		(1<<1)	// CIM Command - Motion detection SMD
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_2		(1<<2)	// CIM Command - Pedometer
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_3		(1<<3)	// CIM Command - Pedometer binning
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_4		(1<<4)	// CIM Command - Bring To See Gesture
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_5		(1<<5)	// CIM Command - Look To See Gesture

#define ICM20948_DMP_INT_STATUS_MOTION_DETECT_SMD	(1<<1)
#define ICM20948_DMP_INT_STATUS_TITL_EVENT			(1<<3)

#if 0
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT			(1<<1)
#define ICM20948_DMP_INT_STATUS_WAKE_ON_MOTION_INT	(1<<3)
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0		(1<<8)	// CI Command
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_2		(1<<9)	// CIM Command - SMD
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_3		(1<<10)	// CIM Command - Pedometer
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_4		(1<<12)	// CIM Command - Pedometer binning
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_5		(1<<13)	// CIM Command - Bring To See Gesture
#define ICM20948_DMP_INT_STATUS_MSG_DMP_INT_6		(1<<14)	// CIM Command - Look To See Gesture
#endif

#define ICM20948_INT_STATUS_REG				(ICM20948_REG_BANK0 | 25)	// 0x19

#define ICM20948_INT_STATUS_I2C_MIST_INT			(1<<0)	// I2C master interrupt flag
#define ICM20948_INT_STATUS_DMP_INT1				(1<<1)	// DMP interrupt on pin 1
#define ICM20948_INT_STATUS_PLL_RDY_INT				(1<<2)	// PLL enable & ready interrupt flag
#define ICM20948_INT_STATUS_WOM_INT					(1<<3)	// Wake on motion interrupt flag

#define ICM20948_INT_STATUS_1_REG			(ICM20948_REG_BANK0 | 26)

#define ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT	(1<<0)	// Raw date ready interrupt

#define ICM20948_INT_STATUS_2_REG			(ICM20948_REG_BANK0 | 27)

#define ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK	(0x1f<<0)	// FIFO overflow interrupt

#define ICM20948_INT_STATUS_3_REG			(ICM20948_REG_BANK0 | 28)

#define ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK	(0x1f<<0)	// Watermark interrupt for FIFO

#define ICM20948_SINGLE_FIFO_PRIORITY_SEL	(ICM20948_REG_BANK0 | 0x26)	// Undocumented
#define ICM20948_SINGLE_FIFO_PRIORITY_SEL_0XE4		(0XE4)	// To have accelerometers data and the interrupt without gyro enables.

#define ICM20948_DELAY_TIMEH_REG			(ICM20948_REG_BANK0 | 40)
#define ICM20948_DELAY_TIMEL_REG			(ICM20948_REG_BANK0 | 41)

#define ICM20948_ACCEL_XOUT_H_REG			(ICM20948_REG_BANK0 | 45)
#define ICM20948_ACCEL_XOUT_L_REG			(ICM20948_REG_BANK0 | 46)
#define ICM20948_ACCEL_YOUT_H_REG			(ICM20948_REG_BANK0 | 47)
#define ICM20948_ACCEL_YOUT_L_REG			(ICM20948_REG_BANK0 | 48)
#define ICM20948_ACCEL_ZOUT_H_REG			(ICM20948_REG_BANK0 | 49)
#define ICM20948_ACCEL_ZOUT_L_REG			(ICM20948_REG_BANK0 | 50)

#define ICM20948_GYRO_XOUT_H_REG			(ICM20948_REG_BANK0 | 51)
#define ICM20948_GYRO_XOUT_L_REG			(ICM20948_REG_BANK0 | 52)
#define ICM20948_GYRO_YOUT_H_REG			(ICM20948_REG_BANK0 | 53)
#define ICM20948_GYRO_YOUT_L_REG			(ICM20948_REG_BANK0 | 54)
#define ICM20948_GYRO_ZOUT_H_REG			(ICM20948_REG_BANK0 | 55)
#define ICM20948_GYRO_ZOUT_L_REG			(ICM20948_REG_BANK0 | 56)

#define ICM20948_TEMP_OUT_H_REG				(ICM20948_REG_BANK0 | 57)
#define ICM20948_TEMP_OUT_L_REG				(ICM20948_REG_BANK0 | 58)

// External I2C access
#define ICM20948_EXT_SLV_SENS_DATA_00_REG	(ICM20948_REG_BANK0 | 59)
#define ICM20948_EXT_SLV_SENS_DATA_23_REG	(ICM20948_REG_BANK0 | 82)
#define ICM20948_EXT_SLV_SENS_DATA_MAX_REG	(ICM20948_EXT_SLV_SENS_DATA_23 - ICM20948_EXT_SLV_SENS_DATA_00)

#define ICM20948_FIFO_EN_1_REG				(ICM20948_REG_BANK0 | 102)

#define ICM20948_FIFO_EN_1_SLV_0_FIFO_EN		(1<<0)	// Enable Slave 0 FIFO
#define ICM20948_FIFO_EN_1_SLV_1_FIFO_EN		(1<<1)	// Enable Slave 1 FIFO
#define ICM20948_FIFO_EN_1_SLV_2_FIFO_EN		(1<<2)	// Enable Slave 2 FIFO
#define ICM20948_FIFO_EN_1_SLV_3_FIFO_EN		(1<<3)	// Enable Slave 3 FIFO

#define ICM20948_FIFO_EN_2_REG				(ICM20948_REG_BANK0 | 103)

#define ICM20948_FIFO_EN_2_TEMP_FIFO_EN			(1<<0)	// Temp out fifo enable
#define ICM20948_FIFO_EN_2_GYRO_X_FIFO_EN		(1<<1)	// Gyro X out fifo enable
#define ICM20948_FIFO_EN_2_GYRO_Y_FIFO_EN		(1<<2)	// Gyro Y out fifo enable
#define ICM20948_FIFO_EN_2_GYRO_Z_FIFO_EN		(1<<3)	// Gyro Z out fifo enable
#define ICM20948_FIFO_EN_2_ACCEL_FIFO_EN		(1<<4)	// Accel out fifo enable
#define ICM20948_FIFO_EN_2_FIFO_EN_ALL			(0x1F)

#define ICM20948_FIFO_RST_REG				(ICM20948_REG_BANK0 | 104)

#define ICM20948_FIFO_RST_FIFO_RESET_MASK		(0x1f<<0)	// Software fifo reset

#define ICM20948_FIFO_MODE_REG				(ICM20948_REG_BANK0 | 105)
#define ICM20948_FIFO_MODE_FIFO_MODE_MASK		(0x1f<<0)	//
#define ICM20948_FIFO_MODE_SNAPSHOT				(1<<0)	//!< Blocking

#define ICM20948_FIFO_COUNTH_REG			(ICM20948_REG_BANK0 | 112)

#define ICM20948_FIFO_COUNTH_FIFO_CNT_MASK		(0x1f<<0)

#define ICM20948_FIFO_COUNTL_REG			(ICM20948_REG_BANK0 | 113)

#define ICM20948_FIFO_R_W_REG				(ICM20948_REG_BANK0 | 114)

#define ICM20948_DATA_RDY_STATUS_REG		(ICM20948_REG_BANK0 | 116)

#define ICM20948_DATA_RDY_STATUS_RAW_DATA_RDY_MASK	(0xf<<0)
#define ICM20948_DATA_RDY_STATUS_WOF_STATUS			(1<<7)		// Wake on FSYNC interrupt status

#define ICM20948_HWTEMP_FIX_DISABLE_REG		(ICM20948_REG_BANK0 | 117)	// Undocumented
#define ICM20948_HWTEMP_FIX_DISABLE_DIS				(1<<3)

#define ICM20948_FIFO_CFG_REG				(ICM20948_REG_BANK0 | 118)
#define ICM20948_FIFO_CFG_MUTLI						(1<<0)		// Set to 1 of interrupt status for each sensor is required
#define ICM20948_FIFO_CFG_SINGLE					(0<<0)

// DMP
#define ICM20948_DMP_MEM_STARTADDR_REG		(ICM20948_REG_BANK0 | 0x7C)
#define ICM20948_DMP_MEM_RW_REG        		(ICM20948_REG_BANK0 | 0x7D)
#define ICM20948_DMP_MEM_BANKSEL_REG		(ICM20948_REG_BANK0 | 0x7E)
#define ICM20948_DMP_PROG_START_ADDRH_REG	(ICM20948_REG_BANK2 | 0x50)
#define ICM20948_DMP_PROG_START_ADDRL_REG	(ICM20948_REG_BANK2 | 0x51)

//*** Register Bank 1

#define ICM20948_SELF_TEST_X_GYRO_REG		(ICM20948_REG_BANK1 | 2)
#define ICM20948_SELF_TEST_Y_GYRO_REG		(ICM20948_REG_BANK1 | 3)
#define ICM20948_SELF_TEST_Z_GYRO_REG		(ICM20948_REG_BANK1 | 4)

#define ICM20948_SELF_TEST_X_ACCEL_REG		(ICM20948_REG_BANK1 | 14)
#define ICM20948_SELF_TEST_Y_ACCEL_REG		(ICM20948_REG_BANK1 | 15)
#define ICM20948_SELF_TEST_Z_ACCEL_REG		(ICM20948_REG_BANK1 | 16)

// Accel offset trim. 0.98 mg steps
#define ICM20948_ACCEL_TRIM_SCALE							(0.000098)
#define ICM20948_XA_OFFS_H_REG				(ICM20948_REG_BANK1 | 20)
#define ICM20948_XA_OFFS_L_REG				(ICM20948_REG_BANK1 | 21)

#define ICM20948_YA_OFFS_H_REG				(ICM20948_REG_BANK1 | 23)
#define ICM20948_YA_OFFS_L_REG				(ICM20948_REG_BANK1 | 24)

#define ICM20948_ZA_OFFS_H_REG				(ICM20948_REG_BANK1 | 26)
#define ICM20948_ZA_OFFS_L_REG				(ICM20948_REG_BANK1 | 27)

#define ICM20948_TIMEBASE_CORRECTION_PLL_REG	(ICM20948_REG_BANK1 | 40)

//*** Register Bank 2

#define ICM20948_GYRO_SMPLRT_DIV_REG		(ICM20948_REG_BANK2 | 0)	// Gyro sample rate divider

#define ICM20948_GYRO_CONFIG_1_REG			(ICM20948_REG_BANK2 | 1)

#define ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE					(1<<0)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_MASK				(3<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_250DPS			(0<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_500DPS			(1<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS			(2<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_2000DPS			(3<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_MASK			(7<<3)	// Format is dAbwB_nXbwY :
																	//		A is integer part of 3db BW, B is fraction.
																	// 		X is integer part of nyquist bandwidth, Y is fraction
#define ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS			3

#define ICM20948_GYRO_CONFIG_2_REG			(ICM20948_REG_BANK2 | 2)

#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_MASK				(7<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_1X				(0<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_2X				(1<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_4X				(2<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_8X				(3<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_16X				(4<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_32X				(5<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_64X				(6<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_128X				(7<<0)
#define ICM20948_GYRO_CONFIG_2_ZGYRO_CTEN					(1<<3)	// Z Gyro self test enable
#define ICM20948_GYRO_CONFIG_2_YGYRO_CTEN					(1<<4)	// Z Gyro self test enable
#define ICM20948_GYRO_CONFIG_2_XGYRO_CTEN					(1<<5)	// Z Gyro self test enable

// Accel offset trim. 0.98 mg steps
#define ICM20948_GYRO_TRIM_SCALE							(0.0305)
#define ICM20948_XG_OFFS_USRH_REG			(ICM20948_REG_BANK2 | 3)
#define ICM20948_XG_OFFS_USRL_REG			(ICM20948_REG_BANK2 | 4)
#define ICM20948_YG_OFFS_USRH_REG			(ICM20948_REG_BANK2 | 5)
#define ICM20948_YG_OFFS_USRL_REG			(ICM20948_REG_BANK2 | 6)
#define ICM20948_ZG_OFFS_USRH_REG			(ICM20948_REG_BANK2 | 7)
#define ICM20948_ZG_OFFS_USRL_REG			(ICM20948_REG_BANK2 | 8)

#define ICM20948_ODR_ALIGN_EN_REG			(ICM20948_REG_BANK2 | 9)

#define ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN					(1<<0)	// Enable ODR start time alignment

#define ICM20948_ACCEL_SMPLRT_DIV_1_REG		(ICM20948_REG_BANK2 | 16)

#define ICM20948_ACCEL_SMPLRT_DIV_1_ACCEL_SMPLRT_DIV_MASK	(0xf<<0)

#define ICM20948_ACCEL_SMPLRT_DIV_2_REG		(ICM20948_REG_BANK2 | 17)

#define ICM20948_ACCEL_INTEL_CTRL_REG		(ICM20948_REG_BANK2 | 18)

#define ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_CMPPREV	(1<<0)	// Select WOM algorithm
#define ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN			(1<<1)	// Enable WOM logic

#define ICM20948_ACCEL_WOM_THR_REG			(ICM20948_REG_BANK2 | 19)

#define ICM20948_ACCEL_CONFIG_REG			(ICM20948_REG_BANK2 | 20)

#define ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE					(1<<0)	// Enable accel DLPF
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK				(3<<1)	// Full scale select mask
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_2G				(0<<1)	// Full scale select 2g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_4G				(1<<1)	// Full scale select 4g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_8G				(2<<1)	// Full scale select 8g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_16G				(3<<1)	// Full scale select 16g
#define ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_MASK			(7<<3)	// Format is dAbwB_nXbwZ :
																	//		A is integer part of 3db BW, B is fraction.
																	//		X is integer part of nyquist bandwidth, Y is fraction
#define ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS			3

#define ICM20948_ACCEL_CONFIG_2_REG			(ICM20948_REG_BANK2 | 21)

#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_MASK				(3<<0)	// Control the number of sample averaged
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_4					(0<<0)	// 1 or 4 samples
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_8					(1<<0)	// 8 samples
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_16					(2<<0)	// 16 samples
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_32					(3<<0)	// 32 samples
#define ICM20948_ACCEL_CONFIG_2_AZ_ST_EN_REG				(1<<2)	// Z accel self test enable
#define ICM20948_ACCEL_CONFIG_2_AY_ST_EN_REG				(1<<3)	// Y accel self test enable
#define ICM20948_ACCEL_CONFIG_2_AX_ST_EN_REG				(1<<4)	// X accel self test enable

#define ICM20948_FSYNC_CONFIG_REG			(ICM20948_REG_BANK2 | 82)

#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_MASK				(0xf<<0)// Enable FSYNC pin data to be sampled
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_DIS				(0<<0)	// Disable FSYNC
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_TEMP_OUT_L		(1<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_GYRO_XOUT_L		(2<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_GYRO_YOUT_L		(3<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_GYRO_ZOUT_L		(4<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_ACCEL_XOUT_L		(5<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_ACCEL_YOUT_L		(6<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_ACCEL_ZOUT_L		(7<<0)
#define ICM20948_FSYNC_CONFIG_WOF_EDGE_INT					(1<<4)	// FSYNC interrupt level
#define ICM20948_FSYNC_CONFIG_WOF_DEGLITCH_EN				(1<<5)	// Enable digital deglitching of FSYNC inpiut for Wake on FSYNC
#define ICM20948_FSYNC_CONFIG_DELAY_TIME_EN					(1<<7)	// Enable delay time measurement between FSYNC event and the first ODR event

#define ICM20948_TEMP_CONFIG_REG			(ICM20948_REG_BANK2 | 83)

#define ICM20948_TEMP_CONFIG_DLPFCFG						(7<<0)	// Low pass filter for temperature sensor

#define ICM20948_MOD_CTRL_USR_REG			(ICM20948_REG_BANK2 | 84)

#define ICM20948_MOD_CTRL_USR_REG_LP_DMP_EN					(1<<0)	// Enable turning on DMP in low power accel mode

//*** Register Bank 3

#define ICM20948_I2C_MST_ODR_CONFIG_REG		(ICM20948_REG_BANK3 | 0)

#define ICM20948_I2C_MST_ODR_CONFIG_MASK					(0xf<<0)

#define ICM20948_I2C_MST_CTRL_REG			(ICM20948_REG_BANK3 | 1)

#define ICM20948_I2C_MST_CTRL_I2C_MST_CLK_MASK				(0xf<<0)// I2C master clock freq
#define ICM20948_I2C_MST_CTRL_I2C_MST_P_NSR					(1<<4)	// Control I2C master transition from one slave to the next
#define ICM20948_I2C_MST_CTRL_MULT_MST_EN					(1<<7)	// Enable multi-master capability

#define ICM20948_I2C_MST_DELAY_CTRL_REG		(ICM20948_REG_BANK3 | 2)

#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV0_DELAY_EN		(1<<0)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV1_DELAY_EN		(1<<1)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV2_DELAY_EN		(1<<2)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV3_DELAY_EN		(1<<3)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV4_DELAY_EN		(1<<4)
#define ICM20948_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW			(1<<7)	// Delays shadowing of external sensor data

#define ICM20948_I2C_SLV0_ADDR_REG			(ICM20948_REG_BANK3 | 3)

#define ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK				(0x7f<<0)	// Physical I2C address slave 0
#define ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR					(0<<7)	// Write transfer

#define ICM20948_I2C_SLV0_REG_REG			(ICM20948_REG_BANK3 | 4)	// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV0_CTRL_REG			(ICM20948_REG_BANK3 | 5)

#define ICM20948_I2C_SLV_MAXLEN								(15)
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LEN_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_GRP					(1<<4)	//
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN					(1<<7)	// Enable reading data from slave

#define ICM20948_I2C_SLV0_DO_REG			(ICM20948_REG_BANK3 | 6)						// Data out to slave

#define ICM20948_I2C_SLV1_ADDR_REG			(ICM20948_REG_BANK3 | 7)

#define ICM20948_I2C_SLV1_ADDR_I2C_ID_1_MASK				(0x7f<<0)	// Physical I2C address slave 1
#define ICM20948_I2C_SLV1_ADDR_I2C_SLV1_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV1_ADDR_I2C_SLV1_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV1_REG_REG			(ICM20948_REG_BANK3 | 8)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV1_CTRL_REG			(ICM20948_REG_BANK3 | 9)

#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_GRP					(1<<4)	//
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_EN					(1<<7)

#define ICM20948_I2C_SLV1_DO_REG			(ICM20948_REG_BANK3 | 10)						// Data out to slave

#define ICM20948_I2C_SLV2_ADDR_REG			(ICM20948_REG_BANK3 | 11)

#define ICM20948_I2C_SLV2_ADDR_I2C_ID_2_MASK				(0x7f<<0)	// Physical I2C address slave 2
#define ICM20948_I2C_SLV2_ADDR_I2C_SLV2_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV2_ADDR_I2C_SLV2_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV2_REG_REG			(ICM20948_REG_BANK3 | 12)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV2_CTRL_REG			(ICM20948_REG_BANK3 | 13)

#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_GRP					(1<<4)	//
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,																// it will only read data, or write data
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_EN					(1<<7)

#define ICM20948_I2C_SLV2_DO_REG			(ICM20948_REG_BANK3 | 14)						// Data out to slave

#define ICM20948_I2C_SLV3_ADDR_REG			(ICM20948_REG_BANK3 | 15)

#define ICM20948_I2C_SLV3_ADDR_I2C_ID_3_MASK				(0x7f<<0)	// Physical I2C address slave 3
#define ICM20948_I2C_SLV3_ADDR_I2C_SLV3_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV3_ADDR_I2C_SLV3_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV3_REG_REG			(ICM20948_REG_BANK3 | 16)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV3_CTRL_REG			(ICM20948_REG_BANK3 | 17)

#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_GRP					(1<<4)	//
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
															// it will only read data, or write data
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_EN					(1<<7)

#define ICM20948_I2C_SLV3_DO_REG			(ICM20948_REG_BANK3 | 18)						// Data out to slave

#define ICM20948_I2C_SLV4_ADDR_REG			(ICM20948_REG_BANK3 | 19)

#define ICM20948_I2C_SLV4_ADDR_I2C_ID_4_MASK				(0x7f<<0)	// Physical I2C address slave 4
#define ICM20948_I2C_SLV4_ADDR_I2C_SLV4_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV4_ADDR_I2C_SLV4_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV4_REG_REG			(ICM20948_REG_BANK3 | 20)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV4_CTRL_REG			(ICM20948_REG_BANK3 | 21)

#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_DLY_MASK			(0x1f<<0)	//
#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_INT_EN 				(1<<6)
#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_EN					(1<<7)

#define ICM20948_I2C_SLV4_DO_REG			(ICM20948_REG_BANK3 | 22)						// Data out to slave

#define ICM20948_I2C_SLV4_DI_REG			(ICM20948_REG_BANK3 | 23)

//*** Mag registers AK09916

#define AK09916_I2C_ADDR1				0xC						// AK09916 I2C 7 bits address
#define AK09916_I2C_ADDR2				0xE						// AK09916 I2C 7 bits address

#define ICM20948_AK09916_WIA1			0x0						// Device ID

#define ICM20948_AK09916_WIA1_ID							(0x48)

#define ICM20948_AK09916_WIA2			0x1						// Device ID

#define ICM20948_AK09916_WIA2_ID							(9)

#define ICM20948_AK09916_ST1			0x10

#define ICM20948_ST1_DRDY									(1<<0)	// Data ready
#define ICM20948_ST1_DOR									(1<<1)	// Data skipped

#define ICM20948_AK09916_HXL			0x11
#define ICM20948_AK09916_HXH			0x12
#define ICM20948_AK09916_HYL			0x13
#define ICM20948_AK09916_HYH			0x14
#define ICM20948_AK09916_HZL			0x15
#define ICM20948_AK09916_HZH			0x16

#define ICM20948_ST2					0x18

#define ICM20948_ST2_HOFL									(1<<3)	// Magnetic sensor overflow

#define ICM20948_AK09916_CNTL2			0x31

#define ICM20948_AK09916_CNTL2_MODE_MASK					(0x1f<<0)
#define ICM20948_AK09916_CNTL2_MODE_PWRDWN					(0<<0)	// Power down
#define ICM20948_AK09916_CNTL2_MODE_SINGLE					(1<<0)	// Single measurement
#define ICM20948_AK09916_CNTL2_MODE_CONT1					(2<<0)	// Continuous mode 1
#define ICM20948_AK09916_CNTL2_MODE_CONT2					(4<<0)	// Continuous mode 2
#define ICM20948_AK09916_CNTL2_MODE_CONT3					(6<<0)	// Continuous mode 3
#define ICM20948_AK09916_CNTL2_MODE_CONT4					(8<<0)	// Continuous mode 4
#define ICM20948_AK09916_CNTL2_MODE_SELFTEST				(0x10<<0)	// Self test

#define ICM20948_AK09916_CNTL3			0x32

#define ICM20948_AK09916_CNTL3_SRST							(1<<0)	// Soft-reset

#define ICM20948_ACC_ADC_RANGE			32767
#define ICM20948_GYRO_ADC_RANGE			32767
#define AK09916_ADC_RANGE				32752

//#define ICM20948_DMP_MEM_BANK_SIZE			256		//!< DMP memory bank size
//#define ICM20948_DMP_PROG_START_ADDR		0x1000U
//#define ICM20948_DMP_LOAD_MEM_START_ADDR	0x90

#define ICM20948_FIFO_SIZE_MAX			1024
#define ICM20948_FIFO_PAGE_SIZE			16

// Fifo header 1 bit field definitions 16 bits
#define ICM20948_FIFO_HEADER_ACCEL					0x8000
#define ICM20948_FIFO_HEADER_ACCEL_SIZE				6
#define ICM20948_FIFO_HEADER_GYRO					0x4000
#define ICM20948_FIFO_HEADER_GYRO_SIZE				12	// 6 bytes Gyro data + 6 bytes Gyro bias
#define ICM20948_FIFO_HEADER_CPASS					0x2000
#define ICM20948_FIFO_HEADER_CPASS_SIZE				6
#define ICM20948_FIFO_HEADER_ALS					0x1000
#define ICM20948_FIFO_HEADER_ALS_SIZE				8
#define ICM20948_FIFO_HEADER_QUAT6					0x0800
#define ICM20948_FIFO_HEADER_QUAT6_SIZE				12
#define ICM20948_FIFO_HEADER_QUAT9					0x0400
#define ICM20948_FIFO_HEADER_QUAT9_SIZE				14
#define ICM20948_FIFO_HEADER_PEDO_QUAT6				0x0200
#define ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE		6
#define ICM20948_FIFO_HEADER_GEOMAG					0x0100
#define ICM20948_FIFO_HEADER_GEOMAG_SIZE			14
#define ICM20948_FIFO_HEADER_PRESS_TEMP				0x0080
#define ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE		6
#define ICM20948_FIFO_HEADER_CALIB_GYRO				0x0040
#define ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE		12
#define ICM20948_FIFO_HEADER_CALIB_CPASS			0x0020
#define ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE		12
#define ICM20948_FIFO_HEADER_STEP_DETECTOR			0x0010
#define ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE		4
#define ICM20948_FIFO_HEADER_HEADER2				0x0008
#define ICM20948_FIFO_HEADER_HEADER2_SIZE			2
#define ICM20948_FIFO_HEADER_FOOTER					0x0001
#define ICM20948_FIFO_FOOTER_SIZE					2

#define ICM20948_FIFO_HEADER_MASK					(ICM20948_FIFO_HEADER_ACCEL | ICM20948_FIFO_HEADER_GYRO | \
													 ICM20948_FIFO_HEADER_CPASS | ICM20948_FIFO_HEADER_ALS | \
													 ICM20948_FIFO_HEADER_QUAT6 | ICM20948_FIFO_HEADER_QUAT9 | \
													 ICM20948_FIFO_HEADER_PEDO_QUAT6 | ICM20948_FIFO_HEADER_GEOMAG | \
													 ICM20948_FIFO_HEADER_PRESS_TEMP | ICM20948_FIFO_HEADER_CALIB_GYRO | \
													 ICM20948_FIFO_HEADER_CALIB_GYRO | ICM20948_FIFO_HEADER_CALIB_CPASS | \
													 ICM20948_FIFO_HEADER_STEP_DETECTOR | ICM20948_FIFO_HEADER_HEADER2)

#define ICM20948_FIFO_HEADER2_ACCEL_ACCUR			0x4000
#define ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE		2
#define ICM20948_FIFO_HEADER2_GYRO_ACCUR			0x2000
#define ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE		2
#define ICM20948_FIFO_HEADER2_CPASS_ACCUR			0x1000
#define ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE		2
#define ICM20948_FIFO_HEADER2_FSYNC					0x0800
#define ICM20948_FIFO_HEADER2_FSYNC_SIZE			2
#define ICM20948_FIFO_HEADER2_PICKUP				0x0400
#define ICM20948_FIFO_HEADER2_PICKUP_SIZE			2
#define ICM20948_FIFO_HEADER2_ACTI_RECOG			0x0080
#define ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE		6
#define ICM20948_FIFO_HEADER2_SECOND_ONOFF			0x0040
#define ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE		2

#define ICM20948_FIFO_HEADER2_MASK					(ICM20948_FIFO_HEADER2_ACCEL_ACCUR | ICM20948_FIFO_HEADER2_GYRO_ACCUR | \
													 ICM20948_FIFO_HEADER2_CPASS_ACCUR | ICM20948_FIFO_HEADER2_FSYNC | \
													 ICM20948_FIFO_HEADER2_PICKUP | ICM20948_FIFO_HEADER2_ACTI_RECOG | \
													 ICM20948_FIFO_HEADER2_SECOND_ONOFF)

#define ICM20948_FIFO_HEADER_SIZE					2

#define ICM20948_FIFO_MAX_PKT_SIZE					(ICM20948_FIFO_HEADER_ACCEL_SIZE + ICM20948_FIFO_HEADER_GYRO_SIZE +\
													 ICM20948_FIFO_HEADER_CPASS_SIZE + ICM20948_FIFO_HEADER_ALS_SIZE +\
													 ICM20948_FIFO_HEADER_QUAT6_SIZE + ICM20948_FIFO_HEADER_QUAT9_SIZE +\
													 ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE + ICM20948_FIFO_HEADER_GEOMAG_SIZE +\
													 ICM20948_FIFO_HEADER_PRESSURE_SIZE + ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE +\
													 ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE + ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE + \
													 ICM20948_FIFO_HEADER_HEADER2_SIZE + ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE +\
													 ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE + ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE +\
													 ICM20948_FIFO_HEADER2_FSYNC_SIZE + ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE +\
													 ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE + ICM20948_FIFO_HEADER_SIZE + \
													 ICM20948_FIFO_FOOTER_SIZE)

#define ICM20948_ACCEL_IDX		0
#define ICM20948_GYRO_IDX		1
#define ICM20948_MAG_IDX		2
#define ICM20948_TEMP_IDX		3
#define ICM20948_NB_SENSOR		4


#pragma pack(push, 1)

#pragma pack(pop)

#ifdef __cplusplus

class AccelIcm20948 : public AccelSensor {
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
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint16_t Scale(uint16_t Value);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint32_t FilterFreq(uint32_t Freq);
	virtual bool Enable();
	void UpdateData(uint64_t Timestamp, int16_t X, int16_t Y, int16_t Z) {
		vData.Timestamp = Timestamp; vData.X = X; vData.Y = Y; vData.Z = Z;
	}

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter = 0, DEVINTR_POL Pol = DEVINTR_POL_LOW, Timer * const pTimer = NULL) = 0;
};

class GyroIcm20948 : public GyroSensor {
public:
	/**
	 * @brief	Initialize gyroscope sensor.
	 *
	 * NOTE : Accelerometer must be initialized first prior to this one.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t Sensitivity(uint32_t Value);	// Gyro
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint32_t FilterFreq(uint32_t Freq);
	virtual bool Enable();
	void UpdateData(uint64_t Timestamp, int16_t X, int16_t Y, int16_t Z) {
		vData.Timestamp = Timestamp; vData.X = X; vData.Y = Y; vData.Z = Z;
	}

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter = 0, DEVINTR_POL Pol = DEVINTR_POL_LOW, Timer * const pTimer = NULL) = 0;
};

class MagIcm20948 : public MagAk09916 {
public:
	/**
	 * @brief	Initialize magnetometer sensor.
	 *
	 * NOTE : Accelerometer must be initialized first prior to this one.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL);
//	virtual uint32_t SamplingFrequency(uint32_t Freq);
	//virtual bool Enable();
	//virtual void Disable();
	void UpdateData(uint64_t Timestamp, int16_t X, int16_t Y, int16_t Z) {
		vData.Timestamp = Timestamp; vData.X = X; vData.Y = Y; vData.Z = Z;
	}

protected:
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) = 0;
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) = 0;

	uint8_t vMagCtrl1Val;
	int16_t vMagSenAdj[3];

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter = 0, DEVINTR_POL Pol = DEVINTR_POL_LOW, Timer * const pTimer = NULL) = 0;
//	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	//virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) = 0;
	//virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) = 0;

	uint32_t vDevAddr;
};

class AgmIcm20948 : public AccelIcm20948, public GyroIcm20948, public MagIcm20948, public TempSensor {
	friend class ImuIcm20948;
	friend class AccelIcm20948;
	friend class GyroIcm20948;
	friend class MagIcm20948;

public:
	AgmIcm20948();

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
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[ICM20948_ACCEL_IDX] = AccelIcm20948::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[ICM20948_ACCEL_IDX];
	}

	/**
	 * @brief	Initialize gyroscope sensor.
	 *
	 * NOTE : Accelerometer must be initialized first prior to this one.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[ICM20948_GYRO_IDX] = GyroIcm20948::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[ICM20948_GYRO_IDX];
	}

	/**
	 * @brief	Initialize magnetometer sensor.
	 *
	 * NOTE : Accelerometer must be initialized first prior to this one.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[ICM20948_MAG_IDX] = MagIcm20948::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[ICM20948_MAG_IDX];
	}

	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	Cfg 	: Reference to configuration data
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
	virtual bool Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	/**
	 * @brief	Power off the device completely.
	 *
	 * If supported, this will put the device in complete power down.
	 * Full re-initialization is required to re-enable the device.
	 */
	virtual void PowerOff();

	/**
	 * @brief	Enable/Disable wake on motion event
	 *
	 * @param bEnable
	 * @param Threshold
	 * @return
	 */
	virtual bool WakeOnEvent(bool bEnable, int Threshold);

	virtual bool StartSampling();
	virtual bool Read(AccelSensorRawData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(GyroSensorRawData_t &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(MagSensorRawData_t &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return MagSensor::Read(Data); }
	virtual void Read(TempSensorData_t &Data) { return TempSensor::Read(Data); }

	int Read(uint16_t RegAddr, uint8_t *pBuff, int BuffLen) {
		return Read((uint8_t*)&RegAddr, 2, pBuff, BuffLen);
	}
	int Write(uint16_t RegAddr, uint8_t *pData, int DataLen) {
		return Write((uint8_t*)&RegAddr, 2, pData, DataLen);
	}

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

	bool UpdateData();
	void UpdateData(SENSOR_TYPE Type, uint64_t Timestamp, uint8_t * const pData);
	virtual void IntHandler();

private:
	AgmIcm20948(const AgmIcm20948&); // no copy constructor

	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter = 0, DEVINTR_POL Pol = DEVINTR_POL_LOW, Timer * const pTimer = NULL);
	bool SelectBank(uint8_t BankNo);
	bool vbSensorEnabled[ICM20948_NB_SENSOR];
	uint8_t vCurrBank;
	SENSOR_TYPE vType;	//!< Bit field indicating the sensors contain within
};

#endif // __cplusplus

/** @} End of group Sensors */

#endif // __AGM_ICM20948_H__

