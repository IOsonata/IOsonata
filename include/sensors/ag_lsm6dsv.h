/**-------------------------------------------------------------------------
@file	ag_lsm6dsv.h

@brief	Definitions for LMS6x accel & gyro sensor


@author	Hoang Nguyen Hoan
@date	Feb. 28, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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
#ifndef __AG_LSM6DSV_H__
#define __AG_LSM6DSV_H__


/** @addtogroup Sensors
  * @{
  */

#define LSM6DSV_I2C_DEV_ADDR0			0x6A	// SA0 = 0
#define LSM6DSV_I2C_DEV_ADDR1			0x6B	// SA0 = 1


#define LSM6DSV_FUNC_CFG_ACCESS_REG				0x1
#define LSM6DSV_FUNC_CFG_ACCESS_OIS_CTRL								(1<<0)	//!< Enables the full control of OIS configurations from the primary interface.
#define LSM6DSV_FUNC_CFG_ACCESS_SPI2_RESET								(1<<1)	//!< Resets the control registers of SPI2 from the primary interface
#define LSM6DSV_FUNC_CFG_ACCESS_SW_POR									(1<<2)	//!< Global reset of the device
#define LSM6DSV_FUNC_CFG_ACCESS_FSM_WR_CTRL								(1<<3)	//!< Enables the control of the CTRL registers to FSM
#define LSM6DSV_FUNC_CFG_ACCESS_SHUB									(1<<6)	//!< Enables access to the sensor hub (I2C master) configuration registers.
#define LSM6DSV_FUNC_CFG_ACCESS_EMB_FCT									(1<<7)	//!< Enables access to the embedded functions configuration registers

#define LSM6DSV_PIN_CTRL_REG					0x2
#define LSM6DSV_PIN_CTRL_IBHR_POR_EN									(1<<5)	//!< Selects the action the device performs after "reset whole chip" I3C pattern
#define LSM6DSV_PIN_CTRL_SDO_PU_EB										(1<<6)	//!< Enables pull-up on SDO pin
#define LSM6DSV_PIN_CTRL_OIS_PU_DIS										(1<<7)	//!< Disables pull-up on both OCS_Aux and SDO_Aux pins

#define LSM6DSV_IF_CFG_REG						0x3
#define LSM6DSV_IF_CFG_I2C_I3C_DIS										(1<<0)	//!< Disable I2C and MIPI I3C interface
#define LSM6DSV_IF_CFG_SIM												(1<<2)	//!< SPI serial interface mode selection : 0 - 4 wire, 1 - 3 wire
#define LSM6DSV_IF_CFG_PP_OD											(1<<3)	//!< Push-pull / open drain selection : 0 - push-pull, 1 - open drain
#define LSM6DSV_IF_CFG_INT_ACTIVE_LOW									(1<<4)	//!< Interrupt active level
#define LSM6DSV_IF_CFG_ASF_CTRL_EN										(1<<5)	//!< Enable antispike filters
#define LSM6DSV_IF_CFG_SHUB_PU_EN										(1<<6)	//!< Enable master I2C pull-up on aux
#define LSM6DSV_IF_CFG_SDA_PU_EN										(1<<7)	//!< Enable pullup on SDA pin

#define LSM6DSV_ODR_TRIG_CFG_REG				0x6
#define LSM6DSV_ODR_TRIG_CFG_MASK										(0xFF)

// Fifo watermark threshold : 1 LSB = TAG (1 byte) + 1 sensor (6 bytes) written in FIFO
#define LSM6DSV_FIFO_CTRL1_REG					0x7

#define LSM6DSV_FIFO_CTRL2_REG					0x8
#define LSM6DSV_FIFO_CTRL2_XL_DUALC_BATCH_FSM_EN						(1<<0)	//!< When dual-channel mode is enabled, this bit enables FSM-triggered batching in FIFO of acc chan 2
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_MASK							(3<<1)
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_NONE							(0<<1)
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_8								(1<<1)	//!< uncompressed data every 8 batch data rate
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_16								(2<<1)	//!< uncompressed data every 16 batch data rate
#define LSM6DSV_FIFO_CTRL2_UNCOMPR_RATE_32								(3<<1)	//!< uncompressed data every 32 batch data rate
#define LSM6DSV_FIFO_CTRL2_ODR_CHG_EN									(1<<4)	//!< Enable ODR change virtual sensor batch
#define LSM6DSV_FIFO_CTRL2_FIFO_COMPR_RT_EN								(1<<6)	//!< Enable FIFO compression algo
#define LSM6DSV_FIFO_CTRL2_STOP_ON_WTM									(1<<7)	//!< Set FIFO depth limited to threshold level

#define LSM6DSV_FIFO_CTRL3_REG					0x9
#define LSM6DSV_FIFO_CTRL3_BDR_XL_MASK									(0xF<<0)//!< Select XL batch data rate mask
#define LSM6DSV_FIFO_CTRL3_BDR_XL_DISABLE								(0<<0)
#define LSM6DSV_FIFO_CTRL3_BDR_XL_1_875HZ								(1<<0)	//!< BDR 1.875 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_7_5HZ									(2<<0)	//!< BDR 7.5 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_15HZ									(3<<0)	//!< BDR 15 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_30HZ									(4<<0)	//!< BDR 30 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_60HZ									(5<<0)	//!< BDR 60 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_120HZ									(6<<0)	//!< BDR 120 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_240HZ									(7<<0)	//!< BDR 240 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_480HZ									(8<<0)	//!< BDR 480 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_960HZ									(9<<0)	//!< BDR 960 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_1920HZ								(10<<0)	//!< BDR 1920 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_3840HZ								(11<<0)	//!< BDR 3840 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_XL_7680HZ								(12<<0)	//!< BDR 7680 Hz

#define LSM6DSV_FIFO_CTRL3_BDR_GY_MASK									(0xF<<0)//!< Select GY batch data rate mask
#define LSM6DSV_FIFO_CTRL3_BDR_GY_DISABLE								(0<<0)
#define LSM6DSV_FIFO_CTRL3_BDR_GY_1_875HZ								(1<<0)	//!< BDR 1.875 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_7_5HZ									(2<<0)	//!< BDR 7.5 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_15HZ									(3<<0)	//!< BDR 15 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_30HZ									(4<<0)	//!< BDR 30 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_60HZ									(5<<0)	//!< BDR 60 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_120HZ									(6<<0)	//!< BDR 120 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_240HZ									(7<<0)	//!< BDR 240 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_480HZ									(8<<0)	//!< BDR 480 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_960HZ									(9<<0)	//!< BDR 960 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_1920HZ								(10<<0)	//!< BDR 1920 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_3840HZ								(11<<0)	//!< BDR 3840 Hz
#define LSM6DSV_FIFO_CTRL3_BDR_GY_7680HZ								(12<<0)	//!< BDR 7680 Hz

#define LSM6DSV_FIFO_CTRL4_REG					0xA
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_MASK								(7<<0)
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_DISABLE							(0<<0)	//!< Disable FIFO
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_STOP_FULL							(1<<0)	//!< Stop on full
#define LSM6DSV_FIFO_CTRL4_FIFO_MODE_OVWR								(6<<0)	//!< Overwrite old data when fifo full
#define LSM6DSV_FIFO_CTRL4_G_EIS_FIFO_EN								(1<<3)	//!< Enable FIFO batching of enhanced EIS gyo output
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_MASK								(3<<4)	//!< Select FIFO batch data rate for temperature
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_1_875HZ							(1<<4)	//!< FIFO Temp. rate 1.875 Hz
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_15HZ								(2<<4)	//!< FIFO Temp. rate 15 Hz
#define LSM6DSV_FIFO_CTRL4_ODR_T_BATCH_60HZ								(3<<4)	//!< FIFO Temp. rate 60 Hz
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_MASK							(3<<6)
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_DISABLE							(0<<6)	//!< FIFO timestamp decimation disable
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_1								(1<<6)	//!< Timesptamp decimation 1
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_8								(2<<6)	//!< Timesptamp decimation 8
#define LSM6DSV_FIFO_CTRL4_DEC_TS_BATCH_32								(3<<6)	//!< Timesptamp decimation 32

#define LSM6DSV_COUNTER_BDR1_REG				0xB
#define LSM6DSV_COUNTER_BDR1_CNT_BDR_TH_9_8_MASK						(3<<0)	//!< Counter BDR bit 9:8
#define LSM6DSV_COUNTER_BDR1_TRIG_COUNTER_BDR_MASK						(3<<5)	//!< Selects the trigger for the internal counter of batch events between the accelerometer, gyroscope and EIS gyroscope
#define LSM6DSV_COUNTER_BDR1_TRIG_COUNTER_BDR_ACC_EVT					(0<<5)	//!< Accel batch event
#define LSM6DSV_COUNTER_BDR1_TRIG_COUNTER_BDR_GYR_EVT					(1<<5)	//!< Gyro batch event
#define LSM6DSV_COUNTER_BDR1_TRIG_COUNTER_BDR_GYR_EIS_EVT				(2<<5)	//!< Gyro EIS batch event

// Counter batch data rate register 2
#define LSM6DSV_COUNTER_BDR2_REG				0xC

#define LSM6DSV_INT1_CTRL_REG					0xD
#define LSM6DSV_INT1_CTRL_DRDY_XL_EN									(1<<0)	//!< Enable accel data ready interrupt 1
#define LSM6DSV_INT1_CTRL_DRDY_G_EN										(1<<1)	//!< Enable gyro data ready interrupt 1
#define LSM6DSV_INT1_CTRL_FIFO_TH_EN									(1<<3)	//!< Enable FIFO threshold interrupt 1
#define LSM6DSV_INT1_CTRL_FIFO_OVR_EN									(1<<4)	//!< Enable FIFO overrun interrupt 1
#define LSM6DSV_INT1_CTRL_FIFO_FULL_EN									(1<<5)	//!< Enable FIFO full interrupt 1
#define LSM6DSV_INT1_CTRL_CNT_BDR_EN									(1<<6)	//!< Enable COUNTER_BDR_IA interrupt 1

#define LSM6DSV_INT2_CTRL_REG					0xE
#define LSM6DSV_INT2_CTRL_DRDY_XL_EN									(1<<0)	//!< Enable accel data ready interrupt 2
#define LSM6DSV_INT2_CTRL_DRDY_G_EN										(1<<1)	//!< Enable gyro data ready interrupt 2
#define LSM6DSV_INT2_CTRL_DRDY_G_EIS									(1<<2)	//!< Enable gyro EIS data ready interrupt 2
#define LSM6DSV_INT1_CTRL_FIFO_TH_EN									(1<<3)	//!< Enable FIFO threshold interrupt 2
#define LSM6DSV_INT1_CTRL_FIFO_OVR_EN									(1<<4)	//!< Enable FIFO overrun interrupt 2
#define LSM6DSV_INT1_CTRL_FIFO_FULL_EN									(1<<5)	//!< Enable FIFO full interrupt 2
#define LSM6DSV_INT1_CTRL_CNT_BDR_EN									(1<<6)	//!< Enable COUNTER_BDR_IA interrupt 2
#define LSM6DSV_INT2_CTRL_EMB_FUNC_ENDOP_EN								(1<<7)	//!< Enable routing the embedded function end of operation interrupt 2

#define LSM6DSV_WHO_AM_I_REG					0xF
#define LSM6DSV_WHO_AM_I_VALUE											0x70

#define LSM6DSV_CTRL1_REG						0x10
#define LSM6DSV_CTRL1_ODR_XL_MASK										(0xF<<0)
#define LSM6DSV_CTRL1_ODR_XL_POWER_DOWN									(0<<0)	//!< Accel power down
#define LSM6DSV_CTRL1_ODR_XL_1_875HZ									(1<<0)	//!< Accel ODR 1.875 Hz
#define LSM6DSV_CTRL1_ODR_XL_7_5Hz										(2<<0)	//!< Accel ODR 7.5 Hz
#define LSM6DSV_CTRL1_ODR_XL_15Hz										(3<<0)	//!< Accel ODR 15 Hz
#define LSM6DSV_CTRL1_ODR_XL_30Hz										(4<<0)	//!< Accel ODR 30 Hz
#define LSM6DSV_CTRL1_ODR_XL_60Hz										(5<<0)	//!< Accel ODR 60 Hz
#define LSM6DSV_CTRL1_ODR_XL_1205Hz										(6<<0)	//!< Accel ODR 120 Hz
#define LSM6DSV_CTRL1_ODR_XL_240Hz										(7<<0)	//!< Accel ODR 240 Hz
#define LSM6DSV_CTRL1_ODR_XL_480Hz										(8<<0)	//!< Accel ODR 480 Hz
#define LSM6DSV_CTRL1_ODR_XL_960Hz										(9<<0)	//!< Accel ODR 960 Hz
#define LSM6DSV_CTRL1_ODR_XL_1920Hz										(10<<0)	//!< Accel ODR 1920 Hz
#define LSM6DSV_CTRL1_ODR_XL_3840Hz										(11<<0)	//!< Accel ODR 3840 Hz
#define LSM6DSV_CTRL1_ODR_XL_7680Hz										(12<<0)	//!< Accel ODR 7680 Hz
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_MASK							(7<<4)
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_HIGH_PERF						(0<<4)	//!< Accel high perf.
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_HIGH_ACCUR_ODR					(1<<4)	//!< Accel high accuracy ODR
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_ODR_TRIG						(3<<4)	//!< Accel ODR triggered
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_LOWPWR_MEAN2					(4<<4)	//!< Low [pwer mode (mean 2)
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_LOWPWR_MEAN4					(5<<4)	//!< Low [pwer mode (mean 4)
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_LOWPWR_MEAN8					(6<<4)	//!< Low [pwer mode (mean 8)
#define LSM6DSV_CTRL1_ODR_ODR_OP_MODE_XL_NORMAL							(7<<4)	//!< Normal

#define LSM6DSV_CTRL2_REG						0x11
#define LSM6DSV_CTRL2_ODR_G_MASK										(0xf<<0)
#define LSM6DSV_CTRL2_ODR_G_POWER_DOWN									(0<<0)
#define LSM6DSV_CTRL2_ODR_G_7_5HZ										(2<<0)	//!< Gyro ODR 7.5Hz
#define LSM6DSV_CTRL2_ODR_G_15HZ										(3<<0)	//!< Gyro ODR 15Hz
#define LSM6DSV_CTRL2_ODR_G_30HZ										(4<<0)	//!< Gyro ODR 30Hz
#define LSM6DSV_CTRL2_ODR_G_60HZ										(5<<0)	//!< Gyro ODR 60Hz
#define LSM6DSV_CTRL2_ODR_G_120HZ										(6<<0)	//!< Gyro ODR 120Hz
#define LSM6DSV_CTRL2_ODR_G_240HZ										(7<<0)	//!< Gyro ODR 240Hz
#define LSM6DSV_CTRL2_ODR_G_480HZ										(8<<0)	//!< Gyro ODR 480Hz
#define LSM6DSV_CTRL2_ODR_G_960HZ										(9<<0)	//!< Gyro ODR 960Hz
#define LSM6DSV_CTRL2_ODR_G_1920HZ										(10<<0)	//!< Gyro ODR 1920Hz
#define LSM6DSV_CTRL2_ODR_G_3840HZ										(11<<0)	//!< Gyro ODR 3840Hz
#define LSM6DSV_CTRL2_ODR_G_7680HZ										(12<<0)	//!< Gyro ODR 7680Hz
#define LSM6DSV_CTRL2_OP_MODE_G_MASK									(7<<4)
#define LSM6DSV_CTRL2_OP_MODE_G_HIGH_PERF								(0<<4)	//!< Gyro high performance
#define LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCUR								(1<<4)	//!< Gyro high accuracy
#define LSM6DSV_CTRL2_OP_MODE_G_ODR_TRIG								(3<<4)	//!< Gyro ODR triggered
#define LSM6DSV_CTRL2_OP_MODE_G_SLEEP									(4<<4)	//!< Gyro sleep
#define LSM6DSV_CTRL2_OP_MODE_G_LOW_POWER								(5<<4)	//!< Gyro low power

#define LSM6DSV_CTRL3_REG						0x12
#define LSM6DSV_CTRL3_SW_RESET											(1<<0)	//!< Soft reset
#define LSM6DSV_CTRL3_IF_INC_EN											(1<<2)	//!< Register address automatically incremented during a multiple byte access with a serial interface (I2C, MIPI I3C, or SPI)
#define LSM6DSV_CTRL3_BDU												(1<<6)	//!< Blocking data update
#define LSM6DSV_CTRL3_BOOT												(1<<7)	//!< Reboot

#define LSM6DSV_CTRL4_REG						0x13
#define LSM6DSV_CTRL4_INT2_IN_H											(1<<0)	//!< Trigger for DEN and embedded func input INT2 pin active high
#define LSM6DSV_CTRL4_DRDY_PULSE										(1<<1)	//!< Data ready 65 us pulse mode
#define LSM6DSV_CTRL4_INT2_DRDY_TEMP_EN									(1<<2)	//!< Data ready temperature interrupt on INT2
#define LSM6DSV_CTRL4_DRDY_MASK											(1<<3)	//!< Mask data ready signals
#define LSM6DSV_CTRL4_INT2_ON_INT1										(1<<4)	//!< Route embedded function interrupt to INT1

#define LSM6DSV_CTRL5_REG						0x14
#define LSM6DSV_CTRL5_INT_EN_I3C										(1<<0)	//!< Enable INT pin when I3C is enable
#define LSM6DSV_CTRL5_BUS_ACT_SEL_MASK									(3<<1)
#define LSM6DSV_CTRL5_BUS_ACT_SEL_2US									(0<<1)	//!< 2 us
#define LSM6DSV_CTRL5_BUS_ACT_SEL_50US									(1<<1)	//!< 50 us
#define LSM6DSV_CTRL5_BUS_ACT_SEL_1MS									(2<<1)	//!< 1 ms
#define LSM6DSV_CTRL5_BUS_ACT_SEL_25MS									(3<<1)	//!< 25 ms

#define LSM6DSV_CTRL6_REG						0x15
#define LSM6DSV_CTRL6_FS_G_MASK											(0xF<<0)
#define LSM6DSV_CTRL6_FS_G_125DPS										(0<<0)	//!< Full scale +-125 dps
#define LSM6DSV_CTRL6_FS_G_250DPS										(1<<0)	//!< Full scale +-250 dps
#define LSM6DSV_CTRL6_FS_G_500DPS										(2<<0)	//!< Full scale +-500 dps
#define LSM6DSV_CTRL6_FS_G_1000DPS										(3<<0)	//!< Full scale +-1000 dps
#define LSM6DSV_CTRL6_FS_G_2000DPS										(4<<0)	//!< Full scale +-2000 dps
#define LSM6DSV_CTRL6_FS_G_4000DPS										(0xC<<0)	//!< Full scale +-4000 dps
#define LSM6DSV_CTRL6_LPF1_G_MASK										(7<<4)

#define LSM6DSV_CTRL7_REG						0x16
#define LSM6DSV_CTRL7_LPF1_G_EN											(1<<0)	//!< Enable gyro digital LPF1 filter

#define LSM6DSV_CTRL8_REG						0x17
#define LSM6DSV_CTRL8_FS_XL_MASK										(3<<0)
#define LSM6DSV_CTRL8_FS_XL_2G											(0<<0)	//!< Accel full scale 2g
#define LSM6DSV_CTRL8_FS_XL_4G											(1<<0)	//!< Accel full scale 4g
#define LSM6DSV_CTRL8_FS_XL_8G											(2<<0)	//!< Accel full scale 8g
#define LSM6DSV_CTRL8_FS_XL_16G											(3<<0)	//!< Accel full scale 16g
#define LSM6DSV_CTRL8_XL_DUALC_EN										(1<<3)	//!< Enable dual-chan
#define LSM6DSV_CTRL8_HP_LPF2_XL_BW_MASK								(7<<5)

#define LSM6DSV_CTRL9_REG						0x18
#define LSM6DSV_CTRL9_USR_OFF_ON_OUT									(1<<0)	//!< Enable accel user offset correction block
#define LSM6DSV_CTRL9_USR_OFF_W											(1<<1)	//!< Weight of accel user offset bits
#define LSM6DSV_CTRL9_LPF2_XL_EN										(1<<3)	//!< Enable accel LPF2 filter
#define LSM6DSV_CTRL9_HP_SLOPE_XL_HIGH_PASS								(1<<4)	//!< accel filter high pass
#define LSM6DSV_CTRL9_XL_FASTSETTL_MODE_EN								(1<<5)	//!< Enable accel LPF2 and HPF fast settling mode
#define LSM6DSV_CTRL9_HP_REF_MODE_XL_EN									(1<<6)	//!< Enable accel high-pass filter reference mode

#define LSM6DSV_CTRL10_REG						0x19
#define LSM6DSV_CTRL10_ST_XL_MASK										(3<<0)
#define LSM6DSV_CTRL10_ST_XL_NORMAL										(0<<0)	//!< Accel normal mode
#define LSM6DSV_CTRL10_ST_XL_POS_SIGN_SELFTEST							(1<<0)	//!< Accel positive sign selftest
#define LSM6DSV_CTRL10_ST_XL_NEG_SIGN_SELFTEST							(2<<0)	//!< Accel negative sign selftest
#define LSM6DSV_CTRL10_ST_G_MASK										(3<<2)
#define LSM6DSV_CTRL10_ST_G_NORMAL										(0<<0)	//!< Gyro normal mode
#define LSM6DSV_CTRL10_ST_G_POS_SIGN_SELFTEST							(1<<0)	//!< Gyro positive sign selftest
#define LSM6DSV_CTRL10_ST_G_NEG_SIGN_SELFTEST							(2<<0)	//!< Gyro negative sign selftest
#define LSM6DSV_CTRL10_EMB_FUNC_DEBUG_EN								(1<<6)	//!< Enable debug mode for the embedded functions

#define LSM6DSV_CTRL_STATUS_REG					0x1A
#define LSM6DSV_CTRL_STATUS_FSM_WR_CTRL_STATUS_READ_ONLY				(1<<2)

#define LSM6DSV_FIFO_STATUS1_REG				0x1B

#define LSM6DSV_FIFO_STATUS2_REG				0x1C
#define LSM6DSV_FIFO_STATUS2_DIFF_FIFO_8								(1<<0)	//!< Number of unread sensor data (TAG + 6 bytes) stored in FIFO.
#define LSM6DSV_FIFO_STATUS2_OVR_LATCHED								(1<<3)	//!< Latched FIFO overrun status
#define LSM6DSV_FIFO_STATUS2_COUNTER_BDR_IA								(1<<4)	//!< Counter BDR reaches the CNT_BDR_TH_[10:0] threshold set in COUNTER_BDR_REG1 (0Bh) and COUNTER_BDR_REG2 (0Ch)
#define LSM6DSV_FIFO_STATUS2_FULL_IA									(1<<5)	//!< Smart FIFO full status
#define LSM6DSV_FIFO_STATUS2_OVR_IA										(1<<6)	//!< FIFO overrun status
#define LSM6DSV_FIFO_STATUS2_WTM_IA										(1<<7)	//!< FIFO watermark status

#define LSM6DSV_ALL_INT_SRC_REG						0x1D
#define LSM6DSV_ALL_INT_SRC_FF_IA										(1<<0)	//!< Free-fall event status
#define LSM6DSV_ALL_INT_SRC_WU_IA										(1<<1)	//!< Wake-up event status
#define LSM6DSV_ALL_INT_SRC_TAP_IA										(1<<2)	//!< Single or double-tap event detection status depending on SINGLE_DOUBLE_TAP_bit value (see WAKE_UP_THS (5Bh) register)
#define LSM6DSV_ALL_INT_SRC_D6D_IA										(1<<4)	//!< Interrupt active for change in position of portrait, landscape, face-up, face-down
#define LSM6DSV_ALL_INT_SRC_SLEEP_CHANGE_IA								(1<<5)	//!< Detects change event in activity/inactivity status
#define LSM6DSV_ALL_INT_SRC_SHUB_IA										(1<<6)	//!< Sensor hub (I2C master) interrupt status
#define LSM6DSV_ALL_INT_SRC_EMB_FUNC_IA									(1<<7)	//!< Embedded functions interrupt status

#define LSM6DSV_STATUS_REG							0x1E
#define LSM6DSV_STATUS_XLDA												(1<<0)	//!< Accelerometer new data available
#define LSM6DSV_STATUS_GDA												(1<<1)	//!< Gyroscope new data available
#define LSM6DSV_STATUS_TDA												(1<<2)	//!< Temperature new data available
#define LSM6DSV_STATUS_GDA_EIS											(1<<4)	//!< Enhanced EIS gyroscope new data available
#define LSM6DSV_STATUS_OIS_DRDY											(1<<5)	//!< Accelerometer OIS or gyroscope OIS new output data available
#define LSM6DSV_STATUS_TIMESTAMP_ENDCOUNT								(1<<7)	//!< Alerts timestamp overflow within 5.6 ms

#define LSM6DSV_OUT_TEMP_L_REG						0x20
#define LSM6DSV_OUT_TEMP_H_REG						0x21

#define LSM6DSV_OUTX_L_G_REG						0x22
#define LSM6DSV_OUTX_H_G_REG						0x23
#define LSM6DSV_OUTY_L_G_REG						0x24
#define LSM6DSV_OUTY_H_G_REG						0x25
#define LSM6DSV_OUTZ_L_G_REG						0x26
#define LSM6DSV_OUTZ_H_G_REG						0x27

#define LSM6DSV_OUTX_L_A_REG						0x28
#define LSM6DSV_OUTX_H_A_REG						0x29
#define LSM6DSV_OUTY_L_A_REG						0x2A
#define LSM6DSV_OUTY_H_A_REG						0x2B
#define LSM6DSV_OUTZ_L_A_REG						0x2C
#define LSM6DSV_OUTZ_H_A_REG						0x2D

#define LSM6DSV_UI_OUTX_L_G_OIS_EIS_REG				0x2E
#define LSM6DSV_UI_OUTX_H_G_OIS_EIS_REG				0x2F
#define LSM6DSV_UI_OUTY_L_G_OIS_EIS_REG				0x30
#define LSM6DSV_UI_OUTY_H_G_OIS_EIS_REG				0x31
#define LSM6DSV_UI_OUTZ_L_G_OIS_EIS_REG				0x32
#define LSM6DSV_UI_OUTZ_H_G_OIS_EIS_REG				0x33

#define LSM6DSV_UI_OUTX_L_A_OIS_DUALC_REG			0x34
#define LSM6DSV_UI_OUTX_H_A_OIS_DUALC_REG			0x35
#define LSM6DSV_UI_OUTY_L_A_OIS_DUALC_REG			0x36
#define LSM6DSV_UI_OUTY_H_A_OIS_DUALC_REG			0x37
#define LSM6DSV_UI_OUTZ_L_A_OIS_DUALC_REG			0x38
#define LSM6DSV_UI_OUTZ_H_A_OIS_DUALC_REG			0x39

#define LSM6DSV_TIMESTAMP0_REG						0x40
#define LSM6DSV_TIMESTAMP1_REG						0x41
#define LSM6DSV_TIMESTAMP2_REG						0x42
#define LSM6DSV_TIMESTAMP3_REG						0x43

#define LSM6DSV_UI_STATUS_OIS_REG					0x44
#define LSM6DSV_UI_STATUS_OIS_XLDA										(1<<0)	//!< Accelerometer OIS data available (reset when one of the high parts of the output data is read)
#define LSM6DSV_UI_STATUS_OIS_GDA										(1<<1)	//!< Gyroscope OIS data available (reset when one of the high parts of the output data is read)
#define LSM6DSV_UI_STATUS_OIS_GYRO_SETTLING								(1<<2)	//!< High when the gyroscope output is in the settling phase

#define LSM6DSV_WAKE_UP_SRC_REG						0X45
#define LSM6DSV_WAKE_UP_SRC_Z_WU										(1<<0)	//!< Wake-up event detection status on Z-axis.
#define LSM6DSV_WAKE_UP_SRC_Y_WU										(1<<1)	//!< Wake-up event detection status on Y-axis
#define LSM6DSV_WAKE_UP_SRC_X_WU										(1<<2)	//!< Wake-up event detection status on X-axis
#define LSM6DSV_WAKE_UP_SRC_WU_IA										(1<<3)	//!< Wake-up event detection status.
#define LSM6DSV_WAKE_UP_SRC_SLEEP_STATE									(1<<4)	//!< Sleep status bit
#define LSM6DSV_WAKE_UP_SRC_FF_IA										(1<<5)	//!< Free-fall event detection status
#define LSM6DSV_WAKE_UP_SRC_SLEEP_CHANGE_IA								(1<<6)	//!< Detects change event in activity/inactivity status.

#define LSM6DSV_TAP_SRC_REG							0X46
#define LSM6DSV_TAP_SRC_Z_TAP											(1<<0)	//!< Tap event detection status on Z-axis
#define LSM6DSV_TAP_SRC_Y_TAP											(1<<1)	//!< Tap event detection status on Y-axis
#define LSM6DSV_TAP_SRC_X_TAP											(1<<2)	//!< Tap event detection status on X-axis
#define LSM6DSV_TAP_SRC_SIGN											(1<<3)	//!< Sign of acceleration detected by tap event
#define LSM6DSV_TAP_SRC_DOUBLE_TAP										(1<<4)	//!< Double-tap event detection status
#define LSM6DSV_TAP_SRC_SINGLE_TAP										(1<<5)	//!< Double-tap event detection status
#define LSM6DSV_TAP_SRC_TAP_IA											(1<<6)	//!< Tap event detection status

#define LSM6DSV_D6D_SRC_REG							0x47	// Read only
#define LSM6DSV_D6D_SRC_XL												(1<<0)
#define LSM6DSV_D6D_SRC_XH												(1<<1)
#define LSM6DSV_D6D_SRC_YL												(1<<2)
#define LSM6DSV_D6D_SRC_YH												(1<<3)
#define LSM6DSV_D6D_SRC_ZL												(1<<4)
#define LSM6DSV_D6D_SRC_ZH												(1<<5)
#define LSM6DSV_D6D_SRC_D6D_IA											(1<<6)

#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_REG		0x49	// Read only
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_STEP_DET					(1<<3)
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_TILT						(1<<4)
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_SIGMOT						(1<<5)
#define LSM6DSV_EMB_FUNC_STATUS_MAINPAGE_IS_FSM_LC						(1<<7)

#define LSM6DSV_FSM_STATUS_MAINPAGE_REG				0x4A	// Read only
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM1								(1<<0)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM2								(1<<1)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM3								(1<<2)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM4								(1<<3)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM5								(1<<4)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM6								(1<<5)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM7								(1<<6)
#define LSM6DSV_FSM_STATUS_MAINPAGE_IS_FSM8								(1<<7)

#define LSM6DSV_INTERNAL_FREQ_FINE_REG				0x4F	// Read only

#define LSM6DSV_FUNCTIONS_ENABLE_REG				0x50
#define LSM6DSV_FUNCTIONS_ENABLE_INACT_EN_MASK							(3<<0)
#define LSM6DSV_FUNCTIONS_ENABLE_INACT_EN_ACC_LP1						(1<<3)
#define LSM6DSV_FUNCTIONS_ENABLE_INACT_EN_ACC_LP1_GYRO_SLEEP			(2<<0)
#define LSM6DSV_FUNCTIONS_ENABLE_INACT_EN_ACC_LP1_GYRO_OFF				(3<<0)
#define LSM6DSV_FUNCTIONS_ENABLE_DIS_RST_LIR_ALL_INT					(1<<3)
#define LSM6DSV_FUNCTIONS_ENABLE_TIMESTAMP_EN							(1<<6)
#define LSM6DSV_FUNCTIONS_ENABLE_INTERRUPTS_ENABLE						(1<<7)

#define LSM6DSV_DEN_REG								0x51
#define LSM6DSV_DEN_DEN_XL_G											(1<<0)	//!< DEN pin info stamped in the accelerometer axis selected by bits DEN_X, DEN_Y, DEN_Z
#define LSM6DSV_DEN_DEN_Z												(1<<1)	//!< DEN stored in Z-axis LSB
#define LSM6DSV_DEN_DEN_Y												(1<<2)
#define LSM6DSV_DEN_DEN_X												(1<<3)
#define LSM6DSV_DEN_DEN_XL_EN											(1<<4)
#define LSM6DSV_DEN_LVL2_EN_MASK										(3<<6)
#define LSM6DSV_DEN_LVL2_EN_TRIGGER_MODE								(2<<6)
#define LSM6DSV_DEN_LVL2_EN_LATCH_MODE									(3<<6)

#define LSM6DSV_INACTIVITY_DUR_REG					0x54
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_MASK							(3<<0)
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_1ST							(0<<0)
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_2ND							(1<<0)
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_3RD							(2<<0)
#define LSM6DSV_INACTIVITY_DUR_INACT_DUR_4TH							(3<<0)
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_MASK						(3<<2)
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_1_875HZ						(0<<2)	//!< 1.875Hz
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_15HZ						(1<<2)	//!< 15Hz
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_30HZ						(2<<2)	//!< 30Hz
#define LSM6DSV_INACTIVITY_DUR_XL_INACT_ODR_60HZ						(3<<2)	//!< 60Hz
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_MASK						(7<<4)
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_7_8125MG					(0<<4)	//!< 7.8125 mg/LSB
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_15_625MG					(1<<4)	//!< 15.625 mg/LSB
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_31_25MG					(2<<4)	//!< 31.25 mg/LSB
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_62_5MG					(3<<4)	//!< 62.5 mg/LSB
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_125MG						(4<<4)	//!< 125 mg/LSB
#define LSM6DSV_INACTIVITY_DUR_WU_INACT_THS_W_250MG						(5<<4)	//!< 250 mg/LSB
#define LSM6DSV_INACTIVITY_DUR_SLEEP_STATUS_ON_INT						(1<<7)

#define LSM6DSV_INACTIVITY_THS_REG					0x55
#define LSM6DSV_INACTIVITY_THS_MASK										(0x3F<<0)

#define LSM6DSV_TAP_CFG0_REG						0x56
#define LSM6DSV_TAP_CFG0_LIR											(1<<0)	//!< Latched interrupt
#define LSM6DSV_TAP_CFG0_TAP_Z_EN										(1<<1)	//!< Enables Z direction in tap recognition
#define LSM6DSV_TAP_CFG0_TAP_Y_EN										(1<<2)
#define LSM6DSV_TAP_CFG0_TAP_X_EN										(1<<3)
#define LSM6DSV_TAP_CFG0_SLOPE_FDS										(1<<4)
#define LSM6DSV_TAP_CFG0_HW_FUNC_MASK_XL_SETTL							(1<<5)
#define LSM6DSV_TAP_CFG0_LOW_PASS_ON_6D									(1<<6)

#define LSM6DSV_TAP_CFG1_REG						0x57
#define LSM6DSV_TAP_CFG1_THS_X_MASK										(0x1F<<0)
#define LSM6DSV_TAP_CFG1_PRIORITY_MASK									(7<<5)

#define LSM6DSV_TAP_CFG2_REG						0x58
#define LSM6DSV_TAP_CFG2_THS_Y_MASK										(0x1F<<0)

#define LSM6DSV_TAP_THS_6D_REG						0x59
#define LSM6DSV_TAP_THS_6D_THS_Z_MASK									(0x1F<<0)
#define LSM6DSV_TAP_THS_6D_SIXD_THS_MASK								(3<<5)
#define LSM6DSV_TAP_THS_6D_SIXD_THS_80D									(0<<5)	//!< 80 degrees
#define LSM6DSV_TAP_THS_6D_SIXD_THS_70D									(0<<5)	//!< 70 degrees
#define LSM6DSV_TAP_THS_6D_SIXD_THS_60D									(0<<5)	//!< 60 degrees
#define LSM6DSV_TAP_THS_6D_SIXD_THS_50D									(0<<5)	//!< 50 degrees
#define LSM6DSV_TAP_THS_6D_D4D_EN										(1<<7)

#define LSM6DSV_TAP_DUR_REG							0x5A
#define LSM6DSV_TAP_DUR_SHOCK_MASK										(3<<0)
#define LSM6DSV_TAP_DUR_QUIET_MASK										(3<<2)
#define LSM6DSV_TAP_DUR_DUR_MASK										(0xF<<4)

#define LSM6DSV_WAKE_UP_THS_REG						0x5B
#define LSM6DSV_WAKE_UP_THS_WK_THS_MASK									(0x3F<<0)
#define LSM6DSV_WAKE_UP_THS_USR_OFF_ON_WU								(1<<6)
#define LSM6DSV_WAKE_UP_THS_SINGLE_DOUBLE_TAP							(1<<7)

#define LSM6DSV_WAKE_UP_DUR_REG						0x5C
#define LSM6DSV_WAKE_UP_DUR_SLEEP_DUR_MASK								(0xF<<0)
#define LSM6DSV_WAKE_UP_DUR_WAKE_DUR_MASK								(3<<5)
#define LSM6DSV_WAKE_UP_DUR_FF_DUR_5									(1<<7)

#define LSM6DSV_FREE_FALL_REG						0x5D
#define LSM6DSV_FREE_FALL_FF_THS_MASK									(7<<0)
#define LSM6DSV_FREE_FALL_FF_THS_156MG									(0<<0)	//!< 156 mg
#define LSM6DSV_FREE_FALL_FF_THS_219MG									(1<<0)	//!< 219 mg
#define LSM6DSV_FREE_FALL_FF_THS_250MG									(2<<0)	//!< 250 mg
#define LSM6DSV_FREE_FALL_FF_THS_312MG									(3<<0)	//!< 312 mg
#define LSM6DSV_FREE_FALL_FF_THS_344MG									(4<<0)	//!< 344 mg
#define LSM6DSV_FREE_FALL_FF_THS_406MG									(5<<0)	//!< 406 mg
#define LSM6DSV_FREE_FALL_FF_THS_469MG									(6<<0)	//!< 469 mg
#define LSM6DSV_FREE_FALL_FF_THS_500MG									(7<<0)	//!< 500 mg
#define LSM6DSV_FREE_FALL_FF_DUR_MASK									(0x1F<<3)

#define LSM6DSV_MD1_CFG_REG							0x5E
#define LSM6DSV_MD1_CFG_INT1_SHUB										(1<<0)
#define LSM6DSV_MD1_CFG_INT1_EMB_FUNC									(1<<1)
#define LSM6DSV_MD1_CFG_INT1_6D											(1<<2)
#define LSM6DSV_MD1_CFG_INT1_DOUBLE_TAP									(1<<3)
#define LSM6DSV_MD1_CFG_INT1_FF											(1<<4)
#define LSM6DSV_MD1_CFG_INT1_WU											(1<<5)
#define LSM6DSV_MD1_CFG_INT1_SINGLE_TAP									(1<<6)
#define LSM6DSV_MD1_CFG_INT1_SLEEP_CHANGE								(1<<7)

#define LSM6DSV_MD2_CFG_REG							0x5F
#define LSM6DSV_MD2_CFG_INT2_TIMESTAMP									(1<<0)
#define LSM6DSV_MD2_CFG_INT2_EMB_FUNC									(1<<1)
#define LSM6DSV_MD2_CFG_INT2_6D											(1<<2)
#define LSM6DSV_MD2_CFG_INT2_DOUBLE_TAP									(1<<3)
#define LSM6DSV_MD2_CFG_INT2_FF											(1<<4)
#define LSM6DSV_MD2_CFG_INT2_WU											(1<<5)
#define LSM6DSV_MD2_CFG_INT2_SINGLE_TAP									(1<<6)
#define LSM6DSV_MD2_CFG_INT2_SLEEP_CHANGE								(1<<7)

#define LSM6DSV_HAODR_CFG_REG						0x62
#define LSM6DSV_HAODR_CFG_HAODR_SEL_MASK								(3<<0)

#define LSM6DSV_EMB_FUNC_CFG_REG					0x63
#define LSM6DSV_EMB_FUNC_CFG_EMB_FUNC_DISABLE							(1<<3)
#define LSM6DSV_EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_XL_SETTL					(1<<4)
#define LSM6DSV_EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_G_SETTL					(1<<5)
#define LSM6DSV_EMB_FUNC_CFG_XL_DualC_BATCH_FROM_IF						(1<<7)

#define LSM6DSV_UI_HANDSHAKE_CTRL_REG				0x64
#define LSM6DSV_UI_HANDSHAKE_CTRL_UI_SHARED_REQ							(1<<0)
#define LSM6DSV_UI_HANDSHAKE_CTRL_UI_SHARED_ACK							(1<<1)

#define LSM6DSV_UI_SPI2_SHARED_0_REG				0x65
#define LSM6DSV_UI_SPI2_SHARED_1_REG				0x66
#define LSM6DSV_UI_SPI2_SHARED_2_REG				0x67
#define LSM6DSV_UI_SPI2_SHARED_3_REG				0x68
#define LSM6DSV_UI_SPI2_SHARED_4_REG				0x69
#define LSM6DSV_UI_SPI2_SHARED_5_REG				0x6A

#define LSM6DSV_CTRL_EIS_REG						0x6B
#define LSM6DSV_CTRL_EIS_FS_G_EIS_MASK									(7<<0)
#define LSM6DSV_CTRL_EIS_FS_G_EIS_125DPS								(0<<0)	//!< +-125 dps
#define LSM6DSV_CTRL_EIS_FS_G_EIS_250DPS								(1<<0)	//!< +-250 dps
#define LSM6DSV_CTRL_EIS_FS_G_EIS_500DPS								(2<<0)	//!< +-500 dps
#define LSM6DSV_CTRL_EIS_FS_G_EIS_1000DPS								(3<<0)	//!< +-1000 dps
#define LSM6DSV_CTRL_EIS_FS_G_EIS_2000DPS								(4<<0)	//!< +-2000 dps
#define LSM6DSV_CTRL_EIS_G_EIS_ON_G_OIS_OUT_REG							(1<<3)
#define LSM6DSV_CTRL_EIS_LPF_G_EIS_BW									(1<<4)
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_MASK									(3<<6)
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_OFF									(0<<6)
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_1_92HZ								(1<<6)	//!< 1.92Hz
#define LSM6DSV_CTRL_EIS_ODR_G_EIS_960HZ								(2<<6)	//!< 960Hz

#define LSM6DSV_UI_INT_OIS_REG						0x6F
#define LSM6DSV_UI_INT_OIS_ST_OIS_CLAMPDIS								(1<<4)
#define LSM6DSV_UI_INT_OIS_DRDY_MASK_OIS								(1<<6)
#define LSM6DSV_UI_INT_OIS_INT2_DRDY_OIS								(1<<7)

#define LSM6DSV_UI_CTRL1_OIS_REG					0x70
#define LSM6DSV_UI_CTRL1_OIS_SPI2_READ_EN								(1<<0)
#define LSM6DSV_UI_CTRL1_OIS_OIS_G_EN									(1<<1)
#define LSM6DSV_UI_CTRL1_OIS_OIS_XL_EN									(1<<2)
#define LSM6DSV_UI_CTRL1_OIS_SIM_OIS_4WIRES								(0<<5)
#define LSM6DSV_UI_CTRL1_OIS_SIM_OIS_3WIRES								(1<<5)

#define LSM6DSV_UI_CTRL2_OIS_REG					0x71
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_MASK								(7<<0)
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_125DPS							(0<<0)	//!< +-125dps
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_250DPS							(1<<0)	//!< +-250dps
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_500DPS							(2<<0)	//!< +-500dps
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_1000DPS							(3<<0)	//!< +-1000dps
#define LSM6DSV_UI_CTRL2_OIS_FS_G_OIS_2000DPS							(4<<0)	//!< +-2000dps
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_MASK							(3<<3)
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_293HZ						(0<<3)	//!< 293Hz, -7.1 degree
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_217HZ						(1<<3)	//!< 217Hz, -9.1
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_158HZ						(2<<3)	//!< 158Hz, -11.9
#define LSM6DSV_UI_CTRL2_OIS_LPF1_G_OIS_BW_476HZ						(3<<3)	//!< 476Hz, -5.1

#define LSM6DSV_UI_CTRL3_OIS_REG					0x72
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_MASK								(3<<0)
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_2G								(0<<0)	//!< +-2g
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_4G								(1<<0)	//!< +-4g
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_8G								(2<<0)	//!< +-8g
#define LSM6DSV_UI_CTRL3_OIS_FS_XL_OIS_16G								(3<<0)	//!< +-16g
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_MASK							(7<<3)
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_749HZ						(0<<3)	//!< 749Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_539HZ						(1<<3)	//!< 539Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_342HZ						(2<<3)	//!< 342Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_162HZ						(3<<3)	//!< 162Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_78_5HZ						(4<<3)	//!< 78.5Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_38_6HZ						(5<<3)	//!< 38.6Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_19_3HZ						(6<<3)	//!< 19.3Hz
#define LSM6DSV_UI_CTRL3_OIS_LPF_XL_OIS_BW_9_8HZ						(7<<3)	//!< 9.8Hz

#define LSM6DSV_X_OFS_USR_REG						0x73
#define LSM6DSV_Y_OFS_USR_REG						0x74
#define LSM6DSV_Z_OFS_USR_REG						0x75

#define LSM6DSV_FIFO_DATA_OUT_TAG_REG				0x78
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_CNT_MASK							(3<<1)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_MASK						(0x1F<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_FIFO_EMPTY					(0<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_GYRO_NC					(1<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ACC_NC						(2<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_TEMP						(3<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_TIMESTAMP					(4<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_CFG_CHANGE					(5<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ACC_NC_T_2					(6<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ACC_NC_T_1					(7<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ACC_2XC					(8<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ACC_3XC					(9<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_GYRO_NC_T_2				(10<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_GYRO_NC_T_1				(11<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_GYRO_2XC					(12<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_GYRO_3XC					(13<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_HUB_SLAVE0					(14<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_HUB_SLAVE1					(15<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_HUB_SLAVE2					(16<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_HUB_SLAVE3					(17<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_STEP_COUNTER				(18<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_SFLP_GAME_ROT_VECT			(19<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_SFLP_GYRO_BIAS				(20<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_SFLP_GRAVITY_VECT			(21<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_HUB_NACK					(22<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ACC_DUALC					(23<<3)
#define LSM6DSV_FIFO_DATA_OUT_TAG_TAG_SENSOR_ENHANCED_EIS_GYRO			(24<<3)

#define LSM6DSV_FIFO_DATA_OUT_X_L_REG				0x79
#define LSM6DSV_FIFO_DATA_OUT_X_H_REG				0x7A
#define LSM6DSV_FIFO_DATA_OUT_Y_L_REG				0x7B
#define LSM6DSV_FIFO_DATA_OUT_Y_H_REG				0x7C
#define LSM6DSV_FIFO_DATA_OUT_Z_L_REG				0x7D
#define LSM6DSV_FIFO_DATA_OUT_Z_H_REG				0x7E


// SPI2 register mapping

#define LSM6DSV_SPI2_WHO_AM_I_REG					0xF
#define LSM6DSV_SPI2_WHO_AM_I_VAL										(0x70)

#define LSM6DSV_SPI2_STATUS_REG_OIS_REG				0x1E
#define LSM6DSV_SPI2_STATUS_REG_OIS_XLDA								(1<<0)
#define LSM6DSV_SPI2_STATUS_REG_OIS_GDA									(1<<1)
#define LSM6DSV_SPI2_STATUS_REG_OIS_GYRO_SETTLING						(1<<2)

#define LSM6DSV_SPI2_OUT_TEMP_L_REG					0x20
#define LSM6DSV_SPI2_OUT_TEMP_H_REG					0x21

#define LSM6DSV_SPI2_OUTX_L_G_OIS_REG				0x22
#define LSM6DSV_SPI2_OUTX_H_G_OIS_REG				0x23
#define LSM6DSV_SPI2_OUTY_L_G_OIS_REG				0x24
#define LSM6DSV_SPI2_OUTY_H_G_OIS_REG				0x25
#define LSM6DSV_SPI2_OUTZ_L_G_OIS_REG				0x26
#define LSM6DSV_SPI2_OUTZ_H_G_OIS_REG				0x27

#define LSM6DSV_SPI2_OUTX_L_A_OIS_REG				0x28
#define LSM6DSV_SPI2_OUTX_H_A_OIS_REG				0x29
#define LSM6DSV_SPI2_OUTY_L_A_OIS_REG				0x2A
#define LSM6DSV_SPI2_OUTY_H_A_OIS_REG				0x2B
#define LSM6DSV_SPI2_OUTZ_L_A_OIS_REG				0x2C
#define LSM6DSV_SPI2_OUTZ_H_A_OIS_REG				0x2D

#define LSM6DSV_SPI2_HANDSHAKE_CTRL_REG				0x6E
#define LSM6DSV_SPI2_HANDSHAKE_CTRL_SPI2_SHARED_ACK						(1<<0)
#define LSM6DSV_SPI2_HANDSHAKE_CTRL_SPI2_SHARED_REQ						(1<<1)

#define LSM6DSV_SPI2_INT_OIS_REG					0x6F
#define LSM6DSV_SPI2_INT_OIS_ST_XL_OIS_MASK								(3<<0)
#define LSM6DSV_SPI2_INT_OIS_ST_XL_OIS_NORMAL							(0<<0)
#define LSM6DSV_SPI2_INT_OIS_ST_XL_OIS_POS_SELFTEST						(1<<0)
#define LSM6DSV_SPI2_INT_OIS_ST_XL_OIS_NEG_SELFTEST						(3<<0)
#define LSM6DSV_SPI2_INT_OIS_ST_G_OIS_MASK								(3<<2)
#define LSM6DSV_SPI2_INT_OIS_ST_G_OIS_NORMAL							(0<<2)
#define LSM6DSV_SPI2_INT_OIS_ST_G_OIS_POS_SELFTEST						(1<<2)
#define LSM6DSV_SPI2_INT_OIS_ST_G_OIS_NEG_SELFTEST						(3<<2)
#define LSM6DSV_SPI2_INT_OIS_ST_OIS_CLAMPDIS							(1<<4)
#define LSM6DSV_SPI2_INT_OIS_DRDY_MASK_OIS								(1<<6)
#define LSM6DSV_SPI2_INT_OIS_INT2_DRDY_OIS								(1<<7)

#define LSM6DSV_SPI2_CTRL1_OIS_REG					0x70
#define LSM6DSV_SPI2_CTRL1_OIS_SPI2_READ_EN								(1<<0)
#define LSM6DSV_SPI2_CTRL1_OIS_OIS_G_EN									(1<<1)
#define LSM6DSV_SPI2_CTRL1_OIS_OIS_XL_EN								(1<<2)
#define LSM6DSV_SPI2_CTRL1_OIS_SIM_OIS_4WIRES							(0<<5)
#define LSM6DSV_SPI2_CTRL1_OIS_SIM_OIS_3WIRES							(1<<5)

#define LSM6DSV_SPI2_CTRL2_OIS_REG					0x71
#define LSM6DSV_SPI2_CTRL2_OIS_FS_G_OIS_MASK							(7<<0)
#define LSM6DSV_SPI2_CTRL2_OIS_FS_G_OIS_125DPS							(0<<0)	//!< +-125dps
#define LSM6DSV_SPI2_CTRL2_OIS_FS_G_OIS_250DPS							(1<<0)	//!< +-250dps
#define LSM6DSV_SPI2_CTRL2_OIS_FS_G_OIS_500DPS							(2<<0)	//!< +-500dps
#define LSM6DSV_SPI2_CTRL2_OIS_FS_G_OIS_1000DPS							(3<<0)	//!< +-1000dps
#define LSM6DSV_SPI2_CTRL2_OIS_FS_G_OIS_2000PS							(4<<0)	//!< +-2000dps
#define LSM6DSV_SPI2_CTRL2_OIS_LPF1_G_OIS_BW_MASK						(3<<3)
#define LSM6DSV_SPI2_CTRL2_OIS_LPF1_G_OIS_BW_293HZ						(0<<3)	//!< 293Hz, -7.1 degree
#define LSM6DSV_SPI2_CTRL2_OIS_LPF1_G_OIS_BW_217HZ						(1<<3)	//!< 217Hz, -9.1 degree
#define LSM6DSV_SPI2_CTRL2_OIS_LPF1_G_OIS_BW_158HZ						(2<<3)	//!< 158Hz, -11.9 degree
#define LSM6DSV_SPI2_CTRL2_OIS_LPF1_G_OIS_BW_476HZ						(3<<3)	//!< 476Hz, -5.1 degree

#define LSM6DSV_SPI2_CTRL3_OIS_REG					0x72
#define LSM6DSV_SPI2_CTRL3_OIS_FS_XL_OIS_MASK							(3<<0)
#define LSM6DSV_SPI2_CTRL3_OIS_FS_XL_OIS_2G								(0<<0)	//!< +-2g
#define LSM6DSV_SPI2_CTRL3_OIS_FS_XL_OIS_4G								(1<<0)	//!< +-4g
#define LSM6DSV_SPI2_CTRL3_OIS_FS_XL_OIS_8G								(2<<0)	//!< +-8g
#define LSM6DSV_SPI2_CTRL3_OIS_FS_XL_OIS_16G							(3<<0)	//!< +-16g
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_MASK						(7<<3)
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_749HZ						(0<<3)	//!< 749Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_539HZ						(1<<3)	//!< 539Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_342HZ						(2<<3)	//!< 342Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_162HZ						(3<<3)	//!< 162Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_78_5HZ						(4<<3)	//!< 78.5Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_38_6HZ						(5<<3)	//!< 38.6Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_19_3HZ						(6<<3)	//!< 19.3Hz
#define LSM6DSV_SPI2_CTRL3_OIS_LPF_XL_OIS_BW_9_8HZ						(7<<3)	//!< 9.8Hz

// Embedded functions register mapping

#define LSM6DSV_PAGE_SEL_REG						0x2
#define LSM6DSV_PAGE_SEL_MASK											(0xF<<4)
#define LSM6DSV_PAGE_SEL_POS											(4)

#define LSM6DSV_EMB_FUNC_EN_A_REG					0x4
#define LSM6DSV_EMB_FUNC_EN_A_SFLP_GAME_EN								(1<<1)
#define LSM6DSV_EMB_FUNC_EN_A_PEDO_EN									(1<<3)
#define LSM6DSV_EMB_FUNC_EN_A_TILT_EN									(1<<4)
#define LSM6DSV_EMB_FUNC_EN_A_SIGN_MOTION_EN							(1<<5)

#define LSM6DSV_EMB_FUNC_EN_B_REG					0x5
#define LSM6DSV_EMB_FUNC_EN_B_FSM_EN									(1<<0)	//!< Enable finite state machine
#define LSM6DSV_EMB_FUNC_EN_B_FIFO_COMPR_EN								(1<<3)
#define LSM6DSV_EMB_FUNC_EN_B_PEDO_ADV_EN								(1<<4)	//!< Enable pedometer advanced features

#define LSM6DSV_EMB_FUNC_EXEC_STATUS_REG			0x7
#define LSM6DSV_EMB_FUNC_EXEC_STATUS_EMB_FUNC_ENDOP						(1<<0)
#define LSM6DSV_EMB_FUNC_EXEC_STATUS_EMB_FUNC_EXEC_OVR					(1<<1)

#define LSM6DSV_PAGE_ADDRESS_REG					0x8

#define LSM6DSV_PAGE_VALUE_REG						0x9

#define LSM6DSV_EMB_FUNC_INT1_REG					0xA
#define LSM6DSV_EMB_FUNC_INT1_INT1_STEP_DETECTOR						(1<<3)
#define LSM6DSV_EMB_FUNC_INT1_INT1_TILT									(1<<4)
#define LSM6DSV_EMB_FUNC_INT1_INT1_SIG_MOT								(1<<5)
#define LSM6DSV_EMB_FUNC_INT1_INT1_FSM_LC								(1<<7)

#define LSM6DSV_FSM_INT1_REG						0xB

#define LSM6DSV_EMB_FUNC_INT2_REG					0xE
#define LSM6DSV_EMB_FUNC_INT2_INT2_STEP_DETECTOR						(1<<3)
#define LSM6DSV_EMB_FUNC_INT2_INT2_TILT									(1<<4)
#define LSM6DSV_EMB_FUNC_INT2_INT2_SIG_MOT								(1<<5)
#define LSM6DSV_EMB_FUNC_INT2_INT2_FSM_LC								(1<<7)

#define LSM6DSV_FSM_INT2_REG						0xF

#define LSM6DSV_EMB_FUNC_STATUS_REG					0x12
#define LSM6DSV_EMB_FUNC_STATUS_IS_STEP_DET								(1<<3)
#define LSM6DSV_EMB_FUNC_STATUS_IS_TILT									(1<<4)
#define LSM6DSV_EMB_FUNC_STATUS_IS_SIGMOT								(1<<5)
#define LSM6DSV_EMB_FUNC_STATUS_IS_FSM_LC								(1<<7)

#define LSM6DSV_FSM_STATUS_REG						0x13

#define LSM6DSV_PAGE_RW_REG							0x17
#define LSM6DSV_PAGE_RW_PAGE_READ										(1<<5)
#define LSM6DSV_PAGE_RW_PAGE_WRITE										(1<<6)
#define LSM6DSV_PAGE_RW_EMB_FUNC_LIR									(1<<7)

#define LSM6DSV_EMB_FUNC_FIFO_EN_A_REG				0x44
#define LSM6DSV_EMB_FUNC_FIFO_EN_A_SFLP_GAME_FIFO_EN					(1<<1)
#define LSM6DSV_EMB_FUNC_FIFO_EN_A_SFLP_GRAVITY_FIFO_EN					(1<<4)
#define LSM6DSV_EMB_FUNC_FIFO_EN_A_SFLP_GBIAS_FIFO_EN					(1<<5)
#define LSM6DSV_EMB_FUNC_FIFO_EN_A_STEP_COUNTER_FIFO_EN					(1<<6)

#define LSM6DSV_FSM_ENABLE_REG						0x46

#define LSM6DSV_FSM_LONG_COUNTER_L_REG				0x48
#define LSM6DSV_FSM_LONG_COUNTER_H_REG				0x49

#define LSM6DSV_INT_ACK_MASK_REG					0x4B

#define LSM6DSV_FSM_OUTS1_REG						0x4C
#define LSM6DSV_FSM_OUTS1_NV											(1<<0)
#define LSM6DSV_FSM_OUTS1_PV											(1<<1)
#define LSM6DSV_FSM_OUTS1_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS1_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS1_NY											(1<<4)
#define LSM6DSV_FSM_OUTS1_PY											(1<<5)
#define LSM6DSV_FSM_OUTS1_NX											(1<<6)
#define LSM6DSV_FSM_OUTS1_PX											(1<<7)

#define LSM6DSV_FSM_OUTS2_REG						0x4D
#define LSM6DSV_FSM_OUTS2_NV											(1<<0)
#define LSM6DSV_FSM_OUTS2_PV											(1<<1)
#define LSM6DSV_FSM_OUTS2_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS2_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS2_NY											(1<<4)
#define LSM6DSV_FSM_OUTS2_PY											(1<<5)
#define LSM6DSV_FSM_OUTS2_NX											(1<<6)
#define LSM6DSV_FSM_OUTS2_PX											(1<<7)

#define LSM6DSV_FSM_OUTS3_REG						0x4CE
#define LSM6DSV_FSM_OUTS3_NV											(1<<0)
#define LSM6DSV_FSM_OUTS3_PV											(1<<1)
#define LSM6DSV_FSM_OUTS3_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS3_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS3_NY											(1<<4)
#define LSM6DSV_FSM_OUTS3_PY											(1<<5)
#define LSM6DSV_FSM_OUTS3_NX											(1<<6)
#define LSM6DSV_FSM_OUTS3_PX											(1<<7)

#define LSM6DSV_FSM_OUTS4_REG						0x4F
#define LSM6DSV_FSM_OUTS4_NV											(1<<0)
#define LSM6DSV_FSM_OUTS4_PV											(1<<1)
#define LSM6DSV_FSM_OUTS4_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS4_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS4_NY											(1<<4)
#define LSM6DSV_FSM_OUTS4_PY											(1<<5)
#define LSM6DSV_FSM_OUTS4_NX											(1<<6)
#define LSM6DSV_FSM_OUTS4_PX											(1<<7)

#define LSM6DSV_FSM_OUTS5_REG						0x50
#define LSM6DSV_FSM_OUTS5_NV											(1<<0)
#define LSM6DSV_FSM_OUTS5_PV											(1<<1)
#define LSM6DSV_FSM_OUTS5_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS5_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS5_NY											(1<<4)
#define LSM6DSV_FSM_OUTS5_PY											(1<<5)
#define LSM6DSV_FSM_OUTS5_NX											(1<<6)
#define LSM6DSV_FSM_OUTS5_PX											(1<<7)

#define LSM6DSV_FSM_OUTS6_REG						0x51
#define LSM6DSV_FSM_OUTS6_NV											(1<<0)
#define LSM6DSV_FSM_OUTS6_PV											(1<<1)
#define LSM6DSV_FSM_OUTS6_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS6_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS6_NY											(1<<4)
#define LSM6DSV_FSM_OUTS6_PY											(1<<5)
#define LSM6DSV_FSM_OUTS6_NX											(1<<6)
#define LSM6DSV_FSM_OUTS6_PX											(1<<7)

#define LSM6DSV_FSM_OUTS7_REG						0x52
#define LSM6DSV_FSM_OUTS7_NV											(1<<0)
#define LSM6DSV_FSM_OUTS7_PV											(1<<1)
#define LSM6DSV_FSM_OUTS7_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS7_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS7_NY											(1<<4)
#define LSM6DSV_FSM_OUTS7_PY											(1<<5)
#define LSM6DSV_FSM_OUTS7_NX											(1<<6)
#define LSM6DSV_FSM_OUTS7_PX											(1<<7)

#define LSM6DSV_FSM_OUTS8_REG						0x53
#define LSM6DSV_FSM_OUTS8_NV											(1<<0)
#define LSM6DSV_FSM_OUTS8_PV											(1<<1)
#define LSM6DSV_FSM_OUTS8_NZ											(1<<2)
#define LSM6DSV_FSM_OUTS8_PZ											(1<<3)
#define LSM6DSV_FSM_OUTS8_NY											(1<<4)
#define LSM6DSV_FSM_OUTS8_PY											(1<<5)
#define LSM6DSV_FSM_OUTS8_NX											(1<<6)
#define LSM6DSV_FSM_OUTS8_PX											(1<<7)

#define LSM6DSV_SFLP_ODR_REG						0x5E
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_MASK								(7<<3)
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_15HZ								(0<<3)	//!< 15Hz
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_30HZ								(1<<3)	//!< 30Hz
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_60HZ								(2<<3)	//!< 60Hz
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_120HZ							(3<<3)	//!< 120Hz
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_240HZ							(4<<3)	//!< 240Hz
#define LSM6DSV_SFLP_ODR_SFLP_GAME_ODR_480HZ							(5<<3)	//!< 480Hz

#define LSM6DSV_FSM_ODR_REG							0x5F
#define LSM6DSV_FSM_ODR_FSM_ODR_MASK									(7<<3)
#define LSM6DSV_FSM_ODR_FSM_ODR_15HZ									(0<<3)	//!< 15Hz
#define LSM6DSV_FSM_ODR_FSM_ODR_30HZ									(1<<3)	//!< 30Hz
#define LSM6DSV_FSM_ODR_FSM_ODR_60HZ									(2<<3)	//!< 60Hz
#define LSM6DSV_FSM_ODR_FSM_ODR_120HZ									(3<<3)	//!< 120Hz
#define LSM6DSV_FSM_ODR_FSM_ODR_240HZ									(4<<3)	//!< 240Hz
#define LSM6DSV_FSM_ODR_FSM_ODR_480HZ									(5<<3)	//!< 480Hz
#define LSM6DSV_FSM_ODR_FSM_ODR_960HZ									(6<<3)	//!< 960Hz

#define LSM6DSV_STEP_COUNTER_L_REG					0x62
#define LSM6DSV_STEP_COUNTER_H_REG					0x63

#define LSM6DSV_EMB_FUNC_SRC_REG					0x64
#define LSM6DSV_EMB_FUNC_SRC_STEPCOUNTER_BIT_SET						(1<<2)
#define LSM6DSV_EMB_FUNC_SRC_STEP_OVERFLOW								(1<<3)
#define LSM6DSV_EMB_FUNC_SRC_STEP_COUNT_DELTA_IA						(1<<4)
#define LSM6DSV_EMB_FUNC_SRC_STEP_DETECTED								(1<<5)
#define LSM6DSV_EMB_FUNC_SRC_PEDO_RST_STEP								(1<<7)

#define LSM6DSV_EMB_FUNC_INIT_A_REG					0x66
#define LSM6DSV_EMB_FUNC_INIT_A_FSM_INIT								(1<<0)
#define LSM6DSV_EMB_FUNC_INIT_A_FIFO_COMPR_INIT							(1<<1)


// Embedded advanced features pages

// Page 0 - embedded advanced features registers

#define LSM6DSV_SFLP_GAME_GBIASX_L_REG				0x6E
#define LSM6DSV_SFLP_GAME_GBIASX_H_REG				0x6F
#define LSM6DSV_SFLP_GAME_GBIASY_L_REG				0x70
#define LSM6DSV_SFLP_GAME_GBIASY_H_REG				0x71
#define LSM6DSV_SFLP_GAME_GBIASZ_L_REG				0x72
#define LSM6DSV_SFLP_GAME_GBIASZ_H_REG				0x73

#define LSM6DSV_FSM_EXT_SENSITIVITY_L_REG			0xBA
#define LSM6DSV_FSM_EXT_SENSITIVITY_H_REG			0xBB

#define LSM6DSV_FSM_EXT_OFFX_L_REG					0xC0
#define LSM6DSV_FSM_EXT_OFFX_H_REG					0xC1
#define LSM6DSV_FSM_EXT_OFFY_L_REG					0xC2
#define LSM6DSV_FSM_EXT_OFFY_H_REG					0xC3
#define LSM6DSV_FSM_EXT_OFFZ_L_REG					0xC4
#define LSM6DSV_FSM_EXT_OFFZ_H_REG					0xC5

#define LSM6DSV_FSM_EXT_MATRIX_XX_L_REG				0xC6
#define LSM6DSV_FSM_EXT_MATRIX_XX_H_REG				0xC7
#define LSM6DSV_FSM_EXT_MATRIX_XY_L_REG				0xC8
#define LSM6DSV_FSM_EXT_MATRIX_XY_H_REG				0xC9
#define LSM6DSV_FSM_EXT_MATRIX_XZ_L_REG				0xCA
#define LSM6DSV_FSM_EXT_MATRIX_XZ_H_REG				0xCB

#define LSM6DSV_FSM_EXT_MATRIX_YY_L_REG				0xCC
#define LSM6DSV_FSM_EXT_MATRIX_YY_H_REG				0xCD
#define LSM6DSV_FSM_EXT_MATRIX_YZ_L_REG				0xCE
#define LSM6DSV_FSM_EXT_MATRIX_YZ_H_REG				0xCF

#define LSM6DSV_FSM_EXT_MATRIX_ZZ_L_REG				0xD0
#define LSM6DSV_FSM_EXT_MATRIX_ZZ_H_REG				0xD1

#define LSM6DSV_EXT_CFG_A_REG						0xD4
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_MASK								(7<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_Z_Y								(0<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_Z_NY								(1<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_Z_X								(2<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_Z_NX								(3<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_Z_Z								(4<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Z_AXIS_Z_NZ								(5<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_MASK								(7<<4)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_Y_Y								(0<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_Y_NY								(1<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_Y_X								(2<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_Y_NX								(3<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_Y_Z								(4<<0)
#define LSM6DSV_EXT_CFG_A_EXT_Y_AXIS_Y_NZ								(5<<0)

#define LSM6DSV_EXT_CFG_B_REG						0xD5
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_MASK								(7<<0)
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_X_Y								(0<<0)
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_X_NY								(1<<0)
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_X_X								(2<<0)
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_X_NX								(3<<0)
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_X_Z								(4<<0)
#define LSM6DSV_EXT_CFG_A_EXT_X_AXIS_X_NZ								(5<<0)


// Page 1 - embedded advanced features registers

#define LSM6DSV_FSM_LC_TIMEOUT_L_REG				0x7A
#define LSM6DSV_FSM_LC_TIMEOUT_H_REG				0x7B

#define LSM6DSV_FSM_PROGRAMS_REG					0x7C

#define LSM6DSV_FSM_START_ADD_L_REG					0x7E
#define LSM6DSV_FSM_START_ADD_H_REG					0x7F

#define LSM6DSV_PEDO_CMD_REG						0x83
#define LSM6DSV_PEDO_CMD_FP_REJECTION_EN								(1<<2)
#define LSM6DSV_PEDO_CMD_CARRY_COUNT_EN									(1<<3)

#define LSM6DSV_PEDO_DEB_STEPS_CONF_REG				0x84
#define LSM6DSV_PEDO_SC_DELTAT_L_REG				0xD0
#define LSM6DSV_PEDO_SC_DELTAT_H_REG				0xD1


// Page 2 - embedded advanced features registers

#define LSM6DSV_EXT_FORMAT_REG						0x00
#define LSM6DSV_EXT_FORMAT_2BYTES										(0<<2)
#define LSM6DSV_EXT_FORMAT_3BYTES										(1<<2)

#define LSM6DSV_EXT_3BYTE_SENSITIVITY_L_REG			0x2
#define LSM6DSV_EXT_3BYTE_SENSITIVITY_H_REG			0x3

#define LSM6DSV_EXT_3BYTE_OFFSET_XL_REG				0x6
#define LSM6DSV_EXT_3BYTE_OFFSET_L_REG				0x7
#define LSM6DSV_EXT_3BYTE_OFFSET_H_REG				0x8


// Sensor hub register mapping

#define LSM6DSV_SENSOR_HUB_1_REG					0x2
#define LSM6DSV_SENSOR_HUB_2_REG					0x3
#define LSM6DSV_SENSOR_HUB_3_REG					0x4
#define LSM6DSV_SENSOR_HUB_4_REG					0x5
#define LSM6DSV_SENSOR_HUB_5_REG					0x6
#define LSM6DSV_SENSOR_HUB_6_REG					0x7
#define LSM6DSV_SENSOR_HUB_7_REG					0x8
#define LSM6DSV_SENSOR_HUB_8_REG					0x9
#define LSM6DSV_SENSOR_HUB_9_REG					0xA
#define LSM6DSV_SENSOR_HUB_10_REG					0xB
#define LSM6DSV_SENSOR_HUB_11_REG					0xC
#define LSM6DSV_SENSOR_HUB_12_REG					0xD
#define LSM6DSV_SENSOR_HUB_13_REG					0xE
#define LSM6DSV_SENSOR_HUB_14_REG					0xF
#define LSM6DSV_SENSOR_HUB_15_REG					0x10
#define LSM6DSV_SENSOR_HUB_16_REG					0x11
#define LSM6DSV_SENSOR_HUB_17_REG					0x12
#define LSM6DSV_SENSOR_HUB_18_REG					0x13

#define LSM6DSV_MASTER_CONFIG_REG					0x14
#define LSM6DSV_MASTER_CONFIG_AUX_SENS_ON_MASK							(3<<0)
#define LSM6DSV_MASTER_CONFIG_AUX_SENS_ON_1								(0<<0)	//!< One sensor
#define LSM6DSV_MASTER_CONFIG_AUX_SENS_ON_2								(1<<0)	//!< Two sensors
#define LSM6DSV_MASTER_CONFIG_AUX_SENS_ON_3								(2<<0)	//!< Three sensors
#define LSM6DSV_MASTER_CONFIG_AUX_SENS_ON_4								(3<<0)	//!< Four sensors
#define LSM6DSV_MASTER_CONFIG_MASTER_ON									(1<<2)
#define LSM6DSV_MASTER_CONFIG_PASS_THROUGH_MODE							(1<<4)
#define LSM6DSV_MASTER_CONFIG_START_CONFIG								(1<<5)
#define LSM6DSV_MASTER_CONFIG_WRITE_ONCE								(1<<6)
#define LSM6DSV_MASTER_CONFIG_RST_MASTER_REGS							(1<<7)

#define LSM6DSV_SLV0_ADD_REG						0x15

#define LSM6DSV_SLV0_SUBADD_REG						0x16

#define LSM6DSV_SLV0_CONFIG_REG						0x17
#define LSM6DSV_SLV0_CONFIG_SLAVE0_READ_COUNT_MASK						(7<<0)
#define LSM6DSV_SLV0_CONFIG_BATCH_EXT_SENS_0_EN							(1<<3)
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_MASK								(7<<5)
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_1_875HZ							(0<<5)	//!< 1.875Hz
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_15HZ								(1<<5)	//!< 15Hz
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_30HZ								(2<<5)	//!< 30Hz
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_60HZ								(3<<5)	//!< 60Hz
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_120HZ								(4<<5)	//!< 120Hz
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_240HZ								(5<<5)	//!< 240Hz
#define LSM6DSV_SLV0_CONFIG_SHUB_ODR_480HZ								(6<<5)	//!< 480Hz

#define LSM6DSV_SLV1_ADD_REG						0x18

#define LSM6DSV_SLV1_SUBADD_REG						0x19

#define LSM6DSV_SLV1_CONFIG_REG						0x1A
#define LSM6DSV_SLV1_CONFIG_SLAVE1_READ_COUNT_MASK						(7<<0)
#define LSM6DSV_SLV1_CONFIG_BATCH_EXT_SENS_1_EN							(1<<3)

#define LSM6DSV_SLV2_ADD_REG						0x1B

#define LSM6DSV_SLV2_SUBADD_REG						0x1C

#define LSM6DSV_SLV2_CONFIG_REG						0x1D
#define LSM6DSV_SLV2_CONFIG_SLAVE2_READ_COUNT_MASK						(7<<0)
#define LSM6DSV_SLV2_CONFIG_BATCH_EXT_SENS_2_EN							(1<<3)

#define LSM6DSV_SLV3_ADD_REG						0x1E

#define LSM6DSV_SLV3_SUBADD_REG						0x1F

#define LSM6DSV_SLV3_CONFIG_REG						0x20
#define LSM6DSV_SLV3_CONFIG_SLAVE3_READ_COUNT_MASK						(7<<0)
#define LSM6DSV_SLV3_CONFIG_BATCH_EXT_SENS_3_EN							(1<<3)

#define LSM6DSV_DATAWRITE_SLV0_REG					0x21

#define LSM6DSV_STATUS_MASTER_REG					0x22
#define LSM6DSV_STATUS_MASTER_SENS_HUB_ENDOP							(1<<0)
#define LSM6DSV_STATUS_MASTER_SLAVE0_NACK								(1<<3)
#define LSM6DSV_STATUS_MASTER_SLAVE1_NACK								(1<<4)
#define LSM6DSV_STATUS_MASTER_SLAVE2_NACK								(1<<5)
#define LSM6DSV_STATUS_MASTER_SLAVE3_NACK								(1<<6)
#define LSM6DSV_STATUS_MASTER_WR_ONCE_DONE								(1<<7)



#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __AG_LSM6DSV_H__
