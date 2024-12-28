/**-------------------------------------------------------------------------
@file	mag_bmm350.h

@brief	Bosch BMM350 magnetometer implementation


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
#ifndef __MAG_BMM350_H__
#define __MAG_BMM350_H__

#include <stdint.h>
#include "sensors/mag_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define BMM350_I2C_7BITS_DEVADDR							0x14
#define BMM350_I2C_7BITS_DEVADDR2							0x15	// When ADSEL = high

#define BMM350_CHIP_ID_REG          	0x0

#define BMM350_CHIP_ID                                      0x33

#define BMM350_ERR_REG					0x2
#define BMM350_ERR_PMU_CMD_ERR								(1<<0)	//!< Command error

#define BMM350_PAD_CTRL_REG				0x3		//!< Configure PAD behavior
#define BMM350_PAD_CTRL_DRV_MASK							(7<<0)	//!< The pad drive strength value mask

#define BMM350_PMU_CMD_AGGR_SET_REG		0x4
#define BMM350_PMU_CMD_AGGR_SET_ODR_MASK					(0xF<<0)
#define BMM350_PMU_CMD_AGGR_SET_ODR_400HZ					(2<<0)	//!< 400 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_200HZ					(2<<0)	//!< 200 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_100HZ					(2<<0)	//!< 100 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_50HZ					(2<<0)	//!< 50 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_25HZ					(2<<0)	//!< 25 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_12_5HZ					(2<<0)	//!< 12.5 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_6_25HZ					(2<<0)	//!< 6.25 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_3_125HZ					(2<<0)	//!< 3.215 Hz
#define BMM350_PMU_CMD_AGGR_SET_ODR_1_5625HZ				(2<<0)	//!< 1.5625 Hz
#define BMM350_PMU_CMD_AGGR_SET_AVG_MASK					(3<<4)
#define BMM350_PMU_CMD_AGGR_SET_AVG_NOAVG					(0<<4)	//!< No averaging
#define BMM350_PMU_CMD_AGGR_SET_AVG_2						(1<<4)
#define BMM350_PMU_CMD_AGGR_SET_AVG_4						(2<<4)
#define BMM350_PMU_CMD_AGGR_SET_AVG_8						(3<<4)

#define BMM350_PMU_CMD_AXIS_EN_REG		0x5
#define BMM350_PMU_CMD_AXIS_EN_X							(1<<0)	//!< Enable X axis
#define BMM350_PMU_CMD_AXIS_EN_Y							(1<<1)	//!< Enable Y axis
#define BMM350_PMU_CMD_AXIS_EN_Z							(1<<2)	//!< Enable Z axis

#define BMM350_PMU_CMD_REG				0x6
#define BMM350_PMU_CMD_MASK									(0xF<<0)
#define BMM350_PMU_CMD_SUSP									(0<<0)	//!< Enter suspend mode
#define BMM350_PMU_CMD_NM									(1<<0)	//!< Normal mode
#define BMM350_PMU_CMD_UOD_OAE								(2<<0)	//!< Update ODR and AVG parameter
#define BMM350_PMU_CMD_FM   								(3<<0)	//!< Forced mode with full CRST recharge
#define BMM350_PMU_CMD_FM_FAST								(4<<0)	//!< Forced mode with fast CRST recharge
#define BMM350_PMU_CMD_FGR									(5<<0)	//!< flux-guide reset with full CRST recharge
#define BMM350_PMU_CMD_FGR_FAST								(6<<0)	//!< flux-guide reset with fast recharge
#define BMM350_PMU_CMD_BR									(7<<0) 	//!< Do bit reset with full recharge
#define BMM350_PMU_CMD_BR_FAST								(8<<0)	//!< Do bit reset with fast recharge

#define BMM350_PMU_CMD_STATUS_0_REG		0x7
#define BMM350_PMU_CMD_STATUS_0_BUSY						(1<<0)	//!< Busy
#define BMM350_PMU_CMD_STATUS_0_ODR_OVWR					(1<<1)	//!< PMU_CMD_SET odr has been overwritten
#define BMM350_PMU_CMD_STATUS_0_AVG_OVWR					(1<<2)	//!< PMU_CMD_SET avg has been overwritten
#define BMM350_PMU_CMD_STATUS_0_NORMAL						(1<<3)	//!< Normal mode
#define BMM350_PMU_CMD_STATUS_0_ILLEGAL						(1<<4)	//!< Cmd not allowed

#define BMM350_PMU_CMD_STATUS_1_REG		0x8
#define BMM350_PMU_CMD_STATUS_1_ODR_S_MASK					(0xF<<0)//!< Effective odr value
#define BMM350_PMU_CMD_STATUS_1_AVG_S_MASK					(3<<4)	//!< Effective avg value

#define BMM350_I3C_ERR_REG				0x9
#define BMM350_I3C_ERR_0									(1<<0)	//!< SDR parity error
#define BMM350_I3C_ERR_3									(1<<3)	//!< S0/S1 error

#define BMM350_I2C_WDT_SET_REG			0xA
#define BMM350_I2C_WDT_SET_WDT_EN							(1<<0)	//!< Enable I2C watchdog
#define BMM350_I2C_WDT_SET_WDT_SEL_1_28MS					(0<<1)	//!< I2C watchdog timeout after 1.28ms
#define BMM350_I2C_WDT_SET_WDT_SEL_40_96MS					(1<<1)	//!< I2C watchdog timeout after 40.96ms

#define BMM350_INT_CTRL_REG				0x2E
#define BMM350_INT_CTRL_MODE_LATCHED						(1<<0)	//!< Interrupt latched mode
#define BMM350_INT_CTRL_POL_HIGH							(1<<1)	//!< Polarity high
#define BMM350_INT_CTRL_PUSHPULL							(1<<2)	//!< Push pull
#define BMM350_INT_CTRL_EN									(1<<3)	//!< Interrupt enable
#define BMM350_INT_CTRL_DRDY_EN								(1<<7)	//!< Data ready interrupot enable

#define BMM350_INT_CTRL_IBI_REG			0x2F
#define BMM350_INT_CTRL_IBI_DRDY_INT_IBI					(1<<0)	//!< Map data ready interrupt to I3C IBI
#define BMM350_INT_CTRL_IBI_CLR_DRAY_STATUS_IBI				(1<<4)	//!< Clear Int status data ready upon I3C IBI

#define BMM350_INT_STATUS_REG			0x30
#define BMM350_INT_STATUS_DRDY								(1<<2)	//!< Interrupt status data ready

#define BMM350_MAG_X_XLSB_REG			0x31
#define BMM350_MAG_X_LSB_REG			0x32
#define BMM350_MAG_X_MSB_REG			0x33

#define BMM350_MAG_Y_XLSB_REG			0x34
#define BMM350_MAG_Y_LSB_REG			0x35
#define BMM350_MAG_Y_MSB_REG			0x36
#define BMM350_MAG_Z_XLSB_REG			0x37
#define BMM350_MAG_Z_LSB_REG			0x38
#define BMM350_MAG_Z_MSB_REG			0x39

#define BMM350_TEMP_XLSB_REG			0x3A
#define BMM350_TEMP_LSB_REG				0x3B
#define BMM350_TEMP_MSB_REG				0x3C

#define BMM350_SENSORTIME_XLSB_REG		0x3D
#define BMM350_SENSORTIME_LSB_REG		0x3E
#define BMM350_SENSORTIME_MSB_REG		0x3F

#define BMM350_OTP_CMD_REG				0x50
#define BMM350_OTP_CMD_WORD_ADDR_MASK						(0x1F<<0)
#define BMM350_OTP_CMD_MASK									(0x7<<5)
#define BMM350_OTP_CMD_DIR_READ								(1<<5)
#define BMM350_OTP_CMD_DIR_PRGM_1B							(2<<5)
#define BMM350_OTP_CMD_DIR_PRGM								(3<<5)
#define BMM350_OTP_CMD_DIR_PWR_OFF_OTP						(4<<5)
#define BMM350_OTP_CMD_EXT_READ								(5<<5)
#define BMM350_OTP_CMD_EXT_PRGM								(6<<5)

#define BMM350_OTP_DATA_MSB_REG			0x52
#define BMM350_OTP_DATA_LSB_REG			0x53

#define BMM350_OTP_STATUS_REG			0x55
#define BMM350_OTP_STATUS_CMD_DONE							(1<<0)	//!< Command done flag
#define BMM350_OTP_STATUS_CUR_PAGE_ADDR_MASK				(0xF<<1)
#define BMM350_OTP_STATUS_ERR_MASK							(7<<5)
#define BMM350_OTP_STATUS_ERR_NONE							(0<<5)
#define BMM350_OTP_STATUS_ERR_BOOT							(1<<5)
#define BMM350_OTP_STATUS_ERR_PAGE_RD						(2<<5)
#define BMM350_OTP_STATUS_ERR_PAGE_PRG						(3<<5)
#define BMM350_OTP_STATUS_ERR_SIGN							(4<<5)
#define BMM350_OTP_STATUS_ERR_INV_CMD						(5<<5)

#define BMM350_TMR_SELTEST_USER_REG		0x60
#define BMM350_TMR_SELTEST_USER_ST_IGEN_EN					(1<<0)	//!< Enable selftest internal current gen
#define BMM350_TMR_SELTEST_USER_ST_N						(1<<1)	//!< Configure execution of neg field self test
#define BMM350_TMR_SELTEST_USER_ST_P						(1<<2)	//!< Configure execution of positive field self test
#define BMM350_TMR_SELTEST_USER_IST_EN_X					(1<<3)	//!< Activate internally generated self test field X
#define BMM350_TMR_SELTEST_USER_IST_EN_Y					(1<<3)	//!< Activate internally generated self test field Y
#define BMM350_TMR_SELTEST_USER_IST_EN_Z					(1<<3)	//!< Activate internally generated self test field Z

#define BMM350_CTRL_USER_REG			0x61
#define BMM350_CTRL_USER_CFG_SENS_TIM_AON					(1<<0)	//!< Force sensor timer always running, even in suspend mode

#define BMM350_CMD_REG					0x7E
#define BMM350_CMD_SOFTRESET								(0xB6)

#define BMM350_ADC_RANGE				((1<<23) - 1)	// 24 bits

#define BMM350_FLUX_DENSITY				2000000 //!< max flux density in nT (2000 uT)


#ifdef __cplusplus

class MagBmm350 : public MagSensor {
public:
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual bool StartSampling() { return true; }
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
};

extern "C" {
#endif // __cplusplus

#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif	// __MAG_BMM350_H__
