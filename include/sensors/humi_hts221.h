/**-------------------------------------------------------------------------
@file	humi_hts221.h

@brief	Definitions for the ST HTS221 humidity and temperature sensor

The HTS221 measures relative humidity and temperature, on I2C or SPI. Each
device is calibrated in the factory: two humidity and two temperature points
are stored in its registers, and the readings are interpolated between them.

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
#ifndef __HUMI_HTS221_H__
#define __HUMI_HTS221_H__

#include <stdint.h>

#include "sensors/humi_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define HTS221_I2C_DEV_ADDR				0x5F

// Register address flag of a multiple byte access : on I2C, bit 7 increments
// the address. On SPI, bit 7 reads and bit 6 increments.
#define HTS221_I2C_ADDR_INC				(1<<7)
#define HTS221_SPI_READ					(1<<7)
#define HTS221_SPI_ADDR_INC				(1<<6)

#define HTS221_WHO_AM_I_REG						0x0F
#define HTS221_WHO_AM_I_ID												0xBC

#define HTS221_AV_CONF_REG						0x10
#define HTS221_AV_CONF_AVGH_MASK										(7<<0)	//!< Humidity samples averaged, 4 << AVGH
#define HTS221_AV_CONF_AVGH_POS											(0)
#define HTS221_AV_CONF_AVGT_MASK										(7<<3)	//!< Temperature samples averaged, 2 << AVGT
#define HTS221_AV_CONF_AVGT_POS											(3)
#define HTS221_AV_CONF_DEFAULT											0x1B	//!< 32 humidity and 16 temperature samples

#define HTS221_CTRL_REG1_REG					0x20
#define HTS221_CTRL_REG1_ODR_MASK										(3<<0)	//!< Output data rate
#define HTS221_CTRL_REG1_ODR_ONE_SHOT									(0<<0)
#define HTS221_CTRL_REG1_ODR_1HZ										(1<<0)
#define HTS221_CTRL_REG1_ODR_7HZ										(2<<0)
#define HTS221_CTRL_REG1_ODR_12_5HZ										(3<<0)
#define HTS221_CTRL_REG1_BDU											(1<<2)	//!< Output registers not updated until both bytes are read
#define HTS221_CTRL_REG1_PD												(1<<7)	//!< Active mode, else power down

#define HTS221_CTRL_REG2_REG					0x21
#define HTS221_CTRL_REG2_ONE_SHOT										(1<<0)	//!< Start one measurement, cleared by the device when done
#define HTS221_CTRL_REG2_HEATER											(1<<1)	//!< Internal heater on
#define HTS221_CTRL_REG2_BOOT											(1<<7)	//!< Reload the calibration, cleared by the device when done

#define HTS221_CTRL_REG3_REG					0x22
#define HTS221_CTRL_REG3_DRDY											(1<<2)	//!< Data ready on the DRDY pin
#define HTS221_CTRL_REG3_PP_OD											(1<<6)	//!< DRDY pin open drain, else push-pull
#define HTS221_CTRL_REG3_DRDY_H_L										(1<<7)	//!< DRDY pin active low

#define HTS221_STATUS_REG						0x27
#define HTS221_STATUS_T_DA												(1<<0)	//!< New temperature data
#define HTS221_STATUS_H_DA												(1<<1)	//!< New humidity data

// Humidity then temperature, 16 bits each
#define HTS221_HUMIDITY_OUT_L_REG				0x28
#define HTS221_TEMP_OUT_L_REG					0x2A

// Calibration, 16 registers
#define HTS221_CALIB_REG						0x30
#define HTS221_CALIB_SIZE				16
#define HTS221_CALIB_H0_RH_X2			0		//!< Humidity of point 0, % x 2
#define HTS221_CALIB_H1_RH_X2			1		//!< Humidity of point 1, % x 2
#define HTS221_CALIB_T0_DEGC_X8			2		//!< Temperature of point 0, degree C x 8, low 8 bits
#define HTS221_CALIB_T1_DEGC_X8			3		//!< Temperature of point 1, degree C x 8, low 8 bits
#define HTS221_CALIB_T1_T0_MSB			5		//!< Bits 9:8 of T0 in bits 1:0, of T1 in bits 3:2
#define HTS221_CALIB_H0_T0_OUT			6		//!< Humidity output of point 0, 16 bits
#define HTS221_CALIB_H1_T0_OUT			10		//!< Humidity output of point 1, 16 bits
#define HTS221_CALIB_T0_OUT				12		//!< Temperature output of point 0, 16 bits
#define HTS221_CALIB_T1_OUT				14		//!< Temperature output of point 1, 16 bits

#ifdef __cplusplus

class HumiHts221 : public HumiSensor, public TempSensor {
public:
	HumiHts221();

	/**
	 * @brief	Initialize the humidity part.
	 *
	 * Cfg.OpMode SENSOR_OPMODE_SINGLE measures once on each StartSampling,
	 * else the device measures continuously at the rate of Cfg.Freq.
	 * Cfg.HumOvrs is the number of humidity samples averaged, 4 to 512,
	 * 0 keeps the device default.
	 */
	virtual bool Init(const HumiSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	/**
	 * @brief	Initialize the temperature part.
	 *
	 * Cfg.TempOvrs is the number of temperature samples averaged, 2 to 256,
	 * 0 keeps the device default. Cfg.Inter not 0 enables the data ready
	 * on the DRDY pin with the polarity of Cfg.IntPol.
	 */
	virtual bool Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	/**
	 * @brief	Set the output data rate.
	 *
	 * @param	Freq : Frequency in mHz, 1 Hz, 7 Hz or 12.5 Hz
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
	virtual void Read(HumiSensorData_t &Data) { HumiSensor::Read(Data); }
	virtual void Read(TempSensorData_t &Data) { TempSensor::Read(Data); }

	/**
	 * @brief	Turn the internal heater on or off
	 *
	 * The heater dries the sensing element after condensation. The
	 * readings are not valid while it is on.
	 */
	void Heater(bool bOn);

protected:
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen);

private:
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	uint8_t vOdr;			//!< ODR field of CTRL_REG1 written by Enable
	int16_t vH0x2;			//!< Humidity of calibration point 0, % x 2
	int16_t vH1x2;			//!< Humidity of calibration point 1, % x 2
	int16_t vH0Out;			//!< Humidity output of point 0
	int16_t vH1Out;			//!< Humidity output of point 1
	int16_t vT0x8;			//!< Temperature of point 0, degree C x 8
	int16_t vT1x8;			//!< Temperature of point 1, degree C x 8
	int16_t vT0Out;			//!< Temperature output of point 0
	int16_t vT1Out;			//!< Temperature output of point 1
};

extern "C" {
#endif // __cplusplus


#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __HUMI_HTS221_H__
