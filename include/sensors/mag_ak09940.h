/**-------------------------------------------------------------------------
@file	mag_ak09940.h

@brief	Implementation of Asahi Kasei AK09940x mag sensor


@author	Hoang Nguyen Hoan
@date	May 3, 2025

@license

MIT License

Copyright (c) 2025 I-SYST inc. All rights reserved.

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

#ifndef __MAG_AK09940_H__
#define __MAG_AK09940_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/mag_sensor.h"
#include "sensors/temp_sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define AK09940_I2C_7BITS_DEVADDR0             0xC
#define AK09940_I2C_7BITS_DEVADDR1             0xD
#define AK09940_I2C_7BITS_DEVADDR2             0xE
#define AK09940_I2C_7BITS_DEVADDR3             0xF

#define AK09940_WIA1_REG    0
#define AK09940_WIA1_COMPANY_ID               0x48

#define AK09940_WIA2_REG    1
#define AK09940_WIA2_DEVICE_ID                0xA3

#define AK09940_COMPANY_DEVICE_ID             0xA348   // Combine company & device id

#define AK09940_ST_REG			0x0F  //!< Status register
#define AK09940_ST_DRDY								(1<<0)	//!< Data ready
#define AK09940_ST_DOR                   			(1<<1)	//!< Data overrun

#define AK09940_ST1_REG			0x10  //!< Status register 1
#define AK09940_ST1_DRDY							(1<<0)	//!< Data ready
#define AK09940_ST1_FNUM_MASK						(0xF<<1)	//!< Fifo status mask, count of fifo set

#define AK09940_HXL_REG			0x11	//!< X data bit 0-7
#define AK09940_HXM_REG			0x12	//!< X data bit 8-15
#define AK09940_HXH_REG			0x13	//!< X data bit 16-17
#define AK09940_HYL_REG			0x14
#define AK09940_HYM_REG			0x15
#define AK09940_HYH_REG			0x16
#define AK09940_HZL_REG			0x17
#define AK09940_HZM_REG			0x18
#define AK09940_HZH_REG			0x19

#define AK09940_TMPS_REG		0x1A	//!< Temperature data. T (C) = 30 - TMPS / 1.7

#define AK09940_ST2_REG			0x1B  	//!< Status register 2
#define AK09940_ST2_DOR								(1<<0)	//!< Data overrun
#define AK09940_ST2_INV								(1<<1)	//!< Invalid data

// Self test data
#define AK09940_SXL_REG			0x20	//!< X data bits 0-7
#define AK09940_SXH_REG			0x21	//!< X data bits 8-10
#define AK09940_SYL_REG			0x22
#define AK09940_SYH_REG			0x23
#define AK09940_SZL_REG			0x24
#define AK09940_SZH_REG			0x25

#define AK09940_CTRL1_REG		0x30
#define AK09940_CTRL1_WM_MASK						(7<<0)	//!< Watermark mask
#define AK09940_CTRL1_DTSET							(1<<5)	//!< 0 - DRDY pin output. 1 - TRG pin : ext trigger pulse input
#define AK09940_CTRL1_MT2_EN						(1<<7)	//!< Ultralow power

#define AK09940_CTRL2_REG		0x31
#define AK09940_CTRL2_TEM_EN						(1<<6)	//!< Enable temperature measurement

#define AK09940_CTRL3_REG		0x32
#define AK09940_CTRL3_MODE_MASK						(0x1F<<0)	//!< Operating mode mask
#define AK09940_CTRL3_MODE_PWRDWN					(0<<0)	//!< Powerdown
#define AK09940_CTRL3_MODE_SINGLE					(1<<0)	//!< Single measurement mode
#define AK09940_CTRL3_MODE_CONTINUOUS_10HZ			(2<<0)	//!< Continuous measurement mode 10 Hz
#define AK09940_CTRL3_MODE_CONTINUOUS_20HZ			(4<<0)	//!< Continuous measurement mode 20 Hz
#define AK09940_CTRL3_MODE_CONTINUOUS_50HZ			(6<<0)	//!< Continuous measurement mode 50 Hz
#define AK09940_CTRL3_MODE_CONTINUOUS_100HZ			(8<<0)	//!< Continuous measurement mode 100 Hz
#define AK09940_CTRL3_MODE_CONTINUOUS_200HZ			(10<<0)	//!< Continuous measurement mode 200 Hz
#define AK09940_CTRL3_MODE_CONTINUOUS_400HZ			(12<<0)	//!< Continuous measurement mode 400 Hz
#define AK09940_CTRL3_MODE_CONTINUOUS_1000HZ		(14<<0)	//!< Continuous measurement mode 1 KHz
#define AK09940_CTRL3_MODE_CONTINUOUS_2500HZ		(15<<0)	//!< Continuous measurement mode 2.5 KHz
#define AK09940_CTRL3_MODE_EXTTRIG					(0x18<<0)	//!< External trigger measurement mode. Must have AK09940_CTRL1_DTSET set
#define AK09940_CTRL3_MODE_SELFTEST					(10<<0)	//!< Self test mode
#define AK09940_CTRL3_MT_MASK						(3<<5)	//!< Sensor drive mask
#define AK09940_CTRL3_MT_LP1						(0<<5)	//!< Low power drive 1
#define AK09940_CTRL3_MT_LP2						(1<<5)	//!< Low power drive 2
#define AK09940_CTRL3_MT_LN1						(2<<5)	//!< Low noise drive 1
#define AK09940_CTRL3_MT_LN2						(3<<5)	//!< Low noise drive 2
#define AK09940_CTRL3_FIFO_EN						(1<<7)	//!< FIFO enable

#define AK09940_CTRL4_REG		0x33
#define AK09940_CTRL4_SRST							(1<<0)	//!< Soft reset

#define AK09940_I2CDIS_REG		0x36
#define AK09940_I2CDIS_DISABLE						(0x1B)	//!< Write this value to disable I2C. Once disable, reset is required to re-enable I2C


#define AK09940_ADC_RANGE			131070
#define AK09940_FLUX_DENSITY        1310700	//!< Max flux density in nT
#define AK09940_SENSITIVITY			10		//!< Sensitivity in nT

#define AK09940_FIFO_WM_LEVEL_MIN	2		//!< Minimum watermark level require for Fifo to function

#pragma pack(push, 1)


#pragma pack(pop)

#ifdef __cplusplus

class TempAk09940 : public TempSensor {
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
//	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter = 0, DEVINTR_POL Pol = DEVINTR_POL_LOW, Timer * const pTimer = NULL) = 0;
};

class MagAk09940 : public MagSensor, public TempAk09940 {
public:
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);

	/**
	 * @brief Power off the device completely.
	 *
	 * If supported, this will put the device in complete power down.
	 * Full re-initialization is required to re-enable the device.
	 */
	virtual void PowerOff();
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

//	virtual bool Read(MagSensorRawData_t &Data) { return MaglSensor::Read(Data); }
//	virtual bool Read(MagSensorRawData_t &Data) { return MaglSensor::Read(Data); }

	/**
	 * @brief	Flush any data stuck in queue or fifo
	 *
	 * This function is implementation specific to allow flushing of sensor data that may
	 * stuck in the queue or fifo that could prevent interrupt to trigger. Some sensors start
	 * steaming data as soon as the sampling frequency is set. It could cause the data ready to trigger
	 * before the application could finish setting up the interrupt handler. Therefore
	 * this function is called to clear all data & interrupt flags so interrupt could resume.
	 * Not all sensors need this so the default is do nothing.
	 */
	virtual void Flush(void);

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler(void);
	virtual bool UpdateData();

protected:
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen) {
		return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
	}

private:
//	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter = 0, DEVINTR_POL Pol = DEVINTR_POL_LOW, Timer * const pTimer = NULL);
	virtual bool StartSampling(void);

	uint8_t vCtrl1Val;
	uint8_t vCtrl3Val;
};


#endif // __cplusplus

/** @} End of group Sensors */

#endif // __AG_AK09940_H__

