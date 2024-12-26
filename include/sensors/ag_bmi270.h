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

#define BMI270_I2C_7BITS_DEVADDR							0x68

#define BMI270_CHIP_ID_REG          	0x0

#define BMI270_CHIP_ID                                      0x24

#define BMI270_ERR_REG					0x2
#define BMI270_ERR_FATAL									(1<<0)
#define BMI270_ERR_INTERNAL_MASK							(0x1E<<1)
#define BMI270_ERR_FIFO										(1<<6)
#define BMI270_ERR_AUX										(1<<7)

#define BMI270_STATUS_REG				0x3
#define BMI270_STATUS_AUX_BUSY								(1<<2)
#define BMI270_STATUS_CMD_RDY								(1<<4)
#define BMI270_STATUS_DRDY_AUX								(1<<5)
#define BMI270_STATUS_DRDY_GYR								(1<<6)	//!< Gyro data ready
#define BMI270_STATUS_DRDY_ACC								(1<<7)	//!< Accel data ready

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
