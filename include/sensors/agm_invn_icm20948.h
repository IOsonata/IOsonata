/**-------------------------------------------------------------------------
@file	agm_icm20948.h

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

This implementation wrap the InvenSense SmartMotion Driver.

@author	Hoang Nguyen Hoan
@date	Dec. 24, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __AGM_INVN_ICM20948_H__
#define __AGM_INVN_ICM20948_H__

#include <stdint.h>

#include "Devices/Drivers/Icm20948/Icm20948.h"

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"
#include "sensors/mag_ak09916.h"
#include "sensors/agm_icm20948.h"

/** @addtogroup Sensors
  * @{
  */

#define ICM20948_ACC_ADC_RANGE			32767
#define ICM20948_GYRO_ADC_RANGE			32767
#define AK09916_ADC_RANGE				32752

#ifdef __cplusplus

class AccelInvnIcm20948 : public AccelSensor {
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
	virtual bool Read(AccelSensorData_t &Data) { memcpy(&Data, &vData, sizeof(AccelSensorData_t)); return true; }

private:

	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	virtual operator inv_icm20948_t * const () = 0;	// Get device interface data (handle)
};

class GyroInvnIcm20948 : public GyroSensor {
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

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	virtual operator inv_icm20948_t * const () = 0;	// Get device interface data (handle)
};

class MagInvnIcm20948 : public MagAk09916 {
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

protected:
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) = 0;
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) = 0;

	uint8_t vMagCtrl1Val;
	int16_t vMagSenAdj[3];

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
//	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
	//virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) = 0;
	//virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) = 0;

	uint32_t vDevAddr;
	virtual operator inv_icm20948_t * const () = 0;	// Get device interface data (handle)
};

class AgmInvnIcm20948 : public AccelInvnIcm20948, public GyroInvnIcm20948, public MagInvnIcm20948 {
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
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		return AccelInvnIcm20948::Init(Cfg, pIntrf, pTimer);
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
		return GyroInvnIcm20948::Init(Cfg, pIntrf, pTimer);
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
		return MagInvnIcm20948::Init(Cfg, pIntrf, pTimer);
	}

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	/**
	 * @brief	Enable/Disable wake on motion event
	 *
	 * @param bEnable
	 * @param Threshold
	 * @return
	 */
	virtual bool WakeOnEvent(bool bEnable, int Threshold);

	virtual bool StartSampling();
/*	virtual uint32_t FilterFreq(uint32_t Freq);

	virtual uint16_t Scale(uint16_t Value);			// Accel
	virtual uint32_t Sensitivity(uint32_t Value);	// Gyro
*/
	virtual bool Read(AccelSensorRawData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(GyroSensorRawData_t &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(MagSensorRawData_t &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return MagSensor::Read(Data); }

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

	bool UpdateData();
	virtual void IntHandler();
	void UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg);
	void UpdateData(SENSOR_TYPE Type, uint64_t Timestamp, uint8_t * const pData);

	operator inv_icm20948_t * const () { return &vIcmDevice; }

	static int InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
	static int InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
	static void SensorEventHandler(void * context, enum inv_icm20948_sensor sensor, uint64_t timestamp, const void * data, const void *arg);

	AgmInvnIcm20948();
	void ResetFifo();

private:
	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	bool SelectBank(uint8_t BankNo);
	size_t ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp);

	bool vbInitialized;
	inv_icm20948_t vIcmDevice;
	//int32_t vCfgAccFsr; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
	//int32_t vCfgGyroFsr; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
	uint8_t vCurrBank;
	bool vbDmpEnabled;
	uint16_t vFifoHdr;	//!< DMP FIFO header
	uint16_t vFifoHdr2;	//!< DMP FIFO header
	//uint8_t vFifo[ICM20948_FIFO_PAGE_SIZE * 2]; //!< FIFO cache
	uint8_t vFifo[ICM20948_FIFO_SIZE_MAX + ICM20948_FIFO_PAGE_SIZE];
	size_t vFifoDataLen;	//!< Data length currently in fifo
	bool vbSensorEnabled[ICM20948_NB_SENSOR];
	SENSOR_TYPE vType;	//!< Bit field indicating the sensors contain within
};

extern "C" {
#endif // __cplusplus

// public C functions

#ifdef __cplusplus
}
#endif // __cplusplus

/** @} End of group Sensors */

#endif // __AGM_INVN_ICM20948_H__

