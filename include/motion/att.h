/**-------------------------------------------------------------------------
@file	att.h

@brief	Attitude estimation capability

This a generic abstraction layer for attitude estimation.  It is a mean to
provide a common interface to different sensor fusion library out there.


@author	Hoang Nguyen Hoan
@date	Aug. 1, 2018

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
#ifndef __ATT_H__
#define __ATT_H__

#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"

/** @addtogroup Motion
  * @{
  */

/// Attitude features
//#define ATT_FEATURE_RAW_ACCEL				(1<<0)		//!< Raw accel sensor data, relevant for when sensor is known
//#define ATT_FEATURE_RAW_GYRO				(1<<1)		//!< Raw gyr sensor data, relevant for when sensor is known
//#define ATT_FEATURE_RAW_MAG					(1<<2)		//!< Raw mag sensor data, relevant for when sensor is known
//#define ATT_FEATURE_ACCEL					(1<<3)		//!< Converted accel data
//#define ATT_FEATURE_GYRO					(1<<4)		//!< Converted gyro data
//#define ATT_FEATURE_MAG						(1<<5)		//!< Converted mag data
#define ATT_FEATURE_EULER					(1<<0)		//!< Euler angles data
#define ATT_FEATURE_QUATERNION				(1<<1)		//!< Quaternion data
#define ATT_FEATURE_COMPASS					(1<<2)		//!< Compasss
#define ATT_FEATURE_GRAVITY					(1<<3)		//!< Gravity vector
#define ATT_FEATURE_EXTERNAL_ACCEL			(1<<4)		//!< External acceleration vector
#define ATT_FEATURE_TAP						(1<<5)		//!< Tap sensing
#define ATT_FEATURE_ROTATION				(1<<6)		//!< Rotation data
#define ATT_FEATURE_VIBRATION				(1<<7)		//!< Vibration data
#define ATT_FEATURE_PEDOMETER				(1<<8)		//!< Pedometer
#define ATT_FEATURE_CYCLING					(1<<9)		//!< Cycling

typedef uint32_t	ATT_FEATURE;

typedef struct __Att_Quat {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Q[4];
		struct {
			float Q1;
			float Q2;
			float Q3;
			float Q4;
		};
	};
} AttQuat_t;


typedef struct __Att_Euler {
	uint64_t Timestamp;	//!< Time stamp count in usec
	float Yaw;
	float Pitch;
	float Roll;
} AttEuler_t;


typedef struct __Att_Gravity {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Val[3];
		struct {
			float X;
			float Y;
			float Z;
		};
	};
} AttGravity_t;


/// External acceleration vector
typedef struct __Att_Extrn_Accel {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Val[3];
		struct {
			float X;
			float Y;
			float Z;
		};
	};
} AttExtAccel_t;


/// Rotation data
typedef struct __Att_Rotation_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
    uint32_t Count; 		//!< Number of rotations
    uint16_t Rpm; 			//!< Revolutions per minute
} AttRotation_t;


typedef struct __Att_Config {
	DevEvtHandler_t EvtHandler;
} AttCfg_t;


#ifdef __cplusplus

class Att : virtual public Device {
public:

	virtual bool Init(const AttCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool UpdateData() = 0;
	virtual void IntHandler() = 0;
	virtual bool Calibrate() = 0;
	virtual void SetAxisAlignmentMatrix(int8_t * const pMatrix) = 0;

	// Features
	virtual bool Euler(bool bEn) = 0;
	virtual bool Compass(bool bEn) = 0;
	virtual bool Quaternion(bool bEn, int NbAxis) = 0;
	virtual bool Read(AttQuat_t &Data) { Data = vQuat; return true; }
	virtual bool Read(AttEuler_t &Data) { Data = vEuler; return true; }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(AccelSensorRawData_t &Data) { return vpAccel->Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return vpAccel->Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(GyroSensorRawData_t &Data) { return vpGyro->Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return vpGyro->Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(MagSensorRawData_t &Data) { return vpMag->Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return vpMag->Read(Data); }

	virtual ATT_FEATURE Feature() { return vActiveFeature; }
	virtual ATT_FEATURE Feature(ATT_FEATURE FeatureBit, bool bEnDis);

	/**
	 * @brief	Set data rate in miliHz
	 *
	 * @param	DataRate : Data rate in miliHz
	 *
	 * @return	Actual data rate set
	 */
	virtual uint32_t Rate(uint32_t DataRate) { vRate = DataRate; return vRate; }

	/** @brief	Get data rate
	 *
	 * @return	Data rate in miliHz
	 */
	virtual uint32_t Rate() { return vRate; }


protected:
//	Timer *vpTimer;			//!< Pointer to Timer object for timestamping
	AccelSensor *vpAccel;	//!< Pointer to accelerometer sensor
	GyroSensor *vpGyro;		//!< Pointer to gyro sensor
	MagSensor *vpMag;		//!< Pointer to magnetometer Sensor
	ATT_FEATURE vActiveFeature;	//!< Orable feature enabled bits - Bit set to 1 : Enabled, 0 : Disabled
	AttQuat_t vQuat;			//!< Last updated quaternion values
	AttEuler_t vEuler;		//!< Last updated euler value
	uint32_t vRate;			//!< Data rate in mHz (mili-Hz)
};

#endif // __cplusplus

/** @} end group Motion */

#endif // __ATT_H__
