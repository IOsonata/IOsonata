/**-------------------------------------------------------------------------
@file	imu_invn_icm20948.cpp

@brief	Implementation of an Inertial Measurement Unit for Invensense ICM-20948

This is an implementation wrapper over Invensense SmartMotion for the ICM-20948
9 axis motion sensor

@author	Hoang Nguyen Hoan
@date	Dec. 26, 2018

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
#include <math.h>

#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"

#include "idelay.h"
#include "convutil.h"
#include "imu/imu_invn_icm20948.h"
#include "sensors/agm_invn_icm20948.h"

//#define ICM20948_WHO_AM_I_ID		0xEA

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

ImuInvnIcm20948::ImuInvnIcm20948()
{
	vpIcm = nullptr;
	vpIcmDevice = nullptr;
	vFifoHdr = vFifoHdr2 = 0;
	vFifoDataLen = 0;
}

#if 0
int ImuInvnIcm20948::InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	ImuInvnIcm20948 *dev = (ImuInvnIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int ImuInvnIcm20948::InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	ImuInvnIcm20948 *dev = (ImuInvnIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

bool ImuInvnIcm20948::Init(const IMU_CFG &Cfg, uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Imu::Init(Cfg, DevAddr, pIntrf, pTimer);
	//Interface(pIntrf);
	//DeviceAddess(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	struct inv_icm20948_serif icm20948_serif;

	icm20948_serif.context   = this;
	icm20948_serif.read_reg  = InvnReadReg;
	icm20948_serif.write_reg = InvnWriteReg;
	icm20948_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
	icm20948_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */
	icm20948_serif.is_spi = vpIntrf->Type() == DEVINTRF_TYPE_SPI;

	inv_icm20948_reset_states(&vIcmDevice, &icm20948_serif);

	inv_icm20948_register_aux_compass(&vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);

	uint8_t d;

	int rc = 	rc = inv_icm20948_get_whoami(&vIcmDevice, &d);

	if (d != ICM20948_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	usDelay(500000);

	// Setup accel and gyro mounting matrix and associated angle for current board
	inv_icm20948_init_matrix(&vIcmDevice);

	for (int i = 0; i < INV_ICM20948_SENSOR_MAX; i++) {
		inv_icm20948_set_matrix(&vIcmDevice, s_CfgMountingMatrix, (inv_icm20948_sensor)i);
	}

	rc = inv_icm20948_initialize(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));
	/* Initialize auxiliary sensors */
	inv_icm20948_register_aux_compass( &vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
	rc = inv_icm20948_initialize_auxiliary(&vIcmDevice);

	// re-initialize base state structure
	inv_icm20948_init_structure(&vIcmDevice);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&vCfgAccFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&vCfgAccFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&vCfgGyroFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&vCfgGyroFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&vCfgGyroFsr);

	rc = inv_icm20948_load(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));

	vpIcmDevice= &vIcmDevice;

	return true;
}
#endif

bool ImuInvnIcm20948::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == NULL)
	{
		return false;
	}

	Imu::Init(Cfg, pAccel, pGyro, pMag);

	vpIcm = (AgmInvnIcm20948*)pAccel;
	//vpSensorDev = (AgmInvnIcm20948*)pAccel;
	vEvtHandler = Cfg.EvtHandler;
	vpIcmDevice = *(AgmInvnIcm20948*)pAccel;

	return true;
}

bool ImuInvnIcm20948::Enable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		//inv_icm20948_set_sensor_period(vpIcmDevice, (inv_icm20948_sensor)i, 20);
		inv_icm20948_enable_sensor(vpIcmDevice, (inv_icm20948_sensor)i, 1);
	}

	return true;
}

void ImuInvnIcm20948::Disable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		inv_icm20948_enable_sensor(vpIcmDevice, (inv_icm20948_sensor)i, 0);
	}
	inv_icm20948_set_chip_power_state(vpIcmDevice, CHIP_AWAKE, 0);
}

void ImuInvnIcm20948::Reset()
{
	inv_icm20948_soft_reset(vpIcmDevice);
}

IMU_FEATURE ImuInvnIcm20948::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	return Imu::Feature();
}

bool ImuInvnIcm20948::Calibrate()
{
	return true;
}

void ImuInvnIcm20948::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool ImuInvnIcm20948::Compass(bool bEn)
{
	return true;
}

bool ImuInvnIcm20948::Pedometer(bool bEn)
{
	return true;
}

bool ImuInvnIcm20948::Quaternion(bool bEn, int NbAxis)
{
	printf("ImuInvnIcm20948::Quaternion\n");

	if (NbAxis < 9)
	{

	}
	else
	{

	}
	return true;
}

bool ImuInvnIcm20948::Tap(bool bEn)
{
	return true;
}

bool ImuInvnIcm20948::UpdateData()
{
	return true;
}
/*
void ImuInvnIcm20948::IntHandler()
{
	//if (vpIcmDev)
	{

//		inv_icm20948_poll_sensor(*vpIcmDev, (void*)this, SensorEventHandler);
	}
	//else
	{
		inv_icm20948_poll_sensor(vpIcmDevice, (void*)this, SensorEventHandler);
	}
}
*/
#if 1
void ImuInvnIcm20948::SensorEventHandler(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	ImuInvnIcm20948 *dev = (ImuInvnIcm20948*)context;

	dev->UpdateData(sensortype, timestamp, data, arg);
}
#endif

void ImuInvnIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	//float raw_bias_data[6];
	inv_sensor_event_t event;
	//uint8_t sensor_id = convert_to_generic_ids[sensortype];

	memset((void *)&event, 0, sizeof(event));
	event.sensor = sensortype;
	event.timestamp = timestamp;
	switch (sensortype)
	{
/*	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
		memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
		memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GYROSCOPE:
		memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GRAVITY:
		memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
		event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
		break;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
		memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
		vAccData.X = event.data.acc.vect[0] * 256.0;
		vAccData.Y = event.data.acc.vect[1] * 256.0;
		vAccData.Z = event.data.acc.vect[2] * 256.0;
		vAccData.Timestamp = timestamp;
		printf("a %d : %d %d %d\r\n", vAccData.Timestamp, vAccData.X, vAccData.Y, vAccData.Z);
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
		memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
		break;*/
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		break;
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:
		memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

#if 0
		vQuat.Q1 = event.data.quaternion.quat[0] * 16384.0;
		vQuat.Q2 = event.data.quaternion.quat[1] * 16384.0;
		vQuat.Q3 = event.data.quaternion.quat[2] * 16384.0;
		vQuat.Q4 = event.data.quaternion.quat[3] * 16384.0;
#else
		{
		float *d = (float*)data;
		float a = *(float*)arg;
		vQuat.Q1 = d[0];
		vQuat.Q2 = d[1];
		vQuat.Q3 = d[2];
		vQuat.Q4 = d[3];

		}
#endif
		vQuat.Timestamp = timestamp;
		break;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
	{
		uint8_t accel_accuracy;
		uint8_t gyro_accuracy;

		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

		accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
		gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
#if 0
		event.data.quaternion.accuracy_flag = min(accel_accuracy, gyro_accuracy);
		vQuat.Q1 = event.data.quaternion.quat[0];// * 32768.0;
		vQuat.Q2 = event.data.quaternion.quat[1];// * 32768.0;
		vQuat.Q3 = event.data.quaternion.quat[2];// * 32768.0;
		vQuat.Q4 = event.data.quaternion.quat[3];// * 32768.0;
		vQuat.Timestamp = timestamp;
#endif
	}
		break;
	case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
		memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
		break;
	case INV_ICM20948_SENSOR_FLIP_PICKUP:
	case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
	case INV_ICM20948_SENSOR_STEP_DETECTOR:
	case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
		event.data.event = true;
		break;
	case INV_ICM20948_SENSOR_B2S:
		event.data.event = true;
		memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
		break;
	case INV_ICM20948_SENSOR_STEP_COUNTER:
		memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
		break;
	case INV_ICM20948_SENSOR_ORIENTATION:
		//we just want to copy x,y,z from orientation data
		memcpy(&(event.data.orientation), data, 3*sizeof(float));
		break;
/*	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
		memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
		break;*/
	default:
		//((AgmInvnIcm20948*)vpAccel)->UpdateData(sensortype, timestamp, data, arg);
		//vpIcm->UpdateData(sensortype, timestamp, data, arg);
		return;
	}

	vpIcm->UpdateData(sensortype, timestamp, data, arg);

	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

void ImuInvnIcm20948::IntHandler()
{
#if 0
	inv_icm20948_poll_sensor(vpIcmDevice, (void*)this, SensorEventHandler);
#else
#if 0
	vpIcm->IntHandler();
#else
	//vpIcm->IntHandler();

	uint64_t t;
	uint16_t regaddr = REG_FIFO_COUNT_H;//ICM20948_FIFO_COUNTH_REG;
	size_t cnt = vpIcm->Read16((uint8_t*)&regaddr, 2);
	cnt = EndianCvt16(cnt);

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	regaddr = REG_FIFO_R_W;
	uint8_t *p = &vFifo[vFifoDataLen];

	while (cnt > ICM20948_FIFO_PAGE_SIZE)
	{
		int l = vpIcm->Read((uint8_t*)&regaddr, 2, p, 16);
		p += l;
		vFifoDataLen += l;
		cnt -= l;

		while (vFifoDataLen > 2)
		{
			if (vFifoHdr == 0 && vFifoHdr2 == 0)
			{
				int l = 0;
				// new packet
				vFifoHdr = ((uint16_t)vFifo[0] << 8U) | ((uint16_t)vFifo[1] & 0xFF);

				if (vFifoHdr & ~ICM20948_FIFO_HEADER_MASK)
				{
					vpIcm->ResetFifo();
					return;
				}

				l = 2;

				if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
				{
					vFifoHdr2 = ((uint16_t)vFifo[2] << 8U) | ((uint16_t)vFifo[3] & 0xFF);

					if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
					{
						vpIcm->ResetFifo();
						return;
					}

					l += 2;
				}
				vFifoDataLen -= l;

				if (vFifoDataLen > 0)
				{
					memmove(vFifo, &vFifo[l], vFifoDataLen);
				}
	//				printf("Header %x %x\n", vFifoHdr, vFifoHdr2);
			}
			int l = ProcessDMPFifo(vFifo, vFifoDataLen, t);
			if (l == 0)
			{
				return;
			}
			vFifoDataLen -= l;
			if (vFifoDataLen > 0)
			{
				memmove(vFifo, &vFifo[l], vFifoDataLen);
			}
		}
	}
#endif
#endif
}

size_t ImuInvnIcm20948::ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp)
{
	bool retval = false;
	size_t cnt = 0;
	//uint16_t regaddr = REG_FIFO_R_W;//ICM20948_FIFO_R_W_REG;
	uint8_t *d = pFifo;//[ICM20948_FIFO_PAGE_SIZE];

	if (vFifoHdr & ICM20948_FIFO_HEADER_ACCEL)
	{
		if (Len < ICM20948_FIFO_HEADER_ACCEL_SIZE)
		{
			return cnt;
		}

#if 0
		AccelSensor::vData.Timestamp = Timestamp;
		AccelSensor::vData.X = ((d[0] << 8) | (d[1] & 0xFF));// << 15;
		AccelSensor::vData.Y = ((d[2] << 8) | (d[3] & 0xFF));// << 15;
		AccelSensor::vData.Z = ((d[4] << 8) | (d[5] & 0xFF));// << 15;

#endif
		vpIcm->UpdateData(SENSOR_TYPE_ACCEL, Timestamp, d);
		d += ICM20948_FIFO_HEADER_ACCEL_SIZE;
		cnt += ICM20948_FIFO_HEADER_ACCEL_SIZE;
		Len -= ICM20948_FIFO_HEADER_ACCEL_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_ACCEL; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GYRO)
	{
		if (Len < ICM20948_FIFO_HEADER_GYRO_SIZE)
		{
			return cnt;
		}

#if 0
		GyroSensor::vData.Timestamp = Timestamp;
		GyroSensor::vData.X = ((uint16_t)d[0] << 8) | ((uint16_t)d[1] & 0xFF);
		GyroSensor::vData.Y = ((uint16_t)d[2] << 8) | ((uint16_t)d[3] & 0xFF);
		GyroSensor::vData.Z = ((uint16_t)d[4] << 8) | ((uint16_t)d[5] & 0xFF);
#endif
		// TODO : Process gyro bias
		vpIcm->UpdateData(SENSOR_TYPE_GYRO, Timestamp, d);

		d += ICM20948_FIFO_HEADER_GYRO_SIZE;
		cnt += ICM20948_FIFO_HEADER_GYRO_SIZE;
		Len -= ICM20948_FIFO_HEADER_GYRO_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GYRO; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CPASS_SIZE)
		{
			return cnt;
		}

#if 0
		MagSensor::vData.Timestamp = Timestamp;
		MagSensor::vData.X = ((int16_t)d[0] << 8) | (d[1] & 0xFF);
		MagSensor::vData.Y = ((int16_t)d[2] << 8) | (d[3] & 0xFF);
		MagSensor::vData.Z = ((int16_t)d[4] << 8) | (d[5] & 0xFF);
#endif
		vpIcm->UpdateData(SENSOR_TYPE_MAG, Timestamp, d);

		d += ICM20948_FIFO_HEADER_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CPASS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_ALS)
	{
		if (Len < ICM20948_FIFO_HEADER_ALS_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_ALS_SIZE;
		cnt += ICM20948_FIFO_HEADER_ALS_SIZE;
		Len -= ICM20948_FIFO_HEADER_ALS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_ALS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_QUAT6)
	{
		if (Len < ICM20948_FIFO_HEADER_QUAT6_SIZE)
		{
			return cnt;
		}

		int32_t q[3];

		q[0] = ((int32_t)d[0] << 24) | (((int32_t)d[1] << 16) & 0xFF0000) | (((int32_t)d[2] << 8) & 0xFF00) | ((int32_t)d[3] & 0xFF);
		q[1] = ((int32_t)d[4] << 24) | (((int32_t)d[5] << 16) & 0xFF0000) | (((int32_t)d[6] << 8) & 0xFF00) | ((int32_t)d[7] & 0xFF);
		q[2] = ((int32_t)d[8] << 24) | (((int32_t)d[9] << 16) & 0xFF0000) | (((int32_t)d[10] << 8) & 0xFF00) | ((int32_t)d[11] & 0xFF);

		vQuat.Q2 = (float)q[0] / (1 << 30);
		vQuat.Q3 = (float)q[1] / (1 << 30);
		vQuat.Q4 = (float)q[2] / (1 << 30);
		vQuat.Q1 = sqrt(1.0 - vQuat.Q2 * vQuat.Q2 - vQuat.Q3 * vQuat.Q3 - vQuat.Q4 * vQuat.Q4);

		d += ICM20948_FIFO_HEADER_QUAT6_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT6_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT6_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT6; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_QUAT9)
	{
		if (Len < ICM20948_FIFO_HEADER_QUAT9_SIZE)
		{
			return cnt;
		}

		int32_t q[3];

		q[0] = ((int32_t)d[0] << 24) | (((int32_t)d[1] << 16) & 0xFF0000) | (((int32_t)d[2] << 8) & 0xFF00) | ((int32_t)d[3] & 0xFF);
		q[1] = ((int32_t)d[4] << 24) | (((int32_t)d[5] << 16) & 0xFF0000) | (((int32_t)d[6] << 8) & 0xFF00) | ((int32_t)d[7] & 0xFF);
		q[2] = ((int32_t)d[8] << 24) | (((int32_t)d[9] << 16) & 0xFF0000) | (((int32_t)d[10] << 8) & 0xFF00) | ((int32_t)d[11] & 0xFF);

		vQuat.Q2 = (float)q[0] / (1 << 30);
		vQuat.Q3 = (float)q[1] / (1 << 30);
		vQuat.Q4 = (float)q[2] / (1 << 30);
		vQuat.Q1 = sqrt(1.0 - vQuat.Q2 * vQuat.Q2 - vQuat.Q3 * vQuat.Q3 - vQuat.Q4 * vQuat.Q4);

		d += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT9_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT9; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_STEP_DETECTOR)
	{
		if (Len < ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE)
		{
			return cnt;
		}

		int32_t stp = ((int32_t)d[0] << 24) | (((int32_t)d[1] << 16) & 0xFF0000) | (((int32_t)d[2] << 8) & 0xFF00) | ((int32_t)d[3] & 0xFF);

		d += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		cnt += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		Len -= ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_STEP_DETECTOR; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GEOMAG)
	{
		if (Len < ICM20948_FIFO_HEADER_GEOMAG_SIZE)
		{
			return cnt;
		}

		int32_t q[3];

		q[0] = ((int32_t)d[0] << 24) | (((int32_t)d[1] << 16) & 0xFF0000) | (((int32_t)d[2] << 8) & 0xFF00) | ((int32_t)d[3] & 0xFF);
		q[1] = ((int32_t)d[4] << 24) | (((int32_t)d[5] << 16) & 0xFF0000) | (((int32_t)d[6] << 8) & 0xFF00) | ((int32_t)d[7] & 0xFF);
		q[2] = ((int32_t)d[8] << 24) | (((int32_t)d[9] << 16) & 0xFF0000) | (((int32_t)d[10] << 8) & 0xFF00) | ((int32_t)d[11] & 0xFF);

		d += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		cnt += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		Len -= ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GEOMAG; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PRESSURE)
	{
		if (Len < ICM20948_FIFO_HEADER_PRESSURE_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		cnt += ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		Len -= ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PRESSURE; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE)
		{
			return cnt;
		}

		int32_t q[3];

		q[0] = ((int32_t)d[0] << 24) | (((int32_t)d[1] << 16) & 0xFF0000) | (((int32_t)d[2] << 8) & 0xFF00) | ((int32_t)d[3] & 0xFF);
		q[1] = ((int32_t)d[4] << 24) | (((int32_t)d[5] << 16) & 0xFF0000) | (((int32_t)d[6] << 8) & 0xFF00) | ((int32_t)d[7] & 0xFF);
		q[2] = ((int32_t)d[8] << 24) | (((int32_t)d[9] << 16) & 0xFF0000) | (((int32_t)d[10] << 8) & 0xFF00) | ((int32_t)d[11] & 0xFF);

		d += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_CPASS; // Clear bit
	}

	if (vFifoHdr2 != 0)
	{
		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACCEL_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE)
			{
				return cnt;
			}

			int16_t a = ((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF);

			d += ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACCEL_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_GYRO_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE)
			{
				return cnt;
			}

			int16_t a = ((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF);

			d += ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_GYRO_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_CPASS_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE);

			d += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_CPASS_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_PICKUP)
		{
			if (Len < ICM20948_FIFO_HEADER2_PICKUP_SIZE)
			{
				return cnt;
			}

			int16_t a = ((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF);

			d += ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			cnt += ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			Len -= ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_PICKUP; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACTI_RECOG)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE)
			{
				return cnt;
			}

			int16_t bacstate = ((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF);
			int32_t bacts = ((int32_t)d[0] << 24) | (((int32_t)d[1] << 16) & 0xFF0000) | (((int32_t)d[2] << 8) & 0xFF00) | ((int32_t)d[3] & 0xFF);

			d += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACTI_RECOG; // Clear bit
		}
	}

	if (Len < ICM20948_FIFO_FOOTER_SIZE)
	{
		vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;
	//printf("Footer size\n");
		return cnt;
	}

	int16_t odrcnt = ((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF);

	cnt += ICM20948_FIFO_FOOTER_SIZE;
	Len -= ICM20948_FIFO_FOOTER_SIZE;
	vFifoHdr &= ~ICM20948_FIFO_HEADER_FOOTER; // Clear bit

//	printf("fh %x %x\n", vFifoHdr, vFifoHdr2);

	vFifoHdr = vFifoHdr2 = 0;
	//printf("h: %x %x : %x %x %x %x\n", header, header2, pFifo[0], pFifo[1], pFifo[2], pFifo[3]);

	return cnt;
}
