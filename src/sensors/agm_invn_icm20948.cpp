/**-------------------------------------------------------------------------
@file	agm_invn_icm20948.cpp

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

This implementation wraps the Invensen SmartMotion driver

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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
//#include "Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Devices/SensorTypes.h"
//#include "Devices/SensorConfig.h"
//#include "EmbUtils/InvScheduler.h"
//#include "EmbUtils/RingByteBuffer.h"
//#include "EmbUtils/Message.h"
//#include "EmbUtils/ErrorHelper.h"
//#include "EmbUtils/DataConverter.h"
//#include "EmbUtils/RingBuffer.h"
//#include "DynamicProtocol/DynProtocol.h"
//#include "DynamicProtocol/DynProtocolTransportUart.h"


#include "idelay.h"
#include "convutil.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_invn_icm20948.h"

#define ICM20948_WHO_AM_I_ID		0xEA

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
/*
static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
	INV_SENSOR_TYPE_ACCELEROMETER,
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_BAC,
	INV_SENSOR_TYPE_STEP_DETECTOR,
	INV_SENSOR_TYPE_STEP_COUNTER,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_ROTATION_VECTOR,
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
	INV_SENSOR_TYPE_MAGNETOMETER,
	INV_SENSOR_TYPE_SMD,
	INV_SENSOR_TYPE_PICK_UP_GESTURE,
	INV_SENSOR_TYPE_TILT_DETECTOR,
	INV_SENSOR_TYPE_GRAVITY,
	INV_SENSOR_TYPE_LINEAR_ACCELERATION,
	INV_SENSOR_TYPE_ORIENTATION,
	INV_SENSOR_TYPE_B2S
};
*/

__attribute__((weak)) void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

__attribute__((weak)) void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
}

int AgmInvnIcm20948::InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	AgmInvnIcm20948 *dev = (AgmInvnIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int AgmInvnIcm20948::InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	AgmInvnIcm20948 *dev = (AgmInvnIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

bool AgmInvnIcm20948::Init(uint32_t DevAddr, DeviceIntrf *pIntrf, Timer *pTimer)
{
	//if (vbInitialized)
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	uint8_t d;
	struct inv_icm20948_serif icm20948_serif;

	icm20948_serif.context   = this;
	icm20948_serif.read_reg  = InvnReadReg;
	icm20948_serif.write_reg = InvnWriteReg;
	icm20948_serif.max_read  = 255; /* maximum number of bytes allowed per serial read */
	icm20948_serif.max_write = 255; /* maximum number of bytes allowed per serial write */
	icm20948_serif.is_spi = vpIntrf->Type() == DEVINTRF_TYPE_SPI;

	inv_icm20948_reset_states(&vIcmDevice, &icm20948_serif);

	inv_icm20948_register_aux_compass(&vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);

	inv_icm20948_get_whoami(&vIcmDevice, &d);

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

	inv_icm20948_initialize(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));
	/* Initialize auxiliary sensors */
	inv_icm20948_register_aux_compass( &vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
	//rc = inv_icm20948_initialize_auxiliary(&vIcmDevice);

	// re-initialize base state structure
	inv_icm20948_init_structure(&vIcmDevice);

	vbInitialized  = true;

	return true;
}

bool AccelInvnIcm20948::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	AccelSensor::vData.Range = AccelSensor::Range(ICM20948_ACC_ADC_RANGE);

	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);
	FilterFreq(Cfg.FltrFreq);

	return true;
}

uint16_t AccelInvnIcm20948::Scale(uint16_t Value)
{
	enum mpu_accel_fs d;

	if (Value < 3)
	{
		d = MPU_FS_2G;
		Value = 2;
	}
	else if (Value < 6)
	{
		d = MPU_FS_4G;
		Value = 4;
	}
	else if (Value < 12)
	{
		d = MPU_FS_8G;
		Value = 8;
	}
	else
	{
		d = MPU_FS_16G;
		Value = 16;
	}

	inv_icm20948_set_accel_fullscale(*this, d);

	return AccelSensor::Scale(Value);
}

uint32_t AccelInvnIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])

	uint32_t div = (1125000 + (Freq >>1)) / Freq - 1;
	uint16_t d = EndianCvt16(div);
	uint16_t regaddr = REG_ACCEL_SMPLRT_DIV_1;

	Write16((uint8_t*)&regaddr, 2, d);

	div++;
//	inv_icm20948_ctrl_set_accel_quaternion_gain(s, div);
//	inv_icm20948_ctrl_set_accel_cal_params(s, div);
//	s->lLastHwSmplrtDividerAcc = div;
	return AccelSensor::SamplingFrequency(1125000 / div);
}

uint32_t AccelInvnIcm20948::FilterFreq(uint32_t Freq)
{
	return 0;
}

bool GyroInvnIcm20948::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	GyroSensor::vData.Range = GyroSensor::Range(ICM20948_GYRO_ADC_RANGE);
	Sensitivity(Cfg.Sensitivity);
	SamplingFrequency(Cfg.Freq);

	return true;
}

uint32_t GyroInvnIcm20948::Sensitivity(uint32_t Value)
{
#if 1
	enum mpu_gyro_fs gfsr;

	if (Value < 325)
	{
		gfsr = MPU_FS_250dps;
		Value = 250;
	}
	else if (Value < 750)
	{
		gfsr = MPU_FS_500dps;
		Value = 500;
	}
	else if (Value < 1500)
	{
		gfsr = MPU_FS_1000dps;
		Value = 1000;
	}
	else
	{
		gfsr = MPU_FS_2000dps;
		Value = 2000;
	}

	inv_icm20948_set_gyro_fullscale(*this, gfsr);
#else

	uint32_t d = Value;
	inv_icm20948_set_fsr(*this, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&d);
	inv_icm20948_set_fsr(*this, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&d);
	inv_icm20948_set_fsr(*this, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&d);

#endif
	return GyroSensor::Sensitivity(Value);
}

uint32_t GyroInvnIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])

	uint32_t div = (1100000 + (Freq >> 1)) / Freq - 1;
	uint16_t regaddr = REG_GYRO_SMPLRT_DIV;
	Write8((uint8_t*)&regaddr, 2, div);

	return GyroSensor::SamplingFrequency(1100000 / (div + 1));
}

uint32_t GyroInvnIcm20948::FilterFreq(uint32_t Freq)
{
	return 0;
}

bool MagInvnIcm20948::Init(const MagSensorCfg_t &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	int rc = inv_icm20948_initialize_auxiliary(*this);

	MagSensor::Range(AK09916_ADC_RANGE);

	return rc == 0;
}

bool AgmInvnIcm20948::Enable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
#if 0
	while(i-- > 0) {
		inv_icm20948_enable_sensor(&vIcmDevice, (inv_icm20948_sensor)i, 1);
	}
#else
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_ACCELEROMETER, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_GYROSCOPE, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_RAW_GYROSCOPE, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1);

#endif
	return true;
}

void AgmInvnIcm20948::Disable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		inv_icm20948_enable_sensor(&vIcmDevice, (inv_icm20948_sensor)i, 0);
	}
}

void AgmInvnIcm20948::Reset()
{
	inv_icm20948_soft_reset(&vIcmDevice);
}

bool AgmInvnIcm20948::StartSampling()
{
	return true;
}

// Implement wake on motion
bool AgmInvnIcm20948::WakeOnEvent(bool bEnable, int Threshold)
{
    uint16_t regaddr;

	if (bEnable == true)
	{
		Reset();

		msDelay(2000);
	}
	else
	{
//	    regaddr = MPU9250_AG_INT_ENABLE;
	    Write8((uint8_t*)&regaddr, 2, 0);

//	    regaddr = MPU9250_AG_PWR_MGMT_1;
		Write8((uint8_t*)&regaddr, 2, 0);
	}

	return true;
}
#if 0
// Accel low pass frequency
uint32_t AgmInvnIcm20948::FilterFreq(uint32_t Freq)
{
	return AccelSensor::FilterFreq(Freq);
}

// Accel scale
uint16_t AgmInvnIcm20948::Scale(uint16_t Value)
{
	return AccelSensor::Scale(Value);
}

// Gyro scale
uint32_t AgmInvnIcm20948::Sensitivity(uint32_t Value)
{

	return GyroSensor::Sensitivity(Value);
}
#endif
bool AgmInvnIcm20948::UpdateData()
{
	inv_icm20948_poll_sensor(&vIcmDevice, (void*)this, SensorEventHandler);

	return true;
}

bool AgmInvnIcm20948::SelectBank(uint8_t BankNo)
{
	if (BankNo > 3 || vCurrBank == BankNo)
		return false;

	vCurrBank = BankNo;

	uint8_t regaddr = REG_BANK_SEL;

	return Write8(&regaddr, 1, (BankNo << 4) & 0xF0);
}

int AgmInvnIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen == 2)
	{
		uint16_t *p = (uint16_t*)pCmdAddr;
		SelectBank(*p >> 7);
		CmdAddrLen--;
		*p &= 0x7f;
	}
//	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
//		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int AgmInvnIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
#if 1
	if (CmdAddrLen == 2)
	{
		uint16_t *p = (uint16_t*)pCmdAddr;
		SelectBank(*p >> 7);
		CmdAddrLen--;
		*p &= 0x7f;
	}
//	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
//		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
#else
	inv_icm20948_write_mems_reg(&vIcmDevice, *(uint16_t*)pCmdAddr, DataLen, pData);
#endif
	return DataLen;
}

void AgmInvnIcm20948::IntHandler()
{
	inv_icm20948_poll_sensor(&vIcmDevice, (void*)this, SensorEventHandler);
}

void AgmInvnIcm20948::SensorEventHandler(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	AgmInvnIcm20948 *dev = (AgmInvnIcm20948*)context;

	dev->UpdateData(sensortype, timestamp, data, arg);
}

void AgmInvnIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	float raw_bias_data[6];
	inv_sensor_event_t event;
	//uint8_t sensor_id = convert_to_generic_ids[sensortype];

	memset((void *)&event, 0, sizeof(event));
	event.sensor = sensortype;
	event.timestamp = timestamp;
	switch (sensortype)
	{
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
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
		{
		//memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
		//memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			float *p = (float*)data;
			GyroSensor::vData.X = p[0] * 256.0;
			GyroSensor::vData.Y = p[1] * 256.0;
			GyroSensor::vData.Z = p[2] * 256.0;
			GyroSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GRAVITY:
		{
			memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
			float *p = (float*)data;
			AccelSensor::vData.X = p[0] * 256.0;
			AccelSensor::vData.Y = p[1] * 256.0;
			AccelSensor::vData.Z = p[2] * 256.0;
			AccelSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		{
			float *p = (float*)data;
			//memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			//memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
			AccelSensor::vData.X = p[0] * 256.0;
			AccelSensor::vData.Y = p[1] * 256.0;
			AccelSensor::vData.Z = p[2] * 256.0;
			AccelSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		{
			float *p = (float*)data;
			//memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
			//memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
			MagSensor::vData.X = p[0] * 256.0;
			MagSensor::vData.Y = p[1] * 256.0;
			MagSensor::vData.Z = p[2] * 256.0;
			MagSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:
		memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
		break;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
	{
		uint8_t accel_accuracy;
		uint8_t gyro_accuracy;

		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

		accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
		gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();

		event.data.quaternion.accuracy_flag = min(accel_accuracy, gyro_accuracy);
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
	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
		memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
		break;
	default:
		return;
	}
}

int AgmInvnIcm20948::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = REG_USER_CTRL;
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | BIT_I2C_MST_EN;

#if 1
		uint8_t d[4];

		regaddr = REG_I2C_SLV0_ADDR;

		d[0] = (DevAddr & 0x7f) | INV_MPU_BIT_I2C_READ;
		d[1] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(15, BuffLen);

			d[2] = INV_MPU_BIT_SLV_EN | (cnt & 0xf);

			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = REG_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~BIT_I2C_MST_EN);

			regaddr = REG_I2C_SLV0_DO;
			cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
			if (cnt <= 0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
			d[1] += cnt;
		}
#else
		regaddr = ICM20948_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);

		regaddr = ICM20948_I2C_SLV0_REG;
		Write8((uint8_t*)&regaddr, 2, *pCmdAddr);

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (1 & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK));

		regaddr = ICM20948_USER_CTRL;
		Write8((uint8_t*)&regaddr, 2, userctrl);

		msDelay(100);

		Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);
		regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
		int cnt = Read((uint8_t*)&regaddr, 2, pBuff, 1);

		BuffLen -= cnt;

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, 0);


#endif
	}
	else
	{
		retval = vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	return retval;
}

int AgmInvnIcm20948::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = REG_USER_CTRL;
		uint8_t d[8];
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | BIT_I2C_MST_EN;

		regaddr = REG_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & 0x7F));

		d[0] = *pCmdAddr;
		d[1] = 1;// | ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN;	// Length : Write is done 1 byte at a time

		while (DataLen > 0)
		{
			d[2] = *pData;

			regaddr = REG_I2C_SLV0_REG;
			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = REG_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~BIT_I2C_MST_EN);

			d[0]++;
			pData++;
			DataLen--;
			retval++;
		}
	}
	else
	{
		retval = vpIntrf->Write(DevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	return retval;
}

#if 0
void sensor_event(const inv_sensor_event_t * event, void * arg){
	/* arg will contained the value provided at init time */
	(void)arg;

	/*
	* Encode sensor event and sent to host over UART through IddWrapper protocol
	*/
	static DynProtocolEdata_t async_edata; /* static to take on .bss */
	static uint8_t async_buffer[256]; /* static to take on .bss */
	uint16_t async_bufferLen;

	async_edata.sensor_id = event->sensor;
	async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
	convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

	if(DynProtocol_encodeAsync(&protocol,
		DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
		async_buffer, sizeof(async_buffer), &async_bufferLen) != 0) {
			goto error_dma_buf;
	}

	DynProTransportUart_tx(&transport, async_buffer, async_bufferLen);
	return;

error_dma_buf:
	INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");

	return;
}
#endif

