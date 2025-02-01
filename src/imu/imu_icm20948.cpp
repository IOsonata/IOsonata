/**-------------------------------------------------------------------------
@file	imu_icm20948.cpp

@brief	Implementation of an Inertial Measurement Unit for Invensense ICM-20948

@author	Hoang Nguyen Hoan
@date	Sept. 9, 2019

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
#include <math.h>

/*
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"
*/

#include "idelay.h"
#include "istddef.h"
#include "convutil.h"
#include "imu/imu_icm20948.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_icm20948DMP.h"


static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const int s_DmpImageSize = sizeof(s_Dmp3Image);

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

int ImuIcm20948::InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	AgmIcm20948 *dev = (AgmIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int ImuIcm20948::InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	AgmIcm20948 *dev = (AgmIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

#if 0
void ImuIcm20948::SensorEventHandler(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	ImuIcm20948 *dev = (ImuIcm20948*)context;

	dev->UpdateData(sensortype, timestamp, data, arg);
}

bool ImuIcm20948::Init(const ImuCfg_t &Cfg, AgmIcm20948 * const pIcm)
{
	if (pIcm == NULL)
	{
		return false;
	}

	vpIcm = pIcm;

	// Disable DMP & FIFO before FIFO can be reseted and DMP firmware loaded
	uint16_t regaddr = ICM20948_USER_CTRL;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);
	vpIcm->Write8((uint8_t*)&regaddr,	2, d);

	// Reset FIFO

	regaddr = ICM20948_FIFO_RST;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0);

	// Upload DMP firmware
	if (vpIcm->UploadDMPImage((uint8_t*)s_Dmp3Image, DMP_CODE_SIZE, DMP_LOAD_START))
	{
		uint8_t dd[2];

		dd[0] = DMP_START_ADDRESS >> 8U;
		dd[1] = DMP_START_ADDRESS & 0xFFU;

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH;
		vpIcm->Write((uint8_t*)&regaddr, 2, dd, 2);

		Init(Cfg, pIcm, pIcm, pIcm);

		regaddr = ICM20948_USER_CTRL;
		d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		return true;
	}

	return false;
}
#endif

bool ImuIcm20948::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	// Min require for IMU are accel & gyro, must be combo device
	if (pAccel == nullptr || pGyro == nullptr || (AgmIcm20948*)pAccel != (AgmIcm20948*)pGyro)
	{
		return false;
	}

	vpIcm = (AgmIcm20948*)pAccel;

	uint16_t d;
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t userctrl = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);

	// Disable all to initialize DMP
	vpIcm->Write8((uint8_t*)&regaddr, 2, userctrl);
	vpIcm->Disable();


	bool res = InitDMP(ICM20948_DMP_PROG_START_ADDR, s_Dmp3Image, ICM20948_DMP_CODE_SIZE);

	if (res == false)
	{
		return false;
	}

	Imu::Init(Cfg, pAccel, pGyro, pMag);
	vEvtHandler = Cfg.EvtHandler;

	ResetDMPCtrlReg();

	// Fifo watermark 80%
	uint16_t val = EndianCvt16(800);
	WriteDMP(ICM20948_DMP_FIFO_WATERMARK_REG, (uint8_t*)&val, 2);

	regaddr = ICM20948_FIFO_CFG_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_CFG_SINGLE);

	// Undocumented value
	regaddr = ICM20948_SINGLE_FIFO_PRIORITY_SEL;
	Write8((uint8_t*)&regaddr, 2, ICM20948_SINGLE_FIFO_PRIORITY_SEL_0XE4);

	SetDMPGyroScale();
	SetDMPAccelScale();

	// These only works in 56 Hz as stated in the INVN example
	d = 0;	// 56 Hz
	WriteDMP(ICM20948_DMP_BAC_RATE, (uint8_t*)&d, 1);
	WriteDMP(ICM20948_DMP_B2S_RATE, (uint8_t*)&d, 1);

	ResetFifo();

	regaddr = ICM20948_INT_ENABLE_1_REG;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_INT_ENABLE_REG;
	d = ICM20948_INT_ENABLE_DMP_INT1_EN;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_INT_ENABLE_2_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN);

	regaddr = ICM20948_INT_ENABLE_3_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_INT_ENABLE_3_FIFO_WM_EN);


	return true;
}

void ImuIcm20948::ResetDMPCtrlReg()
{
	uint8_t d[2] = {0, 0};

	WriteDMP(ICM20948_DMP_DATA_OUT_CTL1_REG, d, 2);
	WriteDMP(ICM20948_DMP_DATA_OUT_CTL2_REG, d, 2);
	WriteDMP(ICM20948_DMP_DATA_INTR_CTL_REG, d, 2);
	WriteDMP(ICM20948_DMP_MOTION_EVENT_CTL_REG, d, 2);
	WriteDMP(ICM20948_DMP_DATA_RDY_STATUS_REG, d, 2);
}

void ImuIcm20948::ResetFifo()
{
	uint16_t regaddr;
	uint16_t cnt;

	regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);
	Write8((uint8_t*)&regaddr, 2, d & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN));

	do {
		regaddr = ICM20948_FIFO_RST_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
		Write8((uint8_t*)&regaddr, 2, ~ICM20948_FIFO_RST_FIFO_RESET_MASK);//0x1e);
		msDelay(1);

		regaddr = ICM20948_FIFO_COUNTH_REG;
		cnt = EndianCvt16(Read16((uint8_t*)&regaddr, 2)) & 0x1FFF;
	} while (cnt != 0);

	regaddr = ICM20948_INT_STATUS_2_REG;
	Write16((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, d);
}

bool ImuIcm20948::SetDMPAccelScale()
{
	int32_t scale, scale2;

	uint16_t d = ((AccelSensor*)vpIcm)->Scale();

	switch (d)
	{
		case 2:
			scale = (1 << 25);  // 33554432L
			scale2 = (1 << 19);	// 524288L
			break;
		case 4:
			scale =  (1 << 26);	// 67108864L
			scale2 = (1 << 18);	// 262144L
			break;
		case 8:
			scale = (1 << 27);  // 134217728L
			scale2 = (1 << 17);	// 131072L
			break;
		case 16:
			scale = (1 << 28);  // 268435456L
			scale2 = (1 << 16);	// 65536L
			break;
	}

	/**
	* Sets scale in DMP to convert accel data to 1g=2^25 regardless of fsr.
	* @param[in] fsr for accel parts
	2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.

	For 2g parts, 2g = 2^15 -> 1g = 2^14,.
	DMP takes raw accel data and left shifts by 16 bits, so 1g=2^14 (<<16) becomes 1g=2^30, to make 1g=2^25, >>5 bits.
	In Q-30 math, >> 5 equals multiply by 2^25 = 33554432.

	For 8g parts, 8g = 2^15 -> 1g = 2^12.
	DMP takes raw accel data and left shifts by 16 bits, so 1g=2^12 (<<16) becomes 1g=2^28, to make 1g=2^25, >>3bits.
	In Q-30 math, >> 3 equals multiply by 2^27 = 134217728.
	*/
	uint16_t regaddr = ICM20948_DMP_ACC_SCALE;
	scale = EndianCvt32(scale);
	WriteDMP(regaddr, (uint8_t*)&scale, 4);

	/**
	* According to input fsr, a scale factor will be set at memory location ACC_SCALE2
	* to convert calibrated accel data to 16-bit format same as what comes out of MPU register.
	* It is a reverse scaling of the scale factor written to ACC_SCALE.
	* @param[in] fsr for accel parts
	2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.
	*/
	regaddr = ICM20948_DMP_ACC_SCALE2;
	scale2 = EndianCvt32(scale2);
	WriteDMP(regaddr, (uint8_t*)&scale2, 4);

	return true;
}

bool ImuIcm20948::SetDMPGyroScale()
{
	int32_t scale = ((GyroSensor*)vpIcm)->Sensitivity();

	switch(scale)
	{
		case 250:
			scale = (1 << 25);
			break;
		case 500:
			scale = (1 << 26);
			break;
		case 1000:
			scale = (1 << 27);
			break;
		case 2000:
			scale = (1 << 28);
			break;
	}
	/**
	* Sets scale in DMP to convert gyro data to 4000dps=2^30 regardless of fsr.
	* @param[in] fsr for gyro parts
	4000: 4000dps. 2000: 2000dps. 1000: 1000dps. 500: 500dps. 250: 250dps.

	For 4000dps parts, 4000dps = 2^15.
	DMP takes raw gyro data and left shifts by 16 bits, so (<<16) becomes 4000dps=2^31, to make 4000dps=2^30, >>1 bit.
	In Q-30 math, >> 1 equals multiply by 2^29 = 536870912.

	For 2000dps parts, 2000dps = 2^15.
	DMP takes raw gyro data and left shifts by 16 bits, so (<<16) becomes 2000dps=2^31, to make 4000dps=2^30, >>2 bits.
	In Q-30 math, >> 2 equals multiply by 2^28 = 268435456.
	*/
	uint16_t regaddr = ICM20948_DMP_GYRO_FULLSCALE;
	scale = EndianCvt32(scale);
	WriteDMP(regaddr, (uint8_t*)&scale, 4);

	/**
	 *
	 * gyro_level should be set to 4 regardless of fullscale, due to the addition of API dmp_icm20648_set_gyro_fsr()
	 * 4 = ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS
	 */
	uint8_t tbpll;
	regaddr = ICM20948_GYRO_SMPLRT_DIV_REG;
	uint8_t div = Read8((uint8_t*)&regaddr, 2);

	regaddr = ICM20948_TIMEBASE_CORRECTION_PLL_REG;
	tbpll = Read8((uint8_t*)&regaddr, 2);



	const uint64_t MagicConstant = 264446880937391ULL;
	const uint64_t MagicConstantScale = 100000ULL;
	uint64_t res;
	int32_t gyrosf;

	if (tbpll & 0x80)
	{
		res = (MagicConstant * (long long)(1ULL << ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS) * (1 + div) / (1270 - (tbpll & 0x7F)) / MagicConstantScale);
	}
	else
	{
		res = (MagicConstant * (long long)(1ULL << ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS) * (1 + div) / (1270 + tbpll) / MagicConstantScale);
	}

	/**
	    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
	    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
	    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
	*/
	if  (res > 0x7FFFFFFF)
		gyrosf = EndianCvt32(0x7FFFFFFF);
	else
		gyrosf = EndianCvt32((int32_t)res);

	regaddr = ICM20948_DMP_GYRO_SF;
	WriteDMP(regaddr, (uint8_t*)&gyrosf, 4);

	return true;
}
#if 0
static const ANDROID_SENSORS s_InvSensor2AndroidSensor[] = {
	ANDROID_SENSOR_ACCELEROMETER,
	ANDROID_SENSOR_GYROSCOPE,
	ANDROID_SENSOR_RAW_ACCELEROMETER,
	ANDROID_SENSOR_RAW_GYROSCOPE,
	ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_ACTIVITY_CLASSIFICATON,
	ANDROID_SENSOR_STEP_DETECTOR,
	ANDROID_SENSOR_STEP_COUNTER,
	ANDROID_SENSOR_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_ROTATION_VECTOR,
	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_GEOMAGNETIC_FIELD,
	ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
	ANDROID_SENSOR_FLIP_PICKUP,
	ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
	ANDROID_SENSOR_GRAVITY,
	ANDROID_SENSOR_LINEAR_ACCELERATION,
	ANDROID_SENSOR_ORIENTATION,
	ANDROID_SENSOR_B2S,
	ANDROID_SENSOR_NUM_MAX,
};

#if 1
static uint8_t sensor_type_2_android_sensorx(enum inv_icm20948_sensor sensor)
{
	switch(sensor) {
	case INV_ICM20948_SENSOR_ACCELEROMETER:                 return ANDROID_SENSOR_ACCELEROMETER;
	case INV_ICM20948_SENSOR_GYROSCOPE:                     return ANDROID_SENSOR_GYROSCOPE;
	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:             return ANDROID_SENSOR_RAW_ACCELEROMETER;
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:                 return ANDROID_SENSOR_RAW_GYROSCOPE;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:   return ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:        return ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED;
	case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:        return ANDROID_SENSOR_ACTIVITY_CLASSIFICATON;
	case INV_ICM20948_SENSOR_STEP_DETECTOR:                 return ANDROID_SENSOR_STEP_DETECTOR;
	case INV_ICM20948_SENSOR_STEP_COUNTER:                  return ANDROID_SENSOR_STEP_COUNTER;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:          return ANDROID_SENSOR_GAME_ROTATION_VECTOR;
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:               return ANDROID_SENSOR_ROTATION_VECTOR;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:   return ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:             return ANDROID_SENSOR_GEOMAGNETIC_FIELD;
	case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:     return ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
	case INV_ICM20948_SENSOR_FLIP_PICKUP:                   return ANDROID_SENSOR_FLIP_PICKUP;
	case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:          return ANDROID_SENSOR_WAKEUP_TILT_DETECTOR;
	case INV_ICM20948_SENSOR_GRAVITY:                       return ANDROID_SENSOR_GRAVITY;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:           return ANDROID_SENSOR_LINEAR_ACCELERATION;
	case INV_ICM20948_SENSOR_ORIENTATION:                   return ANDROID_SENSOR_ORIENTATION;
	case INV_ICM20948_SENSOR_B2S:                           return ANDROID_SENSOR_B2S;
	default:                                                return ANDROID_SENSOR_NUM_MAX;
	}
}
#endif
static unsigned char sensor_needs_compassx(unsigned char androidSensor)
{
	switch(androidSensor) {
		case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
		case ANDROID_SENSOR_ROTATION_VECTOR:
		case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
		case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
		case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
			return 1;

		default :
			return 0;
	}
}

// BAC ped y ration for wearable, the value will influence pedometer result
#define BAC_PED_Y_RATIO_WEARABLE 1073741824

// Determine the fastest ODR for all gravity-based sensors
#define AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_GRAVITY)) \
		newOdr = MIN(s->sGravityOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_GAME_ROTATION_VECTOR)) \
		newOdr = MIN(s->sGrvOdrMs,newOdr);  \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_LINEAR_ACCELERATION)) \
		newOdr = MIN(s->sLinAccOdrMs,newOdr);
#define AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_GRAVITY)) \
		newOdr = MIN(s->sGravityWuOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR)) \
		newOdr = MIN(s->sGrvWuOdrMs,newOdr);  \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION)) \
		newOdr = MIN(s->sLinAccWuOdrMs,newOdr);

// Determine the fastest ODR for all rotation vector-based sensors
#define AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_ORIENTATION)) \
		newOdr = MIN(s->sOriOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_ROTATION_VECTOR)) \
		newOdr = MIN(s->sRvOdrMs,newOdr);
#define AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_ORIENTATION)) \
		newOdr = MIN(s->sOriWuOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR)) \
		newOdr = MIN(s->sRvWuOdrMs,newOdr);

void inv_icm20948_augmented_sensors_update_odrx(struct inv_icm20948 * s, unsigned char androidSensor, unsigned short * updatedDelayPtr)
{
	unsigned short lDelayInMs = 0xFFFF; // max value of uint16_t, so that we can get min value of all enabled sensors
	switch(androidSensor)
	{
		case ANDROID_SENSOR_GRAVITY:
        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_LINEAR_ACCELERATION:
			AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		case ANDROID_SENSOR_WAKEUP_GRAVITY:
        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
			AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		case ANDROID_SENSOR_ORIENTATION:
        case ANDROID_SENSOR_ROTATION_VECTOR:
			AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		case ANDROID_SENSOR_WAKEUP_ORIENTATION:
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
			AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		default :
			break;
	}

}

static void inv_reGenerate_sensorControl(struct inv_icm20948 * s, const short *sen_num_2_ctrl, unsigned short *sensor_control, uint8_t header2_count)
{
	short delta;
	int i, cntr;
	unsigned long tmp_androidSensorsOn_mask;

	//check if only header2 still remaining
	if(header2_count)
		*sensor_control = HEADER2_SET;
	else
		*sensor_control = 0;
	for (i = 0; i < 2; i++) {
		cntr = 32 * i;
		tmp_androidSensorsOn_mask = s->inv_androidSensorsOn_mask[i];
		while (tmp_androidSensorsOn_mask) {
			if (tmp_androidSensorsOn_mask & 1) {
				delta = sen_num_2_ctrl[cntr];
				if (delta != -1) *sensor_control |= delta;
			}
			tmp_androidSensorsOn_mask >>= 1;
			cntr++;
		}
	}
}

/** Computes the sensor control register that needs to be sent to the DMP
* @param[in] androidSensor A sensor number, the numbers correspond to sensors.h definition in Android
* @param[in] enable non-zero to turn sensor on, 0 to turn sensor off
* @param[in] sen_num_2_ctrl Table matching android sensor number to bits in DMP control register
* @param[in,out] sensor_control Sensor control register to write to DMP to enable/disable sensors
*/
static void inv_convert_androidSensor_to_control(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable, const short *sen_num_2_ctrl, unsigned short *sensor_control)
{
	short delta = 0;

	if (androidSensor == ANDROID_SENSOR_ACTIVITY_CLASSIFICATON || androidSensor == ANDROID_SENSOR_FLIP_PICKUP ||
			androidSensor == ANDROID_SENSOR_WAKEUP_TILT_DETECTOR || androidSensor == ANDROID_SENSOR_B2S) {
		if (enable) {
			*sensor_control |= HEADER2_SET;
			//we increment counter
			s->header2_count ++;
		}
		else {
			s->header2_count --;
			// control has to be regenerated when removing sensors because of overlap
			inv_reGenerate_sensorControl(s, sen_num_2_ctrl, sensor_control, s->header2_count);
		}
	}

	if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
		return; // Sensor not supported

	delta = sen_num_2_ctrl[androidSensor];
	if (delta == -1)
		return; // This sensor not supported

	if (enable) {
		s->inv_androidSensorsOn_mask[(androidSensor>>5)] |= 1L << (androidSensor & 0x1F); // Set bit
		*sensor_control |= delta;
	}
	else {
		s->inv_androidSensorsOn_mask[(androidSensor>>5)] &= ~(1L << (androidSensor & 0x1F)); // Clear bit
		// control has to be regenerated when removing sensors because of overlap
		inv_reGenerate_sensorControl(s, sen_num_2_ctrl, sensor_control, s->header2_count);
	}

	return;
}

typedef	struct {
	enum ANDROID_SENSORS AndroidSensor;
	enum INV_SENSORS     InvSensor;
}	MinDelayGenElementT;

#define MinDelayGen(s, list) MinDelayGenActual(s, list, sizeof(list) / sizeof (MinDelayGenElementT))

static unsigned short MinDelayGenActual(struct inv_icm20948 *s, const MinDelayGenElementT *element, unsigned long elementQuan)
{
	unsigned short minDelay = (unsigned short) -1;

	while(elementQuan--) {
		if (inv_icm20948_ctrl_androidSensor_enabled(s, element->AndroidSensor)) {
			unsigned short odrDelay = s->inv_dmp_odr_delays[element->InvSensor];

			if (minDelay > odrDelay)
					minDelay = odrDelay;
		}
		element++;
	} // end while elements to process

	return	minDelay;
}


/** @brief Get minimum ODR to be applied to accel engine based on all accel-based enabled sensors.
* @return ODR in ms we expect to be applied to accel engine
*/
static unsigned short getMinDlyAccel(struct inv_icm20948 *s)
{
	const MinDelayGenElementT MinDelayGenAccelList[] ={
		{ANDROID_SENSOR_ACCELEROMETER,                      INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_RAW_ACCELEROMETER,                  INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_WAKEUP_ACCELEROMETER,               INV_SENSOR_WAKEUP_ACCEL         },
		{ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,        INV_SENSOR_GEOMAG               },
		{ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, INV_SENSOR_WAKEUP_GEOMAG        },
		{ANDROID_SENSOR_STEP_DETECTOR,                      INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_STEP_COUNTER,                       INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,               INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_STEP_COUNTER,                INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,          INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,               INV_SENSOR_WAKEUP_TILT_DETECTOR },
		{ANDROID_SENSOR_GRAVITY,                            INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,               INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,                INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_WAKEUP_GRAVITY,                     INV_SENSOR_WAKEUP_SIXQ_accel    },
		{ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,        INV_SENSOR_WAKEUP_SIXQ_accel    },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         INV_SENSOR_WAKEUP_SIXQ_accel    },
		{ANDROID_SENSOR_ORIENTATION,                        INV_SENSOR_NINEQ_accel          },
		{ANDROID_SENSOR_ROTATION_VECTOR,                    INV_SENSOR_NINEQ_accel          },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,                 INV_SENSOR_WAKEUP_NINEQ_accel   },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             INV_SENSOR_WAKEUP_NINEQ_accel   }
	};

	unsigned short lMinOdr = MinDelayGen(s, MinDelayGenAccelList);

	if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER))
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(s->odr_acc_ms,s->odr_racc_ms);
		else
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = s->odr_acc_ms;
	else
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = s->odr_racc_ms;

	if (s->bac_status != 0)
		lMinOdr = min(lMinOdr, s->inv_dmp_odr_delays[INV_SENSOR_ACTIVITY_CLASSIFIER]);
	if (s->flip_pickup_status != 0)
		lMinOdr = min(lMinOdr, s->inv_dmp_odr_delays[INV_SENSOR_FLIP_PICKUP]);
	if (s->b2s_status != 0)
		lMinOdr = min(lMinOdr, s->inv_dmp_odr_delays[INV_SENSOR_BRING_TO_SEE]);

	/** To have correct algorithm performance and quick convergence of GMRV, it is advised to set accelerometer to 225Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR)
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) )
		lMinOdr = min(lMinOdr, 5);

	/** To have correct algorithm performance and quick convergence of RV, it is advised to set accelerometer to 225Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR)
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) )
		lMinOdr = min(lMinOdr, 5);

	return lMinOdr;
}

/** @brief Get minimum ODR to be applied to gyro engine based on all gyro-based enabled sensors.
* @return ODR in ms we expect to be applied to gyro engine
*/
static unsigned short getMinDlyGyro(struct inv_icm20948 *s)
{
	const MinDelayGenElementT MinDelayGenGyroList[] = {
		{ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,        INV_SENSOR_GYRO              },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED, INV_SENSOR_WAKEUP_GYRO       },
		{ANDROID_SENSOR_GYROSCOPE,                     INV_SENSOR_CALIB_GYRO        },
		{ANDROID_SENSOR_RAW_GYROSCOPE,                 INV_SENSOR_GYRO              },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE,              INV_SENSOR_WAKEUP_CALIB_GYRO },
		{ANDROID_SENSOR_GRAVITY,                       INV_SENSOR_SIXQ              },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,          INV_SENSOR_SIXQ              },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,           INV_SENSOR_SIXQ              },
		{ANDROID_SENSOR_WAKEUP_GRAVITY,                INV_SENSOR_WAKEUP_SIXQ       },
		{ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,   INV_SENSOR_WAKEUP_SIXQ       },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,    INV_SENSOR_WAKEUP_SIXQ       },
		{ANDROID_SENSOR_ORIENTATION,                   INV_SENSOR_NINEQ             },
		{ANDROID_SENSOR_ROTATION_VECTOR,               INV_SENSOR_NINEQ             },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,            INV_SENSOR_WAKEUP_NINEQ      },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,        INV_SENSOR_WAKEUP_NINEQ      }
	};

	unsigned short lMinOdr = MinDelayGen(s, MinDelayGenGyroList);

	if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED))
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(s->odr_gyr_ms,s->odr_rgyr_ms);
		else
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = s->odr_gyr_ms;
	else
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = s->odr_rgyr_ms;

	/** To have correct algorithm performance and quick convergence of RV, it is advised to set gyro to 225Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR)
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) )
		lMinOdr	= min(lMinOdr, 5);

	return lMinOdr;
}

/** @brief Get minimum ODR to be applied to compass engine based on all compass-based enabled sensors.
* @return ODR in ms we expect to be applied to compass engine
*/
static unsigned short getMinDlyCompass(struct inv_icm20948 *s)
{
	const MinDelayGenElementT MinDelayGenCpassList[] = {
		{ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,        INV_SENSOR_COMPASS              },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED, INV_SENSOR_WAKEUP_COMPASS       },
		{ANDROID_SENSOR_GEOMAGNETIC_FIELD,                  INV_SENSOR_CALIB_COMPASS        },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,              INV_SENSOR_WAKEUP_CALIB_COMPASS },
		{ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,        INV_SENSOR_GEOMAG_cpass         },
		{ANDROID_SENSOR_ORIENTATION,                        INV_SENSOR_NINEQ_cpass          },
		{ANDROID_SENSOR_ROTATION_VECTOR,                    INV_SENSOR_NINEQ_cpass          },
		{ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, INV_SENSOR_WAKEUP_GEOMAG_cpass  },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,                 INV_SENSOR_WAKEUP_NINEQ_cpass   },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             INV_SENSOR_WAKEUP_NINEQ_cpass   }
	};

	unsigned short lMinOdr = MinDelayGen(s, MinDelayGenCpassList);

	/** To have correct algorithm performance and quick convergence of GMRV, it is advised to set compass to 70Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR)
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) )
		lMinOdr= min(lMinOdr, 15);
	/** To have correct algorithm performance and quick convergence of RV, it is advised to set compass to 35Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR)
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) )
		lMinOdr = min(lMinOdr, 28);

	return lMinOdr;
}

static short get_multiple_56_rate(unsigned short delayInMs)
{
	short lfreq = 0;

	// > 1KHz
	if( delayInMs < 2 ){
	lfreq = DMP_ALGO_FREQ_900;
	}
	// 225Hz - 500Hz
	else if(( delayInMs >= 2 ) && ( delayInMs < 4 )){
	lfreq = DMP_ALGO_FREQ_450;
	}
	// 112Hz - 225Hz
	else if(( delayInMs >= 4 ) && ( delayInMs < 8 )){
	lfreq = DMP_ALGO_FREQ_225;
	}
	// 56Hz - 112Hz
	else if(( delayInMs >= 8 ) && ( delayInMs < 17 )){
	lfreq = DMP_ALGO_FREQ_112;
	}
	// < 56Hz
	else if(delayInMs >= 17){
	lfreq = DMP_ALGO_FREQ_56;
	}

	return lfreq;
}

static int DividerRateSet(struct inv_icm20948 *s, unsigned short minDelay, unsigned short hwSampleRateDivider, enum INV_SENSORS InvSensor)
{
	int result = 0;

	if (minDelay != 0xFFFF) {
		unsigned short dmpOdrDivider = (minDelay * 1125L) / (hwSampleRateDivider * 1000L); // a divider from (1125Hz/hw_smplrt_divider).

		s->inv_dmp_odr_dividers[InvSensor] = hwSampleRateDivider * dmpOdrDivider;
		result |= dmp_icm20948_set_sensor_rate(s, InvSensor, (dmpOdrDivider - 1));
	}

	return result;
}

static unsigned short SampleRateDividerGet(unsigned short minDelay)
{
	unsigned short delay = min(INV_ODR_MIN_DELAY, minDelay); // because of GYRO_SMPLRT_DIV which relies on 8 bits, we can't have ODR value higher than 200ms
	return delay * 1125L / 1000L; // a divider from 1125Hz.
}

static int inv_set_hw_smplrt_dmp_odrs(struct inv_icm20948 * s)
{
	int result = 0;
	unsigned short minDly, minDly_accel, minDly_gyro;
	unsigned short minDly_cpass;
	unsigned short minDly_pressure;
	unsigned short hw_smplrt_divider = 0;

	const MinDelayGenElementT MinDelayGenPressureList[] = {
		{ANDROID_SENSOR_PRESSURE,                           INV_SENSOR_PRESSURE             },
		{ANDROID_SENSOR_WAKEUP_PRESSURE,                    INV_SENSOR_WAKEUP_PRESSURE      }
	};
	const MinDelayGenElementT MinDelayGenAccel2List[] = {
		{ANDROID_SENSOR_ACCELEROMETER,                      INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_WAKEUP_ACCELEROMETER,               INV_SENSOR_WAKEUP_ACCEL         },
		{ANDROID_SENSOR_RAW_ACCELEROMETER,                  INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,                INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         INV_SENSOR_WAKEUP_SIXQ_accel    }
	};
	const MinDelayGenElementT MinDelayGenAccel3List[] = {
		{ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,        INV_SENSOR_GEOMAG               },
		{ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, INV_SENSOR_WAKEUP_GEOMAG        }
	};
	const MinDelayGenElementT MinDelayGenAccel4List[] = {
		{ANDROID_SENSOR_STEP_DETECTOR,                      INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_STEP_COUNTER,                       INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,               INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_STEP_COUNTER,                INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,          INV_SENSOR_WAKEUP_STEP_COUNTER  }
	};
	const MinDelayGenElementT MinDelayGenGyro2List[] = {
		{ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,             INV_SENSOR_GYRO                 },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,      INV_SENSOR_WAKEUP_GYRO          },
		{ANDROID_SENSOR_GYROSCOPE,                          INV_SENSOR_CALIB_GYRO           },
		{ANDROID_SENSOR_RAW_GYROSCOPE,                      INV_SENSOR_GYRO           },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE,                   INV_SENSOR_WAKEUP_CALIB_GYRO    }
	};
	const MinDelayGenElementT MinDelayGenGyro3List[] = {
		{ANDROID_SENSOR_GYROSCOPE,                          INV_SENSOR_CALIB_GYRO           },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE,                   INV_SENSOR_WAKEUP_CALIB_GYRO    }
	};
	const MinDelayGenElementT MinDelayGenGyro4List[] = {
		{ANDROID_SENSOR_GRAVITY,                            INV_SENSOR_SIXQ                 },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,               INV_SENSOR_SIXQ                 },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,                INV_SENSOR_SIXQ                 },
		{ANDROID_SENSOR_WAKEUP_GRAVITY,                     INV_SENSOR_WAKEUP_SIXQ          },
		{ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,        INV_SENSOR_WAKEUP_SIXQ          },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         INV_SENSOR_WAKEUP_SIXQ          }
	};
	const MinDelayGenElementT MinDelayGenGyro5List[] = {
		{ANDROID_SENSOR_ORIENTATION,                        INV_SENSOR_NINEQ                },
		{ANDROID_SENSOR_ROTATION_VECTOR,                    INV_SENSOR_NINEQ                },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,                 INV_SENSOR_WAKEUP_NINEQ         },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             INV_SENSOR_WAKEUP_NINEQ         }
	};
	const MinDelayGenElementT MinDelayGenCpass2List[] = {
		{ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,        INV_SENSOR_COMPASS              },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,	INV_SENSOR_WAKEUP_COMPASS       }
	};
	const MinDelayGenElementT MinDelayGenCpass3List[] = {
		{ANDROID_SENSOR_GEOMAGNETIC_FIELD,                  INV_SENSOR_CALIB_COMPASS        },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,              INV_SENSOR_WAKEUP_CALIB_COMPASS }
	};
	const MinDelayGenElementT MinDelayGenPressure2List[] = {
		{ANDROID_SENSOR_PRESSURE,                           INV_SENSOR_PRESSURE             },
		{ANDROID_SENSOR_WAKEUP_PRESSURE,                    INV_SENSOR_WAKEUP_PRESSURE      }
	};

	// Engine ACCEL Based
	minDly_accel = getMinDlyAccel(s);

	// Engine Gyro Based
	minDly_gyro  = getMinDlyGyro(s);

	// Engine Cpass Based
	minDly_cpass = getMinDlyCompass(s);

	// Engine Pressure Based
	minDly_pressure	=	MinDelayGen	(s, MinDelayGenPressureList);

	// get min delay of all enabled sensors of all sensor engine groups
	minDly = min(minDly_gyro, minDly_accel);
	minDly = min(minDly, minDly_cpass);
	minDly = min(minDly, minDly_pressure);

	// switch between low power and low noise at 500Hz boundary
	if (minDly != 0xFFFF) {
		// above 500Hz boundary, force LN mode
		if (minDly==1) {
			if (s->base_state.chip_lp_ln_mode == CHIP_LOW_POWER_ICM20948) {
				s->go_back_lp_when_odr_low = 1;
				inv_icm20948_enter_low_noise_mode(s);
			}
		} else { // below 500 Hz boundary, go back to originally requested mode
			if (s->go_back_lp_when_odr_low) {
				s->go_back_lp_when_odr_low = 0;
				inv_icm20948_enter_duty_cycle_mode(s);
			}
		}
	} else // all sensors are turned OFF, force originally requested mode
	{
		if (s->go_back_lp_when_odr_low) {
			s->go_back_lp_when_odr_low = 0;
			inv_icm20948_enter_duty_cycle_mode(s);
		}
	}

	if (minDly_accel != 0xFFFF)    minDly_accel = minDly;
	if (minDly_gyro  != 0xFFFF)    minDly_gyro  = minDly;
	if (minDly_cpass != 0xFFFF)    minDly_cpass = minDly;
	if (minDly_pressure != 0xFFFF) minDly_pressure = minDly;

	if (s->bac_request != 0) {
		unsigned short lBACMinDly = min(INV_ODR_DEFAULT_BAC, minDly_accel);
		// estimate closest decimator value to have 56Hz multiple and apply it
		lBACMinDly = 1000/(get_multiple_56_rate(lBACMinDly));
		dmp_icm20948_set_bac_rate(s, get_multiple_56_rate(lBACMinDly));
		minDly_accel = lBACMinDly;
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);
		result |= DividerRateSet(s, lBACMinDly, hw_smplrt_divider, INV_SENSOR_ACTIVITY_CLASSIFIER);
	}
	if (s->b2s_status != 0) {
		unsigned short lB2SMinDly = min(INV_ODR_DEFAULT_B2S, minDly_accel);
		lB2SMinDly = 1000/(get_multiple_56_rate(lB2SMinDly));
		dmp_icm20948_set_b2s_rate(s, get_multiple_56_rate(lB2SMinDly));
		minDly_accel = lB2SMinDly;
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);
		result |= DividerRateSet(s, lB2SMinDly, hw_smplrt_divider, INV_SENSOR_BRING_TO_SEE);
	}

	// set odrs for each enabled sensors

	// Engine ACCEL Based
	if (minDly_accel != 0xFFFF)	{ // 0xFFFF -- none accel based sensor enable
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);

		if (hw_smplrt_divider != s->lLastHwSmplrtDividerAcc) {

			result |= inv_icm20948_ctrl_set_accel_quaternion_gain(s, hw_smplrt_divider);
			result |= inv_icm20948_ctrl_set_accel_cal_params(s, hw_smplrt_divider);
			result |= inv_icm20948_set_accel_divider(s, hw_smplrt_divider - 1);
			s->lLastHwSmplrtDividerAcc = hw_smplrt_divider;
		}

		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel2List), hw_smplrt_divider, INV_SENSOR_ACCEL);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel3List), hw_smplrt_divider, INV_SENSOR_GEOMAG);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel4List), hw_smplrt_divider, INV_SENSOR_STEP_COUNTER);

	}

	// Engine Gyro Based
	if (minDly_gyro != 0xFFFF) { // 0xFFFF -- none gyro based sensor enable
		hw_smplrt_divider = SampleRateDividerGet(minDly_gyro);

		if (hw_smplrt_divider != s->lLastHwSmplrtDividerGyr) {
			result |= inv_icm20948_set_gyro_divider(s, (unsigned char)(hw_smplrt_divider - 1));
			s->lLastHwSmplrtDividerGyr = hw_smplrt_divider;
		}

		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro2List), hw_smplrt_divider, INV_SENSOR_GYRO);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro3List), hw_smplrt_divider, INV_SENSOR_CALIB_GYRO);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro4List), hw_smplrt_divider, INV_SENSOR_SIXQ);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro5List), hw_smplrt_divider, INV_SENSOR_NINEQ);
	}

	// Engine Cpass and Pressure Based
	if ((minDly_cpass != 0xFFFF) || (minDly_pressure != 0xFFFF)) {
		unsigned int lI2cEffectiveDivider = 0;

		// if compass or pressure are alone, compute 1st stage divider, otherwise it will be taken from accel or gyro
		if ( (minDly_accel == 0xFFFF) && (minDly_gyro == 0xFFFF) )
			hw_smplrt_divider = SampleRateDividerGet(minDly);

		// Apply compass or pressure ODR to I2C and get effective ODR
		// so that 2nd level of divider can take into account real frequency we can expect
		// to determine its divider value
		result |= inv_icm20948_secondary_set_odr(s, hw_smplrt_divider, &lI2cEffectiveDivider);

		// if compass or pressure are alone, recompute 1st stage divider based on configured divider for I2C
		// otherwise divider is taken from accel or gyro, so there is no need to recompute effective divider value
		// based on the divider we just applied
		if ( (minDly_accel == 0xFFFF) && (minDly_gyro == 0xFFFF) )
			hw_smplrt_divider = lI2cEffectiveDivider;

		if (minDly_cpass != 0xFFFF) {
			result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenCpass2List), hw_smplrt_divider, INV_SENSOR_COMPASS);
			result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenCpass3List), hw_smplrt_divider, INV_SENSOR_CALIB_COMPASS);
		}

		if (minDly_pressure != 0xFFFF)
			result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenPressure2List), hw_smplrt_divider, INV_SENSOR_PRESSURE);
	}

	return result;
}

static int inv_enable_sensor_internalx(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep)
{
	int result = 0;
	unsigned short inv_event_control = 0;
	unsigned short data_rdy_status = 0;
	unsigned long steps=0;
	const short inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX]=
	{
		// Unsupported Sensors are -1
		-1, // Meta Data
		-32760, //0x8008, // Accelerometer
		0x0028, // Magnetic Field
		0x0408, // Orientation
		0x4048, // Gyroscope
		0x1008, // Light
		0x0088, // Pressure
		-1, // Temperature
		-1, // Proximity <----------- fixme
		0x0808, // Gravity
		-30712, // 0x8808, // Linear Acceleration
		0x0408, // Rotation Vector
		-1, // Humidity
		-1, // Ambient Temperature
		0x2008, // Magnetic Field Uncalibrated
		0x0808, // Game Rotation Vector
		0x4008, // Gyroscope Uncalibrated
		0, // Significant Motion
		0x0018, // Step Detector
		0x0010, // Step Counter <----------- fixme
		0x0108, // Geomagnetic Rotation Vector
		-1, //ANDROID_SENSOR_HEART_RATE,
		-1, //ANDROID_SENSOR_PROXIMITY,

		-32760, // ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
		0x0028, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
		0x0408, // ANDROID_SENSOR_WAKEUP_ORIENTATION,
		0x4048, // ANDROID_SENSOR_WAKEUP_GYROSCOPE,
		0x1008, // ANDROID_SENSOR_WAKEUP_LIGHT,
		0x0088, // ANDROID_SENSOR_WAKEUP_PRESSURE,
		0x0808, // ANDROID_SENSOR_WAKEUP_GRAVITY,
		-30712, // ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
		0x0408, // ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
		-1,		// ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
		-1,		// ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
		0x2008, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
		0x0808, // ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
		0x4008, // ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
		0x0018, // ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
		0x0010, // ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
		0x0108, // ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
		-1,		// ANDROID_SENSOR_WAKEUP_HEART_RATE,
		0,		// ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
		(short)0x8008, // Raw Acc
		0x4048, // Raw Gyr
	};
	if(enable && !inv_icm20948_ctrl_androidSensor_enabled(s, androidSensor))
		s->skip_sample[inv_icm20948_sensor_android_2_sensor_type(androidSensor)] = 1;

	if (androidSensor == ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION) {
		if (enable) {
			s->smd_status = INV_SMD_EN;
			s->bac_request ++;
		}
		else {
			s->smd_status = 0;
			s->bac_request --;
		}
	}

	if (androidSensor == ANDROID_SENSOR_STEP_DETECTOR) {
		if (enable) {
			s->ped_int_status = INV_PEDOMETER_INT_EN;
			s->bac_request ++;
		}
		else {
			s->ped_int_status = 0;
			s->bac_request --;
		}
	}

	if (androidSensor == ANDROID_SENSOR_STEP_COUNTER) {
		if (enable) {
			s->bac_request ++;
		}
		else {
			s->bac_request --;
		}
	}

	if (androidSensor == ANDROID_SENSOR_FLIP_PICKUP) {
		if (enable){
			s->flip_pickup_status = FLIP_PICKUP_SET;
		}
		else
			s->flip_pickup_status = 0;
	}

	if (androidSensor == ANDROID_SENSOR_B2S) {
		if(enable){
			s->b2s_status = INV_BTS_EN;
			s->bac_request ++;
		}
		else {
			s->b2s_status = 0;
			s->bac_request --;
		}
	}
	if (androidSensor == ANDROID_SENSOR_ACTIVITY_CLASSIFICATON)
		inv_icm20948_ctrl_enable_activity_classifier(s, enable);

	if (androidSensor == ANDROID_SENSOR_WAKEUP_TILT_DETECTOR)
		inv_icm20948_ctrl_enable_tilt(s, enable);

	inv_convert_androidSensor_to_control(s, androidSensor, enable, inv_androidSensor_to_control_bits, &s->inv_sensor_control);
	result = dmp_icm20948_set_data_output_control1(s, s->inv_sensor_control);
	if (s->b2s_status)
		result |= dmp_icm20948_set_data_interrupt_control(s, s->inv_sensor_control|0x8008);
		// result |= dmp_icm20948_set_data_interrupt_control(s, s->inv_sensor_control|0x0000);
	else
		result |= dmp_icm20948_set_data_interrupt_control(s, s->inv_sensor_control);

	if (s->inv_sensor_control & ACCEL_SET)
		s->inv_sensor_control2 |= ACCEL_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~ACCEL_ACCURACY_SET;

	if ((s->inv_sensor_control & GYRO_CALIBR_SET) || (s->inv_sensor_control & GYRO_SET))
		s->inv_sensor_control2 |= GYRO_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~GYRO_ACCURACY_SET;

	if ((s->inv_sensor_control & CPASS_CALIBR_SET) || (s->inv_sensor_control & QUAT9_SET)
		|| (s->inv_sensor_control & GEOMAG_SET) || (s->inv_sensor_control & CPASS_SET))
		s->inv_sensor_control2 |= CPASS_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~CPASS_ACCURACY_SET;

	if(s->flip_pickup_status)
		s->inv_sensor_control2 |= FLIP_PICKUP_SET;
	else
		s->inv_sensor_control2 &= ~FLIP_PICKUP_SET;

	// inv_event_control   |= s->b2s_status;
	if(s->b2s_status)
	{
		inv_event_control |= INV_BRING_AND_LOOK_T0_SEE_EN;
		inv_event_control |= INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines change BAC behavior to wearable platform
		inv_event_control |= INV_BAC_WEARABLE_EN;
		dmp_icm20948_set_ped_y_ratio(s, BAC_PED_Y_RATIO_WEARABLE);
#endif
	}
	else
	{
		inv_event_control &= ~INV_BRING_AND_LOOK_T0_SEE_EN;
		inv_event_control &= ~INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines change BAC behavior to wearable platform
		inv_event_control &= ~INV_BAC_WEARABLE_EN;
#endif
	}

	result |= dmp_icm20948_set_data_output_control2(s, s->inv_sensor_control2);

	// sets DATA_RDY_STATUS in DMP based on which sensors are on
	if (s->inv_androidSensorsOn_mask[0] & INV_NEEDS_GYRO_MASK || s->inv_androidSensorsOn_mask[1] & INV_NEEDS_GYRO_MASK1)
		data_rdy_status |= GYRO_AVAILABLE;

	if (s->inv_androidSensorsOn_mask[0] & INV_NEEDS_ACCEL_MASK || s->inv_androidSensorsOn_mask[1] & INV_NEEDS_ACCEL_MASK1)
		data_rdy_status |= ACCEL_AVAILABLE;

	if (s->flip_pickup_status || s->b2s_status)
		data_rdy_status |= ACCEL_AVAILABLE;

	if (s->bac_status)
		data_rdy_status |= ACCEL_AVAILABLE;

	if (s->inv_androidSensorsOn_mask[0] & INV_NEEDS_COMPASS_MASK || s->inv_androidSensorsOn_mask[1] & INV_NEEDS_COMPASS_MASK1) {
		data_rdy_status |= SECONDARY_COMPASS_AVAILABLE;
		inv_event_control |= INV_COMPASS_CAL_EN;
	}
	// turn on gyro cal only if gyro is available
	if (data_rdy_status & GYRO_AVAILABLE)
		inv_event_control |= INV_GYRO_CAL_EN;

	// turn on acc cal only if acc is available
	if (data_rdy_status & ACCEL_AVAILABLE)
		inv_event_control |= INV_ACCEL_CAL_EN;

	inv_event_control |= s->smd_status | s->ped_int_status;

	if (s->inv_sensor_control & QUAT9_SET)
		inv_event_control |= INV_NINE_AXIS_EN;

	if (s->inv_sensor_control & (PED_STEPDET_SET | PED_STEPIND_SET) || inv_event_control & INV_SMD_EN) {
		inv_event_control |= INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines change BAC behavior to wearable platform
		inv_event_control |= INV_BAC_WEARABLE_EN;
		dmp_icm20948_set_ped_y_ratio(s, BAC_PED_Y_RATIO_WEARABLE);
#endif
	}

	if (s->inv_sensor_control2 & ACT_RECOG_SET) {
		inv_event_control |= INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines this to change BAC behavior to wearable platform
		inv_event_control |= INV_BAC_WEARABLE_EN;
		dmp_icm20948_set_ped_y_ratio(s, BAC_PED_Y_RATIO_WEARABLE);
#endif
	}

	if (s->inv_sensor_control2 & FLIP_PICKUP_SET){
		inv_event_control |= FLIP_PICKUP_EN;
	}

	if (s->inv_sensor_control & GEOMAG_SET)
		inv_event_control |= GEOMAG_EN;

	result |= dmp_icm20948_set_motion_event_control(s, inv_event_control);

	// A sensor was just enabled/disabled, need to recompute the required ODR for all augmented sensor-related sensors
	// The fastest ODR will always be applied to other related sensors
	if (   (androidSensor == ANDROID_SENSOR_GRAVITY)
		|| (androidSensor == ANDROID_SENSOR_GAME_ROTATION_VECTOR)
		|| (androidSensor == ANDROID_SENSOR_LINEAR_ACCELERATION) ) {
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_SIXQ]);
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_SIXQ_accel]);
	}

	if (   (androidSensor == ANDROID_SENSOR_ORIENTATION)
		|| (androidSensor == ANDROID_SENSOR_ROTATION_VECTOR) ) {
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_NINEQ]);
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_NINEQ_accel]);
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_NINEQ_cpass]);
	}

	if (   (androidSensor == ANDROID_SENSOR_WAKEUP_GRAVITY)
		|| (androidSensor == ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR)
		|| (androidSensor == ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION) ) {
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ]);
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ_accel]);
	}

	if (   (androidSensor == ANDROID_SENSOR_WAKEUP_ORIENTATION)
		|| (androidSensor == ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) ) {
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ]);
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_accel]);
		inv_icm20948_augmented_sensors_update_odrx(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_cpass]);
	}

	result |= inv_set_hw_smplrt_dmp_odrs(s);
	result |= inv_icm20948_set_gyro_sf(s, inv_icm20948_get_gyro_divider(s), inv_icm20948_get_gyro_fullscale(s));

	if (!s->inv_sensor_control && !(s->inv_androidSensorsOn_mask[0] & (1L << ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION)) && !s->b2s_status) {
		*mems_put_to_sleep =1 ;
		result |= inv_icm20948_sleep_mems(s);
	}

	// DMP no longer controls PWR_MGMT_2 because of hardware bug, 0x80 set to override default behaviour of inv_icm20948_enable_hw_sensors()
	result |= inv_icm20948_enable_hw_sensors(s, (int)data_rdy_status | 0x80);

	// set DATA_RDY_STATUS in DMP
	if (data_rdy_status & SECONDARY_COMPASS_AVAILABLE)	{
		data_rdy_status |= SECONDARY_COMPASS_AVAILABLE;
	}

	result |= dmp_icm20948_set_data_rdy_status(s, data_rdy_status);

	// To have the all steps when you enable the sensor
	if (androidSensor == ANDROID_SENSOR_STEP_COUNTER)
	{
		if (enable)
		{
			dmp_icm20948_get_pedometer_num_of_steps(s, &steps);
			s->sStepCounterToBeSubtracted = steps - s->sOldSteps;
		}
	}

	return result;
}

int inv_icm20948_ctrl_enable_sensorx(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable)
{
	int result = 0;

	if(sensor_needs_compassx(androidSensor))
		if(!inv_icm20948_get_compass_availability(s))
			return -1;

	inv_icm20948_prevent_lpen_control(s);
	if( s->mems_put_to_sleep ) {
		s->mems_put_to_sleep = 0;
		result |= inv_icm20948_wakeup_mems(s);
	}
	result |= inv_enable_sensor_internalx(s, androidSensor, enable, &s->mems_put_to_sleep);
	inv_icm20948_allow_lpen_control(s);
	return result;
}
#endif
bool ImuIcm20948::Enable()
{
	uint8_t d, userctrl;
	uint16_t dout;
	bool res = vpIcm->Enable();

	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	d = vpIcm->Read8((uint8_t*)&regaddr, 2);
	d |= ICM20948_USER_CTRL_DMP_EN | ICM20948_USER_CTRL_FIFO_EN;
	vpIcm->Write8((uint8_t*)&regaddr, 2, d);

	dout = EndianCvt16(dout);
	regaddr = ICM20948_DMP_DATA_OUT_CTL1_REG;
	WriteDMP(regaddr, (uint8_t*)&dout, 2);

	regaddr = ICM20948_FIFO_EN_2_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_EN_2_FIFO_EN_ALL);

#if 0
	int i = INV_ICM20948_SENSOR_MAX + 1;//INV_SENSOR_TYPE_MAX;

	while(i-- > 0) {
		//inv_icm20948_enable_sensor(&vIcmDevice, (inv_icm20948_sensor)i, 1);
		//uint8_t androidSensor = sensor_type_2_android_sensorx((inv_icm20948_sensor)i);
		uint8_t androidSensor = s_InvSensor2AndroidSensor[i];
#if 0
		if(0!=inv_icm20948_ctrl_enable_sensorx(&vIcmDevice, androidSensor, 1))
		{
			//return 0;
		}
#else
		if(sensor_needs_compassx(androidSensor))
			if(!inv_icm20948_get_compass_availability(&vInvnDev))
			{
				continue;
			}

		inv_icm20948_prevent_lpen_control(&vInvnDev);
		if( vInvnDev.mems_put_to_sleep ) {
			vInvnDev.mems_put_to_sleep = 0;
		inv_icm20948_wakeup_mems(&vInvnDev);
		}
		inv_enable_sensor_internalx(&vInvnDev, androidSensor, 1, &vInvnDev.mems_put_to_sleep);
		//inv_icm20948_allow_lpen_control(&vIcmDevice);
#endif
	}
#endif
	return res;
}

void ImuIcm20948::Disable()
{
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);

	vpIcm->Write8((uint8_t*)&regaddr, 2, d);

	vpIcm->Disable();
}

void ImuIcm20948::Reset()
{
	vpIcm->Reset();
}

IMU_FEATURE ImuIcm20948::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	if (FeatureBit & IMU_FEATURE_EULER)
	{

	}

	if (FeatureBit & IMU_FEATURE_QUATERNION)
	{
		uint16_t f = ICM20948_DMP_QUAT9_SET;
		uint16_t m = ICM20948_DMP_DATA_OUT_CTL1_REG;
		WriteDMP(m, (uint8_t*)&f, 2);
	}

	if (FeatureBit & IMU_FEATURE_COMPASS)
	{

	}

	if (FeatureBit & IMU_FEATURE_GRAVITY)
	{

	}

	if (FeatureBit & IMU_FEATURE_EXTERNAL_ACCEL)
	{

	}

	if (FeatureBit & IMU_FEATURE_TAP)
	{

	}

	if (FeatureBit & IMU_FEATURE_ROTATION)
	{

	}

	if (FeatureBit & IMU_FEATURE_VIBRATION)
	{

	}

	if (FeatureBit & IMU_FEATURE_PEDOMETER)
	{

	}

	if (FeatureBit & IMU_FEATURE_CYCLING)
	{

	}

	return Imu::Feature(FeatureBit, bEnDis);
}

bool ImuIcm20948::Calibrate()
{
	return true;
}

void ImuIcm20948::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool ImuIcm20948::Compass(bool bEn)
{
	return true;
}

bool ImuIcm20948::Pedometer(bool bEn)
{
	return true;
}

bool ImuIcm20948::Quaternion(bool bEn, int NbAxis)
{
	Imu::Feature(IMU_FEATURE_QUATERNION, bEn);
	uint16_t f = ICM20948_DMP_QUAT9_SET;
	uint16_t m = ICM20948_DMP_DATA_OUT_CTL1_REG;

	if (NbAxis <9)
	{
		f = ICM20948_DMP_QUAT6_SET;
	}

	WriteDMP(m, (uint8_t*)&f, 2);

	if (vbIntEn)
	{
		m = ICM20948_DMP_DATA_INTR_CTL_REG;
		WriteDMP(m, (uint8_t*)&f, 2);
	}

	return true;
}

bool ImuIcm20948::Tap(bool bEn)
{
	return true;
}

bool ImuIcm20948::UpdateData()
{
	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}

	return true;
}
#if 0
void ImuIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
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
		//vAccData.X = event.data.acc.vect[0] * 256.0;
		//vAccData.Y = event.data.acc.vect[1] * 256.0;
		//vAccData.Z = event.data.acc.vect[2] * 256.0;
		//vAccData.Timestamp = timestamp;
		//printf("a %d : %d %d %d\r\n", vAccData.Timestamp, vAccData.X, vAccData.Y, vAccData.Z);
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
		memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
		break;
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
//		return;
		;
	}

	//vpIcm->UpdateData(sensortype, timestamp, data, arg);

	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}
#endif
void ImuIcm20948::IntHandler()
{
	uint16_t regaddr = ICM20948_INT_STATUS_REG;
	uint8_t status[4];
	uint8_t d;
	uint64_t t;

	vpIcm->Read((uint8_t*)&regaddr, 2, status, 4);

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	if (status[0] & ICM20948_INT_STATUS_DMP_INT1 || status[3])
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&distatus, 1);

		//g_Uart.printf("DMP int %x\r\n", distatus);
	//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			//printf("distatus %x\n", distatus);
		}
	}

	if (status[2])
	{
		ResetFifo();
		vFifoHdr = vFifoHdr2 = 0;
		vFifoDataLen = 0;

		return;
	}

	regaddr = ICM20948_FIFO_COUNTH_REG;
	size_t cnt = Read16((uint8_t*)&regaddr, 2);
	cnt = EndianCvt16(cnt);

	regaddr = ICM20948_FIFO_R_W_REG;
	uint8_t *p = &vFifo[vFifoDataLen];

	while (cnt > 0)//ICM20948_FIFO_PAGE_SIZE)
	{
		int l = min((size_t)ICM20948_FIFO_PAGE_SIZE, min(cnt, sizeof(vFifo) - vFifoDataLen));

		if (l == 0)
		{
			break;
		}
		l = Read((uint8_t*)&regaddr, 2, p, l);
		p += l;
		vFifoDataLen += l;
		cnt -= l;
	}

	p = vFifo;

	while (vFifoDataLen > 0)
	{
		if (vFifoHdr == 0 && vFifoHdr2 == 0)
		{
			int l = 0;

			// new packet
			vFifoHdr = ((uint16_t)p[0] << 8U) | ((uint16_t)p[1] & 0xFF);

			if ((vFifoHdr & ~ICM20948_FIFO_HEADER_MASK))
			{
				ResetFifo();
				vFifoDataLen = 0;
				vFifoHdr = 0;
				cnt = 0;
				return;
			}

			//vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;
			l = 2;

			if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
			{
				if (vFifoDataLen < 4)
				{
					vFifoHdr = 0;
					return;
				}
				vFifoHdr2 = ((uint16_t)p[2] << 8U) | ((uint16_t)p[3] & 0xFF);

				if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
				{
					ResetFifo();
					vFifoDataLen = 0;
					vFifoHdr = vFifoHdr2 = 0;
					cnt = 0;
					return;
				}

				l += 2;
				vFifoHdr &= ~ICM20948_FIFO_HEADER_HEADER2;
			}
			vFifoDataLen -= l;

			p += l;

		}
		int l = ProcessDMPFifo(p, vFifoDataLen, t);
		if (l == 0)
		{
			break;//return false;
		}
		vFifoDataLen -= l;
		p += l;
	}
	if (vFifoDataLen > 0 && p != vFifo)
	{
		memmove(vFifo, p, vFifoDataLen);
	}
}

int ImuIcm20948::ReadDMP(uint16_t MemAddr, uint8_t *pBuff, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	uint8_t *p = pBuff;
	MemAddr &= 0xFF;

	while (Len > 0)
	{
		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

		regaddr = ICM20948_DMP_MEM_RW_REG;

		int l = min(16, Len);
		l = Read((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
	}

	return Len;
}

int ImuIcm20948::WriteDMP(uint16_t MemAddr, uint8_t *pData, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	uint8_t *p = pData;
	MemAddr &= 0xFF;

	while (Len > 0)
	{
		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, MemAddr);

		regaddr = ICM20948_DMP_MEM_RW_REG;

		int l = min(16, Len);
		l = Write((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
		MemAddr += l;
	}

	return Len;
}

size_t ImuIcm20948::ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp)
{
	size_t cnt = 0;
	uint8_t *d = pFifo;//[ICM20948_FIFO_PAGE_SIZE];

	if (vFifoHdr & ICM20948_FIFO_HEADER_ACCEL)
	{
		if (Len < ICM20948_FIFO_HEADER_ACCEL_SIZE)
		{
			return cnt;
		}

		((AccelIcm20948*)vpAccel)->UpdateData(Timestamp,
											  (((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)),
											  (((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)),
											  (((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)));

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

		((GyroIcm20948*)vpGyro)->UpdateData(Timestamp,
											(((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)),
											(((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)),
											(((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)));

		// TODO : Process gyro bias
		float bias[3];

		bias[0] = (float)(((uint16_t)d[6] << 8) | ((uint16_t)d[7] & 0xFF)) / (float)(1<<20);
		bias[1] = (float)(((uint16_t)d[8] << 8) | ((uint16_t)d[9] & 0xFF)) / (float)(1<<20);
		bias[2] = (float)(((uint16_t)d[10] << 8) | ((uint16_t)d[11] & 0xFF)) / (float)(1<<20);

		vpGyro->SetCalibrationOffset(bias);
		//bias[0] /= (1<<20);
		//bias[1] /= (1<<20);
		//bias[2] /= (1<<20);

		//GyroSensor::vData.X += bias[0];// / (1<<4);
		//GyroSensor::vData.Y += bias[1];// / (1<<4);
		//GyroSensor::vData.Z += bias[2];// / (1<<4);


		if (bias[0] > 0)
		{
		//	g_Uart.printf("bias : %d %d %d\r\n", bias[0], bias[1], bias[2]);
		}
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

		((MagIcm20948*)vpMag)->UpdateData(Timestamp,
										  (((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)),
										  (((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)),
										  (((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)));

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

		d += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT9_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT9; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PEDO_QUAT6)
	{
		if (Len < ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE;
		cnt += ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE;
		Len -= ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PEDO_QUAT6; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GEOMAG)
	{
		if (Len < ICM20948_FIFO_HEADER_GEOMAG_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		cnt += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		Len -= ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GEOMAG; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PRESS_TEMP)
	{
		if (Len < ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE;
		cnt += ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE;
		Len -= ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PRESS_TEMP; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_GYRO)
	{
		// Hardware unit scaled by 2^15
		// Although bit is set but no data, try to read data will corrupt fifo
#if 0
		if (Len < ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE;
#endif
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_GYRO; // Clear bit
	}
	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_CPASS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_STEP_DETECTOR)
	{
		// The unit is uT scaled by 2^16
		if (Len < ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		cnt += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		Len -= ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_STEP_DETECTOR; // Clear bit
	}

	if (vFifoHdr2 != 0)
	{
		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACCEL_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE)
			{
				return cnt;
			}

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

			d += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_CPASS_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_FSYNC)
		{
			if (Len < ICM20948_FIFO_HEADER2_FSYNC_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_FSYNC_SIZE;
			cnt += ICM20948_FIFO_HEADER2_FSYNC_SIZE;
			Len -= ICM20948_FIFO_HEADER2_FSYNC_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_FSYNC; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_PICKUP)
		{
			if (Len < ICM20948_FIFO_HEADER2_PICKUP_SIZE)
			{
				return cnt;
			}

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

			d += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACTI_RECOG; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_SECOND_ONOFF)
		{
			if (Len < ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			cnt += ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			Len -= ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_SECOND_ONOFF; // Clear bit
		}
	}

	if (Len < ICM20948_FIFO_FOOTER_SIZE)
	{
		vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;

		return cnt;
	}

	cnt += ICM20948_FIFO_FOOTER_SIZE;
	Len -= ICM20948_FIFO_FOOTER_SIZE;
	vFifoHdr &= ~ICM20948_FIFO_HEADER_FOOTER; // Clear bit

	vFifoHdr = vFifoHdr2 = 0;

	return cnt;
}

bool ImuIcm20948::InitDMP(uint16_t DmpStartAddr, const uint8_t * const pDmpImage, int Len)
{
	if (pDmpImage == NULL || Len == 0)
		return false;

	// Disable DMP & FIFO before FIFO can be reseted and DMP firmware loaded
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);
	Write8((uint8_t*)&regaddr,	2, d);
#if 0
	// Reset FIFO
	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0);
#else
	ResetFifo();
#endif

	// load external image
	bool res = UploadDMPImage(pDmpImage, Len);

	if (res)
	{
		DmpStartAddr = EndianCvt16(DmpStartAddr);

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH_REG;
		Write16((uint8_t*)&regaddr, 2, DmpStartAddr);

		regaddr = ICM20948_USER_CTRL_REG;
		d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		vbDmpEnabled = true;

		return true;
	}

	return false;
}

bool ImuIcm20948::UploadDMPImage(const uint8_t * const pDmpImage, int Len)
{
	int len = Len, l = 0;
	uint8_t *p = (uint8_t*)pDmpImage;
	uint16_t regaddr;
	uint16_t memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;
	size_t psize = ICM20948_DMP_MEM_BANK_SIZE;

	if (vpIcm->Interface()->Type() == DEVINTRF_TYPE_I2C)
	{
		psize = ICM20948_FIFO_PAGE_SIZE;
	}

	regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t pwrstate;

	pwrstate = Read8((uint8_t*)&regaddr, 2);

	// make sure it is on full power
	Write8((uint8_t*)&regaddr, 2, pwrstate & ~(ICM20948_PWR_MGMT_1_LP_EN | ICM20948_PWR_MGMT_1_SLEEP));

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

	l = min(len, ICM20948_FIFO_PAGE_SIZE - (memaddr % ICM20948_FIFO_PAGE_SIZE));

	while (len > 0)
	{
		regaddr = ICM20948_DMP_MEM_RW_REG;
		l = Write((uint8_t*)&regaddr, 2, p, l);

		p += l;
		memaddr += l;
		len -= l;

		regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

		l = min(len, ICM20948_FIFO_PAGE_SIZE);
	}

	len = Len;
	p = (uint8_t*)pDmpImage;
	memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;

	// Verify

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);
	l = min(len, ICM20948_FIFO_PAGE_SIZE - (memaddr % ICM20948_FIFO_PAGE_SIZE));

	while (len > 0)
	{
		uint8_t m[ICM20948_FIFO_PAGE_SIZE];

		regaddr = ICM20948_DMP_MEM_RW_REG;
		Read((uint8_t*)&regaddr, 2, m, l);

		if (memcmp(p, m, l) != 0)
		{
			return false;
		}

		p += l;
		memaddr += l;
		len -= l;

		regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

		l = min(len, ICM20948_FIFO_PAGE_SIZE);
	}

	// Restore power state
	Write8((uint8_t*)&regaddr, 2, pwrstate);

	return true;
}

