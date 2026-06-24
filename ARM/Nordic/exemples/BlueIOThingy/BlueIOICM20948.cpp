/*
 * BlueIOICM20948.cpp
 *
 *  Created on: Dec 24, 2018
 *      Author: hoan
 */
//#include "ble.h"
//#include "app_scheduler.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_intrf.h"
#include "app_evt_handler.h"
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"

#include "bluetooth/bt_app.h"
//#include "ble_app_nrf5.h"
#include "bluetooth/bt_gatt.h"
#include "idelay.h"
#include "motion/att.h"

//#define INVN
//#define IMU_FUSION_VQF
//#define IMU_FUSION_EQF

// Fusion backend test selector. Define one of:
//   IMU_FUSION_VQF - software VQF fusion (AhrsVqf)
//   IMU_FUSION_EQF - software EqF fusion (AhrsEqf), 6-axis, FPU targets only
// to run software fusion instead of the ICM20948 on-chip DMP. Both read raw
// data from the IOsonata AGM driver, so do not define INVN at the same time,
// and do not define both VQF and EqF together.

#ifdef INVN
#include "motion/mot_invn_icm20948.h"
#include "sensors/agm_invn_icm20948.h"
#else
#include "motion/mot_icm20948.h"
#include "sensors/agm_icm20948.h"
#endif

#ifdef IMU_FUSION_VQF
#include "motion/att_vqf.h"
#endif

#ifdef IMU_FUSION_EQF
#include "motion/att_eqf.h"
#endif

#include "BlueIOICM20948.h"
#include "BlueIOThingy.h"
#include "board.h"

void ImuRawDataSend(AccelSensorData_t &AccData, GyroSensorData_t GyroData, MagSensorData_t &MagData);
void ImuQuatDataSend(long Quat[4]);
static void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,	// 50Hz (in mHz)
	.Scale = 2,
	.FltrFreq = 50000,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 2000,	// +/-2000 dps full scale; 250 default saturates on fast motion
	.FltrFreq = 50000,	// enable gyro DLPF so GYRO_SMPLRT_DIV takes effect; without it the gyro free-runs at 9kHz and the DMP over-integrates
};

static const MagSensorCfg_t s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
};

#ifdef INVN
static AgmInvnIcm20948 s_MotSensor;
#else
static AgmIcm20948 s_MotSensor;
#endif

static const AhrsCfg_t s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

#if defined(IMU_FUSION_VQF)
static AhrsVqf s_Imu;
#elif defined(IMU_FUSION_EQF)
static AhrsEqf s_Imu;
#elif defined(INVN)
static AhrsInvnIcm20948 s_Imu;
#else
static AhrsIcm20948 s_Imu;
#endif

static Timer *s_pTimer = NULL;

#if 0
FusionAhrs ahrs;
FusionAhrsSettings ahrs_settings =  {
	.convention = FusionConventionNwu,
	.gain = 0.5f,
//	.gyroscopeRange = (float)pGyro->Sensitivity(),// 2000.0f, /* replace this with actual gyroscope range in degrees/s */
	.accelerationRejection = 10.0f,
	.magneticRejection = 10.0f,
	.recoveryTriggerPeriod = 5 * 50, /* 5 seconds */
};
#endif

//void ImuDataChedHandler(void * p_event_data, uint16_t event_size)

void ImuDataChedHandler(uint32_t Evt, void *pCtx)
{
	AccelSensorData_t accdata;
	GyroSensorData_t gyrodata;
	MagSensorData_t magdata;
	AhrsQuat_t quat;
	long q[4];

#if 1// INVN
		s_Imu.Read(accdata);
		s_Imu.Read(gyrodata);
		s_Imu.Read(magdata);
	ImuRawDataSend(accdata, gyrodata, magdata);
	s_Imu.Read(quat);

	q[0] = quat.Q[0] * (1 << 30);
	q[1] = quat.Q[1] * (1 << 30);
	q[2] = quat.Q[2] * (1 << 30);
	q[3] = quat.Q[3] * (1 << 30);
	//printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, q[0], q[1], q[2], q[3]);
#else
	s_MotSensor.Read(accdata);
	s_MotSensor.Read(gyrodata);
	s_MotSensor.Read(magdata);
	ImuRawDataSend(accdata, gyrodata, magdata);

    FusionVector gyroscope = {gyrodata.X, gyrodata.Y, gyrodata.Z}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {accdata.X, accdata.Y, accdata.Z}; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {magdata.X, magdata.Y, magdata.Z};

//    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.02);
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 0.02);
    FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs);
	q[0] = fq.array[0] * (1 << 30);
	q[1] = fq.array[1] * (1 << 30);
	q[2] = fq.array[2] * (1 << 30);
	q[3] = fq.array[3] * (1 << 30);
	//printf("F : %f %f %f %f\n", fq.array[0], fq.array[1], fq.array[2], fq.array[3]);
	//printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, q[0], q[1], q[2], q[3]);
	//printf("%f %f %f\n", accdata.X, accdata.Y, accdata.Z);
#endif
	//printf("I : %f %f %f %f\n", quat.Q[0], quat.Q[1], quat.Q[2], quat.Q[3]);
	ImuQuatDataSend(q);
}


static void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			// Defer the read and BLE send to the main loop. The FIFO drain
			// runs in the GPIOTE ISR; doing the send here too races vQuat
			// against the next sample and ties up the connection interval.
			AppEvtHandlerQue(0, 0, ImuDataChedHandler);
			break;
	}
}

uint64_t inv_icm20948_get_time_us(void)
{
	return s_pTimer ? s_pTimer->uSecond() : 0;
}
/*
void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
}
*/

void ICM20948IntHandler(int IntNo, void *pCtx)
{
	AccelSensorData_t accdata;
	GyroSensorData_t gyrodata;
	MagSensorData_t magdata;
	AhrsQuat_t quat;
	long q[4];

	if (IntNo == IMU_INT_NO)
	{
#if defined(IMU_FUSION_VQF) || defined(IMU_FUSION_EQF)
		// Software fusion test path (VQF or EqF). Refresh raw accel/gyro/mag
		// from the sensor FIFO, run the software fusion, then defer the BLE
		// send to the main loop. The on-chip DMP is not used here.
		s_MotSensor.UpdateData();
		s_Imu.UpdateData();
		AppEvtHandlerQue(0, 0, ImuDataChedHandler);
		return;
#elif 1
		s_Imu.IntHandler();	// Drives ImuEvtHandler -> ImuDataChedHandler once
		return;
#else
		s_MotSensor.IntHandler();
#endif
		//AppEvtHandlerQue(0, 0, ImuDataChedHandler);
#if 1
		s_Imu.Read(quat);

		q[0] = quat.Q[0] * (1 << 30);
		q[1] = quat.Q[1] * (1 << 30);
		q[2] = quat.Q[2] * (1 << 30);
		q[3] = quat.Q[3] * (1 << 30);
    	ImuQuatDataSend(q);
#else
		s_MotSensor.Read(accdata);
		s_MotSensor.Read(gyrodata);
		s_MotSensor.Read(magdata);

        FusionVector gyroscope = {gyrodata.X, gyrodata.Y, gyrodata.Z}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {accdata.X, accdata.Y, accdata.Z}; // replace this with actual accelerometer data in g
        FusionVector magnetometer = {magdata.X, magdata.Y, magdata.Z};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.02);
//        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 0.02);
        FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs);
    	ImuRawDataSend(accdata, gyrodata, magdata);

    	long q[4];
    	q[0] = fq.array[0] * (1 << 30);
    	q[1] = fq.array[1] * (1 << 30);
    	q[2] = fq.array[2] * (1 << 30);
    	q[3] = fq.array[3] * (1 << 30);
    	//printf("%f %f %f\n", accdata.X, accdata.Y, accdata.Z);
    	//printf("Quat %d: %d %d %d\r\n", q[0], q[1], q[2], q[3]);
    	ImuQuatDataSend(q);
#endif
	}
}

void ICM20948EnableFeature(uint32_t Feature)
{
	//s_MotionFeature |= Feature;
	s_MotSensor.Enable();
	//s_Imu.Enable();
}

bool ICM20948Init(DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	s_pTimer = pTimer;

	if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		s_AccelCfg.DevAddr = ICM20948_I2C_DEV_ADDR0;
	}

	bool res = s_MotSensor.Init(s_AccelCfg, pIntrf, pTimer);
	if (res == false)
	{
		return res;
	}
	res |= s_MotSensor.Init(s_GyroCfg, pIntrf, pTimer);
	if (res == true)
	{
		res |= s_MotSensor.Init(s_MagCfg, pIntrf, pTimer);
	}
	if (res == true)
	{
		res |= s_Imu.Init(s_ImuCfg, &s_MotSensor, &s_MotSensor, &s_MotSensor);
	}
#if 0
	if (res)
	{
//		s_MotSensor.Enable();
	//	s_Imu.Enable();
	    FusionAhrsInitialise(&ahrs);
/*	    ahrs_settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.gyroscopeRange = (float)pGyro->Sensitivity(),// 2000.0f, // replace this with actual gyroscope range in degrees/s
			.accelerationRejection = 10.0f,
			.magneticRejection = 10.0f,
			.recoveryTriggerPeriod = 5 * SAMPLE_RATE, // 5 seconds
	    };*/

	    ahrs_settings.gyroscopeRange = (float)((GyroSensor*)&s_MotSensor)->Sensitivity();
	    FusionAhrsSetSettings(&ahrs, &ahrs_settings);
	}
#endif
	if (res)
	{
		s_Imu.Enable();
		s_Imu.Quaternion(true, 6);	// TEST: 6-axis, no compass, to compare bad/fill rate
	}

	msDelay(100);
	IOPinConfig(IMU_INT_PORT, IMU_INT_PIN, IMU_INT_PINOP,
				IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL);
	IOPinEnableInterrupt(IMU_INT_NO, 6, IMU_INT_PORT,
						 IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION,
						 ICM20948IntHandler, NULL);

	return res;
}
