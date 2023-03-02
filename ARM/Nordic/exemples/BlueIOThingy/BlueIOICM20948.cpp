/*
 * BlueIOICM20948.cpp
 *
 *  Created on: Dec 24, 2018
 *      Author: hoan
 */
#include "ble.h"
#include "app_scheduler.h"

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

#define INVN

#ifdef INVN
#include "imu/imu_invn_icm20948.h"
#include "sensors/agm_invn_icm20948.h"
#else
#include "imu/imu_icm20948.h"
#include "sensors/agm_icm20948.h"
#endif

#include "BlueIOThingy.h"
#include "board.h"

void ImuRawDataSend(AccelSensorData_t &AccData, GyroSensorData_t GyroData, MagSensorData_t &MagData);
void ImuQuatDataSend(long Quat[4]);
static void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,	// 50Hz (in mHz)
	.Scale = 2,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
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

static const ImuCfg_t s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

#ifdef INVN
static ImuInvnIcm20948 s_Imu;
#else
static ImuIcm20948 s_Imu;
#endif

static Timer *s_pTimer = NULL;

void ImuDataChedHandler(void * p_event_data, uint16_t event_size)
{
	AccelSensorData_t accdata;
	GyroSensorData_t gyrodata;
	MagSensorData_t magdata;
	ImuQuat_t quat;
	long q[4];

	s_Imu.Read(accdata);
	s_Imu.Read(gyrodata);
	s_Imu.Read(magdata);
	ImuRawDataSend(accdata, gyrodata, magdata);
	s_Imu.Read(quat);
	//q[0] = ((float)quat.Q[0] / 32768.0) * (float)(1<<30);
	//q[1] = ((float)quat.Q[1] / 32768.0) * (float)(1<<30);
	//q[2] = ((float)quat.Q[2] / 32768.0) * (float)(1<<30);
	//q[3] = ((float)quat.Q[3] / 32768.0) * (float)(1<<30);
	//q[0] = quat.Q[0] << 15;
	//q[1] = quat.Q[1] << 15;
	//q[2] = quat.Q[2] << 15;
	//q[3] = quat.Q[3] << 15;
	q[0] = quat.Q[0] * (1 << 30);
	q[1] = quat.Q[1] * (1 << 30);
	q[2] = quat.Q[2] * (1 << 30);
	q[3] = quat.Q[3] * (1 << 30);
	//printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, q[0], q[1], q[2], q[3]);
	ImuQuatDataSend(q);
}

static void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			//app_sched_event_put(NULL, 0, ImuDataChedHandler);
			ImuDataChedHandler(NULL, 0);
			//g_MotSensor.Read(accdata);
			break;
	}
}

uint64_t inv_icm20948_get_time_us(void)
{
	return s_pTimer ? s_pTimer->uSecond() : 0;
}

void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
}

void ICM20948IntHandler(int IntNo, void *pCtx)
{
	if (IntNo == BLUEIO_TAG_EVIM_IMU_INT_NO)
	{
		s_Imu.IntHandler();
	}
}

bool ICM20948Init(DeviceIntrf * const pIntrF, Timer * const pTimer)
{
	s_pTimer = pTimer;

	IOPinConfig(BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP,
			IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL);
	IOPinEnableInterrupt(BLUEIO_TAG_EVIM_IMU_INT_NO, 6, BLUEIO_TAG_EVIM_IMU_INT_PORT,
						 BLUEIO_TAG_EVIM_IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION,
						 ICM20948IntHandler, NULL);

	bool res = s_MotSensor.Init(s_AccelCfg, pIntrF);
	if (res == true)
	{
		res |= s_MotSensor.Init(s_GyroCfg, pIntrF);
		if (res == true)
		{
			res |= s_MotSensor.Init(s_MagCfg, pIntrF);
		}
	}
	if (res == true)
	{
		res |= s_Imu.Init(s_ImuCfg, &s_MotSensor, &s_MotSensor, &s_MotSensor);
//		res |= s_Imu.Init(s_ImuCfg, &s_MotSensor);
	}

	if (res)
	{
		s_Imu.Enable();
	}

	return res;
}
