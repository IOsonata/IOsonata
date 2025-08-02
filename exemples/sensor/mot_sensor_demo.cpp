/**-------------------------------------------------------------------------
@example	mot_sensor_demo.cpp


@brief	Motion sensor demo

	This application demo shows how to use Motion sensors.

@author	Hoang Nguyen Hoan
@date	Dec. 21, 2018

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
#include <stdio.h>
#include <atomic>

#include "idelay.h"
#include "coredev/spi.h"
#include "coredev/i2c.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/timer.h"
#include "sensors/agm_mpu9250.h"
#include "sensors/agm_lsm9ds1.h"
#include "sensors/ag_icm456x.h"
#include "sensors/ag_bmi160.h"
#include "sensors/ag_bmi323.h"
#include "sensors/ag_bmi270.h"
#include "sensors/accel_h3lis331dl.h"
#include "imu/imu_mpu9250.h"
#include "imu/imu_xiot_fusion.h"
#include "coredev/uart.h"

#include "Fusion/Fusion.h"

//#define INVN

#ifdef INVN
#include "sensors/agm_invn_icm20948.h"
#include "imu/imu_invn_icm20948.h"
#else
#include "sensors/agm_icm20948.h"
#include "imu/imu_icm20948.h"
#endif
//#include "bmi323.h"
//#include "common.h"

#include "board.h"

std::atomic<bool> g_bTest(false);

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

#ifdef BLYST_MOTION

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

static const IOPinCfg_t s_UartPins[] = UART_PINS;

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,//UARTFIFOSIZE,
	.pRxMem = NULL,//s_UartRxFifo,
	.TxMemSize = 0,//UARTFIFOSIZE,//FIFOSIZE,
	.pTxMem = NULL,//s_UartTxFifo,//g_TxBuff,
	.bDMAMode = true,
};

UART g_Uart;
#endif

//#include "sensor.h"
//#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
//#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

//static const uint8_t dmp3_image[] = {
//#include "imu/icm20948_img.dmp3a.h"
//};

//int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
//int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
//inv_bool_t interface_is_SPI(void);
#ifdef SPI_PINS
static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = 4000000,   // Speed in Hz
	.DataSize = 8,      // Data Size
	.MaxRetry = 5,      // Max retries
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_SECOND_CLK, // Data phase
	.ClkPol = SPICLKPOL_LOW,         // clock polarity
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
	.IntPrio = 6,      // Interrupt priority
	.EvtCB = NULL
};

SPI g_Spi;
#endif

#ifdef I2C_PINS
//********** I2C **********
static const IOPinCfg_t s_I2cPins[] = I2C_PINS;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = 0,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,	// Rate
	.MaxRetry = 5,			// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},		// Slave addresses
	.bDmaEn = true,	// DMA
	.bIntEn = false,		// Use interrupt
	.IntPrio = 7,			// Interrupt prio
	.EvtCB = NULL		// Event callback
};

I2C g_I2c;
#endif

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt);

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler,
};

Timer g_Timer;

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = ACC_DEV_ADDR,//BMI323_I2C_7BITS_DEVADDR,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Scale = 2,
//	.FltrFreq = 50000,
	.Inter = 1,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = ACC_DEV_ADDR,//BMI323_I2C_7BITS_DEVADDR,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
//	.Sensitivity = 10,
//	.FltrFreq = 200,
};

static const MagSensorCfg_t s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,//SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
};

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const ImuCfg_t s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

//#define ICM20948
//#define MPU9250
//#define BMI160
//#define H3LIS331DL
//#define BMI323
//#define BMI270
#define ICM456X

#ifdef ICM20948
#ifdef INVN
ImuInvnIcm20948 g_Imu;
AgmInvnIcm20948 g_MotSensor;
#else
ImuIcm20948 g_Imu;
AgmIcm20948 g_MotSensor;
#endif
#elif defined(MPU9250)
ImuMpu9250 g_Imu;
AgmMpu9250 g_MotSensor;
#elif defined(BMI160)
AgBmi160 g_MotSensor;
#elif defined(BMI323)
AgBmi323 g_MotSensor;
#elif defined(H3LIS331DL)
AccelH3lis331dl g_MotSensor;
#elif defined(BMI270)
AgBmi270 g_MotSensor;
#elif defined(ICM456X)
ImuXiotFusion g_Imu;
AgIcm456x g_MotSensor;
#endif


AccelSensor *g_pAccel = NULL;
GyroSensor *g_pGyro = NULL;
MagSensor *g_pMag = NULL;

uint32_t g_DT = 0;
static uint32_t g_TPrev = 0;

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    }
}

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	AccelSensorData_t accdata;
	ImuQuat_t quat;

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			//g_MotSensor.Read(accdata);
			//g_Imu.Read(accdata);
			//printf("Accel %d: %d %d %d\r\n", accdata.Timestamp, accdata.X, accdata.Y, accdata.Z);
			g_Imu.Read(quat);
			printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, quat.Q1, quat.Q2, quat.Q2, quat.Q3);

			break;
	}
}

uint32_t g_Pdt = 0;

void ImuIntHandler(int IntNo, void *pCtx)
{
	if (IntNo == 0)
	{
//		IOPinSet(0, 24);
		uint64_t t = g_Timer.uSecond();
		g_DT = t - g_TPrev;
		g_TPrev = t;

		//g_Imu.IntHandler();
		g_MotSensor.IntHandler();
		//AccelSensorData_t accdata;

		//g_MotSensor.Read(accdata);
		//printf("Accel %f %f %f\r\n", accdata.X, accdata.Y, accdata.Z);

		g_Pdt = (g_Timer.uSecond() - t);// + g_Pdt) >> 1;
		//IOPinClear(0, 24);
	}
}

uint64_t inv_icm20948_get_time_us(void)
{
	return g_Timer.uSecond();
}

bool HardwareInit()
{
	bool res;

#ifdef BLYST_MOTION
	g_Uart.Init(s_UartCfg);

	g_Uart.printf("BlystMotion Board\n\r");
#endif

	g_Timer.Init(s_TimerCfg);

#ifdef SPI_PINS
	res = g_Spi.Init(s_SpiCfg);
	DeviceIntrf *pintrf = &g_Spi;
#elif defined(I2C_PINS)
	res = g_I2c.Init(s_I2cCfg);
	DeviceIntrf *pintrf = &g_I2c;
#else
#error "No interface defined"
#endif

	//IOPinConfig(IMU_INT_PORT, IMU_INT_PIN, IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	//IOPinEnableInterrupt(0, 6, IMU_INT_PORT, IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION, ImuIntHandler, NULL);

	if (res == true)
	{
		res = g_MotSensor.Init(s_AccelCfg, pintrf, &g_Timer);
		if (res == true)
		{
			g_pAccel = &g_MotSensor;
		}
#if !defined(H3LIS331DL)
		res = g_MotSensor.Init(s_GyroCfg, pintrf);
		if (res == true)
		{
			g_pGyro = &g_MotSensor;
		}

		g_pMag = nullptr;

//		res = g_MotSensor.Init(s_MagCfg, pintrf);
//		if (res == true)
//		{
//			g_pMag = &g_MotSensor;
//		}
#endif

//#if defined(ICM20948) || defined(MPU9250)

//		res = g_Imu.Init(s_ImuCfg, &g_MotSensor);
		res = g_Imu.Init(s_ImuCfg, &g_MotSensor, &g_MotSensor, g_pMag);
//#endif
	}

	if (res == true)
	{
		IOPinSet(IMU_INT_PORT, IMU_INT_PIN);
		IOPinConfig(IMU_INT_PORT, IMU_INT_PIN, IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
		IOPinEnableInterrupt(0, 6, IMU_INT_PORT, IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION, ImuIntHandler, NULL);

		int8_t m[9] = { 1, 0, 0,
						0, 1, 0,
						0, 0, 1 };

		g_Imu.SetAxisAlignmentMatrix(m);
		g_Imu.Quaternion(true, 6);
	}

	//uint64_t period = g_Timer.EnableTimerTrigger(0, 1UL, TIMER_TRIG_TYPE_CONTINUOUS);

	//printf("period %u\r\n", (uint32_t)period);

	return res;
}
//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	bool res = HardwareInit();


	if (res == false)
	{
		printf("Failed initializing hardware\r\n");

		return 1;
	}

	printf("MotionSensorDemo\r\n");

	AccelSensorRawData_t arawdata;
	AccelSensorData_t accdata;
	GyroSensorRawData_t grawdata;
	GyroSensorData_t gyrodata;
	MagSensorRawData_t mrawdata;
	ImuQuat_t quat;

	memset(&arawdata, 0, sizeof(AccelSensorRawData_t));
	memset(&accdata, 0, sizeof(AccelSensorData_t));
	memset(&gyrodata, 0, sizeof(GyroSensorData_t));


	uint32_t prevt = 0;
	int cnt = 10;

	g_MotSensor.Enable();

	g_Imu.Enable();

    FusionAhrs ahrs;
    //FusionAhrsInitialise(&ahrs);

	while (1)
	{
//		uint32_t t = g_Timer.uSecond();

		//NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		__WFE();
		//g_MotSensor.UpdateData();

		uint32_t dt = arawdata.Timestamp - prevt;
		prevt = arawdata.Timestamp;

		g_MotSensor.Read(arawdata);

		if (g_pGyro)
		{
			g_pGyro->Read(grawdata);
		}

		if (g_pMag)
		{
			g_pMag->Read(mrawdata);
		}

		g_MotSensor.Read(accdata);
		g_MotSensor.Read(gyrodata);
		//g_Imu.Read(quat);
        //FusionVector gyroscope = {gyrodata.X, gyrodata.Y, gyrodata.Z}; // replace this with actual gyroscope data in degrees/s
       // FusionVector accelerometer = {accdata.X, accdata.Y, accdata.Z}; // replace this with actual accelerometer data in g

       // FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.02);

        //FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
       // FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs);

		//if (cnt-- < 0)
		{
			cnt = 10;
			//printf("Accel %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)g_Pdt, arawdata.X, arawdata.Y, arawdata.Z);
			printf("Accel %d %d: %0.4f %0.4f %0.4f\r\n", (uint32_t)g_DT, (uint32_t)g_Pdt, accdata.X, accdata.Y, accdata.Z);
			//printf("Gyro %d %d: %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, gyrodata.X, gyrodata.Y, gyrodata.Z);
//	        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
			//printf("Quat %8d %d: %f %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, quat.Q1, quat.Q2, quat.Q3, quat.Q4);
			//printf("Quat %8d %d: %f %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, fq.element.x, fq.element.y, fq.element.z, fq.element.w);
		}
	}
}
