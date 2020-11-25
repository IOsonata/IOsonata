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

//#include "nrf.h"

#include "idelay.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/timer.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_invn_icm20948.h"
#include "sensors/agm_mpu9250.h"
#include "sensors/agm_lsm9ds1.h"
#include "sensors/ag_bmi160.h"
#include "sensors/accel_h3lis331dl.h"
#include "sensors/am_lsm303agr.h"
#include "imu/imu_invn_icm20948.h"
#include "imu/imu_icm20948.h"
#include "imu/imu_mpu9250.h"

#include "board.h"
#include "lsm303agr_reg.h"

std::atomic<bool> g_bTest(false);

static const IOPinCfg_t s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {-1, -1, -1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    //{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    //{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#ifdef NEBLINA
	{NEBLINA_SPI_BMI160_CS_PORT, NEBLINA_SPI_BMI160_CS_PIN, NEBLINA_SPI_BMI160_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{NEBLINA_SPI_H3LIS_CS_PORT, NEBLINA_SPI_H3LIS_CS_PIN, NEBLINA_SPI_H3LIS_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#else
	{BLUEIO_TAG_EVIM_IMU_CS_PORT, 14/*BLUEIO_TAG_EVIM_IMU_CS_PIN*/, BLUEIO_TAG_EVIM_IMU_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_IMU_CS_PORT, 11/*BLUEIO_TAG_EVIM_IMU_CS_PIN*/, BLUEIO_TAG_EVIM_IMU_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},

#endif
};

static const SPICFG s_SpiCfg = {
    SPI_DEVNO,
	SPIPHY_3WIRE,
    SPIMODE_MASTER,
    s_SpiPins,
    sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    4000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
	false,
	false,
    6,      // Interrupt priority
    NULL
};

SPI g_Spi;

static const IOPinCfg_t s_GpioPins[] = {
	{BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 1
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 2
	{BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, // IMU int pin
	{0, 24, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// But 2
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPinCfg_t);

void TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler,
};

//TimerLFnRFx g_Timer;
Timer g_Timer;

static const ACCELSENSOR_CFG s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.Scale = 2,
	.FltrFreq = 0,
	.bInter = false,
	.IntPol = DEVINTR_POL_LOW,
};

static const GYROSENSOR_CFG s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 10,
	.FltrFreq = 200,
};

static const MAGSENSOR_CFG s_MagCfg = {
	.DevAddr = 1,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,//SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
};

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const IMU_CFG s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

//#define ICM20948
//#define MPU9250
//#define BMI160
//#define H3LIS331DL
#define LSM303AGR

#ifdef ICM20948
ImuIcm20948 g_Imu;
AgmIcm20948 g_MotSensor;
#elif 0
AgmIcm20948 g_MotSensor;
#elif defined(MPU9250)
ImuMpu9250 g_Imu;
AgmMpu9250 g_MotSensor;
#elif defined(BMI160)
AgBmi160 g_MotSensor;
#elif defined(H3LIS331DL)
AccelH3lis331dl g_MotSensor;
#elif defined(LSM303AGR)
AccelLsm303agr g_MotSensor;
MagLsm303agr g_MagSensor;
#else
AgmLsm9ds1 g_MotSensor;
#endif

AccelSensor *g_pAccel = NULL;
GyroSensor *g_pGyro = NULL;
MagSensor *g_pMag = NULL;

uint32_t g_DT = 0;
static uint32_t g_TPrev = 0;


stmdev_ctx_t mXLDrvHandle;   // Accel
stmdev_ctx_t mMGDrvHandle;   // Mag

static int32_t LSM303AGR_readXL(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t BufferLen)
{
	int devaddr = (int)handle;
	ReadAddr |= 0x80;
	if (BufferLen > 1)
	{
		ReadAddr |= 0x40;
	}
	return g_Spi.Read(devaddr, &ReadAddr, 1, pBuffer, BufferLen) > 0 ? 0 : 1;
    //return dev->ReadXL(ReadAddr, pBuffer, nBytesToRead) > 0 ? 0 : 1;
}

static int32_t LSM303AGR_writeXL(void *handle, uint8_t WriteAddr, uint8_t *pData, uint16_t DataLen)
{
	int devaddr = (int)handle;
	if (DataLen > 1)
	{
		WriteAddr |= 0x40;
	}
	return g_Spi.Write(devaddr, &WriteAddr, 1, pData, DataLen)  > 0 ? 0 : 1;

//    return dev->WriteXL(WriteAddr, pBuffer, nBytesToWrite) > 0 ? 0 : 1;
}

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    }
}

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	ACCELSENSOR_DATA accdata;
	IMU_QUAT quat;

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			g_MotSensor.Read(accdata);
			//g_Imu.Read(accdata);
			printf("Accel %d: %d %d %d\r\n", accdata.Timestamp, accdata.X, accdata.Y, accdata.Z);
			//g_Imu.Read(quat);
			//printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, quat.Q1, quat.Q2, quat.Q2, quat.Q3);

			break;
	}
}


void ImuIntHandler(int IntNo)
{
	if (IntNo == 0)
	{
//		IOPinSet(0, 24);
		uint32_t t = g_Timer.uSecond();
		g_DT = t - g_TPrev;
		g_TPrev = t;

		//g_Imu.IntHandler();
		g_MotSensor.IntHandler();
		//IOPinClear(0, 24);
	}
}

uint64_t inv_icm20948_get_time_us(void)
{
	return g_Timer.uSecond();
}

void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
}


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

#define TX_BUF_DIM          1000

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

void lsm303agr_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx_xl;
  dev_ctx_xl.write_reg = LSM303AGR_writeXL;
  dev_ctx_xl.read_reg = LSM303AGR_readXL;
  dev_ctx_xl.handle = (void*)0;//LSM303AGR_I2C_ADD_XL;

  stmdev_ctx_t dev_ctx_mg;
  dev_ctx_mg.write_reg = LSM303AGR_writeXL;
  dev_ctx_mg.read_reg = LSM303AGR_readXL;
  dev_ctx_mg.handle = (void*)1;//LSM303AGR_I2C_ADD_MG;

  lsm303agr_xl_spi_mode_set(&dev_ctx_xl, LSM303AGR_SPI_3_WIRE);

  /* Check device ID */
  whoamI = 0;
  lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
  if ( whoamI != LSM303AGR_ID_XL )
    while(1); /*manage here device not found */

  whoamI = 0;
  lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);
  if ( whoamI != LSM303AGR_ID_MG )
    while(1); /*manage here device not found */

  /* Restore default configuration for magnetometer */
  lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
  do {
     lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
  lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
  /* Set accelerometer full scale */
  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
  /* Set / Reset magnetic sensor mode */
  lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation on mag sensor */
  lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Enable temperature sensor */
  lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
  /* Set device in continuos mode */
  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
  /* Set magnetometer in continuos mode */
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);

  /* Read samples in polling mode (no int) */
  while(1)
  {
    /* Read output only if new value is available */
    lsm303agr_reg_t reg;
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

    if (reg.status_reg_a.zyxda)
    {
      /* Read accelerometer data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration.u8bit);
      acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[0] );
      acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[1] );
      acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[2] );

      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      //tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }

    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    if (reg.status_reg_m.zyxda)
    {
      // Read magnetic field data
      memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
      lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic.u8bit);
      magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[0]);
      magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[1]);
      magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[2]);

      printf("Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
      //tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }

    lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);
    if (reg.byte)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lsm303agr_temperature_raw_get(&dev_ctx_xl, data_raw_temperature.u8bit);
      temperature_degC = lsm303agr_from_lsb_hr_to_celsius( data_raw_temperature.i16bit );

      printf("Temperature [degC]:%6.2f\r\n", temperature_degC );
      //tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}

void lsm303agr_self_test(void)
{
#define    BOOT_TIME               5 //ms

/* Self test limits. */
#define    MIN_ST_XL_LIMIT_mg     68.0f
#define    MAX_ST_XL_LIMIT_mg   1440.0f
#define    MIX_ST_MG_LIMIT_mG     15.0f
#define    MAX_ST_MG_LIMIT_mG    500.0f
/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

  uint8_t tx_buffer[1000];
  axis3bit16_t data_raw;
  float maes_st_off[3];
  float maes_st_on[3];
  lsm303agr_reg_t reg;
  float test_val[3];
  uint8_t st_result;
  uint8_t i, j;

  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx_xl;
  dev_ctx_xl.write_reg = LSM303AGR_writeXL;
  dev_ctx_xl.read_reg = LSM303AGR_readXL;
  dev_ctx_xl.handle = (void*)0;
/*
  stmdev_ctx_t dev_ctx_mg;
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void*)LSM303AGR_I2C_ADD_MG;
*/
  /* Initialize self test results */
  st_result = ST_PASS;

  /* Wait boot time and initialize platform specific hardware */
 // platform_init();

  /* Wait sensor boot time */
  msDelay(BOOT_TIME);

  /* Check device ID */
  reg.byte = 0;
  lsm303agr_xl_device_id_get(&dev_ctx_xl, &reg.byte);
  if ( reg.byte != LSM303AGR_ID_XL )
    while(1); /*manage here device not found */
  reg.byte = 0;
//  lsm303agr_mag_device_id_get(&dev_ctx_mg, &reg.byte);
//  if ( reg.byte != LSM303AGR_ID_MG )
//    while(1); /*manage here device not found */

  /*
   * Accelerometer Self Test
   */
  /* Enable Block Data Update. */
  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  /* Set full scale to 2g. */
  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
  /* Set device in normal mode. */
  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
  /* Set Output Data Rate. */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_100Hz);

  /* Wait stable output. */
  msDelay(90);

  /* Check if new value available */
  do {
	  lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
  } while(!reg.status_reg_a.zyxda);
  /* Read dummy data and discard it */
  lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw.u8bit);

  /* Read 5 sample and get the average value for each axis */
  for (i = 0; i < 5; i++){
    /* Check if new value available */
    do {
        lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    } while(!reg.status_reg_a.zyxda);
    /* Read data and accumulate the mg value */
    lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      maes_st_off[j] += lsm303agr_from_fs_2g_nm_to_mg(data_raw.i16bit[j]);
    }
  }
  /* Calculate the mg average values */
  for (i = 0; i < 3; i++){
	  maes_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm303agr_xl_self_test_set(&dev_ctx_xl, LSM303AGR_ST_POSITIVE);
  //lsm303agr_xl_self_test_set(&dev_ctx, LSM303AGR_XL_ST_NEGATIVE);

  /* Wait stable output */
  msDelay(90);

  /* Check if new value available */
  do {
	  lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
  } while(!reg.status_reg_a.zyxda);
  /* Read dummy data and discard it */
  lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw.u8bit);

  /* Read 5 sample and get the average value for each axis */
  for (i = 0; i < 5; i++){
    /* Check if new value available */
    do {
        lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    } while(!reg.status_reg_a.zyxda);
    /* Read data and accumulate the mg value */
    lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      maes_st_on[j] += lsm303agr_from_fs_2g_nm_to_mg(data_raw.i16bit[j]);
    }
  }
  /* Calculate the mg average values */
  for (i = 0; i < 3; i++){
    maes_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i=0; i<3; i++){
    test_val[i] = fabs((maes_st_on[i] - maes_st_off[i]));
  }

  /* Check self test limit */
  for (i=0; i<3; i++){
	  printf("v=%d\n",  test_val[i]);
    if (( MIN_ST_XL_LIMIT_mg > test_val[i] ) ||
		    ( test_val[i] > MAX_ST_XL_LIMIT_mg)){
      st_result = ST_FAIL;
	  }
	}
printf("Selftest : %d\n", st_result);
//while(1);
  /* Disable Self Test */
  lsm303agr_xl_self_test_set(&dev_ctx_xl, LSM303AGR_ST_DISABLE);

  /* Disable sensor. */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_POWER_DOWN);
#if 0
  /*
   * Magnetometer  Self Test
   */
  /* Restore default configuration for magnetometer */
 // lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
 // do {
 //    lsm303agr_mag_reset_get(&dev_ctx_mg, &reg.byte);
 // } while (reg.byte);
  /* Enable Block Data Update. */
 // lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set / Reset sensor mode. */
 // lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation. */
 // lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set device in continuous mode. */
 // lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);
  /* Set Output Data Rate. */
 // lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_100Hz);

  /* Wait stable output. */
  platform_delay(20);

  /* Check if new value available .*/
  do {
	  lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while(!reg.status_reg_m.zyxda);
  /* Read dummy data and discard it. */
  lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw.u8bit);

  /* Read 50 sample and get the average value for each axis */
  for (i = 0; i < 50; i++){
    /* Check if new value available */
    do {
        lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while(!reg.status_reg_m.zyxda);
    /* Read data and accumulate the mg value */
    lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      maes_st_off[j] += lsm303agr_from_lsb_to_mgauss(data_raw.i16bit[j]);
    }
  }
  /* Calculate the mg average values */
  for (i = 0; i < 3; i++){
	  maes_st_off[i] /= 50.0f;
  }

  /* Enable Self Test. */
  lsm303agr_mag_self_test_set(&dev_ctx_mg, PROPERTY_ENABLE);

  /* Wait stable output */
  platform_delay(60);

  /* Check if new value available .*/
  do {
	  lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while(!reg.status_reg_m.zyxda);
  /* Read dummy data and discard it. */
  lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw.u8bit);

  /* Read 50 sample and get the average value for each axis */
  for (i = 0; i < 50; i++){
    /* Check if new value available */
    do {
        lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while(!reg.status_reg_m.zyxda);
    /* Read data and accumulate the mg value */
    lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      maes_st_on[j] += lsm303agr_from_lsb_to_mgauss(data_raw.i16bit[j]);
    }
  }
  /* Calculate the mg average values */
  for (i = 0; i < 3; i++){
	  maes_st_on[i] /= 50.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++){
    test_val[i] = fabs((maes_st_on[i] - maes_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++){
    if (( MIX_ST_MG_LIMIT_mG > test_val[i] ) ||
		    ( test_val[i] > MIX_ST_MG_LIMIT_mG)){
      st_result = ST_FAIL;
	  }
	}

  /* Disable Self Test */
  lsm303agr_mag_self_test_set(&dev_ctx_mg, PROPERTY_DISABLE);

  /* Disable sensor. */
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_POWER_DOWN);

  /* Print self test result */
  if (st_result == ST_PASS) {
    sprintf((char*)tx_buffer, "Self Test - PASS\r\n" );
  }
  else {
    sprintf((char*)tx_buffer, "Self Test - FAIL\r\n" );
  }
  tx_com(tx_buffer, strlen((char const*)tx_buffer));
#endif
}

bool HardwareInit()
{
	bool res;

	g_Timer.Init(s_TimerCfg);

	res = g_Spi.Init(s_SpiCfg);

	if (res == true)
	{
		res = g_MotSensor.Init(s_AccelCfg, &g_Spi, &g_Timer);
		if (res == true)
		{
			g_pAccel = &g_MotSensor;
		}
		g_pGyro = NULL;
#if !defined(H3LIS331DL) && !defined(LSM303AGR)
		res = g_MotSensor.Init(s_GyroCfg, &g_Spi);
		if (res == true)
		{
			g_pGyro = &g_MotSensor;
		}
#endif
		res = g_MagSensor.Init(s_MagCfg, &g_Spi);
		if (res == true)
		{
			g_pMag = &g_MagSensor;
		}
//#endif

#if defined(ICM20948) || defined(MPU9250)
		res = g_Imu.Init(s_ImuCfg, &g_MotSensor, &g_MotSensor, &g_MotSensor);
#endif
	}

	if (res == true)
	{
		//IOPinCfg(s_GpioPins, s_NbGpioPins);
		//IOPinEnableInterrupt(0, 6, BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION, ImuIntHandler);

		int8_t m[9] = { 1, 0, 0,
						0, 1, 0,
						0, 0, 1 };

		//g_Imu.SetAxisAlignmentMatrix(m);
		//g_Imu.Quaternion(true, 6);
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

	//lsm303agr_self_test();
	//lsm303agr_read_data_polling();

	if (res == false)
	{
		printf("Failed initializing hardware\r\n");

		return 1;
	}

	printf("MotionSensorDemo\r\n");

	ACCELSENSOR_RAWDATA arawdata;
	ACCELSENSOR_DATA accdata;
	GYROSENSOR_RAWDATA grawdata;
	GYROSENSOR_DATA gyrodata;
	MAGSENSOR_RAWDATA mrawdata;
	MAGSENSOR_DATA magdata;
	IMU_QUAT quat;

	memset(&arawdata, 0, sizeof(ACCELSENSOR_RAWDATA));
	memset(&accdata, 0, sizeof(ACCELSENSOR_DATA));
	memset(&gyrodata, 0, sizeof(GYROSENSOR_DATA));
/*
    mXLDrvHandle.write_reg = LSM303AGR_writeXL;
    mXLDrvHandle.read_reg = LSM303AGR_readXL;
    mXLDrvHandle.handle = (void*)&g_Spi;

    lsm303agr_xl_spi_mode_set(&mXLDrvHandle, LSM303AGR_SPI_3_WIRE);

    uint32_t d = 0;

    lsm303agr_xl_device_id_get(&mXLDrvHandle, (uint8_t*)&d);
    if (d == LSM303AGR_ID_XL)
    {
    	printf("Found\n");

        lsm303agr_xl_block_data_update_set(&mXLDrvHandle, PROPERTY_ENABLE);

        while(1)
        {
        lsm303agr_reg_t reg;
        int32_t res = lsm303agr_xl_status_get(&mXLDrvHandle, &reg.status_reg_a);
        if (reg.status_reg_a.zyxda)
        {
            res = lsm303agr_acceleration_raw_get(&mXLDrvHandle, (uint8_t*)&arawdata.Val);
            if (res != 0)
               	printf("Accel: %d %d %d\r\n", arawdata.X, arawdata.Y, arawdata.Z);
        }
        }
    }
*/
	uint32_t prevt = 0;
	int cnt = 100;
	while (1)
	{
		uint32_t t = g_Timer.uSecond();

		//NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		//__WFE();
		g_MotSensor.UpdateData();

		uint32_t dt = arawdata.Timestamp - prevt;
		prevt = arawdata.Timestamp;

		g_MotSensor.Read(arawdata);
		g_MotSensor.Read(accdata);

		if (g_pGyro)
		{
			g_pGyro->Read(grawdata);
			g_pGyro->Read(gyrodata);
		}

		if (g_pMag)
		{
			g_pMag->UpdateData();
			g_pMag->Read(mrawdata);
			g_pMag->Read(magdata);
		}

		//g_Imu.Read(accdata);
		//g_Imu.Read(quat);

		if (cnt-- < 0)
		{
			cnt = 100;
			//printf("Accel %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)dt, arawdata.X, arawdata.Y, arawdata.Z);
			//printf("Gyro %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)dt, grawdata.X, grawdata.Y, grawdata.Z);
			//printf("Mag %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)dt, mrawdata.X, mrawdata.Y, mrawdata.Z);
			printf("Accel %d %d: %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, accdata.X, accdata.Y, accdata.Z);
			//printf("Gyro %d %d: %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, gyrodata.X, gyrodata.Y, gyrodata.Z);
			//printf("Mag %d %d: %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, magdata.X, magdata.Y, magdata.Z);
			//printf("Quat %d %d: %f %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, quat.Q1, quat.Q2, quat.Q3, quat.Q4);
		}
	}
}
