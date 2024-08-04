/*
 * main.cpp
 *
 *  Created on: Jul. 25, 2024
 *      Author: hoan
 */
#include <string.h>

#include "idelay.h"
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "coredev/i2c.h"
#include "pwrmgnt/pm_bq25120a.h"
#include "sensors/ag_bmi323.h"

#include "board.h"

static const IOPinCfg_t s_IOPins[] = {
    {LED_RED_PORT, LED_RED_PIN, LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {LED_BLUE_PORT, LED_BLUE_PIN, LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {VDD_EN_PORT, VDD_EN_PIN, VDD_EN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {VDDIO_EN_PORT, VDDIO_EN_PIN, VDDIO_EN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {LS_EN_PORT, LS_EN_PIN, LS_EN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const int s_IOPinsCnt = sizeof(s_IOPins) / sizeof(IOPinCfg_t);

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt);

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler,
};

Timer g_Timer;

static const IOPinCfg_t s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_BMI323_CS_PORT, SPI_BMI323_CS_PIN, SPI_BMI323_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

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
	.DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
	.ClkPol = SPICLKPOL_HIGH,         // clock polarity
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
	.IntPrio = 6,      // Interrupt priority
	.EvtCB = NULL
};

SPI g_Spi;

//********** I2C **********
static const IOPinCfg_t s_I2cPins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,	// Rate
	.MaxRetry = 5,			// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	0,			// Number of slave addresses
	{0,},		// Slave addresses
	true,	// DMA
	false,		// Use interrupt
	7,			// Interrupt prio
	NULL		// Event callback
};

I2C g_I2c;

static const PwrMgntVoutCfg_t s_PmVoutCfg[] = {
	{
		.mVout = 3300,
		.mAlimit = 100,
	},
	{
		.mVout = 3300,
		.mAlimit = 100,
	},
};

PwrMgntCfg_t s_PmicCfg = {
	.DevAddr = BQ25120A_I2C_7BITS_DEVADDR,
	.pVout = (PwrMgntVoutCfg_t*)&s_PmVoutCfg,
	.NbVout = sizeof(s_PmVoutCfg) / sizeof(PwrMgntVoutCfg_t),
	.VEndChrg = 4200,
	.ChrgCurr = 500,
};

PmBq25120a g_Pmic;

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.Scale = 2,
	.FltrFreq = 0,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 10,
	.FltrFreq = 200,
};

AgBmi323 g_MotSensor;
AccelSensor *g_pAccel = NULL;
GyroSensor *g_pGyro = NULL;

uint32_t g_DT = 0;
static uint32_t g_TPrev = 0;

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    }
}

bool HardwareInit()
{
	bool res;

	g_I2c.Init(s_I2cCfg);

	g_Pmic.Init(s_PmicCfg, &g_I2c);

	IOPinCfg(s_IOPins, s_IOPinsCnt);

	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinSet(VDD_EN_PORT, VDD_EN_PIN);
	IOPinSet(LS_EN_PORT, LS_EN_PIN);
	IOPinSet(VDDIO_EN_PORT, VDDIO_EN_PIN);

	g_Timer.Init(s_TimerCfg);

	res = g_Spi.Init(s_SpiCfg);

	msDelay(10);


	res = g_MotSensor.Init(s_AccelCfg, &g_Spi, &g_Timer);
	if (res == true)
	{
		g_pAccel = &g_MotSensor;
	}

	return res;
}

int main()
{
	HardwareInit();
	AccelSensorRawData_t arawdata;
	AccelSensorData_t accdata;
	GyroSensorRawData_t grawdata;
	GyroSensorData_t gyrodata;

	memset(&arawdata, 0, sizeof(AccelSensorRawData_t));
	memset(&accdata, 0, sizeof(AccelSensorData_t));
	memset(&gyrodata, 0, sizeof(GyroSensorData_t));


	uint32_t prevt = 0;
	int cnt = 10;
	while (1)
	{
//		uint32_t t = g_Timer.uSecond();

		//NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		//__WFE();
		g_MotSensor.UpdateData();

		uint32_t dt = arawdata.Timestamp - prevt;
		prevt = arawdata.Timestamp;

		g_MotSensor.Read(arawdata);

		if (g_pGyro)
		{
			g_pGyro->Read(grawdata);
		}

		g_MotSensor.Read(accdata);
		//g_Imu.Read(quat);

		if (cnt-- < 0)
		{
			cnt = 10;
			uint32_t t = accdata.Timestamp;
			//printf("Accel %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)dt, arawdata.X, arawdata.Y, arawdata.Z);
			printf("Accel %d %d: %f %f %f %u\r\n", (uint32_t)g_DT, (uint32_t)dt, accdata.X, accdata.Y, accdata.Z, t);
			//printf("Quat %d %d: %f %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, quat.Q1, quat.Q2, quat.Q3, quat.Q4);
		}
	}
}

