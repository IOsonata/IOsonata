/** Thingy:52 peripheral validation. SPDX-License-Identifier: MIT */
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "sensors/color_bh1745.h"
#include "sensors/gas_ccs811.h"
#include "sensors/accel_lisxdh.h"
#include "sensors/agm_mpu9250.h"
#include "sensors/humi_hts221.h"
#include "sensors/press_lps22hb.h"
#include "board.h"

#define UART_FIFO_SIZE			CFIFO_MEMSIZE(256)

static uint8_t s_UartTxBuffer[UART_FIFO_SIZE];

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP,
	 IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	UART_DEVNO,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	115200,
	8,
	UART_PARITY_NONE,
	1,
	UART_FLWCTRL_NONE,
	true,
	1,
	NULL,
	true,
	0,
	NULL,
	UART_FIFO_SIZE,
	s_UartTxBuffer,
	false,
	false,
	false,
	false,
	0,
	UART_DUPLEX_FULL,
	UART_MODE_UART,
};

static const IOPinCfg_t s_I2cPins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP,
	 IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(s_I2cPins[0]),
	.Rate = 400000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 7,
	.EvtCB = NULL,
};


static const IOPinCfg_t s_AccelI2cPins[] = {
	{THINGY52_ACCEL_I2C_PORT, THINGY52_ACCEL_I2C_SDA_PIN, 1,
	 IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{THINGY52_ACCEL_I2C_PORT, THINGY52_ACCEL_I2C_SCL_PIN, 1,
	 IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static UART s_Uart;
static I2C s_I2c;
static IoExpSx1509 s_Expander;
static ColorBh1745 s_Color;
static GasCcs811 s_Gas;
static I2C s_AccelI2c;
static Timer s_Timer;
static AccelLisxdh s_Accel;
static AgmMpu9250 s_Motion;
static HumiHts221 s_Humidity;
static PressLps22hb s_Pressure;

[[noreturn]] static void Stop(const char *pReason)
{
	s_Uart.printf("STOP: %s\r\n", pReason);
	while (1)
		msDelay(1000);
}

static void InitSensors()
{
	I2CCfg_t busCfg = s_I2cCfg;
	TimerCfg_t timerCfg = {};
	AccelSensorCfg_t accelCfg = {};
	GyroSensorCfg_t gyroCfg = {};
	MagSensorCfg_t magCfg = {};
	HumiSensorCfg_t humCfg = {};
	TempSensorCfg_t tempCfg = {};
	PressureSensorCfg_t pressCfg = {};

	timerCfg.DevNo = 0;
	timerCfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	timerCfg.IntPrio = 7;
	if (!s_Timer.Init(timerCfg))
		Stop("timestamp timer initialization failed");
	busCfg.DevNo = THINGY52_ACCEL_I2C_DEVNO;
	busCfg.pIOPinMap = s_AccelI2cPins;
	busCfg.NbIOPins = sizeof(s_AccelI2cPins) / sizeof(s_AccelI2cPins[0]);
	if (!s_AccelI2c.Init(busCfg))
		Stop("LIS2DH12 bus initialization failed");
	accelCfg.DevAddr = LISXDH_I2C_DEVADDR1;
	accelCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	accelCfg.Freq = 10000;
	accelCfg.Scale = 2;
	if (!s_Accel.Init(accelCfg, &s_AccelI2c, &s_Timer))
		Stop("LIS2DH12 initialization failed");
	if (!Thingy52MotionPower(s_Expander, true))
		Stop("MPU9250 power sequence failed");
	accelCfg.DevAddr = MPU9250_I2C_DEV_ADDR0;
	accelCfg.Freq = 100000;
	gyroCfg.DevAddr = MPU9250_I2C_DEV_ADDR0;
	gyroCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	// Explicit unfiltered 32 kHz mode; the console takes one snapshot/second.
	gyroCfg.Freq = 32000000;
	gyroCfg.Sensitivity = 250;
	gyroCfg.FltrFreq = 0;
	magCfg.DevAddr = MPU9250_I2C_DEV_ADDR0;
	magCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	magCfg.Freq = 8000;
	magCfg.Precision = MAGSENSOR_PRECISION_HIGH;
	if (!s_Motion.Init(accelCfg, &s_I2c, &s_Timer) ||
		!s_Motion.Init(gyroCfg, &s_I2c, &s_Timer) ||
		!s_Motion.Init(magCfg, &s_I2c, &s_Timer) || !s_Motion.Enable())
		Stop("MPU9250 initialization failed");

	humCfg.DevAddr = HTS221_I2C_DEV_ADDR;
	humCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	humCfg.Freq = 1000;
	humCfg.HumOvrs = 32;
	tempCfg.DevAddr = HTS221_I2C_DEV_ADDR;
	tempCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	tempCfg.Freq = 1000;
	tempCfg.TempOvrs = 16;
	if (!s_Humidity.Init(humCfg, &s_I2c, &s_Timer) ||
		!s_Humidity.Init(tempCfg, &s_I2c, &s_Timer) || !s_Humidity.Enable())
		Stop("HTS221 initialization failed");
	pressCfg.DevAddr = LPS22HB_I2C_DEV_ADDR0;
	pressCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	pressCfg.Freq = 1000;
	tempCfg.DevAddr = LPS22HB_I2C_DEV_ADDR0;
	if (!s_Pressure.Init(pressCfg, &s_I2c, &s_Timer) ||
		!s_Pressure.Init(tempCfg, &s_I2c, &s_Timer) || !s_Pressure.Enable())
		Stop("LPS22HB initialization failed");
}

static void ReadSensors()
{
	AccelSensorRawData_t accel;
	GyroSensorRawData_t gyro;
	MagSensorRawData_t mag;
	HumiSensorData_t humidity;
	TempSensorData_t temperature;
	PressureSensorData_t pressure;
	static uint64_t lastEnvironment = 0;
	static uint64_t lastPressure = 0;
	static uint64_t lastMag = 0;

	if (s_Accel.UpdateData())
	{
		s_Accel.Read(accel);
		s_Uart.printf("LIS2DH12 XYZ counts=%d,%d,%d\r\n", (int)accel.X, (int)accel.Y, (int)accel.Z);
	}
	else
		s_Uart.printf("LIS2DH12: no new sample or I2C error\r\n");
	if (s_Motion.UpdateData())
	{
		s_Motion.Read(accel);
		s_Motion.Read(gyro);
		s_Motion.Read(mag);
		s_Uart.printf("MPU9250 accel counts=%d,%d,%d gyro counts=%d,%d,%d\r\n",
			(int)accel.X, (int)accel.Y, (int)accel.Z, (int)gyro.X, (int)gyro.Y, (int)gyro.Z);
		if (mag.Timestamp != lastMag)
		{
			lastMag = mag.Timestamp;
			s_Uart.printf("AK8963 trimmed counts=%d,%d,%d\r\n", (int)mag.X, (int)mag.Y, (int)mag.Z);
		}
		else
			s_Uart.printf("AK8963: no new sample, overflow or I2C error\r\n");
	}
	else
		s_Uart.printf("MPU9250: I2C read failed\r\n");
	if (s_Humidity.UpdateData())
	{
		s_Humidity.Read(humidity);
		s_Humidity.Read(temperature);
		// Do not feed cached/uninitialized values to CCS811 after a partial sample.
		if (humidity.Timestamp == temperature.Timestamp && humidity.Timestamp != lastEnvironment)
		{
			lastEnvironment = humidity.Timestamp;
			s_Uart.printf("HTS221 temperature=%d centi-C humidity=%u centi-percent\r\n",
				(int)temperature.Temperature, (unsigned)humidity.Humidity);
			if (temperature.Temperature >= -4000 && temperature.Temperature <= 8500)
			{
				if (!s_Gas.SetEnvironment((int16_t)temperature.Temperature, humidity.Humidity))
					Stop("CCS811 environment write failed");
			}
			else
				s_Uart.printf("CCS811: temperature outside compensation range\r\n");
		}
		else
			s_Uart.printf("HTS221: incomplete humidity/temperature pair\r\n");
	}
	else
		s_Uart.printf("HTS221: no new sample or I2C error\r\n");
	if (s_Pressure.UpdateData())
	{
		s_Pressure.Read(pressure);
		if (pressure.Timestamp != lastPressure)
		{
			lastPressure = pressure.Timestamp;
			s_Uart.printf("LPS22HB pressure=%u Pa\r\n", (unsigned)pressure.Pressure);
		}
		else
			s_Uart.printf("LPS22HB: no new pressure sample\r\n");
	}
	else
		s_Uart.printf("LPS22HB: no new sample or I2C error\r\n");
}

int main()
{
	GasSensorCfg_t gasCfg = {};
	Bh1745Data_t color = {};
	Ccs811Data_t gas = {};
	uint16_t boot;
	uint16_t app;
	uint8_t error;
	unsigned cycle = 0;
	bool colorIrq;
	bool gasIrq;
	bool pressed;
	bool wasPressed = false;
	bool illuminate = false;

	if (!s_Uart.Init(s_UartCfg))
		return 1;
	s_Uart.printf("Thingy:52 peripheral test\r\n");
	Thingy52PowerInit();
	if (!s_I2c.Init(s_I2cCfg))
		Stop("I2C initialization failed");
	if (!Thingy52ExpanderInit(s_Expander, s_I2c))
		Stop("SX1509 board initialization failed");
	if (!s_Color.Init(&s_I2c, BH1745_I2C_ADDR0) || !s_Color.Configure(0, 1) ||
		!s_Color.ConfigureInterrupt(true, 3, 1, 0, 0xFFFF) || !s_Color.Enable())
		Stop("BH1745 initialization failed");
	if (!Thingy52GasPower(s_Expander, true))
		Stop("CCS811 power sequence failed");
	s_Gas.WakeControl(Thingy52GasWake, &s_Expander);
	gasCfg.DevAddr = CCS811_I2C_ADDR0;
	gasCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	gasCfg.Freq = 1000;
	if (!s_Gas.Init(gasCfg, &s_I2c) || !s_Gas.ConfigureInterrupt(true))
		Stop("CCS811 initialization failed");
	if (!s_Gas.FirmwareVersion(boot, app))
		Stop("CCS811 firmware version read failed");
	s_Uart.printf("CCS811 firmware boot=%04x app=%04x\r\n", (unsigned)boot, (unsigned)app);
	InitSensors();
	s_Uart.printf("Gas readings require conditioning; compensation uses fresh HTS221 samples.\r\n");
	s_Uart.printf("Lightwell cycles RGB. Press button to toggle color-sensor illumination.\r\n");

	while (1)
	{
		msDelay(1000);
		pressed = Thingy52ButtonPressed();
		if (pressed && !wasPressed)
		{
			illuminate = !illuminate;
			if (!Thingy52ColorLight(s_Expander, illuminate, illuminate, illuminate))
				Stop("color illumination write failed");
		}
		wasPressed = pressed;
		if (!Thingy52Lightwell(s_Expander, cycle % 3 == 0, cycle % 3 == 1, cycle % 3 == 2))
			Stop("lightwell write failed");
		colorIrq = Thingy52ColorIrqActive();
		gasIrq = Thingy52GasIrqActive();
		s_Uart.printf("sample=%u button=%u illumination=%u irq(color/gas)=%u/%u\r\n",
			cycle++, (unsigned)pressed, (unsigned)illuminate,
			(unsigned)colorIrq, (unsigned)gasIrq);
		if (s_Color.Read(color))
		{
			s_Uart.printf("RGBC=%u,%u,%u,%u\r\n", (unsigned)color.Red,
				(unsigned)color.Green, (unsigned)color.Blue, (unsigned)color.Clear);
		}
		else
			s_Uart.printf("BH1745: no new sample or I2C error\r\n");
		if (colorIrq && !s_Color.ClearInterrupt())
			Stop("BH1745 interrupt clear failed");
		ReadSensors();
		if (s_Gas.Read(gas))
		{
			s_Uart.printf("eCO2=%u ppm eTVOC=%u ppb ADC=%u current=%u uA\r\n",
				(unsigned)gas.Eco2, (unsigned)gas.Etvoc,
				(unsigned)gas.RawAdc, (unsigned)gas.Current);
		}
		else
		{
			if (!s_Gas.ReadError(error))
				Stop("CCS811 error-register read failed");
			s_Uart.printf("CCS811: no new sample, error=0x%02x\r\n", (unsigned)error);
		}
	}
}
