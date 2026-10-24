/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 X BME688 compatibility test using TphgBme680.

BME688 uses the BME68x register interface supported by the existing
TphgBme680 driver. Thingy:91 X powers its sensor domain through nPM1300 LDO2
configured as a load switch; this example enables that rail before probing
the BME688 at I2C address 0x76.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "pwrmgnt/pm_npm1300.h"
#include "sensors/bsec_interface.h"
#include "sensors/tphg_bme680.h"

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
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 7,
	.EvtCB = NULL,
};

static const GasSensorHeater_t s_HeaterProfile[] = {
	{375, 125},
};

static const GasSensorCfg_t s_GasCfg = {
	BME680_I2C_DEV_ADDR0,
	SENSOR_OPMODE_SINGLE,
	500,
	sizeof(s_HeaterProfile) / sizeof(s_HeaterProfile[0]),
	s_HeaterProfile,
};

static const TPHSensorCfg_t s_TphCfg = {
	BME680_I2C_DEV_ADDR0,
	SENSOR_OPMODE_SINGLE,
	1000,
	2,
	1,
	1,
	1,
	NULL,
};

UART g_Uart;
I2C g_I2c;
PmnPM1300 g_Pmic;
TphgBme680 g_EnvSensor;

int main()
{
	TPHSensorData_t tphData;
	GasSensorData_t gasData;
	bsec_library_return_t bsecStatus;

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 X BME688 compatibility test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C initialization failed\r\n");
		return 1;
	}

	if (!g_Pmic.Init(&g_I2c) ||
		!g_Pmic.SetLoadSwitchMode(SENSOR_ENABLE_NPM1300_LDSW) ||
		!g_Pmic.SetLoadSwitchEnabled(SENSOR_ENABLE_NPM1300_LDSW, true))
	{
		g_Uart.printf("Sensor power enable failed\r\n");
		return 1;
	}

	msDelay(5);

	bsecStatus = bsec_init();

	if (bsecStatus != BSEC_OK)
	{
		g_Uart.printf("BSEC initialization failed: %d\r\n", (int)bsecStatus);
		return 1;
	}

	if (!g_EnvSensor.Init(s_TphCfg, &g_I2c, NULL))
	{
		g_Uart.printf("BME688 BME68x-compatible initialization failed\r\n");
		return 1;
	}

	if (!g_EnvSensor.Init(s_GasCfg, &g_I2c, NULL))
	{
		g_Uart.printf("BME688 gas configuration failed\r\n");
		return 1;
	}

	g_Uart.printf("BME688 detected at 0x76\r\n");

	while (1)
	{
		if (!g_EnvSensor.StartSampling())
		{
			g_Uart.printf("BME688 sample start failed\r\n");
			msDelay(100);
			continue;
		}

		msDelay(1000);

		if (g_EnvSensor.Read(tphData))
		{
			g_Uart.printf("T=%ld cC P=%ld Pa H=%ld cPct\r\n",
				(long)tphData.Temperature, (long)tphData.Pressure,
				(long)tphData.Humidity);
		}
		else
		{
			g_Uart.printf("BME688 TPH sample not ready\r\n");
		}

		if (g_EnvSensor.Read(gasData))
		{
			g_Uart.printf("Gas=%ld ohm idx=%u\r\n",
				(long)gasData.GasRes[gasData.MeasIdx],
				(unsigned)gasData.MeasIdx);
		}
		else
		{
			g_Uart.printf("BME688 gas sample not ready\r\n");
		}
	}

	return 0;
}
