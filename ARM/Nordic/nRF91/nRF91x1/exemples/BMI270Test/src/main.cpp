/**-------------------------------------------------------------------------
@file	main.cpp

@brief	Thingy:91 X BMI270 accelerometer/gyroscope/temperature test.

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

/** Thingy:91 X BMI270 accelerometer/gyroscope/temperature test. */
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "pwrmgnt/pm_npm1300.h"
#include "sensors/ag_bmi270.h"
#include "board.h"
#include "thingy91x_board.h"

#define UART_FIFO_SIZE CFIFO_MEMSIZE(256)

static uint8_t s_UartTxBuffer[UART_FIFO_SIZE];

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	UART_DEVNO, s_UartPins, sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	115200, 8, UART_PARITY_NONE, 1, UART_FLWCTRL_NONE,
	true, 1, NULL, true, 0, NULL, UART_FIFO_SIZE, s_UartTxBuffer,
};

static const IOPinCfg_t s_I2cPins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
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

static const IOPinCfg_t s_SpiPins[] = {
	{SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN, SPI_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{SPI_WIFI_CS_PORT, SPI_WIFI_CS_PIN, SPI_WIFI_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{SPI_BMI270_CS_PORT, SPI_BMI270_CS_PIN, SPI_BMI270_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(s_SpiPins[0]),
	.Rate = 8000000,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 7,
	.DummyByte = 0xFF,
	.EvtCB = NULL,
};

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = BMI270_SPI_CS,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.Scale = 2,
	.FltrFreq = 0,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.IntHandler = NULL,
	.bFifoEn = false,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = BMI270_SPI_CS,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.Sensitivity = 2000,
	.FltrFreq = 0,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.bFifoEn = false,
};

static const TempSensorCfg_t s_TempCfg = {
	.DevAddr = BMI270_SPI_CS,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.IntPrio = 7,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.TempOvrs = 0,
	.FilterCoeff = 0,
	.DataRdyCB = NULL,
};

UART g_Uart;
I2C g_I2c;
SPI g_Spi;
PmnPM1300 g_Pmic;
AgBmi270 g_Imu;

int main()
{
	AccelSensorData_t accel;
	GyroSensorData_t gyro;
	TempSensorData_t temp;

	if (!g_Uart.Init(s_UartCfg))
	{
		return 1;
	}

	g_Uart.printf("Thingy:91 X BMI270 test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C2 initialization failed\r\n");
		return 1;
	}
	if (!g_Pmic.Init(&g_I2c) ||
		!Thingy91XPmicConfigure(g_Pmic) ||
		!Thingy91XSensorPower(g_Pmic, true))
	{
		g_Uart.printf("sensor power enable failed\r\n");
		return 1;
	}

	msDelay(5);

	if (!g_Spi.Init(s_SpiCfg))
	{
		g_Uart.printf("SPI3 initialization failed\r\n");
		return 1;
	}

	if (!g_Imu.Init(s_AccelCfg, &g_Spi, NULL) ||
		!g_Imu.Init(s_GyroCfg, &g_Spi, NULL) ||
		!g_Imu.Init(s_TempCfg, &g_Spi, NULL))
	{
		g_Uart.printf("BMI270 initialization failed\r\n");
		return 1;
	}

	g_Uart.printf("BMI270 ready: SPI3 8 MHz CS2 P0.10, IRQ P0.06 polling\r\n");

	while (1)
	{
		if (g_Imu.UpdateData())
		{
			if (g_Imu.Read(accel) && g_Imu.Read(gyro))
			{
				g_Imu.TempBmi270::Read(temp);
				g_Uart.printf("A %.4f %.4f %.4f g  G %.2f %.2f %.2f dps  T %.2f C\r\n",
					(double)accel.X, (double)accel.Y, (double)accel.Z,
					(double)gyro.X, (double)gyro.Y, (double)gyro.Z,
					(double)temp.Temperature / 100.0);
			}
		}
		msDelay(100);
	}
}
