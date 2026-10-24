/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 BH1749NUC color-sensor test.

The example checks device identity, configures 1x RGB/IR gain at 120 ms,
programs a non-triggering red-channel interrupt window, and prints raw RGB,
IR and second-green samples. The BH1749 output is raw channel data; no
uncalibrated lux or color-temperature conversion is performed.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "sensors/color_bh1749.h"

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

UART g_Uart;
I2C g_I2c;
ColorBh1749 g_Color;

int main()
{
	ColorBh1749Data_t data;
	bool interruptActive;

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 BH1749 test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C initialization failed\r\n");
		return 1;
	}

	if (!g_Color.Init(&g_I2c, BH1749_I2C_ADDR0))
	{
		g_Uart.printf("BH1749 initialization failed\r\n");
		return 1;
	}

	if (!g_Color.Configure(COLOR_BH1749_GAIN_1X, COLOR_BH1749_GAIN_1X,
		COLOR_BH1749_MEASURE_120MS))
	{
		g_Uart.printf("BH1749 measurement configuration failed\r\n");
		return 1;
	}

	if (!g_Color.ConfigureInterrupt(COLOR_BH1749_INT_RED, 0, 0xFFFF,
		COLOR_BH1749_INT_UPDATE_EACH_MEASUREMENT))
	{
		g_Uart.printf("BH1749 interrupt configuration failed\r\n");
		return 1;
	}

	while (1)
	{
		msDelay(150);

		if (g_Color.Read(data))
		{
			g_Uart.printf("R=%u G=%u B=%u IR=%u G2=%u\r\n",
				(unsigned)data.Red, (unsigned)data.Green,
				(unsigned)data.Blue, (unsigned)data.Ir,
				(unsigned)data.Green2);
		}

		if (!g_Color.ReadInterruptStatus(interruptActive))
		{
			g_Uart.printf("BH1749 interrupt-status read failed\r\n");
			return 1;
		}

		if (interruptActive)
			g_Uart.printf("BH1749 threshold interrupt\r\n");
	}

	return 0;
}
