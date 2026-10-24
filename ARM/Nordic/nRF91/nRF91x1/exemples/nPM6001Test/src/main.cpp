/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 X nPM6001 BUCK3 and fault-monitoring test.

The nPM6001 is powered through nPM1300 LDO1 configured as a load switch.
Its READY signal is read through nPM1300 GPIO4. The test leaves BUCK3 voltage
unchanged by default, enables BUCK3, selects PWM operation, and reports
thermal/overcurrent status.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "pwrmgnt/pm_npm1300.h"
#include "pwrmgnt/pm_npm6001.h"

#include "board.h"
#include "thingy91x_board.h"

#define UART_FIFO_SIZE				CFIFO_MEMSIZE(256)
#define NPM6001_READY_TIMEOUT_MS	100

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

UART g_Uart;
I2C g_I2c;
PmnPM1300 g_MainPmic;
PmnPM6001 g_WifiPmic;

int main()
{
	uint16_t milliVolts;
	uint8_t pending;
	unsigned int i;
	bool internalDac;
	bool forcePwm;
	bool ready;
	bool thermalWarning;
	bool overcurrent;

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 X nPM6001 test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C initialization failed\r\n");
		return 1;
	}

	if (!g_MainPmic.Init(&g_I2c) || !Thingy91XPmicConfigure(g_MainPmic))
	{
		g_Uart.printf("nPM1300 board initialization failed\r\n");
		return 1;
	}

	if (!Thingy91XNpm6001Power(g_MainPmic, true))
	{
		g_Uart.printf("nPM6001 ENABLE sequence failed\r\n");
		return 1;
	}

	ready = false;

	for (i = 0; i < NPM6001_READY_TIMEOUT_MS; i++)
	{
		if (!Thingy91XNpm6001Ready(g_MainPmic, ready))
		{
			g_Uart.printf("nPM6001 READY read failed\r\n");
			return 1;
		}

		if (ready)
			break;

		msDelay(1);
	}

	if (!ready)
	{
		g_Uart.printf("nPM6001 READY timeout\r\n");
		return 1;
	}

	if (!g_WifiPmic.Init(&g_I2c))
	{
		g_Uart.printf("nPM6001 initialization failed\r\n");
		return 1;
	}

	if (!g_WifiPmic.GetBuck3Voltage(milliVolts, internalDac))
	{
		g_Uart.printf("BUCK3 voltage read failed\r\n");
		return 1;
	}

	g_Uart.printf("BUCK3=%u mV source=%s\r\n", (unsigned)milliVolts,
		internalDac ? "DAC" : "external");

#if NPM6001_TEST_SET_VOLTAGE
	if (!g_WifiPmic.SetBuck3Voltage(NPM6001_TEST_VOLTAGE_MV))
	{
		g_Uart.printf("BUCK3 voltage programming failed\r\n");
		return 1;
	}
#endif

	if (!g_WifiPmic.SetSoftwareReady(true) ||
		!g_WifiPmic.SetBuck3Enabled(true) ||
		!g_WifiPmic.SetBuck3PwmMode(true))
	{
		g_Uart.printf("BUCK3 start/PWM configuration failed\r\n");
		return 1;
	}

	usDelay(300);

	if (!g_WifiPmic.GetBuck3PwmMode(forcePwm) || !forcePwm)
	{
		g_Uart.printf("BUCK3 PWM verification failed\r\n");
		return 1;
	}

	if (!g_WifiPmic.EnableFaultInterrupts(NPM6001_FAULT_INT_MASK, true))
	{
		g_Uart.printf("nPM6001 fault interrupt configuration failed\r\n");
		return 1;
	}

	if (!g_WifiPmic.ReadThermalWarning(thermalWarning) ||
		!g_WifiPmic.ReadBuck3Overcurrent(overcurrent) ||
		!g_WifiPmic.ReadInterruptPending(pending))
	{
		g_Uart.printf("nPM6001 fault-status read failed\r\n");
		return 1;
	}

	g_Uart.printf("BUCK3 PWM=1 thermal=%u overcurrent=%u pending=0x%02x\r\n",
		thermalWarning ? 1U : 0U, overcurrent ? 1U : 0U,
		(unsigned)pending);

	while (1)
	{
		msDelay(1000);
	}

	return 0;
}
