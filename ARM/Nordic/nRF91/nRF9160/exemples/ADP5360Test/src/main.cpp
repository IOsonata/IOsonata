/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 ADP5360 PMIC status test.

The default test is read-only. It verifies the ADP5360 device ID, reports the
existing 1.8 V BUCK and 3.3 V BUCKBOOST settings, charger/battery status,
fuel-gauge readings, fault flags and power-good flags.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "pwrmgnt/pm_adp5360.h"

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
PmAdp5360 g_Pmic;

int main()
{
	Adp5360ChargerMode_t chargerMode;
	Adp5360BatteryState_t batteryState;
	int32_t milliVolts;
	uint32_t milliAmps;
	uint16_t batteryMilliVolts;
	uint8_t batteryLevel;
	uint8_t faults;
	uint8_t powerGood;
	bool enabled;

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 ADP5360 test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C initialization failed\r\n");
		return 1;
	}

	if (!g_Pmic.Init(&g_I2c))
	{
		g_Uart.printf("ADP5360 initialization failed\r\n");
		return 1;
	}

	if (!g_Pmic.GetVout(ADP5360_VOUT_BUCK, milliVolts, milliAmps, enabled))
	{
		g_Uart.printf("ADP5360 BUCK read failed\r\n");
		return 1;
	}

	g_Uart.printf("BUCK: %ld mV limit=%lu mA enabled=%u\r\n",
		(long)milliVolts, (unsigned long)milliAmps, enabled ? 1U : 0U);

	if (!g_Pmic.GetVout(ADP5360_VOUT_BUCKBOOST, milliVolts, milliAmps,
		enabled))
	{
		g_Uart.printf("ADP5360 BUCKBOOST read failed\r\n");
		return 1;
	}

	g_Uart.printf("BUCKBOOST: %ld mV limit=%lu mA enabled=%u\r\n",
		(long)milliVolts, (unsigned long)milliAmps, enabled ? 1U : 0U);

#if ADP5360_TEST_PROGRAM_RAILS
	if (g_Pmic.SetVout(ADP5360_VOUT_BUCK, 1800, 0) != 1800 ||
		g_Pmic.SetVout(ADP5360_VOUT_BUCKBOOST, 3300, 0) != 3300)
	{
		g_Uart.printf("ADP5360 regulator programming failed\r\n");
		return 1;
	}
#endif

	if (g_Pmic.ChargerMode(chargerMode))
		g_Uart.printf("Charger mode=%u\r\n", (unsigned)chargerMode);
	else
		g_Uart.printf("Charger status unavailable\r\n");

	if (g_Pmic.BatteryState(batteryState))
		g_Uart.printf("Battery state=%u\r\n", (unsigned)batteryState);
	else
		g_Uart.printf("Battery state unavailable\r\n");

	if (g_Pmic.BatteryLevel(batteryLevel))
		g_Uart.printf("Battery SOC=%u %%\r\n", (unsigned)batteryLevel);
	else
		g_Uart.printf("Battery SOC unavailable\r\n");

	if (g_Pmic.BatteryVoltage(batteryMilliVolts))
		g_Uart.printf("Battery voltage=%u mV\r\n",
			(unsigned)batteryMilliVolts);
	else
		g_Uart.printf("Battery voltage unavailable\r\n");

	if (!g_Pmic.ReadFaultStatus(faults) ||
		!g_Pmic.ReadPowerGood(powerGood))
	{
		g_Uart.printf("ADP5360 fault/power-good read failed\r\n");
		return 1;
	}

	g_Uart.printf("Fault=0x%02x PGOOD=0x%02x\r\n",
		(unsigned)faults, (unsigned)powerGood);

	while (1)
	{
		msDelay(1000);
	}

	return 0;
}
