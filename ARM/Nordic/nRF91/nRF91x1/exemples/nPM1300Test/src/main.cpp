/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 X nPM1300 PMIC test.

The default test is board-safe: it attaches without changing charger or
regulator state, configures only the NTC ADC selector, then reports PMIC,
battery-voltage, current, temperature, BUCK, load-switch and board GPIO status.
Programming tests are compile-time guarded and disabled by default.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include <limits.h>

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "pwrmgnt/pm_npm1300.h"

#include "board.h"
#include "thingy91x_board.h"

#define UART_FIFO_SIZE	CFIFO_MEMSIZE(256)

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

static const BatProfile_t s_BatteryProfile = {
	.OpVolt = 3700,
	.ChrgVolt = 4200,
	.Capacity = 1350,
	.ThermBetaConst = 3435,
	.ThermResistor = 10,
};

UART g_Uart;
I2C g_I2c;
PmnPM1300 g_Pmic;

static void PrintStatus()
{
	uint8_t charger;
	uint8_t chargerErrors;
	uint8_t vbus;
	uint8_t ntc;
	uint8_t resetCause;
	uint8_t voltageCode;
	uint8_t buckStatus;
	uint8_t ldswStatus;
	int32_t batteryMv;
	int32_t currentUa;
	int32_t temperature;
	bool npm6001Ready;

	if (!g_Pmic.ReadStatus(charger, chargerErrors, vbus, ntc, resetCause))
	{
		g_Uart.printf("nPM1300 status read failed\r\n");
		return;
	}

	batteryMv = g_Pmic.Voltage();
	currentUa = g_Pmic.Current();
	temperature = g_Pmic.Temperature();

	g_Uart.printf("charger=0x%02x errors=0x%02x vbus=0x%02x ntc=0x%02x reset=0x%02x\r\n",
		(unsigned)charger, (unsigned)chargerErrors, (unsigned)vbus,
		(unsigned)ntc, (unsigned)resetCause);
	g_Uart.printf("VBAT=%ld mV temp=%ld dC battery_state=%u level=%u charging=%u\r\n",
		(long)batteryMv, (long)temperature, (unsigned)g_Pmic.BatteryState(),
		(unsigned)g_Pmic.Level(), g_Pmic.Charging() ? 1U : 0U);

	if (currentUa != INT32_MIN)
		g_Uart.printf("IBAT=%ld uA (positive=discharge)\r\n", (long)currentUa);
	else
		g_Uart.printf("IBAT unavailable\r\n");
	g_Uart.printf("SOC unavailable: no portable battery estimator configured\r\n");

	if (g_Pmic.ReadBuckStatus(0, voltageCode, buckStatus))
		g_Uart.printf("BUCK1 code=%u status=0x%02x\r\n",
			(unsigned)voltageCode, (unsigned)buckStatus);
	if (g_Pmic.ReadBuckStatus(1, voltageCode, buckStatus))
		g_Uart.printf("BUCK2 code=%u status=0x%02x\r\n",
			(unsigned)voltageCode, (unsigned)buckStatus);
	if (g_Pmic.ReadLoadSwitchStatus(ldswStatus))
		g_Uart.printf("LDSW status=0x%02x\r\n", (unsigned)ldswStatus);
	if (Thingy91XNpm6001Ready(g_Pmic, npm6001Ready))
		g_Uart.printf("GPIO4 nPM6001_READY=%u\r\n", npm6001Ready ? 1U : 0U);
}

int main()
{
	FuelGaugeCfg_t fuelCfg = {
		NPM1300_I2C_7BITS_ADDR,
		s_BatteryProfile,
		NULL
	};

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 X nPM1300 test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C initialization failed\r\n");
		return 1;
	}

	if (!g_Pmic.Init(fuelCfg, &g_I2c, &g_Pmic))
	{
		g_Uart.printf("nPM1300 initialization failed\r\n");
		return 1;
	}

#if NPM1300_TEST_PROGRAM_CHARGER
	if (!g_Pmic.SetInputCurrentLimit(500) ||
		g_Pmic.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 675) != 674)
	{
		g_Uart.printf("charger programming failed\r\n");
		return 1;
	}
#endif

#if NPM1300_TEST_PROGRAM_RAILS
	if (g_Pmic.SetVout(THINGY91X_BUCK2_INDEX, 3300, 0) != 3300 ||
		!Thingy91XSensorPower(g_Pmic, true))
	{
		g_Uart.printf("rail programming failed\r\n");
		return 1;
	}
#endif

	PrintStatus();

	while (1)
	{
		msDelay(5000);
		PrintStatus();
	}

	return 0;
}
