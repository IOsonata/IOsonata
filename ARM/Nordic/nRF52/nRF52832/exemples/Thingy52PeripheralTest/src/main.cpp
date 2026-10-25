/** Thingy:52 BH1745, CCS811 and SX1509 validation. SPDX-License-Identifier: MIT */
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "sensors/color_bh1745.h"
#include "sensors/gas_ccs811.h"
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


static UART s_Uart;
static I2C s_I2c;
static IoExpSx1509 s_Expander;
static ColorBh1745 s_Color;
static GasCcs811 s_Gas;

[[noreturn]] static void Stop(const char *pReason)
{
	s_Uart.printf("STOP: %s\r\n", pReason);
	while (1)
		msDelay(1000);
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
	s_Uart.printf("Thingy:52 BH1745 / CCS811 / SX1509 test\r\n");
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
	s_Uart.printf("Gas readings require conditioning; environment compensation uses sensor defaults.\r\n");
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
