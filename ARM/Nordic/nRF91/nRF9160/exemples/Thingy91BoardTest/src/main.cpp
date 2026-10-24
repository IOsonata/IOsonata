/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 user I/O, buzzer and nRF52840 reset integration test.

The test leaves the nRF52840 out of reset, emits one short buzzer tone,
cycles the active-high RGB LED and reports the active-low user button.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "audio/buzzer.h"
#include "coredev/pwm.h"
#include "coredev/uart.h"
#include "idelay.h"

#include "board.h"
#include "thingy91_board.h"

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

static const PwmCfg_t s_BuzzerPwmCfg = {
	.DevNo = THINGY91_BUZZER_PWM_DEVNO,
	.Freq = THINGY91_TEST_BUZZER_FREQ,
	.Mode = PWM_MODE_EDGE,
	.bIntEn = false,
	.IntPrio = 6,
	.pEvtHandler = NULL,
};

static const PwmChanCfg_t s_BuzzerPwmChan = {
	.Chan = THINGY91_BUZZER_PWM_CHAN,
	.Pol = PWM_POL_HIGH,
	.Port = THINGY91_BUZZER_PORT,
	.Pin = THINGY91_BUZZER_PIN,
	.PinOp = IOPINOP_GPIO,
};

UART g_Uart;
Pwm g_BuzzerPwm;
Buzzer g_Buzzer;

int main()
{
	unsigned int step;

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 board integration test\r\n");

	Thingy91UserIoInit();
	Thingy91Nrf52840ResetInit();
	Thingy91Nrf52840Reset(false);

	if (!g_BuzzerPwm.Init(s_BuzzerPwmCfg) ||
		!g_BuzzerPwm.OpenChannel(&s_BuzzerPwmChan, 1) ||
		!g_Buzzer.Init(&g_BuzzerPwm, THINGY91_BUZZER_PWM_CHAN))
	{
		g_Uart.printf("Buzzer initialization failed\r\n");
		return 1;
	}

	g_Buzzer.Volume(100);
	g_Buzzer.Stop();

#if THINGY91_TEST_BUZZER
	g_Uart.printf("Buzzer %u Hz for %u ms\r\n",
		(unsigned)THINGY91_TEST_BUZZER_FREQ,
		(unsigned)THINGY91_TEST_BUZZER_MS);
	g_Buzzer.Play(THINGY91_TEST_BUZZER_FREQ, THINGY91_TEST_BUZZER_MS);
#endif

	g_Uart.printf("nRF52840 reset deasserted\r\n");

	step = 0;
	while (1)
	{
		switch (step)
		{
		case 0:
			Thingy91LedSet(true, false, false);
			break;
		case 1:
			Thingy91LedSet(false, true, false);
			break;
		case 2:
			Thingy91LedSet(false, false, true);
			break;
		default:
			Thingy91LedSet(false, false, false);
			break;
		}

		g_Uart.printf("rgb_step=%u button=%u\r\n",
			step, Thingy91ButtonPressed() ? 1U : 0U);
		step = (step + 1U) & 3U;
		msDelay(500);
	}

	return 0;
}
