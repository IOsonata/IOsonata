/**-------------------------------------------------------------------------
@file	pei_timer_gpiote.c

@brief	nRF52832: timer compare drives GPIOTE directly through PEI

The first LED toggles every 500 ms without a timer or GPIOTE interrupt.
Use BLYST-NANO-DK; the pin is selected from LED_PINS_MAP in board.h.

@license
MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <stdint.h>

#include "nrf.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static PeiConn_t s_Connection;

// Watch g_PeiStatus in debugger: 1 = running; 2 = unsupported LED pin;
// 3 = no PPI channel; 4 = could not enable the connection.
volatile uint32_t g_PeiStatus;

int main(void)
{
	PeiCfg_t cfg;
	uint32_t pin;

	if (s_Leds[0].PortNo != 0 || s_Leds[0].PinNo < 0 || s_Leds[0].PinNo > 31)
	{
		g_PeiStatus = 2;
		return 1;
	}

	pin = (uint32_t)s_Leds[0].PinNo;

	// Configure GPIO through IOsonata before enabling its GPIOTE task.
	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo, IOPINOP_GPIO,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	// TODO: move GPIOTE task-mode setup to the IOsonata pin driver.
	NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
		(pin << GPIOTE_CONFIG_PSEL_Pos) |
		(GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
		(GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos);

	// TODO: expose hardware-only periodic compare in IOsonata Timer driver.
	// TIMER2 runs at 1 MHz, resets itself at COMPARE0 every 500 ms.
	// Neither timer nor GPIOTE interrupts are enabled.
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER2->PRESCALER = 4;
	NRF_TIMER2->CC[0] = 500000;
	NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER2->EVENTS_COMPARE[0] = 0;
	NRF_TIMER2->TASKS_CLEAR = 1;

	cfg.DevNo = 0;
	cfg.Event = (PeiEndpoint_t)&NRF_TIMER2->EVENTS_COMPARE[0];
	cfg.Task = (PeiEndpoint_t)&NRF_GPIOTE->TASKS_OUT[0];
	cfg.Options = 0;

	if (!PeiConnect(&s_Connection, &cfg))
	{
		g_PeiStatus = 3;
		return 1;
	}
	if (!PeiEnable(&s_Connection))
	{
		g_PeiStatus = 4;
		PeiDisconnect(&s_Connection);
		return 1;
	}

	g_PeiStatus = 1;
	NRF_TIMER2->TASKS_START = 1;
	while (1)
	{
		__WFE();
	}
}
