/**-------------------------------------------------------------------------
@file	pei_blinky_port_nrf52.c

@brief	nRF52832 adapter for shared PEI blinky example

Prepares Timer and GPIOTE endpoints; the shared pei_blinky.cpp owns
the PEI relationship. No ISR or software event forwarding is used.

TODO: replace direct hardware preparation with Timer and GPIO endpoint
driver APIs when those become available.
----------------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>

#include "nrf.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"
#include "pei_blinky_port.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;

bool PeiBlinkyPrepare(PeiEndpoint_t *pEvent, PeiEndpoint_t *pTask)
{
	uint32_t pin;

	if (pEvent == NULL || pTask == NULL || s_Leds[0].PortNo != 0 ||
		s_Leds[0].PinNo < 0 || s_Leds[0].PinNo > 31)
	{
		return false;
	}
	pin = (uint32_t)s_Leds[0].PinNo;

	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo, IOPINOP_GPIO,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);

	NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
		(pin << GPIOTE_CONFIG_PSEL_Pos) |
		(GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
		(GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos);

	// TIMER2: 1 MHz, 500 ms toggle, hardware COMPARE0 -> CLEAR shortcut.
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER2->PRESCALER = 4;
	NRF_TIMER2->CC[0] = 500000;
	NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER2->EVENTS_COMPARE[0] = 0;
	NRF_TIMER2->TASKS_CLEAR = 1;

	*pEvent = (PeiEndpoint_t)&NRF_TIMER2->EVENTS_COMPARE[0];
	*pTask = (PeiEndpoint_t)&NRF_GPIOTE->TASKS_OUT[0];
	return true;
}

void PeiBlinkyStart(void)
{
	NRF_TIMER2->TASKS_START = 1;
}

void PeiBlinkyIdle(void)
{
	__WFE();
}
