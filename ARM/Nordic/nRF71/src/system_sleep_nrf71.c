/**-------------------------------------------------------------------------
@file	system_sleep_nrf71.c
@brief	Resumable System ON idle for the nRF71 application core.

The CPU waits for an interrupt while the peripheral, radio, LFCLK and PLL
domains remain under their existing owners. This does not enter System OFF,
reset the MCU, or configure any wake source. System OFF on a non-secure image
requires the platform's secure power service.

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
#include "nrf.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

void SystemSleep(void)
{
	uint32_t state;
	uint32_t scr;

	if (__get_IPSR() != 0U)
	{
		return;
	}

	state = DisableInterrupt();
	scr = SCB->SCR;
	SCB->SCR = scr & ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
	__DSB();
	__WFI();
	__ISB();
	SCB->SCR = scr;
	EnableInterrupt(state);
}
