/**-------------------------------------------------------------------------
@file	power_clock_irq_nrf52.cpp

@brief	Shared POWER_CLOCK interrupt vector of the nRF52

Calls the handler of each owner, the clock and the USB cable events, see
power_clock_irq_nrf52.h. Linked through nRFPowerClockIrqEnable, which both
owners call. The handlers are weak references: a build without one of the
owners does not link it, and the vector skips it.

@author	Hoang Nguyen Hoan
@date	Oct. 3, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "power_clock_irq_nrf52.h"

// Weak references, null when the owner is not linked
extern "C" void nRFClockIrqHandler(void) __attribute__((weak));
extern "C" void nRFUsbPowerIrqHandler(void) __attribute__((weak));

void nRFPowerClockIrqEnable(uint32_t Prio)
{
	if (NVIC_GetEnableIRQ(POWER_CLOCK_IRQn) != 0U)
	{
		return;
	}

	NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
	NVIC_SetPriority(POWER_CLOCK_IRQn, Prio);
	NVIC_EnableIRQ(POWER_CLOCK_IRQn);
}

extern "C" void POWER_CLOCK_IRQHandler(void)
{
	if (nRFClockIrqHandler != nullptr)
	{
		nRFClockIrqHandler();
	}

	if (nRFUsbPowerIrqHandler != nullptr)
	{
		nRFUsbPowerIrqHandler();
	}
}
