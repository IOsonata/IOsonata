/**-------------------------------------------------------------------------
@file	power_clock_irq_nrf52.h

@brief	Shared POWER_CLOCK interrupt of the nRF52

The nRF52 POWER_CLOCK vector serves two owners: the clock (MPSL in a
SoftDevice Controller build) and the USB cable events of the POWER
peripheral. Each owner provides its handler, the vector calls the ones that
are linked. With a SoftDevice, the SoftDevice owns the vector and none of
this runs.

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
#ifndef __POWER_CLOCK_IRQ_NRF52_H__
#define __POWER_CLOCK_IRQ_NRF52_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Enable the shared vector, unless already enabled by its other
 * 			owner, whose priority is then kept.
 *
 * @param	Prio : Interrupt priority
 */
void nRFPowerClockIrqEnable(uint32_t Prio);

/// Clock owner handler, called by the shared vector when linked
void nRFClockIrqHandler(void);

/// USB cable event handler, called by the shared vector when linked
void nRFUsbPowerIrqHandler(void);

#ifdef __cplusplus
}
#endif

#endif // __POWER_CLOCK_IRQ_NRF52_H__
