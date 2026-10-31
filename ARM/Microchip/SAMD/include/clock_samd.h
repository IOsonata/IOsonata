/**-------------------------------------------------------------------------
@file	clock_samd.h

@brief	SAM D11/D21 generic clock generators and channels

SystemInit sets these generic clock generators:

	SAMD_GCLK_MAIN   (0) : main clock, CPU, buses and the SERCOM, TC and EIC
	                       core clocks
	SAMD_GCLK_LF     (1) : 32.768 kHz low frequency clock, XOSC32K or OSC32K,
	                       RTC clock and DFLL48M reference
	SAMD_GCLK_DPLLREF (2): OSC8M / 8, FDPLL96M reference when no crystal is
	                       used

SystemPeriphClockGet(n) returns the frequency of generator n.

Channel IDs are device specific, use the GCLK_CLKCTRL_ID_xxx_Val values of the
Microchip device header.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#ifndef __CLOCK_SAMD_H__
#define __CLOCK_SAMD_H__

#include <stdbool.h>
#include <stdint.h>

#define SAMD_GCLK_MAIN			0	//!< Main clock generator
#define SAMD_GCLK_LF			1	//!< 32.768 kHz low frequency clock generator
#define SAMD_GCLK_DPLLREF		2	//!< FDPLL96M reference without crystal

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Connect a peripheral generic clock channel to a generator.
 *
 * A channel already running on another generator is stopped first, as the
 * GCLK requires before a change of generator. CLKEN is written and not read
 * back, as in ASF and Harmony: the peripheral shows whether its clock runs.
 *
 * @param	Channel   : Channel ID of the device header
 * @param	Generator : Generator number
 *
 * @return	true - channel written with that generator and CLKEN
 *			false - locked on another setting, or the stop or the GCLK
 *			synchronization did not complete
 */
bool SamdGclkChannelEnable(uint8_t Channel, uint8_t Generator);

/**
 * @brief	Stop a peripheral generic clock channel.
 *
 * @param	Channel : Channel ID of the device header
 *
 * @return	true - channel stopped
 */
bool SamdGclkChannelDisable(uint8_t Channel);

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_SAMD_H__
