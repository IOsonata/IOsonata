/**-------------------------------------------------------------------------
@file	system_LPC54628.h

@brief	CMSIS system header for the LPC54628

		Included by the LPC54628.h device header. The clock implementation
		is the family one, system_LPC546xx.c.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#ifndef __SYSTEM_LPC54628_H__
#define __SYSTEM_LPC54628_H__

#include <stdint.h>

#define DEFAULT_SYSTEM_CLOCK			12000000u	//!< Core clock after reset, FRO 12 MHz
#define SYSTEM_CORE_CLOCK_MAX			220000000u	//!< Highest core clock
#define CLK_RTC_32K_CLK					32768u		//!< RTC oscillator 32 kHz output (32k_clk)
#define CLK_FRO_12MHZ					12000000u	//!< FRO 12 MHz (fro_12m)
#define CLK_FRO_48MHZ					48000000u	//!< FRO 48 MHz (fro_48m)
#define CLK_FRO_96MHZ					96000000u	//!< FRO 96 MHz (fro_96m)

#ifdef __cplusplus
extern "C" {
#endif

/// Core clock frequency in Hz, updated by SystemCoreClockUpdate
extern uint32_t SystemCoreClock;

/**
 * @brief	Setup the microcontroller system.
 *
 * Called by ResetEntry before main. Runs the core at SYSTEM_CORE_CLOCK_MAX
 * from the oscillator in g_McuOsc.
 */
void SystemInit(void);

/**
 * @brief	Update SystemCoreClock from the clock registers.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
