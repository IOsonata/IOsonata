/**-------------------------------------------------------------------------
@file	system_ra4m1.h

@brief	RA4M1 system startup and clock reporting.

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
#ifndef __SYSTEM_RA4M1_H__
#define __SYSTEM_RA4M1_H__

#include <stdint.h>
#include "coredev/system_core_clock.h"

/* Startup configuration, overridable in the library build. Oscillator type
 * and frequency remain in the application's strong g_McuOsc definition.
 * MOSC wait code 9 is 262144 MOCO cycles (nominally 32.768 ms), NOT crystal
 * cycles. A crystal that needs longer settling needs additional validation.
 */
#ifndef RA4M1_MOSC_WAIT
#define RA4M1_MOSC_WAIT           9U
#endif
#ifndef RA4M1_SOSC_DRIVE
#define RA4M1_SOSC_DRIVE          0U
#endif
#ifndef RA4M1_SOSC_STARTUP_US
#define RA4M1_SOSC_STARTUP_US     2000000UL
#endif
#ifndef RA4M1_STARTUP_TIMEOUT
#define RA4M1_STARTUP_TIMEOUT     1000000UL
#endif

/* Generic nsDelay's empirical divisor. This initial value is not calibrated
 * on RA4M1; do not use nsDelay for clock-startup or protocol timing here.
 */
#ifndef RA4M1_NSDELAY_FACTOR
#define RA4M1_NSDELAY_FACTOR      27UL
#endif

/* The normal boot image auto-starts the 24 MHz HOCO. Reset execution remains
 * MOCO /16; SystemInit changes HOCO frequency only after leaving low-voltage
 * mode. WDT and IWDT do not auto-start. Security MPU remains disabled.
 * Options are kept with the vectors and placed by gcc_ra4m1.ld.
 */
#define RA4M1_OFS0_VALUE          0xFFFFFFFFUL
#define RA4M1_OFS1_VALUE          0xFFFF8EFFUL

typedef enum __Ra4m1_Startup_Error {
	RA4M1_STARTUP_OK = 0,
	RA4M1_STARTUP_BAD_OSC,
	RA4M1_STARTUP_BAD_LF_OSC,
	RA4M1_STARTUP_BAD_ENTRY,
	RA4M1_STARTUP_PROTECT,
	RA4M1_STARTUP_HOCO,
	RA4M1_STARTUP_POWER_MODE,
	RA4M1_STARTUP_MOCO,
	RA4M1_STARTUP_MOSC,
	RA4M1_STARTUP_PLL,
	RA4M1_STARTUP_FLASH,
	RA4M1_STARTUP_CLOCK,
	RA4M1_STARTUP_LF_CLOCK,
	RA4M1_STARTUP_USB_CLOCK
} Ra4m1StartupError_t;

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;
extern uint32_t SystemnsDelayFactor;
/* Inspect this value if startup stops before main. */
extern volatile uint32_t g_Ra4m1StartupError;

void SystemInit(void);
void SystemCoreClockUpdate(void);
/* PCLK indices: 0 = A, 1 = B, 2 = C, 3 = D. */
uint32_t SystemFlashClockGet(void);
uint32_t SystemUsbClockGet(void);

#ifdef __cplusplus
}
#endif
#endif /* __SYSTEM_RA4M1_H__ */
