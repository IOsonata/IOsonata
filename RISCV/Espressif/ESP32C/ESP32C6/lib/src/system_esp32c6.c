/**-------------------------------------------------------------------------
@file	system_esp32c6.c

@brief	ESP32-C6 chip-specific SystemInit()

Installs the CLIC vector table and delegates everything else to the shared
PCR init layer (system_esp32_pcr.c).

Clock parameters (ESP32-C6 TRM Rev 0.6 §7):
  XTAL :  40 MHz
  SPLL : 480 MHz  →  /3 = 160 MHz,  /6 = 80 MHz
  FOSC :  ~17.5 MHz internal RC

Define ESP32C6_CPU_160MHZ to run at 160 MHz.
Default (no define): stay at ROM bootloader default (80 MHz).

@author	Hoang Nguyen Hoan
@date	Mar. 5, 2026

@license

MIT

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
#include <stdint.h>

/* Tell system_esp32_pcr.c the SPLL output for SystemCoreClockUpdate() */
#define ESP32_PLL_FREQ  480000000UL

extern void Esp32PcrSystemInit(uint32_t CpuHz);

/* Vector table defined in Vectors_esp32c6.c */
extern void (* const g_Esp32C6Vectors[])(void);

/**
 * @brief   System initialisation — called by ResetEntry.c before _start().
 *
 * Stack is valid; .data/.bss are NOT yet initialised.
 */
void SystemInit(void)
{
#if defined(ESP32C6_CPU_160MHZ)
    Esp32PcrSystemInit(160000000UL);
#else
    Esp32PcrSystemInit(0U);     /* 0 = leave clock at ROM bootloader default */
#endif

    /* Install CLIC vectored interrupt table.
     * mtvec[1:0] = 0b11 selects CLIC vectored mode.
     * g_Esp32C6Vectors is aligned to 512 bytes in .iram.text.          */
    uintptr_t vt = (uintptr_t)g_Esp32C6Vectors;
    __asm volatile("csrw mtvec, %0" : : "r"(vt | 0x3U) : "memory");
}
