/**-------------------------------------------------------------------------
@file	system_esp32c6.c

@brief	ESP32-C6 chip-specific SystemInit()

Installs the INTMTX CPU vector table and delegates everything else to the shared
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

#include "esp32xx.h"
#include "esp32xx_intmtx.h"
#include "esp32xx_uart.h"

extern void Esp32PcrSystemInit(uint32_t CpuHz);
extern void RISCV_VectorTable(void);

volatile uintptr_t g_Esp32C6VectorBase;
volatile uintptr_t g_Esp32C6MtvecWritten;
volatile uintptr_t g_Esp32C6MtvecReadback;
volatile uint32_t g_Esp32C6VectorRangeError;

void SystemInit(void)
{
    uintptr_t vt;
    uintptr_t vector;

#if defined(ESP32C6_CPU_160MHZ)
    Esp32PcrSystemInit(160000000UL);
#else
    Esp32PcrSystemInit(0U);
#endif

	// Enable the cycle event used by idelay.h (Espressif PCER/PCMR/PCCR).
	__asm volatile("csrw 0x7E0, %0" : : "r"(1U));
	__asm volatile("csrw 0x7E2, zero");
	__asm volatile("csrw 0x7E1, %0" : : "r"(1U));

    vt = (uintptr_t)RISCV_VectorTable;
    g_Esp32C6VectorBase = vt;
    // Keep the CPU trap table in executable SRAM above the cache reservation.
    if ((vt & 0xFFUL) != 0U || vt < 0x40808000UL || vt >= 0x40810000UL)
    {
        g_Esp32C6VectorRangeError = 1U;
        for (;;)
        {
            __asm volatile("nop");
        }
    }

    __asm volatile("csrci mstatus, 0x8" ::: "memory");

    // Clear ROM routing and CPU priorities before installing drivers.
    ESP32_INTMTX_CLOCK_GATE_REG = 1U;
    ESP32_CPU_INT_ENABLE_REG = 0U;
    ESP32_CPU_INT_TYPE_REG = 0U;
    ESP32_CPU_INT_CLEAR_REG = 0xFFFFFFFFUL;
    ESP32_CPU_INT_CLEAR_REG = 0U;
    ESP32_CPU_INT_THRESH_REG = 0U;
    ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART0) = 0U;
    ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART1) = 0U;
    ESP32_INTMTX_SOURCE_MAP_REG(ESP32_GPIO_INTR_SOURCE_ID) = 0U;
    __asm volatile("fence iorw, iorw" ::: "memory");

    // ESP32-C6 uses basic RISC-V CPU interrupt vectoring (mtvec.MODE = 1),
    // not the CLIC-only mtvec.MODE = 3 used by ESP32-C5/C61.
    vector = vt | 1UL;
    g_Esp32C6MtvecWritten = vector;
    __asm volatile("csrw mtvec, %0" : : "r"(vector) : "memory");
    __asm volatile("csrr %0, mtvec" : "=r"(g_Esp32C6MtvecReadback));

	// PLIC input enables do not replace the CPU mie mask. ROM direct boot
	// can leave mie clear. Unmask after mtvec is installed, with every PLIC
	// input still disabled and mstatus.MIE clear. Drivers enable their inputs.
	__asm volatile("csrw mie, %0" : : "r"(UINT32_MAX) : "memory");
}
