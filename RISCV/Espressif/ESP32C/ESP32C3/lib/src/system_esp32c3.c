/**-------------------------------------------------------------------------
@file	system_esp32c3.c

@brief	ESP32-C3 chip-specific SystemInit()

Calls the shared SYSTEM-peripheral init layer (system_esp32_system.c) to
disable watchdogs and optionally switch the CPU to 160 MHz, then installs
RISCV_TrapHandler into mtvec in direct mode.

Clock parameters (ESP32-C3 TRM Rev 0.4 §7):
  XTAL         :  40 MHz
  PLL_F80M     :  80 MHz  (ROM bootloader default)
  PLL_F160M    : 160 MHz

Define ESP32C3_CPU_160MHZ to run at 160 MHz.
Default (no define): stay at 80 MHz (ROM bootloader default).

Interrupt controller: CLINT (direct mode, mtvec[1:0] = 0b00).
All traps routed to RISCV_TrapHandler (defined in Vectors_esp32c3.c).

@author	Hoang Nguyen Hoan
@date	Mar. 6, 2026

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

extern void Esp32SystemInit(void);

/* Trap handler defined in Vectors_esp32c3.c */
extern void RISCV_TrapHandler(void);

/**
 * @brief   System initialisation — called by ResetEntry.c before _start().
 *
 * Stack is valid; .data/.bss are NOT yet initialised.
 */
void SystemInit(void)
{
	Esp32SystemInit();

	/*
	 * Install trap handler in direct mode (mtvec[1:0] = 0b00).
	 * All exceptions and interrupts are routed to RISCV_TrapHandler,
	 * which dispatches by mcause in software.
	 *
	 * RISCV_TrapHandler must be 4-byte aligned (guaranteed by its
	 * __attribute__((interrupt("machine"), aligned(4))) declaration).
	 */
	uintptr_t vt = (uintptr_t)RISCV_TrapHandler;
	__asm volatile("csrw mtvec, %0" : : "r"(vt & ~3U) : "memory");
}
