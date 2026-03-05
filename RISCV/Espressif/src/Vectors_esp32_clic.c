/**-------------------------------------------------------------------------
@file	Vectors_clic.c

@brief	Shared CLIC exception handlers for Espressif RISC-V CLIC targets

Provides DEF_IRQHandler and the 32 standard RISC-V exception/trap cause
handler slots (mcause values 0..31 in non-interrupt mode) that are common
to all Espressif chips using the CLIC interrupt controller:

    ESP32-C5, ESP32-C6, ESP32-H2, ESP32-H4

This file is compiled once and linked into every CLIC-based target.  The
chip-specific Vectors_espXXXX.c file provides the peripheral IRQ weak
aliases and assembles the full vector table by referencing these symbols.

NOT used by ESP32-C3 / ESP32-H2 variants with INTMTX — those have their
own Vectors_intmtx.c counterpart.

DEF_IRQHandler is defined __attribute__((weak)) so an application or
board-level file can replace it with a diagnostic handler (e.g. one that
logs mcause and mepc before halting).

@author	Hoang Nguyen Hoan
@date	Mar. 5, 2026

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
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Default handler — spins so a debugger can catch unhandled interrupts.
 *
 * Placed in IRAM: must be reachable without Flash XIP cache, particularly
 * during early boot or if the cache is disabled by a fault handler itself.
 *
 * Declared weak so the application can override it.
 *---------------------------------------------------------------------------*/
__attribute__((weak, section(".iram.text"), used))
void DEF_IRQHandler(void)
{
	while (1)
	{
		__asm volatile("nop");
	}
}

/*---------------------------------------------------------------------------
 * Standard RISC-V synchronous exception cause handlers (mcause 0..31).
 *
 * All are weak aliases to DEF_IRQHandler.  Peripheral drivers or board
 * code override individual handlers by providing a non-weak function with
 * the same name.
 *
 * Cause assignments per RISC-V Privileged Specification §3.1.15.
 *---------------------------------------------------------------------------*/

/* cause 0  — Instruction address misaligned */
__attribute__((weak, alias("DEF_IRQHandler"))) void InstrAddrMisalign_Handler(void);
/* cause 1  — Instruction access fault */
__attribute__((weak, alias("DEF_IRQHandler"))) void InstrAccessFault_Handler(void);
/* cause 2  — Illegal instruction */
__attribute__((weak, alias("DEF_IRQHandler"))) void IllegalInstr_Handler(void);
/* cause 3  — Breakpoint */
__attribute__((weak, alias("DEF_IRQHandler"))) void Breakpoint_Handler(void);
/* cause 4  — Load address misaligned */
__attribute__((weak, alias("DEF_IRQHandler"))) void LoadAddrMisalign_Handler(void);
/* cause 5  — Load access fault */
__attribute__((weak, alias("DEF_IRQHandler"))) void LoadAccessFault_Handler(void);
/* cause 6  — Store/AMO address misaligned */
__attribute__((weak, alias("DEF_IRQHandler"))) void StoreAddrMisalign_Handler(void);
/* cause 7  — Store/AMO access fault */
__attribute__((weak, alias("DEF_IRQHandler"))) void StoreAccessFault_Handler(void);
/* cause 8  — Environment call from U-mode */
__attribute__((weak, alias("DEF_IRQHandler"))) void ECallU_Handler(void);
/* cause 9  — Environment call from S-mode */
__attribute__((weak, alias("DEF_IRQHandler"))) void ECallS_Handler(void);
/* cause 10 — Reserved */
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved10_Handler(void);
/* cause 11 — Environment call from M-mode */
__attribute__((weak, alias("DEF_IRQHandler"))) void ECallM_Handler(void);
/* cause 12 — Instruction page fault */
__attribute__((weak, alias("DEF_IRQHandler"))) void InstrPageFault_Handler(void);
/* cause 13 — Load page fault */
__attribute__((weak, alias("DEF_IRQHandler"))) void LoadPageFault_Handler(void);
/* cause 14 — Reserved */
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved14_Handler(void);
/* cause 15 — Store/AMO page fault */
__attribute__((weak, alias("DEF_IRQHandler"))) void StorePageFault_Handler(void);
/* causes 16..31 — Reserved / implementation-defined */
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved16_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved17_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved18_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved19_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved20_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved21_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved22_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved23_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved24_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved25_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved26_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved27_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved28_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved29_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved30_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved31_Handler(void);
