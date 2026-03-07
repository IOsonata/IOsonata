/**-------------------------------------------------------------------------
@file	Vectors_esp32_intmtx.c

@brief	Shared trap/exception handlers for Espressif RISC-V INTMTX targets

Provides DEF_IRQHandler and the 32 standard RISC-V synchronous exception
cause handlers (mcause values 0..31, interrupt bit = 0) common to all
Espressif chips using the INTMTX interrupt matrix with CLINT vectoring:

    ESP32-C3

These symbols are referenced by the chip-specific Vectors_espXXXX.c file
which provides the peripheral dispatch table and the top-level trap handler.

NOT used by ESP32-C5, ESP32-C6, ESP32-H2, ESP32-H4 — those use CLIC and
have their own Vectors_esp32_clic.c counterpart.

DEF_IRQHandler is defined __attribute__((weak)) so an application or
board-level file can replace it with a diagnostic handler.

@author	Hoang Nguyen Hoan
@date	Mar. 6, 2026

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
 * Standard RISC-V synchronous exception cause handlers (mcause 0..31,
 * interrupt bit = 0).
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
/* cause 16..31 — Reserved */
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

/*---------------------------------------------------------------------------
 * Exception dispatch table — indexed by mcause (interrupt bit = 0).
 * Used by RISCV_TrapHandler in Vectors_esp32c3.c.
 *---------------------------------------------------------------------------*/
void (* const g_Esp32IntmtxExcVectors[32])(void) =
{
	[0]  = InstrAddrMisalign_Handler,
	[1]  = InstrAccessFault_Handler,
	[2]  = IllegalInstr_Handler,
	[3]  = Breakpoint_Handler,
	[4]  = LoadAddrMisalign_Handler,
	[5]  = LoadAccessFault_Handler,
	[6]  = StoreAddrMisalign_Handler,
	[7]  = StoreAccessFault_Handler,
	[8]  = ECallU_Handler,
	[9]  = ECallS_Handler,
	[10] = Reserved10_Handler,
	[11] = ECallM_Handler,
	[12] = InstrPageFault_Handler,
	[13] = LoadPageFault_Handler,
	[14] = Reserved14_Handler,
	[15] = StorePageFault_Handler,
	[16] = Reserved16_Handler,
	[17] = Reserved17_Handler,
	[18] = Reserved18_Handler,
	[19] = Reserved19_Handler,
	[20] = Reserved20_Handler,
	[21] = Reserved21_Handler,
	[22] = Reserved22_Handler,
	[23] = Reserved23_Handler,
	[24] = Reserved24_Handler,
	[25] = Reserved25_Handler,
	[26] = Reserved26_Handler,
	[27] = Reserved27_Handler,
	[28] = Reserved28_Handler,
	[29] = Reserved29_Handler,
	[30] = Reserved30_Handler,
	[31] = Reserved31_Handler,
};
