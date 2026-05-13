/**-------------------------------------------------------------------------
@file	Vectors_R9A02.c

@brief	CLIC vector table for the Renesas R9A02G021 (Andes N22 RISC-V).

The R9A02G021 interrupt controller (ICU) connects up to 32 software-routed
peripheral interrupt slots to the Andes N22 CLIC.  Each slot is configured
at runtime by writing the desired Event Link Source ID to ICU->IELSR[n];
the corresponding handler is named IELSR<n>_IRQHandler and is weakly
aliased to DEF_IRQHandler so a driver can override it by defining a
non-weak function with the same name.

Vector table layout (64 entries, 4 bytes each = 256 bytes, naturally
power-of-two aligned for CLIC mtvt):

    [0..31]   RISC-V exception cause handlers   (mcause 0..31 non-interrupt)
    [32..63]  R9A02G021 ICU peripheral slots    (IELSR0_IRQHandler ... IELSR31_IRQHandler)

This table is self-contained -- the exception handlers are defined here
rather than pulled from a shared file, so the Renesas R9A02 tree has no
build-time dependency on the Espressif tree.

Installing the table:
  - Andes N22 CLIC vectored mode uses mtvt (CSR 0x307) for the table base
    and mtvec[1:0] = 11b to enable vectored dispatch.
  - Application or library calls VectorInstall_R9A02() after SystemInit()
    to enable hardware vectoring.  Direct (non-vectored) mtvec mode keeps
    working without this call -- traps still land in the default trap
    handler installed by SystemInit().

Routing a peripheral event to a CLIC slot:
  ICU->IELSR[5] = R9A02_EVENT_SCI0_RXI;        // wire SCI0 RX to slot 5
  void IELSR5_IRQHandler(void) { ... }         // user-supplied handler

Reference: R9A02G021 User's Manual: Hardware (R01UH1059EJ) section on ICU
and CLIC; AndeStar V5 Privileged Architecture Manual for mtvt/mintstatus.

@author	Nguyen Hoan Hoang
@date	May 12, 2026

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

/*===========================================================================
 * Default trap / IRQ handler
 *
 * Weak so application or board code can replace with a diagnostic handler
 * that reads mcause, mepc, mtval and reports the fault before halting.
 *==========================================================================*/
__attribute__((weak, used))
void DEF_IRQHandler(void)
{
    while (1)
    {
        __asm volatile ("nop");
    }
}

/*===========================================================================
 * RISC-V synchronous exception handlers (mcause 0..31, interrupt=0)
 *
 * Cause assignments per RISC-V Privileged Specification.
 *==========================================================================*/

__attribute__((weak, alias("DEF_IRQHandler"))) void InstrAddrMisalign_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void InstrAccessFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IllegalInstr_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Breakpoint_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LoadAddrMisalign_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LoadAccessFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void StoreAddrMisalign_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void StoreAccessFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECallU_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECallS_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved10_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECallM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void InstrPageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LoadPageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Reserved14_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void StorePageFault_Handler(void);
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

/*===========================================================================
 * R9A02G021 ICU peripheral interrupt slots (IELSR0..IELSR31)
 *
 * Routed by writing R9A02_EVENT_* IDs into R9A02_ICU->IELSR[n].  Override
 * a slot by defining a non-weak function with the matching name.
 *==========================================================================*/

__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR8_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR9_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR11_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR12_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR13_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR14_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR15_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR16_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR17_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR18_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR19_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR20_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR22_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR23_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR24_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR25_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR26_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR27_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR28_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR29_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR30_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IELSR31_IRQHandler(void);

/*===========================================================================
 * NMI -- separate from CLIC table, dispatched via the dedicated NMI vector
 *==========================================================================*/
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);

/*===========================================================================
 * Vector table -- 64 function pointers, 256 bytes, naturally pow-2 aligned.
 *
 * Placed in its own section so the linker script can locate it via
 * .vectors and the application can install its address into mtvt.
 *==========================================================================*/
typedef void (*VectorFn_t)(void);

__attribute__((section(".vectors"), aligned(256), used, retain))
const VectorFn_t g_R9A02_VectorTable[64] =
{
    /* RISC-V exception cause handlers [0..31] */
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

    /* R9A02G021 ICU peripheral slots [32..63] -- mapped to IELSR[0..31] */
    [32] = IELSR0_IRQHandler,
    [33] = IELSR1_IRQHandler,
    [34] = IELSR2_IRQHandler,
    [35] = IELSR3_IRQHandler,
    [36] = IELSR4_IRQHandler,
    [37] = IELSR5_IRQHandler,
    [38] = IELSR6_IRQHandler,
    [39] = IELSR7_IRQHandler,
    [40] = IELSR8_IRQHandler,
    [41] = IELSR9_IRQHandler,
    [42] = IELSR10_IRQHandler,
    [43] = IELSR11_IRQHandler,
    [44] = IELSR12_IRQHandler,
    [45] = IELSR13_IRQHandler,
    [46] = IELSR14_IRQHandler,
    [47] = IELSR15_IRQHandler,
    [48] = IELSR16_IRQHandler,
    [49] = IELSR17_IRQHandler,
    [50] = IELSR18_IRQHandler,
    [51] = IELSR19_IRQHandler,
    [52] = IELSR20_IRQHandler,
    [53] = IELSR21_IRQHandler,
    [54] = IELSR22_IRQHandler,
    [55] = IELSR23_IRQHandler,
    [56] = IELSR24_IRQHandler,
    [57] = IELSR25_IRQHandler,
    [58] = IELSR26_IRQHandler,
    [59] = IELSR27_IRQHandler,
    [60] = IELSR28_IRQHandler,
    [61] = IELSR29_IRQHandler,
    [62] = IELSR30_IRQHandler,
    [63] = IELSR31_IRQHandler,
};

/*===========================================================================
 * VectorInstall_R9A02
 *
 * Switch the Andes N22 CLIC into vectored mode and point it at the table
 * above.  Call AFTER SystemInit() (which installs a direct-mode trap
 * handler) when the application wants hardware vectoring.
 *
 * Andes N22 CLIC convention:
 *   - mtvt  (CSR 0x307) holds the table base
 *   - mtvec[1:0] = 11b enables vectored dispatch for non-CLIC mode
 *     interrupts; CLIC interrupts read mtvt directly
 *
 * Leave in direct mode by not calling this function -- the trap handler
 * installed by SystemInit() then catches every fault.
 *==========================================================================*/
#define R9A02_CSR_MTVT          0x307U  /* Andes CLIC table base CSR */

void VectorInstall_R9A02(void)
{
    uintptr_t base = (uintptr_t)&g_R9A02_VectorTable[0];

    /* CSRW mtvt, base -- write directly (CSR number hard-coded as imm12). */
    __asm volatile ("csrw 0x307, %0" : : "r"(base) : "memory");

    /* mtvec: keep the existing handler base but set vectored-mode bits.
     * Read-modify-write so we don't change the base set by SystemInit(). */
    uintptr_t tvec;
    __asm volatile ("csrr %0, mtvec" : "=r"(tvec));
    tvec = (tvec & ~(uintptr_t)0x3U) | 0x3U;    /* mode = 11b */
    __asm volatile ("csrw mtvec, %0" : : "r"(tvec) : "memory");
}
