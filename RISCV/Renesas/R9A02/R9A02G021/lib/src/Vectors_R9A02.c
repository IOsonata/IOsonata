/**-------------------------------------------------------------------------
@file	Vectors_R9A02.c

@brief	R9A02G021 trap entry, CLIC vector table and NMI entry

R9A02G021 User's Manual R01UH1036EJ0110, 2.5.3, 2.7.2.3 and 12.3.1.

mtvec (CLIC mode) points to R9a02TrapEntry: exceptions, and any interrupt
taken without hardware vectoring. mtvt points to g_R9a02VectorTable, 51
entries indexed by CLIC ID. Every CLIC input is hardware vectored
(clicintattr.shv = 1), so the CLIC clears its pending bit on entry.

The table entries are weak aliases of R9a02IrqEntry, which saves the caller
saved registers, mepc and mcause, re-enables mstatus.MIE so a higher level
can preempt, and calls R9a02IrqDispatch with the CLIC ID. An application
can replace an entry with its own function of the same name, built with
__attribute__((interrupt)). interrupt_r9a02.cpp then leaves that input
alone.

The table and the entries go in .vectors sections: the normal boot linker
script places them in the first 2 KB of the code flash around the option
setting memory, the DFU application script after its reset entry.

@author	Hoang Nguyen Hoan
@date	May 12, 2026

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
#include <stdint.h>

#include "r9a02g021.h"
#include "clic.h"
#include "interrupt_r9a02.h"

// Last exception seen by the default handler, for the debugger
static volatile uint32_t s_ExcCause;
static volatile uintptr_t s_ExcEpc;
static volatile uintptr_t s_ExcTval;

void R9a02TrapEntry(void);
void R9a02NmiEntry(void);

// Exceptions, and interrupts without hardware vectoring. The CLIC does not
// clear the pending bit of a non vectored interrupt, the entry clears it.
__attribute__((naked, aligned(64), section(".vectors.text.trap")))
void R9a02TrapEntry(void)
{
	__asm volatile (
		"addi	sp, sp, -80\n"
		"sw		ra, 0(sp)\n"
		"sw		t0, 4(sp)\n"
		"sw		t1, 8(sp)\n"
		"sw		t2, 12(sp)\n"
		"sw		a0, 16(sp)\n"
		"sw		a1, 20(sp)\n"
		"sw		a2, 24(sp)\n"
		"sw		a3, 28(sp)\n"
		"sw		a4, 32(sp)\n"
		"sw		a5, 36(sp)\n"
		"sw		a6, 40(sp)\n"
		"sw		a7, 44(sp)\n"
		"sw		t3, 48(sp)\n"
		"sw		t4, 52(sp)\n"
		"sw		t5, 56(sp)\n"
		"sw		t6, 60(sp)\n"
		"csrr	a0, mcause\n"
		"bltz	a0, 1f\n"
		"csrr	a1, mepc\n"
		"csrr	a2, mtval\n"
		"call	R9a02ExceptionHandler\n"
		"csrw	mepc, a0\n"
		"j		2f\n"
		"1:\n"
		"andi	a0, a0, 0x3FF\n"
		"slli	t0, a0, 2\n"
		"li		t1, %0\n"
		"add	t0, t0, t1\n"
		"sb		zero, 0(t0)\n"
		"call	R9a02IrqDispatch\n"
		"2:\n"
		"lw		ra, 0(sp)\n"
		"lw		t0, 4(sp)\n"
		"lw		t1, 8(sp)\n"
		"lw		t2, 12(sp)\n"
		"lw		a0, 16(sp)\n"
		"lw		a1, 20(sp)\n"
		"lw		a2, 24(sp)\n"
		"lw		a3, 28(sp)\n"
		"lw		a4, 32(sp)\n"
		"lw		a5, 36(sp)\n"
		"lw		a6, 40(sp)\n"
		"lw		a7, 44(sp)\n"
		"lw		t3, 48(sp)\n"
		"lw		t4, 52(sp)\n"
		"lw		t5, 56(sp)\n"
		"lw		t6, 60(sp)\n"
		"addi	sp, sp, 80\n"
		"mret\n"
		:: "i"(R9A02_CLIC_CTRL_BASE)
	);
}

// Hardware vectored interrupts. mepc and mcause are saved before MIE is set
// again: mret restores the previous level from mcause.mpil.
__attribute__((naked, aligned(4), section(".vectors.text.irq")))
void R9a02IrqEntry(void)
{
	__asm volatile (
		"addi	sp, sp, -80\n"
		"sw		ra, 0(sp)\n"
		"sw		t0, 4(sp)\n"
		"sw		t1, 8(sp)\n"
		"sw		t2, 12(sp)\n"
		"sw		a0, 16(sp)\n"
		"sw		a1, 20(sp)\n"
		"sw		a2, 24(sp)\n"
		"sw		a3, 28(sp)\n"
		"sw		a4, 32(sp)\n"
		"sw		a5, 36(sp)\n"
		"sw		a6, 40(sp)\n"
		"sw		a7, 44(sp)\n"
		"sw		t3, 48(sp)\n"
		"sw		t4, 52(sp)\n"
		"sw		t5, 56(sp)\n"
		"sw		t6, 60(sp)\n"
		"csrr	t0, mepc\n"
		"csrr	t1, mcause\n"
		"sw		t0, 64(sp)\n"
		"sw		t1, 68(sp)\n"
		"andi	a0, t1, 0x3FF\n"
		"csrsi	mstatus, 8\n"
		"call	R9a02IrqDispatch\n"
		"csrci	mstatus, 8\n"
		"lw		t0, 64(sp)\n"
		"lw		t1, 68(sp)\n"
		"csrw	mepc, t0\n"
		"csrw	mcause, t1\n"
		"lw		ra, 0(sp)\n"
		"lw		t0, 4(sp)\n"
		"lw		t1, 8(sp)\n"
		"lw		t2, 12(sp)\n"
		"lw		a0, 16(sp)\n"
		"lw		a1, 20(sp)\n"
		"lw		a2, 24(sp)\n"
		"lw		a3, 28(sp)\n"
		"lw		a4, 32(sp)\n"
		"lw		a5, 36(sp)\n"
		"lw		a6, 40(sp)\n"
		"lw		a7, 44(sp)\n"
		"lw		t3, 48(sp)\n"
		"lw		t4, 52(sp)\n"
		"lw		t5, 56(sp)\n"
		"lw		t6, 60(sp)\n"
		"addi	sp, sp, 80\n"
		"mret\n"
	);
}

// NMIADDR needs 64-byte alignment (2.5.8.3). The NMI does not return.
__attribute__((naked, aligned(64), section(".vectors.text.nmi")))
void R9a02NmiEntry(void)
{
	__asm volatile (
		"call	R9a02NmiHandler\n"
		"1:\n"
		"j		1b\n"
	);
}

void MSIP_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void MTIP_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR0_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR1_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR2_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR3_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR4_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR5_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR6_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR7_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR8_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR9_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR10_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR11_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR12_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR13_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR14_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR15_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR16_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR17_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR18_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR19_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR20_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR21_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR22_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR23_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR24_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR25_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR26_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR27_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR28_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR29_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR30_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));
void IELSR31_IRQHandler(void) __attribute__((weak, alias("R9a02IrqEntry")));

// Manual table 12.3. Reserved IDs point to the common entry, the dispatcher
// disables an input it does not know.
__attribute__((section(".vectors.table"), aligned(64), used))
const R9a02Vector_t g_R9a02VectorTable[R9A02_CLIC_INT_COUNT] = {
	R9a02IrqEntry,			// 0 reserved
	R9a02IrqEntry,			// 1 reserved
	R9a02IrqEntry,			// 2 reserved
	MSIP_IRQHandler,		// 3 machine software interrupt
	R9a02IrqEntry,			// 4 reserved
	R9a02IrqEntry,			// 5 reserved
	R9a02IrqEntry,			// 6 reserved
	MTIP_IRQHandler,		// 7 machine timer interrupt
	R9a02IrqEntry,			// 8 reserved
	R9a02IrqEntry,			// 9 reserved
	R9a02IrqEntry,			// 10 reserved
	R9a02IrqEntry,			// 11 reserved
	R9a02IrqEntry,			// 12 reserved
	R9a02IrqEntry,			// 13 reserved
	R9a02IrqEntry,			// 14 reserved
	R9a02IrqEntry,			// 15 reserved
	R9a02IrqEntry,			// 16 reserved
	R9a02IrqEntry,			// 17 reserved
	R9a02IrqEntry,			// 18 reserved
	IELSR0_IRQHandler,		// 19
	IELSR1_IRQHandler,		// 20
	IELSR2_IRQHandler,		// 21
	IELSR3_IRQHandler,		// 22
	IELSR4_IRQHandler,		// 23
	IELSR5_IRQHandler,		// 24
	IELSR6_IRQHandler,		// 25
	IELSR7_IRQHandler,		// 26
	IELSR8_IRQHandler,		// 27
	IELSR9_IRQHandler,		// 28
	IELSR10_IRQHandler,		// 29
	IELSR11_IRQHandler,		// 30
	IELSR12_IRQHandler,		// 31
	IELSR13_IRQHandler,		// 32
	IELSR14_IRQHandler,		// 33
	IELSR15_IRQHandler,		// 34
	IELSR16_IRQHandler,		// 35
	IELSR17_IRQHandler,		// 36
	IELSR18_IRQHandler,		// 37
	IELSR19_IRQHandler,		// 38
	IELSR20_IRQHandler,		// 39
	IELSR21_IRQHandler,		// 40
	IELSR22_IRQHandler,		// 41
	IELSR23_IRQHandler,		// 42
	IELSR24_IRQHandler,		// 43
	IELSR25_IRQHandler,		// 44
	IELSR26_IRQHandler,		// 45
	IELSR27_IRQHandler,		// 46
	IELSR28_IRQHandler,		// 47
	IELSR29_IRQHandler,		// 48
	IELSR30_IRQHandler,		// 49
	IELSR31_IRQHandler,		// 50
};

__attribute__((weak))
uintptr_t R9a02ExceptionHandler(uint32_t Cause, uintptr_t Epc, uintptr_t Tval)
{
	s_ExcCause = Cause;
	s_ExcEpc = Epc;
	s_ExcTval = Tval;

	for (;;)
	{
		__asm volatile ("nop");
	}

	return Epc;
}

__attribute__((weak))
void R9a02NmiHandler(void)
{
	for (;;)
	{
		__asm volatile ("nop");
	}
}

void R9a02TrapInit(void)
{
	uintptr_t base = R9A02_CLIC_CTRL_BASE;
	uint8_t ctl = ClicCtlFromLevel(0U, R9A02_CLIC_NLBITS);

	for (uint32_t i = 0; i < R9A02_CLIC_INT_COUNT; i++)
	{
		CLIC_INT_IE_REG(base, i) = 0U;
		CLIC_INT_ATTR_REG(base, i) = CLIC_INT_ATTR_SHV_Msk;
		CLIC_INT_CTL_REG(base, i) = ctl;
		CLIC_INT_IP_REG(base, i) = 0U;
	}

	for (uint32_t i = 0; i < R9A02_ICU_IELSR_CNT; i++)
	{
		// Writing 0 also clears IR
		R9A02_ICU->IELSR[i] = 0U;
	}

	R9A02_CLIC_CLICCFG = (uint8_t)(R9A02_CLIC_NLBITS << R9A02_CLICCFG_NLBITS_Pos);

	// mintthresh 0x347, mtvt 0x307. mtvec mode bits 11b select CLIC mode.
	__asm volatile (
		"csrw	0x347, zero\n"
		"csrw	0x307, %0\n"
		"csrw	mtvec, %1\n"
		:: "r"((uintptr_t)g_R9a02VectorTable), "r"((uintptr_t)R9a02TrapEntry | 3U)
		: "memory"
	);

	R9A02_CPU_AUX_NMIADDR = (uint32_t)(uintptr_t)R9a02NmiEntry;
}
