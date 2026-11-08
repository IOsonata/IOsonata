/**-------------------------------------------------------------------------
@file	semihost_riscv.c

@brief	Semihosting call for RISC-V

The RISC-V semihosting call is the three instruction sequence

	slli x0, x0, 0x1f
	ebreak
	srai x0, x0, 7

with the operation number in a0 and the parameter in a1. The debugger
recognizes the ebreak by the two instructions around it, performs the
operation, writes the result in a0 and resumes after the srai. The three
instructions must be 32 bit (not compressed) and must not cross a page, so
the sequence is placed at the start of a 16 byte aligned function.

Used by the semihosting console interface, see semihost_intrf.h.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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
#include <stdbool.h>

#include "semihost_intrf.h"

// A RISC-V core cannot tell from software whether a debugger is connected,
// the call is always made.
bool SemihostDebuggerAttached(void)
{
	return true;
}

// int SemihostCall(int Op, void *pArg)
// Op is in a0 and pArg in a1 by the calling convention, which is where the
// debugger expects them. The result comes back in a0.
// The function has its own section so it starts on the 16 byte boundary
// with no padding in front of the sequence. norelax makes the assembler align
// the section itself instead of emitting padding for the linker to remove.
__asm__ (
	"	.section .text.SemihostCall,\"ax\",@progbits\n"
	"	.globl SemihostCall\n"
	"	.type SemihostCall, @function\n"
	"	.option push\n"
	"	.option norelax\n"
	"	.option norvc\n"
	"	.balign 16\n"
	"SemihostCall:\n"
	"	slli x0, x0, 0x1f\n"
	"	ebreak\n"
	"	srai x0, x0, 7\n"
	"	.option pop\n"
	"	ret\n"
	"	.size SemihostCall, . - SemihostCall\n"
	"	.previous\n"
);
