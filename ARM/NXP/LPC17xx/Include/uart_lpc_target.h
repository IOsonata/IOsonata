/*--------------------------------------------------------------------------
File   : uart_lpc_target.h

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : Vendor CMSIS access for the legacy LPC UART driver

Copyright (c) 2011, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#ifndef __UART_LPC_TARGET_H__
#define __UART_LPC_TARGET_H__

#include "LPC17xx.h"

#define LPCUART_DEV_COUNT		4

// UART0, UART1 and UART2/3 have separate vendor types and access widths.
// Select the declared member without casting one peripheral type to another.
#define LPCUART_READ(DevNo, Reg) \
	((DevNo) == 0 ? LPC_UART0->Reg : \
	 (DevNo) == 1 ? LPC_UART1->Reg : \
	 (DevNo) == 2 ? LPC_UART2->Reg : LPC_UART3->Reg)

#define LPCUART_WRITE(DevNo, Reg, Value) \
	do { \
		switch (DevNo) \
		{ \
			case 0: \
				LPC_UART0->Reg = (Value); \
				break; \
			case 1: \
				LPC_UART1->Reg = (Value); \
				break; \
			case 2: \
				LPC_UART2->Reg = (Value); \
				break; \
			case 3: \
				LPC_UART3->Reg = (Value); \
				break; \
		} \
	} while (0)

#endif
