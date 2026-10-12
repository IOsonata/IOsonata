/*--------------------------------------------------------------------------
File   : iopincfg.c

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : Generic I/O pin config

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

#include "LPC17xx.h"
#include "coredev/iopincfg.h"

typedef struct {
	volatile uint32_t *pSelect[2];
	volatile uint32_t *pMode[2];
	volatile uint32_t *pOpenDrain;
} Lpc17PinRegs_t;

static const Lpc17PinRegs_t s_PinRegs[] = {
	{{&LPC_PINCON->PINSEL0, &LPC_PINCON->PINSEL1},
	 {&LPC_PINCON->PINMODE0, &LPC_PINCON->PINMODE1}, &LPC_PINCON->PINMODE_OD0},
	{{&LPC_PINCON->PINSEL2, &LPC_PINCON->PINSEL3},
	 {&LPC_PINCON->PINMODE2, &LPC_PINCON->PINMODE3}, &LPC_PINCON->PINMODE_OD1},
	{{&LPC_PINCON->PINSEL4, &LPC_PINCON->PINSEL5},
	 {&LPC_PINCON->PINMODE4, &LPC_PINCON->PINMODE5}, &LPC_PINCON->PINMODE_OD2},
	{{&LPC_PINCON->PINSEL6, &LPC_PINCON->PINSEL7},
	 {&LPC_PINCON->PINMODE6, &LPC_PINCON->PINMODE7}, &LPC_PINCON->PINMODE_OD3},
	{{&LPC_PINCON->PINSEL8, &LPC_PINCON->PINSEL9},
	 {&LPC_PINCON->PINMODE8, &LPC_PINCON->PINMODE9}, &LPC_PINCON->PINMODE_OD4},
};

// IOPINOP_GPIO and FUNC0 select hardware function 0; FUNCn selects n.
// The caller must choose a function and pin available on the actual package.
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if ((unsigned)PortNo >= 5U || (unsigned)PinNo >= 32U ||
		PinOp < IOPINOP_GPIO || PinOp > IOPINOP_FUNC3)
	{
		return;
	}
	const Lpc17PinRegs_t *regs = &s_PinRegs[PortNo];
	unsigned half = (unsigned)PinNo / 16U;
	unsigned shift = ((unsigned)PinNo % 16U) * 2U;
	uint32_t mask = 1UL << (unsigned)PinNo;
	uint32_t func = PinOp == IOPINOP_GPIO ? 0U : (uint32_t)(PinOp - IOPINOP_FUNC0);
	uint32_t rmode = 0;
	uint32_t state = DisableInterrupt();

	if (Dir == IOPINDIR_OUTPUT)
	{
		LPC_GPIO0[PortNo].FIODIR |= mask;
	}
	else
	{
		LPC_GPIO0[PortNo].FIODIR &= ~mask;
	}
	if (Type == IOPINTYPE_OPENDRAIN)
	{
		*regs->pOpenDrain |= mask;
	}
	else
	{
		*regs->pOpenDrain &= ~mask;
	}
	*regs->pSelect[half] = (*regs->pSelect[half] & ~(3UL << shift)) | (func << shift);

	switch (Resistor)
	{
		case IOPINRES_NONE:
			rmode = 2;
			break;
		case IOPINRES_PULLUP:
			rmode = 0;
			break;
		case IOPINRES_PULLDOWN:
			rmode = 3;
			break;
		case IOPINRES_FOLLOW:
			rmode = 1;
			break;
	}
	*regs->pMode[half] = (*regs->pMode[half] & ~(3UL << shift)) | (rmode << shift);
	EnableInterrupt(state);
}
