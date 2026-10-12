/*--------------------------------------------------------------------------
File   : iopincfg_lpc11uxx.c

Author : Hoang Nguyen Hoan          Oct. 25, 2014

Desc   : I/O pin config implementation on LPC11Uxx

Copyright (c) 2014, I-SYST inc., all rights reserved

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

#include "iopinctrl.h"

#define IOCON_FUNC_MASK		7UL
#define IOCON_MODE_MASK		(3UL << 3)
#define IOCON_HYS			(1UL << 5)
#define IOCON_INV			(1UL << 6)
#define IOCON_ADMODE		(1UL << 7)
#define IOCON_I2CMODE_MASK	(3UL << 8)
#define IOCON_OD			(1UL << 10)

// Named vendor registers: no inferred register layout or nonvolatile access.
static volatile uint32_t * const s_PinReg[] = {
	&LPC_IOCON->RESET_PIO0_0,
	&LPC_IOCON->PIO0_1,
	&LPC_IOCON->PIO0_2,
	&LPC_IOCON->PIO0_3,
	&LPC_IOCON->PIO0_4,
	&LPC_IOCON->PIO0_5,
	&LPC_IOCON->PIO0_6,
	&LPC_IOCON->PIO0_7,
	&LPC_IOCON->PIO0_8,
	&LPC_IOCON->PIO0_9,
	&LPC_IOCON->SWCLK_PIO0_10,
	&LPC_IOCON->TDI_PIO0_11,
	&LPC_IOCON->TMS_PIO0_12,
	&LPC_IOCON->TDO_PIO0_13,
	&LPC_IOCON->TRST_PIO0_14,
	&LPC_IOCON->SWDIO_PIO0_15,
	&LPC_IOCON->PIO0_16,
	&LPC_IOCON->PIO0_17,
	&LPC_IOCON->PIO0_18,
	&LPC_IOCON->PIO0_19,
	&LPC_IOCON->PIO0_20,
	&LPC_IOCON->PIO0_21,
	&LPC_IOCON->PIO0_22,
	&LPC_IOCON->PIO0_23,
	&LPC_IOCON->PIO1_0,
	&LPC_IOCON->PIO1_1,
	&LPC_IOCON->PIO1_2,
	&LPC_IOCON->PIO1_3,
	&LPC_IOCON->PIO1_4,
	&LPC_IOCON->PIO1_5,
	&LPC_IOCON->PIO1_6,
	&LPC_IOCON->PIO1_7,
	&LPC_IOCON->PIO1_8,
	&LPC_IOCON->PIO1_9,
	&LPC_IOCON->PIO1_10,
	&LPC_IOCON->PIO1_11,
	&LPC_IOCON->PIO1_12,
	&LPC_IOCON->PIO1_13,
	&LPC_IOCON->PIO1_14,
	&LPC_IOCON->PIO1_15,
	&LPC_IOCON->PIO1_16,
	&LPC_IOCON->PIO1_17,
	&LPC_IOCON->PIO1_18,
	&LPC_IOCON->PIO1_19,
	&LPC_IOCON->PIO1_20,
	&LPC_IOCON->PIO1_21,
	&LPC_IOCON->PIO1_22,
	&LPC_IOCON->PIO1_23,
	&LPC_IOCON->PIO1_24,
	&LPC_IOCON->PIO1_25,
	&LPC_IOCON->PIO1_26,
	&LPC_IOCON->PIO1_27,
	&LPC_IOCON->PIO1_28,
	&LPC_IOCON->PIO1_29,
	&LPC_IOCON->PIO1_30,
	&LPC_IOCON->PIO1_31,
};

// IOPINOP_GPIO selects GPIO even on reset/debug pins. FUNCn selects FUNC n.
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if (!Lpc11GpioValidPin(PortNo, PinNo) || PinOp < IOPINOP_GPIO || PinOp > IOPINOP_FUNC7)
	{
		return;
	}

	uint32_t func = PinOp == IOPINOP_GPIO ? 0U : (uint32_t)(PinOp - IOPINOP_FUNC0);
	int analogfunc = -1;
	bool i2c = PortNo == 0 && (PinNo == 4 || PinNo == 5);
	volatile uint32_t *reg = s_PinReg[PortNo == 0 ? PinNo : 24 + PinNo];

	if (PortNo == 0)
	{
		if (PinOp == IOPINOP_GPIO && (PinNo == 0 || (PinNo >= 10 && PinNo <= 15)))
		{
			func = 1U;
		}
		if (PinNo >= 11 && PinNo <= 15)
		{
			analogfunc = 2;
		}
		else if (PinNo == 16 || PinNo == 22 || PinNo == 23)
		{
			analogfunc = 1;
		}
	}

	uint32_t state = DisableInterrupt();

	LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6) | (1UL << 16);
	(void)LPC_SYSCON->SYSAHBCLKCTRL;

	uint32_t value = (*reg & ~IOCON_FUNC_MASK) | func;

	if (i2c)
	{
		// P0_4/5 are true open drain, with no programmable pulls/HYS/OD.
		// GPIO uses standard I/O mode; peripheral mode uses standard/fast I2C.
		value &= ~IOCON_I2CMODE_MASK;
		if (func == 0U)
		{
			value |= 1UL << 8;
		}
	}
	else
	{
		value &= ~(IOCON_MODE_MASK | IOCON_HYS | IOCON_INV | IOCON_OD);
		if ((int)func != analogfunc)
		{
			value |= IOCON_HYS;
			if (Resistor == IOPINRES_PULLUP)
			{
				value |= 2UL << 3;
			}
			else if (Resistor == IOPINRES_PULLDOWN)
			{
				value |= 1UL << 3;
			}
			else if (Resistor == IOPINRES_FOLLOW)
			{
				value |= 3UL << 3;
			}
			if (Type == IOPINTYPE_OPENDRAIN)
			{
				value |= IOCON_OD;
			}
		}
		if (analogfunc >= 0)
		{
			value &= ~IOCON_ADMODE;
			if ((int)func != analogfunc)
			{
				value |= IOCON_ADMODE;
			}
		}
	}

	// Set direction before connecting the selected function to the pad.
	IOPinSetDir(PortNo, PinNo, (int)func == analogfunc ? IOPINDIR_INPUT : Dir);
	*reg = value;
	EnableInterrupt(state);
}

// These pads cannot all disconnect their input buffer. Leave GPIO input/no pull.
void IOPinDisable(int PortNo, int PinNo)
{
	IOPinConfig(PortNo, PinNo, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

// Pin interrupts are not implemented yet. Never report successful registration.
void IOPinDisableInterrupt(int IntNo)
{
	(void)IntNo;
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo,
						 IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	(void)IntNo;
	(void)IntPrio;
	(void)PortNo;
	(void)PinNo;
	(void)Sense;
	(void)pEvtCB;
	(void)pCtx;
	return false;
}

int IOPinFindAvailInterrupt(void)
{
	return -1;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense,
						 IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	(void)IntPrio;
	(void)PortNo;
	(void)PinNo;
	(void)Sense;
	(void)pEvtCB;
	(void)pCtx;
	return -1;
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	(void)PortNo;
	(void)PinNo;
	(void)Sense;
}

// LPC11Uxx has no general-purpose programmable drive strength or slew rate.
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	(void)PortNo;
	(void)PinNo;
	(void)Strength;
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	(void)PortNo;
	(void)PinNo;
	(void)Speed;
}
