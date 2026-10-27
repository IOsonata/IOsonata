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

#define LPC11_PINT_COUNT		8U
#define LPC11_PINT_CLOCK		(1UL << 19)

typedef struct {
	IOPinEvtHandler_t Handler;
	void *pContext;
	uint8_t PinSel;
	IOPINSENSE Sense;
} Lpc11PintHook_t;

static Lpc11PintHook_t s_PintHook[LPC11_PINT_COUNT];
static const IRQn_Type s_PintIrq[LPC11_PINT_COUNT] = {
	FLEX_INT0_IRQn,
	FLEX_INT1_IRQn,
	FLEX_INT2_IRQn,
	FLEX_INT3_IRQn,
	FLEX_INT4_IRQn,
	FLEX_INT5_IRQn,
	FLEX_INT6_IRQn,
	FLEX_INT7_IRQn
};

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
	NULL, // Reserved IOCON offset 0xD8; rejected by Lpc11GpioValidPin.
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

// Caller holds the short interrupt lock. Discard old edges before rearming.
static void Lpc11PintSetSense(unsigned IntNo, IOPINSENSE Sense)
{
	uint32_t mask = 1UL << IntNo;

	NVIC_DisableIRQ(s_PintIrq[IntNo]);
	LPC_GPIO_PIN_INT->CIENR = mask;
	LPC_GPIO_PIN_INT->CIENF = mask;
	LPC_GPIO_PIN_INT->ISEL &= ~mask;
	LPC_GPIO_PIN_INT->IST = mask;
	NVIC_ClearPendingIRQ(s_PintIrq[IntNo]);
	s_PintHook[IntNo].Sense = Sense;

	if (Sense == IOPINSENSE_HIGH_TRANSITION || Sense == IOPINSENSE_TOGGLE)
	{
		LPC_GPIO_PIN_INT->SIENR = mask;
	}
	if (Sense == IOPINSENSE_LOW_TRANSITION || Sense == IOPINSENSE_TOGGLE)
	{
		LPC_GPIO_PIN_INT->SIENF = mask;
	}
	if (Sense != IOPINSENSE_DISABLE)
	{
		NVIC_EnableIRQ(s_PintIrq[IntNo]);
	}
}

void IOPinDisableInterrupt(int IntNo)
{
	if ((unsigned)IntNo >= LPC11_PINT_COUNT)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	if (s_PintHook[IntNo].Handler != NULL)
	{
		Lpc11PintSetSense((unsigned)IntNo, IOPINSENSE_DISABLE);
		s_PintHook[IntNo].Handler = NULL;
		s_PintHook[IntNo].pContext = NULL;
	}
	EnableInterrupt(state);
}

// IntNo selects channel 0..7. Configure the pin's digital input before calling.
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo,
						 IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if ((unsigned)IntNo >= LPC11_PINT_COUNT || PortNo > 1U ||
		PinNo > 31U || !Lpc11GpioValidPin((int)PortNo, (int)PinNo) || pEvtCB == NULL ||
		(Sense != IOPINSENSE_HIGH_TRANSITION && Sense != IOPINSENSE_LOW_TRANSITION &&
		 Sense != IOPINSENSE_TOGGLE))
	{
		return false;
	}

	int maxprio = (1U << __NVIC_PRIO_BITS) - 1U;

	if (IntPrio < 0)
	{
		IntPrio = 0;
	}
	else if (IntPrio > maxprio)
	{
		IntPrio = maxprio;
	}
	uint32_t state = DisableInterrupt();

	if (s_PintHook[IntNo].Handler != NULL)
	{
		EnableInterrupt(state);
		return false;
	}

	LPC_SYSCON->SYSAHBCLKCTRL |= LPC11_PINT_CLOCK;
	(void)LPC_SYSCON->SYSAHBCLKCTRL;
	Lpc11PintSetSense((unsigned)IntNo, IOPINSENSE_DISABLE);

	// LPC11Uxx numbers port 1 pins from 24, unlike LPC546xx's stride of 32.
	uint32_t sel = PortNo == 0U ? PinNo : 24U + PinNo;

	LPC_SYSCON->PINTSEL[IntNo] = sel;
	s_PintHook[IntNo].PinSel = (uint8_t)sel;
	s_PintHook[IntNo].pContext = pCtx;
	s_PintHook[IntNo].Handler = pEvtCB;
	NVIC_SetPriority(s_PintIrq[IntNo], (uint32_t)IntPrio);
	Lpc11PintSetSense((unsigned)IntNo, Sense);
	EnableInterrupt(state);
	return true;
}

// A query only; allocation keeps discovery and registration under one lock.
int IOPinFindAvailInterrupt(void)
{
	uint32_t state = DisableInterrupt();
	int intno = -1;

	for (unsigned i = 0; i < LPC11_PINT_COUNT; i++)
	{
		if (s_PintHook[i].Handler == NULL)
		{
			intno = (int)i;
			break;
		}
	}
	EnableInterrupt(state);
	return intno;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense,
						 IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	uint32_t state = DisableInterrupt();
	int intno = IOPinFindAvailInterrupt();

	if (intno >= 0 && !IOPinEnableInterrupt(intno, IntPrio, (uint32_t)PortNo,
										 (uint32_t)PinNo, Sense, pEvtCB, pCtx))
	{
		intno = -1;
	}
	EnableInterrupt(state);
	return intno;
}

// DISABLE pauses detection without releasing the allocated channel.
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	if (!Lpc11GpioValidPin(PortNo, PinNo) ||
		(Sense != IOPINSENSE_DISABLE && Sense != IOPINSENSE_HIGH_TRANSITION &&
		 Sense != IOPINSENSE_LOW_TRANSITION && Sense != IOPINSENSE_TOGGLE))
	{
		return;
	}

	uint32_t sel = PortNo == 0 ? (uint32_t)PinNo : 24U + (uint32_t)PinNo;
	uint32_t state = DisableInterrupt();

	for (unsigned i = 0; i < LPC11_PINT_COUNT; i++)
	{
		if (s_PintHook[i].Handler != NULL && s_PintHook[i].PinSel == sel)
		{
			Lpc11PintSetSense(i, Sense);
		}
	}
	EnableInterrupt(state);
}

static void Lpc11PintDispatch(unsigned IntNo)
{
	uint32_t mask = 1UL << IntNo;
	uint32_t state = DisableInterrupt();
	IOPinEvtHandler_t handler = NULL;
	void *context = NULL;

	if ((LPC_GPIO_PIN_INT->IST & mask) != 0U)
	{
		// Edge mode W1C: acknowledge before callback so a new edge survives.
		LPC_GPIO_PIN_INT->IST = mask;
		if (s_PintHook[IntNo].Sense != IOPINSENSE_DISABLE)
		{
			handler = s_PintHook[IntNo].Handler;
			context = s_PintHook[IntNo].pContext;
		}
	}
	EnableInterrupt(state);

	if (handler != NULL)
	{
		handler((int)IntNo, context);
	}
}

#ifdef __cplusplus
extern "C" {
#endif

void FLEX_INT0_IRQHandler(void)
{
	Lpc11PintDispatch(0);
}

void FLEX_INT1_IRQHandler(void)
{
	Lpc11PintDispatch(1);
}

void FLEX_INT2_IRQHandler(void)
{
	Lpc11PintDispatch(2);
}

void FLEX_INT3_IRQHandler(void)
{
	Lpc11PintDispatch(3);
}

void FLEX_INT4_IRQHandler(void)
{
	Lpc11PintDispatch(4);
}

void FLEX_INT5_IRQHandler(void)
{
	Lpc11PintDispatch(5);
}

void FLEX_INT6_IRQHandler(void)
{
	Lpc11PintDispatch(6);
}

void FLEX_INT7_IRQHandler(void)
{
	Lpc11PintDispatch(7);
}

#ifdef __cplusplus
}
#endif

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

// This port has one application core and no pin ownership selector.
bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	return CoreNo == 0 && Lpc11GpioValidPin(PortNo, PinNo);
}
