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

#define LPC17_GPIO_IRQ_COUNT 46U

typedef struct {
	IOPinEvtHandler_t Handler;
	void *pContext;
	uint32_t Priority;
} Lpc17GpioHook_t;

typedef struct {
	volatile uint32_t *pSelect[2];
	volatile uint32_t *pMode[2];
	volatile uint32_t *pOpenDrain;
} Lpc17PinRegs_t;

static Lpc17GpioHook_t s_GpioHook[LPC17_GPIO_IRQ_COUNT];

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
	*regs->pSelect[half] = (*regs->pSelect[half] & ~(3UL << shift)) | (func << shift);

	// P0.27/28 are always open drain without internal pulls. Their ordinary
	// PINMODE/PINMODE_OD fields are reserved (see UM10360 PINCON chapter).
	if (PortNo == 0 && (PinNo == 27 || PinNo == 28))
	{
		uint32_t padmode = 1UL << (PinNo == 27 ? 1U : 3U);

		// Both I2C0 (FUNC1) and USB OTG I2C (FUNC2) use the I2C pad mode.
		// Leave drive strength and the companion pin's controls unchanged.
		if (func == 1U || func == 2U)
		{
			LPC_PINCON->I2CPADCFG &= ~padmode;
		}
		else
		{
			LPC_PINCON->I2CPADCFG |= padmode;
		}
		EnableInterrupt(state);
		return;
	}
	if (Type == IOPINTYPE_OPENDRAIN)
	{
		*regs->pOpenDrain |= mask;
	}
	else
	{
		*regs->pOpenDrain &= ~mask;
	}

	// USB pads P0.29/30 have valid open-drain controls, but no configurable
	// pulls. Their PINMODE1 fields are reserved (UM10360, table 88).
	if (PortNo == 0 && (PinNo == 29 || PinNo == 30))
	{
		EnableInterrupt(state);
		return;
	}

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

// Leave the pin as a GPIO input without pulls; this is not input-buffer disconnect.
void IOPinDisable(int PortNo, int PinNo)
{
	IOPinConfig(PortNo, PinNo, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	volatile uint32_t *rise;
	volatile uint32_t *fall;
	uint32_t mask;
	uint32_t state;

	if ((unsigned)PinNo >= 32U ||
		(Sense != IOPINSENSE_DISABLE && Sense != IOPINSENSE_HIGH_TRANSITION &&
		Sense != IOPINSENSE_LOW_TRANSITION && Sense != IOPINSENSE_TOGGLE))
	{
		return;
	}
	mask = 1UL << (unsigned)PinNo;
	if (PortNo == 0 && (mask & 0x7FFF8FFFU) != 0)
	{
		rise = &LPC_GPIOINT->IO0IntEnR;
		fall = &LPC_GPIOINT->IO0IntEnF;
	}
	else if (PortNo == 2 && PinNo < 14)
	{
		rise = &LPC_GPIOINT->IO2IntEnR;
		fall = &LPC_GPIOINT->IO2IntEnF;
	}
	else
	{
		return;
	}

	// Preserve other pins and pending events; NVIC ownership belongs to IRQ setup.
	state = DisableInterrupt();
	if (Sense == IOPINSENSE_HIGH_TRANSITION || Sense == IOPINSENSE_TOGGLE)
	{
		*rise |= mask;
	}
	else
	{
		*rise &= ~mask;
	}
	if (Sense == IOPINSENSE_LOW_TRANSITION || Sense == IOPINSENSE_TOGGLE)
	{
		*fall |= mask;
	}
	else
	{
		*fall &= ~mask;
	}
	EnableInterrupt(state);
}

// Fixed IDs follow Port 0 pins, then Port 2 pins starting at 32.
static bool Lpc17GpioIrqValid(unsigned IntNo)
{
	return (IntNo < 32U && ((1UL << IntNo) & 0x7FFF8FFFU) != 0) ||
		(IntNo >= 32U && IntNo < LPC17_GPIO_IRQ_COUNT);
}

// Caller holds the interrupt lock. All GPIO callbacks share EINT3.
static void Lpc17GpioIrqPriority(void)
{
	uint32_t priority = (1U << __NVIC_PRIO_BITS);

	for (unsigned i = 0; i < LPC17_GPIO_IRQ_COUNT; i++)
	{
		if (s_GpioHook[i].Handler != NULL && s_GpioHook[i].Priority < priority)
		{
			priority = s_GpioHook[i].Priority;
		}
	}
	if (priority < (1U << __NVIC_PRIO_BITS))
	{
		NVIC_SetPriority(EINT3_IRQn, priority);
		NVIC_EnableIRQ(EINT3_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(EINT3_IRQn);
	}
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo,
	IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	uint32_t state;
	uint32_t mask;
	int maxprio = (1U << __NVIC_PRIO_BITS) - 1U;

	if ((PortNo != 0U && PortNo != 2U) || PinNo >= 32U ||
		!Lpc17GpioIrqValid((unsigned)IntNo) ||
		(unsigned)IntNo != (PortNo == 0U ? PinNo : 32U + PinNo) ||
		pEvtCB == NULL || (Sense != IOPINSENSE_HIGH_TRANSITION &&
		Sense != IOPINSENSE_LOW_TRANSITION && Sense != IOPINSENSE_TOGGLE))
	{
		return false;
	}
	if (IntPrio < 0)
	{
		IntPrio = 0;
	}
	else if (IntPrio > maxprio)
	{
		IntPrio = maxprio;
	}
	state = DisableInterrupt();
	if (s_GpioHook[IntNo].Handler != NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	IOPinSetSense((int)PortNo, (int)PinNo, IOPINSENSE_DISABLE);
	mask = 1UL << PinNo;
	if (PortNo == 0U)
	{
		LPC_GPIOINT->IO0IntClr = mask;
	}
	else
	{
		LPC_GPIOINT->IO2IntClr = mask;
	}
	s_GpioHook[IntNo].pContext = pCtx;
	s_GpioHook[IntNo].Priority = (uint32_t)IntPrio;
	s_GpioHook[IntNo].Handler = pEvtCB;
	IOPinSetSense((int)PortNo, (int)PinNo, Sense);
	Lpc17GpioIrqPriority();
	EnableInterrupt(state);

	return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense,
	IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	int intno;

	if ((PortNo != 0 && PortNo != 2) || (unsigned)PinNo >= 32U)
	{
		return -1;
	}
	intno = PortNo == 0 ? PinNo : 32 + PinNo;
	if (!IOPinEnableInterrupt(intno, IntPrio, (uint32_t)PortNo, (uint32_t)PinNo,
		Sense, pEvtCB, pCtx))
	{
		return -1;
	}

	return intno;
}

void IOPinDisableInterrupt(int IntNo)
{
	uint32_t state;
	uint32_t mask;

	if (!Lpc17GpioIrqValid((unsigned)IntNo))
	{
		return;
	}
	state = DisableInterrupt();
	if (s_GpioHook[IntNo].Handler != NULL)
	{
		mask = 1UL << ((unsigned)IntNo % 32U);
		IOPinSetSense(IntNo < 32 ? 0 : 2, IntNo % 32, IOPINSENSE_DISABLE);
		if (IntNo < 32)
		{
			LPC_GPIOINT->IO0IntClr = mask;
		}
		else
		{
			LPC_GPIOINT->IO2IntClr = mask;
		}
		s_GpioHook[IntNo].Handler = NULL;
		s_GpioHook[IntNo].pContext = NULL;
		Lpc17GpioIrqPriority();
	}
	EnableInterrupt(state);
}

#ifdef __cplusplus
extern "C" {
#endif

// This driver owns EINT3; the separate EINT3 pin function must remain disabled.
void EINT3_IRQHandler(void)
{
	for (unsigned i = 0; i < LPC17_GPIO_IRQ_COUNT; i++)
	{
		uint32_t pending;
		uint32_t rising;
		uint32_t falling;
		uint32_t mask;
		uint32_t state;
		IOPinEvtHandler_t handler = NULL;
		void *context = NULL;

		if (!Lpc17GpioIrqValid(i))
		{
			continue;
		}
		mask = 1UL << (i % 32U);
		state = DisableInterrupt();
		if (i < 32U)
		{
			rising = LPC_GPIOINT->IO0IntStatR;
			falling = LPC_GPIOINT->IO0IntStatF;
			pending = ((rising & LPC_GPIOINT->IO0IntEnR) |
				(falling & LPC_GPIOINT->IO0IntEnF)) & mask;
			if (((rising | falling) & mask) != 0)
			{
				LPC_GPIOINT->IO0IntClr = mask;
			}
		}
		else
		{
			rising = LPC_GPIOINT->IO2IntStatR;
			falling = LPC_GPIOINT->IO2IntStatF;
			pending = ((rising & LPC_GPIOINT->IO2IntEnR) |
				(falling & LPC_GPIOINT->IO2IntEnF)) & mask;
			if (((rising | falling) & mask) != 0)
			{
				LPC_GPIOINT->IO2IntClr = mask;
			}
		}
		if (pending != 0)
		{
			handler = s_GpioHook[i].Handler;
			context = s_GpioHook[i].pContext;
		}
		EnableInterrupt(state);
		// Clear before callback. Read each next pin live so a callback can
		// disable or reallocate it without receiving an old snapshotted edge.
		if (handler != NULL)
		{
			handler((int)i, context);
		}
	}
}

#ifdef __cplusplus
}
#endif

// No general-purpose programmable drive strength or slew rate on LPC17xx.
// The special I2C0 pad controls are configured with the I2C operating mode.
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
