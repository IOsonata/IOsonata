/**-------------------------------------------------------------------------
@file	iopincfg_r9a02.c

@brief	I/O pin configuration for the Renesas R9A02G021

Implements coredev/iopincfg.h with the PmnPFS registers, R9A02G021 User's
Manual R01UH1036EJ0110 chapter 16, and the IRQ0 to IRQ7 pin interrupts
through the ICU, chapter 12.

PinOp is the PSEL code of manual 16.6: IOPINOP_GPIO for a general I/O,
the R9A02_PSEL_xxx value for a peripheral (R9A02_PSEL_SAU is 0x11, which is
IOPINOP_FUNC16), IOPINOP_FUNC31 for the analog function (ASEL). The caller
picks the code from the pin tables, IOPinConfig checks that the code exists
on this part.

Hardware rules applied by IOPinConfig:
- An SAU pin configured as input gets NCODR = 1, the SAU output buffer of a
  data pin is on in input mode and NCODR = 1 puts it in Hi-Z (16.5.1).
- There is no pull-down. IOPINRES_PULLDOWN is refused.
- P002, P003, P006, P007, P200, P400, P401 have no open-drain output.
- P200 is input only.

Unsupported requests leave the pin unchanged.

@author	Hoang Nguyen Hoan
@date	May. 11, 2026

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
#include <stdbool.h>
#include <stddef.h>

#include "iopinctrl.h"
#include "interrupt_r9a02.h"

#define R9A02_IRQ_LINE_CNT		8

// PSEL codes that exist on this part, manual 16.6
#define R9A02_PSEL_VALID_MSK	((1UL << R9A02_PSEL_UARTA) | (1UL << R9A02_PSEL_IICA) | \
								 (1UL << R9A02_PSEL_CLKOUT_CMP) | (1UL << R9A02_PSEL_CAC) | \
								 (1UL << R9A02_PSEL_REMC) | (1UL << R9A02_PSEL_SAU) | \
								 (1UL << R9A02_PSEL_KINT) | (1UL << R9A02_PSEL_TAU) | \
								 (1UL << R9A02_PSEL_RTC))

#define R9A02_PFS_PIN_MSK		(R9A02_PFS_PODR | R9A02_PFS_PDR | R9A02_PFS_PCR | R9A02_PFS_NCODR | \
								 R9A02_PFS_DSCR_Msk | R9A02_PFS_EOFR_Msk | R9A02_PFS_ISEL | \
								 R9A02_PFS_ASEL | R9A02_PFS_PMR | R9A02_PFS_PSEL_Msk)

typedef struct __R9a02_Pin_Hook {
	IOPinEvtHandler_t Handler;
	void *pCtx;
	int IntNo;					//!< IELSR number from R9a02RegisterIntHandler
	uint8_t Pin;				//!< (port << 4) | pin, valid while Handler is set
} R9a02PinHook_t;

// IRQ pins of the 48-pin part, manual tables 16.6 to 16.9, packed
// (IRQ << 8) | (port << 4) | pin
static const uint16_t s_R9a02PinIrqMap[] = {
	0x030,						// IRQ0_A P300
	0x131,						// IRQ1_A P301
	0x212, 0x233, 0x222,		// IRQ2_A P102, IRQ2_B P303, IRQ2_C P202
	0x307, 0x332, 0x321,		// IRQ3_A P007, IRQ3_B P302, IRQ3_C P201
	0x406, 0x418,				// IRQ4_A P006, IRQ4_B P108
	0x501, 0x519,				// IRQ5_A P001, IRQ5_B P109
	0x600, 0x61B, 0x610,		// IRQ6_A P000, IRQ6_B P111, IRQ6_C P100
	0x703, 0x71A, 0x711,		// IRQ7_A P003, IRQ7_B P110, IRQ7_C P101
};

static R9a02PinHook_t s_R9a02PinHook[R9A02_IRQ_LINE_CNT];

static int R9a02PinIrq(int PortNo, int PinNo)
{
	if (!R9a02PinValid(PortNo, PinNo))
	{
		return -1;
	}

	unsigned pin = ((unsigned)PortNo << 4) | (unsigned)PinNo;

	for (size_t i = 0; i < sizeof(s_R9a02PinIrqMap) / sizeof(s_R9a02PinIrqMap[0]); i++)
	{
		if ((s_R9a02PinIrqMap[i] & 0xFFU) == pin)
		{
			return s_R9a02PinIrqMap[i] >> 8;
		}
	}

	return -1;
}

static bool R9a02PinAnalog(int PortNo, int PinNo)
{
	static const uint16_t s_Analog[R9A02_PORT_COUNT] = { 0x00CF, 0x00F0, 0, 0, 0x0003 };

	return (s_Analog[PortNo] & (1U << (unsigned)PinNo)) != 0U;
}

static bool R9a02PinOpenDrain(int PortNo, int PinNo)
{
	static const uint16_t s_OpenDrain[R9A02_PORT_COUNT] = { 0x0F03, 0x0FFF, 0x00FE, 0x00FF, 0x000C };

	return (s_OpenDrain[PortNo] & (1U << (unsigned)PinNo)) != 0U;
}

// Caller disables interrupts. PSEL and ASEL change only while PMR is 0
// (16.5.3). Keeps the PWPR state of the caller.
static bool R9a02PfsWrite(int PortNo, int PinNo, uint32_t Value)
{
	uint8_t pwpr = R9A02_PFS_PWPR;

	R9A02_PFS_PWPR = 0U;
	R9A02_PFS_PWPR = R9A02_PWPR_PFSWE;

	volatile uint32_t *pfs = &R9A02_PFS(PortNo, PinNo);
	uint32_t old = *pfs & R9A02_PFS_PIN_MSK;

	Value &= R9A02_PFS_PIN_MSK;

	if ((old ^ Value) & (R9A02_PFS_PSEL_Msk | R9A02_PFS_ASEL | R9A02_PFS_PMR))
	{
		if (old & R9A02_PFS_PMR)
		{
			*pfs = old & ~R9A02_PFS_PMR;
		}
		*pfs = Value & ~R9A02_PFS_PMR;
	}
	*pfs = Value;

	bool ok = (*pfs & R9A02_PFS_PIN_MSK & ~R9A02_PFS_PODR) == (Value & ~R9A02_PFS_PODR);

	R9A02_PFS_PWPR = 0U;
	R9A02_PFS_PWPR = (pwpr & R9A02_PWPR_PFSWE) ? R9A02_PWPR_PFSWE : R9A02_PWPR_B0WI;

	return ok;
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if (!R9a02PinValid(PortNo, PinNo) || PinOp < IOPINOP_GPIO || PinOp > IOPINOP_FUNC31 ||
		(Dir != IOPINDIR_INPUT && Dir != IOPINDIR_OUTPUT && Dir != IOPINDIR_BI) ||
		(Resistor != IOPINRES_NONE && Resistor != IOPINRES_PULLUP) ||
		(Type != IOPINTYPE_NORMAL && Type != IOPINTYPE_OPENDRAIN))
	{
		return;
	}

	bool analog = PinOp == IOPINOP_FUNC31;
	bool inonly = PortNo == 2 && PinNo == 0;

	if (analog && (!R9a02PinAnalog(PortNo, PinNo) || Dir != IOPINDIR_INPUT ||
		Resistor != IOPINRES_NONE || Type != IOPINTYPE_NORMAL))
	{
		return;
	}

	if (inonly && (PinOp != IOPINOP_GPIO || Dir != IOPINDIR_INPUT || Resistor != IOPINRES_NONE ||
		Type != IOPINTYPE_NORMAL))
	{
		return;
	}

	if (!analog && PinOp != IOPINOP_GPIO && (R9A02_PSEL_VALID_MSK & (1UL << (unsigned)PinOp)) == 0U)
	{
		return;
	}

	bool sauin = PinOp == R9A02_PSEL_SAU && Dir == IOPINDIR_INPUT;

	if ((Type == IOPINTYPE_OPENDRAIN || sauin) && !R9a02PinOpenDrain(PortNo, PinNo))
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t old = R9A02_PFS(PortNo, PinNo);

	// Configuration does not release a pin interrupt, IOPinDisableInterrupt does
	if (old & R9A02_PFS_ISEL)
	{
		EnableInterrupt(state);

		return;
	}

	uint32_t val = old & R9A02_PFS_PODR;

	if (analog)
	{
		val = R9A02_PFS_ASEL;
	}
	else
	{
		if (Dir == IOPINDIR_OUTPUT)
		{
			val |= R9A02_PFS_PDR;
		}
		if (PinOp != IOPINOP_GPIO)
		{
			val |= ((uint32_t)PinOp << R9A02_PFS_PSEL_Pos) | R9A02_PFS_PMR;
		}
		if (Resistor == IOPINRES_PULLUP)
		{
			val |= R9A02_PFS_PCR;
		}
		if (Type == IOPINTYPE_OPENDRAIN || sauin)
		{
			val |= R9A02_PFS_NCODR;
		}
	}

	R9a02PfsWrite(PortNo, PinNo, val);

	EnableInterrupt(state);
}

void IOPinDisable(int PortNo, int PinNo)
{
	if (!R9a02PinValid(PortNo, PinNo))
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	int line = R9a02PinIrq(PortNo, PinNo);

	if (line >= 0 && s_R9a02PinHook[line].Handler != NULL &&
		s_R9a02PinHook[line].Pin == (((unsigned)PortNo << 4) | (unsigned)PinNo))
	{
		IOPinDisableInterrupt(line);
	}

	uint32_t val = R9A02_PFS(PortNo, PinNo);

	if ((val & R9A02_PFS_ISEL) == 0U)
	{
		// Input, no pull-up, no function, output latch kept
		R9a02PfsWrite(PortNo, PinNo, val & R9A02_PFS_PODR);
	}

	EnableInterrupt(state);
}

static void R9a02PinCallback(int IntNo, void *pCtx)
{
	R9a02PinHook_t *hook = (R9a02PinHook_t *)pCtx;
	uintptr_t state = DisableInterrupt();
	IOPinEvtHandler_t handler = hook->IntNo == IntNo ? hook->Handler : NULL;
	void *ctx = hook->pCtx;
	int line = (int)(hook - s_R9a02PinHook);

	EnableInterrupt(state);

	if (handler != NULL)
	{
		handler(line, ctx);
	}
}

// IntNo is the IRQ line 0 to 7 of the pin. Edge sense only, the IRQCR low
// level mode has no IOPINSENSE value.
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense,
						  IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if ((unsigned)IntNo >= R9A02_IRQ_LINE_CNT || PortNo >= R9A02_PORT_COUNT || PinNo >= R9A02_PORT_PIN_COUNT ||
		R9a02PinIrq((int)PortNo, (int)PinNo) != IntNo || pEvtCB == NULL ||
		IntPrio < 0 || IntPrio > (int)R9A02_CLIC_PRIO_MAX ||
		Sense < IOPINSENSE_LOW_TRANSITION || Sense > IOPINSENSE_TOGGLE)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t old = R9A02_PFS(PortNo, PinNo);

	if (s_R9a02PinHook[IntNo].Handler != NULL ||
		(old & (R9A02_PFS_ASEL | R9A02_PFS_PDR | R9A02_PFS_PMR | R9A02_PFS_ISEL)))
	{
		EnableInterrupt(state);

		return false;
	}

	// One pin per IRQ line (16.2.5 ISEL)
	for (size_t i = 0; i < sizeof(s_R9a02PinIrqMap) / sizeof(s_R9a02PinIrqMap[0]); i++)
	{
		unsigned p = s_R9a02PinIrqMap[i] & 0xFFU;

		if ((s_R9a02PinIrqMap[i] >> 8) == (unsigned)IntNo && R9a02PinValid((int)(p >> 4), (int)(p & 15U)) &&
			(R9A02_PFS(p >> 4, p & 15U) & R9A02_PFS_ISEL))
		{
			EnableInterrupt(state);

			return false;
		}
	}

	// Manual 12.5.6: filter off, detection mode, then the pin and the link
	uint8_t irqmd = Sense == IOPINSENSE_LOW_TRANSITION ? R9A02_IRQCR_IRQMD_FALLING :
					Sense == IOPINSENSE_HIGH_TRANSITION ? R9A02_IRQCR_IRQMD_RISING : R9A02_IRQCR_IRQMD_BOTH;
	uint8_t oldcr = R9A02_ICU->IRQCR[IntNo];

	R9A02_ICU->IRQCR[IntNo] = oldcr & (uint8_t)~R9A02_IRQCR_FLTEN;
	R9A02_ICU->IRQCR[IntNo] = irqmd;

	R9a02PinHook_t *hook = &s_R9a02PinHook[IntNo];

	hook->pCtx = pCtx;
	hook->Pin = (uint8_t)((PortNo << 4) | PinNo);
	hook->IntNo = -1;
	hook->Handler = pEvtCB;

	bool ok = R9A02_ICU->IRQCR[IntNo] == irqmd && R9a02PfsWrite((int)PortNo, (int)PinNo, old | R9A02_PFS_ISEL);
	int intno = ok ? R9a02RegisterIntHandler((uint8_t)(R9A02_EVT_PORT_IRQ0 + IntNo), IntPrio, R9a02PinCallback, hook) : -1;

	if (intno < 0)
	{
		R9a02PfsWrite((int)PortNo, (int)PinNo, old);
		R9A02_ICU->IRQCR[IntNo] = oldcr;
		hook->Handler = NULL;
		hook->pCtx = NULL;
		EnableInterrupt(state);

		return false;
	}

	hook->IntNo = intno;
	EnableInterrupt(state);

	return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	int line = R9a02PinIrq(PortNo, PinNo);

	if (line < 0)
	{
		return -1;
	}

	return IOPinEnableInterrupt(line, IntPrio, (uint32_t)PortNo, (uint32_t)PinNo, Sense, pEvtCB, pCtx) ? line : -1;
}

void IOPinDisableInterrupt(int IntNo)
{
	if ((unsigned)IntNo >= R9A02_IRQ_LINE_CNT)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	R9a02PinHook_t *hook = &s_R9a02PinHook[IntNo];

	if (hook->Handler != NULL)
	{
		int port = hook->Pin >> 4;
		int pin = hook->Pin & 15;

		R9a02UnregisterIntHandler(hook->IntNo);
		R9a02PfsWrite(port, pin, R9A02_PFS(port, pin) & ~R9A02_PFS_ISEL);
		hook->Handler = NULL;
		hook->pCtx = NULL;
		hook->IntNo = -1;
	}

	EnableInterrupt(state);
}

// Port group input edge for the ELC (IOPORT_GROUP1, IOPORT_GROUP2), ports 1
// and 2 only (16.2.5 EOFR). Routing the group event is up to its user.
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	if (!R9a02PinValid(PortNo, PinNo) || PortNo < 1 || PortNo > 2 ||
		Sense < IOPINSENSE_DISABLE || Sense > IOPINSENSE_TOGGLE)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t val = R9A02_PFS(PortNo, PinNo);

	if ((val & R9A02_PFS_ASEL) == 0U)
	{
		val &= ~R9A02_PFS_EOFR_Msk;
		if (Sense == IOPINSENSE_LOW_TRANSITION)
		{
			val |= R9A02_PFS_EOFR_FALLING;
		}
		else if (Sense == IOPINSENSE_HIGH_TRANSITION)
		{
			val |= R9A02_PFS_EOFR_RISING;
		}
		else if (Sense == IOPINSENSE_TOGGLE)
		{
			val |= R9A02_PFS_EOFR_BOTH;
		}
		R9a02PfsWrite(PortNo, PinNo, val);
	}

	EnableInterrupt(state);
}

// No drive strength control on this part. DSCR of P100, P302 and P303 is a
// constant current sink mode enabled through CCDE (16.2.8), not a strength.
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	(void)PortNo;
	(void)PinNo;
	(void)Strength;
}

// No slew rate control on this part
void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	(void)PortNo;
	(void)PinNo;
	(void)Speed;
}
