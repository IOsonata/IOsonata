/**-------------------------------------------------------------------------
@file	iopincfg_samd.c

@brief	SAM D11/D21 I/O pin configuration and pin interrupts

PinOp IOPINOP_FUNC0 to IOPINOP_FUNC7 select the peripheral functions A to H
of the PORT multiplexer.

The PORT has no open drain output. A GPIO pin configured with
IOPINTYPE_OPENDRAIN is left unchanged. SERCOM I2C drives its pins open drain
by itself, peripheral pins take any type.

Pin interrupts use the EIC. IntNo is the EXTINT line of the pin, fixed by
the device: 0 to 7 on SAM D11, 0 to 15 on SAM D21. IOPinAllocateInterrupt
returns it. The pin is switched to the EIC function A while its interrupt is
enabled, its input buffer stays on so IOPinRead still reads it. The NMI pin
(PA14 on SAM D11, PA08 on SAM D21) has no EXTINT line. The EIC takes a new
sense setting only while disabled, the other lines miss the edges of those
few cycles. SamdEicEvent turns on the event output of a line for the event
system, the timer external trigger: a line is used either for a pin
interrupt or for its event.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "samdxxx.h"

#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "clock_samd.h"
#include "evsys_samd.h"

#define SAMD_EIC_SYNC_POLL_LIMIT	100000UL
#define SAMD_PMUX_EIC				0U		// EIC is function A

// EIC CONFIG sense values
#define SAMD_EIC_SENSE_NONE			0U
#define SAMD_EIC_SENSE_RISE			1U
#define SAMD_EIC_SENSE_FALL			2U
#define SAMD_EIC_SENSE_BOTH			3U

#pragma pack(push, 4)

typedef struct {
	uint8_t PortNo;
	uint8_t PinNo;
	uint8_t ExtInt;			//!< EXTINT line of the pin
} SamdExtIntPin_t;

typedef struct {
	IOPinEvtHandler_t Handler;
	void *pCtx;
	int8_t PortNo;			//!< -1 line not used
	int8_t PinNo;
} SamdPinInt_t;

#pragma pack(pop)

// EXTINT line of each pin, from the device header pin definitions
// PIN_PxyyA_EIC_EXTINTn
static const SamdExtIntPin_t s_SamdExtIntPins[] = {
#if defined(__SAMD11D14AM__)
	{ IOPORTA, 2, 2 }, { IOPORTA, 3, 3 }, { IOPORTA, 4, 4 }, { IOPORTA, 5, 5 },
	{ IOPORTA, 6, 6 }, { IOPORTA, 7, 7 }, { IOPORTA, 8, 6 }, { IOPORTA, 9, 7 },
	{ IOPORTA, 10, 2 }, { IOPORTA, 11, 3 }, { IOPORTA, 15, 1 }, { IOPORTA, 16, 0 },
	{ IOPORTA, 17, 1 }, { IOPORTA, 22, 6 }, { IOPORTA, 23, 7 }, { IOPORTA, 24, 4 },
	{ IOPORTA, 25, 5 }, { IOPORTA, 27, 7 }, { IOPORTA, 30, 2 }, { IOPORTA, 31, 3 },
#elif defined(__SAMD21J18A__)
	{ IOPORTA, 0, 0 }, { IOPORTA, 1, 1 }, { IOPORTA, 2, 2 }, { IOPORTA, 3, 3 },
	{ IOPORTA, 4, 4 }, { IOPORTA, 5, 5 }, { IOPORTA, 6, 6 }, { IOPORTA, 7, 7 },
	{ IOPORTA, 9, 9 }, { IOPORTA, 10, 10 }, { IOPORTA, 11, 11 },
	{ IOPORTA, 12, 12 }, { IOPORTA, 13, 13 }, { IOPORTA, 14, 14 },
	{ IOPORTA, 15, 15 }, { IOPORTA, 16, 0 }, { IOPORTA, 17, 1 },
	{ IOPORTA, 18, 2 }, { IOPORTA, 19, 3 }, { IOPORTA, 20, 4 }, { IOPORTA, 21, 5 },
	{ IOPORTA, 22, 6 }, { IOPORTA, 23, 7 }, { IOPORTA, 24, 12 },
	{ IOPORTA, 25, 13 }, { IOPORTA, 27, 15 }, { IOPORTA, 28, 8 },
	{ IOPORTA, 30, 10 }, { IOPORTA, 31, 11 }, { IOPORTB, 0, 0 }, { IOPORTB, 1, 1 },
	{ IOPORTB, 2, 2 }, { IOPORTB, 3, 3 }, { IOPORTB, 4, 4 }, { IOPORTB, 5, 5 },
	{ IOPORTB, 6, 6 }, { IOPORTB, 7, 7 }, { IOPORTB, 8, 8 }, { IOPORTB, 9, 9 },
	{ IOPORTB, 10, 10 }, { IOPORTB, 11, 11 }, { IOPORTB, 12, 12 },
	{ IOPORTB, 13, 13 }, { IOPORTB, 14, 14 }, { IOPORTB, 15, 15 },
	{ IOPORTB, 16, 0 }, { IOPORTB, 17, 1 }, { IOPORTB, 22, 6 }, { IOPORTB, 23, 7 },
	{ IOPORTB, 30, 14 }, { IOPORTB, 31, 15 },
#else
#error Unsupported SAM D device
#endif
};

static const int s_NbSamdExtIntPins = sizeof(s_SamdExtIntPins) / sizeof(SamdExtIntPin_t);

static SamdPinInt_t s_SamdPinInt[EIC_EXTINT_NUM] = {
	[0 ... EIC_EXTINT_NUM - 1] = { NULL, NULL, -1, -1 }
};

// Lines with the event output on, for the event system
static uint32_t s_SamdEicEvent;

static bool SamdPinValid(int PortNo, int PinNo)
{
	return PortNo >= 0 && PortNo < PORT_GROUP_NUMBER && PinNo >= 0 && PinNo < 32;
}

// EXTINT line of a pin, -1 if it has none
static int SamdExtIntFind(int PortNo, int PinNo)
{
	for (int i = 0; i < s_NbSamdExtIntPins; i++)
	{
		if (s_SamdExtIntPins[i].PortNo == PortNo && s_SamdExtIntPins[i].PinNo == PinNo)
		{
			return s_SamdExtIntPins[i].ExtInt;
		}
	}

	return -1;
}

// Peripheral function of a pin, 0 to 7 for A to H
static void SamdPinMux(port_group_registers_t *pPort, int PinNo, uint32_t Func)
{
	uint32_t idx = (uint32_t)PinNo >> 1;
	uint8_t mux = pPort->PORT_PMUX[idx];

	if ((PinNo & 1) != 0)
	{
		mux = (mux & 0x0FU) | (uint8_t)(Func << 4);
	}
	else
	{
		mux = (mux & 0xF0U) | (uint8_t)Func;
	}
	pPort->PORT_PMUX[idx] = mux;
}

bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	// One CPU, no pin owner selection
	return CoreNo == 0 && SamdPinValid(PortNo, PinNo);
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	port_group_registers_t *pPort;
	uint32_t mask;
	uint8_t cfg;

	if (!SamdPinValid(PortNo, PinNo) || PinOp < IOPINOP_GPIO || PinOp > IOPINOP_PERIPHH ||
		(PinOp == IOPINOP_GPIO && Type != IOPINTYPE_NORMAL))
	{
		return;
	}

	pPort = &PORT_REGS->GROUP[PortNo];
	mask = 1UL << PinNo;

	// Drive strength is kept
	cfg = pPort->PORT_PINCFG[PinNo] & PORT_PINCFG_DRVSTR_Msk;

	// OUT selects pull up or pull down
	if (Resistor == IOPINRES_PULLUP || Resistor == IOPINRES_FOLLOW)
	{
		pPort->PORT_OUTSET = mask;
		cfg |= PORT_PINCFG_PULLEN_Msk;
	}
	else if (Resistor == IOPINRES_PULLDOWN)
	{
		pPort->PORT_OUTCLR = mask;
		cfg |= PORT_PINCFG_PULLEN_Msk;
	}

	if (PinOp != IOPINOP_GPIO)
	{
		SamdPinMux(pPort, PinNo, (uint32_t)(PinOp - IOPINOP_FUNC0));
		cfg |= PORT_PINCFG_PMUXEN_Msk | PORT_PINCFG_INEN_Msk;
	}
	else if (Dir == IOPINDIR_OUTPUT)
	{
		pPort->PORT_DIRSET = mask;
	}
	else
	{
		pPort->PORT_DIRCLR = mask;
		cfg |= PORT_PINCFG_INEN_Msk;
	}

	pPort->PORT_PINCFG[PinNo] = cfg;
}

void IOPinDisable(int PortNo, int PinNo)
{
	port_group_registers_t *pPort;

	if (!SamdPinValid(PortNo, PinNo))
	{
		return;
	}

	// Input buffer, pull and peripheral off
	pPort = &PORT_REGS->GROUP[PortNo];
	pPort->PORT_DIRCLR = 1UL << PinNo;
	pPort->PORT_PINCFG[PinNo] = 0;
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	port_group_registers_t *pPort;
	uint8_t cfg;

	if (!SamdPinValid(PortNo, PinNo))
	{
		return;
	}

	pPort = &PORT_REGS->GROUP[PortNo];
	cfg = pPort->PORT_PINCFG[PinNo];
	if (Strength == IOPINSTRENGTH_STRONG)
	{
		cfg |= PORT_PINCFG_DRVSTR_Msk;
	}
	else
	{
		cfg &= (uint8_t)~PORT_PINCFG_DRVSTR_Msk;
	}
	pPort->PORT_PINCFG[PinNo] = cfg;
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	// The PORT has no slew rate setting
	(void)PortNo;
	(void)PinNo;
	(void)Speed;
}

static bool SamdEicSync(void)
{
	uint32_t timeout = SAMD_EIC_SYNC_POLL_LIMIT;

	while ((EIC_REGS->EIC_STATUS & EIC_STATUS_SYNCBUSY_Msk) != 0U)
	{
		if (--timeout == 0U)
		{
			return false;
		}
	}

	return true;
}

// EIC bus and generic clock, then enabled
static bool SamdEicStart(void)
{
	if ((EIC_REGS->EIC_CTRL & EIC_CTRL_ENABLE_Msk) != 0U)
	{
		return true;
	}

	PM_REGS->PM_APBAMASK |= PM_APBAMASK_EIC_Msk;

	// Edge detection runs on the EIC generic clock
	if (!SamdGclkChannelEnable(GCLK_CLKCTRL_ID_EIC_Val, SAMD_GCLK_MAIN))
	{
		return false;
	}

	EIC_REGS->EIC_CTRL = EIC_CTRL_ENABLE_Msk;

	return SamdEicSync();
}

// Sense, filter and event output of a line. CONFIG and EVCTRL are written
// with the EIC disabled. Interrupts disabled by the caller.
static bool SamdEicSense(int ExtInt, IOPINSENSE Sense, bool bEvent)
{
	uint32_t sense;
	uint32_t shift = (uint32_t)(ExtInt & 7) * 4U;
	uint32_t cfg;

	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			sense = SAMD_EIC_SENSE_FALL;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			sense = SAMD_EIC_SENSE_RISE;
			break;
		case IOPINSENSE_TOGGLE:
			sense = SAMD_EIC_SENSE_BOTH;
			break;
		default:
			sense = SAMD_EIC_SENSE_NONE;
			break;
	}

	EIC_REGS->EIC_CTRL = 0;
	if (!SamdEicSync())
	{
		return false;
	}

	// Majority filter on 3 samples with any edge
	cfg = EIC_REGS->EIC_CONFIG[ExtInt >> 3] & ~(0xFUL << shift);
	if (sense != SAMD_EIC_SENSE_NONE)
	{
		cfg |= (sense | EIC_CONFIG_FILTEN0_Msk) << shift;
	}
	EIC_REGS->EIC_CONFIG[ExtInt >> 3] = cfg;

	if (bEvent && sense != SAMD_EIC_SENSE_NONE)
	{
		EIC_REGS->EIC_EVCTRL |= 1UL << ExtInt;
	}
	else
	{
		EIC_REGS->EIC_EVCTRL &= ~(1UL << ExtInt);
	}

	EIC_REGS->EIC_CTRL = EIC_CTRL_ENABLE_Msk;

	return SamdEicSync();
}

void IOPinDisableInterrupt(int IntNo)
{
	uint32_t state;

	if (IntNo < 0 || IntNo >= EIC_EXTINT_NUM)
	{
		return;
	}

	state = DisableInterrupt();

	EIC_REGS->EIC_INTENCLR = 1UL << IntNo;
	EIC_REGS->EIC_INTFLAG = 1UL << IntNo;

	if (s_SamdPinInt[IntNo].PortNo >= 0)
	{
		// Pin back to GPIO
		PORT_REGS->GROUP[s_SamdPinInt[IntNo].PortNo].PORT_PINCFG[s_SamdPinInt[IntNo].PinNo] &=
			(uint8_t)~PORT_PINCFG_PMUXEN_Msk;
		(void)SamdEicSense(IntNo, IOPINSENSE_DISABLE, false);
	}

	s_SamdPinInt[IntNo].Handler = NULL;
	s_SamdPinInt[IntNo].pCtx = NULL;
	s_SamdPinInt[IntNo].PortNo = -1;
	s_SamdPinInt[IntNo].PinNo = -1;

	if (EIC_REGS->EIC_INTENSET == 0U)
	{
		NVIC_DisableIRQ(EIC_IRQn);
	}

	EnableInterrupt(state);
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense,
						  IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	port_group_registers_t *pPort;
	uint32_t state;
	bool res;

	if (!SamdPinValid((int)PortNo, (int)PinNo) || IntNo < 0 || IntNo >= EIC_EXTINT_NUM ||
		SamdExtIntFind((int)PortNo, (int)PinNo) != IntNo || IntPrio < 0)
	{
		return false;
	}

	state = DisableInterrupt();

	// The line may be in use by another pin or by the event system
	if ((s_SamdPinInt[IntNo].PortNo >= 0 &&
		 (s_SamdPinInt[IntNo].PortNo != (int)PortNo || s_SamdPinInt[IntNo].PinNo != (int)PinNo)) ||
		(s_SamdEicEvent & (1UL << IntNo)) != 0U)
	{
		EnableInterrupt(state);

		return false;
	}

	res = SamdEicStart();
	if (res)
	{
		EIC_REGS->EIC_INTENCLR = 1UL << IntNo;

		s_SamdPinInt[IntNo].Handler = pEvtCB;
		s_SamdPinInt[IntNo].pCtx = pCtx;
		s_SamdPinInt[IntNo].PortNo = (int8_t)PortNo;
		s_SamdPinInt[IntNo].PinNo = (int8_t)PinNo;

		res = SamdEicSense(IntNo, Sense, false);
	}

	if (res)
	{
		// EIC function with the input buffer on
		pPort = &PORT_REGS->GROUP[PortNo];
		SamdPinMux(pPort, (int)PinNo, SAMD_PMUX_EIC);
		pPort->PORT_PINCFG[PinNo] |= PORT_PINCFG_PMUXEN_Msk | PORT_PINCFG_INEN_Msk;

		EIC_REGS->EIC_INTFLAG = 1UL << IntNo;
		EIC_REGS->EIC_INTENSET = 1UL << IntNo;

		NVIC_ClearPendingIRQ(EIC_IRQn);
		NVIC_SetPriority(EIC_IRQn, SamdIrqPrio(IntPrio));
		NVIC_EnableIRQ(EIC_IRQn);
	}
	else
	{
		s_SamdPinInt[IntNo].Handler = NULL;
		s_SamdPinInt[IntNo].pCtx = NULL;
		s_SamdPinInt[IntNo].PortNo = -1;
		s_SamdPinInt[IntNo].PinNo = -1;
	}

	EnableInterrupt(state);

	return res;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB,
						   void *pCtx)
{
	// The line is fixed by the pin
	int intno = SamdExtIntFind(PortNo, PinNo);

	if (intno < 0 || s_SamdPinInt[intno].PortNo >= 0)
	{
		return -1;
	}

	if (!IOPinEnableInterrupt(intno, IntPrio, (uint32_t)PortNo, (uint32_t)PinNo, Sense, pEvtCB, pCtx))
	{
		return -1;
	}

	return intno;
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	int intno = SamdExtIntFind(PortNo, PinNo);
	uint32_t state;

	if (intno < 0)
	{
		return;
	}

	state = DisableInterrupt();
	if ((s_SamdEicEvent & (1UL << intno)) == 0U && SamdEicStart())
	{
		(void)SamdEicSense(intno, Sense, false);
	}
	EnableInterrupt(state);
}

bool SamdEicEvent(int ExtInt, IOPINSENSE Sense)
{
	uint32_t state;
	bool res;

	if (ExtInt < 0 || ExtInt >= EIC_EXTINT_NUM)
	{
		return false;
	}

	state = DisableInterrupt();

	// Not on a line with a pin interrupt
	res = s_SamdPinInt[ExtInt].PortNo < 0 && SamdEicStart() &&
		  SamdEicSense(ExtInt, Sense, Sense != IOPINSENSE_DISABLE);
	if (res && Sense != IOPINSENSE_DISABLE)
	{
		s_SamdEicEvent |= 1UL << ExtInt;
	}
	else
	{
		s_SamdEicEvent &= ~(1UL << ExtInt);
	}

	EnableInterrupt(state);

	return res;
}

// Each flag is cleared before its handler, an edge during the handler stays
// pending
void EIC_Handler(void)
{
	uint32_t flags = EIC_REGS->EIC_INTFLAG & EIC_REGS->EIC_INTENSET;

	for (int i = 0; flags != 0U && i < EIC_EXTINT_NUM; i++)
	{
		uint32_t mask = 1UL << i;

		if ((flags & mask) != 0U)
		{
			flags &= ~mask;
			EIC_REGS->EIC_INTFLAG = mask;
			if (s_SamdPinInt[i].Handler != NULL)
			{
				s_SamdPinInt[i].Handler(i, s_SamdPinInt[i].pCtx);
			}
		}
	}
}
