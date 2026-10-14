/**-------------------------------------------------------------------------
@file	interrupt_r9a02.cpp

@brief	R9A02G021 ICU event routing and CLIC interrupt dispatch

Allocates the 32 ICU event link registers IELSRn to peripheral events and
dispatches the CLIC interrupts. See interrupt_r9a02.h.

Event to IELSRn group mapping: R9A02G021 User's Manual R01UH1036EJ0110,
table 12.7. The events that table 12.4 lets connect to the CLIC are listed,
and SYSTEM_SNZREQ, which starts the DTC only: it can be routed without a
handler (R9a02RouteEvent) for the DTC, never to the CPU. ELC_SWEVT0 and
ELC_SWEVT1 connect to the CLIC only after a DTC transfer, they are not
listed.

A slot with IELSRn.DTCE = 1 sends its event to the DTC (12.5.4.2). The DTC
clears DTCE after the last transfer and leaves IR set, the CPU interrupt
then comes on the same slot. The dispatcher leaves a slot alone while DTCE
is 1.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include <stddef.h>

#include "coredev/interrupt.h"
#include "clic.h"
#include "interrupt_r9a02.h"

// Packed (event << 8) | (group << 5) | IELS value, manual table 12.7
#define R9A02_EVTMAP(Evt, Grp, Iels)	(uint16_t)(((uint32_t)(Evt) << 8) | ((uint32_t)(Grp) << 5) | (uint32_t)(Iels))

typedef struct __R9a02_Int_Hook {
	R9a02IRQHandler_t Handler;
	void *pCtx;
	uint32_t Generation;		//!< Changes on every register and unregister
	uint8_t EvtId;
	uint8_t Iels;
} R9a02IntHook_t;

static const uint16_t s_R9a02EvtMap[] = {
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ0, 0, 0x01), R9A02_EVTMAP(R9A02_EVT_PORT_IRQ0, 4, 0x01),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ1, 1, 0x01), R9A02_EVTMAP(R9A02_EVT_PORT_IRQ1, 5, 0x01),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ2, 2, 0x01), R9A02_EVTMAP(R9A02_EVT_PORT_IRQ2, 6, 0x01),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ3, 3, 0x01), R9A02_EVTMAP(R9A02_EVT_PORT_IRQ3, 7, 0x01),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ4, 4, 0x16),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ5, 5, 0x13),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ6, 6, 0x13),
	R9A02_EVTMAP(R9A02_EVT_PORT_IRQ7, 7, 0x11),
	R9A02_EVTMAP(R9A02_EVT_DTC_COMPLETE, 0, 0x02), R9A02_EVTMAP(R9A02_EVT_DTC_COMPLETE, 4, 0x02),
	R9A02_EVTMAP(R9A02_EVT_ICU_SNZCANCEL, 7, 0x04),
	R9A02_EVTMAP(R9A02_EVT_SYSTEM_SNZREQ, 2, 0x05), R9A02_EVTMAP(R9A02_EVT_SYSTEM_SNZREQ, 6, 0x05),
	R9A02_EVTMAP(R9A02_EVT_FCU_FRDYI, 2, 0x02), R9A02_EVTMAP(R9A02_EVT_FCU_FRDYI, 6, 0x02),
	R9A02_EVTMAP(R9A02_EVT_LVD_LVD1, 0, 0x04), R9A02_EVTMAP(R9A02_EVT_LVD_LVD1, 4, 0x04),
	R9A02_EVTMAP(R9A02_EVT_LVD_LVD2, 1, 0x04), R9A02_EVTMAP(R9A02_EVT_LVD_LVD2, 5, 0x04),
	R9A02_EVTMAP(R9A02_EVT_IWDT_NMIUNDF, 3, 0x03), R9A02_EVTMAP(R9A02_EVT_IWDT_NMIUNDF, 7, 0x03),
	R9A02_EVTMAP(R9A02_EVT_WDT_NMIUNDF, 0, 0x06), R9A02_EVTMAP(R9A02_EVT_WDT_NMIUNDF, 4, 0x06),
	R9A02_EVTMAP(R9A02_EVT_RTC_ALM_OR_PRD, 1, 0x05), R9A02_EVTMAP(R9A02_EVT_RTC_ALM_OR_PRD, 5, 0x05),
	R9A02_EVTMAP(R9A02_EVT_IIC0_ENDI, 0, 0x0B),
	R9A02_EVTMAP(R9A02_EVT_IIC1_ENDI, 1, 0x0B),
	R9A02_EVTMAP(R9A02_EVT_KEY_INTKR, 3, 0x0C), R9A02_EVTMAP(R9A02_EVT_KEY_INTKR, 7, 0x0C),
	R9A02_EVTMAP(R9A02_EVT_CAC_FERRI, 2, 0x10),
	R9A02_EVTMAP(R9A02_EVT_CAC_MENDI, 3, 0x10),
	R9A02_EVTMAP(R9A02_EVT_CAC_OVFI, 4, 0x10),
	R9A02_EVTMAP(R9A02_EVT_IOPORT_GROUP1, 2, 0x11),
	R9A02_EVTMAP(R9A02_EVT_IOPORT_GROUP2, 6, 0x11),
	R9A02_EVTMAP(R9A02_EVT_TRNG_RD_REQ, 0, 0x0E),
	R9A02_EVTMAP(R9A02_EVT_SAU0_ENDI0, 0, 0x12), R9A02_EVTMAP(R9A02_EVT_SAU0_ENDI0, 4, 0x1E),
	R9A02_EVTMAP(R9A02_EVT_SAU0_ENDI1, 1, 0x12), R9A02_EVTMAP(R9A02_EVT_SAU0_ENDI1, 5, 0x1E),
	R9A02_EVTMAP(R9A02_EVT_SAU0_ENDI2, 2, 0x12),
	R9A02_EVTMAP(R9A02_EVT_SAU0_ENDI3, 3, 0x12),
	R9A02_EVTMAP(R9A02_EVT_SAU1_ENDI0, 4, 0x12),
	R9A02_EVTMAP(R9A02_EVT_SAU1_ENDI1, 5, 0x12),
	R9A02_EVTMAP(R9A02_EVT_SAU0_INTSRE0, 1, 0x14),
	R9A02_EVTMAP(R9A02_EVT_SAU0_INTSRE1, 3, 0x14),
	R9A02_EVTMAP(R9A02_EVT_SAU1_INTSRE2, 5, 0x14),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI0, 0, 0x16),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI1, 1, 0x16),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI2, 2, 0x16),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI3, 3, 0x16),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI4, 4, 0x15),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI5, 5, 0x15),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI6, 6, 0x15),
	R9A02_EVTMAP(R9A02_EVT_TAU0_ENDI7, 7, 0x15),
	R9A02_EVTMAP(R9A02_EVT_TAU0_MODE8_ENDI1, 2, 0x17),
	R9A02_EVTMAP(R9A02_EVT_TAU0_MODE8_ENDI3, 3, 0x17),
	R9A02_EVTMAP(R9A02_EVT_TML32_OUTI, 0, 0x19), R9A02_EVTMAP(R9A02_EVT_TML32_OUTI, 4, 0x1F),
	R9A02_EVTMAP(R9A02_EVT_REMC_OUTI, 2, 0x19), R9A02_EVTMAP(R9A02_EVT_REMC_OUTI, 6, 0x19),
	R9A02_EVTMAP(R9A02_EVT_UARTA_TX_ENDI0, 0, 0x1C),
	R9A02_EVTMAP(R9A02_EVT_UARTA_RX_ENDI0, 1, 0x1C),
	R9A02_EVTMAP(R9A02_EVT_UARTA_RX_ERI0, 2, 0x1C),
	R9A02_EVTMAP(R9A02_EVT_UARTA_TX_ENDI1, 4, 0x1B),
	R9A02_EVTMAP(R9A02_EVT_UARTA_RX_ENDI1, 5, 0x1B),
	R9A02_EVTMAP(R9A02_EVT_UARTA_RX_ERI1, 6, 0x1B),
	R9A02_EVTMAP(R9A02_EVT_COMP_DET0, 0, 0x1D),
	R9A02_EVTMAP(R9A02_EVT_COMP_DET1, 1, 0x1D),
	R9A02_EVTMAP(R9A02_EVT_ADC_ENDI, 3, 0x1E), R9A02_EVTMAP(R9A02_EVT_ADC_ENDI, 7, 0x1F),
	R9A02_EVTMAP(R9A02_EVT_DOC_DOPCI, 3, 0x1F),
};

static const size_t s_R9a02EvtMapCnt = sizeof(s_R9a02EvtMap) / sizeof(s_R9a02EvtMap[0]);

static R9a02IntHook_t s_IcuHook[R9A02_ICU_IELSR_CNT];
static R9a02IntHook_t s_CoreHook[2];		// [0] MSIP, [1] MTIP

// Event of an IELSRn value, 0 when none
static uint8_t R9a02IelsEvent(uint32_t Group, uint32_t Iels)
{
	if (Iels == 0U)
	{
		return 0U;
	}

	for (size_t i = 0; i < s_R9a02EvtMapCnt; i++)
	{
		uint32_t m = s_R9a02EvtMap[i];

		if (((m >> 5) & 7U) == Group && (m & 0x1FU) == Iels)
		{
			return (uint8_t)(m >> 8);
		}
	}

	return 0U;
}

// IELS value of an event in a group, 0 when the group cannot take it
static uint8_t R9a02EventIels(uint8_t EvtId, uint32_t Group)
{
	for (size_t i = 0; i < s_R9a02EvtMapCnt; i++)
	{
		uint32_t m = s_R9a02EvtMap[i];

		if ((m >> 8) == EvtId && ((m >> 5) & 7U) == Group)
		{
			return (uint8_t)(m & 0x1FU);
		}
	}

	return 0U;
}

static inline uint8_t R9a02ClicCtl(int Prio)
{
	return ClicCtlFromLevel((uint8_t)Prio, R9A02_CLIC_NLBITS);
}

static void R9a02ClicOff(uint32_t ClicId)
{
	CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, ClicId) = 0U;
	CLIC_INT_IP_REG(R9A02_CLIC_CTRL_BASE, ClicId) = 0U;
}

static void R9a02ClicOn(uint32_t ClicId, int Prio)
{
	ClicEnableSlotRaw(R9A02_CLIC_CTRL_BASE, ClicId, CLIC_INT_ATTR_SHV_Msk, R9a02ClicCtl(Prio));
}

// IELSR route of an event. With a handler the CLIC input is enabled at Prio,
// without one it stays off and the IR flag is polled.
static int R9a02IcuRoute(uint8_t EvtId, int Prio, R9a02IRQHandler_t pHandler, void *pCtx)
{
	// SYSTEM_SNZREQ starts the DTC only (table 12.7 note 2)
	if (Prio < 0 || Prio > (int)R9A02_CLIC_PRIO_MAX || EvtId == 0U || EvtId > R9A02_EVT_MAX ||
		(pHandler != nullptr && EvtId == R9A02_EVT_SYSTEM_SNZREQ))
	{
		return -1;
	}

	uintptr_t state = DisableInterrupt();
	int slot = -1;

	for (uint32_t i = 0; i < R9A02_ICU_IELSR_CNT; i++)
	{
		uint32_t group = i & 7U;
		uint32_t iels = R9A02_ICU->IELSR[i] & R9A02_IELSR_IELS_Msk;

		// An event goes to one IELSR only
		if (s_IcuHook[i].EvtId == EvtId || R9a02IelsEvent(group, iels) == EvtId)
		{
			EnableInterrupt(state);

			return -1;
		}

		if (slot < 0 && s_IcuHook[i].EvtId == 0U && iels == 0U &&
			R9a02EventIels(EvtId, group) != 0U &&
			CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, R9A02_CLIC_ID_IELSR0 + i) == 0U &&
			g_R9a02VectorTable[R9A02_CLIC_ID_IELSR0 + i] == R9a02IrqEntry)
		{
			slot = (int)i;
		}
	}

	if (slot < 0)
	{
		EnableInterrupt(state);

		return -1;
	}

	R9a02IntHook_t &hook = s_IcuHook[slot];
	uint32_t clicid = R9A02_CLIC_ID_IELSR0 + (uint32_t)slot;
	uint8_t iels = R9a02EventIels(EvtId, (uint32_t)slot & 7U);

	hook.Handler = pHandler;
	hook.pCtx = pCtx;
	hook.EvtId = EvtId;
	hook.Iels = iels;
	hook.Generation++;

	// Manual 12.5.1: CLIC enable, then the event link
	if (pHandler != nullptr)
	{
		R9a02ClicOn(clicid, Prio);
	}
	R9A02_ICU->IELSR[slot] = iels;

	if ((R9A02_ICU->IELSR[slot] & (R9A02_IELSR_IELS_Msk | R9A02_IELSR_DTCE)) != iels)
	{
		R9A02_ICU->IELSR[slot] = 0U;
		R9a02ClicOff(clicid);
		hook.Handler = nullptr;
		hook.pCtx = nullptr;
		hook.EvtId = 0U;
		hook.Iels = 0U;
		slot = -1;
	}

	EnableInterrupt(state);

	return slot;
}

int R9a02RegisterIntHandler(uint8_t EvtId, int Prio, R9a02IRQHandler_t pHandler, void *pCtx)
{
	return pHandler != nullptr ? R9a02IcuRoute(EvtId, Prio, pHandler, pCtx) : -1;
}

int R9a02RouteEvent(uint8_t EvtId)
{
	return R9a02IcuRoute(EvtId, 0, nullptr, nullptr);
}

void R9a02UnregisterIntHandler(int IntNo)
{
	if (IntNo < 0 || IntNo >= (int)R9A02_ICU_IELSR_CNT)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	R9a02IntHook_t &hook = s_IcuHook[IntNo];

	if (hook.EvtId != 0U)
	{
		// Manual 12.5.2: event link and IR off, then the CLIC input
		R9A02_ICU->IELSR[IntNo] = 0U;
		(void)R9A02_ICU->IELSR[IntNo];
		R9a02ClicOff(R9A02_CLIC_ID_IELSR0 + (uint32_t)IntNo);
		hook.Handler = nullptr;
		hook.pCtx = nullptr;
		hook.EvtId = 0U;
		hook.Iels = 0U;
		hook.Generation++;
	}

	EnableInterrupt(state);
}

bool R9a02ClearPendingInt(int IntNo)
{
	if (IntNo < 0 || IntNo >= (int)R9A02_ICU_IELSR_CNT)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();
	R9a02IntHook_t &hook = s_IcuHook[IntNo];
	uint32_t clicid = R9A02_CLIC_ID_IELSR0 + (uint32_t)IntNo;
	bool pending = false;

	if (hook.EvtId != 0U)
	{
		pending = (R9A02_ICU->IELSR[IntNo] & R9A02_IELSR_IR) != 0U ||
				  CLIC_INT_IP_REG(R9A02_CLIC_CTRL_BASE, clicid) != 0U;

		// DTCE to 0 before IR is cleared (12.2.6)
		R9A02_ICU->IELSR[IntNo] = hook.Iels;
		(void)R9A02_ICU->IELSR[IntNo];
		R9A02_ICU->IELSR[IntNo] = hook.Iels;
		(void)R9A02_ICU->IELSR[IntNo];
		ClicClearPending(R9A02_CLIC_CTRL_BASE, clicid);
	}

	EnableInterrupt(state);

	return pending;
}

uint8_t R9a02IntEvent(int IntNo)
{
	return IntNo >= 0 && IntNo < (int)R9A02_ICU_IELSR_CNT ? s_IcuHook[IntNo].EvtId : 0U;
}

void R9a02IntEnable(int IntNo, bool bEnable)
{
	if (IntNo < 0 || IntNo >= (int)R9A02_ICU_IELSR_CNT)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t clicid = R9A02_CLIC_ID_IELSR0 + (uint32_t)IntNo;

	if (s_IcuHook[IntNo].Handler != nullptr)
	{
		if (bEnable)
		{
			// A request latched while the input was off is dropped
			ClicClearPending(R9A02_CLIC_CTRL_BASE, clicid);
			CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, clicid) = 1U;
		}
		else
		{
			CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, clicid) = 0U;
		}
	}

	EnableInterrupt(state);
}

static R9a02IntHook_t *R9a02CoreHook(uint32_t ClicId)
{
	if (ClicId == R9A02_CLIC_ID_MSIP)
	{
		return &s_CoreHook[0];
	}
	if (ClicId == R9A02_CLIC_ID_MTIP)
	{
		return &s_CoreHook[1];
	}

	return nullptr;
}

bool R9a02RegisterCoreIntHandler(uint32_t ClicId, int Prio, R9a02IRQHandler_t pHandler, void *pCtx)
{
	R9a02IntHook_t *hook = R9a02CoreHook(ClicId);

	if (hook == nullptr || pHandler == nullptr || Prio < 0 || Prio > (int)R9A02_CLIC_PRIO_MAX ||
		g_R9a02VectorTable[ClicId] != R9a02IrqEntry)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();
	bool ok = hook->Handler == nullptr;

	if (ok)
	{
		hook->Handler = pHandler;
		hook->pCtx = pCtx;
		hook->Generation++;
		R9a02ClicOn(ClicId, Prio);
	}

	EnableInterrupt(state);

	return ok;
}

void R9a02UnregisterCoreIntHandler(uint32_t ClicId)
{
	R9a02IntHook_t *hook = R9a02CoreHook(ClicId);

	if (hook == nullptr)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();

	R9a02ClicOff(ClicId);
	hook->Handler = nullptr;
	hook->pCtx = nullptr;
	hook->Generation++;

	EnableInterrupt(state);
}

// Level pins are acknowledged after the owner has handled the level. Keep
// this return path separate so pulse events can tail-call their handler
// without saving a hook, generation and register address across every IRQ.
__attribute__((noinline)) static void R9a02IcuLevelDispatch(uint32_t Slot, uintptr_t State)
{
	R9a02IntHook_t &hook = s_IcuHook[Slot];
	R9a02IRQHandler_t handler = hook.Handler;
	void *ctx = hook.pCtx;
	uint32_t generation = hook.Generation;

	EnableInterrupt(State);
	handler((int)Slot, ctx);

	State = DisableInterrupt();
	if (hook.Generation == generation)
	{
		R9A02_ICU->IELSR[Slot] = hook.Iels;
		(void)R9A02_ICU->IELSR[Slot];
	}
	EnableInterrupt(State);
}

static void R9a02IcuDispatch(uint32_t Slot)
{
	uint32_t clicid = R9A02_CLIC_ID_IELSR0 + Slot;
	uintptr_t state = DisableInterrupt();
	R9a02IntHook_t &hook = s_IcuHook[Slot];
	uint32_t reg = R9A02_ICU->IELSR[Slot];

	if (hook.Handler == nullptr || (reg & R9A02_IELSR_IELS_Msk) != hook.Iels)
	{
		// No owner, or the route was changed behind this module: stop the input
		CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, clicid) = 0U;
		EnableInterrupt(state);

		return;
	}

	// The DTC owns the event while DTCE is 1, the CPU interrupt follows its
	// last transfer with DTCE back at 0
	if ((reg & (R9A02_IELSR_IR | R9A02_IELSR_DTCE)) != R9A02_IELSR_IR)
	{
		EnableInterrupt(state);

		return;
	}

	bool level = hook.EvtId <= R9A02_EVT_PORT_IRQ7 &&
				 (R9A02_ICU->IRQCR[hook.EvtId - R9A02_EVT_PORT_IRQ0] & R9A02_IRQCR_IRQMD_Msk) == R9A02_IRQCR_IRQMD_LOW;

	if (level)
	{
		R9a02IcuLevelDispatch(Slot, state);

		return;
	}

	// Pulse event: clear IR now, a new event sets it again. There is no
	// post-handler work, including when the handler unregisters its route.
	R9A02_ICU->IELSR[Slot] = hook.Iels;
	(void)R9A02_ICU->IELSR[Slot];
	R9a02IRQHandler_t handler = hook.Handler;
	void *ctx = hook.pCtx;

	EnableInterrupt(state);

	handler((int)Slot, ctx);
}

void R9a02IrqDispatch(uint32_t ClicId)
{
	if (ClicId >= R9A02_CLIC_ID_IELSR0 && ClicId < R9A02_CLIC_INT_COUNT)
	{
		R9a02IcuDispatch(ClicId - R9A02_CLIC_ID_IELSR0);

		return;
	}

	R9a02IntHook_t *hook = R9a02CoreHook(ClicId);
	uintptr_t state = DisableInterrupt();
	R9a02IRQHandler_t handler = hook != nullptr ? hook->Handler : nullptr;
	void *ctx = handler != nullptr ? hook->pCtx : nullptr;

	if (handler == nullptr && ClicId < R9A02_CLIC_INT_COUNT)
	{
		CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, ClicId) = 0U;
	}

	EnableInterrupt(state);

	if (handler != nullptr)
	{
		handler((int)ClicId, ctx);
	}
}

