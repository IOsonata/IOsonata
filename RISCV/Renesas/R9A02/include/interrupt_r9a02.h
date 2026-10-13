/**-------------------------------------------------------------------------
@file	interrupt_r9a02.h

@brief	R9A02G021 interrupt routing, CLIC and ICU

CPU side, R9A02G021 User's Manual R01UH1036EJ0110 2.5.3 and 2.7: the CLIC
has 51 inputs. ID 3 is the machine software interrupt, ID 7 the machine
timer interrupt, IDs 19 to 50 are the ICU outputs IELSR0 to IELSR31. Every
input is rising edge, machine mode, hardware vectored through the table at
mtvt. Exceptions go to the trap entry at mtvec.

cliccfg.nlbits is 4: the 4 bits of clicintctl are the interrupt level.
IntPrio 0 to 15 is that level, a higher value is more urgent and preempts a
lower one. The interrupt entry re-enables mstatus.MIE around the handler, so
only a higher level can preempt.

Peripheral side, manual chapter 12: a peripheral event reaches the CPU only
through one of the 32 ICU event link registers IELSRn. The event number to
write depends on the group of IELSRn (n % 8), table 12.7. A peripheral
driver calls R9a02RegisterIntHandler with the R9A02_EVT_xxx event number;
it allocates a free IELSRn in a group that can take the event, programs it
and enables the CLIC input. The handler receives the IELSR number.

The ICU IR flag is cleared by the dispatcher before the handler is called.
Peripheral events on this part are pulses, so an event that comes while the
handler runs sets IR again and is not lost. For an IRQ pin in low level
mode IR is cleared after the handler returns, the handler must remove the
low level first.

Handlers run in interrupt context and must not block. NMI is not routed
through this module.

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
#ifndef __INTERRUPT_R9A02_H__
#define __INTERRUPT_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#include "r9a02g021.h"

#define R9A02_CLIC_NLBITS			4U				//!< All clicintctl bits are level

/// @brief	Interrupt handler
///
/// @param	IntNo	: IELSR number 0 to 31 for a peripheral event,
/// 					  CLIC ID for the machine timer and software interrupts
/// @param	pCtx	: Context given at registration
typedef void (*R9a02IRQHandler_t)(int IntNo, void *pCtx);

/// CLIC vector table entry
typedef void (*R9a02Vector_t)(void);

#ifdef __cplusplus
extern "C" {
#endif

/// CLIC vector table at mtvt, indexed by CLIC ID, in Vectors_R9A02.c
extern const R9a02Vector_t g_R9a02VectorTable[R9A02_CLIC_INT_COUNT];

/// Common interrupt entry, the default of every vector table entry
void R9a02IrqEntry(void);

/// @brief	Route an ICU event to the CPU.
///
/// Configure the peripheral with its interrupt source off before the call.
///
/// @param	EvtId		: R9A02_EVT_xxx event number, manual table 12.4
/// @param	Prio		: Interrupt level 0 to 15, higher is more urgent
/// @param	pHandler	: Handler called in interrupt context
/// @param	pCtx		: Handler context
///
/// @return	IELSR number 0 to 31, -1 if the event is not supported, already
/// 			routed, or no IELSR of its groups is free
int R9a02RegisterIntHandler(uint8_t EvtId, int Prio, R9a02IRQHandler_t pHandler, void *pCtx);

/// @brief	Route an ICU event to an IELSR without a CPU interrupt.
///
/// For a driver that polls: each event sets the IR flag of the IELSR
/// (R9a02EventFlag), the driver clears it (R9a02EventFlagClear). The CLIC
/// input stays off. R9a02UnregisterIntHandler removes the route.
///
/// @param	EvtId		: R9A02_EVT_xxx event number, manual table 12.4
///
/// @return	IELSR number 0 to 31, -1 if the event is not supported, already
/// 			routed, or no IELSR of its groups is free
int R9a02RouteEvent(uint8_t EvtId);

/// @brief	Remove an ICU route.
///
/// Stop the peripheral interrupt source first. The IELSR number may be given
/// to another event afterward.
///
/// @param	IntNo	: IELSR number returned by R9a02RegisterIntHandler or
/// 				  R9a02RouteEvent
void R9a02UnregisterIntHandler(int IntNo);

/// @brief	Drop a pending event of an ICU route.
///
/// Clears the IR flag and the CLIC pending bit, for a driver that has just
/// stopped the peripheral and accounted for the event itself.
///
/// @param	IntNo	: IELSR number returned by R9a02RegisterIntHandler
///
/// @return	true if an event was pending
bool R9a02ClearPendingInt(int IntNo);

/// @brief	IR flag of a route made by R9a02RouteEvent, set by each event.
///
/// @param	IntNo	: IELSR number
///
/// @return	true if an event occurred since the last R9a02EventFlagClear
static inline bool R9a02EventFlag(int IntNo)
{
	return (R9A02_ICU->IELSR[IntNo] & R9A02_IELSR_IR) != 0U;
}

/// @brief	Clear the IR flag of a route made by R9a02RouteEvent.
///
/// IR is cleared by writing 0, the event link is written back unchanged.
/// The event must have ended at least 2 module clock cycles before (12.2.6),
/// the drivers clear IR just before they start the next transfer.
///
/// @param	IntNo	: IELSR number
static inline void R9a02EventFlagClear(int IntNo)
{
	R9A02_ICU->IELSR[IntNo] = R9A02_ICU->IELSR[IntNo] & ~R9A02_IELSR_IR;
}

/// @brief	Set the handler of the machine timer or machine software interrupt.
///
/// @param	ClicId		: R9A02_CLIC_ID_MTIP or R9A02_CLIC_ID_MSIP
/// @param	Prio		: Interrupt level 0 to 15
/// @param	pHandler	: Handler called in interrupt context
/// @param	pCtx		: Handler context
///
/// @return	false if ClicId or Prio is not valid, or the input already has a
/// 			handler or an application vector
bool R9a02RegisterCoreIntHandler(uint32_t ClicId, int Prio, R9a02IRQHandler_t pHandler, void *pCtx);

/// @brief	Disable the machine timer or machine software interrupt.
///
/// @param	ClicId	: R9A02_CLIC_ID_MTIP or R9A02_CLIC_ID_MSIP
void R9a02UnregisterCoreIntHandler(uint32_t ClicId);

/// @brief	Trap and interrupt setup, called by SystemInit.
///
/// Sets mtvec, mtvt, cliccfg and NMIADDR, disables every CLIC input and
/// clears every IELSR.
void R9a02TrapInit(void);

/// @brief	Interrupt dispatcher, called by the interrupt entry.
///
/// @param	ClicId	: CLIC ID from mcause
void R9a02IrqDispatch(uint32_t ClicId);

/// @brief	Exception handler, weak.
///
/// The default stops in a loop with mcause, mepc and mtval in a0 to a2. An
/// application version may return the address to resume at.
///
/// @param	Cause	: mcause
/// @param	Epc		: mepc
/// @param	Tval	: mtval
///
/// @return	Address to resume at
uintptr_t R9a02ExceptionHandler(uint32_t Cause, uintptr_t Epc, uintptr_t Tval);

/// @brief	NMI handler, weak. Called from the NMI entry, which does not return.
void R9a02NmiHandler(void);

#ifdef __cplusplus
}
#endif

#endif
