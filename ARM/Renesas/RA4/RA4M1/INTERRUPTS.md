# RA4M1 interrupt architecture and maintenance guide

This document describes the Renesas RA4M1 interrupt architecture, the role of
`interrupt_ra4m1.cpp`, and the invariants that peripheral-driver maintainers
must preserve. See [README.md](README.md) for port
status and [TIMER.md](TIMER.md) for timer-specific constraints.

## Two distinct classes of vectors

The RA4M1 is an ARM Cortex-M4 with a standard vector table. Its first 16
positions belong to the ARM core:

| Vector position | Meaning |
| --- | --- |
| 0 | Initial MSP value, **not** an interrupt handler |
| 1 | Reset |
| 2 | NMI |
| 3 | HardFault |
| 4, 5, 6 | MemManage, BusFault, UsageFault |
| 7-10 | Reserved |
| 11 | SVCall |
| 12 | DebugMonitor |
| 13 | Reserved |
| 14 | PendSV |
| 15 | SysTick |

These exception positions are fixed by ARM. They do **not** use an ICU event
link and do **not** consume any of the 32 peripheral slots.

The next 32 table positions (vectors 16-47) are NVIC IRQ0-IRQ31, represented
in the port by `IEL0_IRQn` through `IEL31_IRQn`. Their **vector positions are
fixed**, but the *peripheral event source routed into each slot is
programmable*. Each NVIC slot has its own priority, enable, pending, and
active state. The peripherals do not all share a single IRQ or a common
serialized hardware handler.

```text
AGT0 overflow -----------+
GPT1 compare A ----------+        ICU event routing       NVIC         vector table
SCI1 receive ------------+---->  IELSR[slot] = event ---> IRQ(slot) ---> IELn_IRQHandler
GPIO pin IRQ line -------+
```

For example, this is an *illustrative assignment*, not a fixed mapping:

```cpp
R_ICU->IELSR[5] = RA4M1_EVTID_GPT0_OVF;
NVIC_SetPriority(IEL5_IRQn, 6);
NVIC_EnableIRQ(IEL5_IRQn);
```

A GPT0 overflow would then enter vector position 21, associated with NVIC
IRQ5. Production drivers should **not** write an allocated IELSR slot
directly; registration owns that mapping.

## Why `interrupt_ra4m1.cpp` exists

Renesas peripheral event IDs are **not** NVIC IRQ numbers. Unlike MCUs with
fixed `TIMx_IRQn`/`USARTx_IRQn` routing, a RA4M1 peripheral cannot simply
select a permanently assigned NVIC vector. The ICU must first connect its
event to a free CPU slot.

`interrupt_ra4m1.cpp` is IOsonata's shared **ICU event-link allocator,
registration and dispatcher**. It is not an alternative implementation of the
ARM NVIC and is not a single interrupt through which every event is serialized.

The public MCU-specific operations in `include/interrupt_ra4m1.h` are:

- `Ra4m1RegisterIntHandler(event, priority, handler, context)`: validate the
  event and priority, find an unused CPU IRQ slot, connect its IELSR and NVIC
  priority, and return the allocated `IRQn_Type`; returns -1 on failure.
- `Ra4m1UnregisterIntHandler(slot)`: detach a previously owned route after
  the peripheral event source has been quiesced.
- `Ra4m1AcknowledgeInt(slot)`: acknowledge an owned ICU route after the
  peripheral source has been negated/synchronized.

The dispatcher invokes the registered `handler(slot, context)` in interrupt
context. The `slot` is the dynamically assigned CPU slot, **not** the hardware
peripheral event ID, GPIO pin line, or timer trigger number.

`vectors_ra4m1.c` owns the physical vector table and references
`IEL0_IRQHandler` through `IEL31_IRQHandler`. The interrupt manager
provides weak vector entries forwarding to the appropriate slot dispatcher.
A strong application-specific vector override remains possible; the allocator
excludes such a slot rather than silently replacing its handler. Library
extraction depends on those vector references and must not be broken.

## Slot allocation and ownership rules

There are only **32 CPU external IRQ slots across the entire MCU**; there
are many more possible peripheral event sources. The allocator must not reuse a
slot already enabled, active, occupied by an existing event link, owned by a
handler, or overridden by an application-specific vector. Duplicate routing of
an event is rejected, including conflicts with existing DMAC event links.
Unrecognized/reserved event IDs and events not yet supported by this
implementation are rejected explicitly.

This is a resource constraint, not a peripheral limitation: adding more
timers, UARTs, or GPIO interrupts can exhaust the shared pool. Callers must
handle a failed registration and avoid leaving a partially initialized
peripheral or enabled IRQ source behind.

A GPT timer can require **one CPU slot for overflow plus one slot for each
enabled compare trigger**. The observed GPT1 test with four compare triggers
therefore may use five slots. SCI UART can require multiple distinct
receive/transmit/error event routes. A GPIO hardware IRQ *line* (e.g. IRQ6 pin
source) is distinct from whichever **CPU NVIC slot** the allocator assigns.

Do not change an allocated `R_ICU->IELSR[]` entry, the NVIC state of another
owner, or a DMA/DTC event route behind the manager. Do not use a returned CPU
slot as a stable application identifier: release/reallocation may assign a
different slot later.

## Peripheral-source acknowledgment is critical

Routing alone does not make an interrupt safe. A driver must establish the
peripheral's own interrupt source, settle relevant peripheral-register writes,
and then acknowledge the ICU request in the order required by the hardware
manual. The hardware event may remain asserted after entering an ISR.

For non-GPIO events, the registered peripheral handler must negate/clear the
source as appropriate and respect the peripheral's synchronization delay
before returning. The manager's trailing `IELSR.IR` acknowledgement must not
erase a new event that arrived while application callback code ran.

`Ra4m1AcknowledgeInt` is available when a source-specific handler has
already negated/synchronized its request and needs to acknowledge **before**
calling application code. Its generation tracking prevents a trailing
dispatch acknowledgement from clearing an event belonging to a newer
registration or a new callback-time request. The manager does not
unconditionally clear NVIC pending on normal ISR exit: that could lose a
new edge.

GPIO edge events have distinct timing: the current dispatcher acknowledges
their pulse before the callback after a peripheral read/barrier. GPIO
interrupt configuration and the mapping from an MCU pin to its **fixed GPIO
IRQ line** belong to `iopincfg_ra4m1.c`. The MCU pin line is then routed to
a dynamically allocated CPU vector by the ICU manager.

**Maintainer invariant:** never replace the handler with a generic
`NVIC_ClearPendingIRQ()` at ISR exit, clear every IELSR unconditionally, or
acknowledge a peripheral before its hardware request is actually negated.
Those shortcuts can create interrupt storms, suppress a second event, or
corrupt another owner's route.

## Concurrency and callback lifetime

The manager uses short CPU interrupt-masked critical sections to protect
registration state and ICU/NVIC changes. Application/peripheral callbacks
run outside those newly acquired global masks, still in ISR context.
Callbacks must be bounded and nonblocking; they must not assume a thread
context or use slow semihosted printing in latency measurements.

Register and unregister are ownership operations, not synchronization with
an already executing callback. A callback/context object must remain valid
until any active invocation has completed, including when higher-priority
interrupt nesting or callback-time reconfiguration is possible. Generation
checks protect route acknowledgment consistency; they do not themselves
extend an object's lifetime. Stop the event source and account for in-flight
callbacks before destroying a peripheral object.

A driver must never directly invoke another peripheral's callback through
the global interrupt manager. Dispatch is by the single event registered for
the allocated slot.

## Integration rules for new peripherals

1. Use the Renesas vendor CMSIS device header via `ra4m1xxx.h` and its
   register structures, particularly `R_ICU` and peripheral register members.
   Do not add custom peripheral address maps, hand-written register offsets,
   or FSP runtime dependencies.
2. Determine the correct RA4M1 **event ID** from the device hardware manual
   and `interrupt_ra4m1.h`. Do not confuse it with `IRQn_Type`.
3. Configure and quiesce the peripheral source before event registration.
   Register with `Ra4m1RegisterIntHandler`, check failure, and enable/arm
   the peripheral in the documented sequence.
4. On interrupt, negate/acknowledge the *peripheral* event, observe required
   synchronization, and acknowledge the ICU without losing a fresh event.
   Avoid `rdimon` semihosting in time-sensitive callbacks.
5. On shutdown, disable and settle the source **before** unregistering its
   event link. Maintain callback/context lifetime until no handler is active.
6. Consider the 32-slot budget. Avoid allocating IRQs unnecessarily; do not
   commandeer DMA/DTC or strong-override routes.
7. Maintain IOsonata style: file headers/license, grouped declarations,
   explicit braces and one statement per line, and generic peripheral APIs.

The present implementation intentionally does not activate general-purpose
DTC/DMAC routing, every ELC event, software-standby wake configuration, or NMI
services. Their event IDs in the header do not imply support. Extend routing
only alongside correct source-specific acknowledge and lifecycle semantics.

## Validation status and references

The user has verified RA4M1 firmware compilation, Blinky, AGT-based
TimerDemo initialization, and GPT1 with four compare triggers on an EK-RA4M1.
These validate basic allocation and dispatch, **not** exhaustive nested IRQ
behavior, register-route exhaustion, contention, callback re-registration, or
latency bounds. `rdimon` semihosting can halt execution and distort printed
callback-period measurements. Record timestamps in RAM before making
interrupt-latency claims.

Relevant files:

- `include/ra4m1xxx.h`, `include/R7FA4M1AB.h`
- `include/interrupt_ra4m1.h`
- `src/vectors_ra4m1.c`, `src/interrupt_ra4m1.cpp`
- `src/iopincfg_ra4m1.c`, `src/timer_ra4m1.cpp`,
  `src/uart_ra4m1.cpp`

Hardware reference: *Renesas RA4M1 Group User's Manual: Hardware*,
R01UH0887EJ0110 Rev.1.10, especially ICU chapter 13 and the event/interrupt
sections for each peripheral.
