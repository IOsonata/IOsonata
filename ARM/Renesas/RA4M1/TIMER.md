# RA4M1 timers

The RA4M1 timer implementation uses AGT0/1 and GPT0-7 through the existing
`coredev/timer.h` interface. Timer registers and interrupt handling are
configured in the MCU code; board pins belong in the application.

## Device numbering and implemented triggers

| TimerCfg_t.DevNo | Hardware | Counter | Count clock | Triggers |
| --- | --- | --- | --- | --- |
| 0, 1 | AGT0, AGT1 | 16-bit down counter, exposed as increasing ticks | LOCO or subclock, /1 through /128 | One reload/underflow trigger each |
| 2, 3 | GPT0, GPT1 | 32-bit up counter | PCLKD, /1, /4, /16, /64, /256, /1024 | Six compare triggers A through F |
| 4 through 9 | GPT2 through GPT7 | 16-bit up counter | Same PCLKD divisors | Six compare triggers A through F |

`TimerGetLowFreqDevCount()` returns 2; `TimerGetHighFreqDevCount()` returns 8;
`TimerGetHighFreqDevNo()` returns 2. Both C and the existing C++ `Timer` wrapper
use these same device numbers. Every initialized timer reserves one ICU slot
for counter extension. GPT trigger routes are allocated on demand from the
existing ICU manager. The MCU-wide limit of 32 CPU slots still applies; an
unavailable route fails initialization or trigger enable rather than sharing
another client's slot. A completed GPT single shot keeps its route for rearming;
`TimerDisableTrigger()` releases it.

## Frequency and tick counts

A zero requested frequency selects the maximum supported count frequency.
Otherwise the closest hardware divisor is selected, with ties choosing the
higher frequency. Always read the returned frequency. For example, PCLKD at
48 MHz can give 12 MHz or 750 kHz, but not an exact 1 MHz through GPT TPCS.
The integer frequency is the nominal source/divider result, truncated to whole
Hz when necessary, not a measurement of oscillator accuracy.

AGT DEFAULT follows the already initialized low-frequency oscillator descriptor.
LFRC selects an already running LOCO; LFXTAL requires the configured 32768 Hz
crystal and running subclock. GPT DEFAULT uses the current PCLKD. HFRC/HFXTAL
are accepted only when the established system source matches that request.
The timer driver does not start, switch or retune oscillators. Peripherals that use these
clocks must be stopped before changing system clocks; reset/reconfigure the
timer frequency before resuming timing measurements afterwards.

`GetTickCount` returns increasing 64-bit ticks, including one pending hardware
wrap that has not yet been serviced. The read path excludes software ISR
updates, samples the overflow state around the counter read, and rereads the
counter when an overflow races the sample. It does not consume the interrupt.
The overflow handler extends the count once and acknowledges the source before
calling application code. Comparisons account for a non-aligned software origin
after a pause or an AGT period change.

**The overflow interrupt must be serviced at least once per hardware period.**
Hardware flags cannot report multiple coalesced wraps. GPT16 at 48 MHz wraps
approximately every 1.365 ms; use a slower count frequency or GPT0/1 when long
interrupt masking is possible. AGT normally wraps after 65536 ticks, but its
hardware period is shortened while its reload trigger is configured. Missing
multiple AGT underflows also loses elapsed time. These are not wall-clock timers
while stopped, and callback latency is not included in a timing-accuracy claim.

## Trigger behavior

Periods are rounded to the nearest count tick, using split integer arithmetic
rather than multiplying by a rounded nanosecond tick period. Invalid, zero,
overflowing or out-of-range requests return zero. The accepted range is 4 to
65536 ticks for AGT and 4 to the maximum counter value for GPT. The four-tick
lower bound is a register-programming guard, not a guarantee that a CPU callback
can sustain that rate. The caller must budget ISR load and latency.

AGT exposes one trigger implemented with its hardware reload counter. This
keeps continuous intervals running without an ISR stop/restart at every event.
The counter is stopped and synchronized only when enabling, changing, disabling,
resetting or pausing the configuration. The software count origin preserves
elapsed ticks during period changes. A single shot delivers one trigger callback
but leaves the timer counting with that reload period for timekeeping; explicitly
disabling the trigger restores the 65536-tick cycle. Both AGT compare registers
are deliberately unused in this increment: immediate compare updates require
stopping, and running writes are buffered until underflow.

GPT exposes six independent compare triggers with buffering, capture, dead time
and pin outputs disabled. Its physical C/E/D register order is mapped explicitly.
Continuous triggers advance from the previous deadline, not the ISR's arrival
time. If service is late, missed intervals are coalesced into one callback and
the next future phase is programmed. A target crossed during register programming
is detected: a single-shot enable fails instead of waiting for another counter
wrap; a continuous trigger advances with a bounded retry. An unrecoverable
compare-write/programming failure disables that trigger and records a diagnostic.

The handler acknowledges its own ICU route before application code, using the
existing `Ra4m1AcknowledgeInt` function. A fresh callback-time arrival is not
cleared by the dispatcher's trailing acknowledgement. Callbacks are synchronous
ISR calls, outside newly acquired global interrupt masks. They may rearm or
disable triggers; they must not block. An already claimed callback may finish
across higher-priority preemption, so keep its timer object and context alive.
The Timer API has no release function; initialized objects must remain alive
while their timers are in use. `Disable` is a pause, not deallocation.

A trigger-specific handler takes precedence. When it is null, the timer's event
handler receives `TIMER_EVT_TRIGGER(n)`. Counter overflow notification uses
`TIMER_EVT_COUNTER_OVR`; AGT reload events can therefore produce an overflow
notification and a trigger notification. `bTickInt=true` is rejected: this
increment does not synthesize an interrupt for each counter increment.

## Start, stop, reset, and error handling

`TimerDisable` stops and settles the counter, folds elapsed ticks into the
software origin, and disables its CPU interrupts. `TimerEnable` resumes the
count and restarts active trigger periods from the resume point. `TimerReset`
zeros the count and restarts active trigger phases while retaining the previous
running/paused state. `TimerSetFrequency` resets and restarts the timer. Before changing hardware,
it rounds active trigger periods to the new tick interval.
An unrepresentable active trigger rejects the frequency change before mutation.

AGT start/stop waits for TCSTF with a bounded poll and accesses no other AGT
register during the transition. Its module-stop bit is set outside register
access, as required for LOCO/subclock operation. GPT configuration preserves
GTWP protection, observes its delayed final count edge on stop, and uses only
32-bit accesses, including on GPT16. Only zero is written to GPT status flags.
Compare IRQ causes come from their independent ICU latches, not sticky GTST
bits that another timer handler might clear.

Module writes preserve unrelated MSTPCRD and PRCR bits. GPT's shared module
clock remains enabled after pausing one channel; this driver does not gate a
neighbouring GPT. Raw running channels and existing CPU/DTC/DMAC routes for the
channel are rejected. Applications must also stop any other code or ELC peripheral using the channel
before assigning the channel to this driver. Register readback and bounded
handshakes detect the implemented failure cases, not every possible bus fault.
After a hardware failure, the timer or trigger may remain stopped; its previous
configuration is not restored. Read `g_Ra4m1TimerError[DevNo]` from `timer_ra4m1.h`. A failed stop
prevents the affected timer from being enabled; successful reset can recover
an initialized timer after a confirmed stop. A failed initialization never keeps
a caller's object pointer; unconfirmed hardware stop blocks reuse until MCU reset.

External clock input, external capture/trigger, PWM outputs, general ELC routing,
DMA/DTC activation and automatic software-standby wake configuration are outside
this increment. Their absence is explicit; external trigger enable returns false.
No application GPIO is selected or changed. Only AGT1 is a software-standby wake
source in the hardware manual; this port does not configure that wake policy.

## IOC integration and first test

The existing IOC library project links `src/timer_ra4m1.cpp`, its private register
header, the target mapping/diagnostic header, and unchanged shared
`src/coredev/timer.cpp`. The generic source is necessary for the C++ auto-select
trigger overloads. Debug/Release and hard-float settings remain unchanged.

A minimal application can initialize DevNo 0 at the default LF rate and enable
trigger 0 at 500 ms, then inspect a callback counter in the debugger. For a faster
test, use DevNo 2, request 12 MHz and enable trigger 0 at 1 ms. Supply an
application-owned output pin only when measuring callbacks on a scope; there is
no board pin in this driver. Repeat count reads across rollover, pause/resume,
reset and frequency change; compare observed intervals with the returned rate.
Use explicit trigger indices and check `nsTimerEnableTrigger`/`EnableTrigger`
return values so allocation or period failures are not hidden by a wrapper.

## Reproducible validation

Run from the repository root:

```sh
python3 tests/ra4m1/run_timer_validation.py
```

The runner executes the actual timer and ICU source against a register/event
model, with 411 assertions at O0, Os and O2 and under ASan/UBSan. It also runs
the existing 59 startup scenarios, 188 GPIO/ICU assertions and 224 UART
assertions, package/linker/archive checks and IOC metadata validation. Timer
Cortex-M4 C++11 compilation is checked at all three optimization levels for
soft and hard float ABI. The model includes all ten channels, correct access
widths/protection, asynchronous start/stop handshakes, pending-overflow races,
D/E mapping, all six GPT triggers, phase catch-up, programming latency,
callback-time arrivals, callback rearming, shared gates and failure paths.

**The tests use simplified headers, not the production headers.** The UART regression uses its
existing CFifo test double in the standalone source package; its runner also
uses production CFifo when that source is present in a full checkout. The
inherited ARM link probe validates startup/vector archive extraction, not a
complete timer application link. Production CMSIS/newlib GNU firmware build,
IOC managed build, hardware boot, physical periods, interrupt latency and
standby behavior have not been tested here.

## Hardware basis

Renesas RA4M1 Group User's Manual: Hardware, R01UH0887EJ0110 Rev.1.10,
September 29, 2023: chapters 10/12 (module stop and protection), 13 (ICU),
22 (GPT registers, compare sources and usage notes), and 23 (AGT registers,
reload operation and usage notes 23.4.1, 23.4.3 and 23.4.10). Renesas FSP
v6.6.0 `r_gpt.c` and `r_agt.c` were secondary cross-checks, not dependencies.
IOsonata interfaces and Renesas RE01 patterns were read at UART base commit
`9db6fae7ea6b5a06db56f5381c4e2069c2a69e22`.
