# STM32F030x8 port

STM32F030x8 implements the minimum supported MCU set: startup, GPIO, UART
and timer. Other F030 memory variants are not covered by the new timer backend.
GPIO includes pin configuration, digital I/O and pin-specific EXTI allocation.
UART supports polling and FIFO interrupts; its existing mode restrictions remain.

## Peripheral timers

All seven TIM peripherals are implemented through the standard C and C++ Timer
interfaces. Virtual numbering follows the low-power/low-frequency to
high-power/high-frequency convention. This MCU has no LPTIM backend; these TIM
peripherals all use the APB timer clock, with simpler timers ordered first.
The low-frequency device count is 0, high-frequency count is 7, and the first
high-frequency virtual index is 0. The order within that group is a device-table
ordering, not a measured power ranking.

| Virtual DevNo | Hardware | Trigger count | Implementation |
|---|---|---:|---|
| 0 | TIM6 | 1 | Basic timer update event |
| 1 | TIM14 | 1 | Compare channel |
| 2 | TIM16 | 1 | Compare channel |
| 3 | TIM17 | 1 | Compare channel |
| 4 | TIM15 | 2 | Compare channels |
| 5 | TIM3 | 4 | Compare channels |
| 6 | TIM1 | 4 | Compare channels; separate update and compare IRQs |

TIM7 is absent on x8. RTC and watchdogs are separate peripherals. Existing ARM
SysTick support remains unchanged and is not included in this TIM device table.

Each timer provides:

- APB clock/prescaler calculation, including the APB timer x2 rule;
- a 64-bit software-extended count with pending-overflow compensation;
- periodic and single-shot triggers, either a trigger callback or timer event;
- pause/resume preserving count and pending events;
- reset and frequency changes that restart active trigger periods;
- independent clock gates and IRQ routing, and rejection of occupied devices.

Use `TIMER_CLKSRC_DEFAULT`; no independent oscillator is selected. `Freq = 0`
selects the maximum clock rate. Other requests select a rounded, clamped
prescaler (1-65536); use the returned frequency. IRQ priority is 0-3.
Per-counter-tick interrupts (`bTickInt`), external triggers, input capture, DMA
and PWM output are outside this Timer backend. Unsupported configuration and
external-trigger requests return failure. No GPIO is configured by the timer.

Trigger periods are rounded to ticks and must be at least four ticks. Compare
timers support periods up to UINT32_MAX ticks across hardware wraps. TIM6's
single trigger uses ARR and supports up to 65536 ticks; reduce its frequency
for longer periods. Disabling its trigger restores the full 16-bit cycle while
preserving elapsed count. Late periodic service coalesces missed periods into
one callback while retaining phase. Callbacks execute in interrupt context.

Overflow IRQs must be serviced at least once per hardware cycle: 65536 ticks
on compare timers, or the selected ARR cycle on TIM6. For example, a 1 MHz
compare timer wraps every 65.536 ms, while an unprescaled 48 MHz timer wraps
every 1.365 ms. Interrupt masking beyond that interval cannot be reconstructed
from the one-bit update flag. Keep callbacks short and select a practical
counter frequency. These are internal timer interrupts, not RTOS ticks.

## TaktOS and interrupt ownership

The maintainer reports STM32F030x8 use in the TaktOS benchmark. Its
[STM32F0308 platform](https://github.com/IOsonata/TaktOS/tree/main/KVB/Targets/STM32F0308)
uses SysTick for kernel timing and TIM17 for its IRQ probe. This is existing
platform evidence, not an on-board result for the new peripheral timer driver.

Vector wrappers dispatch weakly to the timer driver only when it is linked.
Application IRQ overrides remain valid. Initialization rejects a timer whose
IRQ handler is overridden or already enabled. Thus the benchmark can retain
its TIM17 probe while using another timer; virtual timer 3 cannot be claimed
at the same time. Both TIM1 IRQs must belong to this driver. Do not replace an
IRQ handler or repurpose a peripheral after initializing it through Timer.
Timer handles and callback contexts must outlive the timer; disabling pauses
it and does not transfer ownership to another handle.

## Examples and validation

The [example index](STM32F030x8/exemples/README.md) lists GPIO, timer, UART,
TaktOS and software-only projects, their wiring, dependencies and exclusions.

Build the MCU library, then open
`STM32F030x8/exemples/TimerDemo/ioc/`. It links the existing shared
`exemples/timer/timer_demo.cpp`; application wiring remains in `board.h`.
The supplied board configuration uses the STM32F0308-DISCO PC9/PC8 LEDs and
USART1 PA9/PA10 with an external 3.3 V serial adapter. It does not assume that
the board's ST-LINK/V2 provides a virtual COM port.

Select `TIMER_DEMO_DEVNO` from 0 through 6 in `board.h` and rebuild the example
to exercise each timer. The 10 kHz counter allows all demo periods on TIM6.
Observe `g_TimerInitOk`, `g_TriggerCount`, `g_TriggerPeriod` and UART output.
The example source is unchanged apart from an optional interrupt-priority
configuration, required for the Cortex-M0 priority range.

From the repository root:

```sh
python3 tests/stm32f0/run.py
python3 tests/stm32f0/run_timer.py
python3 tests/stm32f0/run_timer_arm.py
```

The existing host startup/UART/GPIO regressions pass for F030x6/x8/xC and
F070x6/xB. New host models compile the production timer implementation and
vector wrappers with the ST register definitions and UBSan. They exercise all
seven devices and trigger channels, clocks, overflow, long compares, lifecycle,
callback cancellation and TIM17 override refusal. They do not simulate APB
bus latency or instruction-level interrupt preemption.

Arm GNU 14.3.1 Cortex-M0 checks at -O0 and -Os compile the backend and vectors
and link archive smoke images with timer use, application SysTick/TIM17
handlers, and SysTick-only use. The last image does not pull in TimerInit.
These smoke images are not the full TaktOS benchmark or IOC library build.
The new timer implementation and TimerDemo still require board execution.

Register behavior was checked against ST RM0360 Rev 5, clock-tree rules and
timer chapters 13-17. Board pins follow ST UM1658 / 32F0308DISCOVERY.
