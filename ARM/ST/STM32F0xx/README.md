# STM32F030x8 port

STM32F030x8 implements the minimum supported MCU set: startup, GPIO, UART
and timer. The timer driver is the one shared by the STM32 families.
GPIO includes pin configuration, digital I/O and pin-specific EXTI allocation.
UART supports polling, FIFO interrupts and DMA.

## UART DMA

The STM32 UART driver (`ARM/ST/src/uart_stm32.cpp`) runs DMA on USART1 and
USART2 of the F0 parts, with or without `bIntMode`. TX DMA sends blocks
straight from the TX CFIFO, a block ends on USART TC. RX DMA runs a 64-byte
circular ring, moved into the RX CFIFO at half and full ring, on IDLE and by
`RxData`.

With a non-blocking CFIFO (`bFifoBlocking` false) a put on a full CFIFO drops
its oldest bytes. TX DMA then sends a copy: each block, at most 32 bytes, is
taken out of the TX CFIFO and copied, so that a put drops only bytes still
queued, not the block being sent. The copy is made with the line idle, after
TC of the previous block, mostly with interrupts disabled. In the instruction
set emulator at 1 Mbaud, UartLoopback with interrupts and DMA echoed
91 kB/s with a non-blocking CFIFO and 98 kB/s with a blocking one; the
longest stretch with interrupts disabled grew from 880 to 1790 CPU cycles.
Not tested on hardware.

| Virtual DevNo | Hardware | TX DMA | RX DMA | Vector | CSELR request |
|---|---|---|---|---|---|
| 0 | USART1 | DMA1 channel 2 | DMA1 channel 3 | DMA1_Channel2_3 | 8 |
| 1 | USART2 | DMA1 channel 4 | DMA1 channel 5 | DMA1_Channel4_5 | 9 |

F030x4 to x8 and F070 have fixed requests; the USART1 remap bits of
SYSCFG_CFGR1 are cleared. F030xC and F09x select the request in CSELR. On F09x
the vectors are shared with DMA2 and with DMA1 channels 6 and 7, which the
driver does not use. The driver defines these two vectors, and the channels
belong to the UART while DMA is configured. Initialization refuses a TX
channel already configured by another user. The other F0 USARTs refuse DMA.

Hardware, STM32F030x8 on the STM32F0308-DISCO, FTDI, 1 Mbaud (maintainer):
UartLoopback 95-96 kB/s with interrupt and DMA, 94-95 kB/s polled DMA, 0
errors; PRBS TX at line rate in both DMA modes.

## Peripheral timers

The library builds the STM32 timer driver shared by the families:
`ARM/ST/src/timer_stm32.cpp` (TimerInit), `ARM/ST/src/timer_tim_stm32.cpp`
(TIM) and `ARM/ST/include/timer_stm32.h`. All seven TIM peripherals are
implemented through the standard C and C++ Timer interfaces. Virtual numbering
follows the low-power/low-frequency to high-power/high-frequency convention.
This MCU has no LPTIM peripheral; these TIM peripherals all use the APB timer
clock, with simpler timers ordered first. The low-frequency device count is 0,
high-frequency count is 7, and the first high-frequency virtual index is 0.
The order within that group is a device-table ordering, not a measured power
ranking.

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
- periodic and single-shot triggers; a trigger callback, when given, runs
  first, then the timer event handler gets the trigger event;
- pause/resume preserving count and pending events;
- reset and frequency changes that restart active trigger periods;
- independent clock enables and IRQ routing, and rejection of a timer counting
  without the driver or whose vector an application handler replaced.

Use `TIMER_CLKSRC_DEFAULT`; no independent oscillator is selected. `Freq = 0`
selects the maximum clock rate. Other requests select a rounded, clamped
prescaler (1-65536); use the returned frequency. IRQ priority is 0-3.
Per-counter-tick interrupts (`bTickInt`), external triggers, input capture, DMA
and PWM output are outside this Timer driver. Unsupported configuration and
external-trigger requests return failure. No GPIO is configured by the timer.
PWM output is the shared STM32 PWM driver, `ARM/ST/src/pwm_stm32.cpp`, built
by the F030x8 library: the generic PWM API on the TIM output channels with
the timer DevNo of this table, TIM14, TIM16 and TIM17 with one channel, TIM15
two, TIM3 and TIM1 four. See [STM32 PWM](../PWM.md). No example project; not
run on hardware yet. The library also builds the shared STM32 watchdog driver,
`ARM/ST/src/wdt_stm32.cpp`, on the IWDG: timeout up to 26.2 s from the 40 kHz
LSI, window, no timeout event. See [STM32 watchdog](../WDT.md). No example
project; not run on hardware yet. It also builds the shared STM32 RTC driver,
`ARM/ST/src/rtc_stm32.cpp`, with `src/miscdev/rtc.cpp`: calendar in the C
runtime types, alarm A, from the LSE or the 40 kHz LSI; the F030x8 has no
wakeup timer and no backup registers. See [STM32 RTC](../RTC.md). No example
project; not run on hardware yet.

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
platform evidence, not an on-board result for the peripheral timer driver.

The timer vectors of `lib/src/Vectors_STM32F030x8.c` are weak aliases of
`Stm32TimerVector`, which calls the weak `Stm32TimerIRQHandler` of the timer
driver with the interrupt number from SCB ICSR, or the default handler when
the driver is not linked. An application handler replaces the alias of its
vector. TimerInit refuses a timer whose vector does not point to
`Stm32TimerVector`, or that counts without the driver. Thus the benchmark can
retain its TIM17 probe while using another timer; virtual timer 3 cannot be
claimed at the same time. Both TIM1 vectors must belong to the driver. Do not
replace an IRQ handler or repurpose a peripheral after initializing it through
Timer. Timer handles and callback contexts must outlive the timer. TimerInit
with another handle takes the timer from the first one, whose calls then do
nothing; a handle initialized on another timer releases the timer it had.

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
The shared example uses optional UART output and board-selected timer settings.
It configures only the trigger channels supplied by the selected timer, checks
initialization and trigger setup, and prints from the main loop. The interrupt
callback records counts and periods and toggles the LED.

From the repository root:

```sh
python3 tests/stm32f0/run.py
python3 tests/stm32/run_uart.py
python3 tests/stm32/run_timer.py
```

The host clock and EXTI regressions pass for F030x6/x8/xC and F070x6/xB.
The UART test runs the shared driver for F030x6/x8/xC, F070x6/xB and F091xC,
see [tests/stm32f0/README.md](../../../tests/stm32f0/README.md).
`tests/stm32/run_timer.py` runs the shared timer driver with a TIM register
model for F030x8, F072xB and the parts of the other families: every timer
and trigger channel, clocks, overflow, long compares, lifecycle, callback
cancellation, a vector replaced by an application handler, the vector file
aliases. It does not simulate APB bus latency or instruction-level interrupt
preemption. Arm GNU checks compile the driver for every F0 device header at
-O0 and -Os and link F030x8 images: SysTick only (TimerInit not linked),
timer use, and an application TIM3_IRQHandler with timer use.

On 2026-10-08 the maintainer confirmed startup, LED GPIO, USART1 UART
output/retargeting, TIM6 (DevNo 0) and TIM16 (DevNo 2) on STM32F0308-DISCO
with the F030x8 timer driver this shared driver replaces. TIM6 reported 115
consecutive 100000 us intervals over 11.5 seconds through UART. Semihosting
caused the earlier timing variation. A missing level-shifter supply caused
the temporary UART output failure.

The successful hardware runs did not record an exact flashed revision, build
profile or complete tool versions. The shared driver is not yet tested on the
board. TimerDemo, built for each of the seven timers, ran 3.3 s in an
instruction set emulator with TIM, RCC and USART1 models: trigger 0 every
100 ms on each timer, trigger 1 every 1 s on TIM15, triggers 1 to 3 every 1 s,
250 ms and 500 ms on TIM3 and TIM1, the TIM1 compare interrupt on its own
vector. TIM14, TIM17, TIM15, TIM3 and TIM1 still need board tests.

Register behavior was checked against ST RM0360 Rev 5, clock-tree rules and
timer chapters 13-17. Board pins follow ST UM1658 / 32F0308DISCOVERY.
