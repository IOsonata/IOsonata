# SAM4L timers

The SAM4L backend implements the existing generic C/C++ Timer API. It follows
the STM32F0 port's static ownership and absolute-deadline trigger pattern.
No heap, SysTick allocation, pin configuration, or system clock changes are used.
Hardware validation is pending.

| Virtual DevNo | Peripheral | Triggers | Default frequency |
| --- | --- | --- | --- |
| 0 | AST | 1 (alarm 0) | Nearest prescaler rate to 1024 Hz |
| 1, 2, 3 | TC0 channels 0, 1, 2 | 3 per channel (RA, RB, RC) | PBA / 128 |
| 4, 5, 6 | TC1 channels 0, 1, 2 | 3 per channel (RA, RB, RC) | PBA / 128 |

The previous TC initializer was a stub. Its two block-level indices have been
replaced with six independent channel indices. `TimerGetHighFreqDevNo()` is 1.

## Immediate hardware test

1. Build `ARM/Microchip/SAM4L/SAM4LCxC/lib/ioc` in IOcomposer, then the existing
   `ARM/Microchip/SAM4L/SAM4LCxC/exemples/TimerDemo/ioc` project in the same
   Debug or Release configuration. The supplied project targets SAM4LC8C.
2. Confirm LED pins and, if needed, define `MCUOSC` in TimerDemo's `src/board.h`
   for your board. Existing LED definitions are retained; copied, unused UART
   definitions were replaced with USART1 RX PC26 / TX PC27, peripheral A,
   matching the SAM4LCxC CDC example. Open the UART console at 115200 8N1,
   no flow control. `TIMER_DEMO_UART` enables the shared demo's UART retargeting.
3. Start with `TIMER_DEVNO 0` in that board file. Flash and run continuously.
   `g_TimerInitOk` must become true, `g_TriggerCount[0]` must advance about ten
   times per second, and LED1 toggles on each trigger.
4. Repeat with `TIMER_DEVNO` 1 through 6, or define `TIMER_DEMO_DEVNO` in the
   compiler settings. TC trigger counters 0, 1, 2 should advance every 100,
   1000, and 250 ms. LEDs 1 and 2 toggle on triggers 0 and 1.
5. Inspect `g_TriggerPeriod[]` (accepted periods, ns), `g_Period[]` (measured
   periods, ns), and `g_TickCount` (elapsed ns). At 1024 Hz AST rounds 100 ms
   to 102 ticks, or 99.609375 ms. Allow initial callback latency and oscillator
   tolerance. TC's 1-second trigger crosses multiple 16-bit rollovers.

Both project configurations use nosys instead of semihosting. printf is
retargeted to USART1 at 115200 baud; a semihosting console is not required.
The debugger variables and LEDs remain available.
Do not use debugger halts as a timing measurement: AST can continue while halted.
AST also survives debugger, external and watchdog resets (datasheet table 10-12).
Initialization reclaims that retained hardware state when there is no live Timer
owner or enabled AST alarm/overflow NVIC delivery; CR.EN alone is not an ownership
check. The regression test starts with AST already enabled and stale count/flags.

If `g_TimerInitOk` is false, inspect `g_Sam4lTimerInitStage` before another Init:
1 = invalid configuration, 2 = software owner conflict, 3 = clock source not
ready/matching, 4 = no usable frequency, 5 = initial AST synchronization timeout,
6 = active IRQ/peripheral conflict, 7 = hardware setup failure, 8 = reset failure,
9 = start failure. Zero means initialization succeeded. This is target-local
debugger state; the generic Timer API is unchanged.

## Behavior and limits

- Init starts the timer. Disable pauses it; Enable resumes without resetting.
  Reset restarts elapsed ticks and active trigger phases. SetFrequency resets
  and starts the timer, preserving active trigger periods after quantization.
- AST uses the system's already-running CLK32 source. DEFAULT accepts that source;
  LFRC and LFXTAL require the matching selected source. It does not switch the
  shared oscillator. Its prescaler divides by `2^(PSEL+1)`; the default 32.768 kHz
  source gives a maximum counter frequency of 16384 Hz.
- TC supports DEFAULT (PBA divided by 2, 8, 32, or 128), reporting the nearest
  available integer frequency. It enables the required divided PBA clock and
  leaves sibling channels and block synchronization registers untouched.
- Single and continuous triggers accept 4 through UINT32_MAX ticks. Longer
  periods than the hardware counter cycle are supported. Continuous triggers
  retain phase and coalesce missed periods into one callback. Trigger callbacks
  take precedence over the device event handler, matching the generic contract.
- Counters are extended to 64 bits. Interrupts or count reads must service each
  hardware wrap: TC every `65536 / actual_frequency` seconds (about 174.76 ms at
  48 MHz PBA / 128), AST every `2^32 / actual_frequency` seconds. Multiple wraps
  while interrupts are masked cannot be recovered from a single status flag.
- TC requires the PBA clock while running; no deep-sleep clock policy is added.
  AST backup-domain wake and restart behavior is not implemented here.
- External capture and per-tick interrupts are unsupported and rejected.
  Reinitializing a live handle, moving it to another timer, or taking another
  handle's channel is rejected. Keep the Timer object alive while in use.
- Do not retime the system clock while timers run. AST synchronization waits are
  bounded; a timeout disables its NVIC delivery and subsequent Enable fails.

Register sequences follow ATSAM4L8/L4/L2 datasheet 42023H (November 2016),
sections 10.7.5, 19.5/19.6, and 30.6/30.9/30.10. AST uses synchronized W1C
acknowledgement; TC status reads account for rollover and preserve due triggers
through absolute deadlines.

## Reproducible checks

From the repository root:

```sh
c++ -std=c++20 -O2 -IARM/Microchip/SAM4L/include -Iinclude \
  tests/sam4l/timer_test.cpp -o /tmp/sam4l_timer_test
/tmp/sam4l_timer_test
python3 tests/sam4l/timer_build_test.py --toolchain-prefix arm-none-eabi-
```

The register model compiles all three production sources and checks all seven
channels, ownership, clock division, pause/resume, single/continuous triggers,
long deadlines, missed-period coalescing, callback cancellation/reset, rollover
races, read-to-clear/W1C acknowledgement, frequency changes, and a stuck AST
synchronization flag. It does not simulate electrical clock timing.

The Arm check builds and links the existing TimerDemo for every DevNo in Debug
and Release, and checks that every timer vector resolves to its strong handler.
It does not invoke IOcomposer or validate hardware. The shared linker script
currently emits an RWX LOAD segment warning; this change does not modify it.
