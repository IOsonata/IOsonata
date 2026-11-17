# ESP32-C6 ClockDemo

Import `ioc` as an existing Eclipse managed project. Build the ESP32C6 library
and ClockDemo in the same configuration, then flash ClockDemo.elf using the
normal IOcomposer flash command. Debug and Release both use UART0 at
1,000,000 baud, 8N1, without flow control. The DevKit N8 USB-to-UART bridge
connects TX16 and RX17; the native USB JTAG/serial port is not this UART.
Open the terminal before pressing RESET. No jumper is needed. At each
`Entering sleep; press BOOT to resume` prompt, press and release BOOT (GPIO9).
The default run has nine sleep prompts.

The shared source is `exemples/misc/clock_demo.cpp`. Board configuration selects
three repetitions of the following sequence:

1. Select the 40 MHz XTAL, calibrated RC_FAST, then XTAL again.
2. At each source, select APB divisors 1, 2, 4, then 1. Check the selected
   frequency and readback and confirm the CPU rate remains unchanged.
3. After every APB request, run Timer 0 at 1 MHz with a 10 ms interrupt period.
   Report the timer's elapsed time during a 250 ms CPU delay and the number of
   interrupts. Check the delay is within 2 percent of 250 ms. Stop the timer
   before the next clock change.
4. At each CPU source, select and calibrate RC_SLOW, RC32K, then RC_SLOW again.
   Print the measured LF frequency. No external 32 kHz crystal is assumed.

Once per CPU-source step, the demo also selects APB divided by four, drains
UART output, calls Disable(), and enters SystemSleep with BOOT configured for
wake. After return it checks CPU/APB frequencies and the LF descriptor, calls
Enable() on UART, and repeats the delay/interrupt checks before restoring APB.
Cycle 1 sleeps with RC_SLOW, cycle 2 with RC32K and cycle 3 with RC_SLOW again.
The timer is stopped before entry. CPU and APB checks read hardware; the LF
check compares the saved descriptor and does not remeasure the LF oscillator.

Comment out `CLOCK_DEMO_WAKE_PIN` in board.h for the automatic clock-only run.
That configuration does not link the PMU sleep entry.

UART Disable now masks UART interrupts and gates the serial and bus clocks.
Enable restores both clocks and the saved interrupt mask without resetting baud
settings or FIFOs. The application must finish TX and stop incoming traffic
before Disable. No UART wake source is configured. This change requires a new
board run; the nine-wake result below used the earlier empty UART hooks.
Sleep-current measurements remain separate work.
SystemSleep can fall back to CPU WFI; returning with preserved clocks alone
is not proof of PMU entry or a particular current consumption. Confirm that each
prompt waits for BOOT before continuing.

Each operation prints its result. The final line should read:

```
ClockDemo complete: 0 failures
```

A rejected core source is reported as FAIL and its dependent steps are skipped.
The timer result checks that the counter advances and interrupt counts agree
with its elapsed time, allowing one interrupt at the sampling boundary. It
is not an independent measurement of the timer's absolute frequency. The
250 ms delay uses the calibrated CPU frequency, including fractional MHz.
The timer provides a separate PLL-derived reference on C6. UART text
remaining readable checks TX operation throughout the source/rate changes;
this demo does not check UART RX or other peripheral drivers.

On normal completion, the selected sources are XTAL for the CPU and RC_SLOW
for the LF clock, with APB at 40 MHz. The original PLL CPU source is not restored;
the current selector does not expose PLL selection. RESET restores the normal
startup configuration. No watchdog or power-off operation is enabled.

Validation: all 206 C6 library sources and this example compiled and linked
in Debug and Release using RISC-V GNU 15.2.0 and the Eclipse-managed source
lists/settings. The combined clock/sleep run at `b79ea4253` passed on hardware;
the subsequent UART shutdown changes await a board retest.

| Build | text | data | bss |
| --- | ---: | ---: | ---: |
| Debug | 58634 | 616 | 3292 |
| Release | 27818 | 596 | 3024 |

Hardware results before calibrated CPU timing (`e5044c35f`): all three cycles
completed with zero failures. Source selection, APB readback, LF calibration,
UART TX and timer interrupts passed. XTAL gave about 250021 us and 25 interrupts;
RC_FAST gave about 310200 us and 31 interrupts because delay calculation still
used the nominal 20 MHz rate. LF measurements were about 147.7 kHz RC_SLOW and
18.0 kHz RC32K. These observations do not specify oscillator accuracy.

C6 now measures RC_FAST against XTAL before selecting it. CPU and APB readback
use that measured rate. A failed or out-of-range measurement leaves the previous
clock selection and descriptor unchanged. Each RC selection measures again;
there is no background tracking of temperature or supply drift. ROM timing still
uses its integer-MHz interface, while IOsonata delays use full Hz precision.

The updated demo accepts the measured RC rate and derives APB requests from it.
On this board, expect CPU near 16.1 MHz and the 250 ms delay near 250000 us,
with about 25 interrupts for both sources. The new delay check is separate from
the counter/interrupt check; it can now fail even if callbacks agree with elapsed
time. Rebuild both the C6 library and ClockDemo before flashing.

Temporary production-source checks passed at O0 and Os for fractional-frequency
CPU/APB readback, repeated switching, failed calibration rollback, interrupt-mask
restoration, microsecond/nanosecond delay arithmetic and cycle-counter wrap.
Existing C3/C6 startup checks and five-chip timer checks also passed.

Calibrated timing at `6d3947a30` passed all three hardware cycles with zero
failures. RC_FAST measured 16134719-16136308 Hz; its 250 ms delay samples were
249969-250037 us. The 24/25 interrupt counts matched the sampling boundary.
XTAL restoration, divided APB settings, LF calibration and UART output passed.
This result covers the clock-only run. The combined run is recorded below.
Both sleep-enabled and clock-only Debug/Release configurations build.

The first combined run returned from sleep and passed CPU/APB readback and LF
descriptor checks, then stopped before the timer result. The C6 sleep code had
kept the PLL powered only when it supplied the CPU, overlooking the Timer Group
80 MHz PLL source while CPU ran from XTAL or RC_FAST. Light-sleep now keeps PLL
and its analog controls powered in both sleep and wake states regardless of the
CPU source. Deep-sleep still powers the PLL down and restarts through ROM.
A temporary PMU register model checked the retained settings for XTAL, RC_FAST
and PLL CPU selections with both internal LF sources, full saved-register
restoration, and unchanged Deep-sleep wake/reject settings at O0 and Os.
Hardware retest at `b79ea4253` completed three cycles and all nine BOOT wake
operations with `ClockDemo complete: 0 failures`. CPU and APB restoration,
LF descriptor preservation, timer interrupts and delay checks passed after
every wake. RC_FAST measured 16128365-16132336 Hz. The post-resume 250 ms
samples were 250019-250078 us, all with 25 interrupts. The timer stall is
resolved in this run. No sleep-current measurement was made.

The UART now also implements PowerOff: it clears hardware/software FIFOs,
releases interrupt routing and pins, and prevents Enable from restarting the
instance. Init is required to use it again. The Deep-sleep path in PowerOffDemo
uses that operation; ClockDemo uses resumable Disable/Enable.

UART validation after these changes: production-driver host checks passed for
C2/C3/C5/C6/C61 at O0/O2, including polling and interrupt configurations,
reference-counted suspend, repeated resume, queue preservation, PowerOff after
Disable, rejection of Enable after PowerOff, and successful reinitialization.
Both UART source files cross-compiled for all five chips at O0/Os. C6 ClockDemo
and PowerOffDemo rebuilt with all 206 library sources in Debug and Release.
