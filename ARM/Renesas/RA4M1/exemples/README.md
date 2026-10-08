# RA4M1 examples

Import the existing library project at `../lib/ioc` (relative to this directory),
then import the desired example's `ioc` directory with **Existing Projects into
Workspace**. The library project is `IOsonata_RA4M1`. Build its matching Debug or
Release configuration before building the example; the example links that
configuration's archive. All projects use Cortex-M4 Thumb, FPv4-SP-D16,
hard-float ABI, GNU C17/C++23 and the native `gcc_ra4m1.ld` normal-boot layout.
The application project names remain `Blinky`, `TimerDemo`, `UartLoopback`, and
`UartPrbsTxTest`; the MCU is identified by its directory and library, not an
application-name suffix. Remove the old suffixed workspace entries before
reimporting these projects, without deleting their files from disk.

| Directory | IOC project | Shared application source | Initial configuration |
| --- | --- | --- | --- |
| `Blinky` | `Blinky` | `exemples/misc/blinky.c` | Three LED/test outputs, optional falling-edge button |
| `TimerDemo` | `TimerDemo` | `exemples/timer/timer_demo.cpp` | Timer DevNo 0, default count frequency, triggers according to device capability |
| `UartLoopback` | `UartLoopback` | `exemples/uart/uart_loopback.cpp` | SCI0, 115200 8N1, interrupt mode, no DMA |
| `UartPrbsTxTest` | `UartPrbsTxTest` | `exemples/uart/uart_prbs_tx.cpp` | SCI0, 115200 8N1, interrupt mode, no DMA |

The PRBS project additionally links unchanged `src/prbs.c`. Every application
source is an IOC linked resource; none is copied into the MCU example directory.
Each example's `src/board.h` is its application-local configuration. There is no
shared board runtime, generated FSP package, constructor-based GPIO setup or
pin wiring in the MCU library. USB, other unported peripherals, and RTOS examples
are not included.

## Application pin configuration

**Check/edit `src/board.h` before flashing.** The supplied headers provide a
concrete example pin map, not a claim about LEDs, connectors or buttons fitted
to any particular development board. Pins use MCU port/pin numbering.

The LED/test outputs are **P102, P111, P112**. Blinky is written for active-low
LEDs; connect LEDs only with suitable current-limiting resistors. These outputs
can instead be observed with a logic analyzer. The optional button input is
**P000 / IRQ6**, with an internal pull-up and falling-edge sensing. Grounding
it stops Blinky's LED sequence, following the shared example. Remove the button
macros to run the LED sequence continuously. Button debounce is not implemented.
The shared example's button callback uses `printf`; the provided projects do
not configure a console for that output.

Both UART examples use **SCI0: P100 = RXD0, P101 = TXD0**, with
`IOPINOP_FUNC3` (PSEL 4). Connect the adapter TX to P100, adapter RX to P101,
and a common ground. Use logic levels compatible with the target supply, not
RS-232 voltages. These mappings come from Renesas RA4M1 hardware manual
R01UH0887EJ0110, Table 19.6; the GPIO/IRQ choices are in chapters 1 and 19.
All selected pins are present in the port's 40/48/64/100-pin package maps.
Actual package-pad/connector numbers still depend on the chosen part/hardware.

The library's internal HOCO/LOCO defaults are retained: no oscillator override
or external crystal is needed. The initial port's 48 MHz clock configuration
requires the supply conditions documented in `../README.md`.

## What to observe

**Blinky:** each LED/test output is pulsed in sequence using the existing
approximate software delay. This is a GPIO/IRQ bring-up test, not a calibrated
timing test. `g_bBut1Pressed` is visible in the debugger. The button causes the
shared example to return from main; the existing reset/runtime exit handler then
stops execution.

**TimerDemo:** inspect `g_TimerInitOk`, `g_TriggerPeriod[]`
(accepted periods in nanoseconds), `g_TickCount` (elapsed nanoseconds recorded
by the callback), and `g_Period[]` (measured callback intervals in nanoseconds).
P102 toggles on trigger 0, P111 on trigger 1, and P112 on trigger 2 when supported.
`g_Period[0..3]` record trigger intervals and `g_Period[4]` records overflow intervals;
`g_TriggerCount[0..3]` count trigger callbacks. Overflow does not toggle a trigger pin.
A toggle interval is half the full output square-wave period. There is one
`TimerDemo` project for all timer devices, using `coredev/timer.h` and the same
`Timer` object. Set `TIMER_DEVNO` in its `src/board.h` to select a logical device;
leave `TIMER_FREQ=0` to select that device's default rate. No hardware-family
selector, separate example, or MCU-specific timer include is required.

RA4M1 has one device-number space: 0-1 are low-frequency timers and 2-9 are
high-frequency timers. The existing `TimerGetLowFreqDevCount()`,
`TimerGetHighFreqDevCount()` and `TimerGetHighFreqDevNo()` report 2, 8 and 2.
The hardware mapping stays in the MCU backend. Device 0 is the example default;
changing only `TIMER_DEVNO` to 2 selects the first high-frequency device.
`TimerGetMaxTrigger()` determines how many trigger channels the demo can use.
Devices 0-1 expose one trigger in this port, so the remaining accepted periods
remain zero; their reload event also produces the overflow indication.
Devices 2-9 expose six triggers; the demo uses the first four.
The shared demo also accepts the existing `TIMER_DEMO_DEVNO` / `TIMER_DEMO_FREQ`
settings used by other targets; when supplied they take precedence over
`TIMER_DEVNO` / `TIMER_FREQ`. The optional `TIMER_DEMO_UART` console remains
available and requires explicit application UART pin configuration.

The demo requests 100 ms, 1000 ms, 250 ms, and 500 ms, up to the selected
device's trigger capacity. Trigger 1 stays slower than trigger 0 for timers
with ordered compare requirements. A selected device must
also support those periods at the chosen count frequency. Device 0 or 2 works
with the default rate; devices 4-9 need a lower count frequency for a 1000 ms
request (for example, `TIMER_FREQ=10000`, checked against the returned rate).
A rejected period is reported as a failure, not silently accepted. These are
ordinary `TimerCfg_t` device/frequency choices within the same project.

**UartLoopback:** open a serial terminal at 115200 8N1, without flow control.
The firmware writes `UART Loopback Test` at startup and echoes received bytes.
Its application buffer now retains any unaccepted TX suffix before reading
another RX block. This prevents partial UART writes from silently dropping echo
bytes. RX can still overflow if the sender outruns the configured buffering;
this is not a hardware-flow-controlled connection.

**UartPrbsTxTest:** emits the existing `Prbs8` sequence, beginning with 0xFF in
byte mode, with no text banner mixed into the stream. The PRBS state advances
only after a byte is accepted. `g_UartInitOk` is available in both UART examples.
They allocate separate, word-aligned 256-byte RX/TX CFifo payload areas via the
existing `UARTFIFOSIZE` option; memory includes the CFifo headers. Set
`UART_INT_MODE=false` in the example configuration to exercise polling mode.
`UART_BAUDRATE` can be overridden; `UART_DMA_MODE` stays false for this port.

The projects use **newlib-nano + nosys**, not semihosting. No debugger console
is required to run the applications. Ordinary timer/Blinky `printf` output is
not routed to SCI automatically; use the GPIOs/debugger variables or the existing
retarget facilities once deliberately configured. The UART loopback banner is
sent through its UART object, independently of stdio retargeting.

Outputs are `ioc/Debug/<IOC-project-name>.elf` and the corresponding Release
file, plus the managed build's flash image and size report. The linker retains
`ResetEntry` and `__Vectors` from the archive and uses the MCU's option-memory
layout; no alternative reset handler or bootloader layout is supplied.

## Validation

From the repository root:

```sh
python3 tests/ra4m1/run_example_validation.py
python3 tests/ra4m1/run_timer_validation.py
```

The first command checks the four IOC projects and runs 96 host builds/tests
of the shared timer/UART examples. It builds the same timer application with
all ten logical device numbers (0-9) by overriding only `TIMER_DEVNO`, and checks
C API and C++ API selections, O0/Os/O2, and fatal AddressSanitizer/UndefinedBehaviorSanitizer. It exercises init-failure
handling, one/four-trigger selection, trigger-failure cleanup, callback values,
513-byte echo preservation under short/zero writes, and 1024 PRBS bytes with
retries. Its API wrappers/CMSIS are **reduced test contracts**, not a production
C++ wrapper or MCU execution test. The timer API double exercises device
selection and callback handling, not hardware period limits; those are tested
separately in `timer_model.cpp`. Blinky is linked unchanged and receives
project/pin-map checks, not an execution test in this suite.

Use `--metadata-only` for project checks without Clang++, and
`--allow-partial-checkout` only when testing the standalone port package without
unmodified shared repository files. Missing source existence checks are then
reported explicitly. Production GNU/newlib firmware linking, IOC import/managed
build, physical GPIO/UART behavior, oscillator accuracy and timing remain
unverified here. The driver regression limitations in `../TIMER.md` still apply.
