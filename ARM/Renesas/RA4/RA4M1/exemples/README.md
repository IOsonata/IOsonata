# RA4M1 examples

Import the existing library project at `../lib/ioc` (relative to this directory),
then import the desired example's `ioc` directory with **Existing Projects into
Workspace**. The library project is `IOsonata_RA4M1`. Build its matching Debug or
Release configuration before building the example; the example links that
configuration's archive. All projects use Cortex-M4 Thumb, FPv4-SP-D16,
hard-float ABI, GNU C17/C++23 and the native `gcc_ra4m1.ld` normal-boot layout.
The application project names remain `Blinky`, `TimerDemo`, `UartLoopback`,
`UartPrbsTxTest`, `I2CMasterSlave`, and `SPIMasterSlave`; the MCU is identified by its directory and library, not an
application-name suffix. Remove the old suffixed workspace entries before
reimporting these projects, without deleting their files from disk.

| Directory | IOC project | Shared application source | Initial configuration |
| --- | --- | --- | --- |
| `Blinky` | `Blinky` | `exemples/misc/blinky.c` | Three LED/test outputs, optional falling-edge button |
| `TimerDemo` | `TimerDemo` | `exemples/timer/timer_demo.cpp` | Timer DevNo 0, default count frequency, triggers according to device capability |
| `UartLoopback` | `UartLoopback` | `exemples/uart/uart_loopback.cpp` | SCI0, 115200 8N1, interrupt mode, no DMA |
| `UartPrbsTxTest` | `UartPrbsTxTest` | `exemples/uart/uart_prbs_tx.cpp` | SCI0, 115200 8N1, polling with TX DMA |
| `I2CMasterSlave` | `I2CMasterSlave` | `exemples/i2c/i2c_master_slave.cpp` | IIC0, 100 kHz, 7-bit master, polling |
| `SPIMasterSlave` | `SPIMasterSlave` | `exemples/spi/spi_master_slave.cpp` | RSPI0 master (polled, 400 kHz) to RSPI1 slave (interrupt), modes 1 and 3 |

The PRBS project additionally links unchanged `src/prbs.c`. Every application
source is an IOC linked resource; none is copied into the MCU example directory.
Each example's `src/board.h` is its application-local configuration. There is no
shared board runtime, generated FSP package, constructor-based GPIO setup or
pin wiring in the MCU library. USB projects are included; other unported peripherals and RTOS examples are not included.

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

The shared master/slave example uses **IIC0: P401 = SDA0, P408 = SCL0**, with
`IOPINOP_FUNC6` (PSEL 7), leaving SCI0 available for its console. On the
EK-RA4M1 these are J2-8 (P401) and J2-16 (P408), a short-wire option for master SCL. Provide a common ground and external pull-ups
sized for the bus voltage, capacitance and selected speed. The example requests 100 kHz with polling master. IIC1 uses P101 = SDA1 and
P100 = SCL1; connect SDA-to-SDA, SCL-to-SCL and common ground with pull-ups.
On the EK-RA4M1 these are J3-8 (P101, also PMOD A pin 2) and J1-1 (P100, also
PMOD A pin 3). The console of this example is SCI0 on P410 (RX) and P411 (TX).
P205 is not used for IIC1 here: it reaches J2-14 only with solder bridge E12
closed, which the board does not have by default.
A first native IIC slave ISR/callback implementation is now present, but its
address/restart/STOP transitions have not yet been hardware-validated. The
example is ready for bring-up, not a claimed loopback PASS. The shared
example remains unchanged; the target address is 0x22. This concrete example pin pair requires a
64- or 100-pin RA4M1 package; the driver itself also accepts the other legal IIC
pin alternatives documented in ../README.md.

The shared SPI example uses **RSPI0 as master and RSPI1 as slave** on one
board, with the console on SCI0 P410/P411. Wire the pins with four jumpers and a
shared ground. On the EK-RA4M1:

| Signal | RSPI0 master | RSPI1 slave | Jumper |
| --- | --- | --- | --- |
| SCK | P102, J2-6 | P111, J1-33 | J2-6 to J1-33 |
| MOSI | P101, J3-8 | P203, J1-12 | J3-8 to J1-12 |
| MISO | P100, J1-1 | P110, J1-9 | J1-9 to J1-1 |
| Chip select | P103, J1-29 (GPIO) | P112, J1-40 (SSL0) | J1-29 to J1-40 |

All are `IOPINOP_FUNC5` (PSEL 6) except the master chip select, which is a GPIO
the driver toggles. P108 and P109 are not used because the on-board debugger
shares them. The RSPI slave supports modes 1 and 3 only, so `board.h` sets
`SPI_TEST_MODE_MASK` to 0x0A and the shared example skips modes 0 and 2. The
master runs at 400 kHz and polls; set `SPI_MASTER_INT_ENABLE` to `true` to run
the interrupt transfers. This example is built from `spi_ra4m1.cpp` that has
only run against the host model, **Hardware not tested**. See "RSPI master and
slave" in `../README.md` for the limits and the assumptions to confirm.

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
The hardware mapping stays in the MCU implementation. Device 0 is the example default;
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

**I2CMasterSlave:** reuses the shared master/slave loopback program. It requests
IIC0 as polling master and IIC1 as interrupt slave at 100 kHz. The slave uses ICU RXI/TXI/TEI/EEI and the standard IOsonata
READ_RQST, WRITE_RQST and COMPLETED callbacks. The first ISR implementation
is not yet verified on the EK-RA4M1; no hardware PASS is claimed.

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
`UART_BAUDRATE` can be overridden. UartLoopback defaults to interrupt mode without DMA; UartPrbsTxTest defaults to polling with TX DMA. Both use blocking FIFOs, as required by the RA4M1 DMA path.

The supplied projects are configured for **newlib-nano + nosys**, not
semihosting. A locally modified application using `rdimon` and J-Link
semihosting can incur large CPU stalls when printing. No debugger console
is required to run the supplied applications. Ordinary timer/Blinky `printf` output is
not routed to SCI automatically; use the GPIOs/debugger variables or the existing
retarget facilities once deliberately configured. The UART loopback banner is
sent through its UART object, independently of stdio retargeting.

Outputs are `ioc/Debug/<IOC-project-name>.elf` and the corresponding Release
file, plus the managed build's flash image and size report. The linker retains
`ResetEntry` and `__Vectors` from the archive and uses the MCU's option-memory
layout; no alternative reset handler or bootloader layout is supplied.

## Hardware bring-up status (October 11, 2026)

On the EK-RA4M1, the user verified compilation and successful execution of
Blinky and TimerDemo after migrating the port to vendor CMSIS registers.
A Timer DevNo 3 (GPT1) run produced callbacks from four compare triggers at
100, 1000, 250 and 500 ms. The printed periods are not reliable jitter
measurements because the run used J-Link `rdimon` semihosting; acquire timing
samples in RAM or use a nonblocking logging path before measuring latency.
SCI UART is complete and hardware-tested. The new IIC master/slave loopback,
GPIO edge interrupts and the remaining untested timer channels still require
hardware validation.

The 32 peripheral NVIC slots are assigned dynamically through the RA4M1 ICU,
not fixed to a peripheral. See [INTERRUPTS.md](../INTERRUPTS.md) for the
routing model, callback ownership and registration restrictions.

## Validation

From the repository root:

```sh
python3 tests/ra4m1/run_example_validation.py
python3 tests/ra4m1/run_timer_validation.py
```

The first command checks the five IOC project definitions and runs 96 host builds/tests
of the shared timer/UART examples. It builds the same timer application with
all ten logical device numbers (0-9) by overriding only `TIMER_DEVNO`, and checks
C API and C++ API selections, O0/Os/O2, and fatal AddressSanitizer/UndefinedBehaviorSanitizer. It exercises init-failure
handling, one/four-trigger selection, trigger-failure cleanup, callback values,
513-byte echo preservation under short/zero writes, and 1024 PRBS bytes with
retries. It uses **simplified API wrappers and CMSIS definitions**. It does not test
the production C++ wrapper or execute code on the MCU. The timer API double exercises device
selection and callback handling, not hardware period limits; those are tested
separately in `timer_model.cpp`. Blinky and I2CMasterSlave are linked unchanged and receive project/pin-map
checks, not execution tests in this suite.

Use `--metadata-only` for project checks without Clang++, and
`--allow-partial-checkout` only when testing the standalone port package without
unmodified shared repository files. Missing source existence checks are then
reported explicitly. Firmware compilation and basic GPIO/timer execution were subsequently
confirmed by the user on an EK-RA4M1. IIC hardware behavior, oscillator accuracy, interrupt latency, and updated
CMSIS host-model regressions remain unverified. The driver regression limitations in `../TIMER.md` still apply.

## USB device examples

The native RA4M1 USBFS driver is used by all the projects below. They link the generic
applications from `exemples/usb` rather than copying implementation code. The
`src/board.h` configuration enables the 48 MHz HOCO USB clock and assigns the USB
VBUS input to P407. Use the EK-RA4M1 USB device connector.

| Project | Generic source |
| --- | --- |
| `UsbCdcLoopback` | `usb_cdc_loopback.cpp` |
| `UsbCdcIsoStress` | `usb_cdc_iso_stress.cpp` |
| `UsbCdcPrbsTx` | `usb_cdc_prbs_tx.cpp` |
| `UsbDualCdcStress` | `usb_dual_cdc_stress.cpp` |
| `UsbHidKeyboard` | `usb_hid_keyboard.cpp` |
| `UsbHidLoopback` | `usb_hid_loopback.cpp` |
| `UsbIntLoopback` | `usb_int_loopback.cpp` |
| `UsbIsoLoopback` | `usb_iso_loopback.cpp` |
| `UsbMscRamdisk` | `usb_msc_ramdisk.cpp` |
| `UsbCustomBulkLoopback` | `usb_custom_bulk_loopback.cpp` |

`UsbCdcPrbsTx` and `UsbDualCdcStress` also link `src/prbs.c`.
Hardware validation covers CDC PRBS, CDC loopback, concurrent CDC+ISO,
interrupt loopback, and MSC mounting/file read-write on macOS. CDC+ISO also
recovered after repeated unplug/replug without a board reset or crash.
See [USBFS validation](../README.md#usbfs-supply-and-validation) for measured
results and limits; untested examples are not included in those claims.

`UsbMscRamdisk` defines `MSC_SECTOR_COUNT=32U` in both build configurations
to allocate a 16 KiB disk within the MCU's 32 KiB RAM. The shared application
source stays the same across targets.

Use [`UsbCdcIsoStress`](UsbCdcIsoStress/README.md) for concurrent bulk and ISO
traffic. The full `UsbComboStress` composition exceeds RA4M1's physical pipe
capacity and has no RA4M1 project.
