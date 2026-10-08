# SAM4LS C-package port

SAM4LS uses the shared SAM4L startup, GPIO, UART, AST/TC timers, I2C, SPI
and USB device drivers. The IOcomposer library and examples are available
for users to build and try. **SAM4LS is not hardware validated.** The
SAM4LC8C hardware results do not count as tests on SAM4LS.

The supplied library, wizard and example projects select SAM4LS8C.
SAM4LS2C and SAM4LS4C use the same sources with the matching device define
and linker script.

| MCU | C and C++ define | Application linker script |
| --- | --- | --- |
| SAM4LS2C | `__SAM4LS2C__` | `gcc_sam4lx2.ld` |
| SAM4LS4C | `__SAM4LS4C__` | `gcc_sam4lx4.ld` |
| SAM4LS8C | `__SAM4LS8C__` | `gcc_sam4lx8.ld` |

Scripts are in `ARM/Microchip/SAM4L/ldscript`. When changing MCU, replace
the device define in both C and C++ settings of the library and application,
select the application linker script, then clean and rebuild both projects.
Do not link a library built for a different flash-size selection.

## Build and try

1. Import `lib/ioc` into IOcomposer and build Debug or Release.
2. Import `exemples/Blinky/ioc` or `exemples/TimerDemo/ioc` and select the
   same configuration. These projects link the existing shared example
   sources; they do not contain another copy of the demo code.
3. Edit the application's `src/board.h` for your wiring before flashing.
   The example LED uses PC07, active low, with an external series resistor.
   TimerDemo uses USART1: PC26 RX and PC27 TX, peripheral A.
   Connect a 3.3 V UART adapter with common ground and open 115200 8N1.
   These pin choices are example wiring, not a claim about a SAM4LS board.
4. The shared startup defaults to internal oscillators. Define `MCUOSC`
   in the application board file if your board uses external clocks.
5. Blinky toggles the LED. TimerDemo prints trigger periods and elapsed time.
   Select `TIMER_DEVNO` 0 for AST or 1 through 6 for TC channels.

TimerDemo also exercises UART output. The SAM4L
[timer notes](../../../../tests/sam4l/README.md) describe frequency selection,
trigger limits and debugger variables. USB applications need their own clock
and USB pin configuration; see the [USB guide](../../../../docs/usb-user-guide.md#sam4l).

The older DfuBoot and Studio projects are retained. They are not covered by
these build checks. DFU hardware testing remains deferred.

## Build checks

From the repository root:

```sh
python3 tests/sam4l/timer_build_test.py --mcu SAM4LS2C
python3 tests/sam4l/timer_build_test.py --mcu SAM4LS4C
python3 tests/sam4l/timer_build_test.py --mcu SAM4LS8C
```

Arm GNU 14.3.1 compiled and linked Blinky and all seven TimerDemo device
selections in Debug and Release for each MCU: 48 application links.
The checks also compile the shared I2C, SPI and USB controller sources for
each MCU and configuration. The existing RWX LOAD-segment linker warning
remains.

These checks use the target headers, startup, vectors and linker scripts.
They do not run IOcomposer, build every optional library source or test
hardware. SAM4LS hardware testing is still needed.
