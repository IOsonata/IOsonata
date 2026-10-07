# RE01 1500KB examples

Open the `ioc` projects in IOcomposer/Eclipse using **File > Open Projects
from File System** and search for nested projects. Application sources are
linked from the repository's shared `exemples/` directory. New projects keep
their pin assignments in `include/board.h`; TimerDemo retains `src/board.h`.

I2CMasterDemo, SPIMasterDemo, UartRetargetDemo, PulseTrain and TimerDemo provide
Debug/Release configurations for CFB, CFP and DBN. Build the corresponding
configuration of `RE01_1500KB/lib/ioc` first, then the application, for example
`DebugCFB` in both projects. These projects use the normal flash linker script,
newlib-nano and nosys, with no semihosting requirement.

| Project | Shared application source | Behavior |
| --- | --- | --- |
| Blinky | `exemples/misc/blinky.c` | Existing LED example |
| PulseTrain | `exemples/misc/pulse_train_test.c` | GPIO transitions in sequence, spaced 500 us |
| TimerDemo | `exemples/timer/timer_demo.cpp` | Selectable timer, all available compare callbacks and UART period reports |
| UartRetargetDemo | `exemples/uart/uart_retarget_demo.cpp` | UART console using printf/scanf, static RX/TX FIFOs and interrupts |
| UartPrbsTxTest | `exemples/uart/uart_prbs_tx.cpp` | Existing UART PRBS transmission benchmark |
| I2CMasterDemo | `exemples/i2c/i2c_polling_master_demo.cpp` | Synchronous command/read with repeated START at 100 kHz |
| SPIMasterDemo | `exemples/spi/spi_polling_master_demo.cpp` | Synchronous command/read and transmit at 1 MHz, mode 0, 8-bit MSB first |

The existing DFU and scheduler/SLIP projects remain separate. The direct build
check below covers the seven applications in this table.

## Pin assignments and peer setup

The following are sample MCU pin assignments, not a board connector map. Match
each application's `board.h` to your wiring before flashing. The new UART
console/bus examples and TimerDemo use SCI4 at 115200 baud, 8-N-1, without flow
control or DMA. Connect the adapter's TX to RXD4, RX to TXD4 and common ground.
UartPrbsTxTest retains its existing, separate pin map.

| Signal | MCU pin | PinOp | Use |
| --- | --- | --- | --- |
| UART RX | P202 | `IOPINOP_FUNC3` (PSEL 4) | RXD4_B |
| UART TX | P203 | `IOPINOP_FUNC3` (PSEL 4) | TXD4_B |
| I2C SDA | P809 | `IOPINOP_FUNC6` (PSEL 7) | SDA0 |
| I2C SCL | P810 | `IOPINOP_FUNC6` (PSEL 7) | SCL0 |
| SPI SCK | P011 | `IOPINOP_FUNC5` (PSEL 6) | RSPCKA_B |
| SPI MISO | P500 | `IOPINOP_FUNC5` (PSEL 6) | MISOA_B |
| SPI MOSI | P010 | `IOPINOP_FUNC5` (PSEL 6) | MOSIA_B |
| SPI CS | P012 | `IOPINOP_GPIO` | Active-low software chip select 0 |
| PulseTrain / Timer LEDs | P009, P008, P007 | `IOPINOP_GPIO` | Three application outputs |

SPI0 uses the same group B for all peripheral signals. The CS pin is a GPIO,
and the driver holds it low across the command and receive phase. The sample
peer must accept command 0 followed by 16 receive bytes, then a separate
16-byte transmit transaction. Adapt the command and protocol to your peer;
completed SPI clocks alone do not verify the peer's identity or response.

The I2C peer uses 7-bit address 0x22 and a one-byte register offset. The demo
reads five bytes starting at offset 3 and reports a short transfer on NACK or
timeout. Adapt the address/offset to your peer in the application source.
Fit external SDA/SCL pull-ups to the correct I/O supply; the map uses open-drain
pins with internal pull-ups disabled. Both polling examples check initialization
and returned byte counts. Their UART diagnostics run in the main loop.

Pin names and mux values follow Renesas's
[pin configuration source](https://github.com/renesas/re-driver-package/blob/d67d8f1410421e33923e65a776add5401fbb11b8/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/pin.c)
and the package pin tables in the RE01
[datasheet](https://docs.rs-online.com/5a08/A700000007228313.pdf),
R01DS0363EJ0110, section 1.7. Board wiring and electrical behavior have not been
validated on hardware.

## Timer selection

TimerDemo defaults to device 3 (GPT0). Change `TIMER_DEMO_DEVNO=3` in the
selected project's C++ compiler defined symbols to select another device.
`TIMER_DEMO_FREQ=32768` requests a divided clock that allows the example's
periods to fit every 16-bit counter. The actual rate depends on the family
and running peripheral clock. `TIMER_DEMO_UART` enables console reporting.

| Device | Hardware | Compare callbacks used |
| --- | --- | --- |
| 0 | AGT0 | A/B |
| 1 | AGT1 | A |
| 2 | Cascaded TMR0/1 | A/B |
| 3, 4 | GPT0, GPT1 | A/B/C/D |
| 5, 6, 7, 8 | GPT2, GPT3, GPT4, GPT5 | A/B/C/D |

The requested periods are A=100 ms, B=1000 ms, C=250 ms and D=500 ms.
The demo queries the selected timer's trigger count, checks each returned
period, and reports measured periods in microseconds. The first sample includes
setup time; use subsequent measurements. GPIOs toggle for the first three
triggers; D is visible through UART and `g_TriggerCount[3]` in the debugger.
`g_Period[4]` records overflow intervals. Callbacks only update measurements and
GPIOs; formatting is deferred to the main loop. AGT B is slower than A to follow
the port's comparator hardware caution. The timer port's remaining limits and
register-level validation are documented in `tests/re01/README.md`.

## Direct build check

From the repository root:

```sh
python3 tests/re01/build_examples.py --package CFB --all-timers
```

Use `--tool-prefix /path/to/bin/arm-none-eabi-` when needed. `--package CFP`
and `--package DBN` select the other packages; `--timer-devno 8` selects one
timer instead of `--all-timers`. The script resolves the actual project source
and board links, compiles with GNU C++23, links both normal scripts and checks
vector, option-memory, code/data and RAM layout. This is a direct compiler/linker
check, not an Eclipse build or a hardware timing test.

Validated with GCC 14.3.Rel1 for all three packages: 90 images passed the
compiler, linker and memory-layout checks. Project XML checks verified the
six configurations, package defines, source/include links and linker settings.
