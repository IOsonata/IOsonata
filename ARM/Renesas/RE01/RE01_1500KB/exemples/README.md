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
| I2CMasterDemo | `exemples/i2c/i2c_master_demo.cpp` | Polling transmit, receive and register reads at 100 kHz |
| SPIMasterDemo | `exemples/spi/spi_master_demo.cpp` | Polling transmit and command/read at 1 MHz, mode 3, 8-bit MSB first |

The existing DFU and scheduler/SLIP projects remain separate. The direct build
check below covers the seven applications in this table.

## Pin assignments and peer setup

The following are sample MCU pin assignments, not a board connector map. Match
each application's `board.h` to your wiring before flashing. The UART console, I2CMasterDemo and TimerDemo use SCI4 at 115200 baud.
SPIMasterDemo retains the shared example's 1000000-baud console. All use 8-N-1
without flow control or DMA. Connect the adapter's TX to RXD4, RX to TXD4 and
common ground. UartPrbsTxTest retains its existing, separate pin map.

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
and the driver holds it low across the command and receive phase. The shared example sends a 100-byte transaction containing values 0 through
99, then command 0 followed by 20 receive bytes. The peer must use SPI mode 3.
Adapt the command and protocol to your peer;
completed SPI clocks alone do not verify the peer's identity or response.

The I2C peer uses 7-bit address 0x22 and a one-byte register offset. The shared example transmits 11 bytes, receives nine bytes, reads five bytes
starting at offset 3 and then reads five bytes without a command prefix.
Adapt the address/offset to your peer in the application source. Fit external
SDA/SCL pull-ups to the correct I/O supply; the shared example also enables
internal pull-ups. All six RE01 I2CMasterDemo configurations define
`I2C_MASTER_DMA_ENABLE=false` and `I2C_MASTER_INT_ENABLE=false`, selecting the
existing example's synchronous path. SPIMasterDemo already selects polling.
The shared examples retarget printf to UART in Release configurations; inspect
transfer buffers and counts in the debugger for Debug configurations.

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

The original example set passed 90 compiler/linker/layout checks with GCC
14.3.Rel1 across all three packages. After switching to the shared bus examples,
I2CMasterDemo and SPIMasterDemo passed 24 checks covering Debug/Release defines,
all three packages and both normal linker scripts. XML checks verified both
source links and the polling flags in all six I2C configurations. Existing
shared atomic and I2C demo warnings remain; no hardware test was performed.
