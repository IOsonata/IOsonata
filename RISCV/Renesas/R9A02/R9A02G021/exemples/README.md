# R9A02G021 examples

Each example has an `ioc` project and a native Renesas e2 studio `e2` project.
Import the library and examples with **Existing Projects into Workspace**,
leaving **Copy projects into workspace** unchecked so relative source links
continue to resolve inside the repository.

| IDE | Library directory and project | Example directory and project |
| --- | --- | --- |
| IOcomposer, xPack GCC | `../lib/ioc`, `IOsonata_R9A02G021` | `<example>/ioc`, `<example>` |
| e2 studio, Renesas LLVM | `../lib/e2`, `IOsonata_R9A02G021_e2` | `<example>/e2`, `<example>_e2` |

Build the same library configuration (Debug or Release) before the example.
The e2 projects link only `lib/e2/Debug` or `lib/e2/Release`; GCC and LLVM
archives must not be mixed. Project references name the e2 library.

Both use RV32IMAC with Zba, Zbb, Zbs, Zicsr and Zifencei and the ilp32 ABI.
The ioc projects use GNU C17/C++23 and newlib-nano. The e2 projects use
LLVM for Renesas RISC-V 19.1.7.202501, GNU C17/C++17 and its prebuilt runtime.
Select that compiler in e2 studio Toolchain Management. They use
the native Renesas LLVM managed-build integration for R9A02G0214CNE on the
FPB-R9A02G021; no Smart Configurator generated startup is required.

Debug enables semihosting and leaves `NDEBUG` undefined. These RISC-V
toolchains call the library `semihost`, the equivalent of Arm `rdimon`:
GCC uses `--specs=semihost.specs`, and both toolchains link `libsemihost`
with `--whole-archive` / `--no-whole-archive` so its syscalls replace the
weak startup stubs. The e2 linker command places this library before the
application libraries to avoid extracting the UART syscall implementation.
The runtime constructor initializes the host streams. Run Debug with
debugger semihosting enabled; the examples enable UART console retargeting
only in Release.

Release uses nosys without semihosting and defines `NDEBUG` in the C and
C++ compiler settings of both the examples and the matching library.
Use Release for standalone operation and UART performance measurements.

The C, C++, assembler and linker command patterns explicitly place
`-march=rv32imac_zba_zbb_zbs_zicsr_zifencei -mabi=ilp32` after `${FLAGS}`.
This keeps Zicsr and Zifencei enabled when e2 studio supplies its default
device architecture without them. The setting applies to both library
configurations and all examples; `__riscv_zicsr` is supplied by the compiler.
The C, C++ and assembler patterns also include `-c` to produce object files;
linking is performed only by the separate linker step.
After updating an imported project, refresh it and clean/rebuild the library
and example. The build command must contain the explicit architecture above.

The linker-script column below lists the ioc scripts. The e2 projects use
`llvm_R9A02G021.ld`, or `llvm_dfu_boot_r9a02g021.ld` for DfuBoot. Their memory
maps and option-setting words match the ioc scripts. They use LLVM runtime
libraries and `llvm_riscv_flash.ld` sections accepted by LLD. The e2 library's
`start_llvm.c` runs C/C++ initializers and main after `ResetEntryInit` has
initialized RAM and the MCU. The executable names remain `<example>.elf`.
The linker Entry point field is empty: `llvm_riscv_flash.ld` already declares
`ENTRY(ResetEntry)`. A bare `ResetEntry` in that field is passed to clang as
an input filename and causes a "no such file or directory" error.

| Directory | Shared application source | Linker script | Setting |
| --- | --- | --- | --- |
| `Blinky` | `exemples/misc/blinky.c` | `gcc_R9A02G021.ld` | LED1, LED2, switch S1 on IRQ4 |
| `TimerDemo` | `exemples/timer/timer_demo.cpp` | `gcc_R9A02G021.ld` | Timer DevNo 0, default frequency |
| `UartLoopback` | `exemples/uart/uart_loopback.cpp` | `gcc_R9A02G021.ld` | UART0, 115200 8N1, interrupt mode |
| `UartPrbsTxTest` | `exemples/uart/uart_prbs_tx.cpp` | `gcc_R9A02G021.ld` | UART0, 115200 8N1, interrupt mode |
| `UartaLoopback` | `exemples/uart/uart_loopback.cpp` | `gcc_R9A02G021.ld` | UARTA1 (DevNo 4), 115200 8N1, interrupt mode, linker option `-Wl,-u,R9a02UartaInit` |
| `SpiLoopback` | `exemples/spi/spi_loopback.cpp` | `gcc_R9A02G021.ld` | SPI20 (DevNo 4) at 1 MHz, the four modes, interrupt and DTC reception, console on UART0 |
| `SpiMasterSlave` | `exemples/spi/spi_master_slave.cpp` | `gcc_R9A02G021.ld` | SPI20 master (DevNo 4) at 1 MHz wired to the SPI21 slave (DevNo 5), the four modes, DTC on both, console on UART0 |
| `I2CScan` | `exemples/i2c/i2c_scan.cpp` | `gcc_R9A02G021.ld` | IICA1 (DevNo 1) at 100 kHz, HOCO 32 MHz, console on UART0 |
| `I2CMasterSlave` | `exemples/i2c/i2c_master_slave.cpp` | `gcc_R9A02G021.ld` | IICA1 master (DevNo 1) at 100 kHz wired to the IICA0 slave (DevNo 0) at 0x22, HOCO 32 MHz, console on UART0 at 1 Mbaud |
| `ADCDemo` | `src/adc_demo.cpp` (this port) | `gcc_R9A02G021.ld` | ANI2, ANI3, internal reference, temperature sensor, console on UART0 |
| `RtcDemo` | `src/rtc_demo.cpp` (this port) | `gcc_R9A02G021.ld` | RTC on the 32.768 kHz crystal, Software Standby, console on UART0 |
| `AnalogDemo` | `src/analog_demo.cpp` (this port) | `gcc_R9A02G021.ld` | TML32 interval, DAC8 channel 1 ramp, comparator 0 on the internal reference, console on UART0 |
| `AdcSnoozeDemo` | `src/adc_snooze_demo.cpp` (this port) | `gcc_R9A02G021.ld` | ANI2 converted in Snooze every 100 ms, TML32, DTC, ELC, wake-up above 1.65 V, console on UART0 |
| `SelfTest` | `src/self_test.cpp` (this port) | `gcc_R9A02G021.ld` | Hardware self-test, 12 tests with PASS, FAIL or SKIP, console on UART0 |
| `DfuBoot` | `exemples/dfu/dfu_boot_main.cpp`, `dfu_boot_key.c` | `dfu_boot_r9a02g021.ld` | UART0, button P108 |

Application sources are linked resources, not copies. Each example's
`src/board.h` holds its pin and clock settings.

## FPB-R9A02G021 pins

From the FPB-R9A02G021 user's manual, default jumpers:

| Function | MCU pin | Note |
| --- | --- | --- |
| LED1 | P107 | Active high |
| LED2 | P100 | Active high |
| Switch S1 | P108, IRQ4 | Pull-up, falling edge |
| UART0 TX (TxD0_B) | P302 | J-Link OB virtual COM port, J5 and J6 at 2-3 |
| UART0 RX (RxD0_B) | P303 | J-Link OB virtual COM port |

UART pins use `IOPINOP_FUNC16` (PSEL 0x11, SAU). P300 and P301 are the
debug pins (TCKC, TMSC), keep them free.

The pins of the new examples are on the Arduino header and the Pmod
connectors, from the FPB-R9A02G021 quick start guide. The board user's manual
was not available here: check them against the board schematic.

| Example | Function | MCU pin | PinOp |
| --- | --- | --- | --- |
| UartaLoopback | RxDA1, TxDA1 | P008, P403 | `IOPINOP_FUNC5` (PSEL 0x06) |
| SpiLoopback | SCK20, SI20 (MISO), SO20 (MOSI) | P101, P102, P103 | `IOPINOP_FUNC16` |
| SpiLoopback | Chip select | P104 | GPIO |
| SpiMasterSlave | Master SCK20, SI20 (MISO), SO20 (MOSI) | P101, P102, P103 | `IOPINOP_FUNC16` |
| SpiMasterSlave | Master chip select | P109, J13-1 | GPIO output |
| SpiMasterSlave | Slave SCK21, SI21 (MOSI), SO21 (MISO) | P204, P205, P206 | `IOPINOP_FUNC16` |
| SpiMasterSlave | Slave chip select, IRQ3 | P201, J13-7 | GPIO input with pull-up |
| I2CScan | SDAA1, SCLA1 | P010, P011 | `IOPINOP_FUNC6` (PSEL 0x07), open drain |
| I2CMasterSlave | Master SDAA1, SCLA1 | P010, P011 | As I2CScan |
| I2CMasterSlave | Slave SDAA0_B, SCLA0_B | P103, P102 | `IOPINOP_FUNC6` (PSEL 0x07), open drain |
| ADCDemo | ANI2, ANI3 (A2, A3) | P400, P401 | `IOPINOP_FUNC31` (analog) |
| AnalogDemo | DACOUT1, IVCMP0, wired together | P001, P106 | Analog, set by the drivers |
| AdcSnoozeDemo | ANI2 (A2) | P400 | `IOPINOP_FUNC31` (analog) |
| SelfTest | DACOUT1, IVCMP0, wired together | P001, P106 | Analog, set by the drivers |
| SelfTest | ANI2 (A2) to GND, ANI3 (A3) to 3.3 V | P400, P401 | `IOPINOP_FUNC31` (analog) |
| SelfTest | SPI20, nothing connected | P101 to P104 | As SpiLoopback |

The library clock default is kept in every example: HOCO 48 MHz core clock,
LOCO low frequency clock. `board.h` shows how to select the 32.768 kHz crystal
fitted on the board with `MCUOSC`.

## What to observe

**Blinky:** the LEDs are on after start, each one goes off for 250 ms in turn.
Pressing S1 stops the sequence (`g_bBut1Pressed`). The shared example prints
from the button handler through semihosting in Debug. No UART console is
configured, so that output is dropped in Release.

**TimerDemo:** see `g_TimerInitOk`, `g_TriggerPeriod[]`, `g_Period[]` and
`g_TriggerCount[]` in the debugger. LED1 toggles on trigger 0. DevNo 0 (machine
timer, 32768 Hz) and DevNo 1 to 8 (TAU channels 0 to 7) each have one trigger,
so only the 100 ms period is used. For a TAU channel, set `TIMER_DEVNO` to 1 to
8 and `TIMER_FREQ` to a frequency that holds 100 ms in 16 bits, for example
46875 (PCLKB 48 MHz / 1024); the default frequency (PCLKB) gives at most
1.37 ms and the trigger is rejected.

**UartLoopback:** terminal at 115200 8N1, no flow control, on the J-Link OB
virtual COM port. The firmware prints nothing by itself, it echoes what it
receives.

**UartPrbsTxTest:** sends the `Prbs8` sequence at 115200 8N1, starting with
0xFF, no banner.

**UartaLoopback:** connect a 3.3 V serial adapter to P008 (RX) and P403
(TX), terminal at 115200 8N1. The firmware echoes what it receives. The
project links UARTA with `-Wl,-u,R9a02UartaInit` (Other linker flags).

**SpiLoopback:** connect P103 (MOSI) to P102 (MISO). The console on the
virtual COM port (Release configuration, `NDEBUG`) shows each mode and
pattern with PASS or FAIL, then `SPI master loopback PASS`. `board.h` sets
`SPI_MASTER_INT_ENABLE` and `SPI_MASTER_DMA_ENABLE`: the receptions run in
the background with the DTC. Set both false for polling, or only DMA for the
DTC with the call waiting.

**SpiMasterSlave:** all four jumpers are on Pmod 1 J13:

| Signal | Master pin | Slave pin | J13 jumper |
| --- | --- | --- | --- |
| SCK | P101 | P204 | 4 to 8 |
| MOSI | P103 | P205 | 2 to 9 |
| MISO | P102 | P206 | 3 to 10 |
| CS | P109 | P201, IRQ3 | 1 to 7 |

P109 and P201 connect directly to J13 and have board pull-ups. The previous
P104-to-P202 CS path failed the hardware GPIO check: P104 read high while
P202 stayed low, with SPI stopped and the P202 pull-up enabled. The SCK,
MOSI and MISO GPIO checks passed. With the new CS pair, the October 15, 2026
hardware capture passes every mode 0 transfer at 1 MHz, 8 bits, with
master and slave DMA=1 INT=1. Mode 1 fails on its first byte. The CPHA=1
initial buffer-empty handling is now corrected and passes emulator checks;
hardware validation of that correction and modes 1 to 3 remains pending.

The console shows, for each mode,
frames of 1, 16 and 129 bytes sent and received by the master, with PASS or
FAIL, then `SPI master/slave loopback PASS`. The slave takes its buffers at
`DEVINTRF_EVT_STATECHG` and reports each frame at the chip select rising
edge. `SPI_SLAVE_DMA_ENABLE false` in `board.h` tests the slave with one
interrupt per byte; modes 1 and 3 also need an initial buffer-empty
interrupt before the first byte completes. Its interrupt priority is above
the master's.

On a slave completion timeout, the R9A02 example prints the CS pin
configuration, GPIO input registers, SAU1 status and ICU/CLIC routes through
the shared example's `SpiMasterSlaveDiagnostics` hook. P201 should show
`pullup=1 IRQ=1 output=0 peripheral=0`; P109 should be a GPIO output.
The diagnostic source is linked by both projects and runs only after timeout.
After printing the registers, it powers off both SPI interfaces, tests each
connection as GPIO in its normal direction, and checks the CS rising-edge
interrupt. Each pair should report `low=0/0 high=1/1`; the interrupt check
should report one rising callback. It then releases the pins. The original
SPI failure remains a failure; these checks isolate GPIO and IRQ operation
from the SAU configuration. The write-only PRWCNTR is not read.

**I2CScan:** connect I2C devices with pull-ups to P010 (SDA) and P011 (SCL).
The console lists the addresses that answer. The core clock is the HOCO at
32 MHz, the IICA needs PCLKB at 32 MHz or lower. `I2C_SCAN_INT_ENABLE true`
reads the register of `I2C_SCAN_REG_DEVADDR` in the interrupt. `board.h`
gives the pin map for the SAU simplified I2C IIC20 (DevNo 6, P101 SCL, P102
SDA).

**I2CMasterSlave:** wire P010 to P103 (SDA) and P011 to P102 (SCL), with a
pull-up from each line to 3.3 V (4.7 kOhm for example); the internal pull-ups
are off once the IICA pins are in output mode. The console, Release
configuration (`NDEBUG`) at 1,000,000 baud, shows the init result of both
I2C, the write of 8 bytes at offset 4 of the slave memory, their read back
with a restart, then a read of 4 bytes without command that continues at
offset 12 (`0c 0d 0e 0f`), and `I2C master/slave loopback PASS`. Both DMA
switches default to true in the shared example. The master waits for
completion by default; `I2C_MASTER_INT_ENABLE true` enables background
completion. The slave requires interrupts. DTC moves payload bytes while
the CPU continues to handle the IICA protocol at each byte.

Hardware PASS on October 14, 2026: master DMA=0 INT=0 with slave DMA=0
INT=1, and master DMA=1 with either INT=0 or INT=1 with slave DMA=1 INT=1.
Each run passed the 8-byte write/read and the final 4-byte receive.

**AnalogDemo:** wire P001 to P106. The TML32 (LOCO) steps DAC8 channel 1 every
20 ms, a ramp from 0 to VCC in about 2.6 s. Comparator 0 compares it with its
internal reference: the console shows `Above at code 112, 1443 mV` or near on
the way up, `Below at code 0` when the ramp starts again, LED2 follows the
comparator output and LED1 toggles at each event. The internal reference is
1.34 to 1.54 V.

**AdcSnoozeDemo:** apply a voltage to A2 (P400). The MCU stays in Software
Standby; every 100 ms the TML32 starts Snooze, the conversion of A2 runs
without the CPU. At 1.65 V or more the CPU wakes up, prints the voltage,
toggles LED1 and waits one second before the next Software Standby. Below
1.65 V nothing is printed and the current stays low. The debugger connection
may be lost in Software Standby.

**SelfTest:** wire P001 to P106, A2 to GND and A3 to 3.3 V, terminal on the
virtual COM port. The firmware runs 12 tests and prints the values measured
and PASS, FAIL or SKIP for each, then the counts; LED1 stays on without a
failure, LED2 lights on a failure. The tests settle what the manual leaves
open: the ICU flag with the CLIC input off (polling drivers), the TML32
interval formula and 32-bit mode, the DTC with the CLIC, the end of an SPI
reception by the DTC, DAC8 and A/D on shared pins, the A/D scan mode, ANI18
and ANI19, comparator 0 references, the A/D compare window at ADLL. Test 12
enters Software Standby and asks to touch P106 to GND within 15 s (the DAC8
output has 9 kOhm, a short is safe), then whether it was done;
`SELFTEST_CMP_WAKE false` in `board.h` leaves it out. A SKIP names the
missing wire.

**ADCDemo:** every second the console shows the voltage of A2 and A3, the
internal reference with the VCC it gives, and the die temperature from the
factory calibration point. LED1 toggles. `ADC_DEMO_INT_ENABLE` in `board.h`
selects the interrupt mode.

**RtcDemo:** the 32.768 kHz crystal is the low frequency clock. The RTC
wakes the MCU from Software Standby every second: the console shows the date
and time (the build time after a power-on reset) and LED1 toggles. An alarm
every 2 minutes toggles LED2. `RTC_DEMO_STANDBY false` in `board.h` uses
Sleep instead. The debugger connection may be lost in Software Standby.

**DfuBoot:** stage 0 boot for the layout of `ldscript/dfu_layout_r9a02g021.ld`
(boot 36 KB, record 4 KB, application slot from 0xA000). Build the Release
configuration for deployment; the current LLVM size limitation is recorded
below. The Debug configuration defines `DFU_BOOT_ALLOW_UNSIGNED`.
An application for the slot links with `gcc_R9A02G021_dfu.ld`.

## Status

On October 14, 2026, all fifteen current examples compiled from the command
line with LLVM for Renesas RISC-V 19.1.7.202501 in Debug (`-Og`) and Release
(`-Os`). Fourteen linked in each configuration. DfuBoot exceeded its 36 KB
boot region by 12112 bytes in Debug and 1836 bytes in Release. Its Release
main function's generated code is identical before and after the declaration
cleanup; resolving the boot-image size requires separate work.

These checks compiled the 61 library source files needed by the examples,
including the LLVM runtime entry, with the projects' defines, include paths
and target settings. All fourteen linked Debug images contain the
semihosting initializer; none of the Release images does. Compiler AST
checks confirmed that active local declarations are grouped at function
entry in both configurations. The project XML and configuration settings
were also checked.

The xPack GCC builds, IDE import and managed builds, remaining library
modules and on-board execution were not repeated for these changes.

