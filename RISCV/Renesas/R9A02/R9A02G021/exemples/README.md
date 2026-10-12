# R9A02G021 examples

Import the library project `../lib/ioc` (`IOsonata_R9A02G021`), then the
example's `ioc` directory with **Existing Projects into Workspace**. Build the
library configuration (Debug or Release) before the example: the example
links that configuration's archive. All projects use RV32IMAC with Zba, Zbb,
Zbs, Zicsr and Zifencei, ilp32 ABI, GNU C17 and C++23, newlib-nano with nosys.

| Directory | Shared application source | Linker script | Setting |
| --- | --- | --- | --- |
| `Blinky` | `exemples/misc/blinky.c` | `gcc_R9A02G021.ld` | LED1, LED2, switch S1 on IRQ4 |
| `TimerDemo` | `exemples/timer/timer_demo.cpp` | `gcc_R9A02G021.ld` | Timer DevNo 0, default frequency |
| `UartLoopback` | `exemples/uart/uart_loopback.cpp` | `gcc_R9A02G021.ld` | UART0, 115200 8N1, interrupt mode |
| `UartPrbsTxTest` | `exemples/uart/uart_prbs_tx.cpp` | `gcc_R9A02G021.ld` | UART0, 115200 8N1, interrupt mode |
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

The library clock default is kept in every example: HOCO 48 MHz core clock,
LOCO low frequency clock. `board.h` shows how to select the 32.768 kHz crystal
fitted on the board with `MCUOSC`.

## What to observe

**Blinky:** the LEDs are on after start, each one goes off for 250 ms in turn.
Pressing S1 stops the sequence (`g_bBut1Pressed`). The shared example prints
from the button handler; no console is configured, the output is dropped.

**TimerDemo:** see `g_TimerInitOk`, `g_TriggerPeriod[]`, `g_Period[]` and
`g_TriggerCount[]` in the debugger. LED1 toggles on trigger 0. DevNo 0 (machine
timer, 32768 Hz) and DevNo 1 to 8 (TAU channels 0 to 7) each have one trigger,
so only the 100 ms period is used. For a TAU channel, set `TIMER_DEVNO` to 1 to
8 and `TIMER_FREQ` to a frequency that holds 100 ms in 16 bits, for example
46875 (PCLKB 48 MHz / 1024); the default frequency (PCLKB) gives at most
1.37 ms and the trigger is rejected.

**UartLoopback:** terminal at 115200 8N1, no flow control, on the J-Link OB
virtual COM port. The firmware sends `UART Loopback Test` and echoes what it
receives.

**UartPrbsTxTest:** sends the `Prbs8` sequence at 115200 8N1, starting with
0xFF, no banner.

**DfuBoot:** stage 0 boot for the layout of `ldscript/dfu_layout_r9a02g021.ld`
(boot 36 KB, record 4 KB, application slot from 0xA000). Build the Release
configuration: at -O0 (Debug) the boot does not fit in 36 KB. The Debug
configuration defines `DFU_BOOT_ALLOW_UNSIGNED`. An application for the slot
links with `gcc_R9A02G021_dfu.ld`.

## Status

All five examples build and link with xPack riscv-none-elf-gcc 15.2.0
(October 12, 2026). Release sizes (text + data): Blinky 11.6 KB, TimerDemo
19.6 KB, UartLoopback 14.2 KB, UartPrbsTxTest 14.2 KB, DfuBoot 35.1 KB.
None has been run on the board yet.
