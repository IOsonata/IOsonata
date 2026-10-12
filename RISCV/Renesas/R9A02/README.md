# R9A02G021 port

IOsonata port for the Renesas R9A02G021 RISC-V MCU, first board FPB-R9A02G021
(R9A02G0214CNE, 48 pins). References: R9A02G021 User's Manual: Hardware
R01UH1036EJ0110 and datasheet R01DS0422EJ0110. Register definitions are
IOsonata's own, in `include/r9a02g021.h`; no Renesas SDK file is used.

The core is Renesas' RV32IMAC with Zba, Zbb, Zbs, Zicsr and Zifencei,
machine mode only, with a CLIC and a 64-bit machine timer. The peripherals
are not the RA2 set: there is no SCI, AGT or GPT. UART is the SAU, timers are
the TAU and the machine timer.

## Status (October 12, 2026)

| Function | Files | State |
| --- | --- | --- |
| Startup, option-setting memory, linker layout | `RISCV/src/ResetEntry.c`, `ldscript/*.ld` | Built and linked |
| Clocks, module stop, delay | `R9A02G021/lib/src/system_r9a02.c`, `include/system_r9a02.h` | Built |
| Trap, CLIC vector table, ICU event routing | `R9A02G021/lib/src/Vectors_R9A02.c`, `src/interrupt_r9a02.cpp`, `include/interrupt_r9a02.h` | Built |
| GPIO, pin functions, IRQ0 to IRQ7 pin interrupts | `src/iopincfg_r9a02.c`, `include/iopinctrl.h` | Built |
| UART (SAU UART0 to UART2), polling and interrupt | `src/uart_r9a02.cpp` | Built |
| Timers (machine timer, TAU channels 0 to 7) | `src/timer_r9a02.cpp`, `include/timer_r9a02.h` | Built |
| DFU flash target | `src/dfu_r9a02.cpp` | Built, boot linked |

Built with xPack riscv-none-elf-gcc 15.2.0, `-march=rv32imac_zba_zbb_zbs_zicsr_zifencei -mabi=ilp32`.
Nothing has been run on hardware yet. Not implemented: UARTA, IICA, SAU SPI
and I2C, TML32, RTC, ADC, DAC, DTC, low power modes.

## Memory layout

`ldscript/gcc_R9A02G021.ld`, every R9A02G021 part:

| Region | Address | Size | Use |
| --- | --- | --- | --- |
| HEAD | 0x00000000 | 0x810 | Reset entry, CLIC vector table, option-setting memory, trap entries |
| FLASH | 0x00000810 | 128 KB - 0x810 | Code and constants |
| DATA_FLASH | 0x40100000 | 4 KB | Not used by the library |
| RAM | 0x20004000 | 12 KB | SRAM0 (parity): data, heap, stack |
| IRAM | 0x20000000 | 4 KB | ECC SRAM: `.iram.text`, code run from RAM (DFU flash routines) |

`ldscript/r9a02g021_head.ld` builds the first 0x810 bytes. The CPU starts at
address 0 (manual 2.7.2.2). The option-setting memory is inside the code
flash: OFS0 at 0x400, OFS1 at 0x404, flash read protection 0x408 to 0x43B
(always 0xFFFFFFFF, the debugger needs it), OSIS 0x800 to 0x80F. The reset
entry and the vector table go below 0x400, the trap entries between 0x43C and
0x800. Defaults:

| Symbol | Default | Meaning |
| --- | --- | --- |
| `R9A02_OFS0` | 0xFFFFFFFF | IWDT and WDT stopped after reset |
| `R9A02_OFS1` | 0xFFFFCFFF | HOCO 48 MHz, HOCO off after reset, voltage monitor 0 reset off, clock supply type A |
| `R9A02_OSIS0..3` | 0xFFFFFFFF | ID code protection off |

Override one on the link command line, for example
`-Wl,--defsym=R9A02_OFS1=0xFFFFCEFF` to start the HOCO at reset.

General purpose registers are undefined at reset (manual 2.7.2.1), so
`ResetEntry` is a naked function: it sets gp and sp before any C code runs,
then jumps to `ResetEntryInit` (`.iram.text`, `.data`, `.bss`, `SystemInit`,
`SystemCoreClockUpdate`, `_start`).

## Clocks

The core runs from MOCO / 16 (500 kHz) at reset. `SystemInit` applies
`g_McuOsc` (weak, override it or define `MCUOSC` in the application). Library
default: HOCO 48 MHz core clock, LOCO low frequency clock.

| `OscCoreClk` | Core clock |
| --- | --- |
| `OSC_TYPE_RC`, 24000000, 32000000 or 48000000 | HOCO. 48 MHz requires clock supply type A (OFS1.ICSATS = 1, the default) |
| `OSC_TYPE_RC`, 8000000 | MOCO |
| `OSC_TYPE_TCXO`, 1 to 20 MHz | External clock on EXTAL (P300). P300 is also TCKC, the debug clock |
| `OSC_TYPE_XTAL` | Rejected, the part has no main crystal oscillator |

| `OscLowFreq` | Low frequency clock |
| --- | --- |
| `OSC_TYPE_RC`, 32768 | LOCO |
| `OSC_TYPE_XTAL`, 32768 | SOSC crystal (fitted on the FPB board) |

The operating mode follows the clock: high-speed above 24 MHz, middle-speed
otherwise. Flash wait states are set above 32 MHz. An unsupported setting
stops startup: `g_R9a02StartupError` holds the reason (`R9a02StartupError_t`)
and the CPU waits in a `wfi` loop, where a debugger can read it.

PCLKB, the clock of SAU and TAU, defaults to ICLK.
`SystemPeriphClockSet(0, Freq)` picks the closest PCLKB not above ICLK.

## Interrupts

Two layers, like the RA parts, but the CPU side is a RISC-V CLIC, not an NVIC.

- ICU: 32 event link registers IELSR0 to 31. Each peripheral event can be
  routed only to the IELSR slots of its group (slot n is group n % 8,
  manual table 12.7).
- CLIC: 51 interrupt IDs. 3 is the software interrupt (MSIP), 7 the machine
  timer (MTIP), 19 to 50 are IELSR0 to 31. All are rising edge and vectored
  through `mtvt` (`g_R9a02VectorTable`, 64-byte aligned).

`R9a02RegisterIntHandler(EvtId, Prio, Handler, Ctx)` picks the lowest free
IELSR slot in the event's groups, routes the event, enables the CLIC
interrupt and returns the slot number (or -1). Drivers keep the slot number
for `R9a02UnregisterIntHandler` and `R9a02ClearPendingInt`. The ICU IR flag
is cleared before the handler, except for a low-level pin IRQ, cleared after.
`R9a02RegisterCoreIntHandler` does the same for MSIP and MTIP.

Priority `Prio` is 0 to 15 (`cliccfg.nlbits` = 4), higher is more urgent, as
the RISC-V section of `coredev/interrupt.h` defines (`IRQ_PRIO_LOWEST` 1 to
`IRQ_PRIO_CRITICAL` 7). Handlers run with MIE set again, so a higher level
preempts a lower one. Exceptions go to the weak `R9a02ExceptionHandler`
(default: records mcause, mepc and mtval, then loops), NMI to the weak
`R9a02NmiHandler`.

## GPIO

`IOPinConfig` `PinOp` is the PSEL code of manual 16.6: `IOPINOP_GPIO`,
`IOPINOP_FUNC16` for the SAU (PSEL 0x11), and so on. `IOPINOP_FUNC31`
selects the analog function. There is no pull-down. SAU input pins get
NCODR = 1 as the manual requires.

`IOPinEnableInterrupt(IntNo, ...)`: `IntNo` is the IRQ line, 0 to 7, and the
pin must be one of that line's pins (P108 is IRQ4 for example).
`IOPinAllocateInterrupt` finds the line from the pin.

The pin masks follow the package. `R9A02_PACKAGE_PINS` defaults to 48;
define 32, 24 or 16 for the smaller parts.

## UART

| DevNo | Channels | SAU clock |
| --- | --- | --- |
| 0 | UART0, SAU0 channels 0 (TX) and 1 (RX) | CK00 |
| 1 | UART1, SAU0 channels 2 and 3 | CK01 |
| 2 | UART2, SAU1 channels 0 and 1 | CK10 |

Rate: fMCK / (2 x (STCLK + 1)), fMCK = PCLKB / 2^n, at most 32 MHz.
`UARTSetRate` returns the closest rate. UART0 and UART1 are on the same unit:
the prescaler register SPS0 is not written while a channel of the unit runs,
the rate of the second one opened is then set with STCLK only. Data 7 or 8 bits, parity none, even, odd or space (mark is
rejected), 1 or 2 stop bits, no flow control, no DMA. Interrupt mode uses
the IOsonata CFIFO, the TX interrupt runs one level below `IntPrio` so that
the RX interrupt can preempt it.

UARTA is not implemented.

## Timers

`TimerCfg_t.DevNo`, from low frequency, low power to high frequency:

| DevNo | Timer | Count | Triggers |
| --- | --- | --- | --- |
| 0 | Machine timer, LOCO 32768 Hz | 64 bits | 1 (mtimecmp) |
| 1 to 8 | TAU channels 0 to 7, PCLKB / 2^n | 16 bits, extended to 64 by the interval interrupt | 1 (interval period) |

`TimerGetLowFreqDevCount` is 1, `TimerGetHighFreqDevCount` 8,
`TimerGetHighFreqDevNo` 1. DevNo 0 counts MTCLK, which is always the LOCO
(manual 8.5.9), also when the 32768 Hz crystal is the low frequency clock.
The LOCO runs from reset and the library does not stop it. TAU channels share two prescalers (CK00, CK01): a channel
gets the closest frequency of a free prescaler, or of one a running channel
already set. Freq 0 selects PCLKB, where a 16-bit period is at most 1.37 ms
at 48 MHz; ask for a lower frequency for long periods (46875 Hz for up to
1.39 s). TML32 is not a timer device: it has no readable count in 32-bit
mode (manual 19.2).

## DFU

`ldscript/dfu_layout_r9a02g021.ld`: boot 0x0 to 0x8FFF, record 0x9000 to
0x9FFF, application slot 0 0xA000 to 0x1FFFF, recovery flag at 0x20006FF0.
`dfu_boot_r9a02g021.ld` links the boot (with HEAD), `gcc_R9A02G021_dfu.ld`
an application for slot 0. The boot fits only when built with -Os (35.1 KB
of 36 KB); the Debug configuration at -O0 does not fit.

`dfu_r9a02.cpp` programs the code flash in 8-byte units and erases 2 KB
blocks from routines in `.iram.text`, and resets through SWRCR.

## Examples

See [R9A02G021/exemples/README.md](R9A02G021/exemples/README.md).
