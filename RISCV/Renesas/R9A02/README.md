# R9A02G021 port

IOsonata port for the Renesas R9A02G021 RISC-V MCU, first board FPB-R9A02G021
(R9A02G0214CNE, 48 pins). References: R9A02G021 User's Manual: Hardware
R01UH1036EJ0110 and datasheet R01DS0422EJ0110. Register definitions are
IOsonata's own, in `include/r9a02g021.h`; no Renesas SDK file is used.

The core is Renesas' RV32IMAC with Zba, Zbb, Zbs, Zicsr and Zifencei,
machine mode only, with a CLIC and a 64-bit machine timer. The peripherals
are not the RA2 set: there is no SCI, AGT or GPT. UART is the SAU, timers are
the TAU and the machine timer.

## Status (October 14, 2026)

| Function | Files | State |
| --- | --- | --- |
| Startup, option-setting memory, linker layout | `RISCV/src/ResetEntry.c`, `ldscript/*.ld` | Built and linked |
| Clocks, module stop, delay | `R9A02G021/lib/src/system_r9a02.c`, `include/system_r9a02.h` | Built |
| Trap, CLIC vector table, ICU event routing | `R9A02G021/lib/src/Vectors_R9A02.c`, `src/interrupt_r9a02.cpp`, `include/interrupt_r9a02.h` | Built |
| GPIO, pin functions, IRQ0 to IRQ7 pin interrupts | `src/iopincfg_r9a02.c`, `include/iopinctrl.h` | Built |
| SAU channel and prescaler sharing | `src/sau_r9a02.cpp`, `include/sau_r9a02.h` | Built |
| UART (SAU UART0 to UART2), polling, interrupt and DTC TX, reception in Snooze (UART0, UART2) | `src/uart_r9a02.cpp` | UART0 Release PRBS TX at line rate and clean loopback in all four modes across recorded runs; earlier isolated PRBS losses unresolved; Snooze emulated (`tests/r9a02`) |
| UART (UARTA0, UARTA1), polling, interrupt and DTC TX | `src/uarta_r9a02.cpp`, `include/uarta_r9a02.h` | Built, DTC emulated (`tests/r9a02`) |
| SPI (SAU simplified SPI): master with polling, interrupt and DTC reception, slave with interrupt and DTC, slave reception in Snooze (SPI00, SPI20) | `src/spi_r9a02.cpp` | Hardware loopback passed in modes 0-3 at 1 MHz, 8 bits: DMA on both ends, or DMA off with polling/interrupt master and interrupt slave; Snooze emulated |
| I2C master (IICA0, IICA1), polling and interrupt; slave with interrupt; DTC payload transfers in both | `src/i2c_r9a02.cpp`, `include/i2c_r9a02.h` | Hardware loopback PASS without DMA, and with DMA on both ends for polling and interrupt master completion; emulated (`tests/r9a02`) |
| I2C master (SAU simplified I2C IIC00 to IIC21), polling and interrupt | `src/i2c_sau_r9a02.cpp` | Built |
| ADC (ADC12, 10 pins, temperature sensor, internal reference, scan, continuous, hardware trigger, window, Snooze) | `src/adc_r9a02.cpp`, `include/adc_r9a02.h` | Built; polling and interrupt single-round conversion hardware validated on ANI2/ANI3, VREF and temperature; AdcSnoozeDemo low/high input behavior validated |
| RTC (calendar, fixed-cycle interrupt, alarm) | `src/rtc_r9a02.cpp`, `include/rtc_r9a02.h` | Built; SOSC calendar, periodic interrupt, Sleep and Software Standby wake, periodic interrupts and repeated alarms hardware validated |
| DTC (data transfer controller) | `src/dtc_r9a02.cpp`, `include/dtc_r9a02.h` | Built |
| ELC (event link controller) | `src/elc_r9a02.cpp`, `include/elc_r9a02.h` | Built |
| TML32 (32-bit interval timer) | `src/tml32_r9a02.cpp`, `include/tml32_r9a02.h` | Built; 19989 us DAC ramp scheduling hardware validated |
| DAC8 (2 channels) | `src/dac_r9a02.cpp`, `include/dac_r9a02.h` | Built; DAC1 ramp and DAC0 comparator reference hardware validated on R23-loaded board |
| Comparators (CMP0, CMP1) | `src/analog_comp_r9a02.cpp` | Built; CMP0 both-edge interrupts hardware validated with IVREF0 |
| Sleep, Software Standby, wake-up sources, Snooze | `R9A02G021/lib/src/system_r9a02.c`, `include/system_r9a02.h` | Built; RTC Software Standby wake and AdcSnoozeDemo threshold wake hardware validated |
| Timers (machine timer, TAU channels 0 to 7) | `src/timer_r9a02.cpp`, `include/timer_r9a02.h` | Built; machine timer, TAU0 and TAU7 100 ms continuous callbacks hardware validated |
| DFU flash target | `src/dfu_r9a02.cpp` | Built, boot linked, boot emulated |

Built with xPack riscv-none-elf-gcc 15.2.0, `-march=rv32imac_zba_zbb_zbs_zicsr_zifencei -mabi=ilp32`.
The register use of the drivers was checked against the manual. UART0 Release
TX has reached line rate on hardware in all four interrupt/DTC configurations,
across the revisions recorded below. The latest byte-IRQ capture is clean.
The DTC captures reported isolated three-byte and four-byte losses during
their runs; their cause remains unresolved (see UART).
All four loopback configurations now have clean hardware captures. The
polling/DTC rerun after `7b28c9c` reports about 66,000 bytes/s with zero
errors; the sustained loss seen before the RX-service fix is absent.
Not implemented: SPI master transmission by the DTC, the DTC for SAU simplified I2C, UART reception by the DTC,
TML32 8-bit channels and capture, REMC, KINT, DOC, CRC, TRNG.

## Memory layout

`ldscript/gcc_R9A02G021.ld`, every R9A02G021 part:

| Region | Address | Size | Use |
| --- | --- | --- | --- |
| HEAD | 0x00000000 | 0x810 | Reset entry, CLIC vector table, option-setting memory, trap entries |
| FLASH | 0x00000810 | 128 KB - 0x810 | Code and constants |
| DATA_FLASH | 0x40100000 | 4 KB | Not used by the library |
| RAM | 0x20004000 | 12 KB | SRAM0 (parity): data, heap, stack |
| IRAM | 0x20000000 | 4 KB | ECC SRAM: DTC vector table (128 bytes, 1 KB aligned), `.iram.text`, code run from RAM (DFU flash routines) |

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

## Debug

The on-board J-Link OB of the FPB-R9A02G021 connects over 2-wire cJTAG on
P300 (TCKC) and P301 (TMSC), at 4000 kHz. The debug link answers only right
after a reset: the first attempt of J-Link fails (`TDO is constant high`), it
then resets the MCU through the reset pin and connects, so a connection
restarts the program. At 1000 kHz the attempt after the reset fails as well;
SWD and JTAG do not connect (J-Link V9.84). The debugger may lose the MCU in
Software Standby. Keep P300 and P301 free.

Command line:

```
JLinkGDBServerCL -if cJTAG -device R9A02G021 -speed 4000 -endian little -port 2331 -nogui
riscv-none-elf-gdb <example>.elf -ex "target extended-remote localhost:2331"
```

IOcomposer, launch type GDB SEGGER J-Link Debugging. Its Interface choice has
only SWD and JTAG: `-if cJTAG` at the end of the server options takes the
place of the `-if jtag` the launch puts first, the server console then shows
`Target interface: cJTAG`.

| Tab | Setting | Value |
| --- | --- | --- |
| Debugger | Device name | `R9A02G021` |
| Debugger | Interface | JTAG |
| Debugger | Initial speed | Fixed, 4000 kHz |
| Debugger | Other options (J-Link GDB Server Setup) | `-singlerun -strict -timeout 0 -nogui -if cJTAG` |
| Debugger | GDB client executable | `riscv-none-elf-gdb` |
| Startup | Initial Reset and Halt, Low speed | 4000 kHz |
| Startup | JTAG/SWD Speed | Fixed, 4000 kHz |

With the Startup speeds left at their defaults (1000 kHz, Auto) the reset
before the download loses the target: `Target connection lost`, then
`Verify failed` on each block. Use cJTAG at 4000 kHz and reset at the start
of the connection. OpenOCD has not been tried: the probe and its driver must
support cJTAG.

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
otherwise. Flash wait states are set above 32 MHz, and the code flash
prefetch buffer (PFBER) is turned on at startup. An unsupported setting
stops startup: `g_R9a02StartupError` holds the reason (`R9a02StartupError_t`)
and the CPU waits in a `wfi` loop, where a debugger can read it.

PCLKB, the clock of SAU, TAU, IICA and ADC12, defaults to ICLK.
`SystemPeriphClockSet(0, Freq)` picks the closest PCLKB not above ICLK. The
drivers compute their rates from PCLKB at Init and SetRate: change PCLKB with
the SAU (UART, SPI), IICA and ADC drivers powered off and initialize them
again (SPSm note 1, 23.3.1). The IICA needs PCLKB at 32 MHz or lower, with
clock supply type B (OFS1.ICSATS = 0) PCLKB is ICLK, so the core clock is
lowered for it (HOCO 32 MHz).

The RTC, UARTA and TML32 clock is the low frequency clock: `OSC_TYPE_XTAL`
selects the SOSC, which the RTC calendar needs, `OSC_TYPE_RC` the LOCO. The
selection stops a running RTC that is on the other clock (OSMCR note 2). A
calendar counting on the SOSC is kept through a reset other than power-on,
a LOCO request at the warm start then keeps the SOSC.

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

`R9a02RouteEvent(EvtId)` routes an event to an IELSR slot with its CLIC
interrupt off. The polling drivers read its IR flag with `R9a02EventFlag` and
clear it with `R9a02EventFlagClear`: the SAU SPI end of transfer, the IICA
end of byte, the ADC end of conversion and the UARTA reception without
interrupt, none of which has a status flag that tells it alone. The manual
describes polling with the CLIC pending bit instead (12.5.3), IR set with the
CLIC interrupt off is to confirm on hardware (SelfTest test 1).

A driver that polls some events and takes others in its handler turns the
CLIC input of its route off and on with `R9a02IntEnable`: the SPI and I2C
drivers poll the start and the address, and run only the data of a transfer
in the handler. A slot can also start the DTC (`dtc_r9a02.h`): while
IELSRn.DTCE is 1 the dispatcher leaves the slot to the DTC, the CPU
interrupt follows the last transfer. `SYSTEM_SNZREQ` starts the DTC only.

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

A pin can have several analog functions at once: P000 is ANI16, DACOUT0 and
IVREF0, P001 ANI17 and DACOUT1, P106 ANI19 and IVCMP0. The A/D converter,
DAC8 and comparator drivers take their pins with `R9a02AnalogPinUse`
(`iopinctrl.h`): a pin is analog from its first user and released when the
last one stops, so closing an A/D channel does not cut a DAC8 output or a
comparator input on the same pin.

`IOPinEnableInterrupt(IntNo, ...)`: `IntNo` is the IRQ line, 0 to 7, and the
pin must be one of that line's pins (P108 is IRQ4 for example).
`IOPinAllocateInterrupt` finds the line from the pin.

The pin masks follow the package. `R9A02_PACKAGE_PINS` defaults to 48;
define 32, 24 or 16 for the smaller parts.

## SAU sharing

UART, SPI and I2C run on the SAU channels. `sau_r9a02.cpp` keeps which
driver owns each channel and starts the unit clock on the first claim:
SPIInit fails on a channel the UART has, and the reverse. The two operation
clocks of a unit, CKm0 and CKm1 (prescaler SPSm), are shared: a channel takes
the closest rate a free prescaler gives, or one already used by a running
channel, the rate is then set with STCLK only. SPSm is written only while no
channel of the unit runs.

| SAU0 channel | 0 | 1 | 2 | 3 |
| --- | --- | --- | --- | --- |
| UART | UART0 TX | UART0 RX | UART1 TX | UART1 RX |
| SPI | SPI00 | SPI01 | SPI10 | SPI11 |
| I2C | IIC00 | IIC01 | IIC10 | IIC11 |

| SAU1 channel | 0 | 1 |
| --- | --- | --- |
| UART | UART2 TX | UART2 RX |
| SPI | SPI20 | SPI21 |
| I2C | IIC20 | IIC21 |

The SPI and I2C drivers turn off the noise filter of their channel pair
(SNFEN), the UART turns it on.

## UART

| DevNo | Device | Pins on the 48-pin package |
| --- | --- | --- |
| 0 | UART0, SAU0 channels 0 (TX) and 1 (RX) | TxD0 P100 or P302, RxD0 P301 or P303 |
| 1 | UART1, SAU0 channels 2 and 3 | TxD1 P001, RxD1 P000 |
| 2 | UART2, SAU1 channels 0 and 1 | TxD2 P103, RxD2 P102 |
| 3 | UARTA0 | TxDA0 P009, RxDA0 P010 |
| 4 | UARTA1 | TxDA1 P403, RxDA1 P008 |

SAU UART rate: fMCK / (2 x (STCLK + 1)), fMCK = PCLKB / 2^n, at most 32 MHz.
`UARTSetRate` returns the closest rate. Data 7 or 8 bits, parity none, even,
odd or space (mark is rejected), 1 or 2 stop bits, no flow control.
Interrupt mode uses the IOsonata CFIFO, the TX interrupt runs one level below
`IntPrio` so that the RX interrupt can preempt it.

UARTA: its clock is the HOCO (or EXTAL), not PCLKB, and is common to both
channels. Rate fSEL / 2^CK / (2 x k), 200 to 153600 bps as characterized;
UARTInit fails for a rate outside these limits or more than 2 % away from the
closest one, below about 1500 bps with the HOCO at 48 MHz. Data 5, 7 or 8
bits, same parity and stop bit choices. Without interrupts RX_ENDI is polled
through `R9a02RouteEvent`, the UARTA has no receive data flag.

`bDMAMode` enables DTC TX on DevNo 0 to 4. It follows the Microchip SAM4L
UART pattern: copy up to 16 bytes from the TX CFIFO into private storage,
start the block, and chain queued blocks on completion. When the DTC finishes
and its last byte still occupies the transmit buffer, the CPU stages the
next queued block immediately. The following buffer-empty event arms that
prepared block without another copy. SAU uses halfword
storage and writes because SDR requires 16-bit access; UARTA uses bytes.
An idle one-byte write goes directly to the UART register. SAU requests its
transfer-end event, allowing following bytes to accumulate before selecting
the next block, as in STM32's TC path. DMA blocks use buffer-empty events.
While a transfer is active, `TxData` only adds to the CFIFO; its TX event
starts the next byte or queued DMA block. This avoids descriptor setup for each byte of a byte-mode
producer. `UART_EVT_TXREADY` is reported when the queue drains, as in STM32.
The caller may reuse accepted data immediately, including with a non-blocking
FIFO. A positive TX return counts bytes accepted, not bytes already on wire.
RX remains interrupt driven or polled according to `bIntMode`, with immediate
short-burst delivery, data-bit masking and line-error reporting.

The DTC source must be enabled in the CLIC (manual 14.5), so the TX completion
route is enabled even when `bIntMode` is false. It chains blocks without
application callbacks in that mode. With `bIntMode` false, RX is polled on
each `TxData` and `RxData` call and before and after TX completion work in
that mandatory interrupt. This keeps a pending RX byte from waiting in the
one-byte hardware buffer while the CPU stages the next TX block. Foreground
DMA service also checks RX after TX work, as in the STM32 polling path.
An accepted single-byte write to a busy blocking FIFO leaves TX to its
completion interrupt. A full FIFO, non-blocking FIFO, or pending RX causes
`TxData` to poll both directions; `RxData` also services pending TX events.
Thus a full blocking FIFO can make progress with CPU interrupts masked.
Disable and PowerOff stop DTC access before stopping the UART; Enable discards the old TX
queue and DMA staging. A rate change retains staged bytes not yet written to
the UART, subject to the existing SAU behavior of stopping an in-flight frame.
Snooze drains TX before stopping the channels and resumes after wake-up.

Hardware evidence (2026-10-14): `UartPrbsTxTest`, library and application
Release, byte mode, 1,000,000 baud, 8N1, UART0 through J-Link VCOM:

| Interrupts | DMA | Revision | Last reported average bytes/s | Integrity in supplied capture |
|------------|-----|----------|-------------------------------|-------------------------------|
| Off | Off | `bf44b8f8` (earlier run) | 100,016.46 | No errors reported |
| On | Off | `81992b8` | 100,012.83 | No errors reported |
| On | On | `de89b598` (earlier run) | 100,017.46 | One discontinuity, estimated 3 bytes lost, recovered |
| Off | On | `81992b8` | 100,004.76 | One discontinuity, estimated 4 bytes lost, recovered |

All four modes have demonstrated the 100,000-byte/s payload rate of
1 Mbaud, 8N1. The latest rerun covers interrupt-on/DMA-off and
interrupt-off/DMA-on after `81992b8`; the other two rows are earlier runs.

The latest interrupt-on/DMA-off capture has no reported errors across its
30 supplied reports. In interrupt-off/DMA-on, the first 22 supplied reports
are clean, then the counters change to drop 1, lost 1 (4 bytes), recovered 1
and remain flat through the last report. The earlier interrupt-on/DMA-on
capture likewise records drop 1, lost 1 (3 bytes), recovered 1 after clean
reports. These losses occurred during the runs. The logs do not locate
the loss between MCU TX, J-Link VCOM and the host, so DTC data integrity
remains unresolved despite line-rate throughput.

Earlier interrupt-on/DMA-on results were about 86,498 bytes/s at `bf44b8f8`
and 85,726 bytes/s at `7856bcbe`. At `de89b598`, transfer-end notification for
an isolated SAU byte allowed following bytes to accumulate for a DMA block.
Queued blocks still use buffer-empty events for continuous transfer.

At `81992b8`, an isolated byte in byte-IRQ mode also starts with
transfer-end notification, restoring buffer-empty service for queued data.
Polling/DMA avoids full TX/RX service after every accepted byte while the
blocking FIFO is busy, retaining polling when full or RX needs service.
Release and Debug checks cover masked-IRQ progress, RX during TX, byte
ordering and the switch between transfer-end and buffer-empty events.
The hardware rerun now confirms line-rate TX for byte-IRQ and polling/DMA.
Long-run PRBS integrity remains unresolved.
Keep the shared test configuration unchanged when comparing MCUs; Debug is
for development debugging.

Follow-up loopback evidence (2026-10-14): unchanged `UartLoopback` and
`Python/uartprbs_lb.py`, Release, 1,000,000 baud, UART0 through J-Link VCOM.
These captures precede the RX-service correction described below.

| Interrupts | DMA | Typical reported RX bytes/s | Integrity in supplied capture |
|------------|-----|-----------------------------|-------------------------------|
| On | Off | About 65,900 | No errors reported |
| Off | Off | About 66,300 | No errors reported |
| Off | On | About 64,600 | Sustained loss; final report errors 36,783, lost 36,789 |
| On | On | About 65,900 | No errors reported |

The polling/DMA loss is distinct from the earlier isolated PRBS TX losses.
The TX completion interrupt was delaying foreground RX polling without
servicing RX itself. The regression check reproduces overrun when a byte
arrives before or during DMA staging and the next character arrives before
foreground polling resumes. SAU and UARTA now service RX before and after
TX work, without enabling RX interrupts or application callbacks in polling
mode. Existing RX error handling and FIFO policy are retained. The new
interleaving checks fail on the old driver and pass with the correction in
Release and Debug.

Hardware confirmation after `7b28c9c` (2026-10-14), same Release loopback
configuration, interrupt off and DMA on: all 19 supplied reports have
errors 0, with RX rates from 65,924.42 to 66,030.23 bytes/s (about 66 kB/s).
No lost-byte counter is reported. The user stopped the run with Ctrl-C.
This confirms the sustained polling/DMA loopback loss is resolved in this
capture. Together with the earlier three clean modes, all four UART0
loopback configurations have clean hardware evidence. It does not resolve
the isolated losses from the earlier transmit-only PRBS captures.

UARTA is linked only on request, so that images without it stay small (the
DFU boot): link the application with `-Wl,-u,R9a02UartaInit` (the
`UartaLoopback` example does), or call `R9a02UartaInit` directly. Without it
`UARTInit` fails for DevNo 3 and 4.

Reception in Snooze (23.6.3): `R9a02UartSnooze(pDev, true)` (`sau_r9a02.h`)
for UART0 or UART2 after `UARTInit` with `bIntMode`, at 4800 or 9600 bps,
PCLKB the HOCO at 24 or 32 MHz without divider, EXTAL stopped; it returns
false otherwise. The UART then runs on the operation clock of table 23.95
(PCLKB / 2^5 at 4800, / 2^4 at 9600), the closest STCLK in normal operation
(9615 or 4807.7 bps), the STCLK of the table in Snooze. The TX CFIFO is sent
before the channels stop, at the enabling and at each Software Standby. In
Software Standby a start bit on RxD starts Snooze and the character is
received without the CPU: a good one wakes it, the Snooze end interrupt puts
it in the RX CFIFO and the UART back in normal operation (figure 23.41 steps
10 to 12), then `UART_EVT_RXDATA`; one with an error sends the MCU back to
Software Standby. A character that follows before the receive channel is
restarted, right after the wake-up, is lost, and so is one that ends while the
MCU enters Software Standby (the next one wakes it): the sender waits for an
answer, or repeats. One UART at a time, and it replaces any other Snooze set
up (`R9a02SnoozeSet`); once another set up replaces it, the UART stays in
normal operation until `R9a02UartSnooze(pDev, false)`. Not linked into an
image that does not call it. The UartSnoozeDemo example uses it.

## SPI

`SPICfg_t.DevNo` 0 to 5 is SPI00, SPI01, SPI10, SPI11, SPI20, SPI21 (SAU0
channels 0 to 3, SAU1 channels 0 and 1). Master or slave, 7 or 8 bit frames,
MSB or LSB first, the four clock modes; 3-wire mode fails SPIInit.

Master: chip selects on GPIO pins (`SPICSEL_AUTO` or `SPICSEL_MAN`). Pins:
SCK and MOSI `IOPINOP_FUNC16` outputs, MISO `IOPINOP_FUNC16` input, chip
selects `IOPINOP_GPIO` outputs; other settings fail SPIInit. Rate at most
PCLKB / 2 for SPI00, PCLKB / 4 for the others; the electrical limit depends
on VCC (tables 37.32 and 37.33): SPI00 at most 12 MHz from 2.7 V.

Transfer modes:

| Mode | Reception (RxData) | Transmission (TxData) |
| --- | --- | --- |
| Polling | Each byte waits for the SAU end of transfer event | Same |
| `bIntEn` | In the end of transfer interrupt, returns -1, `DEVINTRF_EVT_COMPLETED` with the buffer at the end | Polled, then the release and `DEVINTRF_EVT_COMPLETED`, returns -1 |
| `bDmaEn` | The DTC moves Len - 1 bytes, the CPU the last one; with `bIntEn` in the background, without it the call waits | As above |

The DTC reads SDRmn by bytes, as the DTC example of the manual (14.6.1),
and writes the dummy bytes by 16 bits; SDRmn is written by 16 bits only
(23.3.8). The command of a read (`TxSrData`) is always polled. Reset cancels
a reception in progress.

Slave: the SAU has no slave select input, the chip select is a GPIO input on
an IRQ pin (IRQ0 to IRQ7) and its rising edge ends a frame. Pins: SCK and
MOSI `IOPINOP_FUNC16` inputs, MISO `IOPINOP_FUNC16` output, chip select
`IOPINOP_GPIO` input, exactly these four. `bIntEn` and `SPICSEL_AUTO` are
required, `bDmaEn` needs 8 bit frames. The IOsonata slave sequence:
`DEVINTRF_EVT_STATECHG` before each frame, where the handler gives the
buffers (`SetSlaveRxBuffer`, `SetSlaveTxData`, index 0), then at the chip
select rising edge `DEVINTRF_EVT_COMPLETED` with the receive buffer and the
byte count, -1 on an overrun, a frame longer than the receive buffer, or a
chip select already low when the frame was prepared. The dummy byte follows
the transmit data.

- CPHA=0 uses continuous mode: transmit byte 0 is in the shifter and byte
  1 in SDRmn while CS is high. Each buffer-empty event reads RX and queues
  the byte after the next. CPHA=1 (modes 1 and 3) uses transfer-end events:
  only byte 0 is preloaded, then each byte end reads RX and loads the next
  TX byte. This makes the final received byte available without another
  SCK edge. The master must leave enough time between bytes for this
  read/write service. MISO is always driven and every SCK edge counts:
  the bus is point to point, SCK idle while CS is high.
- Without `bDmaEn` the CPU handles each byte. With it the DTC moves the
  received byte to the buffer and the transmit byte through a 16-bit stage
  to SDRmn. A DTC run ends with the receive buffer or transmit data, then
  the CPU starts the next run. CPHA=0 allows one byte time to queue TX;
  CPHA=1 must load TX before the next byte starts and prepare the next DTC
  run before that byte ends. A register model cannot validate these times.
- Rate is the highest SCK of the master: at most fMCK / 8 with fMCK above
  16 MHz, fMCK / 6 below (table 37.34, VCC 2.7 V or more). fMCK is the
  highest operation clock of the unit, 24 MHz from PCLKB 48 MHz, so 3 MHz.
- The SAU and chip select interrupts run at `IntPrio`; an interrupt at the
  same or a higher level that lasts a byte time makes the slave late.

Slave reception in Snooze (23.5.7): `R9a02SpiSnooze(pDev, true)`
(`sau_r9a02.h`) for a slave on SPI00 (DevNo 0) or SPI20 (DevNo 4), `Rate` at
most 500 kHz, PCLKB the HOCO or the MOCO, EXTAL stopped; it returns false
otherwise. At the Software Standby entry, with the chip select high, the
channel is set to receive only. The first SCK edge starts Snooze, byte 0 is
received without the CPU (MISO stays at its idle level) and its end wakes
the CPU, which takes over the frame from byte 1 as in normal operation.
The master leaves time for the wake-up after byte 0 (Software Standby to
Normal, table 37.21, plus the interrupt) before it clocks byte 1, and does
not expect data from the slave in byte 0. `SetRate` above 500 kHz fails
while it is enabled. One SPI at a time, and it replaces any other Snooze set
up; once another set up replaces it, the slave stays in normal operation.
Not linked into an image that does not call it.

On the FPB board SPI20 is on P101 (SCK20), P102 (SI20) and P103 (SO20), SPI21
on P204 (SCK21), P205 (SI21) and P206 (SO21). The SpiMasterSlave example
wires them together.

The October 15, 2026 hardware capture with master DMA=1 INT=1 and slave
DMA=1 INT=1 passes all mode 0 transfers at 1 MHz, 8 bits: 1, 16 and 129
bytes in both directions, including short frames followed by long frames.
CS uses P109-to-P201 (J13-1 to J13-7). Mode 1 then fails on its first byte
with slave RX=-1 and data a1 instead of 30. The driver had waited for the
first buffer load before SCK, overwritten transmit byte 0 on timeout, and
enabled RX DMA for the initial buffer-empty event. The first correction
(`fb41374`) removed that overwrite; the next hardware capture still passes
mode 0 but reports RX=0 on the first mode 1 byte. In continuous mode that
final RX byte waits in the shifter for another SCK edge. CPHA=1 now uses
transfer-end events and a one-byte TX offset in both CPU and DTC paths.
Debug/Release builds and the corrected register model pass.

The hardware reruns after `2c03bf7` pass all four modes at 1 MHz, 8 bits,
in all four configurations below:

| Master DMA | Master INT | Slave DMA | Slave INT | Result |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1 | 1 | PASS |
| 1 | 0 | 1 | 1 | PASS |
| 0 | 0 | 0 | 1 | PASS |
| 0 | 1 | 0 | 1 | PASS |

Every 1-, 16- and 129-byte master TX/RX check passes, including repeated
transfers and short frames followed by long frames. All four runs end with
`SPI master/slave loopback PASS`. This validates the CPHA=1 transfer-end
correction for DMA and CPU slave service with these master configurations.
The captures did not identify build type.

A subsequent mixed configuration, master DMA=1 INT=0 with slave DMA=0
INT=1, passes all mode 0 checks but fails the mode 1 16-byte master RX:
slave RX=-1, master RX[1]=0xFF instead of 0xA1. The mode 1 one-byte RX
and preceding 16-byte TX checks pass. Modes 2 and 3 are not reached.
Master TX is CPU-polled even with DMA selected; multibyte master RX uses
DTC to write the next dummy byte immediately after reading the previous
byte. The interrupt slave must read RX and load TX between bytes in
CPHA=1 (23.5.6, figure 23.23). This failure is consistent with insufficient
inter-byte time for CPU slave service; the register model does not measure
that timing. The follow-up with master DMA=1 INT=0 and slave DMA=1 INT=1
passes every check in modes 0-3, including the previously failing 16-byte
master RX. This validates master DMA with polled completion when the slave
also uses DMA and supports the slave-service timing explanation for the
mixed failure. Do not count the mixed configuration as validated. Other
combinations, maximum gap-free rate and Snooze still need hardware tests.

## I2C

`I2CCfg_t.DevNo` 0 is IICA0 (SCLA0 P302 or P102, SDAA0 P303 or P103), 1 is
IICA1 (SCLA1 P011, SDAA1 P010). Master or slave, 7-bit addresses, standard,
fast and fast plus modes, `IOPINOP_FUNC6` open-drain pins with external
pull-ups (the internal pull-ups are off in output mode, 16.2.5).
Once the IICA is enabled both pins are set in output mode, whatever direction
the pin map gives (24.2.9). PCLKB must be 1 to 32 MHz (I2CInit fails otherwise), fast mode needs 3.5 MHz
and fast plus 10 MHz. The IICA is enabled only with both lines high: a bus
held low is first cleared by 9 clocks on SCL as a GPIO and a stop. In master
reception the last byte is not acknowledged before a stop or a restart.

Slave (`I2CMODE_SLAVE`, 24.4.16): `bIntEn` and one address, `NbSlaveAddr` 1,
`SlaveAddr[0]` 0x08 to 0x77, `Rate` for the mode (digital filter, IICWL and
IICWH of 24.3.2 (2)). The IICA acknowledges its address, then stretches SCL
after each byte until the interrupt handles it. The events follow the other
IOsonata slaves (RA4M1, SAM4L), slave index 0:
`DEVINTRF_EVT_WRITE_RQST` (give the buffer with `SetWriteRqstBuffer`),
`DEVINTRF_EVT_READ_RQST` with the count of the bytes written before a
restart (give the data with `SetReadRqstData`), `DEVINTRF_EVT_COMPLETED` at
the stop, or at a restart that ends a read or goes to another device, with
the bytes received or sent from the data. A byte that does not fit the
buffer is not acknowledged; past the data the slave sends 0xFF. Extension
codes and the general call are not acknowledged. The master must allow
clock stretching after each byte: an IICA master does, the SAU simplified
I2C does not. The slave handler must run within about 8 clock periods after
a stop, before the master ends the address of its next transfer; later the
stop and that address are reported as one restart. With master and slave on
the same MCU, set the slave interrupt priority above the master's: the
master interrupt waits for the stop condition, which the slave releases. The
slave does not clock the bus to recover it: I2CInit fails when the bus does
not become free, Reset tries again. The I2CMasterSlave example wires IICA1
to IICA0.

DevNo 2 to 7 are the SAU simplified I2C IIC00, IIC01, IIC10, IIC11, IIC20,
IIC21 (`i2c_sau_r9a02.cpp`): one master, no clock stretching during a byte,
no arbitration, 1 kHz to 1 MHz with a 50 % duty. Pins `IOPINOP_FUNC16`, SDA
open drain, SCL open drain or CMOS. The start, restart and stop conditions
are made by software; the manual gives no restart procedure, it is a stop
without the final SDA rise followed by a start. IIC00 is on the debug pins
P300 and P301 of the 48-pin package.

With `bIntEn` the data of Rx, and of Tx up to `R9A02_I2C_TX_COPY_SIZE` bytes
(32, copied), go in the end of byte interrupt; a longer Tx is polled in the
call. Both end with the stop, the release and one `DEVINTRF_EVT_COMPLETED`,
the call returns -1. A NACK ends the transfer with the bytes acknowledged.
The start, the address and the command of a read are polled.

On IICA0 and IICA1, `bDmaEn` moves TX/RX payload bytes through the DTC.
The CPU still handles each protocol event, checks ACKD and selects ACKE;
this is DMA-assisted byte movement, not autonomous block DMA. IICA0
reserves ELC_SWEVT0, IICA1 ELC_SWEVT1, each with an additional ICU slot.
Do not link those software events elsewhere while the instance uses DMA.
`bIntEn` still selects polling or background master completion, and the
slave still requires interrupts. SAU simplified I2C rejects `bDmaEn`.

Hardware validation on FPB-R9A02G021, October 14, 2026, using
`I2CMasterSlave` (IICA1 master, IICA0 slave):

| Master DMA | Master INT | Slave DMA | Slave INT | Result |
| --- | --- | --- | --- | --- |
| 0 | 0 | 0 | 1 | PASS |
| 1 | 1 | 1 | 1 | PASS |
| 1 | 0 | 1 | 1 | PASS |

Each run initialized both devices, wrote 8/8 bytes at offset 4, read back
`a0 a1 a2 a3 a4 a5 a6 a7`, then received 4/4 bytes `0c 0d 0e 0f`.
The DMA runs report `DMA RX 4/4 bytes` and `I2C master/slave loopback PASS`.
These results cover the listed configurations; build type was not included
in the hardware logs.

## ADC

`AdcR9a02` (`include/adc_r9a02.h`) is an `AdcDevice`. `AdcChanCfg_t.Chan` is a
logical channel 0 to 7, `PinP.PinNo` the input: ANI number (0 to 5, 16 to
19), `R9A02_ADC_INPUT_TEMP` or `R9A02_ADC_INPUT_VREF`. Single ended, one
reference, VCC or AVREFP (P002), 8, 10 or 12 bits, one-shot select mode,
software trigger. `AcqTime` selects the conversion clock and normal mode 1 or
2 over all open channels, the internal inputs get the 5 us they need. Polling
or interrupt (`DEV_EVT_DATA_RDY` at the end of the channels). `Calibrate`
measures zero and full scale (ADTES) and corrects the results.

Round of the open channels: scan mode when they are exactly ANI0 to 3, ANI1
to 4 or ANI2 to 5 (one ADC_ENDI for the four), select mode otherwise.
`ADC_CONV_MODE_CONTINUOUS` (interrupt only) repeats the rounds; `AdcCfg_t.Rate`
in mHz is the round rate, started by a trigger of the `Timer` given to
`Init`, 0 for back to back rounds. Results go to the channel FIFO
(`AdcChanCfg_t.pFifoMem`) with the Timer time stamp in us.

R9A02 additions to `AdcDevice`:

- `Trigger(Src, ElcEvt)`: hardware trigger, TAU channel 1, RTC alarm, TML32
  interval or an ELC event; each trigger converts a scan or one channel.
- `Window(Low, High, bOutside)`: compare window in volts (8 bits of the
  reference), results outside of what is asked give no event and are not
  stored. Hardware trigger and Snooze only.
- `Snooze(bEnable)`: conversion in Snooze, started by the Snooze entry
  through the ELC, hardware trigger wait mode; see Low power.

The temperature sensor calibration point is `R9A02_FLCN_TSCDR` (CAL140, the
result at 140 C and VCC 3.3 V, without ADTES correction): see the ADCDemo
example. The manual gives ANI18 and ANI19 on P105 and P106 in the pin tables
and swapped in table 29.12; the driver follows the pin tables, to confirm on
hardware (SelfTest test 9). On the FPB board P105 is Arduino A4 (ANI18),
P106 A5 (ANI19), as the board user's manual gives.

FPB-R9A02G021 ADCDemo hardware tests on October 14, 2026 passed
29 polling and 18 interrupt-mode conversion rounds at 12-bit resolution,
with ANI2/P400 tied to GND and ANI3/P401 tied to 3.3 V. The grounded input
reported 0 to 2 mV, the supply input 3299 mV, and internal VREF and
factory-calibrated temperature were stable and plausible. See
`tests/r9a02/README.md` for measured ranges and untested ADC features.

## RTC

`include/rtc_r9a02.h`: `R9a02RtcInit(Handler, Ctx, IntPrio, bWakeup)`, then
`R9a02RtcSetTime`, `R9a02RtcGetTime` (`struct tm`, 2000 to 2099, 24-hour),
`R9a02RtcSetPeriod` (0.5 s to one month) and `R9a02RtcSetAlarm` (weekdays,
hour, minute). The calendar counts only on the 32.768 kHz crystal (low
frequency clock `OSC_TYPE_XTAL`); on the LOCO only the fixed-cycle interrupt
works. An RTC already counting is kept by `R9a02RtcInit` after a reset other
than power-on. With the alarm enabled the counters are read or written only
with the 1 s fixed-cycle interrupt, from its event (20.2.2).

On the FPB-R9A02G021, the RTC calendar running from SOSC produced consecutive
one-second timestamps across the alarm events at 23:29:00 and 23:31:00.
Both alarms printed `Alarm, next in 2 min set`; periodic output continued
after the second alarm through 23:31:10. The firmware used Software Standby
between events. J-Link sometimes reports 0xDEADBEEF while debugging this
mode, but the UART log demonstrates continued operation; this debugger issue
is not classified as a confirmed firmware fault. A separate Sleep-mode run produced every second from 23:39:01 through
23:44:52, including alarms at 23:41:00 and 23:43:00 with successful
rearming. Absolute RTC accuracy and power-cycle retention remain untested. See `tests/r9a02/README.md`.

## DTC and ELC

`include/dtc_r9a02.h`: `R9a02DtcSet(IntNo, pInfo)` gives the records of an
IELSR slot (16-byte `R9a02DtcInfo_t`, in SRAM), `R9a02DtcEnable` turns the
slot over to the DTC (IELSRn.DTCE), `R9a02DtcDisable` back to the CPU. The
vector table is the `.bss.dtc_vector` section, placed first in IRAM by the
R9A02 linker scripts (1 KB alignment). Records are re-read at each transfer
(DTCCR.RRS = 0). `MRB.DISEL = 1` is not supported. Users: UART TX
(`bDMAMode`), SPI master reception and slave transfers (`bDmaEn`), IICA
master/slave payload transfers (`bDmaEn`), and the TML32 interval flag
(MODE_DTC, MODE_SNOOZE).

`include/elc_r9a02.h`: `R9a02ElcLink(Dest, EvtId)` links an event to the A/D
converter start, a port group, a DAC8 channel or the TML32. The ELC module
runs while a link exists.

## TML32

`include/tml32_r9a02.h`: one interval, 16-bit mode when the count fits, 32-bit
mode otherwise, clock HOCO, MOCO, EXTAL or the low frequency clock (LOCO or
SOSC) divided by 2^0 to 2^7. `R9a02Tml32Init` takes the period closest to
`nsPeriod` and starts. The interval flag holds TML32_OUTI high until cleared:

| Mode | Flag cleared by | Use |
| --- | --- | --- |
| `R9A02_TML32_MODE_INT` | The interrupt, then the handler | Periodic interrupt, wake-up from Software Standby (`bWakeup`, low frequency clock) |
| `R9A02_TML32_MODE_DTC` | The DTC, no CPU | A/D trigger (`R9A02_ADC_TRIG_TML32`), ELC links |
| `R9A02_TML32_MODE_SNOOZE` | The DTC, also at each Snooze entry | Snooze request of `R9a02SnoozeSet` (manual 19.3.7) |

The TML32 runs in Software Standby and Snooze only from the low frequency
clock. Its registers survive the resets other than power-on and voltage
monitor, `R9a02Tml32Init` stops what a previous program left running.

## DAC8 and comparators

`include/dac_r9a02.h`: `DacR9a02` (`DacDevice`) with `OpenChannel(Chan,
bPinOut)`, `WriteCode`, `WriteVolt` (reference VCC, `Vcc()` sets it, 3.3 V by
default), `Trigger` (an ELC event turns the channel on). Channel 0 drives
DACOUT0 (P000) and is the reference of comparator 1, channel 1 DACOUT1
(P001). No output buffer: load 4 MOhm or more. VCC 2.7 V or more.

`AnalogComp` (`converters/analog_comp.h`): DevNo 0 is CMP0 (IVCMP0 P106),
reference the internal 1.44 V (`RefSrc` 0, VCC 2.94 V or more) or IVREF0
P000 (`RefSrc` 1); DevNo 1 is CMP1 (IVCMP1 P107), reference DAC8 channel 0
at `CompVolt` (`RefSrc` 0, the comparator owns the channel) or IVREF1 P104.
Continuous mode only, no hysteresis, no filter, events `ANALOG_COMP_EVT_HIGHER`
and `ANALOG_COMP_EVT_LOWER` from both edges. On the FPB board P107 is LED1.

## Low power

`R9a02LowPowerEnter(bStandby)` enters Sleep (`false`) or Software Standby
(`true`) with WFI and returns after the wake-up, the clocks as before.
`R9a02WakeupSet(EvtId, bEnable)` enables the ICU events that end Software
Standby (IRQ0 to 7, RTC, TML32, IICA, UARTA reception, LVD, KEY, IWDT,
comparators); the event must also be routed with `R9a02RegisterIntHandler`.
The machine timer and the SAU cannot end Software Standby. `AdcR9a02::Disable`
leaves the ADC ready for it. Before Software Standby the application waits
for its UART transmissions to end: the SAU and UARTA clocks stop. The DTC
stops accepting requests for Software Standby (DTCST = 0) unless Snooze uses
it. `R9a02StandbyHookSet` lets a driver act just before and after Software
Standby (the A/D converter in Snooze sets ADM2.AWC there).

Snooze (`R9a02SnoozeSet`): in Software Standby the request source (an IRQ pin,
KEY, RTC, TML32, RXD0, RXD2, RIN0, one at a time) starts Snooze, the
peripherals that work in it run without the CPU. `End` sends the MCU back to
Software Standby (A/D result outside the window, DTC, TML32, SAU or REMC end),
`WakeEvt` wakes the CPU through SELSR0 (A/D end, DTC end, SAU end or error,
REMC, DOC). Snooze is enabled for each Software Standby of
`R9a02LowPowerEnter`, the `WakeEvt` interrupt (ICU_SNZCANCEL) is on only for
the time of it, a request left from Normal mode dropped: 12.2.10 does not say
whether the event also reaches it in Normal mode. A driver checks with
`R9a02SnoozeIsSet` that the set up is still its own. The AdcSnoozeDemo example
converts in Snooze at each TML32 interval and wakes the CPU above a voltage
(manual 10.8.4). The UART and SPI drivers set up their own Snooze for a
reception (see UART and SPI), the UartSnoozeDemo example sleeps until a
character arrives.

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
1.39 s). TML32 is not a timer device: it has no readable count (manual
19.2), see TML32.

The FPB hardware TimerDemo runs reported stable intervals of 100006 us for
machine timer DevNo 0 (11 callbacks) and 100010 us for TAU0/TAU7 DevNo 1/8
(18 callbacks each). These runs exercise continuous 100 ms callbacks only;
the build type was not included in the captures. Remaining timer modes and
channels are not yet hardware validated. See `tests/r9a02/README.md`.

## DFU

`ldscript/dfu_layout_r9a02g021.ld`: boot 0x0 to 0x8FFF, record 0x9000 to
0x9FFF, application slot 0 0xA000 to 0x1FFFF, recovery flag at 0x20006FF0.
`dfu_boot_r9a02g021.ld` links the boot (with HEAD), `gcc_R9A02G021_dfu.ld`
an application for slot 0. The GCC Release DfuBoot image now links using LTO within the existing 36 KB
boot partition (2026-10-15: text 35636, data 464, bss 6705 bytes in the
reported build). The Debug configuration is not validated to fit. UARTA is not
linked into it (see UART).

`dfu_r9a02.cpp` programs the code flash in 8-byte units and erases 2 KB
blocks from routines in `.iram.text`, and resets through SWRCR.

## Examples

See [R9A02G021/exemples/README.md](R9A02G021/exemples/README.md). The
SelfTest example checks on the board the points the manual leaves open or
the port assumed (polling with the CLIC input off, the TML32 interval
formula, the DTC and the CLIC, the end of an SPI reception by the DTC, ANI18
and ANI19, the A/D compare window at ADLL, comparator wake-up from Software
Standby) and prints PASS, FAIL or SKIP for each: run it first on a new board.

