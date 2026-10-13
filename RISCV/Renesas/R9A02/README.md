# R9A02G021 port

IOsonata port for the Renesas R9A02G021 RISC-V MCU, first board FPB-R9A02G021
(R9A02G0214CNE, 48 pins). References: R9A02G021 User's Manual: Hardware
R01UH1036EJ0110 and datasheet R01DS0422EJ0110. Register definitions are
IOsonata's own, in `include/r9a02g021.h`; no Renesas SDK file is used.

The core is Renesas' RV32IMAC with Zba, Zbb, Zbs, Zicsr and Zifencei,
machine mode only, with a CLIC and a 64-bit machine timer. The peripherals
are not the RA2 set: there is no SCI, AGT or GPT. UART is the SAU, timers are
the TAU and the machine timer.

## Status (October 13, 2026)

| Function | Files | State |
| --- | --- | --- |
| Startup, option-setting memory, linker layout | `RISCV/src/ResetEntry.c`, `ldscript/*.ld` | Built and linked |
| Clocks, module stop, delay | `R9A02G021/lib/src/system_r9a02.c`, `include/system_r9a02.h` | Built |
| Trap, CLIC vector table, ICU event routing | `R9A02G021/lib/src/Vectors_R9A02.c`, `src/interrupt_r9a02.cpp`, `include/interrupt_r9a02.h` | Built |
| GPIO, pin functions, IRQ0 to IRQ7 pin interrupts | `src/iopincfg_r9a02.c`, `include/iopinctrl.h` | Built |
| SAU channel and prescaler sharing | `src/sau_r9a02.cpp`, `include/sau_r9a02.h` | Built |
| UART (SAU UART0 to UART2), polling and interrupt | `src/uart_r9a02.cpp` | Built |
| UART (UARTA0, UARTA1), polling and interrupt | `src/uarta_r9a02.cpp`, `include/uarta_r9a02.h` | Built |
| SPI master (SAU simplified SPI), polling, interrupt, DTC reception | `src/spi_r9a02.cpp` | Built |
| I2C master (IICA0, IICA1), polling and interrupt | `src/i2c_r9a02.cpp`, `include/i2c_r9a02.h` | Built |
| I2C master (SAU simplified I2C IIC00 to IIC21), polling and interrupt | `src/i2c_sau_r9a02.cpp` | Built |
| ADC (ADC12, 10 pins, temperature sensor, internal reference, scan, continuous, hardware trigger, window, Snooze) | `src/adc_r9a02.cpp`, `include/adc_r9a02.h` | Built |
| RTC (calendar, fixed-cycle interrupt, alarm) | `src/rtc_r9a02.cpp`, `include/rtc_r9a02.h` | Built |
| DTC (data transfer controller) | `src/dtc_r9a02.cpp`, `include/dtc_r9a02.h` | Built |
| ELC (event link controller) | `src/elc_r9a02.cpp`, `include/elc_r9a02.h` | Built |
| TML32 (32-bit interval timer) | `src/tml32_r9a02.cpp`, `include/tml32_r9a02.h` | Built |
| DAC8 (2 channels) | `src/dac_r9a02.cpp`, `include/dac_r9a02.h` | Built |
| Comparators (CMP0, CMP1) | `src/analog_comp_r9a02.cpp` | Built |
| Sleep, Software Standby, wake-up sources, Snooze | `R9A02G021/lib/src/system_r9a02.c`, `include/system_r9a02.h` | Built |
| Timers (machine timer, TAU channels 0 to 7) | `src/timer_r9a02.cpp`, `include/timer_r9a02.h` | Built |
| DFU flash target | `src/dfu_r9a02.cpp` | Built, boot linked, boot emulated |

Built with xPack riscv-none-elf-gcc 15.2.0, `-march=rv32imac_zba_zbb_zbs_zicsr_zifencei -mabi=ilp32`.
The register use of the drivers was checked against the manual. Nothing has
been run on hardware yet. Not implemented: SPI slave, SPI transmission by the
DTC, the DTC for I2C (the CPU acts at each byte), UART transfers by the DTC,
SAU UART and SPI reception in Snooze, TML32 8-bit channels and capture,
REMC, KINT, DOC, CRC, TRNG.

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
CLIC interrupt off is to confirm on hardware.

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
odd or space (mark is rejected), 1 or 2 stop bits, no flow control, no DMA.
Interrupt mode uses the IOsonata CFIFO, the TX interrupt runs one level below
`IntPrio` so that the RX interrupt can preempt it.

UARTA: its clock is the HOCO (or EXTAL), not PCLKB, and is common to both
channels. Rate fSEL / 2^CK / (2 x k), 200 to 153600 bps as characterized;
UARTInit fails for a rate outside these limits or more than 2 % away from the
closest one, below about 1500 bps with the HOCO at 48 MHz. Data 5, 7 or 8
bits, same parity and stop bit choices. Without interrupts RX_ENDI is polled
through `R9a02RouteEvent`, the UARTA has no receive data flag.

UARTA is linked only on request, so that images without it stay small (the
DFU boot): link the application with `-Wl,-u,R9a02UartaInit` (the
`UartaLoopback` example does), or call `R9a02UartaInit` directly. Without it
`UARTInit` fails for DevNo 3 and 4.

## SPI

`SPICfg_t.DevNo` 0 to 5 is SPI00, SPI01, SPI10, SPI11, SPI20, SPI21 (SAU0
channels 0 to 3, SAU1 channels 0 and 1). Master only, 7 or 8 bit frames, MSB
or LSB first, the four clock modes, chip selects on GPIO pins
(`SPICSEL_AUTO` or `SPICSEL_MAN`). Pins: SCK and MOSI `IOPINOP_FUNC16`
outputs, MISO `IOPINOP_FUNC16` input, chip selects `IOPINOP_GPIO` outputs;
other settings fail SPIInit, as do slave and 3-wire modes. Rate at most
PCLKB / 2 for SPI00, PCLKB / 4 for the others; the electrical limit depends
on VCC (tables 37.32 and 37.33): SPI00 at most 12 MHz from 2.7 V.

Transfer modes:

| Mode | Reception (RxData) | Transmission (TxData) |
| --- | --- | --- |
| Polling | Each byte waits for the SAU end of transfer event | Same |
| `bIntEn` | In the end of transfer interrupt, returns -1, `DEVINTRF_EVT_COMPLETED` with the buffer at the end | Polled, then the release and `DEVINTRF_EVT_COMPLETED`, returns -1 |
| `bDmaEn` | The DTC moves Len - 1 bytes, the CPU the last one; with `bIntEn` in the background, without it the call waits | As above |

The DTC reads SDRmn by bytes, as the DTC example of the manual (14.6.1),
and writes the dummy bytes by 16 bits; transmission by the DTC would need
byte writes to SDRmn, which the manual does not describe. The command of a
read (`TxSrData`) is always polled. Reset cancels a reception in progress.

On the FPB board SPI20 is on P101 (SCK20), P102 (SI20) and P103 (SO20).

## I2C

`I2CCfg_t.DevNo` 0 is IICA0 (SCLA0 P302 or P102, SDAA0 P303 or P103), 1 is
IICA1 (SCLA1 P011, SDAA1 P010). Master, 7-bit addresses, standard, fast and
fast plus modes, `IOPINOP_FUNC6` open-drain pins with external pull-ups.
PCLKB must be 1 to 32 MHz (I2CInit fails otherwise), fast mode needs 3.5 MHz
and fast plus 10 MHz. The IICA is enabled only with both lines high: a bus
held low is first cleared by 9 clocks on SCL as a GPIO and a stop. In master
reception the last byte is not acknowledged before a stop or a restart.

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
The start, the address and the command of a read are polled. `bDmaEn` fails
I2CInit: the ACK of each byte and the NACK of the last received one need the
CPU.

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
hardware. P105 is also on the FPB Arduino header as A1.

## RTC

`include/rtc_r9a02.h`: `R9a02RtcInit(Handler, Ctx, IntPrio, bWakeup)`, then
`R9a02RtcSetTime`, `R9a02RtcGetTime` (`struct tm`, 2000 to 2099, 24-hour),
`R9a02RtcSetPeriod` (0.5 s to one month) and `R9a02RtcSetAlarm` (weekdays,
hour, minute). The calendar counts only on the 32.768 kHz crystal (low
frequency clock `OSC_TYPE_XTAL`); on the LOCO only the fixed-cycle interrupt
works. An RTC already counting is kept by `R9a02RtcInit` after a reset other
than power-on. With the alarm enabled the counters are read or written only
with the 1 s fixed-cycle interrupt, from its event (20.2.2).

## DTC and ELC

`include/dtc_r9a02.h`: `R9a02DtcSet(IntNo, pInfo)` gives the records of an
IELSR slot (16-byte `R9a02DtcInfo_t`, in SRAM), `R9a02DtcEnable` turns the
slot over to the DTC (IELSRn.DTCE), `R9a02DtcDisable` back to the CPU. The
vector table is the `.bss.dtc_vector` section, placed first in IRAM by the
R9A02 linker scripts (1 KB alignment). Records are re-read at each transfer
(DTCCR.RRS = 0). `MRB.DISEL = 1` is not supported. Users: the SPI reception
(`bDmaEn`) and the TML32 interval flag (MODE_DTC, MODE_SNOOZE).

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

Snooze (`R9a02SnoozeSet`): in Software Standby the request source (an IRQ
pin, KEY, RTC, TML32, RXD0, RXD2, RIN0, one at a time) starts Snooze, the
peripherals that work in it run without the CPU. `End` sends the MCU back
to Software Standby (A/D result outside the window, DTC, TML32, SAU or REMC
end), `WakeEvt` wakes the CPU through SELSR0 (A/D end, DTC end, SAU end or
error, REMC, DOC). Snooze is enabled for each Software Standby of
`R9a02LowPowerEnter`. The AdcSnoozeDemo example converts in Snooze at each
TML32 interval and wakes the CPU above a voltage (manual 10.8.4). The SAU
UART and SPI reception in Snooze are not supported by the drivers.

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

## DFU

`ldscript/dfu_layout_r9a02g021.ld`: boot 0x0 to 0x8FFF, record 0x9000 to
0x9FFF, application slot 0 0xA000 to 0x1FFFF, recovery flag at 0x20006FF0.
`dfu_boot_r9a02g021.ld` links the boot (with HEAD), `gcc_R9A02G021_dfu.ld`
an application for slot 0. The boot fits only when built with -Os (36578 of
36864 bytes); the Debug configuration at -O0 does not fit. UARTA is not
linked into it (see UART).

`dfu_r9a02.cpp` programs the code flash in 8-byte units and erases 2 KB
blocks from routines in `.iram.text`, and resets through SWRCR.

## e2 studio

`R9A02G021/lib/e2` is the library as an e2 studio project, `IOsonata_R9A02G021`,
built with LLVM for Renesas RISC-V (19.1.7) for the R9A02G0214CNE on the
FPB-R9A02G021. It links the same sources as the IOC project `lib/ioc`, with
the same defines and include directories, `-march` with Zba, Zbb, Zbs, Zicsr
and Zifencei, C and C++ gnu17.

The library sources were compiled with clang 19.1.7 and these settings, and
the examples linked from them with GNU ld (October 13, 2026); the DFU boot
then passes the emulator tests. Do not mix objects of the two compilers: GCC
for riscv-none-elf makes `uint32_t` an `unsigned long`, clang an
`unsigned int`, so C++ functions taking one get different symbol names.

## Examples

See [R9A02G021/exemples/README.md](R9A02G021/exemples/README.md).
