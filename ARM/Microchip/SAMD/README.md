# SAM D11/D21 MCU port

IOsonata port for the Microchip SAM D11 and SAM D21 Cortex-M0+ MCUs. The
drivers in `src` and `include` are shared by both devices; each device has
its library project, vector table, linker script and examples.

| MCU | Evaluation board | Flash | RAM | Library project |
| --- | --- | ---: | ---: | --- |
| ATSAMD21J18A | SAM D21 Xplained Pro | 256 KiB | 32 KiB | `SAMD21/lib/ioc`, `IOsonata_SAMD21J18A` |
| ATSAMD11D14AM | SAM D11 Xplained Pro | 16 KiB | 4 KiB | `SAMD11/lib/ioc`, `IOsonata_SAMD11D14AM` |

The managed projects select the device header through `include/samdxxx.h`
with `__SAMD21J18A__` or `__SAMD11D14AM__`. The register definitions are the
Microchip device CMSIS headers; no vendor driver library is linked.

## Status

| Function | Source | Implementation |
| --- | --- | --- |
| Startup and clocks | `system_samd.c`, `clock_samd.h` | 48 MHz main clock from `g_McuOsc`, generators 0 to 2 |
| GPIO and pin interrupts | `iopincfg_samd.c`, `iopinctrl.h` | PORT pins, functions A to H, EIC lines |
| UART | `uart_samd.cpp` | SERCOM USART, polling or interrupt with CFIFO, TX by DMA |
| Timers | `timer_samd.cpp`, `timer_samd_rtc.cpp`, `timer_samd_tc.cpp`, `timer_samd_tcc.cpp` | RTC, TC and TCC, 64-bit count, triggers, RTC tick interrupt, TC and TCC external trigger capture |
| Event system | `evsys_samd.c`, `evsys_samd.h` | EVSYS channels for the timer tick and external trigger, EIC event outputs |
| I2C | `i2c_samd.cpp` | SERCOM master (polling, interrupt, DMA) and slave (interrupt) |
| SPI | `spi_samd.cpp` | SERCOM master (polling, interrupt, DMA) and slave (interrupt) |
| DMA | `dmac_samd.c`, `include/dmac_samd.h` | DMAC channels of the UART, SPI and I2C drivers: byte beats on a SERCOM trigger, one block per start |
| SERCOM pads and interrupts | `sercom_samd.c`, `shared_intrf_samd.cpp` | Pad routing from the pins, one interrupt handler per SERCOM |
| USB device | `usb_ctrlr_samd.cpp`, `usb_ctrlr_samd_iso.cpp`, `include/usb_ctrlr.h` | Full speed device: control endpoint, bulk, interrupt and isochronous endpoints 1 to 7 in both directions |
| Analog pins and references | `analog_samd.c`, `include/analog_samd.h` | Pins and SYSCTRL VREF outputs shared by the ADC and the DAC, released by their last user |
| DAC | `dac_samd.cpp`, `include/dac_samd.h` | One 10-bit channel written by software: output buffer to the VOUT pin PA02 and the ADC, internal output to the comparators; INT1V, VDDANA and VREFA references |
| ADC | `adc_samd.cpp`, `include/adc_samd.h` | Single conversions by polling: AIN pins, temperature sensor, bandgap, scaled supplies, DAC output; single ended or differential; gain 1/2 to 16; INT1V, supply and external references; averaging |

Not implemented: UART receive by DMA and DMA of the SPI and I2C slaves (a
configuration asking for it fails init), USB host mode, PWM (the TCC are
timers only), watchdog, sleep modes and the NVM controller. The ADC has no interrupt, event or continuous mode, the DAC no
event start (DATABUF).
Crypto is not supported: the devices have no crypto hardware and their
flash and RAM are too small for the software engines, so the libraries do
not build `src/crypto`.

Validation: the libraries and examples build in Debug and Release with the
xPack Arm GNU 15.2.1 toolchain. The SAM D21 library also builds the
generic network and motion sources: lwip and Fusion are cloned next to
IOsonata by the installer (`external/lwip`, `external/Fusion`). Both
libraries define `__PROGRAM_START`, as the other ports: the CMSIS 6 startup
function of `cmsis_gcc_m.h` does not compile in C++ without the device
header first. The libraries do not link the generic
`ARM/src/timer_systick.cpp`: its `TimerInit` would be a second definer next
to `timer_samd.cpp`, which serves every timer device of the port. The SAM
D21 library links the UART transport of the HCI host,
`bt_hci_ctlr_uart.cpp`, not the IPC one of a dual core part, which defines
`BtHciCtlrH4Open` too. The examples were run on a peripheral model
of the two devices (see Model tests below).

Hardware, SAM D11 Xplained Pro (no 32 kHz crystal, default `g_McuOsc`):
`SystemCoreClock` 48000000 on the FDPLL96M from OSC8M / 8, Blinky toggles
at its 250 ms period. `UartPrbsTxTest` Release (library Release), polling,
byte mode, 1 Mbaud 8N1 on SERCOM2 through the EDBG virtual COM port:
100.09 kB/s, the line rate, with no drop, corrupt or lost byte reported by
`Python/uartprbs_rx.py`. A Release example linked against a Release library
not rebuilt after a startup change ran at 17 kB/s, the rate of the same
loop at 8 MHz: rebuild both before a benchmark.

Hardware, SAM D21 Xplained Pro (default `g_McuOsc`, FDPLL96M from OSC8M):
`UartPrbsTxTest` Release, polling, byte mode, 1 Mbaud 8N1 on SERCOM3
through the EDBG virtual COM port: 99.87 kB/s, the line rate, with 2 drops
counted when the receiver joined the stream and none after. Before the
shorter one byte TxData path it ran at 98.2 kB/s (CPU bound, about 489
cycles per byte against 480 per character). Debug builds the D21 library
and examples at -O0: 33.6 kB/s, not a benchmark configuration.

Timers, I2C, SPI, USB, DMA, the ADC and the DAC are not run on hardware
yet.

## Clocks

`SystemInit` sets the main clock to 48 MHz from the oscillators of
`g_McuOsc`:

| `g_McuOsc` | Main clock |
| --- | --- |
| `CoreOsc` crystal or oscillator on XIN, 0.4 to 32 MHz | FDPLL96M at up to 96 MHz from XOSC / (2 x (DIV + 1)), divided by 2 |
| `LowPwrOsc` crystal or oscillator on XIN32 | DFLL48M closed loop, MUL x 32.768 kHz (47.97 MHz) |
| `bUSBClk` and internal oscillators | DFLL48M in USB clock recovery |
| Internal oscillators only (default) | FDPLL96M at 96 MHz from OSC8M / 8, divided by 2; DFLL48M closed loop on OSC32K (47.97 MHz) when the FDPLL96M step fails |

Generic clock generators: 0 is the main clock (CPU, buses, SERCOM, TC and EIC
core clocks), 1 is 32.768 kHz from XOSC32K or the calibrated OSC32K (RTC,
DFLL48M reference), 2 is OSC8M / 8 (FDPLL96M reference without crystal).
`SystemPeriphClockGet(n)` returns the frequency of generator n. One flash
wait state is used, the datasheet value at 48 MHz for VDD 2.7 to 3.63 V.

Order: main clock on OSC8M at 8 MHz, the 32.768 kHz clock on generator 1,
then the 48 MHz steps. The 32.768 kHz clock does not depend on the 48 MHz
steps. The FDPLL96M and the DFLL48M are stopped first only when they run,
after a start without reset. A generic clock channel is enabled by a
CLKCTRL write with CLKEN, CLKEN is not read back, as in ASF
`system_gclk_chan_enable` and the Harmony clock setup: on the SAM D11
Xplained Pro the FDPLL96M channel on generator 2 did not read back enabled.
The FDPLL96M and DFLL48M locks, bounded, show whether their reference
runs. A channel is stopped as ASF does, moved to generator 0 first. With
internal oscillators only, when the FDPLL96M step fails the DFLL48M locked
on the OSC32K clock of generator 1 is the main clock, `SystemCoreClock`
reads 47972352. A 48 MHz step that fails otherwise leaves the main clock on
OSC8M at 8 MHz, the setting of the main branch, and the application runs;
`SystemCoreClock` then reads 8000000.

Debugger variables in `system_samd.c`: `s_SamdStartupError` is the first
stage that failed, also when the DFLL48M took over, and `s_SamdStartupLine`
its source line (the debugger can show one line for all the failures: the
compiler merges the calls at -Os). Stages: 1 `g_McuOsc` not valid, 2
generic clock generator or channel, 3 OSC8M, 4 XOSC, 5 32.768 kHz
oscillator, 6 DFLL48M, 7 FDPLL96M, 8 main clock switch.
`s_SamdStartupLog` lists up to 4 failures in order, stage << 16 | line.
`s_SamdStartupReg` keeps the clock registers at the end of the startup:
CLKCTRL of the DFLL48M and FDPLL96M channels, GENCTRL of generators 0, 1
and 2, GENDIV of generator 2, GCLK STATUS, PCLKSR, OSC8M, OSC32K, DFLLCTRL,
and DPLLSTATUS << 8 | DPLLCTRLA.

## Pins

`IOPINOP_FUNC0` to `IOPINOP_FUNC7` select the peripheral functions A to H.
SERCOM pins use `IOPINOP_PERIPHC` for SERCOM (function C) and
`IOPINOP_PERIPHD` for SERCOM-ALT (function D); the drivers find the pad of
each pin from its function and reject a pin map the SERCOM cannot take.
DevNo of the UART, I2C and SPI is the SERCOM number. One SERCOM is used by one
interface at a time; an init on a SERCOM enabled for another interface fails.

Pin interrupts use the EIC. The EXTINT line of a pin is fixed by the device:
`IOPinEnableInterrupt` takes that line as `IntNo`, `IOPinAllocateInterrupt`
finds it. The NMI pins (PA14 on SAM D11, PA08 on SAM D21) have no EXTINT
line. Cortex-M0+ interrupt priorities are 0 to 3: a less urgent priority
given to a driver, as `IRQ_PRIO_LOW` (5) or `IRQ_PRIO_LOWEST` (7) of
`coredev/interrupt.h`, gets level 3, as on the LPC11 port. A
negative priority fails the init.

## Peripherals

UART: asynchronous, 5 to 8 data bits, none, even or odd parity, 1 or 2 stop
bits, optional RTS/CTS on pads 2 and 3. TX on pad 0 or 2, RX on any other pad.
The baud rate is the closest of the arithmetic and fractional 16x, 8x and 3x
settings. IrDA, half duplex, 9-bit and mark or space parity are rejected.

DMA: `bDMAMode` of the UART and `bDmaEn` of an SPI or I2C master take DMAC
channels at init, which fails when the channels are not free (12 on SAM
D21, 6 on SAM D11). The polling and interrupt modes stay as without DMA:
without `bIntMode` or `bIntEn` the CPU waits for a DMA transfer or serves it
from the next call, with it the DMAC interrupt ends the transfer. The DMAC
interrupt has the most urgent priority of the drivers using it. PowerOff
gives the channels back: transfers return 0 until the next init.

- UART, one channel on the TX trigger (DRE): the CPU writes what DATA
  takes, the channel sends the rest of the TX CFIFO in blocks of up to half
  the CFIFO, in place in a blocking CFIFO, from a copy of 16 bytes (8 on SAM
  D11) in a non-blocking one. Receive stays on the interrupt or polling:
  the USART has no idle line event to end a DMA receive after a short
  burst.
- SPI master, two channels (RX on RXC, TX on DRE): the receive channel is armed first, a direction without data runs from
  the DummyByte or into a sink byte, the transfer ends with the last byte
  received. `FirstRdData` is not updated by DMA transfers.
- I2C master, one channel switched between the TX (MB) and RX (SB)
  triggers: the CPU sends the address and checks
  its acknowledge, the channel moves the data in smart mode, the CPU
  refuses the last read byte and sends the STOP or keeps the bus for a
  repeated START. ADDR.LENEN is not used, a transfer has any length. A byte
  the slave refuses does not stop a DMA transmit: the count is the length,
  less one when the last byte was refused (without DMA the transmit stops at
  the refused byte). A transfer the CPU waits for is bounded at about 15 ms
  plus a few times its length at the SCL rate.

With a UART in the image the DMA code is linked whether the application
asks for DMA or not: UARTInit, which can open a channel, refers to it. On SAM
D11 that is about 1.7 KB of flash and 200 bytes of RAM (UartPrbsTxTest
Release), and UartSlipPrbsTxTest Release and the Debug builds of
UartPrbsRxTest, UartPrbsTxTestTaktOS and UartSlipPrbsTxTest exceed the 16 KB
flash.

Timers, virtual DevNo from low to high frequency:

| Device | SAM D11 DevNo | SAM D21 DevNo | Counter | Clock | Triggers |
| --- | --- | --- | --- | --- | ---: |
| RTC | 0 | 0 | 32-bit | 32.768 kHz / 1 to 1024, default 32.768 kHz | 1 |
| TC1, TC2 | 1, 2 | | 16-bit | main clock / 1 to 1024, default / 16 (3 MHz) | 2 |
| TC3 to TC7 | | 1 to 5 | 16-bit | same | 2 |
| TCC0 | 3 | 6 | 24-bit | same | 4 |
| TCC1 | | 7 | 24-bit | same | 2 |
| TCC2 | | 8 | 16-bit | same | 2 |

The counts are extended to 64 bits by the overflow interrupt, which must be
served within half a counter period (10.9 ms for a 16-bit counter at 3 MHz,
2.8 s for a 24-bit TCC). A trigger period is at least 4 counts. The TCC runs
in normal frequency mode with the top at the counter maximum, its compare
channels as triggers, without waveform output. On SAM D21, TC3 shares its
generic clock with TCC2, TC4 with TC5, TC6 with TC7 and TCC0 with TCC1; the
port sets them all on generator 0.

The RTC registers are synchronized to the 32.768 kHz clock: a write takes
up to 6 of its periods, about 200 us, and any RTC register write stops the
continuous read of the count. The driver waits for these synchronizations
with the interrupts enabled and disables the interrupts only for the
register accesses (about 11 us on the model). After a write the continuous
read is started again from the RTC interrupt at the end of the
synchronization, so a count read later does not wait; a read right after a
write, such as `TimerGetTickCount` in a trigger callback, waits up to about
400 us. `TimerInit`, `TimerEnable`, `TimerDisable`, `TimerReset` and
`TimerSetFrequency` on the RTC take a few hundred microseconds. The count
read can be late by up to 8 periods of 32.768 kHz, a compare match that
close to the deadline counts as the deadline. A trigger due within about
520 us is checked once its compare is in effect, which takes up to about
400 us more. A new compare waits for the count to pass an old compare value
just ahead of it, up to about 520 us. Use a TC for short periods and short
interrupt latency.

Tick interrupt (`bTickInt`), RTC only: `EvtHandler` gets `TIMER_EVT_TICK` at
each count. The tick is the RTC periodic event at the count frequency,
through a resynchronized event channel and the EVSYS interrupt. The
periodic events start at 32768 Hz / 8, so with the tick the RTC frequency is
4096 Hz at most, the default too; a higher frequency asked gives 4096 Hz.
`TimerInit` fails with `bTickInt` on a TC or TCC: their slowest count, main
clock / 1024, is 46.9 kHz.

External trigger, TC and TCC: `TimerEnableExtTrigger(pTimer, TrigDevNo,
Sense)` takes the EIC line `TrigDevNo`; its edges (`Sense`) go through an
asynchronous event channel to the capture input of the last compare
channel: CC1 of a TC, CC3 of TCC0, CC1 of TCC1 and TCC2. That channel is
then not a trigger: `TimerGetMaxTrigger` gives one less, and
`TimerEnableExtTrigger` fails while it has a trigger. The application sets
the pin of the line to the EIC function A (`IOPINOP_FUNC0`) as an input;
the line can no longer take a pin interrupt. `EvtHandler` gets
`TIMER_EVT_EXTTRIG` with the count captured at the edge in `LastCount`, its
low 32 bits; the capture is extended from the count read in the interrupt,
which must come within a counter turn (21.8 ms for a 16-bit counter at
3 MHz). An edge before the previous capture is read is lost. On a TCC the
capture enable is enable protected: enabling or disabling the trigger stops
the counter for its synchronizations, about a microsecond. The RTC has no
capture input, `TimerEnableExtTrigger` fails on it. The TC capture uses
the event input with no event action, which the datasheet does not state
for a plain capture; it is not hardware validated.

I2C: 7-bit addresses, SDA on pad 0 and SCL on pad 1 of pins with the I2C pad
drive: PA14, PA15, PA22 and PA23 on SAM D11; PA08, PA09, PA12, PA13, PA16,
PA17, PA22, PA23, PB12, PB13, PB16, PB17, PB30 and PB31 on SAM D21J. The SCL
rate is set from the main clock for an assumed 215 ns rise time, from about
90 kHz to 1 MHz, with the low and high times of the I2C mode minimums.
Interrupt master transfers return -1 and complete with
`DEVINTRF_EVT_COMPLETED`; the command phase of `Read` is sent by polling.
A new START waits for the bus to be idle, after the STOP of the previous
transfer, up to about 15 ms. Past that, or after a polling transfer timeout,
the SERCOM is disabled and enabled again with its settings and the bus
state set to idle; a slave holding SDA low is not clocked free. After an
arbitration loss the bus stays busy until the other master's STOP.
The slave answers one address.

SPI: 4-wire, 8-bit, SCK from 93.75 kHz to 24 MHz, master chip selects on GPIO
pins. The slave frames a transfer with its hardware SS pad;
`DEVINTRF_EVT_STATECHG` before each frame lets the handler set the buffers,
and the first byte is loaded while SS is high. The slave resets its SERCOM
when SS goes high to drop the bytes loaded ahead, which takes a few
microseconds before the next frame. A byte written to DATA clears the end of
frame flag (TXC): after such a write the slave reads the SS pin, and a high
SS once SS went low in the frame (SSL flag) or a byte was received is also
the end of the frame.

ADC: `AdcSamd` of `adc_samd.h`, the notes of that header give the inputs,
references, gains and timing. A channel is converted alone with its own
reference, gain stage, sampling time and mode, by polling. On the supply
reference the ADC uses VDDANA / 2 (INTVCC1) and the gain stage at half the
gain asked, so that gain 1 reads 0 to VDDANA; at 2.0 V or less, where
INTVCC1 is not allowed, VDDANA / 1.48. The ADC clock is the main clock
divided by 4 to 512, 30 kHz to 2.1 MHz: 1.5 MHz at 48 MHz. The sampling
time is at least the datasheet minimum of the input (250 ns, 10 us for the
temperature sensor and the bandgap, 3 us for the DAC). Init loads the
factory linearity and bias calibration; the first conversion after an enable
or a reference change is not used. Averaging of 2 to 1024 samples gives a
12-bit result. SAM D21 erratum 1.4.1: single shot conversions have linearity
errors at +125 C (workaround: VDDANA above 3 V, or free running mode, which
the driver does not have).

DAC: `DacSamd` of `dac_samd.h`, a `DacDevice` with the methods of the R9A02
DAC driver (`OpenChannel`, `WriteCode`, `WriteVolt`): the generic
`DacDevice` has only `Init`. One channel, Vout = Code x reference / 1023.
`OpenChannel` turns on the output buffer, which drives the VOUT pin (PA02)
and the ADC DAC input, the internal output to the analog comparators, or
both. `Reference` takes an `AdcRefVolt_t`: INT1V, VDDANA (the default,
3.3 V) or VREFA on PA03. CTRLB is enable protected: written with the DAC
disabled, then the DAC enabled, when the application has it on, and the
start-up time of 10 us waited. `WriteCode` returns when the DAC has the
code; the output follows in about 2.85 us, the DAC gives no end of
conversion. The register sequences run with interrupts disabled, so
`WriteCode` can be called from an interrupt.

The ADC and the DAC share PA02 (ADC AIN0, DAC VOUT), PA03 (ADC AIN1, VREFA
of both) and the SYSCTRL VREF outputs: `analog_samd.c` keeps which driver
uses a pin or an output and releases it with its last user, so one external
reference on PA03 can serve both converters. The bandgap output (BGOUTEN)
is on while the ADC converts the bandgap or either converter uses the INT1V
reference, as ASF does; Harmony and the Arduino core do not set it for
INT1V.

USB, device mode: `UsbInit` and the device classes of `src/usb` run on the
controller port `usb_ctrlr_samd.cpp`; `usb_ctrlr_samd_iso.cpp` is linked
only by an application with an isochronous interface. The USB clock is
generator 0: `g_McuOsc.bUSBClk` true and a main clock within 0.25 % of
48 MHz, else `UsbEnable` fails. With the 32.768 kHz crystal the main clock is
the DFLL48M closed loop on it (47.97 MHz), without crystal the DFLL48M in USB
clock recovery. D- and D+ are PA24 and PA25, function G, set by the port.
The VBUS pin, entry 0 of the USB pin map (`USB_PINS`), is read on its EIC
line; without it the port reports bus power present. The endpoint
descriptors are in RAM, one OUT bank and one IN bank per endpoint number.
The bank address must be word aligned: an IN buffer that is not aligned
sends the bytes up to the next word boundary first, from a copy, and the
completion length tells how many were taken; an OUT buffer must be aligned.
An OUT packet two bytes or more below the bank size is written with its two
CRC bytes: the OUT buffer holds the packet size plus 2, or the bank size
(the packet size rounded up to a power of 2, 1023 above 512). The port
gives this size as `USB_CTRLR_RX_CAPACITY` in `usb_ctrlr.h`, and an
interface opened with a smaller buffer fails, one that only sends as well
(`UsbIntrfConfigure`, `UsbIsoIntrfOpen`, the periodic class registration).
The classes of `src/usb` give 64 bytes (CDC, bulk, MSC, HID, interrupt) and
1023 for isochronous by default; a packet size that is not a power of 2
needs the 2 bytes more. EP0 arms its OUT bank only for the data or status
stage that expects it, a SETUP is received whatever the bank state. A
SETUP the host sends before the interrupt has seen the status stage of the
previous request comes after that completion, which the class needs (CDC
line state) and which applies a SET_ADDRESS. An isochronous OUT frame with a CRC error is reported as failed; whether the hardware holds
the bank after it is not stated, this path is not tested. The address
of SET_ADDRESS is written after its status stage. The suspend interrupt is
on once the device has an address: the SUSPEND flag is also set on a bus
that is not driven, as TinyUSB notes. `bLowPowerSuspend` fails `UsbInit`.
The serial string, when the application gives none, is the 128-bit serial
number of the device in hexadecimal. The configuration descriptor buffer of
the USB core is 768 bytes on SAM D21 and 256 bytes on SAM D11
(`USB_CONFIG_DESC_MAXLEN`). The USB interrupt runs with the other
interrupts disabled, as on SAM4L: the port functions are also called from
other interrupts.

## Examples

Each example project links one source of the top `exemples` folder and its
own `src/board.h`, which gives the pins of the Xplained Pro board. DacDemo
has its source in its own `src`: the generic `DacDevice` has no method to
write a value, the demo uses those of `DacSamd`. COM port
is the EDBG virtual COM port: SERCOM3, PA22 TX and PA23 RX on SAM D21;
SERCOM2, PA10 TX and PA11 RX on SAM D11.

| Example | Source in `exemples` | SAM D21 | SAM D11 | Board connections and output |
| --- | --- | --- | --- | --- |
| Blinky | `misc/blinky.c` | Yes | Yes | LED0; SW0 stops the blinking (EIC on SAM D21, read in the loop on SAM D11) |
| PulseTrain | `misc/pulse_train_test.c` | Yes | Yes | Logic analyzer: SAM D21 EXT1 3 to 10 (PB00, PB01, PB06, PB07, PB02 to PB05); SAM D11 EXT1 6, 8, 11, 12, 15 to 18 and LED0 |
| TimerDemo | `timer/timer_demo.cpp` | Yes | Yes | LED0; results on the COM port on SAM D21, no console on SAM D11 (`g_TriggerCount`, `g_Period` in the debugger) |
| UartPrbsTxTest | `uart/uart_prbs_tx.cpp` | Yes | Yes | COM port at 1000000 baud, `Python/uartprbs_rx.py` checks the stream |
| UartPrbsTxTestTaktOS | `uart/uart_prbs_tx_taktos.cpp` | Yes | Yes | Same stream from a TaktOS thread |
| UartPrbsRxTest | `uart/uart_prbs_rx.cpp` | Yes | Yes | COM port at 1000000 baud, `Python/uartprbs_tx.py` sends, error count in `errcnt` |
| UartLoopback | `uart/uart_loopback.cpp` | Yes | Yes | COM port, the bytes sent come back |
| UartSlipPrbsTxTest | `uart/uart_slip_prbs_tx.cpp` | Yes | Yes | SLIP frames of 92 PRBS bytes on the COM port |
| UartSlipPrbsRxTest | `uart/uart_slip_prbs_rx.cpp` | Yes | | SLIP frames of PRBS bytes in, from UartSlipPrbsTxTest or a SLIP encoder |
| UartRetargetDemo | `uart/uart_retarget_demo.cpp` | Yes | | `printf` and standard input on the COM port at 115200 baud, Debug and Release |
| I2CScan | `i2c/i2c_scan.cpp` | Yes | | Devices on EXT1 11 (SDA, PA08) and 12 (SCL, PA09) |
| I2CMasterDemo, I2CSlaveDemo | `i2c/i2c_master_demo.cpp`, `i2c/i2c_slave_demo.cpp` | Yes | | Two boards, slave address 0x22: EXT1 11 to 11, 12 to 12, GND, 4.7 kohm pull-ups |
| I2CMasterSlave | `i2c/i2c_master_slave.cpp` | Yes | | One board, SERCOM2 to SERCOM4, wiring below |
| SPILoopback | `spi/spi_loopback.cpp` | Yes | | EXT1 16 (MOSI, PA06) to EXT1 17 (MISO, PA04) |
| SPIMasterDemo, SPISlaveDemo | `spi/spi_master_demo.cpp`, `spi/spi_slave_demo.cpp` | Yes | | Two boards: EXT1 15 (SS), 16, 17 and 18 pin to pin, GND |
| SPIMasterSlave | `spi/spi_master_slave.cpp` | Yes | | One board, SERCOM0 to SERCOM5, wiring below |
| UsbCdcLoopback | `usb/usb_cdc_loopback.cpp` | Yes | | Target USB connector, serial port, the bytes sent come back |
| UsbCdcLoopbackTaktOS | `usb/usb_cdc_loopback_taktos.cpp` | Yes | | Same, the USB work in a TaktOS thread |
| UsbCdcPrbsTx | `usb/usb_cdc_prbs_tx.cpp` | Yes | | Serial port, PRBS stream |
| UsbDualCdcStress | `usb/usb_dual_cdc_stress.cpp` | Yes | | Two serial ports, `Python/usb_dual_cdc_stress.py` |
| UsbCustomBulkLoopback | `usb/usb_custom_bulk_loopback.cpp` | Yes | | Vendor interface, bulk loopback |
| UsbIntLoopback | `usb/usb_int_loopback.cpp` | Yes | | Vendor interface, interrupt loopback, `Python/usb_int_loopback.py` |
| UsbIsoLoopback | `usb/usb_iso_loopback.cpp` | Yes | | Vendor interface, isochronous loopback, `Python/usb_iso_loopback.py` |
| UsbHidLoopback | `usb/usb_hid_loopback.cpp` | Yes | | HID reports of 64 bytes echoed |
| UsbHidKeyboard | `usb/usb_hid_keyboard.cpp` | Yes | | Keyboard: SW0 is A, a button from EXT1 5 (PB06) to GND is Caps Lock, LED0 shows Caps Lock |
| UsbMscRamDisk | `usb/usb_msc_ramdisk.cpp` | Yes | | Mass storage, 48 sectors of 512 bytes in RAM |
| UartUsbBridge | `usb/uart_usb_bridge.cpp` | Yes | | Target USB serial port to the COM port, 115200 baud at start, the rate the host sets |
| DacDemo | `src/dac_demo.cpp` of the project | Yes | | COM port, once per second: the DAC on PA02 (EXT3 3) stepped from 0 to 3.3 V and read back by the ADC on its DAC input |
| AdcDemo | `converters/adc_demo.cpp` | Yes | | COM port, once per second: VDDCORE, bandgap, temperature sensor and EXT1 3 (PB00, AIN8) in volt |

Output: the SAM D21 Debug examples print through semihosting (rdimon), the
Release examples on the COM port. UartRetargetDemo uses the COM port in
both configurations, its Debug build does not link librdimon (`stddev.c` of
the library defines `_lseek` too). The SAM D11 examples other than Blinky
build in Debug at `-Os`, as in Release, to fit the flash, and without
semihosting; the Blinky Debug build is `-O0` with semihosting.

The SAM D11 library has the same timer, I2C and SPI drivers, but the
examples that format text with `printf` on the UART do not fit its 16 KB
flash: in Release, I2CScan is over by 3968 bytes, SPILoopback by 3472 and
UartRetargetDemo by 2936; I2CMasterSlave and SPIMasterSlave are about 20 to
22 KB. UartSlipPrbsRxTest links, but its 2432 byte receive FIFO leaves 332
bytes of the 4 KB RAM for the stack, and is not included. The SAM D11
TimerDemo defines `TIMER_DEMO_NO_UART` in its `board.h`: without the UART
and `printf` it takes 13550 bytes of flash in Release and 15510 in Debug.
The SAM D11 library has the USB device port, but the USB stack with one
class does not fit the flash: UsbCdcLoopback Release is over by 3468 bytes.
AdcDemo Release is over by 7892 bytes (`printf`), DacDemo, with the ADC, by
11360 bytes; the ADC driver itself is about 4 KB.

The SAM D21 USB examples use the target USB connector, `g_McuOsc` with the
32.768 kHz crystal and `bUSBClk`, and PA14 for VBUS. UartUsbBridge sends the
USB data on the UART by DMA.

UartPrbsTxTestTaktOS needs the TaktOS repository (github.com/IOsonata/TaktOS)
checked out next to IOsonata, as `../TaktOS`, and its `TaktOS_M0` library
project, `TaktOS/ARM/cm0/ioc`, built in the same configuration. The SAM D11
Debug example links the TaktOS Release library to fit the flash.

Not ported: the examples of crypto and of the PWM, watchdog, RTC calendar
and NVM drivers, which the port does not have, and the examples of
external parts (sensors, serial EEPROM and flash, displays).

SAM D21 Xplained Pro wiring of I2CMasterSlave and SPIMasterSlave:

- I2C: PA08 (EXT1 11) to PB12 (EXT2 7), PA09 (EXT1 12) to PB13 (EXT2 8).
- SPI: PA06 (EXT1 16) to PB22 (EXT3 16), PA04 (EXT1 17) to PB16 (EXT3 17),
  PA07 (EXT1 18) to PB23 (EXT3 18), PA05 (EXT1 15) to PB17 (EXT3 15).

Import the library project of the device first, then the example, and
build both in the same configuration. With the board on its EDBG USB port,
from the example `ioc` folder:

```sh
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg \
    -c "adapter speed 400; program Release/TimerDemo.elf verify reset exit"
```

## Model tests

The Release and Debug examples were run on a unicorn model of the
Cortex-M0+ with the PM, SYSCTRL, GCLK, PORT, EIC, EVSYS, RTC, TC, TCC,
SERCOM (USART, I2C master and slave on one bus, SPI master and slave linked
by the chip select), DMAC, USB device, ADC and DAC registers. The USB model reads and
writes the endpoint descriptors and buffers in RAM as the controller does,
following the SETUP, OUT, IN, bus reset and isochronous behavior of the
datasheet; a scripted host enumerates the device and runs the class test. The model follows the datasheet description,
with synchronization delays on the RTC and TC, an RTC continuous read
stopped by any RTC register write, and one instruction per cycle; it does
not replace a hardware test.

| Test | SAM D21 | SAM D11 |
| --- | --- | --- |
| UartPrbsTxTest | PRBS sequence at 1000000 baud, polling, interrupt, DMA and DMA with interrupt | Same |
| UartLoopback | 3000 bytes at 1000000 baud echoed, no overrun, the four modes | Same |
| Blinky | LED toggles, SW0 interrupt on EXTINT 15 stops it | LED toggles, SW0 read stops it |
| TimerDemo RTC | 100 ms triggers, also across the counter overflow | 100 ms triggers |
| TimerDemo TC | TC3 and TC7: 100 and 1000 ms, also across an overflow | TC1 and TC2: 100 and 1000 ms |
| TimerDemo TCC | TCC0: 100, 1000, 250 and 500 ms; TCC1 and TCC2: 100 and 1000 ms; TCC0 and TCC2 also across an overflow | TCC0: 100, 1000, 250 and 500 ms |
| TimerDemo, no console | | The project of the repository, RTC: LED0 toggles every 100 ms, Debug and Release |
| Timer events (test program) | RTC tick at 1024 Hz: 205 ticks in 200 ms, one per count; 32768 Hz asked with the tick gives 4096 Hz; TC3 and TCC0 capture of 1 ms pulses on PA15 (EXTINT15): 50 captures in 50 ms, every interval 3000 counts at 3 MHz, trigger 0 at 5 ms running meanwhile; capture channel refused as a trigger while the external trigger is on, a trigger again after; pin interrupt refused on the event line; RTC external trigger and TC tick refused | Same with TC1 and TCC0, pulses on PA05 (EXTINT5) |
| I2CMasterSlave | Polling, interrupt, DMA and DMA with interrupt master: PASS | Same |
| SPIMasterSlave | Polling, interrupt, DMA and DMA with interrupt master, modes 0 to 3: PASS | Same |
| UartPrbsRxTest | Debug: 8000 PRBS bytes in, one corrupted, 2 errors printed (that byte and the next), none other | Release runs, no output |
| UartSlipPrbsTxTest | 105 frames of 92 bytes in 0.1 s, decoded PRBS without error | Same |
| UartSlipPrbsRxTest | Debug: 87 frames in, one byte corrupted, 2 errors printed, none other | |
| UartRetargetDemo | Release: the words typed come back from `scanf` through `printf`; Debug prints the prompt | |
| PulseTrain | The 8 pins pulse in turn | The 9 pins pulse in turn |
| I2CScan | No slave on the bus: 0 devices found, Debug and Release | |
| I2CMasterDemo, I2CSlaveDemo | Each on its own, the master gets NACK, 0 bytes; the slave waits | |
| SPILoopback | MOSI linked to MISO in the model: modes 0 to 3, PASS, Debug and Release | |
| SPIMasterDemo, SPISlaveDemo | Each on its own: the master reads 0xFF, the slave waits | |
| USB enumeration, all USB examples | Bus reset, device descriptor at address 0, SET_ADDRESS, descriptors, strings (serial from the serial number words), vendor request stalled then GET_STATUS, SET_CONFIGURATION; Debug and Release | UsbCdcLoopback, image for the model flash, DFLL48M in USB clock recovery |
| UsbCdcLoopback | Line coding set and read back, DTR, greeting, 3594 bytes in 96 OUT packets of 1 to 64 bytes echoed; IN halted and cleared, data toggle reset, echo after; DTR set and GET_LINE_CODING sent right after its status stage, the greeting comes again | Same |
| UsbCdcPrbsTx | 20000 bytes, no sequence break | |
| UsbDualCdcStress | CDC 0: greeting and 2520 PRBS bytes echoed, no break reported; CDC 1: 5105 PRBS bytes, no break, 3205 IN packets of 1 to 3 bytes from a source not word aligned; Debug and Release | |
| UsbCustomBulkLoopback | 2864 bytes in 72 packets echoed | |
| UsbIntLoopback | Alternates 1 to 3: packets of 1, 3, 32 and 64 bytes echoed; alternate 0 | |
| UsbIsoLoopback | Alternates 1 and 6 (9 and 63 bytes): 32 frames each echoed in order, none dropped | |
| UsbHidLoopback | 8 reports of 64 bytes echoed | |
| UsbHidKeyboard | SW0 pressed: report with the A key | |
| UsbMscRamDisk | INQUIRY, TEST UNIT READY, READ CAPACITY (48 blocks), WRITE(10) and READ(10) of a block | |
| UartUsbBridge | Line coding, DTR, USB OUT data on the UART TX by DMA, UART RX data on USB IN; Debug and Release | |
| DMA (test program) | 94 checks, Debug and Release: UART TX DMA with a non-blocking CFIFO in interrupt and polling mode and a blocking CFIFO, the output checked, TXREADY, Disable and Enable during a block; channels taken and given back by UARTInit, SPIInit, I2CInit, PowerOff and an init without DMA, inits failing without a free channel; SPI master DMA on the loopback (asynchronous and long transmit, 70000 bytes from the flash in two blocks, receive of the DummyByte, Read), transfers after PowerOff refused; I2C master in the four modes: transmit, refused bytes (4 without DMA, 7 of 8 with DMA), address refused, receive of 1, 2 and 33 bytes, register read; slaves with DMA refused | |
| DacDemo | DAC 0, 0.826, 1.652, 2.474 and 3.300 V on the VDDANA reference, the ADC reads the same values on its DAC input; Debug and Release | |
| DAC (test program) | 69 checks, Debug and Release: VDDANA, INT1V (bandgap output on) and VREFA (PA03) references, also changed while open (CTRLB written with the DAC disabled) and while disabled (the DAC stays off), codes 0 to 1023 and `WriteVolt` rounding and clamping read back by the ADC, output buffer and internal output selection, VOUT and VREFA pins taken and released, Disable and Enable keep the code, Reset, PA03 as VREFA of the ADC and the DAC kept until both release it, `OpenChannel(0)` and `OpenChannel(0, false)` as on R9A02, refusals (channel 1, no output, code 1024, VREFB, differential, a second object), clock off at power off | Same, image for the model flash |
| AdcDemo | VDDCORE 1.199 V, bandgap 1.100 V, temperature sensor 0.688 V, PB00 1.234 V for the model inputs 1.2, 1.1, 0.688 and 1.234 V; Debug and Release | |
| ADC (test program) | 76 checks, Debug and Release: 9 channels in one scan (supply, INT1V with gain 1/2, AREFA with gain 2, VDDIO, VDDCORE, temperature sensor, bandgap, differential, gain 32 on the supply) within 1 LSB; 8 and 10 bits; sampling of 10 us on the temperature sensor; a long sampling time moving the prescaler to 512; INTVCC0 for a 1.8 V supply; averaging of 16 and 1024 samples; 12 configurations refused; pins, TSEN, BGOUTEN and the clock released at close and power off | Same, image for the model flash |

On SAM D11, the TimerDemo with console, I2CMasterSlave and SPIMasterSlave
ran as images linked for the model with a larger flash and the 4 KB RAM of
the device. The model sets the event detected flag of a resynchronized
event channel without a user, which the datasheet does not state: the RTC
tick depends on it.
The model feeds the UART receiver at about 24 kB/s, the receive tests check
the data, not the rate. The TaktOS examples are built only: the model has no
SysTick or PendSV. The USB model host takes one transaction per 21 us and
has no bus timing: the USB tests check the data and the protocol, not the
rate. It assumes a stall request has priority over a NAK, which the
datasheet does not state. The ADC model converts the voltages given to the
model, with the reference, gain, mode, resolution and averaging of the
registers; the first conversion after an enable or a reference change gives
a wrong value, and a pin input reads 0 V when the pin is not on function B. The
DAC model gives DATA / 1023 x reference to the ADC DAC input through the
output buffer (EOEN), and flags a read within 10 us of the enable and a
CTRLB write with the DAC enabled. The DMAC model reads the descriptor at the
channel enable, moves one beat per SERCOM request (DRE or RXC of the USART
and SPI, MB or SB of the I2C master), writes the descriptor back at the end
of the block and at a disable, and sets TCMPL with BLOCKACT INT; the SPI
master model takes a DATA write while a byte shifts (double buffer).

Separate timer tests on the SAM D21 model, the RTC compared with a TC: the
count read after single triggers and after disabling a trigger, single
triggers 4 to 40 counts ahead (none early or lost), a new compare set with
the old one 1 to 12 counts ahead, continuous triggers of 20, 33 and 100
counts, a counter reset, also across the 32-bit counter overflow; two TC
triggers 0 to 2 counts apart, 1500 times, none lost. With TimerDemo on the
RTC, the interrupts stay disabled at most about 11 us after the
initialization, against about 375 us when the driver waited for the RTC
synchronizations with the interrupts disabled. The model did not reproduce
the TC match between the count and the flag reads, nor an old RTC compare
taken for a new deadline: these two fixes are not covered by a test.
