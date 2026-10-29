# LPC11U35 examples

Import `../lib/ioc` and the desired `<example>/ioc` project into IOcomposer/Eclipse.
Build the library first, then the application with the same Debug or Release
configuration. Project-relative paths in each section refer to that example
directory; host commands run from the repository root.

## Contents

- [I2CScan](#i2cscan)
- [SpiLoopback](#spiloopback)
- [TimerDemo](#timerdemo)
- [UartLoopback](#uartloopback)
- [UsbCdcLoopback](#usbcdcloopback)
- [UsbCdcPrbsTx](#usbcdcprbstx)
- [UsbCompositeLoopback](#usbcompositeloopback)
- [UsbCustomBulkLoopback](#usbcustombulkloopback)
- [UsbHidLoopback](#usbhidloopback)

## I2CScan

The project links the existing generic
`exemples/i2c/i2c_scan.cpp`.

Connect the target device as follows:

| Signal | LPC11U35 pin |
| --- | --- |
| SDA | P0.5 |
| SCL | P0.4 |
| UART output | P0.19 TX |
| UART input | P0.18 RX |
| Ground | Common ground |

SDA and SCL need external pull-ups to the board's I/O supply. P0.4 is also
listed as an LED pin in the Blinky example; ensure that any attached LED
circuit does not load SCL. The scanner does not configure LED outputs.

Connect a 3.3 V UART adapter RX to P0.19. Open the terminal at 1000000 baud,
8-N-1, without flow control before resetting the board. The scan runs once,
then waits. It probes 7-bit addresses 0x08 through 0x77 using START,
address+write and STOP, without writing data bytes.

At the default 100 kHz, expect:

```text
I2C scan, I2C 0
Rate 100000 Hz
Device at 0xNN
1 devices found
```

The address and count depend on the connected devices. With no responding
device, expect `0 devices found` and no lockup. Reset to scan again.

To check repeated START and a one-byte register read, define both
I2C_SCAN_REG_DEVADDR and I2C_SCAN_REG in `src/board.h` or the compiler
options. Choose a readable, side-effect-free register on the attached
device. The example prints its value after scanning. It sends the register
address as one byte, so this option is for devices with an 8-bit register
address. Leave both undefined for address scanning alone.

Change I2C_SCAN_RATE to 400000 to repeat with a fast-mode-capable device.
The LPC11U35 driver supports polling master operation; interrupt and DMA
modes remain disabled in the generic scanner.

Validation: Debug and Release compiler-to-assembly checks at 100 and
400 kHz, with and without the optional register read; project links and
library/linker paths checked. Full firmware linking and board tests remain
pending.

## SpiLoopback

The project links the shared
`exemples/spi/spi_loopback.cpp`; there is no local copy of the application.

For IBK-IDAP-M revision 0, connect **P3 pin 11 (MOSI) to P3 pin 7 (MISO)**.
SSP1 uses P1.15/function 3 for SCK (P2 pin 1), P0.22/function 3 for MISO,
and P0.21/function 2 for MOSI. P0.7 (P3 pin 10) is GPIO chip select; it needs
no jumper for this loopback. SWD and USB pins are not used by SSP1.

Debug output uses semihosting through IDAP-Link. Release has no semihosting
and prints through UART0 TX P0.19 (P2 pin 4), 1000000 baud, 8N1.
Use a 3.3 V UART adapter with a common ground. UART RX is P0.18 (P2 pin 3).

The LPC SSP driver supports polling master operation. Interrupt and DMA are
disabled. The shared example exercises all four SPI modes, several byte
patterns, 16-byte and 129-byte reads, writes, and command/read transfers.
Successful output ends with `SPI master loopback PASS`.

Debug and Release firmware links are checked with the production LPC11U35
startup and linker script. Physical loopback validation remains pending.

## TimerDemo

This project links the existing `exemples/timer/timer_demo.cpp`.
The default timer is DevNo 0 (CT16B0), running at 10 kHz with interrupt
priority 3. Change TIMER_DEMO_DEVNO in `src/board.h`, or define it in the
compiler options, to test each device:

| DevNo | Hardware | Counter width |
| --- | --- | --- |
| 0 | CT16B0 | 16 bits |
| 1 | CT16B1 | 16 bits |
| 2 | CT32B0 | 32 bits |
| 3 | CT32B1 | 32 bits |

The LED mapping matches the IBK-IDAP-M board and LPC11U35 Blinky example.
The green LED on P0.23 toggles every 100 ms and the red LED on P0.16 every 1 s.
Trigger 2 runs every 250 ms and is visible in the UART output and debugger
counters.

Debug defaults to semihosting; enable semihosting in the debugger to see
console output. Release uses UART without semihosting. The wiring and terminal
settings below apply to Release.

Connect a 3.3 V UART adapter RX to P0.19 and connect ground. P0.18 is UART RX.
Open the terminal at 1000000 baud, 8-N-1, without flow control. Expect three
startup lines reporting 100, 1000 and 250 ms, followed by measured periods
near 100000, 1000000 and 250000 us. A trigger's measured period is initially
zero until its first callback. Small variations reflect interrupt latency.

At 10 kHz the 16-bit counter wraps every 6.5536 s. Run DevNo 0 and 1 beyond
several wraps and confirm that the printed count continues increasing and
trigger periods stay stable. In the debugger, g_TimerInitOk should be true,
g_TriggerCount[0..2] should advance, and g_TriggerPeriod[0..2] should contain
the accepted periods in nanoseconds. Repeat with DevNo 2 and 3.

The default rate gives the 16-bit timers a 3.2768 s half-cycle interrupt
service limit. Higher timer frequencies shorten that interval; see
`tests/lpc/timer/README.md` for driver limits.

Hardware validation and a full firmware link remain pending.

## UartLoopback

This project links `exemples/uart/uart_loopback.cpp`.
The application echoes received bytes using the common UART interface.

On IBK-IDAP-M revision 0, connect a 3.3 V USB-UART adapter:

| Adapter | Board |
| --- | --- |
| TX | P2 pin 3, P0.18 UART0 RX |
| RX | P2 pin 4, P0.19 UART0 TX |
| GND | P2 or P3 pin 13 |

Use 1000000 baud, 8N1, no flow control. Do not jumper the board RX and TX
together: the host sends data and the firmware returns it. The default uses
interrupt-driven RX/TX with 256-byte FIFOs. UART DMA is unsupported and off.
Set UART_INT_MODE to false to exercise polling operation.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_lb.py --baud 1000000 --port /dev/cu.usbserial-DEVICE
```

Replace the port with your adapter's port. Check for sustained echo with no
errors or lost bytes. Stop with Ctrl-C. No throughput result is claimed yet.

Debug retains default semihosting and requires an attached debugger with
semihosting enabled. Release uses nosys without semihosting. Neither build
redirects console output into the UART data stream.

Debug and Release application compilation and links against the complete
LPC11U35 library pass. Physical UART testing remains pending.

## UsbCdcLoopback

The project links `exemples/usb/usb_cdc_loopback.cpp`; there is no local copy
of the application or USB stack.

Debug uses -O0, newlib-nano and rdimon semihosting. Enable semihosting in the
IDAP-Link debugger before running Debug. Release uses -Os, newlib-nano and
nosys, with no semihosting requirement. Both use C++23, no exceptions and no
RTTI, and link `gcc_lpc11u35.ld` with its USB SRAM section.

### Board connections

The supplied IBK-IDAP-M revision 0 schematic (03-2021-04, 2021-04-25) routes:

| Carrier connector | IDAP-M module |
| --- | --- |
| J1 pin 1, VBUS through FER1 | M1 pin 1, VBUS |
| J1 pin 2, D- | M1 pin 5, USB_DM |
| J1 pin 3, D+ | M1 pin 6, USB_DP |
| P1 | Module programming/debug SWD |

Connect the USB host to J1 for CDC traffic. Use the separate SWD connection
with IDAP-Link for programming and debugging.

`src/board.h` selects the existing port's 12 MHz external-crystal setting
and enables the USB clock. The native driver requires module-internal VBUS
sensing on P0.3 FUNC1 and an external USB pull-up switch controlled by P0.6
FUNC1. Only VBUS appears in USB_PINS; the driver selects the fixed USB_CONNECT
function itself. No board-level register writes are needed.

The carrier schematic does not show the module's oscillator or internal
VBUS/soft-connect circuit. These settings are the driver's requirements,
not verification of those module connections. Check them against the module
revision before hardware bring-up. Low-power suspend and ISO are unsupported.

### Loopback check

After programming, connect J1 and open the new USB serial port with a terminal.
Assert DTR, disable terminal local echo and use any baud setting (115200 is
convenient). CDC line coding is accepted but does not configure a physical UART.
The device sends `IOsonata USB CDC Loopback` when the port opens. Typed data
should return unchanged through USB bulk OUT and IN.

Close/reopen the port and unplug/replug J1. The device should enumerate and
resume loopback without reflashing or restarting the application. If the
board is powered from J1, unplugging also resets the MCU; use independent
board power when testing attach/detach without reset.

The example retains the generic demo VID/PID and strings. Allocate an
appropriate VID/PID before distributing firmware as a product.

### Validation

`python3 tests/lpc/usb/link.py` from the repository root compiles this board.h
with the generic loopback, archives the required library objects and links both
Debug and Release against the production linker script. It also checks the
example's source links and semihosting settings. This is a command-line link
check, not a full IOcomposer library build or physical USB test.

With GCC 13.2.1, Debug uses 50560 bytes flash and 4580 bytes main RAM; Release
uses 22096 bytes flash and 3788 bytes main RAM. Both use 832 bytes USB SRAM.
Physical enumeration and throughput remain untested while the board is being
replaced. The host models cover enumeration, CDC requests, data and reconnect.

## UsbCdcPrbsTx

The project links `exemples/usb/usb_cdc_prbs_tx.cpp`
and streams PRBS data through the native USB device driver.

Connect the host to J1 on the IBK-IDAP-M carrier for USB traffic. Use the
separate SWD connector with IDAP-Link for programming and debugging.
The board configuration matches UsbCdcLoopback: a 12 MHz external crystal,
VBUS sensing on P0.3 FUNC1 and USB_CONNECT on P0.6 FUNC1. The carrier
schematic does not verify the module's internal clock or soft-connect
circuit; see [UsbCdcLoopback](#usbcdcloopback) for the wiring
and module requirements.

Debug uses semihosting; enable it in the debugger before running. Release
uses nosys without semihosting. Both configurations use the production
LPC11U35 linker script, including the dedicated USB SRAM section.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_rx.py --baud 1000000 --port /dev/cu.usbmodemDEVICE
```

Replace the port with the enumerated CDC port. The receiver asserts DTR by
default, which opens the CDC data stream. The baud setting is CDC line coding
and does not set the USB transfer rate. Check the reported bytes per second
and PRBS error counters over a sustained run. Stop with Ctrl-C.

Test closing and reopening the port and unplugging/reconnecting USB as
separate runs. A new run establishes a fresh PRBS sequence baseline. The
application uses a 2048-byte TX FIFO and a 16-entry application event queue;
it services deferred USB work while waiting for port open or FIFO space.

The shared example retains its demonstration VID/PID and strings. Allocate
an appropriate VID/PID before distributing firmware as a product.

Run `python3 tests/lpc/usb/link.py` to compile and link the USB examples in
Debug and Release and check their memory limits, vector checksum, USB IRQ
and semihosting selection. Physical enumeration, PRBS integrity, throughput
and reconnect testing remain pending while the board is being replaced.

## UsbCompositeLoopback

The project links the shared
`exemples/usb/usb_composite_loopback.cpp`; it contains CDC serial, a 64-byte
vendor HID report interface, and a vendor-specific bulk interface.

All interfaces echo received bytes. The classes allocate interface and endpoint
numbers. AppRun drains the 16-entry AppEvt queue; AppCheckStatus services all
three transports and retains partial TX writes. The example disables remote
wakeup and low-power suspend. It uses small FIFOs to fit LPC11U35 main RAM.

Debug uses rdimon semihosting; enable it in the IDAP-Link debug session.
Release uses nosys without semihosting. Board routing matches
`../UsbCdcLoopback/src/board.h`; review the [USB board connections](#board-connections) and your schematic
before connecting. Cable detach/reattach is handled by the controller and generic USB layer.

From the repository root, with PyUSB and libusb installed:

```sh
python3 Python/usb_composite_loopback.py --cycles 100
python3 Python/usb_composite_loopback.py --cycles 100 --reconfigure
```

The host test discovers the CDC, HID and bulk endpoints from descriptors and
interleaves packet-sized transfers. It compares exact bytes and can repeat
SET_CONFIGURATION. It is a correctness test, not a throughput measurement.
Close serial terminals and other programs using this USB device first. The
host must allow libusb to claim every interface; kernel-driver detachment is
attempted where supported. Use `--serial` to select one of several boards.

Demo identity: VID 0x1209, PID 0x0005, product
`IOsonata USB Composite Loopback`. These are shared test identifiers.

Debug/Release firmware links pass. See `tests/lpc/usb/README.md` for sizes and
host-model coverage. Physical enumeration, traffic and sleep/wakeup remain
pending board validation.

## UsbCustomBulkLoopback

The project links the shared `exemples/usb/usb_custom_bulk_loopback.cpp`.
It returns received bytes through a vendor-specific USB bulk interface using
the generic UsbdBulk class and native LPC11U35 controller.

Build settings match [UsbCdcLoopback](#usbcdcloopback).

### Board

Use J1 for USB traffic and the separate SWD connection for IDAP-Link. The
board configuration matches UsbCdcLoopback: a 12 MHz external crystal, USB
clock enabled, VBUS sense on P0.3 FUNC1, and soft-connect control on P0.6
FUNC1. The native driver configures USB_CONNECT.

The carrier schematic does not show the module's oscillator or internal
VBUS/soft-connect wiring. See [UsbCdcLoopback](#usbcdcloopback) for the carrier
connections and required module wiring. Hardware testing remains pending.

### Host check

The existing host script needs PyUSB and a working libusb installation.
From the repository root, after programming and connecting J1:

```
python3 -m pip install pyusb
python3 Python/usb_custom_bulk_loopback.py --cycles 100
```

The script selects VID 0x1209, PID 0x0002 and product string
`IOsonata Custom Bulk Loopback`. It discovers the vendor interface and bulk
endpoints from descriptors, then compares echoed payloads of 1, 7, 63, 64,
65, 127, 128, 129, 512 and 1024 bytes. Use `--serial` to select one device
when several match. These identifiers are the shared demo values.

Repeat with `--reconfigure` to exercise SET_CONFIGURATION again. Unplug/replug
J1 and rerun the test to check enumeration and loopback. Use independent board
power if testing detach/attach without an MCU reset. This is a vendor USB
interface; it does not create a serial port.

### Build validation

`python3 tests/lpc/usb/link.py` checks the project links, board header, C++23,
Debug/Release semihosting settings, real USB IRQ, vector checksum, IAP RAM
reservation and USB SRAM placement. It builds the shared application against
a library archive and the production linker script, not every optional source
in the IOcomposer library project.

Physical enumeration and throughput have not been tested on the replacement
board. Host models separately cover vendor-bulk transfers and reconnect in a
CDC/HID/vendor-bulk composite device.

Measured with Arm GNU GCC 13.2.1 and binutils 2.42:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM / 2048 bytes | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 48728 | 4552 | 832 | 1200 |
| Release | 21460 | 3760 | 832 | 1464 |

The stack allowance is not a measured runtime high-water mark.

The shared application uses AppRun and AppCheckStatus with the 16-entry event
queue. It checks USB status while idle, retains partial TX writes under FIFO
backpressure, and waits only when no application data or queued event remains.

## UsbHidLoopback

The project links the shared `exemples/usb/usb_hid_loopback.cpp` application and uses the
native LPC11U35 controller with the generic HID class and interrupt transport.
It echoes 64-byte vendor HID output reports as input reports.

Build settings match [UsbCdcLoopback](#usbcdcloopback).

### Board

Use J1 for USB data and the separate SWD connection for IDAP-Link. The board
configuration matches UsbCdcLoopback: a 12 MHz external crystal, USB clock
enabled, VBUS sense on P0.3 FUNC1, and soft-connect control on P0.6 FUNC1.
The driver configures USB_CONNECT; only VBUS is listed in USB_PINS.

The supplied carrier schematic does not show the module's oscillator or
internal VBUS/soft-connect wiring. See [UsbCdcLoopback](#usbcdcloopback) for the
carrier connections and module requirements. Hardware validation remains
pending while the board is being replaced.

### Host check

From the repository root, after programming and connecting J1:

```
python3 -m pip install hidapi
python3 Python/usb_hid_loopback.py --rounds 1000
```

The script selects VID 0x1209, PID 0x0005 and product name
`IOsonata USB HID Loopback`. It compares each returned 64-byte report with
what was sent. Use `--serial` if multiple matching devices are attached.
The VID/PID are the shared demo values, not a product allocation.

Unplug/replug J1 and rerun the script to check enumeration and report traffic.
Use independent board power to exercise detach/attach without an MCU reset.
The optional `--manual-suspend-wake` phase checks host suspend/resume; this
example does not enable device-initiated remote wake or low-power suspend.

### Build validation

`python3 tests/lpc/usb/link.py` builds both CDC and HID examples in Debug and
Release, using the production LPC11U35 linker script. It checks project source
links, C++23, semihosting selection, vector checksum, real USB interrupt handler,
IAP RAM reservation and USB SRAM placement. This is not a full IOcomposer
library build or a physical USB test.

Measured with Arm GNU GCC 13.2.1 and binutils 2.42:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM / 2048 bytes | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 52312 | 3452 | 832 | 1568 |
| Release | 22752 | 2660 | 832 | 1832 |

The stack allowance is not a measured runtime high-water mark. Host-model
CDC/HID tests cover descriptor routing, reports and reconnect; hardware
report traffic and throughput remain untested.
