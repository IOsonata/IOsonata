# LPC11U35 UsbCdcLoopback

Import `ioc` and `../../lib/ioc` into IOcomposer/Eclipse. Build the LPC11U35
library first, then UsbCdcLoopback, using the same configuration for both.
The project links `exemples/usb/usb_cdc_loopback.cpp`; there is no local copy
of the application or USB stack.

Debug uses -O0, newlib-nano and rdimon semihosting. Enable semihosting in the
IDAP-Link debugger before running Debug. Release uses -Os, newlib-nano and
nosys, with no semihosting requirement. Both use C++23, no exceptions and no
RTTI, and link `gcc_lpc11u35.ld` with its USB SRAM section.

## Board connections

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

## Loopback check

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

## Validation

`python3 tests/lpc/usb/link.py` from the repository root compiles this board.h
with the generic loopback, archives the required library objects and links both
Debug and Release against the production linker script. It also checks the
example's source links and semihosting settings. This is a command-line link
check, not a full IOcomposer library build or physical USB test.

With GCC 13.2.1, Debug uses 49200 bytes flash and 4476 bytes main RAM; Release
uses 21544 bytes flash and 3680 bytes main RAM. Both use 832 bytes USB SRAM.
Physical enumeration and throughput remain untested while the board is being
replaced. The host models cover enumeration, CDC requests, data and reconnect.
