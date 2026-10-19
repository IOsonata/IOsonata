# STM32WBA65I-DK1 BLE status display

Build the matching Debug or Release library in `../../lib/ioc` and import
this project's `ioc` directory as BleStatusDisplayDemo. It links the existing
shared `exemples/bluetooth/ble_security_demo.cpp` with BLE_STATUS_DISPLAY.
Board differences remain in board.h; Bluetooth owns pairing and bond storage.

The device advertises as **BLEStatus**, with the BlueIO UART service and
LE Secure Connections MITM security, like BleSecurityDemo. Connect using a
BLE central such as nRF Connect. The OLED shows up to two connections with
their handles, peer addresses and current security levels:

| Display | Live link security |
|---|---|
| open L1 | Unencrypted |
| enc L2 | Encrypted, unauthenticated |
| MITM L3 | Encrypted, authenticated |
| LESC MITM L4 | LE Secure Connections, authenticated |
| unknown | Security query unavailable or unrecognized |

With no connection, the screen says Waiting for central. This reflects the
peer table; it does not independently confirm over-the-air advertising.
Security comes from BtGapConnSecGet, not the configured policy or the mere
presence of a stored bond. The display does not claim a bond or key size
that the ST security query does not report.

For fresh pairing, compare the six-digit number on the OLED and central.
Open the first ST-LINK VCP at **1000000 baud, 8N1, no flow control**. Type y
to accept matching numbers or n to reject. The existing security example
consumes those confirmation bytes. UART service traffic remains protected.
A bonded reconnect should show its live security level without another
comparison prompt, using the existing Bluetooth implementation.

The display uses SPI3 mode 0 and the hardware-tested OLED pin map: SCK PA0,
MOSI PB8, CS PE1, D/C PE0, reset PE3. TIM16 (Timer DevNo 2) timestamps SysLog
and queues a display update every 250 ms. Bluetooth retains Timer DevNo 0/1.
A 16-entry AppEvt queue serves the application. Timer interrupts perform no
OLED transfer or security query; a full queue is retried at the next tick.
No external wiring is required for the display or pairing confirmation.

Validation: this project and the original BleSecurityDemo compile and link
in Debug and Release. `python3 tests/display/run_ssd1315.py` runs the shared
security example with the real peer pool, AppEvt queue and SSD1315 SPI RAM
model at -O0/-Os under ASan/UBSan. Security queries and radio decisions are
modeled. Tests cover two peers, security changes and unavailable queries,
leading-zero comparison numbers, UART acceptance/rejection, disconnect
cleanup, deferred transfers and full-queue recovery. On October 19, 2026, the maintainer confirmed that BleStatusDisplayDemo
works on the STM32WBA65I-DK1.
