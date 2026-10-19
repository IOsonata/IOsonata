# STM32WBA65I-DK1 BLE scanner display

Build the matching Debug or Release MCU library in `../../lib/ioc`, then
import this project's `ioc` directory as BleScanDisplayDemo. It links the
same shared BLE scan example as BleCentralScanDemo, with BLE_SCAN_DISPLAY
enabled by board.h. No external wiring is needed.

The OLED lists four recent advertisers with a name (up to 14 characters)
or a 12-digit Bluetooth address, RSSI in dBm and a signal bar. Addresses
include the address type when matching reports, so advertising and scan
response data update the same entry. An unnamed report preserves a known
name. New addresses replace the oldest entry; entries disappear ten seconds
after their last report. RSSI 127 means unavailable and shows no bar.
Bars span -100 to -20 dBm and clamp at either end; they are signal indicators,
not distance measurements.

The active scan settings remain those of BleCentralScanDemo: 100 ms window
per 1000 ms interval. Names may arrive in a later scan response. The display
refreshes every 250 ms through the AppEvt queue. Timer interrupts and scan
reports send no OLED data. A full queue is retried on the next refresh tick.
The application selects a 16-entry queue and continues to use AppRun.

SPI3 and OLED pins match the working display examples: SCK PA0, MOSI PB8,
CS PE1, D/C PE0, reset PE3, mode 0, 8-bit MSB first, polling, requested
4 MHz. Timer DevNo 2 (TIM16) supplies refresh and expiry timing; Bluetooth
retains Timer DevNo 0/1. UART scan reports remain available on the first
ST-LINK VCP at 1000000 baud, 8N1, no flow control.

Validation: OLED and original UART-only scan projects compile and link in
Debug and Release. `python3 tests/display/run_ssd1315.py` tests the shared
example with the production advertisement-name parser, AppEvt queue and
SSD1315 driver against a SPI RAM model at -O0/-Os under ASan/UBSan. It
checks name length/preservation/sanitization, identity matching, eviction,
RSSI bar pixels, deferred transfers, full-queue recovery, expiry and clock
wrap. The combined BLE/OLED example still needs hardware validation.
