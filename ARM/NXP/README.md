# NXP LPC port status

The current software baseline covers LPC11U35, LPC1769 and LPC54628. Startup,
GPIO, UART, timers, polling-master I2C/SPI and full-speed USB device examples
are integrated. Other NXP directories are not covered by this validation.
Hardware validation is a separate gate: the LPC11U35 board is being replaced,
and host models/builds do not establish physical operation on any board.

## Supported modes

| Area | LPC11U35 | LPC1769 | LPC54628 |
| --- | --- | --- | --- |
| Startup, vector checksum, clock, GPIO | Integrated | Integrated | Integrated |
| UART | Polling/interrupt, no DMA | Polling/interrupt, no DMA | Flexcomm polling/interrupt, no DMA |
| Timer | Shared LPC timer implementation | Shared LPC timer implementation | CTIMER |
| I2C | Polling master | Polling master | Flexcomm polling master |
| SPI | SSP polling master | SSP polling master | Flexcomm polling master |
| USB device | Native full-speed controller | Native CPU/SIE controller | USB0 full-speed controller |
| USB classes/examples | CDC, HID, vendor bulk, composite | CDC, HID, vendor bulk, composite | CDC, HID, vendor bulk, composite |
| USB remote wake | Clock-on suspend | Unsupported | Clock-on suspend |
| USB cable reattachment | Supported | Reset with VBUS present | Supported |
| Firmware update | ROM DFU; no software DfuBoot | Separate existing bootloader | Not validated here |

USB ISO, USB host mode and USB low-power suspend are outside this baseline.
LPC1769 USB DMA and LPC54628 USB1 are not implemented. I2C/SPI slave, interrupt
and DMA operation are not claimed. Optional peripherals outside this table
are not made complete by the tests below.

LPC11U35/LPC54628 share the full-speed USB EP0/data implementation. LPC11U35
and LPC1769 share UART, SSP, I2C master, timer helpers and IAP where hardware
permits. LPC1769's SIE/CPU USB controller has a different register architecture
and remains a family implementation. The generic USB core and classes are
shared across all three; endpoint allocation respects LPC1769 fixed types.

## Examples and builds

Each MCU's `exemples` folder provides TimerDemo, UartLoopback, UartPrbsTxTest,
SpiLoopback, I2CScan, UsbCdcLoopback, UsbCdcPrbsTx, UsbHidLoopback,
UsbCustomBulkLoopback and UsbCompositeLoopback. Import the `ioc` application
and matching MCU library project, then build both in the same configuration.
Application sources are shared under the root `exemples` directory; pin and
clock choices belong in the project's `src/board.h`.

Debug defaults to rdimon semihosting. Enable semihosting when debugging through
IDAP-Link. Release uses nosys without semihosting. Verify each board header
against the physical board before testing. LPC1769 USB requires an external
D+ pull-up switching circuit controlled by SoftConnect P2.9 and a valid 48 MHz
USB clock derived from the board's configured crystal.

Run the complete host suite from the repository root:

```sh
python3 tests/lpc/run.py
```

Firmware checks, using GNU Arm tools with newlib-nano:

```sh
python3 tests/lpc/peripheral_link.py
python3 tests/lpc/timer/link.py
python3 tests/lpc/timer/link17.py
python3 tests/lpc/timer/link546.py
python3 tests/lpc/usb/link.py
python3 tests/lpc/usb/link17.py
python3 tests/lpc/usb/link546.py
```

Each runner accepts `--prefix /path/to/arm-none-eabi-`. The peripheral runner
also accepts `--target 11`, `17` or `546`. These build the actual example
sources, board headers, production startup/linker scripts and required library
objects. They do not compile every optional source in an IOcomposer library.

Completion validation on 2026-10-14 UTC: all 11 LPC host suites pass; generic
USB core (21 cases), HID, bulk, MSC and HCI USB regressions pass; 58 firmware links pass
(34 USB, 18 UART/SPI/I2C, 6 TimerDemo), including Debug/Release semihosting and
vector-checksum checks. LPC1769 real-class integration tests cover all six
CDC/HID/bulk initialization orders, all fixed endpoint types, allocation
exhaustion, transfers, halt recovery and reconfiguration. See
`tests/lpc/usb/README.md` for the current sizes and exact model boundaries.

## Board validation remaining

After the replacement board arrives, run TimerDemo, UART PRBS loopback,
SPI MOSI/MISO loopback, I2C scanning/register reads, then the USB host tests
listed in each example README. The composite host command is:

```sh
python3 Python/usb_composite_loopback.py --cycles 100 --reconfigure
```

Confirm enumeration, payload integrity, cable behavior, suspend/resume,
interrupt latency and runtime stack usage on hardware. Software work for the
supported baseline can move on; hardware qualification remains open.
