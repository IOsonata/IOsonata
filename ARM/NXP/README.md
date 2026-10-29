# NXP LPC port status

The current software baseline covers LPC11U35, LPC1769 and LPC54628. Startup,
GPIO, UART, timers, polling-master I2C/SPI and full-speed USB device examples
are integrated. Other NXP directories are not covered by this validation.
Host models and builds do not establish physical operation. Consult the
peripheral examples for the tested configurations and hardware limits.

## Supported modes

| Area | LPC11U35 | LPC1769 | LPC54628 |
| --- | --- | --- | --- |
| Startup, vector checksum, clock, GPIO | Integrated | Integrated | Integrated |
| UART | Polling/interrupt, no DMA | Polling/interrupt, no DMA | Flexcomm polling/interrupt, TX DMA |
| Timer | Shared LPC timer implementation | Shared LPC timer implementation | CTIMER |
| Watchdog | Not implemented | Not implemented | WWDT |
| RTC | Not implemented | Not implemented | RtcLpc546xx (RtcDevice) |
| ADC | Not implemented | Not implemented | AdcLpc546xx, ADC0 polling, 12 bits, 12 inputs and the temperature sensor |
| PWM | Not implemented | Not implemented | SCT0, 8 channels on the SCT0 output pins, edge mode |
| Internal flash Nvm | Not implemented | Not implemented | NvmIntrf on the IAP ROM, 256 byte pages, byte writes by page update |
| Quad SPI flash | Not implemented | Not implemented | SPIFI as SPI DevNo 10, polling, command mode, 1, 2 or 4 lines, SPI mode 0 or 3 |
| SDRAM | Not implemented | Not implemented | EMC, one SDRAM on a dynamic chip select, generic `SdramInit` (`coredev/sdram.h`), 16 or 32 bit bus |
| SD card | Not implemented | Not implemented | SDIF as the SD/MMC host interface (`Sdmmc`, `sdmmc_lpc546xx.cpp`) of the generic `SDCard`, polling, 1 or 4 data lines |
| LCD | Not implemented | Not implemented | LCD controller, TFT panel from a RGB565 frame buffer (`LcdLpc546xx`, a display controller set up from `DisplayCfg_t` with its panel timings and frame buffer), four orientations; touch panel through the generic GT911 or FT5406 driver |
| Ethernet | Not implemented | Not implemented | ENET MAC and DMA, RMII, LAN8720A or LAN8742A PHY, interrupt events, IP through the shared lwIP service (`src/net/ip_lwip.cpp`) |
| I2S | Not implemented | Not implemented | Flexcomm 6 and 7, master or slave, transmit, receive or full duplex on both, classic I2S 16 bit, polling, interrupt and DMA, MCLK from the audio PLL |
| Crypto | Software | Software | ROM RNG, SHA block for SHA-256 and HMAC after a known-answer test at Init, software AES and P-256 |
| I2C | Polling master | Polling master | Flexcomm master polling/interrupt/DMA, slave interrupt/DMA |
| SPI | SSP polling master | SSP polling master | Flexcomm master polling/interrupt/DMA, slave interrupt/DMA |
| USB device | Native full-speed controller | Native CPU/SIE controller | USB0 full-speed controller (DevNo 0), USB1 high-speed controller (DevNo 1), one at a time |
| USB classes/examples | CDC, HID, vendor bulk, composite | CDC, HID, vendor bulk, composite | CDC, HID, vendor bulk, composite |
| USB remote wake | Clock-on suspend | Unsupported | Clock-on suspend |
| USB cable reattachment | Supported | Reset with VBUS present | Supported |
| Firmware update | ROM DFU; no software DfuBoot | Separate existing bootloader | Not validated here |

USB ISO, USB host mode and USB low-power suspend are outside this baseline.
LPC1769 USB DMA is not implemented. LPC11U35/LPC1769 I2C
and SPI slave, interrupt and DMA operation are not claimed. Optional
peripherals outside this table are not made complete by the tests below.

LPC11U35/LPC54628 share the USB EP0/data implementation of the IP3511
controller, full and high speed, one context per controller. LPC11U35
and LPC1769 share UART, SSP, I2C master, timer helpers and IAP where hardware
permits. LPC1769's SIE/CPU USB controller has a different register architecture
and remains a family implementation. The generic USB core and classes are
shared across all three; endpoint allocation respects LPC1769 fixed types.

## Examples and builds

Each MCU's `exemples` folder provides TimerDemo, UartLoopback, UartPrbsTxTest,
SpiLoopback, I2CScan, UsbCdcLoopback, UsbCdcPrbsTx, UsbHidLoopback,
UsbCustomBulkLoopback and UsbCompositeLoopback. LPC54628 also has
I2CMasterSlave (Flexcomm 2 master wired to Flexcomm 8 slave) and SPIMasterSlave
(Flexcomm 3 master on J14 wired to Flexcomm 9 slave), each on one board,
WdtDemo, RtcDemo, AdcDemo, PwmDemo, NvmDemo, SpifiFlashDemo (the quad SPI
flash of the board under Nvm), SdramTest (the 16 MB SDRAM), SdCardTest (the
SD card slot), AudioToneDemo and AudioLineLoopback (the WM8904 codec:
a tone on the headphones, the line input to the headphones), LcdDemo (the
4.3 inch TFT panel, a test pattern from a frame buffer in the SDRAM, then
its touch panel, GT911 or FT5406 found at startup),
EthernetDemo (DHCP and ping on the RJ45 jack, lwIP 2.2.0 in `external/lwip`
beside IOsonata), CryptoTest, and UsbCdcLoopbackHs, UsbCdcPrbsTxHs and
UsbCustomBulkLoopbackHs (the same USB applications on USB1 at high speed,
`USB_DEVNO 1` and the 12 MHz crystal in board.h, VBUS read by the controller;
board.h shows the GPIO override on P0_16).
Import the `ioc` application
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

## LPC54628 USB1 high speed

USB1 is DevNo 1: EP0 and five bulk or interrupt IN/OUT pairs, 512-byte bulk
packets at high speed (interrupt up to 512), 64-byte packets at full speed
behind a full-speed hub or host. The speed is read at each bus reset and
`UsbCtrlrHighSpeed(1)` reports it. USB0 and USB1 use the same storage in the
USB RAM, so one of them runs at a time.

- Clock: the USB PLL makes 48 MHz from the system oscillator, which must be
  a 12 MHz crystal or TCXO given as the core oscillator (`MCUOSC` in board.h).
  The PLL supply VD3 stays powered after release; the other PLLs use it.
- VBUS, default: no `USB_PINS`. USB1 runs from `UsbEnable` (PLL, PHY and
  clocks on) and reads VBUS on the USB1_VBUS pin (VBUS_DEBOUNCED). The
  connection stays requested; the controller applies the pull-up only with
  VBUS, so the first bus reset of the host is the attachment and DCON_C the
  detachment, as in NXP's IP3511 device driver. A VBUS only supply (a charger)
  is not reported, and nothing reads VBUS after `UsbDisable`.
- VBUS, override: `USB_PINS` entry 0 on any GPIO input wired to the bus power
  keeps USB1 off until VBUS appears on that pin, with its interrupt, as USB0
  does with P0.22. The USB1_VBUS pin must still see the bus power: the PHY
  connects its pull-up only with it.
- Errata (ES_LPC546XX) handled by the port: USB.2, the byte count after an IN
  packet is not checked; USB.15, an IN that follows an OUT packet of 4 + 16 x N
  bytes waits for an IN NAK before it is armed (an IN armed before such an OUT
  is not covered); USB.6, when the first bus reset of an attachment is at full
  speed, `UsbCtrlrProcess` runs NXP's sequence (PHY test mode 100 ms, then
  disconnected 510 ms) once, about 0.6 s in the USB process event; a
  detachment or a restart allows it again. USB.1: OUT packets are always
  armed at the maximum packet size, the NXP workaround. USB.13: interrupt
  endpoints use the generic endpoint type, not the condition of the erratum.
  USB.3 and USB.4: after a remote wakeup the host can see a disconnection and
  enumerate again.

## LPC54628 hardware results

Maintainer results on the LPCXpresso54628, Oct. 27, 2026:

| Example | Configuration | Result |
| --- | --- | --- |
| UartPrbsTxTest | Flexcomm 0, polling, byte mode, no DMA, 1 Mbaud 8N1, function clock FRO 12 MHz (OSR 12, BRG 1) | 100,045 bytes/s average, the line rate of 100,000 bytes/s within the FRO tolerance. 5 bytes dropped and 1 frame error while the receiver joined the stream, none after |
| TimerDemo | CTIMER DevNo 0 at 10 kHz, triggers of 100 ms, 1 s and 250 ms | Measured periods 100000 us, 1000000 us and 250000 us; the millisecond count advances with them |
| CryptoTest | Default crypto provider: ROM RNG, SHA-256/HMAC (SHA block after its known-answer test, or software), software AES and P-256 | 101 passed, 0 failed, 0 skipped: SHA-256 and HMAC against software and RFC 4231, RNG draws, AES ECB/CBC/CTR/CMAC/CCM/GCM known answers and 200 random rounds, P-256 known answers, ECDH and ECDSA. The output does not show whether the SHA block or the software ran |

UART RX (UartLoopback), UART interrupt and TX DMA modes and the other
peripherals have no hardware result yet.

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
