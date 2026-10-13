# LPC tests

Run all LPC host regression suites:

```sh
python3 tests/lpc/run.py
```

Each subfolder has its own `run.py` and README for focused checks:

| Suite | Coverage |
| --- | --- |
| `clock` | LPC11Uxx/LPC17xx clock decoding, setter linkage and device speed limits |
| `gpio` | LPC11Uxx pin configuration/interrupts and LPC17xx access widths/interrupt exclusion |
| `uart` | Shared LPC11Uxx/LPC17xx UART rates, transfers and lifecycle |
| `ssp` | Shared LPC11Uxx/LPC17xx SSP rates, pin validation and polling transfers |
| `i2c` | Shared LPC11Uxx/LPC17xx polling I2C master |
| `timer` | Shared LPC11Uxx/LPC17xx timers and generic helpers |
| `lpc546xx` | LPC546xx CTIMER, Flexcomm I2C and SPI |
| `startup` | LPC11U35/LPC17xx linker checksums, LPC17xx vectors and AHB startup data |
| `iap` | LPC11Uxx IAP compatibility and interrupt state |
| `iap_shared` | Shared IAP commands and interrupt state |
| `usb` | LPC11Uxx EP0, lifecycle, transfers, CDC/HID/vendor bulk and reconfiguration |

The host suites require Python 3, GCC/G++, and GNU binutils. They use temporary
build directories and can be launched from any working directory. Register
models and host linker checks do not replace firmware builds or hardware tests.

## ARM firmware links

With Arm GNU tools, newlib-nano and C++23 support:

```sh
python3 tests/lpc/usb/link.py
python3 tests/lpc/timer/link.py
python3 tests/lpc/timer/link17.py
python3 tests/lpc/timer/link546.py
python3 tests/lpc/usb/link.py --prefix /path/to/bin/arm-none-eabi- --output /tmp/lpc-usb
```

This separate command links eight LPC11U35 firmware builds: CDC, HID, vendor
bulk and a three-class composite, each in Debug and Release. Debug retains
rdimon semihosting; Release uses nosys. It checks production startup, vector
checksum, USB IRQ, IAP reservation, USB SRAM placement and example project
settings. It does not build every optional source in the MCU library project.
The separate timer command links the actual LPC11U35 TimerDemo in Debug and
Release, checks real timer handlers and verifies that USB storage is absent.
The link17 command checks LPC1769 TimerDemo with Cortex-M3 startup and runtime.
The link546 command checks LPC54628 TimerDemo with Cortex-M4 hard-float startup
and runtime, including Debug semihosting and Release UART selection.
Memory measurements and detailed coverage are in `usb/README.md` and
`timer/README.md`.

## Validation status

On October 13, 2026, all 11 host suites passed together using the LPC sources
at `46937def8bb27362b116fb4188a7d74b5a25aef2`. The eight LPC11U35 ARM links
also pass with GCC 13.2.1 and binutils 2.42, as recorded in the USB suite.

LPC11U35 board validation remains pending while the board is being replaced.
A host-model pass does not establish physical UART/SPI/I2C timing, USB
enumeration or throughput, interrupt latency, or runtime stack usage. LPC17xx
and LPC546xx model results likewise do not establish full board validation.

Implemented modes remain deliberate limits: legacy LPC I2C and SSP are
polling masters; legacy UART has polling/interrupt operation without DMA;
LPC11U35 USB has bulk/interrupt endpoints and clock-on suspend, without ISO
or low-power suspend. LPC11U35 uses ROM DFU; no software DfuBoot is provided.
See each suite README for exact family restrictions and hardware checks.
