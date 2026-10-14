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
| `usb` | LPC11Uxx/LPC54628 shared controller and LPC1769 SIE/CPU controller; EP0/data, lifecycle, class allocation, CDC/HID/vendor bulk and reconfiguration |

The host suites require Python 3, GCC/G++, and GNU binutils. They use temporary
build directories and can be launched from any working directory. Register
models and host linker checks do not replace firmware builds or hardware tests.

## ARM firmware links

With Arm GNU tools, newlib-nano and C++23 support:

```sh
python3 tests/lpc/usb/link.py
python3 tests/lpc/usb/link546.py
python3 tests/lpc/usb/link17.py
python3 tests/lpc/timer/link.py
python3 tests/lpc/timer/link17.py
python3 tests/lpc/timer/link546.py
python3 tests/lpc/usb/link.py --prefix /path/to/bin/arm-none-eabi- --output /tmp/lpc-usb
```

The USB link commands build ten LPC11U35, twelve LPC1769 and twelve LPC54628 images.
All cover CDC loopback/PRBS, HID, vendor bulk and a three-class composite in
Debug and Release; LPC1769 and LPC54628 also include an EP0 bring-up image. Debug retains
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

The completion pass on 2026-10-14 UTC passed all 11 host suites and 58 firmware
links: 34 USB, 18 UART/SPI/I2C and 6 TimerDemo. Generic USB core, bulk, HID, MSC and HCI USB
regressions also passed after adding transfer-type-aware endpoint allocation.
LPC1769 integration now exercises the actual generic classes and public driver
in every CDC/HID/bulk registration order, plus endpoint exhaustion.

Run the peripheral firmware links with:

```sh
python3 tests/lpc/peripheral_link.py
```

Use `--target 11`, `--target 17` or `--target 546` to select one MCU, and
`--prefix` to select an installed GNU Arm toolchain. The shared composite
application now has managed projects for all three MCUs. LPC1769 additionally
has the custom-bulk project that was blocked by fixed endpoint types.

See `ARM/NXP/README.md` for the supported modes and explicit limits. Hardware
validation remains pending; the LPC11U35 board is being replaced. Host checks
do not validate electrical timing, USB enumeration/throughput or stack usage.
