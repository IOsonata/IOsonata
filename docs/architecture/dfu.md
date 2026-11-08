# DFU Architecture

IOsonata firmware update is a generic layer, the same on every MCU, over a
target layer, one file per MCU family plus a linker layout per MCU:

```text
generic   dfu_image     image format checks: header, TLVs, hash, signature
          dfu_boot      stage 0: install slot 1, verify and start slot 0
          dfu_writer    writes an image as it arrives, slot 1 or slot 0
          dfu_mgr       DfuMgr, the one API every protocol uses
          dfu_wire      stage 0 wire protocol, dfu_wire.md, any DeviceIntrf
          dfu_smp       SMP server (image and OS groups), any transport
          dfu_serial    SMP serial framing, over a UART or USB CDC DeviceIntrf
          bt_dfu_smp    SMP GATT service, Bluetooth LE
          dfu_http      image download over HTTP/1.1, LTE
target    dfu_<mcu>     internal memory erase and program, start, reset
          gcc_<mcu>_dfu_layout.ld   where everything is
```

Each board selects its recovery protocol and transport in `board.h`.
The nRF52832 example uses DfuWire over UART. The nRF52840 example uses
SMP serial over USB CDC. Its running application uses SMP over BLE to
receive updates. Both paths use an MCUboot image signed with `imgtool`.
For build, signing, provisioning and update commands see the
[firmware update user guide](../dfu-user-guide.md).

## Update paths

| Path | Where it runs | Where the image goes | Parts |
|---|---|---|---|
| By wire, UART or USB CDC | stage 0 recovery | straight to slot 0 and the record | every part with a boot |
| Bluetooth LE | the application | slot 1, stage 0 installs it | parts with an IOsonata BLE stack |
| LTE | the application, HTTP pull | slot 1, stage 0 installs it | nRF91 |

Stage 0 serves recovery when there is no valid application, when the board
button is held at reset, or when the application asked for it
(`DfuRecoveryRequest`). OTA is only in parts with a radio; a part without
one has no slot 1.

ESP32-C3 and ESP32-C6 are updated by wire with their ROM download mode
(esptool, UART or the built in USB Serial/JTAG) and get no stage 0 from
IOsonata. The RP2040 library holds generic sources only (no startup, linker
script or UART port) and STM32F3 has no library project, so neither has a
target layer yet. On nRF54H20 the FLPR and PPR coprocessor images are loaded
by the application core and are updated with it.

## What was taken from MCUboot and what was not

Kept, because the host tools depend on it:

- the image format: 32 byte header (magic `0x96f3b83d`), payload, protected
  and unprotected TLV areas, SHA-256 TLV (`0x10`), key hash TLV (`0x01`),
  ECDSA P-256 signature TLV (`0x22`, DER encoded);
- the SMP groups and field names the clients use (image state, upload, erase,
  reset, echo, parameters, bootloader information);
- the SMP serial framing of MCUboot serial recovery, for a target that
  builds stage 0 with DFU_PROTO_SMP (not the default).

Left out:

- swap, scratch and revert. Stage 0 only overwrites (MCUboot "overwrite
  only" mode, reported as mode 2 from the application, and as mode 0,
  single application, from recovery);
- encrypted images, multi-image, RAM load, direct XIP, compressed images;
- RSA and Ed25519;
- the Zephyr flash map. The linker scripts declare the layout.

## Target layer

`include/dfu/dfu_target.h`, implemented once per library configuration:

| Function | Does |
|---|---|
| `DfuTgtWriteUnit` | program unit: 4 NVMC/RRAMC, 16 MRAM, 2 STM32F0, 4 F4, 8 L4, 16/8 WBA, 256 LPC, 512 SAM4 |
| `DfuTgtEraseUnit(addr)` | erase unit holding addr, per sector where sizes differ (F4, LPC17) |
| `DfuTgtErase(addr)` | erase one unit; RRAM and MRAM write ones |
| `DfuTgtWrite(addr, p, len)` | program erased memory, source anywhere, flash included |
| `DfuTgtEntryValid` | Cortex-M vector table, RISC-V entry word |
| `DfuTgtStart` | quiesce interrupts, start slot 0 |
| `DfuTgtReset` | reset keeping RAM |

The internal memory is read memory mapped. Shared helpers:
`ARM/include/dfu_cm.h` (Cortex-M start, VTOR) and `RISCV/include/dfu_rv.h`.

| File | Families |
|---|---|
| `ARM/Nordic/src/dfu_nrfx.cpp` | nRF52, nRF53, nRF91 (NVMC), nRF54L (RRAMC), nRF54H (MRAM) |
| `ARM/ST/src/dfu_stm32.cpp` | STM32F0, F4, L4, L4+, WBA |
| `ARM/Microchip/SAM4E/src/dfu_sam4e.cpp` | SAM4E (EEFC) |
| `ARM/Microchip/SAM4L/src/dfu_sam4l.cpp` | SAM4L (FLASHCALW) |
| `ARM/NXP/LPC17xx/src/dfu_lpc17xx.cpp` | LPC17 (IAP) |
| `ARM/NXP/LPC546xx/src/dfu_lpc546xx.cpp` | LPC546xx (ROM IAP) |
| `ARM/Renesas/RE01/src/dfu_re01.cpp` | RE01 (flash sequencer) |
| `RISCV/Renesas/R9A02/src/dfu_r9a02.cpp` | R9A02G021 (FLCN flash sequencer) |

On nRF52 with a SoftDevice, and on nRF54L with MPSL, the application writes
slot 1 through `Nvm` and `NvmMcuIntrf` so the radio arbitrates each operation
(`DfuStoreNvm`). Everywhere else the application writes through the target
layer (`DfuStoreTgt`), as the boot does.

## Flash layout

The application is linked where it runs, the same as a build without DFU,
and does not carry the MCUboot header in front of it.

```text
slot 0      application payload, at its link address
record      header and TLV areas of the image in slot 0
slot 1      a complete signed image as uploaded, then a trailer (OTA only)
stage 0     the boot code
__dfu_flag  16 bytes at the end of RAM: the recovery request word
```

The record is what lets slot 0 be reported and verified: the image hash
covers the header, payload and protected TLVs, and the header and TLVs of
the running image are in the record while its payload is in slot 0.

Each layout file defines `__dfu_slot0_start/_end`, `__dfu_rec_start/_end`,
`__dfu_boot_start/_end`, `__dfu_flag`, and for OTA parts
`__dfu_slot1_start/_end`; the NVM regions where the part has them.
`DfuLayoutGet` reads the symbols; no C define repeats an address. The boot
and application scripts both stop their RAM at `__dfu_flag`.

| Layout | Boot | Slot 0 | Slot 1 | Update |
|---|---|---|---|---|
| nRF52832 xxAA S132 | 0x74000, 48 KB | 0x26000, 148 KB | 152 KB | BLE, wire |
| nRF52840 S140 | 0xF4000, 48 KB | 0x27000, 400 KB | 408 KB | BLE, USB CDC (SMP) |
| nRF52805 / nRF52810 S112 | 0x27000, 36 KB | 0x19000, 40 KB | none | wire |
| nRF5340 application | 0x0, 48 KB | 0xD000, 972 KB | none | wire |
| nRF5340 network | 0x1000000, 36 KB | 106 KB | 110 KB | BLE, wire |
| nRF54L15 S145 | 0x0, 48 KB | 0xD000, 660 KB | 666 KB | BLE, wire |
| nRF54LM20A S145 | 0x0, 48 KB | 0xD000, 916 KB | 922 KB | BLE, wire (supplied DfuBoot selects UART) |
| nRF54H20 application | 0xE0A0000, 40 KB | 212 KB | none | wire |
| nRF54H20 radio core | 0xE120000, 48 KB | 228 KB | 232 KB | BLE |
| nRF9160, nRF91x1 | 0x0, 48 KB | 0xD000, 484 KB | 488 KB | LTE, wire |
| STM32F030x8 | 0x8000000, 35 KB | 25 KB | none | wire |
| STM32F401xC | 0x8000000, 48 KB | 192 KB | none | wire |
| STM32L476, L496 | 0x8000000, 48 KB | 956 KB | none | wire |
| STM32L4S9 | 0x8000000, 48 KB | 1960 KB | none | wire |
| STM32WBA5x | 0x8000000, 48 KB | 464 KB | 472 KB | BLE (no lib project, no UART driver yet) |
| SAM4E16E | 0x400000, 48 KB | 968 KB | none | wire |
| SAM4L x2 / x4 / x8 | 0x0, 35 / 48 / 48 KB | 88 / 204 / 460 KB | none | wire |
| LPC1769 | 0x0, 48 KB | 460 KB | none | wire |
| LPC54605 | 0x0, 64 KB | 416 KB | none | wire |
| RE01 1500 KB | 0x0, 64 KB | 1440 KB | none | wire |
| R9A02G021 | 0x0, 36 KB | 88 KB | none | wire |

The nRF52805/nRF52810 S112 DFU application slots are limited to 40 KB to reserve
8 KB PDS NVM0 at 0x23000 and 4 KB application NVM1 at 0x25000, before
the DFU record at 0x26000. Their DFU bootloader is Release-only; validate
Release application size against the reduced slot.

The layout table describes supplied DFU layouts, not every MCU supported by
IOsonata. A working UART or USB port alone does not establish a tested DFU
boot. In particular, LM20 has a USB device implementation, but its supplied
DfuBoot still selects UART. Changing that selection requires checking its
USB setup, linked image size and recovery behavior.

Previously recorded stage 0 builds with the wire protocol measured 24.8 to 30.4 KB at -Os over UART
and 36.2 KB over USB CDC (nRF52840); with SMP it was
35 to 37 KB and 45 KB. The boot regions in the table still have the room
the SMP boot needed. The Debug configurations of the small parts must be
built at -Os as well.

## State units, trailer and record

Every state change is a single program unit written into erased memory. A
state unit is 16 bytes, or the program unit when that is larger. On ECC
flash (STM32L4, WBA) a program unit takes one program between erases, so
each state word has a unit to itself:

```text
slot 1 trailer   [end - 2 units] Pending "PEND", [end - 1 unit] Done "DONE"
record           [unit 0] magic "DREC", [unit 1] HdrLen, TlvLen, header, TLVs
```

| Pending | Done | Meaning |
|---|---|---|
| erased | any | nothing to install |
| PENDING | erased | install slot 1 at the next boot |
| PENDING | DONE | installed, or refused, leave it alone |

An upload erases the trailer first, so an interrupted upload is never
pending. An image must end before the trailer.

## Stage 0

```text
reset
  |
  recovery button held? -> recovery
  |
  slot 1 pending and not done?
  |  yes: verify slot 1 (hash, key hash, signature, entry)
  |       ok:  erase the record, copy payload to slot 0, write the record
  |            (magic last), verify slot 0 again, write DONE
  |       bad: write DONE (never retried), keep slot 0
  |
  recovery asked by the application? -> recovery
  |
  verify slot 0 against the record (hash, signature)
  |  ok:  start slot 0
  |  bad: recovery
  |
DfuWire recovery (direct mode):
  BEGIN:  header and TLVs to RAM; key hash, signature over the signed
          digest and version checked before anything is touched; then
          the record is erased
  WRITE:  payload to slot 0 as it arrives
  FINISH: hash from memory against the signed digest, entry, then the
          record, magic last. RESET starts it.
```

Power loss during an install leaves slot 1 pending, so the next boot copies
again; a Done word cut part way counts as done, as ECC flash cannot take a
second write. Every unit is erased before it is programmed, even when it
reads as ones, which on RRAM means ones written before the data. Power loss during a recovery upload leaves no record once the image
header is in, so the boot stays in recovery; before that nothing was
written and slot 0 still starts. Slot 0 with no record is started only when stage 0 is
built with `DFU_BOOT_ALLOW_UNSIGNED`, for debugger development.

`exemples/dfu/dfu_boot_main.cpp` is the boot every target builds. Its
`board.h` gives `DFU_PROTO` and `DFU_TRANSPORT` (`DFU_TRANSPORT_CDC` on
nRF52840, UART elsewhere), `UART_PINS`, `UART_DEVNO`, the
optional `DFU_BUT_PORT`, `DFU_BUT_PIN` recovery button, `DFU_BOOT_BUFSIZE`
and `DFU_BOOT_FIFODEPTH` for parts with little RAM, `DFU_BOOT_ERASE_MS`,
and `DFU_BOOT_ALLOW_DOWNGRADE`.

For a board built with DFU_PROTO_WIRE, use its serial port:

```sh
python3 Python/dfu_wire.py --port /dev/cu.usbserial-example app_signed.bin
python3 Python/dfu_wire.py --port COM7 app_signed.bin
```

## SMP server

`DfuSmp` is transport independent. A transport hands `Process` one complete
SMP packet and a buffer, and sends what `Process` wrote there.

| Group | Id | Op | Purpose |
|---|---|---|---|
| 0 OS | 0 | write | echo |
| 0 OS | 5 | write | reset, done through the transport after the response |
| 0 OS | 6 | read | parameters: buffer size, buffer count |
| 0 OS | 8 | read | bootloader information, `mode` query answers 2, or 0 in recovery |
| 1 image | 0 | read | image list |
| 1 image | 0 | write | test or confirm by hash |
| 1 image | 1 | write | upload |
| 1 image | 5 | write | erase slot 1, or the record in recovery |

In the application, test and confirm both mark slot 1 pending: overwrite
only mode has no revert. The application checks the uploaded image hash and
entry before it accepts them; the signature is checked by stage 0, so the
key lives in one place. In recovery the signature is checked at the end of
the upload, before the record is written.

In recovery the header goes to RAM first, and the record is erased only once
the whole header is in, just before the first payload byte is written. An
image refused from its header (magic, sizes, or, with downgrade disabled, a
version older than slot 0) leaves slot 0 startable, however the host splits
the header over its first requests; nrfutil sends 23 bytes first. A version
refusal answers image group result 27 in a version 2 response, rc 3 in
version 1.

With downgrade disabled, the manager checks the staged version before marking
it pending, including after reconnect. Stage 0 checks it again against the
version in the slot 0 record before replacing the application. An interrupted
install can resume after erasing the old record. Direct recovery that erases
the record also clears the stored version floor.

## Transports

| Transport | File | Notes |
|---|---|---|
| UART, USB CDC | `src/dfu/dfu_serial.cpp` | SMP serial framing: length, packet, CRC-16/XMODEM, base64, frames of at most 127 bytes starting `06 09` then `04 14`; console text on the same line is skipped |
| Bluetooth LE | `src/bluetooth/bt_dfu_smp.cpp` | SMP service `8D53DC1D-1DB7-4CD3-868B-8A527460AA84`, characteristic `DA2E7828-FBCE-4E01-AE9E-261174997C48`, write without response and notify |
| LTE | `src/dfu/dfu_http.cpp` | HTTP/1.1 GET of the signed image from a web server, Content-Length bodies, `Range` resume on a new connection; TLS by the modem socket |

`exemples/bluetooth/ble_ota.cpp` is the BLE OTA application,
`ARM/Nordic/exemples/lte/lte_ota.cpp` the nRF91 LTE one. The LTE example needs the
Modem library with its IOsonata OS glue (`modem_nrf91.h`).

## Keys, signing and production

```sh
imgtool keygen -k dev_p256.pem -t ecdsa-p256
imgtool getpub -k dev_p256.pem > dfu_boot_key.c
imgtool sign -k dev_p256.pem --header-size 0x20 --pad-header --align 4 \
        -v 1.0.0 -S <slot 1 size - 2 state units> app.bin app_signed.bin
```

`Python/dfu_prod_hex.py` takes the state unit from the layout's
`__dfu_state_unit` (LPC 256, SAM4 512, 16 elsewhere); `DfuLayoutGet`
refuses a layout whose declared unit is not the target's.

For a part without slot 1, `-S` is the slot 0 size plus the header and TLV
room.

`Python/dfu_prod_hex.py --layout <*_dfu_layout.ld> app_signed.bin out.hex`
writes the production hex: the image in slot 1 marked pending where the
layout has slot 1, else the payload in slot 0 with its record. Merge it with
the boot and the SoftDevice.

## Tests

| What | Where |
|---|---|
| Host tests, `make test` in `tests/dfu` | image checks, boot with power loss at every write, DfuMgr, wire protocol with 200000 random frames, SMP server in both modes, BLE transport, serial transport, HTTP download, production hex; memories NOR, RRAM, ECC 16 byte units, 256 byte units, uneven sectors |
| `make wire` | stage 0 recovery driven by `Python/dfu_wire.py` on a pty, a link lost and taken up again |
| `make udp` | the application server driven by `smpclient` over UDP |
| `make serial` | stage 0 recovery with DFU_PROTO_SMP driven by `smpclient` over its serial transport, on a pty |
| `make http` | download from Python's `http.server` with dropped connections |
| Boot machine code | `tests/dfu/boot_emu*.py`: the cross built `DfuBoot` ELF run in an ARM or RISC-V emulator with a model of the memory controller, per family (Nordic, STM32, SAM4, LPC, Renesas) |
| Boot recovery, machine code | `tests/dfu/boot_emu_wire.py`: the nRF52832 `DfuBoot` ELF in the emulator with its UART interrupt, driven by `Python/dfu_wire.py` on a pty: refused images, upload, start, button, application request, power lost during an upload |

On nRF52840 with S140 7.2.0 and Release builds, hardware tests covered
signed USB CDC recovery, recovery entry with Button 1 and return to BLE,
SMP image upload and activation over BLE, downgrade refusal after reconnect,
and an interrupted BLE upload followed by a successful retry. The one-command
`smpmgr upgrade --confirm` installed version 1.3.0. A separate BLE state read
reported it active and confirmed in slot 0.

The interrupted-upload test also showed that old TLVs can survive behind a
new header. The device refused activation with INVALID_HASH. The subsequent
slot 1 bootable-reporting fix passed a host regression. Repeating that
reporting check on hardware and cutting power during the stage 0 install
copy are separate outstanding tests.

LPC11U35 uses its ROM DFU. The software DfuBoot target and slot layouts are
not provided because of its limited memory budget; applications use
ARM/NXP/LPC11xx/ldscript/gcc_lpc11u35.ld and start at flash address zero.
