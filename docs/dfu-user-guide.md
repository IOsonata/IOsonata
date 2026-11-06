# Firmware Update User Guide

On an nRF52840 running the IOsonata `BleOta` application, update firmware over
BLE with one command. Give it the device identifier and the signed release file:

```sh
smpmgr --ble "$ble" upgrade --confirm "$signed_image"
```

The application receives the file in slot 1 and marks it pending. The command
then resets the board. On restart, `DfuBoot` verifies the signature, copies
the payload into slot 0 and starts the application. This is an overwrite
install. A working USB recovery bootloader gives you a way back if the new
application fails to advertise.

Once the board advertises again, verify the running version over BLE:

```sh
smpmgr --ble "$ble" image state-read
```

Look for the expected version in **slot 0** with **active=True**. A reset
acknowledgment alone only tells you that the previous application received
the reset request.

## Prepare a signed release

These commands use the nRF52840/S140 example and run from the IOsonata
repository root on macOS. Build the nRF52840 library, then the
`exemples/BleOta/ioc` project under that target, both in **Release** in
IOcomposer. Build the actual firmware changes before assigning a new version.

Use a Python virtual environment and install the host tools once:

```sh
python3 -m venv ../.venv
. ../.venv/bin/activate
python -m pip install imgtool smpmgr bleak
```

In subsequent terminals, activate the same environment. Set `version` to the
release you are building. If IOcomposer finds objcopy but the shell cannot,
set `objcopy` to the full path to the executable in the installed Arm xPack.

```sh
target=ARM/Nordic/nRF52/nRF52840
app="$target/exemples/BleOta/ioc/Release"
version=1.3.0
signed_image="$app/BleOta-$version.signed.bin"
objcopy=arm-none-eabi-objcopy

"$objcopy" -O binary "$app/BleOta.elf" "$app/BleOta.bin" &&
imgtool sign --key exemples/dfu/dfu_dev_key.pem --version "$version" \
  --header-size 0x20 --pad-header --align 4 --slot-size 0x65FE0 \
  "$app/BleOta.bin" "$signed_image"
```

The example boot trusts the public key paired with the repository's
development key. For a product, keep the signing key private and build
`DfuBoot` with its matching public key. Here `0x65FE0` is the nRF52840
slot 1 capacity after reserving its two 16-byte trailer units. Slot 0 limits
the application payload to 400 KiB.

## Provision a board and install its first application

1. Build the nRF52840 `exemples/DfuBoot/ioc` project in **Release** against
   the rebuilt Release MCU library.
2. For initial provisioning, program the Nordic SDK
   `components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex` and
   `$target/exemples/DfuBoot/ioc/Release/DfuBoot.hex` together. S140 includes
   the MBR. The boot HEX supplies the UICR boot address, `0xF4000`.
   A full-chip erase also clears application data and bonds.
3. Reset and connect native USB. A blank slot 0 enters recovery automatically.
   For a board with a valid application, hold Button 1 (P0.13 on the supplied
   IBK configuration) through reset, then release it. Find the CDC port with
   `python -m serial.tools.list_ports -v` and look for `1915:5300`.
4. Set `port` to the device path shown on your Mac, then upload the signed
   file directly to slot 0:

```sh
port=/dev/cu.usbmodem142301   # replace with your board's port
smpmgr --port "$port" --timeout 30 image upload --slot 0 "$signed_image" &&
smpmgr --port "$port" image state-read
```

With the bootloader running, a successful upload shows the signed version
and hash, `bootable=True` and `active=False`. Release Button 1 and run
`smpmgr --port "$port" os reset`. The example starts S140 and advertises as
`IOsonataOTA`. This same USB procedure recovers a board whose application
cannot start BLE.

## Find the BLE device on macOS

Disconnect any phone already using the example. With the board advertising,
obtain its identifier on the Mac that will perform the update:

```sh
ble=$(python - <<'PY'
import asyncio
from bleak import BleakScanner

device = asyncio.run(BleakScanner.find_device_by_name("IOsonataOTA", timeout=15))
if device is None:
    raise SystemExit("IOsonataOTA not found")
print(device.address)
PY
) && smpmgr --ble "$ble" image state-read
```

Slot 0 should now be active. For the next update, prepare a newer signed
release and use the one-command BLE upgrade above. Leave the upload slot at
its default: the SMP image number is 0, and the running application selects
physical slot 1 for staging.

## Storage and interrupted updates

| Situation | Device behavior |
|---|---|
| BLE upload loses power | Slot 0 can start again. Repeat the upload to replace the partial candidate. A parseable partial image can appear in slot 1; current firmware checks its contents before reporting it bootable and rejects activation on a hash mismatch. |
| An older signed image is uploaded | With downgrade disabled, upload completion and a later mark-by-hash request both check the version. Bytes can remain visible in slot 1 with `pending=False`. |
| Power fails during installation | The bootloader uses the pending image in slot 1 to retry the copy on restart. Host tests exercise this case; physical power loss during copying remains to be tested on nRF52840. |
| Direct USB recovery is interrupted | The old application may already have been overwritten. Re-enter USB recovery and repeat the upload. |

Full-image application-side OTA needs a complete running image and enough
separate storage for the signed replacement. The native nRF52840/S140 layout
provides 400 KiB for slot 0 and 408 KiB for slot 1. The nRF52832 xxAA/S132
layout provides 148 KiB and 152 KiB respectively. A 600 KiB payload exceeds
both native layouts. A 300 KiB payload fits the nRF52840 slots but exceeds
the nRF52832 slots. Include any linked SDC code when measuring the payload.

The native nRF52832 `DfuBoot` uses UART and `Python/dfu_wire.py` at 115200.
The native nRF52840 example uses SMP over USB CDC. The Nordic SDK
`dfu_ble` and `dfu_usb` projects use their own layouts and packages.
See [DFU architecture](architecture/dfu.md) and the
[DfuWire protocol](architecture/dfu_wire.md).
