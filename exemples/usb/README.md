# USB examples

Shared firmware sources live here. Open the matching target project in
IOcomposer under
[ARM/Nordic/nRF52/nRF52840/exemples](../../ARM/Nordic/nRF52/nRF52840/exemples).
Build the nRF52840 IOsonata library first, then clean-build the application
with matching Debug or Release settings.

| Example source | Target project | Purpose |
|---|---|---|
| [usb_cdc_loopback.cpp](usb_cdc_loopback.cpp) | UsbCdcLoopback/ioc | One CDC loopback port |
| [usb_cdc_loopback_taktos.cpp](usb_cdc_loopback_taktos.cpp) | UsbCdcLoopbackTaktOS/ioc | CDC loopback and periodic task under TaktOS |
| [usb_cdc_prbs_tx.cpp](usb_cdc_prbs_tx.cpp) | UsbCdcPrbsTx/ioc | CDC PRBS transmitter |
| [usb_dual_cdc_stress.cpp](usb_dual_cdc_stress.cpp) | UsbDualCdcStress/ioc | CDC loopback and PRBS concurrently |
| [usb_combo_stress.cpp](usb_combo_stress.cpp) | UsbComboStress/ioc | Dual CDC, HID, raw INT and bidirectional ISO |
| [usb_custom_bulk_loopback.cpp](usb_custom_bulk_loopback.cpp) | UsbCustomBulkLoopback/ioc | Vendor Bulk interface |
| [usb_hid_loopback.cpp](usb_hid_loopback.cpp) | UsbHidLoopback/ioc | Vendor HID reports |
| [usb_hid_keyboard.cpp](usb_hid_keyboard.cpp) | UsbHidKeyboard/ioc | Button-driven boot keyboard |
| [usb_hid_3d_mouse.cpp](usb_hid_3d_mouse.cpp) | UsbHid3dMouse/ioc | BMI323 six-axis HID reports |
| [usb_int_loopback.cpp](usb_int_loopback.cpp) | UsbIntLoopback/ioc | Interrupt alternate settings |
| [usb_iso_loopback.cpp](usb_iso_loopback.cpp) | UsbIsoLoopback/ioc | Isochronous alternate settings |
| [usb_msc_ramdisk.cpp](usb_msc_ramdisk.cpp) | UsbMscRamDisk/ioc | Disposable 64 KiB FAT12 RAM disk |

Use the [USB User Guide](../../docs/usb-user-guide.md) for dependency
installation, host commands, buffer ownership and reconnect behavior.
Keyboard and 3D mouse demos require their documented board wiring and sensor;
they are not generic loopback tests.

HID and raw Interrupt examples use separate static, 4-byte-aligned RX/TX
slots sized with `USB_INT_INTRF_PKT_BLKSIZE`. Keep these arrays when adapting
an example; the HID report length does not include the transport header.

USB HCI is implemented by `BtHciUsb`. The maintainer's HciController test
application is not included here; do not substitute a generic USB loopback
for HCI protocol validation.

See [USB with TaktOS](usb_taktos/README.md) for the RTOS build, single-thread
USB ownership, nonblocking transfers and hardware validation procedure.

## Comparing implementations

[TinyUSB comparison](tinyusb_common/README.md) covers the single CDC,
dual CDC and composite baselines. They use IOsonata for the MCU platform and
TinyUSB for the USB stack. Use the same host workload for both implementations
and retain diagnostic counters as well as throughput.

## Host-side checks

From the repository root:

```bash
make -C tests/usb hid-demo-build
make -C tests/usb test ASAN_RUNTIME_OPTIONS=detect_leaks=0:strict_string_checks=1
```

These checks compile against host support and exercise the software stack;
they do not build or run nRF52840 firmware.
