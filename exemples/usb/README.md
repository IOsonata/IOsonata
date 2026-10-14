# USB examples

Shared firmware sources live here. Open the matching target project in
IOcomposer under
[ARM/Nordic/nRF52/nRF52840/exemples](../../ARM/Nordic/nRF52/nRF52840/exemples)
or [ARM/Nordic/nRF54/nRF54LM20x/exemples](../../ARM/Nordic/nRF54/nRF54LM20x/exemples).
Build the matching MCU IOsonata library first, then clean-build the application
with matching Debug or Release settings.

The LM20 set includes all entries below except the SPI-dependent 3D mouse.
See [LM20 project setup](../../docs/usb-nrf54lm20x.md) for available build
configurations, the Cortex-M33 TaktOS library and keyboard wiring.

| Example source | Target project | Purpose |
|---|---|---|
| [usb_cdc_loopback.cpp](usb_cdc_loopback.cpp) | UsbCdcLoopback/ioc | One CDC loopback port |
| [usb_cdc_loopback_taktos.cpp](usb_cdc_loopback_taktos.cpp) | UsbCdcLoopbackTaktOS/ioc | CDC loopback and periodic task under TaktOS |
| [usb_cdc_prbs_tx.cpp](usb_cdc_prbs_tx.cpp) | UsbCdcPrbsTx/ioc | CDC PRBS transmitter |
| [uart_usb_bridge.cpp](uart_usb_bridge.cpp) | UartUsbBridge/ioc | CDC port bridged to a UART |
| [usb_dual_cdc_stress.cpp](usb_dual_cdc_stress.cpp) | UsbDualCdcStress/ioc | CDC loopback and PRBS concurrently |
| [usb_combo_stress.cpp](usb_combo_stress.cpp) | UsbComboStress/ioc | Dual CDC, HID, raw INT and bidirectional ISO |
| [usb_combo_stress_taktos.cpp](usb_combo_stress_taktos.cpp) | UsbComboStressTaktOS/ioc | Composite stress with separate USB service, CDC loopback and PRBS threads |
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

## UART bridge

`UartUsbBridge` connects a UART to a CDC port, in both directions, without
adding anything to the data. The UART rate follows the rate the host sets on
the port; the frame stays 8N1. The UART and its pins are in the board.h of
each project:

| Target project | Board | UART |
|---|---|---|
| nRF52840 | nRF52840 DK | UART0, VCOM0 lines: RX P0.08, TX P0.06 |
| nRF52840 | Nordic Thingy:91 (`NORDIC_THINGY91` in board.h) | UART0 to the nRF9160 UART0: RX P0.11, TX P0.15 |
| nRF54LM20x | nRF54LM20 DK | Debugger serial port 1, UARTE20: RX P1.17, TX P1.16 (`UART_DEVNO 0` for port 0) |
| SAM4LCxC | SAM4L8 Xplained Pro | USART1, EDBG Virtual COM Port: RX PC26, TX PC27 |
| SAM4LSxC | SAM4LS C-package board | USART1: RX PC26, TX PC27 |

On the Thingy:91 the bridge replaces the Connectivity Bridge firmware of the
nRF52840, and the nRF9160 console comes out on the bridge CDC port.

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
