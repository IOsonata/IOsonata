# USB User Guide

IOsonata provides a composable USB device stack for the nRF52840 native USB
controller. Applications select device classes and provide static storage;
the USB stack assigns interface and endpoint numbers and assembles the
configuration descriptor.

Use this guide to build and test an application. See
[USB architecture](architecture/usb.md) for class ownership, descriptor
composition, endpoint dispatch and controller-port rules.

## Supported functions

| Function | Application class or interface | Shared example | nRF52840 project | Host runner |
|---|---|---|---|---|
| CDC ACM | `UsbdCdc` | `exemples/usb/usb_cdc_loopback.cpp` | `UsbCdcLoopback/ioc` | `Python/usb_cdc_loopback.py` |
| Dual CDC ACM | two `UsbdCdc` objects | `exemples/usb/usb_dual_cdc_stress.cpp` | `UsbDualCdcStress/ioc` | `Python/usb_dual_cdc_stress.py` |
| Custom Bulk | `UsbdBulk` | `exemples/usb/usb_custom_bulk_loopback.cpp` | `UsbCustomBulkLoopback/ioc` | `Python/usb_custom_bulk_loopback.py` |
| HID | `UsbdHid` | `exemples/usb/usb_hid_loopback.cpp` | `UsbHidLoopback/ioc` | `Python/usb_hid_loopback.py` |
| Mass Storage | `UsbdMsc` | `exemples/usb/usb_msc_ramdisk.cpp` | `UsbMscRamDisk/ioc` | `Python/usb_msc_test.py` |
| Interrupt transport | `UsbIntIntrf` | `exemples/usb/usb_int_loopback.cpp` | `UsbIntLoopback/ioc` | `Python/usb_int_loopback.py` |
| Isochronous transport | `UsbIsoIntrf` | `exemples/usb/usb_iso_loopback.cpp` | `UsbIsoLoopback/ioc` | `Python/usb_iso_loopback.py` |

The target projects are below
`ARM/Nordic/nRF52/nRF52840/exemples/`. Interrupt and Isochronous loopback
projects validate reusable transports; they are not standard USB device
classes. Bluetooth HCI over USB is provided by `BtHciUsb` for Bluetooth
controller applications.

## Build and run an example

1. Build the nRF52840 IOsonata library as described in
   [Getting Started](getting-started.md).
2. In IOcomposer, open the selected project's `ioc/` directory.
3. Build the matching Debug or Release configuration.
4. Flash the nRF52840 and connect its native USB port to the host.
5. Run the corresponding host runner from the repository root.

The Python runners require only the host packages used by their transport:

- CDC runners use `pyserial`;
- raw USB Bulk, Interrupt, Isochronous and MSC runners use `pyusb` with a
  libusb backend;
- the HID runner uses `hidapi` and the operating system HID driver.

Linux raw-USB access normally requires an appropriate udev rule or root
privileges. On macOS, a raw MSC test may require `sudo` because the operating
system owns the mass-storage interface. Unmounting a volume does not release
that interface from the kernel driver.

## Common application lifecycle

Include the generic USB header and the selected device-class header. Keep the
USB configuration, class objects, FIFO memory, report descriptors and storage
buffers in static or caller-owned storage.

```cpp
static UsbdBulk s_Bulk;

int main(void)
{
    if (!UsbInit(&s_UsbCfg) || !s_Bulk.Init(s_BulkCfg))
    {
        return -1;
    }

    // A board can start without VBUS. UsbProcess() reconnects when it appears.
    (void)UsbEnable(s_UsbCfg.DevNo);

    while (1)
    {
        UsbProcess(s_UsbCfg.DevNo);
        // Application work.
    }
}
```

Initialization order is significant:

1. `UsbInit()` records device identity and initializes the controller.
2. Each class `Init()` allocates its topology, initializes its data path and
   registers its full-speed and high-speed descriptor fragments.
3. `UsbEnable()` prepares and validates the complete configuration descriptor
   before connecting the controller.
4. `UsbProcess()` runs deferred class work and handles VBUS reconnects.

Do not assign interface or endpoint numbers in application configuration.
Adding or removing a class can change the assigned topology without changing
the class application code.

## Device identity and strings

`UsbCfg_t` supplies the device VID, PID, version, power attributes and strings.
The example VID/PID values identify test firmware. Use identifiers assigned to
the product before shipping it.

Each nonzero interface string index used by a class must correspond to a
string registered by the device configuration. Follow a complete example when
adding strings or composing several functions.

## CDC ACM

`UsbdCdc` presents a serial data interface. Start with `UsbCdcLoopback` for one
port or `UsbDualCdcStress` for a composite device with two independent ports.
The host assigns serial-device paths dynamically; discover the current paths
after every reconnect.

Run a single-port loopback with the actual host port:

```bash
./.venv/bin/python3 Python/usb_cdc_loopback.py --port /dev/cu.usbmodemXXXX
```

Run the dual-port stress test with both assigned ports:

```bash
./.venv/bin/python3 Python/usb_dual_cdc_stress.py \
  --loop-port /dev/cu.usbmodemXXXX01 \
  --prbs-port /dev/cu.usbmodemXXXX03 \
  --duration 60
```

## Custom Bulk

`UsbdBulk` supplies one vendor-class interface with a Bulk OUT endpoint and a
Bulk IN endpoint. The application configures subclass, protocol, interface
string, transfer mode and caller-owned RX/TX FIFO memory.

```cpp
#include "cfifo.h"
#include "usb/usb.h"
#include "usb/usbd_bulk.h"

#define BULK_RX_MEM_SIZE USBD_BULK_RXMEM_SIZE(4)
#define BULK_TX_MEM_SIZE CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_RxMem[BULK_RX_MEM_SIZE];
alignas(4) static uint8_t s_TxMem[BULK_TX_MEM_SIZE];
static UsbdBulk s_Bulk;

static const UsbdBulkCfg_t s_BulkCfg = {
    .DevNo = 0,
    .bBlocking = true,
    .RxFifoMemSize = sizeof(s_RxMem),
    .pRxFifoMem = s_RxMem,
    .TxFifoMemSize = sizeof(s_TxMem),
    .pTxFifoMem = s_TxMem,
    .SubClass = 0,
    .Protocol = 0,
    .InterfaceString = 4,
    .FsMps = 0,
    .HsMps = 0,
    .Mode = USBD_BULK_MODE_BYTE,
    .EvtCB = nullptr,
};
```

Use `USBD_BULK_MODE_BYTE` for a byte stream. Use
`USBD_BULK_MODE_PACKET` with storage sized by `USBD_BULK_TXMEM_SIZE()` when
each application write must remain one USB packet, including a zero-length
packet. RX preserves USB packet boundaries internally in either mode.

Derive from `UsbdBulk` and override `Control()` for vendor endpoint-zero
requests. Delegate unhandled requests to `UsbdBulk::Control()`.

Run the descriptor-discovering loopback test with:

```bash
./.venv/bin/python3 Python/usb_custom_bulk_loopback.py
```

Use `--serial` when several matching devices are connected.

## HID

`UsbdHid` embeds `UsbIntIntrf` and owns HID descriptors and standard HID class
requests. The application supplies the report descriptor and report meaning.
Subclass `UsbdHid` only when application-specific control reports are needed,
and delegate unhandled requests to `UsbdHid::Control()`.

The generic loopback is exercised through the native HID driver:

```bash
./.venv/bin/python3 Python/usb_hid_loopback.py
```

Two application demos show report semantics above the reusable class:

- `UsbHidKeyboard/ioc` is a boot keyboard for the I-SYST IBK-NRF52840.
  Button 1 sends `A`; Button 2 sends Caps Lock; LED 1 follows the Caps Lock
  output report.
- `UsbHid3dMouse/ioc` maps Bosch BMI323 acceleration and angular rate to a
  six-axis Generic Desktop Multi-axis Controller report. A desktop does not
  necessarily translate this report into pointer motion; use a compatible 3D
  application or HID report monitor.

The host compile check for both demos is:

```bash
make -C tests/usb hid-demo-build
```

## Mass Storage

`UsbdMsc` implements one Mass Storage Bulk-Only Transport interface, the SCSI
transparent command set and one LUN. The application supplies a statically
owned `DiskIO` object and a sector buffer. The class does not own the medium.

```cpp
static MyDiskIO s_Disk;
alignas(4) static uint8_t s_Sector[512];
static UsbdMsc s_Msc;

static const UsbdMscCfg_t s_MscCfg = {
    .DevNo = 0,
    .pDisk = &s_Disk,
    .pSectorBuffer = s_Sector,
    .SectorBufferSize = sizeof(s_Sector),
    .bReadOnly = false,
    .bRemovable = true,
    .InterfaceString = 4,
    .FsMps = 0,
    .HsMps = 0,
    .pVendor = "I-SYST",
    .pProduct = "IOsonata Disk",
    .pRevision = "1.00",
};
```

The sector buffer must be at least `DiskIO::GetSectSize()` bytes. Initialization
rejects an invalid or oversized sector configuration rather than assuming that
every backend sector fits. Disk reads, writes and SCSI command work run from
`UsbProcess()`, outside the USB interrupt.

The initial SCSI command set includes INQUIRY, TEST UNIT READY, REQUEST SENSE,
READ CAPACITY (10), MODE SENSE (6), START STOP UNIT, READ (10), WRITE (10),
PREVENT/ALLOW MEDIUM REMOVAL, VERIFY (10) without compare data, and
SYNCHRONIZE CACHE.

`UsbMscRamDisk` exposes a dedicated 64 KiB FAT12 RAM disk. It does not expose
firmware, settings, a production filesystem or Bluetooth bond storage. RAM-disk
contents are lost when USB bus power is removed.

Unmount the volume before running the raw write test:

```bash
diskutil unmountDisk /dev/diskN       # macOS
sudo ./.venv/bin/python3 Python/usb_msc_test.py --write-test
```

Substitute the exact disk reported for the test device. The runner overwrites,
hash-verifies and restores its test range, then checks logical eject/reload,
BOT reset and repeated commands. Never use `--write-test` with firmware that
maps MSC to firmware, shared storage or another medium whose contents are not
disposable or backed up.

A logical eject remains effective across BOT and USB bus reset. Removing and
restoring VBUS reloads the configured removable medium. A host can remount it
with Disk Utility or `diskutil mountDisk /dev/diskN`; the BSD disk number can
change after reconnect.

## Interrupt and Isochronous transports

`UsbIntIntrf` and `UsbIsoIntrf` are role-neutral endpoint-pair transports used
to build a USB class. Their loopback firmware deliberately uses a vendor
interface so the endpoint behavior can be tested directly.

```bash
./.venv/bin/python3 Python/usb_int_loopback.py
./.venv/bin/python3 Python/usb_iso_loopback.py
```

Both runners provide a `--manual-suspend-wake` phase. The Interrupt runner
checks alternate settings with different polling intervals. The Isochronous
runner checks scheduled packet flow and diagnostic counters. Use
`Python/usb_iso_diag.py` for focused isochronous endpoint diagnosis.

## Composite devices

Initialize every class object after `UsbInit()` and before `UsbEnable()`. Each
successful class registration contributes its interfaces, endpoints and static
descriptor fragment to the same configuration. The allocator prevents fixed
endpoint assumptions from leaking into reusable application code.

Class initialization is atomic: do not continue to `UsbEnable()` after a class
`Init()` failure. The controller is connected only after the complete
configuration descriptor is prepared and validated.

## Suspend, reset and reconnect

Call `UsbProcess()` continuously. It observes VBUS changes, runs class work and
retries device connection when a board started without a cable. On VBUS
removal, the core calls each device class `Detach()` before reporting the cable
event. Bus reset and unconfiguration call each class `Reset()` and close active
non-control endpoints.

Some applications also act on `UsbSuspended()`. For example, the HID loopback
calls `Suspend()` and `Resume()` on its class when the USB suspend state
changes. Follow the selected example when the class or product has explicit
low-power behavior.

## Host and sanitizer tests

Run the USB host suite with AddressSanitizer and UndefinedBehaviorSanitizer:

```bash
make -C tests/usb clean test \
  ASAN_RUNTIME_OPTIONS=detect_leaks=0:strict_string_checks=1
```

Hardware runners are separate because they require flashed firmware and a
physical USB host connection. The workflow in
`.github/workflows/usb-host-tests.yml` records the host-only regression paths.

## Troubleshooting

### The device does not enumerate

- Confirm that the nRF52840 library and application use matching Debug or
  Release configurations.
- Check every return from `UsbInit()` and class `Init()`.
- Initialize all classes before calling `UsbEnable()`.
- Use the target's native USB data port, not only its debug-probe USB port.
- Confirm VBUS and cable data connectivity.

### A raw PyUSB test reports access denied

Run through an appropriate Linux udev rule or with the privileges needed to
claim the interface. On macOS MSC, first unmount the correct disk to protect
the filesystem, then run the raw test with `sudo`. Unmounting alone does not
detach the kernel mass-storage driver.

### A device does not reappear after reconnect

Keep `UsbProcess()` running and rediscover the host device path. Serial ports
and BSD disk numbers are host-assigned and can change. For MSC, removal of VBUS
reloads a removable medium; a logical eject without VBUS removal intentionally
keeps it not-ready until the host sends a load request.

### More than one matching test device is attached

Pass `--serial` to runners that support it. The raw runners discover interface
and endpoint addresses from descriptors instead of assuming endpoint 1.
