# USB User Guide

IOsonata provides a composable USB device stack with MCU-specific controller
ports. Applications select device classes and provide static storage;
the USB stack assigns interface and endpoint numbers and assembles the
configuration descriptor.

Use this guide to build and test an application. See
[USB architecture](architecture/usb.md) for class ownership, descriptor
composition, endpoint dispatch and controller-port rules.

## Supported functions

| Function | Application class or interface | Shared example | nRF52840 project | Host runner |
|---|---|---|---|---|
| CDC ACM | `UsbdCdc` | `exemples/usb/usb_cdc_loopback.cpp` | `UsbCdcLoopback/ioc` | `Python/usb_cdc_loopback.py` |
| Composite stress | two CDC ports, HID, Interrupt and Isochronous | `exemples/usb/usb_combo_stress.cpp` | `UsbComboStress/ioc` | `Python/usb_combo_stress.py` |
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
- raw USB Bulk, Interrupt and MSC runners use `pyusb` with libusb;
- Isochronous runners use `libusb1` (the Python `usb1` module);
- composite stress uses `pyserial`, `hidapi` and `libusb1` together;
- the HID runner uses `hidapi` and the operating system HID driver.

Linux raw-USB access normally requires an appropriate udev rule or root
privileges. On macOS, a raw MSC test may require `sudo` because the operating
system owns the mass-storage interface. Unmounting a volume does not release
that interface from the kernel driver.

### SAM4L

Build `ARM/Microchip/SAM4L/SAM4LCxC/lib/ioc`, then either `UsbCdcLoopback/ioc`
or `UsbCdcLoopbackTaktOS/ioc` under `SAM4LCxC/exemples`, using matching Debug
or Release configurations. Both projects link the shared CDC application.
For TaktOS, build the sibling `TaktOS/ARM/cm4/ioc` project with its standard
Debug or Release configuration. Its softfp base AAPCS is compatible with the
SAM4L application's software floating point; do not select DebugFPU/ReleaseFPU.

Each project's `src/board.h` supplies `MCUOSC` and pin definitions. The CDC
application defines `g_McuOsc = MCUOSC`, which `SystemInit()` uses to derive
the 48 MHz USB generic clock from PLL0. The current RC clock setup does not
enable USB. The controller checks clock readiness and configures the fixed
DM/DP peripheral pins, PA25/PA26.

The supplied SAM4L8 Xplained Pro configuration uses a 12 MHz crystal, PC11
for VBUS input and PC12 for the host power-switch enable. The application
holds PC12 low and supplies the USB pin map to controller initialization,
which installs the PC11 GPIO callback. GPIO edges queue `UsbProcessQue()`,
and the controller's `UsbCtrlrVbusDetected()` samples the configured pin.
Projects using another external VBUS input change their board pin map.
The Nordic CDC projects use their controller's native cable detection and do
not require these GPIO definitions.

Program through the DEBUG connector, then connect **TARGET USB** to the host.
Check the CDC port assigned to the target; the debugger has a separate port.
Both examples echo received data. The bare-metal example also prints a
greeting when the CDC port opens.

The SAM4L port supports full-speed control, bulk, interrupt and ISO DMA. Eight
physical endpoints provide EP0 plus seven non-control directions; each IN
or OUT data direction uses one endpoint. ISO entry points are linked from
a separate archive member; host operation is unsupported.
The full combo stress workload exceeds this physical endpoint capacity.

Run the controller/platform checks and compile/link checks from the repository
root (`--toolchain-prefix` can select an Arm GNU installation):

```sh
make -C tests/usb test-sam4l
python3 tests/usb/sam4l_cdc_build_test.py --config Release --taktos ../TaktOS
python3 tests/usb/sam4l_cdc_build_test.py --config Debug --taktos ../TaktOS
```

Omit `--taktos` to build just bare metal. The script compiles the examples'
source dependencies and checks project links; it does not build unrelated
peripherals in the library project. Maintainer CDC loopback, ISO packet-size
and ISO suspend/wake results are recorded in the [0.13 notes](releases/0.13.md).
For final-release validation, check enumeration, short/64-byte binary echo,
backpressure, repeated CDC open/close, unplug/replug while EDBG remains powered,
and host suspend/resume. For TaktOS, also check that `g_UsbTaktOSHeartbeat`
advances during traffic.

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

    // A board can start without VBUS. The stack connects when it appears.
    (void)UsbEnable(s_UsbCfg.DevNo);

    // Runs the queued USB work and waits for the next interrupt. An
    // application with its own loop calls AppEvtHandlerExec() in it instead.
    AppRun();
}
```

Initialization order is significant:

1. `UsbInit()` records device identity and initializes the controller.
2. Each class `Init()` allocates its topology, initializes its data path and
   registers its descriptor fragment or speed-aware descriptor builder.
3. `UsbEnable()` prepares and validates the complete configuration descriptor
   before connecting the controller.
4. `AppRun()`, or `AppEvtHandlerExec()` in the application loop, runs what
   the USB stack queued.

The USB stack hands everything that must run outside the interrupt to
`UsbEvtQue()`, the one way it signals work: deferred endpoint events, and one
process event after controller and endpoint events, which runs the class
work (`UsbDeviceClass::Process()`), reports cable changes and retries the
connection. Its library default puts the work in the application event
queue, first in first out. The USB stack never runs the queue itself, so USB
and Bluetooth share it in one loop, and the application does not call
`UsbProcess()`. The default queue holds 4 events. A USB application normally needs more: define
`g_AppEvtHandlerQueMem` and pass its size to `AppEvtHandlerInit()` before
`UsbInit()`, as the USB examples do:

```cpp
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
```

With an RTOS, the thread serving USB owns the deferred work instead: the
application overrides `UsbEvtQue()` to put it in that thread's queue as a
message, and the thread runs each one. The application event queue is then
not linked for USB.

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

`UsbdHid` inherits `UsbIntIntrf` and owns HID descriptors and standard HID class
requests. The application supplies the report descriptor and report meaning.
Subclass `UsbdHid` only when application-specific control reports are needed,
and delegate unhandled requests to `UsbdHid::Control()`.

Supply separate caller-owned RX and TX slots in `UsbdHidCfg_t::pRxBuffer`
and `pTxBuffer`. Both must remain alive for the interface lifetime, be
4-byte aligned and contain at least `USB_INT_INTRF_PKT_BLKSIZE` bytes.
This size includes the transport packet header; a 64-byte report array alone
is not sufficient.

```cpp
alignas(4) static uint8_t s_HidRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
```

Use these arrays in the matching configuration fields, as shown in
[usb_hid_loopback.cpp](../exemples/usb/usb_hid_loopback.cpp).
`UsbIntIntrfCfg_t` has the same slot requirements. Rebuild the MCU library
and application together when migrating from the earlier embedded-buffer API.

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
every storage sector fits. Disk reads, writes and SCSI command work run from
the queued process event, outside the USB interrupt.

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

## Bluetooth HCI over USB

`BtHciUsb` supplies the USB device transport for a Bluetooth controller.
It combines HCI class requests and the HCI data path with an optional SCO
Isochronous endpoint pair. This is separate from the IOsonata Bluetooth
host stack and its pairing/security configuration.

Use [bt_hci_usb.h](../include/bluetooth/bt_hci_usb.h) for the transport
configuration. The maintainer validated the HCI/MSC storage optimization
with HciController for HCI and UsbMscRamDisk for MSC. HciController is not
included as a target project in this repository.

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
successful class registration contributes its interfaces, endpoints and
descriptor fragment or builder to the same configuration. The allocator prevents
fixed endpoint assumptions from leaking into reusable application code.

Stop startup on any class `Init()` failure; do not continue to `UsbEnable()`.
The controller is connected only after the complete configuration descriptor
is prepared and validated.

### Configuration descriptor storage

The nRF52840 library defaults to `USB_CONFIG_DESC_MAXLEN = 768` bytes.
The generic fallback for other targets is 1024 bytes; these are storage
capacities, not the descriptor length reported to the host. UsbComboStress
uses 398 bytes, including the configuration header and all alternate settings.

Device, string, qualifier and HID report descriptor bodies are separate from
this configuration buffer. The 768-byte nRF52840 default covers the existing
class layouts considered in the endpoint-capacity review, including HCI and
MSC; custom descriptors or extra alternate settings can require more.

To change the capacity, define `USB_CONFIG_DESC_MAXLEN` when building the MCU
library, then clean and rebuild the application against matching headers and
library. An application-only definition cannot resize storage in a precompiled
library.

### Composite stress runner

UsbComboStress runs CDC loopback, CDC PRBS TX, HID, raw Interrupt and
bidirectional Isochronous traffic concurrently. Install the host dependencies:

```bash
python3 -m venv .venv
./.venv/bin/python3 -m pip install pyserial hidapi libusb1
./.venv/bin/python3 Python/usb_combo_stress.py \
  --loop-port /dev/cu.usbmodemXXXX01 \
  --prbs-port /dev/cu.usbmodemXXXX03 \
  --duration 2000
```

The host also needs the native libusb library. Replace both serial paths with
the ports assigned to the current firmware. Save the full result, including
pending loopback bytes, ISO diagnostics and any failure text, rather than only
the throughput line. A runner PASS can coexist with nonzero ISO host misses,
skews or unsent frames; it does not mean all diagnostic counters were zero.

The runner resubmits an ISO burst that the host did not carry out: a request
refused at submit or failed as a whole, validation frames the host reports as
not sent, or an OUT transfer started late against IN. These count as host
misses or skews, and only more than three in a row fail the run. OUT guard
frames the host reports as not sent count as host unsent frames; the burst
is still validated. A failure up to the transfer timeout plus one second after
a host pause, or up to ten seconds after a system sleep, is reported as
INCONCLUSIVE instead of FAIL.

See the [USB example index](../exemples/usb/README.md) and
[TinyUSB comparison procedure](../exemples/usb/tinyusb_common/README.md).

## TaktOS integration

The [USB + TaktOS example](../exemples/usb/usb_taktos/README.md) includes an
nRF52840 IOcomposer project. One thread services USB and nonblocking CDC
loopback; a lower-priority periodic thread demonstrates scheduler progress.
The example overrides `UsbEvtQue()` so the USB thread runs the deferred
endpoint work from its own queue; the application event queue is not used.
Follow its partial-write handling and bounded service passes when adapting it.

`UsbComboStressTaktOS` adds separate CDC loopback and PRBS threads alongside
one USB service thread and a heartbeat. Its device composition and host runner
match `UsbComboStress`; HID/INT/ISO retain their callback paths. The integration
guide explains thread priorities, USB work ownership and the hardware checks.

## Suspend, reset and reconnect

Keep the queue running (`AppRun()`, the application loop or the USB thread).
The USB port owns its cable interrupt: VREGUSB on the nRF54LM20, the POWER
USBDETECTED and USBREMOVED events on the nRF52 (through the SoftDevice SoC
events when a SoftDevice is enabled, otherwise on the POWER_CLOCK vector it
shares with the clock). A cable edge queues the process event, which reports
the change, runs class work and retries device connection when a board
started without a cable. On VBUS
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

Keep the queue running and rediscover the host device path. Serial ports
and BSD disk numbers are host-assigned and can change. For MSC, removal of VBUS
reloads a removable medium; a logical eject without VBUS removal intentionally
keeps it not-ready until the host sends a load request.

### More than one matching test device is attached

Pass `--serial` to runners that support it. The raw runners discover interface
and endpoint addresses from descriptors instead of assuming endpoint 1.
