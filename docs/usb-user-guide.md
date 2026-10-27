# USB User Guide

IOsonata provides a composable USB device stack with MCU-specific controller
ports. Applications select device classes and provide static storage;
the USB stack assigns interface and endpoint numbers and assembles the
configuration descriptor.

Use this guide to build and test an application. See
[USB architecture](architecture/usb.md) for class ownership, descriptor
composition, endpoint dispatch and controller-port rules.

## Configuration defaults

The examples use aggregate initializers: omitted fields are zero-initialized.
Applications only need to list required values and choices that differ from
those defaults. There is no generated application configuration behind them.

For `UsbCfg_t`, an omitted `pSerial` uses the MCU unique ID; omitted power flags
select bus power with remote wakeup disabled. The CDC loopback and PRBS examples
explicitly retain their remote-wakeup setting. Omitted pin-map fields are null
and zero; targets needing USB pin configuration still supply `USB_PINS` through
`board.h`. Device identity, power budget and interrupt priority remain explicit.

For `UsbdHidCfg_t`, zero selects the documented HID version and polling defaults
(1 ms full speed, high-speed bInterval 4). Zero subclass/protocol selects ordinary
non-boot HID. Buffer pointers, report definitions and application callbacks must
still be supplied where needed. The keyboard and mouse helpers select their
predefined report layouts. Packet sizes remain explicit in the custom HID
loopback so its 64-byte host protocol is visible in the application.

## Supported functions

| Function | Application class or interface | Shared example | nRF52840 project | Host runner |
|---|---|---|---|---|
| CDC ACM | `UsbdCdc` | `exemples/usb/usb_cdc_loopback.cpp` | `UsbCdcLoopback/ioc` | `Python/usb_cdc_loopback.py` |
| Composite stress | two CDC ports, HID, Interrupt and Isochronous | `exemples/usb/usb_combo_stress.cpp` | `UsbComboStress/ioc` | `Python/usb_combo_stress.py` |
| Dual CDC ACM | two `UsbdCdc` objects | `exemples/usb/usb_dual_cdc_stress.cpp` | `UsbDualCdcStress/ioc` | `Python/usb_dual_cdc_stress.py` |
| Custom Bulk | `UsbdBulk` | `exemples/usb/usb_custom_bulk_loopback.cpp` | `UsbCustomBulkLoopback/ioc` | `Python/usb_custom_bulk_loopback.py` |
| HID | `UsbdHid` | `exemples/usb/usb_hid_loopback.cpp` | `UsbHidLoopback/ioc` | `Python/usb_hid_loopback.py` |
| Mass Storage | `UsbdMsc` | `exemples/usb/usb_msc_ramdisk.cpp` | `UsbMscRamDisk/ioc` | `Python/usb_msc_test.py` |
| Vendor Interrupt | `UsbdInt` | `exemples/usb/usb_int_loopback.cpp` | `UsbIntLoopback/ioc` | `Python/usb_int_loopback.py` |
| Vendor Isochronous | `UsbdIso` | `exemples/usb/usb_iso_loopback.cpp` | `UsbIsoLoopback/ioc` | `Python/usb_iso_loopback.py` |

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

### NXP LPC

LPC11U35, LPC1769 and LPC54628 provide `UsbCdcLoopback`, `UsbCdcPrbsTx`,
`UsbHidLoopback`, `UsbCustomBulkLoopback` and `UsbCompositeLoopback` projects.
Build the MCU library and the application's `ioc` project in the same
configuration. Debug uses rdimon semihosting; Release uses nosys.

The composite application uses one CDC function, HID and vendor bulk, with
shared source in
[usb_composite_loopback.cpp](../exemples/usb/usb_composite_loopback.cpp).
Its PyUSB/libusb runner claims all interfaces and interleaves payload checks:

```sh
python3 Python/usb_composite_loopback.py --cycles 100 --reconfigure
```

Use `--serial` when multiple matching boards are attached. The runner needs
permission to claim CDC, HID and vendor interfaces; a kernel driver must be
releasable on the host. It checks integrity and repeated configuration, not
throughput. This is a smaller composition than the Nordic CDC/HID/INT/ISO
stress workload.

LPC11U35 and LPC54628 share the full-speed EP0/data implementation; LPC1769
uses its separate SIE/CPU driver. The generic allocator selects endpoints
compatible with each class's transfer type. LPC1769 requires VBUS at startup,
the board's SoftConnect circuit and a valid USB clock; reset after cable
reattachment. Automatic reattachment and remote wake are unsupported there.
LPC11U35/LPC54628 support remote wake with clocks kept on during suspend.
ISO, USB host and low-power suspend are outside this LPC baseline.

Host models and selected Debug/Release links pass; physical board testing
remains pending. See [NXP port notes](../ARM/NXP/README.md) and
[LPC USB validation](../tests/lpc/usb/README.md) for memory use and limits.

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
Use optional `GetReportCB` and `SetReportCB` callbacks for control reports.
No subclass or endpoint-zero transfer-stage handler is needed. Existing
`Control()` overrides remain supported for specialized requests.

For a standard six-key boot keyboard, use `UsbdHid::InitKeyboard()`:

```cpp
static UsbdHid s_Hid;
static UsbHidKeyboardReport_t s_Report = {};
alignas(4) static uint8_t s_Rx[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_Tx[USB_INT_INTRF_PKT_BLKSIZE];
static const UsbdHidCfg_t s_HidCfg = {
	.EvtCB = HidEvent,
	.pRxBuffer = s_Rx,
	.pTxBuffer = s_Tx,
	.GetReportCB = HidGetReport,
	.SetReportCB = HidSetReport,
};
// After UsbInit(), before UsbEnable():
if (!s_Hid.InitKeyboard(s_HidCfg))
{
	return -1;
}
```

The library supplies the report descriptor, boot subclass, keyboard protocol,
and eight-byte full/high-speed endpoint packet sizes. It replaces those six
configuration fields in a local copy; the caller's configuration is unchanged.
Other options retain their normal meaning, including controller number,
callbacks, context, strings and intervals. Zero intervals select 1 ms at full
speed and high-speed bInterval 4 (also 1 ms). No heap or extra object storage is
used. `HidEvent`, `HidGetReport` and `HidSetReport` are application callbacks;
the complete example is `exemples/usb/usb_hid_keyboard.cpp`.

The report has modifier bits, a zero reserved byte and six HID key usages.
Send all zeros to release the keys. The host's one-byte output report contains
the five keyboard LED bits. There is no report ID; this fixed layout works in
both boot and report protocol. The application still owns key scanning, report
submission and LED behavior. This helper does not add idle-repeat scheduling.

Custom layouts continue to use `Init()` with an application-defined report
descriptor. `USB_HID_KEYBOARD_REPORT_DESC` remains available for applications
that want to use the standard report definition with their own configuration.

For a three-button desktop mouse, use `InitMouse(s_HidCfg)` with
`UsbHidMouseReport_t`. It supplies a three-byte boot mouse report: button bits
0/1/2 (left/right/middle), then signed relative X/Y movement (-127..127).
The upper button bits must be zero. There is no wheel, output report or report
ID. Boot and report protocol use the same layout.

For a six-axis controller, use `Init3dMouse(s_HidCfg)` with
`UsbHid3dMouseReport_t`. This preserves the BMI323 example's twelve-byte report:
signed relative X/Y/Z/Rx/Ry/Rz (-32767..32767), little-endian on USB. It has no
buttons, output report or report ID and does not advertise boot mouse protocol.
Both mouse helpers replace the same six layout fields as `InitKeyboard()`.
The existing interrupt IN/OUT endpoint pair is retained; OUT reports are not
part of these mouse report definitions. Unused control callbacks may be omitted.

```cpp
// After successful InitMouse(), submit movement through the normal transport.
UsbHidMouseReport_t report = {0, 10, -5};
const int sent = s_Hid.Tx(0,
	reinterpret_cast<const uint8_t *>(&report), sizeof(report));
// Retain the movement in application state if sent != sizeof(report).
```

Relative movement must be submitted once per accepted sample; repeatedly sending
the same delta repeats the movement. Sensor sampling, motion accumulation and
button handling remain application-specific. The descriptor initializer macros
`USB_HID_MOUSE_REPORT_DESC` and `USB_HID_3D_MOUSE_REPORT_DESC` are also available
with `Init()` for applications needing a different endpoint configuration.

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

For example, an unnumbered input report can be returned with:

```cpp
static int GetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t Capacity, void *)
{
	if (Type != USB_HID_REPTYPE_INPUT || ReportId != 0 ||
		Capacity < sizeof(s_Report))
	{
		return -1;
	}
	memcpy(pBuffer, &s_Report, sizeof(s_Report));
	return sizeof(s_Report);
}
```

Set `.GetReportCB = GetReport` in `UsbdHidCfg_t`. For output or feature
reports, set `.SetReportCB` to a function with this signature:

```cpp
static bool SetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	const uint8_t *pBuffer, uint16_t Length, void *pContext);
```

The library assembles the complete SET_REPORT payload before calling the
function. Return `true` to accept it or `false` to stall the request. Short
or oversized transfers are rejected without calling the function. An absent
callback stalls that request. Both callbacks receive the configuration's
`pContext` and run in USB processing context; keep them short and nonblocking.
The report type uses the `USB_HID_REPTYPE_*` constants. Report bytes are passed
unchanged, including any report ID byte; the library does not interpret the
report descriptor or add/remove IDs.

Each object contains a 64-byte control buffer, separate from the interrupt
RX/TX slots. GET_REPORT copies a stable snapshot into it; the core truncates
the reply to the host's requested length. For larger control reports, provide
`.pControlBuffer` and `.ControlBufferSize`: separate, 4-byte aligned storage
that remains valid for the object's lifetime and does not overlap RX/TX slots.
No heap allocation is used. SET_REPORT callbacks run before the status stage,
so they can reject a report; a later status failure cannot undo application
effects already applied by the callback. Interrupt OUT reports still arrive
through `EvtCB` or the transport read API.

Rebuild the MCU library and application together for the added configuration
fields and object storage. Zero-initialize configurations before assigning
individual fields.

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

## Interrupt and Isochronous functions

Use `UsbdInt` and `UsbdIso` for vendor-specific USB device functions. They
allocate one interface and one bidirectional endpoint pair, generate alternate
zero with no endpoints, and generate active alternates from a const table.
`UsbIntIntrf` and `UsbIsoIntrf` remain role-neutral transports underneath them.

For Interrupt, provide separate aligned direct slots and the polling choices:

```cpp
#include "usb/usbd_int.h"

static const UsbdPeriodicAlt_t s_Alternates[] = {
	// FS bytes, HS bytes, FS bInterval, HS bInterval
	{64, 64, 1, 1},
	{64, 64, 4, 4},
	{64, 64, 16, 16},
};
alignas(4) static uint8_t s_Rx[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_Tx[USB_INT_INTRF_PKT_BLKSIZE];
static UsbdInt s_Int;
static const UsbdIntCfg_t s_IntCfg = {
	.Function = {0, s_Alternates, 3, 0, 0, 4},
	.EvtCB = IntEvent,
	.pContext = nullptr,
	.pRxBuffer = s_Rx,
	.pTxBuffer = s_Tx,
};

// After initializing the application event queue:
if (!UsbInit(&s_UsbCfg) || !s_Int.Init(s_IntCfg)) return -1;
(void)UsbEnable(0);
AppRun();
```

`IntEvent` is the application's `DevIntrfEvtHandler_t`. The function fields are
controller number, alternate table, count, vendor subclass, protocol and
interface string index. Index 4 uses `UsbCfg_t::pFuncName`; zero omits the string.
No interface or endpoint number is supplied. The alternate table must remain
alive and unchanged for the registered function's lifetime.

For ISO, use `UsbdIsoCfg_t` with the same `Function` fields and event callback.
Leaving both FIFO pointers null selects the transport's internal two-packet
FIFOs. Caller-owned FIFOs instead use two distinct aligned regions of
`USB_ISO_INTRF_FIFO_MEMSIZE(BufferSize)` bytes. Each alternate must fit that
capacity. The loopback example supplies six choices:

```cpp
static const UsbdPeriodicAlt_t s_IsoAlternates[] = {
	{9, 9, 1, 4}, {17, 17, 1, 4}, {25, 25, 1, 4},
	{33, 33, 1, 4}, {49, 49, 1, 4}, {63, 63, 1, 4},
};
static UsbdIso s_Iso;
static const UsbdIsoCfg_t s_IsoCfg = {
	.Function = {0, s_IsoAlternates, 6, 0, 0, 4},
	.EvtCB = IsoEvent,
};
// Between UsbInit and UsbEnable:
if (!s_Iso.Init(s_IsoCfg)) return -1;
```

Table entries are active settings 1..AltCount. Alternate 0 disables transfers.
Full-speed Interrupt intervals are milliseconds. ISO intervals and high-speed
Interrupt intervals encode `2^(bInterval-1)` frames or microframes. Thus ISO
intervals 1 at full speed and 4 at high speed both request one millisecond.
HS fields are required only on high-speed-capable targets. High-bandwidth
multiple transactions per microframe and USB audio class semantics are not
provided by these vendor functions.

The classes close transfers on unconfiguration, reset and detach. ISO suspend
and resume are handled through the existing deferred USB process pass.
An endpoint-open failure rejects the requested alternate and attempts to reopen
the previous one. A persistent controller failure requires host reconfiguration
or bus reset. Invalid configuration, exhausted ownership and descriptor capacity
fail initialization without retaining the new class's allocation. Hardware pipe
limits are still enforced by the controller when endpoints open.

Subclass `Control()` for application-specific diagnostic requests. The loopback
samples also clear their diagnostic counters when an active alternate opens.
Their request IDs (Interrupt 0x5B, ISO 0x5A), reply formats, VID/PID and alternate
layouts are unchanged. Composite stress samples use the same classes.

```bash
./.venv/bin/python3 Python/usb_int_loopback.py
./.venv/bin/python3 Python/usb_iso_loopback.py
```

Both runners provide a `--manual-suspend-wake` phase. The Interrupt runner
checks alternate settings with different polling intervals. The Isochronous
runner checks scheduled packet flow and diagnostic counters. Use
`Python/usb_iso_diag.py` for focused isochronous endpoint diagnosis.

### HID report definitions

`UsbdHid` continues to generate the USB interface, HID and endpoint descriptors.
Applications supply their report definition because it describes their actual
input/output data. The keyboard, vendor loopback and six-axis mouse have distinct
report layouts and report handling. A general descriptor generator would add
more API than these short definitions remove, so this change retains the existing
report API and examples rather than adding presets with implicit report layouts.

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

Check the target's cable support first: LPC1769 currently requires a reset
with VBUS present after reattachment. On ports with automatic reattachment,
keep the queue running and rediscover the host device path. Serial ports
and BSD disk numbers are host-assigned and can change. For MSC, removal of VBUS
reloads a removable medium; a logical eject without VBUS removal intentionally
keeps it not-ready until the host sends a load request.

### More than one matching test device is attached

Pass `--serial` to runners that support it. The raw runners discover interface
and endpoint addresses from descriptors instead of assuming endpoint 1.
