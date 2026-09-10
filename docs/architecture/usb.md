# USB Architecture

IOsonata keeps USB simple at the application boundary and explicit internally.
An application instantiates the USB class it needs and initializes it.
Interface numbers, endpoint numbers, descriptor placement and endpoint callbacks
are allocated and connected inside the USB stack.

Applications do not select endpoint numbers.

## Naming and USB role model

The `usb_*` layer is role-neutral USB infrastructure. It must not accumulate
policy that only makes sense for USB device mode or only for USB host mode.

```text
usb_*      reusable USB infrastructure
usbd_*     USB device-side classes
usbh_*     future USB host-side classes/drivers
```

Examples:

```text
usb_intrf.*      UsbIntrf       role-neutral endpoint-pair data engine
usb_iso.*        UsbIsoIntrf    role-neutral ISO specialization
usb_int.*        UsbIntIntrf    role-neutral Interrupt specialization

usbd_cdc.*       UsbdCdc        USB device CDC ACM
usbd_bulk.*      UsbdBulk       USB device custom Bulk class
usbd_hid.*       UsbdHid        USB device HID class

usbh_*           future USB host class/driver layer
```

The current stack implements USB device mode. Host support does not exist yet.
The architectural requirement is that `UsbIntrf`, `UsbIsoIntrf` and
`UsbIntIntrf` stay free of device-only class/enumeration policy so they can be
reused when a host layer is added.

Role-specific behavior belongs above them:

```text
                         UsbIntrf
                    role-neutral data path
                      /       |       \
                     /        |        \
              UsbdCdc     UsbdBulk   UsbIsoIntrf / UsbIntIntrf
                                                /      \
                                               /        \
                                         UsbdHid      future host-side
```

Device-side responsibilities such as device descriptors, `SET_CONFIGURATION`,
`SET_INTERFACE`, device class allocation and Chapter 9 ownership do not
belong in `UsbIsoIntrf`.

Future host-side responsibilities such as device enumeration, selecting a
configuration, claiming an interface and selecting an alternate setting also do
not belong in `UsbIsoIntrf`.

`UsbIsoIntrf` and `UsbIntIntrf` own only reusable transfer-type behavior shared
by both roles.

The C++ class hierarchy has one common lifecycle base and distinct bases for
role-specific control behavior:

```text
UsbClass
|-- UsbDeviceClass
`-- UsbHostClass
```

`UsbClass` provides `Reset()` and `Process()`. `UsbDeviceClass` adds `Control()`,
`SelectConfig()` and `SelectInterface()`. Host
matching, attach and detach behavior belongs to `UsbHostClass`. The
role-neutral `UsbIntrf` data path remains separate so a concrete CDC, HID or
vendor class can combine class control with the same RX/TX interface
implementation.

The USB subsystem keeps statically owned class objects in one `UsbClass *`
array. Shared reset and application-context processing dispatch through that
array without testing device versus host controller mode inside each class.

## Current device-side data-path model

`UsbIntrf` is the common bidirectional endpoint-pair data engine. CDC, custom
Bulk, Isochronous and Interrupt all reuse it.

```text
                              UsbIntrf
                    generic endpoint-pair engine
                 fixed RX/TX controller DMA staging
                    selected RX/TX data policy
                    endpoint registration/callback
                           UsbCtrlrEpXfer
                               |
              +----------------+----------------+
              |                |                |
           UsbdCdc          UsbdBulk        UsbIsoIntrf / UsbIntIntrf
              |                |                         |
           Bulk EP          Bulk EP                  ISO / INT EP
           BYTE mode        BYTE/PACKET              DIRECT mode
```

The inheritance view is intentionally small:

```text
                    UsbIntrf
                  /    |     \
                 /     |      \
          UsbdCdc   UsbdBulk   UsbIsoIntrf / UsbIntIntrf
```

The endpoint transfer type and the `UsbIntrf` data-path mode are independent.
Opening an endpoint as Bulk, Isochronous or Interrupt does not create a
different controller transfer API.

## CDC

CDC ACM uses `UsbIntrf` for its Bulk data endpoint pair.

```text
UsbdCdc
   |
   +-- Bulk OUT/IN ----------> UsbIntrf
   |                              |
   |                         UsbCtrlrEpXfer
   |
   +-- Interrupt IN ---------> direct class-owned endpoint
       notifications
```

CDC byte data uses `UsbIntrf` byte-stream TX mode. CDC notification is not part
of the data stream, so the Interrupt IN endpoint is owned directly by
`UsbdCdc`.

## Custom Bulk

`UsbdBulk` is a thin device-side specialization of `UsbIntrf`.

```text
UsbdBulk
    |
    v
UsbIntrf
    |
    +-- byte-stream mode
    |
    +-- packet mode
    |
    v
Bulk OUT/IN endpoints
```

`UsbdBulk` owns device-side descriptor policy and static controller buffers.
`UsbIntrf` owns queuing, staging and endpoint transfer completion.

## Isochronous

`UsbIsoIntrf` is the role-neutral Isochronous specialization of `UsbIntrf`, not
a separate endpoint transfer engine.

Current device-side path:

```text
USB device class
        |
        v
UsbIsoIntrf
        |
        v
UsbIntrf
(DIRECT mode)
        |
        v
Isochronous OUT/IN endpoints
        |
        v
UsbCtrlrEpXfer / endpoint callback
```

Future host-side classes should reuse the same `UsbIsoIntrf` transport layer
instead of introducing a separate `UsbhIsoIntrf` with duplicated transfer
logic.

`UsbIsoIntrf` owns only ISO-specific behavior:

- open/configure an endpoint pair as Isochronous;
- ISO maximum packet size and service interval;
- ISO synchronization/usage attributes;
- DIRECT data-policy selection;
- reset/suspend/resume transport state;
- frame-facing convenience callbacks and counters.

`UsbIntrf` continues to own the common endpoint transfer machinery:

- fixed RX/TX controller buffers;
- endpoint registration;
- the endpoint callback;
- BYTE/PACKET CFifos and DIRECT slots;
- packet-mode and direct-mode staging;
- endpoint transfer submission;
- transfer completion handling.

ISO always configures `UsbIntrf` with `bBlocking = false`. An isochronous
service opportunity cannot be backpressured and retried like Bulk. If software
cannot keep up, data is missed/dropped according to the non-blocking path rather
than delaying the service interval.

There is no independent ISO controller transfer engine and no class-level
completion forwarding path.

## Interrupt transfer

`UsbIntIntrf` is the role-neutral Interrupt specialization of `UsbIntrf`. It
uses the same DIRECT storage policy as `UsbIsoIntrf`, but opens its endpoint
pair with `USB_ENDPATT_TRANS_INT` and owns the interrupt polling interval.

```text
USB class
        |
        v
UsbIntIntrf
        |
        v
UsbIntrf (DIRECT mode)
        |
        v
Interrupt OUT/IN endpoints
```

DIRECT describes only software storage: one current RX slot, one current TX
slot and no CFifo. ISO selects the existing non-blocking behavior, so DIRECT
ignores `USB_CTRLR_EVT_DRDY` while the ISO controller path services scheduled
opportunities. Interrupt selects the existing blocking behavior, so DIRECT
services `USB_CTRLR_EVT_DRDY` through the normal controller transfer call.
There is no separate RX arm or re-arm API.

`UsbIntIntrf` contains no HID report or descriptor behavior. `UsbdHid` embeds
it and owns the device-side HID descriptor, class requests and report policy.

The nRF52840 `UsbIntLoopback` project and
[`Python/usb_int_loopback.py`](../../Python/usb_int_loopback.py) exercise the
transport on hardware without adding class semantics. The test selects three
interrupt intervals, checks alternate-setting close/open behavior, transfers
zero- through maximum-length packets in both directions, forces and recovers
from a busy TX slot, and offers a manual suspend/wake phase.

## HID

`UsbdHid` follows the same device-class pattern as `UsbdBulk` and
`BtHciUsb`: it registers its class instance, receives allocated interface and
endpoint numbers, fills a descriptor fragment supplied by the application and
opens its endpoints when configuration 1 becomes active.

```text
UsbdHid
    |
    +-- HID descriptor and class requests
    +-- application report descriptor
    |
    v
UsbIntIntrf
    |
    v
Interrupt OUT/IN endpoint pair
```

The application supplies the report descriptor and the staged handler for
`GET_REPORT` and `SET_REPORT`. `UsbdHid` handles the HID descriptor,
`GET/SET_IDLE` and boot-subclass `GET/SET_PROTOCOL`. All USB and HID constants
and descriptor structures come from `usb_def.h` and `usb_hiddef.h`.

The `UsbHidLoopback` nRF52840 project uses a vendor-page 64-byte input/output
report and is exercised through the native host HID driver by
[`Python/usb_hid_loopback.py`](../../Python/usb_hid_loopback.py).

## Controller boundary

Every target provides `usb_ctrlr.h` and its controller implementation. Portable
USB code does not depend on MCU-specific register definitions.

The central controller concepts are:

```text
UsbCtrlrInit
UsbCtrlrEpOpen / UsbCtrlrEpClose
UsbCtrlrEpXfer
Event callback
```

Support operations such as start/stop, connect/disconnect, stall, remote wakeup
and SOF enable surround those operations.

The controller API is intentionally transfer-type agnostic. Endpoint type is
established when the endpoint/pipe is configured; the transfer operation remains
the common endpoint transaction primitive.

### Current device initialization

In the current device stack, `UsbInit()` calls `UsbCtrlrInit()` and installs the
core callback for device-wide events such as:

```text
RESET
SUSPEND
RESUME
SOF
```

EP0 setup/control state is owned by `UsbCore`.

This device-core behavior is not part of `UsbIntrf` or `UsbIsoIntrf` and must
not be pushed into those role-neutral layers.

### Endpoint configuration

`UsbCtrlrEpOpen()` receives an endpoint descriptor in the current device port.
The descriptor establishes:

```text
endpoint address and direction
transfer type: Control / Isochronous / Bulk / Interrupt
maximum packet size
interval where applicable
ISO synchronization/usage attributes where applicable
```

There are no class-specific controller transfer calls such as `CdcXfer()`,
`BulkXfer()` or `IsoXfer()`.

### Endpoint transfer

Non-control data endpoints use one common operation:

```text
UsbCtrlrEpXfer(DevNo, EpAddr, Length)
```

The controller already knows the endpoint type because it was established when
the endpoint was opened/configured.

Endpoint callbacks are delivered directly to the registered endpoint owner.
Nonzero endpoint completion is not routed through the USB class table.

```text
controller interrupt
        |
        v
registered endpoint callback
        |
        v
endpoint owner
```

For CDC/Bulk/ISO/Interrupt data pairs in the current device stack, that endpoint
owner is `UsbIntrf`.

## Control endpoint

EP0 is device-side protocol infrastructure. Control transfers have:

```text
SETUP
  |
  +-- DATA
  |
  +-- STATUS
```

`UsbCore` owns that state machine and Chapter 9 processing. This is intentionally
outside `UsbIntrf` and `UsbIsoIntrf` so the reusable data layers do not become
device-mode specific.

## Interrupt endpoints

Interrupt endpoints are normally event/notification endpoints rather than
stream data endpoints. A device class owns them directly when no `UsbIntrf`
stream or packet transport is needed.

CDC ACM notification is the reference device-side pattern:

```text
class state changes
        |
        v
build notification
        |
        v
UsbCtrlrEpXfer(Interrupt IN)
        |
        v
class endpoint callback
```

## Automatic interface and endpoint allocation

Automatic interface/endpoint allocation is currently a device-side composition
facility. USB device classes request resources; applications do not assign
them.

The internal allocator receives requirements such as:

```text
number of interfaces
number of bidirectional endpoint numbers
number of IN-only endpoints
number of OUT-only endpoints
controller-constrained endpoint masks
```

It finds the lowest legal placement accepted by the USB core and controller
capabilities. Assigned numbers remain private device-class state and are used
for endpoint registration, descriptor construction and control-request
ownership.

A controller with a dedicated ISO endpoint advertises it through ISO capability
masks. The allocator satisfies that restriction internally.

This allocator is not part of `UsbIsoIntrf`; a future host stack will have its
own role-specific discovery/allocation process above the same reusable transport
layer.

## UsbIntrf responsibilities

One `UsbIntrf` instance represents one bidirectional transfer channel. In the
current USB device implementation this maps to one endpoint number:

```text
OUT = receive
IN  = transmit
```

The derived/specialized layer supplies fixed controller RX/TX buffers.
`UsbIntrf` registers those buffers once and later submits only endpoint address
and length.

RX packet boundaries are preserved internally with:

```text
UsbPktHdr_t { Length }
+ packet payload storage
```

BYTE/PACKET mode is selected by CFifo block size:

```text
block size 1
    -> byte-stream mode

block size UsbPktHdr_t + packet storage
    -> packet mode
```

Transfer type and data-path mode are separate:

```text
CDC       = Bulk + byte mode
UsbdBulk  = Bulk + byte or packet mode
UsbIsoIntrf = Isochronous + DIRECT mode
UsbIntIntrf = Interrupt + DIRECT mode
UsbdHid   = HID class + UsbIntIntrf
```

## Storage ownership

The USB data path uses static storage.

```text
Derived class / specialization
    -> RX/TX controller DMA staging

UsbIntrf
    -> BYTE/PACKET CFifos or DIRECT slot ownership

UsbCtrlr port
    -> hardware/DMA transaction state
```

CFifo storage and controller DMA staging are intentionally separate so queued
software storage can be reused without modifying memory currently owned by the
controller.

## Current device lifecycle

The application view is:

```text
UsbInit
   |
class Init calls
   |
UsbEnable
   |
host enumeration/configuration
   |
class endpoints open
```

Internally:

1. `UsbInit()` initializes `UsbCore` and the device controller.
2. Each device class requests its interfaces/endpoints from the internal
   allocator.
3. Data classes initialize their inherited `UsbIntrf` with the allocated
   endpoint number, static buffers and CFifos.
4. `UsbIntrfInit()` registers the endpoint buffers and callback.
5. Configuration or alternate-setting selection opens the endpoint descriptors.
6. Transfers use `UsbCtrlrEpXfer()`.
7. Reset/unconfiguration closes endpoints and clears active transport state.

This lifecycle describes the current device stack only. Future host enumeration
and interface selection belong in `usbh_*` and must not change the reusable
`UsbIntrf`, `UsbIsoIntrf` or `UsbIntIntrf` transport model.

## Rules for new USB work

Keep these invariants when adding a class, transfer type or future host support:

1. `usb_*` remains role-neutral reusable USB infrastructure.
2. `usbd_*` contains USB device-side class behavior.
3. Future `usbh_*` contains USB host-side class/driver behavior.
4. Applications do not choose device-side interface or endpoint numbers.
5. Device resource allocation remains internal to the device stack.
6. `UsbCtrlr` contains hardware behavior, not class protocol behavior.
7. The endpoint transfer primitive remains common across Bulk, Interrupt and
   Isochronous operation.
8. Nonzero endpoint events go directly to the registered endpoint owner.
9. `UsbIntrf` remains the common endpoint-pair data engine.
10. CDC and Bulk use BYTE/PACKET; ISO and Interrupt specializations use DIRECT.
11. `UsbIsoIntrf` must not contain device descriptors, device Chapter 9 policy,
    host enumeration policy or class-specific packet semantics.
12. Class-owned Interrupt endpoints do not need to pass through `UsbIntrf` when
    they are only notifications/events.
13. `UsbCore` owns current device EP0 and Chapter 9 processing, not data endpoint
    completion routing.
14. Do not duplicate endpoint transfer machinery inside a specialization or a
    future role-specific class.
15. HID report and descriptor semantics belong in `UsbdHid`, not
    `UsbIntIntrf`.
