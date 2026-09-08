# USB Architecture

IOsonata keeps USB simple at the application boundary and explicit internally.
An application instantiates the USB class/function it needs and initializes it.
Interface numbers, endpoint numbers, descriptor placement and endpoint callbacks
are allocated and connected inside the USB stack.

Applications do not select endpoint numbers.

## Naming and USB role model

The `usb_*` layer is role-neutral USB infrastructure. It must not accumulate
policy that only makes sense for USB device mode or only for USB host mode.

```text
usb_*      reusable USB infrastructure
usbd_*     USB device-side classes/functions
usbh_*     future USB host-side classes/drivers
```

Examples:

```text
usb_intrf.*      UsbIntrf       role-neutral endpoint-pair data engine
usb_iso.*        UsbIsoIntrf    role-neutral ISO specialization

usbd_cdc.*       UsbdCdc        USB device CDC ACM
usbd_bulk.*      UsbdBulk       USB device custom Bulk function

usbh_*           future USB host class/driver layer
```

The current stack implements USB device mode. Host support does not exist yet.
The architectural requirement is that `UsbIntrf` and `UsbIsoIntrf` stay free of
device-only class/enumeration policy so they can be reused when a host layer is
added.

Role-specific behavior belongs above them:

```text
                         UsbIntrf
                    role-neutral data path
                      /       |       \
                     /        |        \
              UsbdCdc     UsbdBulk   UsbIsoIntrf
                                       /      \
                                      /        \
                              device-side    future host-side
                               ISO class       ISO class
```

Device-side responsibilities such as device descriptors, `SET_CONFIGURATION`,
`SET_INTERFACE`, device function allocation and Chapter 9 ownership do not
belong in `UsbIsoIntrf`.

Future host-side responsibilities such as device enumeration, selecting a
configuration, claiming an interface and selecting an alternate setting also do
not belong in `UsbIsoIntrf`.

`UsbIsoIntrf` owns only the reusable ISO transport behavior shared by both
roles.

## Current device-side data-path model

`UsbIntrf` is the common bidirectional endpoint-pair data engine. CDC, custom
Bulk and Isochronous all reuse it.

```text
                              UsbIntrf
                    generic endpoint-pair engine
                 fixed RX/TX controller DMA staging
                       CFifo RX/TX transport
                    endpoint registration/callback
                           UsbCtrlrEpXfer
                               |
              +----------------+----------------+
              |                |                |
           UsbdCdc          UsbdBulk        UsbIsoIntrf
              |                |                |
           Bulk EP          Bulk EP           ISO EP
           byte mode        byte/packet       non-blocking
                            mode              packet mode
```

The inheritance view is intentionally small:

```text
                    UsbIntrf
                  /    |     \
                 /     |      \
          UsbdCdc   UsbdBulk   UsbIsoIntrf
```

The endpoint transfer type and the `UsbIntrf` CFifo mode are independent.
Opening an endpoint as Bulk or Isochronous does not create a different
controller transfer API.

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
USB device class/function
        |
        v
UsbIsoIntrf
        |
        v
UsbIntrf
(non-blocking packet mode)
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
- non-blocking transport selection;
- reset/suspend/resume transport state;
- frame-facing convenience callbacks and counters.

`UsbIntrf` continues to own the common endpoint transfer machinery:

- fixed RX/TX controller buffers;
- endpoint registration;
- the endpoint callback;
- RX/TX CFifos;
- packet-mode staging;
- endpoint transfer submission;
- transfer completion handling.

ISO always configures `UsbIntrf` with `bBlocking = false`. An isochronous
service opportunity cannot be backpressured and retried like Bulk. If software
cannot keep up, data is missed/dropped according to the non-blocking path rather
than delaying the service interval.

There is no independent ISO controller transfer engine and no class-level
completion forwarding path.

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
Nonzero endpoint completion is not routed through the USB function table.

```text
controller interrupt
        |
        v
registered endpoint callback
        |
        v
endpoint owner
```

For CDC/Bulk/ISO data pairs in the current device stack, that endpoint owner is
`UsbIntrf`.

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
facility. USB device functions request resources; applications do not assign
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

TX mode is selected by CFifo block size:

```text
block size 1
    -> byte-stream mode

block size UsbPktHdr_t + packet storage
    -> packet mode
```

Transfer type and CFifo mode are separate:

```text
CDC       = Bulk + byte mode
UsbdBulk  = Bulk + byte or packet mode
ISO       = Isochronous + non-blocking packet mode
```

## Storage ownership

The USB data path uses static storage.

```text
Derived class / specialization
    -> RX/TX controller DMA staging

UsbIntrf
    -> RX/TX CFifo transport

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
`UsbIntrf` / `UsbIsoIntrf` transport model.

## Rules for new USB work

Keep these invariants when adding a class, transfer type or future host support:

1. `usb_*` remains role-neutral reusable USB infrastructure.
2. `usbd_*` contains USB device-side class/function behavior.
3. Future `usbh_*` contains USB host-side class/driver behavior.
4. Applications do not choose device-side interface or endpoint numbers.
5. Device resource allocation remains internal to the device stack.
6. `UsbCtrlr` contains hardware behavior, not class protocol behavior.
7. The endpoint transfer primitive remains common across Bulk, Interrupt and
   Isochronous operation.
8. Nonzero endpoint events go directly to the registered endpoint owner.
9. `UsbIntrf` remains the common endpoint-pair data engine.
10. CDC and Bulk use `UsbIntrf`; ISO is `UsbIntrf` in non-blocking packet mode.
11. `UsbIsoIntrf` must not contain device descriptors, device Chapter 9 policy,
    host enumeration policy or class-specific packet semantics.
12. Class-owned Interrupt endpoints do not need to pass through `UsbIntrf` when
    they are only notifications/events.
13. `UsbCore` owns current device EP0 and Chapter 9 processing, not data endpoint
    completion routing.
14. Do not duplicate endpoint transfer machinery inside a specialization or a
    future role-specific class.
