# USB Architecture

IOsonata keeps USB simple at the application boundary and explicit internally.
An application instantiates the USB device/function it needs and initializes it.
Interface numbers, endpoint numbers, descriptor placement and endpoint callbacks
are allocated and connected inside the USB stack.

```text
Application
    |
    +-- UsbdCdc
    +-- UsbdBulk
    +-- other USB device/function classes
            |
            v
       automatic USB resource allocation
```

Applications do not select endpoint numbers.

## Complete data-path model

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
             +-----------------+-----------------+
             |                                   |
         specializations / users of UsbIntrf     |
             |                                   |
      +------+------+                            |
      |             |                            |
   UsbdCdc       UsbdBulk                    UsbIsoIntrf
      |             |                            |
   Bulk EP       Bulk EP                       ISO EP
   byte mode     byte/packet mode          non-blocking packet mode
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
of the data stream, so the Interrupt IN endpoint is owned directly by `UsbdCdc`.

## Custom Bulk

`UsbdBulk` is a thin public specialization of `UsbIntrf`.

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

`UsbdBulk` owns the descriptor policy and static controller buffers. `UsbIntrf`
owns queuing, staging and endpoint transfer completion.

## Isochronous

Isochronous is another `UsbIntrf` specialization, not a separate transfer
engine.

```text
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

`UsbIsoIntrf` owns only ISO-specific behavior:

- open/close the endpoint pair as Isochronous;
- ISO maximum packet size and service interval;
- ISO synchronization/usage descriptor attributes;
- suspend/resume/reset state;
- frame-facing convenience callbacks and counters.

`UsbIntrf` continues to own the transfer machinery:

- fixed RX/TX controller buffers;
- endpoint registration;
- the controller endpoint callback;
- RX/TX CFifos;
- packet-mode staging;
- `UsbCtrlrEpXfer()` submission;
- transfer completion handling.

ISO always configures `UsbIntrf` with `bBlocking = false`. An isochronous
service opportunity cannot be backpressured and retried like Bulk. If software
cannot keep up, data is missed/dropped according to the non-blocking path rather
than delaying the USB service interval.

There is no path of the form:

```text
UsbCtrlr -> UsbCore -> class -> UsbIsoIntrf
```

and there is no independent ISO controller callback beside the one registered
by `UsbIntrf`.

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

### Controller initialization

`UsbInit()` calls `UsbCtrlrInit()` and installs the device-wide/core callback.
This callback handles controller-wide events such as:

```text
RESET
SUSPEND
RESUME
SOF
```

EP0 setup/control state is also owned by `UsbCore`.

### Endpoint configuration

`UsbCtrlrEpOpen()` receives an endpoint descriptor. The descriptor establishes:

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

The controller already knows the endpoint type because it was established by
`UsbCtrlrEpOpen()`.

Endpoint callbacks are registered with the endpoint owner and are delivered
directly. Nonzero endpoint completion is not routed through the USB function
table.

```text
controller interrupt
        |
        v
registered endpoint callback
        |
        v
endpoint owner
```

For CDC/Bulk/ISO data pairs, that endpoint owner is `UsbIntrf`.

## Control endpoint

EP0 is special because Control transfers have protocol stages:

```text
SETUP
  |
  +-- DATA
  |
  +-- STATUS
```

`UsbCore` owns that state machine and Chapter 9 processing. The controller may
use a separate EP0 transfer entry point because each control request can select
a different buffer, while ordinary data endpoints use fixed registered DMA
buffers.

This does not change the endpoint architecture: EP0 is still controller-driven,
but its protocol owner is `UsbCore` rather than `UsbIntrf`.

## Interrupt endpoints

Interrupt endpoints are normally event/notification endpoints rather than
stream data endpoints. A class owns them directly when no `UsbIntrf` stream or
packet transport is needed.

CDC ACM notification is the reference pattern:

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

The host schedules Interrupt polling according to `bInterval`.

## Automatic interface and endpoint allocation

USB functions request resources; applications do not assign them.

The internal allocator receives requirements such as:

```text
number of interfaces
number of bidirectional endpoint numbers
number of IN-only endpoints
number of OUT-only endpoints
controller-constrained endpoint masks
```

It finds the lowest legal placement accepted by the USB core and controller
capabilities. The assigned numbers remain private class state and are used for
endpoint registration, descriptor construction and control-request ownership.

A controller with a dedicated ISO endpoint advertises it through ISO capability
masks. The allocator satisfies that restriction internally.

## UsbIntrf responsibilities

One `UsbIntrf` instance represents one bidirectional endpoint number:

```text
OUT = receive
IN  = transmit
```

The derived class supplies fixed controller RX/TX buffers. `UsbIntrf` registers
those buffers once and later submits only endpoint address and length.

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

Transfer type and CFifo mode are separate. For example:

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

## Lifecycle

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

1. `UsbInit()` initializes `UsbCore` and the controller.
2. Each class requests its interfaces/endpoints from the internal allocator.
3. Data classes initialize their inherited `UsbIntrf` with the allocated
   endpoint number, static buffers and CFifos.
4. `UsbIntrfInit()` registers the endpoint buffers and callback.
5. Configuration or alternate-setting selection opens the endpoint descriptors.
6. Transfers use `UsbCtrlrEpXfer()`.
7. Reset/unconfiguration closes endpoints and clears active transport state.

## Rules for new USB work

Keep these invariants when adding a new class or transfer type:

1. Applications do not choose interface or endpoint numbers.
2. Resource allocation remains internal.
3. `UsbCtrlr` contains hardware behavior, not class protocol behavior.
4. `UsbCtrlrEpXfer()` remains the common non-control endpoint transfer call.
5. Nonzero endpoint events go directly to the registered endpoint owner.
6. `UsbIntrf` remains the common endpoint-pair data engine.
7. CDC and Bulk use `UsbIntrf`; ISO is `UsbIntrf` in non-blocking packet mode.
8. Class-owned Interrupt endpoints do not need to pass through `UsbIntrf` when
   they are only notifications/events.
9. `UsbCore` owns EP0 and Chapter 9, not data endpoint completion routing.
10. Do not duplicate endpoint transfer machinery inside a specialization.
