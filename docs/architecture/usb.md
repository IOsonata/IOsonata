# USB Architecture

The USB stack is three layers with one boundary between each pair. Everything
that does not change from one MCU to the next is compiled once; everything that
does is supplied by the MCU port.

```text
        Application
             |  UsbInit / UsbEnable / UsbProcess
             v
     Class or function  (UsbdCdc, ...)
             |  UsbFuncCfg_t registration + UsbIntrf data path
             v
     Generic layer      usb.h / usb.cpp / usb_intrf.cpp
             |  UsbCtrlr entry points
             v
     MCU port           usb_ctrlr.h + usb_ctrlr_<family>.cpp
             |
           silicon
```

## Files

| File | Scope |
| --- | --- |
| `include/usb/usb_def.h` | USB specification definitions, descriptors, requests |
| `include/usb/usb.h` | The generic layer and the port entry points |
| `include/usb/usb_intrf.h` | `DeviceIntrf` over a USB endpoint pair |
| `src/usb/usb.cpp` | Identity, Chapter 9, function dispatch, lifecycle |
| `src/usb/usb_intrf.cpp` | Endpoint data path and FIFO handling |
| `<port>/include/usb_ctrlr.h` | What the target's controller can do |
| `<port>/src/usb_ctrlr_<family>.cpp` | Registers, DMA, interrupt |

The class layer sits alongside: `usbd_cdc.h`, `usbd_cdc.cpp` and
`usbd_cdc_desc.cpp` for CDC ACM.

A `usbd_` prefix means device specific. A `usb_` prefix is role neutral and
serves device and host alike. `UsbCtrlr` is role neutral because a host
controller would implement the same shape.

## The port boundary

`usb.h` declares the `UsbCtrlr` entry points once. They do not change across
targets, so they are not repeated per port, the same way `iopincfg.h` declares
the pin API once while each port supplies `iopinctrl.h`.

What does change per target is `usb_ctrlr.h`, which every port with a USB
controller provides on its include path under that exact name. Generic code
includes it by plain name and never switches on a vendor macro.

The include path runs most specific first, from the part directory out to the
repository root, so a part that differs from its family can shadow the family
file by providing its own.

## Capabilities are compile time

`usb_ctrlr.h` answers packet sizes, endpoint counts, speed class and
isochronous support as macros:

```c
USB_CTRLR_CNT
USB_PKT_MAXLEN(DevNo, TransType)    // CONTROL, ISO, BULK, INT
USB_EPIN_CNT(DevNo)
USB_EPOUT_CNT(DevNo)
USB_HIGHSPEED_CAPABLE(DevNo)
USB_ISO_SUPPORTED(DevNo)
```

They are macros rather than a runtime record because an application has to size
its packet ring and CFifo memory statically, before any endpoint exists:

```c
#define CDC_RXFIFO_MEMSIZE \
    USB_INTRF_RXMEM_SIZE(4, USB_PKT_MAXLEN(0, BULK))
```

Packet lengths are allocation bounds: the largest packet the controller can
move at the fastest speed it supports. A slower enumeration negotiates less and
the buffer still fits.

The accessors paste through a second macro, so a named constant expands before
it is pasted and `USB_PKT_MAXLEN(USB_DEVNO, BULK)` works.

## Lifecycle

```text
UsbInit(&cfg)          identity, controller software state, Chapter 9
UsbdCdcInit(...)       one call per function, registers UsbFuncCfg_t
UsbEnable(DevNo)       power, clock, PHY, endpoint zero, bus pull-up
UsbProcess(DevNo)      polled from the application loop
```

Init and enable are separate because functions register between them. That is
the same shape as `BtAppInit` followed by service registration.

`UsbCtrlrStart` brings up power, clock, PHY and endpoint zero in one call. An
earlier split between a power header and a register header forced callers to
make two calls and stated the same facts twice.

Bus power is not reported through a port callback. The port exposes
`UsbCtrlrVbusDetected` and `UsbProcess` polls the level, deriving
`USB_EVT_ATTACHED` and `USB_EVT_DETACHED` itself. The level matters rather than
the edge: a board already on a cable at reset never produces an edge.

## Function registration

One registration per class or vendor function. `UsbFuncCfg_t` carries the
interface and endpoint ownership plus every callback, including
`ProcessHandler`, which `UsbProcess` polls in application context for work a
function cannot do inside the interrupt.

Endpoint zero belongs to the generic layer. Bit zero must be clear in both
endpoint masks and masks may not overlap between functions.

## Data path

`UsbIntrf` applies `DeviceIntrf` to a configured endpoint pair, so a USB
endpoint reads and writes like a UART, SPI or I2C interface. Endpoint address,
transfer type and packet size are instance state; `DeviceIntrf` `DevAddr` is
not used to select an endpoint.

### Storage ownership

`UsbCtrlrEpXfer` keeps the buffer it is given until it reports completion. Any
storage that can be handed back to the other side sooner is therefore not the
same memory. The two directions answer that differently.

```text
controller  ->  RX ring slot  ->  application            (no copy)
application ->  byte or packet CFifo  ->  copy  ->  TX staging  ->  controller
```

`CFifoPut` advances the producer index before the caller fills the block, so a
block is visible while still empty. A block published that way cannot be handed
to the controller, because controller DMA outlives the call that started it.
RX therefore does not use `CFifo` at all, and TX, which does, copies out of the
block before submitting.

### Receive

RX memory is a ring of fixed packet slots, sized by the application:

```c
#define CDC_RXFIFO_MEMSIZE \
    USB_INTRF_RXMEM_SIZE(4, USB_PKT_MAXLEN(0, BULK))
```

Each slot is `UsbPktHdr_t` followed by one endpoint packet, word aligned.
`UsbIntrfRxKick` arms the OUT endpoint on the slot the ring will publish next,
so the controller writes straight into it. The slot becomes visible to the
reader only after `UsbIntrfRxXferComplete` stores the length and advances
`RxPut`, in that order. A reader never sees a partial packet, nothing is
copied, and there is no critical section in the interrupt.

`RxPut` and `RxGet` are monotonic counters, taken modulo the slot count when a
slot address is needed, so a full ring and an empty ring are distinguishable
without a spare slot.

Packet boundaries survive: a short packet stays one packet, a full packet stays
one packet, and a zero length packet is one slot with `Length` zero.

`DeviceIntrfRx()` presents a byte stream on top, consuming across slots and
keeping its read position in `RxOffset` on the interface object rather than in
the stored packet. A partially read slot stays in the ring until its last byte
is taken. A zero length slot is released without consuming any of the caller's
buffer.

Flow control is lossless. When the ring is full the endpoint is left unarmed
and the host is backpressured by USB rather than a packet being dropped.
Releasing a slot rearms the endpoint.

### Transmit

TX mode comes from the CFifo block size alone and has nothing to do with the
endpoint's USB transfer type. Bulk does not imply byte mode.

- Block size 1 is byte mode. Application writes accumulate as bytes and the
  interface packetizes what is queued, up to the packet size, without waiting
  for a full packet. One queued byte becomes a one byte USB packet.
- Block size `sizeof(UsbPktHdr_t) + MPS` is packet mode. The caller splits its
  data into packets and pushes one block each. Exactly `Length` bytes are sent
  per block, blocks are never combined, and `Length` zero sends a zero length
  packet.

CDC is byte mode, which is what keeps a single character interactive.

## More than one controller

`DevNo` selects the controller, matching the `DevNo` convention of `UARTCfg_t`,
`SPICfg_t` and `I2CCfg_t`. It is the controller index, never the USB device
address assigned by SET_ADDRESS.

Every entry point takes it and every configuration struct carries it. Parts
exist with two USB controllers, STM32F427 and some LPC546xx among them.

The current implementations hold file scope state, so one controller is
initialized at a time. That matches every part supported so far, where
`USB_CTRLR_CNT` is 1. Arraying the state is work for the first part that needs
two, and the signatures already allow it.

## Known gaps

- A lost IN completion leaves the transmit token held with nothing to return
  it, because `UsbIntrfSof` declines to act while the token is taken. There is
  no recovery path.
- After a bus suspend `UsbCtrlrEpXfer` refuses, `RxActive` is cleared, and
  nothing rearms on resume while the FIFO is empty.
- Isochronous endpoints are not driven by any port. `USB_ISO_SUPPORTED` reports
  this separately from the packet length the hardware allows.
- There is no host controller layer. The role neutral naming anticipates one.
