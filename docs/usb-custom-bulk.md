# USB Custom Bulk Interface

`UsbdBulk` provides a reusable USB data interface with one bulk OUT endpoint and one bulk IN endpoint. The USB interface uses class code `0xFF`, while the application can name the function for its own product or protocol.

Use the complete loopback example as the starting point:

- firmware: [`../exemples/usb/usb_custom_bulk_loopback.cpp`](../exemples/usb/usb_custom_bulk_loopback.cpp)
- nRF52840 project: [`../ARM/Nordic/nRF52/nRF52840/exemples/UsbCustomBulkLoopback/ioc`](../ARM/Nordic/nRF52/nRF52840/exemples/UsbCustomBulkLoopback/ioc)
- host test: [`../Python/usb_custom_bulk_loopback.py`](../Python/usb_custom_bulk_loopback.py)

## What the application configures

The application chooses the behavior and storage for the custom interface:

- USB controller number;
- subclass and protocol values;
- interface string index;
- RX and TX FIFO memory;
- byte-stream or packet TX mode;
- optional full-speed and high-speed packet sizes;
- optional `DeviceIntrf` event callback.

The application does **not** choose an interface number or endpoint number. `UsbdBulk` requests one interface and one bidirectional bulk endpoint number from the internal USB function allocator.

That keeps endpoint placement out of reusable application code and allows the same custom function to move when it becomes part of a composite USB device.

## Minimal setup

```cpp
#include "cfifo.h"
#include "usb/usb.h"
#include "usb/usbd_bulk.h"

#define USB_DEVNO 0
#define CUSTOM_RXFIFO_PKTCNT 4
#define CUSTOM_RXFIFO_MEMSIZE \
    USBD_BULK_RXMEM_SIZE(CUSTOM_RXFIFO_PKTCNT)
#define CUSTOM_TXFIFO_MEMSIZE CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_RxFifoMem[CUSTOM_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_TxFifoMem[CUSTOM_TXFIFO_MEMSIZE];

static UsbdBulk g_CustomBulk;

static const UsbdBulkCfg_t s_BulkCfg = {
    .bBlocking = true,
    .RxFifoMemSize = CUSTOM_RXFIFO_MEMSIZE,
    .pRxFifoMem = s_RxFifoMem,
    .TxFifoMemSize = CUSTOM_TXFIFO_MEMSIZE,
    .pTxFifoMem = s_TxFifoMem,
    .DevNo = USB_DEVNO,
    .SubClass = 0U,
    .Protocol = 0U,
    .InterfaceString = 4U,
    .FsMps = 0U,
    .HsMps = 0U,
    .Mode = USBD_BULK_MODE_BYTE,
    .EvtCB = nullptr,
};
```

Initialize the USB device first, then the custom function:

```cpp
if (!UsbInit(&s_UsbCfg))
{
    return -1;
}

if (!g_CustomBulk.Init(s_BulkCfg))
{
    return -1;
}

(void)UsbEnable(USB_DEVNO);
```

`UsbdBulk::Init()` performs the interface and endpoint allocation internally. There is no endpoint constant to keep synchronized with the descriptor.

## Descriptors

A custom USB device supplies its device, configuration and string descriptors through `UsbCfg_t::DescHandler`.

`UsbdBulk::Init()` fills the descriptor fragment supplied in
`UsbdBulkCfg_t::pDesc` with the allocated interface and bulk OUT/IN endpoints:

```cpp
#pragma pack(push, 1)
typedef struct __Custom_Config_Descriptor {
    UsbCfgDesc_t Config;
    UsbdBulkDesc_t Bulk;
} CustomConfigDesc_t;
#pragma pack(pop)

static CustomConfigDesc_t s_ConfigDesc;

// In the UsbdBulkCfg_t initializer:
.pDesc = &s_ConfigDesc.Bulk,
```

The application still owns the overall configuration descriptor because it decides which USB functions are present. `MakeDesc()` prevents the application from duplicating the interface and endpoint numbers selected by the allocator.

The loopback example contains the complete descriptor handler, including manufacturer, product, serial and interface strings.

## Vendor control requests

Derive the application class from `UsbdBulk` and override `Control()` when the
vendor interface needs endpoint-zero requests. Return `UsbdBulk::Control()` for
requests the derived class does not handle. This keeps control routing on the
registered class object instead of adding a function-pointer registration path.

## Sending and receiving

`UsbdBulk` inherits the normal `DeviceIntrf` data API. A simple loopback is:

```cpp
uint8_t buffer[USB_PKT_MAXLEN(USB_DEVNO, BULK)];
int pending = 0;
int offset = 0;

while (1)
{
    UsbProcess(USB_DEVNO);

    if (pending > 0)
    {
        int n = g_CustomBulk.TxData(&buffer[offset], pending);
        if (n > 0)
        {
            offset += n;
            pending -= n;
        }
        continue;
    }

    int len = g_CustomBulk.RxData(buffer, sizeof(buffer));
    if (len > 0)
    {
        pending = len;
        offset = 0;
    }
}
```

RX preserves USB packet boundaries internally. In byte mode, TX accepts a byte stream and packetizes queued data up to the active endpoint MPS.

## Byte mode and packet mode

| Mode | TX FIFO | Use when |
| --- | --- | --- |
| `USBD_BULK_MODE_BYTE` | byte CFifo, for example `CFIFO_MEMSIZE(1024)` | the application has a stream of bytes and does not need to preserve each USB IN packet boundary |
| `USBD_BULK_MODE_PACKET` | packet CFifo, normally sized with `USBD_BULK_TXMEM_SIZE()` | each application write represents one USB packet, including explicit zero-length packets |

`FsMps == 0` selects `USBD_BULK_FS_MPS`; `HsMps == 0` selects `USBD_BULK_HS_MPS` on a high-speed-capable controller.

## Adapting it to a product

`Custom Bulk` is only the example name. An application can call the object and interface whatever matches its protocol, for example:

```cpp
static UsbdBulk g_Programmer;
static UsbdBulk g_TestPort;
static UsbdBulk g_DataLink;
```

Change the product and interface strings, subclass, protocol and optional request handler to match the application. Use VID/PID values assigned to the actual product before shipping.

Do not add fixed interface or endpoint numbers to the application configuration. The allocator is responsible for placement.

## Host-side test

The example host program uses PyUSB. From the IOsonata repository virtual environment:

```bash
./.venv/bin/python3 Python/usb_custom_bulk_loopback.py
```

The script validates the expected product string and prints the selected device identity before transferring data:

```text
VID:PID        : 1209:0002
Manufacturer   : I-SYST
Product        : IOsonata Custom Bulk Loopback
Serial         : 3763623F4A40D459
Interface      : Custom Bulk (0)
Bulk OUT       : 0x01
Bulk IN        : 0x81
Bytes          : 212000
Bytes/sec      : 165947.57
Result         : PASS
```

The endpoint addresses are discovered from the USB descriptors; the host test does not assume endpoint 1.

If several devices with the same VID/PID and product string are attached, select one by serial number:

```bash
./.venv/bin/python3 Python/usb_custom_bulk_loopback.py \
    --serial 3763623F4A40D459
```

The script refuses an ambiguous match rather than silently choosing one device.
