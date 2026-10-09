# Bluetooth User Guide

Use this guide to build BLE applications with IOsonata 0.13. It covers the
application API, GATT data paths, security, memory configuration and examples.
See [Bluetooth architecture](architecture/bluetooth.md) for the host,
controller and port boundaries.

## Choose the target and stack first

Build the MCU library and application with matching stack configurations.
A generic header declaring a feature does not mean every controller or vendor
host supports it.

| Configuration family | Host implementation | Starting point |
|---|---|---|
| Nordic SDC | IOsonata generic HCI host above SoftDevice Controller | Matching `Debug_SDC` or `Release_SDC` project |
| nRF52 SoftDevice | Nordic host/controller, adapted by IOsonata | Selected target's SoftDevice configuration |
| nRF54 bare-metal SDK / S145 | Nordic host/controller, adapted by IOsonata | Selected nRF54 project's non-SDC configuration |
| STM32WBA | ST vendor stack through the IOsonata port | Inspect the target project and vendor dependencies |
| nRF91 | IOsonata generic HCI host, HCI over UART to an nRF5x running HciController | nRF9160 or nRF91x1 library, see [HCI over UART](#hci-over-uart-nrf91) |

SDC and SoftDevice are different stack arrangements. Do not combine their
libraries, memory layouts or initialization code. The STM32WBA source is an
integration reference, not a hardware-validation claim for this release.
Check [Supported Targets](supported-targets.md) for recorded board status.

In IOcomposer, open the target's `ioc/` directory, build its matching MCU
library, then clean-build, flash and run the application. Keep pins and board
wiring in `board.h`. See [Getting Started](getting-started.md).

## Pick an example

These project names exist under
[the nRF52840 examples](../ARM/Nordic/nRF52/nRF52840/exemples).
Other MCU directories provide their own subsets.

| Task | Shared source | Project |
|---|---|---|
| Broadcast manufacturer data | [ble_advertiser.cpp](../exemples/bluetooth/ble_advertiser.cpp) | BleAdvertiser/ioc |
| Scan advertising reports | [ble_central_scan.cpp](../exemples/bluetooth/ble_central_scan.cpp) | BleCentralScanDemo/ioc |
| Peripheral GATT and pairing UI | [uart_ble.cpp](../exemples/bluetooth/uart_ble.cpp) | UartBleDemo/ioc |
| Central connection and discovery | [uart_ble_central.cpp](../exemples/bluetooth/uart_ble_central.cpp) | UartBleCentralDemo/ioc |
| GATT through DeviceIntrf | [bleintrf_prbs_tx.cpp](../exemples/bluetooth/bleintrf_prbs_tx.cpp) | BleIntrfPrbsTx/ioc |
| Periodic advertising | [ble_periodic_advertiser.cpp](../exemples/bluetooth/ble_periodic_advertiser.cpp) | BlePeriodicAdvertiser/ioc |
| Periodic synchronization | [ble_periodic_sync.cpp](../exemples/bluetooth/ble_periodic_sync.cpp) | BlePeriodicSync/ioc |
| TaktOS integration | [uart_ble_taktos.cpp](../exemples/bluetooth/uart_ble_taktos.cpp) | UartBleTaktOS/ioc |

The periodic projects use SDC configurations. Use the paired advertiser and
synchronizer on two boards to exercise periodic traffic. Read each example's
board and console setup before substituting hardware.

## Application lifecycle

Keep `BtAppCfg_t`, its referenced data, service definitions and buffers alive
for the application lifetime. The target port owns stack startup and event
processing. With a complete static `s_BtAppCfg` from the selected example,
the bare-metal entry sequence is:

```cpp
#include "app_evt_handler.h"
#include "bluetooth/bt_app.h"

// After board initialization and with s_BtAppCfg defined:
static int RunBluetooth()
{
	if (!BtAppInit(&s_BtAppCfg))
	{
		return -1;
	}
	AppRun();
	return 0;
}
```

`AppRun()` is the bare-metal main loop of the application, shared by every
subsystem. It runs the application event queue, first in first out, and
waits for the next interrupt when it is empty. The Bluetooth stack queues its
deferred work there through `BtEvtQue()`; its first queued event starts
advertising for broadcaster/peripheral roles. Put ongoing work in the
existing callbacks, timers or application events (`AppEvtHandlerQue()`). An
application with its own loop calls `AppEvtHandlerExec()` in it instead.

Use `BtAppInitUserServices()` to add services and `BtAppInitUserData()` for
application initialization, including explicit security initialization.
The current Nordic and ST implementations call the service hook before the
user-data hook; do not depend on the contrary ordering in the older header
comment. Hooks return void, so retain and check application-specific setup
failures before entering the run loop.

Do not read or modify `g_BtAppData` from application code. Use public state
queries and connection handles.

## Roles, advertising and scanning

`Role` selects broadcaster, observer, peripheral, central or mixed behavior.
The link-count field names describe the remote devices:

- `PeriphDevMax`: peripherals this device connects to as a central.
- `CentralDevMax`: centrals this device serves as a peripheral.

Set only the capacities the application needs. On ports with optional
connection support, adding a GATT service or calling `BtGapConnect()` brings
in that support. A connectable peripheral with no service must call
`BtAppConnInit()` from `BtAppInitUserServices()`. This opt-in entry point
is not implemented by every older port.

`BtAppCfg_t` holds the device name, service UUID list, manufacturer data and
advertising settings. `AdvInterval` and connection intervals are in
milliseconds; the public `AdvTimeout` field is in seconds. Some older
examples use a misleading MSEC name/comment for that timeout. Check the
selected port's timeout support rather than copying that label.

`BtAdvEncode()` places records in legacy advertising/scan-response packets
when they fit and selects extended advertising when needed. Extended data
requires controller support; increasing a buffer is not enough to add it.

For scanning:

1. Initialize the application with a central or observer role.
2. Set up `BtGapScanCfg_t` and check `BtAppScanInit()`.
3. Call `BtAppScan()`, then keep the event loop running.
4. Implement `BtAppScanReport()`; return true to continue scanning or false
   to stop.

Copy report data that must outlive the callback. Parse advertising lengths
before reading records. The SDC scan initialization reference also selects
controller scan support at link time.

## Connections and GATT

Use the central example for the sequence of scan, peer selection, connection,
service discovery and notification subscription. A connection request being
accepted does not mean the link is already established.

Use `BtAppEvtConnected(ConnHdl)`, `BtAppEvtDisconnected(ConnHdl)` and
`BtAppEvtSecured(ConnHdl)` for the relevant state transitions.
Use `BtAppDiscoverDevice()` and `BtDeviceDiscovered()` for asynchronous
discovery. Check service/characteristic lookup results before using handles.

Define static `BtGattSrvc_t` and characteristic arrays using the patterns in
[bt_gatt.h](../include/bluetooth/bt_gatt.h) and `uart_ble.cpp`.
Register services with `BtGattSrvcAdd()` in the service hook and check its
return value. Assign the characteristic properties, permissions, maximum
value length and read/write callbacks the application actually needs.

Notification and indication subscriptions are per connection:

- `BtAppNotify()` and `BtAppIndicate()` return the number of links that
  accepted the operation in the shared implementation.
- `BtAppNotifyConn()` and `BtAppIndicateConn()` address one link and return
  a boolean acceptance result.
- An unsubscribed link is skipped. An indication remains outstanding until
  confirmation, so another indication on that link can be refused.
- Acceptance is not an application-level acknowledgement from the peer.

Account for negotiated per-link MTU and characteristic capacity. Configuring
a large `MaxMtu` does not force the peer to negotiate it.

Use `BtAppDisconnectConn()` to select a link and `BtAppDisconnectAll()`
to disconnect all links. The no-argument `BtAppDisconnect()` selects the
first connected link; it is not a broadcast operation.

## BtIntrf: a GATT data interface

`BtIntrf` derives from `DeviceIntrf` and wraps a service's RX write
characteristic and TX notification characteristic. It shares the C
implementation through `BtDevIntrf_t`; it does not implement another stack.

Configure `BtIntrfCfg_t` with the service, characteristic indices, payload
`PacketSize`, FIFO memory and event callback. Use separate static RX and TX
arrays sized with `BTINTRF_CFIFO_TOTAL_MEMSIZE(packetCount, payloadSize)`
and aligned for `CFifo_t`. The macro includes packet and FIFO metadata.
`PacketSize` must fit the class transfer buffer and TX characteristic.

The small default FIFOs support one owner per direction and a 20-byte payload.
Supply explicit storage for larger packets or multiple instances.

Check the byte count returned by `Tx()` and retain any unaccepted bytes.
`RequestToSend()` checks queue capacity; it does not prove peer delivery.
The current implementation sends to the first subscribed peer. Use the
per-connection GATT API for explicit multi-link routing.

RX is packet-backed: a read consumes one queued packet and copies at most the
requested length. A smaller destination does not preserve the rest for a later
read. Size the receive buffer for the configured packet payload.

Initialization installs the service context, RX write callback and TX
completion callback. Do not overwrite them after initialization. Follow
`bleintrf_prbs_tx.cpp` for the application-facing setup.

## Pairing, security and bond persistence

On the updated ports, setting `SecType` alone does not link/start security.
Call `BtAppSecInit()` from `BtAppInitUserData()` and check its result.
`BtAppInit()` rejects a non-NONE security mode if security was not started.

`SecExchg` describes local user interaction capabilities, not a list of keys
to distribute. The UART peripheral and central examples implement pairing
interaction through a console. Generic SMP exposes:

| Interaction | Application callback | Reply |
|---|---|---|
| Numeric comparison | `BtSmpNumericComparison()` | `BtSmpNumericComparisonReply()` |
| Display passkey | `BtSmpPasskeyDisplay()` | Display the value |
| Enter passkey | `BtSmpPasskeyRequest()` | `BtSmpPasskeyReply()` |
| Out-of-band exchange | See `BtSmpOobLocalDataGen()` | Supply peer data with `BtSmpOobPeerDataSet()` |

Preserve the connection handle through deferred user interaction. Reject a
mismatch or cancellation rather than accepting automatically. Use the chosen
port's security adapter; the generic SMP engine is not the pairing engine
inside a Nordic SoftDevice.

A successful pairing does not alone prove that a bond survives reset.
IOsonata uses one portable bond table and persistence format for the generic
SMP host and the Nordic SoftDevice ports. The
[bt_smp_bond_nvm.cpp](../src/bluetooth/bt_smp_bond_nvm.cpp) adapter stores that
table through PDS on an `Nvm`; the selected linker script must reserve the
configured NVM region and the port calls `BtSmpBondNvmInit()` when security is
enabled. Nordic SoftDevice builds do not use Peer Manager, FDS or
`auth_status_tracker` for Bluetooth security persistence.

Validate reconnect both before and after power cycling, and exercise storage
failure and bond deletion. Keep bond storage separate from firmware and any
USB MSC medium. The small `ble_smp_test.cpp` demo is not a substitute for
persistent-bond validation.

## Periodic advertising and synchronization

The supplied examples use the generic HCI path with SDC. Reserve controller
resources in `BtAppCfg_t` before startup:

- `PeriodicAdvCount` for transmitted trains.
- `PeriodicSyncCount` for received trains.
- `PawrAdvCount` and `PawrSyncCount` only when using supported PAwR features.

Zero reserves no resources. A controller can reject unsupported reservations.

The advertiser first configures an extended, non-connectable,
non-scannable set. After `BtAppInit()`, it calls `BtPadvInit()`,
`BtPadvDataSet()` and `BtPadvStart()`; the first queued Bluetooth event
enables the underlying advertising set once `AppRun()` runs. Follow the example's ordering.

The synchronizer scans extended reports, selects the address and SID, calls
`BtPsyncCreate()`, and handles `BtPsyncEstablished()`, `BtPsyncReport()`
and `BtPsyncLost()`. Allow only one pending create attempt in this pattern.

The periodic interval fields use 1.25 ms units; sync timeout uses 10 ms units.
They do not use the same units as application advertising intervals.
The PAwR example switch additionally requires a controller and board
configuration that reserve response resources.

## Static memory and 0.13 migration

Rebuild the MCU library and application together after public layout changes.
Inspect the map file for linked code and pools rather than treating a
configuration macro as proof that precompiled storage changed.

| Resource | Configuration | Notes |
|---|---|---|
| Peer records | `pPeerPoolMem`, `PeerPoolMemSize` | Size with `BT_PEER_POOL_MEMSIZE(N)`; follow the alignment in `bt_peer.h` |
| Long-write assembly | `pLongWrPoolMem`, `LongWrPoolMemSize` | Split across peer slots; null disables this supplied pool |
| Generic ATT database | `g_BtAttDBMemCfg` | Align for `BtAttDBEntry_t`; registration fails when full |
| Discovery caches | `g_BtDevSrvcCacheCfg` | Reserve for peers whose discovered databases coexist |
| SDC controller pool | `g_BtHciCtlrMemPool` | 8-byte alignment; depends on role, links and periodic resources |
| Application event queue | `g_AppEvtHandlerQueMem`, `AppEvtHandlerInit()` | Owned by the application, not by `BtAppCfg_t`; not linked when `BtEvtQue()` is overridden |

These controls apply where the selected port consumes them; vendor-host
memory can have additional SDK-specific requirements.

The nRF52 application path uses AppEvt as its only deferred-work queue.
`BtAppInit()` does not initialize `app_scheduler`. The nRF52832/840 library
configurations retain interrupt dispatch for SDK SoftDevice, timer and
power-management callbacks. The SDK scheduler remains available to SDK-only
examples that initialize and run it themselves; it does not queue AppEvt work.

An application can override the public weak pool descriptors to replace their
default storage. Include the declaring header and define the descriptor once.
An application-only macro cannot resize an already compiled library array.
`AttDBMemSize` limits the database portion used; it does not allocate memory.

For SDC startup failure, inspect `BtHciCtlrErrorGet()`,
`BtHciCtlrErrorValueGet()` and `BtHciCtlrMemPoolSizeNeeded()`. The needed
size is meaningful only after controller configuration reached that stage.
Do not copy the broadcaster's 2400-byte pool into a connected application
without checking the new requirement.

Omitting security, discovery or connection functionality can remove their
code and storage on the updated ports. Savings are feature- and port-specific.

## HCI transports and USB

HCI separates the host from the controller. An application using the generic
host above an on-chip SDC controller is different from a controller exposed
to an external host over USB.

`BtHciUsb` implements the USB device-side HCI transport, including its SCO
option; it does not enable the generic host's GATT services or SMP by itself.
See [the USB guide](usb-user-guide.md#bluetooth-hci-over-usb).
The maintainer tested the USB transport with HciController; that application
is not supplied as a target project in this repository.

### HCI over UART (nRF91)

The nRF91 has no Bluetooth radio. Its port (`bt_app_nrf91.cpp`) runs the
generic host and reaches the controller of an nRF5x running HciController
through `BtHciUart`, the H4 transport in `bluetooth/bt_hci_uart.h`. The same
`BtAppInit()`, `AppRun()` and GATT code as the SDC port apply.

The application defines the UART to the controller as `g_BtHciUartCfg` in a
file that includes `bluetooth/bt_hci_uart.h`. Use hardware flow control and
the rate HciController uses. Leave `pRxMem` and `pTxMem` NULL to take the
port's FIFOs, 2 KB RX and 1 KB TX; a TX FIFO given by the application must
hold one whole packet, `BT_HCI_UART_TXFIFO_MIN` bytes.

- `BtAppInit()` sends HCI Reset, retried while the controller starts, then
  reads the LE ACL buffers for fragmentation and credits. It fails when the
  controller does not answer.
- The device address is a static random address made from the nRF91 device
  identifier.
- Received packets are framed from `BtEvtQue()`, not in the UART interrupt.
  A command waits for its response in the calling context. Call the stack from
  the context that runs the `BtEvtQue()` work, never from an interrupt; with an
  RTOS, from the Bluetooth task.
- The UART driver drops bytes its RX FIFO cannot hold. There is no HCI
  controller-to-host flow control, so size the RX FIFO for the traffic.

Not supported on this port yet: `BtAppSecInit()` (pairing and bonding),
Encrypted Advertising Data and the `TxPower` of `BtAppCfg_t`. Build
`uart_ble.cpp` with `BLE_SC_METHOD=0`, as the nRF9160 `UartBleDemo` project
does.

[UartBleDemo](../ARM/Nordic/nRF91/nRF9160/exemples/UartBleDemo) targets the
nRF9160 DK, with the nRF52840 of the DK running HciController on the
interface lines. Check the pins in its `board.h` against the board revision
and the HciController build; the nRF52840 must route those lines.

## RTOS integration

Override `BtEvtQue()`. The stack calls it, often from interrupt context, for
every piece of work that must run outside the interrupt; send the three
values as a message to the Bluetooth task with an ISR-safe send, and have the
task run `Handler(EvtId, pCtx)` for each message. The port timers keep
queuing the timeout checks, so a silent link is still serviced. Create the
message queue before `BtAppInit()`: the stack can queue work as soon as it is
enabled.

Follow `uart_ble_taktos.cpp` (TaktOS queue) or `UartBleFreeRTOS.cpp`
(FreeRTOS queue) for the bridge pattern.

## Test and troubleshoot

Run the desktop suites from the repository root:

```bash
make -C tests/bluetooth/host
make -C tests/bluetooth/compliance test
make -C tests/bluetooth/compliance strict
```

They use production host code, test controllers/stubs and sanitizers.
See the [host test README](../tests/bluetooth/host/README.md) and
[compliance README](../tests/bluetooth/compliance/README.md).
They do not validate RF behavior or establish Bluetooth qualification.

| Symptom | Check |
|---|---|
| Initialization fails | Matching stack/library, security init, controller error and pool size |
| Connectable device never connects | Role, link capacity, service setup and optional connection initialization |
| Notification is refused | Subscription, live handle, MTU, characteristic length and TX capacity |
| Pairing waits indefinitely | User reply callback, connection handle and event/timeout processing |
| Bond disappears after reset | Persistence adapter, reserved storage and save completion |
| Periodic command is refused | SDC configuration, controller capability and reserved counts |
| Data stops after sleep | Event dispatch, queued work and timer wakeups |

For a hardware report, retain firmware/library revisions, board, stack variant,
SDK/controller version, peer, PHY, MTU, connection interval and full error
counters. No new radio or hardware measurements are claimed by this guide.
