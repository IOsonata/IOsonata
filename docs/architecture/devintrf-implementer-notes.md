# DeviceIntrf Implementer Notes

This document describes the C-level `DevIntrf_t` implementation rules used by the generic helpers and the C++ `DeviceIntrf` wrapper.

Read it together with:

- [`include/device_intrf.h`](../../include/device_intrf.h);
- [`src/device_intrf.cpp`](../../src/device_intrf.cpp);
- the complete target implementation being changed;
- at least one working application using that interface.

A common-interface change must be checked across the available Nordic, ST, Renesas, Microchip, NXP, RISC-V and host implementations. One target is not the architecture.

## What `DeviceIntrf` represents

`DeviceIntrf` is a transferable data path. It is not limited to a physical bus.

Physical examples include UART, I2C, SPI, QSPI, OSPI, USB and network interfaces. Software and internal examples include SLIP, Bluetooth GATT presented as a serial interface, crypto-engine access paths and internal memory-controller adapters.

The selector passed as `DevAddr` is interface-defined:

- I2C: device address;
- SPI: chip-select index;
- internal crypto interface: engine or register-space selection;
- internal memory interface: controller-defined address or region;
- UART: normally ignored because the endpoint is selected by the UART instance.

Do not assume that `DevAddr` always means a bus address.

## `DevIntrf_t` state fields

| Field | Implementer rule |
|---|---|
| `pDevData` | Points to implementation-private state. The generic layer does not interpret it. |
| `IntPrio` | Interrupt priority in the target's numbering scheme. Validate the range in the target initializer. |
| `EvtCB` | Application callback. Treat it as interrupt-context unless the concrete interface explicitly documents deferred context. It must not block. |
| `bBusy` | One-transfer serialization flag. The generic start/stop wrappers own it. Implementation hooks must not take or clear it. |
| `MaxRetry` | Number of additional attempts after the first zero-byte transfer result. It is not the total attempt count. |
| `EnCnt` | Shared enable reference count. The physical interface enables on the 0-to-1 transition and disables on the last release. |
| `Type` | Correct `DEVINTRF_TYPE_*` value. Generic devices may use it for interface-specific address or command conventions. |
| `bDma` | Configured DMA intent. It does not by itself prove that the selected path is implemented or asynchronous. |
| `bIntEn` | Configured interrupt intent. It does not by itself define transfer return semantics. |
| `bTxReady` | Completion state used by generic TX completion helpers. Initialize it consistently with the target transfer path. |
| `bNoStop` | Marks a continuous command/data or repeated-start sequence where the transfer must remain open between phases. |

## Function table

New implementations should populate the complete function table:

- `Disable`, `Enable`;
- `GetRate`, `SetRate`;
- `StartRx`, `RxData`, `StopRx`;
- `StartTx`, `TxData`, `TxSrData`, `StopTx`;
- `Reset`, `PowerOff`, `GetHandle`.

Use an explicit no-op or unsupported handler when an operation does not apply. Some generic helpers retain null checks for compatibility with older ports; new code should not depend on those checks.

## Wrapper ownership of `bBusy`

Application and generic code should use:

```text
DeviceIntrfStartRx() ... DeviceIntrfStopRx()
DeviceIntrfStartTx() ... DeviceIntrfStopTx()
```

The start wrapper atomically acquires `bBusy`, then calls the implementation hook. If the hook fails, the wrapper releases the flag before returning.

The stop wrapper calls the implementation hook, then releases `bBusy`.

Implementation hooks must not independently manipulate `bBusy`. Doing so can release another transfer, deadlock a repeated-start sequence or permit concurrent access to the same controller.

## Basic transfer sequences

### Receive

```text
DeviceIntrfStartRx(selector)
    -> implementation StartRx(selector)
    -> RxData(buffer, length)
    -> DeviceIntrfStopRx()
```

### Transmit

```text
DeviceIntrfStartTx(selector)
    -> implementation StartTx(selector)
    -> TxData(data, length)
    -> DeviceIntrfStopTx()
```

The implementation hooks perform interface-specific work such as selecting a controller window, asserting chip select, preparing DMA, generating a bus condition or doing nothing for a preselected stream.

## Command-then-read and repeated start

`DeviceIntrfRead()` performs one serialized operation:

```text
acquire busy with DeviceIntrfStartTx()
    -> send command/address phase
    -> keep the transfer open
    -> call the raw StartRx hook for the receive phase
    -> receive data
    -> DeviceIntrfStopRx() releases the operation
```

The receive phase calls the raw `StartRx` hook rather than `DeviceIntrfStartRx()`. The busy flag is already held for the combined operation; calling the wrapper again would attempt to acquire it a second time.

`bNoStop` tells completion handling not to close the transfer between the command and data phases.

`TxSrData` exists for controllers with a specialized write/restart/read address phase. A target that does not need it may forward to its ordinary TX implementation.

## Return values and completion

Do not infer completion solely from `bIntEn` or `bDma`.

Common patterns are:

- synchronous direct transfer: return the number of bytes moved;
- interrupt or DMA transfer handed to hardware: return `-1`, then report completion later;
- FIFO-backed UART TX: return the number of bytes accepted into the software FIFO, while the ISR or DMA drains it later;
- retryable zero-byte result: return `0`, allowing the generic retry loop to make another attempt;
- permanent failure: use the interface's documented error convention.

A positive UART return does not necessarily mean every byte is already on the wire.

An interface event reports transport activity or transport completion. A higher-level device operation may continue after the transfer completes. For example, NVM programming or erase completion is separate from the SPI/I2C transfer that issued the command.

## Events

The callback can receive:

- RX timeout and RX data;
- RX FIFO full;
- TX timeout, TX ready and TX FIFO empty;
- state changes;
- read and write requests from a peer or host;
- transfer completion.

Event-buffer meaning is implementation-specific. Document whether the callback receives a direct buffer, a FIFO count, state data or no buffer.

The callback must not block. If substantial processing is required, record the minimum state and defer the work.

## Enable, disable and power off

`Enable` and `Disable` are a shared sleep/wake pair. `Disable` must leave the interface in a state that `Enable` can restore without a complete initialization sequence.

`EnCnt` prevents one device from disabling an interface still used by another device.

`PowerOff` is different. It is terminal until full initialization and therefore has no matching `PowerOn` helper.

## Polling, interrupts and DMA

Execution mode is chosen by the actual controller, direction, transfer length, latency requirement and application usage.

- UART streams are commonly interrupt/FIFO driven.
- Short I2C and SPI master transactions are often faster and simpler in polling mode.
- Long transfers may justify DMA or interrupt completion.
- Slave operation normally requires interrupt handling because the external master controls progress.

A configuration field does not prove that every target path is complete. Read the initializer, transfer functions and interrupt handler before documenting or changing a mode.

## Chunking and transfer limits

Do not add a target transfer limit to `DeviceIntrf`.

- Controller or DMA limits belong in the target interface implementation.
- Device page, erase, frame or protocol boundaries belong in the device driver.
- The application should not have to split transfers because one target controller has a smaller DMA count field.

Chunking is therefore not a generic `DeviceIntrf` capability field.

## Device-operation locking

`bBusy` protects one interface transfer. A device operation can span several transfers and may need its own operation state or lock.

Examples include:

- crypto operations that hold an engine across several register or memory transfers;
- NVM write/erase state machines;
- protocol transactions containing several messages.

That device-level state supplements `DeviceIntrf`; it does not bypass it.

## C and C++ implementations

The C structure, C helpers and C++ class use the same driver implementation. Do not create a separate C++ access path beside the C function table.

A new interface should initialize the C structure completely, then expose the same object through the C++ wrapper when C++ use is required.

## Review checklist

Before accepting a new or changed interface implementation, verify:

- every field is initialized;
- the function table is complete;
- hooks do not manipulate `bBusy`;
- repeated-start handling does not reacquire the busy flag;
- synchronous, FIFO and deferred return values are documented;
- event context and buffer meaning are documented;
- enable reference counting is preserved;
- target transfer limits are handled inside the target port;
- device-level multi-transfer state is kept in the device;
- polling, interrupt and DMA paths are checked in the actual target code;
- at least one working example exercises the path.
