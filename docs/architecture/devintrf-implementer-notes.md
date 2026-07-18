# DeviceIntrf Implementer Notes

Scope: the C-level DevIntrf_t foundation fields and conventions an interface
implementer must honor. This complements, and defers to, the authoritative
architecture documentation:

- Beyond Blinky, Chapter 5 "The Device Interface: The Roots" for the design
  philosophy, the bus walkthroughs (UART, I2C, SPI), polymorphism, and protocol
  stacking (SLIP over any root).
- Beyond Blinky, Chapter 6 "The Device: Fruits of the Orchard" for the Device
  class, inheritance families, the Init grafting pattern, and virtual
  inheritance.
- docs/quick-reference.md for practical project usage.

This file exists because the book teaches the model from the user side, while
an interface implementer also needs the exact meaning of every DevIntrf_t
field and the conventions the framework helpers rely on. Those live in
include/device_intrf.h; this is the condensed map.

---

## DevIntrf_t fields the framework relies on

| Field | Implementer obligation |
|---|---|
| `EvtCB` | Set by the application, called from interrupt context. Fire `DEVINTRF_EVT_RX_DATA`, `TX_READY`, `COMPLETED`, `RX_TIMEOUT`, `TX_TIMEOUT`, FIFO and `STATECHG` events as they occur. Callback must not block. |
| `bBusy` | Per transfer re-entrancy lock. The framework helpers `DeviceIntrfStartTx` / `StartRx` take it before calling your `StartTx` / `StartRx`; `DeviceIntrfStopTx` / `StopRx` clear it after your `StopTx` / `StopRx`. Never take or clear it yourself inside the hooks. |
| `MaxRetry` | `DeviceIntrfRx/Tx/Read/Write` loop the whole transfer up to `MaxRetry` times when zero bytes move. Return 0 from `TxData` / `RxData` on a retryable failure to engage it. |
| `EnCnt` | Enable reference count. Interfaces are shared by several devices; the `Enable` hook fires only on 0 to 1, the `Disable` hook only on the last release. Initialize to 0. |
| `bDma`, `bIntEn` | Transfer mode flags. When `bIntEn` is set, `RxData` may return -1: transfer started, completion arrives later through `EvtCB` as `DEVINTRF_EVT_COMPLETED`. |
| `bNoStop` | Continuous transfer: command/response phases without a stop condition between them. Relevant for interrupt driven restart reads. |
| `MaxTrxLen` | Per transaction byte cap, typically the DMA limit. Long transfers are chunked against it by the caller. |
| `Type` | Set the correct `DEVINTRF_TYPE_*`. Devices may branch on it (the Device base adjusts SPI read/write address conventions by it). |

All function pointers in the table are mandatory. Implement unused ones as
do-nothing functions, never NULL.

## The transfer protocol

The framework helpers orchestrate every transfer as:

    StartTx(DevAddr) -> address/command phase -> data phase -> StopTx

1. `StartTx` / `StartRx` receive the DevAddr and save it internally. DevAddr is
   the device selection scheme: I2C slave address, SPI chip select index, a
   memory sub-block selector. SPI saves it as `CurDevCs` (see spi_nrfx.cpp);
   follow that pattern.
2. `TxSrData` handles the write-restart-read address phase where the hardware
   needs it (small data length, see the header comment).
3. `TxData` / `RxData` move data using the saved DevAddr.
4. `StopTx` / `StopRx` complete the transfer. Busy is cleared by the framework
   wrapper, not by you.

`Enable` / `Disable` are the sleep-wake pair: `Disable` must leave the
interface in a state `Enable` can restore without full re-initialization.
`PowerOff` is terminal until re-initialization and has no PowerOn counterpart.

## Reminders that save debugging time

- A device operation that spans several transfers (for example a crypto engine
  holding a hardware module across a whole computation) needs its own
  operation-level lock. `bBusy` is per transfer; the framework takes and
  releases it around every `Read` / `Write`.
- Do not add access APIs beside the virtuals; override `Read` / `Write` or
  implement the transfer hooks. See Beyond Blinky 2.12 (anti-patterns) and
  5.0.7 (misconceptions).
- Asynchronous behavior, retry, chunking and shared enable are foundation
  features. Check the table above before designing any of them into a device
  or a new interface.
