# IOsonata Bluetooth host status

This document describes the IOsonata-owned Bluetooth LE host used by the Nordic
SoftDevice Controller integration. It separates implemented protocol code from
controller capability and from completed verification. Those are different states.

## Architecture

```text
BtApp / BtIntrf
    |
GAP, GATT and SMP application operations
    |
ATT, L2CAP and SMP protocol engines
    |
generic HCI host
    |
BtHciCtlrDev_t
    |
controller integration, currently Nordic SDC
```

The host does not require an RTOS portability layer. Bare metal, TaktOS, FreeRTOS
and ThreadX can provide their own execution model around the same host by overriding
the existing event wait and notification hooks.

Memory for peers, long writes and application event queues can be supplied by the
application. The host does not require a general-purpose heap for protocol state.

## Implementation matrix

| Area | Implemented | Partial or not yet exposed |
|---|---|---|
| HCI | Command and ACL paths, command credits, ACL credits, TX fragmentation, per-link RX reassembly, bounded packet validation | A second standard-HCI transport has not yet been demonstrated |
| GAP roles | Broadcaster, observer, peripheral, central and mixed central/peripheral | Some advanced procedures lack public operations |
| Advertising | Legacy and extended encoding, manufacturer data, UUID lists, scan-response packing, random advertising-set address | Periodic advertising host operations are not complete |
| Scanning | Legacy and extended scan commands and report delivery | Generic AD iterator/typed decoder is still minimal |
| Connections | Create, disconnect, enhanced connection complete, data length and PHY event handling | Remote Connection Parameter Request reply/update policy is incomplete |
| ATT server | MTU exchange, discovery requests, reads, blobs, multiple reads, writes, signed writes, prepare/execute writes, notifications and indications | EATT is not implemented |
| ATT client | Primary-service, characteristic and CCCD discovery; read/write request generation | General transaction result delivery is incomplete for several response types |
| GATT | Static service database, per-peer CCCDs, persistent CCCDs, notification/indication completion and timeout | Database Hash and Robust Caching procedures are incomplete |
| L2CAP | Fixed channels and signaling parser, including LE credit-based signaling opcodes | LE CoC dynamic channel, credit and SDU data engine is incomplete |
| SMP | Initiator and responder, LE Secure Connections, Just Works, Numeric Comparison, Passkey Entry, SC OOB, key distribution and timeouts | Verification breadth is still behind implementation breadth |
| Bonding | LTK/IRK/CSRK storage, RPA resolution, signed-write counter, CCCD persistence, CRC/versioned records | Controller resolving-list synchronization and automatic RPA rotation are incomplete |
| OOB | LE Secure Connections OOB payload and NFC NDEF publication | Additional OOB transports are application work |
| Services | GAP, GATT, Service Changed, DIS and BAS plus custom static services | Standard-service catalogue is intentionally limited |
| IOsonata integration | `BtIntrf` maps GATT characteristics to `DeviceIntrf` with caller-owned FIFOs and completion-driven TX | Some `DeviceIntrf` lifecycle/rate methods remain placeholders |

## Verification status

Existing verification includes protocol self-tests in the SMP crypto path and
hardware examples for advertising, central operation, GATT data exchange, bonding
and security.

The desktop host test work begins in `tests/bluetooth/host`. The first target runs
the production advertising and UUID sources under AddressSanitizer and
UndefinedBehaviorSanitizer.

The verification sequence is:

1. Pure encoders and bounded parsers.
2. Fake-controller HCI command and ACL flow control.
3. ACL fragmentation and per-link reassembly.
4. L2CAP signaling and malformed packet handling.
5. ATT server request matrix and permissions.
6. ATT client transactions and discovery.
7. SMP association-model and failure matrix.
8. Bond corruption, interrupted persistence and restore.
9. Multi-link state isolation and disconnect at every state.
10. Recorded HCI trace replay, mutation fuzzing and Bluetooth PTS records.

A feature should be described as production-validated only after its relevant host
tests, target tests and interoperability evidence exist. Controller support calls
and parsed event constants alone do not establish an application-visible feature.
