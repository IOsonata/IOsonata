# IOsonata Bluetooth compliance tests

This directory contains the specification-driven Bluetooth host test suite.
It is separate from `tests/bluetooth/host`, which remains the fast unit and
regression suite.

The compliance tests drive IOsonata through the same boundaries used by a real
controller:

```text
IOsonata host
    |
HCI command executor and raw ACL TX
    |
VirtualController
    |
raw HCI events and raw ACL RX
    |
VirtualPeer
```

The simulator uses fixed-capacity storage only. It does not allocate from the
heap, sleep, use wall-clock time, or call IOsonata internals to manufacture an
expected result.

## Current foundation

The initial HCI suite covers:

- deterministic event timing and ordering;
- command status, parameter capture and bounded return data;
- ACL fragmentation and controller credits;
- transport refusal before and after a start fragment;
- malformed Number Of Completed Packets lists;
- short Command Complete events and delayed completion;
- truncated legacy and extended advertising reports;
- orphan ACL continuations;
- interleaved per-link ACL reassembly;
- reassembly-pool exhaustion;
- independent reassembly of fragmented host output by the virtual peer.

Known unimplemented requirements are reported as `SKIP`, never as `PASS`.
Set `BT_COMPLIANCE_STRICT=1` or run `make strict` to fail when any requirement
is skipped.

## Running

```sh
make -C tests/bluetooth/compliance clean test
```

Strict coverage mode:

```sh
make -C tests/bluetooth/compliance clean strict
```

## Expansion order

1. Complete HCI event, command and ACL requirements.
2. L2CAP signaling.
3. ATT client and server procedures.
4. GATT database, discovery, notification and indication procedures.
5. SMP pairing, key distribution, persistence and failure recovery.
6. GAP procedures and multi-link behavior.
7. Vendor-controller port stubs for Nordic and STM32 controller integration.
8. Stateful sequence fuzzing and PTS result import.

The suite validates the IOsonata host and target integration above HCI. RF,
PHY and over-the-air Link Layer qualification still require real controllers,
Bluetooth SIG PTS and RF test equipment.
