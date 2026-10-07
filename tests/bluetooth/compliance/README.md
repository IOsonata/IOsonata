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

## Current suites

The HCI suite covers:

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

The [Makefile](Makefile) also builds and runs
[dual-host ATT](att/bt_att_dual_host_test.cpp) and
[dual-host SMP](smp/bt_smp_dual_host_test.cpp) tests. These execute both sides
of the corresponding host protocol exchanges.

The HCI requirement runner reports unimplemented requirements as `SKIP`,
never as `PASS`. `make strict` sets `BT_COMPLIANCE_STRICT=1` for that
runner, then runs the SMP and ATT binaries normally. It is not a declaration
that every Bluetooth specification requirement is covered.

## Running

```sh
make -C tests/bluetooth/compliance clean test
```

Strict coverage mode:

```sh
make -C tests/bluetooth/compliance clean strict
```

## Scope and application guidance

Use the test sources and requirement results to determine coverage. HCI, ATT
and SMP suites are implemented; broader GAP/GATT procedures, port integration
and new adversarial cases should be assessed individually rather than treated
as covered by the suite name.

See the [Bluetooth User Guide](../../../docs/bluetooth-user-guide.md) for
application setup and the [architecture](../../../docs/architecture/bluetooth.md)
for implementation boundaries.

The suite validates the IOsonata host and target integration above HCI. RF,
PHY and over-the-air Link Layer qualification still require real controllers,
Bluetooth SIG PTS and RF test equipment.
