# Bluetooth host tests

This directory contains controller-free tests for the generic IOsonata Bluetooth host.
The tests compile the production sources directly with the desktop C++ compiler; they
do not replace protocol code with a second test implementation.

## Run

From the repository root:

```sh
make -C tests/bluetooth/host
```

The default build enables AddressSanitizer and UndefinedBehaviorSanitizer. To use a
compiler without sanitizer support:

```sh
make -C tests/bluetooth/host SANITIZERS=
```

## Current coverage

`bt_adv_uuid_test.cpp` verifies:

- legacy versus extended advertising selection;
- AD record insertion, replacement and removal;
- complete and shortened local-name encoding;
- bounded decoding of malformed AD records;
- manufacturer-specific data decoding;
- 128-bit custom UUID extraction and reconstruction;
- standard 16-bit service UUID list encoding;
- complete legacy advertising encoding;
- legacy scan-response placement;
- automatic extended-advertising fallback without name truncation.

`bt_hci_flow_test.cpp` verifies:

- HCI command framing and invalid command parameters;
- command-complete and command-status credit updates;
- ACL credit consumption and completed-packet replenishment;
- outgoing ACL fragmentation and insufficient-credit rejection;
- single-link and interleaved multi-link ACL reassembly;
- orphan and oversized continuation handling;
- bounded malformed legacy and extended advertising events.

`bt_l2cap_signal_test.cpp` verifies:

- unknown and truncated signaling command rejection;
- connection parameter update validation, acceptance and response callbacks;
- LE credit-based connection request and response construction;
- flow-control credit acceptance and invalid-CID rejection;
- disconnection request and response handling;
- unsupported enhanced credit-based connection rejection;
- credit-based reconfiguration requests and response callbacks;
- multiple commands in one signaling PDU;
- bounded response construction when input would generate more responses than fit.

## Planned host layers

1. ATT server requests, long reads, prepare/execute writes and permissions.
2. ATT client discovery and transaction completion.
3. SMP association models, timeout paths and bond restore.
4. Simultaneous links with disconnect at each protocol state.
5. Recorded HCI trace replay and mutation fuzzing.

Host tests must remain independent of an RTOS. Scheduling tests should override only
the existing wait/notify hooks when an execution model needs to be exercised.
