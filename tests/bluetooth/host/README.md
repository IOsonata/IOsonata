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

## Planned host layers

The next test targets should use a fake HCI controller and feed the production host
entry points directly:

1. HCI command credits and ACL buffer credits.
2. ACL fragmentation and per-connection reassembly.
3. L2CAP signaling, including malformed command lengths.
4. ATT server requests, long reads, prepare/execute writes and permissions.
5. ATT client discovery and transaction completion.
6. SMP association models, timeout paths and bond restore.
7. Simultaneous links with disconnect at each protocol state.
8. Recorded HCI trace replay and mutation fuzzing.

Host tests must remain independent of an RTOS. Scheduling tests should override only
the existing wait/notify hooks when an execution model needs to be exercised.
