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

## Additional implemented coverage

The suite has expanded beyond the three groups above. The current
[Makefile](Makefile) is the authoritative list of executed tests.

- ATT database sizing, server requests, client transactions and adversarial
  input: `bt_att_*_test.cpp`.
- GATT services, per-link CCCDs, security checks and TX completion:
  `bt_gatt_*_test.cpp`.
- Peer pools, application multi-link behavior and selected Nordic startup
  paths: `bt_peer_pool_test.cpp`, `bt_app_*_test.cpp`.
- SMP key distribution and signing counters: `bt_smp_keydist_test.cpp` and
  `bt_smp_sign_counter_test.cpp`; SoftDevice LESC adapter checks:
  `bt_lesc_sd_test.cpp`.
- Periodic advertising, synchronization and encrypted advertising data:
  `bt_padv_hci_test.cpp`, `bt_psync_test.cpp`, `bt_ead_test.cpp`.
- Controller initialization, HCI USB and GATT-backed transport:
  `bt_hci_ctlr_*_test.cpp`, `bt_hci_usb_test.cpp`, `bt_intrf_test.cpp`.
- HCI UART (H4) framing, packet order, flush and command waits, including a
  command sent from a packet handler: `bt_hci_uart_test.cpp`.

These are software checks with test controllers or port stubs, not a claim of
complete protocol coverage or hardware qualification. The separate
[compliance suite](../compliance/README.md) includes dual-host ATT and SMP
scenarios. For application setup, see the
[Bluetooth User Guide](../../../docs/bluetooth-user-guide.md).

Host tests must remain independent of an RTOS. Scheduling tests should override only
the existing wait/notify hooks when an execution model needs to be exercised.
