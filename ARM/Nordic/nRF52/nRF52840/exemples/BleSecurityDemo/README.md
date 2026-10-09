# BLE Security Demo — nRF52840

This example uses **declarative security configuration**. There is no pairing
console, command interpreter, or application-managed SMP state machine.
Import `ioc/.project` and build with the matching IOsonata MCU library.

## Configure security in `src/board.h`

Set `BLE_SECURITY_TYPE` and `BLE_SECURITY_EXCHG`; the example passes them
directly to `BtAppCfg_t.SecType` and `BtAppCfg_t.SecExchg`. The defaults
request bonded Just Works pairing without MITM and require no human input.

| Intended association | `BLE_SECURITY_TYPE` | `BLE_SECURITY_EXCHG` |
| --- | --- | --- |
| Just Works, no MITM | `BTGAP_SECTYPE_STATICKEY_NO_MITM` | `BTAPP_SECEXCHG_NONE` |
| LESC Numeric Comparison | `BTGAP_SECTYPE_LESC_MITM` | `BTAPP_SECEXCHG_DISPLAY \| BTAPP_SECEXCHG_YESNO` |
| LESC Passkey display | `BTGAP_SECTYPE_LESC_MITM` | `BTAPP_SECEXCHG_DISPLAY` |
| LESC Passkey entry | `BTGAP_SECTYPE_LESC_MITM` | `BTAPP_SECEXCHG_KEYBOARD` |
| LESC OOB | `BTGAP_SECTYPE_LESC_MITM` | `BTAPP_SECEXCHG_OOB` |

The Bluetooth library owns ECDH, pairing, bond storage, identity resolution,
security event processing and reconnection. The example only calls
`BtAppSecInit()` from `BtAppInitUserData()`.

## When user interaction is necessary

Provide application callbacks *only for the capabilities selected*. The
Bluetooth library invokes:

- `void BtSmpNumericComparison(uint16_t conn, uint32_t value)`: display the
  six-digit value through the application's UI, obtain an explicit yes/no
  decision and call `BtSmpNumericComparisonReply(conn, confirm)`.
- `void BtSmpPasskeyDisplay(uint16_t conn, uint32_t passkey)`: present the
  six-digit passkey to the user.
- `void BtSmpPasskeyRequest(uint16_t conn)`: request six digits using the
  application's own input device, then call `BtSmpPasskeyReply(conn, passkey)`.
  Use `BT_SMP_PASSKEY_INVALID` to cancel.
- For OOB, obtain local values with `BtSmpOobLocalDataGen()` and stage the
  peer data with `BtSmpOobPeerDataSet()` through the application's transport.

These callbacks must never silently approve Numeric Comparison or fabricate a
passkey. They can be asynchronous: retain the connection handle and call the
appropriate reply when the human interaction completes. The library's
rejecting defaults apply when interaction is requested but no application
callback is supplied. There is no UART terminal requirement.

## Separation of responsibilities

`UartBleDemo` is an open UART-over-BLE data bridge. `BleSecurityDemo`
illustrates configuring the library to secure a connection while retaining
the same service as test traffic. User-interface implementation belongs to
the application, not to the generic BLE library or MCU-specific source.

**Validation:** Source/project references were checked, but a clean target
build and hardware regression of this refactor have not been performed.
