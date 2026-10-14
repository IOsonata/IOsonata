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

## Optional application interaction callbacks

The default Just Works configuration needs no callbacks. For authenticated
association methods, provide only the required synchronous application hook:

```cpp
bool BtAppPairConfirm(uint16_t conn, uint32_t number);
bool BtAppPasskeyShow(uint16_t conn, uint32_t passkey);
uint32_t BtAppPasskeyInput(uint16_t conn);
```

The application's implementation presents/obtains the user value and returns
the result. The Bluetooth library performs the actual SMP or SoftDevice
reply; the application must not call `BtSmpNumericComparisonReply` or
`BtSmpPasskeyReply`. A returned `false` rejects confirmation/display,
and `BT_SMP_PASSKEY_INVALID` cancels passkey input.

Callbacks are invoked from Bluetooth event processing and must return promptly.
Do **not** wait synchronously for button/console events inside a callback.
For a product needing asynchronous UI, use the library's existing explicit
reply API until a deferred decision interface is provided; do not assume
that a callback may return before the user has made a decision. No terminal,
command interpreter or manual transaction handling is required for Just Works.

For LESC OOB, the application supplies the out-of-band transport and stages
peer data through the existing `BtSmpOob*` APIs; cryptography and SMP remain
library-owned.

## Separation of responsibilities

`UartBleDemo` is an open UART-over-BLE data bridge. `BleSecurityDemo`
illustrates configuring the library to secure a connection while retaining
the same service as test traffic. User-interface implementation belongs to
the application, not to the generic BLE library or MCU-specific source.

**Validation:** Source/project references were checked, but a clean target
build and hardware regression of this refactor have not been performed.
