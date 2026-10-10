# nRF52832 BLE Security Demo

Import `ioc` as **BleSecurityDemo** and build with the matching nRF52832
IOsonata library. The project uses the shared
`exemples/bluetooth/ble_security_demo.cpp` source and an application-local
`src/board.h` copied from the nRF52832 UartBleDemo configuration.

Choose the security policy with `BLE_SECURITY_TYPE` and
`BLE_SECURITY_EXCHG` in `src/board.h`. The default is bonded Just Works.
For Numeric Comparison, select
`BTGAP_SECTYPE_LESC_MITM` and
`BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO`.

The IOsonata library owns SMP transactions, bonding and persistence. Optional
application interaction callbacks are `BtAppPairConfirm`,
`BtAppPasskeyShow`, and `BtAppPasskeyInput`; no pairing console or
manual SMP protocol reply is required. With `BLE_SECURITY_BUTTON_CONFIRM` enabled in this target's `board.h`, the Numeric Comparison callback displays the number via SysLog and returns a pending decision. Press BUT1 only if both devices show the same six-digit number, or BUT2 to reject. The app-level `BtAppPairDecision()` call is made from `AppCheckStatus()`; IOsonata performs the SMP response. The callback does not block. No security command console is required.
See the [shared security demo guide](../../../nRF52840/exemples/BleSecurityDemo/README.md)
for the supported modes and callback semantics.

This nRF52832 project was derived from the existing MCU-specific UartBleDemo
build configuration. Source references and project structure were checked;
compilation and hardware pairing validation are pending.

## Security rejection test

A BLE ACL connection may remain connected after SMP Pairing Failed. This does
**not** imply that the link has authenticated. The UART GATT service now declares
`.SecType = BLE_SECURITY_TYPE`, so its characteristic reads/writes and CCCD
access must meet the configured security policy. Under LESC_MITM, an
unauthenticated or unencrypted central must **not** read/write UART data,
even when the controller connection remains established.

For the negative test, reject with BUT2. The log records `Pairing numeric
comparison rejected`. Verify that subsequent central UART writes do not reach
the peripheral UART. For the positive test, compare both numbers and press
BUT1; verify `accepted`, DHKey Check and encryption, then UART traffic.
An existing Just Works bond does not satisfy the LESC MITM policy; remove
stale bonds on both peers before comparing new pairing results.
