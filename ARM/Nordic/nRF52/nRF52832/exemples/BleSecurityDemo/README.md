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
manual SMP protocol reply is required. The synchronous hooks must not block.
See the [shared security demo guide](../../../nRF52840/exemples/BleSecurityDemo/README.md)
for the supported modes and callback semantics.

This nRF52832 project was derived from the existing MCU-specific UartBleDemo
build configuration. Source references and project structure were checked;
compilation and hardware pairing validation are pending.
