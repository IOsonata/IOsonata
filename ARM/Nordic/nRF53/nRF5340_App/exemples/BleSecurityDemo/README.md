# nRF5340 application-core BLE security demo

This project runs the shared `exemples/bluetooth/ble_security_demo.cpp`.
Build the matching nRF5340_App library configuration before the application.
The network core must run HciController with its nRF5340 IPC transport.

The default pin map is the nRF5340 Audio DK. Define
`NRF5340_DK_PCA10095` for the standard nRF5340 DK.
The console uses 1000000 baud, 8 data bits, no parity, one stop bit and no
flow control.

| Board | Application UART TX | Application UART RX |
| --- | --- | --- |
| Audio DK | P1.05 | P1.04 |
| Standard DK | P0.20 | P0.22 |

Both Debug and Release use `nrf5340_xxaa_application.ld`. It reserves
`NVM0` at `0x000FD000`, size `0x3000`, for Bluetooth bonds and local
identity records. Code flash ends before this region. The store is a
`NOLOAD` section; the image does not contain initial bond data.
A programmer's full-chip erase still clears it.

After rebuilding and flashing, check for:

```text
PDS: linker NVM0 at 000FD000 size 00003000
PDS: store mounted, bond record ...
```

An empty store is normal on the first boot. Pair with the device, wait for
the successful PDS save message, then reset without erasing flash and
reconnect. The boot log should report restored bond slots and the peer
should reconnect using its stored bond.

The current board configuration uses the shared demo's default security
policy (`BTGAP_SECTYPE_STATICKEY_NO_MITM`, no input/output capability).
It does not enable numeric comparison. Set `BLE_SECURITY_TYPE`,
`BLE_SECURITY_EXCHG` and the shared demo's button or UART confirmation
option when testing a different association model.

If security initialization fails, inspect the preceding crypto or store
error. The final `BtAppInit` message only reports that security did not
finish; it does not establish whether `BtAppSecInit` was called.

The DFU linker script is a separate layout and is not selected by this
project. These instructions apply to the normal application image.
