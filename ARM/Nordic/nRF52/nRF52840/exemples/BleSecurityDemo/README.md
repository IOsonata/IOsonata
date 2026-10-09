# BLE Security Demo (nRF52840)

Import `ioc/.project` as **BleSecurityDemo**. This application links
`exemples/bluetooth/ble_security_demo.cpp` and uses the local `src/board.h`
for physical pins. Build the matching nRF52840 IOsonata MCU library first.

The security **library** owns SMP/LESC, key distribution, peer identities,
bonding and PDS/NVM storage. This demonstration owns only the UART-based user
interaction, method-selection console and optional OOB transport.

UART console at 115200 baud:

- `sec` prints the current association method
- `sec numcomp`, `sec justworks`, `sec passkey-disp`,
  `sec passkey-input`, or `sec oob` selects the next pairing method
- `oob` displays local OOB data; `oob peer <hex>` supplies peer data
- `bond del` requests deletion of stored bonds
- For Numeric Comparison type `y` or `n`; for Passkey Entry type six digits

Optional `BLE_SC_OOB_NFC` requires the platform NFC transport.
Association changes apply to subsequent pairing procedures, not an active one.

The companion `UartBleDemo` is a plain, open-link UART/BLE bridge without
the security command interpreter. This demo preserves the broader security
test matrix rather than implementing the SMP protocol itself.

The project definition was generated from UartBleDemo and structurally checked,
but a clean ARM firmware build and target smoke test have **not** been run.
