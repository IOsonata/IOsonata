# WBA BLE security integration

Updated October 18, 2026. Target: STM32WBA65I-DK1.
Middleware: STM32CubeWBA 1.10.0 and stm32-mw-wpan 2.10.0.

## Hardware validated

BleSecurityDemo passes fresh Numeric Comparison pairing, encrypted data transfer
and bonded reconnect after reset. PDS saved 88 bytes with CRC `89681f20`;
after reset it restored the same length and CRC. Peer lookup, encryption and
pairing returned status `0x00`, without another confirmation prompt.

The initial Numeric Comparison prompt appeared 197 ms after the security
request. Reconnect encryption was enabled 45 ms after the request.
The earlier central log reported mode 1, level 4 for fresh pairing. The latest
reset/reconnect log records encryption and successful bond reuse.

The standalone hardware crypto test passed **51 tests, 0 failed**. See
[target results](STM32WBA65/README.md#crypto-hardware-validation).

## Crypto integration

`src/bt_wba_pka.cpp` uses `CryptoWbaPka` for P-256 public keys and ECDH.
`src/bt_wba_platform.cpp` uses `CryptoWbaAes` for the ST BAES callbacks.
If AES returns busy, the callback retries with `CryptoSoftAes`.

The adapter converts ST's little-endian ECB key, input and output to the
standard AES byte order. CCM reverses the key; nonce, AAD, payload and tag
retain their order. CMAC uses standard byte order. Temporary conversion
buffers are wiped.

The earlier byte-order mismatch was corrected before the successful reconnect
tests. The previous logs alone did not isolate it as the only reconnect cause.

## Regression tests

Run from the repository root:

```sh
python3 tests/stm32wba/run_ble_crypto.py
python3 tests/stm32wba/run_bt_crypto.py
```

The byte-order tests cover FIPS-197 ECB, RFC 4493 CMAC and RFC 3610 CCM,
including in-place operations and invalid-tag rejection. The hardware adapter
test runs with an instruction-set emulator. These tests are separate from
the DK1 hardware results above; no new test run was performed for this
documentation update.

## Remaining limitations and tests

- `BAES_CmacCompute` returns when the output pointer is null. ST permits that
  form to append input before finalization. Incremental CMAC remains
  unimplemented; resolve or explicitly defer it before merge.
- Pairing rejection, persistence failure recovery, and radio scheduling during
  flash writes need separate tests.
- Complete the final Debug/Release build and regression checks on the release
  candidate, and remove temporary security diagnostics.

## Vendor references

- [ECB](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c)
- [CMAC](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_cmac.c)
- [CCM](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ccm.c)
- [AES register adapter](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Interfaces/hw_aes.c)
- [BLE platform interface](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v2.10.0/ble/stack/include/bleplat.h)
