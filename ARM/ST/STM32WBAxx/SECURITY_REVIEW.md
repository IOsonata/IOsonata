# WBA BLE security integration review — 2026-10-18

Scope: STM32WBA adapter, compared with STM32CubeWBA v1.10.0 and stm32-mw-wpan v2.10.0. Common IOsonata cryptography and other MCU ports are unchanged. This is a confirmed-finding checkpoint, not release sign-off.

## Confirmed byte-order mismatch, corrected

In `src/bt_wba_platform.cpp`, BAES_EcbCrypt previously passed the ST key and block directly to CryptoSoftAes. ST's baes_ecb.c and hw_aes.c implement Bluetooth little-endian key, input and output order. CryptoSoftAes takes standard AES byte strings. The adapter now reverses those three arrays.

BAES_CcmCrypt also requires a reversed key, as shown by HW_AES_InitCcm's key register loading. Nonce, AAD, payload and tag retain their order. CMAC retains standard byte order, matching ST's implementation.

Temporary conversion buffers are wiped. No common security API or crypto engine changes are needed.

The mismatch can produce incorrect encryption key derivation and is consistent with the reported local 0x3d MIC failure on bond reuse. Hardware confirmation is still required; this review does not establish that it is the only cause.

## Regression checks

Run from repository root:

```sh
python3 tests/stm32wba/run_ble_crypto.py
```

Uses the actual adapter section and CryptoSoftAes with only Device lifecycle stubbed. FIPS-197 ECB encryption (including in-place), RFC 4493 CMAC example 2, and RFC 3610 CCM vector 1 cover encryption, decryption, in-place decryption and rejection of an invalid tag.

Original adapter: ECB and CCM vector checks fail. Corrected adapter: all checks pass at host -O0 and -Os. These are not ARM Debug/Release firmware builds.

BLEPLAT exposes ECB encryption only. Raw ECB decryption remains unsupported by the software adapter; the test does not claim it is supported.

## Additional confirmed integration gap

BAES_CmacCompute ignores calls with a null output pointer. Contrary to the adapter comment, the vendor BLEPLAT interface explicitly permits such calls to append data before finalization. ST's baes_cmac.c implements this state. Current one-shot CMAC passes, but incremental CMAC is not implemented. This remains an integration issue to resolve before claiming complete platform compatibility; it is not proven to cause the observed bonded reconnect failure.

## Remaining verification

Rebuild the IOsonata WBA library and BleSecurity application. Test reconnect with the existing bond first, without clearing it. Require encrypted authenticated Secure Connections (mode 1, level 4), working notifications, and no 0x3d disconnect. Repeat after WBA reset, then separately test fresh pairing and subsequent reconnect.

The prior user logs demonstrate fresh numeric-comparison pairing and notifications, and restoration of a 312-byte stack cache with successful peer lookup. They do not demonstrate successful bonded reconnection.

Full integration review and target Debug/Release validation remain open, including event delivery on bond reuse, persistence failure recovery and radio scheduling during flash writes, platform callback contracts, and the reported pairing latency.

## Vendor references

- [ECB](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c)
- [CMAC](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_cmac.c)
- [CCM](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ccm.c)
- [AES register adapter](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Interfaces/hw_aes.c)
- [BLE platform contract](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v2.10.0/ble/stack/include/bleplat.h)
