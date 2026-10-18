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
retain their order. CMAC uses standard byte order and retains the chaining
value across whole-block appends with a null output pointer. The final call
applies padding and the CMAC subkey. AES failures invalidate the operation
and clear the final tag. Temporary conversion buffers and completed CMAC
state are wiped.

The earlier byte-order mismatch was corrected before the successful reconnect
tests. The previous logs alone did not isolate it as the only reconnect cause.

## Regression tests

Run from the repository root:

```sh
python3 tests/stm32wba/run_ble_crypto.py
python3 tests/stm32wba/run_bt_crypto.py
python3 tests/stm32wba/run_bond_store.py
```

The host AES adapter test needs g++ and Python cryptography. It uses the real
software AES implementation with a simulated hardware provider to check
byte order, incremental CMAC, busy fallback and failure handling. The bond
adapter test compiles the complete adapter with an in-memory PDS and the real
CRC implementation. Both host tests enable AddressSanitizer and
UndefinedBehaviorSanitizer.

The hardware adapter test needs an Arm GNU toolchain, Python unicorn,
pyelftools and cryptography, and `CUBE_WPAN` pointing to stm32-mw-wpan 2.10.0.
It runs the actual WBA AES/PKA drivers on register models.

October 18 review fixes, tested locally:

- Host AES adapter: **179 checks, 0 failures** at both `-O0` and `-Os`.
- Bond adapter: **9 test groups passed** at both `-O0` and `-Os`, including
  mount/chunk/manifest failures, timed retries, a new request during a flush,
  torn images, missing chunks, CRC rejection and empty images.
- Cortex-M33 crypto integration: **18 checks, 0 failures**, including
  incremental CMAC and three interrupt preemptions while AES is busy.
  The existing model warning about the platform `HW_RNG_Get` initialization
  remains; this test does not count those RNG configuration messages as failures.
- The four changed WBA source files compile at `-O0` and `-Os` with Arm GCC
  13.2.1, Cortex-M33 hard float, CubeWBA 1.10.0 HAL headers and WPAN 2.10.0.
  Existing volatile increment/decrement warnings in the event ring remain.
  These are object compilation checks, not complete application builds.

These results are separate from the DK1 hardware runs above. No new hardware
run was performed for these fixes.

## Bond persistence

A failed mount, chunk write or manifest write leaves the save pending. The
existing one-second Bluetooth timer event retries it; idle-loop checks do not
repeatedly retry a failed flash write. A request received during a flush is
retained for the next attempt.

Chunks are replaced under the existing record keys, then the manifest is
written. A reset during a partial update can lose the stored image; this is
not rollback to the previous image. Restore rejects missing or inconsistent
chunks and clears the cache before the stack starts. The on-flash format is
unchanged, so existing valid bonds remain readable.

Temporary AES/key dumps, bond-image dumps and routine bring-up traces have
been removed. Storage and security-request errors still use SysLog.

## Remaining limitations and tests

- Repeat pairing and bonded reconnect after reset with the corrected firmware.
- Pairing rejection, persistence failure recovery on hardware, and radio
  scheduling during flash writes need separate tests.
- Complete the final Debug/Release application builds and regression checks
  on the release candidate.

## Vendor references

- [ECB](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c)
- [CMAC](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_cmac.c)
- [CCM](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ccm.c)
- [AES register adapter](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Interfaces/hw_aes.c)
- [BLE platform interface](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v2.10.0/ble/stack/include/bleplat.h)
