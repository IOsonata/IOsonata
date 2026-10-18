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
These timings are from the earlier device-side log.

Hardware retest reported after the review fixes on October 18, 2026:
fresh Numeric Comparison pairing, a notification, reset and bonded reconnect
passed. nRF Connect reported mode 1, level 4 on fresh pairing and both
subsequent reconnects, without another Numeric Comparison prompt.

| Central log | Connected | Mode 1, level 4 | Elapsed |
|---|---|---|---|
| First reconnect | 18:01:06.968 | 18:01:07.017 | 49 ms |
| Second reconnect | 18:01:17.402 | 18:01:17.452 | 50 ms |

The first disconnect was reported as connection timeout; the second was a
local host disconnect. Both subsequent connections restored security. The
device log contains a second boot banner. Build configuration and exact
flashed commit were not recorded in the supplied log.


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
- Bond adapter: **13 test groups passed** at both `-O0` and `-Os`, including
  mount/chunk/manifest failures, timed retries with the 1 to 64 s back-off and
  the retry limit, a missing NVM region, a new request during a flush, torn
  images, missing chunks, CRC rejection and empty images.
- Cortex-M33 crypto integration: **18 checks, 0 failures**, including
  incremental CMAC and three interrupt preemptions while AES is busy.
  The existing model warning about the platform `HW_RNG_Get` initialization
  remains; this test does not count those RNG configuration messages as failures.
- The four changed WBA source files compile at `-O0` and `-Os` with Arm GCC
  13.2.1, Cortex-M33 hard float, CubeWBA 1.10.0 HAL headers and WPAN 2.10.0.
  Existing volatile increment/decrement warnings in the event ring remain.
  These are object compilation checks, not complete application builds.

These software results are separate from the maintainer's DK1 hardware
retest recorded above.

## Bond persistence

A failed mount, chunk write or manifest write leaves the save pending. The
existing one-second Bluetooth timer event tries it again after 1 s, then after
2, 4 and up to 64 s; idle-loop checks do not try a failed write again. A save
that still fails after the 64 s wait is dropped until the stack asks for a new
save, so a medium that keeps failing is not rewritten forever. A missing NVM
region is a build setup error and is not retried. A request received during a
flush is retained for the next attempt.

Chunks are replaced under the existing record keys, then the manifest is
written. A reset during a partial update can lose the stored image; this is
not rollback to the previous image. Restore rejects missing or inconsistent
chunks and clears the cache before the stack starts. The on-flash format is
unchanged, so existing valid bonds remain readable.

Temporary AES/key dumps and bond-image dumps have been removed. The store
status lines (PDS mount, bond saved and restored with length and CRC) are
printed in every build, as in the other IOsonata bond stores. Link and security
status (peer lookup, security request, pairing, encryption, disconnection and
host runs of 100 ms or more) is printed when `NDEBUG` is not defined, so the
Release library leaves it out. Errors are always printed.

## Remaining limitations and tests

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
