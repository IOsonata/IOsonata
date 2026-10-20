# WBA BLE security integration

Target: STM32WBA65I-DK1. Middleware: STM32CubeWBA 1.10.0 and
stm32-mw-wpan 2.10.0. See the [port status](STM32WBA65/README.md#port-status)
for the supported configuration and peripheral validation.

## Hardware validation

BleSecurityDemo passes fresh LE Secure Connections Numeric Comparison,
encrypted data transfer and bonded reconnect after reset. The recorded
central logs report security mode 1, level 4 on fresh pairing and subsequent
reconnects without another confirmation prompt. Recorded reconnect encryption
latencies were approximately 45 to 50 ms. Saved/restored cache length and CRC
matched. The logs do not identify the exact flashed commit or configuration.

The standalone hardware crypto test passed 51 tests, 0 failed. See the
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

## Bond persistence

A failed mount, chunk write or manifest write leaves the save pending. The
existing one-second Bluetooth timer event tries it again after 1 s, then after
2, 4 and up to 64 s; idle-loop checks do not try a failed write again. A save
that still fails after the 64 s wait is dropped until the stack asks for a new
save, so a medium that keeps failing is not rewritten forever. A missing NVM
region is a build setup error and is not retried. A request received during a
flush is retained for the next attempt.

Bond images use two banks of chunk keys. A save writes the inactive bank,
then commits one manifest record selecting that bank. Until the manifest
commits, restore reads the previous complete image. PDS protects individual
records and retains both banks during garbage collection. The legacy
eight-byte manifest remains readable as bank 0; the first update writes
bank 1 before replacing it with the twelve-byte manifest.

An interrupted update restores either the old or new complete image.
Corrupt committed data still clears the cache: restore does not fall back
to an older bank, which could restore deliberately deleted bonds.

Both banks and their record overhead must fit in the PDS live set. The
existing 16 KB NVM0 region has two 8 KB sectors, one reserved for garbage
collection. It cannot retain two full 4 KB cache images plus overhead.
If space runs out, the save fails with bounded retries and preserves the
committed image. Chunk records from earlier, larger images also consume
space. No linker partition or NVM1 allocation has changed.

Power-loss recovery has been tested by simulation, including torn programming
and erases, garbage collection, bank switching, empty-image commits and legacy
migration. Store exhaustion retains the committed image. These tests exercise
production PDS, Nvm and flash-driver paths against models; they do not measure
physical interruption behavior on a DK1.

## Flash ECC and deletion

The WBA65 NMI handler delegates flash-read ECC errors to Stm32FlashEccNmi.
The flash driver accepts errors in its own active read, clears ECCD and fails
that read. Other NMIs continue to Default_Handler. An application overriding
NMI_Handler must preserve this delegation. ECC address decoding uses the
RM0515 byte offset. Corrected-error handling and cache invalidation prevent a
masked double error from being accepted without a validated read. With bank
swap, uncertain corrected-error attribution causes a retry or read failure.

PDS commits a clear before reclaiming the erased data. If no sector can be
reclaimed safely, clear returns -ENOMEM. The bond adapter keeps retrying a
failed deletion; a reset before the clear commits can restore the previous
bonds because the pending deletion request is held in RAM.

## Regression tests

Run from the repository root:

```sh
python3 tests/stm32wba/run_ble_crypto.py
python3 tests/stm32wba/run_bt_crypto.py
python3 tests/stm32wba/run_bond_store.py
python3 tests/stm32wba/run_bond_powerloss.py
python3 tests/stm32wba/run_bond_flash.py
python3 tests/stm32wba/run_bond_flash.py --powercut
```

Host adapter tests use g++, Python cryptography, ASan and UBSan. The Cortex-M33
register-model tests need an Arm GNU toolchain, Unicorn and pyelftools; the
crypto adapter runner also needs CUBE_WPAN pointing to stm32-mw-wpan 2.10.0.
Use the pinned emulator environment described in the target README.

Previously recorded software results include 486 bond power-loss cuts per
optimization mode, 330 flash-driver save cuts and 18 deletion cuts. The flash
sweep includes torn-word ECC NMIs, bank swap, refused programs, corrected-error
masking and enabled-cache cases. PDS regression checks also cover failed mount,
failed clear/collection, cut erases and writes that reach flash but report a
failure. These results are simulation evidence, separate from hardware pairing
and reconnect results.

## Limits

- Production devices need secret identity/encryption roots through
  BtAppWbaIdentityGet. The chip-ID-derived defaults are not secret.
- Radio scheduling during flash programming/erase and persistence-failure
  recovery on hardware have not been validated. Flash work relies on
  dual-bank read-while-write with NVM0 in bank 2; it is not arbitrated through
  ST flash_manager/RFTS radio windows.
- A persistent store mount failure makes BtAppInit fail for an application
  configured with security.
- A malformed manifest that passes the record CRC is rejected. Saves remain
  failed until the stored format is repaired; no automatic format migration
  beyond the documented legacy manifest is implemented.
- Analog radio-temperature calibration is not implemented; the platform uses
  the documented fixed-temperature configuration.

Store status reports mount and saved/restored image length and CRC. Errors
are always logged. Link/security diagnostic output is controlled by NDEBUG;
key and bond-image dumps are absent.

## Vendor references

- [ECB](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c)
- [CMAC](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_cmac.c)
- [CCM](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ccm.c)
- [AES register adapter](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Interfaces/hw_aes.c)
- [BLE platform interface](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v2.10.0/ble/stack/include/bleplat.h)


