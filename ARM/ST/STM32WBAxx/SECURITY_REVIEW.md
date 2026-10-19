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


A further hardware retest after the two-bank bond-storage change saved and
restored 88 bytes with matching CRC `d551074e`. Fresh Numeric Comparison
pairing succeeded; after reset, peer lookup, encryption and pairing returned
status `0x00` without another confirmation. Reconnect encryption took 45 ms
(4723 to 4768 ms). The supplied log does not identify the build configuration
or flashed commit. Physical power interruption during a save was not tested.

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
python3 tests/stm32wba/run_bond_powerloss.py
python3 tests/stm32wba/run_bond_flash.py
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

The bond flash test needs an Arm GNU toolchain and Python unicorn and
pyelftools. It runs the real bond store, PDS store and STM32 flash driver on
an emulated Cortex-M33 against a model of the flash controller with write
protection, across simulated resets. The protection is turned on and off
where a board would need an option byte change and a reset, which shows the
retry times, the drop, a retry succeeding mid wait, a partly written save
keeping the committed image, and the missing region case on the real store
code.

October 18 review fixes, tested locally:

- Host AES adapter: **179 checks, 0 failures** at both `-O0` and `-Os`.
- Bond adapter: **13 test groups passed** at both `-O0` and `-Os`, including
  mount/chunk/manifest failures, timed retries with the 1 to 64 s back-off and
  the retry limit, a missing NVM region, a new request during a flush, torn
  images, missing chunks, CRC rejection and empty images.
- Bond store over the flash model: **13 checks, 0 failures**, with tries
  after waits of 1, 2, 4 up to 64 s and the drop at cumulative poll count
  127. Idle-loop calls do not retry a failed save while its retry wait is
  active; the initial save still runs from the application loop. The STM32
  erase defect read LOCK before unlocking, then wrote it back and relocked
  the controller before STRT. The PDS defect left a failed sector-header
  destination unusable until reset, preventing later saves on the two-sector
  store even after writes became available. Both defects are fixed.
  The supplied hardware logs do not establish whether an erase occurred.
  Hardware erase/garbage-collection and physical power-cut tests remain pending.
- Independent review at `f0dfd9f`: the flash model again passed 13 checks.
  Reverting only the erase fix produced 19 failures; reverting only the PDS
  fix produced 10. The generic PDS suite passed 105 checks with 4-byte and
  16-byte programming units under ASan/UBSan. Bond retry tests and the 486
  power cuts per optimization mode also passed. Changed WBA sources compiled
  in Debug and Release. These are software tests, not hardware validation.
- Power cuts through the real flash driver: `run_bond_flash.py --powercut`
  cuts every program and erase of one save, whole or half done, for a bank
  switch in each direction and for a save that runs PDS garbage collection:
  **330 cuts**, every boot after a cut restored the committed image and
  stored the next save, with no quad word programmed over data. This repeats
  the power-loss test through the real `Nvm` layer and flash driver.
- Flash ECC after a power cut: the flash model now raises the double ECC
  error NMI when a quad word left half programmed or half erased is read,
  and the test firmware links the WBA65 vector table. Before the fix, all 37
  half done cuts of the first bank switch stopped the next boot in
  `Default_Handler`. With it, 164 of the 165 half done cuts of the sweep took
  the NMI at the next boot and none stopped; the 330 cuts restored the
  committed image and stored the next save. A read error that passes (two
  failed reads of a sector header) keeps the sector and the image.
- Persistence review, October 18. 39640a3 made PDS take only `-EBADMSG` as
  damaged data. The STM32 driver can only fail a read with 0 bytes, which
  `Nvm::Read` reports as `-EBUSY`, so the mount failed after a half programmed
  quad word: 161 of the 165 half done cuts restored nothing, and with the
  bond store unmounted `BtAppInit` did not start Bluetooth for an
  application with security. It is reverted (4a81644); the sweep recovers
  from all 330 cuts and all 18 cuts of a bond deletion again. The review
  also found, and the following commits fix:
  - The flash driver reported a refused program of a full 64-byte transfer
    as written. Every `Nvm::Write` chunk of 64 bytes took that path, so a
    PROGERR, WRPERR or timeout could leave a chunk unwritten while the
    store committed a manifest naming it.
  - SWAP_BANK was ignored. With the banks swapped an erase of NVM0 erased a
    page of the application in the other bank, and the ECC decode missed
    errors in the read.
  - A corrected ECC error (ECCC) left set keeps the controller from
    recording the address of the next error, so a later double error was
    decoded to the old address and the device stopped. Each read clears
    ECCC first; ECCC set during the read makes the double error the read's.
    (This model of ECCR was wrong; replaced by the second review below.)
  - ICACHE, which caches flash data reads too, is invalidated after a
    program or an erase when an application enables it.
  - PDS: a clear retried after a failed erase fell back to erasing every
    sector in order, starting with the commit header, so a second failure or
    a power cut brought the cleared records back; a failed mount left the
    store usable; a write that reached the medium but reported an error
    gave its sequence number to the next record.
  `run_bond_flash.py` adds the refused programs, the swapped banks and a
  corrected error before a double one; `pds_test` adds the failing clear
  erase, the write that lands but fails and the failed mount. Each fails
  against the code before its fix.
- Second persistence review, October 19. An external review found that
  three of the fixes above were checked against a wrong flash model, and
  one PDS defect; a fifth came out of the new tests. All are fixed:
  - The ECC NMI decoded ADDR_ECC in 8-byte steps, as
    `HAL_FLASHEx_GetEccInfo` does. RM0515 gives the byte offset of the word
    in its bank, and the field widths agree (20 bits on the lines with 1 MB
    banks, 19 on the 512 KB WBA50). The flash model made the inverse
    mistake, so the two cancelled out. With ECCR modelled as RM0515 gives
    it, every double error was rejected and the boot stopped in
    `Default_Handler`.
  - ECCR records an error only while ECCC and ECCD are both clear. The read
    cleared ECCC once, so a corrected error in one flash word hid a double
    error in a later word of the same read: no NMI, and the read returned
    the damaged bytes (the PDS record CRC still caught them in the test). The
    read now copies one flash word at a time with ECCC cleared before each,
    and reads a word again when ECCC was set during it.
  - The ECC NMI did not invalidate an enabled ICACHE, so a line filled by
    the failed read could give the damaged bytes to the retry without an
    NMI. It is now invalidated in the NMI.
  - PDS: `BtPdsClear` erased sector 0 when no sector was erased. With three
    sectors and a collection whose source erase failed, a power cut before
    the commit lost a saved record. The clear now erases only a sector the
    collection could reclaim, and otherwise returns `-ENOMEM` with the
    store unchanged.
  - PDS: a sector whose erase was cut after its header was taken as erased
    at mount. A header written over it later brought its old records back,
    so a clear could succeed and the cleared records reappear after the
    next reset. The mount now requires the whole sector to read erased.
  `run_bond_flash.py` models ECCR as RM0515 gives it and fails any read that
  returns a damaged word without an NMI or a second read of that word. It
  adds corrected errors in an earlier read, in the same read and from
  elsewhere (an interrupt between the clear and the load), and a run with
  ICACHE enabled. `pds_test` adds a clear without an erased sector, with a
  power cut at every step and a clear after the remount, and the cut erase.
  Each new test fails against the code before its fix. On the final code:
  32 emulator checks; the sweep recovers from all 330 cuts of a save and all
  18 cuts of a deletion, and all 165 half done cuts take the ECC NMI at the
  next boot with none stopping; 215 PDS checks with 4- and 16-byte units;
  486 bond power-loss cuts. All 28 WBA65 examples build in Debug and
  Release. These are software tests, not hardware validation.
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

The production PDS and WBA adapter passed 486 simulated power cuts at each
of `-O0` and `-Os`, with AddressSanitizer and UndefinedBehaviorSanitizer.
The tests interrupt programming and erases, including torn operations,
garbage collection, bank switching, empty-image commits and legacy migration.
Store exhaustion also retains the committed image. These are flash-model
tests; physical power-cut testing on the DK1 remains outstanding.

Temporary AES/key dumps and bond-image dumps have been removed. The store
status lines (PDS mount, bond saved and restored with length and CRC) are
printed in every build, as in the other IOsonata bond stores. Link and security
status (peer lookup, security request, pairing, encryption, disconnection and
host runs of 100 ms or more) is printed when `NDEBUG` is not defined, so the
Release library leaves it out. Errors are always printed.

## Remaining limitations and tests

- Pairing rejection, persistence failure recovery on hardware, and radio
  scheduling during flash writes need separate tests.
- Flash ECC after a power cut is handled in software and not yet on the
  board. A cut during a quad word program or a page erase can leave a word
  that fails the flash ECC check; on the WBA a double error raises the NMI.
  `NMI_Handler` of the WBA65 vector table hands it to `Stm32FlashEccNmi` of
  the flash driver, which takes only an error inside its own read: it clears
  ECCD and that read fails. Any other NMI still stops in `Default_Handler`.
  PDS tries a read three times; a read that still fails is a damaged record
  (no more appends to that sector) or a damaged sector header (the sector is
  erased at init, as for a header that fails its check). The address decode
  takes ADDR_ECC as a byte offset, as RM0515 describes it and against the
  shift by 3 in `HAL_FLASHEx_GetEccInfo`; the board has to confirm which is
  right, since with the wrong one every double error stops the device. With
  the banks swapped, whether BK_ECC names the physical or the mapped bank is
  not settled (the HAL ignores the swap), so both readings are taken. An
  application that defines its own `NMI_Handler` must call
  `Stm32FlashEccNmi` first. A physical power cut test on the DK1 remains to
  be done.
- A clear on a store left without a sector it can erase safely (a refused
  or interrupted erase, before the next mount) returns `-ENOMEM`. The bond
  deletion keeps retrying it; a reset before it succeeds restores the bonds,
  since the deletion request is not kept across a reset.
- A bond store that does not mount still makes `BtAppInit` fail for an
  application with security. Whether to reformat the store and start
  without bonds instead is not decided.
- Flash program and erase run without radio arbitration. The store relies on
  the dual-bank read-while-write of the WBA65 with NVM0 in bank 2; the ST
  reference schedules flash work in radio windows (flash_manager, RFTS).
- The largest bond image that can still be updated is about 3.6 KB (19 PDS
  chunk records per bank, 456 of the 512 cache words), from the record and
  sector sizes; not measured.
- A manifest record that reads back but does not decode (bank above 1 or an
  unexpected length) makes every save fail while restore treats it as empty.
  It needs corruption past the record CRC or a newer format; writing bank 0
  and committing a new manifest would recover it.
- Production devices need secret identity/encryption roots supplied through
  `BtAppWbaIdentityGet`. The default derives roots from the chip ID; they are
  not secret. Successful encrypted pairing does not validate provisioning.

The final WBA candidate builds pass for all 20 examples in Debug and Release
with Arm GCC 13.2.1. The shared Bluetooth host suite passes at `-O0` and `-O2`;
USB regression checks also pass. These are local checks, not a current CI run.

## Vendor references

- [ECB](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ecb.c)
- [CMAC](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_cmac.c)
- [CCM](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Modules/BasicAES/baes_ccm.c)
- [AES register adapter](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/Common/WPAN/Interfaces/hw_aes.c)
- [BLE platform interface](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v2.10.0/ble/stack/include/bleplat.h)

