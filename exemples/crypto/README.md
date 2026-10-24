# Generic CryptoTest

Import the MCU library project and its CryptoTest project into IOcomposer.
Rebuild the library and application in matching Debug or Release configurations.
The eight configured examples (nRF52832, nRF5340 application/network, nRF9160, nRF91x1,
STM32H753, STM32WBA65 and STM32L476) link the same `crypto_engine_test.cpp`.

The test declares `Crypto s_Crypto` and calls `s_Crypto.Init()`. The MCU library
binds its internal hardware implementation; `board.h` supplies console configuration only. Software objects in the test are independent comparison references.
No hardware factory names, target headers or linker forcing options are needed.

Release uses the configured UART; check the board's baud rate and pins. Debug
may use semihosting. Expect zero failures. AES-128/256 ECB/CBC/CTR and P-256
private-key import are required: the library uses software when an accelerator lacks an operation.
Only secure-entropy-dependent operations can be skipped on targets without
secure entropy. Busy or initialization
failures are failures, not capability skips. RNG sample tests do not certify
entropy quality.

Host validation:

```
python3 tests/crypto/run_objects.py
python3 tests/crypto/run_aes256.py
python3 tests/crypto/run_pka_models.py
python3 tests/crypto/check_targets.py
python3 tests/stm32wba/run_crypto_link.py
python3 tests/stm32h7/run_crypto_accel.py
python3 tests/stm32h7/run_crypto_accel.py --l4s9
```

The first builds an actual software archive at O0/O2 and checks direct global/local construction,
object identity, independent RNG state, disabled-object rejection and the shared known-answer tests.
The AES-256 differential test compares software, an AES-128-only inherited
engine and hardware override inheritance against OpenSSL at O0/O2 under ASan/UBSan.
It also verifies that busy and failed operations are returned without retry.
The target check compiles target initialization definitions against real driver headers;
small MCU header shims omit register definitions. It is not an MCU build.
The H753 test exercises production H753 AES/HASH drivers against an OpenSSL-backed
register model, including initialization failure/retry and shared-object independence, with ASan/UBSan.
Hardware compilation and board tests remain required.

The PKA host test links the production Nordic App-core initialization and CC3xx
implementation as an archive. OpenSSL models register arithmetic; the driver
runs its actual point operations. The same test exercises BA414EP through an
injected register model. It checks inherited import/signing, hardware agreement,
boundary scalars, invalid peers, key consumption, busy handling, RNG rebinding,
and a CC3xx cleanup timeout. Both optimization levels use ASan/UBSan.

Port review after the App-core import failure:

| Port | Software behavior and hardware overrides | Validation available |
|---|---|---|
| CC3xx: nRF52840, nRF5340 App, nRF91 | Software import/ECDSA; hardware key generation/ECDH | Host model; App-core board had passed generated-key operations before this fix; imported-key board run pending |
| BA414EP: nRF54 | Software import/ECDSA; hardware key generation/ECDH | Host model; board rerun pending |
| CryptoMaster: nRF54 | Hardware AES-128/256 encryption/decryption; shared CMAC/CCM/GCM composition | DMA register model; board rerun pending |
| nRF5340 Net ECB | Inherits software modes and AES-256 | Maintainer board result: 100 passed, no failures/skips before this PKA fix |
| STM32WBA65 | Hardware P-256; software AES-256 and SHA | Production-driver host link and initialization-failure checks; ARM emulator/board rerun pending |
| STM32H753 | Hardware AES-128/256 and one-shot hash/HMAC; software P-256 and streaming hash | Host register model; board rerun pending |
| Other configured MCU libraries | Software algorithms, hardware RNG where implemented | Software archive and target initialization checks |

The ported operations above have working implementations. Software-only entropy
is intentionally non-secure and cannot supply key generation/signing. Desktop
and the skeletal RISC-V nRF54H20 project remain outside this MCU integration.
The host models do not validate silicon timing, DMA, or physical entropy quality.

All MCU builds run the same performance section. The test declares a `Timer`,
selects the first high-frequency device with `TimerGetHighFreqDevNo()`, and
requests 1 MHz through `Timer::Init`. The timer driver selects the actual
frequency. `TickCount()` supplies 64-bit counts; `nSecond()` and `uSecond()`
perform all time conversion. Results are nanoseconds per operation (averaged
over 64 operations) and microseconds per P-256 key import/public-key calculation.
These are elapsed times, not CPU cycle counts. Timing initialization failure
fails the test. No board timing macros, DWT register access or target selection
is present in the generic test.

Host and register-model runners supply a deterministic test-only TimerInit
implementation. It exercises conversion at 32768 Hz and a 32-bit rollover;
its printed times are simulated and must not be used as performance results.
MCU projects link the existing timer driver from their MCU library.

## Checking software linkage

CryptoTest includes software reference objects intentionally, so its map cannot
prove that a normal application discards overridden software implementations.
The PKA and STM32 accelerator runners additionally link a minimal `Crypto`
application from the production target archive at `-O0` and `-Os` with section garbage
collection, without the generic test or software reference objects.

- CryptoCell: software AES, SHA-256 and PRNG symbols must be absent.
- WBA: the production drivers link without software P-256 or PRNG object files.
  Software AES-256 and SHA must remain. The host fixture exercises construction
  and initialization failure; it does not simulate accelerator arithmetic.
- H753/L4S9: software AES and PRNG symbols must be absent. Software SHA streaming
  remains required; one-shot SHA and HMAC use hardware.

The STM32 model also verifies that initializing, disabling and re-enabling one
application object does not reset the shared engines used by another object.
These are host link and register-model checks, not measurements of MCU flash
usage or proof of silicon behavior.

## nRF52832 project

Import `ARM/Nordic/nRF52/nRF52832/lib/ioc` and
`ARM/Nordic/nRF52/nRF52832/exemples/CryptoTest/ioc`. Build the library and test
with matching Debug or Release configurations. The application links the same
`exemples/crypto/crypto_engine_test.cpp` used by the other MCU projects.

Debug uses semihosting. Release uses UART0 at 1 Mbaud, 8N1, without flow control.
The default BLUEIO console mapping is TX P0.07 and RX P0.08; adjust the example's
`src/board.h` for a different board. The application uses the full nRF52832 XXAA
512 KB Flash / 64 KB RAM linker layout, with no running Bluetooth stack required.
Hardware/software crypto selection remains entirely in the MCU library.
