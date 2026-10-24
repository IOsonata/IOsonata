# Generic CryptoTest

Import the MCU library project and its CryptoTest project into IOcomposer.
Rebuild the library and application in matching Debug or Release configurations.
The seven configured examples (nRF5340 application/network, nRF9160, nRF91x1,
STM32H753, STM32WBA65 and STM32L476) link the same `crypto_engine_test.cpp`.

The test declares `Crypto s_Crypto` and calls `s_Crypto.Init()`. The MCU library
binds its internal hardware implementation; `board.h` supplies console and timing data
only. Software objects in the test are independent comparison references.
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
python3 tests/stm32h7/run_crypto_accel.py
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
| CryptoMaster: nRF54 | Software AES-256 and AES-128 ECB/CBC decryption; hardware AES-128 encryption hooks | Source review and shared software/AES tests; board rerun pending |
| nRF5340 Net ECB | Inherits software modes and AES-256 | Maintainer board result: 100 passed, no failures/skips before this PKA fix |
| STM32WBA65 | Software P-256 base with hardware overrides; software AES-256 | Target initialization compiles; ARM emulator/board rerun pending |
| STM32H753 | Hardware AES-128/256 and one-shot hash/HMAC; software P-256 and streaming hash | Host register model; board rerun pending |
| Other configured MCU libraries | Software algorithms, hardware RNG where implemented | Software archive and target initialization checks |

The ported operations above have working implementations. Software-only entropy
is intentionally non-secure and cannot supply key generation/signing. Desktop
and the skeletal RISC-V nRF54H20 project remain outside this MCU integration.
The host models do not validate silicon timing, DMA, or physical entropy quality.
