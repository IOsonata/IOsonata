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
may use semihosting. Expect zero failures. AES-128 and AES-256 ECB/CBC/CTR
are required: the library uses software when an accelerator lacks an operation.
Only secure-entropy-dependent operations can be skipped on targets without
secure entropy. Busy or initialization
failures are failures, not capability skips. RNG sample tests do not certify
entropy quality.

Host validation:

```
python3 tests/crypto/run_objects.py
python3 tests/crypto/run_aes256.py
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
