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
may use semihosting. Expect zero failures. AES-256 and secure-entropy-dependent
operations can be explicitly skipped when unavailable. Busy or initialization
failures are failures, not capability skips. RNG sample tests do not certify
entropy quality.

Host validation:

```
python3 tests/crypto/run_objects.py
python3 tests/crypto/check_targets.py
python3 tests/stm32h7/run_crypto_accel.py
```

The first builds an actual software archive at O0/O2 and checks direct global/local construction,
object identity, independent RNG state, disabled-object rejection and the shared known-answer tests.
The second compiles target initialization definitions against real driver headers;
small MCU header shims omit register definitions. It is not an MCU build.
The third exercises production H753 AES/HASH drivers against an OpenSSL-backed
register model, including initialization failure/retry and shared-object independence, with ASan/UBSan.
Hardware compilation and board tests remain required.
