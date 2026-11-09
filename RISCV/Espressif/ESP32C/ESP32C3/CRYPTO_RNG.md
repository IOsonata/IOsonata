# ESP32-C3 hardware RNG experiment — disabled in normal firmware

The new SAR-ADC-backed hardware RNG path has **not** been validated on
ESP32-C3 hardware. The first integrated version invoked
`Esp32C3RngProbe()` from `Crypto::Init()`. Because
`exemples/crypto/crypto_engine_test.cpp` calls `Crypto::Init()`
**before printing its test banner**, the device could stop immediately after
the ESP-ROM boot message, before any application output.

To restore the known-working startup and crypto behavior:

- `RISCV/Espressif/src/crypto_esp32.cpp` is restored to its pre-RNG
  implementation. `Crypto::Init()` no longer touches the SAR ADC or ROM
  analog-I2C interface.
- The experimental `crypto_rng_esp32c3.cpp` is **not linked** by the
  ESP32-C3 IOsonata library project. No application can accidentally
  activate it through `Crypto::Random()`.
- The default `Crypto::Random()` remains the existing software PRNG,
  and `Crypto::IsSecure()` remains false. ECDH key generation/signing
  still correctly refuse non-secure entropy.
- The experimental source and standalone model test are retained for
  controlled debugging; they are not production-ready.

## Root-cause isolation

The ROM boot text alone does not identify whether the early RNG call
trapped, timed out, or hung. The early invocation was an unsafe change
to a working initialization path regardless of the precise fault.

Only after the standard CryptoTest is seen working again should the
experimental RNG be exercised by a **separate, explicitly initiated**
hardware test with debugger stops before the internal analog ROM calls,
SAR clock/power configuration, and cycle-counter wait loops.

On-chip entropy requires careful ADC/RF ownership. A host register model
cannot establish that the physical noise source is safe and reliable.

## Validation commands

After updating and rebuilding the ESP32-C3 library and application:

```sh
python3 tests/crypto/run_esp_models.py
python3 tests/crypto/run_esp32c3_rng.py
```

The second command is only a mock-register test of the *unlinked*
experimental source. It must not be interpreted as hardware validation.

Expected on-board baseline: normal CryptoTest output, including the
`software PRNG, not for security use` line and skipped secure-RNG tests.
The previous maintainer result was 59 PASS, 0 FAIL, 1 SKIP. A fresh
post-revert hardware run is still needed to confirm recovery.
