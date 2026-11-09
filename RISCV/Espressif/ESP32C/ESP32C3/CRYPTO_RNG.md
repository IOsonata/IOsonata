# ESP32-C3 hardware RNG and crypto validation

The ESP32-C3 `CryptoTest` completed on actual hardware on **November 8, 2026**:

- **101 PASS, 0 FAIL, 0 SKIP**
- Hardware SAR ADC entropy setup and restoration completed successfully (RNG checkpoints 1 through 10).
- Random draws, consecutive-output differences, odd-length output bounds, and a 4 KiB basic distribution check all passed. The test reported `Source: secure random generator`.
- Default P-256 key generation, ECDH agreement, and ECDSA signing and verification passed, including invalid-key and invalid-peer rejection.
- AES-128 and AES-256 ECB/CBC/CTR known-answer tests passed; AES-128 CMAC, CCM, and GCM tests passed; SHA-256 and HMAC-SHA-256 tests passed.
- The 200-round AES comparison and 8-round P-256 comparison against the software reference passed.

This records **functional hardware acceptance** of the tested crypto engine and entropy path. It is *not* a statistical entropy certification or proof of physical entropy quality under every environmental or RF condition.

## Architecture

`RISCV/Espressif/src/crypto_esp32.cpp` uses the ESP32-C3 AES and SHA registers directly. CCM/GCM and related modes use hardware AES as a primitive; P-256 cryptography is software-based. `Crypto::Init()` initializes AES/SHA without probing or modifying the entropy source.

`RISCV/Espressif/src/crypto_rng_esp32c3.cpp` supplies the default `Crypto::Random()` implementation. Each request temporarily powers and configures the SAR ADC's internal noise source, uses the ESP32-C3 RNG data register, then restores the prior register state. Only the required analog internal-reference bit is written via the MCU ROM analog-I2C interface. When the ADC is already active, a request fails without claiming the peripheral; a failed entropy activation or detected stream fault prevents insecure key generation.

The driver uses `IOsonataRiscvCycle32()` from `RISCV/include/idelay.h` to pace reads. On ESP32-C3 this is the vendor MPCCR at CSR `0x7E2`, initialized by `Esp32SystemInit()`. The standard RISC-V `cycle` CSR `0xC00` is not implemented on this MCU and caused the earlier test to stop at checkpoint `RNG 7`.

No ESP-IDF HAL, RTOS, heap, or software-RNG fallback is required for the hardware path.

## Measured RNG latency

The on-board `CryptoTest` included setup and teardown in each timed draw:

| Requested random bytes | Time (nanoseconds per operation) |
| ---: | ---: |
| 1 | 242,375 |
| 16 | 475,968 |
| 256 | 5,339,750 |
| 1024 | 20,903,609 |

This direct entropy mode intentionally favors sustained entropy collection over bulk throughput. If faster long streams become necessary, investigate a *separately validated* cryptographic DRBG seeded and periodically reseeded from hardware entropy. Do not remove the entropy source or reduce timing merely to improve the benchmark.

## Debugging and limits

`ESP32_RNG_TRACE` now defaults to `0` after functional hardware acceptance. Define `ESP32_RNG_TRACE=1` to print first-request numeric checkpoints. The debugger variables `g_Esp32C3RngStage`, `g_Esp32C3RngError`, and `g_Esp32C3RngCallCount` remain available. Checkpoints include `RNG 7` for the settling wait, `RNG 8` for hardware data reads, `RNG 9` for register restoration, and `RNG 10` for successful completion.

The current implementation is **exclusive-use**: do not run it concurrently with ADC conversions, RF controller use, or other users of the internal analog-I2C interface. A shared peripheral ownership manager is still required for concurrent Wi-Fi/Bluetooth/ADC applications.

## Regression tests

From the IOsonata repository root:

```sh
python3 tests/crypto/run_esp32c3_rng.py
python3 tests/crypto/run_esp_models.py
```

These are host-model tests and do not replace the ESP32-C3 board run. Rebuild the `IOsonata_ESP32C3` managed library and `CryptoTest` in IOcomposer to verify any subsequent changes.
