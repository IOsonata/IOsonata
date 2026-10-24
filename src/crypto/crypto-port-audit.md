# Crypto port audit

Reviewed against prerelease_0.13_4, October 2026.

Applications declare `Crypto` directly and call `Init()`. All supported software
operations remain available through that object. Each MCU library selects its
hardware overrides; applications do not route operations or retry unsupported
hardware operations. `exemples/crypto/crypto_engine_test.cpp` remains the shared
test, including its Timer-based benchmarks.

## MCU library coverage

This table distinguishes working software support from hardware acceleration.
A hardware peripheral listed as pending is not claimed as implemented.

| Port | Implementation selected by the library | Status and limits |
|---|---|---|
| SAM4E16E | New AES-128/256 ECB/CBC/CTR; AES-128 CMAC/CCM/GCM use its block hook | SHA/P-256 software. No on-chip TRNG used; software RNG is not secure. |
| SAM4LCxC, SAM4LSxC | New AES-128 and TRNG, AESA clock/countermeasure setup | AES-256 inherited software; SAM4L AESA supports 128-bit keys. SHA/P-256 software. |
| LPC11U35, LPC1769 | Generic software | No secure entropy implementation. |
| LPC54605, LPC54628 | New ROM RNG override, software algorithms | Discards 32 ROM RNG words between returned words, following the NXP driver. Production LPC546xx does not provide the AES/SHA acceleration implied by old register headers. |
| nRF52805, nRF52810, nRF52832 | New default binding of ECB-128 override; existing RNG | SoftDevice ECB service used when enabled; otherwise peripheral DMA. AES-256 and decrypt modes inherit software. |
| nRF52840 | New ECB-128 binding plus existing CC3xx P-256/RNG | CC310 AES/SHA acceleration remains unused; ECB handles AES-128 encryption blocks. |
| nRF5340 network | Existing ECB-128/RNG | AES-256, SHA and P-256 software. |
| nRF5340 application, nRF9160, nRF91x1 | Existing CC3xx P-256/RNG | AES/SHA software; CC3xx AES/SHA acceleration remains incomplete. |
| nRF54H20 application/network, nRF54L15, nRF54LM20x | Existing CryptoMaster/BA414EP/CRACEN RNG | Wider cipher keys, SHA, key import and ECDSA use inherited software where the driver has no override. Hardware coverage is partial. |
| nRF54H20 RISC-V skeleton | Not a complete MCU library | No complete generic crypto integration; not equivalent to the Arm application/network libraries. |
| RP2040 | Generic software | Ring-oscillator bits are not exposed as qualified secure entropy. |
| RA4M1 (ioc and e2) | Generic software | SCE5 AES/TRNG integration remains incomplete. Requires the Renesas SCE initialization and primitive sequences. |
| RE01_1500KB (ioc and e2) | Generic software | TSIP-Lite AES/TRNG integration remains incomplete. Requires the applicable Renesas TSIP-Lite driver. |
| R9A02G021 (ioc and e2) | New TRNG override, inherited software algorithms | Module start, bounded ready wait, four-byte reads and stop sequence. No AES accelerator. |
| STM32F030, STM32F401, STM32L152RE | Generic software | No secure entropy driver selected. |
| STM32G474, STM32L432, STM32L476, STM32L496 | Existing RNG, software algorithms | RNG kernel clock must be provided by target clock setup. Health failures now fail and wipe the entire request. |
| STM32L4S9 | New AES-128/256 and HASH/HMAC binding, existing RNG | Shared H753 cipher/hash logic with L4 AES register operations. Streaming SHA remains software. |
| STM32H753 | Existing CRYP/HASH/HMAC/RNG | Regression tested after adding L4S9. Streaming SHA and P-256 software. |
| STM32WBA65 | Existing AES/RNG/PKA | Software behavior inherited where hardware overrides are absent. |
| ESP32-C3, ESP32-C6 | New AES-128/256 ECB/CBC/CTR; AES-128 CMAC/CCM/GCM use its block hook | SHA acceleration, secure entropy configuration, and C6 ECC acceleration remain incomplete. Software SHA/P-256 remain available. |
| Windows host | Generic software | Host entropy integration is separate from MCU acceleration. |

CMAC, CCM and GCM currently accept AES-128 keys in the generic implementation.
AES-256 ECB/CBC/CTR are mandatory software-supported operations on every complete
MCU library. Hardware timeouts and faults return failure; they never trigger a
software retry that hides the fault.

## Validation performed

New register models compile the production drivers and compare AES work against
OpenSSL. The SAM, ESP32, R9A02 and LPC models execute the unchanged generic test
at `-O0` and `-O2` with address and undefined-behavior sanitizers. Their Timer
fixture is synthetic; displayed timing values are not measured performance.

- SAM4L, R9A02 and LPC546xx: 101 passed, 0 failed, 0 skipped per model run.
- SAM4E and ESP32-C3/C6: 59 passed, 0 failed, 1 explicit secure-entropy skip.
  AES-256 tests run. Key generation/signing with an insecure RNG are rejected.
- STM32H753 and L4S9 accelerator models: 101 passed, 0 failed, 0 skipped;
  timeout recovery, wiping and cross-instance locks checked.
- Nordic ECB model: bare-metal nRF52, nRF52 SoftDevice and nRF5340 paths;
  known answer, service errors, output wiping, stopped DMA recovery, and
  permanently blocked reuse when DMA ownership cannot be resolved.
- STM32 RNG model: current and latched seed/clock errors before and after a
  data read, bounded timeout, request wiping and explicit reset recovery.
- Generic object, AES-256/OpenSSL and CC3xx/BA414EP regression tests.
- Target object compile checks, including nRF52805/810 and STM32L4S9.

No new physical MCU runs or MCU-toolchain builds were performed during this
pass. The maintainer's earlier nRF5340 application/network results remain the
hardware evidence. Host models check driver logic; they do not prove peripheral
clocking, electrical entropy quality, timing or silicon errata behavior.

## Work still required

This is not a claim that every hardware crypto block is implemented. RA4M1 SCE5
and RE01 TSIP-Lite need vendor-driver integration and board validation. The SCE5
vendor sequences contain unbounded waits and must be integrated with a defined
failure/recovery path rather than copied into a supposedly bounded driver.
ESP32 secure entropy requires qualifying/enabling the source, not merely reading
the RNG register; the current library does not own that source configuration.
The remaining SHA/AES/ECC acceleration noted above also requires implementation.

## Register and vendor references

- Renesas R9A02G021 User's Manual: Hardware, TRNG chapter, registers at
  0x40091100 and the enable/start/ready/four-read/stop sequence.
- Microchip SAM4L datasheet and ASF AESA driver/unit tests: AES-128, GCLK4,
  CPU clock selector 7, countermeasure seed and little-endian data words.
- Microchip SAM4E AES component header and ASF AES unit tests: 128/256-bit
  key modes, manual start and data-ready/access-error handling.
- NXP legacy-mcux-sdk `drivers/rng/fsl_rng.c`: LPC546xx ROM API and 32-word
  discard interval. LPC546xx datasheet revision history removes AES/SHA.
- ST STM32L4S9 CMSIS device header and AES key derivation/decrypt sequencing.
- Espressif ESP-IDF v5.4.2 `hal/esp32c3/include/hal/aes_ll.h` and
  `hal/esp32c6/include/hal/aes_ll.h`, with their SYSTEM/PCR register headers:
  AES mode, little-endian words, CPU-block idle state and DS reset dependency.
- Renesas FSP SCE5 plain-key primitives inspected at
  `6e2675336b37fbd805f0c01d5134adfdab4bf294`; not vendored by this change.
