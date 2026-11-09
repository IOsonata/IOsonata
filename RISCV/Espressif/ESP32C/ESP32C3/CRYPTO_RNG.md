# ESP32-C3 hardware RNG and cryptographic entropy

IOsonata uses the ESP32-C3 RNG_DATA register at SYSCON + 0xB0 only when the
internal SAR ADC is providing physical entropy. Reading this register with
neither RF nor SAR ADC running is **not** a secure RNG.

The standalone backend is `RISCV/Espressif/src/crypto_rng_esp32c3.cpp`.
It uses only the MCU register map, with the ESP32-C3 ROM analog-I2C functions
needed to enable the SAR ADC's internal reference. No ESP-IDF HAL, RTOS,
heap, or driver is linked.

## Lifecycle

- `Crypto::Init()` attempts an RNG probe, but AES and SHA remain available
  if entropy initialization fails.
- On a successful probe, `Crypto::IsSecure()` is true while the SAR ADC can
  be acquired. Key generation and signing still check that value.
- `Crypto::Random()` temporarily configures the SAR ADC internal reference
  as a noise source, reads the RNG at a bounded rate, and restores the
  peripheral registers and analog reference on success or failure.
- A conflicting ADC use returns failure without reconfiguration. A source
  activation/health failure clears the entire output and disables further
  secure RNG operations until reboot.
- The test-only deterministic software random source is never selected by
  the production ESP32-C3 `Crypto::Random()` path.

## Resource ownership limitations

The current ESP32-C3 port does not yet contain an integrated ADC or RF
ownership manager. **Do not call the RNG simultaneously with ADC sampling,
Wi-Fi/Bluetooth activity, or another user of the internal analog I2C bus.**
The backend checks the SARADC timer, clock, DMA and forced power state to
reject an already-configured peripheral, but that alone cannot prevent an
independent RF/ADC driver from taking ownership after the check.

Any future ESP32-C3 ADC, Wi-Fi or BLE port must coordinate this shared
resource explicitly before using the secure RNG concurrently. Disable the
SAR entropy source (already done on return) before initializing those
peripherals.

## Validation

The register model test exercises production code with fake MMIO, not fake
entropy. From the repository root:

```sh
python3 tests/crypto/run_esp32c3_rng.py
python3 tests/crypto/run_esp_models.py
```

Then rebuild the **IOsonata_ESP32C3** library and its CryptoTest application
in IOcomposer. Run CryptoTest on actual ESP32-C3 silicon and collect the
final pass/fail/skip counts. In particular, check that the default secure
random source is reported and that key generation, ECDH, and signing pass.

The 4 KiB distribution test is only a gross diagnostic, **not** a
cryptographic entropy certification. A host MMIO model cannot prove the
physical noise source works on the actual chip. Do not mark this hardware
backend validated until that result is supplied.
