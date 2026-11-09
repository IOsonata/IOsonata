# ESP32-C3 hardware RNG

ESP32-C3 supplies a hardware RNG, but physical entropy is guaranteed only
when an RF subsystem or the internal SAR ADC entropy source is active.
IOsonata's native ESP32-C3 implementation uses the internal SAR ADC during
random draws. It uses direct MCU registers and two ROM analog-I2C entry points
(0x40001958/0x40001960). It does **not** link ESP-IDF's HAL.

## Startup and operation

The earlier experimental implementation called the RNG probe from
`Crypto::Init()`. The maintainer reported that CryptoTest then stopped
before printing its banner. The exact trap/hang location was not captured.

The corrected implementation preserves the known-working `Crypto::Init()`:
it initializes AES/SHA but performs **no ADC or RNG operations**. The
ESP32-C3 library once again includes `crypto_rng_esp32c3.cpp`.

`Crypto::Random()` activates the SAR ADC noise source for each request,
samples the hardware RNG at a conservative 80 microseconds per word,
and restores the previous state before returning. It never silently
uses `CryptoSoftRng` on the production ESP32-C3 build.

`Crypto::IsSecure()` reports whether the ESP32-C3 hardware entropy path
is eligible for a new request: the ADC must be idle and no permanent
health/setup failure may have been detected. Each `Random()` call still
checks source activation and the stream; it clears its output and fails
on error. A failed source activation, stalled cycle counter, or repeated
word failure permanently disables the RNG until reboot. Unavailable ADC
resources produce a transient failed request without taking them over.

**This is not a certified entropy source.** The host model proves register
sequencing only; electrical entropy behavior and production suitability
require on-device validation.

## Debugger checkpoints

Two globals have ordinary C++ linkage and are visible in the debugger:

- `g_Esp32C3RngStage` is 1 on entry, 2 when reading the snapshot,
  3 entering ROM analog calls, 4 after analog setup, 5 after ADC
  configuration, 6 once the source is active, 7 during the initial
  bounded wait, 8 during RNG reads, 9 during restoration, and
  10 on success. A failure retains its active stage.
- `g_Esp32C3RngError` is 0 on request entry; 1 means busy,
  2 ADC unavailable/already faulted, 3 setup failed, 4 the cycle
  counter stopped or a clock precondition failed, and 5 repeated RNG
  words.

The cycle-counter waits have a finite iteration limit to avoid a
firmware-wide stall if the counter stops advancing. A ROM routine that
never returns cannot be bounded from the caller; stage 3 distinguishes
that situation from a timer wait.

## Resource isolation

This backend must not run while another ADC, Wi-Fi/Bluetooth controller
or internal analog-I2C client uses the SAR ADC. It refuses an ADC that is
already configured, but a cooperative ADC/RF ownership mechanism is still
required before permitting concurrent users.

## Validation

Run both host suites from the IOsonata directory:

```sh
python3 tests/crypto/run_esp32c3_rng.py
python3 tests/crypto/run_esp_models.py
```

The RNG model tests successful draws, odd-buffer length, register cleanup,
ADC contention, failed analog configuration, repeated RNG words, and a
stopped cycle counter at both -O0 and -O2. It is **not** a physical entropy
measurement. The existing AES/SHA model is compiled without analog
entropy, using an explicitly test-only PRNG configuration.

Clean-build `IOsonata_ESP32C3` and the `CryptoTest` example, flash,
and compare its output against the previously working 59 PASS, 0 FAIL,
1 SKIP. The new hardware RNG is expected to eliminate the skipped
ECDH/signing test **only after the board proves it**. If the board
stops at a stage, capture that stage/error and the exception PC.


## Short first-request RNG trace

The user board passed all pre-RNG CryptoTest checks, then stopped during a
verbose analog-I2C trace. The last confirmed ROM operation was write field
3:3 = 0; the output ended during the next write to 2:2, but the console
contained dropped/interleaved characters. That is not proof the ROM call
was responsible.

The initial analog reads all returned zero for fields 1:0, 3:3 and 2:2.
Those zero writes are now **omitted**. The backend checks fields 3:0
once, refuses to alter nonzero foreign analog state, and only writes
ADC2_ENCAL_REF (bit 6). It restores only that bit.

Verbose per-call UART tracing was replaced with compact numeric
checkpoints for the first RNG request, reducing transmit-buffer pressure.
The stage and error globals are still available in the debugger.
The markers are:

| Marker | Meaning |
| --- | --- |
| RNG 1 | First request acquired the RNG operation |
| RNG 2 | ADC snapshot |
| RNG 30 | About to read analog reference bit 6 |
| RNG 31 | About to check analog fields 3:0 |
| RNG 32 | About to set internal reference bit 6 |
| RNG 4 | Analog reference setup complete |
| RNG 5 | Digital SAR ADC setup complete |
| RNG 33 | About to verify analog bit 6 |
| RNG 6 | Entropy source activated |
| RNG 7 | Settling wait |
| RNG 8 | Reading RNG data |
| RNG 9 | Restoring ADC/analog registers |
| RNG 10 | Successful completion |

No printf takes place while interrupts are masked. Failed requests still
report a concise stage/error line.

Debug and Release builds enable tracing in this diagnostic branch.
After hardware validation, `ESP32_RNG_TRACE=0` can be used to remove it.

Run the ESP32-C3 model and existing AES/SHA regression tests:

```sh
python3 tests/crypto/run_esp32c3_rng.py
python3 tests/crypto/run_esp_models.py
```

Then clean-rebuild `IOsonata_ESP32C3` and `CryptoTest` in IOcomposer.
These tests have not yet established on-device RNG success.
