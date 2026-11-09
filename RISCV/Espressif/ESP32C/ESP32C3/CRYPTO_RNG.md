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


## Temporary UART trace (ESP32-C3)

The next board build includes detailed RNG tracing in both Debug and
Release; `ESP32_RNG_TRACE` defaults to 1 in
`RISCV/Espressif/src/crypto_rng_esp32c3.cpp`. It does not touch
`Crypto::Init()` and leaves the entire RNG implementation linked.

The trace starts immediately after the existing `Random source` banner.
Only the first four random requests print each milestone, but any detected
failure prints a summary. The important checkpoints are:

- `RNG: entered hardware backend`: `Crypto::Random` reached the MCU backend.
- `RNG: request #...`: the driver acquired its internal operation lock.
- `RNG: ADC idle, begin register snapshot`: the peripheral register scan
  completed.
- `RNG: enter analog ROM setup`, followed by individual `before/after
  ROM read/write` messages: locate a possible ROM analog-I2C hang or trap.
- `RNG: ADC configured, check source`: SAR configuration completed.
- `RNG: CPU clock` and `waiting for SAR entropy`: settle timer.
- `RNG: discarding two initial hardware samples` and
  `RNG: first output sample follows`: RNG register reads.
- `RNG: begin ADC/ROM restoration`: restore path.
- `RNG: completed request` or `RNG: FAIL stage=...`: final outcome.

`g_Esp32C3RngStage`, `g_Esp32C3RngError`, and the new
`g_Esp32C3RngCallCount` remain readable in the debugger. The
read-only `IsSecure()` availability check no longer overwrites
the current diagnostic stage.

After the actual hardware failure is identified and corrected, turn off the
temporary UART instrumentation by defining `ESP32_RNG_TRACE=0` and
rerun the Release performance tests. Host register-model tests always build
with this define disabled to keep their results free of UART logging effects.
