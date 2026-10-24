# CryptoTest: NUCLEO-H753ZI

This project runs the existing `exemples/crypto/crypto_engine_test.cpp`, also
used by STM32WBA. The MCU library binds an internal implementation using `CryptoRngStm32` for hardware random numbers,
`CryptoStm32H7Aes` for the CRYP AES accelerator and
`CryptoStm32H7Hash` for one-shot SHA-256 and HMAC-SHA-256 on HASH.
P-256 uses `CryptoUecc`.

AES-128 and AES-256 ECB, CBC and CTR encryption/decryption use the hardware block
primitive. CBC chaining and the full 128-bit CTR increment are handled by
the driver. CMAC, CCM and GCM reuse the `CryptoSoftAes` constructions over
hardware AES-128. CMAC, CCM and GCM still accept AES-128 keys only.
AES-256 Cipher uses all eight key registers and the 256-bit decryption
key preparation mode. AES-192 is not offered.

SHA-256 streaming retains the software implementation: the HASH
peripheral's 54-word saved context exceeds the existing 128-byte caller
context limit. One-shot hashes use hardware, including empty and unaligned
messages. HMAC uses the RFC 2104 construction over hardware digests, including key
hashing for keys longer than 64 bytes. It holds the shared HASH lock across
the entire operation, supports truncated output, wipes temporary key material,
and resets HASH on success or timeout. It does not select the peripheral's
native HMAC mode. No context-size or public API changes are required.

Import `ioc` and `../../lib/ioc` into IOcomposer. Build the library and example
in matching configurations. Use Release for the board test: stdout uses
USART3/ST-LINK VCP, PD9 RX and PD8 TX, at **1 Mbaud, 8N1**. Debug uses
semihosting; enable `monitor arm semihosting enable` before resume. No Ethernet
cable or external wiring is needed.

The shared test prints known-answer checks, RNG sample checks, authenticated
encryption rejection checks, and P-256 signing/agreement checks. Expect zero
failed checks. Software AES supports ECB/CBC encryption and decryption; the
shared example also checks software decryption in place, buffer bounds,
length/IV validation and key permissions. RNG sample checks are sanity checks,
not entropy certification.

The library also includes the existing SHA-256/HMAC engine. Its known-answer
self-test is exercised by `tests/stm32h7/run_crypto.py` along with RNG timeout,
clock/seed errors, output wiping and reset recovery at O0 and Os. The register
model supplies deterministic words and cannot validate physical entropy.

On October 21, 2026, the NUCLEO-H753ZI accelerator run passed all **92
checks, zero failures and no skips**. This validates hardware RNG, CRYP
AES-128/256, one-shot HASH SHA-256 and HASH-accelerated HMAC-SHA-256,
together with software P-256 and the software ECB/CBC decryption checks.

AES validation includes ECB/CBC encryption and decryption, CTR carry and
wrap, CMAC/CCM/GCM known answers, authentication rejection and 200 random
comparison rounds against software. SHA-256 validation covers the self-test
and lengths 0 through 257 against software, with unaligned input/output and
output guards. HMAC long-key and truncation known answers, boundary checks,
overlapping output and invalid-input rejection also passed on the board.

Both P-256 boundary scalars (1 and n - 1) pass public-key import, ECDH,
key retention/consumption, invalid-peer rejection and signature verification,
including independent OpenSSL signatures. The software adapter handles
micro-ecc's co-Z and Shamir boundary cases using its integer API
(`uECC_ENABLE_VLI_API=1`), already enabled in the library project.

Run `python3 tests/stm32h7/run_crypto_accel.py` for the host register model.
It requires gcc, g++ and OpenSSL development headers. At O0 and O2 with
ASan/UBSan, it passes all 92 shared checks through the production CRYP/HASH
driver, plus 200 rounds of AES-256 ECB/CBC/CTR against OpenSSL, in-place
AES, CTR tails/carry/wrap, timeout recovery, output/key clearing and shared
peripheral locking checks. HMAC fault injection covers key hashing, inner
and outer digests, cross-instance exclusion and disabled-engine rejection.
These fault-injection checks are model results, separate from the board run. Register definitions come from the STM32H753
CMSIS header; OpenSSL supplies independent block/digest results.

AES-256 adds 12 shared checks: SP 800-38A encryption/decryption vectors
for ECB/CBC/CTR, in-place operations with unaligned buffers and guards, a
partial CTR tail, key length/usage validation, and block length/IV validation.
These checks passed on the board in both the 87- and 92-check runs.

HMAC adds five checks: RFC 4231 long-key and truncated known answers, key
lengths 0/1/63/64/65/131 with message lengths 0..257 and output lengths 1..32,
overlapping output, and input/key policy validation. The board result is
**92 passed, 0 failed, no skips**. This is functional validation; no crypto
throughput measurement or entropy certification is claimed.


The shared test declares a `Crypto` object and calls its `Init()` member; `board.h`
contains no engine selection. Rebuild both the library and application because
the generic object layout and initialization changed. Historical 92-check board results
above predate this integration. The updated H753 host register model passes
100 checks with zero failures or skips at O0 and O2 under ASan/UBSan. Direct construction, failed initialization/retry and independent access by two
application objects are also checked. Its entropy
adapter uses host OpenSSL; physical RNG remains covered by the separate RNG
register model and prior board tests. A new board run is required for this
object integration.
