# CryptoTest: NUCLEO-H753ZI

This project runs the existing `exemples/crypto/crypto_engine_test.cpp`, also
used by STM32WBA. It selects `CryptoRngStm32` for hardware random numbers and
uses the existing `CryptoSoftAes` and `CryptoUecc` engines for AES and P-256.
It does not exercise the H753 CRYP or HASH accelerators.

Import `ioc` and `../../lib/ioc` into IOcomposer. Build the library and example
in matching configurations. Use Release for the board test: stdout uses
USART3/ST-LINK VCP, PD9 RX and PD8 TX, at **115200 baud, 8N1**. Debug uses
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

On October 21, 2026, the board run passed 56 checks with zero failures,
including software ECB/CBC decryption. That run still skipped P-256 scalar
n - 1.

The updated shared test requires both boundary scalars (1 and n - 1) to pass
public-key import, ECDH, key retention/consumption, invalid-peer rejection,
and signature verification, including independent OpenSSL signatures.
The software adapter handles micro-ecc's co-Z and Shamir boundary cases.
It uses micro-ecc's integer API (`uECC_ENABLE_VLI_API=1`), already enabled in
the library project.

The updated host suite passes 72 checks with zero failures under ASan/UBSan.
The new boundary cases still need a board run: rebuild both the library and
CryptoTest. Ethernet TLS/DTLS is not implemented.
