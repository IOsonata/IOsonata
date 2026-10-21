# CryptoTest: NUCLEO-H753ZI

This project runs the existing `exemples/crypto/crypto_engine_test.cpp`, also
used by STM32WBA. It selects `CryptoRngStm32` for hardware random numbers and
selects `CryptoStm32H7Aes` for the CRYP AES accelerator and
`CryptoStm32H7Hash` for one-shot SHA-256 on HASH. P-256 uses `CryptoUecc`.

AES-128 ECB, CBC and CTR encryption/decryption use the hardware block
primitive. CBC chaining and the full 128-bit CTR increment are handled by
the driver. CMAC, CCM and GCM reuse the `CryptoSoftAes` constructions over
hardware AES. AES-256 is not offered by this engine.

SHA-256 streaming and HMAC retain the software implementation: the HASH
peripheral's 54-word saved context exceeds the existing 128-byte caller
context limit. One-shot hashes use hardware, including empty and unaligned
messages. No context-size or public API changes are required.

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

On October 21, 2026, the NUCLEO-H753ZI board run passed all 72 checks with
zero failures and no skips, including software ECB/CBC decryption and both
P-256 boundary scalars (1 and n - 1).

The updated shared test requires both boundary scalars (1 and n - 1) to pass
public-key import, ECDH, key retention/consumption, invalid-peer rejection,
and signature verification, including independent OpenSSL signatures.
The software adapter handles micro-ecc's co-Z and Shamir boundary cases.
It uses micro-ecc's integer API (`uECC_ENABLE_VLI_API=1`), already enabled in
the library project.

The updated host suite passes 72 checks with zero failures under ASan/UBSan.
That board result validates hardware RNG and software AES/P-256, before
the example selected CRYP and HASH. The accelerator version needs a new
board run. Rebuild both the H753 library and CryptoTest; expect 75 checks,
zero failures and no skips. Three added checks cover hash construction,
SHA-256/HMAC self-test, and SHA-256 lengths 0 through 257 against software
with unaligned input/output and output guards.

Run `python3 tests/stm32h7/run_crypto_accel.py` for the host register model.
It requires gcc, g++ and OpenSSL development headers. At O0 and O2 with
ASan/UBSan, it runs the shared suite through the production CRYP/HASH driver
and checks timeout recovery, output/key clearing and shared peripheral
locking. Register definitions come from the STM32H753 CMSIS header; OpenSSL
supplies independent block/digest results. This does not validate hardware
timing or silicon behavior. An ARM build was not run in this environment
because the cross-compiler was unavailable.

Ethernet TLS/DTLS is not implemented.
