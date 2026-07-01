# Crypto capability and property plan

Status: the Cap/Props split is implemented (`Props` word, `PLAIN_KEYCTX` moved
out of `Cap`, bit 31 free), AES-CMAC (bit 3), AES-CCM (bit 4), AES-GCM
(bit 5), SHA-256 (bit 6), HMAC-SHA-256 (bit 7), ECDSA-P256 sign (bit 8), and
ECDSA-P256 verify (bit 9) are all implemented. An engine advertises only what
it provides.

## Operation capabilities (Cap)

`CryptoDev_t.Cap` (uint32_t) holds operations only:

```text
bit 0   CRYPTO_CAP_AES128_ECB     single-block AES-128 ECB (implemented)
bit 1   CRYPTO_CAP_ECDH_P256      P-256 keygen + ECDH (implemented)
bit 2   reserved                  (freed when RNG was removed; RNG is coredev)
bit 3   CRYPTO_CAP_AES_CMAC       AES-CMAC RFC 4493 (implemented; over any AES-128 ECB engine)
bit 4   CRYPTO_CAP_AES_CCM        AES-CCM AEAD RFC 3610 (implemented; over any AES-128 ECB engine)
bit 5   CRYPTO_CAP_AES_GCM        AES-GCM AEAD SP 800-38D (implemented; over any AES-128 ECB engine)
bit 6   CRYPTO_CAP_SHA256         SHA-256 FIPS 180-4 (implemented; base software core)
bit 7   CRYPTO_CAP_HMAC_SHA256    HMAC-SHA-256 RFC 2104 (implemented; over SHA-256)
bit 8   CRYPTO_CAP_ECDSA_P256_SIGN    ECDSA P-256 sign (implemented; native per EC engine)
bit 9   CRYPTO_CAP_ECDSA_P256_VERIFY  ECDSA P-256 verify (implemented; native per EC engine)
```

Proposed additions, grouped by primitive family. Append only; do not renumber:

```text
bit 10  SHA512              reserved
bit 11  X25519              reserved (Curve25519 ECDH)
bit 12  ED25519_VERIFY      reserved (Ed25519 verify)
bits 13..31                 free for future operations
```

## Properties (Props)

`CryptoDev_t.Props` (uint32_t) describes the engine, not its operations, and is
never part of the `Cap` a consumer queries. A Cryptor and the selector read it
through `CryptoHasProp`:

```text
bit 0   CRYPTO_PROP_PLAIN_KEYCTX    key context is plain zeroable bytes (Cryptor may hand it pMem)
bit 1   CRYPTO_PROP_HARDWARE        backed by a hardware accelerator
bit 2   CRYPTO_PROP_SECURE_DOMAIN   key stays inside a secure domain / keystore
bit 3   CRYPTO_PROP_SYNC            engine is always synchronous (no PENDING completion)
bits 4..31                          reserved
```

`CRYPTO_PROP_SYNC` (the engine is synchronous) is the descriptive counterpart of
`CryptoCfg_t.Flags` `CRYPTO_FLAG_SYNC` (the App requires synchronous). The
selector can match the request against the property.

Property values the engines set today:

```text
uECC        PLAIN_KEYCTX | SYNC
mbedTLS     SYNC
PSA         HARDWARE | SECURE_DOMAIN | SYNC
CC310       HARDWARE | SYNC
SDC AES     HARDWARE | SYNC
```

A Cryptor advertises no properties of its own (`Props` is 0 on the composed
handle); it is a composition over engines, and its ECDH engine keeps the
`PLAIN_KEYCTX` property that `CryptorBuild` checks before forwarding `pMem`.

## Note on the existing SHA-256 utility

IOsonata already has a software SHA-256, `Sha256()` in `src/isha256.c` and
`include/isha256.h` (with `isha1` alongside). It was written for download / DFU
key checking, not as a general crypto primitive: it returns a hex string and
keeps global static state (one hash at a time). It stays as is for that use and
is not shared with the crypto layer.

A crypto-layer SHA-256 (`CRYPTO_CAP_SHA256`) is a separate, independent
operation: raw 32-byte digest, per-instance, and hardware-capable through PSA or
CC310. The crypto wrapper is `CryptoSha256`; the existing global `Sha256()`
keeps its name, so the two do not collide.

## Vtable growth for new operations

Each new operation adds one function pointer to the `CryptoDev_t` vtable and one
inline wrapper, matching the existing `Aes128Ecb` / `EcdhP256KeyGen` /
`EcdhP256` pattern:

```text
Cmac        (*Cmac)(pDev, key, msg, len, mac16, pCtx)
Ccm         (*CcmEncrypt/Decrypt)(pDev, key, nonce, aad, in, out, tag, pCtx)
Sha256      (*Sha256)(pDev, msg, len, digest32, pCtx)
Hmac        (*HmacSha256)(pDev, key, klen, msg, len, mac32, pCtx)
EcdsaSign   (*EcdsaP256Sign)(pDev, pKeyCtx, hash32, sig64, pOpCtx)
EcdsaVerify (*EcdsaP256Verify)(pDev, pubKey64, hash32, sig64, pOpCtx)
```

An engine sets only the pointers it implements and the matching `Cap` bits;
unset pointers stay NULL and the inline wrapper returns
`CRYPTO_STATUS_UNSUPPORTED`. Operations that keep no per-instance secret take a
single `pCtx`; operations with per-use-case key state take the `pKeyCtx` /
`pOpCtx` split, the rule ECDH already follows.

## Order of work

All roadmap primitives below are implemented and validated against
known-answer vectors (uECC path in-sandbox; hardware engines on-target).


1. Cap/Props split and free bit 31 (done).
2. AES-CMAC (bit 3): done. Generic over `Aes128Ecb` (RFC 4493), so it works on
   the controller AES, PSA, and mbedTLS engines; a native `pDev->Cmac` can be
   slotted in later for hardware MAC. Unblocks the SMP toolbox and LoRaWAN MIC.
3. AES-CCM (bit 4): done. Generic AEAD (RFC 3610) over `Aes128Ecb`, with AAD,
   tag verify, and no plaintext release on tag mismatch. For LoRaWAN payloads
   and application-layer AEAD (BLE LE link encryption is done by the radio).
4. SHA-256 (bit 6) and HMAC-SHA-256 (bit 7): done. Built-in FIPS 180-4 core,
   always available through any handle; native via the Sha256 slot later. HMAC
   (RFC 2104) over the core.
5. ECDSA verify (bit 9): done. Native per EC engine (uECC uECC_verify, mbedTLS,
   PSA psa_verify_hash, CC310 nrf_crypto). Verifies a 64-byte r||s signature
   over a 32-byte hash with a 64-byte X||Y public key. For DFU image signatures.
6. AES-GCM (bit 5): done. Generic AEAD (SP 800-38D) over Aes128Ecb: CTR plus
   GHASH, 96-bit IV fast path and general IV, AAD, tag verify, no plaintext
   release on mismatch. For TLS records and application-layer AEAD.
