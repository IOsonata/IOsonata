# Crypto capabilities in the object model

In the object model an engine's capability is not a bitfield. It is expressed
two ways:

1. **Which facets the engine implements.** A `KeyAgreeEngine` does P-256
   agreement; a `CipherEngine` does AES; a `HashEngine` does a digest. A
   consumer that needs agreement holds a `KeyAgreeEngine *`, and the type system
   already guarantees the operation exists.
2. **Which algorithm value the facet method accepts.** Within a facet the
   algorithm is an argument (`CRYPTO_CIPHER_ALG`, `CRYPTO_MAC_ALG`,
   `CRYPTO_HASH_ALG`, `CRYPTO_CURVE`). An engine that does not implement a given
   value returns `CRYPTO_STATUS_UNSUPPORTED` from the base facet method, which it
   simply does not override.

There is no `Cap` word to query and no `CryptoIsCapable`. "Can this engine do X"
is answered by the pointer type plus a non-null check, and "does it support this
algorithm" is answered by the return status. This removes the class of bug where
a `Cap` bit and the backing function pointer disagree.

## Facets

| Facet | Operation | Algorithm argument | Values today |
|---|---|---|---|
| `CipherEngine` | `Cipher` | `CRYPTO_CIPHER_ALG` | `CRYPTO_CIPHER_ECB`, `CRYPTO_CIPHER_CTR`, `CRYPTO_CIPHER_CBC` (AES-128); XTS reserved |
| `MacEngine` | `Mac` | `CRYPTO_MAC_ALG` | `CRYPTO_MAC_CMAC` (AES engines), `CRYPTO_MAC_HMAC` (SHA-256 engine); GMAC reserved |
| `AeadEngine` | `Seal`, `Open` | `CRYPTO_AEAD_ALG` | `CRYPTO_AEAD_AES_CCM` (RFC 3610), `CRYPTO_AEAD_AES_GCM` (SP 800-38D, 96 bit IV) |
| `HashEngine` | `Hash`; `HashInit` / `HashUpdate` / `HashFinal` | `CRYPTO_HASH_ALG` | `CRYPTO_HASH_SHA256`, one-shot and streaming |
| `KeyAgreeEngine` | `KeyGen`, `Agree` | `CRYPTO_CURVE` | `CRYPTO_CURVE_P256` |
| `SignEngine` | `Sign`, `Verify` | `CRYPTO_CURVE` | `CRYPTO_CURVE_P256` (ECDSA) |
| `RngEngine` | `Random` | none | hardware or software source |

Streaming hash contexts are caller storage of `HashCtxSize()` bytes aligned to
`HashCtxAlign()`; every in-tree consumer reserves `CRYPTO_HASHCTX_MAX` with
`alignas(uint64_t)`. `CRYPTO_STATUS_BUSY` marks transient accelerator
contention: outputs are cleared and a caller-held single-use key survives for
retry, while `CRYPTO_STATUS_FAIL` always consumes it.

New primitives are added by extending an algorithm enum (a new
`CRYPTO_CIPHER_*` or `CRYPTO_HASH_*` value) or, for a new primitive family, by
adding a facet. Existing values are never renumbered.

## Engine descriptive properties

The object exposes a few descriptive predicates instead of a `Props` word:

- `IsAsync()` : the engine completes later through the completion handler rather
  than in-call. Default false. A consumer that must have synchronous behavior
  checks `!IsAsync()`.
- `IsSecure()` (on `RngEngine`) : the source is hardware-grade entropy, not a
  statistical PRNG. A security-path caller must use a secure source.

Whether an engine is hardware-backed or keeps its key in a secure keystore is a
property of which concrete class was constructed (`Ba414ep`, `CryptoCc3xx`),
known at the point of construction, so it does not need a runtime property
bit.

## Which engine implements which facet

| Engine | Facets |
|---|---|
| `CryptoUecc` | `KeyAgreeEngine`, `SignEngine` |
| `Ba414ep` | `KeyAgreeEngine` |
| `CryptoCc3xx` | `KeyAgreeEngine` |
| `CryptoSoftAes` | `CipherEngine`, `MacEngine`, `AeadEngine` |
| `CryptoMaster` | `CipherEngine`, `MacEngine`, `AeadEngine` |
| `CryptoSoftSha256` | `HashEngine`, `MacEngine` (HMAC-SHA-256) |
| `CryptoSoftRng`, `CryptoRngNrf`, `CryptoRngStm32` | `RngEngine` |
| `CryptoCtlrSdc` | `CipherEngine` (Bluetooth-owned, controller AES) |

## The software AES base and derived constructions

`CryptoSoftAes` implements `CipherEngine` (AES-128 ECB/CTR/CBC), `MacEngine`
(AES-CMAC) and `AeadEngine` (AES-CCM, AES-GCM). CMAC, CCM and GCM are computed
over the protected virtual block primitive `AesEcbEncrypt` inside one
`AesOpBegin` / `AesOpEnd` bracket per operation, so a hardware AES engine that
overrides those three (`CryptoMaster`) gets hardware-backed CMAC, CCM and GCM
under a single accelerator acquisition with no further code. The GCM GHASH
field multiply is constant-time software on every engine. HMAC-SHA-256
follows the same pattern on the hash side: the generic RFC 2104 construction
in `CryptoSoftSha256` chains the virtual streaming hash calls, so a hardware
digest block that overrides them provides the digest work. The generic HMAC
requires a synchronous hash engine (`!IsAsync()`).

## Note on the DFU SHA-256 utility

`Sha256()` in `src/isha256.c` / `include/isha256.h` is a separate, purpose-built
utility for download / DFU key checking: it returns a hex string and keeps global
state (one hash at a time). It is not part of the crypto engine tree and is not
shared with it. The crypto-layer digest is `CryptoSoftSha256` (the `HashEngine`
facet), which returns a raw 32-byte digest and is per-instance. The two do not
collide and serve different purposes.
