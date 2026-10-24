# Crypto capabilities in the object model

In the object model an engine's capability is not a bitfield. It is expressed
two ways:

1. **Which operation family the engine exposes.** A `KeyAgreeEngine *`,
   `CipherEngine *`, or `HashEngine *` selects the relevant interface. The
   call result still decides whether that concrete engine implements the
   requested operation and algorithm; an interface pointer is not a capability bit.
2. **Which algorithm value the interface method accepts.** Within an interface the
   algorithm is an argument (`CRYPTO_CIPHER_ALG`, `CRYPTO_MAC_ALG`,
   `CRYPTO_HASH_ALG`, `CRYPTO_CURVE`). An engine that does not implement a given
   value returns `CRYPTO_STATUS_UNSUPPORTED` from the base interface method, which it
   simply does not override.

There is no `Cap` word to query and no `CryptoIsCapable`. "Can this engine do X"
is answered by a non-null interface pointer followed by the operation return status.
Algorithm support is also answered by that return status. This removes the class of bug where
a `Cap` bit and the backing function pointer disagree.

## Interfaces

| Interface | Operation | Algorithm argument | Values today |
|---|---|---|---|
| `CipherEngine` | `Cipher` | `CRYPTO_CIPHER_ALG` | `CRYPTO_CIPHER_ECB`, `CRYPTO_CIPHER_CTR`, `CRYPTO_CIPHER_CBC` (AES-128/256); XTS reserved |
| `MacEngine` | `Mac` | `CRYPTO_MAC_ALG` | `CRYPTO_MAC_CMAC` (AES engines), `CRYPTO_MAC_HMAC` (SHA-256 engine); GMAC reserved |
| `AeadEngine` | `Seal`, `Open` | `CRYPTO_AEAD_ALG` | `CRYPTO_AEAD_AES_CCM` (RFC 3610), `CRYPTO_AEAD_AES_GCM` (SP 800-38D, 96 bit IV) |
| `HashEngine` | `Hash`; `HashInit` / `HashUpdate` / `HashFinal` | `CRYPTO_HASH_ALG` | `CRYPTO_HASH_SHA256`, one-shot and streaming |
| `KeyAgreeEngine` | `KeyGen`, `KeyImport`, `Agree` | `CRYPTO_CURVE` | `CRYPTO_CURVE_P256` |
| `SignEngine` | `Sign`, `Verify` | `CRYPTO_CURVE` | `CRYPTO_CURVE_P256` (ECDSA) |
| `RngEngine` | `Random` | none | hardware or software source |

Streaming hash contexts are caller storage of `HashCtxSize()` bytes aligned to
`HashCtxAlign()`; generic consumers reserve `CRYPTO_HASHCTX_MAX` aligned to
`CRYPTO_HASHCTX_ALIGN_MAX`. Key-agreement contexts follow the same size/alignment
rule through `KeyCtxSize()` and `KeyCtxAlign()`. `CRYPTO_STATUS_BUSY` marks transient accelerator
contention: outputs are cleared and a caller-held single-use key survives for
retry, while `CRYPTO_STATUS_FAIL` always consumes it.

New primitives are added by extending an algorithm enum (a new
`CRYPTO_CIPHER_*` or `CRYPTO_HASH_*` value) or, for a new primitive family, by
adding an interface. Existing values are never renumbered.

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

## Which engine implements which interface

| Engine | Interfaces |
|---|---|
| `CryptoUecc` | `KeyAgreeEngine`, `SignEngine` |
| `Ba414ep` | `KeyAgreeEngine`, `SignEngine` (software import and ECDSA inherited) |
| `CryptoCc3xx` | `KeyAgreeEngine`, `SignEngine` (software import and ECDSA inherited) |
| `CryptoWbaPka` | `KeyAgreeEngine`, `SignEngine` (hardware overrides of `CryptoUecc`) |
| `CryptoSoftAes` | `CipherEngine`, `MacEngine`, `AeadEngine` |
| `CryptoMaster` | `CipherEngine`, `MacEngine`, `AeadEngine` |
| `CryptoSoftSha256` | `HashEngine`, `MacEngine` (HMAC-SHA-256) |
| `CryptoSoftRng`, `CryptoRngNrf`, `CryptoRngStm32` | `RngEngine` |
| `CryptoCtlrSdc` | `CipherEngine` (Bluetooth-owned, controller AES) |

## The software AES base and derived constructions

`CryptoSoftAes` implements `CipherEngine` (AES-128/256 ECB/CTR/CBC), `MacEngine`
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
interface), which returns a raw 32-byte digest and is per-instance. The two do not
collide and serve different purposes.

## Generic object initialization

Declare `Crypto g_Crypto` and call `g_Crypto.Init()` after system startup. The
compiler supplies object storage and alignment. The MCU library binds its
hardware implementation internally; software-only targets use the object's
inherited software behavior. No raw buffer, public factory or placement new is
required. All capabilities are methods on the same application object.

The caller checks operation status. Missing acceleration does not make a
software-supported operation optional. P-256 key import and AES-128/256
cipher tests are required on every configured target. Initialization failure, busy and
failed operations must not be hidden as unsupported. A non-secure software RNG
cannot be used for key generation or signing; CryptoTest reports those cases
explicitly. Disabling one object does not disable the library's shared hardware
implementation used by another object.

## Software cipher base and hardware overrides

AES-128 and AES-256 ECB/CBC/CTR are implemented by `CryptoSoftAes`. Hardware
classes override the operations they accelerate. CryptoMaster and STM32WBA
call the base Cipher implementation for AES-256, while H753 accelerates both
key sizes. The nRF5340 ECB class inherits the mode implementation and replaces
its AES-128 block hooks; AES-256 uses the software block without acquiring
that peripheral. The application and generic object perform no retry based
on unsupported status. Busy and failed operations remain errors.

AES-256 shares the AES core and cipher-mode loops with AES-128, uses the
256-bit key expansion and 14 rounds, and wipes its round keys. Key location
and usage checks still apply in the software base. CMAC/CCM/GCM currently
accept AES-128 keys; this cipher change does not add 256-bit keys to those
constructions. AES-256 cipher tests are required on every target.
