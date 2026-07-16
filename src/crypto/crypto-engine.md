# Crypto engine architecture

IOsonata's cryptographic primitives are modeled the same way as every other
device in the framework: as objects on the `Device` base. A crypto engine is a
`Device`, so it carries the same lifecycle (`Enable` / `Disable` / `Reset`) and
event model as a bus or a sensor. On top of `Device` sit the crypto facets, each
a small abstract interface for one family of operations.

## The facet tree

`include/crypto/icrypto.h` defines the facets. Each is a pure interface that
states what operation is wanted, never how or where it runs:

- `CryptoEngine` : the common base, `virtual public Device`. Holds the async
  completion contract (see below).
- `CipherEngine` : symmetric cipher (`Cipher`, AES-128 ECB/CTR/CBC).
- `MacEngine` : keyed message authentication (`Mac`, AES-CMAC).
- `HashEngine` : unkeyed message digest (`Hash`, SHA-256).
- `KeyAgreeEngine` : key generation and key agreement (`KeyGen`, `Agree`, P-256
  ECDH).
- `SignEngine` : signature (`Sign`, `Verify`, ECDSA P-256).
- `RngEngine` : random source (`Random`).

A facet derives `virtual public CryptoEngine`, so a concrete engine that
implements several facets (for example a part that does both AES and ECDH)
inherits one `Device` sub-object through the virtual base rather than several.

A consumer holds a facet pointer (`KeyAgreeEngine *`, `CipherEngine *`) and calls
the facet method. It is blind to whether the work is software, a hardware
accelerator (Arm CryptoCell CC310, Silex CRACEN / BA414EP), or the BLE
controller. An operation a given engine does not implement returns
`CRYPTO_STATUS_UNSUPPORTED`; the base facet method supplies that default, so an
engine only overrides what it provides.

`Device::Enable()` is pure virtual, so every concrete engine must override
`Enable` / `Disable` / `Reset`. An engine that skips them is abstract and will
not instantiate.

## Construction: explicit factories, no selector

Every engine is constructed by a placement-new factory into caller-owned
storage. There is no allocation and no runtime provider selector: the
application constructs exactly the engine it wants and injects it.

```cpp
static uint8_t     s_EcdhMem[BA414EP_MEMSIZE];              // App-owned storage
KeyAgreeEngine *pEcdh = Ba414epCreate(s_EcdhMem, sizeof(s_EcdhMem),
                                      CryptoRngNrfInstance());
CipherEngine   *pAes  = BtCryptoCtlrSdcInit();              // controller AES
BtSmpInit(pEcdh, pAes);                                     // inject into SMP
```

The factory validates the storage (size and alignment) and returns `nullptr` on
a bad arena or if hardware bring-up fails. The storage size comes from the
engine macro (`BA414EP_MEMSIZE`, `CRYPTO_UECC_MEMSIZE`,
`CRYPTO_SOFTSHA256_MEMSIZE`, and so on), which is the object `sizeof`.

Because construction is explicit, the linker pulls only the engines a target
actually constructs. There is no weak-stub registry, no `_none` fallback object,
and no `AUTO` selection symbol: those existed only to satisfy the old
symbol-based selector and are not needed in the object model.

## Per-instance key state

Facets that hold a key between calls (`KeyAgreeEngine`) take a `pKeyCtx` (a
caller-owned key-context buffer of `KeyCtxSize()` bytes). `KeyGen` writes the
private key into it and `Agree` consumes it, so one engine object serves several
independent key contexts. The engine object itself is stateless beyond its
`Device` lifecycle; the scarce hardware resource, not a per-key secret, is what
the object owns.

`CipherEngine`, `MacEngine`, and `HashEngine` take the key (if any) in the call,
so they hold no per-instance key state and one object serves any number of keys.

## The async completion contract

`CryptoEngine` carries an operation and completion model, so an in-call software
engine and an interrupt-driven or off-die hardware engine present the same
interface:

- `CRYPTO_OP` names the operation kind (`CRYPTO_OP_CIPHER`, `CRYPTO_OP_AGREE`,
  `CRYPTO_OP_HASH`, and so on).
- `IsAsync()` reports whether the engine completes in-call (default false) or
  later.
- A synchronous engine returns a final status in-call and never signals
  completion. An async engine returns `CRYPTO_STATUS_PENDING` and later calls the
  protected `Complete(op, status)`, which invokes the handler set with
  `SetCompleteHandler`.

SMP drives both paths through one code path: it arms its pending machinery, and a
sync engine completes inline while an async engine completes through the handler.
Every engine in the tree today is synchronous; the contract is in place so an
async engine can be added without an interface change.

## Security properties

The engine layer is where the cryptographic checks live, because the SMP
consumer is curve-blind and key-blind:

- **Peer key validation**: a P-256 engine rejects a peer public key that is not
  on the curve before the agreement, closing the invalid-curve attack
  (CVE-2018-5383). The software engine checks with `uECC_valid_public_key`; the
  PSA path relies on `psa_raw_key_agreement`, which performs the check; the PKA
  engines validate the point on the accelerator.
- **Single-use ephemeral key**: `Agree` wipes the shared secret on every exit
  and, by default, wipes the private key too, so a second `Agree` without a fresh
  `KeyGen` fails rather than reusing a spent key. A caller that needs one
  ephemeral pair for several concurrent peers passes `bKeepKey = true`, which
  keeps the key after a success; a failure always wipes.
- **Per-instance key context**: the private key lives in caller-owned `pKeyCtx`,
  never in a file static, so concurrent or sequential key contexts never collide.
- **Strong RNG required**: `RngGet` is the platform hardware random source,
  declared in `crypto/icrypto.h`, not a crypto capability. The P-256 engines use
  it for key generation. There is no software default: a part without an RNG
  peripheral does not link. Statistical randomness for tests and non-security use
  comes from `rand_r`, never from `RngGet`, and never the reverse.

## Engines

| Engine | Facets | Location | Notes |
|---|---|---|---|
| `CryptoUecc` | `KeyAgreeEngine`, `SignEngine` | `src/crypto/crypto_uecc.cpp` | Software P-256 (micro-ecc): ECDH and ECDSA. Uses `RngGet` for `KeyGen`/`Sign`; `Verify` needs no RNG. Software fallback on any part. |
| `Ba414ep` | `KeyAgreeEngine` | `src/crypto/ba414ep.cpp`, `ARM/Nordic/src/ba414ep_nrfx.cpp` | Hardware P-256 on the Silex BA414EP public-key accelerator (Nordic CRACEN). nRF54L15 / nRF54H20. Microcode-free, fixed-function; no vendor blob. |
| `CryptoCc3xx` | `KeyAgreeEngine` | `ARM/src/crypto_cc3xx.cpp` | Hardware P-256 on the Arm CryptoCell CC3xx PKA. nRF52840 (CC310). Self-contained register-level driver; the target header `crypto_cc3xx.h` supplies the register base and enable/disable. |
| `CryptoSoftAes` | `CipherEngine`, `MacEngine` | `src/crypto/crypto_softaes.cpp` | Software AES-128 (FIPS-197) and AES-CMAC. The software base of the symmetric facets: a hardware AES block overrides `Cipher`, and the inherited software CMAC then runs over that hardware AES through the virtual `Cipher` call. |
| `CryptoMaster` | `CipherEngine`, `MacEngine` | `src/crypto/cryptomaster.cpp` | Hardware AES-128 on the Silex CryptoMaster (BA411e) block. Overrides `Cipher`; CMAC is the inherited software core over the hardware cipher. |
| `CryptoSoftSha256` | `HashEngine` | `src/crypto/crypto_softsha256.cpp` | Software SHA-256 (FIPS 180-4). Unkeyed digest; serves DFU image verification and any hashing consumer. |
| `CryptoSoftRng` | `RngEngine` | `src/crypto/crypto_softrng.cpp` | Software PRNG base. Statistical use only; not a security random source. |
| `CryptoRngNrf` | `RngEngine` | `ARM/Nordic/src/rng_nrfx.cpp` | Nordic hardware RNG (CRACEN CTR-DRBG on nRF54L/H, NRF_RNG peripheral on nRF52/53/91). Singleton `CryptoRngNrfInstance`; `RngGet` is its C-shim. |
| `CryptoRngStm32` | `RngEngine` | `ARM/ST/src/rng_stm32.cpp` | STM32 hardware RNG peripheral. Singleton `CryptoRngStm32Instance`; `RngGet` C-shim. |
| `CryptoPsa` | `CipherEngine`, `KeyAgreeEngine` | `src/crypto/crypto_psa.cpp` | AES-128 and P-256 over the ARM PSA Crypto API; key held in the PSA keystore by handle. Kept in the tree but linked by no current target: every current part has a native driver or software. Opt-in for a future PSA-only platform. |

Engine class names are provider-class, not target-specific. `CryptoCc3xx` names
the CryptoCell family; the target header pins the actual chip. The `_bm` suffix
is reserved for code that touches the sdk-nrf-bm SDK specifically, never for
generic crypto.

## Subsystem-owned adapters

Some engines are not generic facilities and deliberately live outside the crypto
layer:

| Adapter | Owner | Notes |
|---|---|---|
| `CryptoCtlrSdc` | Bluetooth (`ARM/Nordic/src/bt_crypto_ctlr_sdc.cpp`, `BtCryptoCtlrSdcInit` declared in `bt_smp.h`) | AES-128 ECB through the BLE controller HCI LE Encrypt. Cannot function without a running BLE controller, so only a Bluetooth consumer may use it. It is a `CipherEngine` so SMP composes it into its AES slot uniformly, but it is not a reusable engine and is not advertised in the crypto layer. |

## Composition for SMP

`BtSmpInit(KeyAgreeEngine *pEcdh, CipherEngine *pAes)` takes one ECDH engine and
one AES engine. RNG is the platform hardware RNG through `RngGet`, used by the
engines and the SMP toolbox directly, so it is not a separate slot. A null or
incapable engine leaves that path disabled, so pairing fails loud rather than
running under absent crypto.

The application constructs the right engines for its part at compile time:

- **nRF52840 (CC310):** `CryptoCc3xx` for ECDH, controller AES for the cipher.
- **nRF54L15 / nRF54H20 (CRACEN):** `Ba414ep` for ECDH, controller AES for the
  cipher.
- **nRF52832 (no accelerator):** `CryptoUecc` for ECDH, controller AES.
- **nRF5340 network core:** `CryptoUecc` for ECDH. The CC312 sits in the
  application core secure domain and is not reachable from the network core, so
  software P-256 is the correct choice, not a fallback.

## Host-driven vs stack-owned SMP

The engine composition applies only where IOsonata's `bt_smp.cpp` runs the SMP
state machine in-host:

- **Host-driven** (SDC; sdk-nrf-bm / nRF54 in-host): construct the engines and
  fill the slots.
- **Stack-owned** (STM32WB): the co-processor firmware owns pairing through the
  `ACI_GAP` vendor interface, with the on-die PKA handling ECDH below the API.
  `bt_smp.cpp` is not in the pairing path and no engine is injected.

## Reuse beyond Bluetooth

The facets are not Bluetooth-scoped. The same engines serve any consumer: secure
DFU (`CryptoSoftSha256` + `CryptoUecc::Verify` validate a signed image), and TLS
or encrypted storage as those consumers arrive. New primitives are added by
appending a facet or an algorithm value, never by renumbering existing ones.

## File layout

| File | Role |
|---|---|
| `include/crypto/icrypto.h` | The facet tree: `CryptoEngine` and the facet interfaces, the algorithm and operation enums, the async completion contract, `CryptoKey`, `RngGet`, `CryptoSecureWipe`, and the P-256 helper declarations. |
| `src/crypto/crypto_p256.cpp` | Generic P-256 math: constant-time byte helpers, scalar and field range checks, and the ladder scalar regularization (`P256RegularizeScalar`, `P256RegularBit`). Exports functions only; the curve point arithmetic stays in each engine. |
| `src/crypto/crypto_uecc.cpp` | Software P-256 engine (micro-ecc): ECDH and ECDSA. |
| `src/crypto/crypto_softaes.cpp` | Software AES-128 engine: cipher and CMAC. |
| `src/crypto/cryptomaster.cpp` | Hardware AES engine on the Silex CryptoMaster (BA411e). |
| `src/crypto/crypto_softsha256.cpp` | Software SHA-256 engine. |
| `src/crypto/crypto_softrng.cpp` | Software PRNG base (statistical). |
| `src/crypto/ba414ep.cpp` | Hardware P-256 engine on the Silex BA414EP; `ARM/Nordic/src/ba414ep_nrfx.cpp` binds it to the Nordic CRACEN registers. |
| `src/crypto/crypto_psa.cpp` | AES + P-256 over the PSA Crypto API. Opt-in; no current target links it. |
| `ARM/src/crypto_cc3xx.cpp` | Hardware P-256 engine on the CC3xx PKA; the target header `crypto_cc3xx.h` supplies the register base and enable/disable. nRF52840. |
| `ARM/Nordic/src/rng_nrfx.cpp` | Nordic hardware RNG engine (`CryptoRngNrf`). |
| `ARM/ST/src/rng_stm32.cpp` | STM32 hardware RNG engine (`CryptoRngStm32`). |
