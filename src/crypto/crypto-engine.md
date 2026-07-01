# Crypto engine architecture

IOsonata's cryptographic primitives are abstracted the same way as the
device-communication bus (`DeviceIntrf` / `DevIntrf_t`): a thin behavioral
interface that states what operation is wanted, never how or where it runs.

The crypto module has two layers, the same split as motion fusion over sensors:

- An **engine** is a provider, like a sensor. It implements the operations and
  owns the scarce shared resource behind them (a hardware block, the read-only
  curve tables, an entropy source). One engine object per physical resource.
- A **Cryptor** is a per-use-case instance, like an `Att` / `Nav` fusion object.
  It references one or more engines and forwards each operation to the engine
  that provides that capability, using its own per-instance key state.

The layer collapses when it is trivial. A dedicated software engine is both the
engine and the instance: pass its handle straight to the subsystem. Build a
Cryptor only to share one engine across several use cases, or to compose several
single-capability engines into one handle.

## The interface

`include/crypto/crypto.h` defines `CryptoDev_t`, a C struct of function pointers
(canonical form, like `DevIntrf_t`), plus capability bits, async-tolerant status
codes, and a completion callback. A consumer holds a `CryptoDev_t*` and calls
the `Crypto*` wrappers; it is blind to whether the work is software, a hardware
accelerator (Arm CryptoCell CC310/CC312, STM32 PKA/CRYP), or forwarded across a
TrustZone boundary to a secure-domain engine. Engines and Cryptor instances both
present a `CryptoDev_t`, so a subsystem cannot tell a dedicated engine from a
forwarding instance.

Three properties make it span any surface, regardless of MCU, platform, or
secure state:

- **Capability query** (`CryptoIsCapable`): an engine advertises only what it
  implements. An unsupported operation returns `CRYPTO_STATUS_UNSUPPORTED`,
  never undefined behavior. This is the analog of querying a bus rate.
- **Async tolerance**: an operation may return `CRYPTO_STATUS_PENDING` and
  complete later through the event callback, so a controller-offload or
  secure-service engine fits the same interface as a direct in-call engine.
- **Composability**: an engine is a `CryptoDev_t`; a secure-domain proxy is also
  a `CryptoDev_t` that holds, in its private data, a reference to the real engine
  on the far side and forwards each operation, the way SLIP composes onto a UART
  `DeviceIntrf` without the consumer knowing. A key that must not leave a secure
  domain never crosses the interface: the operation crosses, not the key.

The C++ side mirrors the bus tree. `CryptoDevice` wraps `CryptoDev_t` the way
`DeviceIntrf` wraps `DevIntrf_t`: an `operator CryptoDev_t * const ()` plus
methods that delegate to the `Crypto*` inline wrappers. `CryptoUecc` and
`CryptoMbedtls` are engine objects with an embedded per-instance arena;
`Cryptor` is the instance wrapper. Each passes where a `CryptoDev_t*` is wanted
by implicit conversion.

## Configuration and per-instance state

Each engine `Init` takes a `CryptoCfg_t`. The application fills it and owns the
state buffer it points at:

```c
CryptoDev_t    g_BleEcdh;                       // App-owned instance
static uint8_t g_BleMem[CRYPTO_MEMSIZE_UECC];   // App-owned per-instance state
CryptoCfg_t    cfg = { 0 };
cfg.Provider = CRYPTO_PROVIDER_UECC;
cfg.ReqCaps  = CRYPTO_CAP_ECDH_P256;
cfg.pMem     = g_BleMem;
cfg.MemSize  = sizeof(g_BleMem);
CryptoUeccInit(&g_BleEcdh, &cfg);
BtSmpInit(&g_BleEcdh, &g_BleAes);               // inject into the subsystem
```

`pMem` / `MemSize` is App-owned RAM (no heap) that holds the engine's
per-instance secret state. A separate buffer per instance keeps instances
independent: there is no file-static key, so two instances of one engine never
share a private key. Size the buffer with the provider macro,
`CRYPTO_MEMSIZE_UECC` or `CRYPTO_MEMSIZE_MBEDTLS`. The micro-ecc arena size is
exact. The mbedTLS macro covers the per-instance control structs only; mbedTLS
allocates MPI limbs through its own allocator, so the real working set also
depends on the mbedTLS heap configuration. Each `Init` re-checks `MemSize`
against its true `sizeof` and fails closed if the buffer is too small.

`ReqCaps` is validated at `Init`; an engine that cannot meet it returns false.
`Flags` holds policy bits: `CRYPTO_FLAG_SELFTEST` runs the engine known-answer test
at `Init` and fails `Init` on a test failure; `CRYPTO_FLAG_SYNC` rejects an
async provider; `CRYPTO_FLAG_NO_FALLBACK` stops the AUTO selector from dropping
to a software provider. `DevNo` and `IntPrio` apply to hardware engines only.

The ECDH operations take a separate `pKeyCtx` (per-instance key context) and
`pOpCtx` (operation/completion context). An engine resolves the key context as
`pKeyCtx ? pKeyCtx : pDevData`, so a dedicated engine uses its own `pDevData`
while a Cryptor sharing one engine supplies each instance's own context through
`pKeyCtx`, leaving `pOpCtx` free for async completion correlation. AES takes a
single `pCtx`: its key is passed in, so it holds no per-instance key state.

## Security properties

The engine layer is where the cryptographic checks live, because the SMP
consumer is curve-blind and key-blind:

- **Peer key validation**: both software engines reject a peer public key that
  is not on the P-256 curve before the ECDH (`uECC_valid_public_key`,
  `mbedtls_ecp_check_pubkey`). Without this the engine is open to the
  invalid-curve attack (CVE-2018-5383); neither `uECC_shared_secret` nor
  `mbedtls_ecdh_compute_shared` performs the check.
- **Ephemeral key wipe**: the P-256 private key is single-use. After the DH the
  engine wipes both the shared secret and the private key, so a later operation
  without a fresh key generation fails rather than reusing a spent key.
- **Per-instance state**: secret state lives in App-owned `pMem`, never in a
  file static, so concurrent or sequential instances do not collide.
- **Key-context guard**: a Cryptor forwards its `pMem` arena to the ECDH engine
  as the key context. That is valid only for an engine whose key context is
  plain zeroable bytes (`CRYPTO_PROP_PLAIN_KEYCTX` in `Dev.Props`, set by
  micro-ecc), and the arena must be at least the engine `KeyCtxSize`.
  `CryptorComposeInit` fails closed when the ECDH engine lacks that property
  or when `pMem` is smaller than its `KeyCtxSize`, so a keyed op cannot write
  past the arena. An engine with a structured key context (mbedTLS) does not
  set the property and is composed with `pMem` NULL. Properties live in
  `Dev.Props`, separate from the operation `Cap`, so nothing is masked out of
  the `Cap` the consumer sees.
- **Strong RNG required**: RNG is a coredev service (`coredev/rng.h`), not a
  crypto capability. The P-256 engines call the platform `RngGet` for key
  generation. The default `RngGet` is weak software randomness for non-security
  and test use; a target whose `RngGet` is not backed by a hardware TRNG must
  not run Secure Connections pairing, and must never use the software default
  for key generation.

## Engines

Each engine is a `CryptoDev_t` implementation: the library provides the
operation functions and an `Init` that populates a caller-owned instance. The
library never defines the instance. Each `Init` is compiled per platform: where
the dependency headers exist it brings up the real engine; where they do not, a
stub `Init` returns false.

| Engine Init | Capabilities | Location | Notes |
|---|---|---|---|
| `CryptoUeccInit` | ECDH P-256 | `src/crypto/crypto_uecc.cpp` | Software (micro-ecc). ECDH only. Borrows `RngGet`. |
| `CryptoMbedtlsInit` | AES + ECDH | `src/crypto/crypto_mbedtls.cpp` | Software; hardware-accelerated where the platform mbedTLS sits over CC3xx/CRACEN/PKA (nRF52840, nRF54, nRF91). Also serves TLS/DFU. |
| `CryptoHwInit` | ECDH (+AES on PSA) | `ARM/Nordic/nRF54/src/crypto_psa.cpp`, `ARM/Nordic/nRF52/src/crypto_cc310.cpp` | Architecture hardware engine. Two Nordic implementations of the one public symbol: `crypto_psa.cpp` uses the PSA Crypto API that sdk-nrf-bm ships bare-metal (dispatching to CRACEN on nRF54L; also CC3xx where a PSA lib is present), `crypto_cc310.cpp` uses nrf_crypto over CC310 (nRF5 SDK, nRF52840). Each is guarded by the presence of its SDK header; PSA takes precedence when both are present, so they never collide. A target with neither falls back to the weak stub, and `CryptoInit(AUTO)` then uses software uECC. Other ports (STM32 PKA, ESP) add their own `CryptoHwInit` later. |

Public engine names are provider-class, never target-specific. A port implements
`CryptoHwInit` for its architecture, but the public symbol stays `CryptoHwInit`;
there is no public `CryptoCracenInit` or `CryptoStm32Init`.

## The base file: provider registry and selector

`src/crypto/crypto.cpp` holds the generic, provider-independent part of the
module:

- **Weak fail-closed inits**. An application links only the providers its target
  ships. A reference to a provider that is not linked resolves to a weak
  definition here that returns false, so a missing provider fails closed instead
  of breaking the link. The real provider's strong definition overrides the weak
  one when it is linked.
- **`CryptoInit` selector** (weak). An explicit `Provider` selects one provider
  directly; AUTO tries hardware first, then software unless
  `CRYPTO_FLAG_NO_FALLBACK` is set. A port that needs a different policy installs
  its own strong `CryptoInit`.
- **The Cryptor instance** (below).

## Cryptor: composing engines

A Cryptor references engines and presents its own forwarding `CryptoDev_t`:

```c
Cryptor_t      ble;
static uint8_t bleMem[CRYPTO_MEMSIZE_UECC];
CryptoDev_t   *eng[] = { &g_Ecdh, &g_Aes };
CryptoCfg_t    cfg = { 0 };
cfg.ReqCaps = CRYPTO_CAP_ECDH_P256 | CRYPTO_CAP_AES128_ECB;
cfg.pMem    = bleMem; cfg.MemSize = sizeof(bleMem);
CryptorComposeInit(&ble, &cfg, eng, 2);
CryptoDev_t *h = CryptorHandle(&ble);           // pass h where a CryptoDev_t* is wanted
```

Routing picks the first composed engine whose capability bits cover the
requested operation. The presented `Cap` is the union of the engine caps, and an
operation pointer is set only where an engine covers it, so `Cap` and the
pointers agree; an uncovered operation returns `CRYPTO_STATUS_UNSUPPORTED`.
`CryptorComposeInit` fails closed when `ReqCaps` is not fully covered.

The ECDH operations forward the Cryptor's own `pMem` as the engine key context,
so two Cryptors over one engine keep separate keys. The arena holds the ECDH
private key, the only per-instance secret today; AES is stateless and ignores
it. The arena must be a plain-byte context that is valid when zeroed
(micro-ecc, and slot-handle hardware engines). An engine whose per-instance
context needs structured init (mbedTLS) is composed with `pMem` NULL: the
Cryptor then forwards NULL and that engine runs on its own initialized context
as one shared instance. That mbedTLS context is init-once: `CryptoMbedtlsInit`
must receive fresh memory that does not already hold an initialized context,
since it does not free a prior one (there is no Deinit path today).

The ECDH `pKeyCtx` holds the per-instance key context and `pOpCtx` stays free
for the operation context, so adding an async hardware engine later does not
force an interface change. Async completion correlation through a Cryptor is a
later refinement that arrives with the first async hardware engine.

## Generic engines vs subsystem-owned adapters

The engines above are generic: real, reusable facilities (software or hardware)
that any subsystem (SMP, TLS, DFU) may use. They live in the crypto layer
(`src/crypto/`, or `ARM/Nordic/src/` for a port hardware engine) and are
declared in `crypto.h`.

Some `CryptoDev_t` providers are subsystem-owned adapters, not generic engines,
and deliberately do not appear in `crypto.h`:

| Provider | Owner | Notes |
|---|---|---|
| `BtCryptoCtlrSdcInit` | Bluetooth (`ARM/Nordic/src/bt_crypto_ctlr_sdc.cpp`, declared in `bt_smp.h`) | AES-128 through the BLE controller HCI LE Encrypt. Cannot function without a running BLE controller, so only a Bluetooth consumer may use it. It implements `CryptoDev_t` so SMP composes it into its AES slot uniformly; it is not a reusable engine and is never advertised in the crypto layer. |

A non-Bluetooth consumer browsing `crypto.h` sees only facilities it can use.
The BLE-controller AES is invisible there because no non-Bluetooth code can call
it.

## Composition for SMP

`BtSmpInit(CryptoDev_t *pEcdh, CryptoDev_t *pAes)` takes two engine handles. RNG
is the platform hardware RNG through `RngGet`, used by the engines and the SMP
toolbox directly, so it is not a separate slot. Each handle is validated against
its required capability; an engine lacking it leaves that path disabled so the
operation fails loud rather than pairing under absent crypto.

- **nRF52832 (no CryptoCell):** ECDH from software uECC, AES from the BLE
  controller adapter. Two dedicated engines, no Cryptor needed:

  ```c
  CryptoUeccInit(&ecdh, &ueccCfg);     // software ECDH P-256
  BtCryptoCtlrSdcInit(&aes);           // controller AES
  BtSmpInit(&ecdh, &aes);
  ```

- **nRF52840 / nRF91 (CC310):** one mbedTLS engine covers ECDH and AES, so the
  same handle fills both slots:

  ```c
  CryptoMbedtlsInit(&m, &mCfg);
  BtSmpInit(&m, &m);
  ```

A Cryptor is for the case where several use cases must share one engine, or a
consumer wants a single handle that covers several capabilities. SMP's two-slot
`Init` takes engine handles directly, so it does not require one.

## Host-driven vs stack-owned SMP

The crypto composition applies only where IOsonata's `bt_smp.cpp` runs the SMP
state machine in-host:

- **Host-driven** (SDC; sdk-nrf-bm / nRF54 if in-host): supply crypto engines
  and fill the slots.
- **Stack-owned** (legacy nRF52 SoftDevice; STM32WB): the controller or
  co-processor firmware owns pairing. `bt_smp.cpp` is not active; `SecType` maps
  to the stack's auth-requirement config (for example
  `aci_gap_set_authentication_requirement` on STM32WB). No crypto engine is
  injected.

## Reuse beyond Bluetooth

`CryptoDev_t` is not Bluetooth-scoped. The same engines serve any consumer: TLS
over LTE/Wi-Fi (nRF91 roadmap), secure DFU, encrypted storage. mbedTLS in
particular is one library behind SMP, TLS, and DFU, wrapped once as
`CryptoMbedtlsInit` and used by many. New primitives (AES-GCM/CTR, SHA-256,
HMAC, ECDSA) are appended to the capability bits and the vtable as real
consumers need them; existing bits are never renumbered.

## File layout

| File | Role |
|---|---|
| `include/crypto/crypto.h` | Interface: `CryptoDev_t`, `CryptoCfg_t`, capability/flag bits, engine `Init` declarations, `Cryptor`, and the C++ wrappers. |
| `src/crypto/crypto.cpp` | Base layer: the Cryptor instance, weak fail-closed provider inits, and the `CryptoInit` selector. |
| `src/crypto/crypto_uecc.cpp` | Software ECDH P-256 engine (micro-ecc). |
| `src/crypto/crypto_mbedtls.cpp` | Software AES + ECDH engine (mbedTLS). |
| `ARM/Nordic/nRF54/src/crypto_psa.cpp` | Hardware `CryptoHwInit` via PSA Crypto shipped by sdk-nrf-bm (CRACEN on nRF54L). |
| `ARM/Nordic/nRF52/src/crypto_cc310.cpp` | Hardware `CryptoHwInit` via nrf_crypto over CC310 (nRF5 SDK, nRF52840). |
