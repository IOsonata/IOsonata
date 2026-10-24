# Crypto engine architecture

IOsonata's cryptographic primitives are modeled the same way as every other
device in the framework: as objects on the `Device` base. A crypto engine is a
`Device`, so it carries the same lifecycle (`Enable` / `Disable` / `Reset`) and
event model as a bus or a sensor. On top of `Device` sit the crypto interfaces, each
a small abstract interface for one family of operations.

## Crypto interfaces

`include/crypto/icrypto.h` defines the interfaces. Each is a pure interface that
states what operation is wanted, never how or where it runs:

- `CryptoEngine` : the common base, `virtual public Device`. Holds the async
  completion handler (see below).
- `CipherEngine` : symmetric cipher (`Cipher`, AES-128/256 ECB/CTR/CBC).
- `MacEngine` : keyed message authentication (`Mac`, AES-CMAC).
- `HashEngine` : unkeyed message digest (`Hash`, SHA-256).
- `KeyAgreeEngine` : key generation and key agreement (`KeyGen`, `Agree`, P-256
  ECDH).
- `SignEngine` : signature (`Sign`, `Verify`, ECDSA P-256).
- `RngEngine` : random source (`Random`).

An interface derives `virtual public CryptoEngine`, so a concrete engine that
implements several interfaces (for example a part that does both AES and ECDH)
inherits one `Device` sub-object through the virtual base rather than several.

A consumer holds an interface pointer (`KeyAgreeEngine *`, `CipherEngine *`) and calls
the interface method. It is blind to whether the work is software, a hardware
accelerator (Arm CryptoCell CC310, Silex CRACEN / BA414EP), or the BLE
controller. An operation a given engine does not implement returns
`CRYPTO_STATUS_UNSUPPORTED`; the base interface method supplies that default, so an
engine only overrides what it provides.

`Device::Enable()` is pure virtual, so every concrete engine must override
`Enable` / `Disable` / `Reset`. An engine that skips them is abstract and will
not instantiate.

## Generic crypto object

`Crypto` in `include/crypto/crypto.h` inherits the operation interfaces, with
one shared `Device` base. It does not inherit or contain complete software
engines. Generic methods provide weak software defaults; the MCU library
replaces accelerated methods with strong definitions at link time.

The application declares a normal C++ object. Its constructor initializes
software state without touching peripherals. After system startup, `Init()`
binds it to the implementation provided by the linked MCU library:

```cpp
#include "crypto/crypto.h"

Crypto g_Crypto;

bool InitApplicationCrypto(void)
{
	return g_Crypto.Init();
}
```

The same object provides Cipher, Mac, Seal/Open, Hash, Random, KeyGen/Agree and
Sign/Verify. Its public layout is identical across MCU libraries. There is no
raw object arena, placement construction, heap allocation or public factory.
The compiler handles object storage, alignment and lifetime.

The library supplies one strong `Crypto::Init` definition and strong methods
for its hardware operations. Init enables typed, library-owned hardware engines.
There is no second full `Crypto` object or runtime provider pointer. Software
methods remain selected only for operations without a target definition.

`CryptoAesModes` contains CMAC/CCM/GCM composition over a virtual AES block
operation. Complete AES accelerators derive from it without inheriting software
AES. `CryptoSoftAes` adds the software primitive and cipher modes for targets
that need them. `CryptoHmacSha256` similarly composes streaming hash operations
without retaining software SHA compression when hardware supplies every hash
method. Drivers for partial accelerators select software for the specific key
sizes or operations that hardware cannot execute.

Hardware engines are shared for the lifetime of the MCU library. Repeated
initialization does not reset an engine already in use. `Disable()` marks only
the calling application object invalid; it does not stop another object's
hardware access. `Enable()` rebinds through `Init()`. `Reset()` resets local
software RNG state and reinitializes the application object; hardware fault
recovery remains inside the shared drivers. Hardware engines remain enabled
once initialized. An application's destruction does not shut them down.

A failed hardware initialization returns false and leaves the application object
invalid. It never silently selects software. Operations on an invalid object
fail; output-producing operations clear their output where its size is known.
Initialize before querying context requirements, and finish operations before
Disable, Reset or destruction. Keys and streaming hash contexts remain in caller
storage; do not reuse contexts across implementations. Startup initialization
must be serialized before concurrent operation access begins. Copy construction and assignment are disabled.

Existing consumers accepting capability pointers can receive `&g_Crypto` through
ordinary base conversion. Software-only objects have independent RNG state.

The [MCU port audit](crypto-port-audit.md) lists hardware acceleration and required
software operations for every port. AES-128/256 ECB/CBC/CTR and P-256 import are
required generic operations. Missing hardware support selects the software
implementation inside the library; a hardware error never triggers a software
retry. AES-256 is not an optional CryptoTest capability.

This selection reflects existing IOsonata drivers. A hardware block with no
implemented driver is not advertised as accelerated. Desktop projects and the
incomplete RISC-V nRF54H20 project are not MCU integration targets here.

Specialized legacy placement factories remain available to existing library
consumers. The shared CryptoTest declares a `Crypto` object and calls `Init()`;
explicit software objects remain independent references. Link the matching MCU
library normally. The strong `Crypto::Init` reference extracts its implementation
from the archive, bringing its strong operation definitions with it. Generic
operation defaults are weak; Init itself is not. MCU builds compile with
`-ffunction-sections -fdata-sections` and link with `--gc-sections`, so replaced
software sections are discarded. A software source being compiled into the
archive does not mean its implementation remains in the executable.

CryptoTest deliberately instantiates independent software reference objects.
Use a minimal application without those references to check unwanted software
linkage. The CC3xx and STM32 accelerator runners also build such an application
and inspect its linked symbols.

## Per-instance key state

Interfaces that hold a key between calls (`KeyAgreeEngine`) take a `pKeyCtx` (a
caller-owned key-context buffer of `KeyCtxSize()` bytes). `KeyGen` writes the
private key into it and `Agree` consumes it, so one engine object serves several
independent key contexts. `KeyImport` loads a private scalar the caller was
handed and optionally returns its public key. `CryptoCc3xx` and `Ba414ep`
derive from `CryptoUecc` and inherit software import and ECDSA while overriding
hardware key generation and agreement. `CryptoWbaPka` implements every P-256
operation in hardware and derives directly from `KeyAgreeEngine` and
`SignEngine`. It keeps the exact same `CryptoUecc::KeyCtx` type and `SetRng`
entry point without retaining software P-256 construction tables. Each engine
shares one RNG binding across its methods. The engine object itself is stateless beyond its
`Device` lifecycle; the scarce hardware resource, not a per-key secret, is what
the object owns.

`CipherEngine`, `MacEngine`, and `HashEngine` take the key (if any) in the call,
so they hold no per-instance key state and one object serves any number of keys.

## Asynchronous completion

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
Every engine in the tree today is synchronous; the completion handler allows an
asynchronous engine to be added without an interface change.

## Security properties

The engine layer is where the cryptographic checks live, because the SMP
consumer is curve-blind and key-blind:

- **Peer key validation**: a P-256 engine rejects a peer public key that is not
  on the curve before the agreement, closing the invalid-curve attack
  (CVE-2018-5383). The software engine checks with `uECC_valid_public_key`; the PKA
  engines validate the point on the accelerator.
- **Single-use ephemeral key**: `Agree` wipes the shared secret on every exit
  and, by default, wipes the private key too, so a second `Agree` without a fresh
  `KeyGen` fails rather than reusing a spent key. A caller that needs one
  ephemeral pair for several concurrent peers passes `bKeepKey = true`, which
  keeps the key after a success. A failure wipes it; transient busy status
  retains the key for retry.
- **Per-instance key context**: the private key lives in caller-owned `pKeyCtx`,
  never in a file static, so concurrent or sequential key contexts never collide.
- **Strong RNG required**: P-256 key generation and signing use an injected
  `RngEngine` and require `IsSecure()`. The generic software RNG is a
  deterministic PRNG and reports false. Targets without entropy can hash,
  encrypt and verify signatures, but cannot securely generate keys or sign.
  A selected hardware RNG that fails initialization makes `Init()` fail.

## Engines

| Engine | Interfaces | Location | Notes |
|---|---|---|---|
| `CryptoUecc` | `KeyAgreeEngine`, `SignEngine` | `src/crypto/crypto_uecc.cpp` | Software P-256 (micro-ecc): ECDH and ECDSA. Uses an injected secure `RngEngine` for `KeyGen`/`Sign`; `Verify` needs no RNG. Software fallback on any part. |
| `Ba414ep` | `KeyAgreeEngine`, `SignEngine` | `src/crypto/ba414ep.cpp` | Hardware P-256 key generation/ECDH over the injected CRACEN interface. Inherits software key import and ECDSA. The Nordic interface loads PKE microcode. |
| `CryptoCc3xx` | `KeyAgreeEngine`, `SignEngine` | `ARM/src/crypto_cc3xx.cpp` | Hardware P-256 key generation/ECDH on CC3xx. Inherits software key import and ECDSA. `Cc3xxIntrf` supplies register access and arbitration. |
| `CryptoWbaPka` | `KeyAgreeEngine`, `SignEngine` | `ARM/ST/STM32WBAxx/src/crypto_stm32wba.cpp` | Hardware P-256 import, generation, agreement and ECDSA. Shares the key-context type with `CryptoUecc`, without inheriting its software methods. |
| `CryptoSoftAes` | `CipherEngine`, `MacEngine` | `src/crypto/crypto_aes_modes.cpp` | Software AES-128/256 cipher (FIPS-197) and AES-128 CMAC. CMAC/CCM/GCM use protected virtual block hooks, which hardware subclasses override. |
| `CryptoMaster` | `CipherEngine`, `MacEngine`, `AeadEngine` | `src/crypto/cryptomaster.cpp` | Hardware AES-128/256 encryption and decryption on Silex BA411e. Inherits CMAC/CCM/GCM composition from `CryptoAesModes`, without software AES. |
| `CryptoSoftSha256` | `HashEngine` | `src/crypto/crypto_softsha256.cpp` | Software SHA-256 (FIPS 180-4). Unkeyed digest; serves DFU image verification and any hashing consumer. |
| `CryptoSoftRng` | `RngEngine` | `src/crypto/crypto_softrng.cpp` | Software PRNG base. Statistical use only; not a security random source. |
| `CryptoRngNrf` | `RngEngine` | `ARM/Nordic/src/rng_nrfx.cpp` | Nordic hardware RNG (conditioned CRACEN TRNG on nRF54L/H, NRF_RNG on nRF52/network core, CC3xx on nRF5340 application/nRF91). Platform singleton `CryptoRngNrfInstance`. |
| `CryptoRngStm32` | `RngEngine` | `ARM/ST/src/rng_stm32.cpp` | STM32 hardware RNG peripheral. Platform singleton `CryptoRngStm32Instance`. |

Engine class names are provider-class, not target-specific. `CryptoCc3xx` names
the CryptoCell family; the target header pins the actual chip. The `_bm` suffix
is reserved for code that touches the sdk-nrf-bm SDK specifically, never for
generic crypto.

## Subsystem-owned adapters

Some engines are not generic facilities and deliberately live outside the crypto
layer:

| Adapter | Owner | Notes |
|---|---|---|
| `CryptoCtlrAes`, `CryptoCtlrRng`, `CryptoCtlrMac`, `CryptoCtlrAead`, `CryptoCtlrHash`, `CryptoCtlrSign` | Bluetooth (`src/bluetooth/bt_crypto_ctlr.cpp`, `BtCryptoCtlrInit`, `BtCryptoCtlrRng`, `BtCryptoCtlrMac`, `BtCryptoCtlrAead`, `BtCryptoCtlrHash`, `BtCryptoCtlrHmac` and `BtCryptoCtlrSign` declared in `bt_smp.h`) | AES-128 ECB through the HCI LE Encrypt command and random numbers through LE Rand, on any controller reached through HCI; on a controller with the I-SYST vendor commands (HciController), AES-CMAC and AES-CCM one command per message and random numbers 251 octets per command, one-shot SHA-256 and HMAC (no streaming over a link) and ECDSA P-256 sign and verify, the engines of those being null elsewhere. Cannot function without a running controller, so only a Bluetooth consumer may use them. They implement the facets so SMP and Encrypted Advertising Data compose them into their slots uniformly, but they are not reusable engines and are not advertised in the crypto layer. |

## Composition for SMP

`BtSmpInit(KeyAgreeEngine *pEcdh, CipherEngine *pAes, RngEngine *pRng)`
accepts generic capability pointers. A `Crypto` object can supply all three:
`BtSmpInit(pCrypto, pCrypto, pCrypto)`. The application does not select hardware
classes. Existing Bluetooth-owned controller adapters remain valid where a
controller supplies those operations; this change does not alter their setup.

## Host-driven vs stack-owned SMP

The engine composition applies only where IOsonata's `bt_smp.cpp` runs the SMP
state machine in-host:

- **Host-driven** (SDC; sdk-nrf-bm / nRF54 in-host): construct the engines and
  fill the slots.
- **Stack-owned** (STM32WB): the co-processor firmware owns pairing through the
  `ACI_GAP` vendor interface, with the on-die PKA handling ECDH below the API.
  `bt_smp.cpp` is not in the pairing path and no engine is injected.

## Reuse beyond Bluetooth

The interfaces are not Bluetooth-scoped. The same engines serve any consumer: secure
DFU (`CryptoSoftSha256` + `CryptoUecc::Verify` validate a signed image), and TLS
or encrypted storage as those consumers arrive. New primitives are added by
appending an interface or an algorithm value, never by renumbering existing ones.

## File layout

| File | Role |
|---|---|
| `include/crypto/icrypto.h` | Crypto interfaces: `CryptoEngine` and its operation interfaces, the algorithm and operation enums, the asynchronous completion handling, `CryptoKey`, `CryptoSecureWipe`, and the P-256 helper declarations. |
| `src/crypto/crypto_p256.cpp` | Generic P-256 math: constant-time byte helpers, scalar and field range checks, and the ladder scalar regularization (`P256RegularizeScalar`, `P256RegularBit`). Exports functions only; the curve point arithmetic stays in each engine. |
| `src/crypto/crypto_uecc.cpp` | Software P-256 engine (micro-ecc): ECDH and ECDSA. |
| `src/crypto/crypto_aes_modes.cpp` | Common AES-128 CMAC/CCM/GCM composition and the separate software cipher adapter. |
| `src/crypto/cryptomaster.cpp` | Hardware AES engine on the Silex CryptoMaster (BA411e). |
| `src/crypto/crypto_softsha256.cpp` | Software SHA-256 engine. |
| `src/crypto/crypto_softrng.cpp` | Software PRNG base (statistical). |
| `src/crypto/ba414ep.cpp` | Hardware P-256 engine on the Silex BA414EP; `CracenIntrf` provides Nordic CRACEN register access. |
| `ARM/src/crypto_cc3xx.cpp` | Hardware P-256 engine on the CC3xx PKA; `Cc3xxIntrf` provides target register access and power control. |
| `ARM/Nordic/src/rng_nrfx.cpp` | Nordic hardware RNG engine (`CryptoRngNrf`). |
| `ARM/ST/src/rng_stm32.cpp` | STM32 hardware RNG engine (`CryptoRngStm32`). |
