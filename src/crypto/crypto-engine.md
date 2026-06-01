# Crypto engine architecture

IOsonata's cryptographic primitives are abstracted the same way as the
device-communication bus (`DeviceIntrf` / `DevIntrf_t`): a thin behavioral
interface that says *what* operation is wanted, never *how* or *where* it runs.

## The interface

`include/crypto/crypto.h` defines `CryptoDev_t` — a C struct of function
pointers (canonical form, like `DevIntrf_t`), plus capability bits, async-
tolerant status codes, and a completion callback. A consumer holds a
`CryptoDev_t*` and calls the `Crypto*` wrappers; it is blind to whether the
work is software, a hardware accelerator (Arm CryptoCell CC310/CC312, STM32
PKA/CRYP), or forwarded across a TrustZone boundary to a secure-domain engine.

Three properties make it span "any surface, no matter MCU, platform, secure or
not":

- **Capability query** (`CryptoIsCapable`): an engine advertises only what it
  implements. An unsupported operation returns `CRYPTO_STATUS_UNSUPPORTED`,
  never undefined behavior. This is the analog of querying a bus's rate.
- **Async tolerance**: an operation may return `CRYPTO_STATUS_PENDING` and
  complete later via the event callback, so a controller-offload or secure-
  service backend fits the same interface as a direct in-call engine.
- **Composability**: an engine is a `CryptoDev_t`; a secure-domain proxy is
  *also* a `CryptoDev_t` that holds, in its private data, a reference to the
  real engine on the far side and forwards each operation — exactly as SLIP
  composes onto a UART `DeviceIntrf` without the consumer knowing. Keys that
  must not leave a secure domain never cross the interface: the *operation*
  crosses, not the key.

## Engines

Each engine is a `CryptoDev_t` *implementation*: the library provides the
operation functions and an `Init` that populates a **caller-owned** instance.
The application owns the `CryptoDev_t` object, brings it up with the engine's
`Init`, and injects its pointer into a subsystem - exactly as the App owns an
`SPI`/`I2C` object, calls `Init(cfg)`, and passes it to a sensor. The library
never defines the instance.

```c
CryptoDev_t g_Ecdh;          // App-owned
CryptoUeccInit(&g_Ecdh);     // library configures it
BtSmpInit(&g_Ecdh, &g_Aes, &g_Rng);
```

Each engine's `Init` is compiled per platform: where the dependency headers
exist it brings up the real engine; where they do not, a stub `Init` returns
false (so the App can probe engines at runtime without link errors).

| Engine Init | Capabilities | Location | Notes |
|---|---|---|---|
| `CryptoUeccInit` | ECDH P-256 | `src/crypto/crypto_uecc.cpp` | Software (micro-ecc). ECDH only. |
| `CryptoMbedtlsInit` | AES + ECDH + RNG | `src/crypto/crypto_mbedtls.cpp` | Software; HW-accelerated where the platform's mbedTLS sits over CC3xx/CRACEN/PKA (nRF52840, nRF54, nRF91). Also serves TLS/DFU. |
| `CryptoRngHwInit` | RNG | `ARM/Nordic/src/crypto_rng.cpp` | Hardware RNG peripheral via `RngGet`. Independent of any radio. |
| (future) `CryptoCc3xxInit` | per CC310/CC312 | per-MCU lib (`IOsonata_nRF52840.a`, `nRF9151.a`) | MCU-conditioned hardware engine. |
| (future) `CryptoStm32Init` | per PKA/CRYP | `ARM/ST/...` | Native STM32 hardware engine, when STM32 TLS is implemented. |

### Generic engines vs. subsystem-owned adapters

The engines above are **generic**: real, reusable crypto facilities (software or
hardware) that any subsystem - SMP, TLS, DFU - may use. They live in the crypto
layer (`src/crypto/`, or `ARM/Nordic/src/` for the Nordic hardware RNG) and are
declared in `crypto.h`.

Some `CryptoDev_t` providers are **subsystem-owned adapters**, NOT generic
engines, and deliberately do not appear in `crypto.h`:

| Provider | Owner | Notes |
|---|---|---|
| `BtCryptoCtlrSdcInit` | Bluetooth (`ARM/Nordic/src/bt_crypto_ctlr_sdc.cpp`, declared in `bt_smp.h`) | AES-128 + RNG via the BLE controller's HCI LE Encrypt / LE Rand. Cannot function without a running BLE controller, so only a Bluetooth consumer (SMP) may use it. It implements `CryptoDev_t` purely so SMP composes it into its AES slot uniformly - it is not a reusable crypto engine and is never advertised in the crypto layer. |

The distinction matters: a non-Bluetooth consumer browsing `crypto.h` sees only
facilities it can actually use. The BLE-controller AES is invisible there
because no non-Bluetooth code can call it.

## Composition (the IMU model)

A consumer needs a set of primitives. Like `Imu::Init(Cfg, pAccel, pGyro,
pMag)` - where one chip may supply all three sensors or several chips combine -
a crypto consumer is composed from whatever engines its target provides, each
an App-owned instance.

Bluetooth SMP needs ECDH + AES + RNG, so `BtSmpInit(pEcdh, pAes, pRng)` takes
three slots. The application brings up the engines and fills the slots:

- **nRF52832 (no CryptoCell):** `CryptoUeccInit` + `BtCryptoCtlrSdcInit` +
  `CryptoRngHwInit`, then `BtSmpInit(&ecdh, &aes, &rng)` - ECDH from software
  uECC, AES from the BLE controller, RNG from the hardware peripheral.
- **nRF52840 / nRF91 (CC310):** one `CryptoMbedtlsInit` instance fills all three
  slots: `BtSmpInit(&m, &m, &m)`.

Each slot is validated against the required capability at init; an engine
lacking it leaves the slot disabled so the operation fails loud rather than
pairing under absent crypto.

## Host-driven vs stack-owned SMP

The crypto composition applies only where IOsonata's `bt_smp.cpp` runs the SMP
state machine in-host:

- **Host-driven** (SDC; sdk-nrf-bm/nRF54 if in-host): supply crypto engines and
  fill the slots.
- **Stack-owned** (legacy nRF52 SoftDevice; STM32WB): the controller/co-processor
  firmware owns pairing. `bt_smp.cpp` is not active; `SecType` maps to the
  stack's auth-requirement config (e.g. `aci_gap_set_authentication_requirement`
  on STM32WB) instead. No crypto engine is injected.

## Reuse beyond Bluetooth

`CryptoDev_t` is deliberately not Bluetooth-scoped. The same engines serve any
consumer: TLS over LTE/Wi-Fi (nRF91 roadmap), secure DFU, encrypted storage.
mbedTLS in particular is one library behind SMP, TLS, and DFU — wrapped once as
`CryptoMbedtlsInit`, used by many. New primitives (AES-GCM/CTR, SHA-256, HMAC,
ECDSA) are appended to the capability bits and the vtable as real consumers
need them; existing bits are never renumbered.
