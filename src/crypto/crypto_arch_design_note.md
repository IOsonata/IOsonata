# IOsonata crypto architecture design note

Status: proposal for review. No code committed. This note fixes the object
model, the key object, and the engine tree, then proves the shape against three
consumers (BLE SMP, LoRaWAN, Wi-Fi). Read the worked examples last; they are the
test of whether the abstraction is right.

## 1. Decision summary

- Crypto is modelled on the IOsonata **Device** base, not DeviceIntrf. A crypto
  engine is a controllable hardware entity with a lifecycle and capability
  facets, not a byte transport.
- Each capability facet base is a **working software implementation**, not a pure
  interface. A hardware driver inherits the facet and **overrides only the
  operations it accelerates**; anything it does not override runs the software
  base. This is the AccelSensor / Timer idiom: a base with real method bodies,
  concrete drivers overriding what they need. The software base tree is, by
  itself, a complete software crypto library.
- A real block **multiply-inherits** the facets it implements, with every facet
  `virtual public CryptoEngine` so the shared base subobject is single. This is
  the Sensor : virtual public Device diamond discipline, and it is what makes a
  MAC facet's call to the cipher facet resolve to the hardware override on a
  combined block.
- **No RTTI, no dynamic allocation.** Facet presence is a compile-time base, not
  a runtime query; there is no dynamic_cast and no capability bitmask to keep in
  sync. Per-instance and operation state is caller-provided static memory, the
  way AccelSensor holds vData and CFifo takes an external buffer.
- A **CryptoKey** object carries key material or a handle plus attributes (type,
  location, usage, lifetime). Operations take a CryptoKey, never a raw pointer.
  This is what lets a plaintext software key, an accelerator-slot key, and a
  never-exportable secure-element key share one interface.
- Streaming operations (cipher, AEAD, MAC, hash) are **multi-part operation
  objects** (setup / update / finish) placement-constructed in caller storage,
  with one-shot convenience wrappers for the single-block SMP style. This carries
  Wi-Fi and storage payloads and retires the ad-hoc pOpCtx async patch.
- The existing **Cryptor** composition layer stays, holding typed facet base
  pointers and remaining the owner of a shared hardware lock. With software bases
  present, most single-engine use cases need no Cryptor at all: the engine object
  already answers every facet, in hardware where it can and software where it
  cannot.

## 2. Why Device, not DeviceIntrf

DeviceIntrf is a byte-transport bus: StartRx/RxData/StopRx, StartTx/TxData/StopTx,
retry counts, Tx-ready flags. Its whole vocabulary is moving opaque bytes across
a wire. Crypto does not move bytes across a wire; it transforms data under a key.
Every DeviceIntrf verb would be dead weight or metaphor.

Device is the right base because each of its facets maps to a real crypto need:

| Device facet                         | Crypto need it serves                                   |
| ------------------------------------ | ------------------------------------------------------- |
| Enable / Disable / PowerOff          | Wrapper power, accelerator enable, low-power gate        |
| Reset / Valid()                      | Bring-up, self-test result, fault recovery               |
| EvtHandler (DEV_EVT)                  | Async completion for offload/secure-service engines      |
| Holds a DeviceIntrf*, is not one      | Offload proxy holds the channel to the far side          |
| Virtual-inheritance capability facets | Cipher/Mac/Hash/KeyAgree/Sign/Kdf as separate interfaces |

The last two are the decisive ones. Device holds an interface (Interface(pIntrf))
rather than being one, which is exactly how a secure-domain or HCI-offload crypto
engine should hold the channel to the real engine: the operation crosses the
boundary, the key does not. And Sensor : virtual public Device, with a combined
part multiply-inheriting several sensor facets, is the precedent for a crypto
block implementing several primitive interfaces at once.

## 3. The object model

### 3.1 CryptoKey

A key is data with attributes, not a Device. It is the object operations consume,
analogous to how sensor data is not a Device.

```cpp
enum CRYPTO_KEY_TYPE {
    CRYPTO_KEY_AES_128,
    CRYPTO_KEY_AES_256,
    CRYPTO_KEY_ECC_P256,        // private scalar or public point
    CRYPTO_KEY_ECC_P384,
    CRYPTO_KEY_HMAC,            // arbitrary-length MAC key
    CRYPTO_KEY_DERIVE,         // input keying material for a KDF
};

// Where the key material lives. This is the axis PSA, TF-M, Silicon Labs and NXP
// all converged on, and it is what makes one interface serve software and
// secure-element engines.
enum CRYPTO_KEY_LOCATION {
    CRYPTO_KEY_LOC_PLAIN,      // bytes in application RAM (software / HW-accel)
    CRYPTO_KEY_LOC_SLOT,       // loaded into an accelerator key slot by index
    CRYPTO_KEY_LOC_OPAQUE,     // never leaves a secure domain; referenced by id
};

// Bitmask: what the key may be used for. Enforced by the engine, not the caller.
#define CRYPTO_KEY_USE_ENCRYPT   (1U << 0)
#define CRYPTO_KEY_USE_DECRYPT   (1U << 1)
#define CRYPTO_KEY_USE_SIGN      (1U << 2)
#define CRYPTO_KEY_USE_VERIFY    (1U << 3)
#define CRYPTO_KEY_USE_DERIVE    (1U << 4)
#define CRYPTO_KEY_USE_AGREE     (1U << 5)

struct CryptoKey {
    CRYPTO_KEY_TYPE     Type;
    CRYPTO_KEY_LOCATION Loc;
    uint32_t            Usage;     // CRYPTO_KEY_USE_*
    union {
        struct { const uint8_t *pData; size_t Len; } Plain;   // LOC_PLAIN
        uint32_t SlotIndex;                                   // LOC_SLOT
        uint32_t OpaqueId;                                    // LOC_OPAQUE
    };
};
```

A software AES key is `{AES_128, LOC_PLAIN, USE_ENCRYPT, {.Plain={p,16}}}`.
An IKG-derived nRF54 key that never reaches the CPU is
`{AES_256, LOC_OPAQUE, USE_ENCRYPT|USE_DERIVE, {.OpaqueId=id}}`. The consumer code
is identical; only the engine that accepts that location differs. Engines reject a
location or usage they cannot honour with CRYPTO_STATUS_UNSUPPORTED.

Note: LOC_OPAQUE and LOC_SLOT are defined now so the interface never has to
change to add a secure element or the nRF54 IKG/KMU path, but only LOC_PLAIN
needs an implementation in the first cut.

### 3.2 Operation status and multi-part operations

Status is unchanged from today: OK / PENDING / FAIL / UNSUPPORTED. PENDING plus
the Device EvtHandler carries offload/async engines.

Streaming primitives become operation objects so Wi-Fi/storage payloads and
async state are first class:

```cpp
class CipherOp {                       // returned by CipherEngine::CipherBegin
public:
    virtual CRYPTO_STATUS Update(const uint8_t *pIn, size_t Len, uint8_t *pOut) = 0;
    virtual CRYPTO_STATUS Finish(uint8_t *pOut, size_t *pOutLen) = 0;
    virtual ~CipherOp() {}
};
```

AeadOp adds SetAad() and, on finish, tag output (encrypt) or tag verify (decrypt).
MacOp and HashOp follow the same Update/Finish shape. Single-block SMP calls use
one-shot wrappers that open, update once, and finish internally, so the SMP core
is not forced through the streaming object.

## 4. The engine tree

The facet method bodies are software implementations. `= 0` appears nowhere in a
facet; every method runs. A hardware driver overrides the ones it accelerates.

Key idea, made concrete: a MAC facet computes CMAC by calling the cipher facet's
AES block. Because that call is a virtual method on the same object, a combined
hardware block that overrode the cipher gets hardware AES inside its inherited
software CMAC, with no extra wiring. This delegation already exists today in C
form: CryptoCmac in crypto.cpp computes the MAC over pDev->Aes128Ecb. The
refactor turns that pattern from a hand-maintained special case into ordinary
virtual dispatch.

```cpp
// Common base: lifecycle, self-test, offload channel. Inherits Device.
class CryptoEngine : virtual public Device {
public:
    virtual int  SelfTest() { return 0; }          // 0 = pass; override as needed
    // Enable/Disable/Reset/PowerOff/EvtHandler/Interface inherited from Device.
};

// --- Capability facets: each base is a WORKING SOFTWARE IMPLEMENTATION ---

class CipherEngine : virtual public CryptoEngine {
public:
    // Software AES ECB/CTR/CBC (+ XTS later). A HW driver overrides this.
    virtual CRYPTO_STATUS Cipher(int Alg, int bEncrypt, const CryptoKey &Key,
                                 const uint8_t *pIv, size_t IvLen,
                                 const uint8_t *pIn, size_t Len, uint8_t *pOut);
    virtual CipherOp *CipherBegin(int Alg, int bEncrypt, const CryptoKey &Key,
                                  const uint8_t *pIv, size_t IvLen, void *pStore);
};

class MacEngine : virtual public CryptoEngine {
public:
    // Software CMAC/HMAC/GMAC. CMAC/GMAC call CipherEngine::Cipher (virtual), so
    // they use HW AES automatically on a combined block. A HW driver with a
    // native MAC overrides this; otherwise the software body runs.
    virtual CRYPTO_STATUS Mac(int Alg, const CryptoKey &Key,
                              const uint8_t *pMsg, size_t Len,
                              uint8_t *pMac, size_t MacLen);
    virtual MacOp *MacBegin(int Alg, const CryptoKey &Key, void *pStore);
};

class AeadEngine : virtual public CryptoEngine {
public:
    // Software CCM/GCM over the cipher facet; ChaCha20-Poly1305 later. HW override
    // where a native AEAD exists.
    virtual CRYPTO_STATUS Aead(int Alg, int bEncrypt, const CryptoKey &Key,
                               const uint8_t *pNonce, size_t NonceLen,
                               const uint8_t *pAad, size_t AadLen,
                               const uint8_t *pIn, size_t Len, uint8_t *pOut,
                               uint8_t *pTag, size_t TagLen);
    virtual AeadOp *AeadBegin(int Alg, int bEncrypt, const CryptoKey &Key,
                              const uint8_t *pNonce, size_t NonceLen, void *pStore);
};

class HashEngine : virtual public CryptoEngine {
public:
    // Software SHA-1/256/384/512. HW override where a hash engine exists.
    virtual CRYPTO_STATUS Hash(int Alg, const uint8_t *pMsg, size_t Len,
                               uint8_t *pDigest, size_t DigestLen);
    virtual HashOp *HashBegin(int Alg, void *pStore);
};

class KeyAgreeEngine : virtual public CryptoEngine {
public:
    // Software P-256/384 over crypto_p256 helpers. HW override drives the PKA.
    virtual CRYPTO_STATUS KeyGen(int Curve, void *pKeyCtx, uint8_t *pPubKey);
    virtual CRYPTO_STATUS Agree(int Curve, void *pKeyCtx,
                                const uint8_t *pPeerPubKey, uint8_t *pSharedX);
};

class SignEngine : virtual public CryptoEngine {
public:
    // Software ECDSA. HW override drives the PKA.
    virtual CRYPTO_STATUS Sign(int Curve, const CryptoKey &Key,
                               const uint8_t *pHash, size_t HashLen, uint8_t *pSig);
    virtual CRYPTO_STATUS Verify(int Curve, const uint8_t *pPubKey,
                                 const uint8_t *pHash, size_t HashLen,
                                 const uint8_t *pSig);
};

class KdfEngine : virtual public CryptoEngine {
public:
    // Software HKDF/PBKDF2/WPA-PRF/LoRa-block, built over the mac and cipher
    // facets. Rarely a HW override; the value is the software algorithm.
    virtual CRYPTO_STATUS Derive(int Alg, const CryptoKey &InKey,
                                 const uint8_t *pInfo, size_t InfoLen,
                                 const uint8_t *pSalt, size_t SaltLen,
                                 uint8_t *pOut, size_t OutLen);
};

class RngEngine : virtual public CryptoEngine {
public:
    // Base is a software PRNG (deterministic, no security claim) usable for
    // statistical/non-security needs. A target MCU with a hardware entropy
    // source overrides this with a DRBG. No hardware entropy means no DRBG:
    // the derived class simply is not instantiated and the PRNG base remains.
    // Security key generation must use the DRBG override, never the PRNG base.
    virtual CRYPTO_STATUS Random(uint8_t *pOut, size_t Len);
};
```

The base tree above is a complete software crypto library on its own. Hardware
drivers multiply-inherit their facets and override only the accelerated paths,
exactly like a combined sensor part inherits several sensor interfaces and
implements the ones its silicon provides:

```cpp
// Silex CryptoMaster (BA411e AES + BA413 hash). Overrides Cipher and Hash; the
// inherited software CMAC/CCM/GCM then run over the HW AES via Cipher, unless a
// native MAC/AEAD path is also overridden.
class CryptoMaster : public CipherEngine, public AeadEngine,
                     public MacEngine,   public HashEngine {
    CRYPTO_STATUS Cipher(...) override;   // BA411e AES in HW
    CRYPTO_STATUS Hash(...)   override;   // BA413 hash in HW
    // Mac/Aead not overridden -> inherited software CMAC/CCM run over HW Cipher.
};

// Silex BA414EP public-key core: overrides the PK facets.
class Ba414ep : public KeyAgreeEngine, public SignEngine {
    CRYPTO_STATUS KeyGen(...) override;   // BA414EP point mult
    CRYPTO_STATUS Agree(...)  override;
    CRYPTO_STATUS Sign(...)   override;
    CRYPTO_STATUS Verify(...) override;
};

// A pure software engine is just the facet bases with nothing overridden:
class CryptoSoft : public CipherEngine, public MacEngine, public AeadEngine,
                   public HashEngine,  public KeyAgreeEngine, public SignEngine,
                   public KdfEngine,   public RngEngine {};
```

virtual public on every facet keeps the single CryptoEngine (and Device)
subobject, so this->Cipher inside inherited software CMAC resolves to CryptoMaster's
hardware override. This is the Sensor : virtual public Device discipline.

## 4a. Where the software bodies live (own vs delegate)

Rule: a body delegates to a free function when that function is useful to other
callers; otherwise the body lives in the facet. Applied to what exists today:

- **KeyAgree / Sign -> DELEGATE** to crypto_p256.cpp. Those P256* helpers are
  already free functions reused by both the CC3xx and CRACEN engines, so the
  software facet bodies call them rather than re-implement.
- **Mac / Aead (CMAC, CCM, GCM) -> DELEGATE** to the existing CryptoCmac /
  CryptoCcm* / CryptoGcm* free functions in crypto.cpp. They are already exported
  and already dispatch AES through a virtual/pointer indirection, so they port
  directly to calling the Cipher facet.
- **Cipher (software AES block) and Hash (software SHA-256) -> OWN BODY.** These
  live today as static functions inside crypto.cpp (Sha256Block/Init/Update/Final,
  and the software AES core), reachable by nothing outside that file. There is no
  other caller, so they belong in the CipherEngine / HashEngine bodies rather than
  being promoted to exported free functions. If a future consumer needs a bare
  SHA-256, promote it then.
- **isha256.c stays separate and unshared** (DFU download-key checking, global
  state, hex output). It is not the HashEngine base and must not be merged.

## 5. Composition: Cryptor and the target port

With software bases present, a single engine object already answers every facet,
so many use cases need no Cryptor: pass the engine (for example a CryptoSoft, or a
CryptoMaster) straight to the consumer and it gets hardware where the silicon has
it and software everywhere else.

Cryptor remains for the case that motivated it: composing two separate hardware
cores into one handle. On nRF54 the symmetric facets are on CryptoMaster and the
public-key facets are on Ba414ep; a Cryptor holds a typed pointer per facet
(CipherEngine*, MacEngine*, KeyAgreeEngine*, ...) and forwards. Routing is by
typed base pointer set at init, not dynamic_cast and not a capability bitmask, so
there is no RTTI and no Cap/dispatch drift. Cryptor also owns the shared hardware
lock when its engines share one block.

The nRF54 target port composes the two Silex cores into one usable engine set and
owns the shared CRACEN wrapper lock (the block is shared with the RNG):

```cpp
// Target port: base addresses, wrapper power, shared lock, microcode.
// Produces a CryptoMaster and a Ba414ep bound to this SoC, then a Cryptor
// that presents Cipher+Aead+Mac+Hash+KeyAgree+Sign to consumers.
```

The generic CryptoMaster / Ba414ep classes carry the Silex register model and are
target-independent; the nRF54 port supplies addresses, power, lock and microcode.
This is the same generic-core / vendor-binding split already agreed, now expressed
as classes rather than a C vtable plus a struct of pointers.

## 6. Worked examples: the same tree serving three consumers

### 6.1 BLE SMP LE Secure Connections (today's use case)

Needs: AES-128-ECB (toolbox e), CMAC (f4/f5/f6), P-256 ECDH.

```cpp
// SMP holds a Cryptor presenting Cipher + Mac + KeyAgree.
CryptoKey tk{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_ENCRYPT,
              {.Plain={t, 16}} };

// f5 derives the MacKey/LTK: AES-CMAC-based. One Mac facet call.
crypto.Mac(CRYPTO_MAC_CMAC, tk, in, in_len, mac, 16);

// DHKey: KeyAgree facet, private scalar kept in the engine's KeyCtx.
crypto.KeyGen(CRYPTO_CURVE_P256, keyCtx, localPub);      // on connect
crypto.Agree(CRYPTO_CURVE_P256, keyCtx, peerPub, dhKey); // on peer key
```

Facets exercised: CipherEngine, MacEngine, KeyAgreeEngine. Same set on CryptoMaster
+ BA414EP (hardware) or CryptoSoftAes + CryptoUecc (software), no consumer change.

### 6.2 LoRaWAN 1.0.x end device

Needs: AES-128-ECB (session-key derivation), AES-CMAC (frame MIC), AES-CTR
(payload). Root key AppKey derives NwkSKey/AppSKey.

```cpp
// Session key derivation is a raw AES-ECB block of a fixed input under AppKey.
CryptoKey appKey{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
                  CRYPTO_KEY_USE_DERIVE, {.Plain={app, 16}} };
crypto.Derive(CRYPTO_KDF_LORA_BLOCK, appKey, deriveInput, 16, nullptr, 0,
              nwkSKey, 16);                  // then again for AppSKey

// Frame MIC: AES-CMAC over B0||MHDR||... under NwkSKey.
CryptoKey nwk{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
               CRYPTO_KEY_USE_SIGN, {.Plain={nwkSKey, 16}} };
crypto.Mac(CRYPTO_MAC_CMAC, nwk, micInput, micLen, mic, 4);

// Payload: AES-CTR under AppSKey.
CryptoKey app{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
               CRYPTO_KEY_USE_ENCRYPT, {.Plain={appSKey, 16}} };
crypto.Cipher(CRYPTO_CIPHER_CTR, 1, app, a_i, 16, payload, len, out);
```

Facets exercised: KdfEngine, MacEngine, CipherEngine. Note the KdfEngine derives a
CryptoKey rather than the caller hand-rolling the block, and the same MacEngine
(CMAC) serves both SMP and LoRaWAN. No new interface was needed for LoRaWAN.

### 6.3 Wi-Fi WPA2/WPA3-Personal PTK derivation and frame protection

Needs: PBKDF2 (PSK from passphrase), HMAC-SHA (PRF for PTK), AES-CCM/GCM (CCMP/
GCMP frames), and for WPA3 SAE: P-256/384 EC math plus HMAC.

```cpp
// PSK from passphrase: PBKDF2(HMAC-SHA1, passphrase, SSID, 4096) -> 256-bit PSK.
CryptoKey pass{ CRYPTO_KEY_DERIVE, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_DERIVE,
                {.Plain={(const uint8_t*)passphrase, plen}} };
crypto.Derive(CRYPTO_KDF_PBKDF2_SHA1, pass, ssid, ssidLen, nullptr, 4096, psk, 32);

// PTK from PMK via the WPA PRF (HMAC-SHA). MacEngine HMAC in a loop, or a
// dedicated PRF KDF alg. TK is the last 128 bits of the PTK.
crypto.Derive(CRYPTO_KDF_WPA_PRF, pmkKey, prfLabelAndData, len, nullptr, 0, ptk, 48);

// Frame confidentiality + integrity: AES-CCM (CCMP) under TK.
CryptoKey tk{ CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
              CRYPTO_KEY_USE_ENCRYPT, {.Plain={ptk + 32, 16}} };
crypto.Aead(CRYPTO_AEAD_CCM, 1, tk, nonce, 13, aad, aadLen,
            frame, frameLen, out, mic, 8);
```

Facets exercised: KdfEngine (PBKDF2 and PRF), MacEngine (HMAC), AeadEngine (CCM/
GCM). WPA3 SAE adds KeyAgreeEngine (the finite-field/EC Dragonfly exchange) and
more HMAC; both facets already exist. WPA3-192 raises AES to 256, SHA to 384, and
the curve to P-384; all three are the same facets with a different Alg/type
parameter, which is why the interfaces are parameterized by algorithm rather than
baked to 128/P-256.

## 7. What the three examples prove

- One primitive set, small: Cipher, Aead, Mac, Hash, KeyAgree, Sign, Kdf, Rng.
  No consumer needed an interface outside this set.
- The same facet serves multiple consumers (CMAC in SMP and LoRaWAN; CCM in Wi-Fi
  and BLE CCM links; HMAC in Wi-Fi and any HKDF user).
- Key derivation is first class and pays off immediately for LoRaWAN and Wi-Fi;
  SMP simply does not use KdfEngine.
- Algorithm and key size are parameters, not method names, so 128/256, P-256/384
  and the SHA family widen without touching the interface.
- The CryptoKey location axis is exercised by LOC_PLAIN throughout, with
  LOC_OPAQUE/LOC_SLOT reserved for the secure-element and nRF54 IKG paths so those
  land later without an interface change.

## 8. Scope for the first implementation

Implement only what BLE needs, on the full tree, so the shape is proven before
breadth is added:

- CryptoEngine base + CipherEngine, MacEngine, KeyAgreeEngine facets.
- CryptoKey with LOC_PLAIN only.
- CryptoMaster (Cipher via BA411e, Mac via CMAC-over-ECB) and Ba414ep (KeyAgree
  P-256) for nRF54; CryptoSoftAes + CryptoUecc software fallbacks.
- One-shot Cipher/Mac; streaming CipherOp/AeadOp/MacOp declared but implemented as
  the breadth work for LoRaWAN/Wi-Fi lands.
- Cryptor routing by facet.

Deferred, interface already shaped for them: AeadEngine (CCM/GCM), HashEngine,
KdfEngine (PBKDF2/HKDF/WPA-PRF/LoRa-block), SignEngine, RngEngine as a facet,
LOC_SLOT/LOC_OPAQUE keys.

## 9. Resolved decisions

Resolved by the model choices:

- **Virtual base**: yes. Every facet is virtual public CryptoEngine, matching
  Sensor : virtual public Device, so a combined block has one shared subobject and
  inherited software algorithms call into the hardware overrides.
- **Dispatch**: typed base pointers, no RTTI, no dynamic_cast, no capability
  bitmask. Presence is a compile-time base.
- **Allocation**: none. Per-instance key context and operation objects are
  caller-provided static storage (the pStore arguments above), like AccelSensor's
  vData and CFifo's external buffer. Operation objects are placement-constructed
  in caller storage.
- **Body location**: per-function reuse test in section 4a. Delegate P-256 and
  the existing CMAC/CCM/GCM free functions; own the software AES block and
  SHA-256 bodies until an outside caller appears.

- **CryptoKey shape: inline union (plain / slot / opaque), not a KeyStore.**
  A KeyStore is a key lifecycle manager; it earns its complexity only when keys
  outlive a single operation and are referenced by id across calls (persistent
  secure storage, or a secure element that owns the material). No near-term
  consumer needs that: BLE, LoRaWAN and Wi-Fi all use plaintext keys in RAM. The
  union already reserves LOC_SLOT/LOC_OPAQUE, so adding a secure element later
  needs a new engine that accepts those locations, not an interface change. A
  KeyStore, if ever wanted, layers on top as the producer of LOC_OPAQUE keys
  without touching any facet signature.

- **RNG: an RngEngine facet whose base is a software PRNG, overridden by a
  hardware DRBG on MCUs that have an entropy source.** RngEngine joins the object
  model like every other facet, so a consumer cannot tell the source apart at the
  call site. The honest split: software with no entropy can only be a PRNG
  (deterministic, no security claim), fine for statistical use; a DRBG is a real
  security generator only because the silicon supplies entropy, so it lives in the
  target-MCU derived class that overrides the PRNG base. No hardware entropy means
  no DRBG: the derived class is not instantiated and the PRNG base stands. The old
  RngGet was a Nordic-usage wrapper and is retired into this facet under standard
  IOsonata naming; current direct callers (crypto_p256, crypto_uecc, crypto_mbedtls)
  move to the RngEngine facet. Security-critical generation (P-256 key generation
  and the countermeasure factors) must resolve to the DRBG override; on an MCU
  with no entropy source that operation must fail rather than fall back to the
  PRNG base, since a PRNG-generated private scalar is a broken key.

- **Operation storage: a CRYPTO_*_OP_MEMSIZE macro with a caller-provided arena,
  and no arena at all for one-shot calls.** Streaming ops (the *Begin path for
  Wi-Fi frames and storage) take void *pStore sized by a compile-time macro, the
  CFIFO_MEMSIZE idiom, and are placement-constructed into it: allocation-free and
  each caller sizes for the op it runs, not a worst-case struct. One-shot
  SMP-style calls (single block, run to completion in-call) take no pStore and use
  stack-local state, so a BLE consumer never handles an op arena.

Nothing remains open. Next step is headers implementing the BLE subset on the full
tree (section 8).
