/**-------------------------------------------------------------------------
@file	bt_smp_crypto.h

@brief	SMP crypto provider interface.

bt_smp.cpp needs exactly two cryptographic primitives that it cannot do
itself in a zero-overhead embedded subset:

  - AES-128 single block ECB encrypt (the SMP toolbox c1/s1/f4/f5/f6/g2
    and AES-CMAC are all built on it)
  - P-256 ECDH: a local key pair and the DH shared secret (LE Secure
    Connections)

These are exposed two ways: as a BtSmpCrypto_t vtable (a provider), and as
five C wrapper functions (BtSmpCryptoAes128/P256KeyGen/Ecdh/Rand/SelfTest)
that dispatch through the active provider. bt_smp.cpp calls the wrappers;
the application chooses a provider at init.

The crypto backend is a runtime-selectable FEATURE, not a build-time arch
choice. Every provider is compiled into the lib; the application installs
exactly one via BtSmpInit() (forwarded from BtAppCfg_t.pSmpCrypto), so the
IOsonata lib never needs recompiling to switch backend. This follows the
generic/targeted pattern used across the stack (bt_xx.cpp + bt_xx_<plat>.cpp):

  bt_smp.cpp        - generic SMP state machine + provider dispatch.
  bt_smp_sdc.cpp    - SDC target: controller LE Encrypt/LE Rand + uECC ECDH.
                      Exports g_BtSmpCryptoSdc.
  bt_smp_mbedtls.cpp- portable software provider (mbedtls_aes + mbedtls_ecdh).
                      Exports g_BtSmpCryptoMbedtls. The natural choice where
                      NCS links nrf_security / PSA over CC3xx (nRF52840,
                      nRF54), so the same source uses hardware where present.
  bt_smp_nrf52.cpp  - legacy SoftDevice target (provisioned).
  bt_smp_bm.cpp     - sdk-nrf-bm / nRF54 target (provisioned).
  bt_smp_stm32wba.cpp - STM32WB target (provisioned).

The AES hook is synchronous by contract: callers (AES-CMAC, the f-funcs)
treat it as a pure function. A provider whose backend is asynchronous must
serialise internally before returning.

The ECDH hooks are split so the controller-offload provider can map them
onto the two-step HCI exchange (Read Local P-256 Public Key, then Generate
DHKey on the peer key) while a software provider does both locally. A
provider that offloads to the controller returns BT_SMP_CRYPTO_PENDING and
completes via BtSmpLocalPubKeyReady / BtSmpDhKeyReady.

@author	Hoang Nguyen Hoan
@date	May 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __BT_SMP_CRYPTO_H__
#define __BT_SMP_CRYPTO_H__

#include <stdint.h>
#include <stddef.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "bluetooth/bt_hci.h"

/** @addtogroup Bluetooth
  * @{
  */

// Return codes for the ECDH hooks. SC pairing only proceeds on BT_SMP_CRYPTO_OK;
// BT_SMP_CRYPTO_PENDING tells bt_smp.cpp the result will arrive later through
// the controller event path (BtSmpLocalPubKeyReady / BtSmpDhKeyReady), so it
// should park the state machine rather than fail.
#define BT_SMP_CRYPTO_OK			0
#define BT_SMP_CRYPTO_PENDING		1
#define BT_SMP_CRYPTO_FAIL			(-1)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	SMP crypto provider (vtable).
 *
 * The crypto backend is a runtime-selectable FEATURE, not a build-time arch
 * choice: every provider is compiled into the lib as a const instance, and
 * the application installs exactly one with BtSmpInit() - no recompile of the
 * IOsonata lib to switch. This mirrors the generic/targeted split used by the
 * rest of the stack: bt_smp.cpp is the generic layer; each platform supplies
 * a targeted bt_smp_<plat>.cpp exporting its g_BtSmpCrypto<Plat> instance
 * (bt_smp_sdc.cpp -> g_BtSmpCryptoSdc, etc.), plus the portable software
 * provider bt_smp_mbedtls.cpp -> g_BtSmpCryptoMbedtls usable on any target.
 *
 * Function-pointer contract matches the free-function wrappers below.
 */
typedef struct __Bt_Smp_Crypto {
	const char *pName;		//!< Provider name, for trace/self-test ("sdc", "mbedtls", ...)

	//! AES-128 single block ECB encrypt (synchronous). See BtSmpCryptoAes128.
	void (*Aes128)(BtHciDevice_t * const pDev,
				   const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]);

	//! Generate local P-256 key pair, return public key. See BtSmpCryptoP256KeyGen.
	int (*P256KeyGen)(BtHciDevice_t * const pDev, uint8_t pPubKey[64]);

	//! ECDH shared secret from peer public key. See BtSmpCryptoEcdh.
	int (*Ecdh)(BtHciDevice_t * const pDev,
				const uint8_t pPeerPubKey[64], uint8_t pDhKey[32]);

	//! Cryptographically strong random bytes. See BtSmpCryptoRand.
	void (*Rand)(uint8_t *pBuf, size_t Len);

	//! Optional known-answer self-test, 0 = PASS. May be NULL. See BtSmpCryptoSelfTest.
	int (*SelfTest)(void);
} BtSmpCrypto_t;

/**
 * @brief	Install the active SMP crypto provider.
 *
 * Called once at init (from BtAppInit, which forwards BtAppCfg_t.pSmpCrypto).
 * Passing NULL installs a fail-loud default whose AES zeros its output so the
 * confirm check fails rather than pairing under a null cipher. The five
 * BtSmpCrypto* wrappers below dispatch through whatever is installed here.
 *
 * @param	pCrypto		Provider instance, e.g. &g_BtSmpCryptoSdc. NULL =
 *						fail-loud default.
 */
void BtSmpInit(const BtSmpCrypto_t *pCrypto);

/**
 * @brief	The currently installed provider (never NULL; the fail-loud
 *			default before BtSmpInit). Mainly for trace and self-test.
 */
const BtSmpCrypto_t * BtSmpCryptoActive(void);

//-----------------------------------------------------------------------------
// Provider instances. Each targeted bt_smp_<plat>.cpp / bt_smp_mbedtls.cpp
// exports its instance; the application points BtAppCfg_t.pSmpCrypto at the
// one its target provides. Declared here so the app can name them without
// pulling in platform headers. A given lib defines only the instances its
// platform compiles; referencing one not built is a link error by design.
//-----------------------------------------------------------------------------
extern const BtSmpCrypto_t g_BtSmpCryptoSdc;		//!< SDC: controller AES/rand + uECC ECDH
extern const BtSmpCrypto_t g_BtSmpCryptoMbedtls;	//!< Portable software (HW-accel via platform mbedTLS)

/**
 * @brief	AES-128 single block ECB encrypt. Synchronous.
 *
 * Out = AES-128(Key, In). All buffers are 16 bytes, big-endian as the SMP
 * toolbox specifies (the providers handle any controller byte-order quirk).
 *
 * Dispatches through the active provider (BtSmpInit). The default provider
 * zeros Out so a target with no provider installed fails the confirm check
 * loudly instead of pairing under a null cipher.
 *
 * @param	pDev	HCI device (needed by the controller-offload provider;
 *					ignored by software providers).
 * @param	Key		16-byte key.
 * @param	In		16-byte plaintext block.
 * @param	Out		16-byte ciphertext block out.
 */
void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
					   const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]);

/**
 * @brief	Produce the local P-256 public key for SC pairing.
 *
 * On success (BT_SMP_CRYPTO_OK) the 64-byte public key (X||Y, big-endian)
 * is written to pPubKey and the matching private key is retained internally
 * by the provider for the subsequent ECDH call. On BT_SMP_CRYPTO_PENDING the
 * provider has asked the controller; the result arrives via
 * BtSmpLocalPubKeyReady and pPubKey is not yet valid.
 *
 * @param	pDev		HCI device.
 * @param	pPubKey		64-byte buffer for X||Y (valid only on OK).
 *
 * @return	BT_SMP_CRYPTO_OK | BT_SMP_CRYPTO_PENDING | BT_SMP_CRYPTO_FAIL.
 */
int BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64]);

/**
 * @brief	Compute the ECDH shared secret (DHKey) from the peer public key.
 *
 * Uses the private key generated by the preceding BtSmpCryptoP256KeyGen.
 * On OK, the 32-byte DHKey (big-endian) is written to pDhKey. On PENDING the
 * provider issued LE Generate DHKey; the result arrives via BtSmpDhKeyReady.
 *
 * @param	pDev		HCI device.
 * @param	pPeerPubKey	64-byte peer public key X||Y.
 * @param	pDhKey		32-byte DHKey out (valid only on OK).
 *
 * @return	BT_SMP_CRYPTO_OK | BT_SMP_CRYPTO_PENDING | BT_SMP_CRYPTO_FAIL.
 */
int BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
					const uint8_t pPeerPubKey[64], uint8_t pDhKey[32]);

/**
 * @brief	Fill a buffer with cryptographically strong random bytes.
 *			Weak default is deterministic test filler; every target overrides
 *			it with its TRNG (RngGet on nRF, controller LE Rand, etc.).
 */
void BtSmpCryptoRand(uint8_t *pBuf, size_t Len);

/**
 * @brief	Optional provider self-test. Runs a known-answer P-256 DH vector
 *			through the provider's ECDH. Returns 0 on PASS, nonzero on FAIL.
 *			A weak default returns 0 (no-op) for providers that do not
 *			implement it.
 */
int BtSmpCryptoSelfTest(void);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_SMP_CRYPTO_H__
