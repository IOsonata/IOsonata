/**-------------------------------------------------------------------------
@file	bt_smp_crypto.h

@brief	SMP crypto provider interface.

bt_smp.cpp needs exactly two cryptographic primitives that it cannot do
itself in a zero-overhead embedded subset:

  - AES-128 single block ECB encrypt (the SMP toolbox c1/s1/f4/f5/f6/g2
    and AES-CMAC are all built on it)
  - P-256 ECDH: a local key pair and the DH shared secret (LE Secure
    Connections)

These are declared here as weak C hooks. The generic bt_smp.cpp links
against the hooks only; it never names a provider. Each target links in
exactly one provider implementation file, which overrides the weak
default:

  bt_smp_crypto_sdc.cpp     - controller offload (LE Encrypt + LE Generate
                              DHKey / LE Read Local P-256). Default on the
                              SDC / nrfxlib path. No extra library.
  bt_smp_crypto_mbedtls.cpp - mbedTLS (mbedtls_aes + mbedtls_ecdh). Portable
                              fallback for controllers without the crypto
                              HCI commands, and the natural choice where
                              nRF Connect SDK already pulls in nrf_security /
                              PSA over CC3xx (nRF52840, nRF54).

The AES hook is synchronous by contract: callers (AES-CMAC, the f-funcs)
treat it as a pure function. A provider whose backend is asynchronous must
serialise internally before returning.

The ECDH hooks are split so the controller-offload provider can map them
onto the two-step HCI exchange (Read Local P-256 Public Key, then Generate
DHKey on the peer key) while a software provider does both locally.

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
 * @brief	AES-128 single block ECB encrypt. Synchronous.
 *
 * Out = AES-128(Key, In). All buffers are 16 bytes, big-endian as the SMP
 * toolbox specifies (the providers handle any controller byte-order quirk).
 *
 * Weak default zeros Out so a target with no provider linked fails the
 * confirm check loudly instead of pairing under a null cipher.
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
