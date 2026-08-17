/**-------------------------------------------------------------------------
@file	bt_ead.h

@brief	Encrypted Advertising Data

Core 5.4 Vol 3 Part C 10.10 and Core Specification Supplement Part A 1.23.
Advertising data is wrapped in an Encrypted Data AD structure whose payload
and MIC are protected with AES-CCM under a session key and initialization
vector the two devices already share, and a randomizer that changes whenever
the payload or the random device address changes.

The point is unlinkability: because the randomizer changes with the address,
the bytes on air differ before and after a change, so a scanner cannot follow
a device by the shape of its advertising data the way it could when only the
address rotated.

@author	Hoang Nguyen Hoan
@date	Aug. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#ifndef __BT_EAD_H__
#define __BT_EAD_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "crypto/icrypto.h"

/// Encrypted Data AD type. Taken from the sample data of CSS Part A 2.3,
/// where the advertising data of both sets reads 1E 31 followed by the AD
/// structure.
#define BTEAD_AD_TYPE						0x31

/// Randomizer field, CSS Part A 1.23.2. Five octets, little endian on air.
#define BTEAD_RANDOMIZER_LEN				5

/// MIC field. Four octets, which is what the CCM of Vol 6 Part E produces.
#define BTEAD_MIC_LEN						4

/// Session key and initialization vector, Vol 3 Part C 12.6 Table 12.10.
#define BTEAD_KEY_LEN						16
#define BTEAD_IV_LEN						8

/// Octets an Encrypted Data AD structure adds to the payload it wraps: the
/// randomizer ahead of it and the MIC behind it. The AD length and type
/// octets are on top of this, as for any AD structure.
#define BTEAD_OVERHEAD						(BTEAD_RANDOMIZER_LEN + BTEAD_MIC_LEN)

#pragma pack(push, 1)

/// Key material, Vol 3 Part C 12.6 Table 12.10: a 128 bit session key then a
/// 64 bit IV.
///
/// The two fields are held in the order the algorithm consumes them, and they
/// are not the same way round. SessionKey is fed to AES exactly as the sample
/// data of CSS Part A 2.3 prints it, most significant octet first. Iv is laid
/// out as the CCM nonce needs it, least significant octet first, which is the
/// reverse of how 2.3 prints it. BtEadSelfTest pins both against the
/// published ciphertext.
///
/// How the Encrypted Data Key Material characteristic orders its value on the
/// wire is a separate question, and one this structure does not answer. Do
/// not assume a read of that characteristic can be copied in verbatim without
/// checking it against a peer.
typedef struct __Bt_Ead_Key {
	uint8_t SessionKey[BTEAD_KEY_LEN];	//!< Shared session key
	uint8_t Iv[BTEAD_IV_LEN];			//!< Shared initialization vector
} BtEadKey_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Bind the engines this module needs
 *
 * The same engines the rest of the stack uses. Nothing here works before this
 * is called, which is deliberate: a build with no AES provider fails at init
 * rather than silently advertising in the clear.
 *
 * The RNG is what BtEadRandGen draws the randomizer from. CSS Part A 1.23.4
 * requires it to meet the random number requirements of Vol 2 Part H 2,
 * because a predictable randomizer hands back the tracking the whole feature
 * exists to prevent. It may be null when the caller supplies its own
 * randomizers and never calls BtEadRandGen.
 *
 * @param	pAes	: AES-128 capable cipher engine
 * @param	pRng	: Secure random source, or null
 *
 * @return	true when the AES engine was accepted
 */
bool BtEadInit(CipherEngine *pAes, RngEngine *pRng);

/**
 * @brief	Draw a randomizer
 *
 * Refuses an engine reporting IsSecure() false rather than producing a
 * predictable randomizer, and wipes the buffer when it refuses.
 *
 * @param	Rand	: Receives BTEAD_RANDOMIZER_LEN octets in air order
 *
 * @return	true on success
 */
bool BtEadRandGen(uint8_t Rand[BTEAD_RANDOMIZER_LEN]);

/**
 * @brief	Wrap advertising data in an Encrypted Data AD structure
 *
 * Writes the randomizer, the encrypted payload and the encrypted MIC, which
 * is the AD data of a single Encrypted Data AD structure. The caller adds the
 * AD length and type octets around it.
 *
 * pRand is the five octet randomizer in the order it goes on air, least
 * significant octet first. CSS Part A 1.23.4 requires it to change whenever
 * the payload changes or the random device address changes, and to come from
 * a source meeting the random number requirements of Vol 2 Part H 2: a
 * predictable randomizer gives back the tracking the feature exists to
 * prevent.
 *
 * @param	pKey	: Session key and IV
 * @param	pRand	: Randomizer, BTEAD_RANDOMIZER_LEN octets
 * @param	pPayload: AD structures to encrypt
 * @param	Len		: Payload length
 * @param	pOut	: Output buffer
 * @param	OutLen	: Output buffer size, at least Len + BTEAD_OVERHEAD
 *
 * @return	Octets written, 0 on failure
 */
size_t BtEadEncrypt(const BtEadKey_t * const pKey, const uint8_t *pRand,
					const uint8_t *pPayload, size_t Len,
					uint8_t *pOut, size_t OutLen);

/**
 * @brief	Unwrap an Encrypted Data AD structure
 *
 * pAd is the AD data, so the randomizer, the encrypted payload and the
 * encrypted MIC, without the AD length and type octets.
 *
 * The MIC is verified before anything is returned. A payload that does not
 * authenticate yields 0 and pOut is left wiped, because a caller that acted
 * on unauthenticated advertising data would be taking direction from anyone
 * within radio range.
 *
 * @param	pKey	: Session key and IV
 * @param	pAd		: AD data of the Encrypted Data structure
 * @param	AdLen	: AD data length
 * @param	pOut	: Output buffer for the recovered payload
 * @param	OutLen	: Output buffer size
 *
 * @return	Payload octets recovered, 0 on failure or a bad MIC
 */
size_t BtEadDecrypt(const BtEadKey_t * const pKey, const uint8_t *pAd,
					size_t AdLen, uint8_t *pOut, size_t OutLen);

/**
 * @brief	Known answer self test
 *
 * Runs both sample sets of CSS Part A 2.3 through encrypt and decrypt and
 * compares every octet, so a byte order or nonce mistake shows up here rather
 * than as a peer that cannot read this device.
 *
 * @return	0 on pass, negative on failure
 */
int BtEadSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_EAD_H__
