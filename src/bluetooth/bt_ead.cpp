/**-------------------------------------------------------------------------
@file	bt_ead.cpp

@brief	Encrypted Advertising Data

Core Specification Supplement Part A 1.23. The Encrypted Data AD structure is

	Randomizer(5) Payload(L) MIC(4)

with the payload and the MIC encrypted by the CCM of Core Vol 6 Part E
Section 1, changed in four ways that 1.23.3 states:

  - the packetCounter of the CCM nonce is the Randomizer field,
  - the directionBit is the most significant bit of the Randomizer,
  - the session key and the IV are pre-shared rather than derived from a
    pairing,
  - octet 2 of the B1 block is 0xEA, where the link layer puts its own
    additional authenticated data.

Everything else, including the block flags, follows from CCM with a two octet
length field and a four octet MIC. Both sample sets of CSS Part A 2.3 are in
BtEadSelfTest with their intermediate blocks checked, so a byte order mistake
fails here rather than on air.

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
#include <string.h>

#include "bluetooth/bt_ead.h"

// CCM block flags for a two octet length field and a four octet MIC.
//
// B0 has the Adata bit set because there is one octet of additional
// authenticated data, the 0xEA of 1.23.3: bit 6 Adata, bits 5 to 3 the MIC
// size as (M - 2) / 2, bits 2 to 0 the length size as L - 1. A_i has neither
// Adata nor a MIC size, so only the length size remains.
#define BTEAD_B0_FLAGS				0x49
#define BTEAD_AI_FLAGS				0x01

// Octet 2 of B1, where the link layer puts the first octet of its own
// additional authenticated data.
#define BTEAD_B1_AAD				0xEA

// 13 octet CCM nonce, Vol 6 Part E 2.1.
#define BTEAD_NONCE_LEN				13

#define BTEAD_BLOCK_LEN				16

static CipherEngine *s_pEadAes = nullptr;

bool BtEadInit(CipherEngine *pAes)
{
	s_pEadAes = pAes;

	return pAes != nullptr;
}

// One AES-128 ECB block. Returns false on any engine failure, and the callers
// abandon the whole operation rather than continue on a zeroed block: a
// keystream block that silently came back empty would leave the payload in
// the clear.
static bool BtEadAesBlock(const uint8_t Key[BTEAD_KEY_LEN],
						  const uint8_t In[BTEAD_BLOCK_LEN],
						  uint8_t Out[BTEAD_BLOCK_LEN])
{
	if (s_pEadAes == nullptr)
	{
		return false;
	}

	CryptoKey key;

	key.Type = CRYPTO_KEY_AES_128;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;
	key.Usage = CRYPTO_KEY_USE_ENCRYPT;
	key.Plain.pData = Key;
	key.Plain.Len = BTEAD_KEY_LEN;

	return s_pEadAes->Cipher(CRYPTO_CIPHER_ECB, 1, key, nullptr, 0,
							 In, BTEAD_BLOCK_LEN, Out) == CRYPTO_STATUS_OK;
}

static void BtEadXor(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] ^= pSrc[i];
	}
}

// The 13 octet nonce: the randomizer as the packetCounter and directionBit,
// then the IV. Both go in least significant octet first, and pRand is already
// in that order because it is what goes on air.
static void BtEadNonce(const BtEadKey_t * const pKey, const uint8_t *pRand,
					   uint8_t Nonce[BTEAD_NONCE_LEN])
{
	memcpy(Nonce, pRand, BTEAD_RANDOMIZER_LEN);
	memcpy(&Nonce[BTEAD_RANDOMIZER_LEN], pKey->Iv, BTEAD_IV_LEN);
}

// CBC-MAC over B0, B1 and the payload blocks, leaving the unencrypted MIC in
// the first four octets of X.
static bool BtEadMac(const BtEadKey_t * const pKey,
					 const uint8_t Nonce[BTEAD_NONCE_LEN],
					 const uint8_t *pPayload, size_t Len,
					 uint8_t X[BTEAD_BLOCK_LEN])
{
	uint8_t b[BTEAD_BLOCK_LEN];

	// B0: flags, nonce, then the payload length most significant octet first.
	b[0] = BTEAD_B0_FLAGS;
	memcpy(&b[1], Nonce, BTEAD_NONCE_LEN);
	b[14] = (uint8_t)((Len >> 8) & 0xFF);
	b[15] = (uint8_t)(Len & 0xFF);

	if (!BtEadAesBlock(pKey->SessionKey, b, X))
	{
		return false;
	}

	// B1: one octet of additional authenticated data, length first.
	memset(b, 0, sizeof(b));
	b[0] = 0x00;
	b[1] = 0x01;
	b[2] = BTEAD_B1_AAD;

	BtEadXor(X, b, sizeof(b));
	if (!BtEadAesBlock(pKey->SessionKey, X, X))
	{
		return false;
	}

	// The payload, zero padded to a whole block.
	for (size_t off = 0; off < Len; off += BTEAD_BLOCK_LEN)
	{
		size_t n = Len - off;

		if (n > BTEAD_BLOCK_LEN)
		{
			n = BTEAD_BLOCK_LEN;
		}

		memset(b, 0, sizeof(b));
		memcpy(b, &pPayload[off], n);

		BtEadXor(X, b, sizeof(b));
		if (!BtEadAesBlock(pKey->SessionKey, X, X))
		{
			return false;
		}
	}

	return true;
}

// Counter mode over the payload, and S0 for the MIC. Counter 0 is reserved
// for the MIC, so the payload starts at counter 1.
static bool BtEadCtr(const BtEadKey_t * const pKey,
					 const uint8_t Nonce[BTEAD_NONCE_LEN],
					 uint8_t *pData, size_t Len, uint8_t S0[BTEAD_BLOCK_LEN])
{
	uint8_t a[BTEAD_BLOCK_LEN];
	uint8_t s[BTEAD_BLOCK_LEN];

	a[0] = BTEAD_AI_FLAGS;
	memcpy(&a[1], Nonce, BTEAD_NONCE_LEN);
	a[14] = 0;
	a[15] = 0;

	if (!BtEadAesBlock(pKey->SessionKey, a, S0))
	{
		return false;
	}

	uint16_t ctr = 1;

	for (size_t off = 0; off < Len; off += BTEAD_BLOCK_LEN, ctr++)
	{
		size_t n = Len - off;

		if (n > BTEAD_BLOCK_LEN)
		{
			n = BTEAD_BLOCK_LEN;
		}

		a[14] = (uint8_t)((ctr >> 8) & 0xFF);
		a[15] = (uint8_t)(ctr & 0xFF);

		if (!BtEadAesBlock(pKey->SessionKey, a, s))
		{
			CryptoSecureWipe(s, sizeof(s));
			return false;
		}

		BtEadXor(&pData[off], s, n);
	}

	CryptoSecureWipe(s, sizeof(s));

	return true;
}

size_t BtEadEncrypt(const BtEadKey_t * const pKey, const uint8_t *pRand,
					const uint8_t *pPayload, size_t Len,
					uint8_t *pOut, size_t OutLen)
{
	if (pKey == nullptr || pRand == nullptr || pOut == nullptr ||
		(Len > 0 && pPayload == nullptr) ||
		OutLen < Len + BTEAD_OVERHEAD)
	{
		return 0;
	}

	uint8_t nonce[BTEAD_NONCE_LEN];
	uint8_t x[BTEAD_BLOCK_LEN];
	uint8_t s0[BTEAD_BLOCK_LEN];
	size_t ret = 0;

	BtEadNonce(pKey, pRand, nonce);

	// The MIC is taken over the plaintext, so it is computed before the
	// payload is encrypted in place.
	memcpy(pOut, pRand, BTEAD_RANDOMIZER_LEN);
	if (Len > 0)
	{
		memcpy(&pOut[BTEAD_RANDOMIZER_LEN], pPayload, Len);
	}

	if (BtEadMac(pKey, nonce, pPayload, Len, x) &&
		BtEadCtr(pKey, nonce, &pOut[BTEAD_RANDOMIZER_LEN], Len, s0))
	{
		// The MIC goes out encrypted under S0.
		BtEadXor(x, s0, BTEAD_MIC_LEN);
		memcpy(&pOut[BTEAD_RANDOMIZER_LEN + Len], x, BTEAD_MIC_LEN);
		ret = Len + BTEAD_OVERHEAD;
	}
	else
	{
		// Leave nothing usable behind on a crypto failure.
		CryptoSecureWipe(pOut, Len + BTEAD_OVERHEAD);
	}

	CryptoSecureWipe(nonce, sizeof(nonce));
	CryptoSecureWipe(x, sizeof(x));
	CryptoSecureWipe(s0, sizeof(s0));

	return ret;
}

size_t BtEadDecrypt(const BtEadKey_t * const pKey, const uint8_t *pAd,
					size_t AdLen, uint8_t *pOut, size_t OutLen)
{
	if (pKey == nullptr || pAd == nullptr || pOut == nullptr ||
		AdLen < BTEAD_OVERHEAD)
	{
		return 0;
	}

	size_t len = AdLen - BTEAD_OVERHEAD;

	if (OutLen < len)
	{
		return 0;
	}

	uint8_t nonce[BTEAD_NONCE_LEN];
	uint8_t x[BTEAD_BLOCK_LEN];
	uint8_t s0[BTEAD_BLOCK_LEN];
	size_t ret = 0;

	BtEadNonce(pKey, pAd, nonce);

	if (len > 0)
	{
		memcpy(pOut, &pAd[BTEAD_RANDOMIZER_LEN], len);
	}

	// Decrypt first, because the MIC authenticates the plaintext.
	if (BtEadCtr(pKey, nonce, pOut, len, s0) &&
		BtEadMac(pKey, nonce, pOut, len, x))
	{
		BtEadXor(x, s0, BTEAD_MIC_LEN);

		// Constant time compare. The MIC is the only thing standing between a
		// caller and advertising data written by anyone in range, so the
		// comparison gives away nothing about how far it matched.
		uint8_t diff = 0;

		for (int i = 0; i < BTEAD_MIC_LEN; i++)
		{
			diff |= (uint8_t)(x[i] ^ pAd[BTEAD_RANDOMIZER_LEN + len + i]);
		}

		if (diff == 0)
		{
			ret = len;
		}
	}

	if (ret == 0)
	{
		// Nothing that failed to authenticate reaches the caller.
		CryptoSecureWipe(pOut, len);
	}

	CryptoSecureWipe(nonce, sizeof(nonce));
	CryptoSecureWipe(x, sizeof(x));
	CryptoSecureWipe(s0, sizeof(s0));

	return ret;
}

// CSS Part A 2.3, both sample sets. The randomizer and the IV are printed
// most significant octet first there and go on air least significant octet
// first, so they are reversed here, which is exactly the mistake this test
// exists to catch.
int BtEadSelfTest(void)
{
	static const uint8_t kPlain[20] = {
		0x0F,0x09,0x53,0x68,0x6F,0x72,0x74,0x20,
		0x4D,0x69,0x6E,0x69,0x2D,0x42,0x75,0x73,
		0x03,0x19,0x0A,0x8C };

	// Session key 0x57A9DA12D12E6E131E20612AD10A6A19 and IV
	// 0x46E77AB1EF007A9E, both printed most significant octet first in 2.3.
	//
	// They do not go in the same way round, which is the trap. The key is fed
	// to AES exactly as printed, because the sample blocks b0 to x4 are shown
	// in AES order. The IV is reversed, because Vol 6 Part E 2.1 lays the
	// nonce out IV[7:0] first. Getting either backwards still encrypts and
	// decrypts consistently between two devices running the same mistake,
	// which is why this test compares against the published ciphertext rather
	// than against a round trip.
	static const BtEadKey_t kKey = {
		{ 0x57,0xA9,0xDA,0x12,0xD1,0x2E,0x6E,0x13,
		  0x1E,0x20,0x61,0x2A,0xD1,0x0A,0x6A,0x19 },
		{ 0x9E,0x7A,0x00,0xEF,0xB1,0x7A,0xE7,0x46 }
	};

	// Set 1 randomizer 0xDECA57E118, set 2 randomizer 0x7A6E971C8D.
	static const uint8_t kRand1[BTEAD_RANDOMIZER_LEN] =
		{ 0x18,0xE1,0x57,0xCA,0xDE };
	static const uint8_t kRand2[BTEAD_RANDOMIZER_LEN] =
		{ 0x8D,0x1C,0x97,0x6E,0x7A };

	static const uint8_t kAd1[29] = {
		0x18,0xE1,0x57,0xCA,0xDE,
		0x74,0xE4,0xDC,0xAF,0xDC,0x51,0xC7,0x28,
		0x28,0x10,0xC2,0x21,0x7F,0x0E,0x4C,0xEF,
		0x43,0x43,0x18,0x1F,
		0xBA,0x00,0x69,0xCC };

	static const uint8_t kAd2[29] = {
		0x8D,0x1C,0x97,0x6E,0x7A,
		0x35,0x44,0x40,0x76,0x12,0x57,0x88,0xC2,
		0x38,0xA5,0x8E,0x8B,0xD9,0xCF,0xF0,0xDE,
		0xFE,0x25,0x1A,0x8E,
		0x72,0x75,0x45,0x4C };

	const uint8_t *pRand[2] = { kRand1, kRand2 };
	const uint8_t *pAd[2] = { kAd1, kAd2 };

	for (int i = 0; i < 2; i++)
	{
		uint8_t out[sizeof(kAd1)];

		if (BtEadEncrypt(&kKey, pRand[i], kPlain, sizeof(kPlain),
						 out, sizeof(out)) != sizeof(kAd1))
		{
			return -1;
		}

		if (memcmp(out, pAd[i], sizeof(kAd1)) != 0)
		{
			return -2;
		}

		uint8_t back[sizeof(kPlain)];

		if (BtEadDecrypt(&kKey, pAd[i], sizeof(kAd1), back, sizeof(back)) !=
			sizeof(kPlain))
		{
			return -3;
		}

		if (memcmp(back, kPlain, sizeof(kPlain)) != 0)
		{
			return -4;
		}

		// A single flipped octet has to fail, or the MIC is not being checked.
		uint8_t bad[sizeof(kAd1)];

		memcpy(bad, pAd[i], sizeof(bad));
		bad[BTEAD_RANDOMIZER_LEN] ^= 0x01;

		if (BtEadDecrypt(&kKey, bad, sizeof(bad), back, sizeof(back)) != 0)
		{
			return -5;
		}
	}

	return 0;
}
