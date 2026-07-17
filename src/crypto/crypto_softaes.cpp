/**-------------------------------------------------------------------------
@file	crypto_softaes.cpp

@brief	Software AES-128 cipher and CMAC engine.

		CMAC is computed through the virtual Cipher operation. A hardware subclass
		therefore accelerates inherited CMAC without duplicating the MAC code. The
		public operations enforce CryptoKey usage policy; CMAC uses an internal key
		descriptor for its block-cipher calls after validating SIGN permission.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softaes.h"

#define AES_BLOCK		16U
#define AES128_ROUNDS	10U
#define AES128_RK_WORDS	44U

static void AesWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U)
	{
		*p++ = 0U;
	}
}

static const uint8_t s_SBox[256] = {
	0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
	0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
	0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
	0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
	0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
	0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
	0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
	0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
	0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
	0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
	0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
	0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
	0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
	0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
	0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
	0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t s_Rcon[AES128_ROUNDS] = {
	0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36
};

static inline uint8_t AesXtime(uint8_t a)
{
	return (uint8_t)((a << 1) ^ ((a >> 7) * 0x1b));
}

static void AesKeyExpand(const uint8_t Key[16], uint8_t Rk[AES128_RK_WORDS * 4])
{
	memcpy(Rk, Key, AES_BLOCK);
	for (uint32_t i = 4; i < AES128_RK_WORDS; i++)
	{
		uint8_t t[4] = {
			Rk[(i - 1U) * 4U], Rk[(i - 1U) * 4U + 1U],
			Rk[(i - 1U) * 4U + 2U], Rk[(i - 1U) * 4U + 3U]
		};
		if ((i & 3U) == 0U)
		{
			uint8_t first = t[0];
			t[0] = s_SBox[t[1]];
			t[1] = s_SBox[t[2]];
			t[2] = s_SBox[t[3]];
			t[3] = s_SBox[first];
			t[0] ^= s_Rcon[(i / 4U) - 1U];
		}
		for (uint32_t j = 0; j < 4U; j++)
		{
			Rk[i * 4U + j] = (uint8_t)(Rk[(i - 4U) * 4U + j] ^ t[j]);
		}
	}
}

static void AesEncryptBlock(const uint8_t Rk[AES128_RK_WORDS * 4],
							const uint8_t In[16], uint8_t Out[16])
{
	uint8_t st[16];
	for (int i = 0; i < 16; i++)
	{
		st[i] = (uint8_t)(In[i] ^ Rk[i]);
	}

	for (uint32_t round = 1; round < AES128_ROUNDS; round++)
	{
		uint8_t t[16];
		for (int i = 0; i < 16; i++) st[i] = s_SBox[st[i]];
		t[0]=st[0];  t[4]=st[4];  t[8]=st[8];   t[12]=st[12];
		t[1]=st[5];  t[5]=st[9];  t[9]=st[13];  t[13]=st[1];
		t[2]=st[10]; t[6]=st[14]; t[10]=st[2];  t[14]=st[6];
		t[3]=st[15]; t[7]=st[3];  t[11]=st[7];  t[15]=st[11];
		for (int c = 0; c < 4; c++)
		{
			uint8_t *p = &t[c * 4];
			uint8_t a0=p[0], a1=p[1], a2=p[2], a3=p[3];
			st[c*4]   = (uint8_t)(AesXtime(a0) ^ (AesXtime(a1)^a1) ^ a2 ^ a3);
			st[c*4+1] = (uint8_t)(a0 ^ AesXtime(a1) ^ (AesXtime(a2)^a2) ^ a3);
			st[c*4+2] = (uint8_t)(a0 ^ a1 ^ AesXtime(a2) ^ (AesXtime(a3)^a3));
			st[c*4+3] = (uint8_t)((AesXtime(a0)^a0) ^ a1 ^ a2 ^ AesXtime(a3));
		}
		for (int i = 0; i < 16; i++) st[i] ^= Rk[round * 16U + (uint32_t)i];
	}

	uint8_t t[16];
	for (int i = 0; i < 16; i++) st[i] = s_SBox[st[i]];
	t[0]=st[0];  t[4]=st[4];  t[8]=st[8];   t[12]=st[12];
	t[1]=st[5];  t[5]=st[9];  t[9]=st[13];  t[13]=st[1];
	t[2]=st[10]; t[6]=st[14]; t[10]=st[2];  t[14]=st[6];
	t[3]=st[15]; t[7]=st[3];  t[11]=st[7];  t[15]=st[11];
	for (int i = 0; i < 16; i++)
	{
		Out[i] = (uint8_t)(t[i] ^ Rk[AES128_ROUNDS * 16U + (uint32_t)i]);
	}
	AesWipe(st, sizeof(st));
	AesWipe(t, sizeof(t));
}

static bool AesKeyMaterialOk(const CryptoKey &Key)
{
	return Key.Type == CRYPTO_KEY_AES_128 &&
		Key.Loc == CRYPTO_KEY_LOC_PLAIN &&
		Key.Plain.pData != nullptr && Key.Plain.Len == AES_BLOCK;
}

CRYPTO_STATUS CryptoSoftAes::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
									const CryptoKey &Key,
									const uint8_t *pIv, size_t IvLen,
									const uint8_t *pIn, size_t Len,
									uint8_t *pOut)
{
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT :
									 CRYPTO_KEY_USE_DECRYPT;
	if (!AesKeyMaterialOk(Key) || (Key.Usage & required) == 0U ||
		pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	uint8_t rk[AES128_RK_WORDS * 4U];
	AesKeyExpand(Key.Plain.pData, rk);
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;

	if (Alg == CRYPTO_CIPHER_ECB)
	{
		if ((Len & (AES_BLOCK - 1U)) != 0U)
		{
			st = CRYPTO_STATUS_FAIL;
		}
		else if (bEncrypt == 0)
		{
			st = CRYPTO_STATUS_UNSUPPORTED;
		}
		else
		{
			for (size_t off = 0; off < Len; off += AES_BLOCK)
			{
				AesEncryptBlock(rk, pIn + off, pOut + off);
			}
		}
	}
	else if (Alg == CRYPTO_CIPHER_CTR)
	{
		if (pIv == nullptr || IvLen != AES_BLOCK)
		{
			st = CRYPTO_STATUS_FAIL;
		}
		else
		{
			uint8_t ctr[AES_BLOCK], ks[AES_BLOCK];
			memcpy(ctr, pIv, AES_BLOCK);
			for (size_t off = 0; off < Len; )
			{
				AesEncryptBlock(rk, ctr, ks);
				size_t n = Len - off < AES_BLOCK ? Len - off : AES_BLOCK;
				for (size_t j = 0; j < n; j++) pOut[off+j] = pIn[off+j] ^ ks[j];
				for (int j = AES_BLOCK - 1; j >= 0; j--)
				{
					if (++ctr[j] != 0U) break;
				}
				off += n;
			}
			AesWipe(ctr, sizeof(ctr));
			AesWipe(ks, sizeof(ks));
		}
	}
	else if (Alg == CRYPTO_CIPHER_CBC)
	{
		if (pIv == nullptr || IvLen != AES_BLOCK ||
			(Len & (AES_BLOCK - 1U)) != 0U)
		{
			st = CRYPTO_STATUS_FAIL;
		}
		else if (bEncrypt == 0)
		{
			st = CRYPTO_STATUS_UNSUPPORTED;
		}
		else
		{
			uint8_t chain[AES_BLOCK];
			memcpy(chain, pIv, AES_BLOCK);
			for (size_t off = 0; off < Len; off += AES_BLOCK)
			{
				uint8_t block[AES_BLOCK];
				for (size_t j = 0; j < AES_BLOCK; j++) block[j] = pIn[off+j] ^ chain[j];
				AesEncryptBlock(rk, block, pOut + off);
				memcpy(chain, pOut + off, AES_BLOCK);
				AesWipe(block, sizeof(block));
			}
			AesWipe(chain, sizeof(chain));
		}
	}
	else
	{
		st = CRYPTO_STATUS_UNSUPPORTED;
	}

	AesWipe(rk, sizeof(rk));
	return st;
}

static void CmacSubkey(const uint8_t In[16], uint8_t Out[16])
{
	uint8_t carry = 0U;
	uint8_t msb = In[0] >> 7;
	for (int i = 15; i >= 0; i--)
	{
		uint8_t b = In[i];
		Out[i] = (uint8_t)((b << 1) | carry);
		carry = (b >> 7) & 1U;
	}
	if (msb != 0U) Out[15] ^= 0x87U;
}

CRYPTO_STATUS CryptoSoftAes::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
								 const uint8_t *pMsg, size_t Len,
								 uint8_t *pMac, size_t MacLen)
{
	if (Alg != CRYPTO_MAC_CMAC || !AesKeyMaterialOk(Key) ||
		(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U || pMac == nullptr ||
		MacLen == 0U || MacLen > AES_BLOCK || (Len > 0U && pMsg == nullptr))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// Cipher enforces ENCRYPT usage. CMAC has already authorized SIGN, so use an
	// internal descriptor for the block primitive while retaining virtual dispatch.
	CryptoKey blockKey = Key;
	blockKey.Usage |= CRYPTO_KEY_USE_ENCRYPT;

	uint8_t zero[AES_BLOCK] = {};
	uint8_t L[AES_BLOCK], K1[AES_BLOCK], K2[AES_BLOCK];
	if (Cipher(CRYPTO_CIPHER_ECB, 1, blockKey, nullptr, 0,
			   zero, AES_BLOCK, L) != CRYPTO_STATUS_OK)
	{
		return CRYPTO_STATUS_FAIL;
	}
	CmacSubkey(L, K1);
	CmacSubkey(K1, K2);

	size_t blocks = (Len + AES_BLOCK - 1U) / AES_BLOCK;
	bool complete = Len != 0U && (Len & (AES_BLOCK - 1U)) == 0U;
	if (blocks == 0U) blocks = 1U;

	uint8_t X[AES_BLOCK] = {};
	uint8_t Y[AES_BLOCK];
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;
	for (size_t i = 0; i + 1U < blocks; i++)
	{
		for (size_t j = 0; j < AES_BLOCK; j++) Y[j] = X[j] ^ pMsg[i*AES_BLOCK+j];
		if (Cipher(CRYPTO_CIPHER_ECB, 1, blockKey, nullptr, 0,
				   Y, AES_BLOCK, X) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
			break;
		}
	}

	if (st == CRYPTO_STATUS_OK)
	{
		uint8_t last[AES_BLOCK];
		size_t base = (blocks - 1U) * AES_BLOCK;
		if (complete)
		{
			for (size_t j = 0; j < AES_BLOCK; j++) last[j] = pMsg[base+j] ^ K1[j];
		}
		else
		{
			size_t rem = Len - base;
			for (size_t j = 0; j < AES_BLOCK; j++)
			{
				uint8_t value = j < rem ? pMsg[base+j] : (j == rem ? 0x80U : 0U);
				last[j] = value ^ K2[j];
			}
		}
		for (size_t j = 0; j < AES_BLOCK; j++) Y[j] = X[j] ^ last[j];
		uint8_t full[AES_BLOCK];
		if (Cipher(CRYPTO_CIPHER_ECB, 1, blockKey, nullptr, 0,
				   Y, AES_BLOCK, full) != CRYPTO_STATUS_OK)
		{
			st = CRYPTO_STATUS_FAIL;
		}
		else
		{
			memcpy(pMac, full, MacLen);
		}
		AesWipe(last, sizeof(last));
		AesWipe(full, sizeof(full));
	}

	AesWipe(L, sizeof(L));
	AesWipe(K1, sizeof(K1));
	AesWipe(K2, sizeof(K2));
	AesWipe(X, sizeof(X));
	AesWipe(Y, sizeof(Y));
	return st;
}

int CryptoSoftAes::SelfTest()
{
	static const uint8_t aesKey[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t aesIn[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
	static const uint8_t aesExp[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };
	static const uint8_t cmacKey[16] = {
		0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
		0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
	static const uint8_t cmacExp[16] = {
		0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,
		0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };

	CryptoKey cipherKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
						CRYPTO_KEY_USE_ENCRYPT, {}};
	cipherKey.Plain.pData = aesKey;
	cipherKey.Plain.Len = sizeof(aesKey);
	uint8_t out[16];
	if (Cipher(CRYPTO_CIPHER_ECB, 1, cipherKey, nullptr, 0,
			   aesIn, sizeof(aesIn), out) != CRYPTO_STATUS_OK ||
		memcmp(out, aesExp, sizeof(out)) != 0)
	{
		return -1;
	}

	CryptoKey macKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {}};
	macKey.Plain.pData = cmacKey;
	macKey.Plain.Len = sizeof(cmacKey);
	uint8_t mac[16];
	if (Mac(CRYPTO_MAC_CMAC, macKey, nullptr, 0, mac, sizeof(mac)) !=
		CRYPTO_STATUS_OK || memcmp(mac, cmacExp, sizeof(mac)) != 0)
	{
		return -2;
	}
	return 0;
}

CryptoSoftAes *CryptoSoftAesCreate(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoSoftAes) ||
		((uintptr_t)pMem & (alignof(CryptoSoftAes) - 1U)) != 0U)
	{
		return nullptr;
	}
	CryptoSoftAes *p = new (pMem) CryptoSoftAes();
	p->Enable();
	return p;
}
