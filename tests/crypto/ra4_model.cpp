/**-------------------------------------------------------------------------
@file	ra4_model.cpp

@brief	Host test support: ra4 model.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

// Model the primitive boundary; production register sequences are tested separately.
#include "ra4_model.h"
#include "crypto/crypto.h"
#include "crypto_ra4m1.h"
#include "crypto_sce5.h"
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <initializer_list>

RaSystem g_System = {};
RaMstp g_Mstp = {0xFFFFFFFFU};
static bool s_Fail;
static bool s_FinalFail;
static unsigned s_Ends;
static Crypto *s_pReentrant;
static unsigned s_Blocks;

static bool s_Open;
static bool s_Encrypt;
static size_t s_KeyLen;
static uint8_t s_Key[32];
static unsigned s_Begins;

bool Sce5Init(void)
{
	s_Fail = false;
	return true;
}
bool Sce5Ready(void)
{
	return !s_Fail;
}
bool Sce5RandomWords(uint32_t out[4])
{
	if (s_Fail)
		return false;
	// A draw inside an AES session is a resource conflict in the engine.
	assert(!s_Open);
	return RAND_bytes(reinterpret_cast<unsigned char *>(out), 16) == 1;
}
bool Sce5AesBegin(const uint8_t *key, size_t keyLen, bool encrypt)
{
	if (s_Fail)
		return false;
	assert(keyLen == 16 || keyLen == 32);
	if (s_Open && s_KeyLen == keyLen && s_Encrypt == encrypt && memcmp(s_Key, key, keyLen) == 0)
	{
		return true;
	}
	memcpy(s_Key, key, keyLen);
	s_KeyLen = keyLen;
	s_Encrypt = encrypt;
	s_Open = true;
	s_Begins++;
	return true;
}
bool Sce5AesUpdate(const uint8_t in[16], uint8_t out[16])
{
	int length = 0;
	int final = 0;
	EVP_CIPHER_CTX *ctx;

	if (s_Fail)
		return false;
	assert(s_Open);
	ctx = EVP_CIPHER_CTX_new();
	assert(ctx);
	assert(EVP_CipherInit_ex(ctx, s_KeyLen == 32 ? EVP_aes_256_ecb() : EVP_aes_128_ecb(),
		nullptr, s_Key, nullptr, s_Encrypt) == 1);
	assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
	assert(EVP_CipherUpdate(ctx, out, &length, in, 16) == 1);
	assert(EVP_CipherFinal_ex(ctx, out + length, &final) == 1);
	EVP_CIPHER_CTX_free(ctx);
	s_Blocks++;
	return true;
}
bool Sce5AesEnd(void)
{
	s_Ends++;
	if (s_pReentrant != nullptr)
	{
		Crypto *dev = s_pReentrant;
		s_pReentrant = nullptr;
		uint8_t key[16] = {};
		uint8_t input[16] = {};
		uint8_t output[16];
		uint8_t tag[16] = {};
		CryptoKey k = {};
		k.Type = CRYPTO_KEY_AES_128;
		k.Loc = CRYPTO_KEY_LOC_PLAIN;
		k.Usage = CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT | CRYPTO_KEY_USE_SIGN;
		k.Plain.pData = key;
		k.Plain.Len = sizeof(key);
		unsigned ends = s_Ends;
		assert(dev->Mac(CRYPTO_MAC_CMAC, k, input, 16, nullptr, 16) == CRYPTO_STATUS_UNSUPPORTED);
		assert(dev->Mac(CRYPTO_MAC_CMAC, k, input, 16, tag, 16) == CRYPTO_STATUS_BUSY);
		assert(dev->Seal(CRYPTO_AEAD_AES_GCM, k, input, 12, nullptr, 0,
			input, 16, output, tag, 16) == CRYPTO_STATUS_BUSY);
		assert(dev->Open(CRYPTO_AEAD_AES_GCM, k, input, 12, nullptr, 0,
			input, 16, output, tag, 16) == CRYPTO_STATUS_BUSY);
		assert(dev->Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0,
			input, 16, output) == CRYPTO_STATUS_BUSY);
		assert(dev->Random(output, 16) == CRYPTO_STATUS_BUSY);
		assert(s_Ends == ends);
	}
	if (s_FinalFail)
	{
		s_Fail = true;
	}
	s_Open = false;
	memset(s_Key, 0, sizeof(s_Key));
	return !s_Fail;
}
static void CheckWiped(const uint8_t *pBuffer, size_t Len)
{
	assert(pBuffer[0] == 0xA5 && pBuffer[Len + 1] == 0xA5);
	for (size_t i = 1; i <= Len; i++)
	{
		assert(pBuffer[i] == 0);
	}
}

static void Recover(CryptoRa4m1Aes &Engine, Crypto &CryptoDev)
{
	s_FinalFail = false;
	Engine.Reset();
	assert(Engine.Valid());
	uint8_t random[4];
	assert(CryptoDev.Random(random, sizeof(random)) == CRYPTO_STATUS_OK);
}

static void FinalizationFailures(Crypto &CryptoDev)
{
	s_pReentrant = &CryptoDev;
	CryptoRa4m1Aes recovery;
	assert(recovery.Enable());
	uint8_t key[32] = {};
	uint8_t input[32] = {};
	uint8_t nonce[16] = {};
	uint8_t output[34];
	uint8_t tag[18];
	uint8_t ciphertext[32];
	uint8_t validTag[16];
	CryptoKey k = {};
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT | CRYPTO_KEY_USE_SIGN;
	k.Plain.pData = key;

	for (size_t keyLen : { 16U, 32U })
	{
		k.Type = keyLen == 16 ? CRYPTO_KEY_AES_128 : CRYPTO_KEY_AES_256;
		k.Plain.Len = keyLen;
		for (CRYPTO_CIPHER_ALG alg : { CRYPTO_CIPHER_ECB, CRYPTO_CIPHER_CBC, CRYPTO_CIPHER_CTR })
		{
			for (int encrypt = 0; encrypt <= 1; encrypt++)
			{
				memset(output, 0xA5, sizeof(output));
				s_FinalFail = true;
				assert(CryptoDev.Cipher(alg, encrypt, k, nonce, sizeof(nonce),
					input, sizeof(input), output + 1) == CRYPTO_STATUS_FAIL);
				CheckWiped(output, sizeof(input));
				assert(!s_Open && !Sce5Ready());
				Recover(recovery, CryptoDev);
				// Same operation succeeds after recovery; the lock was released.
				assert(CryptoDev.Cipher(alg, encrypt, k, nonce, sizeof(nonce),
					input, sizeof(input), output + 1) == CRYPTO_STATUS_OK);
			}
		}
	}

	k.Type = CRYPTO_KEY_AES_128;
	k.Plain.Len = 16;
	for (size_t len : { 0U, 17U, 32U })
	{
		memset(tag, 0xA5, sizeof(tag));
		s_FinalFail = true;
		assert(CryptoDev.Mac(CRYPTO_MAC_CMAC, k, input, len, tag + 1, 16) == CRYPTO_STATUS_FAIL);
		CheckWiped(tag, 16);
		Recover(recovery, CryptoDev);
		assert(CryptoDev.Mac(CRYPTO_MAC_CMAC, k, input, len, tag + 1, 16) == CRYPTO_STATUS_OK);

		for (CRYPTO_AEAD_ALG alg : { CRYPTO_AEAD_AES_CCM, CRYPTO_AEAD_AES_GCM })
		{
			memset(output, 0xA5, sizeof(output));
			memset(tag, 0xA5, sizeof(tag));
			s_FinalFail = true;
			assert(CryptoDev.Seal(alg, k, nonce, 12, nullptr, 0,
				input, len, output + 1, tag + 1, 16) == CRYPTO_STATUS_FAIL);
			CheckWiped(output, len);
			CheckWiped(tag, 16);
			Recover(recovery, CryptoDev);
			assert(CryptoDev.Seal(alg, k, nonce, 12, nullptr, 0,
				input, len, ciphertext, validTag, 16) == CRYPTO_STATUS_OK);
			memset(output, 0xA5, sizeof(output));
			s_FinalFail = true;
			assert(CryptoDev.Open(alg, k, nonce, 12, nullptr, 0,
				ciphertext, len, output + 1, validTag, 16) == CRYPTO_STATUS_FAIL);
			CheckWiped(output, len);
			Recover(recovery, CryptoDev);
			assert(CryptoDev.Open(alg, k, nonce, 12, nullptr, 0,
				ciphertext, len, output + 1, validTag, 16) == CRYPTO_STATUS_OK);
			assert(memcmp(output + 1, input, len) == 0);
		}
	}

	// Rejected parameters must not finalize or release another operation.
	unsigned ends = s_Ends;
	assert(CryptoDev.Mac(CRYPTO_MAC_CMAC, k, input, 1, nullptr, 16) == CRYPTO_STATUS_UNSUPPORTED);
	assert(CryptoDev.Seal(CRYPTO_AEAD_AES_GCM, k, nonce, 1, nullptr, 0,
		input, 16, output, tag, 16) == CRYPTO_STATUS_UNSUPPORTED);
	assert(CryptoDev.Open(CRYPTO_AEAD_AES_GCM, k, nonce, 1, nullptr, 0,
		input, 16, output, tag, 16) == CRYPTO_STATUS_UNSUPPORTED);
	assert(s_Ends == ends);
	assert(CryptoDev.Mac(CRYPTO_MAC_CMAC, k, input, 16, tag, 16) == CRYPTO_STATUS_OK);
	// In-place output is also discarded when finalization fails.
	memset(output, 0xA5, sizeof(output));
	s_FinalFail = true;
	assert(CryptoDev.Cipher(CRYPTO_CIPHER_CTR, 1, k, nonce, sizeof(nonce),
		output + 1, 32, output + 1) == CRYPTO_STATUS_FAIL);
	CheckWiped(output, 32);
	Recover(recovery, CryptoDev);
	assert(s_pReentrant == nullptr);
	puts("RA4 finalization failure: cipher/CMAC/CCM/GCM status, wiping and recovery passed");
}

extern int CryptoEngineTest();
int main()
{
	Crypto crypto;
	uint8_t key[16] = {};
	uint8_t in[16] = {};
	uint8_t out[18];
	CryptoKey k = {};

	assert(CryptoEngineTest() == 0);
	assert(s_Blocks > 1000);
	// The key index is generated once per operation, not once per block.
	assert(s_Begins * 2 < s_Blocks);
	assert(!s_Open);
	assert((g_Mstp.MSTPCRC & (1UL << 31)) == 0);
	assert(crypto.Init());
	FinalizationFailures(crypto);
	k.Type = CRYPTO_KEY_AES_128;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = sizeof(key);
	s_Fail = true;
	memset(out, 0xA5, sizeof(out));
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, out+1) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 1; i < 17; i++)
		assert(out[i] == 0);
	assert(out[0] == 0xA5 && out[17] == 0xA5);
	memset(out, 0xA5, sizeof(out));
	assert(crypto.Random(out+1, 3) == CRYPTO_STATUS_FAIL);
	assert(out[0] == 0xA5 && out[4] == 0xA5);
	assert(out[1] == 0 && out[2] == 0 && out[3] == 0);
	assert(!crypto.IsSecure());
	printf("RA4 engine model: %u AES blocks in %u sessions; failure/output bounds passed\n", s_Blocks, s_Begins);
}
