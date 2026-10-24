// OpenSSL comparison and failure injection for shared CCM/GCM seal/open code.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <openssl/evp.h>
#include "crypto/crypto_softaes.h"

class FailingAes : public CryptoSoftAes {
public:
	unsigned Blocks = 0, FailAt = 0, Begins = 0, Ends = 0;
	bool Refuse = false;
protected:
	bool AesOpBegin() override { Begins++; return !Refuse; }
	void AesOpEnd() override { Ends++; }
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]) override
	{
		return ++Blocks != FailAt && CryptoSoftAes::AesEcbEncrypt(Key, In, Out);
	}
};

static void Reference(bool Gcm, const uint8_t *pKey, const uint8_t *pNonce,
	int NonceLen, const uint8_t *pAad, int AadLen, const uint8_t *pMsg, int Len,
	uint8_t *pOut, uint8_t *pTag, int TagLen)
{
	EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
	int count, final;

	assert(ctx != nullptr);
	assert(EVP_EncryptInit_ex(ctx, Gcm ? EVP_aes_128_gcm() : EVP_aes_128_ccm(), nullptr, nullptr, nullptr) == 1);
	assert(EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_AEAD_SET_IVLEN, NonceLen, nullptr) == 1);
	if (!Gcm) assert(EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_AEAD_SET_TAG, TagLen, nullptr) == 1);
	assert(EVP_EncryptInit_ex(ctx, nullptr, nullptr, pKey, pNonce) == 1);
	if (!Gcm) assert(EVP_EncryptUpdate(ctx, nullptr, &count, nullptr, Len) == 1);
	if (AadLen) assert(EVP_EncryptUpdate(ctx, nullptr, &count, pAad, AadLen) == 1);
	assert(EVP_EncryptUpdate(ctx, pOut, &count, pMsg, Len) == 1);
	assert(EVP_EncryptFinal_ex(ctx, pOut + count, &final) == 1);
	assert(EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_AEAD_GET_TAG, TagLen, pTag) == 1);
	EVP_CIPHER_CTX_free(ctx);
}

int main()
{
	CryptoSoftAes engine;
	FailingAes failing;
	CryptoKey key{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
		CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	uint8_t k[16] = {}, nonce[13] = {}, aad[65], msg[257];
	uint8_t ref[257], tag[16], actual[259], actualTag[18], plain[259];
	const size_t lengths[] = {0, 1, 15, 16, 17, 40, 64, 257};
	const size_t aadLengths[] = {0, 1, 14, 15, 16, 31, 65};
	CRYPTO_AEAD_ALG alg;
	size_t len, alen, nlen, tlen, li, ai, i;
	unsigned mode, calls, fail, direction, cases = 0;

	engine.Enable();
	key.Plain.pData = k;
	key.Plain.Len = sizeof(k);
	for (i = 0; i < sizeof(msg); i++) msg[i] = (uint8_t)(17U * i + 3U);
	for (i = 0; i < sizeof(aad); i++) aad[i] = (uint8_t)(31U * i + 7U);
	for (mode = 0; mode < 2; mode++)
	{
		alg = mode ? CRYPTO_AEAD_AES_GCM : CRYPTO_AEAD_AES_CCM;
		for (nlen = mode ? 12 : 7; nlen <= (mode ? 12U : 13U); nlen++)
		for (tlen = 4; tlen <= 16; tlen++)
		{
			if (mode ? (tlen != 4 && tlen != 8 && tlen < 12) : (tlen & 1U)) continue;
			for (li = 0; li < sizeof(lengths) / sizeof(lengths[0]); li++)
			for (ai = 0; ai < sizeof(aadLengths) / sizeof(aadLengths[0]); ai++)
			{
				len = lengths[li];
				alen = aadLengths[ai];
				Reference(mode != 0, k, nonce, nlen, aad, alen, msg, len, ref, tag, tlen);
				memset(actual, 0xA5, sizeof(actual));
				memset(actualTag, 0xA5, sizeof(actualTag));
				assert(engine.Seal(alg, key, nonce, nlen, aad, alen, msg, len, actual + 1, actualTag + 1, tlen) == CRYPTO_STATUS_OK);
				assert(memcmp(actual + 1, ref, len) == 0 && memcmp(actualTag + 1, tag, tlen) == 0);
				assert(actual[0] == 0xA5 && actual[len + 1] == 0xA5);
				assert(actualTag[0] == 0xA5 && actualTag[tlen + 1] == 0xA5);
				memset(plain, 0xA5, sizeof(plain));
				assert(engine.Open(alg, key, nonce, nlen, aad, alen, actual + 1, len, plain + 1, tag, tlen) == CRYPTO_STATUS_OK);
				assert(memcmp(plain + 1, msg, len) == 0 && plain[0] == 0xA5 && plain[len + 1] == 0xA5);
				memcpy(actual + 1, msg, len);
				assert(engine.Seal(alg, key, nonce, nlen, aad, alen, actual + 1, len, actual + 1, actualTag + 1, tlen) == CRYPTO_STATUS_OK);
				assert(memcmp(actual + 1, ref, len) == 0 && memcmp(actualTag + 1, tag, tlen) == 0);
				assert(engine.Open(alg, key, nonce, nlen, aad, alen, actual + 1, len, actual + 1, tag, tlen) == CRYPTO_STATUS_OK);
				assert(memcmp(actual + 1, msg, len) == 0);
				tag[0] ^= 1;
				assert(engine.Open(alg, key, nonce, nlen, aad, alen, ref, len, plain + 1, tag, tlen) == CRYPTO_STATUS_FAIL);
				for (i = 1; i <= len; i++) assert(plain[i] == 0);
				assert(plain[0] == 0xA5 && plain[len + 1] == 0xA5);
				cases++;
			}
		}
		Reference(mode != 0, k, nonce, 12, aad, sizeof(aad), msg, sizeof(msg), ref, tag, 16);
		for (direction = 0; direction < 2; direction++)
		{
			calls = 0;
			for (fail = 0; fail <= calls; fail++)
			{
				failing.Blocks = failing.Begins = failing.Ends = 0;
				failing.FailAt = fail;
				memset(actual, 0xA5, sizeof(actual));
				memset(actualTag, 0xA5, sizeof(actualTag));
				assert((direction ? failing.Seal(alg, key, nonce, 12, aad, sizeof(aad), msg, sizeof(msg), actual + 1, actualTag + 1, 16) :
					failing.Open(alg, key, nonce, 12, aad, sizeof(aad), ref, sizeof(ref), actual + 1, tag, 16)) ==
					(fail ? CRYPTO_STATUS_FAIL : CRYPTO_STATUS_OK));
				assert(failing.Begins == 1 && failing.Ends == 1);
				if (!fail) calls = failing.Blocks;
				else
				{
					assert(failing.Blocks == fail);
					for (i = 1; i <= sizeof(msg); i++) assert(actual[i] == 0);
					if (direction) for (i = 1; i <= 16; i++) assert(actualTag[i] == 0);
				}
				assert(actual[0] == 0xA5 && actual[258] == 0xA5);
				assert(actualTag[0] == 0xA5 && actualTag[17] == 0xA5);
			}
		}
	}
	printf("CCM/GCM: %u OpenSSL cases, in-place, output bounds, bad tags and every AES failure passed\n", cases);
}
