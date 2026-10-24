// OpenSSL comparisons and failure injection for CMAC and CCM/GCM.
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

static void CheckCcmAadLimit()
{
	CryptoSoftAes engine;
	CryptoKey key{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
		CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	uint8_t k[16] = {}, nonce[13] = {}, aad[65280], msg[257];
	uint8_t ref[257], tag[16], out[259], actualTag[18];
	const size_t lengths[] = {0, 1, 31, 257};
	size_t i, li, len, nlen;

	engine.Enable();
	key.Plain.pData = k;
	key.Plain.Len = sizeof(k);
	for (i = 0; i < sizeof(aad); i++) aad[i] = (uint8_t)(i * 31U + 7U);
	for (i = 0; i < sizeof(msg); i++) msg[i] = (uint8_t)(i * 17U + 3U);
	for (nlen = 7; nlen <= 13; nlen++)
	for (li = 0; li < sizeof(lengths) / sizeof(lengths[0]); li++)
	{
		len = lengths[li];
		Reference(false, k, nonce, nlen, aad, sizeof(aad) - 1U, msg, len, ref, tag, 16);
		memset(out, 0xA5, sizeof(out));
		memset(actualTag, 0xA5, sizeof(actualTag));
		assert(engine.Seal(CRYPTO_AEAD_AES_CCM, key, nonce, nlen, aad, sizeof(aad) - 1U,
			msg, len, out + 1, actualTag + 1, 16) == CRYPTO_STATUS_OK);
		assert(memcmp(out + 1, ref, len) == 0 && memcmp(actualTag + 1, tag, 16) == 0);
		assert(out[0] == 0xA5 && out[len + 1] == 0xA5);
		assert(actualTag[0] == 0xA5 && actualTag[17] == 0xA5);
		assert(engine.Open(CRYPTO_AEAD_AES_CCM, key, nonce, nlen, aad, sizeof(aad) - 1U,
			out + 1, len, out + 1, tag, 16) == CRYPTO_STATUS_OK);
		assert(memcmp(out + 1, msg, len) == 0);
		assert(out[0] == 0xA5 && out[len + 1] == 0xA5);
		assert(engine.Seal(CRYPTO_AEAD_AES_CCM, key, nonce, nlen, aad, sizeof(aad),
			msg, len, out + 1, actualTag + 1, 16) == CRYPTO_STATUS_UNSUPPORTED);
	}
	puts("CCM: maximum supported AAD length matches OpenSSL; next length rejected");
}

static void CheckCmac()
{
	FailingAes engine;
	CryptoKey key{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_SIGN, {}};
	uint8_t k[16], msg[257], ref[16], out[18], overlap[257];
	size_t len, tagLen, resultLen, i;
	unsigned seed, fail, calls;

	engine.Enable();
	key.Plain.pData = k;
	key.Plain.Len = sizeof(k);
	for (seed = 0; seed < 4; seed++)
	{
		for (i = 0; i < sizeof(k); i++) k[i] = (uint8_t)(31U * i + seed);
		for (i = 0; i < sizeof(msg); i++) msg[i] = (uint8_t)(17U * i + seed);
		for (len = 0; len <= sizeof(msg); len++)
		{
			assert(EVP_Q_mac(nullptr, "CMAC", nullptr, "AES-128-CBC", nullptr,
				k, sizeof(k), msg, len, ref, sizeof(ref), &resultLen) == ref);
			assert(resultLen == 16);
			for (tagLen = 1; tagLen <= 16; tagLen++)
			{
				memset(out, 0xA5, sizeof(out));
				assert(engine.Mac(CRYPTO_MAC_CMAC, key, len ? msg : nullptr, len,
					out + 1, tagLen) == CRYPTO_STATUS_OK);
				assert(memcmp(out + 1, ref, tagLen) == 0);
				assert(out[0] == 0xA5 && out[tagLen + 1] == 0xA5);
			}
			memcpy(overlap, msg, sizeof(msg));
			assert(engine.Mac(CRYPTO_MAC_CMAC, key, overlap, len, overlap, 16) == CRYPTO_STATUS_OK);
			assert(memcmp(overlap, ref, 16) == 0);
		}
	}
	for (len = 0; len <= sizeof(msg); len++)
	{
		calls = 1U + (len ? (len + 15U) / 16U : 1U);
		for (fail = 1; fail <= calls; fail++)
		{
			engine.Blocks = engine.Begins = engine.Ends = 0;
			engine.FailAt = fail;
			memset(out, 0xA5, sizeof(out));
			assert(engine.Mac(CRYPTO_MAC_CMAC, key, msg, len, out + 1, 16) == CRYPTO_STATUS_FAIL);
			assert(engine.Blocks == fail && engine.Begins == 1 && engine.Ends == 1);
			for (i = 1; i <= 16; i++) assert(out[i] == 0);
			assert(out[0] == 0xA5 && out[17] == 0xA5);
		}
	}
	engine.Refuse = true;
	engine.Blocks = engine.Begins = engine.Ends = 0;
	memset(out, 0xA5, sizeof(out));
	assert(engine.Mac(CRYPTO_MAC_CMAC, key, msg, sizeof(msg), out + 1, 16) == CRYPTO_STATUS_BUSY);
	assert(engine.Blocks == 0 && engine.Begins == 1 && engine.Ends == 0);
	for (i = 1; i <= 16; i++) assert(out[i] == 0);
	assert(out[0] == 0xA5 && out[17] == 0xA5);
	puts("CMAC: 16512 OpenSSL comparisons, overlap, bounds, every AES failure and busy passed");
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

	CheckCmac();
	CheckCcmAadLimit();
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
