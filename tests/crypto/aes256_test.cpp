// Independent AES-256 checks and hardware override inheritance.
#include <cassert>
#include <cstring>
#include <cstdio>
#include <openssl/evp.h>
#include "crypto/crypto.h"
#include "crypto/crypto_softaes.h"

static CRYPTO_STATUS s_TargetStatus = CRYPTO_STATUS_OK;
static unsigned s_TargetCalls;
static unsigned s_BlockCalls;

// Model a hardware override inheriting software support for wider keys.
class Aes128Target : public CryptoSoftAes {
public:
	bool Enable() override { vbValid = true; return true; }
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int Encrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut) override
	{
		s_TargetCalls++;
		if (s_TargetStatus != CRYPTO_STATUS_OK)
		{
			return s_TargetStatus;
		}
		return CryptoSoftAes::Cipher(Alg, Encrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}
};

// Models the inheritance used by the nRF5340 ECB driver. Wide keys must
// never acquire or call the AES-128-only block hooks.
class Aes128Hooks : public CryptoSoftAes {
protected:
	bool AesOpBegin() override { s_BlockCalls++; return false; }
	void AesOpEnd() override { s_BlockCalls++; }
	bool AesEcbEncrypt(const uint8_t *, const uint8_t *, uint8_t *) override
	{
		s_BlockCalls++;
		return false;
	}
};

class AesFaultHooks : public CryptoSoftAes {
public:
	unsigned Begins = 0;
	unsigned Ends = 0;
	unsigned Blocks = 0;
	unsigned FailBlock = 0;
	bool Busy = false;
protected:
	bool AesOpBegin() override { Begins++; return !Busy; }
	void AesOpEnd() override { Ends++; }
	bool AesEcbEncrypt(const uint8_t *pKey, const uint8_t *pIn, uint8_t *pOut) override
	{
		Blocks++;
		return Blocks != FailBlock && CryptoSoftAes::AesEcbEncrypt(pKey, pIn, pOut);
	}
};

static Aes128Target hardware;

CRYPTO_STATUS Crypto::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return hardware.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
}

bool Crypto::Init()
{

	assert(hardware.Enable());
	vbValid = true;
	return true;
}

static void Reference(CRYPTO_CIPHER_ALG Alg, bool Encrypt, const uint8_t *pKey,
		const uint8_t *pIv, const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
	const EVP_CIPHER *cipher = Alg == CRYPTO_CIPHER_ECB ? EVP_aes_256_ecb() :
		(Alg == CRYPTO_CIPHER_CBC ? EVP_aes_256_cbc() : EVP_aes_256_ctr());
	int count;
	int tail;

	assert(ctx != nullptr);
	assert(EVP_CipherInit_ex(ctx, cipher, nullptr, pKey, pIv, Encrypt));
	assert(EVP_CIPHER_CTX_set_padding(ctx, 0));
	assert(EVP_CipherUpdate(ctx, pOut, &count, pIn, (int)Len));
	assert(EVP_CipherFinal_ex(ctx, pOut + count, &tail));
	assert((size_t)(count + tail) == Len);
	EVP_CIPHER_CTX_free(ctx);
}

int main(void)
{
	AesFaultHooks fault;
	Crypto crypto;
	CryptoSoftAes software;
	Aes128Hooks inherited;
	CipherEngine *engines[] = {&crypto, &software, &inherited};
	CRYPTO_CIPHER_ALG modes[] = {CRYPTO_CIPHER_ECB, CRYPTO_CIPHER_CBC, CRYPTO_CIPHER_CTR};
	uint8_t key[33], iv[16], input[258], expected[272], output[258];
	CryptoKey k{CRYPTO_KEY_AES_256, CRYPTO_KEY_LOC_PLAIN,
		CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {{key + 1, 32}}};
	uint32_t seed = 0x2565340;
	size_t len;
	unsigned calls;

	assert(crypto.Init() && software.Enable() && inherited.Enable());
	for (unsigned round = 0; round < 200; round++)
	{
		for (uint8_t &b : key)
		{
			seed = seed * 1664525U + 1013904223U;
			b = seed >> 24;
		}
		for (uint8_t &b : input)
		{
			seed = seed * 1664525U + 1013904223U;
			b = seed >> 24;
		}
		// Exercise counter carry and wrap as well as ordinary IVs.
		memset(iv, 0xff, sizeof(iv));
		iv[15] = (uint8_t)round;
		for (CRYPTO_CIPHER_ALG mode : modes)
		{
			len = mode == CRYPTO_CIPHER_CTR ? round % 257U : (round % 17U) * 16U;
			for (int enc = 0; enc < 2; enc++)
			{
				Reference(mode, enc, key + 1, iv, input + 1, len, expected);
				for (CipherEngine *engine : engines)
				{
					memset(output, 0xa5, sizeof(output));
					assert(engine->Cipher(mode, enc, k, iv, 16, input + 1, len, output + 1) == CRYPTO_STATUS_OK);
					assert(memcmp(output + 1, expected, len) == 0);
					assert(output[0] == 0xa5 && output[len + 1] == 0xa5);
					memcpy(output + 1, input + 1, len);
					assert(engine->Cipher(mode, enc, k, iv, 16, output + 1, len, output + 1) == CRYPTO_STATUS_OK);
					assert(memcmp(output + 1, expected, len) == 0);
					assert(output[0] == 0xa5 && output[len + 1] == 0xa5);
				}
			}
		}
	}
	assert(s_BlockCalls == 0);
	calls = s_TargetCalls;
	s_TargetStatus = CRYPTO_STATUS_BUSY;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_BUSY);
	s_TargetStatus = CRYPTO_STATUS_FAIL;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_FAIL);
	s_TargetStatus = CRYPTO_STATUS_UNSUPPORTED;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_UNSUPPORTED);
	assert(s_TargetCalls == calls + 3);
	puts("AES-256: 200 rounds against OpenSSL; software base, inherited hooks and hardware override PASS");
	puts("Hardware overrides retain software behavior; busy and failed operations do not retry PASS");
	k.Type = CRYPTO_KEY_AES_128;
	k.Plain.Len = 16;
	for (CRYPTO_CIPHER_ALG mode : modes)
	{
		len = mode == CRYPTO_CIPHER_CTR ? 33U : 48U;
		for (int enc = 0; enc < 2; enc++)
		{
			for (unsigned fail = 0; fail <= 3; fail++)
			{
				fault.Begins = fault.Ends = fault.Blocks = 0;
				fault.Busy = fail == 0;
				fault.FailBlock = fail;
				memset(output, 0xa5, sizeof(output));
				memcpy(output + 1, input + 1, len);
				if (enc != 0 || mode == CRYPTO_CIPHER_CTR)
				{
					assert(fault.Cipher(mode, enc, k, iv, 16, output + 1, len, output + 1) ==
						(fail == 0 ? CRYPTO_STATUS_BUSY : CRYPTO_STATUS_FAIL));
					assert(fault.Begins == 1 && fault.Ends == (fail != 0));
					assert(fault.Blocks == fail);
					for (size_t i = 1; i <= len; i++)
					{
						assert(output[i] == 0);
					}
				}
				else
				{
					assert(fault.Cipher(mode, enc, k, iv, 16, output + 1, len, output + 1) == CRYPTO_STATUS_OK);
					assert(fault.Begins == 0 && fault.Ends == 0 && fault.Blocks == 0);
				}
				assert(output[0] == 0xa5 && output[len + 1] == 0xa5);
			}
		}
	}
	puts("AES-128 mode failures: acquisition, every block, in-place wiping and bounds PASS");
	return 0;
}
