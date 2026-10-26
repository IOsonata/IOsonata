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

RaSystem g_System = {};
RaMstp g_Mstp = {0xFFFFFFFFU};
static bool s_Fail;
static unsigned s_Blocks;

static bool s_Open;
static bool s_Encrypt;
static size_t s_KeyLen;
static uint8_t s_Key[32];
static unsigned s_Begins;

bool Sce5Init(void) { return true; }
bool Sce5Ready(void) { return !s_Fail; }
bool Sce5RandomWords(uint32_t out[4])
{
	if (s_Fail) return false;
	// A draw inside an AES session is a resource conflict in the engine.
	assert(!s_Open);
	return RAND_bytes(reinterpret_cast<unsigned char *>(out), 16) == 1;
}
bool Sce5AesBegin(const uint8_t *key, size_t keyLen, bool encrypt)
{
	if (s_Fail) return false;
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

	if (s_Fail) return false;
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
	s_Open = false;
	memset(s_Key, 0, sizeof(s_Key));
	return !s_Fail;
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
	k.Type = CRYPTO_KEY_AES_128;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = sizeof(key);
	s_Fail = true;
	memset(out, 0xA5, sizeof(out));
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, out+1) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 1; i < 17; i++) assert(out[i] == 0);
	assert(out[0] == 0xA5 && out[17] == 0xA5);
	memset(out, 0xA5, sizeof(out));
	assert(crypto.Random(out+1, 3) == CRYPTO_STATUS_FAIL);
	assert(out[0] == 0xA5 && out[4] == 0xA5);
	assert(out[1] == 0 && out[2] == 0 && out[3] == 0);
	assert(!crypto.IsSecure());
	printf("RA4 engine model: %u AES blocks in %u sessions; failure/output bounds passed\n", s_Blocks, s_Begins);
}
