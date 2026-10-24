#include "nrf_ecb_model.h"
#include "crypto_ecb_nrf53.h"
#include <openssl/evp.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
NRF_ECB_Type g_Ecb;
static bool s_SoftDevice, s_Fail, s_StopFail;
static unsigned s_DmaCalls, s_SvcCalls;
static void Encrypt(uint8_t *block)
{
	int n, final;
	EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
	assert(ctx);
	assert(EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, block, nullptr) == 1);
	assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
	assert(EVP_EncryptUpdate(ctx, block+32, &n, block+16, 16) == 1);
	assert(EVP_EncryptFinal_ex(ctx, block+32+n, &final) == 1);
	EVP_CIPHER_CTX_free(ctx);
}
EcbReg &EcbReg::operator=(uintptr_t v)
{
	value = v;
	if (id == 1 && v == 1) {
		s_DmaCalls++;
		if (!s_Fail) {
			Encrypt(reinterpret_cast<uint8_t *>(g_Ecb.ECBDATAPTR.value));
			g_Ecb.EVENTS_ENDECB = 1;
		}
	}
	if (id == 2 && v == 1 && !s_StopFail) g_Ecb.EVENTS_ERRORECB = 1;
	return *this;
}
uint32_t sd_softdevice_is_enabled(uint8_t *enabled) { *enabled = s_SoftDevice; return 0; }
uint32_t sd_ecb_block_encrypt(nrf_ecb_hal_data_t *data)
{
	s_SvcCalls++;
	if (s_Fail) return 1;
	Encrypt(reinterpret_cast<uint8_t *>(data));
	return 0;
}
int main()
{
	CryptoEcbNrf53 crypto;
	uint8_t key[16] = {}, input[16] = {}, output[16];
	const uint8_t expected[16] = {0x66,0xe9,0x4b,0xd4,0xef,0x8a,0x2c,0x3b,0x88,0x4c,0xfa,0x59,0xca,0x34,0x2b,0x2e};
	CryptoKey k = {};
	assert(crypto.Enable());
	k.Type = CRYPTO_KEY_AES_128;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = 16;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_OK);
	assert(memcmp(output, expected, 16) == 0 && s_DmaCalls == 1);
#ifdef SOFTDEVICE_PRESENT
	s_SoftDevice = true;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_OK);
	assert(memcmp(output, expected, 16) == 0 && s_DmaCalls == 1 && s_SvcCalls == 1);
	s_Fail = true;
	memset(output, 0xA5, 16);
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_FAIL);
	assert(s_DmaCalls == 1 && s_SvcCalls == 2);
	for (auto b : output) assert(b == 0);
	s_SoftDevice = false;
#endif
	s_Fail = true;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_FAIL);
	for (auto b : output) assert(b == 0);
	s_Fail = false;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_OK);
	s_Fail = s_StopFail = true;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_FAIL);
	unsigned calls = s_DmaCalls;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output) == CRYPTO_STATUS_BUSY);
	assert(s_DmaCalls == calls);
	puts("ECB model: known answer, SoftDevice routing, failure wiping and unresolved DMA lock passed");
}
