#include "esp_model.h"
#include "esp32xx.h"
#include "crypto/crypto.h"
#include <openssl/evp.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

static EspReg s_Registers[50];
static uint32_t s_Values[50];
static unsigned s_Count, s_Blocks;
static bool s_Timeout;

EspReg &EspRegister(uintptr_t address)
{
	for (unsigned i = 0; i < s_Count; i++) {
		if (s_Registers[i].address == address) return s_Registers[i];
	}
	assert(s_Count < 50);
	s_Registers[s_Count].address = address;
	return s_Registers[s_Count++];
}
EspReg::operator uint32_t() const
{
	if (address == ESP32_AES_BASE + 0x4C) return s_Timeout ? 1U : 0U;
	return s_Values[this - s_Registers];
}
EspReg &EspReg::operator=(uint32_t value)
{
	s_Values[this - s_Registers] = value;
	if (address == ESP32_AES_BASE + 0x48 && value == 1) {
		uint8_t key[32] = {}, input[16], output[32];
		uint32_t mode = EspRegister(ESP32_AES_BASE + 0x40);
		int n = 0, final = 0;
#if defined(ESP32C3)
		assert((EspRegister(ESP32_SYSTEM_BASE + 0x14) & 2U) != 0);
		assert((EspRegister(ESP32_SYSTEM_BASE + 0x1C) & 18U) == 0);
#else
		assert((EspRegister(ESP32_PCR_BASE + 0xC8) & 3U) == 1);
		assert((EspRegister(ESP32_PCR_BASE + 0xE0) & 2U) == 0);
#endif
		assert(EspRegister(ESP32_AES_BASE + 0x44) == 0);
		assert(EspRegister(ESP32_AES_BASE + 0x90) == 0);
		for (unsigned i = 0; i < ((mode & 2U) ? 32U : 16U); i += 4) {
			uint32_t word = EspRegister(ESP32_AES_BASE + i);
			memcpy(key + i, &word, 4);
		}
		for (unsigned i = 0; i < 16; i += 4) {
			uint32_t word = EspRegister(ESP32_AES_BASE + 0x20 + i);
			memcpy(input + i, &word, 4);
		}
		EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
		assert(ctx);
		assert(EVP_CipherInit_ex(ctx, (mode & 2) ? EVP_aes_256_ecb() : EVP_aes_128_ecb(),
			nullptr, key, nullptr, (mode & 4) == 0) == 1);
		assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
		assert(EVP_CipherUpdate(ctx, output, &n, input, 16) == 1);
		assert(EVP_CipherFinal_ex(ctx, output + n, &final) == 1);
		EVP_CIPHER_CTX_free(ctx);
		for (unsigned i = 0; i < 16; i += 4) {
			uint32_t word;
			memcpy(&word, output + i, 4);
			EspRegister(ESP32_AES_BASE + 0x30 + i) = word;
		}
		s_Blocks++;
	}
	return *this;
}
extern int CryptoEngineTest();
int main()
{
	Crypto crypto;
	uint8_t key[16] = {}, input[16] = {}, output[18];
	CryptoKey k = {};
	assert(CryptoEngineTest() == 0);
	assert(s_Blocks > 1000);
	assert(crypto.Init());
	k.Type = CRYPTO_KEY_AES_128;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = sizeof(key);
	memset(output, 0xA5, sizeof(output));
	s_Timeout = true;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output+1) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 1; i < 17; i++) assert(output[i] == 0);
	assert(output[0] == 0xA5 && output[17] == 0xA5);
	for (unsigned i = 0; i < 16; i++) assert(EspRegister(ESP32_AES_BASE + i*4) == 0);
	s_Timeout = false;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, output+1) == CRYPTO_STATUS_OK);
	printf("ESP register model: %u AES blocks; timeout/output bounds and cleanup passed\n", s_Blocks);
}
