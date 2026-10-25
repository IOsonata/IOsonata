#include "sam_model.h"
#include "crypto/crypto.h"
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

SamAes g_Aes;
SamPm g_Pm = {};
SamScif g_Scif = {};
SamRng g_Rng;
SamPmc g_Pmc = {};
bool g_AesTimeout = false, g_RngTimeout = false;
unsigned g_AesBlocks = 0;
static uint8_t s_Key[32], s_Input[16], s_Output[16];
static unsigned s_In, s_Out;
static bool s_RngEnabled, s_RngReady;

SamAes::SamAes()
{
	memset(this, 0, sizeof(*this));
	AES_CR.id = 1;
	AESA_CTRL.id = 2;
	AESA_IDATA.id = 3;
	AESA_ODATA.id = 4;
	AES_ISR.id = 5;
	AESA_SR.id = 6;
	for (unsigned i = 0; i < 8; i++) {
		AES_KEYWR[i].id = 20 + i;
		AESA_KEY[i].AESA_KEY.id = 20 + i;
	}
	for (unsigned i = 0; i < 4; i++) {
		AES_IDATAR[i].id = 30 + i;
		AES_ODATAR[i].id = 40 + i;
	}
}
SamRng::SamRng()
{
	memset(this, 0, sizeof(*this));
	TRNG_ISR.id = 7;
	TRNG_ODATA.id = 8;
	TRNG_CR.id = 10;
}
static void RunBlock(bool sam4l)
{
	int length = 0, final = 0;
	uint32_t mode = sam4l ? g_Aes.AESA_MODE.value : g_Aes.AES_MR.value;
	EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
	assert(ctx);
	const EVP_CIPHER *cipher = (!sam4l && (mode & AES_MR_KEYSIZE_AES256)) ?
		EVP_aes_256_ecb() : EVP_aes_128_ecb();
	assert(EVP_CipherInit_ex(ctx, cipher, nullptr, s_Key, nullptr, mode & 1U) == 1);
	assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
	assert(EVP_CipherUpdate(ctx, s_Output, &length, s_Input, 16) == 1);
	assert(EVP_CipherFinal_ex(ctx, s_Output + length, &final) == 1);
	EVP_CIPHER_CTX_free(ctx);
	g_AesBlocks++;
	s_Out = 0;
}
SamReg &SamReg::operator=(uint32_t v)
{
	value = v;
	if ((id == 1 || id == 2) && (v & 256U)) {
		s_In = s_Out = 0;
		memset(s_Key, 0, sizeof(s_Key));
		memset(s_Input, 0, sizeof(s_Input));
		memset(s_Output, 0, sizeof(s_Output));
	}
	if (id >= 20 && id < 28) {
		memcpy(s_Key + 4 * (id - 20), &v, 4);
	}
	if (id == 10) {
		s_RngEnabled = (v & TRNG_CR_ENABLE) != 0U &&
			(v & 0xFFFFFF00U) == TRNG_CR_KEY(0x524E47U);
		s_RngReady = s_RngEnabled;
	}
	if (id >= 30 && id < 34) {
		memcpy(s_Input + 4 * (id - 30), &v, 4);
	}
	if (id == 3) {
		assert((g_Aes.AESA_CTRL.value & AESA_CTRL_ENABLE) != 0U);
		assert((g_Pm.PM_HSBMASK & PM_HSBMASK_AESA) != 0U);
		assert((g_Scif.SCIF_GCCTRL[4].SCIF_GCCTRL.value &
			(SCIF_GCCTRL_OSCSEL_Msk | SCIF_GCCTRL_CEN)) ==
			(SCIF_GCCTRL_OSCSEL(0U) | SCIF_GCCTRL_CEN));
		assert(s_In < 4);
		memcpy(s_Input + 4 * s_In++, &v, 4);
		if (s_In == 4) {
			RunBlock(true);
		}
	}
	if (id == 1 && (v & 1U)) {
		RunBlock(false);
	}
	return *this;
}
SamReg::operator uint32_t() const
{
	uint32_t v = 0;
	bool clocked;

	if (id == 4) {
		assert(s_Out < 4);
		memcpy(&v, s_Output + 4 * s_Out++, 4);
		return v;
	}
	if (id >= 40 && id < 44) {
		memcpy(&v, s_Output + 4 * (id - 40), 4);
		return v;
	}
	if (id == 5) return g_AesTimeout ? 0U : 1U;
	if (id == 6) return g_AesTimeout ? 0U : 0x10001U;
	if (id == 7) {
		clocked = (g_Pm.PM_HSBMASK & PM_HSBMASK_HTOP0) != 0U &&
			(g_Pm.PM_PBAMASK & PM_PBAMASK_TRNG) != 0U;
		if (g_RngTimeout || !s_RngEnabled || !s_RngReady || !clocked) {
			return 0U;
		}
		s_RngReady = false;
		return TRNG_ISR_DATRDY;
	}
	if (id == 8) {
		assert(s_RngEnabled);
		assert(!s_RngReady);
		assert(RAND_bytes(reinterpret_cast<unsigned char *>(&v), 4) == 1);
		s_RngReady = true;
		return v;
	}
	return value;
}
extern int CryptoEngineTest();
int main()
{
	Crypto crypto;
	uint8_t key[16] = {}, in[16] = {}, out[18];
	CryptoKey k = {};
#ifdef TEST_SAM4L
	g_RngTimeout = true;
	assert(!crypto.Init());
	assert(!crypto.IsSecure());
	g_RngTimeout = false;
#endif
	assert(CryptoEngineTest() == 0);
	assert(g_AesBlocks > 1000U);
	assert(crypto.Init());
	k.Type = CRYPTO_KEY_AES_128;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = sizeof(key);
	memset(out, 0xA5, sizeof(out));
	g_AesTimeout = true;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, out+1) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 1; i < 17; i++) assert(out[i] == 0);
	assert(out[0] == 0xA5 && out[17] == 0xA5);
	g_AesTimeout = false;
	assert(crypto.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, in, 16, out+1) == CRYPTO_STATUS_OK);
#ifdef TEST_SAM4L
	g_RngTimeout = true;
	memset(out, 0xA5, sizeof(out));
	assert(crypto.Random(out+1, 3) == CRYPTO_STATUS_FAIL);
	assert(out[0] == 0xA5 && out[4] == 0xA5);
	assert(out[1] == 0 && out[2] == 0 && out[3] == 0);
#endif
	printf("SAM register model: %u AES blocks; timeout/output bounds passed\n", g_AesBlocks);
}
