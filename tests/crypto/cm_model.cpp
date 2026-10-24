#include "crypto/cryptomaster.h"
#include "crypto/crypto.h"
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <cassert>
#include <cstring>
#include <cstdio>

static uint32_t s_Fetch, s_Push;
static unsigned s_Blocks;
static bool s_Timeout, s_Stuck, s_StuckOnStart;
extern "C" void usDelay(uint32_t) {}

class ModelIntrf : public DeviceIntrf {
public:
	operator DevIntrf_t *() override { return nullptr; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate() override { return 0; }
	void Enable() override {}
	void Disable() override {}
	bool StartTx(uint32_t) override { return true; }
	void StopTx() override {}
	void Reset() override {}
};
static ModelIntrf s_Intrf;
class ModelCm : public CryptoMaster {
public:
	uint32_t Read32(uint8_t *addr, int len) override
	{
		uint32_t offset;
		assert(len == 4);
		memcpy(&offset, addr, 4);
		if (offset == CRYPTOMASTER_HW_PRESENCE) return CRYPTOMASTER_PRESENT_AES;
		if (offset == CRYPTOMASTER_STATUS)
			return s_Stuck ? CRYPTOMASTER_STATUS_RESET_BUSY | CRYPTOMASTER_STATUS_BUSY :
				s_Timeout ? CRYPTOMASTER_STATUS_BUSY : 0;
		return 0;
	}
	bool Write32(uint8_t *addr, int len, uint32_t value) override
	{
		uint32_t offset;
		assert(len == 4);
		memcpy(&offset, addr, 4);
		if (offset == CRYPTOMASTER_FETCH_ADDR) s_Fetch = value;
		if (offset == CRYPTOMASTER_PUSH_ADDR) s_Push = value;
		if (offset == CRYPTOMASTER_START && s_StuckOnStart) s_Stuck = true;
		if (offset == CRYPTOMASTER_START && !s_Timeout && !s_Stuck) {
			auto *in = reinterpret_cast<CryptoMasterDesc_t *>((uintptr_t)s_Fetch);
			auto *out = reinterpret_cast<CryptoMasterDesc_t *>((uintptr_t)s_Push);
			uint32_t cfg;
			int n, final;
			memcpy(&cfg, in[0].pAddr, 4);
			unsigned keyLen = in[1].Size & ~CRYPTOMASTER_DMA_REALIGN;
			assert(keyLen == 16 || keyLen == 32);
			assert((cfg & 4) == (keyLen == 32 ? 4U : 0U));
			assert(in[0].pNext == in+1 && in[1].pNext == in+2);
			assert(in[2].pNext == CRYPTOMASTER_DMA_STOP);
			assert(out->pNext == CRYPTOMASTER_DMA_STOP);
			assert(in[2].pAddr != out->pAddr);
			EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
			assert(ctx);
			assert(EVP_CipherInit_ex(ctx, keyLen == 32 ? EVP_aes_256_ecb() : EVP_aes_128_ecb(),
				nullptr, in[1].pAddr, nullptr, !(cfg & 1)) == 1);
			assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
			assert(EVP_CipherUpdate(ctx, out->pAddr, &n, in[2].pAddr, 16) == 1);
			assert(EVP_CipherFinal_ex(ctx, out->pAddr+n, &final) == 1);
			EVP_CIPHER_CTX_free(ctx);
			s_Blocks++;
		}
		return true;
	}
};
static ModelCm s_Cm;
class ModelCrypto : public Crypto {
public:
	bool Enable() override { vbValid = s_Cm.Init(&s_Intrf); return vbValid; }
	bool IsSecure() const override { return true; }
	CRYPTO_STATUS Random(uint8_t *out, size_t len) override
	{ return RAND_bytes(out, len) == 1 ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL; }
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG a, int e, const CryptoKey &k, const uint8_t *iv,
		size_t il, const uint8_t *in, size_t n, uint8_t *out) override
	{ return s_Cm.Cipher(a,e,k,iv,il,in,n,out); }
	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG a, const CryptoKey &k, const uint8_t *iv, size_t il,
		const uint8_t *ad, size_t al, const uint8_t *in, size_t n, uint8_t *out, uint8_t *tag, size_t tl) override
	{ return s_Cm.Seal(a,k,iv,il,ad,al,in,n,out,tag,tl); }
	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG a, const CryptoKey &k, const uint8_t *iv, size_t il,
		const uint8_t *ad, size_t al, const uint8_t *in, size_t n, uint8_t *out, const uint8_t *tag, size_t tl) override
	{ return s_Cm.Open(a,k,iv,il,ad,al,in,n,out,tag,tl); }
	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG a, const CryptoKey &k, const uint8_t *in, size_t n, uint8_t *out, size_t ol) override
	{ return a == CRYPTO_MAC_CMAC ? s_Cm.Mac(a,k,in,n,out,ol) : Crypto::Mac(a,k,in,n,out,ol); }
};
bool Crypto::Init()
{
	static ModelCrypto hardware;
	if (vbValid) return true;
	if (!hardware.Enable()) return false;
	vpHardware = &hardware;
	vbValid = true;
	return true;
}
extern int CryptoEngineTest();
int main()
{
	Crypto c;
	ModelCm other;
	uint8_t key[32] = {}, input[16] = {}, out[18];
	CryptoKey k = {};
	assert(CryptoEngineTest() == 0);
	assert(s_Blocks > 1000);
	assert(c.Init());
	assert(other.Init(&s_Intrf));
	k.Type = CRYPTO_KEY_AES_256;
	k.Loc = CRYPTO_KEY_LOC_PLAIN;
	k.Usage = CRYPTO_KEY_USE_ENCRYPT;
	k.Plain.pData = key;
	k.Plain.Len = sizeof(key);
	assert(other.OpAcquire());
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_BUSY);
	other.OpRelease();
	s_Timeout = true;
	memset(out, 0xA5, sizeof(out));
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_FAIL);
	assert(out[0] == 0xA5 && out[17] == 0xA5);
	for (unsigned i = 1; i <= 16; i++) assert(out[i] == 0);
	s_Timeout = false;
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_OK);
	s_StuckOnStart = true;
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_FAIL);
	// A late DMA write after the failed call must only reach retained scratch.
	auto *pending = reinterpret_cast<CryptoMasterDesc_t *>((uintptr_t)s_Push);
	assert(pending->pAddr != out+1);
	assert(pending->pNext == CRYPTOMASTER_DMA_STOP);
	memset(pending->pAddr, 0xCC, 16);
	for (unsigned i = 1; i <= 16; i++) assert(out[i] == 0);
	s_StuckOnStart = false;
	s_Stuck = false;
	assert(other.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_FAIL);
	s_Cm.Reset();
	assert(c.Cipher(CRYPTO_CIPHER_ECB, 1, k, nullptr, 0, input, 16, out+1) == CRYPTO_STATUS_OK);
	printf("CryptoMaster: %u DMA blocks, cross-instance exclusion, timeout and quarantine recovery passed\n", s_Blocks);
}
