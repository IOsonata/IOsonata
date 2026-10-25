// Host register models for the production CC3xx and BA414EP drivers.
// OpenSSL supplies arithmetic only; driver sequencing and key handling run unchanged.
// These models do not validate silicon timing or hardware countermeasures.
#include <cassert>
#include <cstring>
#include <type_traits>
#define OPENSSL_SUPPRESS_DEPRECATED
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <openssl/bn.h>
#include <openssl/ec.h>
#include <openssl/obj_mac.h>
#include <openssl/rand.h>
#include "crypto_cc3xx.h"
#include "crypto/ba414ep.h"
#include "crypto/crypto_stm32wba.h"
#include "crypto_rng_nrf.h"
#ifdef CRYPTO_LINK_TEST
#include "crypto/crypto.h"
#else
#include "../../exemples/crypto/crypto_engine_test.cpp"
#endif

static_assert(std::is_base_of<CryptoUecc, CryptoCc3xx>::value);
static_assert(std::is_base_of<CryptoUecc, Ba414ep>::value);
static_assert(!std::is_base_of<CryptoUecc, CryptoWbaPka>::value);
static_assert(std::is_base_of<KeyAgreeEngine, CryptoWbaPka>::value);
static_assert(std::is_base_of<SignEngine, CryptoWbaPka>::value);
static_assert(std::is_same<CryptoUecc::KeyCtx, CryptoCc3xx::KeyCtx>::value);
static_assert(std::is_same<CryptoUecc::KeyCtx, Ba414ep::KeyCtx>::value);
static_assert(std::is_same<CryptoUecc::KeyCtx, CryptoWbaPka::KeyCtx>::value);

static uint32_t Word(const uint8_t *p)
{
	uint32_t v;
	memcpy(&v, p, sizeof(v));
	return v;
}

class CcModel : public Cc3xxIntrf {
public:
	uint32_t Reg[0x1000 / 4] = {};
	uint8_t Input[64] = {}, Output[16] = {};
	unsigned In = 0, Out = 16, AesBlocks = 0, HashBlocks = 0;
	uint32_t Ram[1024] = {};
	bool Held = false;
	bool Busy = false;
	bool FailCleanup = false;
	bool ResultRead = false;
	bool Fault = false;
	unsigned Operations = 0;
	unsigned FailOperation = 0;
	DevIntrf_t Intrf = {};

	operator DevIntrf_t *() override { return &Intrf; }
	DEVINTRF_TYPE Type() override { return DEVINTRF_TYPE_CRYPTO; }
	void Enable() override {}
	void Disable() override {}
	void Reset() override {}
	bool StartRx(uint32_t addr) override {
		assert(addr == CC3XX_ADDR_RNG);
		if (Held || Busy) return false;
		Held = true;
		return true;
	}
	int RxData(uint8_t *out, int len) override {
		assert(Held && len == 48);
		return RAND_bytes(out, len) == 1 ? len : 0;
	}
	void StopRx() override { Held = false; }

	BIGNUM *Operand(unsigned Index, bool Immediate)
	{
		BIGNUM *n = BN_new();
		if (Immediate)
		{
			int v = Index & 16U ? (int)Index - 32 : (int)Index;
			BN_set_word(n, v < 0 ? -v : v);
			BN_set_negative(n, v < 0);
		}
		else
		{
			assert(Reg[Index] + 12 <= 1024);
			BN_lebin2bn((uint8_t *)&Ram[Reg[Index]], 48, n);
		}
		return n;
	}

	void Execute(uint32_t Code)
	{
		BN_CTX *ctx = BN_CTX_new();
		BIGNUM *a = Operand((Code >> 18) & 31, Code & (1U << 23));
		BIGNUM *b = Operand((Code >> 12) & 31, Code & (1U << 17));
		BIGNUM *n = Operand(0, false);
		BIGNUM *r = BN_new();
		BIGNUM *limit = BN_new();
		uint8_t aa[48], bb[48];
		unsigned op = Code >> 27;
		unsigned dest = (Code >> 6) & 31;
		Operations++;
		if (Operations == FailOperation) Fault = true;
		switch (op)
		{
		case 4: assert(BN_add(r, a, b)); break;
		case 5: assert(BN_sub(r, a, b)); break;
		case 6: assert(BN_mod_add(r, a, b, n, ctx)); break;
		case 7: assert(BN_mod_sub(r, a, b, n, ctx)); break;
		case 9: assert(BN_copy(r, a)); break;
		case 8:
		case 10:
			assert(BN_bn2lebinpad(a, aa, 48) == 48);
			assert(BN_bn2lebinpad(b, bb, 48) == 48);
			for (unsigned i = 0; i < 48; i++)
			{
				aa[i] = op == 8 ? aa[i] & bb[i] : aa[i] ^ bb[i];
			}
			BN_lebin2bn(aa, 48, r);
			break;
		case 17: assert(BN_mod_mul(r, a, b, n, ctx)); break;
		case 19: assert(BN_mod_exp(r, a, b, n, ctx)); break;
		default: assert(false);
		}
		Reg[0x88 / 4] = (BN_is_negative(r) ? 1U << 8 : 0U) |
			(BN_is_zero(r) ? 1U << 12 : 0U);
		if (!(Code & (1U << 11)))
		{
			BN_one(limit);
			assert(BN_lshift(limit, limit, 384));
			assert(BN_nnmod(r, r, limit, ctx));
			assert(BN_bn2lebinpad(r, (uint8_t *)&Ram[Reg[dest]], 48) == 48);
		}
		BN_free(a); BN_free(b); BN_free(n); BN_free(r); BN_free(limit);
		BN_CTX_free(ctx);
	}

	int Read(uint32_t, const uint8_t *pAddr, int AddrLen,
		uint8_t *pData, int Len) override
	{
		uint32_t addr = Word(pAddr), v;
		assert(AddrLen == 4 && Len == 4 && addr < sizeof(Reg));
		if (addr == 0xB0 || addr == 0xB4) v = Fault ? 0 : 1;
		else if (addr == 0xDC)
		{
			v = Ram[Reg[0xE4 / 4]];
			ResultRead = true;
		}
		else if (addr == 0xC50) v = 1;
		else if (addr == 0xD50) v = Out >= 16;
		else if (addr == 0xD00)
		{
			assert(Out < 16);
			memcpy(&v, Output+Out, 4);
			Out += 4;
		}
		else v = Reg[addr / 4];
		memcpy(pData, &v, 4);
		return Len;
	}
	int Write(uint32_t, const uint8_t *pAddr, int AddrLen,
		const uint8_t *pData, int Len) override
	{
		uint32_t addr = Word(pAddr), v = Word(pData);
		assert(AddrLen == 4 && Len > 0 && Len % 4 == 0 && addr + Len <= sizeof(Reg));
		if (Len > 4)
		{
			for (int i = 0; i < Len; i += 4)
			{
				uint32_t next = addr + i;
				assert(Write(CC3XX_ADDR_REG, (uint8_t *)&next, 4, pData+i, 4) == 4);
			}
			return Len;
		}
		if (addr == 0x8C) { Fault = false; ResultRead = false; }
		if (addr == 0xD4 && v == 0 && ResultRead && FailCleanup) Fault = true;
		Reg[addr / 4] = v;
		if (addr == 0xD8) Ram[Reg[0xD4 / 4]++] = v;
		if (addr == 0x80) Execute(v);
		if (addr == 0xC58 || addr == 0xC48) In = 0;
		if (addr == 0xC00)
		{
			assert(In < 64);
			memcpy(Input+In, &v, 4);
			In += 4;
			if (Reg[0x900/4] == 1 && In == 16)
			{
				uint32_t control = Reg[0x4C0/4];
				int n, final;
				EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
				assert(ctx);
				assert(EVP_CipherInit_ex(ctx, (control & 0x2000) ? EVP_aes_256_ecb() : EVP_aes_128_ecb(),
					nullptr, (uint8_t *)&Reg[0x400/4], nullptr, !(control & 1)) == 1);
				assert(EVP_CIPHER_CTX_set_padding(ctx, 0) == 1);
				assert(EVP_CipherUpdate(ctx, Output, &n, Input, 16) == 1);
				assert(EVP_CipherFinal_ex(ctx, Output+n, &final) == 1);
				EVP_CIPHER_CTX_free(ctx);
				Out = 0;
				AesBlocks++;
			}
			if (Reg[0x900/4] == 7 && In == 64)
			{
				SHA256_CTX ctx;
				assert(SHA256_Init(&ctx) == 1);
				memcpy(ctx.h, &Reg[0x640/4], 32);
				SHA256_Transform(&ctx, Input);
				memcpy(&Reg[0x640/4], ctx.h, 32);
				HashBlocks++;
			}
		}
		return Len;
	}
};

static CcModel s_Cc;
// This arithmetic model replaces the register transport; run_ccintrf.py
// separately exercises the production C and C++ transport against MMIO storage.
int Cc3xxIntrf::Read(uint32_t dev, const uint8_t *addr, int alen, uint8_t *out, int len)
{
	return s_Cc.Read(dev, addr, alen, out, len);
}
int Cc3xxIntrf::Write(uint32_t dev, const uint8_t *addr, int alen, const uint8_t *in, int len)
{
	return s_Cc.Write(dev, addr, alen, in, len);
}

extern "C" bool Cc3xxEnable(void) { return true; }
extern "C" void Cc3xxDisable(void) { memset(s_Cc.Reg, 0, sizeof(s_Cc.Reg)); }
Cc3xxIntrf *Cc3xxIntrfInstance(void) { return &s_Cc; }
bool Cc3xxIntrf::OpHold(const void *)
{
	if (s_Cc.Held || s_Cc.Busy) return false;
	s_Cc.Held = true;
	return true;
}
bool Cc3xxIntrf::OpRelease(const void *) { s_Cc.Held = false; return true; }

// Production RNG uses the entropy transfer above and production CryptoCell SHA.
#ifndef CRYPTO_LINK_TEST
class BaModel : public DeviceIntrf {
public:
	bool DenyReset = false;
	bool StartTx(uint32_t) override { return !DenyReset; }
	void StopTx() override {}
	void Reset() override {}
	DevIntrf_t Intrf = {};
	uint8_t Ram[16 * 512] = {};
	uint32_t Command = 0, Status = 0;
	unsigned Operations = 0;
	operator DevIntrf_t *() override { return &Intrf; }
	DEVINTRF_TYPE Type() override { return DEVINTRF_TYPE_CRYPTO; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate() override { return 0; }
	void Enable() override {}
	void Disable() override {}
	uint8_t *Slot(unsigned n) { return Ram + (n + 1) * 512 - 32; }
	void Execute()
	{
		BN_CTX *ctx = BN_CTX_new();
		EC_GROUP *group = EC_GROUP_new_by_curve_name(NID_X9_62_prime256v1);
		EC_POINT *point = EC_POINT_new(group);
		BIGNUM *x = BN_bin2bn(Slot(2), 32, nullptr);
		BIGNUM *y = BN_bin2bn(Slot(3), 32, nullptr);
		BIGNUM *k = BN_bin2bn(Slot(14), 32, nullptr);
		BIGNUM *prime = BN_new();
		bool valid;
		Status = 0;
		Operations++;
		assert(EC_GROUP_get_curve(group, prime, nullptr, nullptr, ctx));
		valid = BN_cmp(x, prime) < 0 && BN_cmp(y, prime) < 0 &&
			EC_POINT_set_affine_coordinates(group, point, x, y, ctx) == 1;
		if ((Command & 0xff) == BA414EP_CMD_MOD_ADD)
		{
			// The fixed handover probe is (7 + 8) mod 13.
			assert(Slot(0)[31] == 13 && Slot(6)[31] == 7 && Slot(8)[31] == 8);
			memset(Slot(10), 0, 32); Slot(10)[31] = 2;
		}
		else if (!valid) Status = BA414EP_STATUS_POINT_ERROR;
		else if ((Command & 0xff) == BA414EP_CMD_ECC_PTMUL)
		{
			assert(EC_POINT_mul(group, point, nullptr, point, k, ctx));
			assert(EC_POINT_get_affine_coordinates(group, point, x, y, ctx));
			assert(BN_bn2binpad(x, Slot(6), 32) == 32);
			assert(BN_bn2binpad(y, Slot(7), 32) == 32);
		}
		else assert((Command & 0xff) == BA414EP_CMD_ECC_PTONCURVE);
		BN_free(x); BN_free(y); BN_free(k); BN_free(prime);
		EC_POINT_free(point); EC_GROUP_free(group); BN_CTX_free(ctx);
	}
	int Read(uint32_t Select, const uint8_t *pAddr, int AddrLen,
		uint8_t *pData, int Len) override
	{
		uint32_t addr = Word(pAddr), v = 0;
		assert(AddrLen == 4);
		if (Select == BA414EP_ADDR_MEM)
		{
			assert(addr + Len <= sizeof(Ram)); memcpy(pData, Ram + addr, Len);
		}
		else
		{
			assert(Len == 4);
			if (addr == BA414EP_REG_HWCONFIG) v = 512 | BA414EP_HWCONFIG_PRIME_FIELD_Msk;
			if (addr == BA414EP_REG_STATUS) v = Status;
			memcpy(pData, &v, 4);
		}
		return Len;
	}
	int Write(uint32_t Select, const uint8_t *pAddr, int AddrLen,
		const uint8_t *pData, int Len) override
	{
		uint32_t addr = Word(pAddr);
		assert(AddrLen == 4);
		if (Select == BA414EP_ADDR_MEM)
		{
			assert(addr + Len <= sizeof(Ram)); memcpy(Ram + addr, pData, Len);
		}
		else
		{
			assert(Len == 4);
			if (addr == BA414EP_REG_COMMAND) Command = Word(pData);
			if (addr == BA414EP_REG_CONTROL && (Word(pData) & BA414EP_CONTROL_START)) Execute();
		}
		return Len;
	}
};

class ImportRng : public RngEngine {
public:
	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	CRYPTO_STATUS Status = CRYPTO_STATUS_FAIL;
	bool IsSecure() const override { return true; }
	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override
	{
		memset(pOut, 0, Len);
		return Status;
	}
};

int main()
{
	CryptoCc3xx cc;
	ImportRng importRng;
	Ba414ep ba;
	Ba414ep otherBa;
	BaModel baIntrf;
	CryptoUecc::KeyCtx key;
	uint8_t priv[32], pub[64], secret[32], hash[32] = {}, sig[64];
	CryptoKey descriptor;
	unsigned before;
	TimerCfg_t timerCfg = {};

	assert(CryptoEngineTest() == 0);
	timerCfg.DevNo = TimerGetHighFreqDevNo();
	timerCfg.Freq = 1000000U;
	assert(s_Timer.Init(timerCfg));
	assert(s_Cc.Operations > 0 && s_Cc.AesBlocks > 1000 && s_Cc.HashBlocks > 1000);
	assert(P256IsZero((uint8_t *)s_Cc.Ram, sizeof(s_Cc.Ram)));
	assert(cc.Init(Cc3xxIntrfInstance(), CryptoRngNrfInstance()));
	Hex(s_PrivA, priv, sizeof(priv));
	before = s_Cc.Operations;
	check("CC3xx imports public key through hardware",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, s_PubA, sizeof(pub)) && s_Cc.Operations > before);
	s_Cc.Busy = true;
	memset(secret, 0xa5, sizeof(secret));
	check("CC3xx busy preserves imported key and clears output",
		cc.Agree(CRYPTO_CURVE_P256, &key, pub, secret) == CRYPTO_STATUS_BUSY &&
		key.bKeyValid && P256IsZero(secret, sizeof(secret)));
	s_Cc.Busy = false;
	descriptor = EccKey(priv, CRYPTO_KEY_USE_SIGN);
	check("CC3xx inherited signing receives Init RNG",
		cc.Sign(CRYPTO_CURVE_P256, descriptor, hash, sizeof(hash), sig) == CRYPTO_STATUS_OK &&
		cc.Verify(CRYPTO_CURVE_P256, pub, hash, sizeof(hash), sig) == CRYPTO_STATUS_OK);
	cc.KeyReset(&key);
	assert(cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_OK);
	s_Cc.FailCleanup = true;
	memset(secret, 0xa5, sizeof(secret));
	check("CC3xx cleanup timeout rejects result and consumes key",
		cc.Agree(CRYPTO_CURVE_P256, &key, pub, secret, true) == CRYPTO_STATUS_FAIL &&
		!key.bKeyValid && P256IsZero(secret, sizeof(secret)));
	s_Cc.FailCleanup = false;
	memset(pub, 0xA5, sizeof(pub));
	s_Cc.FailCleanup = true;
	check("CC3xx import cleanup failure clears key and public output",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_FAIL &&
		P256IsZero((uint8_t *)&key, sizeof(key)) && P256IsZero(pub, sizeof(pub)) && !s_Cc.Held);
	s_Cc.FailCleanup = false;
	check("CC3xx import recovers after cleanup fault",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, s_PubA, sizeof(pub)));
	s_Cc.Busy = true;
	memset(pub, 0xA5, sizeof(pub));
	check("CC3xx busy import clears destination key and output",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_BUSY &&
		P256IsZero((uint8_t *)&key, sizeof(key)) && P256IsZero(pub, sizeof(pub)));
	s_Cc.Busy = false;
	cc.SetRng(nullptr);
	before = s_Cc.Operations;
	check("CC3xx context-only import needs no RNG or multiplication",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, nullptr) == CRYPTO_STATUS_OK &&
		key.bKeyValid && memcmp(key.PrivKey, priv, sizeof(priv)) == 0 && s_Cc.Operations == before);
	memset(pub, 0xA5, sizeof(pub));
	check("CC3xx public import requires blinding RNG",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_UNSUPPORTED &&
		P256IsZero((uint8_t *)&key, sizeof(key)) && P256IsZero(pub, sizeof(pub)));

	cc.SetRng(&importRng);
	check("CC3xx import entropy failure clears key and output",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_FAIL &&
		P256IsZero((uint8_t *)&key, sizeof(key)) && P256IsZero(pub, sizeof(pub)));
	importRng.Status = CRYPTO_STATUS_OK;
	check("CC3xx import rejects repeated zero blinding values",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_FAIL &&
		P256IsZero((uint8_t *)&key, sizeof(key)) && P256IsZero(pub, sizeof(pub)));
	cc.SetRng(CryptoRngNrfInstance());
	s_Cc.FailOperation = s_Cc.Operations + 100U;
	check("CC3xx import arithmetic timeout clears key and output",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_FAIL &&
		P256IsZero((uint8_t *)&key, sizeof(key)) && P256IsZero(pub, sizeof(pub)) && !s_Cc.Held);
	s_Cc.FailOperation = 0U;
	check("CC3xx import recovers after arithmetic timeout",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, s_PubA, sizeof(pub)));
	assert(P256IsZero((uint8_t *)s_Cc.Ram, sizeof(s_Cc.Ram)));
	cc.Disable();
	check("CC3xx disabled context import fails",
		cc.KeyImport(CRYPTO_CURVE_P256, &key, priv, nullptr) == CRYPTO_STATUS_FAIL &&
		P256IsZero((uint8_t *)&key, sizeof(key)));
	assert(cc.Enable());

	cc.SetRng(nullptr);
	check("CC3xx RNG removal reaches inherited signing",
		cc.Sign(CRYPTO_CURVE_P256, descriptor, hash, sizeof(hash), sig) == CRYPTO_STATUS_UNSUPPORTED);
	assert(ba.Init(&baIntrf, CryptoRngNrfInstance()));
	before = baIntrf.Operations;
	check("BA414EP inherits software key import",
		ba.KeyImport(CRYPTO_CURVE_P256, &key, priv, pub) == CRYPTO_STATUS_OK &&
		HexEq(pub, s_PubA, sizeof(pub)) && baIntrf.Operations == before);
	check("BA414EP inherited signing receives Init RNG",
		ba.Sign(CRYPTO_CURVE_P256, descriptor, hash, sizeof(hash), sig) == CRYPTO_STATUS_OK &&
		ba.Verify(CRYPTO_CURVE_P256, pub, hash, sizeof(hash), sig) == CRYPTO_STATUS_OK);
	assert(ba.OpAcquire());
	memset(secret, 0xa5, sizeof(secret));
	check("BA414EP busy preserves imported key and clears output",
		ba.Agree(CRYPTO_CURVE_P256, &key, pub, secret) == CRYPTO_STATUS_BUSY &&
		key.bKeyValid && P256IsZero(secret, sizeof(secret)));
	ba.OpRelease();
	ba.SetRng(nullptr);
	check("BA414EP RNG removal reaches inherited signing",
		ba.Sign(CRYPTO_CURVE_P256, descriptor, hash, sizeof(hash), sig) == CRYPTO_STATUS_UNSUPPORTED);
	ba.SetRng(CryptoRngNrfInstance());
	baIntrf.DenyReset = true;
	ba.Reset();
	check("BA414EP failed reset invalidates the engine", !ba.Valid());
	check("BA414EP failed reset releases shared operation lock", otherBa.OpAcquire());
	otherBa.OpRelease();
	baIntrf.DenyReset = false;
	ba.Reset();
	check("BA414EP explicit reset recovers quarantine", ba.Valid());
	ba.KeyReset(&key);
	PkaVectors(&ba, &ba, &s_ReferencePka);
	PkaSoak(&ba, &ba, &s_ReferencePka, &s_ReferencePka);
	assert(P256IsZero(baIntrf.Slot(14), 32));
	assert(P256IsZero(baIntrf.Slot(6), 32));
	assert(P256IsZero(baIntrf.Slot(7), 32));
	assert(P256IsZero(baIntrf.Slot(15), 32));
	printf("CC3xx/BA414EP models: %d passed, %d failed, %d skipped\n", s_pass, s_fail, s_skip);
	s_Timer.Disable();
	return s_fail ? 1 : 0;
}

#else
int main()
{
	Crypto crypto;
	assert(crypto.Init());
	assert(crypto.SelfTest() == 0);
	return 0;
}
#endif
