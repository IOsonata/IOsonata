/**-------------------------------------------------------------------------
@file	crypto_softsha256_test.cpp

@brief	Host validation for the CryptoSoftSha256 engine: SHA-256 one-shot,
		streaming, and HMAC-SHA-256.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softsha256.h"

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

// FIPS 180-4 two-block message vector.
static const char kMsg56[] =
	"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
static const uint8_t kDigest56[32] = {
	0x24,0x8d,0x6a,0x61,0xd2,0x06,0x38,0xb8,0xe5,0xc0,0x26,0x93,0x0c,0x3e,0x60,0x39,
	0xa3,0x3c,0xe4,0x59,0x64,0xff,0x21,0x67,0xf6,0xec,0xed,0xd4,0x19,0xdb,0x06,0xc1 };

// RFC 4231 test case 3: 20 byte 0xaa key, 50 byte 0xdd message.
static const uint8_t kHmac3Expected[32] = {
	0x77,0x3e,0xa9,0x1e,0x36,0x80,0x0e,0x46,0x85,0x4d,0xb8,0xeb,0xd0,0x91,0x81,0xa7,
	0x29,0x59,0x09,0x8b,0x3e,0xf8,0xc1,0x22,0xd9,0x63,0x55,0x14,0xce,0xd5,0x65,0xfe };

// RFC 4231 test case 6: 131 byte 0xaa key (hashed first), message
// "Test Using Larger Than Block-Size Key - Hash Key First".
static const uint8_t kHmac6Expected[32] = {
	0x60,0xe4,0x31,0x59,0x1e,0xe0,0xb6,0x7f,0x0d,0x8a,0x26,0xaa,0xcb,0xf5,0xb7,0x7f,
	0x8e,0x0b,0xc6,0x21,0x37,0x28,0xc5,0x14,0x05,0x46,0x04,0x0f,0x0e,0xe3,0x7f,0x54 };

// Counts the streaming dispatch so the HMAC path over the virtual trio is
// proven the way a hardware digest subclass would hook it.
class CountingSha : public CryptoSoftSha256 {
public:
	int InitCalls = 0;
	int UpdateCalls = 0;
	int FinalCalls = 0;

	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx) override
	{
		InitCalls++;
		return CryptoSoftSha256::HashInit(Alg, pHashCtx);
	}
	CRYPTO_STATUS HashUpdate(void *pHashCtx, const uint8_t *pMsg,
							 size_t Len) override
	{
		UpdateCalls++;
		return CryptoSoftSha256::HashUpdate(pHashCtx, pMsg, Len);
	}
	CRYPTO_STATUS HashFinal(void *pHashCtx, uint8_t *pDigest) override
	{
		FinalCalls++;
		return CryptoSoftSha256::HashFinal(pHashCtx, pDigest);
	}
};

int main(void)
{
	printf("CryptoSoftSha256 OO engine validation\n");

	alignas(CryptoSoftSha256) static uint8_t mem[CRYPTO_SOFTSHA256_MEMSIZE];
	CryptoSoftSha256 *engine = CryptoSoftSha256Create(mem, sizeof(mem));
	check("factory constructs aligned engine", engine != nullptr);
	if (engine == nullptr) return 1;

	check("engine self-test (FIPS 180-4 and RFC 4231)", engine->SelfTest() == 0);

	uint8_t digest[32], digest2[32];
	check("one-shot two-block FIPS 180-4 vector",
		engine->Hash(CRYPTO_HASH_SHA256, (const uint8_t *)kMsg56,
					 sizeof(kMsg56) - 1U, digest) == CRYPTO_STATUS_OK &&
		memcmp(digest, kDigest56, sizeof(digest)) == 0);

	check("streaming context fits the common bound",
		engine->HashCtxSize() > 0U &&
		engine->HashCtxSize() <= CRYPTO_HASHCTX_MAX);

	// Streaming split at odd boundaries must equal the one-shot digest.
	alignas(uint64_t) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	const uint8_t *msg = (const uint8_t *)kMsg56;
	bool stream =
		engine->HashInit(CRYPTO_HASH_SHA256, ctx) == CRYPTO_STATUS_OK &&
		engine->HashUpdate(ctx, msg, 1) == CRYPTO_STATUS_OK &&
		engine->HashUpdate(ctx, msg + 1, 30) == CRYPTO_STATUS_OK &&
		engine->HashUpdate(ctx, msg + 31, 0) == CRYPTO_STATUS_OK &&
		engine->HashUpdate(ctx, msg + 31, sizeof(kMsg56) - 1U - 31U) ==
			CRYPTO_STATUS_OK &&
		engine->HashFinal(ctx, digest2) == CRYPTO_STATUS_OK;
	check("streaming digest equals one-shot",
		stream && memcmp(digest2, kDigest56, sizeof(digest2)) == 0);

	// RFC 4231 case 3.
	uint8_t key3[20]; memset(key3, 0xAA, sizeof(key3));
	uint8_t msg3[50]; memset(msg3, 0xDD, sizeof(msg3));
	CryptoKey hk{CRYPTO_KEY_HMAC, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_SIGN, {}};
	hk.Plain.pData = key3;
	hk.Plain.Len = sizeof(key3);
	check("HMAC RFC 4231 case 3",
		engine->Mac(CRYPTO_MAC_HMAC, hk, msg3, sizeof(msg3), digest, 32U) ==
			CRYPTO_STATUS_OK &&
		memcmp(digest, kHmac3Expected, 32U) == 0);

	// RFC 4231 case 6: the key is longer than the block and is hashed first.
	static const char msg6[] =
		"Test Using Larger Than Block-Size Key - Hash Key First";
	uint8_t key6[131]; memset(key6, 0xAA, sizeof(key6));
	hk.Plain.pData = key6;
	hk.Plain.Len = sizeof(key6);
	check("HMAC RFC 4231 case 6 (key hashed first)",
		engine->Mac(CRYPTO_MAC_HMAC, hk, (const uint8_t *)msg6,
					sizeof(msg6) - 1U, digest, 32U) == CRYPTO_STATUS_OK &&
		memcmp(digest, kHmac6Expected, 32U) == 0);

	// Truncated tag: first MacLen bytes of the full HMAC.
	check("HMAC truncated tag",
		engine->Mac(CRYPTO_MAC_HMAC, hk, (const uint8_t *)msg6,
					sizeof(msg6) - 1U, digest2, 16U) == CRYPTO_STATUS_OK &&
		memcmp(digest2, kHmac6Expected, 16U) == 0);

	// Key policy: usage and type are enforced.
	CryptoKey noSign = hk;
	noSign.Usage = CRYPTO_KEY_USE_DERIVE;
	check("HMAC rejects key without sign usage",
		engine->Mac(CRYPTO_MAC_HMAC, noSign, msg3, sizeof(msg3), digest, 32U) ==
			CRYPTO_STATUS_UNSUPPORTED);
	CryptoKey wrongType = hk;
	wrongType.Type = CRYPTO_KEY_AES_128;
	check("HMAC rejects a non-HMAC key type",
		engine->Mac(CRYPTO_MAC_HMAC, wrongType, msg3, sizeof(msg3), digest,
					32U) == CRYPTO_STATUS_UNSUPPORTED);
	check("HMAC rejects an oversized tag request",
		engine->Mac(CRYPTO_MAC_HMAC, hk, msg3, sizeof(msg3), digest, 33U) ==
			CRYPTO_STATUS_UNSUPPORTED);

	// Dispatch proof: HMAC routes through the virtual streaming trio, three
	// Init/Final rounds for a long key (key hash, inner, outer).
	alignas(CountingSha) static uint8_t countMem[sizeof(CountingSha)];
	CountingSha *counting = new (countMem) CountingSha();
	counting->Enable();
	check("HMAC routes through overridden streaming hash",
		counting->Mac(CRYPTO_MAC_HMAC, hk, (const uint8_t *)msg6,
					  sizeof(msg6) - 1U, digest, 32U) == CRYPTO_STATUS_OK &&
		memcmp(digest, kHmac6Expected, 32U) == 0 &&
		counting->InitCalls == 3 && counting->FinalCalls == 3 &&
		counting->UpdateCalls >= 5);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
