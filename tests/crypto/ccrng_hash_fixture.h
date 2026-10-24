#pragma once
#define OPENSSL_SUPPRESS_DEPRECATED
#include <openssl/sha.h>
#include "crypto/crypto_softsha256.h"
#include "cc3xx_intrf.h"
extern int g_HashFailAfter;
class CryptoCc3xxSym : public HashEngine {
public:
	bool Init(Cc3xxIntrf *) { return true; }
	void Disable() override {}
	bool Enable() override { return true; }
	void Reset() override {}
	size_t HashCtxSize() const override { return sizeof(SHA256_CTX); }
	size_t HashCtxAlign() const override { return alignof(SHA256_CTX); }
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG, void *p) override {
		if (g_HashFailAfter == 0) return CRYPTO_STATUS_FAIL;
		if (g_HashFailAfter > 0) --g_HashFailAfter;
		return SHA256_Init((SHA256_CTX *)p) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS HashUpdate(void *p, const uint8_t *in, size_t n) override {
		return SHA256_Update((SHA256_CTX *)p, in, n) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
	CRYPTO_STATUS HashFinal(void *p, uint8_t *out) override {
		return SHA256_Final(out, (SHA256_CTX *)p) ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};
