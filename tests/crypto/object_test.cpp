// Direct construction, shared Device inheritance and independent software state.
#include <cassert>
#include <cstring>
#include "crypto/crypto.h"

static Crypto s_GlobalCrypto;

int main(void)
{
	Crypto one;
	Crypto two;
	uint8_t a[32];
	uint8_t b[32];
	CipherEngine *cipher = &one;
	MacEngine *mac = &one;
	HashEngine *hash = &one;
	RngEngine *rng = &one;

	assert(!one.Valid());
	assert(!s_GlobalCrypto.Valid());
	memset(a, 0xa5, sizeof(a));
	assert(one.Random(a, sizeof(a)) == CRYPTO_STATUS_FAIL);
	assert(P256IsZero(a, sizeof(a)));
	assert(one.Init());
	assert(two.Init());
	assert(s_GlobalCrypto.Init());
	assert(static_cast<Device *>(cipher) == static_cast<Device *>(mac));
	assert(static_cast<Device *>(hash) == static_cast<Device *>(rng));
	assert(static_cast<Device *>(cipher) == static_cast<Device *>(rng));
	assert(!one.IsSecure());
	assert(one.Random(a, sizeof(a)) == CRYPTO_STATUS_OK);
	assert(two.Random(b, sizeof(b)) == CRYPTO_STATUS_OK);
	assert(memcmp(a, b, sizeof(a)) == 0);
	assert(one.Init());
	assert(one.Random(a, sizeof(a)) == CRYPTO_STATUS_OK);
	assert(memcmp(a, b, sizeof(a)) != 0);
	one.Disable();
	assert(!one.Valid());
	memset(a, 0xa5, sizeof(a));
	assert(one.Random(a, sizeof(a)) == CRYPTO_STATUS_FAIL);
	assert(P256IsZero(a, sizeof(a)));
	assert(two.Random(b, sizeof(b)) == CRYPTO_STATUS_OK);
	assert(one.Enable());
	assert(one.SelfTest() == 0);
	one.Reset();
	assert(one.Valid());
	assert(one.Random(a, sizeof(a)) == CRYPTO_STATUS_OK);
	assert(s_GlobalCrypto.Random(b, sizeof(b)) == CRYPTO_STATUS_OK);
	assert(memcmp(a, b, sizeof(a)) == 0);
	return 0;
}
