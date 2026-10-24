// Caller storage, object inheritance and independent software RNG state.
#include <cassert>
#include <cstring>
#include "crypto/crypto.h"

int main(void)
{
	alignas(CRYPTO_MEMALIGN) uint8_t first[CRYPTO_MEMSIZE];
	alignas(CRYPTO_MEMALIGN) uint8_t second[CRYPTO_MEMSIZE];
	uint8_t a[32];
	uint8_t b[32];
	Crypto *one;
	Crypto *two;
	CipherEngine *cipher;
	MacEngine *mac;
	HashEngine *hash;
	RngEngine *rng;

	assert(CryptoCreate(nullptr, sizeof(first)) == nullptr);
	assert(CryptoCreate(first, 1) == nullptr);
	assert(CryptoCreate(first + 1, sizeof(first) - 1) == nullptr);
	one = CryptoCreate(first, sizeof(first));
	two = CryptoCreate(second, sizeof(second));
	assert(one != nullptr && two != nullptr && one != two);
	cipher = one;
	mac = one;
	hash = one;
	rng = one;
	assert(static_cast<Device *>(cipher) == static_cast<Device *>(mac));
	assert(static_cast<Device *>(hash) == static_cast<Device *>(rng));
	assert(static_cast<Device *>(cipher) == static_cast<Device *>(rng));
	assert(!one->IsSecure());
	assert(one->Random(a, sizeof(a)) == CRYPTO_STATUS_OK);
	assert(two->Random(b, sizeof(b)) == CRYPTO_STATUS_OK);
	assert(memcmp(a, b, sizeof(a)) == 0);
	assert(one->Random(a, sizeof(a)) == CRYPTO_STATUS_OK);
	assert(memcmp(a, b, sizeof(a)) != 0);
	one->Disable();
	one->~Crypto();
	two->Disable();
	two->~Crypto();
	one = CryptoCreate(first, sizeof(first));
	assert(one != nullptr);
	one->Disable();
	one->~Crypto();
	return 0;
}
