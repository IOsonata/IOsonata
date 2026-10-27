/**-------------------------------------------------------------------------
@file	object_test.cpp

@brief	Host test support: object test.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/

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
