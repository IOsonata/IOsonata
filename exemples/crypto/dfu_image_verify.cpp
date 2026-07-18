/**-------------------------------------------------------------------------
@example	dfu_image_verify.cpp

@brief	Example: verify a firmware image signature before applying an update.

@author	Hoang Nguyen Hoan
@date	Jul. 2, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <stdio.h>
#include <string.h>

#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static const uint8_t s_VendorPubKey[64] = {
	0x0f,0x35,0xe9,0xbc,0xf0,0xfb,0x79,0xbe,0x77,0x23,0x12,0xc0,
	0xf1,0xf0,0x9f,0x66,0x9d,0x4a,0x8b,0xa2,0x1c,0x56,0xd2,0x0b,
	0x88,0xfb,0x65,0x4c,0x06,0xeb,0x8c,0x36,0x66,0xcf,0x0c,0xc2,
	0x2c,0x90,0x4d,0xb8,0x40,0x15,0x4f,0xa9,0x65,0x59,0xa2,0x82,
	0xe8,0xa6,0x63,0xc6,0x71,0x85,0xb0,0xe9,0x10,0xa0,0x5c,0x62,
	0xa2,0xd6,0xdf,0xca
};
static const uint8_t s_ImageSignature[64] = {
	0xe0,0xca,0x8a,0x73,0x0c,0x65,0xe4,0x67,0xc8,0xf7,0xf2,0xb2,
	0x22,0xe8,0x81,0xfe,0x25,0x2f,0x13,0xb1,0x7c,0xa8,0x62,0xa9,
	0x0a,0xff,0x2c,0xbb,0x19,0xf4,0x0d,0xda,0xee,0x43,0xe0,0x58,
	0xc7,0x24,0xc1,0x85,0x55,0x14,0xf3,0x08,0x0e,0xae,0x68,0xb2,
	0x2d,0x35,0x94,0xa0,0xff,0x29,0xff,0x1c,0x03,0x97,0x44,0x80,
	0x8e,0x09,0xde,0x0f
};

bool DfuImageVerify(HashEngine *pHash, SignEngine *pVerify,
					const uint8_t VendorPubKey[64],
					const uint8_t *pImage, size_t ImageLen,
					const uint8_t Signature[64])
{
	if (pHash == nullptr || pVerify == nullptr || VendorPubKey == nullptr ||
		Signature == nullptr || (pImage == nullptr && ImageLen != 0U))
	{
		return false;
	}
	uint8_t hash[32];
	if (pHash->Hash(CRYPTO_HASH_SHA256, pImage, ImageLen, hash) !=
		CRYPTO_STATUS_OK)
	{
		return false;
	}
	bool valid = pVerify->Verify(CRYPTO_CURVE_P256, VendorPubKey, hash,
								 sizeof(hash), Signature) == CRYPTO_STATUS_OK;
	CryptoSecureWipe(hash, sizeof(hash));
	return valid;
}

int main(void)
{
	alignas(CryptoSoftSha256)
	static uint8_t hashMem[CRYPTO_SOFTSHA256_MEMSIZE];
	alignas(CryptoUecc)
	static uint8_t verifyMem[CRYPTO_UECC_MEMSIZE];
	HashEngine *hash = CryptoSoftSha256Create(hashMem, sizeof(hashMem));
	SignEngine *verify = CryptoUeccCreate(verifyMem, sizeof(verifyMem), nullptr);
	if (hash == nullptr || verify == nullptr)
	{
		printf("crypto engine init failed\n");
		return 1;
	}

	uint8_t image[256];
	for (int i = 0; i < 256; i++) image[i] = (uint8_t)(i * 3 + 1);
	bool valid = DfuImageVerify(hash, verify, s_VendorPubKey, image,
								sizeof(image), s_ImageSignature);
	printf("genuine image accepted: %s\n", valid ? "yes" : "NO");
	image[100] ^= 1U;
	bool tampered = DfuImageVerify(hash, verify, s_VendorPubKey, image,
								   sizeof(image), s_ImageSignature);
	printf("tampered image rejected: %s\n", tampered ? "NO" : "yes");
	return valid && !tampered ? 0 : 1;
}
