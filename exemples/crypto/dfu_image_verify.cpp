/**-------------------------------------------------------------------------
@file	dfu_image_verify.cpp

@brief	Example: verify a firmware image signature before applying an update.

		On-device flow (DfuImageVerify): SHA-256 the image, then ECDSA-P256
		verify the signature against the vendor public key. The vendor signs the
		image hash offline with the matching private key; only the public key
		and the signature reach the device.

		The engine comes from CryptoInit with CRYPTO_PROVIDER_AUTO, so a part
		with a hardware accelerator uses it and a part without falls back to the
		software uECC engine. SHA-256 is always available from the base core.

		The vendor public key and the signature below are a valid example pair
		for the sample image built in main(); replace them with the real vendor
		key and a signature produced by the release signing step.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "crypto/crypto.h"

// Vendor signing public key (X||Y, big-endian), provisioned in firmware.
static const uint8_t s_VendorPubKey[64] = {
	0x0f,0x35,0xe9,0xbc,0xf0,0xfb,0x79,0xbe,0x77,0x23,0x12,0xc0,
	0xf1,0xf0,0x9f,0x66,0x9d,0x4a,0x8b,0xa2,0x1c,0x56,0xd2,0x0b,
	0x88,0xfb,0x65,0x4c,0x06,0xeb,0x8c,0x36,0x66,0xcf,0x0c,0xc2,
	0x2c,0x90,0x4d,0xb8,0x40,0x15,0x4f,0xa9,0x65,0x59,0xa2,0x82,
	0xe8,0xa6,0x63,0xc6,0x71,0x85,0xb0,0xe9,0x10,0xa0,0x5c,0x62,
	0xa2,0xd6,0xdf,0xca,
};
static const uint8_t s_ImageSignature[64] = {
	0xe0,0xca,0x8a,0x73,0x0c,0x65,0xe4,0x67,0xc8,0xf7,0xf2,0xb2,
	0x22,0xe8,0x81,0xfe,0x25,0x2f,0x13,0xb1,0x7c,0xa8,0x62,0xa9,
	0x0a,0xff,0x2c,0xbb,0x19,0xf4,0x0d,0xda,0xee,0x43,0xe0,0x58,
	0xc7,0x24,0xc1,0x85,0x55,0x14,0xf3,0x08,0x0e,0xae,0x68,0xb2,
	0x2d,0x35,0x94,0xa0,0xff,0x29,0xff,0x1c,0x03,0x97,0x44,0x80,
	0x8e,0x09,0xde,0x0f,
};

// On-device check: true when the signature over SHA-256(image) is valid for the
// vendor public key. VendorPubKey is 64 bytes (X||Y), Signature is 64 (r||s).
bool DfuImageVerify(CryptoDev_t *pCrypto, const uint8_t VendorPubKey[64],
					const uint8_t *pImage, size_t ImageLen,
					const uint8_t Signature[64])
{
	uint8_t hash[32];
	if (CryptoSha256(pCrypto, pImage, ImageLen, hash, NULL) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	return CryptoEcdsaP256Verify(pCrypto, VendorPubKey, hash, Signature, NULL)
		   == CRYPTO_STATUS_OK;
}

int main(void)
{
	// Bring up the crypto engine: hardware where available, else software uECC.
	static uint8_t s_CryptoMem[CRYPTO_MEMSIZE_ECDH];
	CryptoDev_t crypto;
	CryptoCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.Provider = CRYPTO_PROVIDER_AUTO;
	cfg.ReqCaps  = CRYPTO_CAP_ECDSA_P256_VERIFY | CRYPTO_CAP_SHA256;
	cfg.pMem     = s_CryptoMem;
	cfg.MemSize  = sizeof(s_CryptoMem);
	if (!CryptoInit(&crypto, &cfg))
	{
		printf("crypto init failed\n");
		return 1;
	}

	// Sample firmware image (matches the embedded signature).
	uint8_t image[256];
	for (int i = 0; i < 256; i++) { image[i] = (uint8_t)(i * 3 + 1); }

	// Genuine image: signature must verify.
	bool ok = DfuImageVerify(&crypto, s_VendorPubKey, image, sizeof(image),
							 s_ImageSignature);
	printf("genuine image accepted: %s\n", ok ? "yes" : "NO");

	// Tampered image: one byte changed, signature must be rejected.
	image[100] ^= 0x01;
	bool bad = DfuImageVerify(&crypto, s_VendorPubKey, image, sizeof(image),
							  s_ImageSignature);
	printf("tampered image rejected: %s\n", bad ? "NO" : "yes");

	return (ok && !bad) ? 0 : 1;
}
