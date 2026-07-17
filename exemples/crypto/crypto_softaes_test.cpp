/**-------------------------------------------------------------------------
@example	crypto_softaes_test.cpp

@brief	Host validation for the OO CryptoSoftAes engine.

@author	Hoang Nguyen Hoan
@date	Jul. 15, 2026

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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>

#include "crypto/crypto_softaes.h"

static int s_pass, s_fail;
static void check(const char *name, bool ok)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
	if (ok) s_pass++; else s_fail++;
}

static const uint8_t kCmacKey[16] = {
	0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
	0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c };
static const uint8_t kMsg[64] = {
	0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
	0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
	0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
	0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10 };
static const uint8_t kMac0[16] = {
	0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46 };
static const uint8_t kMac16[16] = {
	0x07,0x0a,0x16,0xb4,0x6b,0x4d,0x41,0x44,0xf7,0x9b,0xdd,0x9d,0xd0,0x4a,0x28,0x7c };
static const uint8_t kMac40[16] = {
	0xdf,0xa6,0x67,0x47,0xde,0x9a,0xe6,0x30,0x30,0xca,0x32,0x61,0x14,0x97,0xc8,0x27 };
static const uint8_t kMac64[16] = {
	0x51,0xf0,0xbe,0xbf,0x7e,0x3b,0x9d,0x92,0xfc,0x49,0x74,0x17,0x79,0x36,0x3c,0xfe };

// The inherited CMAC dispatches its block encrypts through the virtual
// AesEcbEncrypt primitive inside one AesOpBegin/AesOpEnd bracket, the way a
// hardware subclass hooks it. This subclass counts both to prove the path.
class CountingAes : public CryptoSoftAes {
public:
	int BlockCalls = 0;
	int BeginCalls = 0;
	int EndCalls = 0;

protected:
	bool AesOpBegin() override
	{
		BeginCalls++;
		return true;
	}
	void AesOpEnd() override
	{
		EndCalls++;
	}
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
					 uint8_t Out[16]) override
	{
		BlockCalls++;
		return CryptoSoftAes::AesEcbEncrypt(Key, In, Out);
	}
};

int main(void)
{
	printf("CryptoSoftAes OO engine validation\n");

	alignas(CryptoSoftAes) static uint8_t mem[CRYPTO_SOFTAES_MEMSIZE];
	CryptoSoftAes *engine = CryptoSoftAesCreate(mem, sizeof(mem));
	check("factory constructs aligned engine", engine != nullptr);
	if (engine == nullptr) return 1;
	CipherEngine *cipher = engine;
	MacEngine *macEngine = engine;

	static const uint8_t aesKey[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
	static const uint8_t aesIn[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
	static const uint8_t aesExpected[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };

	CryptoKey encryptKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
						CRYPTO_KEY_USE_ENCRYPT, {}};
	encryptKey.Plain.pData = aesKey;
	encryptKey.Plain.Len = sizeof(aesKey);
	uint8_t out[16];
	check("FIPS-197 AES-128 ECB known-answer",
		cipher->Cipher(CRYPTO_CIPHER_ECB, 1, encryptKey, nullptr, 0,
					   aesIn, sizeof(aesIn), out) == CRYPTO_STATUS_OK &&
		memcmp(out, aesExpected, sizeof(out)) == 0);

	CryptoKey signKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					 CRYPTO_KEY_USE_SIGN, {}};
	signKey.Plain.pData = kCmacKey;
	signKey.Plain.Len = sizeof(kCmacKey);
	uint8_t tag[16];
	check("RFC 4493 CMAC len 0",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, nullptr, 0, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac0, 16) == 0);
	check("RFC 4493 CMAC len 16",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 16, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac16, 16) == 0);
	check("RFC 4493 CMAC len 40",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 40, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac40, 16) == 0);
	check("RFC 4493 CMAC len 64",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 64, tag, 16) == CRYPTO_STATUS_OK &&
		memcmp(tag, kMac64, 16) == 0);

	CryptoKey ctrKey{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
					CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	ctrKey.Plain.pData = kCmacKey;
	ctrKey.Plain.Len = sizeof(kCmacKey);
	uint8_t iv[16]; memset(iv, 0x24, sizeof(iv));
	uint8_t encrypted[40], plain[40];
	CRYPTO_STATUS enc = cipher->Cipher(CRYPTO_CIPHER_CTR, 1, ctrKey, iv, 16,
									kMsg, 40, encrypted);
	CRYPTO_STATUS dec = cipher->Cipher(CRYPTO_CIPHER_CTR, 0, ctrKey, iv, 16,
									encrypted, 40, plain);
	check("AES-CTR encrypt/decrypt round trip",
		enc == CRYPTO_STATUS_OK && dec == CRYPTO_STATUS_OK &&
		memcmp(plain, kMsg, 40) == 0);

	CryptoKey denied = encryptKey;
	denied.Usage = CRYPTO_KEY_USE_DERIVE;
	check("cipher rejects key without encrypt usage",
		cipher->Cipher(CRYPTO_CIPHER_ECB, 1, denied, nullptr, 0,
					   aesIn, 16, out) == CRYPTO_STATUS_UNSUPPORTED);
	check("CMAC rejects key without sign usage",
		macEngine->Mac(CRYPTO_MAC_CMAC, denied, nullptr, 0, tag, 16) ==
		CRYPTO_STATUS_UNSUPPORTED);
	check("CMAC rejects zero-length tag",
		macEngine->Mac(CRYPTO_MAC_CMAC, signKey, nullptr, 0, tag, 0) ==
		CRYPTO_STATUS_UNSUPPORTED);

	check("engine self-test", engine->SelfTest() == 0);

	alignas(CountingAes) static uint8_t countingMem[sizeof(CountingAes)];
	CountingAes *counting = new (countingMem) CountingAes();
	counting->Enable();
	check("inherited CMAC routes through overridden block primitive",
		counting->Mac(CRYPTO_MAC_CMAC, signKey, kMsg, 16, tag, 16) ==
			CRYPTO_STATUS_OK && counting->BlockCalls >= 2 &&
		memcmp(tag, kMac16, 16) == 0);
	check("CMAC brackets the operation exactly once",
		counting->BeginCalls == 1 && counting->EndCalls == 1);

	// AES-CCM: RFC 3610 packet vector 1, both directions, tamper and policy
	// rejection, in-place operation, and the bracket dispatch proof.
	static const uint8_t ccmKey[16] = {0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,
									   0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF};
	static const uint8_t ccmNonce[13] = {0,0,0,3,2,1,0,0xA0,0xA1,0xA2,0xA3,
										 0xA4,0xA5};
	static const uint8_t ccmCt[23] = {0x58,0x8C,0x97,0x9A,0x61,0xC6,0x63,0xD2,
		0xF0,0x66,0xD0,0xC2,0xC0,0xF9,0x89,0x80,0x6D,0x5F,0x6B,0x61,0xDA,0xC3,
		0x84};
	static const uint8_t ccmTag[8] = {0x17,0xE8,0xD1,0x2C,0xFD,0xF9,0x26,0xE0};
	uint8_t ccmAad[8], ccmMsg[23], ccmOut[23], ccmBack[23], ccmT[16];
	for (int i = 0; i < 8; i++) ccmAad[i] = (uint8_t)i;
	for (int i = 0; i < 23; i++) ccmMsg[i] = (uint8_t)(i + 8);
	CryptoKey ak{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
				 CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	ak.Plain.pData = ccmKey;
	ak.Plain.Len = sizeof(ccmKey);

	check("CCM seal RFC 3610 vector 1",
		engine->Seal(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					 ccmMsg, 23, ccmOut, ccmT, 8) == CRYPTO_STATUS_OK &&
		memcmp(ccmOut, ccmCt, 23) == 0 && memcmp(ccmT, ccmTag, 8) == 0);
	check("CCM open recovers the plaintext",
		engine->Open(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					 ccmCt, 23, ccmBack, ccmTag, 8) == CRYPTO_STATUS_OK &&
		memcmp(ccmBack, ccmMsg, 23) == 0);

	uint8_t badTag[8];
	memcpy(badTag, ccmTag, sizeof(badTag));
	badTag[0] ^= 1U;
	check("CCM open rejects a tampered tag and zeroes the output",
		engine->Open(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					 ccmCt, 23, ccmBack, badTag, 8) == CRYPTO_STATUS_FAIL &&
		ccmBack[0] == 0U && ccmBack[22] == 0U);
	ccmAad[0] ^= 1U;
	check("CCM open rejects modified associated data",
		engine->Open(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					 ccmCt, 23, ccmBack, ccmTag, 8) == CRYPTO_STATUS_FAIL);
	ccmAad[0] ^= 1U;

	memcpy(ccmOut, ccmMsg, sizeof(ccmMsg));
	check("CCM seals in place",
		engine->Seal(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					 ccmOut, 23, ccmOut, ccmT, 8) == CRYPTO_STATUS_OK &&
		memcmp(ccmOut, ccmCt, 23) == 0);

	CryptoKey encOnly = ak;
	encOnly.Usage = CRYPTO_KEY_USE_ENCRYPT;
	check("CCM open rejects a key without decrypt usage",
		engine->Open(CRYPTO_AEAD_AES_CCM, encOnly, ccmNonce, 13, ccmAad, 8,
					 ccmCt, 23, ccmBack, ccmTag, 8) ==
			CRYPTO_STATUS_UNSUPPORTED);
	CryptoKey decOnly = ak;
	decOnly.Usage = CRYPTO_KEY_USE_DECRYPT;
	check("CCM seal rejects a key without encrypt usage",
		engine->Seal(CRYPTO_AEAD_AES_CCM, decOnly, ccmNonce, 13, ccmAad, 8,
					 ccmMsg, 23, ccmOut, ccmT, 8) ==
			CRYPTO_STATUS_UNSUPPORTED);
	check("CCM rejects a bad nonce length",
		engine->Seal(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 6, ccmAad, 8,
					 ccmMsg, 23, ccmOut, ccmT, 8) ==
			CRYPTO_STATUS_UNSUPPORTED);
	check("CCM rejects an odd tag length",
		engine->Seal(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					 ccmMsg, 23, ccmOut, ccmT, 7) ==
			CRYPTO_STATUS_UNSUPPORTED);

	counting->BlockCalls = 0;
	counting->BeginCalls = 0;
	counting->EndCalls = 0;
	check("CCM routes through the block primitive in one bracket",
		counting->Seal(CRYPTO_AEAD_AES_CCM, ak, ccmNonce, 13, ccmAad, 8,
					   ccmMsg, 23, ccmOut, ccmT, 8) == CRYPTO_STATUS_OK &&
		memcmp(ccmOut, ccmCt, 23) == 0 &&
		counting->BlockCalls >= 6 &&
		counting->BeginCalls == 1 && counting->EndCalls == 1);

	// AES-GCM: the McGrew-Viega reference vector (96 bit IV, AAD, full tag),
	// tamper rejection with verify-first zeroed output, and empty message.
	static const uint8_t gk[16] = {0xfe,0xff,0xe9,0x92,0x86,0x65,0x73,0x1c,
								   0x6d,0x6a,0x8f,0x94,0x67,0x30,0x83,0x08};
	static const uint8_t giv[12] = {0xca,0xfe,0xba,0xbe,0xfa,0xce,0xdb,0xad,
									0xde,0xca,0xf8,0x88};
	static const uint8_t gaad[20] = {0xfe,0xed,0xfa,0xce,0xde,0xad,0xbe,0xef,
		0xfe,0xed,0xfa,0xce,0xde,0xad,0xbe,0xef,0xab,0xad,0xda,0xd2};
	static const uint8_t gmsg[60] = {
		0xd9,0x31,0x32,0x25,0xf8,0x84,0x06,0xe5,0xa5,0x59,0x09,0xc5,0xaf,0xf5,0x26,0x9a,
		0x86,0xa7,0xa9,0x53,0x15,0x34,0xf7,0xda,0x2e,0x4c,0x30,0x3d,0x8a,0x31,0x8a,0x72,
		0x1c,0x3c,0x0c,0x95,0x95,0x68,0x09,0x53,0x2f,0xcf,0x0e,0x24,0x49,0xa6,0xb5,0x25,
		0xb1,0x6a,0xed,0xf5,0xaa,0x0d,0xe6,0x57,0xba,0x63,0x7b,0x39};
	static const uint8_t gct[60] = {
		0x42,0x83,0x1e,0xc2,0x21,0x77,0x74,0x24,0x4b,0x72,0x21,0xb7,0x84,0xd0,0xd4,0x9c,
		0xe3,0xaa,0x21,0x2f,0x2c,0x02,0xa4,0xe0,0x35,0xc1,0x7e,0x23,0x29,0xac,0xa1,0x2e,
		0x21,0xd5,0x14,0xb2,0x54,0x66,0x93,0x1c,0x7d,0x8f,0x6a,0x5a,0xac,0x84,0xaa,0x05,
		0x1b,0xa3,0x0b,0x39,0x6a,0x0a,0xac,0x97,0x3d,0x58,0xe0,0x91};
	static const uint8_t gtag[16] = {0x5b,0xc9,0x4f,0xbc,0x32,0x21,0xa5,0xdb,
		0x94,0xfa,0xe9,0x5a,0xe7,0x12,0x1a,0x47};
	static const uint8_t gtagEmpty[16] = {0x32,0x47,0x18,0x4b,0x3c,0x4f,0x69,
		0xa4,0x4d,0xbc,0xd2,0x28,0x87,0xbb,0xb4,0x18};
	uint8_t gout[60], gback[60], gt[16];
	CryptoKey gkd{CRYPTO_KEY_AES_128, CRYPTO_KEY_LOC_PLAIN,
				  CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT, {}};
	gkd.Plain.pData = gk;
	gkd.Plain.Len = sizeof(gk);

	check("GCM seal reference vector",
		engine->Seal(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20,
					 gmsg, 60, gout, gt, 16) == CRYPTO_STATUS_OK &&
		memcmp(gout, gct, 60) == 0 && memcmp(gt, gtag, 16) == 0);
	check("GCM open recovers the plaintext",
		engine->Open(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20,
					 gct, 60, gback, gtag, 16) == CRYPTO_STATUS_OK &&
		memcmp(gback, gmsg, 60) == 0);
	check("GCM empty message tag",
		engine->Seal(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, nullptr, 0,
					 nullptr, 0, nullptr, gt, 16) == CRYPTO_STATUS_OK &&
		memcmp(gt, gtagEmpty, 16) == 0);
	uint8_t gbadTag[16];
	memcpy(gbadTag, gtag, sizeof(gbadTag));
	gbadTag[15] ^= 1U;
	check("GCM open rejects a tampered tag and zeroes the output",
		engine->Open(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20,
					 gct, 60, gback, gbadTag, 16) == CRYPTO_STATUS_FAIL &&
		gback[0] == 0U && gback[59] == 0U);
	check("GCM rejects a non-96-bit nonce",
		engine->Seal(CRYPTO_AEAD_AES_GCM, gkd, giv, 11, gaad, 20,
					 gmsg, 60, gout, gt, 16) == CRYPTO_STATUS_UNSUPPORTED);
	memcpy(gout, gmsg, sizeof(gmsg));
	check("GCM seals in place",
		engine->Seal(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20,
					 gout, 60, gout, gt, 16) == CRYPTO_STATUS_OK &&
		memcmp(gout, gct, 60) == 0);
	counting->BlockCalls = 0;
	counting->BeginCalls = 0;
	counting->EndCalls = 0;
	check("GCM routes through the block primitive in one bracket",
		counting->Seal(CRYPTO_AEAD_AES_GCM, gkd, giv, 12, gaad, 20,
					   gmsg, 60, gout, gt, 16) == CRYPTO_STATUS_OK &&
		memcmp(gt, gtag, 16) == 0 &&
		counting->BlockCalls >= 6 &&
		counting->BeginCalls == 1 && counting->EndCalls == 1);

	printf("\n%d passed, %d failed\n", s_pass, s_fail);
	return s_fail == 0 ? 0 : 1;
}
