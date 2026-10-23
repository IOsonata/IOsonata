/**-------------------------------------------------------------------------
@file	bt_crypto_ctlr_test.cpp

@brief	The crypto engines the controller serves over HCI

A scripted controller answers LE Encrypt and LE Rand from a software AES
and a counter, and the I-SYST vendor commands when told to have them. The
cases check what the engines send, how many commands they spend and that
a controller without the vendor commands is served as before.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_smp.h"
#include "crypto/crypto_softaes.h"

namespace {

int s_Checks = 0;
int s_Failures = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

CryptoSoftAes s_Aes;
BtHciDevice_t s_Dev;
BtHciDevice_t *s_pDev = nullptr;

// What the scripted controller has
uint32_t s_VendorBits = 0;
bool s_bVendorRandomFails = false;

// What went down
int s_CmdCount = 0;
uint16_t s_LastOpCode = 0;
uint8_t s_LastParam[256];
uint8_t s_LastParamLen = 0;
uint8_t s_RandNext = 0;

void Reverse(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1 - i];
	}
}

uint8_t ControllerCommand(BtHciDevice_t * const, uint16_t OpCode, const void *pParam,
						  uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	const uint8_t *p = (const uint8_t *)pParam;
	uint8_t *r = (uint8_t *)pRet;

	s_CmdCount++;
	s_LastOpCode = OpCode;
	s_LastParamLen = ParamLen;
	if (ParamLen)
	{
		memcpy(s_LastParam, pParam, ParamLen);
	}

	CryptoKey key;
	key.Type = CRYPTO_KEY_AES_128;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;

	switch (OpCode)
	{
		case BT_HCI_CMD_CTLR_ENCRYPT:
			{
				// Key and block most significant octet first on the wire
				uint8_t k[16], in[16], out[16];
				Reverse(k, p, 16);
				Reverse(in, p + 16, 16);
				key.Usage = CRYPTO_KEY_USE_ENCRYPT;
				key.Plain.pData = k;
				key.Plain.Len = 16;
				s_Aes.Cipher(CRYPTO_CIPHER_ECB, 1, key, nullptr, 0, in, 16, out);
				Reverse(r, out, RetLen < 16 ? RetLen : 16);
			}
			return 0;

		case BT_HCI_CMD_CTLR_RAND:
			for (int i = 0; i < 8 && i < RetLen; i++)
			{
				r[i] = s_RandNext++;
			}
			return 0;

		case BT_HCI_CMD_VS_ISYST_CRYPTO_SUPPORTED:
			if (s_VendorBits == 0)
			{
				return BT_HCI_ERR_UNKNOWN_COMMAND;
			}
			for (int i = 0; i < 4 && i < RetLen; i++)
			{
				r[i] = (uint8_t)(s_VendorBits >> (8 * i));
			}
			return 0;

		case BT_HCI_CMD_VS_ISYST_RANDOM:
			if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_RANDOM) == 0)
			{
				return BT_HCI_ERR_UNKNOWN_COMMAND;
			}
			if (s_bVendorRandomFails)
			{
				return BT_HCI_ERR_HARDWARE_FAILURE;
			}
			for (int i = 0; i < p[0] && i < RetLen; i++)
			{
				r[i] = s_RandNext++;
			}
			return 0;

		case BT_HCI_CMD_VS_ISYST_AES_CMAC:
			if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_AES_CMAC) == 0)
			{
				return BT_HCI_ERR_UNKNOWN_COMMAND;
			}
			key.Usage = CRYPTO_KEY_USE_SIGN;
			key.Plain.pData = p;
			key.Plain.Len = 16;
			s_Aes.Mac(CRYPTO_MAC_CMAC, key, p + 17, p[16], r, RetLen < 16 ? RetLen : 16);
			return 0;

		case BT_HCI_CMD_VS_ISYST_AES_CCM:
			{
				if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_AES_CCM) == 0)
				{
					return BT_HCI_ERR_UNKNOWN_COMMAND;
				}
				bool dec = (p[0] & BT_HCI_VS_ISYST_AES_CCM_FLAG_DECRYPT) != 0;
				size_t n = 17;
				size_t nonceLen = p[n++];
				const uint8_t *nonce = p + n; n += nonceLen;
				size_t aadLen = p[n++];
				const uint8_t *aad = p + n; n += aadLen;
				size_t tagLen = p[n++];
				size_t msgLen = p[n++];
				const uint8_t *msg = p + n; n += msgLen;
				key.Usage = dec ? CRYPTO_KEY_USE_DECRYPT : CRYPTO_KEY_USE_ENCRYPT;
				key.Plain.pData = p + 1;
				key.Plain.Len = 16;
				CRYPTO_STATUS st;
				if (dec)
				{
					st = s_Aes.Open(CRYPTO_AEAD_AES_CCM, key, nonce, nonceLen, aad, aadLen,
									msg, msgLen, r, p + n, tagLen);
					return st == CRYPTO_STATUS_OK ? 0 : BT_HCI_ERR_AUTHEN_FAILURE;
				}
				st = s_Aes.Seal(CRYPTO_AEAD_AES_CCM, key, nonce, nonceLen, aad, aadLen,
								msg, msgLen, r, r + msgLen, tagLen);
				return st == CRYPTO_STATUS_OK ? 0 : BT_HCI_ERR_HARDWARE_FAILURE;
			}

		case BT_HCI_CMD_VS_ISYST_SHA256:
			if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_SHA256) == 0)
			{
				return BT_HCI_ERR_UNKNOWN_COMMAND;
			}
			// A stand-in digest: the length then the message bytes repeated
			for (int i = 0; i < 32 && i < RetLen; i++)
			{
				r[i] = i == 0 ? p[0] : (p[0] ? p[1 + ((i - 1) % p[0])] : 0);
			}
			return 0;

		case BT_HCI_CMD_VS_ISYST_HMAC_SHA256:
			if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_HMAC_SHA256) == 0)
			{
				return BT_HCI_ERR_UNKNOWN_COMMAND;
			}
			for (int i = 0; i < 32 && i < RetLen; i++)
			{
				r[i] = (uint8_t)(p[0] ^ p[1 + p[0]] ^ i);
			}
			return 0;

		case BT_HCI_CMD_VS_ISYST_ECDSA_P256_SIGN:
			if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_SIGN) == 0)
			{
				return BT_HCI_ERR_UNKNOWN_COMMAND;
			}
			for (int i = 0; i < 64 && i < RetLen; i++)
			{
				r[i] = (uint8_t)(p[i] ^ 0x5A);	// priv then hash, masked
			}
			return 0;

		case BT_HCI_CMD_VS_ISYST_ECDSA_P256_VERIFY:
			{
				if ((s_VendorBits & BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_VERIFY) == 0)
				{
					return BT_HCI_ERR_UNKNOWN_COMMAND;
				}
				// The signature the Sign stand-in gives for pub[0..31] as
				// the private key checks; anything else does not
				for (int i = 0; i < 64; i++)
				{
					if (p[96 + i] != (uint8_t)((i < 32 ? p[i] : p[64 + i - 32]) ^ 0x5A))
					{
						return BT_HCI_ERR_AUTHEN_FAILURE;
					}
				}
				return 0;
			}

		default:
			return BT_HCI_ERR_UNKNOWN_COMMAND;
	}
}

void Start(uint32_t VendorBits)
{
	s_VendorBits = VendorBits;
	s_bVendorRandomFails = false;
	s_CmdCount = 0;
	s_RandNext = 0;
	s_pDev = &s_Dev;
	BtCryptoCtlrReset();
	BtCryptoCtlrRng()->Reset();
}

const uint8_t kKey[16] = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
const uint8_t kMsg[16] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
	0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a };
// RFC 4493 example 2
const uint8_t kCmac[16] = {
	0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
	0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c };

// A controller with none of the vendor commands: one question, then LE
// Encrypt and LE Rand as always, and no CMAC or CCM engine.
void TestAPlainControllerIsServedByTheCoreCommands(void)
{
	Start(0);

	CHECK(BtCryptoCtlrVendorCrypto() == 0);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_CRYPTO_SUPPORTED);
	CHECK(BtCryptoCtlrVendorCrypto() == 0);
	CHECK(s_CmdCount == 1);	// asked once

	CHECK(BtCryptoCtlrMac() == nullptr);
	CHECK(BtCryptoCtlrAead() == nullptr);

	uint8_t out[32];
	s_CmdCount = 0;
	CHECK(BtCryptoCtlrRng()->Random(out, sizeof(out)) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 4 && s_LastOpCode == BT_HCI_CMD_CTLR_RAND);
	CHECK(out[0] == 0 && out[31] == 31);

	CryptoKey key;
	key.Type = CRYPTO_KEY_AES_128;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;
	key.Usage = CRYPTO_KEY_USE_ENCRYPT;
	key.Plain.pData = kKey;
	key.Plain.Len = 16;
	uint8_t block[16];
	uint8_t expect[16];
	s_CmdCount = 0;
	CHECK(BtCryptoCtlrInit()->Cipher(CRYPTO_CIPHER_ECB, 1, key, nullptr, 0, kMsg, 16, block) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_CTLR_ENCRYPT);
	s_Aes.Cipher(CRYPTO_CIPHER_ECB, 1, key, nullptr, 0, kMsg, 16, expect);
	CHECK(memcmp(block, expect, 16) == 0);
}

// With the vendor commands, random numbers come in one command per request
// and the CMAC and CCM engines appear.
void TestTheVendorCommandsAreUsedWhenTheControllerHasThem(void)
{
	Start(BT_HCI_VS_ISYST_CRYPTO_AES_CMAC | BT_HCI_VS_ISYST_CRYPTO_AES_CCM |
		  BT_HCI_VS_ISYST_CRYPTO_RANDOM);

	uint8_t out[300];
	CHECK(BtCryptoCtlrRng()->Random(out, 32) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 2);	// the question and one Random
	CHECK(s_LastOpCode == BT_HCI_CMD_VS_ISYST_RANDOM && s_LastParamLen == 1 && s_LastParam[0] == 32);
	CHECK(out[0] == 0 && out[31] == 31);

	s_CmdCount = 0;
	CHECK(BtCryptoCtlrRng()->Random(out, 300) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 2);	// 251 + 49
	CHECK(s_LastParam[0] == 49);
	CHECK(out[0] == 32 && out[299] == (uint8_t)(32 + 299));

	// A Random that fails on the link leaves the rest to LE Rand
	s_bVendorRandomFails = true;
	s_CmdCount = 0;
	CHECK(BtCryptoCtlrRng()->Random(out, 16) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 3 && s_LastOpCode == BT_HCI_CMD_CTLR_RAND);
	s_bVendorRandomFails = false;

	MacEngine *pMac = BtCryptoCtlrMac();
	CHECK(pMac != nullptr);
	CryptoKey key;
	key.Type = CRYPTO_KEY_AES_128;
	key.Loc = CRYPTO_KEY_LOC_PLAIN;
	key.Usage = CRYPTO_KEY_USE_SIGN;
	key.Plain.pData = kKey;
	key.Plain.Len = 16;
	uint8_t mac[16];
	s_CmdCount = 0;
	CHECK(pMac->Mac(CRYPTO_MAC_CMAC, key, kMsg, 16, mac, 16) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_AES_CMAC);
	CHECK(s_LastParamLen == 33 && s_LastParam[16] == 16 && memcmp(s_LastParam, kKey, 16) == 0);
	CHECK(memcmp(mac, kCmac, 16) == 0);
	// Too long for one command is refused, not split
	uint8_t big[239];
	CHECK(pMac->Mac(CRYPTO_MAC_CMAC, key, big, sizeof(big), mac, 16) == CRYPTO_STATUS_UNSUPPORTED);
	CHECK(pMac->Mac(CRYPTO_MAC_CMAC, key, big, 238, mac, 16) == CRYPTO_STATUS_OK);

	AeadEngine *pAead = BtCryptoCtlrAead();
	CHECK(pAead != nullptr);
	const uint8_t nonce[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
	const uint8_t aad[1] = {0xEA};
	uint8_t sealed[20];
	uint8_t tag[4];
	uint8_t expectSealed[20];
	uint8_t expectTag[4];
	key.Usage = CRYPTO_KEY_USE_ENCRYPT;
	s_CmdCount = 0;
	CHECK(pAead->Seal(CRYPTO_AEAD_AES_CCM, key, nonce, 13, aad, 1, big, 20, sealed, tag, 4) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_AES_CCM);
	CHECK(s_LastParamLen == 1 + 16 + 1 + 13 + 1 + 1 + 2 + 20);
	CHECK(s_LastParam[0] == 0 && s_LastParam[17] == 13 && s_LastParam[31] == 1 &&
		  s_LastParam[32] == 0xEA && s_LastParam[33] == 4 && s_LastParam[34] == 20);
	s_Aes.Seal(CRYPTO_AEAD_AES_CCM, key, nonce, 13, aad, 1, big, 20, expectSealed, expectTag, 4);
	CHECK(memcmp(sealed, expectSealed, 20) == 0 && memcmp(tag, expectTag, 4) == 0);

	uint8_t opened[20];
	key.Usage = CRYPTO_KEY_USE_DECRYPT;
	s_CmdCount = 0;
	CHECK(pAead->Open(CRYPTO_AEAD_AES_CCM, key, nonce, 13, aad, 1, sealed, 20, opened, tag, 4) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastParamLen == 1 + 16 + 1 + 13 + 1 + 1 + 2 + 20 + 4);
	CHECK(s_LastParam[0] == BT_HCI_VS_ISYST_AES_CCM_FLAG_DECRYPT);
	CHECK(memcmp(opened, big, 20) == 0);
	tag[0] ^= 1;
	CHECK(pAead->Open(CRYPTO_AEAD_AES_CCM, key, nonce, 13, aad, 1, sealed, 20, opened, tag, 4) == CRYPTO_STATUS_FAIL);
	bool zero = true;
	for (int i = 0; i < 20; i++) zero = zero && opened[i] == 0;
	CHECK(zero);

	// A controller that has only some of them
	Start(BT_HCI_VS_ISYST_CRYPTO_AES_CMAC);
	CHECK(BtCryptoCtlrMac() != nullptr);
	CHECK(BtCryptoCtlrAead() == nullptr);
	s_CmdCount = 0;
	CHECK(BtCryptoCtlrRng()->Random(out, 8) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_CTLR_RAND);
}

// The hash and signature engines frame their commands and keep to the
// one-shot shapes the link serves.
void TestTheHashAndSignatureEnginesFrameTheirCommands(void)
{
	Start(BT_HCI_VS_ISYST_CRYPTO_SHA256 | BT_HCI_VS_ISYST_CRYPTO_HMAC_SHA256 |
		  BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_SIGN | BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_VERIFY);

	HashEngine *pHash = BtCryptoCtlrHash();
	CHECK(pHash != nullptr);
	uint8_t digest[32];
	s_CmdCount = 0;
	CHECK(pHash->Hash(CRYPTO_HASH_SHA256, kMsg, 16, digest) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_SHA256);
	CHECK(s_LastParamLen == 17 && s_LastParam[0] == 16 && memcmp(&s_LastParam[1], kMsg, 16) == 0);
	CHECK(digest[0] == 16 && digest[1] == kMsg[0]);
	uint8_t big[255];
	CHECK(pHash->Hash(CRYPTO_HASH_SHA256, big, 255, digest) == CRYPTO_STATUS_UNSUPPORTED);
	CHECK(pHash->Hash(CRYPTO_HASH_SHA256, big, 254, digest) == CRYPTO_STATUS_OK);
	// No streaming over a link
	alignas(16) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	CHECK(pHash->HashInit(CRYPTO_HASH_SHA256, ctx) == CRYPTO_STATUS_UNSUPPORTED);

	MacEngine *pHmac = BtCryptoCtlrHmac();
	CHECK(pHmac != nullptr);
	CryptoKey hk;
	hk.Type = CRYPTO_KEY_HMAC;
	hk.Loc = CRYPTO_KEY_LOC_PLAIN;
	hk.Usage = CRYPTO_KEY_USE_SIGN;
	hk.Plain.pData = kKey;
	hk.Plain.Len = 16;
	uint8_t mac[32];
	s_CmdCount = 0;
	CHECK(pHmac->Mac(CRYPTO_MAC_HMAC, hk, kMsg, 16, mac, 32) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_HMAC_SHA256);
	CHECK(s_LastParamLen == 34 && s_LastParam[0] == 16 && s_LastParam[17] == 16 &&
		  memcmp(&s_LastParam[1], kKey, 16) == 0 && memcmp(&s_LastParam[18], kMsg, 16) == 0);
	CHECK(mac[0] == (16 ^ 16) && mac[1] == (16 ^ 16 ^ 1));
	CHECK(pHmac->Mac(CRYPTO_MAC_CMAC, hk, kMsg, 16, mac, 32) == CRYPTO_STATUS_UNSUPPORTED);

	SignEngine *pSign = BtCryptoCtlrSign();
	CHECK(pSign != nullptr);
	uint8_t priv[32], pub[64], hash[32], sig[64];
	for (int i = 0; i < 32; i++) { priv[i] = (uint8_t)(i + 1); hash[i] = (uint8_t)(0x80 + i); }
	memcpy(pub, priv, 32);
	memset(&pub[32], 0x33, 32);
	CryptoKey sk;
	sk.Type = CRYPTO_KEY_ECC_P256;
	sk.Loc = CRYPTO_KEY_LOC_PLAIN;
	sk.Usage = CRYPTO_KEY_USE_SIGN;
	sk.Plain.pData = priv;
	sk.Plain.Len = 32;
	s_CmdCount = 0;
	CHECK(pSign->Sign(CRYPTO_CURVE_P256, sk, hash, 32, sig) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_ECDSA_P256_SIGN && s_LastParamLen == 64);
	CHECK(memcmp(s_LastParam, priv, 32) == 0 && memcmp(&s_LastParam[32], hash, 32) == 0);

	s_CmdCount = 0;
	CHECK(pSign->Verify(CRYPTO_CURVE_P256, pub, hash, 32, sig) == CRYPTO_STATUS_OK);
	CHECK(s_CmdCount == 1 && s_LastOpCode == BT_HCI_CMD_VS_ISYST_ECDSA_P256_VERIFY && s_LastParamLen == 160);
	CHECK(memcmp(s_LastParam, pub, 64) == 0 && memcmp(&s_LastParam[64], hash, 32) == 0 &&
		  memcmp(&s_LastParam[96], sig, 64) == 0);
	sig[10] ^= 1;
	CHECK(pSign->Verify(CRYPTO_CURVE_P256, pub, hash, 32, sig) == CRYPTO_STATUS_FAIL);
	CHECK(pSign->Verify(CRYPTO_CURVE_P384, pub, hash, 32, sig) == CRYPTO_STATUS_UNSUPPORTED);
	CHECK(pSign->Sign(CRYPTO_CURVE_P256, sk, hash, 20, sig) == CRYPTO_STATUS_UNSUPPORTED);

	// Verify only, as a dongle without an entropy source answers
	Start(BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_VERIFY);
	pSign = BtCryptoCtlrSign();
	CHECK(pSign != nullptr);
	CHECK(pSign->Sign(CRYPTO_CURVE_P256, sk, hash, 32, sig) == CRYPTO_STATUS_UNSUPPORTED);
	CHECK(BtCryptoCtlrHash() == nullptr && BtCryptoCtlrHmac() == nullptr);

	Start(0);
	CHECK(BtCryptoCtlrSign() == nullptr);
}

// Without a controller nothing is served and nothing is remembered.
void TestNoControllerFailsClosed(void)
{
	Start(BT_HCI_VS_ISYST_CRYPTO_AES_CMAC);
	s_pDev = nullptr;

	uint8_t out[8];
	CHECK(BtCryptoCtlrVendorCrypto() == 0);
	CHECK(BtCryptoCtlrRng()->Random(out, sizeof(out)) == CRYPTO_STATUS_FAIL);
	CHECK(BtCryptoCtlrMac() == nullptr);
	CHECK(s_CmdCount == 0);

	// The controller comes up: it is asked then
	s_pDev = &s_Dev;
	CHECK(BtCryptoCtlrMac() != nullptr);
}

} // namespace

BtHciDevice_t *BtAppHciDevice(void)
{
	return s_pDev;
}

int main()
{
	s_Aes.Enable();
	memset(&s_Dev, 0, sizeof(s_Dev));
	s_Dev.Command = ControllerCommand;

	TestAPlainControllerIsServedByTheCoreCommands();
	TestTheVendorCommandsAreUsedWhenTheControllerHasThem();
	TestTheHashAndSignatureEnginesFrameTheirCommands();
	TestNoControllerFailsClosed();

	if (s_Failures == 0)
	{
		std::printf("Controller crypto engine tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Controller crypto engine tests: %d failure(s), %d checks\n",
				s_Failures, s_Checks);
	return 1;
}
