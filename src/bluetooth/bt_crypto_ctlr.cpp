/**-------------------------------------------------------------------------
@file	bt_crypto_ctlr.cpp

@brief	Crypto engines served by the Bluetooth controller over HCI

AES-128 ECB through LE Encrypt and random numbers through LE Rand, the two
crypto operations every LE controller offers its host (Core Vol 4 Part E
7.8.22 and 7.8.23). They reach the controller through BtHciCommand, so they
work the same whether the controller is linked into the image or behind a
UART, IPC or USB link.

On top of those, the I-SYST vendor commands of HciController (bt_hci.h,
BT_HCI_CMD_VS_ISYST_*): one AES-CMAC or AES-CCM per message instead of one
LE Encrypt per block, and up to 251 random octets per command instead of
eight. The host asks Crypto Supported once after the controller starts;
a controller without the commands answers Unknown HCI Command and the
engines below keep to LE Encrypt and LE Rand, which every controller has.

These are NOT generic crypto engines and do NOT belong in the crypto layer
(src/crypto). They cannot function without a running controller, so they are
owned by the Bluetooth layer (bt_ prefix) and only a Bluetooth consumer (SMP,
Encrypted Advertising Data) uses them. They implement the CipherEngine and
RngEngine facets so those consumers compose them into their slots uniformly
alongside real crypto engines.

@author	Hoang Nguyen Hoan
@date	May 20, 2026

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
#include <string.h>

#include "crypto/icrypto.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_app.h"

// Internal controller access; not an application API.
BtHciDevice_t *BtAppHciDevice(void);

// The I-SYST crypto commands this controller answers, asked once per
// controller start. Zero until asked, zero on a controller without them.
static uint32_t s_VendorCrypto = 0;
static bool s_bVendorAsked = false;

void BtCryptoCtlrReset(void)
{
	s_VendorCrypto = 0;
	s_bVendorAsked = false;
}

uint32_t BtCryptoCtlrVendorCrypto(void)
{
	if (s_bVendorAsked == false)
	{
		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return 0;
		}

		uint8_t bits[4] = {0};
		s_bVendorAsked = true;
		if (BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_CRYPTO_SUPPORTED, nullptr, 0,
						 bits, sizeof(bits)) == 0)
		{
			s_VendorCrypto = (uint32_t)bits[0] | ((uint32_t)bits[1] << 8) |
							 ((uint32_t)bits[2] << 16) | ((uint32_t)bits[3] << 24);
		}
	}

	return s_VendorCrypto;
}

static void ReverseCopy(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1U - i];
	}
}

// AES-128 ECB, one block per LE Encrypt command. The key and the block are
// sent most significant octet first on the wire, which is the reverse of the
// byte order the engines use, hence the copies.
class CryptoCtlrAes : public CipherEngine {
public:
	CryptoCtlrAes() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len,
						 uint8_t *pOut) override
	{
		(void)pIv;
		(void)IvLen;
		if (Alg != CRYPTO_CIPHER_ECB || bEncrypt == 0 || Len != 16U ||
			Key.Type != CRYPTO_KEY_AES_128 ||
			Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
			(Key.Usage & CRYPTO_KEY_USE_ENCRYPT) == 0U ||
			Key.Plain.pData == nullptr || Key.Plain.Len != 16U ||
			pIn == nullptr || pOut == nullptr)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			memset(pOut, 0, 16U);
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[32];
		uint8_t result[16];
		ReverseCopy(&param[0], Key.Plain.pData, 16U);
		ReverseCopy(&param[16], pIn, 16U);
		int rc = BtHciCommand(pHci, BT_HCI_CMD_CTLR_ENCRYPT, param,
						 sizeof(param), result, sizeof(result));
		CryptoSecureWipe(param, sizeof(param));
		if (rc != 0)
		{
			CryptoSecureWipe(result, sizeof(result));
			memset(pOut, 0, 16U);
			return CRYPTO_STATUS_FAIL;
		}
		ReverseCopy(pOut, result, 16U);
		CryptoSecureWipe(result, sizeof(result));
		return CRYPTO_STATUS_OK;
	}
};

// Random numbers, eight octets per LE Rand command. The controller draws
// them from the hardware entropy source it was started with, which is what
// makes this a secure engine for SMP nonces and the EAD randomizer. A request
// of any length is served from whole commands, the octets left over from the
// last one are kept for the next request.
class CryptoCtlrRng : public RngEngine {
public:
	CryptoCtlrRng() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override { vHeld = 0; }

	bool IsSecure() const override { return true; }

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override
	{
		if (pOut == nullptr)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		// The vendor command fills a whole request in one round trip; the
		// octets held from LE Rand are used up first so none are wasted.
		while (Len > 0 && vHeld == 0 &&
			   (BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_RANDOM) != 0)
		{
			uint8_t n = Len < BT_HCI_VS_ISYST_RANDOM_MAX ? (uint8_t)Len : BT_HCI_VS_ISYST_RANDOM_MAX;
			if (BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_RANDOM, &n, 1, pOut, n) != 0)
			{
				// Fall through to LE Rand for what is left
				break;
			}
			pOut += n;
			Len -= n;
		}

		while (Len > 0)
		{
			if (vHeld == 0)
			{
				if (BtHciCommand(pHci, BT_HCI_CMD_CTLR_RAND, nullptr, 0,
								 vRand, sizeof(vRand)) != 0)
				{
					CryptoSecureWipe(vRand, sizeof(vRand));
					return CRYPTO_STATUS_FAIL;
				}
				vHeld = sizeof(vRand);
			}

			size_t n = Len < vHeld ? Len : vHeld;
			memcpy(pOut, &vRand[sizeof(vRand) - vHeld], n);
			CryptoSecureWipe(&vRand[sizeof(vRand) - vHeld], n);
			vHeld -= n;
			pOut += n;
			Len -= n;
		}

		return CRYPTO_STATUS_OK;
	}

private:
	uint8_t vRand[8];
	size_t vHeld = 0;
};

// AES-CMAC, one vendor command per message. The key and the message are
// in engine order on the wire, as the software engines take them, so there
// is nothing to reverse here. Only present on a controller that says so.
class CryptoCtlrMac : public MacEngine {
public:
	CryptoCtlrMac() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pMac, size_t MacLen) override
	{
		if (Alg != CRYPTO_MAC_CMAC || Key.Type != CRYPTO_KEY_AES_128 ||
			Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
			(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
			Key.Plain.pData == nullptr || Key.Plain.Len != 16U ||
			(pMsg == nullptr && Len != 0U) || pMac == nullptr ||
			MacLen == 0U || MacLen > 16U || Len > 255U - 17U)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[255];
		uint8_t mac[16];
		memcpy(param, Key.Plain.pData, 16U);
		param[16] = (uint8_t)Len;
		if (Len != 0U)
		{
			memcpy(&param[17], pMsg, Len);
		}
		int rc = BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_AES_CMAC, param,
							  (int)(17U + Len), mac, sizeof(mac));
		CryptoSecureWipe(param, 17U + Len);
		if (rc != 0)
		{
			CryptoSecureWipe(mac, sizeof(mac));
			memset(pMac, 0, MacLen);
			return CRYPTO_STATUS_FAIL;
		}
		memcpy(pMac, mac, MacLen);
		CryptoSecureWipe(mac, sizeof(mac));
		return CRYPTO_STATUS_OK;
	}
};

// AES-CCM, one vendor command per packet. The parameters and the output
// fit one command and one event, which bounds a message at what the
// advertising and SMP consumers need. Only present on a controller that
// says so.
class CryptoCtlrAead : public AeadEngine {
public:
	CryptoCtlrAead() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
					   const uint8_t *pNonce, size_t NonceLen,
					   const uint8_t *pAad, size_t AadLen,
					   const uint8_t *pMsg, size_t Len,
					   uint8_t *pOut, uint8_t *pTag, size_t TagLen) override
	{
		if (Check(Alg, Key, CRYPTO_KEY_USE_ENCRYPT, pNonce, NonceLen, pAad, AadLen,
				  pMsg, Len, pOut, pTag, TagLen) == false)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}
		return Run(false, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut,
				   pTag, TagLen);
	}

	CRYPTO_STATUS Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
					   const uint8_t *pNonce, size_t NonceLen,
					   const uint8_t *pAad, size_t AadLen,
					   const uint8_t *pMsg, size_t Len,
					   uint8_t *pOut, const uint8_t *pTag,
					   size_t TagLen) override
	{
		if (Check(Alg, Key, CRYPTO_KEY_USE_DECRYPT, pNonce, NonceLen, pAad, AadLen,
				  pMsg, Len, pOut, pTag, TagLen) == false)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}
		return Run(true, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut,
				   const_cast<uint8_t *>(pTag), TagLen);
	}

private:
	static bool Check(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key, uint32_t Use,
					  const uint8_t *pNonce, size_t NonceLen,
					  const uint8_t *pAad, size_t AadLen,
					  const uint8_t *pMsg, size_t Len,
					  const uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
	{
		// Everything in one command: 1 + 16 + 1 + nonce + 1 + aad + 2 + msg (+ tag)
		return Alg == CRYPTO_AEAD_AES_CCM && Key.Type == CRYPTO_KEY_AES_128 &&
			   Key.Loc == CRYPTO_KEY_LOC_PLAIN && (Key.Usage & Use) != 0U &&
			   Key.Plain.pData != nullptr && Key.Plain.Len == 16U &&
			   pNonce != nullptr && NonceLen >= 7U && NonceLen <= 13U &&
			   (pAad != nullptr || AadLen == 0U) && AadLen <= 255U &&
			   (pMsg != nullptr || Len == 0U) && (pOut != nullptr || Len == 0U) &&
			   pTag != nullptr && TagLen >= 4U && TagLen <= 16U && (TagLen & 1U) == 0U &&
			   21U + NonceLen + AadLen + Len + TagLen <= 255U && Len + TagLen <= 251U;
	}

	CRYPTO_STATUS Run(bool bDecrypt, const CryptoKey &Key,
					  const uint8_t *pNonce, size_t NonceLen,
					  const uint8_t *pAad, size_t AadLen,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pOut, uint8_t *pTag, size_t TagLen)
	{
		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[255];
		uint8_t ret[251];
		size_t n = 0;
		param[n++] = bDecrypt ? BT_HCI_VS_ISYST_AES_CCM_FLAG_DECRYPT : 0;
		memcpy(&param[n], Key.Plain.pData, 16U); n += 16U;
		param[n++] = (uint8_t)NonceLen;
		memcpy(&param[n], pNonce, NonceLen); n += NonceLen;
		param[n++] = (uint8_t)AadLen;
		if (AadLen != 0U)
		{
			memcpy(&param[n], pAad, AadLen); n += AadLen;
		}
		param[n++] = (uint8_t)TagLen;
		param[n++] = (uint8_t)Len;
		if (Len != 0U)
		{
			memcpy(&param[n], pMsg, Len); n += Len;
		}
		if (bDecrypt)
		{
			memcpy(&param[n], pTag, TagLen); n += TagLen;
		}

		size_t retLen = Len + (bDecrypt ? 0U : TagLen);
		int rc = BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_AES_CCM, param, (int)n,
							  ret, (int)retLen);
		CryptoSecureWipe(param, n);
		if (rc != 0)
		{
			CryptoSecureWipe(ret, sizeof(ret));
			if (Len != 0U)
			{
				memset(pOut, 0, Len);
			}
			return CRYPTO_STATUS_FAIL;
		}
		if (Len != 0U)
		{
			memcpy(pOut, ret, Len);
		}
		if (bDecrypt == false)
		{
			memcpy(pTag, &ret[Len], TagLen);
		}
		CryptoSecureWipe(ret, sizeof(ret));
		return CRYPTO_STATUS_OK;
	}
};

// SHA-256 and HMAC-SHA-256 of a message that fits one command. There is
// no streaming digest over a link, so a large image is hashed by the host;
// this serves the short messages, a key hash or a challenge. Only present
// on a controller that says so.
class CryptoCtlrHash : public HashEngine, public MacEngine {
public:
	CryptoCtlrHash() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
					   size_t Len, uint8_t *pDigest) override
	{
		if (Alg != CRYPTO_HASH_SHA256 || (pMsg == nullptr && Len != 0U) ||
			pDigest == nullptr || Len > 254U ||
			(BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_SHA256) == 0)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[255];
		param[0] = (uint8_t)Len;
		if (Len != 0U)
		{
			memcpy(&param[1], pMsg, Len);
		}
		if (BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_SHA256, param, (int)(1U + Len),
						 pDigest, 32) != 0)
		{
			memset(pDigest, 0, 32U);
			return CRYPTO_STATUS_FAIL;
		}
		return CRYPTO_STATUS_OK;
	}

	CRYPTO_STATUS Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
					  const uint8_t *pMsg, size_t Len,
					  uint8_t *pMac, size_t MacLen) override
	{
		if (Alg != CRYPTO_MAC_HMAC || Key.Type != CRYPTO_KEY_HMAC ||
			Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
			(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
			(Key.Plain.pData == nullptr && Key.Plain.Len != 0U) ||
			(pMsg == nullptr && Len != 0U) || pMac == nullptr ||
			MacLen == 0U || MacLen > 32U || Key.Plain.Len + Len > 253U ||
			(BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_HMAC_SHA256) == 0)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[255];
		uint8_t mac[32];
		size_t n = 0;
		param[n++] = (uint8_t)Key.Plain.Len;
		if (Key.Plain.Len != 0U)
		{
			memcpy(&param[n], Key.Plain.pData, Key.Plain.Len); n += Key.Plain.Len;
		}
		param[n++] = (uint8_t)Len;
		if (Len != 0U)
		{
			memcpy(&param[n], pMsg, Len); n += Len;
		}
		int rc = BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_HMAC_SHA256, param, (int)n,
							  mac, sizeof(mac));
		CryptoSecureWipe(param, n);
		if (rc != 0)
		{
			CryptoSecureWipe(mac, sizeof(mac));
			memset(pMac, 0, MacLen);
			return CRYPTO_STATUS_FAIL;
		}
		memcpy(pMac, mac, MacLen);
		CryptoSecureWipe(mac, sizeof(mac));
		return CRYPTO_STATUS_OK;
	}
};

// ECDSA P-256 over a SHA-256 digest, the signature check of a firmware
// image on a host without P-256 of its own. Keys and signature are the raw
// big endian octets the engines use. Only present on a controller that
// says so; Sign needs the entropy source the controller may lack.
class CryptoCtlrSign : public SignEngine {
public:
	CryptoCtlrSign() { vbValid = true; }

	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}

	CRYPTO_STATUS Sign(CRYPTO_CURVE Curve, const CryptoKey &Key,
					   const uint8_t *pHash, size_t HashLen,
					   uint8_t *pSig) override
	{
		if (Curve != CRYPTO_CURVE_P256 || Key.Type != CRYPTO_KEY_ECC_P256 ||
			Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
			(Key.Usage & CRYPTO_KEY_USE_SIGN) == 0U ||
			Key.Plain.pData == nullptr || Key.Plain.Len != 32U ||
			pHash == nullptr || HashLen != 32U || pSig == nullptr ||
			(BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_SIGN) == 0)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[64];
		memcpy(param, Key.Plain.pData, 32U);
		memcpy(&param[32], pHash, 32U);
		int rc = BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_ECDSA_P256_SIGN, param,
							  sizeof(param), pSig, 64);
		CryptoSecureWipe(param, sizeof(param));
		if (rc != 0)
		{
			memset(pSig, 0, 64U);
			return CRYPTO_STATUS_FAIL;
		}
		return CRYPTO_STATUS_OK;
	}

	CRYPTO_STATUS Verify(CRYPTO_CURVE Curve, const uint8_t *pPubKey,
						 const uint8_t *pHash, size_t HashLen,
						 const uint8_t *pSig) override
	{
		if (Curve != CRYPTO_CURVE_P256 || pPubKey == nullptr ||
			pHash == nullptr || HashLen != 32U || pSig == nullptr ||
			(BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_VERIFY) == 0)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = BtAppHciDevice();
		if (pHci == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t param[64 + 32 + 64];
		memcpy(param, pPubKey, 64U);
		memcpy(&param[64], pHash, 32U);
		memcpy(&param[96], pSig, 64U);
		// Any status but Success is a signature that does not check, or a
		// link that failed; neither is a verified image.
		return BtHciCommand(pHci, BT_HCI_CMD_VS_ISYST_ECDSA_P256_VERIFY, param,
							sizeof(param), nullptr, 0) == 0 ?
			   CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
	}
};

static CryptoCtlrHash s_CtlrHash;

HashEngine *BtCryptoCtlrHash(void)
{
	return (BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_SHA256) != 0 ? &s_CtlrHash : nullptr;
}

MacEngine *BtCryptoCtlrHmac(void)
{
	return (BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_HMAC_SHA256) != 0 ? &s_CtlrHash : nullptr;
}

SignEngine *BtCryptoCtlrSign(void)
{
	static CryptoCtlrSign s_Sign;
	return (BtCryptoCtlrVendorCrypto() & (BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_SIGN |
										  BT_HCI_VS_ISYST_CRYPTO_ECDSA_P256_VERIFY)) != 0 ? &s_Sign : nullptr;
}

MacEngine *BtCryptoCtlrMac(void)
{
	static CryptoCtlrMac s_Mac;
	return (BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_AES_CMAC) != 0 ? &s_Mac : nullptr;
}

AeadEngine *BtCryptoCtlrAead(void)
{
	static CryptoCtlrAead s_Aead;
	return (BtCryptoCtlrVendorCrypto() & BT_HCI_VS_ISYST_CRYPTO_AES_CCM) != 0 ? &s_Aead : nullptr;
}

CipherEngine *BtCryptoCtlrInit(void)
{
	static CryptoCtlrAes s_Aes;
	return &s_Aes;
}

RngEngine *BtCryptoCtlrRng(void)
{
	static CryptoCtlrRng s_Rng;
	return &s_Rng;
}
