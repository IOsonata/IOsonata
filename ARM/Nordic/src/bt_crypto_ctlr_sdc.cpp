/**-------------------------------------------------------------------------
@file	bt_crypto_ctlr_sdc.cpp

@brief	Bluetooth-owned CipherEngine: AES-128 via the BLE SoftDevice
		Controller HCI.

This is NOT a generic crypto engine and does NOT belong in the crypto layer
(src/crypto). It cannot function without a running BLE controller, so it is
owned by the Bluetooth layer (bt_ prefix) and only a Bluetooth consumer (SMP)
may use it. It implements the CipherEngine facet purely so SMP can compose it
into its AES slot uniformly alongside real crypto engines.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/icrypto.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_app.h"

static void ReverseCopy(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1U - i];
	}
}

class CryptoCtlrSdc : public CipherEngine {
public:
	CryptoCtlrSdc() { vbValid = true; }
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

		BtHciDevice_t *pHci = g_BtAppData.AppDevice.pHciDev;
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

CipherEngine *BtCryptoCtlrSdcInit(void)
{
	static CryptoCtlrSdc s_Instance;
	return &s_Instance;
}
