/**-------------------------------------------------------------------------
@file	bt_crypto_ctlr_sdc.cpp

@brief	Bluetooth-owned CipherEngine: AES-128 via the BLE SoftDevice
		Controller HCI.

This is NOT a generic crypto engine and does NOT belong in the crypto layer
(src/crypto). It cannot function without a running BLE controller, so it is
owned by the Bluetooth layer (bt_ prefix) and only a Bluetooth consumer (SMP)
may use it. It implements the CipherEngine facet purely so SMP can compose it
into its AES slot uniformly alongside real crypto engines - a non-Bluetooth
consumer (TLS, DFU) must never use it.

It taps the BLE link-layer controller's mandatory HCI primitive LE Encrypt
(AES-128 ECB), which the Bluetooth Core Spec guarantees every LE controller
implements. It issues the command through the generic BtHciCommand path, so it
works with any HCI controller; the SDC dispatch maps it to the typed wrapper.
It provides AES-128 ECB encrypt only; no ECDH (the controller has no LE ECDH
commands), so it composes with an ECDH engine (CryptoUecc or Ba414ep) to cover
the ECDH capability the controller lacks. Random bytes are a target service
(RngGet), not a cipher capability.

SDC-platform only: guarded on the sdc_hci_cmd_le.h header.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>
#include <new>

#include "crypto/icrypto.h"
#include "bluetooth/bt_smp.h"		// declares BtCryptoCtlrSdcInit

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_app.h"

static void ReverseCopy(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1 - i];
	}
}

// @brief	Controller AES-128 ECB as a CipherEngine. Only ECB single-block
//			encrypt is provided (what SMP needs); every other mode declines.
class CryptoCtlrSdc : public CipherEngine {
public:
	CryptoCtlrSdc() { vbValid = true; }

	// Device lifecycle. The controller is brought up by the BLE stack; this
	// engine only wraps its HCI AES, so Enable is a no-op that reports ready.
	bool Enable() override { vbValid = true; return true; }
	void Disable() override {}
	void Reset() override {}
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
						 const CryptoKey &Key,
						 const uint8_t *pIv, size_t IvLen,
						 const uint8_t *pIn, size_t Len, uint8_t *pOut) override
	{
		(void)pIv; (void)IvLen;
		// The controller LE Encrypt is AES-128 ECB encrypt of one 16 byte block.
		if (Alg != CRYPTO_CIPHER_ECB || bEncrypt == 0 || Len != 16 ||
			Key.Loc != CRYPTO_KEY_LOC_PLAIN || Key.Plain.pData == nullptr ||
			Key.Plain.Len != 16 || pIn == nullptr || pOut == nullptr)
		{
			return CRYPTO_STATUS_UNSUPPORTED;
		}

		BtHciDevice_t *pHci = g_BtAppData.AppDevice.pHciDev;
		if (pHci == nullptr)
		{
			memset(pOut, 0, 16);
			return CRYPTO_STATUS_FAIL;
		}

		// LE Encrypt parameters: key then plaintext, each 16 bytes little
		// endian. The SMP toolbox is big-endian, so reverse both ways.
		uint8_t param[32];
		ReverseCopy(&param[0], Key.Plain.pData, 16);
		ReverseCopy(&param[16], pIn, 16);

		uint8_t enc[16];
		if (BtHciCommand(pHci, BT_HCI_CMD_CTLR_ENCRYPT, param, sizeof(param),
						 enc, sizeof(enc)) != 0)
		{
			memset(pOut, 0, 16);		// fail loud
			return CRYPTO_STATUS_FAIL;
		}
		ReverseCopy(pOut, enc, 16);
		return CRYPTO_STATUS_OK;
	}
};

CipherEngine *BtCryptoCtlrSdcInit(void)
{
	// Singleton in internal static storage (no allocation). The controller holds
	// all the state; the engine object is stateless.
	static CryptoCtlrSdc s_Instance;
	return &s_Instance;
}

