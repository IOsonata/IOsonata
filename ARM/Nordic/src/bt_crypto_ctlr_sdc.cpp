/**-------------------------------------------------------------------------
@file	bt_crypto_ctlr_sdc.cpp

@brief	Bluetooth-owned CryptoDev_t: AES-128 via the BLE SoftDevice
		Controller HCI.

This is NOT a generic crypto engine and does NOT belong in the crypto layer
(src/crypto). It cannot function without a running BLE controller, so it is
owned by the Bluetooth layer (bt_ prefix) and only a Bluetooth consumer (SMP)
may use it. It implements the CryptoDev_t interface purely so SMP can compose
it into its AES slot uniformly alongside real crypto engines - but it is never
advertised in crypto.h and a non-Bluetooth consumer (TLS, DFU) must never use
it.

It taps the BLE link-layer controller's mandatory HCI primitive LE Encrypt
(AES-128 ECB), which the Bluetooth Core Spec guarantees every LE controller
implements. It issues the command through the generic BtHciCommand path, so it
works with any HCI controller; the SDC dispatch maps it to the typed wrapper.
It advertises CRYPTO_CAP_AES128_ECB only; no ECDH (the controller has no LE ECDH
commands), so it composes with an ECDH engine (CryptoUeccInit) to cover the ECDH
capability the controller lacks. Random bytes are a coredev service
(crypto/crypto.h), not a crypto capability, so this adapter no longer provides RNG;
a target that uses the controller for entropy wires LE Rand into RngGet.

SDC-platform only: guarded on the sdc_hci_cmd_le.h header.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"
#include "bluetooth/bt_smp.h"		// declares BtCryptoCtlrSdcInit (C linkage)

#if defined(CRYPTO_HAS_SDC) || \
	(defined(__has_include) && __has_include("sdc_hci_cmd_le.h"))

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_app.h"

static void ReverseCopy(uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[Len - 1 - i];
	}
}

static CRYPTO_STATUS CtlrAes128Ecb(CryptoDev_t * const pDev,
								   const uint8_t Key[16], const uint8_t In[16],
								   uint8_t Out[16], void *pCtx)
{
	(void)pDev; (void)pCtx;

	BtHciDevice_t *pHci = g_BtAppData.AppDevice.pHciDev;
	if (pHci == nullptr)
	{
		memset(Out, 0, 16);
		return CRYPTO_STATUS_FAIL;
	}

	// LE Encrypt parameters: key then plaintext, each 16 bytes little endian.
	// The SMP toolbox is big-endian, so reverse both ways.
	uint8_t param[32];
	ReverseCopy(&param[0], Key, 16);
	ReverseCopy(&param[16], In, 16);

	uint8_t enc[16];
	if (BtHciCommand(pHci, BT_HCI_CMD_CTLR_ENCRYPT, param, sizeof(param), enc, sizeof(enc)) != 0)
	{
		memset(Out, 0, 16);		// fail loud
		return CRYPTO_STATUS_FAIL;
	}
	ReverseCopy(Out, enc, 16);
	return CRYPTO_STATUS_OK;
}

bool BtCryptoCtlrSdcInit(CryptoDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}
	memset(pDev, 0, sizeof(*pDev));
	pDev->pDevData       = nullptr;
	pDev->pName          = "ctlr-sdc";
	pDev->Cap            = CRYPTO_CAP_AES128_ECB;	// AES only (no ECDH, no RNG)
	pDev->Props          = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->KeyCtxSize     = 0;										// AES key is passed in; no per-instance key context
	pDev->EvtCB          = nullptr;									// synchronous
	pDev->Aes128Ecb      = CtlrAes128Ecb;
	pDev->EcdhP256KeyGen = nullptr;
	pDev->EcdhP256       = nullptr;
	pDev->SelfTest       = nullptr;
	return true;
}

#else  // SDC HCI not available on this target

bool BtCryptoCtlrSdcInit(CryptoDev_t * const pDev)
{
	(void)pDev;
	return false;	// SDC controller not present in this build
}

#endif // CRYPTO_HAS_SDC || __has_include("sdc_hci_cmd_le.h")
