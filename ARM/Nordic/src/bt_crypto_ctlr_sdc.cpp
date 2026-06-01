/**-------------------------------------------------------------------------
@file	bt_crypto_ctlr_sdc.cpp

@brief	Bluetooth-owned CryptoDev_t: AES-128 / RNG via the BLE SoftDevice
		Controller HCI.

This is NOT a generic crypto engine and does NOT belong in the crypto layer
(src/crypto). It cannot function without a running BLE controller, so it is
owned by the Bluetooth layer (bt_ prefix) and only a Bluetooth consumer (SMP)
may use it. It implements the CryptoDev_t interface purely so SMP can compose
it into its AES slot uniformly alongside real crypto engines - but it is never
advertised in crypto.h and a non-Bluetooth consumer (TLS, DFU) must never use
it.

It taps the BLE link-layer controller's mandatory HCI primitives - LE Encrypt
(AES-128 ECB) and LE Rand - which the Bluetooth Core Spec guarantees every LE
controller implements. The synchronous call form is the Nordic SDC library's
API (sdc_hci_cmd_le_*), matching how the rest of the SDC backend issues HCI
commands. It advertises CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_RNG; no ECDH (the
controller has no LE ECDH commands), so it composes with an ECDH engine
(CryptoUeccInit) - the way a gyro-only chip pairs with an accel chip in an IMU.
On parts with a hardware RNG peripheral, prefer the generic hardware RNG engine
for the RNG slot and use this only for AES.

SDC-platform only: guarded on the sdc_hci_cmd_le.h header.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <string.h>

#include "crypto/crypto.h"

#if defined(CRYPTO_HAS_SDC) || \
	(defined(__has_include) && __has_include("sdc_hci_cmd_le.h"))

extern "C" {
#include "sdc_hci_cmd_le.h"
}

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

	// SMP toolbox is big-endian; LE Encrypt is little-endian. Reverse both.
	sdc_hci_cmd_le_encrypt_t        cmd;
	sdc_hci_cmd_le_encrypt_return_t rsp;

	ReverseCopy(cmd.key, Key, 16);
	ReverseCopy(cmd.plaintext_data, In, 16);

	if (sdc_hci_cmd_le_encrypt(&cmd, &rsp) != 0)
	{
		memset(Out, 0, 16);		// fail loud
		return CRYPTO_STATUS_FAIL;
	}
	ReverseCopy(Out, rsp.encrypted_data, 16);
	return CRYPTO_STATUS_OK;
}

static void CtlrRand(CryptoDev_t * const pDev, uint8_t *pBuf, size_t Len)
{
	(void)pDev;
	size_t off = 0;
	while (off < Len)
	{
		sdc_hci_cmd_le_rand_return_t rr;
		if (sdc_hci_cmd_le_rand(&rr) != 0)
		{
			memset(pBuf + off, 0, Len - off);
			return;
		}
		size_t n = (Len - off) < sizeof(rr.random_number) ?
				   (Len - off) : sizeof(rr.random_number);
		memcpy(pBuf + off, &rr.random_number, n);
		off += n;
	}
}

bool BtCryptoCtlrSdcInit(CryptoDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}
	pDev->pDevData       = nullptr;
	pDev->pName          = "ctlr-sdc";
	pDev->Cap            = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_RNG;	// AES + RNG (no ECDH)
	pDev->EvtCB          = nullptr;									// synchronous
	pDev->Aes128Ecb      = CtlrAes128Ecb;
	pDev->EcdhP256KeyGen = nullptr;
	pDev->EcdhP256       = nullptr;
	pDev->Rand           = CtlrRand;
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
