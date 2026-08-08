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
#include "bluetooth/bt_smp_bond.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_peer.h"

void BtAppConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType,
	uint8_t PeerAddr[6]);

static bool AddressIsSet(const uint8_t Addr[6])
{
	for (size_t i = 0; i < 6; i++)
	{
		if (Addr[i] != 0)
		{
			return true;
		}
	}
	return false;
}

static void SdcEnhancedConnected(uint16_t ConnHdl, uint8_t Role,
	uint8_t PeerIdentityAddrType, const uint8_t PeerIdentityAddr[6],
	const uint8_t LocalRpa[6], const uint8_t PeerRpa[6])
{
	uint8_t peerAddrType = (uint8_t)(PeerIdentityAddrType & 1U);
	uint8_t peerAddr[6];
	memcpy(peerAddr, PeerIdentityAddr, sizeof(peerAddr));

	if (AddressIsSet(PeerRpa))
	{
		peerAddrType = BTADDR_TYPE_RAND;
		memcpy(peerAddr, PeerRpa, sizeof(peerAddr));
	}

	// BtAppConnected starts SMP before it returns. Pre-stamp the local RPA in
	// the peer slot so BtPeerConnected preserves the actual on-air address
	// instead of replacing it with the static identity address.
	if (AddressIsSet(LocalRpa))
	{
		BtDevice_t *pPeer = BtPeerAlloc(ConnHdl);
		if (pPeer != nullptr)
		{
			pPeer->Conn.OwnAddrType = BTADDR_TYPE_RAND;
			memcpy(pPeer->Conn.OwnAddr, LocalRpa, sizeof(pPeer->Conn.OwnAddr));
		}
	}

	BtAppConnected(ConnHdl, Role, peerAddrType, peerAddr);
}

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

bool BtSmpBondLoadComplete(void)
{
	BtHciDevice_t *pHci = g_BtAppData.AppDevice.pHciDev;
	if (pHci == nullptr)
	{
		return false;
	}

	BtHciCapabilities_t localCapabilities;
	const BtHciCapabilities_t *pCapabilities =
		BtHciCapabilitiesForDeviceGet(pHci);
	const uint32_t required = BT_HCI_CAP_VALID_COMMANDS |
		BT_HCI_CAP_VALID_LE_FEATURES |
		BT_HCI_CAP_VALID_RESOLVING_LIST_SIZE;
	if (pCapabilities == nullptr ||
		(pCapabilities->Valid & required) != required)
	{
		if (BtHciCapabilitiesRead(pHci, &localCapabilities) == false)
		{
			return false;
		}
		pCapabilities = &localCapabilities;
	}

	// Controllers without Link Layer Privacy remain supported. IOsonata's bond
	// lookup already resolves peer RPAs in software; this callback only mirrors
	// bonds into a controller resolving list when the controller owns that
	// feature.
	if (BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_LL_PRIVACY) == false ||
		pCapabilities->ResolvingListSize == 0)
	{
		return true;
	}

	uint8_t maxEntries = pCapabilities->ResolvingListSize;
	if (maxEntries > BT_SMP_BOND_MAX)
	{
		maxEntries = BT_SMP_BOND_MAX;
	}

	BtHciPrivacyEntry_t entries[BT_SMP_BOND_MAX] = {};
	uint8_t count = 0;
	int slots = BtSmpBondSlotCount();
	if (slots > BT_SMP_BOND_MAX)
	{
		slots = BT_SMP_BOND_MAX;
	}

	for (int slot = 0; slot < slots && count < maxEntries; slot++)
	{
		if (BtSmpBondIdentityGet(slot,
				&entries[count].PeerIdentityAddrType,
				entries[count].PeerIdentityAddr,
				entries[count].PeerIrk))
		{
			count++;
		}
	}

	if (count == 0)
	{
		CryptoSecureWipe(entries, sizeof(entries));
		return true;
	}

	uint8_t localIrk[16];
	if (BtSmpLocalIrkGet(localIrk) == false)
	{
		CryptoSecureWipe(entries, sizeof(entries));
		CryptoSecureWipe(localIrk, sizeof(localIrk));
		return false;
	}

	bool result = BtHciPrivacyConfigure(pHci, entries, count, localIrk,
		BT_HCI_PRIVACY_RPA_TIMEOUT_DEFAULT, BT_HCI_PRIVACY_MODE_NETWORK);
	CryptoSecureWipe(localIrk, sizeof(localIrk));
	CryptoSecureWipe(entries, sizeof(entries));
	return result;
}

CipherEngine *BtCryptoCtlrSdcInit(void)
{
	BtHciDevice_t *pHci = g_BtAppData.AppDevice.pHciDev;
	if (pHci != nullptr)
	{
		pHci->EnhancedConnected = SdcEnhancedConnected;
	}

	static CryptoCtlrSdc s_Instance;
	return &s_Instance;
}
