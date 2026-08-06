/**-------------------------------------------------------------------------
@file	bt_hci_cap.h

@brief	Bluetooth HCI controller capability discovery.

@author	Hoang Nguyen Hoan
@date	Aug. 6, 2026

@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __BT_HCI_CAP_H__
#define __BT_HCI_CAP_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bluetooth/bt_hci.h"

#define BT_HCI_CAP_VALID_VERSION					(1UL << 0)
#define BT_HCI_CAP_VALID_COMMANDS				(1UL << 1)
#define BT_HCI_CAP_VALID_LE_FEATURES			(1UL << 2)
#define BT_HCI_CAP_VALID_LE_STATES				(1UL << 3)
#define BT_HCI_CAP_VALID_BUFFER_SIZE			(1UL << 4)
#define BT_HCI_CAP_VALID_MAX_DATA_LEN			(1UL << 5)
#define BT_HCI_CAP_VALID_RESOLVING_LIST_SIZE	(1UL << 6)
#define BT_HCI_CAP_VALID_MAX_ADV_DATA_LEN		(1UL << 7)
#define BT_HCI_CAP_VALID_ADV_SET_COUNT			(1UL << 8)
#define BT_HCI_CAP_VALID_PERIODIC_LIST_SIZE		(1UL << 9)

// Bit numbers in the 64-octet Read Local Supported Commands return value.
// The number is octet * 8 + bit, using the octet and bit numbering from
// Core Vol 4 Part E, Section 6.27.
enum {
	BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS = 25U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_SET_ADV_PARAMETERS = 25U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_SET_ADV_DATA = 25U * 8U + 7U,
	BT_HCI_CAP_CMD_LE_SET_SCAN_RESPONSE_DATA = 26U * 8U,
	BT_HCI_CAP_CMD_LE_SET_ADV_ENABLE = 26U * 8U + 1U,
	BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS = 35U * 8U + 6U,
	BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS = 35U * 8U + 7U,
	BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA = 36U * 8U,
	BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA = 36U * 8U + 1U,
	BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE = 36U * 8U + 2U,
	BT_HCI_CAP_CMD_LE_READ_MAX_ADV_DATA_LENGTH = 36U * 8U + 3U,
	BT_HCI_CAP_CMD_LE_READ_SUPPORTED_ADV_SETS = 36U * 8U + 4U,
};

// Bit numbers in the LE Read Local Supported Features return value.
enum {
	BT_HCI_CAP_LE_FEATURE_PHY_2M = 8U,
	BT_HCI_CAP_LE_FEATURE_EXT_ADV = 12U,
};

typedef struct __Bt_Hci_Capabilities {
	uint32_t Valid;
	uint8_t HciVersion;
	uint16_t HciRevision;
	uint8_t LmpVersion;
	uint16_t Manufacturer;
	uint16_t LmpSubversion;
	uint8_t SupportedCommands[64];
	uint8_t LeFeatures[8];
	uint8_t LeStates[8];
	uint16_t LeAclDataLen;
	uint8_t LeAclPacketCount;
	uint16_t IsoDataLen;
	uint8_t IsoPacketCount;
	uint16_t MaxTxOctets;
	uint16_t MaxTxTime;
	uint16_t MaxRxOctets;
	uint16_t MaxRxTime;
	uint8_t ResolvingListSize;
	uint16_t MaxAdvDataLen;
	uint8_t AdvSetCount;
	uint8_t PeriodicAdvListSize;
} BtHciCapabilities_t;

static inline bool BtHciCapabilitiesCommandsKnown(
	const BtHciCapabilities_t *pCapabilities)
{
	return pCapabilities != NULL &&
		(pCapabilities->Valid & BT_HCI_CAP_VALID_COMMANDS) != 0;
}

static inline bool BtHciCapabilitiesCommandSupported(
	const BtHciCapabilities_t *pCapabilities, uint16_t CommandBit)
{
	if (BtHciCapabilitiesCommandsKnown(pCapabilities) == false ||
		CommandBit >= sizeof(pCapabilities->SupportedCommands) * 8U)
	{
		return false;
	}

	return (pCapabilities->SupportedCommands[CommandBit >> 3] &
		(1U << (CommandBit & 7U))) != 0;
}

static inline bool BtHciCapabilitiesLeFeaturesKnown(
	const BtHciCapabilities_t *pCapabilities)
{
	return pCapabilities != NULL &&
		(pCapabilities->Valid & BT_HCI_CAP_VALID_LE_FEATURES) != 0;
}

static inline bool BtHciCapabilitiesLeFeatureSupported(
	const BtHciCapabilities_t *pCapabilities, uint8_t FeatureBit)
{
	if (BtHciCapabilitiesLeFeaturesKnown(pCapabilities) == false ||
		FeatureBit >= sizeof(pCapabilities->LeFeatures) * 8U)
	{
		return false;
	}

	return (pCapabilities->LeFeatures[FeatureBit >> 3] &
		(1U << (FeatureBit & 7U))) != 0;
}

static inline bool BtHciCapabilitiesLegacyAdvertisingSupported(
	const BtHciCapabilities_t *pCapabilities, bool UseRandomAddress,
	bool UseScanResponse)
{
	if (BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_ADV_PARAMETERS) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_ADV_DATA) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_ADV_ENABLE) == false)
	{
		return false;
	}

	if (UseRandomAddress &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS) == false)
	{
		return false;
	}

	if (UseScanResponse &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_SCAN_RESPONSE_DATA) == false)
	{
		return false;
	}

	return true;
}

static inline bool BtHciCapabilitiesExtendedAdvertisingSupported(
	const BtHciCapabilities_t *pCapabilities, bool UseRandomAddress,
	bool UseScanResponse)
{
	if (BtHciCapabilitiesLeFeatureSupported(pCapabilities,
		BT_HCI_CAP_LE_FEATURE_EXT_ADV) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE) == false)
	{
		return false;
	}

	if (UseRandomAddress &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS) == false)
	{
		return false;
	}

	if (UseScanResponse &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA) == false)
	{
		return false;
	}

	return true;
}

#ifdef __cplusplus
extern "C" {
#endif

bool BtHciCapabilitiesRead(BtHciDevice_t * const pDev,
	BtHciCapabilities_t * const pCapabilities);

/**
 * Return the capability record for an HCI device associated with the active
 * controller instance.
 *
 * @return Controller-owned capability record, or NULL when the device has not
 *         been associated with the controller.
 */
const BtHciCapabilities_t *BtHciCapabilitiesForDeviceGet(
	const BtHciDevice_t *pDev);

#ifdef __cplusplus
}
#endif

#endif // __BT_HCI_CAP_H__
