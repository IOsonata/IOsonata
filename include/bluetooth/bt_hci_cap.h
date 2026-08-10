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
// Core Vol 4 Part E, Section 6.27. Commands not used by IOsonata still occupy
// their assigned bits; these positions must never be compressed.
enum {
	BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS = 25U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_SET_ADV_PARAMETERS = 25U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_SET_ADV_DATA = 25U * 8U + 7U,
	BT_HCI_CAP_CMD_LE_SET_SCAN_RESPONSE_DATA = 26U * 8U,
	BT_HCI_CAP_CMD_LE_SET_ADV_ENABLE = 26U * 8U + 1U,
	BT_HCI_CAP_CMD_LE_SET_SCAN_PARAMETERS = 26U * 8U + 2U,
	BT_HCI_CAP_CMD_LE_SET_SCAN_ENABLE = 26U * 8U + 3U,
	BT_HCI_CAP_CMD_LE_CREATE_CONNECTION = 26U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_REMOTE_CONN_PARAM_REQUEST_REPLY = 33U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_REMOTE_CONN_PARAM_REQUEST_NEG_REPLY = 33U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_SET_DATA_LENGTH = 33U * 8U + 6U,
	BT_HCI_CAP_CMD_LE_ADD_DEVICE_TO_RESOLVING_LIST = 34U * 8U + 3U,
	BT_HCI_CAP_CMD_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST = 34U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_CLEAR_RESOLVING_LIST = 34U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_READ_RESOLVING_LIST_SIZE = 34U * 8U + 6U,
	BT_HCI_CAP_CMD_LE_READ_PEER_RESOLVABLE_ADDRESS = 34U * 8U + 7U,
	BT_HCI_CAP_CMD_LE_READ_LOCAL_RESOLVABLE_ADDRESS = 35U * 8U,
	BT_HCI_CAP_CMD_LE_SET_ADDRESS_RESOLUTION_ENABLE = 35U * 8U + 1U,
	BT_HCI_CAP_CMD_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT = 35U * 8U + 2U,
	BT_HCI_CAP_CMD_LE_READ_PHY = 35U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_SET_DEFAULT_PHY = 35U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_SET_PHY = 35U * 8U + 6U,
	BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS = 36U * 8U + 1U,
	BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS = 36U * 8U + 2U,
	BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA = 36U * 8U + 3U,
	BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA = 36U * 8U + 4U,
	BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE = 36U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_READ_MAX_ADV_DATA_LENGTH = 36U * 8U + 6U,
	BT_HCI_CAP_CMD_LE_READ_SUPPORTED_ADV_SETS = 36U * 8U + 7U,
	BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS = 37U * 8U + 5U,
	BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE = 37U * 8U + 6U,
	BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION = 37U * 8U + 7U,
	BT_HCI_CAP_CMD_LE_SET_PRIVACY_MODE = 39U * 8U + 2U,
};

// Bit numbers in the LE Read Local Supported Features return value.
enum {
	BT_HCI_CAP_LE_FEATURE_DATA_LENGTH_EXT = 5U,
	BT_HCI_CAP_LE_FEATURE_LL_PRIVACY = 6U,
	BT_HCI_CAP_LE_FEATURE_PHY_2M = 8U,
	BT_HCI_CAP_LE_FEATURE_CODED_PHY = 11U,
	BT_HCI_CAP_LE_FEATURE_EXT_ADV = 12U,
};

// HCI TX_PHYs/RX_PHYs bit values used by LE Set Default PHY and LE Set PHY.
#define BT_HCI_PHY_1M					(1U << 0)
#define BT_HCI_PHY_2M					(1U << 1)
#define BT_HCI_PHY_CODED				(1U << 2)
#define BT_HCI_PHY_ALL					(BT_HCI_PHY_1M | BT_HCI_PHY_2M | BT_HCI_PHY_CODED)

// LE Set PHY PHY_Options values for LE Coded PHY.
#define BT_HCI_PHY_CODED_ANY			0U
#define BT_HCI_PHY_CODED_S2			1U
#define BT_HCI_PHY_CODED_S8			2U

#define BT_HCI_LE_CONN_HANDLE_MAX		0x0EFFU

// LE privacy controller configuration. Identity address types are the HCI
// values used by the resolving-list commands: 0 = public, 1 = random static.
#define BT_HCI_PRIVACY_ID_ADDR_PUBLIC		0U
#define BT_HCI_PRIVACY_ID_ADDR_RANDOM		1U
#define BT_HCI_PRIVACY_MODE_NETWORK			0U
#define BT_HCI_PRIVACY_MODE_DEVICE			1U
#define BT_HCI_PRIVACY_RPA_TIMEOUT_DEFAULT	0x0384U	//!< 900 seconds
#define BT_HCI_PRIVACY_RPA_TIMEOUT_MAX		0xA1B8U

typedef struct __Bt_Hci_Privacy_Entry {
	uint8_t PeerIdentityAddrType;
	uint8_t PeerIdentityAddr[6];
	uint8_t PeerIrk[16];
} BtHciPrivacyEntry_t;

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

/**
 * Can the host answer an LE Remote Connection Parameter Request event on this
 * controller?
 *
 * The event obliges the host to reply with either LE Remote Connection
 * Parameter Request Reply or its negative form. A controller that raises the
 * event but implements neither reply leaves the peer's Link Layer parameter
 * update unanswered until the connection drops on supervision timeout, so the
 * event must stay masked off on such a controller and the request handled by
 * the controller itself.
 */
static inline bool BtHciCapabilitiesRemoteConnParamRequestSupported(
	const BtHciCapabilities_t *pCapabilities)
{
	return BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_REMOTE_CONN_PARAM_REQUEST_REPLY) &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_REMOTE_CONN_PARAM_REQUEST_NEG_REPLY);
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

static inline bool BtHciCapabilitiesLegacyScanningSupported(
	const BtHciCapabilities_t *pCapabilities)
{
	return BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_SCAN_PARAMETERS) &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_SCAN_ENABLE);
}

static inline bool BtHciCapabilitiesExtendedScanningSupported(
	const BtHciCapabilities_t *pCapabilities)
{
	return BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS) &&
		BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE);
}

static inline bool BtHciCapabilitiesLegacyInitiatingSupported(
	const BtHciCapabilities_t *pCapabilities)
{
	return BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_CREATE_CONNECTION);
}

static inline bool BtHciCapabilitiesExtendedInitiatingSupported(
	const BtHciCapabilities_t *pCapabilities)
{
	return BtHciCapabilitiesCommandSupported(pCapabilities,
		BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION);
}

#ifdef __cplusplus
extern "C" {
#endif

bool BtHciCapabilitiesRead(BtHciDevice_t * const pDev,
	BtHciCapabilities_t * const pCapabilities);

/**
 * Build the LE Event Mask parameter for a controller.
 *
 * Every LE meta event is enabled except the ones this host cannot service on
 * the given controller. An event the host leaves unanswered is worse than an
 * event it never receives, because the controller waits for a reply that never
 * arrives instead of running the procedure on its own.
 *
 * A NULL or incomplete capability record masks off every event whose handling
 * depends on a command, which is the safe reading of an unknown controller.
 *
 * @param   pCapabilities   Capability record for the controller, or NULL.
 * @param   Mask            Receives the 8 octet LE Event Mask, octet 0 first.
 */
void BtHciLeEventMaskBuild(const BtHciCapabilities_t *pCapabilities,
	uint8_t Mask[8]);

/**
 * Return the capability record for an HCI device associated with the active
 * controller instance.
 *
 * @return Controller-owned capability record, or NULL when the device has not
 *         been associated with the controller.
 */
const BtHciCapabilities_t *BtHciCapabilitiesForDeviceGet(
	const BtHciDevice_t *pDev);

/**
 * Program the controller resolving list from the supplied identity/IRK set.
 *
 * The caller owns bond storage and supplies only the identity data needed by
 * HCI. The controller must be quiescent while this runs: advertising, scanning
 * and initiating must be stopped when required by the controller. EntryCount
 * zero disables address resolution and clears the resolving list.
 *
 * @return true when the requested controller state was applied. A failed
 *         partial update leaves address resolution disabled and best-effort
 *         clears the list.
 */
bool BtHciPrivacyConfigure(BtHciDevice_t *pDev,
	const BtHciPrivacyEntry_t *pEntries, uint8_t EntryCount,
	const uint8_t LocalIrk[16], uint16_t RpaTimeoutSec, uint8_t PrivacyMode);

// Request the transmit Link Layer data length for one LE connection.
bool BtHciSetDataLength(BtHciDevice_t *pDev, uint16_t ConnHdl,
	uint16_t TxOctets, uint16_t TxTime);

// Read the current PHY for ConnHdl. Outputs use BT_HCI_PHY_* bit values.
bool BtHciReadPhy(BtHciDevice_t *pDev, uint16_t ConnHdl,
	uint8_t *pTxPhy, uint8_t *pRxPhy);

// Set preferred PHYs for subsequent LE connections. Zero means no preference
// for that direction.
bool BtHciSetDefaultPhy(BtHciDevice_t *pDev, uint8_t TxPhys, uint8_t RxPhys);

// Set PHY preferences for one LE connection. Zero TxPhys or RxPhys means no
// preference for that direction. PhyOptions is BT_HCI_PHY_CODED_*.
bool BtHciSetPhy(BtHciDevice_t *pDev, uint16_t ConnHdl,
	uint8_t TxPhys, uint8_t RxPhys, uint16_t PhyOptions);

#ifdef __cplusplus
}
#endif

#endif // __BT_HCI_CAP_H__
