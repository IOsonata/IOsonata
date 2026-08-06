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

#ifdef __cplusplus
extern "C" {
#endif

bool BtHciCapabilitiesRead(BtHciDevice_t * const pDev,
	BtHciCapabilities_t * const pCapabilities);

#ifdef __cplusplus
}
#endif

#endif // __BT_HCI_CAP_H__
