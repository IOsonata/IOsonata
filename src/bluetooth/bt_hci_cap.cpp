/**-------------------------------------------------------------------------
@file	bt_hci_cap.cpp

@brief	Bluetooth HCI controller capability discovery.

@author	Hoang Nguyen Hoan
@date	Aug. 6, 2026

@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <string.h>

#include "bluetooth/bt_hci_cap.h"

static uint16_t BtHciCapLoadLe16(const uint8_t *pData)
{
	return (uint16_t)(pData[0] | ((uint16_t)pData[1] << 8));
}

static bool BtHciCapRead(BtHciDevice_t * const pDev, uint16_t OpCode,
	uint8_t *pData, uint8_t DataLen)
{
	memset(pData, 0, DataLen);
	return BtHciCommand(pDev, OpCode, nullptr, 0, pData, DataLen) == 0;
}

bool BtHciCapabilitiesRead(BtHciDevice_t * const pDev,
	BtHciCapabilities_t * const pCapabilities)
{
	if (pCapabilities == nullptr)
	{
		return false;
	}

	memset(pCapabilities, 0, sizeof(*pCapabilities));
	if (pDev == nullptr || pDev->Command == nullptr)
	{
		return false;
	}
	BtHciCapabilities_t capabilities = {};
	uint8_t data[64];

	if (BtHciCapRead(pDev, BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO, data, 8) == false)
	{
		return false;
	}
	capabilities.HciVersion = data[0];
	capabilities.HciRevision = BtHciCapLoadLe16(&data[1]);
	capabilities.LmpVersion = data[3];
	capabilities.Manufacturer = BtHciCapLoadLe16(&data[4]);
	capabilities.LmpSubversion = BtHciCapLoadLe16(&data[6]);
	capabilities.Valid |= BT_HCI_CAP_VALID_VERSION;

	if (BtHciCapRead(pDev, BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS,
		data, sizeof(capabilities.SupportedCommands)) == false)
	{
		return false;
	}
	memcpy(capabilities.SupportedCommands, data,
		sizeof(capabilities.SupportedCommands));
	capabilities.Valid |= BT_HCI_CAP_VALID_COMMANDS;

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES,
		data, sizeof(capabilities.LeFeatures)) == false)
	{
		return false;
	}
	memcpy(capabilities.LeFeatures, data, sizeof(capabilities.LeFeatures));
	capabilities.Valid |= BT_HCI_CAP_VALID_LE_FEATURES;

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES,
		data, sizeof(capabilities.LeStates)) == false)
	{
		return false;
	}
	memcpy(capabilities.LeStates, data, sizeof(capabilities.LeStates));
	capabilities.Valid |= BT_HCI_CAP_VALID_LE_STATES;

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT, data, 6))
	{
		capabilities.LeAclDataLen = BtHciCapLoadLe16(&data[0]);
		capabilities.LeAclPacketCount = data[2];
		capabilities.IsoDataLen = BtHciCapLoadLe16(&data[3]);
		capabilities.IsoPacketCount = data[5];
		capabilities.Valid |= BT_HCI_CAP_VALID_BUFFER_SIZE;
	}
	else if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_BUFF_SIZE, data, 3))
	{
		capabilities.LeAclDataLen = BtHciCapLoadLe16(&data[0]);
		capabilities.LeAclPacketCount = data[2];
		capabilities.Valid |= BT_HCI_CAP_VALID_BUFFER_SIZE;
	}

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN, data, 8))
	{
		capabilities.MaxTxOctets = BtHciCapLoadLe16(&data[0]);
		capabilities.MaxTxTime = BtHciCapLoadLe16(&data[2]);
		capabilities.MaxRxOctets = BtHciCapLoadLe16(&data[4]);
		capabilities.MaxRxTime = BtHciCapLoadLe16(&data[6]);
		capabilities.Valid |= BT_HCI_CAP_VALID_MAX_DATA_LEN;
	}

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE, data, 1))
	{
		capabilities.ResolvingListSize = data[0];
		capabilities.Valid |= BT_HCI_CAP_VALID_RESOLVING_LIST_SIZE;
	}

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN, data, 2))
	{
		capabilities.MaxAdvDataLen = BtHciCapLoadLe16(data);
		capabilities.Valid |= BT_HCI_CAP_VALID_MAX_ADV_DATA_LEN;
	}

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS, data, 1))
	{
		capabilities.AdvSetCount = data[0];
		capabilities.Valid |= BT_HCI_CAP_VALID_ADV_SET_COUNT;
	}

	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE, data, 1))
	{
		capabilities.PeriodicAdvListSize = data[0];
		capabilities.Valid |= BT_HCI_CAP_VALID_PERIODIC_LIST_SIZE;
	}

	*pCapabilities = capabilities;
	return true;
}
