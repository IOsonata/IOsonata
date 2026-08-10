/**-------------------------------------------------------------------------
@file	bt_hci_ctlr.cpp

@brief	Generic implementation of Bluetooth controller device.


@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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

#include "bluetooth/bt_hci_ctlr.h"

// Generic HCI controller layer. The ATT and L2CAP handling that previously
// sat here belongs above the HCI line and is provided by bt_att and
// bt_hci_host. Per target HCI controller transport lives in the target file
// bt_hci_ctlr_<target>.cpp, for example bt_hci_ctlr_sdc.cpp for the nrfxlib
// SDC controller.

#include <memory.h>
#include "istddef.h"

// RX packet fifo. Holds controller->host packets until BtHciCtlrProcess drains
// them to RxHandler. Sized for BT_HCI_CTLR_MTU_MAX sized packets.
#define BTHCICTLR_FIFO_MEM_SIZE			BTHCICTLR_PKT_CFIFO_TOTAL_MEMSIZE(4, BT_HCI_CTLR_MTU_MAX)
alignas(4) static uint8_t s_BtHciCtlrRxFifoMem[BTHCICTLR_FIFO_MEM_SIZE];

// The current controller instance. Existing ports expose one controller per
// firmware image; this association lets generic HCI operations obtain the
// controller-owned capability record after the target identifies its host.
static BtHciCtlrDev_t *s_pBtHciCtlrActive = nullptr;

static uint16_t BtHciCapLoadLe16(const uint8_t *pData)
{
	return (uint16_t)(pData[0] | ((uint16_t)pData[1] << 8));
}

static void BtHciCapStoreLe16(uint8_t *pData, uint16_t Value)
{
	pData[0] = (uint8_t)(Value & 0xFFU);
	pData[1] = (uint8_t)(Value >> 8);
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

	// LE Read Supported States is optional here. Some controllers, the Nordic
	// SoftDevice Controller among them, expose no way to issue it, and no
	// caller requires BT_HCI_CAP_VALID_LE_STATES. Leave the bit clear when the
	// read fails rather than discarding the whole capability set.
	if (BtHciCapRead(pDev, BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES,
		data, sizeof(capabilities.LeStates)))
	{
		memcpy(capabilities.LeStates, data, sizeof(capabilities.LeStates));
		capabilities.Valid |= BT_HCI_CAP_VALID_LE_STATES;
	}

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

const BtHciCapabilities_t *BtHciCtlrCapabilitiesGet(const BtHciCtlrDev_t *pDev)
{
	return pDev != nullptr ? &pDev->Capabilities : nullptr;
}

const BtHciCapabilities_t *BtHciCapabilitiesForDeviceGet(
	const BtHciDevice_t *pDev)
{
	if (pDev == nullptr || s_pBtHciCtlrActive == nullptr ||
		s_pBtHciCtlrActive->pHciDev != pDev)
	{
		return nullptr;
	}

	return &s_pBtHciCtlrActive->Capabilities;
}

static bool BtHciPrivacyIrkValid(const uint8_t Irk[16])
{
	if (Irk == nullptr)
	{
		return false;
	}

	uint8_t nz = 0;
	for (int i = 0; i < 16; i++)
	{
		nz |= Irk[i];
	}
	return nz != 0;
}

static bool BtHciPrivacyEntryValid(const BtHciPrivacyEntry_t *pEntry)
{
	return pEntry != nullptr &&
		(pEntry->PeerIdentityAddrType == BT_HCI_PRIVACY_ID_ADDR_PUBLIC ||
		 pEntry->PeerIdentityAddrType == BT_HCI_PRIVACY_ID_ADDR_RANDOM) &&
		BtHciPrivacyIrkValid(pEntry->PeerIrk);
}

static void BtHciPrivacyAbort(BtHciDevice_t *pDev)
{
	uint8_t disable = 0;
	(void)BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE,
		&disable, sizeof(disable), nullptr, 0);
	(void)BtHciCommand(pDev, BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR,
		nullptr, 0, nullptr, 0);
}

bool BtHciPrivacyConfigure(BtHciDevice_t *pDev,
	const BtHciPrivacyEntry_t *pEntries, uint8_t EntryCount,
	const uint8_t LocalIrk[16], uint16_t RpaTimeoutSec, uint8_t PrivacyMode)
{
	if (pDev == nullptr || pDev->Command == nullptr ||
		PrivacyMode > BT_HCI_PRIVACY_MODE_DEVICE ||
		RpaTimeoutSec == 0 || RpaTimeoutSec > BT_HCI_PRIVACY_RPA_TIMEOUT_MAX ||
		(EntryCount != 0 && (pEntries == nullptr ||
		 BtHciPrivacyIrkValid(LocalIrk) == false)))
	{
		return false;
	}

	for (uint8_t i = 0; i < EntryCount; i++)
	{
		if (BtHciPrivacyEntryValid(&pEntries[i]) == false)
		{
			return false;
		}
	}

	BtHciCapabilities_t localCapabilities;
	const BtHciCapabilities_t *pCapabilities =
		BtHciCapabilitiesForDeviceGet(pDev);
	const uint32_t required = BT_HCI_CAP_VALID_COMMANDS |
		BT_HCI_CAP_VALID_LE_FEATURES |
		BT_HCI_CAP_VALID_RESOLVING_LIST_SIZE;
	if (pCapabilities == nullptr ||
		(pCapabilities->Valid & required) != required)
	{
		if (BtHciCapabilitiesRead(pDev, &localCapabilities) == false)
		{
			return false;
		}
		pCapabilities = &localCapabilities;
	}

	if (BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_LL_PRIVACY) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_CLEAR_RESOLVING_LIST) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_READ_RESOLVING_LIST_SIZE) == false ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_ADDRESS_RESOLUTION_ENABLE) == false ||
		(EntryCount != 0 &&
		 BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_ADD_DEVICE_TO_RESOLVING_LIST) == false) ||
		(EntryCount != 0 &&
		 BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT) == false) ||
		(PrivacyMode == BT_HCI_PRIVACY_MODE_DEVICE && EntryCount != 0 &&
		 BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_PRIVACY_MODE) == false) ||
		EntryCount > pCapabilities->ResolvingListSize)
	{
		return false;
	}

	uint8_t enable = 0;
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE,
		&enable, sizeof(enable), nullptr, 0) != 0)
	{
		return false;
	}

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR,
		nullptr, 0, nullptr, 0) != 0)
	{
		return false;
	}

	if (EntryCount == 0)
	{
		return true;
	}

	uint8_t timeout[2];
	BtHciCapStoreLe16(timeout, RpaTimeoutSec);
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_RESOLVABLE_PRIVATE_ADDR_TIMEOUT,
		timeout, sizeof(timeout), nullptr, 0) != 0)
	{
		BtHciPrivacyAbort(pDev);
		return false;
	}

	for (uint8_t i = 0; i < EntryCount; i++)
	{
		uint8_t add[39];
		add[0] = pEntries[i].PeerIdentityAddrType;
		memcpy(&add[1], pEntries[i].PeerIdentityAddr, 6);
		memcpy(&add[7], pEntries[i].PeerIrk, 16);
		memcpy(&add[23], LocalIrk, 16);

		if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV,
			add, sizeof(add), nullptr, 0) != 0)
		{
			BtHciPrivacyAbort(pDev);
			return false;
		}

		if (PrivacyMode == BT_HCI_PRIVACY_MODE_DEVICE)
		{
			uint8_t mode[8];
			mode[0] = pEntries[i].PeerIdentityAddrType;
			memcpy(&mode[1], pEntries[i].PeerIdentityAddr, 6);
			mode[7] = PrivacyMode;

			if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PRIVACY_MODE,
				mode, sizeof(mode), nullptr, 0) != 0)
			{
				BtHciPrivacyAbort(pDev);
				return false;
			}
		}
	}

	enable = 1;
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE,
		&enable, sizeof(enable), nullptr, 0) != 0)
	{
		BtHciPrivacyAbort(pDev);
		return false;
	}

	return true;
}

bool BtHciSetDataLength(BtHciDevice_t *pDev, uint16_t ConnHdl,
	uint16_t TxOctets, uint16_t TxTime)
{
	// Core Vol 4 Part E 7.8.33. Tx_Octets is the requested maximum LL Data
	// PDU payload and Tx_Time is its requested transmission time in usec.
	if (pDev == nullptr || pDev->Command == nullptr ||
		ConnHdl > BT_HCI_LE_CONN_HANDLE_MAX ||
		TxOctets < 0x001BU || TxOctets > 0x00FBU ||
		TxTime < 0x0148U || TxTime > 0x4290U)
	{
		return false;
	}

	BtHciCapabilities_t localCapabilities;
	const BtHciCapabilities_t *pCapabilities =
		BtHciCapabilitiesForDeviceGet(pDev);
	const uint32_t required = BT_HCI_CAP_VALID_COMMANDS |
		BT_HCI_CAP_VALID_LE_FEATURES;
	if (pCapabilities == nullptr ||
		(pCapabilities->Valid & required) != required)
	{
		if (BtHciCapabilitiesRead(pDev, &localCapabilities) == false)
		{
			return false;
		}
		pCapabilities = &localCapabilities;
	}

	if (BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_DATA_LENGTH) == false ||
		BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_DATA_LENGTH_EXT) == false)
	{
		return false;
	}

	if ((pCapabilities->Valid & BT_HCI_CAP_VALID_MAX_DATA_LEN) != 0 &&
		(TxOctets > pCapabilities->MaxTxOctets ||
		 TxTime > pCapabilities->MaxTxTime))
	{
		return false;
	}

	uint8_t param[6];
	BtHciCapStoreLe16(&param[0], ConnHdl);
	BtHciCapStoreLe16(&param[2], TxOctets);
	BtHciCapStoreLe16(&param[4], TxTime);
	uint8_t result[2] = {};

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_DATA_LEN,
		param, sizeof(param), result, sizeof(result)) != 0)
	{
		return false;
	}

	return BtHciCapLoadLe16(result) == ConnHdl;
}

static const BtHciCapabilities_t *BtHciPhyCapabilitiesGet(
	BtHciDevice_t *pDev, BtHciCapabilities_t *pLocal)
{
	if (pDev == nullptr || pDev->Command == nullptr || pLocal == nullptr)
	{
		return nullptr;
	}

	const BtHciCapabilities_t *pCapabilities =
		BtHciCapabilitiesForDeviceGet(pDev);
	const uint32_t required = BT_HCI_CAP_VALID_COMMANDS |
		BT_HCI_CAP_VALID_LE_FEATURES;
	if (pCapabilities != nullptr &&
		(pCapabilities->Valid & required) == required)
	{
		return pCapabilities;
	}

	if (BtHciCapabilitiesRead(pDev, pLocal) == false)
	{
		return nullptr;
	}

	return pLocal;
}

static bool BtHciPhyMaskSupported(
	const BtHciCapabilities_t *pCapabilities, uint8_t Phys)
{
	if ((Phys & (uint8_t)~BT_HCI_PHY_ALL) != 0)
	{
		return false;
	}

	if ((Phys & BT_HCI_PHY_2M) != 0 &&
		BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_PHY_2M) == false)
	{
		return false;
	}

	if ((Phys & BT_HCI_PHY_CODED) != 0 &&
		BtHciCapabilitiesLeFeatureSupported(pCapabilities,
			BT_HCI_CAP_LE_FEATURE_CODED_PHY) == false)
	{
		return false;
	}

	return true;
}

static uint8_t BtHciPhyValueToMask(uint8_t Phy)
{
	switch (Phy)
	{
		case 0x01:
			return BT_HCI_PHY_1M;
		case 0x02:
			return BT_HCI_PHY_2M;
		case 0x03:
			return BT_HCI_PHY_CODED;
		default:
			return 0;
	}
}

bool BtHciReadPhy(BtHciDevice_t *pDev, uint16_t ConnHdl,
	uint8_t *pTxPhy, uint8_t *pRxPhy)
{
	if (pTxPhy == nullptr || pRxPhy == nullptr ||
		ConnHdl > BT_HCI_LE_CONN_HANDLE_MAX)
	{
		return false;
	}

	BtHciCapabilities_t localCapabilities;
	const BtHciCapabilities_t *pCapabilities =
		BtHciPhyCapabilitiesGet(pDev, &localCapabilities);
	if (pCapabilities == nullptr ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_READ_PHY) == false)
	{
		return false;
	}

	uint8_t param[2];
	BtHciCapStoreLe16(param, ConnHdl);
	uint8_t result[4] = {};
	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_READ_PHY,
		param, sizeof(param), result, sizeof(result)) != 0)
	{
		return false;
	}

	uint16_t resultHdl = BtHciCapLoadLe16(result);
	uint8_t txPhy = BtHciPhyValueToMask(result[2]);
	uint8_t rxPhy = BtHciPhyValueToMask(result[3]);
	if (resultHdl != ConnHdl || txPhy == 0 || rxPhy == 0 ||
		BtHciPhyMaskSupported(pCapabilities, txPhy) == false ||
		BtHciPhyMaskSupported(pCapabilities, rxPhy) == false)
	{
		return false;
	}

	*pTxPhy = txPhy;
	*pRxPhy = rxPhy;
	return true;
}

bool BtHciSetDefaultPhy(BtHciDevice_t *pDev, uint8_t TxPhys, uint8_t RxPhys)
{
	BtHciCapabilities_t localCapabilities;
	const BtHciCapabilities_t *pCapabilities =
		BtHciPhyCapabilitiesGet(pDev, &localCapabilities);
	if (pCapabilities == nullptr ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_DEFAULT_PHY) == false ||
		BtHciPhyMaskSupported(pCapabilities, TxPhys) == false ||
		BtHciPhyMaskSupported(pCapabilities, RxPhys) == false)
	{
		return false;
	}

	uint8_t param[3];
	param[0] = (TxPhys == 0 ? 0x01U : 0U) |
		(RxPhys == 0 ? 0x02U : 0U);
	param[1] = TxPhys;
	param[2] = RxPhys;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_DEFAULT_PHY,
		param, sizeof(param), nullptr, 0) == 0;
}

bool BtHciSetPhy(BtHciDevice_t *pDev, uint16_t ConnHdl,
	uint8_t TxPhys, uint8_t RxPhys, uint16_t PhyOptions)
{
	if (ConnHdl > BT_HCI_LE_CONN_HANDLE_MAX ||
		PhyOptions > BT_HCI_PHY_CODED_S8)
	{
		return false;
	}

	BtHciCapabilities_t localCapabilities;
	const BtHciCapabilities_t *pCapabilities =
		BtHciPhyCapabilitiesGet(pDev, &localCapabilities);
	if (pCapabilities == nullptr ||
		BtHciCapabilitiesCommandSupported(pCapabilities,
			BT_HCI_CAP_CMD_LE_SET_PHY) == false ||
		BtHciPhyMaskSupported(pCapabilities, TxPhys) == false ||
		BtHciPhyMaskSupported(pCapabilities, RxPhys) == false)
	{
		return false;
	}

	uint8_t param[7];
	BtHciCapStoreLe16(&param[0], ConnHdl);
	param[2] = (TxPhys == 0 ? 0x01U : 0U) |
		(RxPhys == 0 ? 0x02U : 0U);
	param[3] = TxPhys;
	param[4] = RxPhys;
	BtHciCapStoreLe16(&param[5], PhyOptions);

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PHY,
		param, sizeof(param), nullptr, 0) == 0;
}

// Queue one controller->host packet. Hdl identifies the value, data copied up
// to PacketSize. Returns bytes queued, 0 if the fifo is full.
static size_t BtHciCtlrReceive(BtHciCtlrDev_t * const pDev, uint16_t Hdl, void * const pData, size_t Len)
{
	BtHciCtlrPkt_t *p = (BtHciCtlrPkt_t*)CFifoPut(pDev->hRxFifo);
	if (p)
	{
		// Data capacity is the element size less the packet header.
		size_t cap = pDev->PacketSize - BTHCICTLR_PKTHDR_LEN;
		size_t l = min(Len, cap);

		p->Len = l;
		p->ValHdl = Hdl;
		p->Off = 0;

		memcpy(p->Data, pData, l);

		return l;
	}

	return 0;
}

// DevIntrf ops. The HCI controller is not a byte bus: it exchanges formed
// packets through Send / SendCommand and the RX fifo, so the rate and rx/tx
// stream ops are no-ops.
static void BtHciCtlrIntrfDisable(DevIntrf_t * const) {}
static void BtHciCtlrIntrfEnable(DevIntrf_t * const) {}
static uint32_t BtHciCtlrIntrfGetRate(DevIntrf_t * const) { return 0; }
static uint32_t BtHciCtlrIntrfSetRate(DevIntrf_t * const, uint32_t) { return 0; }
static bool BtHciCtlrIntrfStartRx(DevIntrf_t * const, uint32_t) { return true; }
static int BtHciCtlrIntrfRxData(DevIntrf_t * const, uint8_t *, int) { return 0; }
static void BtHciCtlrIntrfStopRx(DevIntrf_t * const) {}

static bool BtHciCtlrIntrfStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	BtHciCtlrDev_t *p = (BtHciCtlrDev_t *)pDev->pDevData;

	p->ConnHdl = DevAddr >> 16;
	p->ValHdl = DevAddr & 0xFFFF;

	return true;
}

// Route a formed ACL packet down through the bound transport send. The target
// controller binds pDev->Send in its Enable (SDC binds it to sdc_hci_data_put).
static int BtHciCtlrIntrfTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	BtHciCtlrDev_t *p = (BtHciCtlrDev_t *)pDev->pDevData;

	return (p->Send != nullptr) ? (int)p->Send(p, (void*)pData, DataLen) : 0;
}

static void BtHciCtlrIntrfStopTx(DevIntrf_t * const) {}
static void BtHciCtlrIntrfPowerOff(DevIntrf_t * const) {}

bool BtHciCtlrInit(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->PacketSize == 0)
	{
		pDev->PacketSize = BT_HCI_CTLR_MTU_MAX + BTHCICTLR_PKTHDR_LEN;
	}
	else
	{
		if (pCfg->PacketSize > UINT32_MAX - BTHCICTLR_PKTHDR_LEN)
		{
			return false;
		}
		pDev->PacketSize = pCfg->PacketSize + BTHCICTLR_PKTHDR_LEN;
	}

	uint8_t *pRxFifoMem = pCfg->pRxFifoMem;
	uint32_t rxFifoMemSize;

	if (pRxFifoMem == nullptr)
	{
		pRxFifoMem = s_BtHciCtlrRxFifoMem;
		rxFifoMemSize = BTHCICTLR_FIFO_MEM_SIZE;
	}
	else
	{
		if (pCfg->RxFifoMemSize <= 0)
		{
			return false;
		}
		rxFifoMemSize = (uint32_t)pCfg->RxFifoMemSize;
	}

	// CFifo stores its header directly in the supplied block and therefore
	// requires word-aligned memory. It also accepts a block too small for one
	// element by returning a FIFO with MaxIdxCnt == 0, so enforce one complete
	// controller packet here before initialization.
	if (((uintptr_t)pRxFifoMem & (alignof(CFifo_t) - 1U)) != 0 ||
		rxFifoMemSize < CFIFO_TOTAL_MEMSIZE(1, pDev->PacketSize))
	{
		return false;
	}

	pDev->hRxFifo = CFifoInit(pRxFifoMem, rxFifoMemSize,
							  (uint32_t)pDev->PacketSize, true);
	if (pDev->hRxFifo == nullptr)
	{
		return false;
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_BT;
	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.Disable = BtHciCtlrIntrfDisable;
	pDev->DevIntrf.Enable = BtHciCtlrIntrfEnable;
	pDev->DevIntrf.GetRate = BtHciCtlrIntrfGetRate;
	pDev->DevIntrf.SetRate = BtHciCtlrIntrfSetRate;
	pDev->DevIntrf.StartRx = BtHciCtlrIntrfStartRx;
	pDev->DevIntrf.RxData = BtHciCtlrIntrfRxData;
	pDev->DevIntrf.StopRx = BtHciCtlrIntrfStopRx;
	pDev->DevIntrf.StartTx = BtHciCtlrIntrfStartTx;
	pDev->DevIntrf.TxData = BtHciCtlrIntrfTxData;
	pDev->DevIntrf.StopTx = BtHciCtlrIntrfStopTx;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.PowerOff = BtHciCtlrIntrfPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.EvtCB = pCfg->EvtHandler;
	pDev->Receive = BtHciCtlrReceive;
	pDev->Send = nullptr;			// bound by the target controller Enable
	pDev->SendCommand = nullptr;	// bound by the target controller Enable
	pDev->RxHandler = pCfg->RxHandler;
	pDev->OnWake = pCfg->OnWake;
	memset(&pDev->Capabilities, 0, sizeof(pDev->Capabilities));
	pDev->pHciDev = nullptr;
	pDev->CapabilitiesApplied = false;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

size_t BtHciCtlrSendCommand(BtHciCtlrDev_t * const pDev, uint16_t OpCode, const void *pParam, uint8_t ParamLen)
{
	if (pDev == nullptr || pDev->SendCommand == nullptr)
	{
		return 0;
	}

	if (ParamLen > 0 && pParam == nullptr)
	{
		return 0;
	}

	// Standard HCI command packet: opcode little endian, parameter total
	// length, then the parameters.
	uint8_t buf[BTHCICTLR_CMD_HDR_LEN + BTHCICTLR_CMD_PARAM_MAX];

	buf[0] = (uint8_t)(OpCode & 0xFF);
	buf[1] = (uint8_t)((OpCode >> 8) & 0xFF);
	buf[2] = ParamLen;

	if (ParamLen > 0 && pParam != nullptr)
	{
		memcpy(&buf[BTHCICTLR_CMD_HDR_LEN], pParam, ParamLen);
	}

	return pDev->SendCommand(pDev, buf, BTHCICTLR_CMD_HDR_LEN + ParamLen);
}

// Target controller bring-up. The weak default succeeds, which is right for a
// port whose controller needs nothing beyond the generic wiring: a plain HCI
// transport over UART or USB has no vendor stack to start.
__attribute__((weak)) bool BtHciCtlrStart(BtHciCtlrDev_t * const pDev,
										  const BtHciCtlrCfg_t *pCfg)
{
	(void)pDev;
	(void)pCfg;

	return true;
}

// One bring-up sequence for every port. The ordering is the point: generic
// init assigns RxHandler, Receive and the interface table, and returns early
// without them on a bad packet size, a misaligned or undersized RX FIFO
// buffer, or a CFifoInit failure. Starting the controller anyway leaves an
// enabled controller with no receive path, delivering nothing and saying
// nothing. This used to be each port's own function, and the SDC one
// discarded the init result.
bool BtHciCtlrEnable(BtHciCtlrDev_t * const pDev, const BtHciCtlrCfg_t *pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	if (BtHciCtlrInit(pDev, pCfg) == false ||
		BtHciCtlrStart(pDev, pCfg) == false)
	{
		return false;
	}

	s_pBtHciCtlrActive = pDev;
	return true;
}
