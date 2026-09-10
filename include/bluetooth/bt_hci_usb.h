/**-------------------------------------------------------------------------
@file	bt_hci_usb.h

@brief	Bluetooth HCI USB transport.

BtHciUsb implements the Bluetooth USB transport. HCI commands arrive on endpoint
zero, Events use a dedicated interrupt IN endpoint, ACL data uses the inherited
UsbIntrf bulk endpoint pair, and optional SCO data uses the synchronous
interface through UsbIsoIntrf. Interface and endpoint numbers are allocated
internally. Optional Bulk Serialization mode adds HCI interface alternate
setting 1 and carries every HCI packet over the bulk endpoint pair with the
standard one-byte packet indicator.

One successful DeviceIntrf transfer is one complete HCI packet. DevAddr is the
HCI packet type. USB packetization remains internal and no H:4 type byte is
added to a legacy Bluetooth USB transfer.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __BT_HCI_USB_H__
#define __BT_HCI_USB_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb.h"
#include "usb/usb_intrf.h"
#include "usb/usb_iso.h"

/** @addtogroup Bluetooth
  * @{
  */

#define BT_HCI_USB_CONFIG_VALUE			1U
#define BT_HCI_USB_SUBCLASS_RF			0x01U
#define BT_HCI_USB_PROTOCOL_BT			0x01U
#define BT_HCI_USB_EVENT_FS_MPS			16U
#define BT_HCI_USB_EVENT_HS_MPS			16U
#define BT_HCI_USB_ACL_FS_MPS			64U
#define BT_HCI_USB_ACL_HS_MPS			512U
#define BT_HCI_USB_SCO_ALT_COUNT		6U
#define BT_HCI_USB_SCO_FS_INTERVAL		1U
#define BT_HCI_USB_SCO_HS_INTERVAL		4U
#define BT_HCI_USB_EVENT_FS_INTERVAL		1U
#define BT_HCI_USB_EVENT_HS_INTERVAL		1U

#define BT_HCI_USB_COMMAND_HEADER_SIZE		3U
#define BT_HCI_USB_EVENT_HEADER_SIZE		2U
#define BT_HCI_USB_ACL_HEADER_SIZE			4U
#define BT_HCI_USB_SCO_HEADER_SIZE			3U
#define BT_HCI_USB_ISO_HEADER_SIZE			4U
#define BT_HCI_USB_COMMAND_MAX_SIZE		258U
#define BT_HCI_USB_EVENT_MAX_SIZE			257U
#define BT_HCI_USB_PACKET_MAX_SIZE			1024U
#define BT_HCI_USB_SCO_MAX_SIZE			258U
#define BT_HCI_USB_SCO_MAX_MPS			63U
#define BT_HCI_USB_SCO_RX_BUFFER_COUNT	3U
#define BT_HCI_USB_SCO_BUFFER_NONE		0xFFU

#define BT_HCI_USB_EVENT_MAX_MPS			USB_PKT_MAXLEN(0, INT)
#define BT_HCI_USB_ACL_MAX_MPS			USB_PKT_MAXLEN(0, BULK)
#define BT_HCI_USB_ACL_PKT_BLKSIZE \
	USB_INTRF_PKT_BLKSIZE(BT_HCI_USB_ACL_MAX_MPS)
#define BT_HCI_USB_ACL_RXMEM_SIZE(NbPkt) \
	USB_INTRF_RXMEM_SIZE((NbPkt), BT_HCI_USB_ACL_MAX_MPS)
#define BT_HCI_USB_ACL_TXMEM_SIZE(NbPkt) \
	CFIFO_TOTAL_MEMSIZE((NbPkt), BT_HCI_USB_ACL_PKT_BLKSIZE)

typedef enum __Bt_Hci_Usb_Packet_Type {
	BT_HCI_USB_PACKET_NONE = 0x00U,
	BT_HCI_USB_PACKET_COMMAND = 0x01U,
	BT_HCI_USB_PACKET_ACL = 0x02U,
	BT_HCI_USB_PACKET_SCO = 0x03U,
	BT_HCI_USB_PACKET_EVENT = 0x04U,
	BT_HCI_USB_PACKET_ISO = 0x05U,
} BtHciUsbPacketType_t;

typedef int (*BtHciUsbRxData_t)(DevIntrf_t * const pDev,
							   uint8_t *pBuffer, int BufferLen);
typedef int (*BtHciUsbTxData_t)(DevIntrf_t * const pDev,
							   const uint8_t *pData, int DataLen);

#pragma pack(push, 1)

/// Descriptor fragment for the legacy Bluetooth USB HCI function.
typedef struct __Bt_Hci_Usb_Descriptor {
	UsbInrtfAssDesc_t Association;
	UsbIntrfDesc_t Hci;
	UsbEndPointDesc_t EventIn;
	UsbEndPointDesc_t AclOut;
	UsbEndPointDesc_t AclIn;
	UsbIntrfDesc_t Sync;
} BtHciUsbDesc_t;

typedef struct __Bt_Hci_Usb_Sco_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} BtHciUsbScoAltDesc_t;

typedef struct __Bt_Hci_Usb_Serial_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} BtHciUsbSerialAltDesc_t;

/// HCI legacy alt-0, serialized alt-1, then synchronous alt-0.
typedef struct __Bt_Hci_Usb_Serial_Descriptor {
	UsbInrtfAssDesc_t Association;
	UsbIntrfDesc_t Hci;
	UsbEndPointDesc_t EventIn;
	UsbEndPointDesc_t AclOut;
	UsbEndPointDesc_t AclIn;
	BtHciUsbSerialAltDesc_t Serialized;
	UsbIntrfDesc_t Sync;
} BtHciUsbSerialDesc_t;

/// Legacy HCI descriptors followed by synchronous alternates 1 through 6.
typedef struct __Bt_Hci_Usb_Sco_Descriptor {
	BtHciUsbDesc_t Legacy;
	BtHciUsbScoAltDesc_t Alt[BT_HCI_USB_SCO_ALT_COUNT];
} BtHciUsbScoDesc_t;

/// Serialized HCI descriptors followed by synchronous alternates 1 through 6.
typedef struct __Bt_Hci_Usb_Full_Descriptor {
	BtHciUsbSerialDesc_t Base;
	BtHciUsbScoAltDesc_t Alt[BT_HCI_USB_SCO_ALT_COUNT];
} BtHciUsbFullDesc_t;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct __Bt_Hci_Usb_Config {
	int DevNo;
	bool bBlocking;
	bool bSco;				//!< Add synchronous alternate settings and SCO transport
	bool bBulkSerialization;	//!< Add HCI alt-1 serialized bulk transport
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	uint8_t InterfaceString;
	uint16_t EventFsMps;			//!< Zero selects BT_HCI_USB_EVENT_FS_MPS
	uint16_t EventHsMps;			//!< Zero selects BT_HCI_USB_EVENT_HS_MPS
	uint16_t AclFsMps;				//!< Zero selects BT_HCI_USB_ACL_FS_MPS
	uint16_t AclHsMps;				//!< Zero selects BT_HCI_USB_ACL_HS_MPS
	uint8_t EventFsInterval;		//!< Zero selects BT_HCI_USB_EVENT_FS_INTERVAL
	uint8_t EventHsInterval;		//!< Zero selects BT_HCI_USB_EVENT_HS_INTERVAL
	// Descriptor fragment buffers. Init fills the one matching the selected
	// layout with the allocated interface and endpoint numbers and the
	// controller speed MPS, so the application places it in its configuration
	// descriptor and does not build it separately. Set the buffer for the
	// chosen bSco/bBulkSerialization combination and leave the rest null:
	// pDesc when neither flag is set, pScoDesc for bSco only, pSerialDesc for
	// bBulkSerialization only, pFullDesc when both are set.
	BtHciUsbDesc_t *pDesc;
	BtHciUsbScoDesc_t *pScoDesc;
	BtHciUsbSerialDesc_t *pSerialDesc;
	BtHciUsbFullDesc_t *pFullDesc;
	DevIntrfEvtHandler_t EvtCB;
} BtHciUsbCfg_t;

#pragma pack(pop)

// Natural alignment: IntrfData and ScoIso embed DevIntrf_t whose pointer and
// atomic members must stay naturally aligned on 64-bit host test builds.
typedef struct __Bt_Hci_Usb_Dev {
	UsbDevIntrf_t IntrfData;		//!< ACL endpoint data path, owned by value
	UsbIsoIntrf_t ScoIso;
	BtHciUsbRxData_t AclRxData;
	BtHciUsbTxData_t AclTxData;
	DevIntrfEvtHandler_t EvtCB;
	int HciItfNo;					//!< Internal allocation
	int SyncItfNo;					//!< Internal allocation
	int DevNo;
	uint8_t EventEpNo;				//!< Internal IN endpoint allocation
	uint8_t AclEpNo;				//!< Internal bidirectional endpoint allocation
	uint8_t ScoEpNo;				//!< Internal isochronous endpoint allocation
	uint8_t InterfaceString;
	uint16_t EventFsMps;
	uint16_t EventHsMps;
	uint16_t AclFsMps;
	uint16_t AclHsMps;
	uint8_t EventFsInterval;
	uint8_t EventHsInterval;
	BtHciUsbPacketType_t RxType;
	BtHciUsbPacketType_t TxType;
	BtHciUsbPacketType_t BulkRxType;
	bool Configured;
	bool ScoEnabled;
	bool BulkSerializationSupported;
	bool BulkSerialization;
	bool CommandPending;
	bool AclRxPending;
	bool EventTxActive;
	bool EventTxNeedZlp;
	bool EventTxZlp;
	bool ScoTxActive;
	uint8_t ScoAlt;
	uint8_t HciAlt;
	uint8_t ScoRxBuildIndex;
	uint8_t ScoRxPendingIndex;
	uint8_t ScoRxReadIndex;
	uint16_t CommandLength;
	uint16_t AclRxLength;
	uint16_t AclRxExpected;
	uint16_t EventTxLength;
	uint16_t EventTxOffset;
	uint16_t EventTxChunkLength;
	uint16_t ScoRxLength;
	uint16_t ScoRxExpected;
	uint16_t ScoRxPacketLength[BT_HCI_USB_SCO_RX_BUFFER_COUNT];
	uint16_t ScoTxLength;
	uint16_t ScoTxOffset;
	uint16_t ScoTxChunkLength;
	uint32_t CommandBuffer[(BT_HCI_USB_COMMAND_MAX_SIZE + 3U) / 4U];
	uint32_t AclRxBuffer[(BT_HCI_USB_PACKET_MAX_SIZE + 4U) / 4U];
	uint32_t AclRxTransfer[(BT_HCI_USB_ACL_MAX_MPS + 3U) / 4U];
	uint32_t AclTxTransfer[(BT_HCI_USB_ACL_MAX_MPS + 3U) / 4U];
	uint32_t AclTxPacket[(BT_HCI_USB_ACL_PKT_BLKSIZE + 3U) / 4U];
	uint32_t EventTxBuffer[(BT_HCI_USB_EVENT_MAX_SIZE + 3U) / 4U];
	uint32_t EventTxTransfer[(BT_HCI_USB_EVENT_MAX_MPS + 3U) / 4U];
	uint32_t ScoRxBuffer[BT_HCI_USB_SCO_RX_BUFFER_COUNT]
		[(BT_HCI_USB_SCO_MAX_SIZE + 3U) / 4U];
	uint32_t ScoTxBuffer[(BT_HCI_USB_SCO_MAX_SIZE + 3U) / 4U];
} BtHciUsbDev_t;

#ifdef __cplusplus
extern "C" {
#endif

bool BtHciUsbInit(BtHciUsbDev_t * const pHci, const BtHciUsbCfg_t *pCfg);

bool BtHciUsbRequestToSend(BtHciUsbDev_t *pHci, int NbBytes);

#ifdef __cplusplus
}

class BtHciUsb : public UsbDeviceClass, public DeviceIntrf {
public:
	BtHciUsb() = default;
	BtHciUsb(const BtHciUsb &) = delete;
	BtHciUsb &operator = (const BtHciUsb &) = delete;

	bool Init(const BtHciUsbCfg_t &Cfg);
	bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override;

	operator DevIntrf_t * () override {
		return &vBtHciUsb.IntrfData.DevIntrf;
	}
	operator BtHciUsbDev_t * () { return &vBtHciUsb; }
	DevIntrf_t *Data(void) { return &vBtHciUsb.IntrfData.DevIntrf; }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vBtHciUsb.IntrfData.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vBtHciUsb.IntrfData.DevIntrf);
	}

	__attribute__((always_inline))
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vBtHciUsb.IntrfData.DevIntrf, DevAddr, pData, DataLen);
	}

	__attribute__((always_inline))
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vBtHciUsb.IntrfData.DevIntrf, DevAddr, pBuff, BuffLen);
	}

	__attribute__((always_inline))
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vBtHciUsb.IntrfData.DevIntrf, pData, DataLen);
	}

	__attribute__((always_inline))
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vBtHciUsb.IntrfData.DevIntrf, pBuff, BuffLen);
	}

	bool RequestToSend(int NbBytes) override {
		return BtHciUsbRequestToSend(&vBtHciUsb, NbBytes);
	}

private:
	BtHciUsbDev_t vBtHciUsb = {};
};
#endif

/** @} End of group Bluetooth */

#endif	// __BT_HCI_USB_H__
