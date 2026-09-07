/**-------------------------------------------------------------------------
@file	usbd_hci.h

@brief	USB Bluetooth HCI device function.

UsbdHci implements the Bluetooth USB transport. HCI commands arrive on endpoint
zero, Events use a dedicated interrupt IN endpoint, ACL data uses the inherited
UsbIntrf bulk endpoint pair, and optional SCO data uses the synchronous
interface isochronous endpoint pair. Interface and endpoint numbers are
allocated internally. Optional Bulk Serialization mode adds HCI interface
alternate setting 1 and carries every HCI packet over the bulk endpoint pair
with the standard one-byte packet indicator.

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
#ifndef __USBD_HCI_H__
#define __USBD_HCI_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb.h"
#include "usb/usb_intrf.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_HCI_CONFIG_VALUE			1U
#define USBD_HCI_SUBCLASS_RF			0x01U
#define USBD_HCI_PROTOCOL_BT			0x01U
#define USBD_HCI_EVENT_FS_MPS			16U
#define USBD_HCI_EVENT_HS_MPS			16U
#define USBD_HCI_ACL_FS_MPS			64U
#define USBD_HCI_ACL_HS_MPS			512U
#define USBD_HCI_SCO_ALT_COUNT		6U
#define USBD_HCI_SCO_FS_INTERVAL		1U
#define USBD_HCI_SCO_HS_INTERVAL		4U
#define USBD_HCI_EVENT_FS_INTERVAL		1U
#define USBD_HCI_EVENT_HS_INTERVAL		1U

#define USBD_HCI_COMMAND_HEADER_SIZE		3U
#define USBD_HCI_EVENT_HEADER_SIZE		2U
#define USBD_HCI_ACL_HEADER_SIZE			4U
#define USBD_HCI_SCO_HEADER_SIZE			3U
#define USBD_HCI_ISO_HEADER_SIZE			4U
#define USBD_HCI_COMMAND_MAX_SIZE		258U
#define USBD_HCI_EVENT_MAX_SIZE			257U
#define USBD_HCI_PACKET_MAX_SIZE			1024U
#define USBD_HCI_SCO_MAX_SIZE			258U
#define USBD_HCI_SCO_MAX_MPS			63U
#define USBD_HCI_SCO_RX_BUFFER_COUNT	3U
#define USBD_HCI_SCO_BUFFER_NONE		0xFFU

#define USBD_HCI_EVENT_MAX_MPS			USB_PKT_MAXLEN(0, INT)
#define USBD_HCI_ACL_MAX_MPS			USB_PKT_MAXLEN(0, BULK)
#define USBD_HCI_ACL_PKT_BLKSIZE \
	USB_INTRF_PKT_BLKSIZE(USBD_HCI_ACL_MAX_MPS)
#define USBD_HCI_ACL_RXMEM_SIZE(NbPkt) \
	USB_INTRF_RXMEM_SIZE((NbPkt), USBD_HCI_ACL_MAX_MPS)
#define USBD_HCI_ACL_TXMEM_SIZE(NbPkt) \
	CFIFO_TOTAL_MEMSIZE((NbPkt), USBD_HCI_ACL_PKT_BLKSIZE)

typedef enum __Usbd_Hci_Packet_Type {
	USBD_HCI_PACKET_NONE = 0x00U,
	USBD_HCI_PACKET_COMMAND = 0x01U,
	USBD_HCI_PACKET_ACL = 0x02U,
	USBD_HCI_PACKET_SCO = 0x03U,
	USBD_HCI_PACKET_EVENT = 0x04U,
	USBD_HCI_PACKET_ISO = 0x05U,
} UsbdHciPacketType_t;

typedef int (*UsbdHciRxData_t)(DevIntrf_t * const pDev,
							   uint8_t *pBuffer, int BufferLen);
typedef int (*UsbdHciTxData_t)(DevIntrf_t * const pDev,
							   const uint8_t *pData, int DataLen);

#pragma pack(push, 1)

/// Descriptor fragment for the legacy Bluetooth USB HCI function.
typedef struct __Usbd_Hci_Descriptor {
	UsbInrtfAssDesc_t Association;
	UsbIntrfDesc_t Hci;
	UsbEndPointDesc_t EventIn;
	UsbEndPointDesc_t AclOut;
	UsbEndPointDesc_t AclIn;
	UsbIntrfDesc_t Sync;
} UsbdHciDesc_t;

typedef struct __Usbd_Hci_Sco_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} UsbdHciScoAltDesc_t;

typedef struct __Usbd_Hci_Serial_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} UsbdHciSerialAltDesc_t;

/// HCI legacy alt-0, serialized alt-1, then synchronous alt-0.
typedef struct __Usbd_Hci_Serial_Descriptor {
	UsbInrtfAssDesc_t Association;
	UsbIntrfDesc_t Hci;
	UsbEndPointDesc_t EventIn;
	UsbEndPointDesc_t AclOut;
	UsbEndPointDesc_t AclIn;
	UsbdHciSerialAltDesc_t Serialized;
	UsbIntrfDesc_t Sync;
} UsbdHciSerialDesc_t;

/// Legacy HCI descriptors followed by synchronous alternates 1 through 6.
typedef struct __Usbd_Hci_Sco_Descriptor {
	UsbdHciDesc_t Legacy;
	UsbdHciScoAltDesc_t Alt[USBD_HCI_SCO_ALT_COUNT];
} UsbdHciScoDesc_t;

/// Serialized HCI descriptors followed by synchronous alternates 1 through 6.
typedef struct __Usbd_Hci_Full_Descriptor {
	UsbdHciSerialDesc_t Base;
	UsbdHciScoAltDesc_t Alt[USBD_HCI_SCO_ALT_COUNT];
} UsbdHciFullDesc_t;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct __Usbd_Hci_Config {
	int DevNo;
	bool bBlocking;
	bool bSco;				//!< Add synchronous alternate settings and SCO transport
	bool bBulkSerialization;	//!< Add HCI alt-1 serialized bulk transport
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	uint8_t InterfaceString;
	uint16_t EventFsMps;			//!< Zero selects USBD_HCI_EVENT_FS_MPS
	uint16_t EventHsMps;			//!< Zero selects USBD_HCI_EVENT_HS_MPS
	uint16_t AclFsMps;				//!< Zero selects USBD_HCI_ACL_FS_MPS
	uint16_t AclHsMps;				//!< Zero selects USBD_HCI_ACL_HS_MPS
	uint8_t EventFsInterval;		//!< Zero selects USBD_HCI_EVENT_FS_INTERVAL
	uint8_t EventHsInterval;		//!< Zero selects USBD_HCI_EVENT_HS_INTERVAL
	DevIntrfEvtHandler_t EvtCB;
} UsbdHciCfg_t;

typedef struct __Usbd_Hci_Dev {
	UsbDevIntrf_t *pAcl;
	UsbdHciRxData_t AclRxData;
	UsbdHciTxData_t AclTxData;
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
	UsbdHciPacketType_t RxType;
	UsbdHciPacketType_t TxType;
	UsbdHciPacketType_t BulkRxType;
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
	uint16_t ScoRxPacketLength[USBD_HCI_SCO_RX_BUFFER_COUNT];
	uint16_t ScoTxLength;
	uint16_t ScoTxOffset;
	uint16_t ScoTxChunkLength;
	uint32_t CommandBuffer[(USBD_HCI_COMMAND_MAX_SIZE + 3U) / 4U];
	uint32_t AclRxBuffer[(USBD_HCI_PACKET_MAX_SIZE + 4U) / 4U];
	uint32_t AclRxTransfer[(USBD_HCI_ACL_MAX_MPS + 3U) / 4U];
	uint32_t AclTxTransfer[(USBD_HCI_ACL_MAX_MPS + 3U) / 4U];
	uint32_t AclTxPacket[(USBD_HCI_ACL_PKT_BLKSIZE + 3U) / 4U];
	uint32_t EventTxBuffer[(USBD_HCI_EVENT_MAX_SIZE + 3U) / 4U];
	uint32_t EventTxTransfer[(USBD_HCI_EVENT_MAX_MPS + 3U) / 4U];
	uint32_t ScoRxBuffer[USBD_HCI_SCO_RX_BUFFER_COUNT]
		[(USBD_HCI_SCO_MAX_SIZE + 3U) / 4U];
	uint32_t ScoRxTransfer[(USBD_HCI_SCO_MAX_MPS + 3U) / 4U];
	uint32_t ScoTxBuffer[(USBD_HCI_SCO_MAX_SIZE + 3U) / 4U];
	uint32_t ScoTxTransfer[(USBD_HCI_SCO_MAX_MPS + 3U) / 4U];
} UsbdHciDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbdHciInit(UsbdHciDev_t * const pHci,
				 UsbDevIntrf_t * const pAcl,
				 const UsbdHciCfg_t *pCfg);

/** Build the Bluetooth IAD, HCI interface/endpoints and sync alt-0 fragment. */
bool UsbdHciMakeDesc(UsbdHciDesc_t *pDesc, const UsbdHciDev_t *pHci,
					 UsbSpeed_t Speed);

/** Build the legacy fragment followed by synchronous alternates 1 through 6. */
bool UsbdHciMakeScoDesc(UsbdHciScoDesc_t *pDesc,
						const UsbdHciDev_t *pHci, UsbSpeed_t Speed);

/** Build HCI legacy alt-0, serialized alt-1 and synchronous alt-0. */
bool UsbdHciMakeSerialDesc(UsbdHciSerialDesc_t *pDesc,
						   const UsbdHciDev_t *pHci, UsbSpeed_t Speed);

/** Build serialized HCI and all synchronous alternate settings. */
bool UsbdHciMakeFullDesc(UsbdHciFullDesc_t *pDesc,
						 const UsbdHciDev_t *pHci, UsbSpeed_t Speed);

bool UsbdHciRequestToSend(UsbdHciDev_t *pHci, int NbBytes);

#ifdef __cplusplus
}

class UsbdHci : public UsbIntrf {
public:
	UsbdHci() = default;

	bool Init(const UsbdHciCfg_t &Cfg) {
		return UsbdHciInit(&vUsbdHci, &vUsbDevIntrf, &Cfg);
	}

	DevIntrf_t *Data(void) { return static_cast<DevIntrf_t *>(*this); }

	bool MakeDesc(UsbdHciDesc_t *pDesc, UsbSpeed_t Speed) const {
		return UsbdHciMakeDesc(pDesc, &vUsbdHci, Speed);
	}

	bool MakeScoDesc(UsbdHciScoDesc_t *pDesc, UsbSpeed_t Speed) const {
		return UsbdHciMakeScoDesc(pDesc, &vUsbdHci, Speed);
	}

	bool MakeSerialDesc(UsbdHciSerialDesc_t *pDesc, UsbSpeed_t Speed) const {
		return UsbdHciMakeSerialDesc(pDesc, &vUsbdHci, Speed);
	}

	bool MakeFullDesc(UsbdHciFullDesc_t *pDesc, UsbSpeed_t Speed) const {
		return UsbdHciMakeFullDesc(pDesc, &vUsbdHci, Speed);
	}

	bool RequestToSend(int NbBytes) override {
		return UsbdHciRequestToSend(&vUsbdHci, NbBytes);
	}

private:
	UsbdHciDev_t vUsbdHci = {};
};
#endif

/** @} End of group USBD */

#endif	// __USBD_HCI_H__
