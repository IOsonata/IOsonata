/**-------------------------------------------------------------------------
@file	usbd_msc.h

@brief	USB Mass Storage Bulk-Only Transport device class.

UsbdMsc owns the BOT and SCSI policy for one statically supplied DiskIO
medium. UsbIntrf owns the bidirectional bulk endpoint data path. The class
does not allocate or own the medium or the sector-transfer buffer.

----------------------------------------------------------------------------*/
#ifndef __USBD_MSC_H__
#define __USBD_MSC_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usb/usb.h"
#include "usb/usb_intrf.h"
#include "usb/usb_mscdef.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_MSC_CONFIG_VALUE			1U
#define USBD_MSC_FS_MPS				64U
#define USBD_MSC_HS_MPS				512U
#define USBD_MSC_MAX_MPS			USB_PKT_MAXLEN(0, BULK)
#define USBD_MSC_FIFO_PKT_COUNT		2U
#define USBD_MSC_PKT_BLKSIZE			USB_INTRF_PKT_BLKSIZE(USBD_MSC_MAX_MPS)
#define USBD_MSC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(USBD_MSC_FIFO_PKT_COUNT, USBD_MSC_MAX_MPS)
#define USBD_MSC_TXFIFO_MEMSIZE \
	CFIFO_TOTAL_MEMSIZE(USBD_MSC_FIFO_PKT_COUNT, USBD_MSC_PKT_BLKSIZE)

typedef enum __Usbd_Msc_Bot_State {
	USBD_MSC_BOT_WAIT_CBW,
	USBD_MSC_BOT_DATA_IN,
	USBD_MSC_BOT_DATA_OUT,
	USBD_MSC_BOT_SEND_CSW,
	USBD_MSC_BOT_FAILED,
	USBD_MSC_BOT_PHASE_ERROR,
	USBD_MSC_BOT_RESET_RECOVERY,
} UsbdMscBotState_t;

#pragma pack(push, 1)

typedef struct __Usbd_Msc_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} UsbdMscDesc_t;

#pragma pack(pop)

#ifdef __cplusplus

class DiskIO;

typedef struct __Usbd_Msc_Config {
	int DevNo;
	DiskIO *pDisk;
	uint8_t *pSectorBuffer;
	uint16_t SectorBufferSize;
	bool bReadOnly;
	bool bRemovable;
	uint8_t InterfaceString;
	uint16_t FsMps;				//!< Zero selects USBD_MSC_FS_MPS
	uint16_t HsMps;				//!< Zero selects USBD_MSC_HS_MPS
	const char *pVendor;			//!< INQUIRY vendor, padded or truncated to 8 bytes
	const char *pProduct;		//!< INQUIRY product, padded or truncated to 16 bytes
	const char *pRevision;		//!< INQUIRY revision, padded or truncated to 4 bytes
} UsbdMscCfg_t;

typedef enum __Usbd_Msc_Tx_Kind {
	USBD_MSC_TX_NONE,
	USBD_MSC_TX_DATA,
	USBD_MSC_TX_ZLP,
	USBD_MSC_TX_CSW,
} UsbdMscTxKind_t;

// Natural alignment is required by the embedded DevIntrf_t atomics and by
// controller DMA buffers.
typedef struct __Usbd_Msc_Dev {
	UsbDevIntrf_t IntrfData;
	DiskIO *pDisk;
	uint8_t *pSectorBuffer;
	uint16_t SectorBufferSize;
	uint16_t SectorSize;
	uint32_t SectorCount;
	int ItfNo;
	int DevNo;
	uint8_t EpNo;
	uint8_t InterfaceString;
	uint16_t FsMps;
	uint16_t HsMps;
	bool bReadOnly;
	bool bRemovable;
	bool bConfigured;
	bool bCommandFailed;
	bool bPhaseError;
	bool bStallAfterData;
	bool bNeedZlp;
	bool bResetSeen;
	bool bRecoveryAfterCsw;
	bool bTxFailed;
	UsbdMscBotState_t State;
	UsbdMscTxKind_t PendingTx;
	uint16_t PendingTxLength;
	uint8_t SenseKey;
	uint8_t SenseAsc;
	uint8_t SenseAscq;
	uint32_t HostLength;
	uint32_t DeviceLength;
	uint32_t TransferLimit;
	uint32_t Transferred;
	uint32_t Lba;
	uint32_t BlocksRemaining;
	uint16_t SectorOffset;
	uint16_t ResponseLength;
	uint16_t ResponseOffset;
	UsbMscCmdBlkWrapper_t Cbw;
	UsbMscCmdStatusWrapper_t Csw;
	UsbdMscDesc_t FsDesc;
	UsbdMscDesc_t HsDesc;
	char Vendor[8];
	char Product[16];
	char Revision[4];
	uint8_t Response[36];
	uint32_t RxFifo[(USBD_MSC_RXFIFO_MEMSIZE + sizeof(uint32_t) - 1U) /
					 sizeof(uint32_t)];
	uint32_t TxFifo[(USBD_MSC_TXFIFO_MEMSIZE + sizeof(uint32_t) - 1U) /
					 sizeof(uint32_t)];
	uint32_t RxTransfer[(USBD_MSC_MAX_MPS + sizeof(uint32_t) - 1U) /
					 sizeof(uint32_t)];
	uint32_t TxTransfer[(USBD_MSC_MAX_MPS + sizeof(uint32_t) - 1U) /
					 sizeof(uint32_t)];
	uint32_t TxPacket[(USBD_MSC_PKT_BLKSIZE + sizeof(uint32_t) - 1U) /
					 sizeof(uint32_t)];
	uint32_t RxPacket[(USBD_MSC_MAX_MPS + sizeof(uint32_t) - 1U) /
					 sizeof(uint32_t)];
	uint8_t MaxLun;
} UsbdMscDev_t;

class UsbdMsc : public UsbDeviceClass, public DeviceIntrf {
public:
	UsbdMsc() = default;
	UsbdMsc(const UsbdMsc &) = delete;
	UsbdMsc &operator = (const UsbdMsc &) = delete;

	operator DevIntrf_t * () override { return &vUsbdMsc.IntrfData.DevIntrf; }
	operator UsbdMscDev_t * () { return &vUsbdMsc; }
	DevIntrf_t *Data(void) { return &vUsbdMsc.IntrfData.DevIntrf; }

	bool Init(const UsbdMscCfg_t &Cfg);
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override;
	bool SelectConfig(uint8_t ConfigValue) override;
	void Reset(void) override;
	void Process(void) override;

	UsbdMscBotState_t BotState(void) const { return vUsbdMsc.State; }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vUsbdMsc.IntrfData.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vUsbdMsc.IntrfData.DevIntrf);
	}

	bool RequestToSend(int NbBytes) override {
		return UsbIntrfRequestToSend(&vUsbdMsc.IntrfData, NbBytes);
	}

	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vUsbdMsc.IntrfData.DevIntrf, DevAddr,
			pData, DataLen);
	}

	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vUsbdMsc.IntrfData.DevIntrf, DevAddr,
			pBuff, BuffLen);
	}

	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vUsbdMsc.IntrfData.DevIntrf,
			pData, DataLen);
	}

	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vUsbdMsc.IntrfData.DevIntrf,
			pBuff, BuffLen);
	}

private:
	UsbdMscDev_t vUsbdMsc = {};
};

#endif

/** @} End of group USBD */

#endif	// __USBD_MSC_H__
