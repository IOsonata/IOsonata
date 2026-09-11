/**-------------------------------------------------------------------------
@file	usbd_msc.cpp

@brief	USB Mass Storage Bulk-Only Transport device implementation.

----------------------------------------------------------------------------*/
#include <string.h>

#include "storage/diskio.h"
#include "usb/usbd_epalloc.h"
#include "usb/usbd_msc.h"

typedef enum __Usbd_Msc_Data_Direction {
	USBD_MSC_DATA_NONE,
	USBD_MSC_DATA_IN,
	USBD_MSC_DATA_OUT,
} UsbdMscDataDirection_t;

static uint16_t UsbdMscGetBe16(const uint8_t *pData)
{
	return (uint16_t)(((uint16_t)pData[0] << 8) | pData[1]);
}

static uint32_t UsbdMscGetBe32(const uint8_t *pData)
{
	return ((uint32_t)pData[0] << 24) | ((uint32_t)pData[1] << 16) |
		((uint32_t)pData[2] << 8) | pData[3];
}

static void UsbdMscPutBe32(uint8_t *pData, uint32_t Value)
{
	pData[0] = (uint8_t)(Value >> 24);
	pData[1] = (uint8_t)(Value >> 16);
	pData[2] = (uint8_t)(Value >> 8);
	pData[3] = (uint8_t)Value;
}

static uint16_t UsbdMscMps(const UsbdMscDev_t *pMsc)
{
	return UsbCtrlrHighSpeed(pMsc->DevNo) ? pMsc->HsMps : pMsc->FsMps;
}

static uint8_t *UsbdMscRxBuffer(UsbdMscDev_t *pMsc)
{
	return reinterpret_cast<uint8_t *>(pMsc->RxTransfer);
}

static uint8_t *UsbdMscTxBuffer(UsbdMscDev_t *pMsc)
{
	return reinterpret_cast<uint8_t *>(pMsc->TxTransfer);
}

static uint8_t *UsbdMscRxPacket(UsbdMscDev_t *pMsc)
{
	return reinterpret_cast<uint8_t *>(pMsc->RxPacket);
}

static UsbPkt_t *UsbdMscTxPacket(UsbdMscDev_t *pMsc)
{
	return reinterpret_cast<UsbPkt_t *>(pMsc->TxPacket);
}

static void UsbdMscSetSense(UsbdMscDev_t *pMsc, uint8_t Key,
							 uint8_t Asc, uint8_t Ascq = 0U)
{
	pMsc->SenseKey = Key;
	pMsc->SenseAsc = Asc;
	pMsc->SenseAscq = Ascq;
}

static void UsbdMscFail(UsbdMscDev_t *pMsc, uint8_t Key, uint8_t Asc)
{
	pMsc->bCommandFailed = true;
	UsbdMscSetSense(pMsc, Key, Asc);
}

static void UsbdMscFail(UsbdMscDev_t *pMsc, uint8_t Key, uint8_t Asc,
							 uint8_t Ascq)
{
	pMsc->bCommandFailed = true;
	UsbdMscSetSense(pMsc, Key, Asc, Ascq);
}

static bool UsbdMscMediumReady(UsbdMscDev_t *pMsc)
{
	if (!pMsc->bMediumPresent)
	{
		UsbdMscFail(pMsc, USB_MSC_SENSE_NOT_READY,
			USB_MSC_ASC_MEDIUM_NOT_PRESENT);
		return false;
	}
	if (!pMsc->bMediumReady)
	{
		UsbdMscFail(pMsc, USB_MSC_SENSE_NOT_READY,
			USB_MSC_ASC_LUN_NOT_READY,
			USB_MSC_ASCQ_INITIALIZING_REQUIRED);
		return false;
	}
	return true;
}

static void UsbdMscCopyInquiry(char *pDest, size_t Length,
							const char *pSource)
{
	memset(pDest, ' ', Length);
	if (pSource == nullptr)
	{
		return;
	}

	size_t length = 0U;
	while (length < Length && pSource[length] != '\0')
	{
		length++;
	}
	memcpy(pDest, pSource, length);
}

static bool UsbdMscOpenEndpoint(UsbdMscDev_t *pMsc, uint8_t EpAddr,
								uint16_t Mps)
{
	UsbEndPointDesc_t desc = {};
	desc.bLength = sizeof(desc);
	desc.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.bEndpointAddress = EpAddr;
	desc.bmAttributes = USB_ENDPATT_TRANS_BULK;
	desc.wMaxPacketSize = Mps;
	desc.bInterval = 0U;
	return UsbCtrlrEpOpen(pMsc->DevNo, &desc);
}

static void UsbdMscCloseEndpoints(UsbdMscDev_t *pMsc)
{
	UsbCtrlrEpClose(pMsc->DevNo, USB_ENDPADDR_DIROUT(pMsc->EpNo));
	UsbCtrlrEpClose(pMsc->DevNo, USB_ENDPADDR_DIRIN(pMsc->EpNo));
}

static bool UsbdMscRestartEndpoints(UsbdMscDev_t *pMsc)
{
	const uint8_t in = USB_ENDPADDR_DIRIN(pMsc->EpNo);
	const uint8_t out = USB_ENDPADDR_DIROUT(pMsc->EpNo);
	const bool haltIn = UsbEpHalted(pMsc->DevNo, in);
	const bool haltOut = UsbEpHalted(pMsc->DevNo, out);
	const uint16_t mps = UsbdMscMps(pMsc);

	UsbdMscCloseEndpoints(pMsc);
	if (!UsbIntrfConfigure(&pMsc->IntrfData, mps) ||
		!UsbdMscOpenEndpoint(pMsc, in, mps) ||
		!UsbdMscOpenEndpoint(pMsc, out, mps))
	{
		UsbdMscCloseEndpoints(pMsc);
		UsbIntrfUnconfigure(&pMsc->IntrfData);
		pMsc->bConfigured = false;
		return false;
	}
	if (haltIn)
	{
		(void)UsbEpSetHalt(pMsc->DevNo, in, true);
	}
	if (haltOut)
	{
		(void)UsbEpSetHalt(pMsc->DevNo, out, true);
	}
	return true;
}

static bool UsbdMscFillDesc(UsbdMscDesc_t *pDesc,
							 const UsbdMscDev_t *pMsc, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || pMsc == nullptr || pMsc->ItfNo < 0 ||
		pMsc->ItfNo > UINT8_MAX || pMsc->EpNo == 0U || pMsc->EpNo > 15U)
	{
		return false;
	}

	const uint16_t mps = Speed == USB_SPEED_HIGH ? pMsc->HsMps : pMsc->FsMps;
	if (mps == 0U || mps > USBD_MSC_MAX_MPS)
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));
	pDesc->Interface.bLength = sizeof(pDesc->Interface);
	pDesc->Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Interface.bInterfaceNumber = (uint8_t)pMsc->ItfNo;
	pDesc->Interface.bNumEndpoints = 2U;
	pDesc->Interface.bInterfaceClass = USB_INTRFCLASS_MSC;
	pDesc->Interface.bInterfaceSubClass = USB_MSC_SUBCLASS_SCSI;
	pDesc->Interface.bInterfaceProtocol = USB_MSC_PROT_BULK;
	pDesc->Interface.iInterface = pMsc->InterfaceString;

	pDesc->Out.bLength = sizeof(pDesc->Out);
	pDesc->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pMsc->EpNo);
	pDesc->Out.bmAttributes = USB_ENDPATT_TRANS_BULK;
	pDesc->Out.wMaxPacketSize = mps;
	pDesc->In = pDesc->Out;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pMsc->EpNo);
	return true;
}

static bool UsbdMscTxIdle(const UsbdMscDev_t *pMsc)
{
	return atomic_load_explicit(&pMsc->IntrfData.DevIntrf.bTxReady,
		memory_order_acquire);
}

static void UsbdMscClearCommand(UsbdMscDev_t *pMsc)
{
	pMsc->bCommandFailed = false;
	pMsc->bPhaseError = false;
	pMsc->bStallAfterData = false;
	pMsc->bNeedZlp = false;
	pMsc->bRecoveryAfterCsw = false;
	pMsc->bTxFailed = false;
	pMsc->PendingTx = USBD_MSC_TX_NONE;
	pMsc->PendingTxLength = 0U;
	pMsc->HostLength = 0U;
	pMsc->DeviceLength = 0U;
	pMsc->TransferLimit = 0U;
	pMsc->Transferred = 0U;
	pMsc->Lba = 0U;
	pMsc->BlocksRemaining = 0U;
	pMsc->SectorOffset = 0U;
	pMsc->ResponseLength = 0U;
	pMsc->ResponseOffset = 0U;
	memset(&pMsc->Cbw, 0, sizeof(pMsc->Cbw));
	memset(&pMsc->Csw, 0, sizeof(pMsc->Csw));
}

static void UsbdMscResetBot(UsbdMscDev_t *pMsc, bool ResetMedium)
{
	UsbdMscClearCommand(pMsc);
	pMsc->bResetSeen = false;
	pMsc->State = USBD_MSC_BOT_WAIT_CBW;
	UsbdMscSetSense(pMsc, USB_MSC_SENSE_NONE, 0U);
	if (pMsc->bConfigured)
	{
		(void)UsbIntrfConfigure(&pMsc->IntrfData, UsbdMscMps(pMsc));
	}
	else
	{
		UsbIntrfUnconfigure(&pMsc->IntrfData);
	}
	if (ResetMedium && pMsc->pDisk != nullptr)
	{
		pMsc->bRemovalPrevented = false;
		pMsc->pDisk->Reset();
	}
}

static int UsbdMscDataEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
							 uint8_t *, int)
{
	if (pDev == nullptr || pDev->pDevData == nullptr)
	{
		return 0;
	}

	UsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pDev->pDevData);
	UsbdMscDev_t *pMsc = static_cast<UsbdMscDev_t *>(pIntrf->pClassContext);
	if (pMsc != nullptr && Event == DEVINTRF_EVT_TX_TIMEOUT)
	{
		pMsc->bTxFailed = true;
	}
	return 0;
}

static void UsbdMscStall(UsbdMscDev_t *pMsc, UsbdMscDataDirection_t Direction)
{
	const uint8_t epAddr = Direction == USBD_MSC_DATA_IN ?
		USB_ENDPADDR_DIRIN(pMsc->EpNo) : USB_ENDPADDR_DIROUT(pMsc->EpNo);
	(void)UsbEpSetHalt(pMsc->DevNo, epAddr, true);
}

static void UsbdMscFinishCommand(UsbdMscDev_t *pMsc)
{
	pMsc->Csw.dCSWSignature = USB_MSC_CSW_SIGNATURE;
	pMsc->Csw.dCSWTag = pMsc->Cbw.dCBWTag;
	pMsc->Csw.dCSWDataResidue = pMsc->HostLength - pMsc->Transferred;
	pMsc->Csw.bCSWStatus = pMsc->bPhaseError ?
		USB_MSC_CMDSTATUS_PHASE_ERR :
		(pMsc->bCommandFailed ? USB_MSC_CMDSTATUS_FAILED :
		 USB_MSC_CMDSTATUS_PASS);

	if (pMsc->bStallAfterData)
	{
		const UsbdMscDataDirection_t direction =
			(pMsc->Cbw.bmCBWFlags & USB_MSC_CBW_FLAG_IN) != 0U ?
			USBD_MSC_DATA_IN : USBD_MSC_DATA_OUT;
		UsbdMscStall(pMsc, direction);
	}

	pMsc->State = pMsc->bPhaseError ? USBD_MSC_BOT_PHASE_ERROR :
		(pMsc->bCommandFailed ? USBD_MSC_BOT_FAILED :
		 USBD_MSC_BOT_SEND_CSW);
	pMsc->bRecoveryAfterCsw = pMsc->bPhaseError;
}

static bool UsbdMscQueuePacket(UsbdMscDev_t *pMsc, const uint8_t *pData,
								uint16_t Length, UsbdMscTxKind_t Kind)
{
	if (!UsbdMscTxIdle(pMsc) || pMsc->PendingTx != USBD_MSC_TX_NONE ||
		Length > pMsc->IntrfData.Mps)
	{
		return false;
	}

	UsbPkt_t *pPacket = UsbdMscTxPacket(pMsc);
	memset(pPacket, 0, USBD_MSC_PKT_BLKSIZE);
	pPacket->Hdr.Length = Length;
	if (Length > 0U)
	{
		memcpy(pPacket->Data, pData, Length);
	}
	if (DeviceIntrfTxData(&pMsc->IntrfData.DevIntrf,
		reinterpret_cast<uint8_t *>(pPacket), USBD_MSC_PKT_BLKSIZE) !=
		(int)USBD_MSC_PKT_BLKSIZE)
	{
		return false;
	}
	pMsc->PendingTx = Kind;
	pMsc->PendingTxLength = Length;
	return true;
}

static void UsbdMscCompletePendingTx(UsbdMscDev_t *pMsc)
{
	if (pMsc->PendingTx == USBD_MSC_TX_NONE || !UsbdMscTxIdle(pMsc))
	{
		return;
	}

	const UsbdMscTxKind_t kind = pMsc->PendingTx;
	const uint16_t length = pMsc->PendingTxLength;
	pMsc->PendingTx = USBD_MSC_TX_NONE;
	pMsc->PendingTxLength = 0U;

	if (kind == USBD_MSC_TX_DATA)
	{
		pMsc->Transferred += length;
		if (pMsc->ResponseLength != 0U)
		{
			pMsc->ResponseOffset = (uint16_t)(pMsc->ResponseOffset + length);
		}
		else
		{
			pMsc->SectorOffset = (uint16_t)(pMsc->SectorOffset + length);
			if (pMsc->SectorOffset == pMsc->SectorSize)
			{
				pMsc->SectorOffset = 0U;
				pMsc->Lba++;
				pMsc->BlocksRemaining--;
			}
		}
	}
	else if (kind == USBD_MSC_TX_CSW)
	{
		const bool recovery = pMsc->bRecoveryAfterCsw;
		UsbdMscClearCommand(pMsc);
		if (recovery)
		{
			pMsc->State = USBD_MSC_BOT_RESET_RECOVERY;
			pMsc->bResetSeen = false;
		}
		else
		{
			pMsc->State = USBD_MSC_BOT_WAIT_CBW;
		}
	}
}

static void UsbdMscPrepareInquiry(UsbdMscDev_t *pMsc)
{
	memset(pMsc->Response, 0, sizeof(pMsc->Response));
	pMsc->Response[0] = 0x00U;
	pMsc->Response[1] = pMsc->bRemovable ? 0x80U : 0x00U;
	pMsc->Response[2] = 0x06U;
	pMsc->Response[3] = 0x02U;
	pMsc->Response[4] = 31U;
	memcpy(&pMsc->Response[8], pMsc->Vendor, sizeof(pMsc->Vendor));
	memcpy(&pMsc->Response[16], pMsc->Product, sizeof(pMsc->Product));
	memcpy(&pMsc->Response[32], pMsc->Revision, sizeof(pMsc->Revision));
}

static void UsbdMscPrepareSense(UsbdMscDev_t *pMsc)
{
	memset(pMsc->Response, 0, sizeof(pMsc->Response));
	pMsc->Response[0] = 0x70U;
	pMsc->Response[2] = pMsc->SenseKey;
	pMsc->Response[7] = 10U;
	pMsc->Response[12] = pMsc->SenseAsc;
	pMsc->Response[13] = pMsc->SenseAscq;
}

static bool UsbdMscRangeValid(const UsbdMscDev_t *pMsc, uint32_t Lba,
								uint32_t Blocks)
{
	return Blocks == 0U ||
		(Lba < pMsc->SectorCount && Blocks <= pMsc->SectorCount - Lba);
}

static UsbdMscDataDirection_t UsbdMscPrepareScsi(UsbdMscDev_t *pMsc)
{
	const uint8_t *cdb = pMsc->Cbw.CBWCB;
	const uint8_t cdbLength = pMsc->Cbw.bCBWCBLength;
	uint32_t blocks;

	switch (cdb[0])
	{
		case USB_MSC_SCSI_INQUIRY:
			if (cdbLength < 6U || (cdb[1] & 3U) != 0U || cdb[2] != 0U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			UsbdMscPrepareInquiry(pMsc);
			pMsc->ResponseLength = cdb[4] < sizeof(pMsc->Response) ?
				cdb[4] : sizeof(pMsc->Response);
			pMsc->DeviceLength = pMsc->ResponseLength;
			return USBD_MSC_DATA_IN;

		case USB_MSC_SCSI_TEST_UNIT_READY:
			if (cdbLength < 6U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
			}
			else
			{
				(void)UsbdMscMediumReady(pMsc);
			}
			return USBD_MSC_DATA_NONE;

		case USB_MSC_SCSI_REQUEST_SENSE:
			if (cdbLength < 6U || (cdb[1] & 1U) != 0U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			UsbdMscPrepareSense(pMsc);
			pMsc->ResponseLength = cdb[4] < 18U ? cdb[4] : 18U;
			pMsc->DeviceLength = pMsc->ResponseLength;
			UsbdMscSetSense(pMsc, USB_MSC_SENSE_NONE, 0U);
			return USBD_MSC_DATA_IN;

		case USB_MSC_SCSI_READ_CAPACITY_10:
			if (cdbLength < 10U || UsbdMscGetBe32(&cdb[2]) != 0U ||
				(cdb[8] & 1U) != 0U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			pMsc->DeviceLength = 8U;
			if (!UsbdMscMediumReady(pMsc))
			{
				return USBD_MSC_DATA_IN;
			}
			memset(pMsc->Response, 0, sizeof(pMsc->Response));
			UsbdMscPutBe32(&pMsc->Response[0], pMsc->SectorCount - 1U);
			UsbdMscPutBe32(&pMsc->Response[4], pMsc->SectorSize);
			pMsc->ResponseLength = 8U;
			return USBD_MSC_DATA_IN;

		case USB_MSC_SCSI_MODE_SENSE_6:
			if (cdbLength < 6U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			memset(pMsc->Response, 0, sizeof(pMsc->Response));
			pMsc->Response[0] = 3U;
			pMsc->Response[2] = pMsc->bReadOnly ? 0x80U : 0x00U;
			pMsc->ResponseLength = cdb[4] < 4U ? cdb[4] : 4U;
			pMsc->DeviceLength = pMsc->ResponseLength;
			return USBD_MSC_DATA_IN;

		case USB_MSC_SCSI_READ_10:
		case USB_MSC_SCSI_WRITE_10:
			if (cdbLength < 10U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			pMsc->Lba = UsbdMscGetBe32(&cdb[2]);
			blocks = UsbdMscGetBe16(&cdb[7]);
			pMsc->BlocksRemaining = blocks;
			pMsc->DeviceLength = blocks * (uint32_t)pMsc->SectorSize;
			if (!UsbdMscMediumReady(pMsc))
			{
				return cdb[0] == USB_MSC_SCSI_READ_10 ?
					USBD_MSC_DATA_IN : USBD_MSC_DATA_OUT;
			}
			if (!UsbdMscRangeValid(pMsc, pMsc->Lba, blocks))
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_LBA_OUT_OF_RANGE);
				return cdb[0] == USB_MSC_SCSI_READ_10 ?
					USBD_MSC_DATA_IN : USBD_MSC_DATA_OUT;
			}
			if (cdb[0] == USB_MSC_SCSI_WRITE_10 && pMsc->bReadOnly)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_DATA_PROTECT,
					USB_MSC_ASC_WRITE_PROTECTED);
			}
			return cdb[0] == USB_MSC_SCSI_READ_10 ?
				USBD_MSC_DATA_IN : USBD_MSC_DATA_OUT;

		case USB_MSC_SCSI_PREVENT_ALLOW:
			if (cdbLength < 6U || (cdb[4] & 0xFCU) != 0U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
			}
			else
			{
				pMsc->bRemovalPrevented = (cdb[4] & 1U) != 0U;
			}
			return USBD_MSC_DATA_NONE;

		case USB_MSC_SCSI_START_STOP_UNIT:
			if (cdbLength < 6U || (cdb[4] & 0xFCU) != 0U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
			}
			else if ((cdb[4] & 2U) != 0U && !pMsc->bRemovable)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
			}
			else if ((cdb[4] & 3U) == 2U && pMsc->bRemovalPrevented)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_MEDIUM_REMOVAL_PREVENTED,
					USB_MSC_ASCQ_REMOVAL_PREVENTED);
			}
			else if ((cdb[4] & 1U) != 0U)
			{
				pMsc->pDisk->Reset();
				pMsc->bMediumPresent = true;
				pMsc->bMediumReady = true;
			}
			else
			{
				pMsc->pDisk->Flush();
				pMsc->bMediumReady = false;
				if ((cdb[4] & 2U) != 0U)
				{
					pMsc->bMediumPresent = false;
				}
			}
			return USBD_MSC_DATA_NONE;

		case USB_MSC_SCSI_VERIFY_10:
			if (cdbLength < 10U || (cdb[1] & 2U) != 0U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			pMsc->Lba = UsbdMscGetBe32(&cdb[2]);
			blocks = UsbdMscGetBe16(&cdb[7]);
			if (!UsbdMscMediumReady(pMsc))
			{
				return USBD_MSC_DATA_NONE;
			}
			if (!UsbdMscRangeValid(pMsc, pMsc->Lba, blocks))
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_LBA_OUT_OF_RANGE);
			}
			return USBD_MSC_DATA_NONE;

		case USB_MSC_SCSI_SYNCHRONIZE_CACHE:
			if (cdbLength < 10U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
			}
			else if (UsbdMscMediumReady(pMsc))
			{
				pMsc->pDisk->Flush();
			}
			return USBD_MSC_DATA_NONE;

		default:
			UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
				USB_MSC_ASC_INVALID_COMMAND);
			return USBD_MSC_DATA_NONE;
	}
}

static void UsbdMscStartCommand(UsbdMscDev_t *pMsc)
{
	pMsc->HostLength = pMsc->Cbw.dCBWDataTransferLength;
	pMsc->Csw.dCSWTag = pMsc->Cbw.dCBWTag;
	const UsbdMscDataDirection_t deviceDirection = UsbdMscPrepareScsi(pMsc);
	const UsbdMscDataDirection_t hostDirection = pMsc->HostLength == 0U ?
		USBD_MSC_DATA_NONE :
		((pMsc->Cbw.bmCBWFlags & USB_MSC_CBW_FLAG_IN) != 0U ?
		 USBD_MSC_DATA_IN : USBD_MSC_DATA_OUT);

	if (pMsc->HostLength == 0U)
	{
		if (pMsc->DeviceLength != 0U)
		{
			pMsc->bPhaseError = true;
		}
		UsbdMscFinishCommand(pMsc);
		return;
	}

	if (pMsc->DeviceLength == 0U)
	{
		UsbdMscStall(pMsc, hostDirection);
		UsbdMscFinishCommand(pMsc);
		return;
	}

	if (hostDirection != deviceDirection)
	{
		pMsc->bPhaseError = true;
		UsbdMscStall(pMsc, hostDirection);
		UsbdMscFinishCommand(pMsc);
		return;
	}

	if (pMsc->bCommandFailed)
	{
		UsbdMscStall(pMsc, hostDirection);
		UsbdMscFinishCommand(pMsc);
		return;
	}

	pMsc->TransferLimit = pMsc->HostLength < pMsc->DeviceLength ?
		pMsc->HostLength : pMsc->DeviceLength;
	if (pMsc->HostLength < pMsc->DeviceLength)
	{
		pMsc->bPhaseError = true;
		pMsc->bStallAfterData = true;
	}
	else if (deviceDirection == USBD_MSC_DATA_OUT &&
		pMsc->HostLength > pMsc->DeviceLength)
	{
		pMsc->bStallAfterData = true;
	}
	else if (deviceDirection == USBD_MSC_DATA_IN &&
		pMsc->HostLength > pMsc->DeviceLength &&
		(pMsc->DeviceLength % pMsc->IntrfData.Mps) == 0U)
	{
		pMsc->bNeedZlp = true;
	}

	pMsc->State = deviceDirection == USBD_MSC_DATA_IN ?
		USBD_MSC_BOT_DATA_IN : USBD_MSC_BOT_DATA_OUT;
}

static bool UsbdMscValidCbw(const UsbMscCmdBlkWrapper_t *pCbw,
							 uint16_t Length)
{
	return Length == sizeof(*pCbw) &&
		pCbw->dCBWSignature == USB_MSC_CBW_SIGNATURE &&
		pCbw->bCBWLUN == 0U && pCbw->bCBWCBLength >= 1U &&
		pCbw->bCBWCBLength <= sizeof(pCbw->CBWCB) &&
		(pCbw->bmCBWFlags & USB_MSC_CBW_FLAG_RESERVED) == 0U;
}

static void UsbdMscInvalidCbw(UsbdMscDev_t *pMsc)
{
	(void)UsbEpSetHalt(pMsc->DevNo, USB_ENDPADDR_DIROUT(pMsc->EpNo), true);
	(void)UsbEpSetHalt(pMsc->DevNo, USB_ENDPADDR_DIRIN(pMsc->EpNo), true);
	pMsc->bResetSeen = false;
	pMsc->State = USBD_MSC_BOT_RESET_RECOVERY;
}

static void UsbdMscProcessCbw(UsbdMscDev_t *pMsc)
{
	uint8_t *pPacket = UsbdMscRxPacket(pMsc);
	const int length = DeviceIntrfRxData(&pMsc->IntrfData.DevIntrf,
		pPacket, pMsc->IntrfData.Mps);
	if (length <= 0)
	{
		return;
	}

	if (length == (int)sizeof(pMsc->Cbw))
	{
		memcpy(&pMsc->Cbw, pPacket, sizeof(pMsc->Cbw));
	}
	if (!UsbdMscValidCbw(&pMsc->Cbw, (uint16_t)length))
	{
		UsbdMscInvalidCbw(pMsc);
		return;
	}

	pMsc->bCommandFailed = false;
	pMsc->bPhaseError = false;
	pMsc->bStallAfterData = false;
	pMsc->bNeedZlp = false;
	pMsc->Transferred = 0U;
	pMsc->ResponseLength = 0U;
	pMsc->ResponseOffset = 0U;
	pMsc->SectorOffset = 0U;
	UsbdMscStartCommand(pMsc);
}

static void UsbdMscProcessDataIn(UsbdMscDev_t *pMsc)
{
	UsbdMscCompletePendingTx(pMsc);
	if (pMsc->bTxFailed)
	{
		pMsc->bTxFailed = false;
		pMsc->PendingTx = USBD_MSC_TX_NONE;
		pMsc->PendingTxLength = 0U;
		(void)UsbIntrfConfigure(&pMsc->IntrfData, UsbdMscMps(pMsc));
		UsbdMscFail(pMsc, USB_MSC_SENSE_NOT_READY,
			USB_MSC_ASC_MEDIUM_NOT_PRESENT);
		pMsc->bStallAfterData = true;
		UsbdMscFinishCommand(pMsc);
		return;
	}
	if (pMsc->PendingTx != USBD_MSC_TX_NONE)
	{
		return;
	}
	if (pMsc->Transferred >= pMsc->TransferLimit)
	{
		if (pMsc->bNeedZlp)
		{
			if (UsbdMscQueuePacket(pMsc, nullptr, 0U, USBD_MSC_TX_ZLP))
			{
				pMsc->bNeedZlp = false;
			}
			return;
		}
		UsbdMscFinishCommand(pMsc);
		return;
	}

	const uint32_t remaining = pMsc->TransferLimit - pMsc->Transferred;
	uint16_t length = remaining < pMsc->IntrfData.Mps ?
		(uint16_t)remaining : pMsc->IntrfData.Mps;
	const uint8_t *pData;
	if (pMsc->ResponseLength != 0U)
	{
		const uint16_t responseRemaining =
			(uint16_t)(pMsc->ResponseLength - pMsc->ResponseOffset);
		if (length > responseRemaining)
		{
			length = responseRemaining;
		}
		pData = &pMsc->Response[pMsc->ResponseOffset];
	}
	else
	{
		if (pMsc->SectorOffset == 0U &&
			!pMsc->pDisk->SectRead(pMsc->Lba, pMsc->pSectorBuffer))
		{
			UsbdMscFail(pMsc, USB_MSC_SENSE_MEDIUM_ERROR,
				USB_MSC_ASC_UNRECOVERED_READ);
			pMsc->bStallAfterData = true;
			UsbdMscFinishCommand(pMsc);
			return;
		}
		const uint16_t sectorRemaining =
			(uint16_t)(pMsc->SectorSize - pMsc->SectorOffset);
		if (length > sectorRemaining)
		{
			length = sectorRemaining;
		}
		pData = &pMsc->pSectorBuffer[pMsc->SectorOffset];
	}
	(void)UsbdMscQueuePacket(pMsc, pData, length, USBD_MSC_TX_DATA);
}

static void UsbdMscProcessDataOut(UsbdMscDev_t *pMsc)
{
	uint8_t *pPacket = UsbdMscRxPacket(pMsc);
	const int received = DeviceIntrfRxData(&pMsc->IntrfData.DevIntrf,
		pPacket, pMsc->IntrfData.Mps);
	if (received <= 0)
	{
		return;
	}

	uint32_t use = (uint32_t)received;
	const uint32_t remaining = pMsc->TransferLimit - pMsc->Transferred;
	if (use > remaining)
	{
		use = remaining;
	}

	uint32_t offset = 0U;
	while (offset < use)
	{
		uint32_t length = pMsc->SectorSize - pMsc->SectorOffset;
		if (length > use - offset)
		{
			length = use - offset;
		}
		memcpy(&pMsc->pSectorBuffer[pMsc->SectorOffset],
			&pPacket[offset], length);
		pMsc->SectorOffset = (uint16_t)(pMsc->SectorOffset + length);
		pMsc->Transferred += length;
		offset += length;

		if (pMsc->SectorOffset == pMsc->SectorSize)
		{
			if (!pMsc->pDisk->SectWrite(pMsc->Lba, pMsc->pSectorBuffer))
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_MEDIUM_ERROR,
					USB_MSC_ASC_WRITE_ERROR);
				pMsc->bStallAfterData = true;
				UsbdMscFinishCommand(pMsc);
				return;
			}
			pMsc->SectorOffset = 0U;
			pMsc->Lba++;
			pMsc->BlocksRemaining--;
		}
	}

	if (pMsc->Transferred >= pMsc->TransferLimit)
	{
		UsbdMscFinishCommand(pMsc);
	}
}

static void UsbdMscProcessCsw(UsbdMscDev_t *pMsc)
{
	if (pMsc->PendingTx != USBD_MSC_TX_NONE)
	{
		UsbdMscCompletePendingTx(pMsc);
		return;
	}
	if (
		UsbEpHalted(pMsc->DevNo, USB_ENDPADDR_DIRIN(pMsc->EpNo)))
	{
		return;
	}

	if (UsbdMscQueuePacket(pMsc,
		reinterpret_cast<const uint8_t *>(&pMsc->Csw), sizeof(pMsc->Csw),
		USBD_MSC_TX_CSW))
	{
		pMsc->State = USBD_MSC_BOT_SEND_CSW;
	}
}

static void UsbdMscProcessInternal(UsbdMscDev_t *pMsc)
{
	if (pMsc == nullptr || !pMsc->bConfigured)
	{
		return;
	}

	switch (pMsc->State)
	{
		case USBD_MSC_BOT_WAIT_CBW:
			UsbdMscProcessCbw(pMsc);
			break;

		case USBD_MSC_BOT_DATA_IN:
			UsbdMscProcessDataIn(pMsc);
			break;

		case USBD_MSC_BOT_DATA_OUT:
			UsbdMscProcessDataOut(pMsc);
			break;

		case USBD_MSC_BOT_SEND_CSW:
		case USBD_MSC_BOT_FAILED:
		case USBD_MSC_BOT_PHASE_ERROR:
			UsbdMscProcessCsw(pMsc);
			break;

		case USBD_MSC_BOT_RESET_RECOVERY:
			if (pMsc->bResetSeen &&
				!UsbEpHalted(pMsc->DevNo, USB_ENDPADDR_DIRIN(pMsc->EpNo)) &&
				!UsbEpHalted(pMsc->DevNo, USB_ENDPADDR_DIROUT(pMsc->EpNo)))
			{
				pMsc->bResetSeen = false;
				pMsc->State = USBD_MSC_BOT_WAIT_CBW;
			}
			break;
	}
}

static bool UsbdMscInitInternal(UsbdMscDev_t *pMsc,
								const UsbdMscCfg_t *pCfg,
								UsbDeviceClass *pClass)
{
	if (pMsc == nullptr || pCfg == nullptr || pClass == nullptr ||
		UsbGetCfg(pCfg->DevNo) == nullptr || pCfg->pDisk == nullptr ||
		pCfg->pSectorBuffer == nullptr || pCfg->SectorBufferSize == 0U)
	{
		return false;
	}

	const uint16_t sectorSize = pCfg->pDisk->GetSectSize();
	if (sectorSize == 0U)
	{
		return false;
	}
	const uint32_t sectorCount = pCfg->pDisk->GetNbSect();
	if (sectorCount == 0U ||
		sectorSize > pCfg->SectorBufferSize)
	{
		return false;
	}

	pMsc->DevNo = pCfg->DevNo;
	pMsc->pDisk = pCfg->pDisk;
	pMsc->pSectorBuffer = pCfg->pSectorBuffer;
	pMsc->SectorBufferSize = pCfg->SectorBufferSize;
	pMsc->SectorSize = sectorSize;
	pMsc->SectorCount = sectorCount;
	pMsc->bReadOnly = pCfg->bReadOnly;
	pMsc->bRemovable = pCfg->bRemovable;
	pMsc->bMediumPresent = true;
	pMsc->bMediumReady = true;
	pMsc->bRemovalPrevented = false;
	pMsc->InterfaceString = pCfg->InterfaceString;
	pMsc->FsMps = pCfg->FsMps != 0U ? pCfg->FsMps : USBD_MSC_FS_MPS;
	pMsc->HsMps = pCfg->HsMps != 0U ? pCfg->HsMps : USBD_MSC_HS_MPS;
	if (pMsc->FsMps > USBD_MSC_MAX_MPS ||
		(USB_HIGHSPEED_CAPABLE(pMsc->DevNo) &&
		 pMsc->HsMps > USBD_MSC_MAX_MPS))
	{
		return false;
	}

	UsbdMscCopyInquiry(pMsc->Vendor, sizeof(pMsc->Vendor),
		pCfg->pVendor != nullptr ? pCfg->pVendor : "I-SYST");
	UsbdMscCopyInquiry(pMsc->Product, sizeof(pMsc->Product),
		pCfg->pProduct != nullptr ? pCfg->pProduct : "IOsonata MSC");
	UsbdMscCopyInquiry(pMsc->Revision, sizeof(pMsc->Revision),
		pCfg->pRevision != nullptr ? pCfg->pRevision : "1.00");

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;
	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(pMsc->DevNo, &req, pClass, &alloc))
	{
		return false;
	}
	pMsc->ItfNo = alloc.FirstInterface;
	pMsc->EpNo = alloc.Bidirectional[0];

	UsbIntrfCfg_t intrfCfg = {};
	intrfCfg.DevNo = pMsc->DevNo;
	intrfCfg.EpNo = pMsc->EpNo;
	intrfCfg.bBlocking = true;
	intrfCfg.Mode = USB_INTRF_MODE_PACKET;
	intrfCfg.RxFifoMemSize = sizeof(pMsc->RxFifo);
	intrfCfg.pRxFifoMem = reinterpret_cast<uint8_t *>(pMsc->RxFifo);
	intrfCfg.TxFifoMemSize = sizeof(pMsc->TxFifo);
	intrfCfg.pTxFifoMem = reinterpret_cast<uint8_t *>(pMsc->TxFifo);
	intrfCfg.TxFifoBlkSize = USBD_MSC_PKT_BLKSIZE;
	intrfCfg.BufferSize = USBD_MSC_MAX_MPS;
	intrfCfg.pRxBuffer = UsbdMscRxBuffer(pMsc);
	intrfCfg.pTxBuffer = UsbdMscTxBuffer(pMsc);
	intrfCfg.EvtCB = UsbdMscDataEvent;
	if (!UsbIntrfInit(&pMsc->IntrfData, &intrfCfg))
	{
		return false;
	}
	pMsc->IntrfData.pClassContext = pMsc;

	if (!UsbdMscFillDesc(&pMsc->FsDesc, pMsc, USB_SPEED_FULL))
	{
		return false;
	}
	const void *pHsDesc = nullptr;
	uint16_t hsDescLength = 0U;
	if (USB_HIGHSPEED_CAPABLE(pMsc->DevNo))
	{
		if (!UsbdMscFillDesc(&pMsc->HsDesc, pMsc, USB_SPEED_HIGH))
		{
			return false;
		}
		pHsDesc = &pMsc->HsDesc;
		hsDescLength = sizeof(pMsc->HsDesc);
	}

	pMsc->MaxLun = 0U;
	pMsc->bConfigured = false;
	UsbdMscResetBot(pMsc, false);
	return UsbDescriptorRegister(pMsc->DevNo, pClass, &pMsc->FsDesc,
		sizeof(pMsc->FsDesc), pHsDesc, hsDescLength);
}

bool UsbdMsc::Init(const UsbdMscCfg_t &Cfg)
{
	return UsbdMscInitInternal(&vUsbdMsc, &Cfg, this);
}

bool UsbdMsc::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					  uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup == nullptr || Stage == USB_CTRL_ABORT ||
		(uint8_t)pSetup->wIndex != (uint8_t)vUsbdMsc.ItfNo ||
		(pSetup->wIndex & 0xFF00U) != 0U)
	{
		return false;
	}

	if (pSetup->bRequest == USB_MSC_REQCODE_MAXLUN &&
		pSetup->bmRequestType == (USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS |
			USB_REQTYPE_INTERFACE) && pSetup->wValue == 0U &&
		pSetup->wLength == 1U)
	{
		if (Stage == USB_CTRL_SETUP)
		{
			if (ppData == nullptr || pLength == nullptr)
			{
				return false;
			}
			*ppData = &vUsbdMsc.MaxLun;
			*pLength = 1U;
		}
		return true;
	}

	if (pSetup->bRequest == USB_MSC_REQCODE_BOMSR &&
		pSetup->bmRequestType == (USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
			USB_REQTYPE_INTERFACE) && pSetup->wValue == 0U &&
		pSetup->wLength == 0U)
	{
		if (Stage == USB_CTRL_SETUP)
		{
			const bool recovery =
				vUsbdMsc.State == USBD_MSC_BOT_RESET_RECOVERY ||
				vUsbdMsc.State == USBD_MSC_BOT_PHASE_ERROR;
			UsbdMscResetBot(&vUsbdMsc, false);
			if (vUsbdMsc.bConfigured && !UsbdMscRestartEndpoints(&vUsbdMsc))
			{
				return false;
			}
			vUsbdMsc.bResetSeen = recovery;
			vUsbdMsc.State = recovery ? USBD_MSC_BOT_RESET_RECOVERY :
				USBD_MSC_BOT_WAIT_CBW;
		}
		return true;
	}

	return false;
}

bool UsbdMsc::SelectConfig(uint8_t ConfigValue)
{
	if (vUsbdMsc.bConfigured)
	{
		UsbdMscCloseEndpoints(&vUsbdMsc);
		vUsbdMsc.bConfigured = false;
	}
	UsbIntrfUnconfigure(&vUsbdMsc.IntrfData);
	if (ConfigValue == 0U)
	{
		UsbdMscResetBot(&vUsbdMsc, false);
		return true;
	}
	if (ConfigValue != USBD_MSC_CONFIG_VALUE)
	{
		return false;
	}

	const uint16_t mps = UsbdMscMps(&vUsbdMsc);
	if (!UsbIntrfConfigure(&vUsbdMsc.IntrfData, mps) ||
		!UsbdMscOpenEndpoint(&vUsbdMsc,
			USB_ENDPADDR_DIRIN(vUsbdMsc.EpNo), mps) ||
		!UsbdMscOpenEndpoint(&vUsbdMsc,
			USB_ENDPADDR_DIROUT(vUsbdMsc.EpNo), mps))
	{
		UsbdMscCloseEndpoints(&vUsbdMsc);
		UsbIntrfUnconfigure(&vUsbdMsc.IntrfData);
		return false;
	}
	vUsbdMsc.bConfigured = true;
	UsbdMscResetBot(&vUsbdMsc, false);
	return true;
}

void UsbdMsc::Detach(void)
{
	if (vUsbdMsc.pDisk != nullptr)
	{
		vUsbdMsc.pDisk->Flush();
		vUsbdMsc.pDisk->Reset();
	}
	vUsbdMsc.bMediumPresent = true;
	vUsbdMsc.bMediumReady = true;
	vUsbdMsc.bRemovalPrevented = false;
}

void UsbdMsc::Reset(void)
{
	if (vUsbdMsc.bConfigured)
	{
		UsbdMscCloseEndpoints(&vUsbdMsc);
		vUsbdMsc.bConfigured = false;
	}
	UsbdMscResetBot(&vUsbdMsc, true);
}

void UsbdMsc::Process(void)
{
	UsbdMscProcessInternal(&vUsbdMsc);
}
