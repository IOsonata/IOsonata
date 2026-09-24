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

// Inlined: the compiler turns this into one load and a byte reverse, which
// is smaller than a call.
static inline __attribute__((always_inline))
uint32_t UsbdMscGetBe32(const uint8_t *pData)
{
	return ((uint32_t)pData[0] << 24) | ((uint32_t)pData[1] << 16) |
		((uint32_t)pData[2] << 8) | pData[3];
}

// Kept out of line: the byte stores are larger than a call at each use.
__attribute__((noinline))
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

static uint8_t *UsbdMscRxPacket(UsbdMscDev_t *pMsc)
{
	return reinterpret_cast<uint8_t *>(pMsc->RxPacket);
}

static UsbPkt_t *UsbdMscTxPacket(UsbdMscDev_t *pMsc)
{
	return reinterpret_cast<UsbPkt_t *>(pMsc->TxPacket);
}

static void UsbdMscStall(UsbdMscDev_t *pMsc, UsbdMscDataDirection_t Direction)
{
	(void)UsbEpSetHalt(pMsc->DevNo, pMsc->EpNo,
		Direction == USBD_MSC_DATA_IN, true);
}

// Kept out of line: the call sites are smaller than the inlined argument setup.
__attribute__((noinline))
static bool UsbdMscHalted(const UsbdMscDev_t *pMsc, bool bIn)
{
	return UsbEpHalted(pMsc->DevNo, pMsc->EpNo, bIn);
}

static void UsbdMscSetSense(UsbdMscDev_t *pMsc, uint8_t Key,
							 uint8_t Asc, uint8_t Ascq = 0U)
{
	pMsc->SenseKey = Key;
	pMsc->SenseAsc = Asc;
	pMsc->SenseAscq = Ascq;
}

// Kept out of line: one copy of the failed flag and sense stores for all
// SCSI failure paths.
__attribute__((noinline))
static void UsbdMscFail(UsbdMscDev_t *pMsc, uint8_t Key, uint8_t Asc,
							 uint8_t Ascq = 0U)
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
							const char *pSource, const char *pDefault)
{
	memset(pDest, ' ', Length);
	if (pSource == nullptr)
	{
		pSource = pDefault;
		if (pSource == nullptr)
		{
			return;
		}
	}

	size_t length = 0U;
	while (length < Length && pSource[length] != '\0')
	{
		length++;
	}
	memcpy(pDest, pSource, length);
}

static bool UsbdMscOpenEndpoint(UsbdMscDev_t *pMsc, bool bIn,
								uint16_t MaxPacketSize)
{
	return UsbCtrlrEpOpenData(pMsc->DevNo, pMsc->EpNo, bIn,
		USB_ENDPATT_TRANS_BULK, MaxPacketSize);
}

static void UsbdMscCloseEndpoints(UsbdMscDev_t *pMsc)
{
	UsbCtrlrEpClose(pMsc->DevNo, pMsc->EpNo, false);
	UsbCtrlrEpClose(pMsc->DevNo, pMsc->EpNo, true);
}

// Close the bulk endpoints when they are open.
static void UsbdMscDeconfigure(UsbdMscDev_t *pMsc)
{
	if (pMsc->bConfigured)
	{
		UsbdMscCloseEndpoints(pMsc);
		pMsc->bConfigured = false;
	}
}

// Configure the data path and open both bulk endpoints at the current bus
// speed. On failure everything opened so far is closed again.
static bool UsbdMscOpenEndpoints(UsbdMscDev_t *pMsc)
{
	const uint16_t mps = UsbdMscMps(pMsc);

	if (!UsbIntrfConfigure(pMsc->pData, mps) ||
		!UsbdMscOpenEndpoint(pMsc, true, mps) ||
		!UsbdMscOpenEndpoint(pMsc, false, mps))
	{
		UsbdMscCloseEndpoints(pMsc);
		UsbIntrfUnconfigure(pMsc->pData);
		return false;
	}
	return true;
}

static bool UsbdMscRestartEndpoints(UsbdMscDev_t *pMsc)
{
	const bool haltIn = UsbdMscHalted(pMsc, true);
	const bool haltOut = UsbdMscHalted(pMsc, false);

	UsbdMscCloseEndpoints(pMsc);
	if (!UsbdMscOpenEndpoints(pMsc))
	{
		pMsc->bConfigured = false;
		return false;
	}
	if (haltIn)
	{
		UsbdMscStall(pMsc, USBD_MSC_DATA_IN);
	}
	if (haltOut)
	{
		UsbdMscStall(pMsc, USBD_MSC_DATA_OUT);
	}
	return true;
}

static constexpr UsbdMscDesc_t UsbdMscDescTemplate(void)
{
	UsbdMscDesc_t desc = {};
	desc.Interface.bLength = sizeof(desc.Interface);
	desc.Interface.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Interface.bNumEndpoints = 2U;
	desc.Interface.bInterfaceClass = USB_INTRFCLASS_MSC;
	desc.Interface.bInterfaceSubClass = USB_MSC_SUBCLASS_SCSI;
	desc.Interface.bInterfaceProtocol = USB_MSC_PROT_BULK;
	desc.Out.bLength = sizeof(desc.Out);
	desc.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
	desc.Out.bmAttributes = USB_ENDPATT_TRANS_BULK;
	desc.In = desc.Out;
	return desc;
}

static constexpr UsbdMscDesc_t s_MscDescTemplate = UsbdMscDescTemplate();

// Kept out of line: shared by the registered patch callback and the weak
// UsbdMscMakeDesc.
__attribute__((noinline))
static void UsbdMscPatchDesc(UsbdMscDesc_t *pDesc,
							 const UsbdMscDev_t *pMsc, UsbSpeed_t Speed)
{
	const uint16_t mps = Speed == USB_SPEED_HIGH ? pMsc->HsMps : pMsc->FsMps;
	pDesc->Interface.bInterfaceNumber = (uint8_t)pMsc->ItfNo;
	pDesc->Interface.iInterface = pMsc->InterfaceString;
	pDesc->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(pMsc->EpNo);
	pDesc->Out.wMaxPacketSize = mps;
	pDesc->In.bEndpointAddress = USB_ENDPADDR_DIRIN(pMsc->EpNo);
	pDesc->In.wMaxPacketSize = mps;
}

static void UsbdMscPatchRegistered(const UsbDeviceClass *pClass,
									 uint8_t *pDesc, UsbSpeed_t Speed)
{
	const UsbdMscDev_t *pMsc = *static_cast<const UsbdMsc *>(pClass);
	UsbdMscPatchDesc(reinterpret_cast<UsbdMscDesc_t *>(pDesc), pMsc, Speed);
}

// Weak so an application can replace runtime fragment building with a static
// fragment. When overridden, this default is dropped by unused-section removal.
// Pair a replacement with a strong UsbGetDescriptor for fully static
// descriptors, or the assembled configuration will not match. This class is
// C++ only, so an override matches the C++ symbol.
__attribute__((weak))
bool UsbdMscMakeDesc(UsbdMscDesc_t *pDesc,
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

	memcpy(pDesc, &s_MscDescTemplate, sizeof(*pDesc));
	UsbdMscPatchDesc(pDesc, pMsc, Speed);
	return true;
}

static bool UsbdMscTxIdle(const UsbdMscDev_t *pMsc)
{
	return atomic_load_explicit(&pMsc->pData->DevIntrf.bTxReady,
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
	// HostLength, DeviceLength, TransferLimit, Transferred, Lba,
	// BlocksRemaining, SectorOffset, ResponseLength, ResponseOffset, Cbw and
	// Csw are adjacent: one clear covers the whole transfer state.
	static_assert(offsetof(UsbdMscDev_t, Csw) + sizeof(pMsc->Csw) -
		offsetof(UsbdMscDev_t, HostLength) ==
		6U * sizeof(uint32_t) + 3U * sizeof(uint16_t) +
		sizeof(pMsc->Cbw) + sizeof(pMsc->Csw),
		"MSC transfer state must be contiguous");
	memset(reinterpret_cast<uint8_t *>(pMsc) +
		offsetof(UsbdMscDev_t, HostLength), 0,
		offsetof(UsbdMscDev_t, Csw) + sizeof(pMsc->Csw) -
		offsetof(UsbdMscDev_t, HostLength));
}

static void UsbdMscResetBot(UsbdMscDev_t *pMsc, bool ResetMedium)
{
	UsbdMscClearCommand(pMsc);
	pMsc->bResetSeen = false;
	pMsc->State = USBD_MSC_BOT_WAIT_CBW;
	UsbdMscSetSense(pMsc, USB_MSC_SENSE_NONE, 0U);
	if (pMsc->bConfigured)
	{
		(void)UsbIntrfConfigure(pMsc->pData, UsbdMscMps(pMsc));
	}
	else
	{
		UsbIntrfUnconfigure(pMsc->pData);
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

static void UsbdMscFinishCommand(UsbdMscDev_t *pMsc)
{
	// Copied as a word: a constant assigned to the packed field is expanded
	// into byte stores.
	const uint32_t signature = USB_MSC_CSW_SIGNATURE;

	memcpy(&pMsc->Csw.dCSWSignature, &signature, sizeof(signature));
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
		Length > pMsc->pData->Mps)
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
	if (DeviceIntrfTxData(&pMsc->pData->DevIntrf,
		reinterpret_cast<uint8_t *>(pPacket), USBD_MSC_PKT_BLKSIZE) !=
		(int)USBD_MSC_PKT_BLKSIZE)
	{
		return false;
	}
	pMsc->PendingTx = Kind;
	pMsc->PendingTxLength = Length;
	return true;
}

// One sector fully transferred: step to the next block. Kept out of line
// for the data-in and data-out paths.
__attribute__((noinline))
static void UsbdMscNextSector(UsbdMscDev_t *pMsc)
{
	pMsc->SectorOffset = 0U;
	pMsc->Lba++;
	pMsc->BlocksRemaining--;
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
				UsbdMscNextSector(pMsc);
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

// Start a data-in response of Length bytes built in pMsc->Response.
__attribute__((noinline))
static void UsbdMscStartResponse(UsbdMscDev_t *pMsc, uint8_t Length)
{
	memset(pMsc->Response, 0, sizeof(pMsc->Response));
	pMsc->ResponseLength = Length;
	pMsc->DeviceLength = Length;
}

// Allocation length from CDB byte 4 capped at the size of the response.
static uint8_t UsbdMscAllocLength(const UsbdMscDev_t *pMsc, uint8_t MaxLength)
{
	const uint8_t alloc = pMsc->Cbw.CBWCB[4];

	return alloc < MaxLength ? alloc : MaxLength;
}

static void UsbdMscPrepareInquiry(UsbdMscDev_t *pMsc)
{
	UsbdMscStartResponse(pMsc,
		UsbdMscAllocLength(pMsc, sizeof(pMsc->Response)));
	// Response[0] stays 0x00 from the clear: direct access block device.
	pMsc->Response[1] = pMsc->bRemovable ? 0x80U : 0x00U;
	pMsc->Response[2] = 0x06U;
	pMsc->Response[3] = 0x02U;
	pMsc->Response[4] = 31U;
	// Vendor, product and revision are adjacent in both the device state and
	// the INQUIRY response, so one copy covers all three.
	static_assert(offsetof(UsbdMscDev_t, Product) ==
		offsetof(UsbdMscDev_t, Vendor) + sizeof(pMsc->Vendor) &&
		offsetof(UsbdMscDev_t, Revision) ==
		offsetof(UsbdMscDev_t, Product) + sizeof(pMsc->Product),
		"INQUIRY identity strings must be contiguous");
	memcpy(&pMsc->Response[8], reinterpret_cast<const uint8_t *>(pMsc) +
		offsetof(UsbdMscDev_t, Vendor),
		sizeof(pMsc->Vendor) + sizeof(pMsc->Product) + sizeof(pMsc->Revision));
}

static void UsbdMscPrepareSense(UsbdMscDev_t *pMsc)
{
	UsbdMscStartResponse(pMsc, UsbdMscAllocLength(pMsc, 18U));
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

// Medium check followed by the LBA range check for the block commands. Both
// failures set the command failed with their own sense code.
static bool UsbdMscReadyRange(UsbdMscDev_t *pMsc, uint32_t Blocks)
{
	if (!UsbdMscMediumReady(pMsc))
	{
		return false;
	}
	if (!UsbdMscRangeValid(pMsc, pMsc->Lba, Blocks))
	{
		UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
			USB_MSC_ASC_LBA_OUT_OF_RANGE);
		return false;
	}
	return true;
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
			UsbdMscStartResponse(pMsc, 8U);
			UsbdMscPutBe32(&pMsc->Response[0], pMsc->SectorCount - 1U);
			UsbdMscPutBe32(&pMsc->Response[4], pMsc->SectorSize);
			return USBD_MSC_DATA_IN;

		case USB_MSC_SCSI_MODE_SENSE_6:
			if (cdbLength < 6U)
			{
				UsbdMscFail(pMsc, USB_MSC_SENSE_ILLEGAL_REQUEST,
					USB_MSC_ASC_INVALID_FIELD);
				return USBD_MSC_DATA_NONE;
			}
			UsbdMscStartResponse(pMsc, UsbdMscAllocLength(pMsc, 4U));
			pMsc->Response[0] = 3U;
			pMsc->Response[2] = pMsc->bReadOnly ? 0x80U : 0x00U;
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
			if (!UsbdMscReadyRange(pMsc, blocks))
			{
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
			(void)UsbdMscReadyRange(pMsc, blocks);
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
		(pMsc->DeviceLength % pMsc->pData->Mps) == 0U)
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
	UsbdMscStall(pMsc, USBD_MSC_DATA_OUT);
	UsbdMscStall(pMsc, USBD_MSC_DATA_IN);
	pMsc->bResetSeen = false;
	pMsc->State = USBD_MSC_BOT_RESET_RECOVERY;
}

static void UsbdMscProcessCbw(UsbdMscDev_t *pMsc)
{
	uint8_t *pPacket = UsbdMscRxPacket(pMsc);
	const int length = DeviceIntrfRxData(&pMsc->pData->DevIntrf,
		pPacket, pMsc->pData->Mps);
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
		(void)UsbIntrfConfigure(pMsc->pData, UsbdMscMps(pMsc));
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
	uint16_t length = remaining < pMsc->pData->Mps ?
		(uint16_t)remaining : pMsc->pData->Mps;
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
	const int received = DeviceIntrfRxData(&pMsc->pData->DevIntrf,
		pPacket, pMsc->pData->Mps);
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
			UsbdMscNextSector(pMsc);
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
	if (UsbdMscHalted(pMsc, true))
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
			if (pMsc->bResetSeen && !UsbdMscHalted(pMsc, true) &&
				!UsbdMscHalted(pMsc, false))
			{
				pMsc->bResetSeen = false;
				pMsc->State = USBD_MSC_BOT_WAIT_CBW;
			}
			break;
	}
}

static bool UsbdMscInitInternal(UsbdMscDev_t *pMsc,
								UsbDevIntrf_t *pData,
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

	pMsc->pData = pData;
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

	UsbdMscCopyInquiry(pMsc->Vendor, sizeof(pMsc->Vendor), pCfg->pVendor,
		"I-SYST");
	UsbdMscCopyInquiry(pMsc->Product, sizeof(pMsc->Product), pCfg->pProduct,
		"IOsonata MSC");
	UsbdMscCopyInquiry(pMsc->Revision, sizeof(pMsc->Revision),
		pCfg->pRevision, "1.00");

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
	intrfCfg.EvtCB = UsbdMscDataEvent;
	if (!UsbIntrfInit(pMsc->pData, &intrfCfg))
	{
		return false;
	}
	pMsc->pData->pClassContext = pMsc;

	pMsc->MaxLun = 0U;
	pMsc->bConfigured = false;
	UsbdMscResetBot(pMsc, false);
	return UsbDescRegister(pMsc->DevNo, pClass,
		&s_MscDescTemplate, sizeof(s_MscDescTemplate),
		UsbdMscPatchRegistered);
}

bool UsbdMsc::Init(const UsbdMscCfg_t &Cfg)
{
	return UsbdMscInitInternal(&vUsbdMsc, &vUsbDevIntrf, &Cfg, this);
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
	UsbdMscDeconfigure(&vUsbdMsc);
	UsbIntrfUnconfigure(vUsbdMsc.pData);
	if (ConfigValue == 0U)
	{
		UsbdMscResetBot(&vUsbdMsc, false);
		return true;
	}
	if (ConfigValue != USBD_MSC_CONFIG_VALUE)
	{
		return false;
	}

	if (!UsbdMscOpenEndpoints(&vUsbdMsc))
	{
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
	UsbdMscDeconfigure(&vUsbdMsc);
	UsbdMscResetBot(&vUsbdMsc, true);
}

void UsbdMsc::Process(void)
{
	UsbdMscProcessInternal(&vUsbdMsc);
}
