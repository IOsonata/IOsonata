/**-------------------------------------------------------------------------
@file	usbd_msc_test.cpp

@brief	Host tests for the USB Mass Storage BOT and SCSI device class.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "storage/diskio.h"
#include "usb/usbd_msc.h"

#define EP_NO		1U
#define ITF_NO		0U
#define SECTOR_SIZE	128U
#define SECTOR_COUNT	16U

static UsbCfg_t s_UsbCfg;
static UsbDeviceClass *s_ClassObject;
static const uint8_t *s_FsDescriptor;
static uint16_t s_FsDescriptorLength;
static UsbEndPointDesc_t s_OpenDesc[2];
static int s_OpenCount;
static int s_CloseCount;
static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutReady;
static bool s_OutDma;
static uint16_t s_OutLength;
static uint8_t s_OutData[USBD_MSC_MAX_MPS];
static bool s_InBusy;
static uint16_t s_InLength;
static bool s_HaltIn;
static bool s_HaltOut;
static uint8_t s_Capture[SECTOR_SIZE * 4U + 256U];
static size_t s_CaptureLength;
static int s_Fail;

#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

class RamDisk : public DiskIO {
public:
	uint16_t GetSectSize(void) override { return SECTOR_SIZE; }
	uint32_t GetNbSect(void) override { return SECTOR_COUNT; }
	uint32_t GetSize(void) override {
		return (SECTOR_SIZE * SECTOR_COUNT) / 1024U;
	}
	bool SectRead(uint32_t SectNo, uint8_t *pBuff) override {
		if (FailRead || SectNo >= SECTOR_COUNT || pBuff == nullptr)
			return false;
		memcpy(pBuff, Data[SectNo], SECTOR_SIZE);
		return true;
	}
	bool SectWrite(uint32_t SectNo, uint8_t *pData) override {
		if (FailWrite || SectNo >= SECTOR_COUNT || pData == nullptr)
			return false;
		memcpy(Data[SectNo], pData, SECTOR_SIZE);
		return true;
	}
	void Reset(void) override { ResetCount++; }

	void Fill(void) {
		for (uint32_t s = 0; s < SECTOR_COUNT; s++)
		{
			for (uint32_t n = 0; n < SECTOR_SIZE; n++)
			{
				Data[s][n] = (uint8_t)(s * 17U + n);
			}
		}
		FailRead = false;
		FailWrite = false;
		ResetCount = 0;
	}

	uint8_t Data[SECTOR_COUNT][SECTOR_SIZE] = {};
	bool FailRead = false;
	bool FailWrite = false;
	int ResetCount = 0;
};

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbCtrlrHighSpeed(int) { return false; }

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr || s_OpenCount >= 2)
		return false;
	s_OpenDesc[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t EpAddr) {
	s_CloseCount++;
	s_OpenCount = 0;
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_InBusy = false;
		s_InLength = 0U;
	}
	else
	{
		s_OutReady = false;
		s_OutDma = false;
		s_OutLength = 0U;
	}
}

bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer, bool,
	UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_InBuffer = pBuffer;
		s_InHandler = Handler;
		s_InContext = pContext;
	}
	else
	{
		s_OutBuffer = pBuffer;
		s_OutHandler = Handler;
		s_OutContext = pContext;
	}
	return pBuffer != nullptr && Handler != nullptr;
}

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint16_t Length)
{
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		if (s_InBusy || s_HaltIn)
			return false;
		s_InBusy = true;
		s_InLength = Length;
		return true;
	}
	if (!s_OutReady || s_OutDma || s_HaltOut)
		return false;
	s_OutDma = true;
	return true;
}

bool UsbCtrlrEp0Xfer(int, uint8_t, uint8_t *, uint16_t) { return true; }

bool UsbEpSetHalt(int DevNo, uint8_t EpAddr, bool Halt)
{
	if (DevNo != 0 || USB_ENDPADDR_NUM(EpAddr) != EP_NO)
		return false;
	if (USB_ENDPADDR_IS_IN(EpAddr))
		s_HaltIn = Halt;
	else
		s_HaltOut = Halt;
	return true;
}

bool UsbEpHalted(int DevNo, uint8_t EpAddr)
{
	return DevNo == 0 && USB_ENDPADDR_NUM(EpAddr) == EP_NO &&
		(USB_ENDPADDR_IS_IN(EpAddr) ? s_HaltIn : s_HaltOut);
}
}

bool UsbClassRegister(int DevNo, UsbDeviceClass *pClass,
	uint8_t FirstInterface, uint8_t InterfaceCount,
	uint16_t EpInMask, uint16_t EpOutMask)
{
	if (DevNo != 0 || pClass == nullptr || s_ClassObject != nullptr ||
		FirstInterface != ITF_NO || InterfaceCount != 1U ||
		EpInMask != (1U << EP_NO) || EpOutMask != (1U << EP_NO))
	{
		return false;
	}
	s_ClassObject = pClass;
	return true;
}

bool UsbDescriptorRegister(int DevNo, UsbDeviceClass *pClass,
	const void *pFsDescriptor, uint16_t FsDescriptorLength,
	const void *, uint16_t)
{
	if (DevNo != 0 || pClass != s_ClassObject || pFsDescriptor == nullptr)
		return false;
	s_FsDescriptor = static_cast<const uint8_t *>(pFsDescriptor);
	s_FsDescriptorLength = FsDescriptorLength;
	return true;
}

static void ResetFake(void)
{
	memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
	memset(s_OpenDesc, 0, sizeof(s_OpenDesc));
	memset(s_OutData, 0, sizeof(s_OutData));
	memset(s_Capture, 0, sizeof(s_Capture));
	s_UsbCfg.DevNo = 0;
	s_ClassObject = nullptr;
	s_FsDescriptor = nullptr;
	s_FsDescriptorLength = 0U;
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_OutBuffer = nullptr;
	s_InBuffer = nullptr;
	s_OutHandler = nullptr;
	s_InHandler = nullptr;
	s_OutContext = nullptr;
	s_InContext = nullptr;
	s_OutReady = false;
	s_OutDma = false;
	s_OutLength = 0U;
	s_InBusy = false;
	s_InLength = 0U;
	s_HaltIn = false;
	s_HaltOut = false;
	s_CaptureLength = 0U;
}

static UsbdMscCfg_t MakeCfg(RamDisk &Disk, uint8_t *pSectorBuffer,
							 bool ReadOnly = false)
{
	UsbdMscCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.pDisk = &Disk;
	cfg.pSectorBuffer = pSectorBuffer;
	cfg.SectorBufferSize = SECTOR_SIZE;
	cfg.bReadOnly = ReadOnly;
	cfg.bRemovable = true;
	cfg.InterfaceString = 4U;
	cfg.pVendor = "I-SYST";
	cfg.pProduct = "MSC HOST TEST";
	cfg.pRevision = "1.00";
	return cfg;
}

static void DeliverOut(const uint8_t *pData, uint16_t Length)
{
	CHECK(Length <= sizeof(s_OutData));
	CHECK(!s_OutReady);
	CHECK(s_OutHandler != nullptr);
	if (Length > sizeof(s_OutData) || s_OutReady || s_OutHandler == nullptr)
		return;
	if (Length > 0U)
		memcpy(s_OutData, pData, Length);
	s_OutLength = Length;
	s_OutReady = true;
	s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), USB_CTRLR_EVT_DRDY,
		Length, USB_CTRLR_XFER_SUCCESS, s_OutContext);
	if (!s_OutDma)
		return;
	if (Length > 0U)
		memcpy(s_OutBuffer, s_OutData, Length);
	s_OutReady = false;
	s_OutDma = false;
	s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
		s_OutLength, USB_CTRLR_XFER_SUCCESS, s_OutContext);
}

static void CompleteIn(void)
{
	CHECK(s_InBusy);
	if (!s_InBusy)
		return;
	CHECK(s_CaptureLength + s_InLength <= sizeof(s_Capture));
	if (s_CaptureLength + s_InLength <= sizeof(s_Capture) && s_InLength > 0U)
	{
		memcpy(&s_Capture[s_CaptureLength], s_InBuffer, s_InLength);
		s_CaptureLength += s_InLength;
	}
	const uint16_t length = s_InLength;
	s_InBusy = false;
	s_InHandler(USB_ENDPADDR_DIRIN(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
		length, USB_CTRLR_XFER_SUCCESS, s_InContext);
}

static UsbMscCmdBlkWrapper_t MakeCbw(uint32_t Tag, uint32_t Length,
								  bool In, uint8_t Opcode, uint8_t CdbLength)
{
	UsbMscCmdBlkWrapper_t cbw = {};
	cbw.dCBWSignature = USB_MSC_CBW_SIGNATURE;
	cbw.dCBWTag = Tag;
	cbw.dCBWDataTransferLength = Length;
	cbw.bmCBWFlags = In ? USB_MSC_CBW_FLAG_IN : 0U;
	cbw.bCBWLUN = 0U;
	cbw.bCBWCBLength = CdbLength;
	cbw.CBWCB[0] = Opcode;
	return cbw;
}

static void PutBe16(uint8_t *pData, uint16_t Value)
{
	pData[0] = (uint8_t)(Value >> 8);
	pData[1] = (uint8_t)Value;
}

static void PutBe32(uint8_t *pData, uint32_t Value)
{
	pData[0] = (uint8_t)(Value >> 24);
	pData[1] = (uint8_t)(Value >> 16);
	pData[2] = (uint8_t)(Value >> 8);
	pData[3] = (uint8_t)Value;
}

static const UsbMscCmdStatusWrapper_t *LastCsw(void)
{
	if (s_CaptureLength < sizeof(UsbMscCmdStatusWrapper_t))
		return nullptr;
	return reinterpret_cast<const UsbMscCmdStatusWrapper_t *>(
		&s_Capture[s_CaptureLength - sizeof(UsbMscCmdStatusWrapper_t)]);
}

static void Pump(UsbdMsc &Msc, int Limit = 200)
{
	while (Limit-- > 0)
	{
		if (s_InBusy)
			CompleteIn();
		Msc.Process();
		if (!s_InBusy && Msc.BotState() == USBD_MSC_BOT_WAIT_CBW)
			return;
		if (!s_InBusy && Msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY)
			return;
	}
	CHECK(false);
}

static void SendCbw(UsbdMsc &Msc, const UsbMscCmdBlkWrapper_t &Cbw)
{
	DeliverOut(reinterpret_cast<const uint8_t *>(&Cbw), sizeof(Cbw));
	Msc.Process();
}

static void RunInCommand(UsbdMsc &Msc, const UsbMscCmdBlkWrapper_t &Cbw)
{
	s_CaptureLength = 0U;
	SendCbw(Msc, Cbw);
	Pump(Msc);
}

static void CheckPassedCsw(uint32_t Tag, uint32_t Residue = 0U)
{
	const UsbMscCmdStatusWrapper_t *pCsw = LastCsw();
	CHECK(pCsw != nullptr);
	if (pCsw != nullptr)
	{
		CHECK(pCsw->dCSWSignature == USB_MSC_CSW_SIGNATURE);
		CHECK(pCsw->dCSWTag == Tag);
		CHECK(pCsw->dCSWDataResidue == Residue);
		CHECK(pCsw->bCSWStatus == USB_MSC_CMDSTATUS_PASS);
	}
}

static void TestInitDescriptorAndControl(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(s_ClassObject == &msc);
	CHECK(s_FsDescriptorLength == sizeof(UsbdMscDesc_t));
	const UsbdMscDesc_t *pDesc =
		reinterpret_cast<const UsbdMscDesc_t *>(s_FsDescriptor);
	CHECK(pDesc != nullptr);
	if (pDesc != nullptr)
	{
		CHECK(pDesc->Interface.bInterfaceClass == USB_INTRFCLASS_MSC);
		CHECK(pDesc->Interface.bInterfaceSubClass == USB_MSC_SUBCLASS_SCSI);
		CHECK(pDesc->Interface.bInterfaceProtocol == USB_MSC_PROT_BULK);
		CHECK(pDesc->Out.bEndpointAddress == USB_ENDPADDR_DIROUT(EP_NO));
		CHECK(pDesc->In.bEndpointAddress == USB_ENDPADDR_DIRIN(EP_NO));
	}

	CHECK(msc.SelectConfig(1U));
	CHECK(s_OpenCount == 2);
	UsbSetupData_t setup = {};
	setup.bmRequestType = USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_MSC_REQCODE_MAXLUN;
	setup.wIndex = ITF_NO;
	setup.wLength = 1U;
	uint8_t *pData = nullptr;
	uint16_t length = 0U;
	CHECK(msc.Control(&setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(pData != nullptr && *pData == 0U && length == 1U);
	setup.wLength = 2U;
	CHECK(!msc.Control(&setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(msc.SelectConfig(0U));
	CHECK(s_CloseCount == 2);
	CHECK(msc.Rate() == 0U);
}

static void TestReadOnlyCommands(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(1U, 36U, true,
		USB_MSC_SCSI_INQUIRY, 6U);
	cbw.CBWCB[4] = 36U;
	RunInCommand(msc, cbw);
	CHECK(s_CaptureLength == 36U + sizeof(UsbMscCmdStatusWrapper_t));
	CHECK(memcmp(&s_Capture[8], "I-SYST  ", 8U) == 0);
	CheckPassedCsw(1U);

	cbw = MakeCbw(2U, 0U, false, USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	RunInCommand(msc, cbw);
	CheckPassedCsw(2U);

	cbw = MakeCbw(3U, 8U, true, USB_MSC_SCSI_READ_CAPACITY_10, 10U);
	RunInCommand(msc, cbw);
	CHECK(s_Capture[3] == SECTOR_COUNT - 1U);
	CHECK(s_Capture[7] == SECTOR_SIZE);
	CheckPassedCsw(3U);

	cbw = MakeCbw(4U, 4U, true, USB_MSC_SCSI_MODE_SENSE_6, 6U);
	cbw.CBWCB[4] = 4U;
	RunInCommand(msc, cbw);
	CHECK(s_Capture[0] == 3U);
	CheckPassedCsw(4U);

	cbw = MakeCbw(5U, 0U, false, USB_MSC_SCSI_PREVENT_ALLOW, 6U);
	cbw.CBWCB[4] = 1U;
	RunInCommand(msc, cbw);
	CheckPassedCsw(5U);

	cbw = MakeCbw(6U, 0U, false, USB_MSC_SCSI_VERIFY_10, 10U);
	PutBe32(&cbw.CBWCB[2], 1U);
	PutBe16(&cbw.CBWCB[7], 2U);
	RunInCommand(msc, cbw);
	CheckPassedCsw(6U);

	cbw = MakeCbw(7U, 0U, false, USB_MSC_SCSI_SYNCHRONIZE_CACHE, 10U);
	RunInCommand(msc, cbw);
	CheckPassedCsw(7U);

	cbw = MakeCbw(8U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	cbw.CBWCB[4] = 1U;
	RunInCommand(msc, cbw);
	CheckPassedCsw(8U);

	cbw = MakeCbw(9U, 64U, true, USB_MSC_SCSI_INQUIRY, 6U);
	cbw.CBWCB[4] = 36U;
	RunInCommand(msc, cbw);
	CHECK(s_CaptureLength == 36U + sizeof(UsbMscCmdStatusWrapper_t));
	CheckPassedCsw(9U, 28U);
}

static void TestReadWriteAndRepeatedCommands(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(10U, SECTOR_SIZE * 2U, true,
		USB_MSC_SCSI_READ_10, 10U);
	PutBe32(&cbw.CBWCB[2], 2U);
	PutBe16(&cbw.CBWCB[7], 2U);
	RunInCommand(msc, cbw);
	CHECK(s_CaptureLength == SECTOR_SIZE * 2U + sizeof(UsbMscCmdStatusWrapper_t));
	CHECK(memcmp(s_Capture, disk.Data[2], SECTOR_SIZE) == 0);
	CHECK(memcmp(&s_Capture[SECTOR_SIZE], disk.Data[3], SECTOR_SIZE) == 0);
	CheckPassedCsw(10U);

	uint8_t writeData[SECTOR_SIZE * 2U];
	for (size_t n = 0; n < sizeof(writeData); n++)
		writeData[n] = (uint8_t)(0xA5U ^ n);
	cbw = MakeCbw(11U, sizeof(writeData), false, USB_MSC_SCSI_WRITE_10, 10U);
	PutBe32(&cbw.CBWCB[2], 4U);
	PutBe16(&cbw.CBWCB[7], 2U);
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	for (size_t offset = 0; offset < sizeof(writeData); offset += USBD_MSC_FS_MPS)
	{
		DeliverOut(&writeData[offset], USBD_MSC_FS_MPS);
		msc.Process();
	}
	Pump(msc);
	CHECK(memcmp(disk.Data[4], writeData, SECTOR_SIZE) == 0);
	CHECK(memcmp(disk.Data[5], &writeData[SECTOR_SIZE], SECTOR_SIZE) == 0);
	CheckPassedCsw(11U);

	cbw = MakeCbw(12U, SECTOR_SIZE, true, USB_MSC_SCSI_READ_10, 10U);
	PutBe32(&cbw.CBWCB[2], 4U);
	PutBe16(&cbw.CBWCB[7], 1U);
	RunInCommand(msc, cbw);
	CHECK(memcmp(s_Capture, writeData, SECTOR_SIZE) == 0);
	CheckPassedCsw(12U);
}

static void TestFailuresSenseResidueAndPhase(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(20U, 0U, false, 0xFFU, 6U);
	RunInCommand(msc, cbw);
	const UsbMscCmdStatusWrapper_t *pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);

	cbw = MakeCbw(21U, 18U, true, USB_MSC_SCSI_REQUEST_SENSE, 6U);
	cbw.CBWCB[4] = 18U;
	RunInCommand(msc, cbw);
	CHECK(s_Capture[2] == USB_MSC_SENSE_ILLEGAL_REQUEST);
	CHECK(s_Capture[12] == USB_MSC_ASC_INVALID_COMMAND);
	CheckPassedCsw(21U);

	cbw = MakeCbw(22U, SECTOR_SIZE, true, USB_MSC_SCSI_READ_10, 10U);
	PutBe32(&cbw.CBWCB[2], SECTOR_COUNT);
	PutBe16(&cbw.CBWCB[7], 1U);
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	CHECK(s_HaltIn);
	CHECK(msc.BotState() == USBD_MSC_BOT_FAILED);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));
	Pump(msc);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->dCSWDataResidue == SECTOR_SIZE);
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);

	cbw = MakeCbw(23U, 8U, false, USB_MSC_SCSI_INQUIRY, 6U);
	cbw.CBWCB[4] = 8U;
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	CHECK(s_HaltOut);
	Pump(msc);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->dCSWDataResidue == 8U);
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_PHASE_ERR);
	CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
}

static void TestReadOnlyAndStorageFailure(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector, true)));
	CHECK(msc.SelectConfig(1U));

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(30U, SECTOR_SIZE, false,
		USB_MSC_SCSI_WRITE_10, 10U);
	PutBe16(&cbw.CBWCB[7], 1U);
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	CHECK(s_HaltOut);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIROUT(EP_NO), false));
	Pump(msc);
	const UsbMscCmdStatusWrapper_t *pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);
	CHECK(pCsw != nullptr && pCsw->dCSWDataResidue == SECTOR_SIZE);

	CHECK(msc.SelectConfig(0U));
	CHECK(msc.SelectConfig(1U));
	disk.FailRead = true;
	cbw = MakeCbw(31U, SECTOR_SIZE, true, USB_MSC_SCSI_READ_10, 10U);
	PutBe16(&cbw.CBWCB[7], 1U);
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	msc.Process();
	CHECK(s_HaltIn);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));
	Pump(msc);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);
	CHECK(pCsw != nullptr && pCsw->dCSWDataResidue == SECTOR_SIZE);
}

static void TestWriteFailure(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	disk.FailWrite = true;
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));

	uint8_t data[SECTOR_SIZE] = {};
	UsbMscCmdBlkWrapper_t cbw = MakeCbw(32U, SECTOR_SIZE, false,
		USB_MSC_SCSI_WRITE_10, 10U);
	PutBe16(&cbw.CBWCB[7], 1U);
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	DeliverOut(data, USBD_MSC_FS_MPS);
	msc.Process();
	DeliverOut(&data[USBD_MSC_FS_MPS], USBD_MSC_FS_MPS);
	msc.Process();
	CHECK(s_HaltOut);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIROUT(EP_NO), false));
	Pump(msc);
	const UsbMscCmdStatusWrapper_t *pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);
	CHECK(pCsw != nullptr && pCsw->dCSWDataResidue == 0U);
}

static void BulkReset(UsbdMsc &Msc)
{
	UsbSetupData_t setup = {};
	setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_MSC_REQCODE_BOMSR;
	setup.wIndex = ITF_NO;
	CHECK(Msc.Control(&setup, USB_CTRL_SETUP, nullptr, nullptr));
}

static void TestMediumEjectAndRestart(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(60U, 0U, false,
		USB_MSC_SCSI_PREVENT_ALLOW, 6U);
	cbw.CBWCB[4] = 1U;
	RunInCommand(msc, cbw);
	CheckPassedCsw(60U);

	cbw = MakeCbw(61U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	cbw.CBWCB[4] = 2U;
	RunInCommand(msc, cbw);
	const UsbMscCmdStatusWrapper_t *pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);

	cbw = MakeCbw(62U, 18U, true, USB_MSC_SCSI_REQUEST_SENSE, 6U);
	cbw.CBWCB[4] = 18U;
	RunInCommand(msc, cbw);
	CHECK(s_Capture[2] == USB_MSC_SENSE_ILLEGAL_REQUEST);
	CHECK(s_Capture[12] == USB_MSC_ASC_MEDIUM_REMOVAL_PREVENTED);
	CHECK(s_Capture[13] == USB_MSC_ASCQ_REMOVAL_PREVENTED);
	CheckPassedCsw(62U);

	cbw = MakeCbw(63U, 0U, false, USB_MSC_SCSI_PREVENT_ALLOW, 6U);
	RunInCommand(msc, cbw);
	CheckPassedCsw(63U);

	cbw = MakeCbw(64U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	cbw.CBWCB[4] = 2U;
	RunInCommand(msc, cbw);
	CheckPassedCsw(64U);

	BulkReset(msc);
	cbw = MakeCbw(65U, 0U, false, USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	RunInCommand(msc, cbw);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);

	cbw = MakeCbw(66U, 18U, true, USB_MSC_SCSI_REQUEST_SENSE, 6U);
	cbw.CBWCB[4] = 18U;
	RunInCommand(msc, cbw);
	CHECK(s_Capture[2] == USB_MSC_SENSE_NOT_READY);
	CHECK(s_Capture[12] == USB_MSC_ASC_MEDIUM_NOT_PRESENT);
	CheckPassedCsw(66U);

	cbw = MakeCbw(67U, 8U, true, USB_MSC_SCSI_READ_CAPACITY_10, 10U);
	s_CaptureLength = 0U;
	SendCbw(msc, cbw);
	CHECK(s_HaltIn);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));
	Pump(msc);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->dCSWDataResidue == 8U);
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);

	cbw = MakeCbw(68U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	cbw.CBWCB[4] = 3U;
	RunInCommand(msc, cbw);
	CheckPassedCsw(68U);
	CHECK(disk.ResetCount == 1);

	cbw = MakeCbw(69U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	RunInCommand(msc, cbw);
	CheckPassedCsw(69U);
	cbw = MakeCbw(70U, 0U, false, USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	RunInCommand(msc, cbw);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);
	cbw = MakeCbw(71U, 18U, true, USB_MSC_SCSI_REQUEST_SENSE, 6U);
	cbw.CBWCB[4] = 18U;
	RunInCommand(msc, cbw);
	CHECK(s_Capture[12] == USB_MSC_ASC_LUN_NOT_READY);
	CHECK(s_Capture[13] == USB_MSC_ASCQ_INITIALIZING_REQUIRED);

	cbw = MakeCbw(72U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	cbw.CBWCB[4] = 1U;
	RunInCommand(msc, cbw);
	CheckPassedCsw(72U);

	cbw = MakeCbw(73U, 0U, false, USB_MSC_SCSI_START_STOP_UNIT, 6U);
	cbw.CBWCB[4] = 2U;
	RunInCommand(msc, cbw);
	msc.Reset();
	CHECK(msc.SelectConfig(1U));
	cbw = MakeCbw(74U, 0U, false, USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	RunInCommand(msc, cbw);
	pCsw = LastCsw();
	CHECK(pCsw != nullptr && pCsw->bCSWStatus == USB_MSC_CMDSTATUS_FAILED);
}

static void TestMalformedCbwAndResetRecovery(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));

	uint8_t shortCbw[20] = {};
	DeliverOut(shortCbw, sizeof(shortCbw));
	msc.Process();
	CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
	CHECK(s_HaltIn && s_HaltOut);
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));
	msc.Process();
	CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIROUT(EP_NO), false));
	msc.Process();
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(40U, 0U, false,
		USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	cbw.bmCBWFlags = 1U;
	SendCbw(msc, cbw);
	CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
	CHECK(s_HaltIn && s_HaltOut);

	struct MalformedCase {
		uint32_t Signature;
		uint8_t Lun;
		uint8_t CdbLength;
	};
	const MalformedCase cases[] = {
		{0U, 0U, 6U},
		{USB_MSC_CBW_SIGNATURE, 1U, 6U},
		{USB_MSC_CBW_SIGNATURE, 0U, 0U},
		{USB_MSC_CBW_SIGNATURE, 0U, 17U},
	};
	for (const MalformedCase &malformed : cases)
	{
		CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));
		CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIROUT(EP_NO), false));
		CHECK(msc.SelectConfig(0U));
		CHECK(msc.SelectConfig(1U));
		cbw = MakeCbw(41U, 0U, false,
			USB_MSC_SCSI_TEST_UNIT_READY, malformed.CdbLength);
		cbw.dCBWSignature = malformed.Signature;
		cbw.bCBWLUN = malformed.Lun;
		SendCbw(msc, cbw);
		CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
		CHECK(s_HaltIn && s_HaltOut);
	}
}

static void TestResetAcrossPhasesAndReconnect(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMsc msc;
	CHECK(msc.Init(MakeCfg(disk, sector)));
	CHECK(msc.SelectConfig(1U));
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);

	UsbMscCmdBlkWrapper_t cbw = MakeCbw(50U, SECTOR_SIZE, true,
		USB_MSC_SCSI_READ_10, 10U);
	PutBe16(&cbw.CBWCB[7], 1U);
	SendCbw(msc, cbw);
	CHECK(msc.BotState() == USBD_MSC_BOT_DATA_IN);
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);

	cbw = MakeCbw(51U, SECTOR_SIZE, false, USB_MSC_SCSI_WRITE_10, 10U);
	PutBe16(&cbw.CBWCB[7], 1U);
	SendCbw(msc, cbw);
	CHECK(msc.BotState() == USBD_MSC_BOT_DATA_OUT);
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);

	cbw = MakeCbw(52U, 0U, false, USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	SendCbw(msc, cbw);
	CHECK(msc.BotState() == USBD_MSC_BOT_SEND_CSW);
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);

	cbw = MakeCbw(54U, SECTOR_SIZE, true, USB_MSC_SCSI_READ_10, 10U);
	PutBe32(&cbw.CBWCB[2], SECTOR_COUNT);
	PutBe16(&cbw.CBWCB[7], 1U);
	SendCbw(msc, cbw);
	CHECK(msc.BotState() == USBD_MSC_BOT_FAILED);
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));

	cbw = MakeCbw(55U, 8U, false, USB_MSC_SCSI_INQUIRY, 6U);
	cbw.CBWCB[4] = 8U;
	SendCbw(msc, cbw);
	CHECK(msc.BotState() == USBD_MSC_BOT_PHASE_ERROR);
	BulkReset(msc);
	CHECK(msc.BotState() == USBD_MSC_BOT_RESET_RECOVERY);
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIROUT(EP_NO), false));
	CHECK(UsbEpSetHalt(0, USB_ENDPADDR_DIRIN(EP_NO), false));
	msc.Process();
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);

	msc.Reset();
	CHECK(disk.ResetCount == 1);
	CHECK(msc.BotState() == USBD_MSC_BOT_WAIT_CBW);
	CHECK(msc.Rate() == 0U);
	CHECK(msc.SelectConfig(1U));
	cbw = MakeCbw(53U, 0U, false, USB_MSC_SCSI_TEST_UNIT_READY, 6U);
	RunInCommand(msc, cbw);
	CheckPassedCsw(53U);
}

static void TestSectorBufferBounds(void)
{
	ResetFake();
	RamDisk disk;
	disk.Fill();
	alignas(4) uint8_t sector[SECTOR_SIZE];
	UsbdMscCfg_t cfg = MakeCfg(disk, sector);
	cfg.SectorBufferSize = SECTOR_SIZE - 1U;
	UsbdMsc msc;
	CHECK(!msc.Init(cfg));
}

int main(void)
{
	TestInitDescriptorAndControl();
	TestReadOnlyCommands();
	TestReadWriteAndRepeatedCommands();
	TestFailuresSenseResidueAndPhase();
	TestReadOnlyAndStorageFailure();
	TestWriteFailure();
	TestMediumEjectAndRestart();
	TestMalformedCbwAndResetRecovery();
	TestResetAcrossPhasesAndReconnect();
	TestSectorBufferBounds();

	if (s_Fail != 0)
	{
		printf("%d failed\n", s_Fail);
		return 1;
	}
	printf("all pass\n");
	return 0;
}
