/**-------------------------------------------------------------------------
@example	usb_msc_ramdisk.cpp

@brief	USB Mass Storage Class static RAM-disk example.

This example exposes only a dedicated 64 KiB RAM disk. It does not expose
firmware, filesystem, settings or Bluetooth bond-storage regions.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "storage/diskio.h"
#include "usb/usb.h"
#include "usb/usbd_msc.h"

#define USB_DEVNO			0
#define MSC_SECTOR_SIZE		512U
#define MSC_SECTOR_COUNT	128U
#define MSC_STR_INTERFACE	4U

class MscRamDisk : public DiskIO {
public:
	void Init(void)
	{
		memset(vData, 0, sizeof(vData));
		uint8_t *pBoot = vData[0];
		pBoot[0] = 0xEBU;
		pBoot[1] = 0x3CU;
		pBoot[2] = 0x90U;
		memcpy(&pBoot[3], "IOsonata", 8U);
		Put16(&pBoot[11], MSC_SECTOR_SIZE);
		pBoot[13] = 1U;
		Put16(&pBoot[14], 1U);
		pBoot[16] = 2U;
		Put16(&pBoot[17], 32U);
		Put16(&pBoot[19], MSC_SECTOR_COUNT);
		pBoot[21] = 0xF8U;
		Put16(&pBoot[22], 1U);
		Put16(&pBoot[24], 32U);
		Put16(&pBoot[26], 1U);
		pBoot[36] = 0x80U;
		pBoot[38] = 0x29U;
		pBoot[39] = 0x49U;
		pBoot[40] = 0x4FU;
		pBoot[41] = 0x53U;
		pBoot[42] = 0x4EU;
		memcpy(&pBoot[43], "IOSONATA   ", 11U);
		memcpy(&pBoot[54], "FAT12   ", 8U);
		pBoot[510] = 0x55U;
		pBoot[511] = 0xAAU;

		vData[1][0] = 0xF8U;
		vData[1][1] = 0xFFU;
		vData[1][2] = 0xFFU;
		memcpy(vData[2], vData[1], MSC_SECTOR_SIZE);
		memcpy(&vData[3][0], "IOSONATA   ", 11U);
		vData[3][11] = 0x08U;
	}

	uint16_t GetSectSize(void) override { return MSC_SECTOR_SIZE; }
	uint32_t GetNbSect(void) override { return MSC_SECTOR_COUNT; }
	uint32_t GetSize(void) override {
		return (MSC_SECTOR_SIZE * MSC_SECTOR_COUNT) / 1024U;
	}
	bool SectRead(uint32_t SectNo, uint8_t *pBuff) override
	{
		if (SectNo >= MSC_SECTOR_COUNT || pBuff == nullptr)
			return false;
		memcpy(pBuff, vData[SectNo], MSC_SECTOR_SIZE);
		return true;
	}
	bool SectWrite(uint32_t SectNo, uint8_t *pData) override
	{
		if (SectNo >= MSC_SECTOR_COUNT || pData == nullptr)
			return false;
		memcpy(vData[SectNo], pData, MSC_SECTOR_SIZE);
		return true;
	}

private:
	static void Put16(uint8_t *pData, uint16_t Value)
	{
		pData[0] = (uint8_t)Value;
		pData[1] = (uint8_t)(Value >> 8);
	}

	alignas(4) uint8_t vData[MSC_SECTOR_COUNT][MSC_SECTOR_SIZE];
};

static MscRamDisk s_RamDisk;
static UsbdMsc s_Msc;
alignas(4) static uint8_t s_SectorBuffer[MSC_SECTOR_SIZE];

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0004,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata MSC RAM Disk",
	.pSerial = nullptr,
	.pFuncName = "Mass Storage",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_NONE,
	.DeviceSubClass = 0U,
	.DeviceProtocol = 0U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

static const UsbdMscCfg_t s_MscCfg = {
	.DevNo = USB_DEVNO,
	.pDisk = &s_RamDisk,
	.pSectorBuffer = s_SectorBuffer,
	.SectorBufferSize = sizeof(s_SectorBuffer),
	.bReadOnly = false,
	.bRemovable = true,
	.InterfaceString = MSC_STR_INTERFACE,
	.FsMps = 0U,
	.HsMps = 0U,
	.pVendor = "I-SYST",
	.pProduct = "IOsonata RAM Disk",
	.pRevision = "1.00",
};

int main(void)
{
	s_RamDisk.Init();
	if (!UsbInit(&s_UsbCfg) || !s_Msc.Init(s_MscCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);

	while (1)
	{
		UsbProcess(USB_DEVNO);
	}
	return 0;
}
