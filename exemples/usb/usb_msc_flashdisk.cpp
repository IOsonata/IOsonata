/**-------------------------------------------------------------------------
@example	usb_msc_flashdisk.cpp

@brief	USB Mass Storage Class disk on the MCU internal flash.

The flash version of usb_msc_ramdisk.cpp. The disk is the NVM1 region of the
linker script, so its size is set there and nowhere else, and the image fails
to link if it grows into the disk. It does not expose firmware, filesystem,
settings or Bluetooth bond-storage regions.

The content stays across resets and power cycles. The example provisions
FAT12 metadata only when sector zero lacks a boot signature, as the original
example did. Existing signed volumes are preserved.

The host sees 512-byte logical sectors. NvmDiskIO caches one 4 KiB erase
page in caller-owned RAM. Writes to that page are coalesced and committed
before MSC acknowledges WRITE(10). There is no wear levelling.

@author	Hoang Nguyen Hoan
@date	Nov. 4, 2026

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
#include <stdint.h>
#include <string.h>

#include "storage/diskio_nvm.h"
#include "storage/fatfs.h"
#include "storage/nvm_mcu_intrf.h"
#include "app_evt_handler.h"
#include "usb/usb.h"
#if __has_include("board.h")
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "board.h"
#endif
#include "usb/usbd_msc.h"
#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

// board.h can select another controller
#ifndef USB_DEVNO
#define USB_DEVNO			0
#endif
// Expose conventional 512-byte USB sectors backed by 4 KiB erase pages.
#define MSC_SECTOR_BUFF_SIZE	512U
#define MSC_ERASE_WORK_SIZE	4096U
// Linker-reserved USB disk region (NVM1); NVM0, if present, is PDS.
#define MSC_NVM_REGION		1
#define MSC_STR_INTERFACE	4U

static NvmMcuIntrf s_MemIntrf;
static Nvm s_Nvm;
static NvmDiskIO s_Disk;
static UsbdMsc s_Msc;
alignas(4) static uint8_t s_SectorBuffer[MSC_SECTOR_BUFF_SIZE];
alignas(4) static uint8_t s_EraseWork[MSC_ERASE_WORK_SIZE];

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0004,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata MSC Flash Disk",
	.pFuncName = "Mass Storage",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#endif
	.MaxPower = 100,
};

static const UsbdMscCfg_t s_MscCfg = {
	.DevNo = USB_DEVNO,
	.pDisk = &s_Disk,
	.pSectorBuffer = s_SectorBuffer,
	.SectorBufferSize = sizeof(s_SectorBuffer),
	.bRemovable = true,
	.InterfaceString = MSC_STR_INTERFACE,
	.pProduct = "IOsonata Flash",
};

// Provision a FAT12 superfloppy only when sector 0 has no boot signature.
// This is a one-time operation: never clear a signed existing volume.
// Initialize the boot sector last, so an interrupted initialization retries.
static bool MscDiskProvision(void)
{
	const uint16_t sectSize = s_Disk.GetSectSize();
	const uint32_t nbSect = s_Disk.GetNbSect();
	const uint16_t fatSect = (uint16_t)((((nbSect + 2U) * 3U + 1U) / 2U +
		sectSize - 1U) / sectSize);
	const uint32_t rootSect = FATFS_RSVDSECCNT_FAT16 +
		(uint32_t)FATFS_NBFAT * fatSect;
	const uint32_t rootCount = sectSize / 32U;

	if (!s_Disk.SectRead(0, s_SectorBuffer))
	{
		return false;
	}
	if (s_SectorBuffer[510] == 0x55 && s_SectorBuffer[511] == 0xAA)
	{
		// Do not silently overwrite a volume with an incompatible sector size.
		return s_SectorBuffer[11] == (uint8_t)sectSize &&
			s_SectorBuffer[12] == (uint8_t)(sectSize >> 8);
	}
	if (nbSect < rootSect + 2U || nbSect > 4084U || sectSize != 512U)
	{
		return false;
	}

	for (uint32_t i = 0; i < (uint32_t)FATFS_NBFAT * fatSect; i++)
	{
		memset(s_SectorBuffer, 0, sectSize);
		if (i % fatSect == 0)
		{
			s_SectorBuffer[0] = FATFS_MEDIA_FIXED;
			s_SectorBuffer[1] = 0xFF;
			s_SectorBuffer[2] = 0xFF;
		}
		if (!s_Disk.SectWrite(FATFS_RSVDSECCNT_FAT16 + i, s_SectorBuffer))
		{
			return false;
		}
	}

	memset(s_SectorBuffer, 0, sectSize);
	memcpy(s_SectorBuffer, "IOSONATA   ", 11);
	s_SectorBuffer[11] = FATFS_DIRATTR_VOLUME_ID;
	if (!s_Disk.SectWrite(rootSect, s_SectorBuffer))
	{
		return false;
	}

	FATFS_BSBPB *pBoot = reinterpret_cast<FATFS_BSBPB *>(s_SectorBuffer);

	memset(s_SectorBuffer, 0, sectSize);
	pBoot->JmpBoot[0] = 0xEB;
	pBoot->JmpBoot[1] = 0x3C;
	pBoot->JmpBoot[2] = 0x90;
	memcpy(pBoot->OEMName, "IOsonata", sizeof(pBoot->OEMName));
	pBoot->BytsPerSec = sectSize;
	pBoot->SecPerClus = 1;
	pBoot->RsvdSecCnt = FATFS_RSVDSECCNT_FAT16;
	pBoot->NumFATs = FATFS_NBFAT;
	pBoot->RootEntCnt = rootCount;
	pBoot->TotSec16 = (uint16_t)nbSect;
	pBoot->Media = FATFS_MEDIA_FIXED;
	pBoot->FATSz16 = fatSect;
	pBoot->SecPerTrk = 32;
	pBoot->NumHeads = 1;
	pBoot->BPB.Bpb16.DrvNum = 0x80;
	pBoot->BPB.Bpb16.BootSig = 0x29;
	pBoot->BPB.Bpb16.VolID = 0x4E534F49;
	memcpy(pBoot->BPB.Bpb16.VolLab, "IOSONATA   ", 11);
	memcpy(pBoot->BPB.Bpb16.FilSysType, "FAT12   ", 8);
	pBoot->Signature_word[0] = 0x55;
	pBoot->Signature_word[1] = 0xAA;

	if (!s_Disk.SectWrite(0, s_SectorBuffer))
	{
		return false;
	}
	if (!s_Disk.Commit())
	{
		return false;
	}

	// Verify the boot sector from physical flash, bypassing the page cache.
	uint8_t verify[512];
	if (s_Nvm.Read(0, verify, sizeof(verify)) != (int)sizeof(verify))
	{
		return false;
	}
	return memcmp(verify, s_SectorBuffer, sizeof(verify)) == 0;
}

int main(void)
{
	NvmCfg_t cfg = {};

	if (!NvmMcuCfgInit(cfg, MSC_NVM_REGION))
	{
		return -1;
	}
	if (!s_MemIntrf.Init())
	{
		return -1;
	}
	if (!s_Nvm.Init(cfg, &s_MemIntrf))
	{
		return -1;
	}
	if (!s_Disk.Init(s_Nvm, MSC_SECTOR_BUFF_SIZE, s_EraseWork,
			sizeof(s_EraseWork), true))
	{
		return -1;
	}
	if (!MscDiskProvision())
	{
		return -1;
	}
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)))
	{
		return -1;
	}
	if (!UsbInit(&s_UsbCfg))
	{
		return -1;
	}
	if (!s_Msc.Init(s_MscCfg))
	{
		return -1;
	}
	if (!UsbEnable(USB_DEVNO))
	{
		return -1;
	}

	AppRun();
	return 0;
}
