/**-------------------------------------------------------------------------
@example	usb_msc_flashdisk.cpp

@brief	USB Mass Storage Class disk on the MCU internal flash.

The flash version of usb_msc_ramdisk.cpp. The disk is the NVM1 region of the
linker script, so its size is set there and nowhere else, and the image fails
to link if it grows into the disk. It does not expose firmware, filesystem,
settings or Bluetooth bond-storage regions.

The content stays across resets and power cycles. The volume is formatted only
when sector 0 has no 55AA signature, which is the case on a blank region.

A sector is one erase unit, the smallest sector NvmDiskIO accepts, so the host
sees that block size in READ CAPACITY rather than 512 bytes. Each sector the
host writes is one erase and one program, with no wear levelling. It runs from
the USB process event, which holds the main loop until the memory has finished.

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
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"
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
// Largest sector FAT allows. UsbdMsc refuses a medium with a larger one.
#define MSC_SECTOR_BUFF_SIZE	4096U
// Linker region holding the disk. NVM0 is the Bluetooth bond store.
#define MSC_NVM_REGION		1
#define MSC_STR_INTERFACE	4U

static NvmIntrf s_MemIntrf;
static Nvm s_Nvm;
static NvmDiskIO s_Disk;
static UsbdMsc s_Msc;
alignas(4) static uint8_t s_SectorBuffer[MSC_SECTOR_BUFF_SIZE];

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

// Write an empty FAT12 volume with one sector per cluster and a one sector root
// directory. The boot sector goes last, so a format cut short by a reset is
// started over at the next power up.
static bool MscDiskFormat(DiskIO &Disk, uint8_t *pBuff)
{
	const uint16_t sectSize = Disk.GetSectSize();
	const uint32_t nbSect = Disk.GetNbSect();
	// 12 bits per entry, two of them reserved. The sector count is a little
	// above the cluster count, which can only round the table size up.
	const uint16_t fatSect =
		(uint16_t)(((nbSect + 2U) * 3U / 2U + sectSize - 1U) / sectSize);
	const uint32_t rootSect =
		FATFS_RSVDSECCNT_FAT16 + (uint32_t)FATFS_NBFAT * fatSect;

	for (uint32_t i = 0; i < (uint32_t)FATFS_NBFAT * fatSect; i++)
	{
		memset(pBuff, 0, sectSize);
		if ((i % fatSect) == 0U)
		{
			pBuff[0] = FATFS_MEDIA_FIXED;
			pBuff[1] = 0xFFU;
			pBuff[2] = 0xFFU;
		}
		if (!Disk.SectWrite(FATFS_RSVDSECCNT_FAT16 + i, pBuff))
		{
			return false;
		}
	}

	memset(pBuff, 0, sectSize);
	memcpy(pBuff, "IOSONATA   ", 11U);
	pBuff[11] = FATFS_DIRATTR_VOLUME_ID;
	if (!Disk.SectWrite(rootSect, pBuff))
	{
		return false;
	}

	FATFS_BSBPB *pBoot = reinterpret_cast<FATFS_BSBPB *>(pBuff);

	memset(pBuff, 0, sectSize);
	pBoot->JmpBoot[0] = 0xEBU;
	pBoot->JmpBoot[1] = 0x3CU;
	pBoot->JmpBoot[2] = 0x90U;
	memcpy(pBoot->OEMName, "IOsonata", sizeof(pBoot->OEMName));
	pBoot->BytsPerSec = sectSize;
	pBoot->SecPerClus = 1U;
	pBoot->RsvdSecCnt = FATFS_RSVDSECCNT_FAT16;
	pBoot->NumFATs = FATFS_NBFAT;
	pBoot->RootEntCnt = sectSize / 32U;
	pBoot->TotSec16 = (uint16_t)nbSect;
	pBoot->Media = FATFS_MEDIA_FIXED;
	pBoot->FATSz16 = fatSect;
	pBoot->SecPerTrk = 32U;
	pBoot->NumHeads = 1U;
	pBoot->BPB.Bpb16.DrvNum = 0x80U;
	pBoot->BPB.Bpb16.BootSig = 0x29U;
	pBoot->BPB.Bpb16.VolID = 0x4E534F49U;
	memcpy(pBoot->BPB.Bpb16.VolLab, "IOSONATA   ", 11U);
	memcpy(pBoot->BPB.Bpb16.FilSysType, "FAT12   ", 8U);
	// FAT reads the signature at byte 510. It is repeated at the end of the
	// sector, the other place tools look for it on a sector larger than 512.
	pBoot->Signature_word[0] = 0x55U;
	pBoot->Signature_word[1] = 0xAAU;
	pBuff[sectSize - 2U] = 0x55U;
	pBuff[sectSize - 1U] = 0xAAU;

	return Disk.SectWrite(0U, pBuff);
}

static bool MscDiskInit(void)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	NvmMcuCfg(cfg);
	// The region is the device, the same way the bond store sits on NVM0.
	cfg.BaseAddr = NvmRegionAddr(MSC_NVM_REGION);
	cfg.TotalSize = NvmRegionSize(MSC_NVM_REGION);

	if (!s_MemIntrf.Init() || !s_Nvm.Init(cfg, &s_MemIntrf) ||
		!s_Disk.Init(s_Nvm) || !s_Disk.SectRead(0U, s_SectorBuffer))
	{
		return false;
	}

	const FATFS_BSBPB *pBoot =
		reinterpret_cast<const FATFS_BSBPB *>(s_SectorBuffer);

	// Keep whatever the host or an earlier run left, FAT or partition table.
	if (pBoot->Signature_word[0] == 0x55U && pBoot->Signature_word[1] == 0xAAU)
	{
		return true;
	}

	return MscDiskFormat(s_Disk, s_SectorBuffer);
}

int main(void)
{
	if (!MscDiskInit() ||
		!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !s_Msc.Init(s_MscCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);

	AppRun();
	return 0;
}
