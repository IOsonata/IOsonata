/**-------------------------------------------------------------------------
@example	usb_msc_flashdisk.cpp

@brief	USB Mass Storage Class disk on the MCU internal flash.

The flash version of usb_msc_ramdisk.cpp. The disk is the NVM1 region of the
linker script, so its size is set there and nowhere else, and the image fails
to link if it grows into the disk. It does not expose firmware, filesystem,
settings or Bluetooth bond-storage regions.

This example uses a disposable NVM1 test volume: recognized filesystems
are preserved across resets; unrecognized contents cause an explicit
NVM1-only erase, then FatFs formatting. The generic FatFs mount helper
never formats a nonblank disk.

The 832 KiB variants use native 4 KiB sectors to avoid repeatedly
rewriting a full erase page for each 512-byte host write. The 256 KiB
variant uses 512-byte sectors with erase-unit scratch because it has
fewer than 128 physical sectors. There is no wear levelling.

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

#include "storage/diskio_nvm.h"
#include "ff.h"
#include "ff_diskio.h"
#include "storage/nvm_mcu_intrf.h"
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
#define MSC_STR_INTERFACE	4U

static NvmMcuIntrf s_MemIntrf;
static Nvm s_Nvm;
static NvmDiskIO s_Disk;
static FATFS s_FatFs;
static UsbdMsc s_Msc;

// Two FAT copies. Zero alignment uses GET_BLOCK_SIZE from DiskIO:
// one logical 4 KiB sector, or eight logical 512-byte sectors.
static const MKFS_PARM s_MkFsOptions = {
	FM_FAT | FM_SFD, 2, 0, 0, 0
};
alignas(4) static uint8_t s_SectorBuffer[MSC_SECTOR_BUFF_SIZE];
// Separate from USB / FatFs input: one physical erase unit for partial writes.
alignas(4) static uint8_t s_EraseBuffer[MSC_SECTOR_BUFF_SIZE];

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

int main(void)
{
	NvmCfg_t cfg = {};
	FatFsMountCfg_t mount = {};
	FRESULT res;
	uint64_t diskSize;
	uint32_t eraseSize;
	uint32_t sectorSize;

	// NVM1 is the linker-reserved USB disk, not the whole MCU flash.
	if (NvmMcuCfgInit(cfg, 1) == false)
	{
		return -1;
	}

	if (s_MemIntrf.Init() == false)
	{
		return -1;
	}

	if (s_Nvm.Init(cfg, &s_MemIntrf) == false)
	{
		return -1;
	}

	// On 832 KiB media, 208 native 4 KiB sectors are sufficient for
	// FatFs (minimum 128). Avoid eight erase/program operations for
	// each 4 KiB written by the host. The smaller 256 KiB region has
	// 64 physical pages and needs 512-byte logical sectors instead.
	diskSize = s_Nvm.Size();
	eraseSize = s_Nvm.EraseSize();

	if (eraseSize == 0 || eraseSize > MSC_SECTOR_BUFF_SIZE ||
		diskSize == 0 || (diskSize % eraseSize) != 0)
	{
		return -1;
	}

	sectorSize = diskSize / eraseSize >= 128U ? eraseSize : 512U;

	if (!s_Disk.Init(s_Nvm, sectorSize, s_EraseBuffer,
			sizeof(s_EraseBuffer)))
	{
		return -1;
	}

	// The firmware mounts and initializes the disk, then releases it
	// before USB MSC makes the same volume available to the host.
	mount.pDisk = &s_Disk;
	mount.pFatFs = &s_FatFs;
	mount.pPath = "0:";
	// Only scan for blank media after the normal mount has failed.
	mount.bFormatIfBlank = false;
	mount.pWork = s_SectorBuffer;
	mount.WorkSize = sizeof(s_SectorBuffer);
	mount.pLabelIfUnnamed = "0:IOSONATA";
	mount.pFormatOptions = &s_MkFsOptions;

	res = FatFsMount(mount);

	if (res == FR_NO_FILESYSTEM)
	{
		// This example owns a disposable flash disk. Old 256 KiB FAT
		// contents remain inside the enlarged 832 KiB NVM1 allocation.
		// Erase only NVM1 when no usable filesystem is present.
		// Generic FatFsMount never erases nonblank storage itself.
		if (FatFsUnmount("0:") != FR_OK)
		{
			return -1;
		}

		if (diskSize > UINT32_MAX ||
			s_Nvm.Erase(0, (uint32_t)diskSize) != 0 ||
			s_Nvm.Sync() != 0)
		{
			return -1;
		}

		// The nRF52 controller skips page erases that are already 0xFF.
		mount.bFormatIfBlank = true;
		res = FatFsMount(mount);
	}

	// I/O failures must not erase the disk or expose invalid FAT to USB.
	if (res != FR_OK || FatFsUnmount("0:") != FR_OK)
	{
		return -1;
	}

	if (AppEvtHandlerInit(g_AppEvtHandlerQueMem,
			sizeof(g_AppEvtHandlerQueMem)) == false)
	{
		return -1;
	}

	if (UsbInit(&s_UsbCfg) == false)
	{
		return -1;
	}

	if (s_Msc.Init(s_MscCfg) == false)
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);

	AppRun();

	return 0;
}
