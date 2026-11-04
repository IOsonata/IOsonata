/**-------------------------------------------------------------------------
@example	usb_msc_flashdisk.cpp

@brief	USB Mass Storage Class disk on the MCU internal flash.

The flash version of usb_msc_ramdisk.cpp. The disk is the NVM1 region of the
linker script, so its size is set there and nowhere else, and the image fails
to link if it grows into the disk. It does not expose firmware, filesystem,
settings or Bluetooth bond-storage regions.

The content stays across resets and power cycles. An erased disk is formatted automatically. Existing signed volumes are preserved.
Unrecognized nonblank media are never reformatted.

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

#include "storage/diskio_nvm.h"
#include "storage/fatfs_format.h"
#include "storage/nvm_intrf.h"
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

int main(void)
{
	if (!s_Disk.InitMcu(s_Nvm, s_MemIntrf, MSC_NVM_REGION) ||
		!FatFsFormatIfBlank(s_Disk, s_SectorBuffer, sizeof(s_SectorBuffer)) ||
		!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !s_Msc.Init(s_MscCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);

	AppRun();
	return 0;
}
