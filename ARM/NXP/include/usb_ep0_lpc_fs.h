/**-------------------------------------------------------------------------
@file usb_ep0_lpc_fs.h
@brief Private shared LPC full-speed control endpoint state.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
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
#ifndef __USB_EP0_LPC_FS_H__
#define __USB_EP0_LPC_FS_H__

#include "usb_ctrlr_lpc.h"
#include "usb_mem_lpc_fs.h"
#include "usb_lpc_fs_target.h"

typedef struct __Lpc_Usb_Fs_Ep0 {
	uint32_t Generation;
	uint16_t RemainingOut;
	uint16_t InLength;
	uint16_t OutLength;
	uint8_t OldAddress;
	bool Enabled;
	bool InBusy;
	bool OutBusy;
	bool DirectionIn;
	bool StatusOut;
	bool Halted;
	bool Aborted;
	bool AddressPending;
	// Kept across a control-transfer reset.
	LpcUsbFsReg_t *pReg;
	LpcUsbFsMemory_t *pMemory;
	bool (*Ready)(void);
	void (*Fault)(void);
	void (*Event)(const UsbCtrlrEvt_t *pEvent);
} LpcUsbFsEp0_t;

#ifdef __cplusplus
extern "C" {
#endif

void LpcUsbFsEp0Command(LpcUsbFsEp0_t *pEp0, uint32_t Clear, uint32_t Set, uint32_t Ack);
uint32_t LpcUsbFsEp0BufferAddress(LpcUsbFsEp0_t *pEp0, unsigned Physical);
bool LpcUsbFsEp0Retire(LpcUsbFsEp0_t *pEp0, uint32_t Mask);
void LpcUsbFsEp0Reset(LpcUsbFsEp0_t *pEp0);
int LpcUsbFsEp0Send(LpcUsbFsEp0_t *pEp0, int DevNo, uint8_t *pBuffer, int Length);
bool LpcUsbFsEp0Status(LpcUsbFsEp0_t *pEp0, int DevNo, uint8_t EpAddr);
void LpcUsbFsEp0SetAddress(LpcUsbFsEp0_t *pEp0, int DevNo, uint8_t Address);
void LpcUsbFsEp0Stall(LpcUsbFsEp0_t *pEp0);
void LpcUsbFsEp0Interrupt(LpcUsbFsEp0_t *pEp0, uint32_t Pending);

#ifdef __cplusplus
}
#endif
#endif
