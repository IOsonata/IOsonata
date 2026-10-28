/**-------------------------------------------------------------------------
@file usb_ep0_lpc_fs.h
@brief Private shared LPC USB device (IP3511) control endpoint state.

One state per controller, full or high speed. The port fills the fields kept
across a reset: the registers, the endpoint list and buffers, the list
layout and the number of endpoints. The list holds two words per physical
endpoint (OUT, then IN, of each endpoint number). Buffers are 64 byte
aligned: EP0 OUT, EP0 IN, then one buffer of BufSize bytes per data
physical endpoint. A list word holds the buffer offset (address / 64) and the
byte count: bits 15:0 and 25:16 on a full-speed controller, bits 10:0 and
25:11 on a high-speed one (bHsList).

bHsErrata enables the handling of the LPC546xx high-speed device errata
(ES_LPC546XX): USB.2, the byte count of a list word is not correct after an
IN packet, it is not checked; USB.15, the first byte of an IN packet that
follows an OUT packet of 4 + 16 x N bytes is sent as 0: after such an OUT
(OutQuirk) a data IN endpoint is armed only once an IN token has been NAKed
(DEVCMDSTAT INTONNAK_AI). A SETUP or an IN packet with data clears OutQuirk.
An IN packet already armed when such an OUT packet arrives is not covered.

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
	bool OutQuirk;				//!< Last OUT packet of 4 + 16 x N bytes, bHsErrata
	// Kept across a control-transfer reset.
	LpcUsbFsReg_t *pReg;
	volatile uint32_t *pList;	//!< Endpoint command list, 256 byte aligned
	uint8_t *pSetup;			//!< SETUP buffer
	uint8_t *pBuf;				//!< EP0 OUT and IN buffers, then the data endpoint buffers
	uint16_t BufSize;			//!< Buffer of a data endpoint: 64 or more, a multiple of 64
	uint8_t NbEp;				//!< Endpoint numbers, EP0 included
	bool bHsList;				//!< High-speed list layout
	bool bHighSpeed;			//!< Bus at high speed, set by the port at the bus reset
	bool bHsErrata;				//!< LPC546xx high-speed device errata USB.2 and USB.15
	bool (*Ready)(void);
	void (*Fault)(void);
	void (*Event)(const UsbCtrlrEvt_t *pEvent);
} LpcUsbFsEp0_t;

#ifdef __cplusplus
extern "C" {
#endif

void LpcUsbFsEp0Command(LpcUsbFsEp0_t *pEp0, uint32_t Clear, uint32_t Set, uint32_t Ack);
// Buffer of a physical endpoint, its list word with a byte count, and the
// byte count of a list word.
uint8_t *LpcUsbFsEp0Packet(LpcUsbFsEp0_t *pEp0, unsigned Physical);
uint32_t LpcUsbFsEp0BufferAddress(LpcUsbFsEp0_t *pEp0, unsigned Physical);
uint32_t LpcUsbFsEp0Entry(LpcUsbFsEp0_t *pEp0, unsigned Physical, uint32_t Length);
uint16_t LpcUsbFsEp0Length(LpcUsbFsEp0_t *pEp0, uint32_t Word);
bool LpcUsbFsEp0Retire(LpcUsbFsEp0_t *pEp0, uint32_t Mask);
// Record the length of an OUT packet received or an IN packet sent (USB.15)
void LpcUsbFsEp0Quirk(LpcUsbFsEp0_t *pEp0, bool In, uint16_t Length);
void LpcUsbFsEp0Reset(LpcUsbFsEp0_t *pEp0);
int LpcUsbFsEp0Send(LpcUsbFsEp0_t *pEp0, uint8_t *pBuffer, int Length);
bool LpcUsbFsEp0Status(LpcUsbFsEp0_t *pEp0, uint8_t EpAddr);
void LpcUsbFsEp0SetAddress(LpcUsbFsEp0_t *pEp0, uint8_t Address);
void LpcUsbFsEp0Stall(LpcUsbFsEp0_t *pEp0);
void LpcUsbFsEp0Interrupt(LpcUsbFsEp0_t *pEp0, uint32_t Pending);

#ifdef __cplusplus
}
#endif
#endif
