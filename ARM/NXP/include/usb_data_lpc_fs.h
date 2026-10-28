/**-------------------------------------------------------------------------
@file usb_data_lpc_fs.h
@brief Private shared LPC USB device (IP3511) bulk and interrupt endpoint
state, full and high speed.

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
#ifndef __USB_DATA_LPC_FS_H__
#define __USB_DATA_LPC_FS_H__

#include "usb_ep0_lpc_fs.h"

typedef struct __Lpc_Usb_Fs_Data_Endpoint {
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	uint8_t *pBuffer;
	uint32_t Generation;
	uint16_t Mps;
	uint16_t Length;
	uint16_t Remaining;
	uint16_t Transferred;
	bool Busy;
	bool Halted;
	bool NakWait;				//!< IN armed at the next IN NAK, USB.15
} LpcUsbFsDataEndpoint_t;

// Data physical endpoints of the largest controller: 5 IN and 5 OUT
#define LPCUSB_DATA_EP_MAX	10

typedef struct __Lpc_Usb_Fs_Data {
	LpcUsbFsEp0_t *pEp0;
	bool *pPrepared;
	bool *pClosing;
	IRQn_Type Irq;
	LpcUsbFsDataEndpoint_t Endpoint[LPCUSB_DATA_EP_MAX];
	uint32_t RxReady;
} LpcUsbFsData_t;

#ifdef __cplusplus
extern "C" {
#endif

// Each family supplies the statically allocated context of a controller,
// NULL for a DevNo it does not have.
LpcUsbFsData_t *LpcUsbFsDataContext(int DevNo);
void LpcUsbFsDataInit(LpcUsbFsData_t *pData);
bool LpcUsbFsDataCurrent(LpcUsbFsData_t *pData);
void LpcUsbFsDataForget(LpcUsbFsData_t *pData, uint32_t Mask);
void LpcUsbFsDataInterrupt(LpcUsbFsData_t *pData, unsigned Physical);
// Physical endpoint mask of the data endpoints of the controller
uint32_t LpcUsbFsDataMask(LpcUsbFsData_t *pData);

#ifdef __cplusplus
}
#endif
#endif
