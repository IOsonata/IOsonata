/**-------------------------------------------------------------------------
@file	usb_data_lpc17xx.h

@brief	Private LPC17xx CPU-mode data transfer state.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef USB_DATA_LPC17XX_H
#define USB_DATA_LPC17XX_H
#include "usb_ctrlr_lpc.h"

typedef struct __Lpc17_Usb_Data_Endpoint {
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	bool (*Ready)(void);
	void (*Fault)(void);
	void (*Kick)(uint8_t EpNo);
	uint8_t *pBuffer;
	uint32_t Generation;
	uint16_t Remaining;
	uint16_t Transferred;
	uint16_t PacketLength;
	uint16_t Mps;
	uint8_t EpNo;
	bool In;
	bool Enabled;
	bool Busy;
	bool Halted;
} Lpc17UsbDataEndpoint_t;

#ifdef __cplusplus
extern "C" {
#endif
// Optional IRQ binding, after binding control and before enabling interrupts.
// Array has 30 entries indexed by physical endpoint minus two. It must remain
// alive until Lpc17UsbIrqUnbind, which clears both control and data bindings.
// Owner's Ready must reject work while the bound control object is disabled.
bool Lpc17UsbIrqBindData(Lpc17UsbDataEndpoint_t Endpoint[30]);
// Zero-initialize and assign callbacks before Open. The owner realizes and
// enables hardware first. Ready rejects bus events and shutdown.
// Kick requests a later OUT IRQ pass; it must not invoke Dispatch inline.
// Fault must stop hardware before returning accepted buffers through Forget.
bool Lpc17UsbDataOpen(Lpc17UsbDataEndpoint_t *pEp, uint8_t EpNo, bool In, uint16_t Mps);
// Caller must retire hardware first and block all reentrant Open/submissions
// through Ready during shutdown. Preserves callback bindings; emits CANCEL.
void Lpc17UsbDataForget(Lpc17UsbDataEndpoint_t *pEp);
// Owner flushes hardware and masks the endpoint IRQ before this operation.
// Active work reports FAILED on halt; new submissions while halted wait.
void Lpc17UsbDataHalt(Lpc17UsbDataEndpoint_t *pEp, bool Halt);
// IN stages one packet at a time, completes once for the total length, and
// sends a ZLP only when explicitly requested with Length=0.
bool Lpc17UsbDataSend(Lpc17UsbDataEndpoint_t *pEp, uint8_t *pBuffer, uint16_t Length);
// OUT reserves one packet destination; completion never occurs inline here.
bool Lpc17UsbDataReceive(Lpc17UsbDataEndpoint_t *pEp, uint8_t *pBuffer, uint16_t Capacity);
// IRQ owner supplies raw endpoint status and Generation captured before any
// callbacks in that interrupt batch. One packet is serviced per invocation.
// IN must be called only for a genuine acknowledged endpoint interrupt.
void Lpc17UsbDataDispatch(Lpc17UsbDataEndpoint_t *pEp, uint32_t Generation, uint8_t Status);
#ifdef __cplusplus
}
#endif
#endif
