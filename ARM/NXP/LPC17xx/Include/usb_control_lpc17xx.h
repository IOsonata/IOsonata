/**-------------------------------------------------------------------------
@file	usb_control_lpc17xx.h

@brief	Private LPC17xx control endpoint event handling.

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
#ifndef USB_CONTROL_LPC17XX_H
#define USB_CONTROL_LPC17XX_H
#include "usb_ctrlr_lpc.h"

typedef struct __Lpc17_Usb_Control {
	uint32_t Generation;
	uint16_t RemainingOut;
	uint16_t InLength;
	uint8_t Address;
	uint8_t PendingAddress;
	bool AddressPending;
	bool Enabled;
	bool InBusy;
	bool StatusIn;
	bool StatusOut;
	bool DirectionIn;
	bool Halted;
	void (*Event)(const UsbCtrlrEvt_t *pEvent);
	void (*Fault)(void);
} Lpc17UsbControl_t;

#ifdef __cplusplus
extern "C" {
#endif
// Bind before enabling the USB NVIC/device IRQs; unbind after disabling them.
// The object must remain alive while bound. Only EP0 slow IRQs and DEV_STAT
// are supported. Fault must stop/disconnect hardware, including on VBUS loss.
bool Lpc17UsbIrqBind(Lpc17UsbControl_t *pControl);
void Lpc17UsbIrqUnbind(void);
// Zero-initialize the object and assign Event/Fault before enabling it.
// Reset software state to address zero, after the caller resets hardware.
// The caller services bus reset first and disables control NAK interrupts.
void Lpc17UsbControlReset(Lpc17UsbControl_t *pControl, bool Enable);
// Service confirmed hardware RST before endpoint dispatch. Emits RESET only
// after EP0 recovery succeeds; failures disable software state and call Fault.
void Lpc17UsbControlBusReset(Lpc17UsbControl_t *pControl);
int Lpc17UsbControlSend(Lpc17UsbControl_t *pControl, const uint8_t *pData, int Length);
bool Lpc17UsbControlStatus(Lpc17UsbControl_t *pControl, uint8_t EpAddr);
// Program the SIE address before status IN; hardware applies it after status.
// Confirm software address only on the acknowledged status interrupt.
bool Lpc17UsbControlSetAddress(Lpc17UsbControl_t *pControl, uint8_t Address);
void Lpc17UsbControlStall(Lpc17UsbControl_t *pControl);
// Called in USB interrupt context. No callbacks are queued. OUT event bytes
// are valid only during Event. One pending snapshot is serviced per call.
void Lpc17UsbControlInterrupt(Lpc17UsbControl_t *pControl);
#ifdef __cplusplus
}
#endif
#endif
