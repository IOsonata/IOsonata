/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	LPC11Uxx USB device controller interface.

Full-speed device controller with endpoint numbers 0 through 4 in each
direction. EP0 is control; endpoints 1 through 4 implement bulk and interrupt
transfers with a 64-byte packet buffer per direction. The silicon ISO packet
limit is recorded below, but USB_ISO_SUPPORTED and its masks are zero because
this driver does not implement ISO. Remote wakeup uses the normal clock-on
suspend mode; the generic core owns host permission checks.

Init currently requires one VBUS pin-map entry: P0.3, FUNC1, input, no pull,
normal type. Start configures the fixed P0.6 FUNC1 USB_CONNECT output for the
external soft-connect circuit. USB_DM/DP are dedicated pads. Boards without
VBUS sensing or soft-connect switching are not supported by this lifecycle.
Low-power suspend must be false; the USB clock remains on during suspend.

The declarations follow the existing IOsonata USB controller interface.
ROM DFU is independent of this application USB interface.

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
#ifndef __USB_CTRLR_H__
#define __USB_CTRLR_H__

#include "usb_ctrlr_lpc.h"

// NXP LPC11U3x data sheet section 7.9.1.1: ten physical endpoints.
// Counts include EP0; each endpoint number has independent IN and OUT.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 0,
	USB_EPIN_CNT_0 = 5,
	USB_EPOUT_CNT_0 = 5,
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 64,
	USB_CTRLR0_INT_PKT_LEN_MAX = 64,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 1023,
	USB_ISO_SUPPORTED_0 = 0,
	USB_ISO_EPIN_MASK_0 = 0,
	USB_ISO_EPOUT_MASK_0 = 0,
};

#define USB_EPIN_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPIN_CNT_0 : 0)
#define USB_EPOUT_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPOUT_CNT_0 : 0)
#define USB_HIGHSPEED_CAPABLE(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_HIGHSPEED_CAPABLE_0 : 0)
#define USB_ISO_SUPPORTED(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_SUPPORTED_0 : 0)
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPIN_MASK_0 : 0U)
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPOUT_MASK_0 : 0U)
#define USB_CTRLR_PKT_LEN_MAX(CtrlrNo, TransType) \
	((CtrlrNo) != 0 ? 0 : \
	 (TransType) == CONTROL ? USB_CTRLR0_CONTROL_PKT_LEN_MAX : \
	 (TransType) == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : \
	 (TransType) == BULK ? USB_CTRLR0_BULK_PKT_LEN_MAX : \
	 (TransType) == INT ? USB_CTRLR0_INT_PKT_LEN_MAX : 0)

#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)


#endif
