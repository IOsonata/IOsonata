/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief LPC1769 full-speed USB device controller.

Requires a valid 48 MHz USB clock derived from the configured external main
oscillator. Clock-on suspend only; no remote wake, SOF, ISO or DMA.
Interrupt endpoints: 1, 4, 7, 10, 13. Bulk: 2, 5, 8, 11, 14, 15.
Pin map order: VBUS P1.30 FUNC2 input, DP P0.29 FUNC1 input,
DM P0.30 FUNC1 input, active-low SoftConnect P2.9 FUNC1 output.
All pins use no pull and normal type. SoftConnect must control an external
pull-up circuit. Automatic VBUS rising-edge detection is not implemented;
use with VBUS present at startup for this bring-up stage.

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

// Counts span the fixed hardware numbering; allocation masks select legal types.
// Counts include EP0; each endpoint number has independent IN and OUT.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 0,
	USB_EPIN_CNT_0 = 16,
	USB_EPOUT_CNT_0 = 16,
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 64,
	USB_CTRLR0_INT_PKT_LEN_MAX = 64,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 0,
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

// Logical endpoint numbers, identical capabilities in both directions.
#define USB_EPIN_TYPE_MASK(CtrlrNo, TransType) \
	((CtrlrNo) != 0 ? 0U : \
	 (TransType) == BULK ? 0xC924U : \
	 (TransType) == INT ? 0x2492U : \
	 (TransType) == CONTROL ? 1U : 0U)
#define USB_EPOUT_TYPE_MASK(CtrlrNo, TransType) \
	USB_EPIN_TYPE_MASK(CtrlrNo, TransType)

#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)


#endif
