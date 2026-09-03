/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	Host test stand in for a port supplied usb_ctrlr.h.

Describes one full speed controller with eight endpoints in each direction.
The host tests link against a fake controller, so these numbers only have to
be a valid target shape.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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

#define USB_CTRLR_CNT					1

#define USB_PASTE3_(a, b, c)			a##b##c
#define USB_PASTE3(a, b, c)				USB_PASTE3_(a, b, c)
#define USB_PASTE2_(a, b)				a##b
#define USB_PASTE2(a, b)				USB_PASTE2_(a, b)

#define USB_PKT_MAXLEN_0_CONTROL		64
#define USB_PKT_MAXLEN_0_BULK			64
#define USB_PKT_MAXLEN_0_INT			64
#define USB_PKT_MAXLEN_0_ISO			1023

#define USB_PKT_MAXLEN(CtrlrNo, TransType) \
	USB_PASTE3(USB_PKT_MAXLEN_, CtrlrNo, USB_PASTE2(_, TransType))

#define USB_EPIN_CNT(CtrlrNo)			8
#define USB_EPOUT_CNT(CtrlrNo)			8
#define USB_HIGHSPEED_CAPABLE(CtrlrNo)	0
#define USB_ISO_SUPPORTED(CtrlrNo)		1

#endif
