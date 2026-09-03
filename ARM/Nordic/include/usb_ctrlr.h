/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	USB controller description for Nordic parts.

Everything about the USB controllers that changes from one target to the next.
Every target that has a USB controller provides a usb_ctrlr.h of its own on its
include path, the same way it provides iopinctrl.h. Generic USB code includes
it by plain name and never switches on a vendor macro.

The UsbCtrlr entry points do not change across targets, so they are declared
once in usb.h rather than repeated here. This file is the part that differs.

The values here describe the silicon, not a class or a configuration. They are
constants so an application can size CFifo memory and DMA staging buffers
statically, before any endpoint is configured and without calling into the
stack.

Controller numbering starts at zero and matches the CtrlrNo argument of the
USB API. The accessor macros expand named constants before pasting their
arguments. The expanded CtrlrNo and TransType still have to be compile-time
tokens, not variables or expressions.

Packet lengths are allocation bounds: the largest packet the controller can
move on that transfer type at the fastest speed it supports. The value an
endpoint actually negotiates comes from its descriptor and may be smaller. A
buffer sized from these macros is large enough at every speed.

TransType tokens match the transfer types in usb_def.h: CONTROL, ISO, BULK,
INT.

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

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_peripherals.h"

/** @addtogroup USB
  * @{
  */

/// Maximum packet length in bytes for one controller and transfer type.
#define USB_PKT_MAXLEN_(CtrlrNo, TransType)	USB_PKT_MAXLEN_##CtrlrNo##_##TransType
#define USB_PKT_MAXLEN(CtrlrNo, TransType)	USB_PKT_MAXLEN_(CtrlrNo, TransType)

/// IN endpoint numbers, including endpoint zero.
#define USB_EPIN_CNT_(CtrlrNo)				USB_EPIN_CNT_##CtrlrNo
#define USB_EPIN_CNT(CtrlrNo)				USB_EPIN_CNT_(CtrlrNo)

/// OUT endpoint numbers, including endpoint zero.
#define USB_EPOUT_CNT_(CtrlrNo)				USB_EPOUT_CNT_##CtrlrNo
#define USB_EPOUT_CNT(CtrlrNo)				USB_EPOUT_CNT_(CtrlrNo)

/// Nonzero when the controller can enumerate at high speed.
#define USB_HIGHSPEED_CAPABLE_(CtrlrNo)		USB_HIGHSPEED_CAPABLE_##CtrlrNo
#define USB_HIGHSPEED_CAPABLE(CtrlrNo)		USB_HIGHSPEED_CAPABLE_(CtrlrNo)

/// Nonzero when this port drives the controller's isochronous endpoints.
#define USB_ISO_SUPPORTED_(CtrlrNo)			USB_ISO_SUPPORTED_##CtrlrNo
#define USB_ISO_SUPPORTED(CtrlrNo)			USB_ISO_SUPPORTED_(CtrlrNo)

#if defined(USBD_PRESENT)

/// Number of USB controllers on this part.
#define USB_CTRLR_CNT						1

// nRF52840 and nRF5340 USBD. Full speed only. Endpoint numbers 0 through 7 in
// each direction. Endpoint 8 is isochronous only and is not counted here
// because usbd_ctrlr_nrf52 does not drive it.
#define USB_HIGHSPEED_CAPABLE_0				0
#define USB_EPIN_CNT_0						8
#define USB_EPOUT_CNT_0						8

#define USB_PKT_MAXLEN_0_CONTROL			64
#define USB_PKT_MAXLEN_0_BULK				64
#define USB_PKT_MAXLEN_0_INT				64

// The hardware isochronous endpoint carries a full-speed maximum of 1023
// bytes. usbd_ctrlr_nrf52 leaves isochronous out, so allocating against this
// value buys nothing until that changes. USB_ISO_SUPPORTED says which is true.
#define USB_PKT_MAXLEN_0_ISO				1023
#define USB_ISO_SUPPORTED_0					0

#elif defined(USBHS_PRESENT)

/// Number of USB controllers on this part.
#define USB_CTRLR_CNT						1

// nRF54 USBHS. High speed capable, so the bulk and interrupt bounds are the
// high-speed maxima. A full-speed host negotiates smaller packets at
// enumeration and a buffer sized from these still fits.
#define USB_HIGHSPEED_CAPABLE_0				1
#define USB_EPIN_CNT_0						16
#define USB_EPOUT_CNT_0						16

#define USB_PKT_MAXLEN_0_CONTROL			64
#define USB_PKT_MAXLEN_0_BULK				512
#define USB_PKT_MAXLEN_0_INT				1024

#define USB_PKT_MAXLEN_0_ISO				1024
#define USB_ISO_SUPPORTED_0					0

#else
#error "usb_ctrlr: this part has no USB controller"
#endif

/** @} End of group USB */

#endif	// __USB_CTRLR_H__
