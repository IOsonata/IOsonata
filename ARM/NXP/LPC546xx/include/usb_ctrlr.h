/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	LPC546xx USB0 and USB1 USB device controller interface.

DevNo 0 is USB0, a full-speed device with EP0 and four independent IN/OUT
bulk or interrupt endpoint pairs. DevNo 1 is USB1, a high-speed device with
EP0 and five pairs; it runs at full speed behind a full-speed hub or host.
Both use the shared LPC IP3511 transfer implementation, one controller at a
time: the USB RAM and the USB clocks of the other controller must be free.
Remote wakeup uses clock-on suspend; the generic core checks host permission.
ISO and low-power suspend are not implemented. Configure
bLowPowerSuspend=false.

USB0 Init requires one VBUS entry: P0.22 FUNC7, input, no pull, normal type.
This route supports GPIO edge detection while USB clocks are stopped. USB1
by default takes no pin map: the controller stays on from UsbCtrlrStart and
reads VBUS on the USB1_VBUS pin; the first bus reset of the host is the
attachment, and a VBUS only supply (a charger) is not reported. As the
override, one VBUS entry on any GPIO input wired to the bus power keeps USB1
off until VBUS appears on that pin. USB1_VBUS must see the bus power in both
cases: the PHY connects its pull-up only with it. DM/DP and the pull-up of
both controllers are internal to their PHY. USB1 needs a 12 MHz system oscillator,
crystal or TCXO, given as the core oscillator in g_McuOsc: the USB PLL makes
the 48 MHz USB1 clock from it. USB1 handles the errata USB.2, USB.6 and USB.15
of ES_LPC546XX (usb_ep0_lpc_fs.h, usb_lpc546xx.c).

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

// USB0 full-speed controller: ten physical endpoints. USB1 high-speed
// controller: twelve. Counts include EP0; each endpoint number has
// independent IN and OUT.
enum {
	USB_CTRLR_CNT = 2,
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
	USB_HIGHSPEED_CAPABLE_1 = 1,
	USB_EPIN_CNT_1 = 6,
	USB_EPOUT_CNT_1 = 6,
	USB_CTRLR1_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR1_BULK_PKT_LEN_MAX = 512,
	USB_CTRLR1_INT_PKT_LEN_MAX = 512,
	USB_CTRLR1_ISO_PKT_LEN_MAX = 1023,
	USB_ISO_SUPPORTED_1 = 0,
	USB_ISO_EPIN_MASK_1 = 0,
	USB_ISO_EPOUT_MASK_1 = 0,
};

#define USB_EPIN_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPIN_CNT_0 : (CtrlrNo) == 1 ? USB_EPIN_CNT_1 : 0)
#define USB_EPOUT_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPOUT_CNT_0 : (CtrlrNo) == 1 ? USB_EPOUT_CNT_1 : 0)
#define USB_HIGHSPEED_CAPABLE(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_HIGHSPEED_CAPABLE_0 : (CtrlrNo) == 1 ? USB_HIGHSPEED_CAPABLE_1 : 0)
#define USB_ISO_SUPPORTED(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_SUPPORTED_0 : (CtrlrNo) == 1 ? USB_ISO_SUPPORTED_1 : 0)
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPIN_MASK_0 : \
	 (CtrlrNo) == 1 ? (uint32_t)USB_ISO_EPIN_MASK_1 : 0U)
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPOUT_MASK_0 : \
	 (CtrlrNo) == 1 ? (uint32_t)USB_ISO_EPOUT_MASK_1 : 0U)
#define USB_CTRLR_PKT_LEN_MAX(CtrlrNo, TransType) \
	((CtrlrNo) == 0 ? \
	 ((TransType) == CONTROL ? USB_CTRLR0_CONTROL_PKT_LEN_MAX : \
	  (TransType) == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : \
	  (TransType) == BULK ? USB_CTRLR0_BULK_PKT_LEN_MAX : \
	  (TransType) == INT ? USB_CTRLR0_INT_PKT_LEN_MAX : 0) : \
	 (CtrlrNo) == 1 ? \
	 ((TransType) == CONTROL ? USB_CTRLR1_CONTROL_PKT_LEN_MAX : \
	  (TransType) == ISO ? USB_CTRLR1_ISO_PKT_LEN_MAX : \
	  (TransType) == BULK ? USB_CTRLR1_BULK_PKT_LEN_MAX : \
	  (TransType) == INT ? USB_CTRLR1_INT_PKT_LEN_MAX : 0) : 0)

#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)


#endif
