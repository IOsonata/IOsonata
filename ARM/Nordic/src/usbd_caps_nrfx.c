/**-------------------------------------------------------------------------
@file	usbd_caps_nrfx.c

@brief	USB device controller capabilities for Nordic parts.

Reports the resources of the controller instantiated by the Nordic MDK. This
is consumed by the common IOsonata USB device stack; no USB class or external
USB stack is involved.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#include "nrf.h"
#include "nrf_peripherals.h"

#include "usb/usbd.h"

#if defined(USBD_PRESENT)

static const UsbdCaps_t s_UsbdCaps = {
	.MaxSpeed = USBD_SPEED_FULL,
	.EpInCnt = 8U,
	.EpOutCnt = 8U,
	.Ep0Mps = 64U,
};

#elif defined(USBHS_PRESENT)

static const UsbdCaps_t s_UsbdCaps = {
	.MaxSpeed = USBD_SPEED_HIGH,
	.EpInCnt = 16U,
	.EpOutCnt = 16U,
	.Ep0Mps = 64U,
};

#else
#error "usbd_caps_nrfx: this part has no USB device controller"
#endif

const UsbdCaps_t *UsbdGetCaps(void)
{
	return &s_UsbdCaps;
}
