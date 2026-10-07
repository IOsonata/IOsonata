/**-------------------------------------------------------------------------
@file	tusb_config.h

@brief	TinyUSB configuration for the nRF54LM20 composite USB stress benchmark.

The USBHS controller runs at high speed with TinyUSB's DWC2 driver in its
default slave (FIFO) mode. The project links its own copy of dcd_dwc2.c,
which adds incomplete isochronous OUT handling.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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
#ifndef __TUSB_CONFIG_H__
#define __TUSB_CONFIG_H__

#define CFG_TUSB_MCU				OPT_MCU_NRF54
#define CFG_TUSB_OS					OPT_OS_NONE
#define CFG_TUSB_DEBUG				0

#define CFG_TUSB_RHPORT0_MODE		(OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#define CFG_TUD_ENABLED				1
#define CFG_TUH_ENABLED				0
#define CFG_TUD_MAX_SPEED			OPT_MODE_HIGH_SPEED

// TinyUSB's DWC2 port names the core NRF_USBHSCORE0 unless the board is the
// LM20A engineering sample. The LM20A and LM20B MDKs name it NRF_USBHSCORE.
#define NRF_USBHSCORE0				NRF_USBHSCORE

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN			__attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE		64

#define CFG_TUD_CDC					2
#define CFG_TUD_HID					1
#define CFG_TUD_VENDOR				1
#define CFG_TUD_MSC					0
#define CFG_TUD_MIDI				0

// CDC bulk endpoints follow the link speed: 512 bytes at high speed. The RX
// queue holds four high-speed packets, as in the IOsonata LM20 combo.
#define CFG_TUD_CDC_NOTIFY			1
#define CFG_TUD_CDC_RX_BUFSIZE		2048
#define CFG_TUD_CDC_TX_BUFSIZE		2048

#define CFG_TUD_HID_EP_BUFSIZE		64

// Raw interrupt interface. Alternate settings require direct/non-buffered mode.
#define CFG_TUD_VENDOR_RX_BUFSIZE	0
#define CFG_TUD_VENDOR_TX_BUFSIZE	0
#define CFG_TUD_VENDOR_ALT_SETTINGS	1
#define CFG_TUD_VENDOR_EP_INT_OUT	1
#define CFG_TUD_VENDOR_EP_INT_IN	1
#define CFG_TUD_VENDOR_EP_INT_OUT_BUFSIZE	64
#define CFG_TUD_VENDOR_EP_INT_IN_BUFSIZE	64

// EP8 ISO is handled by the application's class driver so the same endpoint
// address can change MPS across alt 1..6.
#define CFG_TUD_VENDOR_EP_ISO_OUT	0
#define CFG_TUD_VENDOR_EP_ISO_IN	0

#endif
