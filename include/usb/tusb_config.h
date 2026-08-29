/**-------------------------------------------------------------------------
@file	tusb_config.h

@brief	Device stack configuration.

The library default. Every value here is derived from the MCU part define the
project already sets, so an application does not have to configure the device
stack to get a working USB port.

An application that needs something else puts its own copy of this file in a
directory that comes earlier on the include path.

@author	Hoang Nguyen Hoan
@date	Aug. 28, 2026

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

//
// Which controller. Taken from the part define rather than asked of the
// application, the same way every other driver in the tree reads the MDK.
//
// OPT_MCU_NRF5X is the nRF52 USBD, a full speed controller with 8 CBI
// endpoints and one isochronous pair. OPT_MCU_NRF54 is the nRF54LM20 USBHS,
// a DesignWare core with 16 endpoints, running at high speed.
//
#ifndef CFG_TUSB_MCU
	#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20A_ENGA_XXAA) || \
		defined(NRF54LM20B_XXAA) || defined(NRF54LM20B_ENGA_XXAA)
		#define CFG_TUSB_MCU				OPT_MCU_NRF54
	#elif defined(NRF52840_XXAA) || defined(NRF52833_XXAA) || \
		defined(NRF52820_XXAA)
		#define CFG_TUSB_MCU				OPT_MCU_NRF5X
	#else
		#error "No USB device controller known for this part"
	#endif
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS							OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG						0
#endif

#define CFG_TUD_ENABLED						1
#define CFG_TUH_ENABLED						0

//
// Speed. The nRF54LM20 USBHS is a high speed controller and the descriptor
// layer sizes its bulk endpoints from this.
//
#ifndef CFG_TUD_MAX_SPEED
	#if CFG_TUSB_MCU == OPT_MCU_NRF54
		#define CFG_TUD_MAX_SPEED			OPT_MODE_HIGH_SPEED
	#else
		#define CFG_TUD_MAX_SPEED			OPT_MODE_FULL_SPEED
	#endif
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN					__attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE				64

//
// Two CDC functions by default : one for the application and one left for a
// log or a second stream. An application that wants only one still gets one
// interface descriptor, since the descriptor layer builds from
// UsbDevCfg_t.NbCdc and not from this number.
//
#ifndef CFG_TUD_CDC
#define CFG_TUD_CDC							2
#endif

#define CFG_TUD_MSC							0
#define CFG_TUD_HID							0
#define CFG_TUD_MIDI						0
#define CFG_TUD_VENDOR						0

#define CFG_TUD_CDC_NOTIFY					1

//
// Endpoint sizes follow the bus speed. A high speed bulk endpoint is 512
// octets and nothing else is legal. The ring buffers hold two packets, which
// is the smallest size that lets one packet be filled while another is on
// the bus.
//
#if CFG_TUD_MAX_SPEED == OPT_MODE_HIGH_SPEED
	#define CFG_TUD_CDC_RX_EPSIZE			512
	#define CFG_TUD_CDC_TX_EPSIZE			512
	#define CFG_TUD_CDC_RX_BUFSIZE			1024
	#define CFG_TUD_CDC_TX_BUFSIZE			1024
#else
	#define CFG_TUD_CDC_RX_EPSIZE			64
	#define CFG_TUD_CDC_TX_EPSIZE			64
	#define CFG_TUD_CDC_RX_BUFSIZE			512
	#define CFG_TUD_CDC_TX_BUFSIZE			512
#endif

#endif	// __TUSB_CONFIG_H__
