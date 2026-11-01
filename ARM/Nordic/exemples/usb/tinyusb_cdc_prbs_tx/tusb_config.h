/**-------------------------------------------------------------------------
@file	tusb_config.h

@brief	TinyUSB configuration for the CDC PRBS transmit benchmark.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

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

#define CFG_TUSB_MCU				OPT_MCU_NRF5X
#define CFG_TUSB_OS				OPT_OS_NONE
#define CFG_TUSB_DEBUG				0

#define CFG_TUSB_RHPORT0_MODE		(OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENABLED				1
#define CFG_TUH_ENABLED				0
#define CFG_TUD_MAX_SPEED			OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN			__attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE		64

#define CFG_TUD_CDC				1
#define CFG_TUD_MSC				0
#define CFG_TUD_HID				0
#define CFG_TUD_MIDI				0
#define CFG_TUD_VENDOR				0

#define CFG_TUD_CDC_NOTIFY			1
#define CFG_TUD_CDC_RX_BUFSIZE		256
#define CFG_TUD_CDC_TX_BUFSIZE		2048
#define CFG_TUD_CDC_RX_EPSIZE		64
#define CFG_TUD_CDC_TX_EPSIZE		64

#endif
