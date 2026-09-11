/**-------------------------------------------------------------------------
@file	board.h

@brief	IBK-NRF52840 pin definitions for the USB HID keyboard demo

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

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
#ifndef __USB_HID_KEYBOARD_BOARD_H__
#define __USB_HID_KEYBOARD_BOARD_H__

#include "blyst840_boards.h"

#define HID_BUTTON_PORT		IBK_NRF52840_BUT1_PORT
#define HID_BUTTON_PIN		IBK_NRF52840_BUT1_PIN
#define HID_BUTTON_PINOP	IBK_NRF52840_BUT1_PINOP

#define HID_CAPS_BUTTON_PORT		IBK_NRF52840_BUT2_PORT
#define HID_CAPS_BUTTON_PIN		IBK_NRF52840_BUT2_PIN
#define HID_CAPS_BUTTON_PINOP	IBK_NRF52840_BUT2_PINOP

#define HID_LED_PORT			IBK_NRF52840_LED1_PORT
#define HID_LED_PIN			IBK_NRF52840_LED1_PIN
#define HID_LED_PINOP		IBK_NRF52840_LED1_PINOP

#endif // __USB_HID_KEYBOARD_BOARD_H__
