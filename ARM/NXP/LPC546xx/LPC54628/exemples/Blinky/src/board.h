/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPCXpresso54628

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#ifndef __BOARD_H__
#define __BOARD_H__

// Button on P0_4 (BOARD_SW4 in the NXP SDK), active low. P0_5 and P0_6 are
// the other two buttons of the same row. Pin interrupt channel 0 can only
// select port 0 and 1 pins.
#define BUT1_PORT		0
#define BUT1_PIN		4
#define BUT1_PINOP		IOPINOP_GPIO
#define BUT1_SENSE		IOPINSENSE_LOW_TRANSITION
#define BUT1_INT		0
#define BUT1_INT_PRIO	2

// LED1 P3_14, LED2 P3_3 and LED3 P2_2, all active low.
#define LED1_PORT		3
#define LED1_PIN		14
#define LED1_PINOP		IOPINOP_GPIO

#define LED2_PORT		3
#define LED2_PIN		3
#define LED2_PINOP		IOPINOP_GPIO

#define LED3_PORT		2
#define LED3_PIN		2
#define LED3_PINOP		IOPINOP_GPIO

#define BUTTON_PINS_MAP		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif
