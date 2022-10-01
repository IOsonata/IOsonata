/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Aug. 31, 2014
@author	Thinh Tran
@date	Mar. 23, 2022

@license

Copyright (c) 2014-2022, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

/* SAM4LCxC High-freq and Low-freq clock sources */
#define MCUOSC 			{{OSC_TYPE_XTAL, 12000000, 20}, {OSC_TYPE_RC, 32000, 20}, false} //External Crystal 12 MHz
//#define MCUOSC		{OSC_TYPE_RC, 1000000, OSC_TYPE_RC, 32000} // Internal RC1M 1MHz
//#define MCUOSC		{OSC_TYPE_RC, 4000000, OSC_TYPE_RC, 32000} // Internal RCFAST 4/8/12 MHz
//#define MCUOSC 			{OSC_TYPE_RC, 80000000, OSC_TYPE_RC, 32000}// Internal RC80M --> max 40MHz CPU clock



// USER LED
#define LED1_PORT		IOPORTC
#define LED1_PIN		7
#define LED1_PINOP		IOPINOP_GPIO

// Button
#define BUT1_PORT		IOPORTC
#define BUT1_PIN		24
#define BUT1_PINOP		IOPINOP_GPIO
#define BUT1_SENSE_INT	0
#define BUT1_INT_PRIO	6
#define BUT1_SENSE		IOPINSENSE_LOW_TRANSITION

#define BUTTON_PINS_MAP { \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP { \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define PULSE_TRAIN_PINS_MAP	{ \
	{0, 4, 0}, {0, 6, 0}, {0, 7, 0}, \
	{1, 0, 0}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {1, 6, 0}, {1, 7, 0}, \
	{2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, \
	{3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, \
}

//#define BOARD_OSC			{ OSC_TYPE_XTAL, 16000000, OSC_TYPE_RC,	32000 }

#endif // __BOARD_H__

