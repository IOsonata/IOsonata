/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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

// IBK-IDAP-M revision 0, schematic 03-2021-04 (2021-04-25).

// S2: active-low user button on P2.7. S1 is reset, not a GPIO button.
#define BUT1_PORT		0
#define BUT1_PIN		1
#define BUT1_PINOP		IOPINOP_GPIO
#define BUT1_SENSE		IOPINSENSE_LOW_TRANSITION
#define BUT1_SENSE_INT	0
#define BUT1_INT_PRIO	3

// LED1: green, active-low, also on P3.1.
#define LED1_PORT		0
#define LED1_PIN		23
#define LED1_PINOP		IOPINOP_GPIO

// LED2: red, active-low, also on P3.2.
#define LED2_PORT		0
#define LED2_PIN		16
#define LED2_PINOP		IOPINOP_GPIO

#define BUTTON_PINS_MAP		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Exposed P2/P3 GPIOs, excluding S2, reset, LEDs and the P1 SWD signals.
#define PULSE_TRAIN_PINS_MAP { \
	{1, 15, IOPINOP_GPIO}, {0, 17, IOPINOP_GPIO}, {0, 18, IOPINOP_GPIO}, \
	{0, 19, IOPINOP_GPIO}, {1, 19, IOPINOP_GPIO}, {0, 20, IOPINOP_GPIO}, \
	{0, 2, IOPINOP_GPIO}, {0, 4, IOPINOP_GPIO}, {0, 5, IOPINOP_GPIO}, \
	{0, 14, IOPINOP_GPIO}, {0, 13, IOPINOP_GPIO}, {0, 12, IOPINOP_GPIO}, \
	{0, 11, IOPINOP_GPIO}, {0, 22, IOPINOP_GPIO}, {0, 9, IOPINOP_GPIO}, \
	{0, 8, IOPINOP_GPIO}, {0, 7, IOPINOP_GPIO}, {0, 21, IOPINOP_GPIO}, \
}

#endif // __BOARD_H__


