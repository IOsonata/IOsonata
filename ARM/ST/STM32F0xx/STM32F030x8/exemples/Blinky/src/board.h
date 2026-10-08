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

// STM32F0308-DISCO (UM1658): B1 user button on PA0, active high.
// Use EXTI0 so the shared example does not use its active-low polling path.
#define BUT1_PORT		0
#define BUT1_PIN		0
#define BUT1_PINOP		0
#define BUT1_SENSE		IOPINSENSE_HIGH_TRANSITION
#define BUT1_INT		0
#define BUT1_INT_PRIO	2

// LD3 green on PC9 and LD4 blue on PC8, both active high.
#define LED1_PORT		2
#define LED1_PIN		9
#define LED1_PINOP		0

#define LED2_PORT		2
#define LED2_PIN		8
#define LED2_PINOP		0

#define BUTTON_PINS_MAP		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Full STM32F030R8 LQFP64 GPIO test: PA0-15, PB0-15, PC0-15, PD2,
// PF0/1/4/5/6/7 (55 pins). NRST, BOOT0 and supply pins are not GPIO.
// This also drives SWD PA13/PA14 and oscillator pins: run standalone with
// conflicting ST-LINK/oscillator connections isolated and an internal clock.
#define PULSE_TRAIN_PINS_MAP { \
	{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, \
	{0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, \
	{0, 8, 0}, {0, 9, 0}, {0, 10, 0}, {0, 11, 0}, \
	{0, 12, 0}, {0, 13, 0}, {0, 14, 0}, {0, 15, 0}, \
	{1, 0, 0}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, \
	{1, 4, 0}, {1, 5, 0}, {1, 6, 0}, {1, 7, 0}, \
	{1, 8, 0}, {1, 9, 0}, {1, 10, 0}, {1, 11, 0}, \
	{1, 12, 0}, {1, 13, 0}, {1, 14, 0}, {1, 15, 0}, \
	{2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, \
	{2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, \
	{2, 8, 0}, {2, 9, 0}, {2, 10, 0}, {2, 11, 0}, \
	{2, 12, 0}, {2, 13, 0}, {2, 14, 0}, {2, 15, 0}, \
	{3, 2, 0}, {5, 0, 0}, {5, 1, 0}, {5, 4, 0}, \
	{5, 5, 0}, {5, 6, 0}, {5, 7, 0}, \
}

#endif // __BOARD_H__
