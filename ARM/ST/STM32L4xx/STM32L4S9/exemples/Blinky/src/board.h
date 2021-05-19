/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Aug. 31, 2014

@license

Copyright (c) 2014, I-SYST inc., all rights reserved

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

// *** STM32L476G-DISCO
// Joy left
#define BUT1_PORT		IOPORTA
#define BUT1_PIN		1
#define BUT1_PINOP		0
#define BUT1_SENSE		IOPINSENSE_HIGH_TRANSITION
#define BUT1_SENSE_INT	BUT1_PIN
#define BUT1_INT_PRIO	6

// Joy center
#define BUT2_PORT		IOPORTA
#define BUT2_PIN		0
#define BUT2_PINOP		0
#define BUT2_SENSE		IOPINSENSE_HIGH_TRANSITION
#define BUT2_SENSE_INT	BUT2_PIN
#define BUT2_INT_PRIO	6

// Joy down
#define BUT3_PORT		IOPORTA
#define BUT3_PIN		5
#define BUT3_PINOP		0
#define BUT3_SENSE		IOPINSENSE_HIGH_TRANSITION
#define BUT3_SENSE_INT	BUT3_PIN
#define BUT3_INT_PRIO	6

// Joy up
#define BUT4_PORT		IOPORTA
#define BUT4_PIN		3
#define BUT4_PINOP		0
#define BUT4_SENSE		IOPINSENSE_HIGH_TRANSITION
#define BUT4_SENSE_INT	BUT4_PIN
#define BUT4_INT_PRIO	6

// Joy right
#define BUT5_PORT		IOPORTA
#define BUT5_PIN		2
#define BUT5_PINOP		0
#define BUT5_SENSE		IOPINSENSE_HIGH_TRANSITION
#define BUT5_SENSE_INT	BUT5_PIN
#define BUT5_INT_PRIO	6

// LED2
#define LED1_PORT		IOPORTB	// Port B
#define LED1_PIN		7
#define LED1_PINOP		0

// LED3
#define LED2_PORT		IOPORTB
#define LED2_PIN		14
#define LED2_PINOP		0

#define BUTTON_PINS_MAP		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
	{BUT3_PORT, BUT3_PIN, BUT3_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
	{BUT4_PORT, BUT4_PIN, BUT4_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
	{BUT5_PORT, BUT5_PIN, BUT5_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP { \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define PULSE_TRAIN_PINS_MAP	{ \
	{0, 4, 0}, {0, 6, 0}, {0, 7, 0}, \
	{1, 0, 0}, {1, 1, 0}, {1, 2, 0}, {1, 3, 0}, {1, 4, 0}, {1, 5, 0}, {1, 6, 0}, {1, 7, 0}, \
	{2, 0, 0}, {2, 1, 0}, {2, 2, 0}, {2, 3, 0}, {2, 4, 0}, {2, 5, 0}, {2, 6, 0}, {2, 7, 0}, \
	{3, 0, 0}, {3, 1, 0}, {3, 2, 0}, {3, 3, 0}, {3, 4, 0}, {3, 5, 0}, {3, 6, 0}, {3, 7, 0}, \
}

#endif // __BOARD_H__

