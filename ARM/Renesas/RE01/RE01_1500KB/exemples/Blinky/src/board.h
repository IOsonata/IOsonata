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

#define MCUOSC		{ OSC_TYPE_XTAL, 32000000, OSC_TYPE_XTAL, 32768, true }

// Button 1
#define BUT1_PORT		1
#define BUT1_PIN		0
#define BUT1_PINOP		0
#define BUT1_SENSE		IOPINSENSE_LOW_TRANSITION
#define BUT1_SENSE_INT	0
#define BUT1_INT_PRIO	6

// Button 2
#define BUT2_PORT		5
#define BUT2_PIN		8
#define BUT2_PINOP		0
#define BUT2_SENSE		IOPINSENSE_TOGGLE
#define BUT2_SENSE_INT	4
#define BUT2_INT_PRIO	6

#define BUT3_PORT		4
#define BUT3_PIN		10
#define BUT3_PINOP		0
#define BUT3_SENSE		IOPINSENSE_LOW_TRANSITION
#define BUT3_SENSE_INT	2
#define BUT3_INT_PRIO	6

// LED
#define LED0_PORT		0
#define LED0_PIN		9
#define LED0_PINOP		0

#define LED1_PORT		0
#define LED1_PIN		8
#define LED1_PINOP		0

#define LED2_PORT		0
#define LED2_PIN		7
#define LED2_PINOP		0

#define BUTTON_PINS_MAP		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BUT3_PORT, BUT3_PIN, BUT3_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP	{ \
	{LED0_PORT, LED0_PIN, LED0_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define PULSE_TRAIN_PINS_MAP	{ \
	{LED0_PORT, LED0_PIN, LED0_PINOP}, {LED1_PORT, LED1_PIN, LED1_PINOP}, {LED2_PORT, LED2_PIN, LED2_PINOP}, }

#endif // __BOARD_H__

