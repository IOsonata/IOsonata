/**-------------------------------------------------------------------------
@file	board.h

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

#include "blyst840_boards.h"

#define BOARD			IBK_NRF52840

#if BOARD == IBK_NRF52840

#define BUTTON_PINS_MAP		IBK_NRF52840_BUT_PINS_CFG
#define BUT1_SENSE			IBK_NRF52840_BUT1_SENSE
#define BUT2_SENSE			IBK_NRF52840_BUT2_SENSE

#define LED_PINS_MAP		IBK_NRF52840_LED_PINS_CFG

#elif BOARD == UDG_NRF52840

#define BUTTON_PINS_MAP		UDG_NRF52840_BUT_PINS_CFG
#define BUT1_SENSE			UDG_NRF52840_BUT1_SENSE

#define LED_PINS_MAP		UDG_NRF52840_LED_PINS_CFG

#elif BOARD == BLUEIO840

#define BUTTON_PINS_MAP		BLUEIO840_BUT_PINS_CFG
#define BUT1_SENSE			BLUEIO840_BUT1_SENSE
#define BUT2_SENSE			BLUEIO840_BUT2_SENSE

#define LED_PINS_MAP		BLUEIO840_LED_PINS_CFG

#elif BOARD == BLUEPYRO_CR24S_M4025

#define LED_PINS_MAP		BLUEPYRO_CR24S_M4025_LED_PINS_CFG

#define LED1_PORT			BLUEPYRO_CR24S_M4025_LED1_PORT
#define LED1_PIN			BLUEPYRO_CR24S_M4025_LED1_PIN
#define LED1_ACTIVE			BLUEPYRO_CR24S_M4025_LED1_ACTIVE
#define LED2_PORT			BLUEPYRO_CR24S_M4025_LED2R_PORT
#define LED2_PIN			BLUEPYRO_CR24S_M4025_LED2R_PIN
#define LED2_ACTIVE			BLUEPYRO_CR24S_M4025_LED2R_ACTIVE
#define LED3_PORT			BLUEPYRO_CR24S_M4025_LED2G_PORT
#define LED3_PIN			BLUEPYRO_CR24S_M4025_LED2G_PIN
#define LED3_ACTIVE			BLUEPYRO_CR24S_M4025_LED2G_ACTIVE
#define LED4_PORT			BLUEPYRO_CR24S_M4025_LED2B_PORT
#define LED4_PIN			BLUEPYRO_CR24S_M4025_LED2B_PIN
#define LED4_ACTIVE			BLUEPYRO_CR24S_M4025_LED2B_ACTIVE

#endif

#define BUT1_INT		0
#define BUT1_INT_PRIO	6
#define BUT2_INT		1
#define BUT2_INT_PRIO	6

#define PULSE_TRAIN_PINS_MAP	{ \
	{0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, \
	{0, 8, 0}, {0, 9, 0}, {0, 10, 0}, {0, 11, 0}, {0, 12, 0}, {0, 13, 0}, {0, 14, 0}, {0, 15, 0}, \
	{0, 16, 0}, {0, 17, 0}, {0, 18, 0}, {0, 19, 0}, {0, 20, 0}, {0, 21, 0}, {0, 22, 0}, {0, 23, 0}, \
	{0, 24, 0}, {0, 25, 0}, {0, 26, 0}, {0, 27, 0}, {0, 28, 0}, {0, 29, 0}, {0, 30, 0}, {0, 31, 0} \
}

#endif // __BOARD_H__

