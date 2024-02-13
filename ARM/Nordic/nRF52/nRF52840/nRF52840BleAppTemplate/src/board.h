/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Thinh Tran
@date	Feb. 13, 2024

@license

Copyright (c) 2024, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : info at i-syst dot com

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

#define LED_PINS_MAP		IBK_NRF52840_BUT_PINS_CFG

#elif BOARD == UDG_NRF52840

#define BUTTON_PINS_MAP		UDG_NRF52840_BUT_PINS_CFG
#define BUT1_SENSE			UDG_NRF52840_BUT1_SENSE

#define LED_PINS_MAP		UDG_NRF52840_LED_PINS_CFG

#endif

#define BUT1_INT		0
#define BUT1_INT_PRIO	6
#define BUT2_INT		1
#define BUT2_INT_PRIO	6

#define PULSE_TRAIN_PINS_MAP	BLYST840_PULSE_TRAIN_PINS

#define TIMER_DEVNO		2

#endif // __BOARD_H__

