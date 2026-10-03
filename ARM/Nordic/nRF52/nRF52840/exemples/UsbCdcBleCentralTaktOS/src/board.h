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

// IBK_NRF52840 board

#define LED1_PORT			IBK_NRF52840_LED1_PORT
#define LED1_PIN			IBK_NRF52840_LED1_PIN
#define LED1_PINOP			IBK_NRF52840_LED1_PINOP

#define LED2_PORT			IBK_NRF52840_LED2_PORT
#define LED2_PIN			IBK_NRF52840_LED2_PIN
#define LED2_PINOP			IBK_NRF52840_LED2_PINOP

#define LED3_PORT			IBK_NRF52840_LED3_PORT
#define LED3_PIN			IBK_NRF52840_LED3_PIN
#define LED3_PINOP			IBK_NRF52840_LED3_PINOP

#define CONNECT_LED_PORT	IBK_NRF52840_LED3_PORT
#define CONNECT_LED_PIN		IBK_NRF52840_LED3_PIN
#define CONNECT_LED_LOGIC	IBK_NRF52840_LED3_LOGIC

#define LED_PINS	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif
