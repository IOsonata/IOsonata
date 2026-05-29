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

#include "nrf.h"

#define QTD
#ifdef QTD

#define LED1_PORT				1//0
#define LED1_PIN				7//25	// P0.25
#define LED1_PINOP				0

#define LED2_PORT				1//0
#define LED2_PIN				3//26	// P0.26
#define LED2_PINOP				0

#define LED3_PORT				1//0
#define LED3_PIN				2//27	// P0.27
#define LED3_PINOP				0

#define LED4_PORT				1//0
#define LED4_PIN				5//28	// P0.28
#define LED4_PINOP				0

#define LED_RED_PORT			1
#define LED_RED_PIN				1	// P1.01
#define LED_RED_PINOP			0
#define LED_GREEN_PORT			0
#define LED_GREEN_PIN			25	// P0.25
#define LED_GREEN_PINOP			0
#define LED_BLUE_PORT			1
#define LED_BLUE_PIN			4	// P1.04
#define LED_BLUE_PINOP			0

#define DISPLAY_POWER_PORT		0
#define DISPLAY_POWER_PIN		4	// P0.04
#define DISPLAY_POWER_PINOP		0

#define EINK_BSI_PORT			0
#define EINK_BSI_PIN			22
#define EINK_BSI_PINOP			0

#define EINK_BUSY_PORT			0
#define EINK_BUSY_PIN			19
#define EINK_BUSY_PINOP			0

#define EINK_RESET_PORT			1
#define EINK_RESET_PIN			0
#define EINK_RESET_PINOP		0

#define EINK_DC_PORT			0
#define EINK_DC_PIN				21
#define EINK_DC_PINOP			0

#define EINK_SCK_PORT			0
#define EINK_SCK_PIN			20
#define EINK_SCK_PINOP			0

#define EINK_SDA_PORT			0
#define EINK_SDA_PIN			17
#define EINK_SDA_PINOP			0

#define EINK_CS_PORT			0
#define EINK_CS_PIN				24
#define EINK_CS_PINOP			0
#else
#define EINK_BSI_PORT			0
#define EINK_BSI_PIN			22//6//21
#define EINK_BSI_PINOP			0

#define EINK_BUSY_PORT			0
#define EINK_BUSY_PIN			20//4//20
#define EINK_BUSY_PINOP			0

#define EINK_RESET_PORT			0
#define EINK_RESET_PIN			5//8
#define EINK_RESET_PINOP		0

#define EINK_DC_PORT			0
#define EINK_DC_PIN				2//7
#define EINK_DC_PINOP			0

#define EINK_SCK_PORT			0
#define EINK_SCK_PIN			13//31//13
#define EINK_SCK_PINOP			0

#define EINK_SDA_PORT			0
#define EINK_SDA_PIN			26//12
#define EINK_SDA_PINOP			0

#define EINK_CS_PORT			0
#define EINK_CS_PIN				19//3//19
#define EINK_CS_PINOP			0
#endif

#endif // __BOARD_H__

