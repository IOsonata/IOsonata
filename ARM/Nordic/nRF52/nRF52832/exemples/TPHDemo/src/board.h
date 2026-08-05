/**-------------------------------------------------------------------------
@file	board.h

@brief	Contains I/O pin defines for board application specific
		This file is included with each project.  To be modified to map
		for the board used.

@author Hoang Nguyen Hoan
@date	May 15, 2016

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

// LED Pin
#define LED_PIN                 31
#define LED_PORT                0

// Button Pin
#define BUTTON_PIN              3
#define BUTTON_PORT             0

// I2C Pins for BME680
#define I2C_SDA_PORT            0
#define I2C_SDA_PIN             23

#define I2C_SCL_PORT            0
#define I2C_SCL_PIN             24

// BME680 Interrupt Pin (optional)
#define BME680_INT_PORT         0
#define BME680_INT_PIN          25

// UART Pins for Debug Output
#define UART_RX_PORT            0
#define UART_RX_PIN             8

#define UART_TX_PORT            0
#define UART_TX_PIN             6

#endif // __BOARD_H__

