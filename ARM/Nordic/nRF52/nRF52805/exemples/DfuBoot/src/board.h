/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

Stage 0 DFU boot: the UART the boot serves recovery on. IBK-BLYST-NANO
pins by default; change them for the board.

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

#include "blystnano_boards.h"

#define UART_DEVNO			0

#define UART_RX_PORT		IBK_BLYST_NANO_UART_RX_PORT
#define UART_RX_PIN			IBK_BLYST_NANO_UART_RX_PIN
#define UART_RX_PINOP		IBK_BLYST_NANO_UART_RX_PINOP
#define UART_TX_PORT		IBK_BLYST_NANO_UART_TX_PORT
#define UART_TX_PIN			IBK_BLYST_NANO_UART_TX_PIN
#define UART_TX_PINOP		IBK_BLYST_NANO_UART_TX_PINOP

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif // __BOARD_H__
