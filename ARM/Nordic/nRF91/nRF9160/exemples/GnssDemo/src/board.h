/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, GNSS demo on the nRF9160 DK

The console is the nRF9160 UART0 on the interface MCU VCOM port.

@author	Hoang Nguyen Hoan
@date	Oct. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "modem_ipc_nrf91.h"
#include "gnss_nrf91.h"

// Console, UART0 to the interface MCU (VCOM0)
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			28
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			29
#define UART_TX_PINOP		1

// Receiver: the GNSS of the modem, on the IPC interface of the modem
#define GNSS_RECEIVER		GnssNrf91
#define GNSS_INTRF			ModemIpcIntrf
#define GNSS_INTRF_CFG_T	ModemIpcIntrfCfg_t

// Configuration of the modem interface: Tmr the time base of the modem waits,
// Trig its trigger kept for them, Handler the interface event handler
#define GNSS_INTRF_CFG(Tmr, Trig, Handler)	{ \
	.IntPrio = 6, \
	.pTimer = Tmr, \
	.TimerTrigNo = Trig, \
	.EvtCB = Handler, \
}

// Timer of the time stamps and of the modem waits: RTC0, RTC1 is the timer of
// the Bluetooth port (bt_app_nrf91.cpp)
#define GNSS_TIMER_DEVNO	0

// Receiver commands of the DK: MAGPIO sets the antenna tuning for the GNSS
// band, COEX0 turns the LNA of the onboard GNSS antenna on while GNSS runs
#define GNSS_CMD_LIST		{ \
	"AT%XMAGPIO=1,0,0,1,1,1574,1577", \
	"AT%XCOEX0=1,1,1565,1586" \
}

#endif
