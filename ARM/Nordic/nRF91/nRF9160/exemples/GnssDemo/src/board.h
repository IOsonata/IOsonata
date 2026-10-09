/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, GNSS demo on the nRF9160 DK
or the Nordic Thingy:91

The console is the nRF9160 UART0, the SysLog output: on the nRF9160 DK the
interface MCU VCOM0 port, on the Thingy:91 the first USB serial port of its
nRF52840. Select the board below.

GNSS needs modem firmware 1.3.4 or newer, which revision 2 of the nRF9160
runs. Revision 1 runs at most firmware 1.2.8 and has no GNSS: GnssDemo then
prints "GNSS init failed".

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

// Board selection, define one:
//	NRF9160_DK		nRF9160 DK, console on the interface MCU VCOM0
//	NORDIC_THINGY91	Nordic Thingy:91, console on the first USB serial port of
//					its nRF52840 running the Connectivity Bridge firmware
#define NRF9160_DK
//#define NORDIC_THINGY91

// Console, UART0, the SysLog output
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PINOP		1

#if defined(NORDIC_THINGY91)
// P0.19 and P0.18, to UART0 of the nRF52840
#define UART_RX_PIN			19
#define UART_TX_PIN			18
#elif defined(NRF9160_DK)
#define UART_RX_PIN			28
#define UART_TX_PIN			29
#else
#error "Select the board: NRF9160_DK or NORDIC_THINGY91"
#endif

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

// Receiver commands of the board: MAGPIO sets the antenna tuning, COEX0 turns
// the LNA of the onboard GNSS antenna on while GNSS runs. The Thingy:91 MAGPIO
// also tunes its antenna for the LTE bands, as Nordic sets it for this board.
#if defined(NORDIC_THINGY91)
#define GNSS_CMD_LIST		{ \
	"AT%XMAGPIO=1,1,1,7,1,746,803,2,698,748,2,1710,2200,3,824,894,4,880,960,5,791,849,7,1565,1586", \
	"AT%XCOEX0=1,1,1565,1586" \
}
#else
#define GNSS_CMD_LIST		{ \
	"AT%XMAGPIO=1,0,0,1,1,1574,1577", \
	"AT%XCOEX0=1,1,1565,1586" \
}
#endif

#endif
