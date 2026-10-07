/**-------------------------------------------------------------------------
@example	lte_modem_info.cpp

@brief	nRF91 modem bring up: start the modem and print what it reports.

Starts the Modem library with the IOsonata glue (modem_nrf91.h) and prints
the modem firmware version, the IMEI, the hardware version and the
functional mode on the console UART. It does not attach to a network.

The application runs non secure after the secure stage
(secure_boot_nrf91.cpp): link it with nrf91xx_xxaa_ns.ld, a non secure
configuration of the library and the Modem library, cellular variant:

	sdk-nrfxlib/nrf_modem/lib/cellular/<nrf9160 or nrf9120>/hard-float/libmodem.a

The board.h of the project gives the console UART pins.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#include <stdint.h>

#include "nrf.h"
#include "nrf_modem.h"
#include "nrf_modem_at.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "modem_nrf91.h"

#include "board.h"

// Longest AT response printed
#define LTE_MODEM_INFO_RESP_SIZE		256

// Trigger of the timer the modem waits use
#define LTE_MODEM_INFO_TIMER_TRIG		0

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

// Low frequency timer for the timeouts of the modem waits
static const TimerCfg_t s_TimerCfg = {
	.DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = 6,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static TimerDev_t s_TimerDev;

static const nRF91ModemCfg_t s_ModemCfg = {
	.Fw = NRF91_MODEM_FW_CELLULAR,
	.IntPrio = 6,
	.pTimer = &s_TimerDev,
	.TimerTrigNo = LTE_MODEM_INFO_TIMER_TRIG,
	.ShmTxSize = 0,
	.ShmRxSize = 0,
	.HeapSize = 0,
	.pShmTrace = nullptr,
	.ShmTraceSize = 0,
	.pLog = nullptr,
	.FaultHandler = nullptr,
	.DfuHandler = nullptr,
};

static UART s_Uart;
static char s_Resp[LTE_MODEM_INFO_RESP_SIZE];

static void LteModemInfoAt(const char *pCmd)
{
	int res = nrf_modem_at_cmd(s_Resp, sizeof(s_Resp), "%s", pCmd);

	if (res == 0)
	{
		s_Uart.printf("%s\r\n%s", pCmd, s_Resp);
	}
	else
	{
		s_Uart.printf("%s failed %d\r\n", pCmd, res);
	}
}

int main()
{
	s_Uart.Init(s_UartCfg);

	s_Uart.printf("LteModemInfo\r\n");

	if (!TimerInit(&s_TimerDev, &s_TimerCfg))
	{
		s_Uart.printf("Timer init failed\r\n");
	}

	int res = nRF91ModemInit(&s_ModemCfg);

	if (res != 0)
	{
		s_Uart.printf("Modem init failed %d, firmware update result 0x%x\r\n",
					  res, (unsigned)nRF91ModemDfuResult());
	}
	else
	{
		s_Uart.printf("Modem library %s\r\n", nrf_modem_build_version());

		LteModemInfoAt("AT+CGMR");			// Modem firmware version
		LteModemInfoAt("AT+CGSN");			// IMEI
		LteModemInfoAt("AT%HWVERSION");		// Hardware version
		LteModemInfoAt("AT+CFUN?");			// Functional mode
	}

	bool faultshown = false;

	while (1)
	{
		__WFE();

		struct nrf_modem_fault_info fault;

		if (!faultshown && nRF91ModemFault(&fault))
		{
			s_Uart.printf("Modem fault 0x%x at 0x%x\r\n",
						  (unsigned)fault.reason, (unsigned)fault.program_counter);
			faultshown = true;
		}
	}

	return 0;
}
