/**-------------------------------------------------------------------------
@example	netcore_start.cpp

@brief	Application core image that only starts the nRF5340 network core.

After every reset the network core is held off until the application core
releases it. A network core image flashed alone therefore stops running at the
next reset or power cycle. Flash this image on the application core when the
network core runs by itself (BleAdvertiser, HCI controller on UART, ...).

The board LED turns on once the network core is released, so a dark LED
means this image is not running.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include "nrf.h"

#include "coredev/iopincfg.h"
#include "iopinctrl.h"

// Board LED and network-core console pins
#include "board.h"

int main()
{
	NRF_GPIO_Type *gpio = nRFGpioGetReg(NET_UART_PORT);
	uint32_t pins = (1UL << NET_UART_RX_PIN) | (1UL << NET_UART_TX_PIN);

	// The network core cannot take pins from the application core. Grant
	// access before releasing it; its UART driver configures direction and
	// drive strength while preserving MCUSEL.
#if !defined(NRF_TRUSTZONE_NONSECURE)
	NRF_SPU_S->GPIOPORT[NET_UART_PORT].PERM &= ~pins;
#else
	(void)pins;
#endif
	gpio->PIN_CNF[NET_UART_RX_PIN] =
		(gpio->PIN_CNF[NET_UART_RX_PIN] & ~GPIO_PIN_CNF_MCUSEL_Msk) |
		(GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos);
	gpio->PIN_CNF[NET_UART_TX_PIN] =
		(gpio->PIN_CNF[NET_UART_TX_PIN] & ~GPIO_PIN_CNF_MCUSEL_Msk) |
		(GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos);
	__DSB();

#if defined(NRF_RESET_S) && !defined(NRF_TRUSTZONE_NONSECURE)
	NRF_RESET_S->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
#else
	NRF_RESET_NS->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
#endif

	IOPinConfig(LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
#if LED1_ACTIVE_LOW
	IOPinClear(LED1_PORT, LED1_PIN);
#else
	IOPinSet(LED1_PORT, LED1_PIN);
#endif

	while (1)
	{
		__WFI();
	}

	return 0;
}
