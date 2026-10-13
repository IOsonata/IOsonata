/**-------------------------------------------------------------------------
@file	usb_hw_lpc17xx.c

@brief	Private LPC17xx USB clock validation and exclusive power ownership.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include "LPC17xx.h"
#include "usb_hw_lpc17xx.h"

#define LPC17_USB_POWER (1UL << 31)
#define LPC17_USB_CLOCKS 0x12U
#define LPC17_USB_WAIT 100000

static bool s_Prepared;

static bool Lpc17UsbClockValid(uint32_t MainHz)
{
	if (MainHz < 1000000U || MainHz > 25000000U ||
		(LPC_SC->SCS & 0x60U) != 0x60U || SystemCoreClock < 18000000U)
		return false;
	const uint32_t pll1 = LPC_SC->PLL1STAT;
	if ((pll1 & (1U << 9)) != 0U)
	{
		if ((pll1 & (7U << 8)) != (7U << 8) || MainHz < 10000000U)
			return false;
		const uint64_t output = (uint64_t)MainHz * ((pll1 & 31U) + 1U);
		const uint64_t cco = output * 2U * (1U << ((pll1 >> 5) & 3U));
		return output == 48000000U && cco >= 156000000U && cco <= 320000000U;
	}
	const uint32_t pll0 = LPC_SC->PLL0STAT;
	const uint32_t divider = (LPC_SC->USBCLKCFG & 15U) + 1U;
	if ((LPC_SC->CLKSRCSEL & 3U) != 1U ||
		(pll0 & (5U << 24)) != (5U << 24) || (divider & 1U) != 0U)
		return false;
	const uint64_t numerator = (uint64_t)MainHz * 2U * ((pll0 & 0x7FFFU) + 1U);
	const uint32_t denominator = ((pll0 >> 16) & 0xFFU) + 1U;
	return numerator == (uint64_t)48000000U * denominator * divider &&
		numerator >= (uint64_t)275000000U * denominator &&
		numerator <= (uint64_t)550000000U * denominator;
}

bool Lpc17UsbHwPrepare(uint32_t MainHz)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool ready = false;
	if (!s_Prepared && (LPC_SC->PCONP & LPC17_USB_POWER) == 0U &&
		Lpc17UsbClockValid(MainHz))
	{
		LPC_SC->PCONP |= LPC17_USB_POWER;
		LPC_USB->USBClkCtrl = LPC17_USB_CLOCKS;
		int wait = LPC17_USB_WAIT;
		while ((LPC_USB->USBClkSt & LPC17_USB_CLOCKS) != LPC17_USB_CLOCKS && --wait > 0)
		{
		}
		ready = wait > 0;
		if (ready)
			s_Prepared = true;
		else
		{
			LPC_USB->USBClkCtrl = 0U;
			LPC_SC->PCONP &= ~LPC17_USB_POWER;
		}
	}
	__set_PRIMASK(state);
	return ready;
}

bool Lpc17UsbHwRelease(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool released = !s_Prepared;
	if (s_Prepared && LPC_USB->USBReEp == 0U && LPC_USB->USBEpDMASt == 0U &&
		LPC_USB->USBDevIntEn == 0U && LPC_USB->USBEpIntEn == 0U &&
		LPC_USB->USBDMAIntEn == 0U)
	{
		LPC_USB->USBClkCtrl = 0U;
		LPC_SC->PCONP &= ~LPC17_USB_POWER;
		s_Prepared = false;
		released = true;
	}
	__set_PRIMASK(state);
	return released;
}
