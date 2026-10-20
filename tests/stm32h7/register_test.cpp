/**-------------------------------------------------------------------------
@file	register_test.cpp

@brief	STM32H753 register tests.

Runs under tests/stm32h7/run.py, not on a board.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "iopinctrl.h"
#include "usb/usb.h"

extern USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs(void);
extern IRQn_Type Stm32UsbOtgIrq(void);
extern uint16_t Stm32UsbOtgFifoWords(void);
extern bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *);
extern void Stm32UsbOtgReset(void);

// The emulator clears HSI48RDY when this flag is set.
extern "C" { volatile uint32_t UsbClockFailure; }

#define CHECK(Expr) do { if (!(Expr)) { return __LINE__; } } while (0)

extern "C" int main(void)
{
	CHECK(SystemCoreClock == 400000000UL);
	CHECK(SystemD2Clock == 200000000UL);
	for (int i = 0; i < 4; i++)
	{
		CHECK(SystemPeriphClockGet(i) == 100000000UL);
		CHECK(SystemPeriphClockSet(i, 24000000UL) == 25000000UL);
		CHECK(SystemPeriphClockSet(i, 0) == 12500000UL);
		CHECK(SystemPeriphClockSet(i, UINT32_MAX) == 100000000UL);
	}
	CHECK(SystemPeriphClockGet(-1) == 0);
	CHECK(SystemPeriphClockSet(4, 1) == 0);

	static const IOPinCfg_t pins[] = {
		{ IOPORTD, 9, IOPINOP_FUNC7, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
		{ IOPORTD, 8, IOPINOP_FUNC7, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
	};
	UARTCfg_t cfg = {};
	cfg.DevNo = 3;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.FlowControl = UART_FLWCTRL_NONE;
	cfg.IntPrio = 6;
	cfg.bIntMode = true;
	cfg.bFifoBlocking = true;
	static UARTDev_t uart;
	RCC->D2CCIP2R = RCC_D2CCIP2R_USART28SEL;
	CHECK(UARTInit(&uart, &cfg));
	CHECK(USART3->BRR == 100);
	CHECK((RCC->D2CCIP2R & RCC_D2CCIP2R_USART28SEL) == 0);
	CHECK(RCC->APB1LENR & RCC_APB1LENR_USART3EN);
	CHECK(RCC->AHB4ENR & RCC_AHB4ENR_GPIODEN);
	CHECK((GPIOD->AFR[1] & 0xFFU) == 0x77U);
	CHECK((GPIOD->MODER & (15U << 16)) == (10U << 16));
	CHECK(USART3->CR1 & USART_CR1_RXNEIE_RXFNEIE);
	cfg.bDMAMode = true;
	CHECK(UARTInit(&uart, &cfg) == false);

	TimerCfg_t tc = {};
	tc.DevNo = 8; // TIM2
	tc.Freq = 100000;
	tc.IntPrio = 6;
	tc.ClkSrc = TIMER_CLKSRC_DEFAULT;
	static TimerDev_t timer;
	CHECK(TimerGetLowFreqDevCount() == 0);
	CHECK(TimerGetHighFreqDevCount() == 14);
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 1999);
	// APB /1: HCLK is still half the CPU clock; do not double it.
	RCC->D2CFGR &= ~RCC_D2CFGR_D2PPRE1;
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 1999);
	// APB /8 with TIMPRE=1: kernel is four times PCLK, 100 MHz.
	RCC->D2CFGR |= 6U << RCC_D2CFGR_D2PPRE1_Pos;
	RCC->CFGR |= RCC_CFGR_TIMPRE;
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 999);
	// Every advertised TIM maps to the correct owned vector.
	for (int i = 0; i < TimerGetHighFreqDevCount(); i++)
	{
		tc.DevNo = i;
		CHECK(TimerInit(&timer, &tc));
	}
	// USB2 FS setup must preserve the system clock and unrelated selectors.
	const uint32_t pll = RCC->PLL1DIVR;
	const uint32_t selectors = RCC->D2CCIP2R;
	const uint32_t supply = PWR->CR3;
	UsbCtrlrCfg_t usb = {};
	usb.IntPrio = 6;
	// Model reset-enabled ULPI sleep gating and an unrelated clock bit.
	RCC->AHB1LPENR = RCC_AHB1LPENR_USB2OTGFSULPILPEN | RCC_AHB1LPENR_DMA1LPEN;
	const uint32_t sleep = RCC->AHB1LPENR;
	CHECK(Stm32UsbOtgPowerInit(&usb));
	CHECK(Stm32UsbOtgRegs() == USB2_OTG_FS);
	CHECK(Stm32UsbOtgIrq() == OTG_FS_IRQn);
	CHECK(Stm32UsbOtgFifoWords() == 1024);
	CHECK(USB_EPIN_CNT(0) == 9 && USB_EPOUT_CNT(0) == 9);
	CHECK(!USB_HIGHSPEED_CAPABLE(0));
	CHECK(RCC->PLL1DIVR == pll);
	CHECK((RCC->D2CCIP2R & ~RCC_D2CCIP2R_USBSEL) ==
		(selectors & ~RCC_D2CCIP2R_USBSEL));
	CHECK((RCC->D2CCIP2R & RCC_D2CCIP2R_USBSEL) == RCC_D2CCIP2R_USBSEL);
	CHECK((CRS->CFGR & CRS_CFGR_SYNCSRC) == CRS_CFGR_SYNCSRC);
	CHECK((CRS->CFGR & CRS_CFGR_RELOAD) == 47999U);
	CHECK((CRS->CR & (CRS_CR_CEN | CRS_CR_AUTOTRIMEN)) ==
		(CRS_CR_CEN | CRS_CR_AUTOTRIMEN));
	CHECK(PWR->CR3 == (supply | PWR_CR3_USB33DEN));
	CHECK(RCC->AHB1ENR & RCC_AHB1ENR_USB2OTGFSEN);
	CHECK(RCC->AHB1LPENR == ((sleep & ~RCC_AHB1LPENR_USB2OTGFSULPILPEN) |
		RCC_AHB1LPENR_USB2OTGFSLPEN));
	CHECK((GPIOA->AFR[1] & 0xFF000U) == 0xAA000U);
	CHECK((GPIOA->MODER & (3U << 18)) == 0); // PA9 input
	CHECK((GPIOA->MODER & (15U << 22)) == (10U << 22));
	const uint32_t reset = RCC->AHB1RSTR;
	Stm32UsbOtgReset();
	CHECK(RCC->AHB1RSTR == reset);
	UsbClockFailure = 1;
	CHECK(!Stm32UsbOtgPowerInit(&usb));
	CHECK(RCC->PLL1DIVR == pll);
	return 0;
}

