/**-------------------------------------------------------------------------
@file	usb_otg_target_stm32.cpp

@brief	STM32 OTG clock, PHY and interrupt selection.

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
#include "usb/usb.h"

#ifndef HSE_VALUE
#define HSE_VALUE 25000000U
#endif

extern void Stm32UsbOtgInterrupt(void);

#if defined(STM32WBA65xx) || defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32H753xx)
static bool ClockReady(volatile uint32_t *pReg, uint32_t Mask)
{
	for (unsigned i = 0; i < 2000000; i++)
	{
		if ((*pReg & Mask) == Mask)
		{
			return true;
		}
	}
	return false;
}

#endif

#if defined(STM32WBA65xx)

USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs(void)
{
	return USB_OTG_HS;
}

IRQn_Type Stm32UsbOtgIrq(void)
{
	return USB_OTG_HS_IRQn;
}

uint16_t Stm32UsbOtgFifoWords(void)
{
	return 1024;
}

void Stm32UsbOtgReset(void)
{
	RCC->AHB2RSTR |= RCC_AHB2RSTR_OTGRST;
	__DSB();
	RCC->AHB2RSTR &= ~RCC_AHB2RSTR_OTGRST;
	__DSB();
}

bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *pCfg)
{
	(void)pCfg;
	// WBA6 HSE is the 32 MHz radio crystal. Do not retune the system PLL.
	RCC->CR |= RCC_CR_HSEON;
	if (!ClockReady(&RCC->CR, RCC_CR_HSERDY))
	{
		return false;
	}
	RCC->AHB4ENR |= RCC_AHB4ENR_PWREN;
	RCC->APB7ENR |= RCC_APB7ENR_SYSCFGEN;
	(void)RCC->APB7ENR;
	RCC->CCIPR2 &= ~RCC_CCIPR2_OTGHSSEL;
	SYSCFG->OTGHSPHYCR = (SYSCFG->OTGHSPHYCR & ~SYSCFG_OTGHSPHYCR_CLKSEL) |
		SYSCFG_OTGHSPHYCR_CLKSEL_0 | SYSCFG_OTGHSPHYCR_CLKSEL_1 |
		SYSCFG_OTGHSPHYCR_CLKSEL_3;
	// ST CubeWBA 1.10.0 USBX MSP sequence. These operations must run even
	// when another peripheral has already enabled the PWR register clock.
	PWR->SVMCR |= PWR_SVMCR_USV;
	PWR->VOSR &= ~PWR_VOSR_VDD11USBDIS;
	PWR->VOSR |= PWR_VOSR_USBPWREN;
	PWR->VOSR |= PWR_VOSR_USBBOOSTEN;
	if (!ClockReady(&PWR->VOSR, PWR_VOSR_USBBOOSTRDY))
	{
		return false;
	}
	SYSCFG->OTGHSPHYCR |= SYSCFG_OTGHSPHYCR_EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGEN | RCC_AHB2ENR_OTGHSPHYEN;
	(void)RCC->AHB2ENR;
	// PD6/PD7 are dedicated PHY pads; no GPIO alternate-function setup.
	return true;
}

extern "C" void OTG_HS_IRQHandler(void)
{
	Stm32UsbOtgInterrupt();
	__DSB();
}

#elif defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32H753xx)

USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs(void)
{
	return USB_OTG_FS;
}

IRQn_Type Stm32UsbOtgIrq(void)
{
	return OTG_FS_IRQn;
}

uint16_t Stm32UsbOtgFifoWords(void)
{
#if defined(STM32H753xx)
	// RM0433: both OTG cores have 4 KB, including USB2 with the FS PHY.
	return 1024;
#else
	return 320;
#endif
}

void Stm32UsbOtgReset(void)
{
#if defined(STM32H753xx)
	RCC->AHB1RSTR |= RCC_AHB1RSTR_USB2OTGFSRST;
	__DSB();
	RCC->AHB1RSTR &= ~RCC_AHB1RSTR_USB2OTGFSRST;
#else
	RCC->AHB2RSTR |= RCC_AHB2RSTR_OTGFSRST;
	__DSB();
	RCC->AHB2RSTR &= ~RCC_AHB2RSTR_OTGFSRST;
#endif
	__DSB();
}

bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *pCfg)
{
	if (SystemCoreClockGet() < 14200000)
	{
		return false;
	}
#if defined(STM32H753xx)
	// Dedicated HSI48, synchronized to USB2 SOF, leaves the system PLL alone.
	RCC->CR |= RCC_CR_HSI48ON;
	if (!ClockReady(&RCC->CR, RCC_CR_HSI48RDY))
	{
		return false;
	}
	RCC->D2CCIP2R |= RCC_D2CCIP2R_USBSEL;
	RCC->APB1HENR |= RCC_APB1HENR_CRSEN;
	(void)RCC->APB1HENR;
	CRS->CR = 32U << CRS_CR_TRIM_Pos;
	CRS->CFGR = CRS_CFGR_SYNCSRC | (34U << CRS_CFGR_FELIM_Pos) | 47999U;
	CRS->ICR = CRS_ICR_SYNCOKC | CRS_ICR_SYNCWARNC | CRS_ICR_ERRC | CRS_ICR_ESYNCC;
	CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
	// VDD33USB must be supplied externally (NUCLEO-H753ZI SB13).
	// USB33RDY describes the internal regulator, which is not used here.
	PWR->CR3 |= PWR_CR3_USB33DEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
	(void)RCC->AHB1ENR;
#elif defined(STM32F401xC) || defined(STM32F401xE)
	// The F401 system port starts on HSI. An unused main PLL can supply
	// USB independently without switching SYSCLK. Board builds must provide
	// their real HSE_VALUE (the ST family default is 25 MHz).
	if ((RCC->CR & RCC_CR_PLLON) == 0)
	{
		if (HSE_VALUE % 1000000 != 0 || HSE_VALUE < 2000000 || HSE_VALUE > 50000000)
		{
			return false;
		}
		RCC->CR |= RCC_CR_HSEON;
		if (!ClockReady(&RCC->CR, RCC_CR_HSERDY))
		{
			return false;
		}
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (HSE_VALUE / 1000000) |
			(192U << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLP_0 |
			(4U << RCC_PLLCFGR_PLLQ_Pos);
		RCC->CR |= RCC_CR_PLLON;
	}
	if (!ClockReady(&RCC->CR, RCC_CR_PLLRDY))
	{
		return false;
	}
	const uint32_t pll = RCC->PLLCFGR;
	const uint32_t source = (pll & RCC_PLLCFGR_PLLSRC) != 0 ? HSE_VALUE : 16000000;
	const uint32_t m = pll & RCC_PLLCFGR_PLLM;
	const uint32_t n = (pll & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
	const uint32_t q = (pll & RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos;
	if (m == 0 || q == 0 || static_cast<uint64_t>(source) * n != 48000000ULL * m * q ||
		(pll & RCC_PLLCFGR_PLLSRC) == 0)
	{
		// HSI without calibration does not meet USB full-speed accuracy.
		return false;
	}
#else
	// The L476/L496 system port already provides MSI at 48 MHz. Lock it
	// to LSE for USB accuracy; never change the MSI range under a live PLL.
	const uint32_t range = (RCC->CR & RCC_CR_MSIRGSEL) != 0 ?
		(RCC->CR & RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos :
		(RCC->CSR & RCC_CSR_MSISRANGE) >> RCC_CSR_MSISRANGE_Pos;
	if (range != 11 || (RCC->CR & RCC_CR_MSIRDY) == 0 ||
		(RCC->BDCR & RCC_BDCR_LSERDY) == 0)
	{
		return false;
	}
	RCC->CR |= RCC_CR_MSIPLLEN;
	RCC->CCIPR |= RCC_CCIPR_CLK48SEL;
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	(void)RCC->APB1ENR1;
	PWR->CR2 |= PWR_CR2_USV;
#endif
#if !defined(STM32H753xx)
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	(void)RCC->AHB2ENR;
#endif
	// PA11/PA12 are the FS data pads on these MCUs. PA9 is VBUS sense.
	IOPinConfig(0, 9, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 11, IOPINOP_FUNC10, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 12, IOPINOP_FUNC10, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSetSpeed(0, 11, IOPINSPEED_TURBO);
	IOPinSetSpeed(0, 12, IOPINSPEED_TURBO);
	if (pCfg->pIOPinMap != nullptr && pCfg->NbIOPins > 0)
	{
		IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	}
	return true;
}

extern "C" void OTG_FS_IRQHandler(void)
{
	Stm32UsbOtgInterrupt();
	__DSB();
}

#else
#error "STM32 OTG clock and PHY setup is not implemented for this device"
#endif

