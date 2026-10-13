/* LPC1769 USB preparation regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "LPC17xx.h"
#include "usb_hw_lpc17xx.h"

uint32_t SystemCoreClock = 120000000U;
static uint32_t s_Mask;

uint32_t TestGetPrimask(void)
{
	return s_Mask;
}
void TestDisableIRQ(void)
{
	s_Mask = 1;
}
void TestSetPrimask(uint32_t Mask)
{
	s_Mask = Mask;
}
static void SetReadOnly(volatile const uint32_t *pReg, uint32_t Value)
{
	*(volatile uint32_t *)pReg = Value;
}
int main(void)
{
	assert(mmap((void *)0x400FC000, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)0x5000C000, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	LPC_SC->SCS = 0x60;
	LPC_SC->CLKSRCSEL = 1;
	LPC_SC->USBCLKCFG = 9;
	SetReadOnly(&LPC_SC->PLL0STAT, (7U << 24) | 19U);
	LPC_SC->PCONP = 8;
	assert(!Lpc17UsbHwPrepare(12000000)); // clocks never become ready
	assert(LPC_SC->PCONP == 8 && LPC_USB->USBClkCtrl == 0);
	SetReadOnly(&LPC_USB->USBClkSt, 0x12);
	assert(Lpc17UsbHwPrepare(12000000));
	assert(s_Mask == 0 && LPC_USB->USBClkCtrl == 0x12);
	assert(!Lpc17UsbHwPrepare(12000000));
	LPC_USB->USBReEp = 1;
	assert(!Lpc17UsbHwRelease());
	LPC_USB->USBReEp = 0;
	SetReadOnly(&LPC_USB->USBEpDMASt, 1);
	assert(!Lpc17UsbHwRelease());
	SetReadOnly(&LPC_USB->USBEpDMASt, 0);
	LPC_USB->USBDevIntEn = 1;
	assert(!Lpc17UsbHwRelease());
	LPC_USB->USBDevIntEn = 0;
	s_Mask = 1;
	assert(Lpc17UsbHwRelease() && s_Mask == 1);
	assert(LPC_SC->PCONP == 8 && Lpc17UsbHwRelease());
	LPC_SC->PCONP |= 1U << 31;
	assert(!Lpc17UsbHwPrepare(12000000));
	assert(LPC_SC->PCONP == ((1U << 31) | 8));
	LPC_SC->PCONP = 8;
	SystemCoreClock = 17999999;
	assert(!Lpc17UsbHwPrepare(12000000));
	SystemCoreClock = 18000000;
	LPC_SC->USBCLKCFG = 8;
	assert(!Lpc17UsbHwPrepare(12000000));
	LPC_SC->USBCLKCFG = 9;
	LPC_SC->CLKSRCSEL = 0;
	assert(!Lpc17UsbHwPrepare(12000000));
	SetReadOnly(&LPC_SC->PLL1STAT, (7U << 8) | (1U << 5) | 3U);
	assert(Lpc17UsbHwPrepare(12000000)); // PLL1 independent of PLL0 source
	assert(Lpc17UsbHwRelease());
	assert(!Lpc17UsbHwPrepare(16000000));
	SetReadOnly(&LPC_SC->PLL1STAT, (3U << 8) | (1U << 5) | 3U);
	assert(!Lpc17UsbHwPrepare(12000000)); // connected but unlocked
	SetReadOnly(&LPC_SC->PLL1STAT, (7U << 8) | 3U);
	assert(!Lpc17UsbHwPrepare(12000000)); // CCO too low
	LPC_SC->SCS = 0;
	assert(!Lpc17UsbHwPrepare(12000000));
	assert(LPC_SC->PCONP == 8 && s_Mask == 1);
	puts("LPC1769 USB PLL validation, power ownership and clock timeout: PASS");
	return 0;
}
