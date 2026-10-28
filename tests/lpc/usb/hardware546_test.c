/**-------------------------------------------------------------------------
@file hardware546_test.c
@brief LPC546xx USB hardware ownership and memory regression.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
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
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "LPC546xx.h"
#include "coredev/system_core_clock.h"
#include "usb_hw_lpc546xx.h"

// The controller a fixture runs: USB0 unless it says otherwise
#ifndef TEST_USB_IRQ
#define TEST_USB_IRQ USB0_IRQn
#endif
#define TEST_VD_STATUS (*(volatile uint32_t *)0x40020054UL)
#define TEST_VD3_READY (1U << 5)
#define TEST_VD5_READY (1U << 9)

uint32_t g_IrqMasked;
McuOsc_t g_McuOsc = {{OSC_TYPE_XTAL, 12000000, 20, 0}, {OSC_TYPE_RC, 32768, 20, 0}, true};
static uint32_t s_Priority;
static bool s_Enabled;
// NVIC state of the other controller
static uint32_t s_OtherPriority;
static bool s_OtherEnabled;
static bool s_DividerStuck;
static bool s_Divider1Stuck;
static bool s_PllStuck;
static unsigned s_Waits;
static unsigned s_FroCalls;
static bool s_FroFail;
static unsigned s_DelayUs;
static void (*s_DelayHook)(uint32_t Us);
static bool s_Usb1Reset;

void __DSB(void)
{
	if (SYSCON->PRESETCTRLSET[2] & SYSCON_PRESETCTRL_USB1D_RST_MASK)
		s_Usb1Reset = true;
	for (int i = 0; i < 3; i++)
	{
		SYSCON->AHBCLKCTRL[i] |= SYSCON->AHBCLKCTRLSET[i];
		SYSCON->AHBCLKCTRL[i] &= ~SYSCON->AHBCLKCTRLCLR[i];
		SYSCON->AHBCLKCTRLSET[i] = 0;
		SYSCON->AHBCLKCTRLCLR[i] = 0;
		SYSCON->PRESETCTRL[i] |= SYSCON->PRESETCTRLSET[i];
		SYSCON->PRESETCTRL[i] &= ~SYSCON->PRESETCTRLCLR[i];
		SYSCON->PRESETCTRLSET[i] = 0;
		SYSCON->PRESETCTRLCLR[i] = 0;
	}
	for (int i = 0; i < 2; i++)
	{
		SYSCON->PDRUNCFG[i] |= SYSCON->PDRUNCFGSET[i];
		SYSCON->PDRUNCFG[i] &= ~SYSCON->PDRUNCFGCLR[i];
		SYSCON->PDRUNCFGSET[i] = 0;
		SYSCON->PDRUNCFGCLR[i] = 0;
	}
}

// Model time: the USB PLL locks once powered, unless stuck
void ModelDelayUs(uint32_t Us)
{
	s_DelayUs += Us;
	if (!s_PllStuck && (SYSCON->PDRUNCFG[1] & SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK) == 0U)
		SYSCON->USBPLLSTAT = SYSCON_USBPLLSTAT_LOCK_MASK;
	if (s_DelayHook != NULL)
		s_DelayHook(Us);
}

void __DMB(void)
{
	assert((SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK) == 0);
	assert(SYSCON->AHBCLKCTRL[2] & SYSCON_AHBCLKCTRL_USB1RAM_MASK);
}

void __NOP(void)
{
	s_Waits++;
	assert(s_Waits <= 1024);
}

void TestUsbDivider(uint32_t Value)
{
	SYSCON->USB0CLKDIV = Value | (s_DividerStuck ? SYSCON_USB0CLKDIV_REQFLAG_MASK : 0);
}

void TestUsb1Divider(uint32_t Value)
{
	SYSCON->USB1CLKDIV = Value | (s_Divider1Stuck ? SYSCON_USB1CLKDIV_REQFLAG_MASK : 0);
}

void TestFroSetup(uint32_t Frequency)
{
	assert(g_IrqMasked == 1);
	assert(SYSCON->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_ROM_MASK);
	assert((SYSCON->PDRUNCFG[0] & (SYSCON_PDRUNCFG_PDEN_ROM_MASK |
		SYSCON_PDRUNCFG_PDEN_VD6_MASK | SYSCON_PDRUNCFG_PDEN_FRO_MASK)) == 0);
	assert(Frequency == 48000000U || Frequency == 96000000U);
	s_FroCalls++;
	if (!s_FroFail)
	{
		SYSCON->FROCTRL = SYSCON_FROCTRL_HSPDCLK_MASK | SYSCON_FROCTRL_WRTRIM_MASK |
			(Frequency == 96000000U ? SYSCON_FROCTRL_SEL_MASK : 0U) | 0x2345;
	}
}

static bool *TestIrqEnabled(IRQn_Type Irq)
{
	assert(Irq == USB0_IRQn || Irq == USB1_IRQn);
	return Irq == TEST_USB_IRQ ? &s_Enabled : &s_OtherEnabled;
}

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	*TestIrqEnabled(Irq) = false;
}

uint32_t NVIC_GetEnableIRQ(IRQn_Type Irq)
{
	return *TestIrqEnabled(Irq);
}

void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	(void)TestIrqEnabled(Irq);
}

void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	*(Irq == TEST_USB_IRQ ? &s_Priority : &s_OtherPriority) = Priority;
	(void)TestIrqEnabled(Irq);
}

static void Map(uintptr_t Address)
{
	void *p = mmap((void *)Address, Address == SYSCON_BASE ? 0x21000 : 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

	assert(p == (void *)Address);
}

static void Reset(void)
{
	memset(SYSCON, 0, sizeof(*SYSCON));
	memset(USB0, 0, sizeof(*USB0));
	memset(USBFSH, 0, sizeof(*USBFSH));
	memset(USBHSD, 0, sizeof(*USBHSD));
	memset(USBHSH, 0, sizeof(*USBHSH));
	SYSCON->FROCTRL = SYSCON_FROCTRL_HSPDCLK_MASK | 0x1234;
	SYSCON->USB0CLKSEL = 7;
	SYSCON->USB0CLKDIV = SYSCON_USB0CLKDIV_HALT_MASK;
	SYSCON->USB1CLKSEL = 7;
	SYSCON->USB1CLKDIV = SYSCON_USB1CLKDIV_HALT_MASK;
	SYSCON->USBPLLCTRL = 0x5555;
	// Core on the system PLL from the 12 MHz crystal: VD2_ANA, VD3 and the
	// system oscillator powered; USB1 PHY, USB PLL and VD5 off.
	SYSCON->PDRUNCFG[0] = SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_USB0_PHY_MASK |
		SYSCON_PDRUNCFG_PDEN_VD5_MASK;
	SYSCON->PDRUNCFG[1] = SYSCON_PDRUNCFG_PDEN_USB1_PHY_MASK | SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
	TEST_VD_STATUS = TEST_VD3_READY | TEST_VD5_READY;
	USBFSH->PORTMODE = USBFSH_PORTMODE_ID_EN_MASK;
	USBHSH->PORTMODE = USBHSH_PORTMODE_ID0_EN_MASK;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_XTAL;
	g_McuOsc.CoreOsc.Freq = 12000000;
	s_DividerStuck = false;
	s_Divider1Stuck = false;
	s_PllStuck = false;
	s_DelayUs = 0;
	s_DelayHook = NULL;
	s_Usb1Reset = false;
	s_Waits = 0;
	s_FroCalls = 0;
	s_FroFail = false;
	g_IrqMasked = 0;
}

#define TEST_USB_PLL 0xD3FU

// Nothing of USB1 changed by a refused preparation
static void HsUntouched(uint32_t Power0, uint32_t Power1, uint32_t Pll)
{
	assert(SYSCON->PDRUNCFG[0] == Power0 && SYSCON->PDRUNCFG[1] == Power1);
	assert(SYSCON->USBPLLCTRL == Pll);
	// A stuck divider model keeps its request flag
	assert(SYSCON->USB1CLKSEL == 7 &&
		(SYSCON->USB1CLKDIV & ~SYSCON_USB1CLKDIV_REQFLAG_MASK) == SYSCON_USB1CLKDIV_HALT_MASK);
	assert(SYSCON->AHBCLKCTRL[2] == 0);
	assert(USBHSH->PORTMODE == USBHSH_PORTMODE_ID0_EN_MASK);
	assert(g_IrqMasked == 0);
}

static void HsRefused(void)
{
	const uint32_t power0 = SYSCON->PDRUNCFG[0];
	const uint32_t power1 = SYSCON->PDRUNCFG[1];
	const uint32_t pll = SYSCON->USBPLLCTRL;

	assert(Lpc546xxUsbHsHwPrepare(6) == NULL);
	HsUntouched(power0, power1, pll);
}

// USB1: the USB PLL from the 12 MHz system oscillator, power, clocks,
// device mode, high-speed list and release
static void HsHardwareTest(void)
{
	Reset();
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	HsRefused();
	Reset();
	g_McuOsc.CoreOsc.Freq = 16000000;
	HsRefused();
	Reset();
	g_McuOsc.CoreOsc.Type = OSC_TYPE_TCXO;
	assert(Lpc546xxUsbHsHwPrepare(6) != NULL);
	assert(Lpc546xxUsbHsHwRelease());
	Reset();
	SYSCON->PDRUNCFG[1] |= SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
	HsRefused();
	Reset();
	SYSCON->PDRUNCFG[0] |= SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK;
	HsRefused();

	// One controller at a time, and no host controller
	Reset();
	assert(Lpc546xxUsbHwPrepare(6) != NULL);
	assert(Lpc546xxUsbHsHwPrepare(6) == NULL);
	assert(Lpc546xxUsbHwRelease());
	assert(Lpc546xxUsbHsHwPrepare(6) != NULL);
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	assert(Lpc546xxUsbHsHwRelease());
	Reset();
	SYSCON->AHBCLKCTRL[1] = SYSCON_AHBCLKCTRL_USB0D_MASK;
	assert(Lpc546xxUsbHsHwPrepare(6) == NULL);
	SYSCON->AHBCLKCTRL[1] = 0;
	uint32_t others[] = {SYSCON_AHBCLKCTRL_USB1D_MASK, SYSCON_AHBCLKCTRL_USB1H_MASK,
		SYSCON_AHBCLKCTRL_USB0HMR_MASK, SYSCON_AHBCLKCTRL_USB0HSL_MASK};
	for (unsigned i = 0; i < sizeof(others) / sizeof(others[0]); i++)
	{
		SYSCON->AHBCLKCTRL[2] = others[i];
		assert(Lpc546xxUsbHsHwPrepare(6) == NULL);
		assert(SYSCON->AHBCLKCTRL[2] == others[i]);
	}

	// Normal preparation and release
	Reset();
	const uint32_t power0 = SYSCON->PDRUNCFG[0];
	const uint32_t power1 = SYSCON->PDRUNCFG[1];
	const uint32_t control = SYSCON_USB1CLKCTRL_POL_FS_DEV_CLK_MASK | SYSCON_USB1CLKCTRL_HS_DEV_WAKEUP_N_MASK;
	SYSCON->USB1CLKCTRL = control;
	Lpc546xxUsbHsMemory_t *mem = Lpc546xxUsbHsHwPrepare(100);
	assert(mem != NULL && (uintptr_t)mem == 0x40100000U);
	assert(s_OtherPriority == (1U << __NVIC_PRIO_BITS) - 1U && !s_OtherEnabled);
	assert(USBHSD->INTEN == 0);
	assert((USBHSD->DEVCMDSTAT & (USBHSD_DEVCMDSTAT_DEV_EN_MASK | USBHSD_DEVCMDSTAT_DCON_MASK)) == 0);
	assert(USBHSD->DEVCMDSTAT & USBHSD_DEVCMDSTAT_FORCE_NEEDCLK_MASK);
	assert(SYSCON->USBPLLCTRL == TEST_USB_PLL && s_DelayUs >= 72U);
	assert((SYSCON->PDRUNCFG[1] & (SYSCON_PDRUNCFG_PDEN_USB1_PHY_MASK | SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK)) == 0);
	assert((SYSCON->PDRUNCFG[0] & (SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_VD5_MASK |
		SYSCON_PDRUNCFG_PDEN_VD3_MASK)) == 0);
	assert(SYSCON->USB1CLKSEL == 2 && SYSCON->USB1CLKDIV == 0);
	assert(SYSCON->USB1CLKCTRL == (SYSCON_USB1CLKCTRL_AP_FS_DEV_CLK_MASK | SYSCON_USB1CLKCTRL_HS_DEV_WAKEUP_N_MASK));
	assert(SYSCON->AHBCLKCTRL[2] == (SYSCON_AHBCLKCTRL_USB1D_MASK | SYSCON_AHBCLKCTRL_USB1RAM_MASK));
	assert(s_Usb1Reset && (SYSCON->PRESETCTRL[2] & (SYSCON_PRESETCTRL_USB1D_RST_MASK |
		SYSCON_PRESETCTRL_USB1RAM_RST_MASK)) == 0);
	assert(USBHSH->PORTMODE == USBHSH_PORTMODE_DEV_ENABLE_MASK);
	assert(USBHSD->EPLISTSTART == (uintptr_t)mem && USBHSD->DATABUFSTART == 0x40100000U);
	for (unsigned i = 0; i < 12; i++)
	{
		const uintptr_t buffer = (uintptr_t)(i < 2 ? mem->Packet0[i] : mem->Packet[i - 2]);
		assert((buffer & 63U) == 0);
		assert(mem->Command[i * 2] == (((buffer >> 6) & 0x7FFU) | (1U << 30)));
		if (i > 1)
			assert(mem->Command[i * 2 + 1] == (1U << 30));
	}
	assert(mem->Command[1] == (((uintptr_t)mem->Setup >> 6) & 0x7FFU) && mem->Command[3] == 0);
	assert(Lpc546xxUsbHsHwPrepare(6) == NULL);
	USBHSD->DEVCMDSTAT |= USBHSD_DEVCMDSTAT_DEV_EN_MASK;
	assert(!Lpc546xxUsbHsHwRelease());
	USBHSD->DEVCMDSTAT &= ~USBHSD_DEVCMDSTAT_DEV_EN_MASK;
	USBHSD->EPSKIP = 1U << 11;
	assert(!Lpc546xxUsbHsHwRelease());
	USBHSD->EPSKIP = 0;
	mem->Command[22] |= 1U << 31;
	assert(!Lpc546xxUsbHsHwRelease());
	mem->Command[22] &= ~(1U << 31);
	assert(Lpc546xxUsbHsHwRelease());
	assert(SYSCON->PDRUNCFG[0] == power0 && SYSCON->PDRUNCFG[1] == power1);
	assert(SYSCON->USBPLLCTRL == 0x5555);
	assert(SYSCON->USB1CLKSEL == 7 && SYSCON->USB1CLKDIV == SYSCON_USB1CLKDIV_HALT_MASK);
	assert(SYSCON->USB1CLKCTRL == control);
	assert(SYSCON->AHBCLKCTRL[2] == 0);
	assert(USBHSH->PORTMODE == USBHSH_PORTMODE_ID0_EN_MASK);
	assert(Lpc546xxUsbHsHwRelease());
	assert(g_IrqMasked == 0);

	// The PLL supply is powered and stays powered: the other PLLs use it
	Reset();
	SYSCON->PDRUNCFG[0] |= SYSCON_PDRUNCFG_PDEN_VD3_MASK;
	SYSCON->AHBCLKCTRL[2] = SYSCON_AHBCLKCTRL_USB1RAM_MASK;
	assert(Lpc546xxUsbHsHwPrepare(-1) != NULL);
	assert(s_OtherPriority == 0);
	assert(Lpc546xxUsbHsHwRelease());
	assert((SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_VD3_MASK) == 0);
	assert(SYSCON->AHBCLKCTRL[2] == SYSCON_AHBCLKCTRL_USB1RAM_MASK);

	// A PLL already at 48 MHz is used as it is and left running
	Reset();
	SYSCON->PDRUNCFG[1] &= ~SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
	SYSCON->USBPLLCTRL = TEST_USB_PLL;
	SYSCON->USBPLLSTAT = SYSCON_USBPLLSTAT_LOCK_MASK;
	assert(Lpc546xxUsbHsHwPrepare(6) != NULL);
	assert(s_DelayUs == 0);
	assert(Lpc546xxUsbHsHwRelease());
	assert((SYSCON->PDRUNCFG[1] & SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK) == 0);
	assert(SYSCON->USBPLLCTRL == TEST_USB_PLL);

	// A PLL at another rate, or not locked, is refused
	Reset();
	SYSCON->PDRUNCFG[1] &= ~SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
	SYSCON->USBPLLCTRL = 0x1234;
	SYSCON->USBPLLSTAT = SYSCON_USBPLLSTAT_LOCK_MASK;
	HsRefused();
	Reset();
	SYSCON->PDRUNCFG[1] &= ~SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
	SYSCON->USBPLLCTRL = TEST_USB_PLL;
	HsRefused();

	// The PLL does not lock: bounded wait, then everything restored
	Reset();
	s_PllStuck = true;
	HsRefused();
	assert(s_DelayUs == 72U + 100U * 10U);

	// VD5 or VD3 not ready
	Reset();
	TEST_VD_STATUS = TEST_VD3_READY;
	HsRefused();
	Reset();
	TEST_VD_STATUS = TEST_VD5_READY;
	HsRefused();

	// The USB1 clock divider does not take the value
	Reset();
	s_Divider1Stuck = true;
	HsRefused();
	assert(s_Waits == 1023);
}

static void MapAll(void)
{
	Map(SYSCON_BASE);
	Map(USB0_BASE);
	Map(USBFSH_BASE);
	Map(USBHSD_BASE);
	Map(USBHSH_BASE);
}

int main(void)
{
	MapAll();
	Reset();
#ifdef TEST_BAD_LAYOUT
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	assert(SYSCON->AHBCLKCTRL[1] == 0);
	assert(SYSCON->USB0CLKSEL == 7);
	assert(Lpc546xxUsbHsHwPrepare(6) == NULL);
	assert(SYSCON->AHBCLKCTRL[2] == 0 && SYSCON->USB1CLKSEL == 7);
	puts("LPC546xx USB rejects misplaced DMA storage: PASS");
	return 0;
#endif
// Normal boot has FRO 12 MHz active and HF output off. Also cover
	// an externally clocked system with the entire FRO powered down.
	for (int high = 0; high < 2; high++)
	{
		for (int poweredDown = 0; poweredDown < 2; poweredDown++)
		{
			Reset();
			SYSCON->MAINCLKSELA = 1;
			SYSCON->MAINCLKSELB = 2;
			SYSCON->SYSPLLCTRL = 0x123456;
			SYSCON->FROCTRL = (high ? SYSCON_FROCTRL_SEL_MASK : 0) | 0x1234;
			SYSCON->PDRUNCFG[0] |= SYSCON_PDRUNCFG_PDEN_ROM_MASK | SYSCON_PDRUNCFG_PDEN_VD6_MASK;
			if (poweredDown)
			{
				SYSCON->PDRUNCFG[0] |= SYSCON_PDRUNCFG_PDEN_FRO_MASK;
			}
			uint32_t power = SYSCON->PDRUNCFG[0];

			assert(Lpc546xxUsbHwPrepare(6) != NULL);
			assert(s_FroCalls == 1);
			assert(SYSCON->AHBCLKCTRL[0] == 0);
			assert((SYSCON->PDRUNCFG[0] & (SYSCON_PDRUNCFG_PDEN_ROM_MASK |
				SYSCON_PDRUNCFG_PDEN_VD6_MASK)) == (SYSCON_PDRUNCFG_PDEN_ROM_MASK |
				SYSCON_PDRUNCFG_PDEN_VD6_MASK));
			assert((SYSCON->FROCTRL & SYSCON_FROCTRL_TRIM_MASK) == 0x2345);
			assert(SYSCON->USB0CLKDIV == (uint32_t)high);
			assert(Lpc546xxUsbHwRelease());
			assert(SYSCON->PDRUNCFG[0] == power);
			assert((SYSCON->FROCTRL & SYSCON_FROCTRL_HSPDCLK_MASK) == 0);
			assert((SYSCON->FROCTRL & SYSCON_FROCTRL_TRIM_MASK) == 0x2345);
			assert(SYSCON->MAINCLKSELA == 1 && SYSCON->MAINCLKSELB == 2);
			assert(SYSCON->SYSPLLCTRL == 0x123456);
			assert(g_IrqMasked == 0);
		}
	}
	Reset();
	SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK_MASK;
	SYSCON->AHBCLKCTRL[0] = SYSCON_AHBCLKCTRL_ROM_MASK;
	assert(Lpc546xxUsbHwPrepare(6) != NULL);
	assert(s_FroCalls == 1);
	assert(Lpc546xxUsbHwRelease());
	assert(SYSCON->AHBCLKCTRL[0] == SYSCON_AHBCLKCTRL_ROM_MASK);
	assert((SYSCON->PDRUNCFG[0] & (SYSCON_PDRUNCFG_PDEN_ROM_MASK | SYSCON_PDRUNCFG_PDEN_VD6_MASK)) == 0);
	Reset();
	SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK_MASK;
	s_FroFail = true;
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	assert(s_FroCalls == 1);
	assert(SYSCON->FROCTRL == 0x1234);
	assert(SYSCON->AHBCLKCTRL[0] == 0 && SYSCON->AHBCLKCTRL[1] == 0 && SYSCON->AHBCLKCTRL[2] == 0);
	assert(g_IrqMasked == 0);
	Reset();
	SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK_MASK;
	s_DividerStuck = true;
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	assert(s_FroCalls == 1);
	assert((SYSCON->FROCTRL & SYSCON_FROCTRL_HSPDCLK_MASK) == 0);
	assert(SYSCON->AHBCLKCTRL[0] == 0);
	Reset();
	SYSCON->AHBCLKCTRL[1] = SYSCON_AHBCLKCTRL_USB0D_MASK;
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	SYSCON->AHBCLKCTRL[1] = 0;
	uint32_t others[] = {SYSCON_AHBCLKCTRL_USB1D_MASK, SYSCON_AHBCLKCTRL_USB1H_MASK,
		SYSCON_AHBCLKCTRL_USB0HMR_MASK, SYSCON_AHBCLKCTRL_USB0HSL_MASK};

	for (unsigned i = 0; i < sizeof(others) / sizeof(others[0]); i++)
	{
		SYSCON->AHBCLKCTRL[2] = others[i];
		assert(Lpc546xxUsbHwPrepare(6) == NULL);
		assert(SYSCON->AHBCLKCTRL[2] == others[i]);
	}
	SYSCON->AHBCLKCTRL[2] = 0;
	s_DividerStuck = true;
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	assert(s_Waits == 1023);
	assert(SYSCON->USB0CLKSEL == 7);
	assert(SYSCON->AHBCLKCTRL[1] == 0 && SYSCON->AHBCLKCTRL[2] == 0);
	assert(USBFSH->PORTMODE == USBFSH_PORTMODE_ID_EN_MASK);
	assert(g_IrqMasked == 0);
	Reset();
	for (int high = 0; high < 2; high++)
	{
		if (high)
		{
			SYSCON->FROCTRL |= SYSCON_FROCTRL_SEL_MASK;
			SYSCON->AHBCLKCTRL[2] = SYSCON_AHBCLKCTRL_USB1RAM_MASK;
			SYSCON->PDRUNCFG[0] = 0;
		}
		LpcUsbFsMemory_t *mem = Lpc546xxUsbHwPrepare(high ? 100 : -1);

		assert(mem != NULL);
		assert(s_FroCalls == 0);
		assert(s_Priority == (high ? (1U << __NVIC_PRIO_BITS) - 1U : 0U));
		assert(!s_Enabled && USB0->INTEN == 0);
		assert((USB0->DEVCMDSTAT & (USB_DEVCMDSTAT_DEV_EN_MASK | USB_DEVCMDSTAT_DCON_MASK)) == 0);
		assert(SYSCON->USB0CLKDIV == (uint32_t)high);
		assert(SYSCON->FROCTRL & SYSCON_FROCTRL_USBCLKADJ_MASK);
		assert(USBFSH->PORTMODE & USBFSH_PORTMODE_DEV_ENABLE_MASK);
		assert((USBFSH->PORTMODE & USBFSH_PORTMODE_ID_EN_MASK) == 0);
		assert(USB0->EPLISTSTART == (uintptr_t)mem);
		assert(USB0->DATABUFSTART == ((uintptr_t)mem & USB_DATABUFSTART_DA_BUF_MASK));
		for (unsigned i = 0; i < 10; i++)
		{
			assert(mem->Command[i * 2] == ((((uintptr_t)mem->Packet[i] >> 6) & 0xFFFFU) | (1U << 30)));
		}
		assert(mem->Command[1] == (((uintptr_t)mem->Setup >> 6) & 0xFFFFU));
		assert(Lpc546xxUsbHwPrepare(6) == NULL);
		USB0->DEVCMDSTAT |= USB_DEVCMDSTAT_DEV_EN_MASK;
		assert(!Lpc546xxUsbHwRelease());
		USB0->DEVCMDSTAT &= ~USB_DEVCMDSTAT_DEV_EN_MASK;
		USB0->EPSKIP = 1;
		assert(!Lpc546xxUsbHwRelease());
		USB0->EPSKIP = 0;
		mem->Command[4] |= 1U << 31;
		assert(!Lpc546xxUsbHwRelease());
		mem->Command[4] &= ~(1U << 31);
		assert(Lpc546xxUsbHwRelease());
		assert(SYSCON->AHBCLKCTRL[1] == 0);
		assert(SYSCON->AHBCLKCTRL[2] == (high ? SYSCON_AHBCLKCTRL_USB1RAM_MASK : 0));
		assert(SYSCON->PDRUNCFG[0] == (high ? 0 : SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK |
			SYSCON_PDRUNCFG_PDEN_USB0_PHY_MASK | SYSCON_PDRUNCFG_PDEN_VD5_MASK));
		assert(SYSCON->USB0CLKSEL == 7);
		assert(SYSCON->USB0CLKDIV == SYSCON_USB0CLKDIV_HALT_MASK);
		assert((SYSCON->FROCTRL & SYSCON_FROCTRL_USBCLKADJ_MASK) == 0);
		assert(USBFSH->PORTMODE == USBFSH_PORTMODE_ID_EN_MASK);
		assert(Lpc546xxUsbHwRelease());
		assert(g_IrqMasked == 0);
	}
	puts("LPC546xx USB FRO startup, clocks, SRAM, ownership and bounded failure cleanup: PASS");
	HsHardwareTest();
	puts("LPC546xx USB1 PLL, power, clocks, SRAM, ownership and failure cleanup: PASS");
	return 0;
}
