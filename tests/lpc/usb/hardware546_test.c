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
#include "usb_hw_lpc546xx.h"

uint32_t g_IrqMasked;
static uint32_t s_Priority;
static bool s_Enabled;
static bool s_DividerStuck;
static unsigned s_Waits;
static unsigned s_FroCalls;
static bool s_FroFail;

void __DSB(void)
{
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
	SYSCON->PDRUNCFG[0] |= SYSCON->PDRUNCFGSET[0];
	SYSCON->PDRUNCFG[0] &= ~SYSCON->PDRUNCFGCLR[0];
	SYSCON->PDRUNCFGSET[0] = 0;
	SYSCON->PDRUNCFGCLR[0] = 0;
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

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB0_IRQn);
	s_Enabled = false;
}

void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB0_IRQn);
}

void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	assert(Irq == USB0_IRQn);
	s_Priority = Priority;
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
	SYSCON->FROCTRL = SYSCON_FROCTRL_HSPDCLK_MASK | 0x1234;
	SYSCON->USB0CLKSEL = 7;
	SYSCON->USB0CLKDIV = SYSCON_USB0CLKDIV_HALT_MASK;
	SYSCON->PDRUNCFG[0] = SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_USB0_PHY_MASK;
	USBFSH->PORTMODE = USBFSH_PORTMODE_ID_EN_MASK;
	s_DividerStuck = false;
	s_Waits = 0;
	s_FroCalls = 0;
	s_FroFail = false;
	g_IrqMasked = 0;
}

int main(void)
{
	Map(SYSCON_BASE);
	Map(USB0_BASE);
	Map(USBFSH_BASE);
	Reset();
#ifdef TEST_BAD_LAYOUT
	assert(Lpc546xxUsbHwPrepare(6) == NULL);
	assert(SYSCON->AHBCLKCTRL[1] == 0);
	assert(SYSCON->USB0CLKSEL == 7);
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
		assert(SYSCON->PDRUNCFG[0] == (high ? 0 : SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_USB0_PHY_MASK));
		assert(SYSCON->USB0CLKSEL == 7);
		assert(SYSCON->USB0CLKDIV == SYSCON_USB0CLKDIV_HALT_MASK);
		assert((SYSCON->FROCTRL & SYSCON_FROCTRL_USBCLKADJ_MASK) == 0);
		assert(USBFSH->PORTMODE == USBFSH_PORTMODE_ID_EN_MASK);
		assert(Lpc546xxUsbHwRelease());
		assert(g_IrqMasked == 0);
	}
	puts("LPC546xx USB FRO startup, clocks, SRAM, ownership and bounded failure cleanup: PASS");
	return 0;
}
