/**-------------------------------------------------------------------------
@file usb_hw_lpc546xx.c
@brief LPC546xx USB0 and USB1 clocks, device mode and endpoint SRAM preparation.

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
#include <stddef.h>
#include <string.h>

#include "LPC546xx.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "usb_hw_lpc546xx.h"

#define LPCUSB_ACTIVE (1UL << 31)
#define LPCUSB_DISABLED (1UL << 30)
#define LPCUSB_NO_WAKE (USB_DEVCMDSTAT_DSUS_MASK | USB_DEVCMDSTAT_LPM_SUS_MASK)
#define LPCUSB_STATUS (USB_DEVCMDSTAT_SETUP_MASK | USB_DEVCMDSTAT_DCON_C_MASK | \
	USB_DEVCMDSTAT_DSUS_C_MASK | USB_DEVCMDSTAT_DRES_C_MASK)
#define LPCUSB_INTERRUPTS (USB_EPSKIP_SKIP_MASK | USB_INTSTAT_FRAME_INT_MASK | USB_INTSTAT_DEV_INT_MASK)
#define LPCUSB_POWER (SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_USB0_PHY_MASK | \
	SYSCON_PDRUNCFG_PDEN_FRO_MASK)
#define LPCUSB_ROM_POWER (SYSCON_PDRUNCFG_PDEN_ROM_MASK | SYSCON_PDRUNCFG_PDEN_VD6_MASK)
#define LPCUSB_FRO_ROM_ENTRY 0x030091DFU
#define LPCUSB_FRO_CONTROL (SYSCON_FROCTRL_USBCLKADJ_MASK | SYSCON_FROCTRL_HSPDCLK_MASK)
#define LPCUSB_OTHER_CLOCKS (SYSCON_AHBCLKCTRL_USB1H_MASK | SYSCON_AHBCLKCTRL_USB1D_MASK | \
	SYSCON_AHBCLKCTRL_USB0HMR_MASK | SYSCON_AHBCLKCTRL_USB0HSL_MASK)
// Reserved FROCTRL bits excluded by NXP's LPC54628 CLOCK_EnableUsbfs0DeviceClock.
#define LPCUSB_FRO_RESERVED ((1UL << 15) | (15UL << 26))
#define LPCUSB_WAIT_COUNT 1024

// USB1. The device command and status bits above are at the same place.
#define LPCUSB_HS_INTERRUPTS (USBHSD_EPSKIP_SKIP_MASK | USBHSD_INTSTAT_FRAME_INT_MASK | \
	USBHSD_INTSTAT_DEV_INT_MASK)
#define LPCUSB_HS_POWER0 (SYSCON_PDRUNCFG_PDEN_USB_RAM_MASK | SYSCON_PDRUNCFG_PDEN_VD5_MASK)
#define LPCUSB_HS_POWER1 (SYSCON_PDRUNCFG_PDEN_USB1_PHY_MASK | SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK)
// Voltage domain status of the NXP power library: VD3 supplies the PLLs, VD5
// the USB1 PHY.
#define LPCUSB_VD_STATUS (*(volatile uint32_t *)0x40020054UL)
#define LPCUSB_VD3_READY (1UL << 5)
#define LPCUSB_VD5_READY (1UL << 9)
#define LPCUSB_VD_WAIT 100000UL
// USB PLL setup of NXP's LPC54628 CLOCK_EnableUsbhs0DeviceClock: 12 MHz / 4 x
// 64 = 192 MHz CCO, / 4 = 48 MHz, non integer mode.
#define LPCUSB_HS_OSC_FREQ 12000000UL
#define LPCUSB_PLL_CONTROL (SYSCON_USBPLLCTRL_MSEL(0x3FU) | SYSCON_USBPLLCTRL_PSEL(1U) | \
	SYSCON_USBPLLCTRL_NSEL(3U))
#define LPCUSB_PLL_FIELDS (SYSCON_USBPLLCTRL_MSEL_MASK | SYSCON_USBPLLCTRL_PSEL_MASK | \
	SYSCON_USBPLLCTRL_NSEL_MASK | SYSCON_USBPLLCTRL_DIRECT_MASK | \
	SYSCON_USBPLLCTRL_BYPASS_MASK | SYSCON_USBPLLCTRL_FBSEL_MASK)
#define LPCUSB_PLL_START_US 72U
#define LPCUSB_PLL_POLL_US 10U
#define LPCUSB_PLL_POLL_COUNT 100
#define LPCUSB_USB1CLK_USB_PLL 2U
// A high-speed buffer offset holds address bits 16:6 within this page
#define LPCUSB_HS_PAGE_MASK 0x1FFFFUL

// One controller at a time: both use the same storage in the USB RAM.
static union {
	LpcUsbFsMemory_t Fs;
	Lpc546xxUsbHsMemory_t Hs;
} s_Memory __attribute__((section(".usb_ram"), aligned(256)));
static bool s_Prepared;
static uint32_t s_AddedRamClock;
static uint32_t s_AddedPower;
static uint32_t s_OldClockSelect;
static uint32_t s_OldClockDivide;
static uint32_t s_OldClockControl;
static uint32_t s_OldPortMode;
static uint32_t s_OldFroControl;

_Static_assert(offsetof(LpcUsbFsMemory_t, Setup) == 128, "SETUP alignment");
_Static_assert(offsetof(LpcUsbFsMemory_t, Packet) == 192, "packet alignment");
_Static_assert(sizeof(LpcUsbFsMemory_t) == 832, "USB SRAM storage size");
_Static_assert(offsetof(Lpc546xxUsbHsMemory_t, Setup) == 128, "USB1 SETUP alignment");
_Static_assert(offsetof(Lpc546xxUsbHsMemory_t, Packet0) == 192, "USB1 EP0 packet alignment");
_Static_assert(offsetof(Lpc546xxUsbHsMemory_t, Packet) == 320, "USB1 packet alignment");
_Static_assert(sizeof(Lpc546xxUsbHsMemory_t) == 5440, "USB1 SRAM storage size");

static bool s_HsPrepared;
static uint32_t s_HsAddedRamClock;
static uint32_t s_HsAddedPower0;
static uint32_t s_HsAddedPower1;
static uint32_t s_HsOldClockSelect;
static uint32_t s_HsOldClockDivide;
static uint32_t s_HsOldClockControl;
static uint32_t s_HsOldPortMode;
static uint32_t s_HsOldPllControl;

// Storage in the USB RAM, aligned for the endpoint list, and no controller,
// host or device, of either port active
static bool Lpc546xxUsbHwFree(uintptr_t Address)
{
	return !s_Prepared && !s_HsPrepared && Address >= FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS &&
		Address <= FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS + FSL_FEATURE_USB_USB_RAM - sizeof(s_Memory) &&
		(Address & 255U) == 0U &&
		(SYSCON->AHBCLKCTRL[1] & SYSCON_AHBCLKCTRL_USB0D_MASK) == 0U &&
		(SYSCON->AHBCLKCTRL[2] & LPCUSB_OTHER_CLOCKS) == 0U;
}

static void Lpc546xxUsbRestore(void)
{
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_USB0HSL_MASK;
	__DSB();
	USBFSH->PORTMODE = s_OldPortMode;
	SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_USB0HSL_MASK | s_AddedRamClock;
	SYSCON->AHBCLKCTRLCLR[1] = SYSCON_AHBCLKCTRL_USB0D_MASK;
	__DSB();
	SYSCON->USB0CLKCTRL = s_OldClockControl;
	SYSCON->USB0CLKSEL = s_OldClockSelect;
	SYSCON->USB0CLKDIV = s_OldClockDivide;
	// Retain the calibrated trim; restore the HF output and SOF adjustment.
	SYSCON->FROCTRL = (SYSCON->FROCTRL & ~(LPCUSB_FRO_RESERVED |
		LPCUSB_FRO_CONTROL)) | s_OldFroControl;
	SYSCON->PDRUNCFGSET[0] = s_AddedPower;
	__DSB();
}

// NXP LPC54628 CLOCK_SetupFROClocking uses this ROM calibration entry.
// Keep the selected 48/96 MHz rate; do not switch the CPU clock or PLL.
static bool Lpc546xxUsbFroStart(uint32_t Fro)
{
	if ((s_AddedPower & SYSCON_PDRUNCFG_PDEN_FRO_MASK) == 0U &&
		(Fro & SYSCON_FROCTRL_HSPDCLK_MASK) != 0U)
	{
		return true;
	}
	uint32_t romPower = SYSCON->PDRUNCFG[0] & LPCUSB_ROM_POWER;
	uint32_t romClock = SYSCON_AHBCLKCTRL_ROM_MASK & ~SYSCON->AHBCLKCTRL[0];
	uint32_t frequency = (Fro & SYSCON_FROCTRL_SEL_MASK) != 0U ? 96000000U : 48000000U;

	// The ROM calibration reads OTP on VD6. Both must be powered and the
	// ROM AHB clock enabled before the call, with interrupts masked.
	SYSCON->PDRUNCFGCLR[0] = LPCUSB_ROM_POWER | SYSCON_PDRUNCFG_PDEN_FRO_MASK;
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_ROM_MASK;
	__DSB();
	((void (*)(uint32_t))LPCUSB_FRO_ROM_ENTRY)(frequency);
	__DSB();
	SYSCON->AHBCLKCTRLCLR[0] = romClock;
	SYSCON->PDRUNCFGSET[0] = romPower;
	__DSB();

	uint32_t ready = SYSCON->FROCTRL;

	if ((ready & SYSCON_FROCTRL_HSPDCLK_MASK) == 0U ||
		(ready & SYSCON_FROCTRL_SEL_MASK) != (Fro & SYSCON_FROCTRL_SEL_MASK))
	{
		SYSCON->FROCTRL = Fro & ~LPCUSB_FRO_RESERVED;
		return false;
	}
	SYSCON->FROCTRL = (ready & ~LPCUSB_FRO_RESERVED) | SYSCON_FROCTRL_WRTRIM_MASK;
	return true;
}

LpcUsbFsMemory_t *Lpc546xxUsbHwPrepare(int IntPrio)
{
	uintptr_t address = (uintptr_t)&s_Memory;
	uint32_t state = DisableInterrupt();
	uint32_t fro = SYSCON->FROCTRL;

	// Clock ownership is exclusive. Never reset a ROM/native controller or
	// change shared RAM power while another USB controller may be using it.
	if (!Lpc546xxUsbHwFree(address))
	{
		EnableInterrupt(state);
		return NULL;
	}

	s_AddedRamClock = SYSCON_AHBCLKCTRL_USB1RAM_MASK & ~SYSCON->AHBCLKCTRL[2];
	s_AddedPower = SYSCON->PDRUNCFG[0] & LPCUSB_POWER;
	s_OldClockSelect = SYSCON->USB0CLKSEL;
	s_OldClockDivide = SYSCON->USB0CLKDIV;
	s_OldClockControl = SYSCON->USB0CLKCTRL;
	s_OldFroControl = fro & LPCUSB_FRO_CONTROL;

	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_USB0HSL_MASK;
	__DSB();
	s_OldPortMode = USBFSH->PORTMODE;
	// The device/host mux is in the host register block, even in device mode.
	USBFSH->PORTMODE = (s_OldPortMode & ~USBFSH_PORTMODE_ID_EN_MASK) |
		USBFSH_PORTMODE_DEV_ENABLE_MASK;
	SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_USB0HSL_MASK;
	__DSB();

	if (!Lpc546xxUsbFroStart(fro))
	{
		Lpc546xxUsbRestore();
		EnableInterrupt(state);
		return NULL;
	}
	// ROM may have updated calibration. Never write back the pre-call trim.
	fro = SYSCON->FROCTRL;
	SYSCON->PDRUNCFGCLR[0] = LPCUSB_POWER;
	SYSCON->FROCTRL = (fro & ~LPCUSB_FRO_RESERVED) | SYSCON_FROCTRL_USBCLKADJ_MASK;
	SYSCON->USB0CLKSEL = SYSCON_USB0CLKSEL_SEL(0);
	SYSCON->USB0CLKDIV = SYSCON_USB0CLKDIV_DIV((fro & SYSCON_FROCTRL_SEL_MASK) != 0U ? 1U : 0U);
	int wait = LPCUSB_WAIT_COUNT;

	while ((SYSCON->USB0CLKDIV & SYSCON_USB0CLKDIV_REQFLAG_MASK) != 0U && --wait > 0)
	{
		__NOP();
	}
	if (wait == 0)
	{
		Lpc546xxUsbRestore();
		EnableInterrupt(state);
		return NULL;
	}

	// Keep the functional clock running during preparation and normal suspend.
	SYSCON->USB0CLKCTRL = (s_OldClockControl & ~(SYSCON_USB0CLKCTRL_PU_DISABLE_MASK |
		SYSCON_USB0CLKCTRL_POL_FS_DEV_CLK_MASK)) |
		SYSCON_USB0CLKCTRL_AP_FS_DEV_CLK_MASK;
	SYSCON->AHBCLKCTRLSET[1] = SYSCON_AHBCLKCTRL_USB0D_MASK;
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_USB1RAM_MASK;
	SYSCON->PRESETCTRLSET[1] = SYSCON_PRESETCTRL_USB0D_RST_MASK;
	__DSB();
	SYSCON->PRESETCTRLCLR[1] = SYSCON_PRESETCTRL_USB0D_RST_MASK;
	SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_USB1RAM_RST_MASK;
	__DSB();

	NVIC_DisableIRQ(USB0_IRQn);
	USB0->INTEN = 0U;
	USB0->DEVCMDSTAT = USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK | LPCUSB_NO_WAKE | LPCUSB_STATUS;
	memset(&s_Memory, 0, sizeof(s_Memory.Fs));
	for (unsigned physical = 0; physical < 10U; physical++)
	{
		uint32_t buffer = (uint32_t)(uintptr_t)s_Memory.Fs.Packet[physical];

		s_Memory.Fs.Command[physical * 2U] = ((buffer >> 6) & 0xFFFFU) | LPCUSB_DISABLED;
		s_Memory.Fs.Command[physical * 2U + 1U] = LPCUSB_DISABLED;
	}
	s_Memory.Fs.Command[1] = ((uint32_t)(uintptr_t)s_Memory.Fs.Setup >> 6) & 0xFFFFU;
	s_Memory.Fs.Command[3] = 0U;
	__DMB();
	USB0->EPLISTSTART = (uint32_t)address;
	USB0->DATABUFSTART = (uint32_t)address & USB_DATABUFSTART_DA_BUF_MASK;
	USB0->EPINUSE = 0U;
	USB0->EPBUFCFG = 0U;
	USB0->INTSTAT = LPCUSB_INTERRUPTS;
	NVIC_ClearPendingIRQ(USB0_IRQn);
	int maxprio = (1 << __NVIC_PRIO_BITS) - 1;

	NVIC_SetPriority(USB0_IRQn, IntPrio < 0 ? 0U :
		(IntPrio > maxprio ? (uint32_t)maxprio : (uint32_t)IntPrio));
	s_Prepared = true;
	EnableInterrupt(state);
	return &s_Memory.Fs;
}

bool Lpc546xxUsbHwRelease(void)
{
	uint32_t state = DisableInterrupt();

	if (!s_Prepared)
	{
		EnableInterrupt(state);
		return true;
	}
	if ((USB0->DEVCMDSTAT & (USB_DEVCMDSTAT_DEV_EN_MASK | USB_DEVCMDSTAT_DCON_MASK)) != 0U ||
		(USB0->EPSKIP & USB_EPSKIP_SKIP_MASK) != 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	for (unsigned i = 0; i < 20U; i++)
	{
		if (i != 1U && (s_Memory.Fs.Command[i] & LPCUSB_ACTIVE) != 0U)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	NVIC_DisableIRQ(USB0_IRQn);
	USB0->INTEN = 0U;
	USB0->DEVCMDSTAT = LPCUSB_NO_WAKE | LPCUSB_STATUS;
	USB0->INTSTAT = LPCUSB_INTERRUPTS;
	__DSB();
	NVIC_ClearPendingIRQ(USB0_IRQn);
	Lpc546xxUsbRestore();
	s_Prepared = false;
	EnableInterrupt(state);
	return true;
}

static void Lpc546xxUsbHsRestore(void)
{
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_USB1H_MASK;
	__DSB();
	USBHSH->PORTMODE = s_HsOldPortMode;
	SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_USB1H_MASK | SYSCON_AHBCLKCTRL_USB1D_MASK |
		s_HsAddedRamClock;
	__DSB();
	SYSCON->USB1CLKCTRL = s_HsOldClockControl;
	SYSCON->USB1CLKSEL = s_HsOldClockSelect;
	SYSCON->USB1CLKDIV = s_HsOldClockDivide;
	// The PHY, and the PLL when started here
	SYSCON->PDRUNCFGSET[1] = s_HsAddedPower1;
	if ((s_HsAddedPower1 & SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK) != 0U)
	{
		SYSCON->USBPLLCTRL = s_HsOldPllControl;
	}
	SYSCON->PDRUNCFGSET[0] = s_HsAddedPower0;
	__DSB();
}

static bool Lpc546xxUsbVdReady(uint32_t Ready)
{
	uint32_t wait = LPCUSB_VD_WAIT;

	while ((LPCUSB_VD_STATUS & Ready) == 0U && --wait > 0U)
	{
	}
	return wait != 0U;
}

// NXP's LPC54628 CLOCK_SetUsbPLLFreq sequence. A PLL already running is used
// as it is when it gives 48 MHz.
static bool Lpc546xxUsbPllStart(void)
{
	if ((s_HsAddedPower1 & SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK) == 0U)
	{
		return (SYSCON->USBPLLCTRL & LPCUSB_PLL_FIELDS) == LPCUSB_PLL_CONTROL &&
			(SYSCON->USBPLLSTAT & SYSCON_USBPLLSTAT_LOCK_MASK) != 0U;
	}
	SYSCON->USBPLLCTRL = LPCUSB_PLL_CONTROL;
	SYSCON->PDRUNCFGCLR[1] = SYSCON_PDRUNCFG_PDEN_USB1_PLL_MASK;
	__DSB();
	usDelay(LPCUSB_PLL_START_US);
	for (int poll = 0; poll < LPCUSB_PLL_POLL_COUNT; poll++)
	{
		if ((SYSCON->USBPLLSTAT & SYSCON_USBPLLSTAT_LOCK_MASK) != 0U)
		{
			return true;
		}
		usDelay(LPCUSB_PLL_POLL_US);
	}
	return false;
}

Lpc546xxUsbHsMemory_t *Lpc546xxUsbHsHwPrepare(int IntPrio)
{
	uintptr_t address = (uintptr_t)&s_Memory;
	uint32_t state = DisableInterrupt();

	// The USB PLL input is the system oscillator: it must run at 12 MHz.
	if (!Lpc546xxUsbHwFree(address) ||
		(g_McuOsc.CoreOsc.Type != OSC_TYPE_XTAL && g_McuOsc.CoreOsc.Type != OSC_TYPE_TCXO) ||
		g_McuOsc.CoreOsc.Freq != LPCUSB_HS_OSC_FREQ ||
		(SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK) != 0U ||
		(SYSCON->PDRUNCFG[1] & SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK) != 0U)
	{
		EnableInterrupt(state);
		return NULL;
	}

	s_HsAddedRamClock = SYSCON_AHBCLKCTRL_USB1RAM_MASK & ~SYSCON->AHBCLKCTRL[2];
	s_HsAddedPower0 = SYSCON->PDRUNCFG[0] & LPCUSB_HS_POWER0;
	s_HsAddedPower1 = SYSCON->PDRUNCFG[1] & LPCUSB_HS_POWER1;
	s_HsOldClockSelect = SYSCON->USB1CLKSEL;
	s_HsOldClockDivide = SYSCON->USB1CLKDIV;
	s_HsOldClockControl = SYSCON->USB1CLKCTRL;
	s_HsOldPllControl = SYSCON->USBPLLCTRL;

	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_USB1H_MASK;
	__DSB();
	s_HsOldPortMode = USBHSH->PORTMODE;
	// The device/host mux is in the host register block, even in device mode.
	USBHSH->PORTMODE = (s_HsOldPortMode & ~USBHSH_PORTMODE_ID0_EN_MASK) |
		USBHSH_PORTMODE_DEV_ENABLE_MASK;
	SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_USB1H_MASK;
	__DSB();

	// NXP's POWER_SetUsbPhy and POWER_SetPLL
	SYSCON->PDRUNCFGCLR[0] = LPCUSB_HS_POWER0;
	__DSB();
	bool ready = Lpc546xxUsbVdReady(LPCUSB_VD5_READY);
	if (ready)
	{
		SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_VD3_MASK;
		__DSB();
		ready = Lpc546xxUsbVdReady(LPCUSB_VD3_READY) && Lpc546xxUsbPllStart();
	}
	int wait = LPCUSB_WAIT_COUNT;
	if (ready)
	{
		SYSCON->USB1CLKSEL = SYSCON_USB1CLKSEL_SEL(LPCUSB_USB1CLK_USB_PLL);
		SYSCON->USB1CLKDIV = SYSCON_USB1CLKDIV_DIV(0U);
		while ((SYSCON->USB1CLKDIV & SYSCON_USB1CLKDIV_REQFLAG_MASK) != 0U && --wait > 0)
		{
			__NOP();
		}
	}
	if (!ready || wait == 0)
	{
		Lpc546xxUsbHsRestore();
		EnableInterrupt(state);
		return NULL;
	}

	// Keep the functional clock running during preparation and normal suspend.
	SYSCON->USB1CLKCTRL = (s_HsOldClockControl & ~SYSCON_USB1CLKCTRL_POL_FS_DEV_CLK_MASK) |
		SYSCON_USB1CLKCTRL_AP_FS_DEV_CLK_MASK;
	SYSCON->PDRUNCFGCLR[1] = SYSCON_PDRUNCFG_PDEN_USB1_PHY_MASK;
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_USB1D_MASK | SYSCON_AHBCLKCTRL_USB1RAM_MASK;
	SYSCON->PRESETCTRLSET[2] = SYSCON_PRESETCTRL_USB1D_RST_MASK;
	__DSB();
	SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_USB1D_RST_MASK | SYSCON_PRESETCTRL_USB1RAM_RST_MASK;
	__DSB();

	NVIC_DisableIRQ(USB1_IRQn);
	USBHSD->INTEN = 0U;
	USBHSD->DEVCMDSTAT = USBHSD_DEVCMDSTAT_FORCE_NEEDCLK_MASK | LPCUSB_NO_WAKE | LPCUSB_STATUS;
	memset(&s_Memory, 0, sizeof(s_Memory.Hs));
	for (unsigned physical = 0; physical < 12U; physical++)
	{
		uint32_t buffer = (uint32_t)(uintptr_t)(physical < 2U ? s_Memory.Hs.Packet0[physical] :
			s_Memory.Hs.Packet[physical - 2U]);

		s_Memory.Hs.Command[physical * 2U] = ((buffer >> 6) & 0x7FFU) | LPCUSB_DISABLED;
		s_Memory.Hs.Command[physical * 2U + 1U] = LPCUSB_DISABLED;
	}
	s_Memory.Hs.Command[1] = ((uint32_t)(uintptr_t)s_Memory.Hs.Setup >> 6) & 0x7FFU;
	s_Memory.Hs.Command[3] = 0U;
	__DMB();
	USBHSD->EPLISTSTART = (uint32_t)address;
	USBHSD->DATABUFSTART = (uint32_t)address & ~LPCUSB_HS_PAGE_MASK;
	USBHSD->EPINUSE = 0U;
	USBHSD->EPBUFCFG = 0U;
	USBHSD->INTSTAT = LPCUSB_HS_INTERRUPTS;
	NVIC_ClearPendingIRQ(USB1_IRQn);
	int maxprio = (1 << __NVIC_PRIO_BITS) - 1;

	NVIC_SetPriority(USB1_IRQn, IntPrio < 0 ? 0U :
		(IntPrio > maxprio ? (uint32_t)maxprio : (uint32_t)IntPrio));
	s_HsPrepared = true;
	EnableInterrupt(state);
	return &s_Memory.Hs;
}

bool Lpc546xxUsbHsHwRelease(void)
{
	uint32_t state = DisableInterrupt();

	if (!s_HsPrepared)
	{
		EnableInterrupt(state);
		return true;
	}
	if ((USBHSD->DEVCMDSTAT & (USBHSD_DEVCMDSTAT_DEV_EN_MASK | USBHSD_DEVCMDSTAT_DCON_MASK)) != 0U ||
		(USBHSD->EPSKIP & USBHSD_EPSKIP_SKIP_MASK) != 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	for (unsigned i = 0; i < 24U; i++)
	{
		if (i != 1U && (s_Memory.Hs.Command[i] & LPCUSB_ACTIVE) != 0U)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	NVIC_DisableIRQ(USB1_IRQn);
	USBHSD->INTEN = 0U;
	USBHSD->DEVCMDSTAT = LPCUSB_NO_WAKE | LPCUSB_STATUS;
	USBHSD->INTSTAT = LPCUSB_HS_INTERRUPTS;
	__DSB();
	NVIC_ClearPendingIRQ(USB1_IRQn);
	Lpc546xxUsbHsRestore();
	s_HsPrepared = false;
	EnableInterrupt(state);
	return true;
}
