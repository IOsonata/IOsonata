/**-------------------------------------------------------------------------
@file usb_hw_lpc546xx.c
@brief LPC546xx USB0 clocks, device mode and endpoint SRAM preparation.

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

static LpcUsbFsMemory_t s_Memory
	__attribute__((section(".usb_ram"), aligned(256)));
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
	if (s_Prepared || address < FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS ||
		address > FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS + FSL_FEATURE_USB_USB_RAM - sizeof(s_Memory) ||
		(address & 255U) != 0U ||
		(SYSCON->AHBCLKCTRL[1] & SYSCON_AHBCLKCTRL_USB0D_MASK) != 0U ||
		(SYSCON->AHBCLKCTRL[2] & LPCUSB_OTHER_CLOCKS) != 0U)
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
	int wait = 1024;

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
	memset(&s_Memory, 0, sizeof(s_Memory));
	for (unsigned physical = 0; physical < 10U; physical++)
	{
		uint32_t buffer = (uint32_t)(uintptr_t)s_Memory.Packet[physical];

		s_Memory.Command[physical * 2U] = ((buffer >> 6) & 0xFFFFU) | LPCUSB_DISABLED;
		s_Memory.Command[physical * 2U + 1U] = LPCUSB_DISABLED;
	}
	s_Memory.Command[1] = ((uint32_t)(uintptr_t)s_Memory.Setup >> 6) & 0xFFFFU;
	s_Memory.Command[3] = 0U;
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
	return &s_Memory;
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
		if (i != 1U && (s_Memory.Command[i] & LPCUSB_ACTIVE) != 0U)
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
