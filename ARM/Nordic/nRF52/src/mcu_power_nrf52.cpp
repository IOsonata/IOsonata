/**-------------------------------------------------------------------------
@file	mcu_power_nrf52.cpp

@brief	MCU power control implementation on the nRF52

Power off is System OFF. RESETREAS keeps adding reasons until it is cleared,
an empty value is a power on or brown out reset. The retention registers are
GPREGRET and GPREGRET2, 8 bits each. DC/DC stages: REG1 (DCDCEN) supplies
the core on every part, REG0 (DCDCEN0) is the high voltage input stage of
the nRF52840.

RAM is in blocks RAM[n] of sections: RAM0 to RAM7 have two 4 KB sections
each (the first 64 KB), RAM8 of the nRF52833/840 has 32 KB sections. The
RAM size comes from FICR INFO.RAM. RAM is also mapped at 0x00800000 for
code, both addresses are accepted.

When a SoftDevice is enabled it owns POWER, the functions then go through
its sd_power calls.

With a debugger attached, System OFF is emulated: the CPU keeps running and
McuPowerOff stays in its WFE loop.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

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
#include <stddef.h>

#include "nrf.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_mbr.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#include "coredev/mcu_power.h"

#define MCU_PWR_NRF52_NB_RETREG		2
#define MCU_PWR_NRF52_RETREG_MSK	0xFFUL

// RAM sections: 4 KB in the first 64 KB (RAM0 to RAM7), 32 KB above (RAM8)
#define MCU_PWR_NRF52_RAM_DATA		0x20000000UL
#define MCU_PWR_NRF52_RAM_CODE		0x00800000UL
#define MCU_PWR_NRF52_SMALL_SECT	4096UL
#define MCU_PWR_NRF52_SMALL_END		65536UL
#define MCU_PWR_NRF52_SMALL_PERBLK	2UL
#define MCU_PWR_NRF52_LARGE_SECT	32768UL
#define MCU_PWR_NRF52_NB_RAMBLK		(sizeof(NRF_POWER->RAM) / sizeof(NRF_POWER->RAM[0]))

static bool s_bMcuPwrResetRead = false;
static uint32_t s_McuPwrResetReason = 0;

// sd_softdevice_is_enabled is an SVC that only a programmed SoftDevice
// implements, so the image is looked for first: its magic number follows the
// MBR.
static bool McuPwrSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	const volatile uint32_t *pMagic = (const volatile uint32_t *)(
		MBR_SIZE + SOFTDEVICE_INFO_STRUCT_OFFSET + 4U);

	if (*pMagic != 0x51B1E5DBUL)
	{
		return false;
	}

	uint8_t enabled = 0U;

	(void)sd_softdevice_is_enabled(&enabled);

	return enabled != 0U;
#else
	return false;
#endif
}

static uint32_t McuPwrResetMap(uint32_t ResetReas)
{
	uint32_t r = 0;

	if (ResetReas == 0)
	{
		return MCU_RESET_POWERON;
	}
	if (ResetReas & POWER_RESETREAS_RESETPIN_Msk)
	{
		r |= MCU_RESET_PIN;
	}
	if (ResetReas & POWER_RESETREAS_DOG_Msk)
	{
		r |= MCU_RESET_WDT;
	}
	if (ResetReas & POWER_RESETREAS_SREQ_Msk)
	{
		r |= MCU_RESET_SOFT;
	}
	if (ResetReas & POWER_RESETREAS_LOCKUP_Msk)
	{
		r |= MCU_RESET_LOCKUP;
	}
	if (ResetReas & POWER_RESETREAS_OFF_Msk)
	{
		r |= MCU_RESET_WAKE_PIN;
	}
#ifdef POWER_RESETREAS_LPCOMP_Msk
	if (ResetReas & POWER_RESETREAS_LPCOMP_Msk)
	{
		r |= MCU_RESET_WAKE_COMP;
	}
#endif
	if (ResetReas & POWER_RESETREAS_DIF_Msk)
	{
		r |= MCU_RESET_DEBUG;
	}
#ifdef POWER_RESETREAS_NFC_Msk
	if (ResetReas & POWER_RESETREAS_NFC_Msk)
	{
		r |= MCU_RESET_WAKE_NFC;
	}
#endif
#ifdef POWER_RESETREAS_VBUS_Msk
	if (ResetReas & POWER_RESETREAS_VBUS_Msk)
	{
		r |= MCU_RESET_WAKE_USB;
	}
#endif

	return r;
}

uint32_t McuResetReason(void)
{
	if (s_bMcuPwrResetRead == false)
	{
		uint32_t reas = 0;

		if (McuPwrSdEnabled())
		{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
			(void)sd_power_reset_reason_get(&reas);
			(void)sd_power_reset_reason_clr(reas);
#endif
		}
		else
		{
			reas = NRF_POWER->RESETREAS;
			NRF_POWER->RESETREAS = reas;
		}

		s_McuPwrResetReason = McuPwrResetMap(reas);
		s_bMcuPwrResetRead = true;
	}

	return s_McuPwrResetReason;
}

void McuPowerOff(void)
{
	if (McuPwrSdEnabled())
	{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
		(void)sd_power_system_off();
#endif
	}
	else
	{
		NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		__DSB();
	}

	// Only reached when System OFF is emulated
	while (1)
	{
		__WFE();
	}
}

int McuRetRegCount(void)
{
	return MCU_PWR_NRF52_NB_RETREG;
}

uint32_t McuRetRegRead(int Idx)
{
	if (Idx < 0 || Idx >= MCU_PWR_NRF52_NB_RETREG)
	{
		return 0;
	}

	uint32_t val = 0;

	if (McuPwrSdEnabled())
	{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
		(void)sd_power_gpregret_get((uint32_t)Idx, &val);
#endif
	}
	else
	{
		val = Idx == 0 ? NRF_POWER->GPREGRET : NRF_POWER->GPREGRET2;
	}

	return val & MCU_PWR_NRF52_RETREG_MSK;
}

bool McuRetRegWrite(int Idx, uint32_t Val)
{
	if (Idx < 0 || Idx >= MCU_PWR_NRF52_NB_RETREG)
	{
		return false;
	}

	Val &= MCU_PWR_NRF52_RETREG_MSK;

	if (McuPwrSdEnabled())
	{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
		// The SoftDevice only sets or clears bits
		if (sd_power_gpregret_clr((uint32_t)Idx, MCU_PWR_NRF52_RETREG_MSK) != NRF_SUCCESS)
		{
			return false;
		}
		if (Val != 0 && sd_power_gpregret_set((uint32_t)Idx, Val) != NRF_SUCCESS)
		{
			return false;
		}
#endif
	}
	else if (Idx == 0)
	{
		NRF_POWER->GPREGRET = Val;
	}
	else
	{
		NRF_POWER->GPREGRET2 = Val;
	}

	return true;
}

bool McuDcDcEnable(int Stage, bool bEn)
{
	bool sd = McuPwrSdEnabled();

	if (Stage == MCU_DCDC_MAIN)
	{
		if (sd)
		{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
			return sd_power_dcdc_mode_set(bEn ? NRF_POWER_DCDC_ENABLE : NRF_POWER_DCDC_DISABLE) == NRF_SUCCESS;
#endif
		}
		NRF_POWER->DCDCEN = bEn ? POWER_DCDCEN_DCDCEN_Enabled : POWER_DCDCEN_DCDCEN_Disabled;

		return true;
	}

#ifdef POWER_DCDCEN0_DCDCEN_Msk
	if (Stage == MCU_DCDC_HIGHVOLT)
	{
		if (sd)
		{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT && defined(S140)
			return sd_power_dcdc0_mode_set(bEn ? NRF_POWER_DCDC_ENABLE : NRF_POWER_DCDC_DISABLE) == NRF_SUCCESS;
#else
			return false;
#endif
		}
		NRF_POWER->DCDCEN0 = bEn ? POWER_DCDCEN0_DCDCEN_Enabled : POWER_DCDCEN0_DCDCEN_Disabled;

		return true;
	}
#endif

	return false;
}

bool McuRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	uint32_t ramsize = NRF_FICR->INFO.RAM * 1024UL;
	uintptr_t base;

	if (Addr >= MCU_PWR_NRF52_RAM_DATA && Addr - MCU_PWR_NRF52_RAM_DATA < ramsize)
	{
		base = MCU_PWR_NRF52_RAM_DATA;
	}
	else if (Addr >= MCU_PWR_NRF52_RAM_CODE && Addr - MCU_PWR_NRF52_RAM_CODE < ramsize)
	{
		base = MCU_PWR_NRF52_RAM_CODE;
	}
	else
	{
		return false;
	}

	uint32_t off = (uint32_t)(Addr - base);

	if (Size > ramsize - off)
	{
		return false;
	}

	uint32_t end = off + Size;
	bool sd = McuPwrSdEnabled();

	while (off < end)
	{
		uint32_t blk;
		uint32_t sect;
		uint32_t next;

		if (off < MCU_PWR_NRF52_SMALL_END)
		{
			blk = off / (MCU_PWR_NRF52_SMALL_SECT * MCU_PWR_NRF52_SMALL_PERBLK);
			sect = (off / MCU_PWR_NRF52_SMALL_SECT) % MCU_PWR_NRF52_SMALL_PERBLK;
			next = (off / MCU_PWR_NRF52_SMALL_SECT + 1) * MCU_PWR_NRF52_SMALL_SECT;
		}
		else
		{
			blk = MCU_PWR_NRF52_SMALL_END / (MCU_PWR_NRF52_SMALL_SECT * MCU_PWR_NRF52_SMALL_PERBLK);
			sect = (off - MCU_PWR_NRF52_SMALL_END) / MCU_PWR_NRF52_LARGE_SECT;
			next = MCU_PWR_NRF52_SMALL_END + (sect + 1) * MCU_PWR_NRF52_LARGE_SECT;
		}

		if (blk >= MCU_PWR_NRF52_NB_RAMBLK)
		{
			return false;
		}

		uint32_t msk = POWER_RAM_POWERSET_S0RETENTION_Msk << sect;

		if (sd)
		{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
			uint32_t rc = bRetain ? sd_power_ram_power_set((uint8_t)blk, msk) :
									sd_power_ram_power_clr((uint8_t)blk, msk);

			if (rc != NRF_SUCCESS)
			{
				return false;
			}
#endif
		}
		else if (bRetain)
		{
			NRF_POWER->RAM[blk].POWERSET = msk;
		}
		else
		{
			NRF_POWER->RAM[blk].POWERCLR = msk;
		}

		off = next;
	}

	return true;
}
