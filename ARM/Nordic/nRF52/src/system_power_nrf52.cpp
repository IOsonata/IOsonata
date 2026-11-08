/**-------------------------------------------------------------------------
@file	system_power_nrf52.cpp

@brief	Power functions of system_core_clock.h on the nRF52

Power off is System OFF. RESETREAS keeps adding reasons until it is cleared,
an empty value is a power on or brown out reset. The retention registers are
GPREGRET and GPREGRET2, 8 bits each. The DC/DC regulator is set by SystemInit
from g_McuOsc.

RAM is in blocks RAM[n] of sections: RAM0 to RAM7 have two 4 KB sections
each (the first 64 KB), RAM8 of the nRF52833/840 has 32 KB sections. The
RAM size comes from FICR INFO.RAM. RAM is also mapped at 0x00800000 for
code, both addresses are accepted.

When a SoftDevice is enabled it owns POWER, the functions then go through
its sd_power calls.

With a debugger attached, System OFF is emulated: the CPU keeps running and
SystemPowerOff stays in its WFE loop.

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

#include "coredev/system_core_clock.h"

#define SYS_PWR_NRF52_NB_RETREG		2
#define SYS_PWR_NRF52_RETREG_MSK	0xFFUL

// RAM sections: 4 KB in the first 64 KB (RAM0 to RAM7), 32 KB above (RAM8)
#define SYS_PWR_NRF52_RAM_DATA		0x20000000UL
#define SYS_PWR_NRF52_RAM_CODE		0x00800000UL
#define SYS_PWR_NRF52_SMALL_SECT	4096UL
#define SYS_PWR_NRF52_SMALL_END		65536UL
#define SYS_PWR_NRF52_SMALL_PERBLK	2UL
#define SYS_PWR_NRF52_LARGE_SECT	32768UL
#define SYS_PWR_NRF52_NB_RAMBLK		(sizeof(NRF_POWER->RAM) / sizeof(NRF_POWER->RAM[0]))

static bool s_bSysPwrResetRead = false;
static uint32_t s_SysPwrResetReason = 0;

// sd_softdevice_is_enabled is an SVC that only a programmed SoftDevice
// implements, so the image is looked for first: its magic number follows the
// MBR.
static bool SysPwrSdEnabled(void)
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

static uint32_t SysPwrResetMap(uint32_t ResetReas)
{
	uint32_t r = 0;

	if (ResetReas == 0)
	{
		return SYSTEM_RESET_POWERON;
	}
	if (ResetReas & POWER_RESETREAS_RESETPIN_Msk)
	{
		r |= SYSTEM_RESET_PIN;
	}
	if (ResetReas & POWER_RESETREAS_DOG_Msk)
	{
		r |= SYSTEM_RESET_WDT;
	}
	if (ResetReas & POWER_RESETREAS_SREQ_Msk)
	{
		r |= SYSTEM_RESET_SOFT;
	}
	if (ResetReas & POWER_RESETREAS_LOCKUP_Msk)
	{
		r |= SYSTEM_RESET_LOCKUP;
	}
	if (ResetReas & POWER_RESETREAS_OFF_Msk)
	{
		r |= SYSTEM_RESET_WAKE_PIN;
	}
#ifdef POWER_RESETREAS_LPCOMP_Msk
	if (ResetReas & POWER_RESETREAS_LPCOMP_Msk)
	{
		r |= SYSTEM_RESET_WAKE_COMP;
	}
#endif
	if (ResetReas & POWER_RESETREAS_DIF_Msk)
	{
		r |= SYSTEM_RESET_DEBUG;
	}
#ifdef POWER_RESETREAS_NFC_Msk
	if (ResetReas & POWER_RESETREAS_NFC_Msk)
	{
		r |= SYSTEM_RESET_WAKE_NFC;
	}
#endif
#ifdef POWER_RESETREAS_VBUS_Msk
	if (ResetReas & POWER_RESETREAS_VBUS_Msk)
	{
		r |= SYSTEM_RESET_WAKE_USB;
	}
#endif

	return r;
}

uint32_t SystemResetReason(void)
{
	if (s_bSysPwrResetRead == false)
	{
		uint32_t reas = 0;

		if (SysPwrSdEnabled())
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

		s_SysPwrResetReason = SysPwrResetMap(reas);
		s_bSysPwrResetRead = true;
	}

	return s_SysPwrResetReason;
}

void SystemPowerOff(void)
{
	if (SysPwrSdEnabled())
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

int SystemRetRegCount(void)
{
	return SYS_PWR_NRF52_NB_RETREG;
}

uint32_t SystemRetRegRead(int Idx)
{
	if (Idx < 0 || Idx >= SYS_PWR_NRF52_NB_RETREG)
	{
		return 0;
	}

	uint32_t val = 0;

	if (SysPwrSdEnabled())
	{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
		(void)sd_power_gpregret_get((uint32_t)Idx, &val);
#endif
	}
	else
	{
		val = Idx == 0 ? NRF_POWER->GPREGRET : NRF_POWER->GPREGRET2;
	}

	return val & SYS_PWR_NRF52_RETREG_MSK;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if (Idx < 0 || Idx >= SYS_PWR_NRF52_NB_RETREG)
	{
		return false;
	}

	Val &= SYS_PWR_NRF52_RETREG_MSK;

	if (SysPwrSdEnabled())
	{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
		// The SoftDevice only sets or clears bits
		if (sd_power_gpregret_clr((uint32_t)Idx, SYS_PWR_NRF52_RETREG_MSK) != NRF_SUCCESS)
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

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	uint32_t ramsize = NRF_FICR->INFO.RAM * 1024UL;
	uintptr_t base;

	if (Addr >= SYS_PWR_NRF52_RAM_DATA && Addr - SYS_PWR_NRF52_RAM_DATA < ramsize)
	{
		base = SYS_PWR_NRF52_RAM_DATA;
	}
	else if (Addr >= SYS_PWR_NRF52_RAM_CODE && Addr - SYS_PWR_NRF52_RAM_CODE < ramsize)
	{
		base = SYS_PWR_NRF52_RAM_CODE;
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
	bool sd = SysPwrSdEnabled();

	while (off < end)
	{
		uint32_t blk;
		uint32_t sect;
		uint32_t next;

		if (off < SYS_PWR_NRF52_SMALL_END)
		{
			blk = off / (SYS_PWR_NRF52_SMALL_SECT * SYS_PWR_NRF52_SMALL_PERBLK);
			sect = (off / SYS_PWR_NRF52_SMALL_SECT) % SYS_PWR_NRF52_SMALL_PERBLK;
			next = (off / SYS_PWR_NRF52_SMALL_SECT + 1) * SYS_PWR_NRF52_SMALL_SECT;
		}
		else
		{
			blk = SYS_PWR_NRF52_SMALL_END / (SYS_PWR_NRF52_SMALL_SECT * SYS_PWR_NRF52_SMALL_PERBLK);
			sect = (off - SYS_PWR_NRF52_SMALL_END) / SYS_PWR_NRF52_LARGE_SECT;
			next = SYS_PWR_NRF52_SMALL_END + (sect + 1) * SYS_PWR_NRF52_LARGE_SECT;
		}

		if (blk >= SYS_PWR_NRF52_NB_RAMBLK)
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
