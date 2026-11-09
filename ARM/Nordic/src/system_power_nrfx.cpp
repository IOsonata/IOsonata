/**-------------------------------------------------------------------------
@file	system_power_nrfx.cpp

@brief	Power functions of system_core_clock.h on the nRF53, nRF54L and nRF91

The nRF52 has its own, system_power_nrf52.cpp, for the SoftDevice paths.

Reset reason: RESETREAS of RESET, of POWER on the nRF91. It keeps adding
reasons until it is cleared, an empty value is a power on or brown out
reset. On the nRF5340 network core, MFORCEOFF, the core held off by the
application core and released, is reported as a power on. SECTAMPER of the
nRF54L has no SYSTEM_RESET_xxx reason and is not reported.

Power off is System OFF through REGULATORS. The nRF5340 network core has no
REGULATORS, power off is done by the application core, SystemPowerOff
returns there. On the nRF91 the modem must be shut down before. On the nRF54L
RESETREAS is cleared before System OFF, a reason left in it can wake the part
up at once, and the VPR retention bit (MEMCONF POWER[1].RET MEM0) is cleared
so the VPR stays in reset on wake up.

The retention registers are GPREGRET[0] and GPREGRET[1] of POWER, 8 bits
each.

RAM is in sections of the same size, grouped in blocks:
- nRF5340 application core: 8 blocks of 16 sections of 4 KB, VMC;
- nRF5340 network core: 4 blocks of 4 sections of 4 KB, VMC;
- nRF91: 8 blocks of 4 sections of 8 KB, VMC;
- nRF54L: sections of 32 KB, MEMCONF POWER[0] RET and RET2.
The RAM size comes from FICR INFO.RAM. VMC retention is off at reset, as on
the nRF52. MEMCONF retention is on for all RAM at reset, SystemRamRetain
records the sections and SystemPowerOff writes them to RET and RET2, the
other sections are not kept.

POWER is non secure on the nRF91, the Modem library uses it
(nrfx_config_nrf91.h). The nRF5340 application core and the nRF91 use the
secure REGULATORS and VMC.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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

#include "coredev/system_core_clock.h"

#if defined(NRF5340_XXAA_APPLICATION)
#define SYS_PWR_RESET				NRF_RESET_S
#define SYS_PWR_POWER				NRF_POWER_S
#define SYS_PWR_REGULATORS			NRF_REGULATORS_S
#define SYS_PWR_SYSTEMOFF			REGULATORS_SYSTEMOFF_SYSTEMOFF_Enter
#define SYS_PWR_VMC					NRF_VMC_S
#define SYS_PWR_FICR				NRF_FICR_S
#define SYS_PWR_RAM_BASE			0x20000000UL
#define SYS_PWR_RAM_SECT			4096UL
#define SYS_PWR_RAM_PERBLK			16UL
#elif defined(NRF5340_XXAA_NETWORK)
#define SYS_PWR_RESET				NRF_RESET_NS
#define SYS_PWR_POWER				NRF_POWER_NS
#define SYS_PWR_VMC					NRF_VMC_NS
#define SYS_PWR_FICR				NRF_FICR_NS
#define SYS_PWR_RAM_BASE			0x21000000UL
#define SYS_PWR_RAM_SECT			4096UL
#define SYS_PWR_RAM_PERBLK			4UL
#elif defined(NRF91_SERIES)
#define SYS_PWR_RESET				NRF_POWER_NS
#define SYS_PWR_POWER				NRF_POWER_NS
#define SYS_PWR_REGULATORS			NRF_REGULATORS_S
#define SYS_PWR_SYSTEMOFF			REGULATORS_SYSTEMOFF_SYSTEMOFF_Enable
#define SYS_PWR_VMC					NRF_VMC_S
#define SYS_PWR_FICR				NRF_FICR_S
#define SYS_PWR_RAM_BASE			0x20000000UL
#define SYS_PWR_RAM_SECT			8192UL
#define SYS_PWR_RAM_PERBLK			4UL
#elif defined(NRF54L_SERIES)
#define SYS_PWR_RESET				NRF_RESET
#define SYS_PWR_POWER				NRF_POWER
#define SYS_PWR_REGULATORS			NRF_REGULATORS
#define SYS_PWR_SYSTEMOFF			REGULATORS_SYSTEMOFF_SYSTEMOFF_Enter
#define SYS_PWR_MEMCONF				NRF_MEMCONF
#define SYS_PWR_FICR				NRF_FICR
#define SYS_PWR_RAM_BASE			0x20000000UL
#define SYS_PWR_RAM_SECT			32768UL
#define SYS_PWR_RAM_PERBLK			32UL		// One RET bit per section
#define SYS_PWR_NB_RAMBLK			1			// POWER[0], RAM sections
#define SYS_PWR_VPR_BLK				1			// POWER[1], VPR saved context
#else
#error "system_power_nrfx.cpp is for the nRF53, nRF54L and nRF91"
#endif

#define SYS_PWR_NB_RETREG			2
#define SYS_PWR_RETREG_MSK			0xFFUL

#ifdef SYS_PWR_VMC
#define SYS_PWR_NB_RAMBLK			(sizeof(SYS_PWR_VMC->RAM) / sizeof(SYS_PWR_VMC->RAM[0]))
#endif

typedef struct {
	uint32_t Msk;		// RESETREAS bit
	uint32_t Reason;	// SYSTEM_RESET_xxx
} SysPwrResetMap_t;

// RESETREAS bits of the part to reset reasons
static const SysPwrResetMap_t s_SysPwrResetMap[] = {
#ifdef RESET_RESETREAS_RESETPIN_Msk
	{ RESET_RESETREAS_RESETPIN_Msk, SYSTEM_RESET_PIN },
#endif
#ifdef RESET_RESETREAS_DOG0_Msk
	{ RESET_RESETREAS_DOG0_Msk, SYSTEM_RESET_WDT },
#endif
#ifdef RESET_RESETREAS_DOG1_Msk
	{ RESET_RESETREAS_DOG1_Msk, SYSTEM_RESET_WDT },
#endif
#ifdef RESET_RESETREAS_LDOG_Msk
	{ RESET_RESETREAS_LDOG_Msk, SYSTEM_RESET_WDT },
#endif
#ifdef RESET_RESETREAS_SREQ_Msk
	{ RESET_RESETREAS_SREQ_Msk, SYSTEM_RESET_SOFT },
#endif
#ifdef RESET_RESETREAS_LSREQ_Msk
	{ RESET_RESETREAS_LSREQ_Msk, SYSTEM_RESET_SOFT },
#endif
#ifdef RESET_RESETREAS_LOCKUP_Msk
	{ RESET_RESETREAS_LOCKUP_Msk, SYSTEM_RESET_LOCKUP },
#endif
#ifdef RESET_RESETREAS_LLOCKUP_Msk
	{ RESET_RESETREAS_LLOCKUP_Msk, SYSTEM_RESET_LOCKUP },
#endif
#ifdef RESET_RESETREAS_CTRLAP_Msk
	{ RESET_RESETREAS_CTRLAP_Msk, SYSTEM_RESET_DEBUG },
#endif
#ifdef RESET_RESETREAS_LCTRLAP_Msk
	{ RESET_RESETREAS_LCTRLAP_Msk, SYSTEM_RESET_DEBUG },
#endif
#ifdef RESET_RESETREAS_CTRLAPSOFT_Msk
	{ RESET_RESETREAS_CTRLAPSOFT_Msk, SYSTEM_RESET_DEBUG },
#endif
#ifdef RESET_RESETREAS_CTRLAPHARD_Msk
	{ RESET_RESETREAS_CTRLAPHARD_Msk, SYSTEM_RESET_DEBUG },
#endif
#ifdef RESET_RESETREAS_CTRLAPPIN_Msk
	{ RESET_RESETREAS_CTRLAPPIN_Msk, SYSTEM_RESET_DEBUG },
#endif
#ifdef RESET_RESETREAS_DIF_Msk
	{ RESET_RESETREAS_DIF_Msk, SYSTEM_RESET_DEBUG },
#endif
#ifdef RESET_RESETREAS_OFF_Msk
	{ RESET_RESETREAS_OFF_Msk, SYSTEM_RESET_WAKE_PIN },
#endif
#ifdef RESET_RESETREAS_LPCOMP_Msk
	{ RESET_RESETREAS_LPCOMP_Msk, SYSTEM_RESET_WAKE_COMP },
#endif
#ifdef RESET_RESETREAS_NFC_Msk
	{ RESET_RESETREAS_NFC_Msk, SYSTEM_RESET_WAKE_NFC },
#endif
#ifdef RESET_RESETREAS_VBUS_Msk
	{ RESET_RESETREAS_VBUS_Msk, SYSTEM_RESET_WAKE_USB },
#endif
#ifdef RESET_RESETREAS_GRTC_Msk
	{ RESET_RESETREAS_GRTC_Msk, SYSTEM_RESET_WAKE_TIMER },
#endif
#ifdef RESET_RESETREAS_MFORCEOFF_Msk
	{ RESET_RESETREAS_MFORCEOFF_Msk, SYSTEM_RESET_POWERON },
#endif
#if defined(NRF91_SERIES)
	{ POWER_RESETREAS_RESETPIN_Msk, SYSTEM_RESET_PIN },
	{ POWER_RESETREAS_DOG_Msk, SYSTEM_RESET_WDT },
	{ POWER_RESETREAS_SREQ_Msk, SYSTEM_RESET_SOFT },
	{ POWER_RESETREAS_LOCKUP_Msk, SYSTEM_RESET_LOCKUP },
	{ POWER_RESETREAS_CTRLAP_Msk, SYSTEM_RESET_DEBUG },
	{ POWER_RESETREAS_DIF_Msk, SYSTEM_RESET_DEBUG },
	{ POWER_RESETREAS_OFF_Msk, SYSTEM_RESET_WAKE_PIN },
#endif
};

static bool s_bSysPwrResetRead = false;
static uint32_t s_SysPwrResetReason = 0;
#ifdef SYS_PWR_MEMCONF
static uint32_t s_SysPwrRamRet = 0;		// Sections kept in System OFF, one bit each
#endif

uint32_t SystemResetReason(void)
{
	if (s_bSysPwrResetRead == false)
	{
		uint32_t reas = SYS_PWR_RESET->RESETREAS;
		uint32_t r = 0;

		SYS_PWR_RESET->RESETREAS = reas;

		if (reas == 0)
		{
			r = SYSTEM_RESET_POWERON;
		}
		for (size_t i = 0; i < sizeof(s_SysPwrResetMap) / sizeof(s_SysPwrResetMap[0]); i++)
		{
			if (reas & s_SysPwrResetMap[i].Msk)
			{
				r |= s_SysPwrResetMap[i].Reason;
			}
		}

		s_SysPwrResetReason = r;
		s_bSysPwrResetRead = true;
	}

	return s_SysPwrResetReason;
}

void SystemPowerOff(void)
{
#ifdef SYS_PWR_MEMCONF
	uint32_t nbsect = SYS_PWR_FICR->INFO.RAM * 1024UL / SYS_PWR_RAM_SECT;
	uint32_t msk = nbsect < 32 ? (1UL << nbsect) - 1 : 0xFFFFFFFFUL;

	SYS_PWR_MEMCONF->POWER[0].RET = (SYS_PWR_MEMCONF->POWER[0].RET & ~msk) | s_SysPwrRamRet;
#ifdef MEMCONF_POWER_RET2_MEM0_Msk
	SYS_PWR_MEMCONF->POWER[0].RET2 = (SYS_PWR_MEMCONF->POWER[0].RET2 & ~msk) | s_SysPwrRamRet;
#endif
	SYS_PWR_MEMCONF->POWER[SYS_PWR_VPR_BLK].RET &= ~MEMCONF_POWER_RET_MEM0_Msk;
	SYS_PWR_RESET->RESETREAS = 0xFFFFFFFFUL;
#endif

#ifdef SYS_PWR_REGULATORS
	SYS_PWR_REGULATORS->SYSTEMOFF = SYS_PWR_SYSTEMOFF;
	__DSB();

	// Only reached when System OFF is emulated
	while (1)
	{
		__WFE();
	}
#endif
}

int SystemRetRegCount(void)
{
	return SYS_PWR_NB_RETREG;
}

uint32_t SystemRetRegRead(int Idx)
{
	if (Idx < 0 || Idx >= SYS_PWR_NB_RETREG)
	{
		return 0;
	}

	return SYS_PWR_POWER->GPREGRET[Idx] & SYS_PWR_RETREG_MSK;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if (Idx < 0 || Idx >= SYS_PWR_NB_RETREG)
	{
		return false;
	}

	SYS_PWR_POWER->GPREGRET[Idx] = Val & SYS_PWR_RETREG_MSK;

	return true;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	uint32_t ramsize = SYS_PWR_FICR->INFO.RAM * 1024UL;

	if (Addr < SYS_PWR_RAM_BASE || Addr - SYS_PWR_RAM_BASE >= ramsize)
	{
		return false;
	}

	uint32_t off = (uint32_t)(Addr - SYS_PWR_RAM_BASE);

	if (Size > ramsize - off)
	{
		return false;
	}

	uint32_t end = off + Size;

	while (off < end)
	{
		uint32_t unit = off / SYS_PWR_RAM_SECT;
		uint32_t blk = unit / SYS_PWR_RAM_PERBLK;
		uint32_t sect = unit % SYS_PWR_RAM_PERBLK;

		if (blk >= SYS_PWR_NB_RAMBLK)
		{
			return false;
		}

#ifdef SYS_PWR_VMC
		uint32_t msk = VMC_RAM_POWERSET_S0RETENTION_Msk << sect;

		if (bRetain)
		{
			SYS_PWR_VMC->RAM[blk].POWERSET = msk;
		}
		else
		{
			SYS_PWR_VMC->RAM[blk].POWERCLR = msk;
		}
#else
		if (bRetain)
		{
			s_SysPwrRamRet |= 1UL << sect;
		}
		else
		{
			s_SysPwrRamRet &= ~(1UL << sect);
		}
#endif

		off = (unit + 1) * SYS_PWR_RAM_SECT;
	}

	return true;
}
