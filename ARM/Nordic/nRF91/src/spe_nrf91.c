/**-------------------------------------------------------------------------
@file	spe_nrf91.c

@brief	nRF91 secure stage: start a non secure application

Programs the SPU from the TrustZone layout of the linker scripts and branches
to the non secure application. See spe_nrf91.h.

The flash region size is the flash size, from the layout, over the number of
SPU flash regions (32 KB on a 1 MB part). The RAM region size is the MDK one.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#if !defined(__ARM_FEATURE_CMSE) || (__ARM_FEATURE_CMSE != 3)
#error "spe_nrf91.c is the secure stage: build it with -mcmse"
#endif

#ifdef NRF_TRUSTZONE_NONSECURE
#error "spe_nrf91.c is the secure stage, it is not part of a non secure build"
#endif

#include <arm_cmse.h>

#include "spe_nrf91.h"

/** @addtogroup TrustZone
  * @{
  */

#define SPE_NRF91_ARRAY_CNT(a)		(sizeof(a) / sizeof((a)[0]))

// Non secure entry, called with the security state change and the register
// clearing the compiler emits for it.
typedef void __attribute__((cmse_nonsecure_call)) nRF91SpeNsEntry_t(void);

// TrustZone layout, from tz_layout_nrf91xx.ld
extern const uint8_t __tz_s_flash_start[];
extern const uint8_t __tz_ns_flash_start[];
extern const uint8_t __tz_ns_flash_end[];
extern const uint8_t __tz_s_ram_start[];
extern const uint8_t __tz_ns_ram_start[];
extern const uint8_t __tz_ns_ram_end[];

static bool nRF91SpeSecPeriph(const nRF91SpeCfg_t * const pCfg, int Id)
{
	if (pCfg == NULL || pCfg->pSecPeriph == NULL)
	{
		return false;
	}

	for (int i = 0; i < pCfg->NbSecPeriph; i++)
	{
		if (pCfg->pSecPeriph[i] == Id)
		{
			return true;
		}
	}

	return false;
}

bool nRF91SpeStart(const nRF91SpeCfg_t * const pCfg)
{
	NRF_SPU_Type * const spu = NRF_SPU_S;
	const uintptr_t flashstart = (uintptr_t)__tz_s_flash_start;
	const uintptr_t nsflash = (uintptr_t)__tz_ns_flash_start;
	const uintptr_t flashend = (uintptr_t)__tz_ns_flash_end;
	const uintptr_t ramstart = (uintptr_t)__tz_s_ram_start;
	const uintptr_t nsram = (uintptr_t)__tz_ns_ram_start;
	const uintptr_t nsramend = (uintptr_t)__tz_ns_ram_end;
	const uint32_t nbflash = SPE_NRF91_ARRAY_CNT(spu->FLASHREGION);
	const uint32_t nbram = SPE_NRF91_ARRAY_CNT(spu->RAMREGION);
	const uint32_t flashsize = (uint32_t)((flashend - flashstart) / nbflash);
	const uint32_t *vec = (const uint32_t *)nsflash;
	const uint32_t sp = vec[0];
	const uint32_t pc = vec[1];

	// The application vector table: initial stack pointer in non secure RAM,
	// below the FICR copy, 8 byte aligned, reset handler a Thumb address in
	// non secure flash.
	if (sp <= nsram || sp > nsramend || (sp & 7U) != 0 ||
		(pc & 1U) == 0 || (pc & ~1U) < nsflash || (pc & ~1U) >= flashend)
	{
		return false;
	}

	// Nothing secure runs once the application has started
	for (uint32_t i = 0; i < SPE_NRF91_ARRAY_CNT(NVIC->ICER); i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFFUL;
		NVIC->ICPR[i] = 0xFFFFFFFFUL;
	}
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

	// Flash and RAM from the non secure start to the end are non secure,
	// readable, writable and executable.
	for (uint32_t i = 0; i < nbflash; i++)
	{
		if (flashstart + i * flashsize >= nsflash)
		{
			spu->FLASHREGION[i].PERM = SPU_FLASHREGION_PERM_EXECUTE_Msk |
									   SPU_FLASHREGION_PERM_WRITE_Msk |
									   SPU_FLASHREGION_PERM_READ_Msk;
		}
	}

	for (uint32_t i = 0; i < nbram; i++)
	{
		if (ramstart + i * SPU_RAMREGION_SIZE >= nsram)
		{
			spu->RAMREGION[i].PERM = SPU_RAMREGION_PERM_EXECUTE_Msk |
									 SPU_RAMREGION_PERM_WRITE_Msk |
									 SPU_RAMREGION_PERM_READ_Msk;
		}
	}

	// Every peripheral whose security can be chosen goes non secure with its
	// DMA and its interrupt, except the ones the configuration keeps secure.
	// The interrupt of an always non secure peripheral (GPIOTE1, FPU) goes to
	// the application too. The peripheral id is its interrupt number.
	for (int id = 0; id < (int)SPE_NRF91_ARRAY_CNT(spu->PERIPHID); id++)
	{
		uint32_t perm = spu->PERIPHID[id].PERM;
		uint32_t map = (perm & SPU_PERIPHID_PERM_SECUREMAPPING_Msk) >>
					   SPU_PERIPHID_PERM_SECUREMAPPING_Pos;

		if ((perm & SPU_PERIPHID_PERM_PRESENT_Msk) == 0)
		{
			continue;
		}

		if (map == SPU_PERIPHID_PERM_SECUREMAPPING_NonSecure)
		{
			NVIC_SetTargetState((IRQn_Type)id);
		}
		else if ((map == SPU_PERIPHID_PERM_SECUREMAPPING_UserSelectable ||
				  map == SPU_PERIPHID_PERM_SECUREMAPPING_Split) &&
				 !nRF91SpeSecPeriph(pCfg, id))
		{
			spu->PERIPHID[id].PERM = perm & ~(SPU_PERIPHID_PERM_SECATTR_Msk |
											  SPU_PERIPHID_PERM_DMASEC_Msk);
			NVIC_SetTargetState((IRQn_Type)id);
		}
	}

	spu->GPIOPORT[0].PERM = pCfg != NULL ? pCfg->SecGpio : 0;
	spu->DPPI[0].PERM = pCfg != NULL ? pCfg->SecDppi : 0;

	// Attribution comes from the SPU alone, which SystemInit also sets when
	// built with -mcmse.
	SAU->CTRL |= SAU_CTRL_ALLNS_Msk;

	// BusFault, HardFault and NMI to the application, SecureFault reported
	// as such.
	SCB->AIRCR = (SCB->AIRCR & ~SCB_AIRCR_VECTKEY_Msk) |
				 (0x05FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_BFHFNMINS_Msk;
	SCB->SHCSR |= SCB_SHCSR_SECUREFAULTENA_Msk;

	// Floating point for the application
	SCB->NSACR |= SCB_NSACR_CP10_Msk | SCB_NSACR_CP11_Msk;

	SCB_NS->VTOR = (uint32_t)nsflash;
	__TZ_set_MSP_NS(sp);
	__TZ_set_CONTROL_NS(0);

	__DSB();
	__ISB();

	nRF91SpeNsEntry_t *entry = (nRF91SpeNsEntry_t *)cmse_nsfptr_create((void *)pc);

	entry();

	// The application does not return
	while (1)
	{
		__WFE();
	}
}

/** @} End of group TrustZone */
