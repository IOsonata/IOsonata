/*

Copyright (c) 2009-2023 ARM Limited. All rights reserved.

    SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the License); you may
not use this file except in compliance with the License.
You may obtain a copy of the License at

    www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an AS IS BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

NOTICE: This file has been modified by Nordic Semiconductor ASA.

*/

/* NOTE: Template files (including this one) are application specific and therefore expected to
   be copied into the application project folder prior to its use! */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "system_nrf54l.h"
#include "system_nrf54l_approtect.h"
#include "system_config_sau.h"

#include "coredev/system_core_clock.h"
/*lint ++flb "Enter library region" */

#define __SYSTEM_CLOCK_DEFAULT      (64000000ul)

#if defined ( __CC_ARM ) || defined ( __GNUC__ )
    uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_DEFAULT;
#elif defined ( __ICCARM__ )
    __root uint32_t SystemCoreClock = __SYSTEM_CLOCK_DEFAULT;
#endif    

// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 20, 0},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 0},
	.bUSBClk = false
};

void SystemCoreClockUpdate(void)
{
    switch(NRF_OSCILLATORS->PLL.CURRENTFREQ)
    {
        case OSCILLATORS_PLL_CURRENTFREQ_CURRENTFREQ_CK64M:
            SystemCoreClock = 64000000ul;
            break;
        case OSCILLATORS_PLL_CURRENTFREQ_CURRENTFREQ_CK128M:
            SystemCoreClock = 128000000ul;
            break;
    }
}

void SystemInit(void)
{

#if !defined(NRF_TRUSTZONE_NONSECURE)

#if !defined(NRF_APPLY_FICR_TRIMS)
    /* TEMPORARY: Apply trims to PDKs. */
    *((uint32_t *)0x50120550) = 0x007FF3B8;
    *((uint32_t *)0x50120644) = 0x3118CCC0;
    *((uint32_t *)0x50120640) = 0x000FCD8C;
    *((uint32_t *)0x50120824) = 0x00000FFF;
    *((uint32_t *)0x50120A08) = 0x000000AA;
    *((uint32_t *)0x500CF900) = 0x00001101;
    *((uint32_t *)0x50120908) = 0x00000005;
    *((uint32_t *)0x50106624) = 0x00040008;
    *((uint32_t *)0x5008A844) = 0x0000041A;
    *((uint32_t *)0x5008A74C) = 0x70000001;
    *((uint32_t *)0x5008A848) = 0x00000168;
    *((uint32_t *)0x5008A8A0) = 0x00580F0D;
#else
    /* Trimming of the device. Copy all the trimming values from FICR into the target addresses. Trim until one ADDR is not initialized. */
    uint32_t index = 0;
    for (index = 0; (index < FICR_TRIMCNF_MaxCount) && ((uint32_t)NRF_FICR->TRIMCNF[index].ADDR != 0xFFFFFFFFul); index++){
        #if defined ( __ICCARM__ )
            /* IAR will complain about the order of volatile pointer accesses. */
            #pragma diag_suppress=Pa082
        #endif
        *((volatile uint32_t *)NRF_FICR->TRIMCNF[index].ADDR) = NRF_FICR->TRIMCNF[index].DATA;
        #if defined ( __ICCARM__ )
            #pragma diag_default=Pa082
        #endif
    }
#endif
//            INTCAP = (((CAPACITANCE-5.5)*(FICR->XOSC32MTRIM.SLOPE+791)) +
//                       FICR->XOSC32MTRIM.OFFSET<<2)>>8;
	uint32_t intcap = 0;
	if (g_McuOsc.CoreOsc.LoadCap > 0)
	{
		uint32_t slope = (NRF_FICR->XOSC32MTRIM & FICR_XOSC32MTRIM_SLOPE_Msk) >> FICR_XOSC32MTRIM_SLOPE_Pos;
		uint32_t offset = (NRF_FICR->XOSC32MTRIM & FICR_XOSC32MTRIM_OFFSET_Msk) >> FICR_XOSC32MTRIM_OFFSET_Pos;
		intcap = (uint32_t)((((float)g_McuOsc.CoreOsc.LoadCap - 5.5) * (slope + 791)) + (offset << 2))>>8;
	}
	NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP = intcap;

#endif

    #ifdef __CORTEX_M
        #ifndef NRF_SKIP_CLOCK_CONFIGURATION
            #if defined(CONFIG_CPU_FREQ_MHZ) && (CONFIG_CPU_FREQ_MHZ==64)
            NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK64M;
            #elif defined(CONFIG_CPU_FREQ_MHZ) && (CONFIG_CPU_FREQ_MHZ==128)
            NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK128M;
            #elif defined(CONFIG_CPU_FREQ_MHZ)
                #error "Illegal CPU frequency set"
            #else
            NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK128M;

            #endif
        #endif

        #if !defined(NRF_TRUSTZONE_NONSECURE) && defined(__ARM_FEATURE_CMSE)
            #ifndef NRF_SKIP_TAMPC_CONFIGURATION
                nrf54l_handle_approtect();
            #endif
            #if defined(__FPU_PRESENT) && __FPU_PRESENT
                /* Allow Non-Secure code to run FPU instructions.
                * If only the secure code should control FPU power state these registers should be configured accordingly in the secure application code. */
                SCB->NSACR |= (3UL << 10ul);
            #endif

            #ifndef NRF_SKIP_SAU_CONFIGURATION   
                configure_default_sau();
            #endif          
        #endif

        /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
        * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
        * operations are not used in your code. */
        #if (__FPU_USED == 1ul)
            SCB->CPACR |= (3UL << 20ul) | (3UL << 22ul);
            __DSB();
            __ISB();
        #endif
    #endif

    /* Configure new vector table offset register if defined. */
#ifdef NRF_VTOR_CONFIG
    SCB->VTOR = NRF_VTOR_CONFIG;
#endif
}

/*lint --flb "Leave library region" */
