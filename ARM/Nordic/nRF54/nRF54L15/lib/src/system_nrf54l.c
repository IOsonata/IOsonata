/*

Copyright (c) 2009-2024 ARM Limited. All rights reserved.

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
#if 0
	// BLYSTL15
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 10, 100},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 70},
#else
	// Nordic DK
	// 32MHz, 8pF
	// 32.768kHz, 9pF
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 20, 80},
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 90},
#endif
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
    #ifdef __CORTEX_M
        #ifndef NRF_SKIP_CLOCK_CONFIGURATION
            #if defined(NRF_CONFIG_CPU_FREQ_MHZ) && (NRF_CONFIG_CPU_FREQ_MHZ==64)
                NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK64M;
            #elif defined(NRF_CONFIG_CPU_FREQ_MHZ) && (NRF_CONFIG_CPU_FREQ_MHZ==128)
                NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK128M;
            #elif defined(NRF_CONFIG_CPU_FREQ_MHZ)
                #error "Illegal CPU frequency set"
            #else
                NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK128M;
            #endif
        #endif

        #if !defined(NRF_TRUSTZONE_NONSECURE) && defined(__ARM_FEATURE_CMSE)
            #ifndef NRF_SKIP_TAMPC_SETUP
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

        #if !defined (NRF_DISABLE_FICR_TRIMCNF)
            /* Trimming of the device. Copy all the trimming values from FICR into the target addresses. Trim
               until one ADDR is not initialized. */
            uint32_t index = 0ul;
            for (index = 0ul; index < FICR_TRIMCNF_MaxCount && NRF_FICR_NS->TRIMCNF[index].ADDR != 0xFFFFFFFFul && NRF_FICR_NS->TRIMCNF[index].ADDR != 0x00000000ul; index++) {
            #if defined ( __ICCARM__ )
                /* IAR will complain about the order of volatile pointer accesses. */
                #pragma diag_suppress=Pa082
            #endif
            * ((volatile uint32_t*)NRF_FICR_NS->TRIMCNF[index].ADDR) = NRF_FICR_NS->TRIMCNF[index].DATA;
            #if defined ( __ICCARM__ )
                #pragma diag_default=Pa082
            #endif
            }
        #endif

        /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
        * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
        * operations are not used in your code. */
        
        /* Allow Non-Secure code to run FPU instructions.
         * If only the secure code should control FPU power state these registers should be configured accordingly in the secure application code. */
        SCB->NSACR |= (3UL << 10ul);

        #if (__FPU_USED == 1ul)
            SCB->CPACR |= (3UL << 20ul) | (3UL << 22ul);
            __DSB();
            __ISB();
        #endif

        #if !defined(NRF_TRUSTZONE_NONSECURE) && defined(__ARM_FEATURE_CMSE)
            #if defined(NRF_CONFIG_NFCT_PINS_AS_GPIOS)
                NRF_NFCT_S->PADCONFIG = (NFCT_PADCONFIG_ENABLE_Disabled << NFCT_PADCONFIG_ENABLE_Pos);
            #endif 

            /* Enable SWO trace functionality. If ENABLE_SWO is not defined, SWO pin will be used as GPIO (see Product
            Specification to see which one). */
            #if defined (ENABLE_SWO)
                        // Enable Trace And Debug peripheral
                NRF_TAD_S->ENABLE = TAD_ENABLE_ENABLE_Msk;
                NRF_TAD_S->CLOCKSTART = TAD_CLOCKSTART_START_Msk;

                // Set up Trace pad SPU firewall
                NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA0_PIN);

                // Configure trace port pad
                NRF_P0_S->PIN_CNF[TRACE_TRACEDATA0_PIN] = TRACE_PIN_CNF_VALUE;

                // Select trace pin
                NRF_TAD_S->PSEL.TRACEDATA0 = TRACE_TRACEDATA0_PIN;

                // Set trace port speed to 64 MHz
                NRF_TAD_S->TRACEPORTSPEED = TAD_TRACEPORTSPEED_TRACEPORTSPEED_64MHz;
            #endif

                /* Enable Trace functionality. If ENABLE_TRACE is not defined, TRACE pins will be used as GPIOs (see Product
                Specification to see which ones). */
            #if defined (ENABLE_TRACE)
                // Enable Trace And Debug peripheral
                NRF_TAD_S->ENABLE = TAD_ENABLE_ENABLE_Msk;
                NRF_TAD_S->CLOCKSTART = TAD_CLOCKSTART_START_Msk;

                // Set up Trace pads SPU firewall
                NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACECLK_PIN);
                NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA0_PIN);
                NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA1_PIN);
                NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA2_PIN);
                NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA3_PIN);

                // Configure trace port pads
                NRF_P0_S->PIN_CNF[TRACE_TRACECLK_PIN] = TRACE_PIN_CNF_VALUE;
                NRF_P0_S->PIN_CNF[TRACE_TRACEDATA0_PIN] = TRACE_PIN_CNF_VALUE;
                NRF_P0_S->PIN_CNF[TRACE_TRACEDATA1_PIN] = TRACE_PIN_CNF_VALUE;
                NRF_P0_S->PIN_CNF[TRACE_TRACEDATA2_PIN] = TRACE_PIN_CNF_VALUE;
                NRF_P0_S->PIN_CNF[TRACE_TRACEDATA3_PIN] = TRACE_PIN_CNF_VALUE;

                // Select trace pins
                NRF_TAD_S->PSEL.TRACECLK = TRACE_TRACECLK_PIN;
                NRF_TAD_S->PSEL.TRACEDATA0 = TRACE_TRACEDATA0_PIN;
                NRF_TAD_S->PSEL.TRACEDATA1 = TRACE_TRACEDATA1_PIN;
                NRF_TAD_S->PSEL.TRACEDATA2 = TRACE_TRACEDATA2_PIN;
                NRF_TAD_S->PSEL.TRACEDATA3 = TRACE_TRACEDATA3_PIN;

                // Set trace port speed to 64 MHz
                NRF_TAD_S->TRACEPORTSPEED = TAD_TRACEPORTSPEED_TRACEPORTSPEED_64MHz;
            #endif
        #endif

        #if !defined(NRF_TRUSTZONE_NONSECURE) && !defined (NRF_SKIP_GLITCHDETECTOR_DISABLE)
            /* Disable glitch detector */
            #if defined (GLITCHDET_GLITCHDETECTORS)
                NRF_GLITCHDET_S->GLITCHDETECTOR.CONFIG = (GLITCHDET_GLITCHDETECTOR_CONFIG_ENABLE_Disable << GLITCHDET_GLITCHDETECTOR_CONFIG_ENABLE_Pos);
            #else
                NRF_GLITCHDET_S->CONFIG = (GLITCHDET_CONFIG_ENABLE_Disable << GLITCHDET_CONFIG_ENABLE_Pos);
            #endif
        #endif
    #endif
	/* As specified in the nRF54L15 PS:
	 * CAPVALUE = (((CAPACITANCE-5.5)*(FICR->XOSC32MTRIM.SLOPE+791)) +
	 *              FICR->XOSC32MTRIM.OFFSET<<2)>>8;
	 * where CAPACITANCE is the desired total load capacitance value in pF,
	 * holding any value between 4.0 pF and 17.0 pF in 0.25 pF steps.
	 */
	uint32_t intcap = 0;
	if (g_McuOsc.CoreOsc.LoadCap > 0)
	{
		int32_t slope = (NRF_FICR->XOSC32MTRIM & FICR_XOSC32MTRIM_SLOPE_Msk) >> FICR_XOSC32MTRIM_SLOPE_Pos;
		slope = ((-(slope>>8))<<8) | slope;
		uint32_t offset = (NRF_FICR->XOSC32MTRIM & FICR_XOSC32MTRIM_OFFSET_Msk) >> FICR_XOSC32MTRIM_OFFSET_Pos;
		intcap = (uint32_t)(((g_McuOsc.CoreOsc.LoadCap - 55) * (slope + 791)) / 10 + (offset << 2))>>8;
	}
	NRF_OSCILLATORS->XOSC32M.CONFIG.INTCAP = intcap;

	intcap = 0;
	if (g_McuOsc.LowPwrOsc.LoadCap > 0)
	{
		/* As specified in the nRF54L15 PS:
		 * CAPVALUE = round( (CAPACITANCE - 4) * (FICR->XOSC32KTRIM.SLOPE + 0.765625 * 2^9)/(2^9)
		 *            + FICR->XOSC32KTRIM.OFFSET/(2^6) );
		 * where CAPACITANCE is the desired capacitor value in pF, holding any
		 * value between 4 pF and 18 pF in 0.5 pF steps.
		 */
		int32_t slope = (NRF_FICR->XOSC32KTRIM & FICR_XOSC32KTRIM_SLOPE_Msk) >> FICR_XOSC32KTRIM_SLOPE_Pos;
		slope = ((-(slope>>8))<<8) | slope;
		uint32_t offset = (NRF_FICR->XOSC32KTRIM & FICR_XOSC32KTRIM_OFFSET_Msk) >> FICR_XOSC32KTRIM_OFFSET_Pos;

		intcap = (((g_McuOsc.LowPwrOsc.LoadCap - 40UL) * (uint32_t)(slope + 392) + 5) / 5120) + (offset >> 6);
	}
	NRF_OSCILLATORS->XOSC32KI.INTCAP = intcap;
}

/*lint --flb "Leave library region" */
