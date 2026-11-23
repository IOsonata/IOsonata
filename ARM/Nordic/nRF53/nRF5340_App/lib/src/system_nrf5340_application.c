/*

Copyright (c) 2009-2026 ARM Limited. All rights reserved.

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
#include "nrf_peripherals.h"
#include "system_nrf53.h"
#include "nrf53_erratas.h"
#include "system_nrf53_approtect.h"

#include "coredev/system_core_clock.h"
#include "idelay.h"
/*lint ++flb "Enter library region" */

void SystemStoreFICRNS(void);
void SystemLockFICRNS(void);


/* IOsonata starts the nRF5340 application core at 128 MHz. */
#define __SYSTEM_CLOCK_MAX      (128000000UL)
#define __SYSTEM_CLOCK_DEFAULT  (128000000UL)

#define TRACE_PIN_CNF_VALUE (   (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | \
                                (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | \
                                (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | \
                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | \
                                (GPIO_PIN_CNF_MCUSEL_TND << GPIO_PIN_CNF_MCUSEL_Pos))

#define TRACE_TRACECLK_PIN   TAD_PSEL_TRACECLK_PIN_Traceclk
#define TRACE_TRACEDATA0_PIN TAD_PSEL_TRACEDATA0_PIN_Tracedata0
#define TRACE_TRACEDATA1_PIN TAD_PSEL_TRACEDATA1_PIN_Tracedata1
#define TRACE_TRACEDATA2_PIN TAD_PSEL_TRACEDATA2_PIN_Tracedata2
#define TRACE_TRACEDATA3_PIN TAD_PSEL_TRACEDATA3_PIN_Tracedata3

// Calibrated loop iterations per second.
extern uint32_t SystemMicroSecLoopCnt;
uint32_t SystemnsDelayFactor = 1000000UL;

#if defined ( __CC_ARM ) || defined ( __GNUC__ )
    uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_DEFAULT;
#elif defined ( __ICCARM__ )
    __root uint32_t SystemCoreClock = __SYSTEM_CLOCK_DEFAULT;
#endif

// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
	.CoreOsc = { OSC_TYPE_XTAL,	32000000, 20, 0 },
	.LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 0 },
	.bUSBClk = false
};

uint32_t SystemCoreClockGet(void)
{
	SystemCoreClockUpdate();
	return SystemCoreClock;
}

// The application core changes HCLK128M only. Peripheral PCLK domains
// remain independent of this divider.
#if defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_APP_CLOCK	NRF_CLOCK_NS
#else
#define NRF_APP_CLOCK	NRF_CLOCK_S
#endif

#define NRF_APP_HCLK_PERFORMANCE	(CLOCK_HFCLKCTRL_HCLK_Div1 << CLOCK_HFCLKCTRL_HCLK_Pos)
#define NRF_APP_HCLK_LOW_POWER	(CLOCK_HFCLKCTRL_HCLK_Div2 << CLOCK_HFCLKCTRL_HCLK_Pos)

// Runtime delays use calibrated assembly loops, never DWT.
// Keep separate measured defaults for 128 MHz and 64 MHz; any intermediate
// CPU rate uses the cached loop cost without enabling trace hardware.
static ArmDelayCalibration_t s_DelayCalibration;

void SystemCoreClockUpdate(void)
{
	uint32_t cpuHz;

#if defined(NRF_TRUSTZONE_NONSECURE)
	cpuHz = __SYSTEM_CLOCK_MAX >>
		(NRF_CLOCK_NS->HFCLKCTRL & CLOCK_HFCLKCTRL_HCLK_Msk);
#else
	cpuHz = __SYSTEM_CLOCK_MAX >>
		(NRF_CLOCK_S->HFCLKCTRL & CLOCK_HFCLKCTRL_HCLK_Msk);
#endif
	SystemCoreClock = cpuHz;

	// Refresh the runtime assembly-loop factors whenever HCLK changes.
	// This path never enables DWT or debug/trace clocks.
	if (s_DelayCalibration.UsCycles == 0U ||
		s_DelayCalibration.CpuHz != cpuHz)
	{
		if (cpuHz == __SYSTEM_CLOCK_DEFAULT)
		{
			// Aligned countdown: HF Timer measured 76,932 us for
			// 100 ms at 26 cycles; 20 cycles matches 6,399,256 loops/s.
			(void)ArmDelayClockInit(&s_DelayCalibration, cpuHz, 20U, 3U);
		}
		else if (cpuHz == (__SYSTEM_CLOCK_MAX / 2U))
		{
			// Aligned countdown: HF Timer measured 75,881 us for
			// 100 ms at 29 cycles; 22 cycles matches 2,908,364 loops/s.
			(void)ArmDelayClockInit(&s_DelayCalibration, cpuHz, 22U, 3U);
		}
		else
		{
			// Retain a proportional fallback for other supported speeds.
			(void)ArmDelayClockUpdate(&s_DelayCalibration, cpuHz);
		}
	}
}

/**
 * @brief Select the nRF5340 application CPU's running power mode.
 *
 * The HFCLKCTRL divider affects HCLK128M, not the independent PCLK clocks.
 * No LFCLK/HFXO ownership changes or peripheral resets are needed.
 */
bool SystemPowerModeSet(SYSTEM_POWER_MODE Mode)
{
	uint32_t requested;
	uint32_t current;
	uint32_t settle;

	if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
	{
		return false;
	}

	switch (Mode)
	{
		case SYSTEM_POWER_MODE_PERFORMANCE:
			requested = NRF_APP_HCLK_PERFORMANCE;
			break;
		case SYSTEM_POWER_MODE_LOW_POWER:
			requested = NRF_APP_HCLK_LOW_POWER;
			break;
		default:
			return false;
	}

	current = NRF_APP_CLOCK->HFCLKCTRL & CLOCK_HFCLKCTRL_HCLK_Msk;
	if (current == requested)
	{
		return true;
	}
	if (current != NRF_APP_HCLK_PERFORMANCE &&
		current != NRF_APP_HCLK_LOW_POWER)
	{
		return false;
	}

	// Preserve fields other than the CPU HCLK divider.
	NRF_APP_CLOCK->HFCLKCTRL =
		(NRF_APP_CLOCK->HFCLKCTRL & ~CLOCK_HFCLKCTRL_HCLK_Msk) | requested;
	__DSB();
	__ISB();

	// The clock divider takes up to several microseconds to settle.
	// This CPU-local loop is independent of Timer, RTC and UART.
	for (settle = 0U; settle < 4096U; settle++)
	{
		__NOP();
	}

	if ((NRF_APP_CLOCK->HFCLKCTRL & CLOCK_HFCLKCTRL_HCLK_Msk) != requested)
	{
		return false;
	}

	SystemCoreClockUpdate();
	return SystemCoreClock ==
		(Mode == SYSTEM_POWER_MODE_PERFORMANCE ? 128000000UL : 64000000UL);
}

SYSTEM_POWER_MODE SystemPowerModeGet(void)
{
	return (NRF_APP_CLOCK->HFCLKCTRL & CLOCK_HFCLKCTRL_HCLK_Msk) ==
		NRF_APP_HCLK_LOW_POWER ?
		SYSTEM_POWER_MODE_LOW_POWER : SYSTEM_POWER_MODE_PERFORMANCE;
}

void SystemInit(void)
{
    #if !defined(NRF_TRUSTZONE_NONSECURE)
        /* Perform Secure-mode initialization routines. */

        /* Set all ARM SAU regions to NonSecure if TrustZone extensions are enabled.
        * Nordic SPU should handle Secure Attribution tasks */
        #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
            SAU->CTRL |= (1ul << SAU_CTRL_ALLNS_Pos);
        #endif

        #if NRF53_ERRATA_97_ENABLE_WORKAROUND
            /* Workaround for Errata 97 "ERASEPROTECT, APPROTECT, or startup problems" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_97())
            {
                if (*((volatile uint32_t *)0x50004A20ul) == 0ul)
                {
                    *((volatile uint32_t *)0x50004A20ul) = 0xDul;
                    *((volatile uint32_t *)0x5000491Cul) = 0x1ul;
                    *((volatile uint32_t *)0x5000491Cul) = 0x0ul;
                }
            }
        #endif

        /* Trimming of the device. Copy all the trimming values from FICR into the target addresses. Trim
         until one ADDR is not initialized. */
        uint32_t index = 0ul;
        for (index = 0ul; index < 32ul && NRF_FICR_S->TRIMCNF[index].ADDR != 0xFFFFFFFFul; index++){
            #if defined ( __ICCARM__ )
                /* IAR will complain about the order of volatile pointer accesses. */
                #pragma diag_suppress=Pa082
            #endif
            *((volatile uint32_t *)NRF_FICR_S->TRIMCNF[index].ADDR) = NRF_FICR_S->TRIMCNF[index].DATA;
            #if defined ( __ICCARM__ )
                #pragma diag_default=Pa082
            #endif
        }

        #if NRF53_ERRATA_64_ENABLE_WORKAROUND
            /* errata 64 must be before errata 42, as errata 42 is dependant on the changes in errata 64*/
            /* Workaround for Errata 64 "VREGMAIN has invalid configuration when CPU is running at 128 MHz" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_64())
            {
                *((volatile uint32_t *)0x5000470Cul) = 0x29ul;
                *((volatile uint32_t *)0x5000473Cul) = 0x3ul;
            }
        #endif


        #if NRF53_ERRATA_42_ENABLE_WORKAROUND
            /* Workaround for Errata 42 "Reset value of HFCLKCTRL is invalid" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_42())
            {
                *((volatile uint32_t *)0x50039530ul) = 0xBEEF0044ul;
                NRF_CLOCK_S->HFCLKCTRL = CLOCK_HFCLKCTRL_HCLK_Div2 << CLOCK_HFCLKCTRL_HCLK_Pos;
            }
        #endif

        #if NRF53_ERRATA_46_ENABLE_WORKAROUND
            /* Workaround for Errata 46 "Higher power consumption of LFRC" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_46())
            {
                *((volatile uint32_t *)0x5003254Cul) = 0ul;
            }
        #endif


        #if NRF53_ERRATA_49_ENABLE_WORKAROUND
            /* Workaround for Errata 49 "SLEEPENTER and SLEEPEXIT events asserted after pin reset" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_49())
            {
                if (NRF_RESET_S->RESETREAS & RESET_RESETREAS_RESETPIN_Msk)
                {
                    NRF_POWER_S->EVENTS_SLEEPENTER = 0ul;
                    NRF_POWER_S->EVENTS_SLEEPEXIT = 0ul;
                }
            }
        #endif

        #if NRF53_ERRATA_55_ENABLE_WORKAROUND
            /* Workaround for Errata 55 "Bits in RESETREAS are set when they should not be" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_55())
            {
                if (NRF_RESET_S->RESETREAS & RESET_RESETREAS_RESETPIN_Msk){
                    NRF_RESET_S->RESETREAS = ~RESET_RESETREAS_RESETPIN_Msk;
                }
            }
        #endif


        #if NRF53_ERRATA_69_ENABLE_WORKAROUND
            /* Workaround for Errata 69 "VREGMAIN configuration is not retained in System OFF" found at the Errata document
               for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf53_errata_69())
            {
                *((volatile uint32_t *)0x5000470Cul) =0x65ul;
            }
        #endif

        #if NRF53_ERRATA_140_ENABLE_WORKAROUND
            if (nrf53_errata_140())
            {
                if (*(volatile uint32_t *)0x50032420ul & 0x80000000ul)
                {
                    /* Reset occured during calibration */
                    NRF_CLOCK_S->LFCLKSRC = CLOCK_LFCLKSRC_SRC_LFSYNT;
                    NRF_CLOCK_S->TASKS_LFCLKSTART = 1ul;
                    while (NRF_CLOCK_S->EVENTS_LFCLKSTARTED == 0ul) {}
                    NRF_CLOCK_S->EVENTS_LFCLKSTARTED = 0ul;
                    NRF_CLOCK_S->TASKS_LFCLKSTOP = 1ul;
                    NRF_CLOCK_S->LFCLKSRC = CLOCK_LFCLKSRC_SRC_LFRC;
                }
            }
        #endif

        #if NRF53_ERRATA_160_ENABLE_WORKAROUND
            if (nrf53_errata_160())
            {
                *((volatile uint32_t *)0x5000470Cul) = 0x7Eul;
                *((volatile uint32_t *)0x5000493Cul) = 0x7Eul;
                *((volatile uint32_t *)0x50002118ul) = 0x7Ful;
                *((volatile uint32_t *)0x50039E04ul) = 0x0ul;
                *((volatile uint32_t *)0x50039E08ul) = 0x0ul;
                *((volatile uint32_t *)0x50101110ul) = 0x0ul;
                *((volatile uint32_t *)0x50002124ul) = 0x0ul;
                *((volatile uint32_t *)0x5000212Cul) = 0x0ul;
                *((volatile uint32_t *)0x502012A0ul) = 0x0ul;
            }
        #endif

        #if !defined(NRF_SKIP_FICR_NS_COPY_TO_RAM)
            SystemStoreFICRNS();
        #endif
        
        #if (defined(CONFIG_NFCT_PINS_AS_GPIOS) || defined(NRF_CONFIG_NFCT_PINS_AS_GPIOS))

            if ((NRF_UICR_S->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
            {
                NRF_NVMC_S->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;

                while (NRF_NVMC_S->READY == NVMC_READY_READY_Busy);
                NRF_UICR_S->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;

                while (NRF_NVMC_S->READY == NVMC_READY_READY_Busy);
                NRF_NVMC_S->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;

                while (NRF_NVMC_S->READY == NVMC_READY_READY_Busy);
                NVIC_SystemReset();
            }

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
            NRF_P0_S->PIN_CNF[TRACE_TRACECLK_PIN] =   TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA0_PIN] = TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA1_PIN] = TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA2_PIN] = TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA3_PIN] = TRACE_PIN_CNF_VALUE;

            // Select trace pins
            NRF_TAD_S->PSEL.TRACECLK   = TRACE_TRACECLK_PIN;
            NRF_TAD_S->PSEL.TRACEDATA0 = TRACE_TRACEDATA0_PIN;
            NRF_TAD_S->PSEL.TRACEDATA1 = TRACE_TRACEDATA1_PIN;
            NRF_TAD_S->PSEL.TRACEDATA2 = TRACE_TRACEDATA2_PIN;
            NRF_TAD_S->PSEL.TRACEDATA3 = TRACE_TRACEDATA3_PIN;

            // Set trace port speed to 64 MHz
            NRF_TAD_S->TRACEPORTSPEED = TAD_TRACEPORTSPEED_TRACEPORTSPEED_64MHz;

        #endif

        /* Allow Non-Secure code to run FPU instructions.
         * If only the secure code should control FPU power state these registers should be configured accordingly in the secure application code. */
        SCB->NSACR |= (3UL << 10ul);

        /* Handle fw-branch APPROTECT setup. */
        nrf53_handle_approtect();

    #endif

    /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
    * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
    * operations are not used in your code. */
    #if (__FPU_USED == 1ul)
        SCB->CPACR |= (3UL << 20ul) | (3UL << 22ul);
        __DSB();
        __ISB();
    #endif

	// IOsonata boots the application core at its maximum 128 MHz speed.
	// The 64 MHz reset divider is available later through SystemPowerModeSet.
	NRF_APP_CLOCK->HFCLKCTRL = NRF_APP_HCLK_PERFORMANCE;
	__DSB();
	__ISB();

	// Oscillator trim and capacitor registers use secure aliases.
	// Nonsecure images must leave their configuration to secure startup.
#if !defined(NRF_TRUSTZONE_NONSECURE)
	// XL1 (P0.00) and XL2 (P0.01) are shared with GPIO on nRF5340.
	// Allocate them to the crystal peripheral before LFXO is requested.
	// The pins must not be configured as GPIO while LFXO owns them.
	if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL ||
		g_McuOsc.LowPwrOsc.Type == OSC_TYPE_TCXO)
	{
		NRF_P0_S->PIN_CNF[0] =
			(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
		NRF_P0_S->PIN_CNF[1] =
			(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
		NRF_P0_S->PIN_CNF[0] |=
			(GPIO_PIN_CNF_MCUSEL_Peripheral << GPIO_PIN_CNF_MCUSEL_Pos);
		NRF_P0_S->PIN_CNF[1] |=
			(GPIO_PIN_CNF_MCUSEL_Peripheral << GPIO_PIN_CNF_MCUSEL_Pos);
	}

	// Oscillator load capacitors, taken from g_McuOsc
	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL && g_McuOsc.CoreOsc.LoadCap > 0)
	{
		uint32_t offset = (NRF_FICR_S->XOSC32MTRIM & FICR_XOSC32MTRIM_OFFSET_Msk) >> FICR_XOSC32MTRIM_OFFSET_Pos;
		uint32_t slope = (NRF_FICR_S->XOSC32MTRIM & FICR_XOSC32MTRIM_SLOPE_Msk) >> FICR_XOSC32MTRIM_SLOPE_Pos;
		uint32_t caps = (((slope + 56) * ((uint32_t)(g_McuOsc.CoreOsc.LoadCap * 2.0) - 14)) +
						((offset - 8) << 4) + 32) >> 6;
		NRF_OSCILLATORS_S->XOSC32MCAPS = ((caps << OSCILLATORS_XOSC32MCAPS_CAPVALUE_Pos) & OSCILLATORS_XOSC32MCAPS_CAPVALUE_Msk) |
										 OSCILLATORS_XOSC32MCAPS_ENABLE_Msk;
	}

	if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL && g_McuOsc.LowPwrOsc.LoadCap > 0)
	{
		NRF_OSCILLATORS_S->XOSC32KI.INTCAP = OSCILLATORS_XOSC32KI_INTCAP_INTCAP_C7PF;
	}
#endif
}

/* Workaround to allow NS code to access FICR. Override NRF_FICR_NS to move FICR_NS buffer. */
#define FICR_SIZE 0x1000ul
#define RAM_BASE 0x20000000ul
#define RAM_END  0x2FFFFFFFul

/* Copy FICR_S to FICR_NS RAM region */
void SystemStoreFICRNS()
{
    if ((uint32_t)NRF_FICR_NS < RAM_BASE || (uint32_t)NRF_FICR_NS + FICR_SIZE > RAM_END)
    {
        /* FICR_NS is not in RAM. */
        return;
    }
    /* Copy FICR to NS-accessible RAM block. */
    volatile uint32_t * from            = (volatile uint32_t *)((uint32_t)NRF_FICR_S + (FICR_SIZE - sizeof(uint32_t)));
    volatile uint32_t * to              = (volatile uint32_t *)((uint32_t)NRF_FICR_NS + (FICR_SIZE - sizeof(uint32_t)));
    volatile uint32_t * copy_from_end   = (volatile uint32_t *)NRF_FICR_S;
    while (from >= copy_from_end)
    {
        *(to--) = *(from--);
    }

    /* Make RAM region NS. */
    uint32_t ram_region = ((uint32_t)NRF_FICR_NS - (uint32_t)RAM_BASE) / SPU_RAMREGION_SIZE;
    NRF_SPU_S->RAMREGION[ram_region].PERM &= ~(1ul << SPU_RAMREGION_PERM_SECATTR_Pos);
}

/* Block write and execute access to FICR RAM region */
void SystemLockFICRNS()
{
    if ((uint32_t)NRF_FICR_NS < RAM_BASE || (uint32_t)NRF_FICR_NS + FICR_SIZE > RAM_END)
    {
        /* FICR_NS is not in RAM. */
        return;
    }

    uint32_t ram_region = ((uint32_t)NRF_FICR_NS - (uint32_t)RAM_BASE) / SPU_RAMREGION_SIZE;
    NRF_SPU_S->RAMREGION[ram_region].PERM &=
        ~(
            (1ul << SPU_RAMREGION_PERM_WRITE_Pos) |
            (1ul << SPU_RAMREGION_PERM_EXECUTE_Pos)
        );
    NRF_SPU_S->RAMREGION[ram_region].PERM |= 1ul << SPU_RAMREGION_PERM_LOCK_Pos;
}

/*lint --flb "Leave library region" */
