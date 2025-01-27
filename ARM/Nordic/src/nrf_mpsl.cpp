/**-------------------------------------------------------------------------
@file	nrf_mpsl.cpp

@brief	MPSL configuration.

MPSL configuration & initialization.

@author	Nguyen Hoan Hoang
@date	Dec. 21, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Softare.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "nrf.h"
#include "mpsl.h"

#include "nrf_mpsl.h"
#include "mpsl_fem_init.h"
#include "nrfx_power.h"

#include "coredev/uart.h"
#include "coredev/system_core_clock.h"

/******** For DEBUG ************/
#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

static void MpslAssert(const char * const file, const uint32_t line)
{
	DEBUG_PRINTF("MPSL Fault: %s, %d\n", file, line);
	while(1);
}

bool MpslInit(void)
{
	int32_t res = 0;

	mpsl_clock_lfclk_cfg_t lfclk = {MPSL_CLOCK_LF_SRC_RC, 0,};
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

	// Set default clock based on system oscillator settings
	if (lfosc->Type == OSC_TYPE_RC)
	{
		lfclk.source = MPSL_CLOCK_LF_SRC_RC;
		lfclk.rc_ctiv = MPSL_RECOMMENDED_RC_CTIV;
		lfclk.rc_temp_ctiv = MPSL_RECOMMENDED_RC_TEMP_CTIV;
	}
	else
	{
		lfclk.accuracy_ppm = lfosc->Accuracy;
		lfclk.source = MPSL_CLOCK_LF_SRC_XTAL;
	}

	lfclk.skip_wait_lfclk_started = MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED;

	mpsl_fem_init();

	DEBUG_PRINTF("mpsl_init\r\n");

	// Initialize Nordic multi-protocol support library (MPSL)
#ifdef NRF54L_SERIES
	//NVIC_DisableIRQ(GRTC_3_IRQn);
	//NVIC_SetPriority(GRTC_3_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	//NVIC_EnableIRQ(GRTC_3_IRQn);
	//NRF_GRTC->INTENSET3 = 0xF80;
	NRF_GRTC->MODE |= 2;
	NRF_GRTC->TASKS_START = 1;


	res = mpsl_init(&lfclk, SWI00_IRQn, MpslAssert);
	res = mpsl_clock_hfclk_latency_set(MPSL_CLOCK_HF_LATENCY_TYPICAL);
	mpsl_pan_rfu();
#else
	res = mpsl_init(&lfclk, PendSV_IRQn, MpslAssert);
#endif
	DEBUG_PRINTF("mpsl_init res = %x\r\n", res);

	if (res < 0)
	{
		return false;
	}

#if 0
	static uint8_t timeslot_context = 0;
	res = mpsl_timeslot_session_count_set((void *) timeslot_context,
			MPSL_TIMESLOT_SESSION_COUNT);
#endif


#ifdef NRF54L_SERIES
	NVIC_SetPriority(SWI00_IRQn, MPSL_HIGH_IRQ_PRIORITY + 4);
	NVIC_EnableIRQ(SWI00_IRQn);
	NVIC_SetPriority(CLOCK_POWER_IRQn, MPSL_HIGH_IRQ_PRIORITY + 4);
	NVIC_EnableIRQ(CLOCK_POWER_IRQn);

	//NVIC_SetPriority(TIMER10_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	//NVIC_EnableIRQ(TIMER10_IRQn);
	NVIC_SetPriority(RADIO_0_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	NVIC_EnableIRQ(RADIO_0_IRQn);
#else
	NVIC_SetPriority(PendSV_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(PendSV_IRQn);
#endif


	return true;
}

extern "C" {

#ifdef NRF54L_SERIES
void SWI00_IRQHandler(void)
#else
void PendSV_Handler(void)
#endif
{
	DEBUG_PRINTF("SWI00_IRQHandler\r\n");
	mpsl_low_priority_process();
}

#ifdef NRF54L_SERIES
void RADIO_0_IRQHandler(void)
#else
void RADIO_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RADIO_Handler\r\n");
	MPSL_IRQ_RADIO_Handler();
}

#ifdef NRF54L_SERIES
void CLOCK_POWER_IRQHandler(void)
#else
void POWER_CLOCK_IRQHandler()
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_CLOCK_Handler\r\n");
	MPSL_IRQ_CLOCK_Handler();
}

#ifdef NRF54L_SERIES
void GRTC_3_IRQHandler(void)
#else
void RTC0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RTC0_Handler\r\n");
	MPSL_IRQ_RTC0_Handler();
}

#ifdef NRF54L15_XXAA
void TIMER10_IRQHandler(void)
#else
void TIMER0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("TIMER10_IRQHandler\r\n");
	MPSL_IRQ_TIMER0_Handler();
}

#if defined(NRF54L_SERIES)
/** @brief MPSL requesting CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it needs CONSTLAT to be on.
 * It only calls the function on nRF54L Series devices.
 */
void mpsl_constlat_request_callback(void)
{
	nrfx_power_constlat_mode_request();
}

/** @brief De-request CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it no longer needs CONSTLAT to be on.
 * It only only calls the function on nRF54L Series devices.
 */
void mpsl_lowpower_request_callback(void)
{
	nrfx_power_constlat_mode_free();
}
#endif

}	// extern "C"
