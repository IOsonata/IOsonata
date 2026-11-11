/**-------------------------------------------------------------------------
@file	nrf_mpsl.h

@brief	MPSL configuration

MPSL configuration.

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
#ifndef __NRF_MPSL_H__
#define __NRF_MPSL_H__

// Interrupt MPSL pends for its low priority processing. Any interrupt that
// nothing else uses will do. The nRF5340 network core has no SWI5 and the
// nRF54H20 cores have no SWI5_EGU5; both use SWI0.
#if defined(NRF5340_XXAA_NETWORK) || defined(NRF54H20_XXAA)
#define MPSL_LOW_PRIO_IRQn			SWI0_IRQn
#define MPSL_LOW_PRIO_IRQHandler	SWI0_IRQHandler
#else
#define MPSL_LOW_PRIO_IRQn			SWI5_EGU5_IRQn
#define MPSL_LOW_PRIO_IRQHandler	SWI5_EGU5_IRQHandler
#endif


#ifdef __cplusplus
extern "C" {
#endif

bool MpslInit(void);

/**
 * @brief	Open the timeslot session and offer it as the memory arbiter.
 *
 * The memory controller is shared with the radio, and only MPSL knows when a
 * window is free. This opens a timeslot session and registers it with the
 * memory interface, so a write or an erase runs inside a granted window
 * instead of behind the stack's back.
 *
 * Called once the radio is up, from whoever brought MPSL up. Until it is,
 * the memory interface drives the controller directly, which is correct only
 * while no radio is running.
 *
 * @return	0 on success, negative errno on failure.
 */
int MpslNvmArbiterStart(void);

/**
 * @brief	Stop arbitrating and close the session.
 *
 * After this the memory interface goes back to driving the controller
 * directly, so it must not be called while the radio is running.
 */
void MpslNvmArbiterStop(void);

/**
 * @brief	Request or release the 32 MHz crystal through MPSL
 *
 * SystemCoreClockSelect calls it for each request of the crystal and for the
 * last release. Once MPSL is initialized it owns CLOCK and the crystal is
 * requested from MPSL. The weak default in system_clock_nrfx.cpp is used when
 * the SoftDevice Controller is not in the build.
 *
 * @param	bOn : true - request and wait until it runs, false - release
 *
 * @return	-1 - MPSL is not initialized, the caller drives CLOCK
 * 			0 - done
 * 			1 - the crystal did not start
 */
int nRFxMpslHfxo(bool bOn);

/**
 * @brief	Tell whether MPSL owns CLOCK
 *
 * SystemLowFreqClockSelect leaves LFCLK alone while MPSL is initialized, its
 * source was set by MpslInit. The weak default in system_clock_nrfx.cpp
 * returns false when the SoftDevice Controller is not in the build.
 *
 * @return	true - MPSL is initialized
 */
bool nRFxMpslActive(void);

#ifdef __cplusplus
}
#endif

#endif	// __NRF_MPSL_H__
