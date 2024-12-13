/**-------------------------------------------------------------------------
@file	sd_dispatch.cpp

@brief	Nordic Softdevice event dispatch

@author	Hoang Nguyen Hoan
@date	Dec. 13, 2024

@license

MIT License

Copyright (c) 2024, I-SYST, all rights reserved

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
#include <atomic>
#include <assert.h>

#include "nrf_sdh.h"
#include "nrf_section_iter.h"
#ifndef __ARMCC_VERSION
#include "nrf_crypto.h"
#endif

#include "sd_dispatch.h"

typedef struct _SDDispatch_Data {
	std::atomic<SdDispatch_t> Dispatch;
	void * pCtx;
} SDDispatchData_t;

static std::atomic<SdDispatch_t> s_SDDispatchFct(nullptr);

//static SdDispatch_t s_SDDispatchFct = nullptr;

void SetSoftdeviceDispatch(SdDispatch_t pDispatchFct)
{
	s_SDDispatchFct = pDispatchFct;
}

extern "C" void SD_EVT_IRQHandler(void)
//extern "C" void SWI2_IRQHandler(void)
{
//	assert(s_SDDispatchFct != nullptr);

	if (s_SDDispatchFct)
	{
		s_SDDispatchFct();
	}
}
#if 0
#ifndef __ARMCC_VERSION

#if NRF_MODULE_ENABLED(NRF_CRYPTO) && NRF_MODULE_ENABLED(NRF_CRYPTO_BACKEND_CC310)
extern nrf_crypto_backend_info_t const cc310_backend;
// Just to make the linker to keep the nrf_hw_backend
__attribute__ ((used)) static uint32_t s_pcc310_backend_info = (uint32_t)&cc310_backend;
#endif

#if NRF_MODULE_ENABLED(NRF_CRYPTO) && NRF_MODULE_ENABLED(NRF_CRYPTO_BACKEND_NRF_HW_RNG)
extern nrf_crypto_backend_info_t const nrf_hw_backend;
// Just to make the linker to keep the nrf_hw_backend
__attribute__ ((used)) static uint32_t s_pnrf_hw_backend_info = (uint32_t)&nrf_hw_backend;
#endif

#endif
#endif
