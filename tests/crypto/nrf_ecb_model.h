/**-------------------------------------------------------------------------
@file	nrf_ecb_model.h

@brief	Host test support: nrf ecb model.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#pragma once
#include <stdint.h>
struct EcbReg {
	unsigned id;
	uintptr_t value;
	EcbReg &operator=(uintptr_t v);
	operator uintptr_t() const
	{
		return value;
	}
};
struct NRF_ECB_Type {
	EcbReg EVENTS_ENDECB{0,0}, EVENTS_ERRORECB{0,0}, ECBDATAPTR{0,0};
	EcbReg TASKS_STARTECB{1,0}, TASKS_STOPECB{2,0};
};
extern NRF_ECB_Type g_Ecb;
#define NRF_ECB (&g_Ecb)
#define NRF_ECB_NS (&g_Ecb)
inline void __DMB()
{
}
#define NRF_SUCCESS 0U
struct nrf_ecb_hal_data_t
{
	uint8_t key[16], cleartext[16], ciphertext[16];
};
uint32_t sd_softdevice_is_enabled(uint8_t *enabled);
uint32_t sd_ecb_block_encrypt(nrf_ecb_hal_data_t *data);

extern "C" void ResetEntry(void);
extern uint32_t g_EcbResetVector;
