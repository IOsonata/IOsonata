/**-------------------------------------------------------------------------
@file	entropy_model.h

@brief	Host test support: entropy model.

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
#include <stddef.h>
extern unsigned g_Reads, g_DiscardReads;
extern bool g_Timeout, g_ModuleRunning;
struct EntropyReg {
	unsigned id;
	uint8_t value;
	operator uint8_t() const;
	EntropyReg &operator=(uint8_t v);
};
struct EntropyRegs {
	EntropyReg TRNGSDR{1, 0}, TRNGSCR0{2, 0}, TRNGSCR1{3, 0};
};
extern EntropyRegs g_Trng;
#define R9A02_TRNG (&g_Trng)
#define R9A02_TRNG_EN 8U
#define R9A02_TRNG_ST 4U
#define R9A02_TRNG_RDY 128U
#define R9A02_MSTPCR_C 2U
#define R9A02_MSTPCRC_TRNG (1U << 28)
inline void R9a02ModuleStart(unsigned, uint32_t)
{
}
inline bool R9a02ModuleRunning(unsigned, uint32_t)
{
	return g_ModuleRunning;
}
uint32_t ModelRomRead();
struct OtpApi { uint32_t (*rngRead)(); };
struct RomApi
{
	OtpApi *otpApiBase;
};
extern OtpApi g_Otp;
extern RomApi g_Rom;
extern RomApi *g_RomPointer;
#define ROM_API g_RomPointer
#define OTP_API (g_Rom.otpApiBase)
