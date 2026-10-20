/**-------------------------------------------------------------------------
@file	crypto_fw.cpp

@brief	STM32WBA crypto engine test firmware

Runs on the instruction set emulator of run_crypto.py, which models the AES,
PKA and RNG blocks and the RCC bits they need. The firmware is the generic
test exemples/crypto/crypto_engine_test.cpp, built with the board.h of the
WBA65 CryptoTest project, so the engines run the same code as on the board.
Printed lines go to the emulator through CryptoFwConsole.

Not hardware validation.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

int CryptoEngineTest(void);

extern "C" volatile int32_t g_CryptoFwResult;

extern "C" void CryptoFwConsole(const char *pStr, int Len) __attribute__((noinline));
extern "C" void CryptoFwDone(void) __attribute__((noinline));

volatile int32_t g_CryptoFwResult = -1000;

extern "C" void CryptoFwConsole(const char *pStr, int Len)
{
	__asm volatile ("" : : "r" (pStr), "r" (Len) : "memory");
}

extern "C" void CryptoFwDone(void)
{
	__asm volatile ("" ::: "memory");
}

// stdout of the test, newlib calls this
extern "C" int _write(int Fd, const char *pBuf, int Len)
{
	(void)Fd;
	CryptoFwConsole(pBuf, Len);

	return Len;
}

// The test reads the core clock for its timing line only
extern "C" uint32_t SystemCoreClockGet(void)
{
	return 100000000U;
}

extern "C" void __cxa_pure_virtual(void)
{
	while (1)
	{
	}
}

void operator delete(void *) noexcept {}
void operator delete(void *, size_t) noexcept {}

extern "C" int main(void)
{
	g_CryptoFwResult = CryptoEngineTest();
	CryptoFwDone();

	return 0;
}

// Minimal start: stack, data and bss, then main
extern "C" uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

extern "C" void ResetHandler(void)
{
	uint32_t *src = &_sidata;

	for (uint32_t *dst = &_sdata; dst < &_edata; )
	{
		*dst++ = *src++;
	}
	for (uint32_t *dst = &_sbss; dst < &_ebss; )
	{
		*dst++ = 0;
	}
	main();
	while (1)
	{
	}
}

extern "C" __attribute__((section(".isr_vector"), used))
void (* const __Vectors[16 + 128])(void) = {
	(void (*)(void))&_estack,
	ResetHandler,
};
