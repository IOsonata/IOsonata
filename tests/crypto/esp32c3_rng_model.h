/**-------------------------------------------------------------------------
@file	esp32c3_rng_model.h

@brief	ESP32-C3 RNG register-model declarations.

@author	IOsonata contributors
@date	Nov. 8, 2026

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
#ifndef TEST_ESP32C3_RNG_MODEL_H
#define TEST_ESP32C3_RNG_MODEL_H

#include <stdint.h>
#ifdef __cplusplus
class EspRngModelReg {
public:
	uintptr_t Addr;
	operator uint32_t() const;
	EspRngModelReg &operator=(uint32_t Value);
	EspRngModelReg &operator|=(uint32_t Value)
	{
		return *this = (uint32_t)*this | Value;
	}
	EspRngModelReg &operator&=(uint32_t Value)
	{
		return *this = (uint32_t)*this & Value;
	}
};
EspRngModelReg &EspRngModelRegister(uintptr_t Addr);
#define ESP32_RNG_REG(addr) EspRngModelRegister(addr)
#endif
#endif
