/**-------------------------------------------------------------------------
@file	sce5_primitive_test.c

@brief	Host test support: sce5 primitive test.

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

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint32_t registers[128];
static unsigned accesses;
static unsigned failAfter;
static uint32_t *Register(unsigned offset)
{
	uint32_t *p = &registers[offset / 4U];
	accesses++;
	if (offset == 0x104U)
		*p |= 1U << 31;
	if (offset == 0x04U)
		*p |= 1U << 30;
	if (offset == 0x00U)
		*p &= ~((1U << 31) | (1U << 25));
	if (offset == 0x64U)
		*p &= ~(1U << 11);
	if (offset == 0xA8U)
		*p &= ~(1U << 6);
	if (offset == 0x18U)
		*p = 1U << 13;
	if (offset == 0x1CU || offset == 0x1BCU)
		*p = 0;
	if (failAfter && accesses >= failAfter) {
		if (offset == 0x104U)
			*p &= ~(1U << 31);
		if (offset == 0x04U)
			*p &= ~(1U << 30);
		if (offset == 0x00U)
			*p |= 1U << 25;
		if (offset == 0x64U)
			*p |= 1U << 11;
		if (offset == 0xA8U)
			*p |= 1U << 6;
		if (offset == 0x18U)
			*p |= 1U << 12;
	}
	return p;
}
#define SCE_REG(offset) (*Register(offset))
#define SCE_POLL_MAX 8U
#include "../../ARM/Renesas/RA4/RA4M1/src/crypto_sce5.c"
static bool Operation(unsigned operation)
{
	uint8_t key[32] = {0};
	uint8_t input[16] = {0};
	uint8_t output[16];
	uint32_t random[4];
	bool ok;

	memset(output, 0xA5, sizeof(output));
	memset(random, 0xA5, sizeof(random));
	if (operation == 0) {
		ok = Sce5RandomWords(random);
		if (!ok) {
			for (unsigned j = 0; j < 4; j++)
				assert(random[j] == 0);
		}
		return ok;
	}
	// A failed update wipes its block; a failed begin or end leaves the
	// block output to the engine, which wipes the whole request.
	ok = Sce5AesBegin(key, operation <= 2 ? 16 : 32, (operation & 1U) != 0);
	if (ok) {
		ok = Sce5AesUpdate(input, output);
		if (!ok) {
			for (unsigned j = 0; j < 16; j++)
				assert(output[j] == 0);
		}
	}
	if (ok)
		ok = Sce5AesEnd();
	return ok;
}

int main(void)
{
	uint32_t output[4];
	uint8_t key[32] = {0}, input[16] = {0}, cipher[16];
	unsigned total;
	assert(Sce5Init());
	assert(Sce5Ready());
	assert(Sce5RandomWords(output));
	assert(Sce5AesBegin(key, 16, true));
	assert(Sce5AesUpdate(input, cipher));
	assert(Sce5AesUpdate(input, cipher));
	assert(!Sce5RandomWords(output));
	assert(Sce5AesEnd());
	assert(Sce5AesBegin(key, 32, false));
	assert(Sce5AesUpdate(input, cipher));
	assert(Sce5AesEnd());
	total = accesses;
	for (unsigned n = 1; n < total; n += total/32 + 1) {
		memset(registers, 0, sizeof(registers));
		s_Ready = false;
		accesses = 0;
		failAfter = n;
		bool ok = Sce5Init();
		if (ok)
			ok = Sce5RandomWords(output);
		if (ok)
			ok = Sce5AesBegin(key, 16, true) && Sce5AesUpdate(input, cipher) && Sce5AesEnd();
		if (ok)
			ok = Sce5AesBegin(key, 32, false) && Sce5AesUpdate(input, cipher) && Sce5AesEnd();
		assert(!ok && !Sce5Ready());
		memset(output, 0xA5, sizeof(output));
		assert(!Sce5RandomWords(output));
		for (unsigned j = 0; j < 4; j++)
			assert(output[j] == 0);
		assert(accesses < total + 200);
	}
	for (unsigned operation = 0; operation < 5; operation++) {
		failAfter = 0;
		s_Ready = false;
		assert(Sce5Init());
		accesses = 0;
		assert(Operation(operation));
		unsigned operationAccesses = accesses;
		unsigned failures = 0;
		for (unsigned n = 1; n < operationAccesses; n += operationAccesses/32 + 1) {
			// Restore an initialized model without repeating its startup microcode.
			memset(registers, 0, sizeof(registers));
			s_Ready = true;
			s_Fault = false;
			accesses = 0;
			failAfter = n;
			if (!Operation(operation)) {
				assert(!Sce5Ready());
				failures++;
			}
			assert(accesses < operationAccesses + 200);
		}
		assert(failures >= 20);
	}
	failAfter = 0;
	assert(Sce5Init());
	assert(Sce5Ready());
	assert(Operation(0));
	printf("SCE5 RNG/AES-128/AES-256 encrypt/decrypt fault injection and reinitialization passed\n");
	printf("SCE5 production primitives: initialization/import/block/RNG paths and 32 injected poll failures passed (%u accesses)\n",total);
}
