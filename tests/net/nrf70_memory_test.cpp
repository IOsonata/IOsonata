/**-------------------------------------------------------------------------
@file nrf70_memory_test.cpp
@brief Fixed block ownership and nRF70 lock allocation regression.
@license MIT License
Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "net/nrf70_osal_memory.h"
#include "net/nrf70_osal_sync.h"

int main(void)
{
	assert(Nrf70OsalMemAlloc(1) == nullptr);
	assert(Nrf70OsalSpinAlloc() == nullptr);
	alignas(max_align_t) uint8_t small[32];
	alignas(max_align_t) uint8_t medium[64];
	alignas(max_align_t) uint8_t large[128];
	Nrf70OsalMemBlock_t blocks[] = {
		{ large, sizeof(large), true },
		{ small, sizeof(small), true },
		{ medium, sizeof(medium), true },
	};
	assert(Nrf70OsalMemInit(blocks, 3));
	assert(!blocks[0].Used && !blocks[1].Used && !blocks[2].Used);
	assert(Nrf70OsalMemAlloc(0) == nullptr);
	assert(Nrf70OsalMemAlloc(SIZE_MAX) == nullptr);
	assert(Nrf70OsalMemAlloc(129) == nullptr);

	for (unsigned round = 0; round < 100; round++)
	{
		void *a = Nrf70OsalMemAlloc(17);
		void *b = Nrf70OsalMemAlloc(33);
		void *c = Nrf70OsalMemAlloc(65);
		assert(a == small && b == medium && c == large);
		assert((uintptr_t)a % alignof(max_align_t) == 0);
		memset(a, 0x11, 17);
		memset(b, 0x22, 33);
		memset(c, 0x33, 65);
		assert(Nrf70OsalMemAlloc(1) == nullptr);
		assert(!Nrf70OsalMemInit(nullptr, 0));
		assert(!Nrf70OsalMemInit(blocks, 3));
		Nrf70OsalMemFree((uint8_t *)a + 1);
		Nrf70OsalMemFree(nullptr);
		assert(Nrf70OsalMemAlloc(1) == nullptr);
		Nrf70OsalMemFree(b);
		Nrf70OsalMemFree(b);
		assert(Nrf70OsalMemAlloc(64) == medium);
		assert(small[16] == 0x11 && large[64] == 0x33);
		Nrf70OsalMemFree(c);
		Nrf70OsalMemFree(a);
		Nrf70OsalMemFree(b);
	}

	// Invalid replacement tables leave the working configuration intact.
	Nrf70OsalMemBlock_t invalid = { small + 1, 16, false };
	assert(!Nrf70OsalMemInit(&invalid, 1));
	invalid.pMem = small;
	invalid.Size = 0;
	assert(!Nrf70OsalMemInit(&invalid, 1));
	invalid.Size = SIZE_MAX;
	assert(!Nrf70OsalMemInit(&invalid, 1));
	invalid.pMem = nullptr;
	invalid.Size = 16;
	assert(!Nrf70OsalMemInit(&invalid, 1));
	assert(!Nrf70OsalMemInit(nullptr, 1));
	assert(!Nrf70OsalMemInit(blocks, 0));
	assert(!Nrf70OsalMemInit(blocks, SIZE_MAX));
	Nrf70OsalMemBlock_t overlap[] = {
		{ large, 128, false }, { large + alignof(max_align_t), 16, false },
	};
	assert(!Nrf70OsalMemInit(overlap, 2));
	alignas(max_align_t) Nrf70OsalMemBlock_t self = {};
	self.pMem = &self;
	self.Size = sizeof(self);
	assert(!Nrf70OsalMemInit(&self, 1));
	memset(small, 0xa5, sizeof(small));
	assert(Nrf70OsalMemZalloc(19) == small);
	for (unsigned i = 0; i < sizeof(small); i++)
	{
		assert(small[i] == (i < 19 ? 0 : 0xa5));
	}
	Nrf70OsalMemFree(small);

	// Lock callbacks consume the same bounded pool and return its slots.
	void *locks[3];
	for (unsigned i = 0; i < 3; i++)
	{
		locks[i] = Nrf70OsalSpinAlloc();
		assert(locks[i] != nullptr);
		assert(*(uint32_t *)locks[i] == 0);
		Nrf70OsalSpinInit(locks[i]);
		Nrf70OsalSpinTake(locks[i]);
		Nrf70OsalSpinRelease(locks[i]);
	}
	assert(Nrf70OsalSpinAlloc() == nullptr);
	assert(Nrf70OsalMemZalloc(1) == nullptr);
	assert(!Nrf70OsalMemInit(nullptr, 0));
	Nrf70OsalSpinFree(locks[1]);
	assert(Nrf70OsalSpinAlloc() == locks[1]);
	for (unsigned i = 0; i < 3; i++)
	{
		Nrf70OsalSpinFree(locks[i]);
	}
	Nrf70OsalSpinFree(nullptr);
	assert(Nrf70OsalMemInit(nullptr, 0));
	assert(Nrf70OsalMemAlloc(1) == nullptr);
	assert(Nrf70OsalMemInit(blocks, 3));
	assert(Nrf70OsalMemAlloc(128) == large);
	Nrf70OsalMemFree(large);
	assert(Nrf70OsalMemInit(nullptr, 0));
	puts("nRF70 fixed memory and locks: PASS");
	return 0;
}
