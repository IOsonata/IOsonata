/**-------------------------------------------------------------------------
@file	atomic.c

@brief	Implementation of require atomic functions for std::atomic

@author	Hoang Nguyen Hoan
@date	Mar. 22, 2015

@license

Copyright (c) 2015, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <signal.h>
#include <stdbool.h>

#include "cmsis_compiler.h"

unsigned __atomic_fetch_add_4(volatile void *d, unsigned val, int mem)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	*(unsigned*)d += val;
	__set_PRIMASK(primask);

	return *(unsigned*)d;
}

unsigned __atomic_fetch_sub_4(volatile void *d, unsigned val, int mem)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	*(unsigned*)d -= val;
	__set_PRIMASK(primask);

	return *(unsigned*)d;
}

#if 0
// Missing for M0
unsigned __atomic_exchange_4(volatile void *d, unsigned val, int mem)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	unsigned r = *(unsigned*)d;
	*(unsigned*)d = val;
	__set_PRIMASK(primask);

	return r;
}
#endif

// Missing for M0
bool __atomic_test_and_set(volatile void *d, int mem)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	unsigned r = *(unsigned*)d;
	*(unsigned*)d = 1;
	__set_PRIMASK(primask);

	return r;
}
