/**-------------------------------------------------------------------------
@file	cc.h

@brief	Compiler definitions for the lwIP raw API.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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

#ifndef __IOSONATA_LWIP_CC_H__
#define __IOSONATA_LWIP_CC_H__
#include <stdint.h>
#include <inttypes.h>
#define BYTE_ORDER LITTLE_ENDIAN

// A library defines IOSONATA_LWIP_POOL_SECTION, a section name without
// quotes such as .ram2, to put the lwIP pools in a RAM region of its own.
// memp_init sets up the pools, the startup does not need to clear them.
#ifdef IOSONATA_LWIP_POOL_SECTION
#define IOSONATA_LWIP_STR2(x)	#x
#define IOSONATA_LWIP_STR(x)	IOSONATA_LWIP_STR2(x)
#define LWIP_DECLARE_MEMORY_ALIGNED(variable_name, size) \
	u8_t variable_name[LWIP_MEM_ALIGN_BUFFER(size)] \
	__attribute__((section(IOSONATA_LWIP_STR(IOSONATA_LWIP_POOL_SECTION))))
#endif
#define LWIP_PLATFORM_DIAG(x) \
	do \
	{ \
	} while (0)
#define LWIP_PLATFORM_ASSERT(x) \
	do \
	{ \
		(void)(x); \
		__builtin_trap(); \
	} while (0)
#endif
