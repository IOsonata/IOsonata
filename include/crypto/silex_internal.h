/**-------------------------------------------------------------------------
@file	silex_internal.h

@brief	IOsonata silex internal.

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

// Internal operation exclusion for devices sharing the Silex register transport.
// Not an application interface. The transport busy flag still protects each transfer.
// MIT License, Copyright (c) 2026 I-SYST.
#ifndef IOSONATA_SILEX_INTERNAL_H
#define IOSONATA_SILEX_INTERNAL_H
#include <stdatomic.h>

inline atomic_flag &SilexOperationFlag()
{
	static atomic_flag busy = ATOMIC_FLAG_INIT;
	return busy;
}

inline bool SilexOpAcquire()
{
	return !atomic_flag_test_and_set(&SilexOperationFlag());
}

inline void SilexOpRelease()
{
	atomic_flag_clear(&SilexOperationFlag());
}
#endif
