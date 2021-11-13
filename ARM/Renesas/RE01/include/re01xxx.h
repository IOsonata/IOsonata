/**-------------------------------------------------------------------------
@file	re01xxx.h

@brief	Renesas RE01 series definitions

@author	Hoang Nguyen Hoan
@date	Nov. 10, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#ifndef __RE01XXX_H__
#define __RE01XXX_H__

#ifndef __unix__
#ifdef __GNUC__
#ifndef __PROGRAM_START
#define __PROGRAM_START		// Define to fix compile bug in CMSIS 5.6
#endif
#endif
#include "cmsis_compiler.h"
#endif

#ifdef RE01_1500KB
#include "RE01_1500KB.h"
#elif defined(RE01_256KB)
#include "RE01_256KB.h"
#endif

#endif // __RE01XXX_H__




