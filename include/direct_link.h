/**-------------------------------------------------------------------------
@file	direct_link.h

@brief	Implementation of excelitas direct link single wire interface


@author	Hoang Nguyen Hoan
@date	Nov. 16, 2019

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

#ifndef __DIRECT_LINK_H__
#define __DIRECT_LINK_H__

#include "iopinctrl.h"

typedef void (*INTHANDLER)();

typedef struct __Direct_Link_Dev {
	IOPINCFG Pin;
	INTHANDLER IntHnadler;
} DIRECTLINK_DEV;

#ifdef __cplusplus
extern "C" {
#endif

bool DirectLinkInit(DIRECTLINK_DEV *pDev, int PortNo, int PinNo, int PinOp);
uint32_t DirectLinkRead(DIRECTLINK_DEV *pDev, int NbBits);

#ifdef __cplusplus
}
#endif

#endif // __DIRECT_LINK_H__

