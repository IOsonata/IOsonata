/**-------------------------------------------------------------------------
@file	BlueIOICM20948.h

@brief	ICM20948 feature controls for BlueIOThingy.

@author	hoan
@date	Jul 25, 2018

@license

MIT License

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#ifndef __BLUEIOICM20948_H__
#define __BLUEIOICM20948_H__


#ifdef __cplusplus

#include "device_intrf.h"
#include "coredev/timer.h"

extern "C" {
#endif

void ICM20948EnableFeature(uint32_t Feature);

#ifdef __cplusplus
}
#endif

#endif	// __BLUEIOICM20948_H__
