/**-------------------------------------------------------------------------
@file	nrfx_coredep.h

@brief	Include path adapter for TinyUSB's nRF DWC2 port.

TinyUSB's dwc2_nrf.h includes soc/nrfx_coredep.h, the location used by older
nrfx releases. nrfx 4 moved the header to lib/nrfx_coredep.h. This adapter
lets the stock TinyUSB source build against the nrfx copy used by IOsonata.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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
#ifndef __TINYUSB_SOC_NRFX_COREDEP_H__
#define __TINYUSB_SOC_NRFX_COREDEP_H__

#include "lib/nrfx_coredep.h"

#endif
