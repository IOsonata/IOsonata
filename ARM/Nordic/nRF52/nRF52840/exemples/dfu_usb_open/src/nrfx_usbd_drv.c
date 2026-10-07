/**-------------------------------------------------------------------------
@file	nrfx_usbd_drv.c

@brief	Compilation wrapper for the nRF5 SDK nrfx USBD peripheral driver.

The app_usbd library (app_usbd.c, app_usbd_core.c, app_usbd_cdc_acm.c) and
the DFU USB serial transport (nrf_dfu_serial_usb.c) call the nrfx_usbd_*
driver API, but nrfx_usbd.c was not part of this project, producing
"undefined reference to nrfx_usbd_*" link errors.

This translation unit compiles the SDK driver in place. The quoted include
below is resolved by GCC against every -I directory; since
<nRF5_SDK>/modules/nrfx/drivers/include is already on the include path
(it provides nrfx_usbd.h), "../src/nrfx_usbd.c" resolves to
<nRF5_SDK>/modules/nrfx/drivers/src/nrfx_usbd.c.

The driver body is guarded internally by NRFX_CHECK(NRFX_USBD_ENABLED), so
NRFX_USBD_ENABLED (or legacy USBD_ENABLED) must be 1 in sdk_config.h.

@author	Hoang Nguyen Hoan
@date	Sep. 23, 2026

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

#include "sdk_config.h"
#include "nrfx.h"

#if !NRFX_CHECK(NRFX_USBD_ENABLED)
#error "NRFX_USBD_ENABLED must be set to 1 in sdk_config.h for the USB DFU transport"
#endif

#include "../src/nrfx_usbd.c"
