/**-------------------------------------------------------------------------
@file	stm32wbaxx_hal_conf.h

@brief	HAL configuration name used by the STM32CubeWBA BLE platform files.

linklayer_plat.c includes stm32wbaxx_hal_conf.h by name. IOsonata does not
use the ST HAL, so no HAL module is selected here. This file only defines
assert_param, which the platform files call. As in the HAL without
USE_FULL_ASSERT, the check is compiled out. See stm32wbaxx_hal.h.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#ifndef __STM32WBAXX_HAL_CONF_H__
#define __STM32WBAXX_HAL_CONF_H__

#define assert_param(expr)		((void)0U)

#endif // __STM32WBAXX_HAL_CONF_H__
