/**-------------------------------------------------------------------------
@file	stm32_rtos.h

@brief	Sequencer declarations expected by the STM32CubeWBA platform files.

The sequencer is not built. bt_wba_platform.cpp implements UTIL_SEQ_RegTask and
UTIL_SEQ_SetTask over the IOsonata Bluetooth event queue.

@author	Hoang Nguyen Hoan
@date	Oct. 16, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#ifndef __STM32_RTOS_H__
#define __STM32_RTOS_H__

#include <stdint.h>
#include "app_conf.h"

#define TASK_PRIO_RNG				0
#define TASK_PRIO_LINK_LAYER		0
#define UTIL_SEQ_RFU				0

#ifdef __cplusplus
extern "C" {
#endif

void UTIL_SEQ_RegTask(uint32_t TaskBm, uint32_t Flags, void (*Task)(void));
void UTIL_SEQ_SetTask(uint32_t TaskBm, uint32_t Prio);

#ifdef __cplusplus
}
#endif

#endif
