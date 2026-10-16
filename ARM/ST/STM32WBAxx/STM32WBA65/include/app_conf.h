/**-------------------------------------------------------------------------
@file	app_conf.h

@brief	Configuration of the STM32CubeWBA platform files built with the STM32WBA65 BLE library.

The ST platform files (linklayer_plat.c, ll_sys_if.c, pka_ctrl.c, hw_pka.c and
the ST stack headers) include this file by name. It holds the interrupt
numbers and priorities of the radio and PKA, the link layer task identifiers
and the power table selection. The BLE stack sizes (CFG_BLE_*) have defaults in
bt_app_stm32wba.cpp, a project can define them before the adapter is built.

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
#ifndef __APP_CONF_H__
#define __APP_CONF_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32wbaxx.h"
#include "ble_defs.h"

#ifndef TRUE
#define TRUE						1
#endif
#ifndef FALSE
#define FALSE						0
#endif

#ifndef UNUSED
#define UNUSED(x)					((void)(x))
#endif

#define CFG_SCM_SUPPORTED			0
#define CFG_LPM_LEVEL				0U
#define CFG_LOG_SUPPORTED			0U

// Radio interrupts. The radio uses HASH_IRQn as its low priority software
// interrupt, the HASH peripheral is not used
#define RADIO_INTR_NUM				RADIO_IRQn
#define RADIO_INTR_PRIO_HIGH		0
#define RADIO_INTR_PRIO_LOW			5
#define RADIO_SW_LOW_INTR_NUM		HASH_IRQn
#define RADIO_SW_LOW_INTR_PRIO		15

#define PKA_INTR_PRIO_PROCEND		7

#define CFG_RF_TX_POWER_TABLE_ID	0

// Sequencer tasks used by the link layer and the RNG
typedef enum {
	CFG_TASK_HW_RNG,
	CFG_TASK_LINK_LAYER,
	CFG_TASK_PKACTRL,
	CFG_TASK_NBR
} CFG_Task_Id_t;

#include "utilities_conf.h"

#endif
