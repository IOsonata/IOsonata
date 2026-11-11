/**-------------------------------------------------------------------------
@file	esp32xx_pcr.h

@brief	ESP32 PCR (Peripheral Clock and Reset) controller register
        definitions for ESP32-C5, ESP32-C6 and ESP32-C61.

C2/C3 use SYSTEM instead. The C5/C6/C61 HP UART PCR blocks have a
12-byte stride: bus clock/reset, serial clock, and memory power control.
C61 adds UART2. XTAL source selectors differ between C6 and C5/C61.

@author	Nguyen Hoan Hoang
@date	May 9, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __ESP32XX_PCR_H__
#define __ESP32XX_PCR_H__

#include <stdint.h>

#include "esp32xx.h"

#if defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)

#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

/*---------------------------------------------------------------------------
 * PCR base.  Identical on C5 and C6.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_PCR_BASE
#define ESP32_PCR_BASE                      0x60096000UL
#endif

/*---------------------------------------------------------------------------
 * UART0 / UART1 (HP UART) PCR registers.
 *---------------------------------------------------------------------------*/
#define ESP32_PCR_UART0_CONF_REG            ESP32_REG32(ESP32_PCR_BASE + 0x000U)
#define ESP32_PCR_UART0_SCLK_CONF_REG       ESP32_REG32(ESP32_PCR_BASE + 0x004U)
#define ESP32_PCR_UART0_PD_CTRL_REG         ESP32_REG32(ESP32_PCR_BASE + 0x008U)
#define ESP32_PCR_UART1_CONF_REG            ESP32_REG32(ESP32_PCR_BASE + 0x00CU)
#define ESP32_PCR_UART1_SCLK_CONF_REG       ESP32_REG32(ESP32_PCR_BASE + 0x010U)
#define ESP32_PCR_UART1_PD_CTRL_REG         ESP32_REG32(ESP32_PCR_BASE + 0x014U)

// Per-UART register accessors keyed by HP UART device number.
#define ESP32_PCR_UART_CONF_REG(devno)      ESP32_REG32(ESP32_PCR_BASE + (uint32_t)(devno) * 0x0CU + 0x000U)
#define ESP32_PCR_UART_SCLK_CONF_REG(devno) ESP32_REG32(ESP32_PCR_BASE + (uint32_t)(devno) * 0x0CU + 0x004U)
#define ESP32_PCR_UART_PD_CTRL_REG(devno)   ESP32_REG32(ESP32_PCR_BASE + (uint32_t)(devno) * 0x0CU + 0x008U)

/*---------------------------------------------------------------------------
 * CONF_REG bit positions.
 *
 * Note: CLK_EN defaults to 1 on cold boot for UART0 (so the ROM bootloader
 * can print).  Code that always writes CLK_EN = 1 is therefore idempotent.
 *---------------------------------------------------------------------------*/
#define ESP32_PCR_UART_CONF_CLK_EN_Pos      0U
#define ESP32_PCR_UART_CONF_CLK_EN_Msk      (1UL << ESP32_PCR_UART_CONF_CLK_EN_Pos)
#define ESP32_PCR_UART_CONF_RST_EN_Pos      1U
#define ESP32_PCR_UART_CONF_RST_EN_Msk      (1UL << ESP32_PCR_UART_CONF_RST_EN_Pos)

/*---------------------------------------------------------------------------
 * SCLK_CONF_REG bit positions.
 *
 * SCLK source select for the UART core clock (separate from APB clock to
 * the registers). C6 selects XTAL/RC_FAST/PLL with 3/2/1.
 * C5 and C61 select XTAL/RC_FAST/PLL with 0/1/2.
 *---------------------------------------------------------------------------*/
#define ESP32_PCR_UART_SCLK_SEL_Pos         20U
#define ESP32_PCR_UART_SCLK_SEL_Msk         (3UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#if defined(ESP32C6)
#define ESP32_PCR_UART_SCLK_SEL_XTAL        (3UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#define ESP32_PCR_UART_SCLK_SEL_RC_FAST     (2UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#define ESP32_PCR_UART_SCLK_SEL_PLL_F80M    (1UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#else
#define ESP32_PCR_UART_SCLK_SEL_XTAL        (0UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#define ESP32_PCR_UART_SCLK_SEL_RC_FAST     (1UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#define ESP32_PCR_UART_SCLK_SEL_PLL_F80M    (2UL << ESP32_PCR_UART_SCLK_SEL_Pos)
#endif
#define ESP32_PCR_UART_SCLK_DIV_NUM_Pos     12U
#define ESP32_PCR_UART_SCLK_DIV_NUM_Msk     (0xFFUL << ESP32_PCR_UART_SCLK_DIV_NUM_Pos)
#define ESP32_PCR_UART_SCLK_DIV_A_Pos       6U
#define ESP32_PCR_UART_SCLK_DIV_A_Msk       (0x3FUL << ESP32_PCR_UART_SCLK_DIV_A_Pos)
#define ESP32_PCR_UART_SCLK_DIV_B_Pos       0U
#define ESP32_PCR_UART_SCLK_DIV_B_Msk       (0x3FUL << ESP32_PCR_UART_SCLK_DIV_B_Pos)
#define ESP32_PCR_UART_SCLK_EN_Pos          22U
#define ESP32_PCR_UART_SCLK_EN_Msk          (1UL << ESP32_PCR_UART_SCLK_EN_Pos)

#endif // ESP32C5 || ESP32C6

#endif // __ESP32XX_PCR_H__
