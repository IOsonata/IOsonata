/**-------------------------------------------------------------------------
@file	system_esp32_uart_clock.c

@brief	Per-chip UART peripheral clock-gating helpers.

C2/C3 use SYSTEM for the peripheral gate and UART CLK_CONF for the
serial clock. C5/C6/C61 use PCR for both; UART CLK_CONF only enables TX/RX.
The XTAL selector differs between C6 and C5/C61. Preserve UART0's ROM
console state and reset the other instances before initialization.

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
#include <stdint.h>
#include <stdbool.h>

#include "esp32xx.h"
#include "esp32xx_uart.h"


// C2/C3 use SYSTEM; C5/C6/C61 use PCR. Preserve the ROM console.
#if defined(ESP32C2) || defined(ESP32C3)
#define SYSTEM_PERIP_CLK_EN0_REG ESP32_REG32(ESP32_SYSTEM_BASE + 0x10U)
#define SYSTEM_PERIP_RST_EN0_REG ESP32_REG32(ESP32_SYSTEM_BASE + 0x18U)

void Esp32UartClockEnable(int devno)
{
	if (devno >= 0 && devno < 2)
	{
		SYSTEM_PERIP_CLK_EN0_REG |= 1UL << (devno == 0 ? 2U : 5U);
	}
}

void Esp32UartReset(int devno)
{
	if (devno != 1)
	{
		return;
	}
#if defined(ESP32C3)
	ESP32_UART_CLK_CONF_REG(ESP32_UART1_BASE) |= ESP32_UART_CLK_CONF_RST_CORE_Msk;
#endif
	SYSTEM_PERIP_RST_EN0_REG |= 1UL << 5;
	SYSTEM_PERIP_RST_EN0_REG &= ~(1UL << 5);
#if defined(ESP32C3)
	ESP32_UART_CLK_CONF_REG(ESP32_UART1_BASE) &= ~ESP32_UART_CLK_CONF_RST_CORE_Msk;
#endif
}

void Esp32UartClockConfig(int devno, uint32_t Divider)
{
	uint32_t base;
	uint32_t clock;

	base = devno == 0 ? ESP32_UART0_BASE : ESP32_UART1_BASE;
	clock = ESP32_UART_CLK_CONF_REG(base);
	clock &= ~(ESP32_UART_CLK_CONF_SCLK_SEL_Msk | ESP32_UART_CLK_CONF_SCLK_DIV_NUM_Msk |
		ESP32_UART_CLK_CONF_SCLK_DIV_A_Msk | ESP32_UART_CLK_CONF_SCLK_DIV_B_Msk);
	clock |= ESP32_UART_CLK_CONF_SCLK_SEL_XTAL | ESP32_UART_CLK_CONF_SCLK_EN_Msk |
		ESP32_UART_CLK_CONF_TX_SCLK_EN_Msk | ESP32_UART_CLK_CONF_RX_SCLK_EN_Msk;
	clock |= (Divider - 1U) << ESP32_UART_CLK_CONF_SCLK_DIV_NUM_Pos;
	ESP32_UART_CLK_CONF_REG(base) = clock;
}
#else
#include "esp32xx_pcr.h"

void Esp32UartClockEnable(int devno)
{
	if (devno >= 0 && devno < ESP32_HP_UART_COUNT)
	{
		ESP32_PCR_UART_CONF_REG(devno) |= ESP32_PCR_UART_CONF_CLK_EN_Msk;
	}
}

void Esp32UartReset(int devno)
{
	uint32_t conf;

	if (devno <= 0 || devno >= ESP32_HP_UART_COUNT)
	{
		return;
	}
	conf = ESP32_PCR_UART_CONF_REG(devno) | ESP32_PCR_UART_CONF_CLK_EN_Msk;
	ESP32_PCR_UART_CONF_REG(devno) = conf | ESP32_PCR_UART_CONF_RST_EN_Msk;
	ESP32_PCR_UART_CONF_REG(devno) = conf;
}

void Esp32UartClockConfig(int devno, uint32_t Divider)
{
	uint32_t base;

	base = devno == 0 ? ESP32_UART0_BASE : ESP32_UART1_BASE;
#if defined(ESP32C61)
	if (devno == 2)
	{
		base = ESP32_UART2_BASE;
	}
#endif
	ESP32_PCR_UART_SCLK_CONF_REG(devno) = ESP32_PCR_UART_SCLK_SEL_XTAL |
		ESP32_PCR_UART_SCLK_EN_Msk | ((Divider - 1U) << ESP32_PCR_UART_SCLK_DIV_NUM_Pos);
	ESP32_UART_CLK_CONF_REG(base) |= ESP32_UART_CLK_CONF_TX_SCLK_EN_Msk |
		ESP32_UART_CLK_CONF_RX_SCLK_EN_Msk;
}
#endif
