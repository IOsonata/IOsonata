/**-------------------------------------------------------------------------
@file	system_esp32_uart_clock.c

@brief	Per-chip UART peripheral clock-gating helpers.

Encapsulates the bus clock enable + system-bus reset bracket that was
previously inlined in UARTInit.  Same function names on all chips so
the UART driver can call them without #ifdef awareness.  The actual
implementations diverge:

  ESP32-C3        SYSTEM controller (PERIP_CLK_EN0 / PERIP_RST_EN0)
  ESP32-C5/C6     PCR controller    (UARTn CONF_REG : CLK_EN, RST_EN)

UART0 is the ROM console on all three chips; resetting it post-boot
combined with a GPIO-matrix transition wedges the TX FSM (verified
against ESP-IDF behaviour in v5.3 hal/esp32c3/uart_ll.h).  The reset
helper therefore short-circuits to a no-op on devno == 0.

This file follows the IOsonata pattern of one TU per "chip generation":
both the C3 and C5/C6 implementations live here, gated by `#if defined`.
The user may either link this file directly or merge its body into the
matching `system_esp32_system.c` / `system_esp32_pcr.c` per their
preferred layout.

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


// ===========================================================================
// ESP32-C3: SYSTEM controller
// ===========================================================================
#if defined(ESP32C3)

// SYSTEM block register fragments (matched against esp-idf
// soc/esp32c3/include/soc/system_reg.h).  Defined locally to avoid
// pulling in the rest of the SYSTEM block for what's a tiny use case.
#ifndef SYSTEM_PERIP_CLK_EN0_REG
#define SYSTEM_PERIP_CLK_EN0_REG    (*(volatile uint32_t *)0x600C0010UL)
#define SYSTEM_PERIP_RST_EN0_REG    (*(volatile uint32_t *)0x600C0018UL)
#define SYSTEM_UART_CLK_EN          (1UL << 2)
#define SYSTEM_UART1_CLK_EN         (1UL << 5)
#define SYSTEM_UART_RST             (1UL << 2)
#define SYSTEM_UART1_RST            (1UL << 5)
#endif

void Esp32UartClockEnable(int devno)
{
    if (devno == 0)
    {
        SYSTEM_PERIP_CLK_EN0_REG |= SYSTEM_UART_CLK_EN;
    }
    else if (devno == 1)
    {
        SYSTEM_PERIP_CLK_EN0_REG |= SYSTEM_UART1_CLK_EN;
    }
}

// Pulse the full system-bus + UART-core reset bracket.
//
// UART0 is the ROM console.  IDF deliberately skips the module reset
// for the console UART (esp_driver_uart/src/uart.c:218) since pulsing
// it post-boot wedges the TX FSM when combined with the GPIO-matrix
// pin transition that follows.  We do the same: no-op for devno == 0.
void Esp32UartReset(int devno)
{
    if (devno != 1)
    {
        return;
    }

    uint32_t base = (uint32_t)ESP32_UART1_BASE;

    ESP32_UART_CLK_CONF_REG(base) |= ESP32_UART_CLK_CONF_RST_CORE_Msk;
    SYSTEM_PERIP_RST_EN0_REG     |=  SYSTEM_UART1_RST;
    SYSTEM_PERIP_RST_EN0_REG     &= ~SYSTEM_UART1_RST;
    ESP32_UART_CLK_CONF_REG(base) &= ~ESP32_UART_CLK_CONF_RST_CORE_Msk;
}

#endif // ESP32C3


// ===========================================================================
// ESP32-C5 / ESP32-C6: PCR controller
// ===========================================================================
#if defined(ESP32C5) || defined(ESP32C6)

#include "esp32xx_pcr.h"

void Esp32UartClockEnable(int devno)
{
    if (devno < 0 || devno > 1)
    {
        return;
    }
    ESP32_PCR_UART_CONF_REG(devno) |= ESP32_PCR_UART_CONF_CLK_EN_Msk;
}

void Esp32UartReset(int devno)
{
    if (devno != 1)
    {
        return;
    }

    uint32_t base = (uint32_t)ESP32_UART1_BASE;
    uint32_t conf = ESP32_PCR_UART_CONF_REG(1);
    conf |= ESP32_PCR_UART_CONF_CLK_EN_Msk;

    ESP32_UART_CLK_CONF_REG(base) |= ESP32_UART_CLK_CONF_RST_CORE_Msk;
    ESP32_PCR_UART_CONF_REG(1)     = conf | ESP32_PCR_UART_CONF_RST_EN_Msk;
    ESP32_PCR_UART_CONF_REG(1)     = conf;
    ESP32_UART_CLK_CONF_REG(base) &= ~ESP32_UART_CLK_CONF_RST_CORE_Msk;
}

#endif // ESP32C5 || ESP32C6
