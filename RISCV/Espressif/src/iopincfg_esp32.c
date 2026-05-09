/**-------------------------------------------------------------------------
@file   iopincfg_esp32.c

@brief  I/O pin configuration for Espressif RISC-V targets.

The active register writes use target-aware raw register macros from
esp32xx_gpio.h.  The GPIO blocks are similar across C3/C5/C6, but C5 uses
different offsets and wider GPIO-matrix selector fields.

@author Hoang Nguyen Hoan
@date   Mar. 2026

@license

MIT

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

#include <stdint.h>
#include <stdbool.h>

#include "esp32xx.h"
#include "esp32xx_gpio.h"

#include "coredev/iopincfg.h"

static inline bool ESP32IsValidGpioPin(int PinNo)
{
    return (PinNo >= 0) && ((uint32_t)PinNo < ESP32_GPIO_PIN_COUNT);
}

void IOPinConfig(int PortNo, int PinNo, int PinOp,
                 IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
    (void)PortNo;
    (void)PinOp;

    if (!ESP32IsValidGpioPin(PinNo))
    {
        return;
    }

    uint32_t pin = (uint32_t)PinNo;
    uint32_t mask = 1UL << pin;

    uint32_t iomux = ESP32_IOMUX_MCU_SEL_GPIO
                   | (2UL << ESP32_IOMUX_FUN_DRV_Pos);

    if ((Dir == IOPINDIR_INPUT) || (Dir == IOPINDIR_BI))
    {
        iomux |= (1UL << ESP32_IOMUX_FUN_IE_Pos);
    }

    if (Resistor == IOPINRES_PULLUP)
    {
        iomux |= (1UL << ESP32_IOMUX_FUN_WPU_Pos);
    }
    else if (Resistor == IOPINRES_PULLDOWN)
    {
        iomux |= (1UL << ESP32_IOMUX_FUN_WPD_Pos);
    }

    ESP32_IOMUX_PAD_REG(pin) = iomux;

    /* Select simple GPIO output and use GPIO_ENABLE as output-enable source. */
    ESP32_GPIO_FUNC_OUT_REG32(pin) = ESP32_GPIO_FUNC_OUT_GPIO_VALUE;

    uint32_t pincfg = ESP32_GPIO_PIN_REG32(pin);
    if (Type == IOPINTYPE_OPENDRAIN)
    {
        pincfg |= ESP32_GPIO_PIN_PAD_DRIVER_Msk;
    }
    else
    {
        pincfg &= ~ESP32_GPIO_PIN_PAD_DRIVER_Msk;
    }
    ESP32_GPIO_PIN_REG32(pin) = pincfg;

    if ((Dir == IOPINDIR_OUTPUT) || (Dir == IOPINDIR_BI))
    {
        ESP32_GPIO_ENABLE_W1TS_REG32 = mask;
    }
    else
    {
        ESP32_GPIO_ENABLE_W1TC_REG32 = mask;
    }
}

void IOPinDisable(int PortNo, int PinNo)
{
    (void)PortNo;

    if (!ESP32IsValidGpioPin(PinNo))
    {
        return;
    }

    uint32_t pin = (uint32_t)PinNo;
    uint32_t mask = 1UL << pin;

    ESP32_GPIO_ENABLE_W1TC_REG32 = mask;
    ESP32_IOMUX_PAD_REG(pin) = 0U;
    ESP32_GPIO_FUNC_OUT_REG32(pin) = ESP32_GPIO_FUNC_OUT_GPIO_VALUE;
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
    (void)PortNo;

    if (!ESP32IsValidGpioPin(PinNo))
    {
        return;
    }

    uint32_t pin = (uint32_t)PinNo;
    uint32_t v = ESP32_IOMUX_PAD_REG(pin) & ~ESP32_IOMUX_FUN_DRV_Msk;
    v |= ((Strength == IOPINSTRENGTH_STRONG) ? 3UL : 2UL)
       << ESP32_IOMUX_FUN_DRV_Pos;
    ESP32_IOMUX_PAD_REG(pin) = v;
}

void IOPinDisableInterrupt(int IntNo)
{
    (void)IntNo;
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio,
                          uint32_t PortNo, uint32_t PinNo,
                          IOPINSENSE Sense,
                          IOPinEvtHandler_t pEvtCB, void *pCtx)
{
    (void)IntNo; (void)IntPrio; (void)PortNo; (void)PinNo;
    (void)Sense; (void)pEvtCB;  (void)pCtx;
    return false;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo,
                           IOPINSENSE Sense,
                           IOPinEvtHandler_t pEvtCB, void *pCtx)
{
    (void)IntPrio; (void)PortNo; (void)PinNo;
    (void)Sense;   (void)pEvtCB; (void)pCtx;
    return -1;
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
    (void)PortNo; (void)PinNo; (void)Sense;
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
    (void)PortNo; (void)PinNo; (void)Speed;
}
