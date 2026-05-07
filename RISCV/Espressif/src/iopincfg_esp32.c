/**-------------------------------------------------------------------------
@file	iopincfg_esp32.c

@brief	I/O pin configuration for Espressif RISC-V family
        (ESP32-C3, C5, C6, H2, H4)

The IO_MUX, GPIO matrix, and W1TS/W1TC/ENABLE register layouts are
common across the entire Espressif RISC-V line; only the valid pin
range differs.  This file supplies the slow-path IOPinConfig() that
the toolchain-common iopincfg.h declares.  Fast atomic operations
(IOPinSet / IOPinClear / IOPinToggle / IOPinSetDir / IOPinRead) are
inline in iopinctrl.h and need no implementation here.

Current scope: GPIO output and basic input with pull-up/pull-down,
sufficient for first-boot validation (Blinky) and the bring-up of
peripheral drivers that route signals through the GPIO matrix later.
Interrupt routing, drive-strength fine-tuning, and RTC IO are stubbed.

References:
  ESP32-C3 TRM Rev 0.4 §5  (IO_MUX) and §6 (GPIO Matrix)
  ESP32-C6 TRM Rev 0.6 §5  (same layout, more pads)

@author	Hoang Nguyen Hoan
@date	Mar. 2026

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

#include "coredev/iopincfg.h"

/*---------------------------------------------------------------------------
 * IO_MUX -- per-pad function selection and electrical configuration.
 * Pad N register: 0x60009004 + N*4
 * Field layout (TRM "IO MUX Pad Configuration Register"):
 *   [14:12] MCU_SEL  -- function selector. 1 = simple GPIO.
 *   [11:10] FUN_DRV  -- drive strength (0..3, ~5/10/20/40 mA).
 *   [9]     FUN_IE   -- input enable in normal operation.
 *   [8]     FUN_WPU  -- weak pull-up enable.
 *   [7]     FUN_WPD  -- weak pull-down enable.
 *---------------------------------------------------------------------------*/
#define ESP32_IOMUX_BASE        0x60009000UL
#define IOMUX_PAD_REG(n)        (*(volatile uint32_t *)(ESP32_IOMUX_BASE + 0x04U + ((unsigned)(n) * 4U)))

#define IOMUX_FUN_WPD_Pos       7
#define IOMUX_FUN_WPU_Pos       8
#define IOMUX_FUN_IE_Pos        9
#define IOMUX_FUN_DRV_Pos       10
#define IOMUX_FUN_DRV_Msk       (3UL << IOMUX_FUN_DRV_Pos)
#define IOMUX_MCU_SEL_Pos       12
#define IOMUX_MCU_SEL_Msk       (7UL << IOMUX_MCU_SEL_Pos)
#define IOMUX_MCU_SEL_GPIO      (1UL << IOMUX_MCU_SEL_Pos)

/*---------------------------------------------------------------------------
 * GPIO matrix (base 0x60004000).
 *   GPIO_FUNCn_OUT_SEL_CFG : 0x60004554 + N*4   -- output signal routing
 *   GPIO_PINn_REG          : 0x60004074 + N*4   -- per-pin: PAD_DRIVER (OD) etc.
 *   GPIO_ENABLE_W1TS_REG   : 0x60004024
 *   GPIO_ENABLE_W1TC_REG   : 0x60004028
 *
 * SIG_GPIO_OUT_IDX = 128 makes a pad a "simple GPIO output" -- driven
 * directly from GPIO_OUT_REG instead of a peripheral signal.  Same
 * value across the supported RISC-V chips (C3/C5/C6/H2/H4).  The
 * original Xtensa ESP32 used 256; do NOT use that here.
 *---------------------------------------------------------------------------*/
#define ESP32_GPIO_BASE         0x60004000UL
#define GPIO_FUNC_OUT_SEL(n)    (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x554U + ((unsigned)(n) * 4U)))
#define GPIO_PIN_CFG(n)         (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x074U + ((unsigned)(n) * 4U)))
#define GPIO_ENABLE_W1TS_REG    (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x024U))
#define GPIO_ENABLE_W1TC_REG    (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x028U))

#define GPIO_OUT_SEL_Msk        0xFFUL    /* OUT_SEL is bits [7:0] on RISC-V ESP32 */
#define GPIO_OEN_SEL_Pos        9
#define GPIO_OEN_INV_SEL_Pos    10
#define GPIO_OUT_INV_SEL_Pos    11

#define GPIO_PAD_DRIVER_Pos     2   /* GPIO_PINn_REG: 1 = open-drain output */

/* Signal index that means "drive this pad directly from GPIO_OUT_REG"
 * (instead of routing a peripheral signal through the GPIO matrix).
 * RISC-V ESP32 family has 128 peripheral output signals; index 128 is
 * the simple-GPIO-output magic.  The original Xtensa ESP32 used 256
 * because it has more signals — DO NOT use 256 here. */
#define SIG_GPIO_OUT_IDX        128U

/*---------------------------------------------------------------------------
 * Pin range guard.  Only the upper bound differs per chip.
 *---------------------------------------------------------------------------*/
#if   defined(SOC_ESP32C3) || defined(__ESP32C3__)
#  define ESP32_MAX_PIN  21
#elif defined(SOC_ESP32C5) || defined(__ESP32C5__)
#  define ESP32_MAX_PIN  28
#elif defined(SOC_ESP32C6) || defined(__ESP32C6__)
#  define ESP32_MAX_PIN  29
#elif defined(SOC_ESP32H2) || defined(__ESP32H2__)
#  define ESP32_MAX_PIN  18
#elif defined(SOC_ESP32H4) || defined(__ESP32H4__)
#  define ESP32_MAX_PIN  35
#else
#  define ESP32_MAX_PIN  35   /* safe upper bound for the family */
#endif


/**
 * @brief	Configure a single GPIO pin.
 *
 * @param	PortNo   Always 0 on Espressif RISC-V (single GPIO port). Ignored.
 * @param	PinNo    GPIO pad number, 0..ESP32_MAX_PIN.
 * @param	PinOp    IOPINOP_GPIO (0) for plain GPIO.  Non-zero values are
 *                   reserved for peripheral function routing through the
 *                   GPIO matrix; not implemented in this revision.
 * @param	Dir      IOPINDIR_INPUT, IOPINDIR_OUTPUT, or IOPINDIR_BI.
 * @param	Resistor IOPINRES_NONE / IOPINRES_PULLUP / IOPINRES_PULLDOWN.
 * @param	Type     IOPINTYPE_NORMAL or IOPINTYPE_OPENDRAIN.
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp,
                 IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
    (void)PortNo;
    (void)PinOp;    /* peripheral-signal routing not implemented yet */

    if (PinNo < 0 || PinNo > ESP32_MAX_PIN)
    {
        return;
    }

    uint32_t mask = 1UL << (unsigned)PinNo;

    /*
     * Build the IO_MUX pad value:
     *   - select GPIO function (MCU_SEL = 1)
     *   - default drive strength 2 (~20 mA)
     *   - input enable per direction
     *   - pull-up / pull-down per Resistor
     */
    uint32_t iomux = IOMUX_MCU_SEL_GPIO
                   | (2UL << IOMUX_FUN_DRV_Pos);

    if (Dir == IOPINDIR_INPUT || Dir == IOPINDIR_BI)
    {
        iomux |= (1UL << IOMUX_FUN_IE_Pos);
    }

    if (Resistor == IOPINRES_PULLUP)
    {
        iomux |= (1UL << IOMUX_FUN_WPU_Pos);
    }
    else if (Resistor == IOPINRES_PULLDOWN)
    {
        iomux |= (1UL << IOMUX_FUN_WPD_Pos);
    }

    IOMUX_PAD_REG(PinNo) = iomux;

    /*
     * GPIO matrix: route simple GPIO out to this pad (bypass peripheral
     * signals).  OUT_SEL = SIG_GPIO_OUT_IDX, OEN_SEL = 0 (use GPIO_ENABLE
     * register), no inversion.
     */
    GPIO_FUNC_OUT_SEL(PinNo) = SIG_GPIO_OUT_IDX & GPIO_OUT_SEL_Msk;

    /* Open-drain: GPIO_PINn_REG.PAD_DRIVER = 1 */
    if (Type == IOPINTYPE_OPENDRAIN)
    {
        GPIO_PIN_CFG(PinNo) |= (1UL << GPIO_PAD_DRIVER_Pos);
    }
    else
    {
        GPIO_PIN_CFG(PinNo) &= ~(1UL << GPIO_PAD_DRIVER_Pos);
    }

    /* Enable / disable the output driver. */
    if (Dir == IOPINDIR_OUTPUT || Dir == IOPINDIR_BI)
    {
        GPIO_ENABLE_W1TS_REG = mask;
    }
    else
    {
        GPIO_ENABLE_W1TC_REG = mask;
    }
}


/**
 * @brief	Release a pin: disable output, drop pulls, deselect peripheral.
 */
void IOPinDisable(int PortNo, int PinNo)
{
    (void)PortNo;

    if (PinNo < 0 || PinNo > ESP32_MAX_PIN)
    {
        return;
    }

    uint32_t mask = 1UL << (unsigned)PinNo;

    GPIO_ENABLE_W1TC_REG = mask;
    IOMUX_PAD_REG(PinNo) = 0U;
    GPIO_FUNC_OUT_SEL(PinNo) = SIG_GPIO_OUT_IDX & GPIO_OUT_SEL_Msk;
}


/**
 * @brief	Set drive strength.  Strong = 40 mA bracket, Regular = 20 mA.
 */
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
    (void)PortNo;

    if (PinNo < 0 || PinNo > ESP32_MAX_PIN)
    {
        return;
    }

    uint32_t v = IOMUX_PAD_REG(PinNo) & ~IOMUX_FUN_DRV_Msk;
    v |= ((Strength == IOPINSTRENGTH_STRONG) ? 3UL : 2UL) << IOMUX_FUN_DRV_Pos;
    IOMUX_PAD_REG(PinNo) = v;
}


/*---------------------------------------------------------------------------
 * Stubs -- to be implemented when pin interrupts and per-pad slew/speed
 * control become required.  Defining them keeps the link clean for any
 * library code that references the full iopincfg.h API.
 *---------------------------------------------------------------------------*/

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
