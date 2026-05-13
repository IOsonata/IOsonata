/**-------------------------------------------------------------------------
@file	mstpcr_r9a02.c

@brief	R9A02G021 Module Stop Control implementation.

See mstpcr_r9a02.h for the API description.

@author	Nguyen Hoan Hoang
@date	May 12, 2026

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
#include "mstpcr_r9a02.h"

/* PRCR -- protect register: 16-bit, top 8 bits = key (0xA5), low 8 = enables.
 * Only PRC1 (bit 1) gates writes to MSTPCRA / low-power-mode registers. */
#define R9A02_PRCR_ADDR             0x4001E3FEUL
#define R9A02_PRCR_KEY              0xA500U
#define R9A02_PRCR_PRC1             (1U << 1)

#define R9A02_REG32(addr)           (*(volatile uint32_t *)(uintptr_t)(addr))
#define R9A02_REG16(addr)           (*(volatile uint16_t *)(uintptr_t)(addr))

/*---------------------------------------------------------------------------
 * Internal: map register index to memory address.
 *---------------------------------------------------------------------------*/
static uint32_t Mod_RegAddr(uint8_t RegIdx)
{
    switch (RegIdx)
    {
        case R9A02_MOD_REG_A:   return R9A02_MSTPCRA_ADDR;
        case R9A02_MOD_REG_B:   return R9A02_MSTPCRB_ADDR;
        case R9A02_MOD_REG_C:   return R9A02_MSTPCRC_ADDR;
        case R9A02_MOD_REG_D:   return R9A02_MSTPCRD_ADDR;
        default:                return 0;
    }
}

/*---------------------------------------------------------------------------
 * Internal: read-modify-write a single MSTP bit, handling PRCR for MSTPCRA.
 *---------------------------------------------------------------------------*/
static void Mod_SetBit(R9A02_Module_t Module, bool SetStop)
{
    uint8_t  reg_idx = R9A02_MOD_REG(Module);
    uint8_t  bit     = R9A02_MOD_BIT(Module);
    uint32_t addr    = Mod_RegAddr(reg_idx);
    uint32_t mask    = 1UL << bit;
    bool     need_prcr = (reg_idx == R9A02_MOD_REG_A);

    if (addr == 0U)
    {
        return;     /* Invalid module ID */
    }

    if (need_prcr)
    {
        R9A02_REG16(R9A02_PRCR_ADDR) = R9A02_PRCR_KEY | R9A02_PRCR_PRC1;
    }

    if (SetStop)
    {
        R9A02_REG32(addr) |= mask;
    }
    else
    {
        R9A02_REG32(addr) &= ~mask;
    }

    if (need_prcr)
    {
        R9A02_REG16(R9A02_PRCR_ADDR) = R9A02_PRCR_KEY;      /* re-lock */
    }

    /* Renesas RA manuals require a dummy read-back after MSTP write to
     * guarantee the change is visible before the next peripheral access. */
    (void)R9A02_REG32(addr);
}

/*===========================================================================
 * Public API
 *==========================================================================*/

void R9A02_ModuleStart(R9A02_Module_t Module)
{
    Mod_SetBit(Module, false);      /* clear MSTP bit -> clock running */
}

void R9A02_ModuleStop(R9A02_Module_t Module)
{
    Mod_SetBit(Module, true);       /* set MSTP bit   -> clock gated   */
}

bool R9A02_ModuleIsStopped(R9A02_Module_t Module)
{
    uint8_t  reg_idx = R9A02_MOD_REG(Module);
    uint8_t  bit     = R9A02_MOD_BIT(Module);
    uint32_t addr    = Mod_RegAddr(reg_idx);

    if (addr == 0U)
    {
        return true;
    }

    return (R9A02_REG32(addr) & (1UL << bit)) != 0U;
}
