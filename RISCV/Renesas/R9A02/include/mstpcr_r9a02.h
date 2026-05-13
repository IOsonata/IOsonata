/**-------------------------------------------------------------------------
@file	mstpcr_r9a02.h

@brief	R9A02G021 Module Stop Control helper.

Every Renesas RA-family peripheral sits behind a Module Stop Control
Register (MSTPCRA/B/C/D) bit that must be CLEARED before the peripheral
will respond to register accesses.  After reset, all module-stop bits
are SET (peripheral clocks gated off) -- writing peripheral registers
without first clearing the corresponding MSTP bit results in bus errors
or silently dropped writes depending on bus configuration.

This helper exposes a R9A02_Module_t enum that encodes both the target
register (MSTPCRA/B/C/D) and the bit position, plus three API calls:

    R9A02_ModuleStart(Module)   -- clear MSTP bit (enable peripheral clock)
    R9A02_ModuleStop(Module)    -- set MSTP bit   (gate peripheral clock)
    R9A02_ModuleIsStopped(Module)

Internally each call handles the PRCR(PRC1) unlock-write-relock sequence
needed for MSTPCRA (which lives inside the SYSTEM register block).
MSTPCRB/C/D do not require PRCR unlock but are still serialised through
the same code path.

Register addresses verified against Renesas FSP master,
mcu/ra2e1/R7FA2E1A9.h (R_MSTP base + offsets) and the SYSTEM register
block layout for MSTPCRA.

Bit assignments below cover the peripherals R9A02G021 actually populates;
RA2-family parts share these bit positions, see R9A02G021 User's Manual
section "Module Stop Control" for the authoritative table.

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
#ifndef __MSTPCR_R9A02_H__
#define __MSTPCR_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * MSTPCR register addresses (RA2-family layout, verified against FSP)
 *
 * Note: MSTPCRA lives inside the SYSTEM register block (offset 0x1C from
 * SYSC_BASE), NOT inside the R_MSTP block at 0x40047000.  FSP works around
 * this by offsetting R_MSTP_BASE so the struct's MSTPCRB lands at the
 * correct address; we use plain absolute addresses to avoid the trick.
 *==========================================================================*/
#define R9A02_MSTPCRA_ADDR          0x4001E01CUL    //!< Inside SYSC, needs PRCR unlock
#define R9A02_MSTPCRB_ADDR          0x40047000UL
#define R9A02_MSTPCRC_ADDR          0x40047004UL
#define R9A02_MSTPCRD_ADDR          0x40047008UL

/*===========================================================================
 * Module identifier encoding
 *
 * Layout: bits[6:5] = register index (0=A, 1=B, 2=C, 3=D)
 *         bits[4:0] = bit position within that register
 *
 * Construct via R9A02_MOD(reg, bit); decode via R9A02_MOD_REG / R9A02_MOD_BIT.
 *==========================================================================*/
#define R9A02_MOD_REG_A             0U
#define R9A02_MOD_REG_B             1U
#define R9A02_MOD_REG_C             2U
#define R9A02_MOD_REG_D             3U

#define R9A02_MOD(reg, bit)         (((uint8_t)(reg) << 5) | (uint8_t)((bit) & 0x1F))
#define R9A02_MOD_REG(m)            (((uint8_t)(m) >> 5) & 0x3U)
#define R9A02_MOD_BIT(m)            ((uint8_t)(m) & 0x1FU)

/*===========================================================================
 * R9A02G021 module identifiers
 *
 * Bit positions per RA2-family Module Stop Control register table.  Only
 * modules actually present on R9A02G021 are listed; reading/writing the
 * MSTP bit for an absent module is harmless but pointless.
 *==========================================================================*/
typedef enum {
    /* MSTPCRA -- system / DMA */
    R9A02_MOD_DTC               = R9A02_MOD(R9A02_MOD_REG_A, 22),   //!< Data Transfer Controller

    /* MSTPCRB -- comms peripherals */
    R9A02_MOD_IIC1              = R9A02_MOD(R9A02_MOD_REG_B,  8),
    R9A02_MOD_IIC0              = R9A02_MOD(R9A02_MOD_REG_B,  9),
    R9A02_MOD_SPI1              = R9A02_MOD(R9A02_MOD_REG_B, 18),
    R9A02_MOD_SPI0              = R9A02_MOD(R9A02_MOD_REG_B, 19),
    R9A02_MOD_SCI9              = R9A02_MOD(R9A02_MOD_REG_B, 22),
    R9A02_MOD_SCI2              = R9A02_MOD(R9A02_MOD_REG_B, 29),
    R9A02_MOD_SCI1              = R9A02_MOD(R9A02_MOD_REG_B, 30),
    R9A02_MOD_SCI0              = R9A02_MOD(R9A02_MOD_REG_B, 31),

    /* MSTPCRC -- misc */
    R9A02_MOD_CAC               = R9A02_MOD(R9A02_MOD_REG_C,  0),
    R9A02_MOD_CRC               = R9A02_MOD(R9A02_MOD_REG_C,  1),
    R9A02_MOD_CTSU              = R9A02_MOD(R9A02_MOD_REG_C,  3),
    R9A02_MOD_DOC               = R9A02_MOD(R9A02_MOD_REG_C, 13),
    R9A02_MOD_TRNG              = R9A02_MOD(R9A02_MOD_REG_C, 28),
    R9A02_MOD_AES               = R9A02_MOD(R9A02_MOD_REG_C, 31),

    /* MSTPCRD -- timers, analog */
    R9A02_MOD_AGT1              = R9A02_MOD(R9A02_MOD_REG_D,  2),
    R9A02_MOD_AGT0              = R9A02_MOD(R9A02_MOD_REG_D,  3),
    R9A02_MOD_GPT3_GPT0         = R9A02_MOD(R9A02_MOD_REG_D,  5),   //!< GPT3..GPT0 share one bit on RA2
    R9A02_MOD_POEG              = R9A02_MOD(R9A02_MOD_REG_D, 14),
    R9A02_MOD_ELC               = R9A02_MOD(R9A02_MOD_REG_D, 15),
    R9A02_MOD_ADC0              = R9A02_MOD(R9A02_MOD_REG_D, 16),
    R9A02_MOD_ACMPLP            = R9A02_MOD(R9A02_MOD_REG_D, 22)
} R9A02_Module_t;

/*===========================================================================
 * API
 *==========================================================================*/

/**
 * @brief	Enable a peripheral's clock by clearing its MSTP bit.
 *
 * After return the peripheral's registers respond to access.  Calling on a
 * module that is already started is a no-op.
 */
void R9A02_ModuleStart(R9A02_Module_t Module);

/**
 * @brief	Gate a peripheral's clock by setting its MSTP bit.
 *
 * Call after the peripheral has been brought to a safe quiescent state.
 * Stopping a module with active transactions can leave the bus hung.
 */
void R9A02_ModuleStop(R9A02_Module_t Module);

/**
 * @brief	Returns true if the module is currently stopped (clock gated).
 */
bool R9A02_ModuleIsStopped(R9A02_Module_t Module);

#ifdef __cplusplus
}
#endif

#endif // __MSTPCR_R9A02_H__
