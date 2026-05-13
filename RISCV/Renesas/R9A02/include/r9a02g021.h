/**-------------------------------------------------------------------------
@file	r9a02g021.h

@brief	Renesas R9A02G021 RISC-V MCU - device-level CMSIS-style header.

Ultra-low-power 48 MHz general-purpose MCU built around the Renesas
in-house Andes N22 32-bit RISC-V core (RV32I + M/A/C/B extensions, dynamic
branch prediction, stack monitor, CLIC-style interrupt controller).

R9A02G021 is RA2-family compatible at the peripheral level -- its memory
map mirrors the Cortex-M0+ RA2E1/RA2E2 parts, with the ARM CMSIS-NVIC
replaced by the Andes RISC-V CLIC routing the ICU/ELC event sources.

This header is the canonical reference for IOsonata driver code targeting
R9A02G021.  It provides:

  - Comprehensive RA2-family peripheral base addresses.
  - PORT_Type / PFS_Type / PMISC_Type register structures.
  - ICU register block with PWPR unlock sequence values.
  - Event Link Source enumeration (ELC_EVENT_*).
  - PFS PSEL function-select code definitions.
  - Peripheral instance counts specific to R9A02G021.
  - Andes N22 / CLIC base addresses (machine-mode CSR conventions).

Source-of-truth:
  - R9A02G021 User's Manual: Hardware (Renesas R01UH1059EJ, Rev 1.10).
  - Renesas FSP master branch, mcu/ra2e1 (closest RA-family sibling)
    -- ra/fsp/src/bsp/cmsis/Device/RENESAS/Include/R7FA2E1A9.h
    -- ra/fsp/src/bsp/mcu/ra2e1/bsp_elc.h
  - https://www.renesas.com/products/r9a02g021

NOTE: R9A02G021 implements a SUBSET of the RA2-family register map.
Peripheral base addresses are listed for every RA2 block at its
standardized offset; the instance-count section below indicates which
blocks are actually populated on this device.  Reading an unmapped block
will return zero or raise a bus fault depending on the bus controller
configuration.

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
#ifndef __R9A02G021_H__
#define __R9A02G021_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __IO
#define __IO        volatile
#define __I         volatile const
#define __O         volatile
#endif

/*===========================================================================
 *                     CPU / SYSTEM CONFIGURATION
 *==========================================================================*/

/* Andes N22 RISC-V core. */
#define __ANDES_N22                 1
#define __ICACHE_PRESENT            0       //!< No instruction cache on N22
#define __DCACHE_PRESENT            0
#define __FPU_PRESENT               0
#define __MPU_PRESENT               1
#define __VECTABLE_OFFSET           0

/* Maximum HOCO frequency (chip operates from HOCO; no external XTAL). */
#define R9A02_HOCO_FREQ_HZ          48000000UL
#define R9A02_MOCO_FREQ_HZ          8000000UL
#define R9A02_LOCO_FREQ_HZ          32768UL
#define R9A02_SUBCLK_FREQ_HZ        32768UL

/* SRAM and flash sizes -- maximum across R9A02G021 part-number variants.
 * Specific part numbers may have less. */
#define R9A02_FLASH_BASE            0x00000000UL
#define R9A02_FLASH_SIZE_MAX        (256U * 1024U)
#define R9A02_DATA_FLASH_BASE       0x40100000UL
#define R9A02_DATA_FLASH_SIZE_MAX   (4U * 1024U)
#define R9A02_SRAM_BASE             0x20000000UL
#define R9A02_SRAM_SIZE_MAX         (32U * 1024U)
#define R9A02_OPTION_BYTES_BASE     0x0100A100UL    //!< OFS0/OFS1/SECMPUS settings

/*===========================================================================
 *                      PERIPHERAL BASE ADDRESSES
 *
 * Addresses are RA2-family standardized.  All values are verified against
 * Renesas FSP RA2E1 (mcu/ra2e1) which uses the identical map.
 *==========================================================================*/

/* Bus / memory protection. */
#define R9A02_MPU_MMPU_BASE         0x40000000UL    //!< Memory MPU (bus master)
#define R9A02_MPU_SMPU_BASE         0x40000C00UL    //!< System MPU (slave)
#define R9A02_MPU_SPMON_BASE        0x40000D00UL    //!< Stack-pointer monitor
#define R9A02_BUS_BASE              0x40003000UL
#define R9A02_DTC_BASE              0x40005400UL    //!< Data Transfer Controller
#define R9A02_ICU_BASE              0x40006000UL    //!< Interrupt Controller Unit
#define R9A02_DEBUG_BASE            0x4001B000UL
#define R9A02_SYSTEM_BASE           0x4001E000UL    //!< SYSC: clock, reset, low-power, OPCCR/SOPCCR
#define R9A02_SYSC_BASE             R9A02_SYSTEM_BASE

/* I/O ports. */
#define R9A02_PORT_BASE             0x40040000UL
#define R9A02_PORT0_BASE            (R9A02_PORT_BASE + 0x000UL)
#define R9A02_PORT1_BASE            (R9A02_PORT_BASE + 0x020UL)
#define R9A02_PORT2_BASE            (R9A02_PORT_BASE + 0x040UL)
#define R9A02_PORT3_BASE            (R9A02_PORT_BASE + 0x060UL)
#define R9A02_PORT4_BASE            (R9A02_PORT_BASE + 0x080UL)
#define R9A02_PORT5_BASE            (R9A02_PORT_BASE + 0x0A0UL)
#define R9A02_PORT6_BASE            (R9A02_PORT_BASE + 0x0C0UL)
#define R9A02_PORT7_BASE            (R9A02_PORT_BASE + 0x0E0UL)
#define R9A02_PORT8_BASE            (R9A02_PORT_BASE + 0x100UL)
#define R9A02_PFS_BASE              0x40040800UL    //!< Port Function Select array
#define R9A02_PMISC_BASE            0x40040D00UL    //!< Port miscellaneous (PWPR, etc.)
#define R9A02_ELC_BASE              0x40041000UL    //!< Event Link Controller
#define R9A02_GPT_POEG0_BASE        0x40042000UL    //!< GPT output enable group 0
#define R9A02_GPT_POEG1_BASE        0x40042100UL
#define R9A02_GPT_POEG2_BASE        0x40042200UL
#define R9A02_GPT_POEG3_BASE        0x40042300UL

/* Timers / watchdogs / RTC / CAC. */
#define R9A02_RTC_BASE              0x40044000UL    //!< RTC
#define R9A02_WDT_BASE              0x40044200UL    //!< Watchdog timer
#define R9A02_IWDT_BASE             0x40044400UL    //!< Independent watchdog timer
#define R9A02_CAC_BASE              0x40044600UL    //!< Clock-accuracy measurement

/* Communications. */
#define R9A02_IIC0_BASE             0x40053000UL    //!< I2C bus 0
#define R9A02_IIC1_BASE             0x40053100UL    //!< I2C bus 1 (if populated)
#define R9A02_DOC_BASE              0x40054100UL    //!< Data Operation Circuit

/* Analog. */
#define R9A02_ADC0_BASE             0x4005C000UL    //!< 12-bit SAR ADC
#define R9A02_ACMPLP_BASE           0x40085E00UL    //!< Low-power analog comparator

/* SCI: RA2 places SCI0..SCI9 at 0x40070000 + n*0x20.  R9A02G021 populates
 * SCI0, SCI1, SCI2, SCI9.  Other slots read as zero. */
#define R9A02_SCI_BASE              0x40070000UL
#define R9A02_SCI0_BASE             (R9A02_SCI_BASE + 0x00UL)
#define R9A02_SCI1_BASE             (R9A02_SCI_BASE + 0x20UL)
#define R9A02_SCI2_BASE             (R9A02_SCI_BASE + 0x40UL)
#define R9A02_SCI3_BASE             (R9A02_SCI_BASE + 0x60UL)
#define R9A02_SCI4_BASE             (R9A02_SCI_BASE + 0x80UL)
#define R9A02_SCI5_BASE             (R9A02_SCI_BASE + 0xA0UL)
#define R9A02_SCI6_BASE             (R9A02_SCI_BASE + 0xC0UL)
#define R9A02_SCI7_BASE             (R9A02_SCI_BASE + 0xE0UL)
#define R9A02_SCI8_BASE             (R9A02_SCI_BASE + 0x100UL)
#define R9A02_SCI9_BASE             (R9A02_SCI_BASE + 0x120UL)

/* SPI: 0x40072000 + n*0x100, R9A02G021 populates SPI0, SPI1. */
#define R9A02_SPI0_BASE             0x40072000UL
#define R9A02_SPI1_BASE             0x40072100UL
#define R9A02_SPI2_BASE             0x40072200UL

#define R9A02_CRC_BASE              0x40074000UL    //!< CRC calculator

/* GPT: 0x40078000 + n*0x100.  R9A02G021 populates GPT0..GPT3. */
#define R9A02_GPT_BASE              0x40078000UL
#define R9A02_GPT0_BASE             (R9A02_GPT_BASE + 0x000UL)
#define R9A02_GPT1_BASE             (R9A02_GPT_BASE + 0x100UL)
#define R9A02_GPT2_BASE             (R9A02_GPT_BASE + 0x200UL)
#define R9A02_GPT3_BASE             (R9A02_GPT_BASE + 0x300UL)
#define R9A02_GPT4_BASE             (R9A02_GPT_BASE + 0x400UL)
#define R9A02_GPT5_BASE             (R9A02_GPT_BASE + 0x500UL)
#define R9A02_GPT6_BASE             (R9A02_GPT_BASE + 0x600UL)
#define R9A02_GPT7_BASE             (R9A02_GPT_BASE + 0x700UL)
#define R9A02_GPT_OPS_BASE          0x40078FF0UL    //!< GPT output-protection-state

#define R9A02_KINT_BASE             0x40080000UL    //!< Key matrix interrupt
#define R9A02_CTSU2_BASE            0x40082000UL    //!< Capacitive Touch Sensing Unit (if present)

/* AGT (asynchronous general-purpose timer).  0x40084000 + n*0x100.
 * R9A02G021 populates AGT0, AGT1. */
#define R9A02_AGTX0_BASE            0x40084000UL
#define R9A02_AGT0_BASE             R9A02_AGTX0_BASE
#define R9A02_AGTX1_BASE            0x40084100UL
#define R9A02_AGT1_BASE             R9A02_AGTX1_BASE

/* Crypto. */
#define R9A02_AES_BASE              0x400D0000UL    //!< AES (CAU) hardware accelerator
#define R9A02_TRNG_BASE             0x400D1000UL    //!< True random number generator

/* Flash control.  R9A02G021 is a low-power part so uses FACI_LP / FLCN,
 * not the HP variant.  This base is also where TSN (temperature sensor)
 * trim values live. */
#define R9A02_FACI_LP_BASE          0x407EC000UL
#define R9A02_FLCN_BASE             R9A02_FACI_LP_BASE
#define R9A02_TSN_BASE              R9A02_FACI_LP_BASE
#define R9A02_CTSUTRIM_BASE         R9A02_FACI_LP_BASE

/*===========================================================================
 *                  CLIC -- Andes RISC-V interrupt controller
 *
 * The Andes N22 implements a CLIC-style interrupt controller, not the
 * standard RISC-V PLIC.  Memory-mapped CLIC region on R9A02G021 begins
 * at 0xE0000000 per the Andes N22 reference; the exact register layout
 * follows the AndeStar CLIC specification (mcause/mintstatus CSRs are
 * also used in tandem with the memory-mapped block).
 *
 * Note: Verify the CLIC_BASE value against the R9A02G021 manual section
 * 19 (ICU) before using CLIC memory-mapped registers; this header
 * intentionally limits CLIC use to CSR-based access for portability.
 *==========================================================================*/
#define R9A02_CLIC_BASE             0xE0000000UL    //!< CLIC memory-mapped block
#define R9A02_CLIC_MAX_INT          240U            //!< Andes N22 CLIC supports up to 240 sources

/*===========================================================================
 *                      I/O PORT REGISTER BLOCK
 *
 * Layout shared across all RA2 PORT0..PORT8 instances (16 bytes each).
 *
 * PCNTR1   - bits[15:0]  PDR   port direction (1=output, 0=input)
 *            bits[31:16] PODR  output data (1=high, 0=low)
 * PCNTR2   - bits[15:0]  PIDR  input data (read-only)
 *            bits[31:16] EIDR  event input
 * PCNTR3   - bits[15:0]  PORR  output reset (write 1 to clear, atomic)
 *            bits[31:16] POSR  output set   (write 1 to set,   atomic)
 * EORR     - event output reset (16-bit)
 * EOSR     - event output set   (16-bit)
 *
 * The PORR/POSR atomic set/clear registers are the preferred fast path
 * for IOPinSet / IOPinClear -- they avoid the read-modify-write hazard
 * that plagues the PODR approach.
 *==========================================================================*/
typedef struct {
    union {
        __IO uint32_t   PCNTR1;             //!< 0x00 Port control 1 (PDR | PODR)
        struct {
            __IO uint16_t   PDR;            //!< 0x00 Direction (1=out, 0=in)
            __IO uint16_t   PODR;           //!< 0x02 Output data
        };
    };
    union {
        __I  uint32_t   PCNTR2;             //!< 0x04 Port control 2 (PIDR | EIDR)
        struct {
            __I  uint16_t   PIDR;           //!< 0x04 Input data
            __IO uint16_t   EIDR;           //!< 0x06 Event input data
        };
    };
    union {
        __O  uint32_t   PCNTR3;             //!< 0x08 Port control 3 (PORR | POSR)
        struct {
            __O  uint16_t   PORR;           //!< 0x08 Output reset (W1C)
            __O  uint16_t   POSR;           //!< 0x0A Output set   (W1S)
        };
    };
    __IO uint16_t       EORR;               //!< 0x0C Event output reset
    __IO uint16_t       EOSR;               //!< 0x0E Event output set
} R9A02_PORT_Type;

#define R9A02_PORT0                 ((R9A02_PORT_Type *) R9A02_PORT0_BASE)
#define R9A02_PORT1                 ((R9A02_PORT_Type *) R9A02_PORT1_BASE)
#define R9A02_PORT2                 ((R9A02_PORT_Type *) R9A02_PORT2_BASE)
#define R9A02_PORT3                 ((R9A02_PORT_Type *) R9A02_PORT3_BASE)
#define R9A02_PORT4                 ((R9A02_PORT_Type *) R9A02_PORT4_BASE)
#define R9A02_PORT5                 ((R9A02_PORT_Type *) R9A02_PORT5_BASE)
#define R9A02_PORT6                 ((R9A02_PORT_Type *) R9A02_PORT6_BASE)
#define R9A02_PORT7                 ((R9A02_PORT_Type *) R9A02_PORT7_BASE)
#define R9A02_PORT8                 ((R9A02_PORT_Type *) R9A02_PORT8_BASE)

/* Indexed accessor: R9A02_PORT(n) returns the same pointer as R9A02_PORT<n>.
 * Used by iopinctrl.h for variable-port-index GPIO calls (IOPinSetDir, etc.).
 * Each PORT instance occupies 0x20 bytes; the spacing is RA2-family standard. */
#define R9A02_PORT(n)               ((R9A02_PORT_Type *)(R9A02_PORT_BASE + ((uint32_t)(n) * 0x20UL)))

/*===========================================================================
 *                      PFS - Port Function Select
 *
 * One 32-bit PFS register per pin: PFS[port * 16 + pin].  PMR / ASEL /
 * ISEL / EOFR / DSCR / NCODR / PCR / PDR / PIDR / PODR all map into one
 * word per pin.
 *==========================================================================*/

/* PFS register bit positions. */
#define PFS_PODR_Pos                0U
#define PFS_PODR_Msk                (1UL << PFS_PODR_Pos)
#define PFS_PIDR_Pos                1U
#define PFS_PIDR_Msk                (1UL << PFS_PIDR_Pos)
#define PFS_PDR_Pos                 2U
#define PFS_PDR_Msk                 (1UL << PFS_PDR_Pos)
#define PFS_PCR_Pos                 4U
#define PFS_PCR_Msk                 (1UL << PFS_PCR_Pos)
#define PFS_NCODR_Pos               6U      //!< N-channel open-drain
#define PFS_NCODR_Msk               (1UL << PFS_NCODR_Pos)
#define PFS_DSCR_Pos                10U     //!< Drive strength [11:10]
#define PFS_DSCR_Msk                (3UL << PFS_DSCR_Pos)
#define PFS_DSCR_LOW                (0UL << PFS_DSCR_Pos)
#define PFS_DSCR_MID                (1UL << PFS_DSCR_Pos)
#define PFS_DSCR_HIGH               (3UL << PFS_DSCR_Pos)
#define PFS_EOFR_Pos                12U     //!< Event-on edge [13:12]
#define PFS_EOFR_Msk                (3UL << PFS_EOFR_Pos)
#define PFS_EOFR_RISING             (1UL << PFS_EOFR_Pos)
#define PFS_EOFR_FALLING            (2UL << PFS_EOFR_Pos)
#define PFS_EOFR_BOTH               (3UL << PFS_EOFR_Pos)
#define PFS_ISEL_Pos                14U     //!< IRQ input select
#define PFS_ISEL_Msk                (1UL << PFS_ISEL_Pos)
#define PFS_ASEL_Pos                15U     //!< Analog (ADC) select
#define PFS_ASEL_Msk                (1UL << PFS_ASEL_Pos)
#define PFS_PMR_Pos                 16U     //!< Peripheral select (vs. GPIO)
#define PFS_PMR_Msk                 (1UL << PFS_PMR_Pos)
#define PFS_PSEL_Pos                24U     //!< Peripheral selector [28:24]
#define PFS_PSEL_Msk                (0x1FUL << PFS_PSEL_Pos)

/* PSEL function codes -- write to PFS.PSEL[28:24] together with PMR=1 to
 * route the pin to the listed alternate function.  These are RA2-family
 * standardized codes; R9A02G021 supports a subset matching its peripherals.
 */
#define PFS_PSEL_GPIO               (0x00UL << PFS_PSEL_Pos)    //!< Use PMR=0
#define PFS_PSEL_AGT                (0x01UL << PFS_PSEL_Pos)
#define PFS_PSEL_GPT0               (0x02UL << PFS_PSEL_Pos)
#define PFS_PSEL_GPT1               (0x03UL << PFS_PSEL_Pos)
#define PFS_PSEL_RTC                (0x04UL << PFS_PSEL_Pos)
#define PFS_PSEL_ADC                (0x05UL << PFS_PSEL_Pos)    //!< Analog-trigger
#define PFS_PSEL_IIC                (0x07UL << PFS_PSEL_Pos)
#define PFS_PSEL_KINT               (0x08UL << PFS_PSEL_Pos)
#define PFS_PSEL_CLKOUT             (0x09UL << PFS_PSEL_Pos)
#define PFS_PSEL_CAC_REFCLK         (0x0AUL << PFS_PSEL_Pos)
#define PFS_PSEL_SCI0               (0x0BUL << PFS_PSEL_Pos)
#define PFS_PSEL_SCI1               (0x0CUL << PFS_PSEL_Pos)
#define PFS_PSEL_SCI2               (0x0DUL << PFS_PSEL_Pos)
#define PFS_PSEL_SCI9               (0x0EUL << PFS_PSEL_Pos)
#define PFS_PSEL_SPI                (0x0FUL << PFS_PSEL_Pos)
#define PFS_PSEL_CTSU               (0x12UL << PFS_PSEL_Pos)
#define PFS_PSEL_ACMPLP             (0x13UL << PFS_PSEL_Pos)
#define PFS_PSEL_TRACE              (0x18UL << PFS_PSEL_Pos)

/* R9A02_-prefixed aliases for the PFS bitfield positions and masks.
 * These match the naming convention used in iopincfg_r9a02.c and other
 * IOsonata Renesas drivers.  The non-prefixed PFS_* names above are kept
 * for backward compatibility. */
#define R9A02_PFS_PODR_Pos          PFS_PODR_Pos
#define R9A02_PFS_PODR_Msk          PFS_PODR_Msk
#define R9A02_PFS_PIDR_Pos          PFS_PIDR_Pos
#define R9A02_PFS_PIDR_Msk          PFS_PIDR_Msk
#define R9A02_PFS_PDR_Pos           PFS_PDR_Pos
#define R9A02_PFS_PDR_Msk           PFS_PDR_Msk
#define R9A02_PFS_PCR_Pos           PFS_PCR_Pos
#define R9A02_PFS_PCR_Msk           PFS_PCR_Msk
#define R9A02_PFS_NCODR_Pos         PFS_NCODR_Pos
#define R9A02_PFS_NCODR_Msk         PFS_NCODR_Msk
#define R9A02_PFS_DSCR_Pos          PFS_DSCR_Pos
#define R9A02_PFS_DSCR_Msk          PFS_DSCR_Msk
#define R9A02_PFS_DSCR_LOW          PFS_DSCR_LOW
#define R9A02_PFS_DSCR_MID          PFS_DSCR_MID
#define R9A02_PFS_DSCR_HIGH         PFS_DSCR_HIGH
#define R9A02_PFS_EOFR_Pos          PFS_EOFR_Pos
#define R9A02_PFS_EOFR_Msk          PFS_EOFR_Msk
#define R9A02_PFS_EOFR_RISING       PFS_EOFR_RISING
#define R9A02_PFS_EOFR_FALLING      PFS_EOFR_FALLING
#define R9A02_PFS_EOFR_BOTH         PFS_EOFR_BOTH
#define R9A02_PFS_ISEL_Pos          PFS_ISEL_Pos
#define R9A02_PFS_ISEL_Msk          PFS_ISEL_Msk
#define R9A02_PFS_ASEL_Pos          PFS_ASEL_Pos
#define R9A02_PFS_ASEL_Msk          PFS_ASEL_Msk
#define R9A02_PFS_PMR_Pos           PFS_PMR_Pos
#define R9A02_PFS_PMR_Msk           PFS_PMR_Msk
#define R9A02_PFS_PSEL_Pos          PFS_PSEL_Pos
#define R9A02_PFS_PSEL_Msk          PFS_PSEL_Msk

/* PFS register addressing helper.
 * Each pin has a 32-bit PFS register at offset (port * 16 + pin) * 4 from
 * the PFS array base.  Used by drivers that need per-pin access by
 * port/pin index. */
#define R9A02_PFS_REG_ADDR(port, pin) \
    ((volatile uint32_t *)(R9A02_PFS_BASE + \
     (((uint32_t)(port) * 16U + (uint32_t)(pin)) * 4U)))

/*===========================================================================
 *                      PMISC - Port Misc registers (PWPR)
 *
 * The Port-Function-Select Write Protect Register gates all writes to
 * PFS.  Without unlocking, PFS writes are ignored silently.
 *
 * Unlock sequence:
 *      PMISC->PWPR = 0;                  // clear B0WI
 *      PMISC->PWPR = PFS_PWPR_PFSWE;     // set PFSWE
 *      ... write PFS register(s) ...
 *      PMISC->PWPR = 0;                  // re-arm
 *      PMISC->PWPR = PFS_PWPR_B0WI;      // lock B0WI again
 *==========================================================================*/
typedef struct {
    __I  uint8_t        RESERVED0[0x03];
    __IO uint8_t        PWPR;               //!< 0x03 Pin function select write-protect
    __I  uint8_t        RESERVED1[0xFC];
    __IO uint16_t       PFENET;             //!< 0x100 Ethernet pin function enable
    __I  uint8_t        RESERVED2[0x0E];
    __IO uint16_t       PRWCNTR;            //!< 0x110 Power-on pin reset wait
} R9A02_PMISC_Type;

#define R9A02_PMISC                 ((R9A02_PMISC_Type *) R9A02_PMISC_BASE)
#define PMISC                       R9A02_PMISC     //!< Short alias matching IOsonata RA driver naming

#define PFS_PWPR_PFSWE_Pos          6U
#define PFS_PWPR_PFSWE              (1UL << PFS_PWPR_PFSWE_Pos)
#define PFS_PWPR_B0WI_Pos           7U
#define PFS_PWPR_B0WI               (1UL << PFS_PWPR_B0WI_Pos)

/* R9A02_-prefixed aliases for the PMISC PWPR fields, matching driver naming. */
#define R9A02_PMISC_PWPR_PFSWE_Pos  PFS_PWPR_PFSWE_Pos
#define R9A02_PMISC_PWPR_PFSWE_Msk  PFS_PWPR_PFSWE
#define R9A02_PMISC_PWPR_B0WI_Pos   PFS_PWPR_B0WI_Pos
#define R9A02_PMISC_PWPR_B0WI_Msk   PFS_PWPR_B0WI

/*===========================================================================
 *                      ICU - Interrupt Controller Unit
 *
 * ICU sits between the peripheral events and the CLIC.  Each CLIC
 * vector slot has an IELSR (Event Link Source) register that selects
 * which peripheral event triggers it.  IRQCRi controls the eight
 * external IRQ pins (IRQ0..IRQ7) including edge/level configuration.
 *==========================================================================*/
typedef struct {
    __IO uint8_t        IRQCR[8];           //!< 0x000 IRQ control 0..7 (edge sel)
    __I  uint8_t        RESERVED0[0x08];
    __IO uint8_t        NMISR;              //!< 0x10  NMI status
    __I  uint8_t        RESERVED1[0x01];
    __IO uint8_t        NMIER;              //!< 0x12  NMI enable
    __I  uint8_t        RESERVED2[0x01];
    __IO uint8_t        NMICLR;             //!< 0x14  NMI status clear
    __I  uint8_t        RESERVED3[0x01];
    __IO uint8_t        NMICR;              //!< 0x16  NMI pin control
    __I  uint8_t        RESERVED4[0xE9];
    __IO uint32_t       IELSR[32];          //!< 0x100 Event link source 0..31
} R9A02_ICU_Type;

#define R9A02_ICU                   ((R9A02_ICU_Type *) R9A02_ICU_BASE)

/* IRQCR bit positions. */
#define ICU_IRQCR_IRQMD_Pos         0U      //!< Sense mode [1:0]
#define ICU_IRQCR_IRQMD_LOWLEVEL    (0UL << ICU_IRQCR_IRQMD_Pos)
#define ICU_IRQCR_IRQMD_FALLING     (1UL << ICU_IRQCR_IRQMD_Pos)
#define ICU_IRQCR_IRQMD_RISING      (2UL << ICU_IRQCR_IRQMD_Pos)
#define ICU_IRQCR_IRQMD_BOTH        (3UL << ICU_IRQCR_IRQMD_Pos)
#define ICU_IRQCR_FCLKSEL_Pos       4U      //!< Filter clock [5:4]
#define ICU_IRQCR_FLTEN_Pos         7U      //!< Filter enable
#define ICU_IRQCR_FLTEN             (1UL << ICU_IRQCR_FLTEN_Pos)

/* IELSR bit positions. */
#define ICU_IELSR_IELS_Pos          0U      //!< Event source select [8:0]
#define ICU_IELSR_IELS_Msk          (0x1FFUL << ICU_IELSR_IELS_Pos)
#define ICU_IELSR_IR_Pos            16U     //!< Interrupt status flag
#define ICU_IELSR_IR                (1UL << ICU_IELSR_IR_Pos)
#define ICU_IELSR_DTCE_Pos          24U     //!< DTC enable on this event
#define ICU_IELSR_DTCE              (1UL << ICU_IELSR_DTCE_Pos)

/*===========================================================================
 *                  ELC Event Source IDs (write to IELSR.IELS)
 *
 * Selected from RA2E1 BSP (mcu/ra2e1/bsp_elc.h).  R9A02G021 implements
 * the subset of these events corresponding to its populated peripherals.
 *==========================================================================*/
typedef enum {
    /* Sentinel. */
    R9A02_EVENT_NONE                    = 0x000,    //!< Link disabled

    /* External pin interrupts. */
    R9A02_EVENT_ICU_IRQ0                = 0x001,
    R9A02_EVENT_ICU_IRQ1                = 0x002,
    R9A02_EVENT_ICU_IRQ2                = 0x003,
    R9A02_EVENT_ICU_IRQ3                = 0x004,
    R9A02_EVENT_ICU_IRQ4                = 0x005,
    R9A02_EVENT_ICU_IRQ5                = 0x006,
    R9A02_EVENT_ICU_IRQ6                = 0x007,
    R9A02_EVENT_ICU_IRQ7                = 0x008,

    /* DTC. */
    R9A02_EVENT_DTC_COMPLETE            = 0x009,
    R9A02_EVENT_DTC_END                 = 0x00A,

    /* System. */
    R9A02_EVENT_ICU_SNOOZE_CANCEL       = 0x00B,
    R9A02_EVENT_FCU_FRDYI               = 0x00C,    //!< Flash ready
    R9A02_EVENT_LVD_LVD1                = 0x00D,
    R9A02_EVENT_LVD_LVD2                = 0x00E,
    R9A02_EVENT_CGC_MOSC_STOP           = 0x00F,
    R9A02_EVENT_LPM_SNOOZE_REQUEST      = 0x010,

    /* AGT0 / AGT1. */
    R9A02_EVENT_AGT0_INT                = 0x011,
    R9A02_EVENT_AGT0_COMPARE_A          = 0x012,
    R9A02_EVENT_AGT0_COMPARE_B          = 0x013,
    R9A02_EVENT_AGT1_INT                = 0x014,
    R9A02_EVENT_AGT1_COMPARE_A          = 0x015,
    R9A02_EVENT_AGT1_COMPARE_B          = 0x016,

    /* Watchdogs / RTC. */
    R9A02_EVENT_IWDT_UNDERFLOW          = 0x017,
    R9A02_EVENT_WDT_UNDERFLOW           = 0x018,
    R9A02_EVENT_RTC_ALARM               = 0x019,
    R9A02_EVENT_RTC_PERIOD              = 0x01A,
    R9A02_EVENT_RTC_CARRY               = 0x01B,

    /* ADC0. */
    R9A02_EVENT_ADC0_SCAN_END           = 0x01C,
    R9A02_EVENT_ADC0_SCAN_END_B         = 0x01D,
    R9A02_EVENT_ADC0_WINDOW_A           = 0x01E,
    R9A02_EVENT_ADC0_WINDOW_B           = 0x01F,
    R9A02_EVENT_ADC0_COMPARE_MATCH      = 0x020,
    R9A02_EVENT_ADC0_COMPARE_MISMATCH   = 0x021,

    /* ACMPLP. */
    R9A02_EVENT_ACMPLP0_INT             = 0x023,
    R9A02_EVENT_ACMPLP1_INT             = 0x024,

    /* IIC0. */
    R9A02_EVENT_IIC0_RXI                = 0x027,
    R9A02_EVENT_IIC0_TXI                = 0x028,
    R9A02_EVENT_IIC0_TEI                = 0x029,
    R9A02_EVENT_IIC0_ERI                = 0x02A,
    R9A02_EVENT_IIC0_WUI                = 0x02B,

    /* DOC / CAC. */
    R9A02_EVENT_DOC_INT                 = 0x034,
    R9A02_EVENT_CAC_FREQUENCY_ERROR     = 0x035,
    R9A02_EVENT_CAC_MEASUREMENT_END     = 0x036,
    R9A02_EVENT_CAC_OVERFLOW            = 0x037,

    /* GPT0 / GPT1 (32-bit) and GPT4-GPT9 (16-bit) -- subset shown for
     * GPT0; the other timers follow 6 events each at consecutive IDs. */
    R9A02_EVENT_GPT0_CAPTURE_COMPARE_A  = 0x046,
    R9A02_EVENT_GPT0_CAPTURE_COMPARE_B  = 0x047,
    R9A02_EVENT_GPT0_COMPARE_C          = 0x048,
    R9A02_EVENT_GPT0_COMPARE_D          = 0x049,
    R9A02_EVENT_GPT0_COUNTER_OVERFLOW   = 0x04A,
    R9A02_EVENT_GPT0_COUNTER_UNDERFLOW  = 0x04B,

    /* SCI0..SCI2, SCI9 (R9A02G021 populated SCIs). */
    R9A02_EVENT_SCI0_RXI                = 0x071,
    R9A02_EVENT_SCI0_TXI                = 0x072,
    R9A02_EVENT_SCI0_TEI                = 0x073,
    R9A02_EVENT_SCI0_ERI                = 0x074,
    R9A02_EVENT_SCI0_AM                 = 0x075,
    R9A02_EVENT_SCI0_RXI_OR_ERI         = 0x076,
    R9A02_EVENT_SCI1_RXI                = 0x077,
    R9A02_EVENT_SCI1_TXI                = 0x078,
    R9A02_EVENT_SCI1_TEI                = 0x079,
    R9A02_EVENT_SCI1_ERI                = 0x07A,
    R9A02_EVENT_SCI1_AM                 = 0x07B,
    R9A02_EVENT_SCI9_RXI                = 0x07C,
    R9A02_EVENT_SCI9_TXI                = 0x07D,
    R9A02_EVENT_SCI9_TEI                = 0x07E,
    R9A02_EVENT_SCI9_ERI                = 0x07F,
    R9A02_EVENT_SCI9_AM                 = 0x080,
    R9A02_EVENT_SCI2_RXI                = 0x08E,
    R9A02_EVENT_SCI2_TXI                = 0x08F,
    R9A02_EVENT_SCI2_TEI                = 0x090,
    R9A02_EVENT_SCI2_ERI                = 0x091,
    R9A02_EVENT_SCI2_AM                 = 0x092,

    /* SPI0. */
    R9A02_EVENT_SPI0_RXI                = 0x081,
    R9A02_EVENT_SPI0_TXI                = 0x082,
    R9A02_EVENT_SPI0_IDLE               = 0x083,
    R9A02_EVENT_SPI0_ERI                = 0x084,
    R9A02_EVENT_SPI0_TEI                = 0x085,

    /* Crypto. */
    R9A02_EVENT_AES_WRREQ               = 0x08B,
    R9A02_EVENT_AES_RDREQ               = 0x08C,
    R9A02_EVENT_TRNG_RDREQ              = 0x08D
} R9A02_Event_t;

/*===========================================================================
 *                      SCI - Serial Communications Interface
 *
 * Standard RA SCI register layout used for UART/SPI/SmartCard modes.
 * Only the UART-mode subset is needed for the IOsonata UART driver;
 * see the RA2 manual section 25 for the full register description.
 *==========================================================================*/
typedef struct {
    __IO uint8_t        SMR;                //!< 0x00 Serial mode
    __IO uint8_t        BRR;                //!< 0x01 Baud rate
    __IO uint8_t        SCR;                //!< 0x02 Serial control
    __IO uint8_t        TDR;                //!< 0x03 Transmit data
    __IO uint8_t        SSR;                //!< 0x04 Serial status
    __I  uint8_t        RDR;                //!< 0x05 Receive data
    __IO uint8_t        SCMR;               //!< 0x06 Smart card mode
    __IO uint8_t        SEMR;               //!< 0x07 Serial extended mode
    __IO uint8_t        SNFR;               //!< 0x08 Noise filter
    __IO uint8_t        SIMR1;              //!< 0x09 I2C mode 1
    __IO uint8_t        SIMR2;              //!< 0x0A I2C mode 2
    __IO uint8_t        SIMR3;              //!< 0x0B I2C mode 3
    __IO uint8_t        SISR;               //!< 0x0C I2C status
    __IO uint8_t        SPMR;               //!< 0x0D SPI mode
    __IO uint16_t       TDRHL;              //!< 0x0E TX FIFO low byte
    union {
        __I  uint16_t   RDRHL;              //!< 0x10 RX FIFO low byte
        struct {
            __IO uint8_t MDDR;              //!< 0x12 Modulation duty
        };
    };
} R9A02_SCI_Type;

#define R9A02_SCI0                  ((R9A02_SCI_Type *) R9A02_SCI0_BASE)
#define R9A02_SCI1                  ((R9A02_SCI_Type *) R9A02_SCI1_BASE)
#define R9A02_SCI2                  ((R9A02_SCI_Type *) R9A02_SCI2_BASE)
#define R9A02_SCI9                  ((R9A02_SCI_Type *) R9A02_SCI9_BASE)

/* SCR (Serial Control Register) bit positions. */
#define SCI_SCR_CKE_Pos             0U      //!< Clock enable [1:0]
#define SCI_SCR_CKE_Msk             (3UL << SCI_SCR_CKE_Pos)
#define SCI_SCR_TEIE_Pos            2U      //!< Transmit-end interrupt enable
#define SCI_SCR_TEIE                (1UL << SCI_SCR_TEIE_Pos)
#define SCI_SCR_MPIE_Pos            3U      //!< Multi-processor interrupt enable
#define SCI_SCR_REIE_Pos            3U      //!< Receive-error interrupt enable
#define SCI_SCR_RE_Pos              4U      //!< Receive enable
#define SCI_SCR_RE                  (1UL << SCI_SCR_RE_Pos)
#define SCI_SCR_TE_Pos              5U      //!< Transmit enable
#define SCI_SCR_TE                  (1UL << SCI_SCR_TE_Pos)
#define SCI_SCR_RIE_Pos             6U      //!< Receive-interrupt enable
#define SCI_SCR_RIE                 (1UL << SCI_SCR_RIE_Pos)
#define SCI_SCR_TIE_Pos             7U      //!< Transmit-interrupt enable
#define SCI_SCR_TIE                 (1UL << SCI_SCR_TIE_Pos)

/* SSR (Serial Status Register) bit positions. */
#define SCI_SSR_TDRE_Pos            7U      //!< Transmit-data register empty
#define SCI_SSR_TDRE                (1UL << SCI_SSR_TDRE_Pos)
#define SCI_SSR_RDRF_Pos            6U      //!< Receive-data register full
#define SCI_SSR_RDRF                (1UL << SCI_SSR_RDRF_Pos)
#define SCI_SSR_ORER_Pos            5U      //!< Overrun error
#define SCI_SSR_ORER                (1UL << SCI_SSR_ORER_Pos)
#define SCI_SSR_FER_Pos             4U      //!< Framing error
#define SCI_SSR_FER                 (1UL << SCI_SSR_FER_Pos)
#define SCI_SSR_PER_Pos             3U      //!< Parity error
#define SCI_SSR_PER                 (1UL << SCI_SSR_PER_Pos)
#define SCI_SSR_TEND_Pos            2U      //!< Transmit end
#define SCI_SSR_TEND                (1UL << SCI_SSR_TEND_Pos)

/*===========================================================================
 *                  Peripheral instance counts (R9A02G021 specific)
 *
 * Some entries are MAXIMA across the R9A02G021 family; smaller package
 * variants populate fewer pins / less SRAM / etc.  See part-number
 * datasheet for variant-specific values.
 *==========================================================================*/
#define R9A02_CPU_CORE_COUNT        1U
#define R9A02_CPU_INTERRUPT_COUNT   32U     //!< CLIC vector slots routed via IELSR
#define R9A02_PORT_COUNT            9U      //!< PORT0..PORT8 (max)
#define R9A02_GPIO_PIN_PER_PORT     16U
#define R9A02_GPIO_PIN_COUNT_MAX    (R9A02_PORT_COUNT * R9A02_GPIO_PIN_PER_PORT)

/* Driver-facing aliases used by iopincfg_r9a02.c and friends. */
#define R9A02_MAX_PORT              R9A02_PORT_COUNT
#define R9A02_PINS_PER_PORT         R9A02_GPIO_PIN_PER_PORT
#define R9A02_EXTERNAL_IRQ_COUNT    8U      //!< IRQ0..IRQ7
#define R9A02_SCI_COUNT             4U      //!< SCI0/1/2/9 populated
#define R9A02_SPI_COUNT             2U      //!< SPI0/1
#define R9A02_IIC_COUNT             2U      //!< IIC0/1 (some variants only have IIC0)
#define R9A02_AGT_COUNT             2U      //!< AGT0/1
#define R9A02_GPT_COUNT             4U      //!< GPT0..GPT3 (32-bit) on R9A02G021
#define R9A02_WDT_COUNT             1U
#define R9A02_IWDT_COUNT            1U
#define R9A02_RTC_COUNT             1U
#define R9A02_ADC_COUNT             1U      //!< 12-bit SAR
#define R9A02_CRC_COUNT             1U
#define R9A02_DOC_COUNT             1U
#define R9A02_DTC_COUNT             1U
#define R9A02_AES_COUNT             1U
#define R9A02_TRNG_COUNT            1U
#define R9A02_DAC_COUNT             0U      //!< No DAC on R9A02G021
#define R9A02_USB_COUNT             0U      //!< No USB on R9A02G021
#define R9A02_CAN_COUNT             0U      //!< No CAN on R9A02G021
#define R9A02_ETHERNET_COUNT        0U      //!< No EMAC on R9A02G021

#ifdef __cplusplus
}
#endif

#endif // __R9A02G021_H__
