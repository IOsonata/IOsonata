/**-------------------------------------------------------------------------
@file	esp32xx_gpio.h

@brief	ESP32 GPIO/IO_MUX definitions for the IOsonata RISC-V family.

Covers C3/C5/C6.  Provides three things:
  1. Raw register macros (ESP32_GPIO_*_REG32, ESP32_IOMUX_PAD_REG) that
     work uniformly across all supported chips.  Use these in driver
     code that must compile for any chip.
  2. Bitfield Pos/Msk definitions for the GPIO matrix and IO_MUX
     register fields.  C5 widened the matrix selector fields and
     therefore has different bit positions than C3/C6.
  3. A typed register overlay (ESP32_GPIO_Type, ESP32_IOMUX_Type) for
     the C3/C6 layout.  C5 has substantially different register
     offsets and is NOT supported through these structs -- C5 code
     must use the raw *_REG32 macros instead.


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
#ifndef __ESP32XX_GPIO_H__
#define __ESP32XX_GPIO_H__

#include <stdint.h>
#include <stddef.h>

#include "esp32xx.h"

/*---------------------------------------------------------------------------
 * Cross-chip fallback macros.
 *
 * The chip-specific header normally supplies these.  These fallbacks are
 * defensive in case the chip header is bypassed.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_GPIO_BASE
#  if   defined(ESP32C3)
#    define ESP32_GPIO_BASE                 ESP32C3_GPIO_BASE
#  elif defined(ESP32C5)
#    define ESP32_GPIO_BASE                 ESP32C5_GPIO_BASE
#  elif defined(ESP32C6)
#    define ESP32_GPIO_BASE                 ESP32C6_GPIO_BASE
#  endif
#endif

#ifndef ESP32_IOMUX_BASE
#  if   defined(ESP32C3)
#    define ESP32_IOMUX_BASE                ESP32C3_IOMUX_BASE
#  elif defined(ESP32C5)
#    define ESP32_IOMUX_BASE                ESP32C5_IOMUX_BASE
#  elif defined(ESP32C6)
#    define ESP32_IOMUX_BASE                ESP32C6_IOMUX_BASE
#  endif
#endif

#ifndef ESP32_GPIO_PIN_COUNT
#  if   defined(ESP32C3)
#    define ESP32_GPIO_PIN_COUNT            ESP32C3_GPIO_PIN_COUNT
#  elif defined(ESP32C5)
#    define ESP32_GPIO_PIN_COUNT            ESP32C5_GPIO_PIN_COUNT
#  elif defined(ESP32C6)
#    define ESP32_GPIO_PIN_COUNT            ESP32C6_GPIO_PIN_COUNT
#  endif
#endif

#ifndef ESP32_GPIO_PIN_REG_COUNT
#  if   defined(ESP32C3)
#    define ESP32_GPIO_PIN_REG_COUNT        ESP32C3_GPIO_PIN_REG_COUNT
#  elif defined(ESP32C5)
#    define ESP32_GPIO_PIN_REG_COUNT        ESP32C5_GPIO_PIN_REG_COUNT
#  elif defined(ESP32C6)
#    define ESP32_GPIO_PIN_REG_COUNT        ESP32C6_GPIO_PIN_REG_COUNT
#  endif
#endif

#ifndef ESP32_GPIO_FUNC_IN_COUNT
#  if   defined(ESP32C3)
#    define ESP32_GPIO_FUNC_IN_COUNT        ESP32C3_GPIO_FUNC_IN_COUNT
#  elif defined(ESP32C5)
#    define ESP32_GPIO_FUNC_IN_COUNT        ESP32C5_GPIO_FUNC_IN_COUNT
#  elif defined(ESP32C6)
#    define ESP32_GPIO_FUNC_IN_COUNT        ESP32C6_GPIO_FUNC_IN_COUNT
#  endif
#endif

#ifndef ESP32_GPIO_FUNC_OUT_COUNT
#  if   defined(ESP32C3)
#    define ESP32_GPIO_FUNC_OUT_COUNT       ESP32C3_GPIO_FUNC_OUT_COUNT
#  elif defined(ESP32C5)
#    define ESP32_GPIO_FUNC_OUT_COUNT       ESP32C5_GPIO_FUNC_OUT_COUNT
#  elif defined(ESP32C6)
#    define ESP32_GPIO_FUNC_OUT_COUNT       ESP32C6_GPIO_FUNC_OUT_COUNT
#  endif
#endif

#ifndef ESP32_GPIO_FUNC_OUT_REG_COUNT
#  if   defined(ESP32C3)
#    define ESP32_GPIO_FUNC_OUT_REG_COUNT   ESP32C3_GPIO_FUNC_OUT_REG_COUNT
#  elif defined(ESP32C5)
#    define ESP32_GPIO_FUNC_OUT_REG_COUNT   ESP32C5_GPIO_FUNC_OUT_REG_COUNT
#  elif defined(ESP32C6)
#    define ESP32_GPIO_FUNC_OUT_REG_COUNT   ESP32C6_GPIO_FUNC_OUT_REG_COUNT
#  endif
#endif

/*---------------------------------------------------------------------------
 * GPIO matrix magic values.
 *
 * - SEL_GPIO is the FUNC_OUT_SEL value that drives a pad directly from
 *   GPIO_OUT/GPIO_ENABLE instead of routing a peripheral output through
 *   the matrix.  It equals the number of peripheral output signals.
 *
 * - CONST_HIGH / CONST_LOW are FUNC_IN_SEL values that feed a constant
 *   logic level into a peripheral input instead of a pad.
 *
 * Values verified against ESP-IDF v5.3 components/soc/<chip>/include
 * (gpio_pins.h for C3/C6, gpio_struct.h FUNC_IN_SEL field doc for C5).
 *---------------------------------------------------------------------------*/
#if defined(ESP32C5)
#  define ESP32_GPIO_FUNC_OUT_SEL_GPIO      256U
#  define ESP32_GPIO_FUNC_IN_CONST_HIGH     0x40U
#  define ESP32_GPIO_FUNC_IN_CONST_LOW      0x60U
#else
#  define ESP32_GPIO_FUNC_OUT_SEL_GPIO      128U
#  define ESP32_GPIO_FUNC_IN_CONST_HIGH     0x1EU
#  define ESP32_GPIO_FUNC_IN_CONST_LOW      0x1FU
#endif

/*---------------------------------------------------------------------------
 * GPIO_PINn.INT_TYPE selector values (common to C3/C5/C6).
 *---------------------------------------------------------------------------*/
#define ESP32_GPIO_INT_DISABLE              0U
#define ESP32_GPIO_INT_RISING_EDGE          1U
#define ESP32_GPIO_INT_FALLING_EDGE         2U
#define ESP32_GPIO_INT_ANY_EDGE             3U
#define ESP32_GPIO_INT_LOW_LEVEL            4U
#define ESP32_GPIO_INT_HIGH_LEVEL           5U

/*---------------------------------------------------------------------------
 * GPIO_PINn.INT_ENA bit positions inside the 5-bit field.
 *---------------------------------------------------------------------------*/
#define ESP32_GPIO_PIN_INT_ENA_CPU          (1U << 0)   //!< Route to CPU level interrupt.
#define ESP32_GPIO_PIN_INT_ENA_NMI          (1U << 1)   //!< Route to CPU NMI line.

/*---------------------------------------------------------------------------
 * GPIO_PINn bit positions and masks (common to C3/C5/C6).
 *---------------------------------------------------------------------------*/
#define ESP32_GPIO_PIN_SYNC2_BYPASS_Pos     0U
#define ESP32_GPIO_PIN_SYNC2_BYPASS_Msk     (3UL << ESP32_GPIO_PIN_SYNC2_BYPASS_Pos)
#define ESP32_GPIO_PIN_PAD_DRIVER_Pos       2U
#define ESP32_GPIO_PIN_PAD_DRIVER_Msk       (1UL << ESP32_GPIO_PIN_PAD_DRIVER_Pos)
#define ESP32_GPIO_PIN_SYNC1_BYPASS_Pos     3U
#define ESP32_GPIO_PIN_SYNC1_BYPASS_Msk     (3UL << ESP32_GPIO_PIN_SYNC1_BYPASS_Pos)
#define ESP32_GPIO_PIN_INT_TYPE_Pos         7U
#define ESP32_GPIO_PIN_INT_TYPE_Msk         (7UL << ESP32_GPIO_PIN_INT_TYPE_Pos)
#define ESP32_GPIO_PIN_WAKEUP_ENABLE_Pos    10U
#define ESP32_GPIO_PIN_WAKEUP_ENABLE_Msk    (1UL << ESP32_GPIO_PIN_WAKEUP_ENABLE_Pos)
#define ESP32_GPIO_PIN_CONFIG_Pos           11U
#define ESP32_GPIO_PIN_CONFIG_Msk           (3UL << ESP32_GPIO_PIN_CONFIG_Pos)
#define ESP32_GPIO_PIN_INT_ENA_Pos          13U
#define ESP32_GPIO_PIN_INT_ENA_Msk          (0x1FUL << ESP32_GPIO_PIN_INT_ENA_Pos)
#define ESP32_GPIO_PIN_INT_ENA_CPU_Msk      (1UL << ESP32_GPIO_PIN_INT_ENA_Pos)
#define ESP32_GPIO_PIN_INT_ENA_NMI_Msk      (1UL << (ESP32_GPIO_PIN_INT_ENA_Pos + 1U))

/*---------------------------------------------------------------------------
 * GPIO_FUNCn_IN_SEL_CFG and GPIO_FUNCn_OUT_SEL_CFG bit positions and masks.
 *
 * C5 widened both selector fields:
 *   IN_SEL  : 5 bits on C3/C6, 7 bits on C5  (29..127 GPIO matrix sources)
 *   OUT_SEL : 8 bits on C3/C6, 9 bits on C5  (256..511 peripheral signals)
 *
 * Single chip-conditional block; no #undef dance.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C5)
#  define ESP32_GPIO_FUNC_IN_SEL_Pos        0U
#  define ESP32_GPIO_FUNC_IN_SEL_Msk        (0x7FUL << ESP32_GPIO_FUNC_IN_SEL_Pos)
#  define ESP32_GPIO_FUNC_IN_INV_SEL_Pos    7U
#  define ESP32_GPIO_FUNC_IN_INV_SEL_Msk    (1UL << ESP32_GPIO_FUNC_IN_INV_SEL_Pos)
#  define ESP32_GPIO_SIG_IN_SEL_Pos         8U
#  define ESP32_GPIO_SIG_IN_SEL_Msk         (1UL << ESP32_GPIO_SIG_IN_SEL_Pos)

#  define ESP32_GPIO_FUNC_OUT_SEL_Pos       0U
#  define ESP32_GPIO_FUNC_OUT_SEL_Msk       (0x1FFUL << ESP32_GPIO_FUNC_OUT_SEL_Pos)
#  define ESP32_GPIO_FUNC_OUT_INV_SEL_Pos   9U
#  define ESP32_GPIO_FUNC_OUT_INV_SEL_Msk   (1UL << ESP32_GPIO_FUNC_OUT_INV_SEL_Pos)
#  define ESP32_GPIO_FUNC_OEN_SEL_Pos       10U
#  define ESP32_GPIO_FUNC_OEN_SEL_Msk       (1UL << ESP32_GPIO_FUNC_OEN_SEL_Pos)
#  define ESP32_GPIO_FUNC_OEN_INV_SEL_Pos   11U
#  define ESP32_GPIO_FUNC_OEN_INV_SEL_Msk   (1UL << ESP32_GPIO_FUNC_OEN_INV_SEL_Pos)
#else
#  define ESP32_GPIO_FUNC_IN_SEL_Pos        0U
#  define ESP32_GPIO_FUNC_IN_SEL_Msk        (0x1FUL << ESP32_GPIO_FUNC_IN_SEL_Pos)
#  define ESP32_GPIO_FUNC_IN_INV_SEL_Pos    5U
#  define ESP32_GPIO_FUNC_IN_INV_SEL_Msk    (1UL << ESP32_GPIO_FUNC_IN_INV_SEL_Pos)
#  define ESP32_GPIO_SIG_IN_SEL_Pos         6U
#  define ESP32_GPIO_SIG_IN_SEL_Msk         (1UL << ESP32_GPIO_SIG_IN_SEL_Pos)

#  define ESP32_GPIO_FUNC_OUT_SEL_Pos       0U
#  define ESP32_GPIO_FUNC_OUT_SEL_Msk       (0xFFUL << ESP32_GPIO_FUNC_OUT_SEL_Pos)
#  define ESP32_GPIO_FUNC_OUT_INV_SEL_Pos   8U
#  define ESP32_GPIO_FUNC_OUT_INV_SEL_Msk   (1UL << ESP32_GPIO_FUNC_OUT_INV_SEL_Pos)
#  define ESP32_GPIO_FUNC_OEN_SEL_Pos       9U
#  define ESP32_GPIO_FUNC_OEN_SEL_Msk       (1UL << ESP32_GPIO_FUNC_OEN_SEL_Pos)
#  define ESP32_GPIO_FUNC_OEN_INV_SEL_Pos   10U
#  define ESP32_GPIO_FUNC_OEN_INV_SEL_Msk   (1UL << ESP32_GPIO_FUNC_OEN_INV_SEL_Pos)
#endif

/*---------------------------------------------------------------------------
 * IO_MUX_GPIOn bit positions and masks (common to C3/C5/C6).
 *---------------------------------------------------------------------------*/
#define ESP32_IOMUX_MCU_OE_Pos              0U
#define ESP32_IOMUX_MCU_OE_Msk              (1UL << ESP32_IOMUX_MCU_OE_Pos)
#define ESP32_IOMUX_SLP_SEL_Pos             1U
#define ESP32_IOMUX_SLP_SEL_Msk             (1UL << ESP32_IOMUX_SLP_SEL_Pos)
#define ESP32_IOMUX_MCU_WPD_Pos             2U
#define ESP32_IOMUX_MCU_WPD_Msk             (1UL << ESP32_IOMUX_MCU_WPD_Pos)
#define ESP32_IOMUX_MCU_WPU_Pos             3U
#define ESP32_IOMUX_MCU_WPU_Msk             (1UL << ESP32_IOMUX_MCU_WPU_Pos)
#define ESP32_IOMUX_MCU_IE_Pos              4U
#define ESP32_IOMUX_MCU_IE_Msk              (1UL << ESP32_IOMUX_MCU_IE_Pos)
#define ESP32_IOMUX_MCU_DRV_Pos             5U
#define ESP32_IOMUX_MCU_DRV_Msk             (3UL << ESP32_IOMUX_MCU_DRV_Pos)
#define ESP32_IOMUX_FUN_WPD_Pos             7U
#define ESP32_IOMUX_FUN_WPD_Msk             (1UL << ESP32_IOMUX_FUN_WPD_Pos)
#define ESP32_IOMUX_FUN_WPU_Pos             8U
#define ESP32_IOMUX_FUN_WPU_Msk             (1UL << ESP32_IOMUX_FUN_WPU_Pos)
#define ESP32_IOMUX_FUN_IE_Pos              9U
#define ESP32_IOMUX_FUN_IE_Msk              (1UL << ESP32_IOMUX_FUN_IE_Pos)
#define ESP32_IOMUX_FUN_DRV_Pos             10U
#define ESP32_IOMUX_FUN_DRV_Msk             (3UL << ESP32_IOMUX_FUN_DRV_Pos)
#define ESP32_IOMUX_MCU_SEL_Pos             12U
#define ESP32_IOMUX_MCU_SEL_Msk             (7UL << ESP32_IOMUX_MCU_SEL_Pos)
#define ESP32_IOMUX_FILTER_EN_Pos           15U
#define ESP32_IOMUX_FILTER_EN_Msk           (1UL << ESP32_IOMUX_FILTER_EN_Pos)

// MCU_SEL value selecting "GPIO" function (i.e. routing through GPIO matrix).
#define ESP32_IOMUX_MCU_SEL_GPIO            (1UL << ESP32_IOMUX_MCU_SEL_Pos)

/*===========================================================================
 * Typed register overlay (C3/C6 only).
 *
 * C5 has substantially different register offsets and is intentionally
 * omitted -- C5 code must use the raw *_REG32 macros further down in
 * this file.  The static_asserts below verify that the C3/C6 layout
 * lines up with the hardware register map.
 *
 * Bitfield structures use named-union access for both whole-register
 * and per-field views.  The bitfield layouts are valid for C3 and C6;
 * the GPIO_PIN bitfield is also valid for C5 but C5 has different
 * FUNC_IN/OUT bitfield widths, which is one more reason this overlay
 * is gated to !ESP32C5.
 *===========================================================================*/
#if !defined(ESP32C5)

typedef struct __ESP32_GPIO_BT_SELECT_Bits {
    uint32_t BT_SEL              : 32;   //!< Bit-select register (chip-specific use).
} ESP32_GPIO_BT_SELECT_Bits;

typedef struct __ESP32_GPIO_BITMAP_Bits {
    uint32_t DATA                : ESP32_GPIO_PIN_COUNT;          //!< One bit per usable pad.
    uint32_t RESERVED            : (32U - ESP32_GPIO_PIN_COUNT);
} ESP32_GPIO_BITMAP_Bits;

typedef struct __ESP32_GPIO_STRAP_Bits {
    uint32_t STRAPPING           : 16;   //!< Strap-pin sample (chip-specific bit assignment).
    uint32_t RESERVED            : 16;
} ESP32_GPIO_STRAP_Bits;

typedef struct __ESP32_GPIO_PIN_Bits {
    uint32_t SYNC2_BYPASS        : 2;    //!< Second-stage input synchronizer control.
    uint32_t PAD_DRIVER          : 1;    //!< 0: normal output, 1: open-drain output.
    uint32_t SYNC1_BYPASS        : 2;    //!< First-stage input synchronizer control.
    uint32_t RESERVED0           : 2;
    uint32_t INT_TYPE            : 3;    //!< ESP32_GPIO_INT_*.
    uint32_t WAKEUP_ENABLE       : 1;    //!< Wake CPU from light sleep on this GPIO.
    uint32_t CONFIG              : 2;    //!< Reserved R/W field.
    uint32_t INT_ENA             : 5;    //!< INT_ENA field; use ESP32_GPIO_PIN_INT_ENA_*.
    uint32_t RESERVED1           : 14;
} ESP32_GPIO_PIN_Bits;

typedef struct __ESP32_GPIO_FUNC_IN_Bits {
    uint32_t IN_SEL              : 5;    //!< Pad index, or CONST_HIGH/CONST_LOW.
    uint32_t IN_INV_SEL          : 1;    //!< 1: invert input.
    uint32_t SIG_IN_SEL          : 1;    //!< 1: route via matrix; 0: direct from IO_MUX.
    uint32_t RESERVED            : 25;
} ESP32_GPIO_FUNC_IN_Bits;

typedef struct __ESP32_GPIO_FUNC_OUT_Bits {
    uint32_t OUT_SEL             : 8;    //!< Peripheral signal index, or 128 = direct GPIO drive.
    uint32_t OUT_INV_SEL         : 1;    //!< 1: invert output.
    uint32_t OEN_SEL             : 1;    //!< 1: OE from GPIO_ENABLE; 0: from peripheral.
    uint32_t OEN_INV_SEL         : 1;    //!< 1: invert output-enable.
    uint32_t RESERVED            : 21;
} ESP32_GPIO_FUNC_OUT_Bits;

typedef struct __ESP32_GPIO_CLK_EN_Bits {
    uint32_t CLK_EN              : 1;    //!< 1: GPIO register clock free-running.
    uint32_t RESERVED            : 31;
} ESP32_GPIO_CLK_EN_Bits;

typedef struct __ESP32_GPIO_DATE_Bits {
    uint32_t DATE                : 28;
    uint32_t RESERVED            : 4;
} ESP32_GPIO_DATE_Bits;

/*---------------------------------------------------------------------------
 * GPIO matrix register block (C3/C6 layout).
 *
 * Offsets verified against ESP-IDF v5.3
 * components/soc/esp32c3/include/soc/gpio_reg.h and
 * components/soc/esp32c6/include/soc/gpio_reg.h.
 *
 * All field offsets are fixed; only the per-pad PIN[] and
 * FUNC_OUT_CFG[] array lengths grow with the chip's register-bank
 * count (PIN_REG_COUNT / FUNC_OUT_REG_COUNT).  Reserved gaps are
 * computed so the next named register lands at its hardware offset
 * regardless of the array length.
 *---------------------------------------------------------------------------*/
typedef struct __ESP32_GPIO_Reg {
    union {
        volatile uint32_t BT_SELECT;        //!< 0x000  Bit-select register.
        volatile ESP32_GPIO_BT_SELECT_Bits BT_SELECT_Bit;
    };
    union {
        volatile uint32_t OUT;              //!< 0x004  Output value latch.
        volatile ESP32_GPIO_BITMAP_Bits OUT_Bit;
    };
    union {
        volatile uint32_t OUT_W1TS;         //!< 0x008  Output W1TS.
        volatile ESP32_GPIO_BITMAP_Bits OUT_W1TS_Bit;
    };
    union {
        volatile uint32_t OUT_W1TC;         //!< 0x00C  Output W1TC.
        volatile ESP32_GPIO_BITMAP_Bits OUT_W1TC_Bit;
    };
    uint32_t RESERVED0[4];  // 0x010..0x01C reserved (room for OUT1).
    union {
        volatile uint32_t ENABLE;           //!< 0x020  Output-driver enable.
        volatile ESP32_GPIO_BITMAP_Bits ENABLE_Bit;
    };
    union {
        volatile uint32_t ENABLE_W1TS;      //!< 0x024  Enable W1TS.
        volatile ESP32_GPIO_BITMAP_Bits ENABLE_W1TS_Bit;
    };
    union {
        volatile uint32_t ENABLE_W1TC;      //!< 0x028  Enable W1TC.
        volatile ESP32_GPIO_BITMAP_Bits ENABLE_W1TC_Bit;
    };
    uint32_t RESERVED1[3];  // 0x02C..0x034 reserved.
    union {
        volatile uint32_t STRAPPING;        //!< 0x038  Strap-pin sampled values.
        volatile ESP32_GPIO_STRAP_Bits STRAPPING_Bit;
    };
    union {
        volatile uint32_t IN;               //!< 0x03C  Input value.
        volatile ESP32_GPIO_BITMAP_Bits IN_Bit;
    };
    uint32_t RESERVED2;  // 0x040 reserved (room for IN1).
    union {
        volatile uint32_t ISTATUS;          //!< 0x044  Interrupt status (per-pin).
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_Bit;
    };
    union {
        volatile uint32_t ISTATUS_W1TS;     //!< 0x048  Status W1TS.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_W1TS_Bit;
    };
    union {
        volatile uint32_t ISTATUS_W1TC;     //!< 0x04C  Status W1TC.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_W1TC_Bit;
    };
    uint32_t RESERVED3[3];  // 0x050..0x058 reserved.
    union {
        volatile uint32_t PROCPU_INT;       //!< 0x05C  CPU interrupt status.
        volatile ESP32_GPIO_BITMAP_Bits PROCPU_INT_Bit;
    };
    union {
        volatile uint32_t PCPU_NMI_INT;     //!< 0x060  CPU NMI status.
        volatile ESP32_GPIO_BITMAP_Bits PCPU_NMI_INT_Bit;
    };
    union {
        volatile uint32_t CPUSDIO_INT;      //!< 0x064  SDIO host CPU int status.
        volatile ESP32_GPIO_BITMAP_Bits CPUSDIO_INT_Bit;
    };
    uint32_t RESERVED4[3];  // 0x068..0x070 reserved (PCPU_INT1 on some chips).
    union {
        volatile uint32_t PIN[ESP32_GPIO_PIN_REG_COUNT];
                                            //!< 0x074  Per-pad sync/wake/PAD_DRIVER cfg.
        volatile ESP32_GPIO_PIN_Bits PIN_Bit[ESP32_GPIO_PIN_REG_COUNT];
    };
    uint32_t RESERVED5[(0x014CU - (0x0074U + (ESP32_GPIO_PIN_REG_COUNT * 4U))) / 4U];

    union {
        volatile uint32_t ISTATUS_NEXT;     //!< 0x14C  Edge-detect helper.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_NEXT_Bit;
    };
    uint32_t RESERVED6;  // 0x150 reserved.
    union {
        volatile uint32_t FUNC_IN_CFG[ESP32_GPIO_FUNC_IN_COUNT];
                                            //!< 0x154  Peripheral-input routing.
        volatile ESP32_GPIO_FUNC_IN_Bits FUNC_IN_CFG_Bit[ESP32_GPIO_FUNC_IN_COUNT];
    };
    uint32_t RESERVED7[(0x0554U - (0x0154U + (ESP32_GPIO_FUNC_IN_COUNT * 4U))) / 4U];

    union {
        volatile uint32_t FUNC_OUT_CFG[ESP32_GPIO_FUNC_OUT_REG_COUNT];
                                            //!< 0x554  Per-pad output routing.
        volatile ESP32_GPIO_FUNC_OUT_Bits FUNC_OUT_CFG_Bit[ESP32_GPIO_FUNC_OUT_REG_COUNT];
    };
    uint32_t RESERVED8[(0x062CU - (0x0554U + (ESP32_GPIO_FUNC_OUT_REG_COUNT * 4U))) / 4U];

    union {
        volatile uint32_t CLK_EN;           //!< 0x62C  Clock-gate.
        volatile ESP32_GPIO_CLK_EN_Bits CLK_EN_Bit;
    };
    uint32_t RESERVED9[(0x06FCU - 0x0630U) / 4U];

    union {
        volatile uint32_t DATE;             //!< 0x6FC  Version/date register.
        volatile ESP32_GPIO_DATE_Bits DATE_Bit;
    };
} ESP32_GPIO_Type;

/*---------------------------------------------------------------------------
 * IO_MUX register block (C3/C6 layout).
 *
 *   0x000          PIN_CTRL    Clock-output selection.
 *   0x004 + 4*n    GPIO[n]     IO_MUX_GPIOn pad configuration.
 *   0x0FC          DATE        IO_MUX version register.
 *
 * C5 IO_MUX has a different layout (no PIN_CTRL, GPIO[N] starts at
 * 0x000, DATE at 0x1FC) and is excluded from this overlay; C5 code
 * uses ESP32_IOMUX_PAD_REG(n) below.
 *---------------------------------------------------------------------------*/
typedef struct __ESP32_IOMUX_Reg {
    volatile uint32_t PIN_CTRL;             //!< 0x000  Clock-output configuration.
    volatile uint32_t GPIO[ESP32_GPIO_PIN_REG_COUNT];
                                            //!< 0x004  IO_MUX_GPIOn pad config.
    uint32_t RESERVED0[(0x00FCU - (0x0004U + (ESP32_GPIO_PIN_REG_COUNT * 4U))) / 4U];
    volatile uint32_t DATE;                 //!< 0x0FC  IO_MUX version register.
} ESP32_IOMUX_Type;

#define ESP32_GPIO                          ((ESP32_GPIO_Type  *)((uintptr_t)ESP32_GPIO_BASE))
#define ESP32_IOMUX                         ((ESP32_IOMUX_Type *)((uintptr_t)ESP32_IOMUX_BASE))

#define ESP32_STATIC_ASSERT(cond, name)     typedef char esp32_static_assert_##name[(cond) ? 1 : -1]

ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_BT_SELECT_Bits) == 4U, gpio_bt_select_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_BITMAP_Bits)    == 4U, gpio_bitmap_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_STRAP_Bits)     == 4U, gpio_strap_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_PIN_Bits)       == 4U, gpio_pin_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_FUNC_IN_Bits)   == 4U, gpio_func_in_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_FUNC_OUT_Bits)  == 4U, gpio_func_out_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_CLK_EN_Bits)    == 4U, gpio_clk_en_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_DATE_Bits)      == 4U, gpio_date_bits_size);

ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, OUT)            == 0x0004U, gpio_out_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, ENABLE)         == 0x0020U, gpio_enable_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, STRAPPING)      == 0x0038U, gpio_strapping_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, IN)             == 0x003CU, gpio_in_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, ISTATUS)        == 0x0044U, gpio_status_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, PROCPU_INT)     == 0x005CU, gpio_procpu_int_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, PCPU_NMI_INT)   == 0x0060U, gpio_pcpu_nmi_int_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, CPUSDIO_INT)    == 0x0064U, gpio_cpusdio_int_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, PIN)            == 0x0074U, gpio_pin_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, ISTATUS_NEXT)   == 0x014CU, gpio_status_next_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, FUNC_IN_CFG)    == 0x0154U, gpio_func_in_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, FUNC_OUT_CFG)   == 0x0554U, gpio_func_out_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, CLK_EN)         == 0x062CU, gpio_clk_en_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, DATE)           == 0x06FCU, gpio_date_offset);

ESP32_STATIC_ASSERT(offsetof(ESP32_IOMUX_Type, PIN_CTRL)      == 0x0000U, iomux_pin_ctrl_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_IOMUX_Type, GPIO)          == 0x0004U, iomux_gpio_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_IOMUX_Type, DATE)          == 0x00FCU, iomux_date_offset);

#endif // !defined(ESP32C5)

/*===========================================================================
 * Raw register-address macros (cross-chip).
 *
 * These work for C3, C5, and C6.  Driver code that must compile for
 * any chip in the family should use these instead of the typed
 * overlay above.
 *===========================================================================*/
#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

#define ESP32_GPIO_OUT_REG_OFFSET           0x004U
#define ESP32_GPIO_OUT_W1TS_OFFSET          0x008U
#define ESP32_GPIO_OUT_W1TC_OFFSET          0x00CU

#if defined(ESP32C5)
#  define ESP32_GPIO_ENABLE_REG_OFFSET      0x034U
#  define ESP32_GPIO_ENABLE_W1TS_OFFSET     0x038U
#  define ESP32_GPIO_ENABLE_W1TC_OFFSET     0x03CU
#  define ESP32_GPIO_IN_REG_OFFSET          0x064U
#  define ESP32_GPIO_STATUS_REG_OFFSET      0x074U
#  define ESP32_GPIO_STATUS_W1TS_OFFSET     0x078U
#  define ESP32_GPIO_STATUS_W1TC_OFFSET     0x07CU
#  define ESP32_GPIO_STATUS_NEXT_OFFSET     0x0C4U
#  define ESP32_GPIO_PIN0_REG_OFFSET        0x0D4U
#  define ESP32_GPIO_FUNC_IN0_REG_OFFSET    0x2D4U
#  define ESP32_GPIO_FUNC_OUT0_REG_OFFSET   0xAD4U
#  define ESP32_GPIO_CLOCK_GATE_OFFSET      0xDF8U
#  define ESP32_GPIO_DATE_OFFSET            0xDFCU
#  define ESP32_IOMUX_PAD0_REG_OFFSET       0x000U
#  define ESP32_IOMUX_DATE_OFFSET           0x1FCU
#else
#  define ESP32_GPIO_ENABLE_REG_OFFSET      0x020U
#  define ESP32_GPIO_ENABLE_W1TS_OFFSET     0x024U
#  define ESP32_GPIO_ENABLE_W1TC_OFFSET     0x028U
#  define ESP32_GPIO_IN_REG_OFFSET          0x03CU
#  define ESP32_GPIO_STATUS_REG_OFFSET      0x044U
#  define ESP32_GPIO_STATUS_W1TS_OFFSET     0x048U
#  define ESP32_GPIO_STATUS_W1TC_OFFSET     0x04CU
#  define ESP32_GPIO_STATUS_NEXT_OFFSET     0x14CU
#  define ESP32_GPIO_PIN0_REG_OFFSET        0x074U
#  define ESP32_GPIO_FUNC_IN0_REG_OFFSET    0x154U
#  define ESP32_GPIO_FUNC_OUT0_REG_OFFSET   0x554U
#  define ESP32_GPIO_CLOCK_GATE_OFFSET      0x62CU
#  define ESP32_GPIO_DATE_OFFSET            0x6FCU
#  define ESP32_IOMUX_PAD0_REG_OFFSET       0x004U
#  define ESP32_IOMUX_DATE_OFFSET           0x0FCU
#endif

#define ESP32_GPIO_OUT_REG32                ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_OUT_REG_OFFSET)
#define ESP32_GPIO_OUT_W1TS_REG32           ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_OUT_W1TS_OFFSET)
#define ESP32_GPIO_OUT_W1TC_REG32           ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_OUT_W1TC_OFFSET)
#define ESP32_GPIO_ENABLE_REG32             ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_ENABLE_REG_OFFSET)
#define ESP32_GPIO_ENABLE_W1TS_REG32        ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_ENABLE_W1TS_OFFSET)
#define ESP32_GPIO_ENABLE_W1TC_REG32        ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_ENABLE_W1TC_OFFSET)
#define ESP32_GPIO_IN_REG32                 ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_IN_REG_OFFSET)
#define ESP32_GPIO_STATUS_REG32             ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_STATUS_REG_OFFSET)
#define ESP32_GPIO_STATUS_W1TS_REG32        ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_STATUS_W1TS_OFFSET)
#define ESP32_GPIO_STATUS_W1TC_REG32        ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_STATUS_W1TC_OFFSET)
#define ESP32_GPIO_STATUS_NEXT_REG32        ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_STATUS_NEXT_OFFSET)
#define ESP32_GPIO_PIN_REG32(pin)           ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_PIN0_REG_OFFSET     + ((uint32_t)(pin) * 4U))
#define ESP32_GPIO_FUNC_IN_REG32(sig)       ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_FUNC_IN0_REG_OFFSET + ((uint32_t)(sig) * 4U))
#define ESP32_GPIO_FUNC_OUT_REG32(pin)      ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_FUNC_OUT0_REG_OFFSET + ((uint32_t)(pin) * 4U))
#define ESP32_GPIO_CLOCK_GATE_REG32         ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_CLOCK_GATE_OFFSET)
#define ESP32_GPIO_DATE_REG32               ESP32_REG32(ESP32_GPIO_BASE  + ESP32_GPIO_DATE_OFFSET)

#define ESP32_IOMUX_PAD_REG(pin)            ESP32_REG32(ESP32_IOMUX_BASE + ESP32_IOMUX_PAD0_REG_OFFSET + ((uint32_t)(pin) * 4U))
#define ESP32_IOMUX_DATE_REG32              ESP32_REG32(ESP32_IOMUX_BASE + ESP32_IOMUX_DATE_OFFSET)

/*---------------------------------------------------------------------------
 * Composite value: program FUNC_OUT_CFG[pin] for "drive directly from
 * GPIO_OUT/GPIO_ENABLE" (i.e. simple GPIO output, no peripheral signal).
 *---------------------------------------------------------------------------*/
#define ESP32_GPIO_FUNC_OUT_GPIO_VALUE                                              \
    (((uint32_t)ESP32_GPIO_FUNC_OUT_SEL_GPIO << ESP32_GPIO_FUNC_OUT_SEL_Pos)        \
     | ESP32_GPIO_FUNC_OEN_SEL_Msk)

#endif // __ESP32XX_GPIO_H__
