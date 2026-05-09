/**-------------------------------------------------------------------------
@file	esp32xx_gpio.h

@brief	ESP32 GPIO definition


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
 * Chip selection defaults.
 *
 * Keep the register-map structures common.  The chip header supplies the
 * actual base addresses; this block only normalizes the common names used
 * below when the chip header has not already done it.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_GPIO_BASE
#if defined(ESP32C3)
#define ESP32_GPIO_BASE             ESP32C3_GPIO_BASE
#elif defined(ESP32C6)
#define ESP32_GPIO_BASE             ESP32C6_GPIO_MATRIX_BASE
#endif
#endif

#ifndef ESP32_IOMUX_BASE
#if defined(ESP32C3)
#define ESP32_IOMUX_BASE            ESP32C3_IOMUX_BASE
#elif defined(ESP32C6)
#define ESP32_IOMUX_BASE            ESP32C6_IOMUX_BASE
#endif
#endif

#ifndef ESP32_GPIO_PIN_COUNT
#if defined(ESP32C3)
#define ESP32_GPIO_PIN_COUNT        22U
#elif defined(ESP32C6)
#define ESP32_GPIO_PIN_COUNT        31U
#endif
#endif

#ifndef ESP32_GPIO_FUNC_IN_COUNT
#define ESP32_GPIO_FUNC_IN_COUNT    128U
#endif

#ifndef ESP32_GPIO_FUNC_OUT_COUNT
#define ESP32_GPIO_FUNC_OUT_COUNT   ESP32_GPIO_PIN_COUNT
#endif

#ifndef ESP32_GPIO_FUNC_OUT_SEL_GPIO
#if defined(ESP32C5)
#define ESP32_GPIO_FUNC_OUT_SEL_GPIO    256U    //!< FUNC_OUT_SEL value for simple GPIO output on ESP32-C5.
#else
#define ESP32_GPIO_FUNC_OUT_SEL_GPIO    128U    //!< FUNC_OUT_SEL value for simple GPIO output on C3/C6-class GPIO matrix.
#endif
#endif

#ifndef ESP32_GPIO_FUNC_IN_CONST_HIGH
#define ESP32_GPIO_FUNC_IN_CONST_HIGH   0x1EU   //!< FUNC_IN_SEL value for constant high input.
#endif

#ifndef ESP32_GPIO_FUNC_IN_CONST_LOW
#define ESP32_GPIO_FUNC_IN_CONST_LOW    0x1FU   //!< FUNC_IN_SEL value for constant low input.
#endif

/* GPIO_PINn_INT_TYPE values. */
#define ESP32_GPIO_INT_DISABLE          0U
#define ESP32_GPIO_INT_RISING_EDGE      1U
#define ESP32_GPIO_INT_FALLING_EDGE     2U
#define ESP32_GPIO_INT_ANY_EDGE         3U
#define ESP32_GPIO_INT_LOW_LEVEL        4U
#define ESP32_GPIO_INT_HIGH_LEVEL       5U

/* GPIO_PINn_INT_ENA bit values inside the 5-bit INT_ENA field. */
#define ESP32_GPIO_PIN_INT_ENA_CPU      (1U << 0)   //!< Route this GPIO interrupt to the CPU interrupt line.
#define ESP32_GPIO_PIN_INT_ENA_NMI      (1U << 1)   //!< Route this GPIO interrupt to the CPU NMI line.

/*--------------------------------------------------------------------------
 * GPIO/GPIO-matrix bitfield definitions.
 *
 * The register map below keeps every original 32-bit register member name
 * for direct whole-register access.  Each register also has a *_Bit view for
 * field-level access.  Use the 32-bit register member for W1TS/W1TC writes so
 * writes stay explicit and do not become read-modify-write sequences.
 *--------------------------------------------------------------------------*/
#pragma pack(push, 1)

typedef struct __ESP32_GPIO_BT_SELECT_Bits {
    uint32_t BT_SEL              : 32;   //!< Reserved/implementation-specific bit-select field.
} ESP32_GPIO_BT_SELECT_Bits;

typedef struct __ESP32_GPIO_BITMAP_Bits {
    uint32_t DATA                : ESP32_GPIO_PIN_COUNT;          //!< One bit per GPIO pad.
    uint32_t RESERVED            : (32U - ESP32_GPIO_PIN_COUNT);   //!< Reserved/invalid bits.
} ESP32_GPIO_BITMAP_Bits;

typedef struct __ESP32_GPIO_STRAP_Bits {
    uint32_t STRAPPING           : 16;   //!< Strap-pin sample field. Chip-specific bit assignment.
    uint32_t RESERVED            : 16;
} ESP32_GPIO_STRAP_Bits;

typedef struct __ESP32_GPIO_PIN_Bits {
    uint32_t SYNC2_BYPASS        : 2;    //!< Second-stage input synchronization control.
    uint32_t PAD_DRIVER          : 1;    //!< 0: normal output, 1: open-drain output.
    uint32_t SYNC1_BYPASS        : 2;    //!< First-stage input synchronization control.
    uint32_t RESERVED0           : 2;
    uint32_t INT_TYPE            : 3;    //!< ESP32_GPIO_INT_* trigger selection.
    uint32_t WAKEUP_ENABLE       : 1;    //!< Wake CPU from light sleep on this GPIO interrupt.
    uint32_t CONFIG              : 2;    //!< Reserved R/W configuration field.
    uint32_t INT_ENA             : 5;    //!< Interrupt output enable field; use ESP32_GPIO_PIN_INT_ENA_*.
    uint32_t RESERVED1           : 14;
} ESP32_GPIO_PIN_Bits;

typedef struct __ESP32_GPIO_FUNC_IN_Bits {
    uint32_t IN_SEL              : 5;    //!< GPIO number, ESP32_GPIO_FUNC_IN_CONST_HIGH, or CONST_LOW.
    uint32_t IN_INV_SEL          : 1;    //!< 1: invert selected input signal.
    uint32_t SIG_IN_SEL          : 1;    //!< 1: route via GPIO matrix, 0: direct IO_MUX input.
    uint32_t RESERVED            : 25;
} ESP32_GPIO_FUNC_IN_Bits;

typedef struct __ESP32_GPIO_FUNC_OUT_Bits {
    uint32_t OUT_SEL             : 8;    //!< Peripheral output signal index; 128 selects GPIO_OUT/ENABLE.
    uint32_t OUT_INV_SEL         : 1;    //!< 1: invert selected output signal.
    uint32_t OEN_SEL             : 1;    //!< 1: output enable from GPIO_ENABLE, 0: peripheral output enable.
    uint32_t OEN_INV_SEL         : 1;    //!< 1: invert selected output-enable signal.
    uint32_t RESERVED            : 21;
} ESP32_GPIO_FUNC_OUT_Bits;

typedef struct __ESP32_GPIO_CLK_EN_Bits {
    uint32_t CLK_EN              : 1;    //!< 1: GPIO register clock free-running.
    uint32_t RESERVED            : 31;
} ESP32_GPIO_CLK_EN_Bits;

typedef struct __ESP32_GPIO_DATE_Bits {
    uint32_t DATE                : 28;   //!< GPIO version value.
    uint32_t RESERVED            : 4;
} ESP32_GPIO_DATE_Bits;

/*---------------------------------------------------------------------------
 * GPIO/GPIO-matrix register map.
 *
 * This structure is common across the ESP32 RISC-V series used by IOsonata.
 * Chip headers provide ESP32_GPIO_BASE, ESP32_GPIO_PIN_COUNT,
 * ESP32_GPIO_FUNC_IN_COUNT, and ESP32_GPIO_FUNC_OUT_COUNT.
 *
 * Do not name this ESP32_IOMUX_Type: it does not describe the IO_MUX pad
 * register block.  It describes the GPIO/GPIO-matrix block.
 *---------------------------------------------------------------------------*/
typedef struct __ESP32_GPIO_Reg {
    union {
        volatile uint32_t BT_SELECT;        //!< 0x0000 GPIO bit-select register.
        volatile ESP32_GPIO_BT_SELECT_Bits BT_SELECT_Bit;
    };
    union {
        volatile uint32_t OUT;              //!< 0x0004 GPIO output data latch.
        volatile ESP32_GPIO_BITMAP_Bits OUT_Bit;
    };
    union {
        volatile uint32_t OUT_W1TS;         //!< 0x0008 Write 1 to set GPIO_OUT bits.
        volatile ESP32_GPIO_BITMAP_Bits OUT_W1TS_Bit;
    };
    union {
        volatile uint32_t OUT_W1TC;         //!< 0x000C Write 1 to clear GPIO_OUT bits.
        volatile ESP32_GPIO_BITMAP_Bits OUT_W1TC_Bit;
    };
    uint32_t          RESERVED0[4];         //!< 0x0010..0x001C Reserved.

    union {
        volatile uint32_t ENABLE;           //!< 0x0020 GPIO output-enable latch.
        volatile ESP32_GPIO_BITMAP_Bits ENABLE_Bit;
    };
    union {
        volatile uint32_t ENABLE_W1TS;      //!< 0x0024 Write 1 to enable output drivers.
        volatile ESP32_GPIO_BITMAP_Bits ENABLE_W1TS_Bit;
    };
    union {
        volatile uint32_t ENABLE_W1TC;      //!< 0x0028 Write 1 to disable output drivers.
        volatile ESP32_GPIO_BITMAP_Bits ENABLE_W1TC_Bit;
    };
    uint32_t          RESERVED1[3];         //!< 0x002C..0x0034 Reserved.

    union {
        volatile uint32_t STRAPPING;        //!< 0x0038 Strap-pin sampled values.
        volatile ESP32_GPIO_STRAP_Bits STRAPPING_Bit;
    };
    union {
        volatile uint32_t IN;               //!< 0x003C GPIO input values.
        volatile ESP32_GPIO_BITMAP_Bits IN_Bit;
    };
    uint32_t          RESERVED2;            //!< 0x0040 Reserved.

    union {
        volatile uint32_t ISTATUS;          //!< 0x0044 Latched GPIO interrupt status.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_Bit;
    };
    union {
        volatile uint32_t ISTATUS_W1TS;     //!< 0x0048 Write 1 to set latched interrupt bits.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_W1TS_Bit;
    };
    union {
        volatile uint32_t ISTATUS_W1TC;     //!< 0x004C Write 1 to clear latched interrupt bits.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_W1TC_Bit;
    };
    uint32_t          RESERVED3[3];         //!< 0x0050..0x0058 Reserved.

    union {
        volatile uint32_t PROCPU_INT;       //!< 0x005C GPIO interrupt status routed to CPU.
        volatile ESP32_GPIO_BITMAP_Bits PROCPU_INT_Bit;
    };
    uint32_t          RESERVED4[5];         //!< 0x0060..0x0070 Reserved.

    union {
        volatile uint32_t PIN[ESP32_GPIO_PIN_COUNT];
                                             //!< 0x0074 Per-pin interrupt and pad-driver config.
        volatile ESP32_GPIO_PIN_Bits PIN_Bit[ESP32_GPIO_PIN_COUNT];
                                             //!< 0x0074 Per-pin bitfield view.
    };

    uint32_t          RESERVED5[(0x014CU - (0x0074U + (ESP32_GPIO_PIN_COUNT * 4U))) / 4U];
                                             //!< Gap from last PINn register to ISTATUS_NEXT.

    union {
        volatile uint32_t ISTATUS_NEXT;     //!< 0x014C Live GPIO interrupt source status.
        volatile ESP32_GPIO_BITMAP_Bits ISTATUS_NEXT_Bit;
    };
    uint32_t          RESERVED6;            //!< 0x0150 Reserved.

    union {
        volatile uint32_t FUNC_IN_CFG[ESP32_GPIO_FUNC_IN_COUNT];
                                             //!< 0x0154 Peripheral input signal selectors.
        volatile ESP32_GPIO_FUNC_IN_Bits FUNC_IN_CFG_Bit[ESP32_GPIO_FUNC_IN_COUNT];
                                             //!< 0x0154 Peripheral input signal selector bitfield view.
    };

    uint32_t          RESERVED7[(0x0554U - (0x0154U + (ESP32_GPIO_FUNC_IN_COUNT * 4U))) / 4U];
                                             //!< Gap from input selectors to output selectors.

    union {
        volatile uint32_t FUNC_OUT_CFG[ESP32_GPIO_FUNC_OUT_COUNT];
                                             //!< 0x0554 GPIO output signal selectors.
        volatile ESP32_GPIO_FUNC_OUT_Bits FUNC_OUT_CFG_Bit[ESP32_GPIO_FUNC_OUT_COUNT];
                                             //!< 0x0554 GPIO output signal selector bitfield view.
    };

    uint32_t          RESERVED8[(0x062CU - (0x0554U + (ESP32_GPIO_FUNC_OUT_COUNT * 4U))) / 4U];
                                             //!< Gap from output selectors to clock gate.

    union {
        volatile uint32_t CLK_EN;           //!< 0x062C GPIO clock gate control.
        volatile ESP32_GPIO_CLK_EN_Bits CLK_EN_Bit;
    };
    uint32_t          RESERVED9[(0x06FCU - 0x0630U) / 4U];
                                             //!< Gap from clock gate to version register.

    union {
        volatile uint32_t DATE;             //!< 0x06FC GPIO version register.
        volatile ESP32_GPIO_DATE_Bits DATE_Bit;
    };
} ESP32_GPIO_Type;

/*---------------------------------------------------------------------------
 * IO_MUX pad-control register map.
 *
 * This is the physical pad configuration block.  Keep it separate from
 * ESP32_GPIO_Type: GPIO_Type owns output/input/status and GPIO-matrix
 * routing, while IOMUX_Type owns pad function selection, pulls, drive
 * strength, input enable, sleep pad state, and optional direct peripheral
 * bypass.
 *
 * Layout used by the ESP32 RISC-V series:
 *   0x0000          PIN_CTRL       Clock-output selection register.
 *   0x0004 + 4*n    GPIO[n]        IO_MUX_GPIOn pad configuration.
 *   0x00FC          DATE           IO_MUX version register.
 *---------------------------------------------------------------------------*/
typedef struct __ESP32_IOMUX_Reg {
    volatile uint32_t PIN_CTRL;          //!< 0x0000 Clock-output configuration.

    volatile uint32_t GPIO[ESP32_GPIO_PIN_COUNT];
                                           //!< 0x0004 IO_MUX_GPIOn pad configuration.

    uint32_t          RESERVED0[(0x00FCU - (0x0004U + (ESP32_GPIO_PIN_COUNT * 4U))) / 4U];
                                           //!< Gap from last pad register to DATE.

    volatile uint32_t DATE;               //!< 0x00FC IO_MUX version register.
} ESP32_IOMUX_Type;

#pragma pack(pop)

#define ESP32_STATIC_ASSERT(cond, name) typedef char esp32_static_assert_##name[(cond) ? 1 : -1]

ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_BT_SELECT_Bits) == 4U, gpio_bt_select_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_BITMAP_Bits) == 4U, gpio_bitmap_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_STRAP_Bits) == 4U, gpio_strap_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_PIN_Bits) == 4U, gpio_pin_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_FUNC_IN_Bits) == 4U, gpio_func_in_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_FUNC_OUT_Bits) == 4U, gpio_func_out_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_CLK_EN_Bits) == 4U, gpio_clk_en_bits_size);
ESP32_STATIC_ASSERT(sizeof(ESP32_GPIO_DATE_Bits) == 4U, gpio_date_bits_size);

ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, OUT) == 0x0004U, gpio_out_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, ENABLE) == 0x0020U, gpio_enable_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, STRAPPING) == 0x0038U, gpio_strapping_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, ISTATUS) == 0x0044U, gpio_status_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, PROCPU_INT) == 0x005CU, gpio_procpu_int_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, PIN) == 0x0074U, gpio_pin_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, ISTATUS_NEXT) == 0x014CU, gpio_status_next_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, FUNC_IN_CFG) == 0x0154U, gpio_func_in_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, FUNC_OUT_CFG) == 0x0554U, gpio_func_out_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, CLK_EN) == 0x062CU, gpio_clk_en_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_GPIO_Type, DATE) == 0x06FCU, gpio_date_offset);

ESP32_STATIC_ASSERT(offsetof(ESP32_IOMUX_Type, PIN_CTRL) == 0x0000U, iomux_pin_ctrl_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_IOMUX_Type, GPIO) == 0x0004U, iomux_gpio_offset);
ESP32_STATIC_ASSERT(offsetof(ESP32_IOMUX_Type, DATE) == 0x00FCU, iomux_date_offset);

/* GPIO_PINn bit positions and masks. */
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

/* GPIO_FUNCn_IN_SEL_CFG bit positions and masks. */
#define ESP32_GPIO_FUNC_IN_SEL_Pos          0U
#define ESP32_GPIO_FUNC_IN_SEL_Msk          (0x1FUL << ESP32_GPIO_FUNC_IN_SEL_Pos)
#define ESP32_GPIO_FUNC_IN_INV_SEL_Pos      5U
#define ESP32_GPIO_FUNC_IN_INV_SEL_Msk      (1UL << ESP32_GPIO_FUNC_IN_INV_SEL_Pos)
#define ESP32_GPIO_SIG_IN_SEL_Pos           6U
#define ESP32_GPIO_SIG_IN_SEL_Msk           (1UL << ESP32_GPIO_SIG_IN_SEL_Pos)

/* GPIO_FUNCn_OUT_SEL_CFG bit positions and masks. */
#define ESP32_GPIO_FUNC_OUT_SEL_Pos         0U
#define ESP32_GPIO_FUNC_OUT_SEL_Msk         (0xFFUL << ESP32_GPIO_FUNC_OUT_SEL_Pos)
#define ESP32_GPIO_FUNC_OUT_INV_SEL_Pos     8U
#define ESP32_GPIO_FUNC_OUT_INV_SEL_Msk     (1UL << ESP32_GPIO_FUNC_OUT_INV_SEL_Pos)
#define ESP32_GPIO_FUNC_OEN_SEL_Pos         9U
#define ESP32_GPIO_FUNC_OEN_SEL_Msk         (1UL << ESP32_GPIO_FUNC_OEN_SEL_Pos)
#define ESP32_GPIO_FUNC_OEN_INV_SEL_Pos     10U
#define ESP32_GPIO_FUNC_OEN_INV_SEL_Msk     (1UL << ESP32_GPIO_FUNC_OEN_INV_SEL_Pos)

/* IO_MUX_GPIOn bit positions. */
#define ESP32_IOMUX_MCU_OE_Pos        0U
#define ESP32_IOMUX_SLP_SEL_Pos       1U
#define ESP32_IOMUX_MCU_WPD_Pos       2U
#define ESP32_IOMUX_MCU_WPU_Pos       3U
#define ESP32_IOMUX_MCU_IE_Pos        4U
#define ESP32_IOMUX_MCU_DRV_Pos       5U
#define ESP32_IOMUX_MCU_DRV_Msk       (3UL << ESP32_IOMUX_MCU_DRV_Pos)
#define ESP32_IOMUX_FUN_WPD_Pos       7U
#define ESP32_IOMUX_FUN_WPU_Pos       8U
#define ESP32_IOMUX_FUN_IE_Pos        9U
#define ESP32_IOMUX_FUN_DRV_Pos       10U
#define ESP32_IOMUX_FUN_DRV_Msk       (3UL << ESP32_IOMUX_FUN_DRV_Pos)
#define ESP32_IOMUX_MCU_SEL_Pos       12U
#define ESP32_IOMUX_MCU_SEL_Msk       (7UL << ESP32_IOMUX_MCU_SEL_Pos)
#define ESP32_IOMUX_FILTER_EN_Pos     15U
#define ESP32_IOMUX_FILTER_EN_Msk     (1UL << ESP32_IOMUX_FILTER_EN_Pos)

/* Select GPIO/matrix function in IO_MUX_GPIOn.MCU_SEL. */
#define ESP32_IOMUX_MCU_SEL_GPIO      (1UL << ESP32_IOMUX_MCU_SEL_Pos)

#define ESP32_GPIO                    ((ESP32_GPIO_Type *)((uintptr_t)ESP32_GPIO_BASE))
#define ESP32_IOMUX                   ((ESP32_IOMUX_Type *)((uintptr_t)ESP32_IOMUX_BASE))
#define ESP32_IOMUX_PAD_REG(pin)      (ESP32_IOMUX->GPIO[(unsigned)(pin)])



#undef ESP32_GPIO_FUNC_IN_SEL_Msk
#undef ESP32_GPIO_FUNC_IN_INV_SEL_Pos
#undef ESP32_GPIO_FUNC_IN_INV_SEL_Msk
#undef ESP32_GPIO_SIG_IN_SEL_Pos
#undef ESP32_GPIO_SIG_IN_SEL_Msk
#undef ESP32_GPIO_FUNC_OUT_SEL_Msk
#undef ESP32_GPIO_FUNC_OUT_INV_SEL_Pos
#undef ESP32_GPIO_FUNC_OUT_INV_SEL_Msk
#undef ESP32_GPIO_FUNC_OEN_SEL_Pos
#undef ESP32_GPIO_FUNC_OEN_SEL_Msk
#undef ESP32_GPIO_FUNC_OEN_INV_SEL_Pos
#undef ESP32_GPIO_FUNC_OEN_INV_SEL_Msk
/*---------------------------------------------------------------------------
 * Target-aware GPIO/IO_MUX raw register helpers.
 *
 * The ESP32 RISC-V GPIO blocks are similar but not offset-identical.  C3/C6
 * use the compact GPIO matrix layout, while C5 moved several GPIO registers
 * and widened the matrix selector fields.  Use these macros for driver code
 * that must work across C3/C5/C6 instead of relying on ESP32_GPIO_Type field
 * offsets.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_REG32
#define ESP32_REG32(addr)        (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

#define ESP32_GPIO_OUT_REG_OFFSET          0x004U
#define ESP32_GPIO_OUT_W1TS_OFFSET         0x008U
#define ESP32_GPIO_OUT_W1TC_OFFSET         0x00CU

#if defined(ESP32C5)
#define ESP32_GPIO_ENABLE_REG_OFFSET       0x034U
#define ESP32_GPIO_ENABLE_W1TS_OFFSET      0x038U
#define ESP32_GPIO_ENABLE_W1TC_OFFSET      0x03CU
#define ESP32_GPIO_IN_REG_OFFSET           0x064U
#define ESP32_GPIO_STATUS_REG_OFFSET       0x074U
#define ESP32_GPIO_STATUS_W1TS_OFFSET      0x078U
#define ESP32_GPIO_STATUS_W1TC_OFFSET      0x07CU
#define ESP32_GPIO_STATUS_NEXT_OFFSET      0x0C4U
#define ESP32_GPIO_PIN0_REG_OFFSET         0x0D4U
#define ESP32_GPIO_FUNC_IN0_REG_OFFSET     0x2D4U
#define ESP32_GPIO_FUNC_OUT0_REG_OFFSET    0xAD4U
#define ESP32_GPIO_CLOCK_GATE_OFFSET       0xDF8U
#define ESP32_GPIO_DATE_OFFSET             0xDFCU
#define ESP32_IOMUX_PAD0_REG_OFFSET        0x000U
#define ESP32_IOMUX_DATE_OFFSET            0x1FCU
#define ESP32_GPIO_FUNC_IN_SEL_WIDTH       7U
#define ESP32_GPIO_FUNC_IN_SEL_Msk         (0x7FUL << ESP32_GPIO_FUNC_IN_SEL_Pos)
#define ESP32_GPIO_FUNC_IN_INV_SEL_Pos     7U
#define ESP32_GPIO_FUNC_IN_INV_SEL_Msk     (1UL << ESP32_GPIO_FUNC_IN_INV_SEL_Pos)
#define ESP32_GPIO_SIG_IN_SEL_Pos          8U
#define ESP32_GPIO_SIG_IN_SEL_Msk          (1UL << ESP32_GPIO_SIG_IN_SEL_Pos)
#define ESP32_GPIO_FUNC_OUT_SEL_WIDTH      9U
#define ESP32_GPIO_FUNC_OUT_SEL_Msk        (0x1FFUL << ESP32_GPIO_FUNC_OUT_SEL_Pos)
#define ESP32_GPIO_FUNC_OUT_INV_SEL_Pos    9U
#define ESP32_GPIO_FUNC_OUT_INV_SEL_Msk    (1UL << ESP32_GPIO_FUNC_OUT_INV_SEL_Pos)
#define ESP32_GPIO_FUNC_OEN_SEL_Pos        10U
#define ESP32_GPIO_FUNC_OEN_SEL_Msk        (1UL << ESP32_GPIO_FUNC_OEN_SEL_Pos)
#define ESP32_GPIO_FUNC_OEN_INV_SEL_Pos    11U
#define ESP32_GPIO_FUNC_OEN_INV_SEL_Msk    (1UL << ESP32_GPIO_FUNC_OEN_INV_SEL_Pos)
#else
#define ESP32_GPIO_ENABLE_REG_OFFSET       0x020U
#define ESP32_GPIO_ENABLE_W1TS_OFFSET      0x024U
#define ESP32_GPIO_ENABLE_W1TC_OFFSET      0x028U
#define ESP32_GPIO_IN_REG_OFFSET           0x03CU
#define ESP32_GPIO_STATUS_REG_OFFSET       0x044U
#define ESP32_GPIO_STATUS_W1TS_OFFSET      0x048U
#define ESP32_GPIO_STATUS_W1TC_OFFSET      0x04CU
#define ESP32_GPIO_STATUS_NEXT_OFFSET      0x14CU
#define ESP32_GPIO_PIN0_REG_OFFSET         0x074U
#define ESP32_GPIO_FUNC_IN0_REG_OFFSET     0x154U
#define ESP32_GPIO_FUNC_OUT0_REG_OFFSET    0x554U
#define ESP32_GPIO_CLOCK_GATE_OFFSET       0x62CU
#define ESP32_GPIO_DATE_OFFSET             0x6FCU
#define ESP32_IOMUX_PAD0_REG_OFFSET        0x004U
#define ESP32_IOMUX_DATE_OFFSET            0x0FCU
#define ESP32_GPIO_FUNC_OEN_SEL_Pos        9U
#define ESP32_GPIO_FUNC_OEN_SEL_Msk        (1UL << ESP32_GPIO_FUNC_OEN_SEL_Pos)
#define ESP32_GPIO_FUNC_OEN_INV_SEL_Pos    10U
#define ESP32_GPIO_FUNC_OEN_INV_SEL_Msk    (1UL << ESP32_GPIO_FUNC_OEN_INV_SEL_Pos)
#endif

#define ESP32_GPIO_OUT_REG32               ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_OUT_REG_OFFSET)
#define ESP32_GPIO_OUT_W1TS_REG32          ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_OUT_W1TS_OFFSET)
#define ESP32_GPIO_OUT_W1TC_REG32          ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_OUT_W1TC_OFFSET)
#define ESP32_GPIO_ENABLE_REG32            ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_ENABLE_REG_OFFSET)
#define ESP32_GPIO_ENABLE_W1TS_REG32       ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_ENABLE_W1TS_OFFSET)
#define ESP32_GPIO_ENABLE_W1TC_REG32       ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_ENABLE_W1TC_OFFSET)
#define ESP32_GPIO_IN_REG32                ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_IN_REG_OFFSET)
#define ESP32_GPIO_STATUS_REG32            ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_STATUS_REG_OFFSET)
#define ESP32_GPIO_STATUS_W1TS_REG32       ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_STATUS_W1TS_OFFSET)
#define ESP32_GPIO_STATUS_W1TC_REG32       ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_STATUS_W1TC_OFFSET)
#define ESP32_GPIO_STATUS_NEXT_REG32       ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_STATUS_NEXT_OFFSET)
#define ESP32_GPIO_PIN_REG32(pin)          ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_PIN0_REG_OFFSET + ((uint32_t)(pin) * 4U))
#define ESP32_GPIO_FUNC_IN_REG32(sig)      ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_FUNC_IN0_REG_OFFSET + ((uint32_t)(sig) * 4U))
#define ESP32_GPIO_FUNC_OUT_REG32(pin)     ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_FUNC_OUT0_REG_OFFSET + ((uint32_t)(pin) * 4U))
#define ESP32_GPIO_CLOCK_GATE_REG32        ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_CLOCK_GATE_OFFSET)
#define ESP32_GPIO_DATE_REG32              ESP32_REG32(ESP32_GPIO_BASE + ESP32_GPIO_DATE_OFFSET)

#undef ESP32_IOMUX_PAD_REG
#define ESP32_IOMUX_PAD_REG(pin)           ESP32_REG32(ESP32_IOMUX_BASE + ESP32_IOMUX_PAD0_REG_OFFSET + ((uint32_t)(pin) * 4U))
#define ESP32_IOMUX_DATE_REG32             ESP32_REG32(ESP32_IOMUX_BASE + ESP32_IOMUX_DATE_OFFSET)

#define ESP32_GPIO_FUNC_OUT_GPIO_VALUE     \
    (((uint32_t)ESP32_GPIO_FUNC_OUT_SEL_GPIO << ESP32_GPIO_FUNC_OUT_SEL_Pos) | ESP32_GPIO_FUNC_OEN_SEL_Msk)


#endif // __ESP32XX_GPIO_H__
