/**-------------------------------------------------------------------------
@file	esp32xx_spi.h

@brief	ESP32 General-Purpose SPI (GP-SPI2) register definitions for the
        IOsonata RISC-V family.

Covers the GP-SPI2 controller on C3 / C5 / C6.  The SPI memory
controllers (SPI0, SPI1 / SPIMEM0, SPIMEM1) are NOT covered here -- those
are reserved for the flash/PSRAM access path and have a different
register layout.

Register offsets are identical across C3, C5, and C6 for the GP-SPI
controller.  Only the per-instance base address differs:

  * C3:    SPI2 at 0x60024000
  * C5/C6: GP_SPI2 at 0x60081000

ESP32-C3/C5/C6 expose only one general-purpose SPI controller (SPI2).
Earlier ESP32 (Xtensa) variants had SPI3 as well; the C-series does
not.

Bitfield positions and masks have been verified against ESP-IDF v5.3
components/soc/<chip>/include/soc/spi_reg.h.

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
#ifndef __ESP32XX_SPI_H__
#define __ESP32XX_SPI_H__

#include <stdint.h>

#include "esp32xx.h"

#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

/*---------------------------------------------------------------------------
 * Per-instance GP-SPI base addresses.
 *
 * The chip header already defines the instance-specific name; here we
 * provide the no-prefix common alias.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_SPI2_BASE
#  if   defined(ESP32C3)
#    define ESP32_SPI2_BASE                 ESP32C3_SPI2_BASE
#  elif defined(ESP32C5)
#    define ESP32_SPI2_BASE                 ESP32C5_GP_SPI2_BASE
#  elif defined(ESP32C6)
#    define ESP32_SPI2_BASE                 ESP32C6_GP_SPI2_BASE
#  endif
#endif

/*---------------------------------------------------------------------------
 * GP-SPI hardware constants.
 *---------------------------------------------------------------------------*/
#define ESP32_SPI_W_REG_COUNT               16U          //!< W0..W15 data buffer registers (64 bytes total).
#define ESP32_SPI_W_FIFO_BYTES              64U

/*---------------------------------------------------------------------------
 * Per-source-id values.  Verified against IDF interrupts.h.
 *
 * On C5/C6 the source is named ETS_GPSPI2_INTR_SOURCE / ETS_GSPI2_INTR_SOURCE
 * but routes the same GP-SPI2 controller.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C3)
#  define ESP32_SPI_INTR_SOURCE_SPI2        19U
#elif defined(ESP32C6)
#  define ESP32_SPI_INTR_SOURCE_SPI2        76U
#elif defined(ESP32C5)
#  define ESP32_SPI_INTR_SOURCE_SPI2        78U
#endif

/*===========================================================================
 * GP-SPI register offsets (identical across C3 / C5 / C6).
 *===========================================================================*/
#define ESP32_SPI_CMD_OFFSET                0x000U
#define ESP32_SPI_ADDR_OFFSET               0x004U
#define ESP32_SPI_CTRL_OFFSET               0x008U
#define ESP32_SPI_CLOCK_OFFSET              0x00CU
#define ESP32_SPI_USER_OFFSET               0x010U
#define ESP32_SPI_USER1_OFFSET              0x014U
#define ESP32_SPI_USER2_OFFSET              0x018U
#define ESP32_SPI_MS_DLEN_OFFSET            0x01CU
#define ESP32_SPI_MISC_OFFSET               0x020U
#define ESP32_SPI_DIN_MODE_OFFSET           0x024U
#define ESP32_SPI_DIN_NUM_OFFSET            0x028U
#define ESP32_SPI_DOUT_MODE_OFFSET          0x02CU
#define ESP32_SPI_DMA_CONF_OFFSET           0x030U
#define ESP32_SPI_DMA_INT_ENA_OFFSET        0x034U
#define ESP32_SPI_DMA_INT_CLR_OFFSET        0x038U
#define ESP32_SPI_DMA_INT_RAW_OFFSET        0x03CU
#define ESP32_SPI_DMA_INT_ST_OFFSET         0x040U
#if defined(ESP32C5) || defined(ESP32C6)
#  define ESP32_SPI_DMA_INT_SET_OFFSET      0x044U      //!< Software-trigger DMA interrupts.
#endif
#define ESP32_SPI_W0_OFFSET                 0x098U      //!< First of 16 contiguous data buffer registers.
#define ESP32_SPI_SLAVE_OFFSET              0x0E0U
#define ESP32_SPI_SLAVE1_OFFSET             0x0E4U
#define ESP32_SPI_CLK_GATE_OFFSET           0x0E8U
#define ESP32_SPI_DATE_OFFSET               0x0F0U

/*===========================================================================
 * Per-instance register access macros.
 *
 * Pass the SPI base address (ESP32_SPI2_BASE).
 *===========================================================================*/
#define ESP32_SPI_CMD_REG(base)             ESP32_REG32((base) + ESP32_SPI_CMD_OFFSET)
#define ESP32_SPI_ADDR_REG(base)            ESP32_REG32((base) + ESP32_SPI_ADDR_OFFSET)
#define ESP32_SPI_CTRL_REG(base)            ESP32_REG32((base) + ESP32_SPI_CTRL_OFFSET)
#define ESP32_SPI_CLOCK_REG(base)           ESP32_REG32((base) + ESP32_SPI_CLOCK_OFFSET)
#define ESP32_SPI_USER_REG(base)            ESP32_REG32((base) + ESP32_SPI_USER_OFFSET)
#define ESP32_SPI_USER1_REG(base)           ESP32_REG32((base) + ESP32_SPI_USER1_OFFSET)
#define ESP32_SPI_USER2_REG(base)           ESP32_REG32((base) + ESP32_SPI_USER2_OFFSET)
#define ESP32_SPI_MS_DLEN_REG(base)         ESP32_REG32((base) + ESP32_SPI_MS_DLEN_OFFSET)
#define ESP32_SPI_MISC_REG(base)            ESP32_REG32((base) + ESP32_SPI_MISC_OFFSET)
#define ESP32_SPI_DIN_MODE_REG(base)        ESP32_REG32((base) + ESP32_SPI_DIN_MODE_OFFSET)
#define ESP32_SPI_DIN_NUM_REG(base)         ESP32_REG32((base) + ESP32_SPI_DIN_NUM_OFFSET)
#define ESP32_SPI_DOUT_MODE_REG(base)       ESP32_REG32((base) + ESP32_SPI_DOUT_MODE_OFFSET)
#define ESP32_SPI_DMA_CONF_REG(base)        ESP32_REG32((base) + ESP32_SPI_DMA_CONF_OFFSET)
#define ESP32_SPI_DMA_INT_ENA_REG(base)     ESP32_REG32((base) + ESP32_SPI_DMA_INT_ENA_OFFSET)
#define ESP32_SPI_DMA_INT_CLR_REG(base)     ESP32_REG32((base) + ESP32_SPI_DMA_INT_CLR_OFFSET)
#define ESP32_SPI_DMA_INT_RAW_REG(base)     ESP32_REG32((base) + ESP32_SPI_DMA_INT_RAW_OFFSET)
#define ESP32_SPI_DMA_INT_ST_REG(base)      ESP32_REG32((base) + ESP32_SPI_DMA_INT_ST_OFFSET)
#if defined(ESP32C5) || defined(ESP32C6)
#  define ESP32_SPI_DMA_INT_SET_REG(base)   ESP32_REG32((base) + ESP32_SPI_DMA_INT_SET_OFFSET)
#endif
#define ESP32_SPI_W_REG(base, n)            ESP32_REG32((base) + ESP32_SPI_W0_OFFSET + ((uint32_t)(n) * 4U))
#define ESP32_SPI_SLAVE_REG(base)           ESP32_REG32((base) + ESP32_SPI_SLAVE_OFFSET)
#define ESP32_SPI_SLAVE1_REG(base)          ESP32_REG32((base) + ESP32_SPI_SLAVE1_OFFSET)
#define ESP32_SPI_CLK_GATE_REG(base)        ESP32_REG32((base) + ESP32_SPI_CLK_GATE_OFFSET)
#define ESP32_SPI_DATE_REG(base)            ESP32_REG32((base) + ESP32_SPI_DATE_OFFSET)

/*===========================================================================
 * SPI_CMD bit positions.
 *===========================================================================*/
#define ESP32_SPI_CMD_CONF_BITLEN_Pos       0U
#define ESP32_SPI_CMD_CONF_BITLEN_Msk       (0x3FFFFUL << ESP32_SPI_CMD_CONF_BITLEN_Pos)
#define ESP32_SPI_CMD_UPDATE_Pos            23U          //!< Write 1 to commit configuration changes.
#define ESP32_SPI_CMD_UPDATE_Msk            (1UL << ESP32_SPI_CMD_UPDATE_Pos)
#define ESP32_SPI_CMD_USR_Pos               24U          //!< Write 1 to start a user-defined transfer.
#define ESP32_SPI_CMD_USR_Msk               (1UL << ESP32_SPI_CMD_USR_Pos)

/*===========================================================================
 * SPI_USER bit positions (key fields only).
 *===========================================================================*/
#define ESP32_SPI_USER_DOUTDIN_Pos          0U           //!< Full-duplex master mode.
#define ESP32_SPI_USER_DOUTDIN_Msk          (1UL << ESP32_SPI_USER_DOUTDIN_Pos)
#define ESP32_SPI_USER_QPI_MODE_Pos         3U
#define ESP32_SPI_USER_QPI_MODE_Msk         (1UL << ESP32_SPI_USER_QPI_MODE_Pos)
#define ESP32_SPI_USER_TSCK_I_EDGE_Pos      5U
#define ESP32_SPI_USER_TSCK_I_EDGE_Msk      (1UL << ESP32_SPI_USER_TSCK_I_EDGE_Pos)
#define ESP32_SPI_USER_CS_HOLD_Pos          6U
#define ESP32_SPI_USER_CS_HOLD_Msk          (1UL << ESP32_SPI_USER_CS_HOLD_Pos)
#define ESP32_SPI_USER_CS_SETUP_Pos         7U
#define ESP32_SPI_USER_CS_SETUP_Msk         (1UL << ESP32_SPI_USER_CS_SETUP_Pos)
#define ESP32_SPI_USER_RSCK_I_EDGE_Pos      8U
#define ESP32_SPI_USER_RSCK_I_EDGE_Msk      (1UL << ESP32_SPI_USER_RSCK_I_EDGE_Pos)
#define ESP32_SPI_USER_CK_OUT_EDGE_Pos      9U           //!< Clock polarity in master mode.
#define ESP32_SPI_USER_CK_OUT_EDGE_Msk      (1UL << ESP32_SPI_USER_CK_OUT_EDGE_Pos)
#define ESP32_SPI_USER_FWRITE_DUAL_Pos      12U
#define ESP32_SPI_USER_FWRITE_DUAL_Msk      (1UL << ESP32_SPI_USER_FWRITE_DUAL_Pos)
#define ESP32_SPI_USER_FWRITE_QUAD_Pos      13U
#define ESP32_SPI_USER_FWRITE_QUAD_Msk      (1UL << ESP32_SPI_USER_FWRITE_QUAD_Pos)
#define ESP32_SPI_USER_USR_MISO_HIGHPART_Pos 24U
#define ESP32_SPI_USER_USR_MISO_HIGHPART_Msk (1UL << ESP32_SPI_USER_USR_MISO_HIGHPART_Pos)
#define ESP32_SPI_USER_USR_MOSI_HIGHPART_Pos 25U
#define ESP32_SPI_USER_USR_MOSI_HIGHPART_Msk (1UL << ESP32_SPI_USER_USR_MOSI_HIGHPART_Pos)
#define ESP32_SPI_USER_USR_DUMMY_IDLE_Pos   26U
#define ESP32_SPI_USER_USR_DUMMY_IDLE_Msk   (1UL << ESP32_SPI_USER_USR_DUMMY_IDLE_Pos)
#define ESP32_SPI_USER_USR_MOSI_Pos         27U          //!< Enable the MOSI phase.
#define ESP32_SPI_USER_USR_MOSI_Msk         (1UL << ESP32_SPI_USER_USR_MOSI_Pos)
#define ESP32_SPI_USER_USR_MISO_Pos         28U          //!< Enable the MISO phase.
#define ESP32_SPI_USER_USR_MISO_Msk         (1UL << ESP32_SPI_USER_USR_MISO_Pos)
#define ESP32_SPI_USER_USR_DUMMY_Pos        29U          //!< Enable the dummy-cycle phase.
#define ESP32_SPI_USER_USR_DUMMY_Msk        (1UL << ESP32_SPI_USER_USR_DUMMY_Pos)
#define ESP32_SPI_USER_USR_ADDR_Pos         30U          //!< Enable the address phase.
#define ESP32_SPI_USER_USR_ADDR_Msk         (1UL << ESP32_SPI_USER_USR_ADDR_Pos)
#define ESP32_SPI_USER_USR_COMMAND_Pos      31U          //!< Enable the command phase.
#define ESP32_SPI_USER_USR_COMMAND_Msk      (1UL << ESP32_SPI_USER_USR_COMMAND_Pos)

/*===========================================================================
 * SPI_USER1 bit positions (address/dummy bit lengths).
 *===========================================================================*/
#define ESP32_SPI_USER1_USR_DUMMY_CYCLELEN_Pos  0U
#define ESP32_SPI_USER1_USR_DUMMY_CYCLELEN_Msk  (0xFFUL << ESP32_SPI_USER1_USR_DUMMY_CYCLELEN_Pos)
#define ESP32_SPI_USER1_MST_WFULL_ERR_END_Pos   16U
#define ESP32_SPI_USER1_MST_WFULL_ERR_END_Msk   (1UL << ESP32_SPI_USER1_MST_WFULL_ERR_END_Pos)
#define ESP32_SPI_USER1_USR_ADDR_BITLEN_Pos     27U      //!< Address phase length, value = bits-1.
#define ESP32_SPI_USER1_USR_ADDR_BITLEN_Msk     (0x1FUL << ESP32_SPI_USER1_USR_ADDR_BITLEN_Pos)

/*===========================================================================
 * SPI_USER2 bit positions (command field).
 *===========================================================================*/
#define ESP32_SPI_USER2_USR_COMMAND_VALUE_Pos   0U
#define ESP32_SPI_USER2_USR_COMMAND_VALUE_Msk   (0xFFFFUL << ESP32_SPI_USER2_USR_COMMAND_VALUE_Pos)
#define ESP32_SPI_USER2_MST_REMPTY_ERR_END_Pos  27U
#define ESP32_SPI_USER2_MST_REMPTY_ERR_END_Msk  (1UL << ESP32_SPI_USER2_MST_REMPTY_ERR_END_Pos)
#define ESP32_SPI_USER2_USR_COMMAND_BITLEN_Pos  28U      //!< Command field length, value = bits-1.
#define ESP32_SPI_USER2_USR_COMMAND_BITLEN_Msk  (0xFUL << ESP32_SPI_USER2_USR_COMMAND_BITLEN_Pos)

/*===========================================================================
 * SPI_MS_DLEN bit positions (master/slave data bit length).
 *===========================================================================*/
#define ESP32_SPI_MS_DLEN_MS_DATA_BITLEN_Pos    0U       //!< Data phase length, value = bits-1.
#define ESP32_SPI_MS_DLEN_MS_DATA_BITLEN_Msk    (0x3FFFFUL << ESP32_SPI_MS_DLEN_MS_DATA_BITLEN_Pos)

/*===========================================================================
 * SPI_CLOCK bit positions.
 *
 * SCK = ((src_clk / (PRE+1)) / (N+1)) * (H+1)/(L+1) when H/L are used,
 * or simply src_clk / ((PRE+1)*(N+1)) for symmetric clock.
 *===========================================================================*/
#define ESP32_SPI_CLOCK_CLKCNT_L_Pos        0U
#define ESP32_SPI_CLOCK_CLKCNT_L_Msk        (0x3FUL << ESP32_SPI_CLOCK_CLKCNT_L_Pos)
#define ESP32_SPI_CLOCK_CLKCNT_H_Pos        6U
#define ESP32_SPI_CLOCK_CLKCNT_H_Msk        (0x3FUL << ESP32_SPI_CLOCK_CLKCNT_H_Pos)
#define ESP32_SPI_CLOCK_CLKCNT_N_Pos        12U
#define ESP32_SPI_CLOCK_CLKCNT_N_Msk        (0x3FUL << ESP32_SPI_CLOCK_CLKCNT_N_Pos)
#define ESP32_SPI_CLOCK_CLKDIV_PRE_Pos      18U
#define ESP32_SPI_CLOCK_CLKDIV_PRE_Msk      (0xFUL << ESP32_SPI_CLOCK_CLKDIV_PRE_Pos)
#define ESP32_SPI_CLOCK_CLK_EQU_SYSCLK_Pos  31U          //!< 1 = SCK equals source clock (no divider).
#define ESP32_SPI_CLOCK_CLK_EQU_SYSCLK_Msk  (1UL << ESP32_SPI_CLOCK_CLK_EQU_SYSCLK_Pos)

/*===========================================================================
 * SPI_MISC bit positions (CS configuration and pin polarity).
 *===========================================================================*/
#define ESP32_SPI_MISC_CS0_DIS_Pos          0U
#define ESP32_SPI_MISC_CS0_DIS_Msk          (1UL << ESP32_SPI_MISC_CS0_DIS_Pos)
#define ESP32_SPI_MISC_CS1_DIS_Pos          1U
#define ESP32_SPI_MISC_CS1_DIS_Msk          (1UL << ESP32_SPI_MISC_CS1_DIS_Pos)
#define ESP32_SPI_MISC_CS2_DIS_Pos          2U
#define ESP32_SPI_MISC_CS2_DIS_Msk          (1UL << ESP32_SPI_MISC_CS2_DIS_Pos)
#define ESP32_SPI_MISC_CK_DIS_Pos           6U
#define ESP32_SPI_MISC_CK_DIS_Msk           (1UL << ESP32_SPI_MISC_CK_DIS_Pos)
#define ESP32_SPI_MISC_MASTER_CS_POL_Pos    7U           //!< Bitmask: which CS lines are active-high.
#define ESP32_SPI_MISC_MASTER_CS_POL_Msk    (7UL << ESP32_SPI_MISC_MASTER_CS_POL_Pos)
#define ESP32_SPI_MISC_CK_IDLE_EDGE_Pos     29U          //!< Clock polarity (CPOL).
#define ESP32_SPI_MISC_CK_IDLE_EDGE_Msk     (1UL << ESP32_SPI_MISC_CK_IDLE_EDGE_Pos)
#define ESP32_SPI_MISC_CS_KEEP_ACTIVE_Pos   30U
#define ESP32_SPI_MISC_CS_KEEP_ACTIVE_Msk   (1UL << ESP32_SPI_MISC_CS_KEEP_ACTIVE_Pos)

/*===========================================================================
 * SPI_DMA_INT (RAW / ENA / ST / CLR / SET) bit positions.
 *
 * Same bit positions across all four DMA-INT registers.
 *===========================================================================*/
#define ESP32_SPI_DMA_INT_INFIFO_FULL_ERR   (1UL << 0)
#define ESP32_SPI_DMA_INT_OUTFIFO_EMPTY_ERR (1UL << 1)
#define ESP32_SPI_DMA_INT_SLV_EX_QPI        (1UL << 2)
#define ESP32_SPI_DMA_INT_SLV_EN_QPI        (1UL << 3)
#define ESP32_SPI_DMA_INT_SLV_CMD7          (1UL << 4)
#define ESP32_SPI_DMA_INT_SLV_CMD8          (1UL << 5)
#define ESP32_SPI_DMA_INT_SLV_CMD9          (1UL << 6)
#define ESP32_SPI_DMA_INT_SLV_CMDA          (1UL << 7)
#define ESP32_SPI_DMA_INT_SLV_RD_DMA_DONE   (1UL << 8)
#define ESP32_SPI_DMA_INT_SLV_WR_DMA_DONE   (1UL << 9)
#define ESP32_SPI_DMA_INT_SLV_RD_BUF_DONE   (1UL << 10)
#define ESP32_SPI_DMA_INT_SLV_WR_BUF_DONE   (1UL << 11)
#define ESP32_SPI_DMA_INT_TRANS_DONE        (1UL << 12)  //!< Master: transfer complete.
#define ESP32_SPI_DMA_INT_DMA_SEG_TRANS     (1UL << 13)
#define ESP32_SPI_DMA_INT_SEG_MAGIC_ERR     (1UL << 14)
#define ESP32_SPI_DMA_INT_BUF_ADDR_ERR      (1UL << 15)
#define ESP32_SPI_DMA_INT_CMD_ERR           (1UL << 16)
#define ESP32_SPI_DMA_INT_MST_RX_AFIFO_WFULL_ERR (1UL << 17)
#define ESP32_SPI_DMA_INT_MST_TX_AFIFO_REMPTY_ERR (1UL << 18)
#define ESP32_SPI_DMA_INT_APP2              (1UL << 19)
#define ESP32_SPI_DMA_INT_APP1              (1UL << 20)

/*===========================================================================
 * SPI_CLK_GATE bit positions.
 *===========================================================================*/
#define ESP32_SPI_CLK_GATE_CLK_EN_Pos       0U           //!< Master clock enable.
#define ESP32_SPI_CLK_GATE_CLK_EN_Msk       (1UL << ESP32_SPI_CLK_GATE_CLK_EN_Pos)
#define ESP32_SPI_CLK_GATE_MST_CLK_ACTIVE_Pos  1U
#define ESP32_SPI_CLK_GATE_MST_CLK_ACTIVE_Msk  (1UL << ESP32_SPI_CLK_GATE_MST_CLK_ACTIVE_Pos)
#define ESP32_SPI_CLK_GATE_MST_CLK_SEL_Pos     2U        //!< 0 = APB / PLL_F80M, 1 = XTAL.
#define ESP32_SPI_CLK_GATE_MST_CLK_SEL_Msk     (1UL << ESP32_SPI_CLK_GATE_MST_CLK_SEL_Pos)

#endif // __ESP32XX_SPI_H__
