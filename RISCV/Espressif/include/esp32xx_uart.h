/**-------------------------------------------------------------------------
@file	esp32xx_uart.h

@brief	ESP32 UART register definitions for the IOsonata RISC-V family.

Covers C3 / C5 / C6.  Two layouts share this header:

  * C3:    classic UART layout.  CONF0 at 0x20 is direct R/W; CLK_CONF
           at 0x78; ID register at 0x80; no REG_UPDATE register
           (writes auto-sync because UART_ID.HIGH_SPEED defaults to 1).
  * C5/C6: reorganized "_SYNC" layout.  CONF0_SYNC at 0x20; CLK_CONF
           moved to 0x88; pulse/count counters relocated; new
           REG_UPDATE register at 0x98 for committing _SYNC writes;
           ID register at 0x9C.

The same logical fields (CONF0, CONF1, INT_*, STATUS) exist on both
layouts but at different offsets.  This header exposes them through
chip-aware register access macros that hide the offset differences.

Bitfield positions and masks have been verified against ESP-IDF v5.3
components/soc/<chip>/include/soc/uart_reg.h.

UART instance numbering:
  * C3:    UART0 at 0x60000000, UART1 at 0x60010000  (64 KB stride)
  * C5/C6: UART0 at 0x60000000, UART1 at 0x60001000  (4 KB stride),
           plus LP_UART at 0x600B1400.

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
#ifndef __ESP32XX_UART_H__
#define __ESP32XX_UART_H__

#include <stdint.h>

#include "esp32xx.h"

#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

/*---------------------------------------------------------------------------
 * Per-instance UART base addresses.
 *
 * Each chip header already defines its own ESP32CN_UARTn_BASE; this
 * block provides the no-prefix common aliases that drivers use.
 *---------------------------------------------------------------------------*/
#ifndef ESP32_UART0_BASE
#  if   defined(ESP32C3)
#    define ESP32_UART0_BASE                ESP32C3_UART0_BASE
#  elif defined(ESP32C5)
#    define ESP32_UART0_BASE                ESP32C5_UART0_BASE
#  elif defined(ESP32C6)
#    define ESP32_UART0_BASE                ESP32C6_UART0_BASE
#  endif
#endif

#ifndef ESP32_UART1_BASE
#  if   defined(ESP32C3)
#    define ESP32_UART1_BASE                ESP32C3_UART1_BASE
#  elif defined(ESP32C5)
#    define ESP32_UART1_BASE                ESP32C5_UART1_BASE
#  elif defined(ESP32C6)
#    define ESP32_UART1_BASE                ESP32C6_UART1_BASE
#  endif
#endif

#if defined(ESP32C5) || defined(ESP32C6)
#  ifndef ESP32_LP_UART_BASE
#    if defined(ESP32C5)
#      define ESP32_LP_UART_BASE            ESP32C5_LP_UART_BASE
#    else
#      define ESP32_LP_UART_BASE            ESP32C6_LP_UART_BASE
#    endif
#  endif
#endif

/*---------------------------------------------------------------------------
 * UART hardware constants.
 *---------------------------------------------------------------------------*/
#define ESP32_UART_HW_FIFO_LEN              128U        //!< TX/RX hardware FIFO depth (entries).
#define ESP32_UART_XTAL_HZ                  40000000UL  //!< XTAL clock used as UART source on C3/C5/C6.

/*---------------------------------------------------------------------------
 * Per-source-id values.  Verified against IDF interrupts.h.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C3)
#  define ESP32_UART_INTR_SOURCE_UART0      21U
#  define ESP32_UART_INTR_SOURCE_UART1      22U
#elif defined(ESP32C6)
#  define ESP32_UART_INTR_SOURCE_LP_UART    16U
#  define ESP32_UART_INTR_SOURCE_UART0      43U
#  define ESP32_UART_INTR_SOURCE_UART1      44U
#elif defined(ESP32C5)
#  define ESP32_UART_INTR_SOURCE_LP_UART    16U
#  define ESP32_UART_INTR_SOURCE_UART0      47U
#  define ESP32_UART_INTR_SOURCE_UART1      48U
#endif

/*===========================================================================
 * Common register offsets (same on C3 and C5/C6).
 *===========================================================================*/
#define ESP32_UART_FIFO_OFFSET              0x00U
#define ESP32_UART_INT_RAW_OFFSET           0x04U
#define ESP32_UART_INT_ST_OFFSET            0x08U
#define ESP32_UART_INT_ENA_OFFSET           0x0CU
#define ESP32_UART_INT_CLR_OFFSET           0x10U
#define ESP32_UART_RX_FILT_OFFSET           0x18U
#define ESP32_UART_STATUS_OFFSET            0x1CU
#define ESP32_UART_CONF1_OFFSET             0x24U
#define ESP32_UART_MEM_CONF_OFFSET          0x60U

/*===========================================================================
 * Layout-divergent offsets.
 *===========================================================================*/
#if defined(ESP32C3)
// C3 classic layout (no _SYNC suffix; CONF0 directly R/W).
#  define ESP32_UART_CLKDIV_OFFSET          0x14U
#  define ESP32_UART_CONF0_OFFSET           0x20U
#  define ESP32_UART_LOWPULSE_OFFSET        0x28U
#  define ESP32_UART_HIGHPULSE_OFFSET       0x2CU
#  define ESP32_UART_RXD_CNT_OFFSET         0x30U
#  define ESP32_UART_FLOW_CONF_OFFSET       0x34U
#  define ESP32_UART_SLEEP_CONF_OFFSET      0x38U
#  define ESP32_UART_IDLE_CONF_OFFSET       0x48U
#  define ESP32_UART_AT_CMD_PRECNT_OFFSET   0x50U
#  define ESP32_UART_AT_CMD_POSTCNT_OFFSET  0x54U
#  define ESP32_UART_AT_CMD_GAPTOUT_OFFSET  0x58U
#  define ESP32_UART_AT_CMD_CHAR_OFFSET     0x5CU
#  define ESP32_UART_POSPULSE_OFFSET        0x70U
#  define ESP32_UART_NEGPULSE_OFFSET        0x74U
#  define ESP32_UART_CLK_CONF_OFFSET        0x78U
#  define ESP32_UART_DATE_OFFSET            0x7CU
#  define ESP32_UART_ID_OFFSET              0x80U
// REG_UPDATE does not exist on C3; UART_ID.HIGH_SPEED bit 30 defaults
// to 1 so writes auto-sync to the core domain without explicit update.
#elif defined(ESP32C5) || defined(ESP32C6)
// C5/C6 _SYNC layout: CONF0_SYNC requires a REG_UPDATE write to commit.
#  define ESP32_UART_CLKDIV_OFFSET          0x14U   //!< CLKDIV_SYNC_REG.
#  define ESP32_UART_CONF0_OFFSET           0x20U   //!< CONF0_SYNC_REG.
#  define ESP32_UART_HWFC_CONF_OFFSET       0x2CU   //!< HWFC_CONF_SYNC_REG.
#  define ESP32_UART_SLEEP_CONF0_OFFSET     0x30U
#  define ESP32_UART_SLEEP_CONF1_OFFSET     0x34U
#  define ESP32_UART_SLEEP_CONF2_OFFSET     0x38U
#  define ESP32_UART_SWFC_CONF0_OFFSET      0x3CU   //!< SWFC_CONF0_SYNC_REG.
#  define ESP32_UART_SWFC_CONF1_OFFSET      0x40U
#  define ESP32_UART_TXBRK_CONF_OFFSET      0x44U   //!< TXBRK_CONF_SYNC_REG.
#  define ESP32_UART_IDLE_CONF_OFFSET       0x48U   //!< IDLE_CONF_SYNC_REG.
#  define ESP32_UART_RS485_CONF_OFFSET      0x4CU   //!< RS485_CONF_SYNC_REG.
#  define ESP32_UART_AT_CMD_PRECNT_OFFSET   0x50U
#  define ESP32_UART_AT_CMD_POSTCNT_OFFSET  0x54U
#  define ESP32_UART_AT_CMD_GAPTOUT_OFFSET  0x58U
#  define ESP32_UART_AT_CMD_CHAR_OFFSET     0x5CU
#  define ESP32_UART_TOUT_CONF_OFFSET       0x64U   //!< TOUT_CONF_SYNC_REG.
#  define ESP32_UART_MEM_TX_STATUS_OFFSET   0x68U
#  define ESP32_UART_MEM_RX_STATUS_OFFSET   0x6CU
#  define ESP32_UART_FSM_STATUS_OFFSET      0x70U
#  define ESP32_UART_POSPULSE_OFFSET        0x74U
#  define ESP32_UART_NEGPULSE_OFFSET        0x78U
#  define ESP32_UART_LOWPULSE_OFFSET        0x7CU
#  define ESP32_UART_HIGHPULSE_OFFSET       0x80U
#  define ESP32_UART_RXD_CNT_OFFSET         0x84U
#  define ESP32_UART_CLK_CONF_OFFSET        0x88U
#  define ESP32_UART_DATE_OFFSET            0x8CU
#  define ESP32_UART_AFIFO_STATUS_OFFSET    0x90U
#  define ESP32_UART_REG_UPDATE_OFFSET      0x98U   //!< Write 1 to commit _SYNC register changes.
#  define ESP32_UART_ID_OFFSET              0x9CU
#endif

/*===========================================================================
 * Per-instance register access macros.
 *
 * Pass the UART base address (ESP32_UART0_BASE / ESP32_UART1_BASE /
 * ESP32_LP_UART_BASE).
 *===========================================================================*/
#define ESP32_UART_FIFO_REG(base)           ESP32_REG32((base) + ESP32_UART_FIFO_OFFSET)
#define ESP32_UART_INT_RAW_REG(base)        ESP32_REG32((base) + ESP32_UART_INT_RAW_OFFSET)
#define ESP32_UART_INT_ST_REG(base)         ESP32_REG32((base) + ESP32_UART_INT_ST_OFFSET)
#define ESP32_UART_INT_ENA_REG(base)        ESP32_REG32((base) + ESP32_UART_INT_ENA_OFFSET)
#define ESP32_UART_INT_CLR_REG(base)        ESP32_REG32((base) + ESP32_UART_INT_CLR_OFFSET)
#define ESP32_UART_CLKDIV_REG(base)         ESP32_REG32((base) + ESP32_UART_CLKDIV_OFFSET)
#define ESP32_UART_RX_FILT_REG(base)        ESP32_REG32((base) + ESP32_UART_RX_FILT_OFFSET)
#define ESP32_UART_STATUS_REG(base)         ESP32_REG32((base) + ESP32_UART_STATUS_OFFSET)
#define ESP32_UART_CONF0_REG(base)          ESP32_REG32((base) + ESP32_UART_CONF0_OFFSET)
#define ESP32_UART_CONF1_REG(base)          ESP32_REG32((base) + ESP32_UART_CONF1_OFFSET)
#define ESP32_UART_IDLE_CONF_REG(base)      ESP32_REG32((base) + ESP32_UART_IDLE_CONF_OFFSET)
#define ESP32_UART_AT_CMD_PRECNT_REG(base)  ESP32_REG32((base) + ESP32_UART_AT_CMD_PRECNT_OFFSET)
#define ESP32_UART_AT_CMD_POSTCNT_REG(base) ESP32_REG32((base) + ESP32_UART_AT_CMD_POSTCNT_OFFSET)
#define ESP32_UART_AT_CMD_GAPTOUT_REG(base) ESP32_REG32((base) + ESP32_UART_AT_CMD_GAPTOUT_OFFSET)
#define ESP32_UART_AT_CMD_CHAR_REG(base)    ESP32_REG32((base) + ESP32_UART_AT_CMD_CHAR_OFFSET)
#define ESP32_UART_MEM_CONF_REG(base)       ESP32_REG32((base) + ESP32_UART_MEM_CONF_OFFSET)
#define ESP32_UART_POSPULSE_REG(base)       ESP32_REG32((base) + ESP32_UART_POSPULSE_OFFSET)
#define ESP32_UART_NEGPULSE_REG(base)       ESP32_REG32((base) + ESP32_UART_NEGPULSE_OFFSET)
#define ESP32_UART_LOWPULSE_REG(base)       ESP32_REG32((base) + ESP32_UART_LOWPULSE_OFFSET)
#define ESP32_UART_HIGHPULSE_REG(base)      ESP32_REG32((base) + ESP32_UART_HIGHPULSE_OFFSET)
#define ESP32_UART_RXD_CNT_REG(base)        ESP32_REG32((base) + ESP32_UART_RXD_CNT_OFFSET)
#define ESP32_UART_CLK_CONF_REG(base)       ESP32_REG32((base) + ESP32_UART_CLK_CONF_OFFSET)
#define ESP32_UART_DATE_REG(base)           ESP32_REG32((base) + ESP32_UART_DATE_OFFSET)
#define ESP32_UART_ID_REG(base)             ESP32_REG32((base) + ESP32_UART_ID_OFFSET)

#if defined(ESP32C5) || defined(ESP32C6)
#  define ESP32_UART_HWFC_CONF_REG(base)    ESP32_REG32((base) + ESP32_UART_HWFC_CONF_OFFSET)
#  define ESP32_UART_SWFC_CONF0_REG(base)   ESP32_REG32((base) + ESP32_UART_SWFC_CONF0_OFFSET)
#  define ESP32_UART_SWFC_CONF1_REG(base)   ESP32_REG32((base) + ESP32_UART_SWFC_CONF1_OFFSET)
#  define ESP32_UART_TXBRK_CONF_REG(base)   ESP32_REG32((base) + ESP32_UART_TXBRK_CONF_OFFSET)
#  define ESP32_UART_RS485_CONF_REG(base)   ESP32_REG32((base) + ESP32_UART_RS485_CONF_OFFSET)
#  define ESP32_UART_TOUT_CONF_REG(base)    ESP32_REG32((base) + ESP32_UART_TOUT_CONF_OFFSET)
#  define ESP32_UART_FSM_STATUS_REG(base)   ESP32_REG32((base) + ESP32_UART_FSM_STATUS_OFFSET)
#  define ESP32_UART_AFIFO_STATUS_REG(base) ESP32_REG32((base) + ESP32_UART_AFIFO_STATUS_OFFSET)
#  define ESP32_UART_REG_UPDATE_REG(base)   ESP32_REG32((base) + ESP32_UART_REG_UPDATE_OFFSET)
#endif

/*===========================================================================
 * UART_INT_RAW / INT_ST / INT_ENA / INT_CLR bit positions.
 *
 * Same bit assignments on C3, C5, C6.
 *===========================================================================*/
#define ESP32_UART_INT_RXFIFO_FULL          (1UL << 0)   //!< RX FIFO level >= RXFIFO_FULL_THRHD.
#define ESP32_UART_INT_TXFIFO_EMPTY         (1UL << 1)   //!< TX FIFO level <  TXFIFO_EMPTY_THRHD.
#define ESP32_UART_INT_PARITY_ERR           (1UL << 2)   //!< Parity error detected on RX.
#define ESP32_UART_INT_FRM_ERR              (1UL << 3)   //!< Framing error detected on RX.
#define ESP32_UART_INT_RXFIFO_OVF           (1UL << 4)   //!< RX FIFO overflow.
#define ESP32_UART_INT_DSR_CHG              (1UL << 5)
#define ESP32_UART_INT_CTS_CHG              (1UL << 6)
#define ESP32_UART_INT_BRK_DET              (1UL << 7)
#define ESP32_UART_INT_RXFIFO_TOUT          (1UL << 8)   //!< RX timeout (no new bytes for RX_TOUT_THRHD).
#define ESP32_UART_INT_SW_XON               (1UL << 9)
#define ESP32_UART_INT_SW_XOFF              (1UL << 10)
#define ESP32_UART_INT_GLITCH_DET           (1UL << 11)
#define ESP32_UART_INT_TX_BRK_DONE          (1UL << 12)
#define ESP32_UART_INT_TX_BRK_IDLE_DONE     (1UL << 13)
#define ESP32_UART_INT_TX_DONE              (1UL << 14)  //!< TX FIFO drained and last bit shifted out.
#define ESP32_UART_INT_RS485_PARITY_ERR     (1UL << 15)
#define ESP32_UART_INT_RS485_FRM_ERR        (1UL << 16)
#define ESP32_UART_INT_RS485_CLASH          (1UL << 17)
#define ESP32_UART_INT_AT_CMD_CHAR_DET      (1UL << 18)

/*===========================================================================
 * UART_CONF0 (CONF0_SYNC on C5/C6) bit positions.
 *===========================================================================*/
#define ESP32_UART_CONF0_PARITY_Pos         0U           //!< 0 = even, 1 = odd.
#define ESP32_UART_CONF0_PARITY_Msk         (1UL << ESP32_UART_CONF0_PARITY_Pos)
#define ESP32_UART_CONF0_PARITY_EN_Pos      1U
#define ESP32_UART_CONF0_PARITY_EN_Msk      (1UL << ESP32_UART_CONF0_PARITY_EN_Pos)
#define ESP32_UART_CONF0_BIT_NUM_Pos        2U           //!< 5/6/7/8 bits = 0/1/2/3.
#define ESP32_UART_CONF0_BIT_NUM_Msk        (3UL << ESP32_UART_CONF0_BIT_NUM_Pos)
#define ESP32_UART_CONF0_STOP_BIT_Pos       4U           //!< 1 / 1.5 / 2 stop bits = 1/2/3.
#define ESP32_UART_CONF0_STOP_BIT_Msk       (3UL << ESP32_UART_CONF0_STOP_BIT_Pos)
#define ESP32_UART_CONF0_TX_FLOW_EN_Pos     15U
#define ESP32_UART_CONF0_TX_FLOW_EN_Msk     (1UL << ESP32_UART_CONF0_TX_FLOW_EN_Pos)
#define ESP32_UART_CONF0_RXFIFO_RST_Pos     17U
#define ESP32_UART_CONF0_RXFIFO_RST_Msk     (1UL << ESP32_UART_CONF0_RXFIFO_RST_Pos)
#define ESP32_UART_CONF0_TXFIFO_RST_Pos     18U
#define ESP32_UART_CONF0_TXFIFO_RST_Msk     (1UL << ESP32_UART_CONF0_TXFIFO_RST_Pos)
#define ESP32_UART_CONF0_RXD_INV_Pos        19U
#define ESP32_UART_CONF0_RXD_INV_Msk        (1UL << ESP32_UART_CONF0_RXD_INV_Pos)
#define ESP32_UART_CONF0_TXD_INV_Pos        22U
#define ESP32_UART_CONF0_TXD_INV_Msk        (1UL << ESP32_UART_CONF0_TXD_INV_Pos)

/*===========================================================================
 * UART_CONF1 bit positions (RX/TX threshold and timeout enable).
 *===========================================================================*/
#define ESP32_UART_CONF1_RXFIFO_FULL_THRHD_Pos   0U
#define ESP32_UART_CONF1_RXFIFO_FULL_THRHD_Msk   (0x1FFUL << ESP32_UART_CONF1_RXFIFO_FULL_THRHD_Pos)
#define ESP32_UART_CONF1_TXFIFO_EMPTY_THRHD_Pos  9U
#define ESP32_UART_CONF1_TXFIFO_EMPTY_THRHD_Msk  (0x1FFUL << ESP32_UART_CONF1_TXFIFO_EMPTY_THRHD_Pos)
#define ESP32_UART_CONF1_RX_TOUT_THRHD_Pos       16U
#define ESP32_UART_CONF1_RX_TOUT_THRHD_Msk       (0x3FFUL << ESP32_UART_CONF1_RX_TOUT_THRHD_Pos)
#define ESP32_UART_CONF1_RX_TOUT_EN_Pos          21U
#define ESP32_UART_CONF1_RX_TOUT_EN_Msk          (1UL << ESP32_UART_CONF1_RX_TOUT_EN_Pos)

/*===========================================================================
 * UART_STATUS bit positions.
 *===========================================================================*/
#define ESP32_UART_STATUS_RXFIFO_CNT_Pos    0U           //!< Bytes currently in RX FIFO.
#define ESP32_UART_STATUS_RXFIFO_CNT_Msk    (0x3FFUL << ESP32_UART_STATUS_RXFIFO_CNT_Pos)
#define ESP32_UART_STATUS_TXFIFO_CNT_Pos    16U          //!< Bytes currently in TX FIFO.
#define ESP32_UART_STATUS_TXFIFO_CNT_Msk    (0x3FFUL << ESP32_UART_STATUS_TXFIFO_CNT_Pos)

/*===========================================================================
 * UART_CLKDIV bit positions (integer + fractional baud divider).
 *===========================================================================*/
#define ESP32_UART_CLKDIV_INT_Pos           0U
#define ESP32_UART_CLKDIV_INT_Msk           (0xFFFUL << ESP32_UART_CLKDIV_INT_Pos)
#define ESP32_UART_CLKDIV_FRAG_Pos          20U
#define ESP32_UART_CLKDIV_FRAG_Msk          (0xFUL << ESP32_UART_CLKDIV_FRAG_Pos)

/*===========================================================================
 * UART_CLK_CONF bit positions (clock source selection and enable).
 *
 * Same bit positions on C3 and C5/C6 even though the register lives
 * at different offsets between the two layouts.
 *===========================================================================*/
#define ESP32_UART_CLK_CONF_SCLK_DIV_B_Pos   0U
#define ESP32_UART_CLK_CONF_SCLK_DIV_B_Msk   (0x3FUL << ESP32_UART_CLK_CONF_SCLK_DIV_B_Pos)
#define ESP32_UART_CLK_CONF_SCLK_DIV_A_Pos   6U
#define ESP32_UART_CLK_CONF_SCLK_DIV_A_Msk   (0x3FUL << ESP32_UART_CLK_CONF_SCLK_DIV_A_Pos)
#define ESP32_UART_CLK_CONF_SCLK_DIV_NUM_Pos 12U
#define ESP32_UART_CLK_CONF_SCLK_DIV_NUM_Msk (0xFFUL << ESP32_UART_CLK_CONF_SCLK_DIV_NUM_Pos)
#define ESP32_UART_CLK_CONF_SCLK_SEL_Pos     20U
#define ESP32_UART_CLK_CONF_SCLK_SEL_Msk     (3UL << ESP32_UART_CLK_CONF_SCLK_SEL_Pos)
#define ESP32_UART_CLK_CONF_SCLK_SEL_APB     (1UL << ESP32_UART_CLK_CONF_SCLK_SEL_Pos)   //!< APB / 80 MHz default.
#define ESP32_UART_CLK_CONF_SCLK_SEL_RC8M    (2UL << ESP32_UART_CLK_CONF_SCLK_SEL_Pos)   //!< 8 MHz internal / PLL.
#define ESP32_UART_CLK_CONF_SCLK_SEL_XTAL    (3UL << ESP32_UART_CLK_CONF_SCLK_SEL_Pos)   //!< 40 MHz crystal.
#define ESP32_UART_CLK_CONF_SCLK_EN_Pos      22U
#define ESP32_UART_CLK_CONF_SCLK_EN_Msk      (1UL << ESP32_UART_CLK_CONF_SCLK_EN_Pos)
#define ESP32_UART_CLK_CONF_RST_CORE_Pos     23U          //!< UART core soft reset.
#define ESP32_UART_CLK_CONF_RST_CORE_Msk     (1UL << ESP32_UART_CLK_CONF_RST_CORE_Pos)
#define ESP32_UART_CLK_CONF_TX_SCLK_EN_Pos   24U
#define ESP32_UART_CLK_CONF_TX_SCLK_EN_Msk   (1UL << ESP32_UART_CLK_CONF_TX_SCLK_EN_Pos)
#define ESP32_UART_CLK_CONF_RX_SCLK_EN_Pos   25U
#define ESP32_UART_CLK_CONF_RX_SCLK_EN_Msk   (1UL << ESP32_UART_CLK_CONF_RX_SCLK_EN_Pos)

/*===========================================================================
 * UART_ID bit positions.
 *
 * On C3, ID lives at offset 0x80 and includes HIGH_SPEED (bit 30,
 * default 1) which selects auto-sync mode -- writes to CONF0/CONF1/etc.
 * commit immediately without REG_UPDATE.  The REG_UPDATE bit (bit 31)
 * is then used as a status indicator.
 *
 * On C5/C6, ID lives at offset 0x9C and is mostly read-only; the
 * separate REG_UPDATE register at 0x98 serves the same purpose.
 *===========================================================================*/
#define ESP32_UART_ID_REG_UPDATE_Pos        31U
#define ESP32_UART_ID_REG_UPDATE_Msk        (1UL << ESP32_UART_ID_REG_UPDATE_Pos)
#if defined(ESP32C3)
#  define ESP32_UART_ID_HIGH_SPEED_Pos      30U
#  define ESP32_UART_ID_HIGH_SPEED_Msk      (1UL << ESP32_UART_ID_HIGH_SPEED_Pos)
#endif

/*---------------------------------------------------------------------------
 * GPIO matrix signal indices for UART signals.
 *
 * These are the "function index" values written into GPIO_FUNC_OUT_SEL_CFG
 * (for outputs) and GPIO_FUNC_IN_SEL_CFG (for inputs) when routing through
 * the GPIO matrix.  Identical on C5 and C6.  Values verified against
 * IDF gpio_sig_map.h.
 *---------------------------------------------------------------------------*/
#if defined(ESP32C3)
#define ESP32_U0RXD_IN_IDX                  6U
#define ESP32_U0TXD_OUT_IDX                 6U
#define ESP32_U0CTS_IN_IDX                  7U
#define ESP32_U0RTS_OUT_IDX                 7U
#define ESP32_U1RXD_IN_IDX                  9U
#define ESP32_U1TXD_OUT_IDX                 9U
#define ESP32_U1CTS_IN_IDX                  10U
#define ESP32_U1RTS_OUT_IDX                 10U
#elif defined(ESP32C5) || defined(ESP32C6)
#define ESP32_U0RXD_IN_IDX                  6U
#define ESP32_U0TXD_OUT_IDX                 6U
#define ESP32_U0CTS_IN_IDX                  7U
#define ESP32_U0RTS_OUT_IDX                 7U
#define ESP32_U1RXD_IN_IDX                  9U
#define ESP32_U1TXD_OUT_IDX                 9U
#define ESP32_U1CTS_IN_IDX                  10U
#define ESP32_U1RTS_OUT_IDX                 10U
#endif

/*---------------------------------------------------------------------------
 * Per-chip UART peripheral clock-gating helpers.
 *
 * Defined in system_esp32_uart_clock.c (or, equivalently, in the matching
 * system_esp32_system.c / system_esp32_pcr.c if the user prefers to
 * merge the helper bodies into their existing system files).
 *---------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void Esp32UartClockEnable(int devno);
void Esp32UartReset(int devno);

#ifdef __cplusplus
}
#endif

#endif // __ESP32XX_UART_H__
