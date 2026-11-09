/**-------------------------------------------------------------------------
@file	esp32xx_i2c.h

@brief	ESP32 I2C controller register definitions for the IOsonata
        RISC-V family.

HP I2C0 definitions for ESP32-C2/C3/C5/C6/C61, checked against the
ESP-IDF v5.5.1 per-chip soc register headers and hal/i2c_ll.h.
All five controllers have eight command slots. C2 has 16-byte TX/RX
FIFOs; C3/C5/C6/C61 have 32-byte TX/RX FIFOs.
C2/C3 use the local CLK_CONF register; C5/C6/C61 use PCR clock control.
LP I2C clock and pin routing are separate and are not described here.

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
#ifndef __ESP32XX_I2C_H__
#define __ESP32XX_I2C_H__

#include <stdint.h>

#include "esp32xx.h"

#ifndef ESP32_REG32
#define ESP32_REG32(addr)                   (*(volatile uint32_t *)((uintptr_t)(addr)))
#endif

/* HP I2C0 uses the chip header's common peripheral address. */
#if defined(ESP32C2) || defined(ESP32C3) || defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)
#define ESP32_I2C0_BASE			ESP32_I2C_BASE
#else
#error "HP I2C register definitions require ESP32C2/C3/C5/C6/C61."
#endif

#if defined(ESP32C2)
#define ESP32_I2C_FIFO_DEPTH		16U
#else
#define ESP32_I2C_FIFO_DEPTH		32U
#endif
#define ESP32_I2C_CMD_QUEUE_DEPTH	8U

/* GPIO matrix input and output indices are identical for each I2C line. */
#if defined(ESP32C2) || defined(ESP32C3)
#define ESP32_I2C_SCL_SIGNAL		53U
#define ESP32_I2C_SDA_SIGNAL		54U
#elif defined(ESP32C6)
#define ESP32_I2C_SCL_SIGNAL		45U
#define ESP32_I2C_SDA_SIGNAL		46U
#else
#define ESP32_I2C_SCL_SIGNAL		46U
#define ESP32_I2C_SDA_SIGNAL		47U
#endif

#if defined(ESP32C2)
#define ESP32_I2C_INTR_SOURCE_I2C0	22U
#elif defined(ESP32C3)
#define ESP32_I2C_INTR_SOURCE_I2C0	29U
#elif defined(ESP32C6)
#define ESP32_I2C_INTR_SOURCE_LP_I2C	17U
#define ESP32_I2C_INTR_SOURCE_I2C0	50U
#elif defined(ESP32C5)
#define ESP32_I2C_INTR_SOURCE_LP_I2C	17U
#define ESP32_I2C_INTR_SOURCE_I2C0	56U
#elif defined(ESP32C61)
#define ESP32_I2C_INTR_SOURCE_I2C0	45U
#endif

/*===========================================================================
 * HP I2C register offsets. C2 has no slave stretch register.
 *===========================================================================*/
#define ESP32_I2C_SCL_LOW_PERIOD_OFFSET     0x00U
#define ESP32_I2C_CTR_OFFSET                0x04U
#define ESP32_I2C_SR_OFFSET                 0x08U
#define ESP32_I2C_TO_OFFSET                 0x0CU
#if !defined(ESP32C2)
#define ESP32_I2C_SLAVE_ADDR_OFFSET         0x10U
#endif
#define ESP32_I2C_FIFO_ST_OFFSET            0x14U
#define ESP32_I2C_FIFO_CONF_OFFSET          0x18U
#define ESP32_I2C_DATA_OFFSET               0x1CU
#define ESP32_I2C_INT_RAW_OFFSET            0x20U
#define ESP32_I2C_INT_CLR_OFFSET            0x24U
#define ESP32_I2C_INT_ENA_OFFSET            0x28U
#define ESP32_I2C_INT_STATUS_OFFSET         0x2CU
#define ESP32_I2C_SDA_HOLD_OFFSET           0x30U
#define ESP32_I2C_SDA_SAMPLE_OFFSET         0x34U
#define ESP32_I2C_SCL_HIGH_PERIOD_OFFSET    0x38U
#define ESP32_I2C_SCL_START_HOLD_OFFSET     0x40U
#define ESP32_I2C_SCL_RSTART_SETUP_OFFSET   0x44U
#define ESP32_I2C_SCL_STOP_HOLD_OFFSET      0x48U
#define ESP32_I2C_SCL_STOP_SETUP_OFFSET     0x4CU
#define ESP32_I2C_FILTER_CFG_OFFSET         0x50U
#if defined(ESP32C2) || defined(ESP32C3)
#define ESP32_I2C_CLK_CONF_OFFSET           0x54U
#endif
#define ESP32_I2C_COMD0_OFFSET              0x58U       //!< First of CMD_QUEUE_DEPTH command-queue registers.
#define ESP32_I2C_SCL_ST_TIME_OUT_OFFSET      0x78U
#define ESP32_I2C_SCL_MAIN_ST_TIME_OUT_OFFSET 0x7CU
#define ESP32_I2C_SCL_SP_CONF_OFFSET          0x80U
#if !defined(ESP32C2)
#define ESP32_I2C_SCL_STRETCH_CONF_OFFSET     0x84U
#endif
#define ESP32_I2C_DATE_OFFSET               0xF8U
#define ESP32_I2C_TXFIFO_START_ADDR_OFFSET    0x100U  //!< Mapped TX FIFO window for non-FIFO mode.
#define ESP32_I2C_RXFIFO_START_ADDR_OFFSET    0x180U  //!< Mapped RX FIFO window for non-FIFO mode.

/*===========================================================================
 * Per-instance register access macros.
 *
 * Pass ESP32_I2C0_BASE for the HP controller.
 *===========================================================================*/
#define ESP32_I2C_SCL_LOW_PERIOD_REG(base)  ESP32_REG32((base) + ESP32_I2C_SCL_LOW_PERIOD_OFFSET)
#define ESP32_I2C_CTR_REG(base)             ESP32_REG32((base) + ESP32_I2C_CTR_OFFSET)
#define ESP32_I2C_SR_REG(base)              ESP32_REG32((base) + ESP32_I2C_SR_OFFSET)
#define ESP32_I2C_TO_REG(base)              ESP32_REG32((base) + ESP32_I2C_TO_OFFSET)
#if !defined(ESP32C2)
#define ESP32_I2C_SLAVE_ADDR_REG(base)      ESP32_REG32((base) + ESP32_I2C_SLAVE_ADDR_OFFSET)
#endif
#define ESP32_I2C_FIFO_ST_REG(base)         ESP32_REG32((base) + ESP32_I2C_FIFO_ST_OFFSET)
#define ESP32_I2C_FIFO_CONF_REG(base)       ESP32_REG32((base) + ESP32_I2C_FIFO_CONF_OFFSET)
#define ESP32_I2C_DATA_REG(base)            ESP32_REG32((base) + ESP32_I2C_DATA_OFFSET)
#define ESP32_I2C_INT_RAW_REG(base)         ESP32_REG32((base) + ESP32_I2C_INT_RAW_OFFSET)
#define ESP32_I2C_INT_CLR_REG(base)         ESP32_REG32((base) + ESP32_I2C_INT_CLR_OFFSET)
#define ESP32_I2C_INT_ENA_REG(base)         ESP32_REG32((base) + ESP32_I2C_INT_ENA_OFFSET)
#define ESP32_I2C_INT_STATUS_REG(base)      ESP32_REG32((base) + ESP32_I2C_INT_STATUS_OFFSET)
#define ESP32_I2C_SDA_HOLD_REG(base)        ESP32_REG32((base) + ESP32_I2C_SDA_HOLD_OFFSET)
#define ESP32_I2C_SDA_SAMPLE_REG(base)      ESP32_REG32((base) + ESP32_I2C_SDA_SAMPLE_OFFSET)
#define ESP32_I2C_SCL_HIGH_PERIOD_REG(base) ESP32_REG32((base) + ESP32_I2C_SCL_HIGH_PERIOD_OFFSET)
#define ESP32_I2C_SCL_START_HOLD_REG(base)  ESP32_REG32((base) + ESP32_I2C_SCL_START_HOLD_OFFSET)
#define ESP32_I2C_SCL_RSTART_SETUP_REG(base) ESP32_REG32((base) + ESP32_I2C_SCL_RSTART_SETUP_OFFSET)
#define ESP32_I2C_SCL_STOP_HOLD_REG(base)   ESP32_REG32((base) + ESP32_I2C_SCL_STOP_HOLD_OFFSET)
#define ESP32_I2C_SCL_STOP_SETUP_REG(base)  ESP32_REG32((base) + ESP32_I2C_SCL_STOP_SETUP_OFFSET)
#define ESP32_I2C_FILTER_CFG_REG(base)      ESP32_REG32((base) + ESP32_I2C_FILTER_CFG_OFFSET)
#if defined(ESP32C2) || defined(ESP32C3)
#define ESP32_I2C_CLK_CONF_REG(base)        ESP32_REG32((base) + ESP32_I2C_CLK_CONF_OFFSET)
#endif
#define ESP32_I2C_COMD_REG(base, n)         ESP32_REG32((base) + ESP32_I2C_COMD0_OFFSET + ((uint32_t)(n) * 4U))
#define ESP32_I2C_SCL_ST_TIME_OUT_REG(base)       ESP32_REG32((base) + ESP32_I2C_SCL_ST_TIME_OUT_OFFSET)
#define ESP32_I2C_SCL_MAIN_ST_TIME_OUT_REG(base)  ESP32_REG32((base) + ESP32_I2C_SCL_MAIN_ST_TIME_OUT_OFFSET)
#define ESP32_I2C_SCL_SP_CONF_REG(base)           ESP32_REG32((base) + ESP32_I2C_SCL_SP_CONF_OFFSET)
#if !defined(ESP32C2)
#define ESP32_I2C_SCL_STRETCH_CONF_REG(base)      ESP32_REG32((base) + ESP32_I2C_SCL_STRETCH_CONF_OFFSET)
#endif
#define ESP32_I2C_DATE_REG(base)            ESP32_REG32((base) + ESP32_I2C_DATE_OFFSET)

/*===========================================================================
 * I2C_CTR control register bit positions.
 *===========================================================================*/
#define ESP32_I2C_CTR_SDA_FORCE_OUT_Pos     0U           //!< 1 = open-drain output, 0 = direct output.
#define ESP32_I2C_CTR_SDA_FORCE_OUT_Msk     (1UL << ESP32_I2C_CTR_SDA_FORCE_OUT_Pos)
#define ESP32_I2C_CTR_SCL_FORCE_OUT_Pos     1U           //!< 1 = open-drain output, 0 = direct output.
#define ESP32_I2C_CTR_SCL_FORCE_OUT_Msk     (1UL << ESP32_I2C_CTR_SCL_FORCE_OUT_Pos)
#define ESP32_I2C_CTR_RX_FULL_ACK_LEVEL_Pos 3U           //!< ACK level when RX FIFO reaches its threshold.
#define ESP32_I2C_CTR_RX_FULL_ACK_LEVEL_Msk (1UL << ESP32_I2C_CTR_RX_FULL_ACK_LEVEL_Pos)
#define ESP32_I2C_CTR_MS_MODE_Pos           4U           //!< 1 = master, 0 = slave.
#define ESP32_I2C_CTR_MS_MODE_Msk           (1UL << ESP32_I2C_CTR_MS_MODE_Pos)
#define ESP32_I2C_CTR_TRANS_START_Pos       5U           //!< Write 1 to start the queued command sequence.
#define ESP32_I2C_CTR_TRANS_START_Msk       (1UL << ESP32_I2C_CTR_TRANS_START_Pos)
#define ESP32_I2C_CTR_TX_LSB_FIRST_Pos      6U
#define ESP32_I2C_CTR_TX_LSB_FIRST_Msk      (1UL << ESP32_I2C_CTR_TX_LSB_FIRST_Pos)
#define ESP32_I2C_CTR_RX_LSB_FIRST_Pos      7U
#define ESP32_I2C_CTR_RX_LSB_FIRST_Msk      (1UL << ESP32_I2C_CTR_RX_LSB_FIRST_Pos)
#define ESP32_I2C_CTR_CLK_EN_Pos            8U           //!< 1 = enable internal register clock gating.
#define ESP32_I2C_CTR_CLK_EN_Msk            (1UL << ESP32_I2C_CTR_CLK_EN_Pos)
#define ESP32_I2C_CTR_ARBITRATION_EN_Pos    9U
#define ESP32_I2C_CTR_ARBITRATION_EN_Msk    (1UL << ESP32_I2C_CTR_ARBITRATION_EN_Pos)
#define ESP32_I2C_CTR_FSM_RST_Pos           10U          //!< Write 1 to reset the I2C FSM.
#define ESP32_I2C_CTR_FSM_RST_Msk           (1UL << ESP32_I2C_CTR_FSM_RST_Pos)
#define ESP32_I2C_CTR_CONF_UPGATE_Pos       11U          //!< Write 1 to commit configuration to the I2C clock domain.
#define ESP32_I2C_CTR_CONF_UPGATE_Msk       (1UL << ESP32_I2C_CTR_CONF_UPGATE_Pos)

/*===========================================================================
 * I2C_SR status register bit positions.
 *===========================================================================*/
#define ESP32_I2C_SR_RESP_REC_Pos           0U           //!< Last received ACK level (0 = ACK, 1 = NACK).
#define ESP32_I2C_SR_RESP_REC_Msk           (1UL << ESP32_I2C_SR_RESP_REC_Pos)
#if !defined(ESP32C2)
#define ESP32_I2C_SR_SLAVE_RW_Pos           1U
#define ESP32_I2C_SR_SLAVE_RW_Msk           (1UL << ESP32_I2C_SR_SLAVE_RW_Pos)
#endif
#define ESP32_I2C_SR_ARB_LOST_Pos           3U
#define ESP32_I2C_SR_ARB_LOST_Msk           (1UL << ESP32_I2C_SR_ARB_LOST_Pos)
#define ESP32_I2C_SR_BUS_BUSY_Pos           4U
#define ESP32_I2C_SR_BUS_BUSY_Msk           (1UL << ESP32_I2C_SR_BUS_BUSY_Pos)
#if !defined(ESP32C2)
#define ESP32_I2C_SR_SLAVE_ADDRESSED_Pos    5U
#define ESP32_I2C_SR_SLAVE_ADDRESSED_Msk    (1UL << ESP32_I2C_SR_SLAVE_ADDRESSED_Pos)
#endif
#define ESP32_I2C_SR_RXFIFO_CNT_Pos         8U
#define ESP32_I2C_SR_RXFIFO_CNT_Msk         ((ESP32_I2C_FIFO_DEPTH * 2UL - 1UL) << ESP32_I2C_SR_RXFIFO_CNT_Pos)
#define ESP32_I2C_SR_TXFIFO_CNT_Pos         18U
#define ESP32_I2C_SR_TXFIFO_CNT_Msk         ((ESP32_I2C_FIFO_DEPTH * 2UL - 1UL) << ESP32_I2C_SR_TXFIFO_CNT_Pos)
#define ESP32_I2C_SR_SCL_MAIN_STATE_LAST_Pos 24U
#define ESP32_I2C_SR_SCL_MAIN_STATE_LAST_Msk (7UL << ESP32_I2C_SR_SCL_MAIN_STATE_LAST_Pos)
#define ESP32_I2C_SR_SCL_STATE_LAST_Pos     28U
#define ESP32_I2C_SR_SCL_STATE_LAST_Msk     (7UL << ESP32_I2C_SR_SCL_STATE_LAST_Pos)

/*===========================================================================
 * I2C_FIFO_CONF bit positions.
 *===========================================================================*/
#define ESP32_I2C_FIFO_CONF_RXFIFO_WM_THRHD_Pos 0U       //!< RX watermark, fires RXFIFO_WM_INT when count >= threshold.
#define ESP32_I2C_FIFO_CONF_RXFIFO_WM_THRHD_Msk ((ESP32_I2C_FIFO_DEPTH - 1UL) << ESP32_I2C_FIFO_CONF_RXFIFO_WM_THRHD_Pos)
#define ESP32_I2C_FIFO_CONF_TXFIFO_WM_THRHD_Pos 5U       //!< TX watermark, fires TXFIFO_WM_INT when count <= threshold.
#define ESP32_I2C_FIFO_CONF_TXFIFO_WM_THRHD_Msk ((ESP32_I2C_FIFO_DEPTH - 1UL) << ESP32_I2C_FIFO_CONF_TXFIFO_WM_THRHD_Pos)
#define ESP32_I2C_FIFO_CONF_NONFIFO_EN_Pos      10U      //!< 1 = use mapped FIFO window addressing.
#define ESP32_I2C_FIFO_CONF_NONFIFO_EN_Msk      (1UL << ESP32_I2C_FIFO_CONF_NONFIFO_EN_Pos)
#if !defined(ESP32C2)
#define ESP32_I2C_FIFO_CONF_FIFO_ADDR_CFG_EN_Pos 11U
#define ESP32_I2C_FIFO_CONF_FIFO_ADDR_CFG_EN_Msk (1UL << ESP32_I2C_FIFO_CONF_FIFO_ADDR_CFG_EN_Pos)
#endif
#define ESP32_I2C_FIFO_CONF_RX_FIFO_RST_Pos     12U      //!< Write 1 then 0 to reset RX FIFO.
#define ESP32_I2C_FIFO_CONF_RX_FIFO_RST_Msk     (1UL << ESP32_I2C_FIFO_CONF_RX_FIFO_RST_Pos)
#define ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Pos     13U      //!< Write 1 then 0 to reset TX FIFO.
#define ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Msk     (1UL << ESP32_I2C_FIFO_CONF_TX_FIFO_RST_Pos)
#define ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Pos     14U      //!< Enable FIFO over/underrun protection.
#define ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Msk     (1UL << ESP32_I2C_FIFO_CONF_FIFO_PRT_EN_Pos)

/*===========================================================================
 * I2C_INT (RAW / ENA / STATUS / CLR) bit positions.
 *
 * Same layout across all four registers.
 *===========================================================================*/
#define ESP32_I2C_INT_RXFIFO_WM             (1UL << 0)   //!< RX FIFO crossed RXFIFO_WM_THRHD.
#define ESP32_I2C_INT_TXFIFO_WM             (1UL << 1)   //!< TX FIFO dropped to TXFIFO_WM_THRHD.
#define ESP32_I2C_INT_RXFIFO_OVF            (1UL << 2)
#define ESP32_I2C_INT_END_DETECT            (1UL << 3)   //!< END command executed.
#define ESP32_I2C_INT_BYTE_TRANS_DONE       (1UL << 4)   //!< Single byte transferred.
#define ESP32_I2C_INT_ARBITRATION_LOST      (1UL << 5)
#define ESP32_I2C_INT_MST_TXFIFO_UDF        (1UL << 6)
#define ESP32_I2C_INT_TRANS_COMPLETE        (1UL << 7)   //!< STOP command executed (transaction done).
#define ESP32_I2C_INT_TIME_OUT              (1UL << 8)
#define ESP32_I2C_INT_TRANS_START           (1UL << 9)
#define ESP32_I2C_INT_NACK                  (1UL << 10)  //!< Slave returned NACK.
#define ESP32_I2C_INT_TXFIFO_OVF            (1UL << 11)
#define ESP32_I2C_INT_RXFIFO_UDF            (1UL << 12)
#define ESP32_I2C_INT_SCL_ST_TO             (1UL << 13)
#define ESP32_I2C_INT_SCL_MAIN_ST_TO        (1UL << 14)
#define ESP32_I2C_INT_DET_START             (1UL << 15)

/*===========================================================================
 * I2C_CLK_CONF bit positions (clock source selection and divider).
 *
 * C2/C3 only. Integer division uses DIV_NUM + 1 when A and B are zero.
 * SCLK_SEL = 0 selects XTAL; 1 selects RC_FAST.
 *===========================================================================*/
#if defined(ESP32C2) || defined(ESP32C3)
#define ESP32_I2C_CLK_CONF_SCLK_DIV_NUM_Pos 0U
#define ESP32_I2C_CLK_CONF_SCLK_DIV_NUM_Msk (0xFFUL << ESP32_I2C_CLK_CONF_SCLK_DIV_NUM_Pos)
#define ESP32_I2C_CLK_CONF_SCLK_DIV_A_Pos   8U
#define ESP32_I2C_CLK_CONF_SCLK_DIV_A_Msk   (0x3FUL << ESP32_I2C_CLK_CONF_SCLK_DIV_A_Pos)
#define ESP32_I2C_CLK_CONF_SCLK_DIV_B_Pos   14U
#define ESP32_I2C_CLK_CONF_SCLK_DIV_B_Msk   (0x3FUL << ESP32_I2C_CLK_CONF_SCLK_DIV_B_Pos)
#define ESP32_I2C_CLK_CONF_SCLK_SEL_Pos     20U          //!< 0 = XTAL, 1 = RC_FAST.
#define ESP32_I2C_CLK_CONF_SCLK_SEL_Msk     (1UL << ESP32_I2C_CLK_CONF_SCLK_SEL_Pos)
#define ESP32_I2C_CLK_CONF_SCLK_ACTIVE_Pos  21U          //!< 1 = source clock active.
#define ESP32_I2C_CLK_CONF_SCLK_ACTIVE_Msk  (1UL << ESP32_I2C_CLK_CONF_SCLK_ACTIVE_Pos)

#else
/* HP clock source/divider live in PCR on C5/C6/C61. */
#if defined(ESP32C61)
#define ESP32_I2C_PCR_CONF_OFFSET		0x2CU
#define ESP32_I2C_PCR_SCLK_CONF_OFFSET	0x30U
#else
#define ESP32_I2C_PCR_CONF_OFFSET		0x20U
#define ESP32_I2C_PCR_SCLK_CONF_OFFSET	0x24U
#endif
#define ESP32_I2C_PCR_CONF_REG		ESP32_REG32(ESP32_PCR_BASE + ESP32_I2C_PCR_CONF_OFFSET)
#define ESP32_I2C_PCR_SCLK_CONF_REG	ESP32_REG32(ESP32_PCR_BASE + ESP32_I2C_PCR_SCLK_CONF_OFFSET)
#define ESP32_I2C_PCR_CLK_EN		(1UL << 0)
#define ESP32_I2C_PCR_RST_EN		(1UL << 1)
#define ESP32_I2C_PCR_SCLK_DIV_A_Pos	0U
#define ESP32_I2C_PCR_SCLK_DIV_A_Msk	(0x3FUL << 0)
#define ESP32_I2C_PCR_SCLK_DIV_B_Pos	6U
#define ESP32_I2C_PCR_SCLK_DIV_B_Msk	(0x3FUL << 6)
#define ESP32_I2C_PCR_SCLK_DIV_NUM_Pos	12U
#define ESP32_I2C_PCR_SCLK_DIV_NUM_Msk	(0xFFUL << 12)
#define ESP32_I2C_PCR_SCLK_SEL_Pos	20U
#define ESP32_I2C_PCR_SCLK_SEL_Msk	(1UL << 20)
#define ESP32_I2C_PCR_SCLK_EN		(1UL << 22)
#endif

/*===========================================================================
 * I2C_COMDn (command queue) bit positions.
 *
 * Each COMDn register holds one command:
 *   bits [13:0]  = COMMAND value (encodes opcode + byte count + ACK bits)
 *   bit  [31]    = COMMAND_DONE flag (set by hardware when this slot has
 *                  completed; cleared by writing 0 along with the next
 *                  command).
 *
 * Common opcodes (TRM table):
 *   1 = WRITE, 2 = STOP, 3 = READ, 4 = END, 6 = RSTART.
 *===========================================================================*/
#define ESP32_I2C_COMD_COMMAND_Pos          0U
#define ESP32_I2C_COMD_COMMAND_Msk          (0x3FFFUL << ESP32_I2C_COMD_COMMAND_Pos)
#define ESP32_I2C_COMD_DONE_Pos             31U
#define ESP32_I2C_COMD_DONE_Msk             (1UL << ESP32_I2C_COMD_DONE_Pos)

#define ESP32_I2C_COMD_BYTE_NUM_Pos	0U
#define ESP32_I2C_COMD_BYTE_NUM_Msk	0xFFUL
#define ESP32_I2C_COMD_ACK_EN		(1UL << 8)
#define ESP32_I2C_COMD_ACK_EXP		(1UL << 9)
#define ESP32_I2C_COMD_ACK_VAL		(1UL << 10)
#define ESP32_I2C_COMD_OPCODE_Pos		11U
#define ESP32_I2C_COMD_OPCODE_Msk		(7UL << 11)

#define ESP32_I2C_OPCODE_RSTART             6U
#define ESP32_I2C_OPCODE_WRITE              1U
#define ESP32_I2C_OPCODE_READ               3U
#define ESP32_I2C_OPCODE_STOP               2U
#define ESP32_I2C_OPCODE_END                4U

#endif // __ESP32XX_I2C_H__
