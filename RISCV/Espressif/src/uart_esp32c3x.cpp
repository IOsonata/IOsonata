/**-------------------------------------------------------------------------
@file	uart_esp32c3.cpp

@brief	IOsonata UART driver for ESP32-C3 (UART0, UART1)

Targets the legacy SYSTEM-peripheral / INTMTX class of Espressif RISC-V
chips (currently the C3 only — C5/C6/H2/H4 use PCR and CLIC and need their
own driver).

UART register layout: ESP32-C3 TRM Rev 0.4, §17.
  UART0 base : 0x60000000
  UART1 base : 0x60010000

Per-UART register offsets used here:
  0x00  FIFO_REG       — RX read / TX write byte port (HW FIFO is 128 bytes)
  0x04  INT_RAW_REG    — raw interrupt flags
  0x08  INT_ST_REG     — masked status (flag & enable)
  0x0C  INT_ENA_REG    — interrupt enable mask
  0x10  INT_CLR_REG    — write-1-to-clear, like STM32 ICR
  0x14  CLKDIV_REG     — [11:0] integer + [23:20] fractional baud divisor
  0x1C  STATUS_REG     — [9:0] RXFIFO_CNT, [25:16] TXFIFO_CNT
  0x20  CONF0_REG      — bit width, parity, stop, FIFO/core resets, flow ena
  0x24  CONF1_REG      — RX FIFO full threshold, RX timeout enable
  0x78  CLK_CONF_REG   — SCLK_SEL, SCLK_DIV_NUM, SCLK enables, RST_CORE
  0x80  ID_REG         — bit 31 = REG_UPDATE: latch new config to core clock

Clock source choice: SCLK_SEL = 3 (XTAL, 40 MHz) keeps baud independent of
CPU/PLL changes (Esp32SystemInit may switch CPU between 80 MHz and 160 MHz).
With SCLK_DIV_NUM = 0 the UART core clock is XTAL = 40 MHz directly
(C3 formula: SCLK = source / (SCLK_DIV_NUM + 1)).

Baud divider: baud = 40 MHz / (CLKDIV + CLKDIV_FRAG / 16)
  e.g. 115200 baud →  CLKDIV = 347, CLKDIV_FRAG = 4 → 115273 baud (+0.06%).

Pin routing: TX uses GPIO_FUNC_OUT_SEL on the TX pin to drive UART TXD signal
(U0TXD = 6, U1TXD = 9).  RX uses GPIO_FUNC_IN_SEL on the RXD signal index to
read from the chosen GPIO via the matrix.  Pads are first put in MCU_SEL =
GPIO mode by IOPinCfg() so the matrix is in charge.

Interrupt routing: peripheral source (UART0 = 21, UART1 = 22) is mapped to
CPU interrupt 1 in INT_MTX, the CPU INT 1 is enabled at priority 1, and
machine external interrupts are turned on (mie.MEIE, mstatus.MIE).  Any
peripheral routed to CPU INT 1 will fire mcause = 0x8000000B; the trap
handler in Vectors_esp32c3.c reads INTMTX status and dispatches by source
bit.  Multiple peripherals can share the same CPU INT line.

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
#include <string.h>
#include <stdint.h>

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "idelay.h"

/*---------------------------------------------------------------------------
 * UART peripheral registers
 *---------------------------------------------------------------------------*/
#define UART0_BASE                  0x60000000UL
#define UART1_BASE                  0x60010000UL

#define UART_FIFO(base)             (*(volatile uint32_t *)((base) + 0x00U))
#define UART_INT_RAW(base)          (*(volatile uint32_t *)((base) + 0x04U))
#define UART_INT_ST(base)           (*(volatile uint32_t *)((base) + 0x08U))
#define UART_INT_ENA(base)          (*(volatile uint32_t *)((base) + 0x0CU))
#define UART_INT_CLR(base)          (*(volatile uint32_t *)((base) + 0x10U))
#define UART_CLKDIV(base)           (*(volatile uint32_t *)((base) + 0x14U))
#define UART_STATUS(base)           (*(volatile uint32_t *)((base) + 0x1CU))
#define UART_CONF0(base)            (*(volatile uint32_t *)((base) + 0x20U))
#define UART_CONF1(base)            (*(volatile uint32_t *)((base) + 0x24U))
#define UART_IDLE_CONF(base)        (*(volatile uint32_t *)((base) + 0x48U))
#define UART_CLK_CONF(base)         (*(volatile uint32_t *)((base) + 0x78U))
#define UART_ID(base)               (*(volatile uint32_t *)((base) + 0x80U))

/* INT bits */
#define UART_INT_RXFIFO_FULL        (1UL << 0)
#define UART_INT_TXFIFO_EMPTY       (1UL << 1)
#define UART_INT_PARITY_ERR         (1UL << 2)
#define UART_INT_FRM_ERR            (1UL << 3)
#define UART_INT_RXFIFO_OVF         (1UL << 4)
#define UART_INT_RXFIFO_TOUT        (1UL << 8)
#define UART_INT_GLITCH_DET         (1UL << 11)

/* CONF0 bits */
#define UART_CONF0_PARITY           (1UL << 0)      /* 0 = even, 1 = odd */
#define UART_CONF0_PARITY_EN        (1UL << 1)
#define UART_CONF0_BIT_NUM_Pos      2               /* 5/6/7/8 = 0/1/2/3 */
#define UART_CONF0_BIT_NUM_Msk      (3UL << UART_CONF0_BIT_NUM_Pos)
#define UART_CONF0_STOP_BIT_Pos     4               /* 1 / 1.5 / 2 = 1/2/3 */
#define UART_CONF0_STOP_BIT_Msk     (3UL << UART_CONF0_STOP_BIT_Pos)
#define UART_CONF0_TX_FLOW_EN       (1UL << 15)
#define UART_CONF0_RXFIFO_RST       (1UL << 17)
#define UART_CONF0_TXFIFO_RST       (1UL << 18)
#define UART_CONF0_RXD_INV          (1UL << 19)
#define UART_CONF0_TXD_INV          (1UL << 22)
/* NOTE: bit 23 of CONF0 is RTS_INV on the C3, not RST_CORE.  RST_CORE
 * lives in CLK_CONF (UART_CLKCONF_RST_CORE below). */

/* STATUS_REG fields */
#define UART_STATUS_RXFIFO_CNT_Pos  0
#define UART_STATUS_RXFIFO_CNT_Msk  (0x3FFUL << UART_STATUS_RXFIFO_CNT_Pos)
#define UART_STATUS_TXFIFO_CNT_Pos  16
#define UART_STATUS_TXFIFO_CNT_Msk  (0x3FFUL << UART_STATUS_TXFIFO_CNT_Pos)

/* CLK_CONF fields */
#define UART_CLKCONF_SCLK_DIV_B_Pos     0
#define UART_CLKCONF_SCLK_DIV_B_Msk     (0x3FUL << UART_CLKCONF_SCLK_DIV_B_Pos)
#define UART_CLKCONF_SCLK_DIV_A_Pos     6
#define UART_CLKCONF_SCLK_DIV_A_Msk     (0x3FUL << UART_CLKCONF_SCLK_DIV_A_Pos)
#define UART_CLKCONF_SCLK_DIV_NUM_Pos   12
#define UART_CLKCONF_SCLK_DIV_NUM_Msk   (0xFFUL << UART_CLKCONF_SCLK_DIV_NUM_Pos)
#define UART_CLKCONF_SCLK_SEL_Pos       20
#define UART_CLKCONF_SCLK_SEL_Msk       (3UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_SEL_APB       (1UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_SEL_RC8M      (2UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_SEL_XTAL      (3UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_EN            (1UL << 22)
#define UART_CLKCONF_RST_CORE           (1UL << 23)   /* CLK_CONF[23] — UART core soft reset (C3) */
#define UART_CLKCONF_TX_SCLK_EN         (1UL << 24)
#define UART_CLKCONF_RX_SCLK_EN         (1UL << 25)
/* NOTE: bits 26 / 27 of CLK_CONF are unused on the C3 (TX/RX_RST_CORE
 * are S2/S3 family features, not C3).  Don't confuse with bit 23 above. */

/* CLKDIV fields */
#define UART_CLKDIV_INT_Msk             0xFFFUL
#define UART_CLKDIV_FRAG_Pos            20
#define UART_CLKDIV_FRAG_Msk            (0xFUL << UART_CLKDIV_FRAG_Pos)

/* ID_REG: write 1 to bit 31 to latch CONF/CLK changes into core clock domain */
#define UART_ID_REG_UPDATE              (1UL << 31)

#define UART_HW_FIFO_LEN                128

/* UART clock source: XTAL = 40 MHz, fixed by hardware */
#define UART_XTAL_HZ                    40000000UL

/*---------------------------------------------------------------------------
 * SYSTEM peripheral — clock gating / reset for UART0 and UART1
 *---------------------------------------------------------------------------*/
#define SYSTEM_BASE                     0x600C0000UL
#define SYSTEM_PERIP_CLK_EN0_REG        (*(volatile uint32_t *)(SYSTEM_BASE + 0x010U))
#define SYSTEM_PERIP_RST_EN0_REG        (*(volatile uint32_t *)(SYSTEM_BASE + 0x018U))

#define SYSTEM_UART_CLK_EN              (1UL << 2)
#define SYSTEM_UART1_CLK_EN             (1UL << 5)
#define SYSTEM_UART_RST                 (1UL << 2)
#define SYSTEM_UART1_RST                (1UL << 5)

/*---------------------------------------------------------------------------
 * GPIO matrix — input/output signal routing
 * (output GPIO_FUNC_OUT_SEL is already declared in iopincfg_esp32.c)
 *---------------------------------------------------------------------------*/
#define GPIO_BASE                       0x60004000UL
#define GPIO_FUNC_OUT_SEL_REG(pin)      (*(volatile uint32_t *)(GPIO_BASE + 0x554U + ((unsigned)(pin) * 4U)))
#define GPIO_FUNC_IN_SEL_REG(sig)       (*(volatile uint32_t *)(GPIO_BASE + 0x154U + ((unsigned)(sig) * 4U)))

#define GPIO_SIG_IN_SEL                 (1UL << 6)  /* 1 = use GPIO matrix */
#define GPIO_FUNC_IN_INV_SEL            (1UL << 5)

/* UART signal indices (ESP32-C3 TRM gpio_sig_map) */
#define U0RXD_IN_IDX                    6U
#define U0TXD_OUT_IDX                   6U
#define U0CTS_IN_IDX                    7U
#define U0RTS_OUT_IDX                   7U
#define U1RXD_IN_IDX                    9U
#define U1TXD_OUT_IDX                   9U
#define U1CTS_IN_IDX                    10U
#define U1RTS_OUT_IDX                   10U

/*---------------------------------------------------------------------------
 * Interrupt matrix — route a peripheral source to a CPU interrupt line
 * INTMTX_BASE = 0x600C2000.  Per-source MAP register is at offset 4 * source.
 *---------------------------------------------------------------------------*/
#define INTMTX_BASE                     0x600C2000UL
#define INTMTX_SOURCE_MAP_REG(src)      (*(volatile uint32_t *)(INTMTX_BASE + ((unsigned)(src) * 4U)))

/* CPU-side interrupt control (same peripheral, register file alias INTERRUPT_CORE0) */
#define INTC_CPU_INT_ENABLE_REG         (*(volatile uint32_t *)(INTMTX_BASE + 0x104U))
#define INTC_CPU_INT_TYPE_REG           (*(volatile uint32_t *)(INTMTX_BASE + 0x108U))
#define INTC_CPU_INT_PRI_REG(n)         (*(volatile uint32_t *)(INTMTX_BASE + 0x114U + ((unsigned)(n) * 4U)))
#define INTC_CPU_INT_THRESH_REG         (*(volatile uint32_t *)(INTMTX_BASE + 0x194U))

/* Peripheral source numbers (TRM Rev 0.4, Table 9-2) */
#define INTSRC_UART0                    21U
#define INTSRC_UART1                    22U

/* All UART interrupts share CPU INT 1 here.  The single trap handler in
 * Vectors_esp32c3.c reads INTMTX status and dispatches by source bit, so
 * the choice of CPU INT level doesn't change which C handler runs — only
 * priority arbitration.  */
#define UART_CPU_INT_LEVEL              1U
#define UART_CPU_INT_PRIO               1U          /* 1..15, > THRESH */

/*---------------------------------------------------------------------------
 * Per-instance driver data
 *---------------------------------------------------------------------------*/
#define ESP32C3_UART_CFIFO_SIZE         CFIFO_MEMSIZE(64)

#pragma pack(push, 4)
typedef struct {
    int          DevNo;
    uint32_t     Base;
    UARTDev_t   *pUartDev;
    uint8_t      RxFifoMem[ESP32C3_UART_CFIFO_SIZE];
    uint8_t      TxFifoMem[ESP32C3_UART_CFIFO_SIZE];
} ESP32C3_UARTDEV;
#pragma pack(pop)

static ESP32C3_UARTDEV s_Esp32c3UartDev[] = {
    { .DevNo = 0, .Base = UART0_BASE, .pUartDev = nullptr },
    { .DevNo = 1, .Base = UART1_BASE, .pUartDev = nullptr },
};
static const int s_NbUartDev = sizeof(s_Esp32c3UartDev) / sizeof(s_Esp32c3UartDev[0]);

UARTDev_t const * const UARTGetInstance(int DevNo)
{
    if (DevNo < 0 || DevNo >= s_NbUartDev) return nullptr;
    return s_Esp32c3UartDev[DevNo].pUartDev;
}

/*---------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/
static inline uint32_t Esp32c3UartTxFree(uint32_t base)
{
    return UART_HW_FIFO_LEN -
           ((UART_STATUS(base) & UART_STATUS_TXFIFO_CNT_Msk) >> UART_STATUS_TXFIFO_CNT_Pos);
}

static inline uint32_t Esp32c3UartRxAvail(uint32_t base)
{
    return (UART_STATUS(base) & UART_STATUS_RXFIFO_CNT_Msk) >> UART_STATUS_RXFIFO_CNT_Pos;
}

static inline void Esp32c3UartRegUpdate(uint32_t base)
{
    /* Write 1 to ID_REG[31].  Hardware self-clears once the new config has
     * been synchronised into the UART core clock domain.  Spin briefly. */
    UART_ID(base) |= UART_ID_REG_UPDATE;
    while (UART_ID(base) & UART_ID_REG_UPDATE) { /* spin */ }
}

static void Esp32c3UartFifoReset(uint32_t base)
{
    /* Pulse RXFIFO_RST + TXFIFO_RST in CONF0.  Spec wants a write of 1
     * followed by a write of 0; both must be latched via REG_UPDATE. */
    uint32_t c = UART_CONF0(base);
    UART_CONF0(base) = c | UART_CONF0_RXFIFO_RST | UART_CONF0_TXFIFO_RST;
    Esp32c3UartRegUpdate(base);
    UART_CONF0(base) = c & ~(UART_CONF0_RXFIFO_RST | UART_CONF0_TXFIFO_RST);
    Esp32c3UartRegUpdate(base);
}

static void Esp32c3UartSetBaud(uint32_t base, uint32_t baud)
{
    /* Source = SCLK = XTAL / (SCLK_DIV_NUM + 1).  We program
     * SCLK_DIV_NUM = 0 in UARTInit, so SCLK = XTAL = 40 MHz. */
    uint32_t src_hz = UART_XTAL_HZ;

    /* CLKDIV is 12.4 fixed-point: divisor = src/baud, scaled by 16 */
    uint32_t scaled = (uint32_t)(((uint64_t)src_hz * 16ULL + (baud / 2U)) / baud);
    uint32_t intpart = scaled >> 4;
    uint32_t frag    = scaled & 0xFU;

    if (intpart < 1U) intpart = 1U;
    if (intpart > UART_CLKDIV_INT_Msk) intpart = UART_CLKDIV_INT_Msk;

    UART_CLKDIV(base) = (intpart & UART_CLKDIV_INT_Msk)
                      | ((frag << UART_CLKDIV_FRAG_Pos) & UART_CLKDIV_FRAG_Msk);
}

static void Esp32c3UartIntMtxEnable(uint32_t source)
{
    /* Map source -> CPU INT 1 */
    INTMTX_SOURCE_MAP_REG(source) = UART_CPU_INT_LEVEL;

    /* CPU INT 1: level-triggered (TYPE bit 0), priority 1 */
    INTC_CPU_INT_TYPE_REG &= ~(1UL << UART_CPU_INT_LEVEL);
    INTC_CPU_INT_PRI_REG(UART_CPU_INT_LEVEL) = UART_CPU_INT_PRIO;
    INTC_CPU_INT_THRESH_REG = 0U;     /* threshold below all priorities */
    INTC_CPU_INT_ENABLE_REG |= (1UL << UART_CPU_INT_LEVEL);

    /* Enable global mstatus.MIE.  The C3 does NOT implement the standard
     * `mie` CSR (0x304) — writes raise illegal-instruction.  Instead the
     * per-source CPU-INT enable lives in INTC_CPU_INT_ENABLE_REG, which
     * was set above; only the hart-level mstatus.MIE remains.
     * mstatus.MIE = bit 3, set via csrsi mstatus, 8. */
    __asm volatile ("csrsi mstatus, 8" ::: "memory");
}

/*---------------------------------------------------------------------------
 * IRQ handler — invoked from Esp32C3ExtIRQDispatch (Vectors_esp32c3.c)
 *---------------------------------------------------------------------------*/
static void Esp32c3UartIRQHandler(ESP32C3_UARTDEV *pDev)
{
    uint32_t base = pDev->Base;
    UARTDev_t *udev = pDev->pUartDev;
    uint32_t st = UART_INT_ST(base);

    /* Clear handled flags.  RXFIFO_FULL / RXFIFO_TOUT auto-clear by reading
     * the FIFO; the rest are edge-style and need an explicit ICR write. */
    if (st & (UART_INT_FRM_ERR | UART_INT_PARITY_ERR | UART_INT_RXFIFO_OVF | UART_INT_GLITCH_DET))
    {
        UART_INT_CLR(base) = UART_INT_FRM_ERR | UART_INT_PARITY_ERR
                           | UART_INT_RXFIFO_OVF | UART_INT_GLITCH_DET;
    }

    if (st & (UART_INT_RXFIFO_FULL | UART_INT_RXFIFO_TOUT))
    {
        while (Esp32c3UartRxAvail(base) > 0U)
        {
            uint8_t c = (uint8_t)UART_FIFO(base);
            uint8_t *p = CFifoPut(udev->hRxFifo);
            if (p != nullptr) *p = c;
            else udev->RxDropCnt++;
        }
        UART_INT_CLR(base) = UART_INT_RXFIFO_FULL | UART_INT_RXFIFO_TOUT;
        udev->bRxReady = true;

        if (udev->EvtCallback)
        {
            udev->EvtCallback(udev, UART_EVT_RXDATA, nullptr, CFifoUsed(udev->hRxFifo));
        }
    }

    if (st & UART_INT_TXFIFO_EMPTY)
    {
        /* Drain SW FIFO into HW FIFO until either runs out. */
        while (Esp32c3UartTxFree(base) > 0U)
        {
            uint8_t *p = CFifoGet(udev->hTxFifo);
            if (p == nullptr) break;
            UART_FIFO(base) = *p;
        }
        if (CFifoUsed(udev->hTxFifo) == 0)
        {
            /* No more bytes queued: mask TXFIFO_EMPTY so we don't storm. */
            UART_INT_ENA(base) &= ~UART_INT_TXFIFO_EMPTY;
            udev->bTxReady = true;
            if (udev->EvtCallback)
            {
                udev->EvtCallback(udev, UART_EVT_TXREADY, nullptr, 0);
            }
        }
        UART_INT_CLR(base) = UART_INT_TXFIFO_EMPTY;
    }
}

extern "C" void UART0_IRQHandler(void)
{
    Esp32c3UartIRQHandler(&s_Esp32c3UartDev[0]);
}

extern "C" void UART1_IRQHandler(void)
{
    Esp32c3UartIRQHandler(&s_Esp32c3UartDev[1]);
}

/*---------------------------------------------------------------------------
 * DevIntrf operations
 *---------------------------------------------------------------------------*/
static uint32_t Esp32c3UartGetRate(DevIntrf_t * const pDev)
{
    return ((ESP32C3_UARTDEV *)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t Esp32c3UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
    ESP32C3_UARTDEV *dev = (ESP32C3_UARTDEV *)pDev->pDevData;
    Esp32c3UartSetBaud(dev->Base, Rate);
    Esp32c3UartRegUpdate(dev->Base);
    dev->pUartDev->Rate = Rate;
    return Rate;
}

static void Esp32c3UartEnable(DevIntrf_t * const pDev) { (void)pDev; }
static void Esp32c3UartDisable(DevIntrf_t * const pDev) { (void)pDev; }
static void Esp32c3UartPowerOff(DevIntrf_t * const pDev) { (void)pDev; }
static void Esp32c3UartReset(DevIntrf_t * const pDev) { (void)pDev; }

static bool Esp32c3UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr) { (void)pDev; (void)DevAddr; return true; }
static void Esp32c3UartStopRx(DevIntrf_t * const pDev) { (void)pDev; }
static bool Esp32c3UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr) { (void)pDev; (void)DevAddr; return true; }
static void Esp32c3UartStopTx(DevIntrf_t * const pDev) { (void)pDev; }

static int Esp32c3UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
    ESP32C3_UARTDEV *dev = (ESP32C3_UARTDEV *)pDev->pDevData;
    UARTDev_t *udev = dev->pUartDev;
    int cnt = 0;

    if (pDev->bIntEn)
    {
        /* Drain SW CFifo (filled from HW FIFO by ISR) */
        uintptr_t state = DisableInterrupt();
        while (BuffLen > 0)
        {
            int l = BuffLen;
            uint8_t *p = CFifoGetMultiple(udev->hRxFifo, &l);
            if (p == nullptr) break;
            memcpy(pBuff, p, l);
            pBuff += l;
            BuffLen -= l;
            cnt += l;
        }
        EnableInterrupt(state);
    }
    else
    {
        /* Polled: drain HW FIFO directly */
        while (BuffLen > 0 && Esp32c3UartRxAvail(dev->Base) > 0U)
        {
            *pBuff++ = (uint8_t)UART_FIFO(dev->Base);
            BuffLen--;
            cnt++;
        }
    }

    return cnt;
}

static int Esp32c3UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
    ESP32C3_UARTDEV *dev = (ESP32C3_UARTDEV *)pDev->pDevData;
    UARTDev_t *udev = dev->pUartDev;
    int cnt = 0;

    /* ---------------------------------------------------------------
     * DIAGNOSTIC (TEMPORARY): on every entry, dump driver state via
     * direct register pokes to UART0 FIFO — bypassing whatever state
     * the driver itself might be in.  Stage B has already proven the
     * UART core works at this point, so this output is reliable.
     *
     * Wire format: E0 <BASE byte 0> <BASE 1> <BASE 2> <BASE 3> E0
     *              <bIntEn?00:11> <TxFree value> <STATUS hi byte> E0
     *
     * Expected:    E0 60 00 00 00 E0 00 80 00 E0
     *              ^-- dev->Base = 0x60000000 (big-endian)
     *                              ^-- bIntEn = 0 (polled mode)
     *                                 ^-- TxFree = 128 (FIFO empty)
     *                                    ^-- STATUS[31:24] = 0
     * --------------------------------------------------------------- */
    {
        volatile uint32_t *FIFO_DBG   = (volatile uint32_t *)0x60000000UL;
        volatile uint32_t *STATUS_DBG = (volatile uint32_t *)0x6000001CUL;
        uint32_t base_val = dev->Base;
        uint32_t status_val = *STATUS_DBG;
        uint32_t txfree = 128U - ((status_val >> 16) & 0x3FFU);

        #define DBG_PUSH(b)  do { \
            uint32_t g = 0; \
            while ((((*STATUS_DBG) >> 16) & 0x3FFU) >= 128U) { \
                if (++g >= 1000000U) break; \
            } \
            *FIFO_DBG = (uint32_t)(b); \
        } while (0)

        DBG_PUSH(0xE0U);
        DBG_PUSH((base_val >> 24) & 0xFFU);
        DBG_PUSH((base_val >> 16) & 0xFFU);
        DBG_PUSH((base_val >>  8) & 0xFFU);
        DBG_PUSH( base_val        & 0xFFU);
        DBG_PUSH(0xE0U);
        DBG_PUSH(pDev->bIntEn ? 0x11U : 0x00U);
        DBG_PUSH(txfree & 0xFFU);
        DBG_PUSH((status_val >> 24) & 0xFFU);
        DBG_PUSH(0xE0U);

        #undef DBG_PUSH
    }

    if (pDev->bIntEn)
    {
        int rtry = pDev->MaxRetry;
        while (DataLen > 0 && rtry-- > 0)
        {
            uintptr_t state = DisableInterrupt();

            while (DataLen > 0)
            {
                int l = DataLen;
                uint8_t *p = CFifoPutMultiple(udev->hTxFifo, &l);
                if (p == nullptr) break;
                memcpy(p, pData, l);
                pData += l;
                DataLen -= l;
                cnt += l;
            }

            /* Kick the ISR pump if idle.  We arm TXFIFO_EMPTY and let the
             * ISR drain SW -> HW.  We seed at least one byte directly so
             * TXFIFO_EMPTY edge fires reliably even if all we queued fits
             * in the HW FIFO right now. */
            if (udev->bTxReady && CFifoUsed(udev->hTxFifo) > 0)
            {
                udev->bTxReady = false;
                while (Esp32c3UartTxFree(dev->Base) > 0U)
                {
                    uint8_t *p = CFifoGet(udev->hTxFifo);
                    if (p == nullptr) break;
                    UART_FIFO(dev->Base) = *p;
                }
                UART_INT_CLR(dev->Base) = UART_INT_TXFIFO_EMPTY;
                UART_INT_ENA(dev->Base) |= UART_INT_TXFIFO_EMPTY;
            }

            EnableInterrupt(state);

            /* If SW FIFO is full there's nothing useful to retry on; bail. */
            if (DataLen > 0 && CFifoAvail(udev->hTxFifo) == 0)
            {
                udev->TxDropCnt += DataLen;
                break;
            }
        }
    }
    else
    {
        /* Polled: spin on HW TX FIFO free space.  Safe but blocking. */
        while (DataLen > 0)
        {
            while (Esp32c3UartTxFree(dev->Base) == 0U) { /* spin */ }
            UART_FIFO(dev->Base) = *pData++;
            DataLen--;
            cnt++;
        }
    }

    return cnt;
}

/*---------------------------------------------------------------------------
 * Pin configuration
 *
 * For the C3, every UART signal has at most one "default" pin that supports
 * an IOMUX direct path (function 0 of that pad routes the UART signal
 * straight through, bypassing the GPIO matrix entirely).  The defaults are:
 *
 *     UART0  TX = GPIO 21 (IOMUX func 0 = U0TXD)
 *     UART0  RX = GPIO 20 (IOMUX func 0 = U0RXD)
 *     UART1: no default — always GPIO matrix
 *
 * IDF prefers the IOMUX direct path for the default pin
 * (uart_try_set_iomux_pin in components/esp_driver_uart/src/uart.c) and
 * falls back to the GPIO matrix only when a non-default pin is requested.
 * The ROM bootloader also uses the direct path for UART0 — so when our
 * code starts running, GPIO 21 already has IOMUX[21].MCU_SEL = 0 and is
 * driving the U0TXD signal directly.
 *
 * Why this matters: IOPinCfg() calls IOPinConfig() which writes
 *     IOMUX[pin].MCU_SEL = 1            (GPIO matrix mode)
 *     GPIO_FUNC_OUT_SEL[pin] = 128      (SIG_GPIO_OUT_IDX, simple GPIO)
 *     GPIO_OUT_REG[pin] = 0             (default low)
 * before the driver routes U0TXD into the matrix.  During that brief
 * window the pad is driven LOW from a GPIO source — the receiver sees
 * a long start-bit-shaped low, then a snap back to high when we finally
 * write GPIO_FUNC_OUT_SEL[21] = U0TXD.  That decodes to one spurious
 * 0x00 / 0x80 byte at every Init().
 *
 * For the default pins we sidestep the whole matrix dance: leave MCU_SEL
 * at 0 (or set it to 0 explicitly) and let IOMUX route U0TXD/U0RXD
 * straight through.  No GPIO_FUNC_OUT_SEL writes, no transient glitch,
 * no GPIO_ENABLE bookkeeping (the UART signal is its own driver).
 *
 * For non-default pins (UART1, or UART0 on a remapped pin) we keep the
 * GPIO matrix path as before.
 *---------------------------------------------------------------------------*/

/* IOMUX register access (must match iopincfg_esp32.c constants). */
#define ESP32_IOMUX_BASE                0x60009000UL
#define IOMUX_PAD_REG_DRV(n)            (*(volatile uint32_t *)(ESP32_IOMUX_BASE + 0x04U + ((unsigned)(n) * 4U)))

/* On the C3, IOMUX_PAD_REG fields are (per soc/io_mux_reg.h):
 *   bit  7   FUN_PD       (function pull-down enable)
 *   bit  8   FUN_PU       (function pull-up enable)
 *   bit  9   FUN_IE       (function input enable)
 *   bits 11:10 FUN_DRV    (function drive strength, 0..3)
 *   bits 14:12 MCU_SEL    (function selector — 3 BITS, not 2)
 *   bit  15  FILTER_EN
 * plus low-order MCU_* bits (deep-sleep state) at 0..6.
 */
#define IOMUX_FUN_IE_BIT                (1UL << 9)
#define IOMUX_FUN_DRV_2                 (2UL << 10)
#define IOMUX_MCU_SEL_Msk               (7UL << 12)   /* 3 bits */
#define IOMUX_MCU_SEL_FUNC0             (0UL << 12)   /* direct path (UART signal) */
#define IOMUX_MCU_SEL_FUNC1             (1UL << 12)   /* GPIO matrix path */

/* Default-pin lookup: returns IOMUX function for direct routing, or -1
 * if the (devno, role, pin) combination has no IOMUX direct path. */
static int Esp32c3UartIomuxFunc(int DevNo, int RoleIdx, int Pin)
{
    if (DevNo == 0)
    {
        if (RoleIdx == UARTPIN_TX_IDX && Pin == 21) return 0;   /* FUNC_U0TXD_U0TXD */
        if (RoleIdx == UARTPIN_RX_IDX && Pin == 20) return 0;   /* FUNC_U0RXD_U0RXD */
    }
    return -1;
}

static void Esp32c3UartCfgPins(int DevNo, const IOPinCfg_t *pCfg, int NbPins,
                               UART_FLWCTRL FlowCtrl)
{
    int rx_pin = pCfg[UARTPIN_RX_IDX].PinNo;
    int tx_pin = pCfg[UARTPIN_TX_IDX].PinNo;

    uint32_t rxd_sig, txd_sig, cts_sig, rts_sig;
    if (DevNo == 0)
    {
        rxd_sig = U0RXD_IN_IDX;
        txd_sig = U0TXD_OUT_IDX;
        cts_sig = U0CTS_IN_IDX;
        rts_sig = U0RTS_OUT_IDX;
    }
    else
    {
        rxd_sig = U1RXD_IN_IDX;
        txd_sig = U1TXD_OUT_IDX;
        cts_sig = U1CTS_IN_IDX;
        rts_sig = U1RTS_OUT_IDX;
    }

    /* TX pin --------------------------------------------------------- */
    int tx_iomux_func = Esp32c3UartIomuxFunc(DevNo, UARTPIN_TX_IDX, tx_pin);
    if (tx_iomux_func >= 0)
    {
        /* Direct IOMUX path.  Read-modify-write: only touch MCU_SEL.
         * Preserve every other bit ROM had set (FUN_DRV, FUN_PU, FUN_IE,
         * FILTER_EN, MCU_OE).  ROM had MCU_SEL = 0 already for U0TXD on
         * GPIO 21, so in the common case this is a write-back of the
         * same value with no observable transition on the pad. */
        uint32_t v = IOMUX_PAD_REG_DRV(tx_pin);
        v &= ~IOMUX_MCU_SEL_Msk;
        v |= ((uint32_t)tx_iomux_func << 12) & IOMUX_MCU_SEL_Msk;
        IOMUX_PAD_REG_DRV(tx_pin) = v;
    }
    else if (tx_pin >= 0)
    {
        /* GPIO matrix path (non-default pin or UART1).  Use the original
         * sequence: pad config via IOPinConfig, then route U0TXD signal
         * through the matrix. */
        IOPinConfig(pCfg[UARTPIN_TX_IDX].PortNo, tx_pin, 0,
                    IOPINDIR_OUTPUT, pCfg[UARTPIN_TX_IDX].Res,
                    pCfg[UARTPIN_TX_IDX].Type);
        GPIO_FUNC_OUT_SEL_REG(tx_pin) = txd_sig & 0xFFUL;
    }

    /* RX pin --------------------------------------------------------- */
    int rx_iomux_func = Esp32c3UartIomuxFunc(DevNo, UARTPIN_RX_IDX, rx_pin);
    if (rx_iomux_func >= 0)
    {
        /* Direct IOMUX path.  RMW + ensure FUN_IE = 1 so the input
         * buffer is on (RX needs it).  Apply pull-up/down if requested. */
        uint32_t v = IOMUX_PAD_REG_DRV(rx_pin);
        v &= ~(IOMUX_MCU_SEL_Msk | (1UL << 8) | (1UL << 7));   /* clear MCU_SEL, FUN_PU, FUN_PD */
        v |= ((uint32_t)rx_iomux_func << 12) & IOMUX_MCU_SEL_Msk;
        v |= IOMUX_FUN_IE_BIT;
        if (pCfg[UARTPIN_RX_IDX].Res == IOPINRES_PULLUP)   v |= (1UL << 8);
        else if (pCfg[UARTPIN_RX_IDX].Res == IOPINRES_PULLDOWN) v |= (1UL << 7);
        IOMUX_PAD_REG_DRV(rx_pin) = v;
    }
    else if (rx_pin >= 0)
    {
        /* GPIO matrix path */
        IOPinConfig(pCfg[UARTPIN_RX_IDX].PortNo, rx_pin, 0,
                    IOPINDIR_INPUT, pCfg[UARTPIN_RX_IDX].Res,
                    pCfg[UARTPIN_RX_IDX].Type);
        GPIO_FUNC_IN_SEL_REG(rxd_sig) = ((uint32_t)rx_pin & 0x1FU) | GPIO_SIG_IN_SEL;
    }

    /* Optional CTS / RTS via GPIO matrix (no IOMUX direct path on C3). */
    if (FlowCtrl == UART_FLWCTRL_HW && NbPins >= 4)
    {
        int cts_pin = pCfg[UARTPIN_CTS_IDX].PinNo;
        int rts_pin = pCfg[UARTPIN_RTS_IDX].PinNo;

        if (cts_pin >= 0)
        {
            IOPinConfig(pCfg[UARTPIN_CTS_IDX].PortNo, cts_pin, 0,
                        IOPINDIR_INPUT, pCfg[UARTPIN_CTS_IDX].Res,
                        pCfg[UARTPIN_CTS_IDX].Type);
            GPIO_FUNC_IN_SEL_REG(cts_sig) = ((uint32_t)cts_pin & 0x1FU) | GPIO_SIG_IN_SEL;
        }
        if (rts_pin >= 0)
        {
            IOPinConfig(pCfg[UARTPIN_RTS_IDX].PortNo, rts_pin, 0,
                        IOPINDIR_OUTPUT, pCfg[UARTPIN_RTS_IDX].Res,
                        pCfg[UARTPIN_RTS_IDX].Type);
            GPIO_FUNC_OUT_SEL_REG(rts_pin) = rts_sig & 0xFFUL;
        }
    }
}

/*---------------------------------------------------------------------------
 * UARTInit — top-level driver init
 *---------------------------------------------------------------------------*/
bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
    if (pDev == nullptr || pCfg == nullptr) return false;
    if (pCfg->pIOPinMap == nullptr || pCfg->NbIOPins < 2) return false;
    if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev) return false;

    int devno = pCfg->DevNo;
    ESP32C3_UARTDEV *dev = &s_Esp32c3UartDev[devno];
    uint32_t base = dev->Base;

    pDev->DevIntrf.pDevData = dev;
    dev->pUartDev = pDev;

    /* 1. Enable APB clock to the UART, and reset the peripheral.
     *
     * UART0 IS THE ROM CONSOLE.  ESP-IDF deliberately SKIPS the module
     * reset on whichever UART is the console (esp_driver_uart/src/uart.c:218):
     *
     *     if (uart_num != CONFIG_ESP_CONSOLE_UART_NUM) {
     *         uart_ll_reset_register(uart_num);
     *     }
     *
     * ROM hands UART0 off in a working state — its CLKDIV, CONF0, FIFO,
     * and TX state machine are all sane.  Pulsing SYSTEM_UART_RST against
     * a running core (even with the C3 RST_CORE bracket) interacts badly
     * with the GPIO-matrix pin transition that follows in step 11: the
     * core comes out of reset, briefly accepts FIFO writes, but its TX
     * shifter never re-engages.  Symptom: Tx() pushes bytes into the
     * HW FIFO indefinitely without anything appearing on the wire.
     *
     * Just enable the bus clock (already on for UART0 from ROM, but the
     * write is idempotent) and proceed.  CONF0/CONF1/CLKDIV/CLK_CONF
     * are then explicitly programmed below to known values, so we don't
     * rely on whatever ROM left in those registers — only on the fact
     * that the UART core is alive and clocked.
     *
     * For UART1 the full C3 RST_CORE bracket is still required, since
     * ROM never enabled UART1: without it the core latches garbage out
     * of reset (per IDF v5.3 hal/esp32c3/uart_ll.h:107). */
    if (devno == 0)
    {
        SYSTEM_PERIP_CLK_EN0_REG |= SYSTEM_UART_CLK_EN;
        /* No module reset.  UART0 is the ROM console; resetting it
         * combined with the GPIO-matrix transition wedges the TX FSM. */
    }
    else
    {
        SYSTEM_PERIP_CLK_EN0_REG |= SYSTEM_UART1_CLK_EN;
        UART_CLK_CONF(base) |= UART_CLKCONF_RST_CORE;
        SYSTEM_PERIP_RST_EN0_REG |=  SYSTEM_UART1_RST;
        SYSTEM_PERIP_RST_EN0_REG &= ~SYSTEM_UART1_RST;
        UART_CLK_CONF(base) &= ~UART_CLKCONF_RST_CORE;
    }

    /* 2. Mask all UART interrupts while we configure (safe regardless of
     *    whether UART is running — these are interrupt-enable bits, not
     *    state machine bits). */
    UART_INT_ENA(base) = 0U;
    UART_INT_CLR(base) = 0xFFFFFFFFUL;

    /* 3. Clock source selection.
     *
     * UART0: surgical update.  ROM left CLK_CONF with all the right
     * enables (SCLK_EN, TX_SCLK_EN, RX_SCLK_EN are all 1 by default and
     * ROM keeps them so).  We only want to (a) point the source at XTAL
     * so baud is independent of any later CPU/APB clock change, and
     * (b) zero the integer pre-divider so SCLK = XTAL = 40 MHz exactly.
     * Read-modify-write only those two fields (sclk_sel, sclk_div_num);
     * leaving sclk_div_a, sclk_div_b at 0, and the enable bits and
     * rst_core untouched at whatever they were (always 1, 1, 1, 0 in
     * practice, but we don't disturb them).
     *
     * UART1: full overwrite is fine — the RST_CORE bracket above already
     * reset CLK_CONF to defaults, so this just sets up our chosen state.
     *
     * Why this matters: the previous code did an UNCONDITIONAL full
     * overwrite of CLK_CONF on UART0.  When UART0 came into Init()
     * already running (post-Stage-A or after ROM banner remnants), the
     * full overwrite SWITCHED THE SCLK SOURCE underneath an in-flight
     * TX byte.  The TX shifter wedged at variable points — symptom was
     * a non-deterministic truncated AA stream (sometimes 2 bytes,
     * sometimes 10).  Surgical update doesn't disturb the shifter. */
    if (devno == 0)
    {
        uint32_t cc = UART_CLK_CONF(base);
        cc &= ~(UART_CLKCONF_SCLK_SEL_Msk | UART_CLKCONF_SCLK_DIV_NUM_Msk
              | UART_CLKCONF_SCLK_DIV_A_Msk | UART_CLKCONF_SCLK_DIV_B_Msk);
        cc |= UART_CLKCONF_SCLK_SEL_XTAL;   /* sclk_div_num stays 0 → /1 */
        UART_CLK_CONF(base) = cc;
    }
    else
    {
        UART_CLK_CONF(base) = UART_CLKCONF_SCLK_SEL_XTAL
                            | UART_CLKCONF_SCLK_EN
                            | UART_CLKCONF_TX_SCLK_EN
                            | UART_CLKCONF_RX_SCLK_EN;
    }

    /* 4. Baud — write CLKDIV.  Atomic register write, doesn't disturb the
     *    TX state machine (the divider just changes how fast the next
     *    bit transitions; if a byte is in mid-transmit the worst case
     *    is one mis-timed bit, not a wedged shifter). */
    Esp32c3UartSetBaud(base, (uint32_t)pCfg->Rate);

    /* 5. Frame format: data bits, parity, stop.  Read-modify-write
     *    preserves CONF0.mem_clk_en and any other bits ROM/Init set. */
    uint32_t conf0 = UART_CONF0(base) & ~(UART_CONF0_BIT_NUM_Msk | UART_CONF0_STOP_BIT_Msk
                                         | UART_CONF0_PARITY | UART_CONF0_PARITY_EN
                                         | UART_CONF0_TX_FLOW_EN
                                         | UART_CONF0_RXD_INV | UART_CONF0_TXD_INV);

    int bn = pCfg->DataBits;
    if (bn < 5) bn = 5;
    if (bn > 8) bn = 8;
    conf0 |= ((uint32_t)(bn - 5) << UART_CONF0_BIT_NUM_Pos);

    /* STOP_BIT field: 1 = 1 bit, 2 = 1.5, 3 = 2 */
    conf0 |= ((pCfg->StopBits >= 2 ? 3U : 1U) << UART_CONF0_STOP_BIT_Pos);

    switch (pCfg->Parity)
    {
        case UART_PARITY_EVEN:
            conf0 |= UART_CONF0_PARITY_EN;        /* PARITY=0 = even */
            break;
        case UART_PARITY_ODD:
            conf0 |= UART_CONF0_PARITY_EN | UART_CONF0_PARITY;
            break;
        default:
            break;                                /* none */
    }

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
    {
        conf0 |= UART_CONF0_TX_FLOW_EN;
    }

    UART_CONF0(base) = conf0;

    /* 6. CONF1: RX FIFO full threshold + RX timeout.
     *    Affects RX-side interrupts only; safe to write any time. */
    UART_CONF1(base) = (64U << 0)            /* RXFIFO_FULL_THRHD */
                     | (96U << 9)            /* TXFIFO_EMPTY_THRHD */
                     | (1UL << 21);          /* RX_TOUT_EN */

    /* 6a. IDLE_CONF: clear tx_idle_num so there's no inter-byte gap.
     *     C3 default is 0x100 (256 baud cycles ≈ 2.22 ms at 115200),
     *     dropping observed line rate from ~11500 B/s to ~430 B/s.
     *     IDF's uart_hal_init clears this to 0; we do the same. */
    UART_IDLE_CONF(base) = 0U;

    /* 7. UART0: skip REG_UPDATE and FIFO reset.
     *
     * On the C3, ID[30] (HIGH_SPEED) is 1 by default — register writes
     * auto-sync to the core clock domain, so REG_UPDATE is unnecessary
     * (IDF's C3 uart_ll.h does not have a uart_ll_update function at
     * all, unlike C5/C6/H2/P4 where it's required).
     *
     * Also: do NOT pulse RXFIFO_RST/TXFIFO_RST on a running UART0.  The
     * old code's FIFO-reset bracket would clear the FIFO mid-transmit
     * if any of Stage A's bytes were still in flight, contributing to
     * the truncated-AA-stream symptom.  UART0 entered Init() with a
     * running TX shifter; we leave it that way.
     *
     * For UART1 we did the full module reset above, which already
     * initialised the FIFOs — no separate reset needed here either. */

    /* 8. Pin/matrix routing */
    Esp32c3UartCfgPins(devno, (const IOPinCfg_t *)pCfg->pIOPinMap, pCfg->NbIOPins,
                       pCfg->FlowControl);

    /* 10. Software FIFOs */
    if (pCfg->pRxMem && pCfg->RxMemSize > 0)
    {
        pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
    }
    else
    {
        pDev->hRxFifo = CFifoInit(dev->RxFifoMem, ESP32C3_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
    }

    if (pCfg->pTxMem && pCfg->TxMemSize > 0)
    {
        pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
    }
    else
    {
        pDev->hTxFifo = CFifoInit(dev->TxFifoMem, ESP32C3_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
    }

    pDev->Rate         = pCfg->Rate;
    pDev->DataBits     = pCfg->DataBits;
    pDev->Parity       = pCfg->Parity;
    pDev->StopBits     = pCfg->StopBits;
    pDev->FlowControl  = pCfg->FlowControl;
    pDev->Mode         = pCfg->Mode;
    pDev->Duplex       = pCfg->Duplex;
    pDev->EvtCallback  = pCfg->EvtCallback;
    pDev->bRxReady     = false;
    pDev->bTxReady     = true;
    pDev->RxOvrErrCnt  = 0;
    pDev->ParErrCnt    = 0;
    pDev->FramErrCnt   = 0;
    pDev->RxDropCnt    = 0;
    pDev->TxDropCnt    = 0;

    pDev->DevIntrf.Type     = DEVINTRF_TYPE_UART;
    pDev->DevIntrf.bIntEn   = pCfg->bIntMode;
    pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
    pDev->DevIntrf.EnCnt    = 1;
    pDev->DevIntrf.Disable  = Esp32c3UartDisable;
    pDev->DevIntrf.Enable   = Esp32c3UartEnable;
    pDev->DevIntrf.PowerOff = Esp32c3UartPowerOff;
    pDev->DevIntrf.Reset    = Esp32c3UartReset;
    pDev->DevIntrf.GetRate  = Esp32c3UartGetRate;
    pDev->DevIntrf.SetRate  = Esp32c3UartSetRate;
    pDev->DevIntrf.StartRx  = Esp32c3UartStartRx;
    pDev->DevIntrf.RxData   = Esp32c3UartRxData;
    pDev->DevIntrf.StopRx   = Esp32c3UartStopRx;
    pDev->DevIntrf.StartTx  = Esp32c3UartStartTx;
    pDev->DevIntrf.TxData   = Esp32c3UartTxData;
    pDev->DevIntrf.StopTx   = Esp32c3UartStopTx;
    atomic_flag_clear(&pDev->DevIntrf.bBusy);

    /* 11. Optional interrupt routing */
    if (pCfg->bIntMode)
    {
        UART_INT_CLR(base) = 0xFFFFFFFFUL;
        UART_INT_ENA(base) = UART_INT_RXFIFO_FULL | UART_INT_RXFIFO_TOUT
                           | UART_INT_FRM_ERR | UART_INT_PARITY_ERR
                           | UART_INT_RXFIFO_OVF;
        /* TXFIFO_EMPTY is enabled on demand by Esp32c3UartTxData. */

        Esp32c3UartIntMtxEnable(devno == 0 ? INTSRC_UART0 : INTSRC_UART1);
    }

    return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
    (void)pDev;
    (void)LineState;
}
