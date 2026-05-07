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
With SCLK_DIV_NUM = 1 the UART core clock is XTAL = 40 MHz directly.

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
#define UART_CONF0_RST_CORE         (1UL << 23)

/* STATUS_REG fields */
#define UART_STATUS_RXFIFO_CNT_Pos  0
#define UART_STATUS_RXFIFO_CNT_Msk  (0x3FFUL << UART_STATUS_RXFIFO_CNT_Pos)
#define UART_STATUS_TXFIFO_CNT_Pos  16
#define UART_STATUS_TXFIFO_CNT_Msk  (0x3FFUL << UART_STATUS_TXFIFO_CNT_Pos)

/* CLK_CONF fields */
#define UART_CLKCONF_SCLK_DIV_NUM_Pos   12
#define UART_CLKCONF_SCLK_DIV_NUM_Msk   (0xFFUL << UART_CLKCONF_SCLK_DIV_NUM_Pos)
#define UART_CLKCONF_SCLK_SEL_Pos       20
#define UART_CLKCONF_SCLK_SEL_Msk       (3UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_SEL_APB       (1UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_SEL_RC8M      (2UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_SEL_XTAL      (3UL << UART_CLKCONF_SCLK_SEL_Pos)
#define UART_CLKCONF_SCLK_EN            (1UL << 22)
#define UART_CLKCONF_TX_SCLK_EN         (1UL << 24)
#define UART_CLKCONF_RX_SCLK_EN         (1UL << 25)
#define UART_CLKCONF_TX_RST_CORE        (1UL << 26)
#define UART_CLKCONF_RX_RST_CORE        (1UL << 27)

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
    /* Source = XTAL / SCLK_DIV_NUM (we use 1) = 40 MHz */
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

    /* Enable RISC-V machine external interrupt + global mstatus.MIE.
     * mie.MEIE = bit 11 = 0x800.  mstatus.MIE = bit 3.   */
    __asm volatile (
        "li   t0, 0x800\n\t"
        "csrs mie, t0\n\t"
        "csrsi mstatus, 8\n\t"
        ::: "t0", "memory"
    );
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
 * Pads are driven through the GPIO matrix (works for any GPIO 0..21 on the
 * C3, modulo strapping pins).  IO_MUX is left in MCU_SEL = GPIO mode by the
 * IOPinCfg() call; routing is done here.
 *---------------------------------------------------------------------------*/
static void Esp32c3UartCfgPins(int DevNo, const IOPinCfg_t *pCfg, int NbPins,
                               UART_FLWCTRL FlowCtrl)
{
    /* First put pads into GPIO mode with appropriate direction & pulls. */
    IOPinCfg(pCfg, NbPins);

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

    /* RX: route the chosen GPIO into the UART RXD signal via the matrix. */
    GPIO_FUNC_IN_SEL_REG(rxd_sig) = ((uint32_t)rx_pin & 0x1FU) | GPIO_SIG_IN_SEL;

    /* TX: route the UART TXD signal out through the chosen pin's output mux. */
    GPIO_FUNC_OUT_SEL_REG(tx_pin) = txd_sig & 0xFFUL;

    if (FlowCtrl == UART_FLWCTRL_HW && NbPins >= 4)
    {
        int cts_pin = pCfg[UARTPIN_CTS_IDX].PinNo;
        int rts_pin = pCfg[UARTPIN_RTS_IDX].PinNo;
        GPIO_FUNC_IN_SEL_REG(cts_sig) = ((uint32_t)cts_pin & 0x1FU) | GPIO_SIG_IN_SEL;
        GPIO_FUNC_OUT_SEL_REG(rts_pin) = rts_sig & 0xFFUL;
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

    /* 1. Enable peripheral clock + release peripheral reset */
    if (devno == 0)
    {
        SYSTEM_PERIP_CLK_EN0_REG |= SYSTEM_UART_CLK_EN;
        SYSTEM_PERIP_RST_EN0_REG |=  SYSTEM_UART_RST;
        SYSTEM_PERIP_RST_EN0_REG &= ~SYSTEM_UART_RST;
    }
    else
    {
        SYSTEM_PERIP_CLK_EN0_REG |= SYSTEM_UART1_CLK_EN;
        SYSTEM_PERIP_RST_EN0_REG |=  SYSTEM_UART1_RST;
        SYSTEM_PERIP_RST_EN0_REG &= ~SYSTEM_UART1_RST;
    }

    /* 2. Mask all UART interrupts while we configure */
    UART_INT_ENA(base) = 0U;
    UART_INT_CLR(base) = 0xFFFFFFFFUL;

    /* 3. Clock source = XTAL @ 40 MHz, divider 1, both TX/RX clocks on */
    UART_CLK_CONF(base) = UART_CLKCONF_SCLK_SEL_XTAL
                        | (1UL << UART_CLKCONF_SCLK_DIV_NUM_Pos)
                        | UART_CLKCONF_SCLK_EN
                        | UART_CLKCONF_TX_SCLK_EN
                        | UART_CLKCONF_RX_SCLK_EN;

    /* 4. Baud */
    Esp32c3UartSetBaud(base, (uint32_t)pCfg->Rate);

    /* 5. Frame format: data bits, parity, stop */
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

    /* 6. CONF1: RX FIFO full threshold + RX timeout
     * RXFIFO_FULL_THRHD = 64 → wake up at half the 128-byte HW FIFO. */
    UART_CONF1(base) = (64U << 0)            /* RXFIFO_FULL_THRHD */
                     | (96U << 9)            /* TXFIFO_EMPTY_THRHD */
                     | (1UL << 21);          /* RX_TOUT_EN */

    /* 7. Latch CONF/CLKDIV into the UART core clock domain */
    Esp32c3UartRegUpdate(base);

    /* 8. Reset FIFOs (after REG_UPDATE so they reset against the new config) */
    Esp32c3UartFifoReset(base);

    /* 9. Pin/matrix routing */
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
