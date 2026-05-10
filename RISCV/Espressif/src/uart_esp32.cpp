/**-------------------------------------------------------------------------
@file	uart_esp32.cpp

@brief	IOsonata UART driver for ESP32 RISC-V (UART0, UART1)

Cross-chip driver supporting ESP32-C3, ESP32-C5, and ESP32-C6 in a
single source file.  Per-chip differences (interrupt controller
architecture, peripheral clock-gating, default IOMUX pins, UART
register layout) are isolated to small `#if defined(...)` blocks; the
data plane (interrupt handler, FIFO drain, polled and interrupt-driven
Rx/Tx paths) is identical on all three.

Register access goes through the cross-chip macros in
esp32xx_uart.h (UART), esp32xx_intmtx.h / esp32xx_clic.h (interrupt
controllers), esp32xx_irq.h (cross-chip IRQ install helper),
esp32xx_pcr.h (C5/C6 peripheral clock-gating) and esp32xx_gpio.h
(IOMUX and GPIO matrix).  The C3-vs-C5/C6 UART layout differences
(CLK_CONF moved 0x78 -> 0x88, separate REG_UPDATE register on C5/C6,
etc.) are hidden by those headers; this file uses the same names
regardless of target.

C3 register layout reference: ESP32-C3 TRM Rev 0.4, section 17.
C5/C6 register layout reference: ESP-IDF master / v5.3 register
headers (uart_reg.h, pcr_reg.h, gpio_sig_map.h, uart_pins.h).

UART instance bases:
  C3:    UART0 0x60000000, UART1 0x60010000  (64 KB stride)
  C5/C6: UART0 0x60000000, UART1 0x60001000  (4 KB stride)

Clock source: SCLK_SEL = 3 (XTAL, 40 MHz) keeps baud independent of any
later CPU/PLL change.  With SCLK_DIV_NUM = 0 the UART core runs at
XTAL = 40 MHz directly (formula: SCLK = source / (SCLK_DIV_NUM + 1)).

Baud divider: baud = 40 MHz / (CLKDIV + CLKDIV_FRAG / 16)
  e.g. 115200 baud -> CLKDIV = 347, CLKDIV_FRAG = 4 -> 115273 baud
  (+0.06 %).

Pin routing: TX writes the UART TXD signal index to GPIO_FUNC_OUT_SEL
on the chosen TX pin.  RX writes the RX pin index to GPIO_FUNC_IN_SEL
of the RXD signal.  IOPinConfig() puts both pads into MCU_SEL = GPIO
mode beforehand so the matrix is in charge.

Interrupt routing: matrix source IDs come from esp32xx_uart.h
(ESP32_UART_INTR_SOURCE_UART0/1).  Each peripheral source is routed to
one CPU INT input and the ISR is installed directly on that input.
This driver uses CPU INT 2 for UART0 and CPU INT 3 for UART1, avoiding
the ESP32-C3 reserved CPU INT 1 and the permanently disabled CPU INT 6.

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

#include "esp32xx.h"
#include "esp32xx_gpio.h"
#include "esp32xx_intmtx.h"
#include "esp32xx_irq.h"
#include "esp32xx_uart.h"

// Signal indices for U0RXD/U0TXD/U0CTS/U0RTS and U1 equivalents are
// defined in esp32xx_uart.h (ESP32_U0RXD_IN_IDX etc.) for all three
// chips.

// ===========================================================================
// External IRQ-count counters (provided elsewhere -- Vectors file or test
// harness -- for cross-module inspection).
// ===========================================================================
extern "C" volatile uint32_t g_Esp32C3Uart0IrqCount;
extern "C" volatile uint32_t g_Esp32C3Uart1IrqCount;

// ===========================================================================
// Local CPU-INT level assignment.  CPU INT 1 is reserved by IDF for Wi-Fi
// and CPU INT 6 is the permanently disabled input on C3.  Use free level-1
// inputs.  UART_CPU_INT_PRIO = 1..15, must be > THRESH (which we set to 0).
// ===========================================================================
#define UART0_CPU_INT_LEVEL             2U
#define UART1_CPU_INT_LEVEL             3U
#define UART_CPU_INT_PRIO               1U

/*---------------------------------------------------------------------------
 * Per-instance driver data
 *---------------------------------------------------------------------------*/
#define ESP32_UART_CFIFO_SIZE         CFIFO_MEMSIZE(64)

#pragma pack(push, 4)
typedef struct {
    int          DevNo;
    uint32_t     Base;
    UARTDev_t   *pUartDev;
    uint8_t      RxFifoMem[ESP32_UART_CFIFO_SIZE];
    uint8_t      TxFifoMem[ESP32_UART_CFIFO_SIZE];
} ESP32_UARTDEV;
#pragma pack(pop)

static ESP32_UARTDEV s_Esp32UartDev[] = {
    { .DevNo = 0, .Base = ESP32_UART0_BASE, .pUartDev = nullptr },
    { .DevNo = 1, .Base = ESP32_UART1_BASE, .pUartDev = nullptr },
};
static const int s_NbUartDev = sizeof(s_Esp32UartDev) / sizeof(s_Esp32UartDev[0]);

extern "C" {
volatile uint32_t g_Esp32C3UartFix9BuildMarker = 0x26050809UL;
const char g_Esp32C3UartFix9BuildString[] = "ESP32C3_UART_IRQ_FIX9_SRAM1_VECTOR_260508";
volatile uint32_t g_Esp32C3UartDbgUart0Map;
volatile uint32_t g_Esp32C3UartDbgUart1Map;
volatile uint32_t g_Esp32C3UartDbgCpuEnable;
volatile uint32_t g_Esp32C3UartDbgCpuType;
volatile uint32_t g_Esp32C3UartDbgCpuEip;
volatile uint32_t g_Esp32C3UartDbgCpuPri2;
volatile uint32_t g_Esp32C3UartDbgCpuPri3;
volatile uint32_t g_Esp32C3UartDbgCpuThresh;
volatile uint32_t g_Esp32C3UartDbgSrcStatus0;
volatile uint32_t g_Esp32C3UartDbgSrcStatus1;
volatile uint32_t g_Esp32C3UartDbgUart0Raw;
volatile uint32_t g_Esp32C3UartDbgUart0Ena;
volatile uint32_t g_Esp32C3UartDbgUart0St;
volatile uint32_t g_Esp32C3UartDbgUart1Raw;
volatile uint32_t g_Esp32C3UartDbgUart1Ena;
volatile uint32_t g_Esp32C3UartDbgUart1St;

void Esp32UartDebugSnapshot(void)
{
#if defined(ESP32C3) || defined(ESP32C6)
    g_Esp32C3UartDbgUart0Map   = ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART0);
    g_Esp32C3UartDbgUart1Map   = ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART1);
    g_Esp32C3UartDbgCpuEnable  = ESP32_CPU_INT_ENABLE_REG;
    g_Esp32C3UartDbgCpuType    = ESP32_CPU_INT_TYPE_REG;
    g_Esp32C3UartDbgCpuEip     = ESP32_CPU_INT_EIP_STATUS_REG;
    g_Esp32C3UartDbgCpuPri2    = ESP32_CPU_INT_PRI_REG(UART0_CPU_INT_LEVEL);
    g_Esp32C3UartDbgCpuPri3    = ESP32_CPU_INT_PRI_REG(UART1_CPU_INT_LEVEL);
    g_Esp32C3UartDbgCpuThresh  = ESP32_CPU_INT_THRESH_REG;
    g_Esp32C3UartDbgSrcStatus0 = ESP32_INTMTX_INTR_STATUS_0_REG;
    g_Esp32C3UartDbgSrcStatus1 = ESP32_INTMTX_INTR_STATUS_1_REG;
    g_Esp32C3UartDbgUart0Raw   = ESP32_UART_INT_RAW_REG(ESP32_UART0_BASE);
    g_Esp32C3UartDbgUart0Ena   = ESP32_UART_INT_ENA_REG(ESP32_UART0_BASE);
    g_Esp32C3UartDbgUart0St    = ESP32_UART_INT_ST_REG(ESP32_UART0_BASE);
    g_Esp32C3UartDbgUart1Raw   = ESP32_UART_INT_RAW_REG(ESP32_UART1_BASE);
    g_Esp32C3UartDbgUart1Ena   = ESP32_UART_INT_ENA_REG(ESP32_UART1_BASE);
    g_Esp32C3UartDbgUart1St    = ESP32_UART_INT_ST_REG(ESP32_UART1_BASE);
#endif
}
}

UARTDev_t const * const UARTGetInstance(int DevNo)
{
    if (DevNo < 0 || DevNo >= s_NbUartDev) return nullptr;
    return s_Esp32UartDev[DevNo].pUartDev;
}

/*---------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/
static inline uint32_t Esp32UartTxFree(uint32_t base)
{
    return ESP32_UART_HW_FIFO_LEN -
           ((ESP32_UART_STATUS_REG(base) & ESP32_UART_STATUS_TXFIFO_CNT_Msk) >> ESP32_UART_STATUS_TXFIFO_CNT_Pos);
}

static inline uint32_t Esp32UartRxAvail(uint32_t base)
{
    return (ESP32_UART_STATUS_REG(base) & ESP32_UART_STATUS_RXFIFO_CNT_Msk) >> ESP32_UART_STATUS_RXFIFO_CNT_Pos;
}

static inline void Esp32UartRegUpdate(uint32_t base)
{
#if defined(ESP32C3)
    // C3: ID[31] is the REG_UPDATE / commit bit.  Hardware self-clears
    // once the new config has been synchronised into the UART core clock
    // domain.  Spin briefly.  Note that on C3 ID[30] (HIGH_SPEED) defaults
    // to 1, which makes most CONF0/CLKDIV writes auto-sync without this
    // round trip; we still invoke it for correctness.
    ESP32_UART_ID_REG(base) |= ESP32_UART_ID_REG_UPDATE_Msk;
    while (ESP32_UART_ID_REG(base) & ESP32_UART_ID_REG_UPDATE_Msk) {}
#elif defined(ESP32C5) || defined(ESP32C6)
    // C5/C6: separate REG_UPDATE register at offset 0x98.  Bit 0 is the
    // commit bit; hardware self-clears.
    ESP32_UART_REG_UPDATE_REG(base) = 1U;
    while (ESP32_UART_REG_UPDATE_REG(base) & 1U) {}
#endif
}

static void Esp32UartFifoReset(uint32_t base)
{
    // Pulse RXFIFO_RST + TXFIFO_RST in CONF0.  Spec wants a write of 1
    // followed by a write of 0; both must be latched via REG_UPDATE.
    uint32_t c = ESP32_UART_CONF0_REG(base);
    ESP32_UART_CONF0_REG(base) = c | ESP32_UART_CONF0_RXFIFO_RST_Msk | ESP32_UART_CONF0_TXFIFO_RST_Msk;
    Esp32UartRegUpdate(base);
    ESP32_UART_CONF0_REG(base) = c & ~(ESP32_UART_CONF0_RXFIFO_RST_Msk | ESP32_UART_CONF0_TXFIFO_RST_Msk);
    Esp32UartRegUpdate(base);
}

static void Esp32UartSetBaud(uint32_t base, uint32_t baud)
{
    // Source = SCLK = XTAL / (SCLK_DIV_NUM + 1).  We program
    // SCLK_DIV_NUM = 0 in UARTInit, so SCLK = XTAL = 40 MHz.
    uint32_t src_hz = ESP32_UART_XTAL_HZ;

    // CLKDIV is 12.4 fixed-point: divisor = src/baud, scaled by 16
    uint32_t scaled = (uint32_t)(((uint64_t)src_hz * 16ULL + (baud / 2U)) / baud);
    uint32_t intpart = scaled >> 4;
    uint32_t frag    = scaled & 0xFU;

    if (intpart < 1U) intpart = 1U;
    if (intpart > ESP32_UART_CLKDIV_INT_Msk) intpart = ESP32_UART_CLKDIV_INT_Msk;

    ESP32_UART_CLKDIV_REG(base) = (intpart & ESP32_UART_CLKDIV_INT_Msk)
                      | ((frag << ESP32_UART_CLKDIV_FRAG_Pos) & ESP32_UART_CLKDIV_FRAG_Msk);
}

static void Esp32UartIntMtxEnable(uint32_t source, uint32_t cpu_int_id, Esp32IrqHandler handler)
{
    Esp32EnableSourceIrq(source, cpu_int_id, UART_CPU_INT_PRIO, handler);
#if defined(ESP32C3) || defined(ESP32C6)
    Esp32UartDebugSnapshot();
#endif
}

/*---------------------------------------------------------------------------
 * IRQ handler - invoked from the ESP32-C3 CPU interrupt input
 *---------------------------------------------------------------------------*/
static void Esp32UartIRQHandler(ESP32_UARTDEV *pDev)
{
    uint32_t base = pDev->Base;
    UARTDev_t *udev = pDev->pUartDev;
    uint32_t st = ESP32_UART_INT_ST_REG(base);

    // Clear handled flags.  RXFIFO_FULL / RXFIFO_TOUT auto-clear by reading
    // the FIFO; the rest are edge-style and need an explicit ICR write.
    if (st & (ESP32_UART_INT_FRM_ERR | ESP32_UART_INT_PARITY_ERR | ESP32_UART_INT_RXFIFO_OVF | ESP32_UART_INT_GLITCH_DET))
    {
        ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_FRM_ERR | ESP32_UART_INT_PARITY_ERR
                           | ESP32_UART_INT_RXFIFO_OVF | ESP32_UART_INT_GLITCH_DET;
    }

    if (st & (ESP32_UART_INT_RXFIFO_FULL | ESP32_UART_INT_RXFIFO_TOUT))
    {
        while (Esp32UartRxAvail(base) > 0U)
        {
            uint8_t c = (uint8_t)ESP32_UART_FIFO_REG(base);
            uint8_t *p = CFifoPut(udev->hRxFifo);
            if (p != nullptr) *p = c;
            else udev->RxDropCnt++;
        }
        ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_RXFIFO_FULL | ESP32_UART_INT_RXFIFO_TOUT;
        udev->bRxReady = true;

        if (udev->EvtCallback)
        {
            udev->EvtCallback(udev, UART_EVT_RXDATA, nullptr, CFifoUsed(udev->hRxFifo));
        }
    }

    if (st & ESP32_UART_INT_TXFIFO_EMPTY)
    {
        // Drain SW FIFO into HW FIFO until either runs out.
        while (Esp32UartTxFree(base) > 0U)
        {
            uint8_t *p = CFifoGet(udev->hTxFifo);
            if (p == nullptr) break;
            ESP32_UART_FIFO_REG(base) = *p;
        }
        if (CFifoUsed(udev->hTxFifo) == 0)
        {
            // No more bytes queued: mask TXFIFO_EMPTY so we don't storm.
            ESP32_UART_INT_ENA_REG(base) &= ~ESP32_UART_INT_TXFIFO_EMPTY;
            udev->bTxReady = true;
            if (udev->EvtCallback)
            {
                udev->EvtCallback(udev, UART_EVT_TXREADY, nullptr, 0);
            }
        }
        ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_TXFIFO_EMPTY;
    }
}

/*---------------------------------------------------------------------------
 * IRQ entry points.  These override the weak `alias("DEF_IRQHandler")`
 * declarations in Vectors_esp32c3.c.
 *
 * `__attribute__((used))` keeps the compiler from discarding the symbol
 * even if it appears statically unreachable in the current TU; combined
 * with the strong definition this guarantees that the linker resolves
 * the [21]/[22] entries in s_Esp32C3PeriphVectors[] to OUR functions
 * rather than the weak DEF_IRQHandler infinite loop, regardless of
 * archive member ordering.
 *
 *---------------------------------------------------------------------------*/
extern "C" __attribute__((used))
void UART0_IRQHandler(void)
{
    g_Esp32C3Uart0IrqCount++;
    Esp32UartIRQHandler(&s_Esp32UartDev[0]);
}

extern "C" __attribute__((used))
void UART1_IRQHandler(void)
{
    g_Esp32C3Uart1IrqCount++;
    Esp32UartIRQHandler(&s_Esp32UartDev[1]);
}

/*---------------------------------------------------------------------------
 * DevIntrf operations
 *---------------------------------------------------------------------------*/
static uint32_t Esp32UartGetRate(DevIntrf_t * const pDev)
{
    return ((ESP32_UARTDEV *)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t Esp32UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
    ESP32_UARTDEV *dev = (ESP32_UARTDEV *)pDev->pDevData;
    Esp32UartSetBaud(dev->Base, Rate);
    Esp32UartRegUpdate(dev->Base);
    dev->pUartDev->Rate = Rate;
    return Rate;
}

static void Esp32UartEnable(DevIntrf_t * const pDev) { (void)pDev; }
static void Esp32UartDisable(DevIntrf_t * const pDev) { (void)pDev; }
static void Esp32UartPowerOff(DevIntrf_t * const pDev) { (void)pDev; }
static void Esp32UartReset(DevIntrf_t * const pDev) { (void)pDev; }

static bool Esp32UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr) { (void)pDev; (void)DevAddr; return true; }
static void Esp32UartStopRx(DevIntrf_t * const pDev) { (void)pDev; }
static bool Esp32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr) { (void)pDev; (void)DevAddr; return true; }
static void Esp32UartStopTx(DevIntrf_t * const pDev) { (void)pDev; }

static int Esp32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
    ESP32_UARTDEV *dev = (ESP32_UARTDEV *)pDev->pDevData;
    UARTDev_t *udev = dev->pUartDev;
    int cnt = 0;

    if (pDev->bIntEn)
    {
        // Drain SW CFifo (filled from HW FIFO by ISR)
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
        // Polled: drain HW FIFO directly
        while (BuffLen > 0 && Esp32UartRxAvail(dev->Base) > 0U)
        {
            *pBuff++ = (uint8_t)ESP32_UART_FIFO_REG(dev->Base);
            BuffLen--;
            cnt++;
        }
    }

    return cnt;
}

static int Esp32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
    ESP32_UARTDEV *dev = (ESP32_UARTDEV *)pDev->pDevData;
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

            // Kick the ISR pump if idle.  We arm TXFIFO_EMPTY and let the
            // ISR drain SW -> HW.  We seed at least one byte directly so
            // TXFIFO_EMPTY edge fires reliably even if all we queued fits
            // in the HW FIFO right now.
            if (udev->bTxReady && CFifoUsed(udev->hTxFifo) > 0)
            {
                udev->bTxReady = false;
                while (Esp32UartTxFree(dev->Base) > 0U)
                {
                    uint8_t *p = CFifoGet(udev->hTxFifo);
                    if (p == nullptr) break;
                    ESP32_UART_FIFO_REG(dev->Base) = *p;
                }
                ESP32_UART_INT_CLR_REG(dev->Base) = ESP32_UART_INT_TXFIFO_EMPTY;
                ESP32_UART_INT_ENA_REG(dev->Base) |= ESP32_UART_INT_TXFIFO_EMPTY;
            }

            EnableInterrupt(state);

            // If SW FIFO is full there's nothing useful to retry on; bail.
            if (DataLen > 0 && CFifoAvail(udev->hTxFifo) == 0)
            {
                udev->TxDropCnt += DataLen;
                break;
            }
        }
    }
    else
    {
        // Polled: spin on HW TX FIFO free space.  Safe but blocking.
        while (DataLen > 0)
        {
            while (Esp32UartTxFree(dev->Base) == 0U) {}
            ESP32_UART_FIFO_REG(dev->Base) = *pData++;
            DataLen--;
            cnt++;
        }
    }

    return cnt;
}

#if 0
/*---------------------------------------------------------------------------
 * Pin configuration
 *
 * Pad-level setup (direction, pull, type) is done by IOPinCfg() in
 * iopincfg_esp32.c.  Matrix routing of UART signals to those pads is
 * done by ESP32GPIOConnectInput() / ESP32GPIOConnectOutput(), also in
 * iopincfg_esp32.c.  The UART driver itself does no IOMUX or matrix
 * register writes.
 *
 * Signal indices come from esp32xx_uart.h (ESP32_U0RXD_IN_IDX etc.) and
 * are identical across C3 / C5 / C6 (verified against IDF gpio_sig_map).
 *---------------------------------------------------------------------------*/
static void Esp32UartConnectSignals(int DevNo, const IOPinCfg_t *pCfg, int NbPins,
                                     UART_FLWCTRL FlowCtrl)
{
    uint32_t rxd_sig, txd_sig, cts_sig, rts_sig;
    if (DevNo == 0)
    {
        rxd_sig = ESP32_U0RXD_IN_IDX;
        txd_sig = ESP32_U0TXD_OUT_IDX;
        cts_sig = ESP32_U0CTS_IN_IDX;
        rts_sig = ESP32_U0RTS_OUT_IDX;
    }
    else
    {
        rxd_sig = ESP32_U1RXD_IN_IDX;
        txd_sig = ESP32_U1TXD_OUT_IDX;
        cts_sig = ESP32_U1CTS_IN_IDX;
        rts_sig = ESP32_U1RTS_OUT_IDX;
    }

    int rx_pin = pCfg[UARTPIN_RX_IDX].PinNo;
    int tx_pin = pCfg[UARTPIN_TX_IDX].PinNo;

    if (tx_pin >= 0) ESP32GPIOConnectOutput(tx_pin, txd_sig, false, false);
    if (rx_pin >= 0) ESP32GPIOConnectInput (rxd_sig, rx_pin, false);

    if (FlowCtrl == UART_FLWCTRL_HW && NbPins >= 4)
    {
        int cts_pin = pCfg[UARTPIN_CTS_IDX].PinNo;
        int rts_pin = pCfg[UARTPIN_RTS_IDX].PinNo;
        if (cts_pin >= 0) ESP32GPIOConnectInput (cts_sig, cts_pin, false);
        if (rts_pin >= 0) ESP32GPIOConnectOutput(rts_pin, rts_sig, false, false);
    }
}
#endif

/*---------------------------------------------------------------------------
 * UARTInit — top-level driver init
 *---------------------------------------------------------------------------*/
bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
    if (pDev == nullptr || pCfg == nullptr) return false;
    if (pCfg->pIOPinMap == nullptr || pCfg->NbIOPins < 2) return false;
    if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev) return false;

    int devno = pCfg->DevNo;
    ESP32_UARTDEV *dev = &s_Esp32UartDev[devno];
    uint32_t base = dev->Base;

    pDev->DevIntrf.pDevData = dev;
    dev->pUartDev = pDev;

    // 1. Enable APB clock to the UART, and reset the peripheral.

    // UART0 IS THE ROM CONSOLE.  ESP-IDF deliberately SKIPS the module
    // reset on whichever UART is the console (esp_driver_uart/src/uart.c:218):

    //     if (uart_num != CONFIG_ESP_CONSOLE_UART_NUM) {
    //         uart_ll_reset_register(uart_num);
    //     }

    // ROM hands UART0 off in a working state — its CLKDIV, CONF0, FIFO,
    // and TX state machine are all sane.  Pulsing SYSTEM_UART_RST against
    // a running core (even with the C3 RST_CORE bracket) interacts badly
    // with the GPIO-matrix pin transition that follows in step 11: the
    // core comes out of reset, briefly accepts FIFO writes, but its TX
    // shifter never re-engages.  Symptom: Tx() pushes bytes into the
    // HW FIFO indefinitely without anything appearing on the wire.

    // Just enable the bus clock (already on for UART0 from ROM, but the
    // write is idempotent) and proceed.  CONF0/CONF1/CLKDIV/CLK_CONF
    // are then explicitly programmed below to known values, so we don't
    // rely on whatever ROM left in those registers — only on the fact
    // that the UART core is alive and clocked.

    // For UART1 the full C3 RST_CORE bracket is still required, since
    // ROM never enabled UART1: without it the core latches garbage out
    // of reset (per IDF v5.3 hal/esp32c3/uart_ll.h:107).
    Esp32UartClockEnable(devno);
    Esp32UartReset(devno);   // no-op for devno == 0 (ROM console)

    // 2. Mask all UART interrupts while we configure (safe regardless of
    //    whether UART is running — these are interrupt-enable bits, not
    //    state machine bits).
    ESP32_UART_INT_ENA_REG(base) = 0U;
    ESP32_UART_INT_CLR_REG(base) = 0xFFFFFFFFUL;

    // 3. Clock source selection.

    // UART0: surgical update.  ROM left CLK_CONF with all the right
    // enables (SCLK_EN, TX_SCLK_EN, RX_SCLK_EN are all 1 by default and
    // ROM keeps them so).  We only want to (a) point the source at XTAL
    // so baud is independent of any later CPU/APB clock change, and
    // (b) zero the integer pre-divider so SCLK = XTAL = 40 MHz exactly.
    // Read-modify-write only those two fields (sclk_sel, sclk_div_num);
    // leaving sclk_div_a, sclk_div_b at 0, and the enable bits and
    // rst_core untouched at whatever they were (always 1, 1, 1, 0 in
    // practice, but we don't disturb them).

    // UART1: full overwrite is fine — the RST_CORE bracket above already
    // reset CLK_CONF to defaults, so this just sets up our chosen state.

    // Why this matters: the previous code did an UNCONDITIONAL full
    // overwrite of CLK_CONF on UART0.  When UART0 came into Init()
    // already running (post-Stage-A or after ROM banner remnants), the
    // full overwrite SWITCHED THE SCLK SOURCE underneath an in-flight
    // TX byte.  The TX shifter wedged at variable points — symptom was
    // a non-deterministic truncated AA stream (sometimes 2 bytes,
    // sometimes 10).  Surgical update doesn't disturb the shifter.
    if (devno == 0)
    {
        uint32_t cc = ESP32_UART_CLK_CONF_REG(base);
        cc &= ~(ESP32_UART_CLK_CONF_SCLK_SEL_Msk | ESP32_UART_CLK_CONF_SCLK_DIV_NUM_Msk
              | ESP32_UART_CLK_CONF_SCLK_DIV_A_Msk | ESP32_UART_CLK_CONF_SCLK_DIV_B_Msk);
        cc |= ESP32_UART_CLK_CONF_SCLK_SEL_XTAL;   // sclk_div_num stays 0 → /1
        ESP32_UART_CLK_CONF_REG(base) = cc;
    }
    else
    {
        ESP32_UART_CLK_CONF_REG(base) = ESP32_UART_CLK_CONF_SCLK_SEL_XTAL
                            | ESP32_UART_CLK_CONF_SCLK_EN_Msk
                            | ESP32_UART_CLK_CONF_TX_SCLK_EN_Msk
                            | ESP32_UART_CLK_CONF_RX_SCLK_EN_Msk;
    }

    // 4. Baud — write CLKDIV.  Atomic register write, doesn't disturb the
    //    TX state machine (the divider just changes how fast the next
    //    bit transitions; if a byte is in mid-transmit the worst case
    //    is one mis-timed bit, not a wedged shifter).
    Esp32UartSetBaud(base, (uint32_t)pCfg->Rate);

    // 5. Frame format: data bits, parity, stop.  Read-modify-write
    //    preserves CONF0.mem_clk_en and any other bits ROM/Init set.
    uint32_t conf0 = ESP32_UART_CONF0_REG(base) & ~(ESP32_UART_CONF0_BIT_NUM_Msk | ESP32_UART_CONF0_STOP_BIT_Msk
                                         | ESP32_UART_CONF0_PARITY_Msk | ESP32_UART_CONF0_PARITY_EN_Msk
                                         | ESP32_UART_CONF0_TX_FLOW_EN_Msk
                                         | ESP32_UART_CONF0_RXD_INV_Msk | ESP32_UART_CONF0_TXD_INV_Msk);

    int bn = pCfg->DataBits;
    if (bn < 5) bn = 5;
    if (bn > 8) bn = 8;
    conf0 |= ((uint32_t)(bn - 5) << ESP32_UART_CONF0_BIT_NUM_Pos);

    // STOP_BIT field: 1 = 1 bit, 2 = 1.5, 3 = 2
    conf0 |= ((pCfg->StopBits >= 2 ? 3U : 1U) << ESP32_UART_CONF0_STOP_BIT_Pos);

    switch (pCfg->Parity)
    {
        case UART_PARITY_EVEN:
            conf0 |= ESP32_UART_CONF0_PARITY_EN_Msk;        // PARITY=0 = even
            break;
        case UART_PARITY_ODD:
            conf0 |= ESP32_UART_CONF0_PARITY_EN_Msk | ESP32_UART_CONF0_PARITY_Msk;
            break;
        default:
            break;                                // none
    }

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
    {
        conf0 |= ESP32_UART_CONF0_TX_FLOW_EN_Msk;
    }

    ESP32_UART_CONF0_REG(base) = conf0;

    // 6. CONF1: RX FIFO full threshold + RX timeout.
    //    Affects RX-side interrupts only; safe to write any time.
    ESP32_UART_CONF1_REG(base) = (64U << 0)            // RXFIFO_FULL_THRHD
                     | (16U << 9)            // TXFIFO_EMPTY_THRHD
                     | (1UL << 21);          // RX_TOUT_EN

    // 6a. IDLE_CONF: clear tx_idle_num so there's no inter-byte gap.
    //     C3 default is 0x100 (256 baud cycles ≈ 2.22 ms at 115200),
    //     dropping observed line rate from ~11500 B/s to ~430 B/s.
    //     IDF's uart_hal_init clears this to 0; we do the same.
    ESP32_UART_IDLE_CONF_REG(base) = 0U;

    // 7. UART0: skip REG_UPDATE and FIFO reset.

    // On the C3, ID[30] (HIGH_SPEED) is 1 by default — register writes
    // auto-sync to the core clock domain, so REG_UPDATE is unnecessary
    // (IDF's C3 uart_ll.h does not have a uart_ll_update function at
    // all, unlike C5/C6/H2/P4 where it's required).

    // Also: do NOT pulse RXFIFO_RST/TXFIFO_RST on a running UART0.  The
    // old code's FIFO-reset bracket would clear the FIFO mid-transmit
    // if any of Stage A's bytes were still in flight, contributing to
    // the truncated-AA-stream symptom.  UART0 entered Init() with a
    // running TX shifter; we leave it that way.

    // For UART1 we did the full module reset above, which already
    // initialised the FIFOs — no separate reset needed here either.

    // 8. Pin/matrix routing
    IOPinCfg((IOPinCfg_t *)pCfg->pIOPinMap, pCfg->NbIOPins);
//    Esp32UartConnectSignals(devno, (const IOPinCfg_t *)pCfg->pIOPinMap,
//                              pCfg->NbIOPins, pCfg->FlowControl);

    // 10. Software FIFOs
    if (pCfg->pRxMem && pCfg->RxMemSize > 0)
    {
        pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
    }
    else
    {
        pDev->hRxFifo = CFifoInit(dev->RxFifoMem, ESP32_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
    }

    if (pCfg->pTxMem && pCfg->TxMemSize > 0)
    {
        pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
    }
    else
    {
        pDev->hTxFifo = CFifoInit(dev->TxFifoMem, ESP32_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
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
    pDev->DevIntrf.Disable  = Esp32UartDisable;
    pDev->DevIntrf.Enable   = Esp32UartEnable;
    pDev->DevIntrf.PowerOff = Esp32UartPowerOff;
    pDev->DevIntrf.Reset    = Esp32UartReset;
    pDev->DevIntrf.GetRate  = Esp32UartGetRate;
    pDev->DevIntrf.SetRate  = Esp32UartSetRate;
    pDev->DevIntrf.StartRx  = Esp32UartStartRx;
    pDev->DevIntrf.RxData   = Esp32UartRxData;
    pDev->DevIntrf.StopRx   = Esp32UartStopRx;
    pDev->DevIntrf.StartTx  = Esp32UartStartTx;
    pDev->DevIntrf.TxData   = Esp32UartTxData;
    pDev->DevIntrf.StopTx   = Esp32UartStopTx;
    atomic_flag_clear(&pDev->DevIntrf.bBusy);

    // 11. Optional interrupt routing
    if (pCfg->bIntMode)
    {
        ESP32_UART_INT_CLR_REG(base) = 0xFFFFFFFFUL;
        ESP32_UART_INT_ENA_REG(base) = ESP32_UART_INT_RXFIFO_FULL | ESP32_UART_INT_RXFIFO_TOUT
                           | ESP32_UART_INT_FRM_ERR | ESP32_UART_INT_PARITY_ERR
                           | ESP32_UART_INT_RXFIFO_OVF;
        // TXFIFO_EMPTY is enabled on demand by Esp32UartTxData.

        if (devno == 0)
        {
            Esp32UartIntMtxEnable(ESP32_UART_INTR_SOURCE_UART0, UART0_CPU_INT_LEVEL, UART0_IRQHandler);
        }
        else
        {
            Esp32UartIntMtxEnable(ESP32_UART_INTR_SOURCE_UART1, UART1_CPU_INT_LEVEL, UART1_IRQHandler);
        }
    }

    return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
    (void)pDev;
    (void)LineState;
}
