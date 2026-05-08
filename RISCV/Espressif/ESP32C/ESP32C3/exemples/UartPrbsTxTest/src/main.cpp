/**-------------------------------------------------------------------------
@file	main.cpp  (UartPrbsTxTest — three-stage bisect)

@brief	Three-stage UART validation that pinpoints exactly where things break.

	Stage A: BEFORE Init — write 16 × 0xAA directly to UART0 FIFO at
	         0x60000000, polling TXFIFO_CNT in STATUS @ 0x6000001C.
	         Tests UART core + ROM-supplied state.

	Stage B: AFTER Init — write 16 × 0xCC directly to the SAME UART0
	         FIFO, same polling.  This bypasses the IOsonata driver
	         dispatch entirely and tells us whether Init left the
	         hardware in a state where direct register writes still
	         transmit.  If Stage B works but driver Tx doesn't, the
	         bug is in the driver dispatch (vtable, pDevData,
	         class plumbing).  If Stage B also fails, the bug is in
	         Init's register sequence wedging the core.

	Stage C: driver path — single g_Uart.Tx(0x55) once, then a tight
	         loop.  This is what we actually care about.

	WS2812 (GPIO 8) stages, 500 ms each so they're clearly visible:
	    RED       reached main()
	    YELLOW    Stage A (pre-Init  direct poke) PASSED — 16 × AA out
	    MAGENTA   Stage A FIFO never drained (UART core dead from boot)
	    ORANGE    Driver Init() returned false
	    GREEN     Init() returned true
	    CYAN      Stage B (post-Init direct poke) PASSED — 16 × CC out
	    PINK      Stage B FIFO never drained (Init wedged the core)
	    BLUE      driver Tx of single 0x55 returned cnt = 1
	    WHITE     in driver Tx loop (success)

	Expected wire sequence on success:
	    16 × AA  →  16 × CC  →  one 0x55  →  continuous 0x55 stream

@author	Hoang Nguyen Hoan
@date	May 2026
@license MIT
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"

#include "board.h"

/*=====================================================================
 * WS2812 stage indicator on GPIO 8.
 *====================================================================*/
#define GPIO_OUT_W1TS_REG       (*(volatile uint32_t *)(0x60004008UL))
#define GPIO_OUT_W1TC_REG       (*(volatile uint32_t *)(0x6000400CUL))

#define WS2812_PIN              8U
#define WS2812_MASK             (1UL << WS2812_PIN)

#define WS_NOP_T0H()  __asm volatile (".rept 28 \n nop \n .endr")
#define WS_NOP_T0L()  __asm volatile (".rept 56 \n nop \n .endr")
#define WS_NOP_T1H()  __asm volatile (".rept 56 \n nop \n .endr")
#define WS_NOP_T1L()  __asm volatile (".rept 24 \n nop \n .endr")

#define WS_SEND_BIT(cond)                       \
    do {                                        \
        if (cond) {                             \
            GPIO_OUT_W1TS_REG = WS2812_MASK;    \
            WS_NOP_T1H();                       \
            GPIO_OUT_W1TC_REG = WS2812_MASK;    \
            WS_NOP_T1L();                       \
        } else {                                \
            GPIO_OUT_W1TS_REG = WS2812_MASK;    \
            WS_NOP_T0H();                       \
            GPIO_OUT_W1TC_REG = WS2812_MASK;    \
            WS_NOP_T0L();                       \
        }                                       \
    } while (0)

static void ws2812_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t bits = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    for (int i = 23; i >= 0; i--)
    {
        WS_SEND_BIT(bits & (1UL << i));
    }
    GPIO_OUT_W1TC_REG = WS2812_MASK;
}

static void hang_with_color(uint8_t r, uint8_t g, uint8_t b)
{
    while (1)
    {
        ws2812_set(r, g, b);
        msDelay(500);
        ws2812_set(0, 0, 0);
        msDelay(500);
    }
}

/*=====================================================================
 * Direct register poke helper.  Writes `count` bytes of `value` to the
 * UART0 FIFO at 0x60000000, polling TXFIFO_CNT in STATUS to wait for
 * FIFO space and then for full drain.  Returns true on success, false
 * if either the inner spin or the drain spin times out (FIFO never
 * drained → UART core not transmitting).
 *====================================================================*/
static bool direct_poke(uint8_t value, int count)
{
    volatile uint32_t *FIFO   = (volatile uint32_t *)0x60000000UL;
    volatile uint32_t *STATUS = (volatile uint32_t *)0x6000001CUL;

    for (int i = 0; i < count; i++)
    {
        uint32_t guard = 0;
        while ((((*STATUS) >> 16) & 0x3FFU) >= 128U)
        {
            if (++guard >= 1000000U) return false;
        }
        *FIFO = (uint32_t)value;
    }

    /* Drain check */
    uint32_t guard = 0;
    while ((((*STATUS) >> 16) & 0x3FFU) > 0U)
    {
        if (++guard >= 5000000U) return false;
    }
    return true;
}

/*=====================================================================
 * UART configuration — polled mode (bIntMode = false)
 *====================================================================*/
static const IOPinCfg_t s_UartPins[] = UART_PINS;

static const UARTCfg_t s_UartCfg = {
    .DevNo          = UART_DEVNO,
    .pIOPinMap      = s_UartPins,
    .NbIOPins       = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
    .Rate           = 115200,
    .DataBits       = 8,
    .Parity         = UART_PARITY_NONE,
    .StopBits       = 1,
    .FlowControl    = UART_FLOWCTRL,
    .bIntMode       = false,
    .IntPrio        = 1,
    .EvtCallback    = nullptr,
    .bFifoBlocking  = true,
    .RxMemSize      = 0,
    .pRxMem         = nullptr,
    .TxMemSize      = 0,
    .pTxMem         = nullptr,
    .bDMAMode       = false,
    .bIrDAMode      = false,
    .bIrDAInvert    = false,
    .bIrDAFixPulse  = false,
    .IrDAPulseDiv   = 0,
    .Duplex         = UART_DUPLEX_FULL,
    .Mode           = UART_MODE_UART,
};

UART g_Uart;

/*=====================================================================
 * main
 *====================================================================*/
extern "C" int main(void)
{
    /* WS2812 indicator pin */
    IOPinConfig(0, WS2812_PIN, 0,
                IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    GPIO_OUT_W1TC_REG = WS2812_MASK;
    msDelay(2);
    ws2812_set(0, 0, 0);
    msDelay(1);

    /*--- Stage 1: RED — entered main ------------------------------*/
    ws2812_set(32, 0, 0);
    msDelay(500);

    /*--- Stage A: pre-Init direct poke, 16 × 0xAA ----------------*/
    if (!direct_poke(0xAAU, 16))
    {
        hang_with_color(32, 0, 32);     /* MAGENTA: pre-Init poke failed */
    }

    /*--- Stage 2: YELLOW — Stage A passed ------------------------*/
    ws2812_set(32, 32, 0);
    msDelay(500);
    ws2812_set(0, 0, 0);
    msDelay(100);

    /*--- Init the IOsonata driver --------------------------------*/
    if (!g_Uart.Init(s_UartCfg))
    {
        hang_with_color(32, 16, 0);     /* ORANGE: Init returned false */
    }

    /*--- Stage 3: GREEN — Init OK --------------------------------*/
    ws2812_set(0, 32, 0);
    msDelay(500);
    ws2812_set(0, 0, 0);
    msDelay(100);

    /*--- Stage B: POST-Init direct poke, 16 × 0xCC ---------------*
     * This bypasses the driver entirely and writes directly to the
     * same UART0 FIFO Stage A used.  If this succeeds, the UART
     * core is still alive after Init — meaning the issue (if any)
     * is in the driver dispatch, NOT the hardware setup.            */
    if (!direct_poke(0xCCU, 16))
    {
        hang_with_color(32, 16, 16);    /* PINK: post-Init poke failed */
    }

    /*--- Stage 4: CYAN — Stage B passed --------------------------*/
    ws2812_set(0, 32, 32);
    msDelay(500);
    ws2812_set(0, 0, 0);
    msDelay(100);

    /*--- Stage C-1: single driver Tx of 0x55 ---------------------*/
    uint8_t test_byte = 0x55;
    int n = g_Uart.Tx(&test_byte, 1);
    if (n != 1)
    {
        hang_with_color(32, 0, 32);     /* MAGENTA: driver Tx returned wrong count */
    }

    /*--- Stage 5: BLUE — single driver Tx returned cnt=1 ---------*/
    ws2812_set(0, 0, 32);
    msDelay(500);

    /*--- Stage 6: WHITE — entering tight Tx loop -----------------*/
    ws2812_set(32, 32, 32);

    while (1)
    {
        uint8_t b = 0x55;
        (void)g_Uart.Tx(&b, 1);
    }
}
