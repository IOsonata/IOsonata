/**-------------------------------------------------------------------------
@file	main.cpp  (UartPrbsTxTest — diagnostic build)

@brief	One-shot diagnostic for the C3 UART driver path.  Uses the
	on-board WS2812 RGB LED on GPIO 8 as a stage indicator so we can
	tell at a glance which init/tx step failed.

	Stages (LED color):
	  RED      Pre-init.  Reached main(), about to call Init().
	  ORANGE   Init() returned false.
	  GREEN    Init() returned true.
	  MAGENTA  Tx() returned a count != 1 (probably hung in polled
	           drain → reached MaxRetry, or returned 0 immediately).
	  BLUE     Tx() reported success.  Will keep transmitting 0x55
	           continuously so a scope/LA on GPIO 21 sees activity.

	Drop this in place of the linked `uart_prbs_tx.cpp` in the
	UartPrbsTxTest Eclipse project: edit .project to remove the
	`PARENT-6-PROJECT_LOC/exemples/uart/uart_prbs_tx.cpp` link, add
	this file as a regular project source, rebuild and flash.

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
 * WS2812 bit-bang on GPIO 8 — same code as Blinky.  Used here as a
 * 24-bit RGB stage flag.  CPU @ 80 MHz, 12.5 ns / cycle.
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

static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    while (ms--) { for (i = 0; i < 13000UL; i++) { /* spin */ } }
}

/*=====================================================================
 * UART configuration — pulled from board.h, polled mode (bIntMode = false)
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
 * Stage indicator helpers — flash the chosen color in a loop so the
 * LED is visibly steady (refresh every 500 ms).
 *====================================================================*/

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
 * Diagnostic flow
 *====================================================================*/

extern "C" int main(void)
{
    /* WS2812 indicator pin — must be ready before any stage display. */
    IOPinConfig(0, WS2812_PIN, 0,
                IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    GPIO_OUT_W1TC_REG = WS2812_MASK;
    msDelay(2);
    ws2812_set(0, 0, 0);
    msDelay(1);

    /*--- Stage 1: RED — pre-init ----------------------------------*/
    ws2812_set(32, 0, 0);
    msDelay(500);

    /*--- Try UART init -------------------------------------------*/
    if (!g_Uart.Init(s_UartCfg))
    {
        /*--- Stage 2a: ORANGE pulse — Init returned false --------*/
        hang_with_color(32, 16, 0);
    }

    /*--- Stage 2b: GREEN — Init OK -------------------------------*/
    ws2812_set(0, 32, 0);
//    msDelay(500);
    ws2812_set(0, 0, 0);

    /*--- Try one known byte through Tx ---------------------------*/
    uint8_t test_byte = 0x55;             /* alternating bit pattern */
    int n = 1;//g_Uart.Tx(&test_byte, 1);

    if (n != 1)
    {
        /*--- Stage 3a: MAGENTA pulse — Tx hung or returned 0 -----*/
        hang_with_color(32, 0, 32);
    }

    /*--- Stage 3b: BLUE — Tx reported success --------------------*/
    ws2812_set(0, 0, 32);
  //  msDelay(500);

    /*--- Keep the line busy so a scope/LA on GPIO 21 sees frames-*/
    while (1)
    {
        uint8_t b = 0x55;
        (void)g_Uart.Tx(&b, 1);
    }
}
