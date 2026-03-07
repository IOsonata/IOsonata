/**-------------------------------------------------------------------------
@file	Vectors_esp32c3.c

@brief	ESP32-C3 peripheral interrupt dispatch via INTMTX

The ESP32-C3 uses the RISC-V CLINT interrupt controller (not CLIC).
mtvec is installed in direct mode (mtvec[1:0] = 0b00), so ALL traps
(exceptions and interrupts) are routed to the single entry point
RISCV_TrapHandler, which is installed into mtvec by SystemInit().

Interrupt routing:
  1. A peripheral fires its interrupt.
  2. The INTMTX (Interrupt Matrix) routes it to one of 31 CPU interrupt
     lines based on programmed INTMTX_CORE0_xxx_INT_MAP_REG values.
  3. The RISC-V core takes a machine external interrupt (mcause = 0x8000000B).
  4. RISCV_TrapHandler reads mcause, calls Esp32C3ExtIRQDispatch().
  5. Esp32C3ExtIRQDispatch() reads INTMTX status registers, calls the
     matching peripheral handler from g_Esp32C3PeriphVectors[].

Peripheral interrupt numbering: ESP32-C3 TRM Rev 0.4, Table 9-2.
62 peripheral interrupt sources (IRQ 0..61).

INTMTX base address: 0x600C2000 (ESP32-C3 TRM, §9.3).

@author	Hoang Nguyen Hoan
@date	Mar. 6, 2026

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
#include <stdint.h>

/*---------------------------------------------------------------------------
 * INTMTX — Interrupt Matrix
 * ESP32-C3 TRM Rev 0.4, §9.3   Base: 0x600C2000
 *
 * INTMTX_CORE0_INT_STATUS_REG_0: bits [31:0]  → peripheral IRQs 0..31
 * INTMTX_CORE0_INT_STATUS_REG_1: bits [29:0]  → peripheral IRQs 32..61
 *---------------------------------------------------------------------------*/
#define INTMTX_BASE                       0x600C2000UL
#define INTMTX_CORE0_INT_STATUS_REG_0     (*(volatile uint32_t *)(INTMTX_BASE + 0x0F0U))
#define INTMTX_CORE0_INT_STATUS_REG_1     (*(volatile uint32_t *)(INTMTX_BASE + 0x0F4U))

/*---------------------------------------------------------------------------
 * Local weak DEF_IRQHandler — required for alias attribute.
 * Vectors_esp32_intmtx.c also defines this weak; the linker keeps one copy.
 *---------------------------------------------------------------------------*/
__attribute__((weak, section(".iram.text"), used))
void DEF_IRQHandler(void)
{
	while (1)
	{
		__asm volatile("nop");
	}
}

/*---------------------------------------------------------------------------
 * Exception cause dispatch table — defined in Vectors_esp32_intmtx.c.
 *---------------------------------------------------------------------------*/
extern void (* const g_Esp32IntmtxExcVectors[32])(void);

/*---------------------------------------------------------------------------
 * Machine software and timer interrupt handlers (weak, overridable).
 *---------------------------------------------------------------------------*/
__attribute__((weak, alias("DEF_IRQHandler"))) void MSWI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MTIMER_IRQHandler(void);

/*---------------------------------------------------------------------------
 * ESP32-C3 peripheral interrupt handlers (IRQ 0..61).
 * Source numbering: ESP32-C3 TRM Rev 0.4, Table 9-2.
 * Override by defining a non-weak function with the same name.
 *---------------------------------------------------------------------------*/

/* IRQ 0  — WIFI_MAC */
__attribute__((weak, alias("DEF_IRQHandler"))) void WIFI_MAC_IRQHandler(void);
/* IRQ 1  — WIFI_MAC_NMI */
__attribute__((weak, alias("DEF_IRQHandler"))) void WIFI_MAC_NMI_IRQHandler(void);
/* IRQ 2  — WIFI_PWR */
__attribute__((weak, alias("DEF_IRQHandler"))) void WIFI_PWR_IRQHandler(void);
/* IRQ 3  — WIFI_BB */
__attribute__((weak, alias("DEF_IRQHandler"))) void WIFI_BB_IRQHandler(void);
/* IRQ 4  — BT_MAC */
__attribute__((weak, alias("DEF_IRQHandler"))) void BT_MAC_IRQHandler(void);
/* IRQ 5  — BT_BB */
__attribute__((weak, alias("DEF_IRQHandler"))) void BT_BB_IRQHandler(void);
/* IRQ 6  — BT_BB_NMI */
__attribute__((weak, alias("DEF_IRQHandler"))) void BT_BB_NMI_IRQHandler(void);
/* IRQ 7  — RWBT */
__attribute__((weak, alias("DEF_IRQHandler"))) void RWBT_IRQHandler(void);
/* IRQ 8  — RWBLE */
__attribute__((weak, alias("DEF_IRQHandler"))) void RWBLE_IRQHandler(void);
/* IRQ 9  — RWBT_NMI */
__attribute__((weak, alias("DEF_IRQHandler"))) void RWBT_NMI_IRQHandler(void);
/* IRQ 10 — RWBLE_NMI */
__attribute__((weak, alias("DEF_IRQHandler"))) void RWBLE_NMI_IRQHandler(void);
/* IRQ 11 — I2C_MASTER */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C_MASTER_IRQHandler(void);
/* IRQ 12 — SLC0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SLC0_IRQHandler(void);
/* IRQ 13 — SLC1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SLC1_IRQHandler(void);
/* IRQ 14 — APB_CTRL */
__attribute__((weak, alias("DEF_IRQHandler"))) void APB_CTRL_IRQHandler(void);
/* IRQ 15 — UHCI0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void UHCI0_IRQHandler(void);
/* IRQ 16 — GPIO */
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_IRQHandler(void);
/* IRQ 17 — GPIO_NMI */
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_NMI_IRQHandler(void);
/* IRQ 18 — SPI1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI1_IRQHandler(void);
/* IRQ 19 — SPI2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI2_IRQHandler(void);
/* IRQ 20 — I2S0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S0_IRQHandler(void);
/* IRQ 21 — UART0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void UART0_IRQHandler(void);
/* IRQ 22 — UART1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void UART1_IRQHandler(void);
/* IRQ 23 — LEDC */
__attribute__((weak, alias("DEF_IRQHandler"))) void LEDC_IRQHandler(void);
/* IRQ 24 — EFUSE */
__attribute__((weak, alias("DEF_IRQHandler"))) void EFUSE_IRQHandler(void);
/* IRQ 25 — TWAI (CAN) */
__attribute__((weak, alias("DEF_IRQHandler"))) void TWAI_IRQHandler(void);
/* IRQ 26 — USB_SERIAL_JTAG */
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_SERIAL_JTAG_IRQHandler(void);
/* IRQ 27 — RTC_CORE */
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_CORE_IRQHandler(void);
/* IRQ 28 — RMT */
__attribute__((weak, alias("DEF_IRQHandler"))) void RMT_IRQHandler(void);
/* IRQ 29 — I2C_EXT0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C_EXT0_IRQHandler(void);
/* IRQ 30 — TIMER1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER1_IRQHandler(void);
/* IRQ 31 — TIMER2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER2_IRQHandler(void);
/* IRQ 32 — TG0 Timer 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG0_T0_IRQHandler(void);
/* IRQ 33 — TG0 Watchdog */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG0_WDT_IRQHandler(void);
/* IRQ 34 — TG1 Timer 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG1_T0_IRQHandler(void);
/* IRQ 35 — TG1 Watchdog */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG1_WDT_IRQHandler(void);
/* IRQ 36 — CACHE_IA */
__attribute__((weak, alias("DEF_IRQHandler"))) void CACHE_IA_IRQHandler(void);
/* IRQ 37 — SYSTIMER Target 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSTIMER_TARGET0_IRQHandler(void);
/* IRQ 38 — SYSTIMER Target 1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSTIMER_TARGET1_IRQHandler(void);
/* IRQ 39 — SYSTIMER Target 2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSTIMER_TARGET2_IRQHandler(void);
/* IRQ 40 — SPI_MEM_REJECT_CACHE */
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI_MEM_REJECT_CACHE_IRQHandler(void);
/* IRQ 41 — ICACHE_PRELOAD0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void ICACHE_PRELOAD0_IRQHandler(void);
/* IRQ 42 — ICACHE_SYNC0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void ICACHE_SYNC0_IRQHandler(void);
/* IRQ 43 — APB_ADC */
__attribute__((weak, alias("DEF_IRQHandler"))) void APB_ADC_IRQHandler(void);
/* IRQ 44 — DMA_CH0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA_CH0_IRQHandler(void);
/* IRQ 45 — DMA_CH1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA_CH1_IRQHandler(void);
/* IRQ 46 — DMA_CH2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA_CH2_IRQHandler(void);
/* IRQ 47 — RSA */
__attribute__((weak, alias("DEF_IRQHandler"))) void RSA_IRQHandler(void);
/* IRQ 48 — AES */
__attribute__((weak, alias("DEF_IRQHandler"))) void AES_IRQHandler(void);
/* IRQ 49 — SHA */
__attribute__((weak, alias("DEF_IRQHandler"))) void SHA_IRQHandler(void);
/* IRQ 50 — FROM_CPU_INTR0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU_INTR0_IRQHandler(void);
/* IRQ 51 — FROM_CPU_INTR1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU_INTR1_IRQHandler(void);
/* IRQ 52 — FROM_CPU_INTR2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU_INTR2_IRQHandler(void);
/* IRQ 53 — FROM_CPU_INTR3 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU_INTR3_IRQHandler(void);
/* IRQ 54 — ASSIST_DEBUG */
__attribute__((weak, alias("DEF_IRQHandler"))) void ASSIST_DEBUG_IRQHandler(void);
/* IRQ 55 — DMA_APBPERI_PMS */
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA_APBPERI_PMS_IRQHandler(void);
/* IRQ 56 — CORE0_IRAM0_PMS */
__attribute__((weak, alias("DEF_IRQHandler"))) void CORE0_IRAM0_PMS_IRQHandler(void);
/* IRQ 57 — CORE0_DRAM0_PMS */
__attribute__((weak, alias("DEF_IRQHandler"))) void CORE0_DRAM0_PMS_IRQHandler(void);
/* IRQ 58 — CORE0_PIF_PMS */
__attribute__((weak, alias("DEF_IRQHandler"))) void CORE0_PIF_PMS_IRQHandler(void);
/* IRQ 59 — CORE0_PIF_PMS_SIZE */
__attribute__((weak, alias("DEF_IRQHandler"))) void CORE0_PIF_PMS_SIZE_IRQHandler(void);
/* IRQ 60 — BAK_PMS_VIOLATE */
__attribute__((weak, alias("DEF_IRQHandler"))) void BAK_PMS_VIOLATE_IRQHandler(void);
/* IRQ 61 — CACHE_CORE0_ACS */
__attribute__((weak, alias("DEF_IRQHandler"))) void CACHE_CORE0_ACS_IRQHandler(void);

/*---------------------------------------------------------------------------
 * Peripheral dispatch table — indexed by peripheral IRQ number (0..61).
 * Used by Esp32C3ExtIRQDispatch() when a machine external interrupt fires.
 * Override individual entries by providing non-weak handler functions above.
 *---------------------------------------------------------------------------*/
static void (* const s_Esp32C3PeriphVectors[62])(void) =
{
	[0]  = WIFI_MAC_IRQHandler,
	[1]  = WIFI_MAC_NMI_IRQHandler,
	[2]  = WIFI_PWR_IRQHandler,
	[3]  = WIFI_BB_IRQHandler,
	[4]  = BT_MAC_IRQHandler,
	[5]  = BT_BB_IRQHandler,
	[6]  = BT_BB_NMI_IRQHandler,
	[7]  = RWBT_IRQHandler,
	[8]  = RWBLE_IRQHandler,
	[9]  = RWBT_NMI_IRQHandler,
	[10] = RWBLE_NMI_IRQHandler,
	[11] = I2C_MASTER_IRQHandler,
	[12] = SLC0_IRQHandler,
	[13] = SLC1_IRQHandler,
	[14] = APB_CTRL_IRQHandler,
	[15] = UHCI0_IRQHandler,
	[16] = GPIO_IRQHandler,
	[17] = GPIO_NMI_IRQHandler,
	[18] = SPI1_IRQHandler,
	[19] = SPI2_IRQHandler,
	[20] = I2S0_IRQHandler,
	[21] = UART0_IRQHandler,
	[22] = UART1_IRQHandler,
	[23] = LEDC_IRQHandler,
	[24] = EFUSE_IRQHandler,
	[25] = TWAI_IRQHandler,
	[26] = USB_SERIAL_JTAG_IRQHandler,
	[27] = RTC_CORE_IRQHandler,
	[28] = RMT_IRQHandler,
	[29] = I2C_EXT0_IRQHandler,
	[30] = TIMER1_IRQHandler,
	[31] = TIMER2_IRQHandler,
	[32] = TG0_T0_IRQHandler,
	[33] = TG0_WDT_IRQHandler,
	[34] = TG1_T0_IRQHandler,
	[35] = TG1_WDT_IRQHandler,
	[36] = CACHE_IA_IRQHandler,
	[37] = SYSTIMER_TARGET0_IRQHandler,
	[38] = SYSTIMER_TARGET1_IRQHandler,
	[39] = SYSTIMER_TARGET2_IRQHandler,
	[40] = SPI_MEM_REJECT_CACHE_IRQHandler,
	[41] = ICACHE_PRELOAD0_IRQHandler,
	[42] = ICACHE_SYNC0_IRQHandler,
	[43] = APB_ADC_IRQHandler,
	[44] = DMA_CH0_IRQHandler,
	[45] = DMA_CH1_IRQHandler,
	[46] = DMA_CH2_IRQHandler,
	[47] = RSA_IRQHandler,
	[48] = AES_IRQHandler,
	[49] = SHA_IRQHandler,
	[50] = FROM_CPU_INTR0_IRQHandler,
	[51] = FROM_CPU_INTR1_IRQHandler,
	[52] = FROM_CPU_INTR2_IRQHandler,
	[53] = FROM_CPU_INTR3_IRQHandler,
	[54] = ASSIST_DEBUG_IRQHandler,
	[55] = DMA_APBPERI_PMS_IRQHandler,
	[56] = CORE0_IRAM0_PMS_IRQHandler,
	[57] = CORE0_DRAM0_PMS_IRQHandler,
	[58] = CORE0_PIF_PMS_IRQHandler,
	[59] = CORE0_PIF_PMS_SIZE_IRQHandler,
	[60] = BAK_PMS_VIOLATE_IRQHandler,
	[61] = CACHE_CORE0_ACS_IRQHandler,
};

/*---------------------------------------------------------------------------
 * Esp32C3ExtIRQDispatch — called from RISCV_TrapHandler on machine external
 * interrupt (mcause = 11).
 *
 * Reads both INTMTX status registers and dispatches to every pending
 * peripheral handler.  Handles all simultaneously-asserted IRQs in a
 * single pass (lowest-numbered first).
 *---------------------------------------------------------------------------*/
__attribute__((section(".iram.text")))
static void Esp32C3ExtIRQDispatch(void)
{
	uint32_t st0 = INTMTX_CORE0_INT_STATUS_REG_0;   /* IRQs  0..31 */
	uint32_t st1 = INTMTX_CORE0_INT_STATUS_REG_1;   /* IRQs 32..61 */

	/* Dispatch IRQs 0..31 */
	while (st0)
	{
		uint32_t bit = __builtin_ctz(st0);           /* lowest set bit */
		s_Esp32C3PeriphVectors[bit]();
		st0 &= st0 - 1U;                             /* clear lowest bit */
	}

	/* Dispatch IRQs 32..61 */
	while (st1)
	{
		uint32_t bit = __builtin_ctz(st1);
		if (bit < 30U)                               /* 62 sources = 32+30 */
		{
			s_Esp32C3PeriphVectors[32U + bit]();
		}
		st1 &= st1 - 1U;
	}
}

/*---------------------------------------------------------------------------
 * RISCV_TrapHandler — single entry point for all machine-mode traps.
 *
 * Installed into mtvec in direct mode (mtvec[1:0] = 0b00) by SystemInit().
 *
 * GCC __attribute__((interrupt("machine"))) generates proper RISC-V trap
 * entry/exit (full register save, mret return) automatically.
 *
 * Placed in IRAM so it is reachable even when the Flash XIP cache is off.
 *---------------------------------------------------------------------------*/
__attribute__((interrupt("machine"), section(".iram.text"), used))
void RISCV_TrapHandler(void)
{
	uint32_t mcause;
	__asm volatile("csrr %0, mcause" : "=r"(mcause));

	if (mcause & 0x80000000U)
	{
		/* Asynchronous interrupt — mcause[30:0] is the interrupt cause */
		switch (mcause & 0x7FFFFFFFU)
		{
			case 3U:   /* Machine software interrupt */
				MSWI_IRQHandler();
				break;

			case 7U:   /* Machine timer interrupt */
				MTIMER_IRQHandler();
				break;

			case 11U:  /* Machine external interrupt — dispatch via INTMTX */
				Esp32C3ExtIRQDispatch();
				break;

			default:
				DEF_IRQHandler();
				break;
		}
	}
	else
	{
		/* Synchronous exception — dispatch via exception table */
		uint32_t cause = mcause & 0x1FU;   /* cap at 31 */
		g_Esp32IntmtxExcVectors[cause]();
	}
}
