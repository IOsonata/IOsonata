/**-------------------------------------------------------------------------
@file	Vectors_esp32c6.c

@brief	CLIC peripheral interrupt vector table for the ESP32-C6

Provides the 64 ESP32-C6 peripheral IRQ handler weak aliases and assembles
the complete 96-entry CLIC vector table:

    [0..31]   RISC-V exception cause handlers  — defined in Vectors_clic.c
    [32..95]  ESP32-C6 peripheral IRQs 0..63   — defined here

Interrupt source numbering: ESP32-C6 TRM Rev 0.6, Table 8-1.

The exception handler symbols (InstrAddrMisalign_Handler, etc.) are
provided by the shared Vectors_clic.c compilation unit and declared extern
here.  DEF_IRQHandler is defined weak in Vectors_clic.c; the weak alias
declarations for peripheral IRQs below require it to be visible in this
translation unit, so it is also defined weak here — the linker will
deduplicate to the single copy from Vectors_clic.c.

Alignment: 96 entries × 4 bytes = 384 bytes → aligned to 512 (next
power-of-two ≥ 384), satisfying the CLIC mtvec alignment requirement.

@author	Hoang Nguyen Hoan
@date	Mar. 5, 2026

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
 * Local weak DEF_IRQHandler — required for the alias attribute below.
 * Vectors_clic.c also defines this weak; the linker keeps one copy.
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
 * RISC-V exception cause handlers — defined in Vectors_clic.c (shared).
 *---------------------------------------------------------------------------*/
extern void InstrAddrMisalign_Handler(void);
extern void InstrAccessFault_Handler(void);
extern void IllegalInstr_Handler(void);
extern void Breakpoint_Handler(void);
extern void LoadAddrMisalign_Handler(void);
extern void LoadAccessFault_Handler(void);
extern void StoreAddrMisalign_Handler(void);
extern void StoreAccessFault_Handler(void);
extern void ECallU_Handler(void);
extern void ECallS_Handler(void);
extern void Reserved10_Handler(void);
extern void ECallM_Handler(void);
extern void InstrPageFault_Handler(void);
extern void LoadPageFault_Handler(void);
extern void Reserved14_Handler(void);
extern void StorePageFault_Handler(void);
extern void Reserved16_Handler(void);
extern void Reserved17_Handler(void);
extern void Reserved18_Handler(void);
extern void Reserved19_Handler(void);
extern void Reserved20_Handler(void);
extern void Reserved21_Handler(void);
extern void Reserved22_Handler(void);
extern void Reserved23_Handler(void);
extern void Reserved24_Handler(void);
extern void Reserved25_Handler(void);
extern void Reserved26_Handler(void);
extern void Reserved27_Handler(void);
extern void Reserved28_Handler(void);
extern void Reserved29_Handler(void);
extern void Reserved30_Handler(void);
extern void Reserved31_Handler(void);

/*---------------------------------------------------------------------------
 * ESP32-C6 peripheral interrupt handlers (IRQ 0..63).
 * Source numbering: ESP32-C6 TRM Rev 0.6, Table 8-1.
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
/* IRQ 7  — LP_TIMER */
__attribute__((weak, alias("DEF_IRQHandler"))) void LP_TIMER_IRQHandler(void);
/* IRQ 8  — COEX */
__attribute__((weak, alias("DEF_IRQHandler"))) void COEX_IRQHandler(void);
/* IRQ 9  — BLE_TIMER */
__attribute__((weak, alias("DEF_IRQHandler"))) void BLE_TIMER_IRQHandler(void);
/* IRQ 10 — BLE_SEC */
__attribute__((weak, alias("DEF_IRQHandler"))) void BLE_SEC_IRQHandler(void);
/* IRQ 11 — I2C_MASTER */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C_MASTER_IRQHandler(void);
/* IRQ 12 — APB_ADC */
__attribute__((weak, alias("DEF_IRQHandler"))) void APB_ADC_IRQHandler(void);
/* IRQ 13 — GDMA_CH0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void GDMA_CH0_IRQHandler(void);
/* IRQ 14 — GDMA_CH1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void GDMA_CH1_IRQHandler(void);
/* IRQ 15 — GDMA_CH2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void GDMA_CH2_IRQHandler(void);
/* IRQ 16 — RSA */
__attribute__((weak, alias("DEF_IRQHandler"))) void RSA_IRQHandler(void);
/* IRQ 17 — AES */
__attribute__((weak, alias("DEF_IRQHandler"))) void AES_IRQHandler(void);
/* IRQ 18 — SHA */
__attribute__((weak, alias("DEF_IRQHandler"))) void SHA_IRQHandler(void);
/* IRQ 19 — ECC */
__attribute__((weak, alias("DEF_IRQHandler"))) void ECC_IRQHandler(void);
/* IRQ 20 — ECDSA */
__attribute__((weak, alias("DEF_IRQHandler"))) void ECDSA_IRQHandler(void);
/* IRQ 21 — GPIO */
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_IRQHandler(void);
/* IRQ 22 — GPIO_NMI */
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIO_NMI_IRQHandler(void);
/* IRQ 23 — PAU */
__attribute__((weak, alias("DEF_IRQHandler"))) void PAU_IRQHandler(void);
/* IRQ 24 — HP_APM_M0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void HP_APM_M0_IRQHandler(void);
/* IRQ 25 — HP_APM_M1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void HP_APM_M1_IRQHandler(void);
/* IRQ 26 — HP_APM_M2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void HP_APM_M2_IRQHandler(void);
/* IRQ 27 — HP_APM_M3 */
__attribute__((weak, alias("DEF_IRQHandler"))) void HP_APM_M3_IRQHandler(void);
/* IRQ 28 — LP_APM0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void LP_APM0_IRQHandler(void);
/* IRQ 29 — Reserved */
__attribute__((weak, alias("DEF_IRQHandler"))) void IRQ29_IRQHandler(void);
/* IRQ 30 — MSPI */
__attribute__((weak, alias("DEF_IRQHandler"))) void MSPI_IRQHandler(void);
/* IRQ 31 — Reserved */
__attribute__((weak, alias("DEF_IRQHandler"))) void IRQ31_IRQHandler(void);
/* IRQ 32 — SW interrupt from CPU 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU0_IRQHandler(void);
/* IRQ 33 — SW interrupt from CPU 1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU1_IRQHandler(void);
/* IRQ 34 — SW interrupt from CPU 2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU2_IRQHandler(void);
/* IRQ 35 — SW interrupt from CPU 3 */
__attribute__((weak, alias("DEF_IRQHandler"))) void FROM_CPU3_IRQHandler(void);
/* IRQ 36 — TG0 Timer 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG0_T0_IRQHandler(void);
/* IRQ 37 — TG0 Watchdog */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG0_WDT_IRQHandler(void);
/* IRQ 38 — TG1 Timer 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG1_T0_IRQHandler(void);
/* IRQ 39 — TG1 Watchdog */
__attribute__((weak, alias("DEF_IRQHandler"))) void TG1_WDT_IRQHandler(void);
/* IRQ 40 — Cache */
__attribute__((weak, alias("DEF_IRQHandler"))) void CACHE_IRQHandler(void);
/* IRQ 41 — UART0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void UART0_IRQHandler(void);
/* IRQ 42 — UART1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void UART1_IRQHandler(void);
/* IRQ 43 — LP UART */
__attribute__((weak, alias("DEF_IRQHandler"))) void LP_UART_IRQHandler(void);
/* IRQ 44 — I2C0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C0_IRQHandler(void);
/* IRQ 45 — I2C1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_IRQHandler(void);
/* IRQ 46 — SPI2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI2_IRQHandler(void);
/* IRQ 47 — TWAI0 (CAN) */
__attribute__((weak, alias("DEF_IRQHandler"))) void TWAI0_IRQHandler(void);
/* IRQ 48 — TWAI1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void TWAI1_IRQHandler(void);
/* IRQ 49 — Reserved */
__attribute__((weak, alias("DEF_IRQHandler"))) void IRQ49_IRQHandler(void);
/* IRQ 50 — LEDC */
__attribute__((weak, alias("DEF_IRQHandler"))) void LEDC_IRQHandler(void);
/* IRQ 51 — ADC */
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC_IRQHandler(void);
/* IRQ 52 — MCPWM0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void MCPWM0_IRQHandler(void);
/* IRQ 53 — Reserved */
__attribute__((weak, alias("DEF_IRQHandler"))) void IRQ53_IRQHandler(void);
/* IRQ 54 — PCNT */
__attribute__((weak, alias("DEF_IRQHandler"))) void PCNT_IRQHandler(void);
/* IRQ 55 — PARL_IO */
__attribute__((weak, alias("DEF_IRQHandler"))) void PARL_IO_IRQHandler(void);
/* IRQ 56 — RMT */
__attribute__((weak, alias("DEF_IRQHandler"))) void RMT_IRQHandler(void);
/* IRQ 57 — I2S0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S0_IRQHandler(void);
/* IRQ 58 — USB Device */
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_DEVICE_IRQHandler(void);
/* IRQ 59 — SYSTIMER Target 0 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSTIMER_TARGET0_IRQHandler(void);
/* IRQ 60 — SYSTIMER Target 1 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSTIMER_TARGET1_IRQHandler(void);
/* IRQ 61 — SYSTIMER Target 2 */
__attribute__((weak, alias("DEF_IRQHandler"))) void SYSTIMER_TARGET2_IRQHandler(void);
/* IRQ 62 — IEEE 802.15.4 */
__attribute__((weak, alias("DEF_IRQHandler"))) void IEEE802154_IRQHandler(void);
/* IRQ 63 — Reserved / alignment filler */
__attribute__((weak, alias("DEF_IRQHandler"))) void IRQ63_IRQHandler(void);

/*---------------------------------------------------------------------------
 * The vector table.
 *
 * Layout:
 *   [0..31]   RISC-V exception/trap cause handlers  (from Vectors_clic.c)
 *   [32..95]  ESP32-C6 peripheral IRQs 0..63        (defined above)
 *
 * 96 entries × 4 bytes = 384 bytes → aligned to 512 (next power-of-two).
 * SystemInit() writes:  csrw mtvec, (&g_Esp32C6Vectors | 0x3)
 *---------------------------------------------------------------------------*/
__attribute__((section(".iram.text"), used, aligned(512)))
void (* const g_Esp32C6Vectors[])(void) =
{
	/* ── RISC-V exception cause handlers [0..31] ─────────────────────── */
	[0]  = InstrAddrMisalign_Handler,
	[1]  = InstrAccessFault_Handler,
	[2]  = IllegalInstr_Handler,
	[3]  = Breakpoint_Handler,
	[4]  = LoadAddrMisalign_Handler,
	[5]  = LoadAccessFault_Handler,
	[6]  = StoreAddrMisalign_Handler,
	[7]  = StoreAccessFault_Handler,
	[8]  = ECallU_Handler,
	[9]  = ECallS_Handler,
	[10] = Reserved10_Handler,
	[11] = ECallM_Handler,
	[12] = InstrPageFault_Handler,
	[13] = LoadPageFault_Handler,
	[14] = Reserved14_Handler,
	[15] = StorePageFault_Handler,
	[16] = Reserved16_Handler,
	[17] = Reserved17_Handler,
	[18] = Reserved18_Handler,
	[19] = Reserved19_Handler,
	[20] = Reserved20_Handler,
	[21] = Reserved21_Handler,
	[22] = Reserved22_Handler,
	[23] = Reserved23_Handler,
	[24] = Reserved24_Handler,
	[25] = Reserved25_Handler,
	[26] = Reserved26_Handler,
	[27] = Reserved27_Handler,
	[28] = Reserved28_Handler,
	[29] = Reserved29_Handler,
	[30] = Reserved30_Handler,
	[31] = Reserved31_Handler,

	/* ── ESP32-C6 peripheral IRQs [32..95] (IRQ 0..63) ──────────────── */
	[32] = WIFI_MAC_IRQHandler,
	[33] = WIFI_MAC_NMI_IRQHandler,
	[34] = WIFI_PWR_IRQHandler,
	[35] = WIFI_BB_IRQHandler,
	[36] = BT_MAC_IRQHandler,
	[37] = BT_BB_IRQHandler,
	[38] = BT_BB_NMI_IRQHandler,
	[39] = LP_TIMER_IRQHandler,
	[40] = COEX_IRQHandler,
	[41] = BLE_TIMER_IRQHandler,
	[42] = BLE_SEC_IRQHandler,
	[43] = I2C_MASTER_IRQHandler,
	[44] = APB_ADC_IRQHandler,
	[45] = GDMA_CH0_IRQHandler,
	[46] = GDMA_CH1_IRQHandler,
	[47] = GDMA_CH2_IRQHandler,
	[48] = RSA_IRQHandler,
	[49] = AES_IRQHandler,
	[50] = SHA_IRQHandler,
	[51] = ECC_IRQHandler,
	[52] = ECDSA_IRQHandler,
	[53] = GPIO_IRQHandler,
	[54] = GPIO_NMI_IRQHandler,
	[55] = PAU_IRQHandler,
	[56] = HP_APM_M0_IRQHandler,
	[57] = HP_APM_M1_IRQHandler,
	[58] = HP_APM_M2_IRQHandler,
	[59] = HP_APM_M3_IRQHandler,
	[60] = LP_APM0_IRQHandler,
	[61] = IRQ29_IRQHandler,
	[62] = MSPI_IRQHandler,
	[63] = IRQ31_IRQHandler,
	[64] = FROM_CPU0_IRQHandler,
	[65] = FROM_CPU1_IRQHandler,
	[66] = FROM_CPU2_IRQHandler,
	[67] = FROM_CPU3_IRQHandler,
	[68] = TG0_T0_IRQHandler,
	[69] = TG0_WDT_IRQHandler,
	[70] = TG1_T0_IRQHandler,
	[71] = TG1_WDT_IRQHandler,
	[72] = CACHE_IRQHandler,
	[73] = UART0_IRQHandler,
	[74] = UART1_IRQHandler,
	[75] = LP_UART_IRQHandler,
	[76] = I2C0_IRQHandler,
	[77] = I2C1_IRQHandler,
	[78] = SPI2_IRQHandler,
	[79] = TWAI0_IRQHandler,
	[80] = TWAI1_IRQHandler,
	[81] = IRQ49_IRQHandler,
	[82] = LEDC_IRQHandler,
	[83] = ADC_IRQHandler,
	[84] = MCPWM0_IRQHandler,
	[85] = IRQ53_IRQHandler,
	[86] = PCNT_IRQHandler,
	[87] = PARL_IO_IRQHandler,
	[88] = RMT_IRQHandler,
	[89] = I2S0_IRQHandler,
	[90] = USB_DEVICE_IRQHandler,
	[91] = SYSTIMER_TARGET0_IRQHandler,
	[92] = SYSTIMER_TARGET1_IRQHandler,
	[93] = SYSTIMER_TARGET2_IRQHandler,
	[94] = IEEE802154_IRQHandler,
	[95] = IRQ63_IRQHandler,
};
