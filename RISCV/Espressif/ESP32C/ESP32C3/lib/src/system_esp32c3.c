/**-------------------------------------------------------------------------
@file	system_esp32c3.c

@brief	ESP32-C3 chip-specific SystemInit()

C3-specific policy and bring-up.  All INTMTX register addresses come
from the shared esp32xx_intmtx.h.  Anything chip-agnostic (clock tree,
matrix register layout) is in the cross-chip files.

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "esp32xx.h"
#include "esp32xx_intmtx.h"
#include "esp32xx_uart.h"   // for ESP32_UART_INTR_SOURCE_UART0 / _UART1

extern void Esp32SystemInit(void);
extern void RISCV_VectorTable(void);

#define ESP32C3_INTMTX_DISABLED_INUM    0U  //!< CPU INT 0 == "disabled" per TRM.

volatile uintptr_t g_Esp32C3VectorBase;
volatile uintptr_t g_Esp32C3MtvecWritten;
volatile uintptr_t g_Esp32C3MtvecReadback;
volatile uint32_t  g_Esp32C3VectorRangeError;

void SystemInit(void)
{
    Esp32SystemInit();

    uintptr_t vt = (uintptr_t)RISCV_VectorTable;
    g_Esp32C3VectorBase = vt;

    // ESP32-C3 mtvec BASE must be 256-byte aligned. The vector object is
    // aligned in Vectors_esp32c3.c and kept first in .iram.text by the linker.
    // Do not silently round a bad address down; that sends interrupts into
    // unrelated code before the table.
    if (((vt & 0xFFUL) != 0U) || (vt < 0x40380000UL) || (vt >= 0x40384000UL))
    {
        g_Esp32C3VectorRangeError = 1U;
        while (1)
        {
            __asm volatile("nop");
        }
    }

    __asm volatile("csrci mstatus, 0x8" ::: "memory");

    // Reset CPU interrupt inputs before drivers attach their sources.
    // Register addresses come from esp32xx_intmtx.h so this stays in
    // sync with the cross-chip header.
    ESP32_INTMTX_CLOCK_GATE_REG = 1U;
    ESP32_CPU_INT_ENABLE_REG    = 0U;
    ESP32_CPU_INT_TYPE_REG      = 0U;            // 0 = level for all lines
    ESP32_CPU_INT_CLEAR_REG     = 0xFFFFFFFFUL;  // pulse: assert
    ESP32_CPU_INT_CLEAR_REG     = 0U;            // pulse: release
    ESP32_CPU_INT_THRESH_REG    = 0U;

    // ROM may leave UART console sources routed to CPU interrupt 5. Clear
    // the UART source map registers before the UART driver installs its
    // own handlers.  TRM defines a zero source map value as disabled.
    ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART0) = ESP32C3_INTMTX_DISABLED_INUM;
    ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART1) = ESP32C3_INTMTX_DISABLED_INUM;
    __asm volatile ("fence iorw, iorw" ::: "memory");

    // Bit 0 of mtvec selects vectored mode.
    uintptr_t mtvec = vt | 1UL;
    g_Esp32C3MtvecWritten = mtvec;
    __asm volatile("csrw mtvec, %0" : : "r"(mtvec) : "memory");
    __asm volatile("csrr %0, mtvec" : "=r"(g_Esp32C3MtvecReadback));
}
