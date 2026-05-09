/**-------------------------------------------------------------------------
@file	system_esp32c3.c

@brief	ESP32-C3 chip-specific SystemInit()

----------------------------------------------------------------------------*/
#include <stdint.h>

extern void Esp32SystemInit(void);
extern void RISCV_VectorTable(void);

#define ESP32C3_INTMTX_BASE             0x600C2000UL
#define ESP32C3_INTMTX_SOURCE_MAP(src)  (*(volatile uint32_t *)(ESP32C3_INTMTX_BASE + ((uint32_t)(src) * 4U)))
#define ESP32C3_INTMTX_DISABLED_INUM    0U
#define ESP32C3_INTSRC_UART0            21U
#define ESP32C3_INTSRC_UART1            22U

volatile uintptr_t g_Esp32C3VectorBase;
volatile uintptr_t g_Esp32C3MtvecWritten;
volatile uintptr_t g_Esp32C3MtvecReadback;
volatile uint32_t g_Esp32C3VectorRangeError;
volatile uint32_t g_Esp32C3SystemFix9BuildMarker = 0x26050809UL;

void SystemInit(void)
{
    Esp32SystemInit();

    uintptr_t vt = (uintptr_t)RISCV_VectorTable;
    g_Esp32C3VectorBase = vt;

    /* ESP32-C3 mtvec BASE must be 256-byte aligned. The vector object is
     * aligned in Vectors_esp32c3.c and kept first in .iram.text by the linker.
     * Do not silently round a bad address down; that sends interrupts into
     * unrelated code before the table. */
    if (((vt & 0xFFUL) != 0U) || (vt < 0x40380000UL) || (vt >= 0x40384000UL))
    {
        g_Esp32C3VectorRangeError = 1U;
        while (1)
        {
            __asm volatile("nop");
        }
    }

    __asm volatile("csrci mstatus, 0x8" ::: "memory");

    /* Reset CPU interrupt inputs before drivers attach their sources. */
    *(volatile uint32_t *)0x600C2100UL = 1U;          /* INTMTX clock gate */
    *(volatile uint32_t *)0x600C2104UL = 0U;          /* CPU_INT_ENABLE */
    *(volatile uint32_t *)0x600C2108UL = 0U;          /* CPU_INT_TYPE: level */
    *(volatile uint32_t *)0x600C210CUL = 0xFFFFFFFFUL;/* CPU_INT_CLEAR set */
    *(volatile uint32_t *)0x600C210CUL = 0U;          /* CPU_INT_CLEAR release */
    *(volatile uint32_t *)0x600C2194UL = 0U;          /* CPU_INT_THRESH */

    /* ROM may leave UART console sources routed to CPU interrupt 5. Clear
     * the UART source map registers before the UART driver installs its
     * own handlers. The TRM defines a zero source map value as disabled. */
    ESP32C3_INTMTX_SOURCE_MAP(ESP32C3_INTSRC_UART0) = ESP32C3_INTMTX_DISABLED_INUM;
    ESP32C3_INTMTX_SOURCE_MAP(ESP32C3_INTSRC_UART1) = ESP32C3_INTMTX_DISABLED_INUM;
    __asm volatile ("fence iorw, iorw" ::: "memory");

    /* Bit 0 selects vectored mode. */
    uintptr_t mtvec = vt | 1UL;
    g_Esp32C3MtvecWritten = mtvec;
    __asm volatile("csrw mtvec, %0" : : "r"(mtvec) : "memory");
    __asm volatile("csrr %0, mtvec" : "=r"(g_Esp32C3MtvecReadback));
}
