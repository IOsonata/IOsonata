/**-------------------------------------------------------------------------
@file	Vectors_esp32c3.c

@brief	ESP32-C3 RISC-V CPU interrupt vector table and INTMTX dispatch

ESP32-C3 mtvec is vectored mode only. The vector base must be aligned to
256 bytes. CPU interrupt id n enters at mtvec.BASE + 4*n. Each entry in
RISCV_VectorTable is one 32-bit jump instruction to RISCV_TrapHandler.

Peripheral sources are routed by the interrupt matrix to CPU interrupt IDs.
The common trap handler first dispatches the CPU interrupt ID from mcause,
then falls back to the interrupt-matrix source status registers. This matches
how ESP-IDF treats the C3 interrupt matrix: more peripheral sources exist
than CPU interrupt inputs, so software must check the active source status
when an interrupt input is shared.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

extern void RISCV_TrapHandler(void);

__asm__(
    ".pushsection .iram.text.vectors,\"ax\",@progbits\n"
    ".option push\n"
    ".option norvc\n"
    ".balign 256\n"
    ".global RISCV_VectorTable\n"
    ".type RISCV_VectorTable, @function\n"
    "RISCV_VectorTable:\n"
    "  .rept 32\n"
    "    jal x0, RISCV_TrapHandler\n"
    "  .endr\n"
    ".global RISCV_VectorTableEnd\n"
    "RISCV_VectorTableEnd:\n"
    ".size RISCV_VectorTable, . - RISCV_VectorTable\n"
    ".option pop\n"
    ".popsection\n"
);

extern void RISCV_VectorTable(void);
extern void RISCV_VectorTableEnd(void);
#define ESP32C3_INTMTX_BASE                 0x600C2000UL
#define ESP32C3_INTMTX_INTR_STATUS_0_REG    (*(volatile uint32_t *)(ESP32C3_INTMTX_BASE + 0x0F8U))
#define ESP32C3_INTMTX_INTR_STATUS_1_REG    (*(volatile uint32_t *)(ESP32C3_INTMTX_BASE + 0x0FCU))
#define ESP32C3_INTMTX_MAX_SOURCE           64U

volatile uint32_t g_Esp32C3DefIrqCount;

__attribute__((weak, section(".iram.text"), used))
void DEF_IRQHandler(void)
{
    g_Esp32C3DefIrqCount++;
    while (1)
    {
        __asm volatile("nop");
    }
}

typedef void (*Esp32C3IrqHandler)(void);

static volatile Esp32C3IrqHandler s_Esp32C3CpuIrqHandlers[32];
static volatile Esp32C3IrqHandler s_Esp32C3SourceIrqHandlers[ESP32C3_INTMTX_MAX_SOURCE];

volatile uint32_t g_Esp32C3TrapCount;
volatile uint32_t g_Esp32C3MtxDispatchCount;
volatile uint32_t g_Esp32C3UnhandledTrapCount;
volatile uint32_t g_Esp32C3LastMcause;
volatile uint32_t g_Esp32C3LastMepc;
volatile uint32_t g_Esp32C3LastMtval;
volatile uint32_t g_Esp32C3LastCpuIntId;
volatile uint32_t g_Esp32C3LastSourceStatus0;
volatile uint32_t g_Esp32C3LastSourceStatus1;
volatile uint32_t g_Esp32C3UnhandledSourceStatus0;
volatile uint32_t g_Esp32C3UnhandledSourceStatus1;
volatile uint32_t g_Esp32C3CpuDispatchCount[32];
volatile uint32_t g_Esp32C3Uart0IrqCount;
volatile uint32_t g_Esp32C3Uart1IrqCount;

void Esp32C3SetCpuIrqHandler(uint32_t cpu_int_id, Esp32C3IrqHandler handler)
{
    if ((cpu_int_id > 0U) && (cpu_int_id < 32U))
    {
        s_Esp32C3CpuIrqHandlers[cpu_int_id] = handler;
        __asm volatile ("fence iorw, iorw" ::: "memory");
    }
}

void Esp32C3ClearCpuIrqHandler(uint32_t cpu_int_id)
{
    if ((cpu_int_id > 0U) && (cpu_int_id < 32U))
    {
        s_Esp32C3CpuIrqHandlers[cpu_int_id] = 0;
        __asm volatile ("fence iorw, iorw" ::: "memory");
    }
}

void Esp32C3SetSourceIrqHandler(uint32_t source_id, Esp32C3IrqHandler handler)
{
    if (source_id < ESP32C3_INTMTX_MAX_SOURCE)
    {
        s_Esp32C3SourceIrqHandlers[source_id] = handler;
        __asm volatile ("fence iorw, iorw" ::: "memory");
    }
}

void Esp32C3ClearSourceIrqHandler(uint32_t source_id)
{
    if (source_id < ESP32C3_INTMTX_MAX_SOURCE)
    {
        s_Esp32C3SourceIrqHandlers[source_id] = 0;
        __asm volatile ("fence iorw, iorw" ::: "memory");
    }
}

__attribute__((section(".iram.text"), used, optimize("O0")))
void Esp32C3IntMtxDispatch(void)
{
    uint32_t status0 = ESP32C3_INTMTX_INTR_STATUS_0_REG;
    uint32_t status1 = ESP32C3_INTMTX_INTR_STATUS_1_REG;
    bool handled_any = false;

    g_Esp32C3MtxDispatchCount++;
    g_Esp32C3LastSourceStatus0 = status0;
    g_Esp32C3LastSourceStatus1 = status1;

    /* Do not use ctz/clz helpers in the trap path.  A helper call can land
     * outside IRAM and hides whether the CPU reached the dispatch path. */
    for (uint32_t bit = 0U; bit < 32U; bit++)
    {
        uint32_t mask = 1UL << bit;
        if ((status0 & mask) != 0U)
        {
            Esp32C3IrqHandler handler = s_Esp32C3SourceIrqHandlers[bit];
            if (handler != 0)
            {
                handled_any = true;
                handler();
            }
            else
            {
                g_Esp32C3UnhandledSourceStatus0 |= mask;
            }
        }
    }

    for (uint32_t bit = 0U; bit < 32U; bit++)
    {
        uint32_t mask = 1UL << bit;
        if ((status1 & mask) != 0U)
        {
            uint32_t source = 32U + bit;
            if (source < ESP32C3_INTMTX_MAX_SOURCE)
            {
                Esp32C3IrqHandler handler = s_Esp32C3SourceIrqHandlers[source];
                if (handler != 0)
                {
                    handled_any = true;
                    handler();
                }
                else
                {
                    g_Esp32C3UnhandledSourceStatus1 |= mask;
                }
            }
        }
    }

    if (!handled_any)
    {
        g_Esp32C3UnhandledTrapCount++;
        DEF_IRQHandler();
    }
}

__attribute__((interrupt("machine"), aligned(4), section(".iram.text"), used, optimize("O0")))
void RISCV_TrapHandler(void)
{
    uint32_t mcause;
    uint32_t mepc;
    uint32_t mtval;

    __asm volatile("csrr %0, mcause" : "=r"(mcause));
    __asm volatile("csrr %0, mepc" : "=r"(mepc));
    __asm volatile("csrr %0, mtval" : "=r"(mtval));

    g_Esp32C3TrapCount++;
    g_Esp32C3LastMcause = mcause;
    g_Esp32C3LastMepc = mepc;
    g_Esp32C3LastMtval = mtval;

    if ((mcause & 0x80000000UL) != 0U)
    {
        uint32_t cpu_int_id = mcause & 0x1FU;
        g_Esp32C3LastCpuIntId = cpu_int_id;

        if ((cpu_int_id > 0U) && (cpu_int_id < 32U))
        {
            Esp32C3IrqHandler handler = s_Esp32C3CpuIrqHandlers[cpu_int_id];
            g_Esp32C3CpuDispatchCount[cpu_int_id]++;
            if (handler != 0)
            {
                handler();
                return;
            }
        }

        Esp32C3IntMtxDispatch();
        return;
    }

    g_Esp32C3UnhandledTrapCount++;
    DEF_IRQHandler();
}
