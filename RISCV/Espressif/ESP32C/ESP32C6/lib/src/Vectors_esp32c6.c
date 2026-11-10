/**-------------------------------------------------------------------------
@file	Vectors_esp32c6.c

@brief	ESP32-C6 CPU interrupt matrix vector and exception dispatch.

The C6 has 32 CPU interrupt inputs, with external interrupt routing via
INTMTX and priorities via the machine-mode PLIC. It is NOT a 96-slot CLIC controller.
CPU-local lines 0, 3, 4, and 7 are reserved. Peripheral source IDs
0..79 are monitored through three INTMTX status registers.

The exported Esp32C3* registration names are retained temporarily because
the shared esp32xx_irq.c and uart_esp32.cpp currently use those names on
both C3 and C6. The two MCU libraries are compiled separately.

@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
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
#include "esp32xx.h"
#include "esp32xx_intmtx.h"
#define ESP32C3_INTMTX_INTR_STATUS_0_REG    ESP32_INTMTX_INTR_STATUS_0_REG
#define ESP32C3_INTMTX_INTR_STATUS_1_REG    ESP32_INTMTX_INTR_STATUS_1_REG
#define ESP32C3_INTMTX_INTR_STATUS_2_REG    ESP32_INTMTX_INTR_STATUS_2_REG
#define ESP32C3_INTMTX_MAX_SOURCE           80U

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
volatile uint32_t g_Esp32C3LastSourceStatus2;
volatile uint32_t g_Esp32C3UnhandledSourceStatus0;
volatile uint32_t g_Esp32C3UnhandledSourceStatus1;
volatile uint32_t g_Esp32C3UnhandledSourceStatus2;
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
    uint32_t status2 = ESP32C3_INTMTX_INTR_STATUS_2_REG;
    bool handled_any = false;

    g_Esp32C3MtxDispatchCount++;
    g_Esp32C3LastSourceStatus0 = status0;
    g_Esp32C3LastSourceStatus1 = status1;
    g_Esp32C3LastSourceStatus2 = status2;

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

    // The C6 has sources 64..79, unlike the C3's two-status layout.
    for (uint32_t bit = 0U; bit < 32U; bit++)
    {
        uint32_t mask = 1UL << bit;
        uint32_t source = 64U + bit;
        if ((status2 & mask) != 0U && source < ESP32C3_INTMTX_MAX_SOURCE)
        {
            Esp32C3IrqHandler handler = s_Esp32C3SourceIrqHandlers[source];
            if (handler != 0)
            {
                handled_any = true;
                handler();
            }
            else
            {
                g_Esp32C3UnhandledSourceStatus2 |= mask;
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
