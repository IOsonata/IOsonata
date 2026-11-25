#!/usr/bin/env python3
"""Run the production SAMD standby code against a host register model."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ARM/Microchip/SAMD/src/system_sleep_samd.c"

DEVICE = r"""
#ifndef SAMDXXX_H
#define SAMDXXX_H
#include <stdint.h>
typedef struct { volatile uint32_t SCR; } MockScb;
extern MockScb g_Scb;
#define SCB (&g_Scb)
#define SCB_SCR_SLEEPDEEP_Msk (1U << 2)
#define SCB_SCR_SLEEPONEXIT_Msk (1U << 1)
uint32_t MockIpsr(void);
void MockWfi(void);
#define __get_IPSR() MockIpsr()
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define __WFI() MockWfi()
#endif
"""
INTERRUPT = r"""
#ifndef MOCK_INTERRUPT_H
#define MOCK_INTERRUPT_H
#include <stdint.h>
uint32_t DisableInterrupt(void);
void EnableInterrupt(uint32_t State);
#endif
"""
FIXTURE = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "samdxxx.h"
MockScb g_Scb;
static unsigned g_Wfi, g_Arm, g_Done, g_Disable, g_Enable, g_Ipsr;
void SystemSleep(void);
uint32_t MockIpsr(void) { return g_Ipsr; }
uint32_t DisableInterrupt(void) { g_Disable++; return 0U; }
void EnableInterrupt(uint32_t State) { assert(State == 0U); g_Enable++; }
void MockWfi(void)
{
	assert((g_Scb.SCR & SCB_SCR_SLEEPDEEP_Msk) != 0U);
	assert((g_Scb.SCR & SCB_SCR_SLEEPONEXIT_Msk) == 0U);
	g_Wfi++;
}
void SamdStandbyWakeArm(void) { g_Arm++; }
void SamdStandbyWakeDone(void) { g_Done++; }
uint32_t SystemCoreClockGet(void) { return 48000000U; }
static void Reset(void)
{
	g_Wfi = g_Arm = g_Done = g_Disable = g_Enable = g_Ipsr = 0U;
	g_Scb.SCR = 0x100U | SCB_SCR_SLEEPONEXIT_Msk;
}
int main(void)
{
	Reset();
	g_Ipsr = 1U;
	SystemSleep();
	assert(g_Wfi == 0U && g_Disable == 0U);
	Reset();
	SystemSleep();
	assert(g_Wfi == 1U && g_Arm == 1U && g_Done == 1U);
	assert(g_Disable == 1U && g_Enable == 1U);
	assert(g_Scb.SCR == (0x100U | SCB_SCR_SLEEPONEXIT_Msk));
	puts("SAMD standby entry, SCR restoration and interrupt context refusal PASS");
	return 0;
}
"""

def main():
    with tempfile.TemporaryDirectory(prefix="samd-standby-") as temp:
        folder = Path(temp)
        (folder / "coredev").mkdir()
        (folder / "samdxxx.h").write_text(DEVICE)
        (folder / "coredev/interrupt.h").write_text(INTERRUPT)
        (folder / "fixture.c").write_text(FIXTURE)
        binary = folder / "test"
        subprocess.run([
            os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
            "-Werror", "-O2", "-I", str(folder), "-I", str(ROOT / "include"),
            str(SOURCE), str(folder / "fixture.c"), "-o", str(binary)
        ], check=True)
        subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    main()
