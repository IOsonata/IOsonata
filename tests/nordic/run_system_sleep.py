#!/usr/bin/env python3
"""Compile and execute the real Nordic SystemSleep functions on a host model.

Extract only the production entry points so the host can substitute CPU
registers and SoftDevice event wait without including target CMSIS headers.
This does not validate low-power hardware or target ABI.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BACKENDS = (
	("nrf52", "ARM/Nordic/nRF52/src/system_power_nrf52.cpp", False),
	("nrf52_sd", "ARM/Nordic/nRF52/src/system_power_nrf52.cpp", True),
	("nrfx", "ARM/Nordic/src/system_power_nrfx.cpp", False),
)

MODEL = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t SCR; } MockScb;
static MockScb g_Scb;
#define SCB (&g_Scb)
#define SCB_SCR_SLEEPDEEP_Msk (1U << 2)
#define SCB_SCR_SLEEPONEXIT_Msk (1U << 1)
static uint32_t s_Ipsr, s_Mask;
static unsigned s_Disable, s_Enable, s_Wfi, s_SdWait, s_ClockService;
void nRFxClockService(void)
{
	assert(s_Ipsr == 0U);
	s_ClockService++;
}
static bool s_SdOn;
uint32_t __get_IPSR(void) { return s_Ipsr; }
uint32_t DisableInterrupt(void)
{
	uint32_t state = s_Mask;
	s_Disable++;
	s_Mask = 1U;
	return state;
}
void EnableInterrupt(uint32_t state)
{
	s_Enable++;
	s_Mask = state;
}
void MockWfi(void)
{
	assert(s_Mask == 1U);
	assert((g_Scb.SCR & (SCB_SCR_SLEEPDEEP_Msk |
		SCB_SCR_SLEEPONEXIT_Msk)) == 0U);
	s_Wfi++;
}
#define __WFI() MockWfi()
#define __DSB() ((void)0)
#define __ISB() ((void)0)
bool SysPwrSdEnabled(void) { return s_SdOn; }
uint32_t sd_app_evt_wait(void) { s_SdWait++; return 0U; }
"""


NORDIC_GPIO_WAKE = r"""
#include <stddef.h>
typedef struct { uint32_t PIN_CNF[32]; } NRF_GPIO_Type;
#define GPIO_PIN_CNF_SENSE_Msk (3U << 16)
static NRF_GPIO_Type s_TestGpio;
NRF_GPIO_Type *nRFGpioGetReg(int PortNo)
{
	return PortNo == 0 ? &s_TestGpio : NULL;
}
typedef enum {
	IOPINSENSE_DISABLE,
	IOPINSENSE_LOW_TRANSITION,
	IOPINSENSE_HIGH_TRANSITION
} IOPINSENSE;
static int s_SysPwrSleepPort = -1;
static int s_SysPwrSleepPin = -1;
static IOPINSENSE s_SysPwrSleepSense = IOPINSENSE_DISABLE;
static int s_IrqAllocated;
static int s_IrqReleased;
static bool s_AllowIrq = true;
static void SysPwrSleepWake(int IntNo, void *pCtx)
{
	(void)IntNo;
	(void)pCtx;
}
int IOPinAllocateInterrupt(int Prio, int PortNo, int PinNo, IOPINSENSE Sense,
	void (*Handler)(int, void *), void *pCtx)
{
	assert(s_Mask == 1U);
	assert(Prio == 2 && PortNo == 0 && PinNo == 11);
	assert(Sense == IOPINSENSE_LOW_TRANSITION && Handler != NULL);
	(void)pCtx;
	if (!s_AllowIrq)
	{
		// Exercise the cleanup path even if a failed allocation touched SENSE.
		s_TestGpio.PIN_CNF[PinNo] &= ~GPIO_PIN_CNF_SENSE_Msk;
		return -1;
	}
	s_IrqAllocated++;
	return 3;
}
void IOPinDisableInterrupt(int Chan)
{
	assert(s_Mask == 1U && Chan == 3);
	s_TestGpio.PIN_CNF[11] &= ~GPIO_PIN_CNF_SENSE_Msk;
	s_IrqReleased++;
}
"""

FIXTURE = r"""
int main(void)
{
	unsigned i;

	for (i = 0U; i < 2U; i++)
	{
		s_Ipsr = 0U;
		s_SdOn = false;
		s_Mask = i;
		s_Disable = s_Enable = s_Wfi = s_SdWait = s_ClockService = 0U;
		g_Scb.SCR = 0x100U | SCB_SCR_SLEEPDEEP_Msk |
			SCB_SCR_SLEEPONEXIT_Msk;
		SystemSleep();
		assert(s_Wfi == 1U && s_Disable == 1U && s_Enable == 1U);
		assert(s_ClockService == 1U);
		assert(s_SdWait == 0U && s_Mask == i);
		assert(g_Scb.SCR == (0x100U | SCB_SCR_SLEEPDEEP_Msk |
			SCB_SCR_SLEEPONEXIT_Msk));
		s_Ipsr = 11U;
		SystemSleep();
		assert(s_ClockService == 1U);
		assert(s_Wfi == 1U && s_Disable == 1U && s_Enable == 1U);
	}
#ifdef SOFTDEVICE_PRESENT
	s_Ipsr = 0U;
	s_SdOn = true;
	s_Mask = 0U;
	s_Disable = s_Enable = s_Wfi = s_SdWait = 0U;
	g_Scb.SCR = 0x106U;
	SystemSleep();
	assert(s_Wfi == 0U && s_SdWait == 1U);
#ifdef NRF_GPIO_SLEEP_MODEL
	assert(s_Disable == 2U && s_Enable == 2U && s_Mask == 0U);
#else
	assert(s_Disable == 0U && s_Enable == 0U && s_Mask == 0U);
#endif
	assert(g_Scb.SCR == 0x106U);
#endif
#if defined(SOFTDEVICE_PRESENT) && defined(NRF_GPIO_SLEEP_MODEL)
	// A SoftDevice wait may not silently undo caller PRIMASK.
	s_Ipsr = 0U;
	s_SdOn = true;
	s_Mask = 1U;
	s_Disable = s_Enable = s_Wfi = s_SdWait = 0U;
	SystemSleep();
	assert(s_Disable == 1U && s_Enable == 1U);
	assert(s_Mask == 1U && s_Wfi == 0U && s_SdWait == 0U);
#endif
#ifdef NRF_GPIO_SLEEP_MODEL
	s_Ipsr = 0U;
	s_SdOn = false;
	s_Mask = 0U;
	s_Disable = s_Enable = s_Wfi = s_SdWait = 0U;
	s_IrqAllocated = s_IrqReleased = 0;
	s_SysPwrSleepPort = 0;
	s_SysPwrSleepPin = 11;
	s_SysPwrSleepSense = IOPINSENSE_LOW_TRANSITION;
	g_Scb.SCR = 0x106U;
	s_TestGpio.PIN_CNF[11] = 0x4000U | (1U << 16);
	SystemSleep();
	assert(s_TestGpio.PIN_CNF[11] == (0x4000U | (1U << 16)));
	SystemSleep();
	assert(s_TestGpio.PIN_CNF[11] == (0x4000U | (1U << 16)));
	assert(s_Wfi == 2U && s_IrqAllocated == 2 && s_IrqReleased == 2);
	assert(s_Disable == 2U && s_Enable == 2U && s_Mask == 0U);
	assert(g_Scb.SCR == 0x106U);

	s_AllowIrq = false;
	SystemSleep();
	assert(s_TestGpio.PIN_CNF[11] == (0x4000U | (1U << 16)));
	assert(s_Wfi == 2U && s_IrqAllocated == 2 && s_IrqReleased == 2);
	assert(s_Disable == 3U && s_Enable == 3U);
#ifdef SOFTDEVICE_PRESENT
	s_SdOn = true;
	s_AllowIrq = true;
	s_IrqAllocated = s_IrqReleased = 0;
	s_Disable = s_Enable = s_Wfi = s_SdWait = 0U;
	SystemSleep();
	assert(s_SdWait == 1U && s_Wfi == 0U);
	assert(s_TestGpio.PIN_CNF[11] == (0x4000U | (1U << 16)));
	assert(s_IrqAllocated == 1 && s_IrqReleased == 1);
	assert(s_Disable == 2U && s_Enable == 2U && s_Mask == 0U);
	s_SdOn = false;
#endif
#endif
	return 0;
}
"""


def sleep_function(source):
	start = source.index("void SystemSleep(void)")
	brace = source.index("{", start)
	level = 0
	for pos in range(brace, len(source)):
		if source[pos] == "{":
			level += 1
		elif source[pos] == "}":
			level -= 1
			if level == 0:
				return source[start:pos + 1]
	raise RuntimeError("SystemSleep body incomplete")


def main():
	with tempfile.TemporaryDirectory(prefix="nordic-sleep-") as directory:
		folder = Path(directory)
		for name, relative, softdevice in BACKENDS:
			source = (ROOT / relative).read_text(encoding="utf-8")
			body = MODEL
			if name.startswith("nrf52") or name == "nrfx":
				body += NORDIC_GPIO_WAKE
			body += sleep_function(source) + FIXTURE
			test = folder / (name + ".cpp")
			output = folder / name
			test.write_text(body, encoding="utf-8")
			cmd = [os.environ.get("CXX", "g++"), "-std=c++17",
				"-Wall", "-Wextra", "-Werror", "-O2"]
			if name.startswith("nrf52") or name == "nrfx":
				cmd.append("-DNRF_GPIO_SLEEP_MODEL=1")
			if softdevice:
				cmd.append("-DSOFTDEVICE_PRESENT=1")
			cmd += [str(test), "-o", str(output)]
			subprocess.run(cmd, check=True)
			subprocess.run([str(output)], check=True)
			print(name + ": SystemSleep host model PASS", flush=True)
	print("Nordic power sleep models PASS")


if __name__ == "__main__":
	main()
