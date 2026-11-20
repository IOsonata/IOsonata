#!/usr/bin/env python3
"""Compile production HFXO ownership transitions with modeled clock backends.

The source functions are unchanged. Only hardware, interrupt state and
SoftDevice/MPSL calls are replaced with a deterministic host model.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"

MODEL = r"""
#include <assert.h>
#include <stdint.h>
#include <limits.h>
typedef enum { OSC_TYPE_RC, OSC_TYPE_XTAL, OSC_TYPE_TCXO } OSC_TYPE;
typedef enum {
	NRFX_HFXO_OWNER_NONE,
	NRFX_HFXO_OWNER_DIRECT,
	NRFX_HFXO_OWNER_SOFTDEVICE,
	NRFX_HFXO_OWNER_MPSL
} nRFxHfxoOwner_t;

static uint32_t s_nRFxHfxoReqCnt;
static bool s_bNRFxHfxoTransition;
static bool s_bNRFxHfxoPendingStop;
static nRFxHfxoOwner_t s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
static uint32_t s_Ipsr;
static uint32_t s_Mask;
static unsigned s_Starts, s_Stops;
static bool s_Stack, s_FailStop, s_FailStart;

uint32_t __get_IPSR(void) { return s_Ipsr; }
uint32_t DisableInterrupt(void)
{
	uint32_t saved = s_Mask;
	s_Mask = 1U;
	return saved;
}
void EnableInterrupt(uint32_t saved) { s_Mask = saved; }
bool nRFxClockStackActive(void) { return s_Stack; }
bool nRFxHfxoStart(nRFxHfxoOwner_t *pOwner)
{
	s_Starts++;
	if (s_FailStart)
	{
		return false;
	}
	*pOwner = s_Stack ? NRFX_HFXO_OWNER_MPSL : NRFX_HFXO_OWNER_DIRECT;
	return true;
}
bool nRFxHfxoStop(nRFxHfxoOwner_t Owner)
{
	s_Stops++;
	if (s_FailStop)
	{
		return false;
	}
	if (Owner == NRFX_HFXO_OWNER_MPSL ||
		Owner == NRFX_HFXO_OWNER_SOFTDEVICE)
	{
		return s_Stack;
	}
	return Owner == NRFX_HFXO_OWNER_NONE || !s_Stack;
}
"""

FIXTURE = r"""
void ResetModel(void)
{
	assert(s_Mask == 0U);
	s_Ipsr = 0U;
	s_Starts = s_Stops = 0U;
	s_Stack = false;
	s_FailStop = false;
	s_FailStart = false;
	s_nRFxHfxoReqCnt = 0U;
	s_bNRFxHfxoTransition = false;
	s_bNRFxHfxoPendingStop = false;
	s_nRFxHfxoOwner = NRFX_HFXO_OWNER_NONE;
}

int main(void)
{
	// A nested request releases in two steps; direct STOP has no ISR wait.
	ResetModel();
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	assert(s_Starts == 1U && s_nRFxHfxoReqCnt == 2U);
	s_Ipsr = 7U;
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	assert(s_nRFxHfxoReqCnt == 1U && s_Stops == 0U);
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	assert(s_nRFxHfxoReqCnt == 0U && s_Stops == 1U);
	assert(!s_bNRFxHfxoPendingStop && s_Mask == 0U);

	// An MPSL request stopped in an IRQ is released from thread context.
	ResetModel();
	s_Stack = true;
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	s_Ipsr = 11U;
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	assert(s_nRFxHfxoReqCnt == 0U && s_Stops == 0U);
	assert(s_bNRFxHfxoPendingStop);
	nRFxClockService();
	assert(s_Stops == 0U);
	s_Ipsr = 0U;
	nRFxClockService();
	assert(s_Stops == 1U && !s_bNRFxHfxoPendingStop);
	assert(s_nRFxHfxoOwner == NRFX_HFXO_OWNER_NONE);

	// A new consumer can cancel a deferred stop without a second START.
	ResetModel();
	s_Stack = true;
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	s_Ipsr = 5U;
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	s_Ipsr = 0U;
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	assert(s_Starts == 1U && s_Stops == 0U);
	assert(s_nRFxHfxoReqCnt == 1U && !s_bNRFxHfxoPendingStop);
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	assert(s_Stops == 1U && s_nRFxHfxoReqCnt == 0U);

	// A failed backend release must keep its pending ownership.
	ResetModel();
	s_Stack = true;
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	s_Ipsr = 4U;
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	s_Ipsr = 0U;
	s_FailStop = true;
	nRFxClockService();
	assert(s_bNRFxHfxoPendingStop);
	assert(s_nRFxHfxoOwner == NRFX_HFXO_OWNER_MPSL);
	s_FailStop = false;
	nRFxClockService();
	assert(!s_bNRFxHfxoPendingStop && s_Stops == 2U);

	// Direct ownership cannot be released through a newly active BLE stack.
	ResetModel();
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	s_Stack = true;
	s_Ipsr = 6U;
	assert(SystemCoreClockSelect(OSC_TYPE_RC, 0U));
	s_Ipsr = 0U;
	nRFxClockService();
	assert(s_bNRFxHfxoPendingStop);
	s_Stack = false;
	nRFxClockService();
	assert(!s_bNRFxHfxoPendingStop);

	// First acquisition can wait, and is therefore not allowed in an ISR.
	ResetModel();
	s_Ipsr = 3U;
	assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	assert(s_Starts == 0U && s_nRFxHfxoReqCnt == 0U);
	s_Ipsr = 0U;
	s_FailStart = true;
	assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U));
	assert(s_nRFxHfxoReqCnt == 0U && s_Mask == 0U);
	return 0;
}
"""


def production_function(source, signature):
	start = source.index(signature)
	open_brace = source.index("{", start)
	depth = 0
	for pos in range(open_brace, len(source)):
		if source[pos] == "{":
			depth += 1
		elif source[pos] == "}":
			depth -= 1
			if depth == 0:
				return source[start:pos + 1]
	raise ValueError("Incomplete function: " + signature)


def main():
	source = SOURCE.read_text(encoding="utf-8")
	clock_service = production_function(source, "void nRFxClockService(void)")
	clock_select = production_function(source, "bool SystemCoreClockSelect(")
	body = MODEL + clock_service + "\n" + clock_select + FIXTURE

	with tempfile.TemporaryDirectory(prefix="nordic-hfxo-") as directory:
		path = Path(directory)
		(src, exe) = (path / "hfxo.cpp", path / "hfxo")
		src.write_text(body, encoding="utf-8")
		subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17",
			"-Wall", "-Wextra", "-Werror", "-O2", str(src), "-o",
			str(exe)], check=True)
		subprocess.run([str(exe)], check=True)
	print("Nordic HFXO ISR and deferred-release host model PASS")


if __name__ == "__main__":
	main()
