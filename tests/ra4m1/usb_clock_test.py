#!/usr/bin/env python3
"""Check RA4M1 USB clock reporting with the production CMSIS register layout."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PORT = ROOT / "ARM/Renesas/RA4/RA4M1"
source = (PORT / "src/system_ra4m1.c").read_text()
header = (PORT / "include/R7FA4M1AB.h").read_text()
end = header.index("} R_SYSTEM_Type;") + len("} R_SYSTEM_Type;")
layout = header[header.rfind("typedef struct", 0, end):end]


def function(name):
    start = source.index(name + "(")
    start = source.rfind("\n", 0, start) + 1
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


preamble = """
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
/* Host stimulus may write hardware status fields. */
#define __IM volatile
#define __IOM volatile
#define __OM volatile
"""
harness = """
static R_SYSTEM_Type regs;
#define R_SYSTEM (&regs)
#define RA4M1_HOCOCR2 (R_SYSTEM->HOCOCR2)
#define RA4M1_OSCSF_HOCO 1U
#define RA4M1_OSCSF_MOSC 8U
#define RA4M1_OSCSF_PLL 32U
#define RA4M1_CK_HOCO 0U
#define RA4M1_CK_MOCO 1U
#define RA4M1_CK_LOCO 2U
#define RA4M1_CK_MOSC 3U
#define RA4M1_CK_SOSC 4U
#define RA4M1_CK_PLL 5U
#define RA4M1_MOCO_HZ 8000000U
#define RA4M1_LOCO_HZ 32768U
static uint32_t s_MainOscFreq = 12000000U;
static struct { bool bUSBClk; } g_McuOsc = {true};
static uint32_t primask;
static uint32_t DisableInterrupt(void) { uint32_t old = primask; primask = 1; return old; }
static void EnableInterrupt(uint32_t old) { primask = old; }
_Static_assert(offsetof(R_SYSTEM_Type, USBCKCR_ALT) == 0xD0, "RA4M1 USB selector");
_Static_assert(offsetof(R_SYSTEM_Type, USBCKCR) == 0x74, "Other-family selector");
_Static_assert(offsetof(R_SYSTEM_Type, HOCOCR2) == 0x37, "HOCO frequency");
"""
main = """
int main(void)
{
	/* A valid HOCO at 48 MHz, with the unrelated selector left at reset. */
	regs.HOCOCR2 = 0x20;
	regs.OSCSF = 1;
	regs.USBCKCR_ALT = 1;
	assert(SystemUsbClockGet() == 48000000U);
	assert(primask == 0);
	regs.USBCKCR = 7;
	assert(SystemUsbClockGet() == 48000000U);
	regs.HOCOCR = 1;
	assert(SystemUsbClockGet() == 0);
	regs.HOCOCR = 0;
	regs.OSCSF = 0;
	assert(SystemUsbClockGet() == 0);
	regs.OSCSF = 1;
	regs.HOCOCR2 = 0x10;
	assert(SystemUsbClockGet() == 0);
	regs.HOCOCR2 = 0x20;
	regs.HOCOUTCR = 1;
	assert(SystemUsbClockGet() == 0);
	regs.HOCOUTCR = 0;
	g_McuOsc.bUSBClk = false;
	assert(SystemUsbClockGet() == 0);
	g_McuOsc.bUSBClk = true;
	/* PLL at 48 MHz must use the same selector even if 0x74 says HOCO. */
	regs.USBCKCR_ALT = 0;
	regs.USBCKCR = 1;
	regs.HOCOCR = 1;
	regs.OSCSF = 0x28;
	regs.PLLCCR2 = 0x47;
	assert(SystemUsbClockGet() == 48000000U);
	regs.PLLCR = 1;
	assert(SystemUsbClockGet() == 0);
	regs.PLLCR = 0;
	primask = 1;
	assert(SystemUsbClockGet() == 48000000U);
	assert(primask == 1);
	puts("PASS: CMSIS offsets, HOCO/PLL USB selection, invalid clocks, interrupt state");
}
"""
code = preamble + layout + harness + "\n".join(
    function(name) for name in
    ("Ra4m1HocoHz", "Ra4m1PllHz", "Ra4m1SourceHz", "SystemUsbClockGet")
) + main
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "clock.c").write_text(code)
    for opt in ("-O0", "-Os", "-O2"):
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                        "-Werror", opt, str(path / "clock.c"), "-o", str(path / "clock")],
                       check=True)
        subprocess.run([str(path / "clock")], check=True)
