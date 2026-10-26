#!/usr/bin/env python3
"""Exercise the RA4M1 debug trace without stopping a live enumeration."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp").read_text()
start = source.index("struct Ra4UsbTraceEntry")
end = source.index("\n#else", start)
code = """
#include <cassert>
#include <cstdint>
static unsigned prints;
static int trace_printf(const char *, ...) { ++prints; return 0; }
#define printf trace_printf
struct { bool Started, Suspended; } s_Usb;
struct { uint16_t DCPCTR, SYSSTS0, INTSTS0; } regs;
#define R_USB_FS0 (&regs)
static uint32_t g_Ra4UsbStage;
static uint16_t g_Ra4UsbSyscfg, g_Ra4UsbIntsts0;
enum { RA4USB_INT_VBSTS = 0x80 };
""" + source[start:end] + """
int main()
{
	s_Usb.Started = true;
	regs.INTSTS0 = RA4USB_INT_VBSTS;
	regs.DCPCTR = 0x101;
	regs.SYSSTS0 = 1;
	for (unsigned i = 0; i < 32; ++i)
	{
		Ra4UsbTraceRecord((uint16_t)(0x1000 + i), 0xDF00);
		Ra4UsbTracePoll();
	}
	assert(prints == 0);
	assert(s_TraceCount == 16);
	assert(s_Trace[0].Status == 0x1000);
	assert(s_Trace[15].Status == 0x100F);
	assert(s_Trace[0].Dcp == 0x101 && s_Trace[0].Line == 1);
	s_Usb.Suspended = true;
	Ra4UsbTracePoll();
	assert(prints == 0);
	regs.INTSTS0 = 0;
	Ra4UsbTracePoll();
	assert(prints == 17 && s_TracePrinted == 16);
	Ra4UsbTracePoll();
	assert(prints == 17);
	s_Usb.Suspended = false;
	regs.INTSTS0 = RA4USB_INT_VBSTS;
	g_Ra4UsbStage = 6;
	Ra4UsbTracePoll();
	assert(prints == 17);
}
"""
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "trace.cpp").write_text(code)
    for opt in ("-O0", "-Os", "-O2"):
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall",
                        "-Wextra", "-Werror", opt, str(path / "trace.cpp"),
                        "-o", str(path / "trace")], check=True)
        subprocess.run([str(path / "trace")], check=True)
        print("PASS", opt, "attached/suspended silence, bounded records, unplug dump")
