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
struct { uint16_t DCPCTR, SYSSTS0, INTSTS0, USBADDR, USBREQ,
	USBVAL, USBINDX, USBLENG, CFIFOSEL, CFIFOCTR, DCPMAXP; } regs;
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
	regs.USBADDR = 7;
	regs.USBREQ = 0x680;
	regs.USBVAL = 0x100;
	regs.USBLENG = 18;
	regs.CFIFOSEL = 0x20;
	regs.CFIFOCTR = 0x8000;
	regs.DCPMAXP = 64;
	const uint8_t prefix[] = {0x12, 1, 0, 2, 0xef, 2, 1, 64};
	Ra4UsbTraceTx(nullptr, 0);
	Ra4UsbTraceTx(prefix, 3);
	for (unsigned i = 0; i < 64; ++i)
	{
		Ra4UsbTraceTx(prefix, sizeof(prefix));
		Ra4UsbTraceRecord((uint16_t)(0x1000 + i), 0xDF00);
		Ra4UsbTracePoll();
	}
	assert(prints == 0);
	assert(s_TxTraceCount == 8);
	assert(s_TxTrace[0].Length == 0 && s_TxTrace[0].Data[0] == 0);
	assert(s_TxTrace[1].Length == 3 && s_TxTrace[1].Data[3] == 0);
	for (unsigned i = 0; i < sizeof(prefix); ++i)
		assert(s_TxTrace[2].Data[i] == prefix[i]);
	assert(s_TxTrace[2].Length == 8 && s_TxTrace[2].Requested == 18);
	assert(s_TxTrace[2].Select == 0x20 && s_TxTrace[2].MaxPacket == 64);
	assert(s_TraceCount == 32);
	assert(s_Trace[0].Status == 0x1000);
	assert(s_Trace[15].Status == 0x100F);
	assert(s_Trace[31].Status == 0x101F);
	assert(s_Trace[0].Address == 7 && s_Trace[0].Request == 0x680);
	assert(s_Trace[0].Value == 0x100 && s_Trace[0].Length == 18);
	assert(s_Trace[0].Dcp == 0x101 && s_Trace[0].Line == 1);
	s_Usb.Suspended = true;
	Ra4UsbTracePoll();
	assert(prints == 0);
	regs.INTSTS0 = 0;
	Ra4UsbTracePoll();
	assert(prints == 41 && s_TracePrinted == 32 && s_TxTracePrinted == 8);
	Ra4UsbTracePoll();
	assert(prints == 41);
	s_Usb.Suspended = false;
	regs.INTSTS0 = RA4USB_INT_VBSTS;
	g_Ra4UsbStage = 6;
	Ra4UsbTracePoll();
	assert(prints == 41);
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
