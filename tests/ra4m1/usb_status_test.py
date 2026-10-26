#!/usr/bin/env python3
"""Check immediate and deferred EP0 status completion from PID=NAK."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp").read_text()

def function(signature):
    start = source.index(signature)
    return source[start:source.index("\n}", start) + 2]

constants = source[source.index("enum\n{"):source.index("typedef struct __Ra4UsbRegistration")]
code = """
#include <cassert>
#include <cstdint>
#define RA4USB_LIVE(...) ((void)0)
enum { USB_ENDPADDR_DIR_IN = 0x80, USB_ENDPADDR_DIR_OUT = 0,
	USB_CTRLR_XFER_SUCCESS = 0 };
struct { uint16_t INTSTS0, DCPCTR; } regs;
#define R_USB_FS0 (&regs)
struct { bool Started, Closing, Ep0Halted, Ep0StatusPending, Ep0InBusy;
	uint8_t Ep0StatusAddr; } s_Usb;
static unsigned completions;
static uint32_t DisableInterrupt() { return 0; }
static void EnableInterrupt(uint32_t) {}
static void Ra4UsbHandleSetup(uint16_t) {}
static void Ra4UsbControlComplete(uint8_t, uint8_t *, uint16_t, int)
{
	assert(regs.DCPCTR == 5); // CCPL and PID=BUF before reporting completion
	++completions;
}
""" + constants + function("static void Ra4UsbHandleControlStage(") + "\n" + function("bool UsbCtrlrEp0Status(") + """
int main()
{
	s_Usb.Started = true;
	// No-data write: SET_CONFIGURATION / SET_CONTROL_LINE_STATE.
	regs.INTSTS0 = RA4USB_CTSQ_WRND;
	regs.DCPCTR = 0x40;
	assert(UsbCtrlrEp0Status(0, USB_ENDPADDR_DIR_IN));
	assert(completions == 1 && !s_Usb.Ep0StatusPending);
	// Deferred control-write status after the last OUT data packet.
	regs.INTSTS0 = RA4USB_CTSQ_WRDS;
	regs.DCPCTR = 0;
	assert(UsbCtrlrEp0Status(0, USB_ENDPADDR_DIR_IN));
	assert(s_Usb.Ep0StatusPending && completions == 1);
	Ra4UsbHandleControlStage(RA4USB_CTSQ_WRSS);
	assert(completions == 2 && !s_Usb.Ep0StatusPending);
	// Control-read status remains supported.
	regs.INTSTS0 = RA4USB_CTSQ_RDSS;
	assert(UsbCtrlrEp0Status(0, USB_ENDPADDR_DIR_OUT));
	assert(completions == 3);
	assert(!UsbCtrlrEp0Status(1, USB_ENDPADDR_DIR_OUT));
	s_Usb.Ep0Halted = true;
	assert(!UsbCtrlrEp0Status(0, USB_ENDPADDR_DIR_OUT));
}
"""
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "status.cpp").write_text(code)
    for opt in ("-O0", "-Os", "-O2"):
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall",
                        "-Wextra", "-Werror", opt, str(path / "status.cpp"),
                        "-o", str(path / "status")], check=True)
        subprocess.run([str(path / "status")], check=True)
        print("PASS", opt, "no-data/write/read status, PID=BUF before completion")
