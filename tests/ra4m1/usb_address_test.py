#!/usr/bin/env python3
"""Exercise the production USBFS interrupt handler for hardware SET_ADDRESS."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp").read_text()
constants = source[source.index("enum\n{"):source.index("typedef struct __Ra4UsbRegistration")]
start = source.index("static bool Ra4UsbHandleOneInterrupt(void)")
end = source.index("static void Ra4UsbInterrupt(", start)
code = """
#include <cassert>
#include <cstdint>
enum UsbCtrlrEvtType_t { USB_CTRLR_EVT_ADDRESS, USB_CTRLR_EVT_SOF,
	USB_CTRLR_EVT_RESUME, USB_CTRLR_EVT_SUSPEND };
struct UsbCtrlrEvt_t { UsbCtrlrEvtType_t Type; uint8_t Address; uint16_t FrameNo; };
struct { uint16_t INTSTS0, INTENB0, USBADDR, FRMNUM, BRDYSTS, BRDYENB,
	BEMPSTS, BEMPENB, NRDYSTS, NRDYENB; } regs;
#define R_USB_FS0 (&regs)
struct { bool Suspended; } s_Usb;
static unsigned addressEvents, resets;
static uint8_t coreAddress;
static void UsbDevProcessEvent(int dev, const UsbCtrlrEvt_t *event)
{
	assert(dev == 0);
	if (event->Type == USB_CTRLR_EVT_ADDRESS)
	{
		++addressEvents;
		coreAddress = event->Address;
	}
}
static void Ra4UsbReset() { ++resets; coreAddress = 0; }
static void Ra4UsbBusEvent(UsbCtrlrEvtType_t) {}
static void UsbProcessQue(int) {}
static void Ra4UsbHandleBrdy(uint16_t) {}
static void Ra4UsbHandleBemp(uint16_t) {}
static void Ra4UsbHandleNrdy(uint16_t) {}
static void Ra4UsbHandleControlStage(uint16_t) {}
""" + constants + source[start:end] + """
int main()
{
	regs.INTENB0 = 0xdf00;
	// Hardware trace: addressed state can coexist with a pending SETUP.
	regs.INTSTS0 = 0x30a8;
	regs.USBADDR = 7;
	assert(Ra4UsbHandleOneInterrupt());
	assert(addressEvents == 1 && coreAddress == 7);
	assert((regs.INTSTS0 & RA4USB_INT_VALID) != 0);
	assert((regs.INTSTS0 & RA4USB_INT_DVST) == 0);
	// Readdressing and reserved high bits must not corrupt the core address.
	regs.INTSTS0 = 0x10a0;
	regs.USBADDR = 0x807f;
	assert(Ra4UsbHandleOneInterrupt());
	assert(addressEvents == 2 && coreAddress == 127);
	// Suspend is not a new address assignment.
	regs.INTSTS0 = 0x10e0;
	assert(Ra4UsbHandleOneInterrupt());
	assert(addressEvents == 2 && s_Usb.Suspended);
	regs.INTSTS0 = 0x1090;
	assert(Ra4UsbHandleOneInterrupt());
	assert(resets == 1 && coreAddress == 0 && addressEvents == 2);
	regs.INTSTS0 = 0;
	assert(!Ra4UsbHandleOneInterrupt());
}
"""
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "address.cpp").write_text(code)
    for opt in ("-O0", "-Os", "-O2"):
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall",
                        "-Wextra", "-Werror", opt, str(path / "address.cpp"),
                        "-o", str(path / "address")], check=True)
        subprocess.run([str(path / "address")], check=True)
        print("PASS", opt, "hardware address event, pending SETUP, reset, suspend")
