#!/usr/bin/env python3
"""Check production USB startup supply selection and shutdown in a host model."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp").read_text()
config_start = source.index("#ifndef RA4USB_USE_INTERNAL_REGULATOR")
config_end = source.index("#endif", config_start) + len("#endif")
start = source.index("static bool Ra4UsbClockEnable(void)")
end_start = source.index("static void Ra4UsbClockDisable(void)", start)
end = source.index("\n}", end_start) + 2
code = """
#include <cassert>
#include <cstdint>
struct { uint32_t MSTPCRB; } mstp;
struct { uint16_t USBMC, SYSCFG; } usb;
#define R_MSTP (&mstp)
#define R_USB_FS0 (&usb)
#define __DSB() ((void)0)
enum { RA4USB_MSTP_MASK = 1 << 11, RA4USB_USBMC_VDCEN = 1 << 7,
	RA4USB_SYSCFG_SCKE = 1 << 10, RA4USB_SYSCFG_USBE = 1,
	RA4USB_WAIT_COUNT = 4096 };
""" + source[config_start:config_end] + "\n" + source[start:end] + """
int main()
{
	for (unsigned initial : {0U, 0x80U})
	{
		mstp.MSTPCRB = 0xFFFFFFFFU;
		usb.USBMC = initial | 1U;
		assert(Ra4UsbClockEnable());
		assert(usb.USBMC == (RA4USB_USE_INTERNAL_REGULATOR ? 0x81 : 1));
		assert(usb.SYSCFG == 0x401);
		assert((mstp.MSTPCRB & RA4USB_MSTP_MASK) == 0);
		Ra4UsbClockDisable();
		assert(usb.USBMC == 1 && usb.SYSCFG == 0);
		assert((mstp.MSTPCRB & RA4USB_MSTP_MASK) != 0);
	}
}
"""
code = "#include <initializer_list>\n" + code
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "supply.cpp").write_text(code)
    for opt in ("-O0", "-Os", "-O2"):
        for define in ([], ["-DRA4USB_USE_INTERNAL_REGULATOR=1"]):
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11",
                            "-Wall", "-Wextra", "-Werror", opt, *define,
                            str(path / "supply.cpp"), "-o", str(path / "supply")],
                           check=True)
            subprocess.run([str(path / "supply")], check=True)
        print("PASS", opt, "external/internal supply, stale VDCEN, shutdown")
