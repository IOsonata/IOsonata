#!/usr/bin/env python3
"""Check live semihosting output without suspend/unplug/capture gates."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp').read_text()
start = source.index('#if !defined(NDEBUG)\n// Live bring-up')
end = source.index('alignas(4) static uint8_t s_Ep0In', start)
trace = source[start:end]
code = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <string>
static unsigned prints, flushes, mask;
static std::string output;
static uint32_t DisableInterrupt() { unsigned old = mask; mask = 1; return old; }
static void EnableInterrupt(uint32_t old) { mask = old; }
static int trace_printf(const char *format, ...)
{
	assert(mask == 1);
	char text[512];
	va_list args; va_start(args, format);
	int n = vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	assert(n >= 0 && (unsigned)n < sizeof(text));
	output += text; ++prints; return n;
}
static int trace_fflush(FILE *)
{
	assert(mask == 1); ++flushes; assert(flushes == prints); return 0;
}
#define printf trace_printf
#define fflush trace_fflush
struct { bool Started, Suspended; } s_Usb;
struct { uint16_t DCPCTR, SYSSTS0, INTSTS0, USBADDR, CFIFOSEL,
	CFIFOCTR, DCPMAXP, SYSCFG; } regs;
#define R_USB_FS0 (&regs)
static uint32_t g_Ra4UsbStage;
enum { RA4USB_INT_VBSTS = 0x80 };
''' + trace + r'''
int main()
{
	s_Usb.Started = true;
	regs.INTSTS0 = 0x80;
	const uint8_t bytes[] = {0x12, 1, 0, 2, 0xef, 2, 1, 64};
	Ra4UsbTraceRecord(0x28a9, 0xdf00);
	assert(prints == 1 && flushes == 1 && mask == 0);
	assert(output.find("irq status=28a9 enable=df00") != std::string::npos);
	Ra4UsbTraceTx(bytes, sizeof(bytes));
	assert(prints == 2 && output.find("bytes=12 01 00 02 ef 02 01 40") != std::string::npos);
	// Zero-length transfers do not dereference a null data pointer.
	Ra4UsbTraceTx(nullptr, 0);
	assert(output.find("bytes=00 00 00 00 00 00 00 00") != std::string::npos);
	for (unsigned i = 0; i < 64; ++i) { Ra4UsbTraceRecord(0x28a9, 0xdf00); }
	assert(prints == 67); // No full-capture cutoff.
	Ra4UsbTracePoll(); assert(prints == 69); // Attached and active.
	s_Usb.Suspended = true;
	mask = 1; // ISR/critical-section caller keeps its original mask.
	Ra4UsbTracePoll(); assert(prints == 71 && mask == 1);
	regs.INTSTS0 = 0;
	Ra4UsbTracePoll(); assert(prints == 73);
	assert(output.find("VBUS=0") != std::string::npos);
	assert(prints == flushes);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / 'trace.cpp').write_text(code)
    for opt in ('-O0', '-Os', '-O2'):
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall',
                        '-Wextra', '-Werror', opt, str(p / 'trace.cpp'),
                        '-o', str(p / 'trace')], check=True)
        subprocess.run([str(p / 'trace')], check=True)
        print('PASS', opt, 'immediate trace, flush, IRQ mask preservation, no capture gate')
    (p / 'release.cpp').write_text(trace + '\nvoid test() { RA4USB_LIVE("hidden"); Ra4UsbTracePoll(); Ra4UsbTraceRecord(0, 0); Ra4UsbTraceTx(nullptr, 0); }\n')
    result = subprocess.run([os.environ.get('CXX', 'c++'), '-DNDEBUG', '-E',
                             '-P', str(p / 'release.cpp')], check=True,
                            capture_output=True, text=True)
    assert 'printf' not in result.stdout and 'hidden' not in result.stdout
    print('PASS Release trace compiles out')
