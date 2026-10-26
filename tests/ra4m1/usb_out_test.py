#!/usr/bin/env python3
"""Empty OUT FIFO must be armed before waiting for received data."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp').read_text()
def function(signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2]
constants = source[source.index('enum\n{'):source.index('typedef struct __Ra4UsbRegistration')]
code = r'''
#include <cassert>
#include <cstdint>
#define RA4USB_RXTRACE(...) ((void)0)
#include <initializer_list>
''' + constants + r'''
enum { ISO = 1, BULK = 2 };
struct { struct { int Type; } Pipe[10]; } s_Usb;
struct {
	uint16_t CFIFOSEL, CFIFOCTR, BRDYSTS, NRDYSTS, BRDYENB, NRDYENB;
	uint8_t CFIFOLL;
} regs;
#define R_USB_FS0 (&regs)
static uint16_t pid;
static bool pidOk = true;
static bool Ra4UsbSetPid(uint8_t, uint16_t value)
{
	if (!pidOk) { return false; }
	pid = value; return true;
}
''' + function('static inline uint16_t Ra4UsbPipeBit(') + '\n' + function('static bool Ra4UsbSelectFifo(') + '\n' + function('static bool Ra4UsbArmOut(') + '\n' + function('static int Ra4UsbFifoRead(') + r'''
int main()
{
	for (uint8_t pipe : {0, 3, 6})
	{
		regs = {}; pid = RA4USB_PID_NAK;
		s_Usb.Pipe[pipe].Type = BULK;
		assert(Ra4UsbArmOut(pipe)); // FRDY=0: no received packet yet.
		assert(pid == RA4USB_PID_BUF);
		assert(regs.BRDYENB & (1U << pipe));
		assert(regs.NRDYENB & (1U << pipe));
		assert((regs.CFIFOSEL & RA4USB_CFIFO_ISEL) == 0);
		uint8_t data[7] = {};
		// A read still requires FRDY; arming must not weaken data access.
		regs.CFIFOCTR = 0;
		assert(Ra4UsbFifoRead(pipe, data, sizeof(data)) == -1);
		// Host can now deliver the seven-byte CDC SET_LINE_CODING payload.
		regs.CFIFOCTR = RA4USB_CFIFO_FRDY | 7;
		regs.CFIFOLL = 0x5a;
		assert(Ra4UsbFifoRead(pipe, data, sizeof(data)) == 7);
		for (uint8_t byte : data) { assert(byte == 0x5a); }
	}
	regs = {}; s_Usb.Pipe[1].Type = ISO;
	assert(Ra4UsbArmOut(1));
	assert(regs.BRDYENB == 2 && regs.NRDYENB == 0);
	pidOk = false; regs = {};
	assert(!Ra4UsbArmOut(0)); assert(regs.BRDYENB == 0);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / 'out.cpp').write_text(code)
    for opt in ('-O0', '-Os', '-O2'):
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall',
                        '-Wextra', '-Werror', opt, str(p / 'out.cpp'),
                        '-o', str(p / 'out')], check=True)
        subprocess.run([str(p / 'out')], check=True)
        print('PASS', opt, 'empty OUT arm, FRDY-protected read, EP0/data/ISO pipes, failure')
    old = code.replace('Ra4UsbSelectFifo(Pipe, false, 0U, false)', 'Ra4UsbSelectFifo(Pipe, false)')
    (p / 'old.cpp').write_text(old)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', str(p / 'old.cpp'),
                    '-o', str(p / 'old')], check=True)
    result = subprocess.run([str(p / 'old')], capture_output=True, text=True)
    assert result.returncode != 0 and 'Ra4UsbArmOut(pipe)' in result.stderr
    print('PASS regression: previous OUT arm cannot accept a packet with FRDY=0')
