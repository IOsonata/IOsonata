#!/usr/bin/env python3
"""Check BEMP completion against the USBFS transmit-buffer ownership bit."""
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
#include <initializer_list>
''' + constants + r'''
enum { USB_CTRLR_EVT_XFER_CMPL };
struct Ra4UsbPipe_t { bool Busy, In; uint8_t *pBuffer; uint16_t Length; };
struct { Ra4UsbPipe_t Pipe[10]; } s_Usb;
struct { uint16_t BEMPENB, NRDYENB, DCPCTR, PIPE_CTR[9]; } regs;
#define R_USB_FS0 (&regs)
static unsigned completions, nakCalls;
static bool refill;
static uint8_t data[64];
static bool Ra4UsbSetPid(uint8_t pipe, uint16_t pid)
{
	assert(pid == RA4USB_PID_NAK);
	regs.PIPE_CTR[pipe - 1] = pid;
	++nakCalls; return true;
}
static void Ra4UsbNotify(uint8_t pipe, int event, uint16_t length)
{
	assert(event == USB_CTRLR_EVT_XFER_CMPL && length == 64);
	assert(!s_Usb.Pipe[pipe].Busy && s_Usb.Pipe[pipe].pBuffer == nullptr);
	++completions;
	if (refill)
	{
		s_Usb.Pipe[pipe] = {true, true, data, 64};
		regs.BEMPENB |= 1U << pipe;
		regs.NRDYENB |= 1U << pipe;
		regs.PIPE_CTR[pipe - 1] = RA4USB_PID_BUF;
	}
}
''' + function('static inline uint16_t Ra4UsbPipeBit(') + '\n' + function('static inline volatile uint16_t *Ra4UsbPipeCtr(') + '\n' + function('static void Ra4UsbHandleIn(') + r'''
int main()
{
	for (uint8_t pipe : {1, 2, 3, 4, 5})
	{
		const uint16_t bit = 1U << pipe;
		completions = nakCalls = 0;
		refill = false;
		s_Usb.Pipe[pipe] = {true, true, data, 64};
		regs.BEMPENB = regs.NRDYENB = bit;
		regs.PIPE_CTR[pipe - 1] = RA4USB_INBUFM | RA4USB_PID_BUF;
		Ra4UsbHandleIn(pipe);
		assert(completions == 0 && nakCalls == 0);
		assert(s_Usb.Pipe[pipe].Busy && s_Usb.Pipe[pipe].pBuffer == data);
		assert(s_Usb.Pipe[pipe].Length == 64);
		assert(regs.BEMPENB == bit && regs.NRDYENB == bit);
		regs.PIPE_CTR[pipe - 1] = RA4USB_PID_BUF;
		refill = true;
		Ra4UsbHandleIn(pipe);
		assert(completions == 1 && nakCalls == 1);
		assert(s_Usb.Pipe[pipe].Busy && regs.BEMPENB == bit);
		// The synchronous callback's next transfer must remain armed.
		assert(regs.PIPE_CTR[pipe - 1] == RA4USB_PID_BUF);
	}
	// Interrupt pipes 6-9 do not use the pipe 1-5 INBUFM gate.
	refill = false;
	for (uint8_t pipe : {6, 7, 8, 9})
	{
		completions = nakCalls = 0;
		s_Usb.Pipe[pipe] = {true, true, data, 64};
		regs.PIPE_CTR[pipe - 1] = RA4USB_INBUFM | RA4USB_PID_BUF;
		Ra4UsbHandleIn(pipe);
		assert(completions == 1 && nakCalls == 1);
	}
	// A stale BEMP for an idle pipe must never complete a transfer.
	completions = 0;
	s_Usb.Pipe[3].Busy = false;
	regs.PIPE_CTR[2] = RA4USB_INBUFM;
	Ra4UsbHandleIn(3); assert(completions == 0);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / 'in.cpp').write_text(code)
    for opt in ('-O0', '-Os', '-O2'):
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall',
                        '-Wextra', '-Werror', opt, str(p / 'in.cpp'),
                        '-o', str(p / 'in')], check=True)
        subprocess.run([str(p / 'in')], check=True)
        print('PASS', opt, 'early BEMP, empty buffer, callback rearm, interrupt pipes, stale BEMP')
