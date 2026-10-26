#!/usr/bin/env python3
"""Exercise OUT buffer requests, completion, backpressure and callback teardown."""
from pathlib import Path
import os
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
source = (root / 'ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp').read_text()
def function(sig):
    i = source.index(sig)
    return source[i:source.index('\n}', i) + 2]
code = r'''
#include <cassert>
#include <cstdint>
#define RA4USB_RXTRACE(...) ((void)0)
enum { USB_CTRLR_EVT_DRDY, USB_CTRLR_EVT_XFER_CMPL, USB_CTRLR_EVT_XFER_FAILED,
	RA4USB_PID_NAK = 0, RA4USB_PID_BUF = 1 };
struct Ra4UsbPipe_t { bool In, Busy, Halted; uint16_t Mps, Length; uint8_t *pBuffer; };
struct { bool Started, Closing; Ra4UsbPipe_t Pipe[10]; } s_Usb;
struct { uint16_t BRDYENB, NRDYENB; } regs;
#define R_USB_FS0 (&regs)
static uint8_t buffer[64];
static unsigned queued, completions, requests, failures;
static bool rxPending, closeOnComplete, nakOk = true;
static uint16_t pid;
static bool Ra4UsbSetPid(uint8_t, uint16_t value) { pid = value; return nakOk; }
static int Ra4UsbFifoRead(uint8_t, uint8_t *data, uint16_t capacity)
{
	assert(pid == RA4USB_PID_NAK && data == buffer && capacity == 64);
	return 64;
}
static void reserve(uint8_t pipe)
{
	rxPending = queued == 2;
	if (!rxPending)
	{
		s_Usb.Pipe[pipe].Busy = true;
		s_Usb.Pipe[pipe].pBuffer = buffer;
		s_Usb.Pipe[pipe].Length = 64;
		regs.BRDYENB |= 1U << pipe;
		pid = RA4USB_PID_BUF;
	}
}
static void Ra4UsbNotify(uint8_t pipe, int event, uint16_t length)
{
	if (event == USB_CTRLR_EVT_DRDY) { ++requests; reserve(pipe); }
	else if (event == USB_CTRLR_EVT_XFER_CMPL)
	{
		assert(!s_Usb.Pipe[pipe].Busy && length == 64);
		++queued; ++completions;
		if (closeOnComplete) { s_Usb.Pipe[pipe] = {}; }
	}
	else { ++failures; }
}
''' + function('static inline uint16_t Ra4UsbPipeBit(') + '\n' + function('static void Ra4UsbRequestOut(') + '\n' + function('static void Ra4UsbHandleOut(') + r'''
int main()
{
	s_Usb.Started = true;
	s_Usb.Pipe[3].Mps = 64;
	Ra4UsbRequestOut(3); // Endpoint open: request the first destination.
	assert(requests == 1 && s_Usb.Pipe[3].Busy);
	Ra4UsbRequestOut(3); assert(requests == 1); // No second reservation.
	for (unsigned i = 0; i < 100; ++i)
	{
		assert(s_Usb.Pipe[3].Busy);
		Ra4UsbHandleOut(3);
		if (queued == 2)
		{
			assert(rxPending && !s_Usb.Pipe[3].Busy && pid == RA4USB_PID_NAK);
			--queued; reserve(3); // Generic RxPending retry after consumption.
		}
	}
	assert(completions == 100 && failures == 0 && s_Usb.Pipe[3].Busy);
	closeOnComplete = true;
	const unsigned before = requests;
	Ra4UsbHandleOut(3);
	assert(requests == before && s_Usb.Pipe[3].Mps == 0);
	s_Usb.Pipe[3].Mps = 64;
	s_Usb.Pipe[3].Halted = true;
	Ra4UsbRequestOut(3); assert(requests == before);
	s_Usb.Pipe[3].Halted = false;
	s_Usb.Closing = true;
	Ra4UsbRequestOut(3); assert(requests == before);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p / 'rx.cpp').write_text(code)
    for opt in ('-O0', '-Os', '-O2'):
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall',
                        '-Wextra', '-Werror', opt, str(p / 'rx.cpp'),
                        '-o', str(p / 'rx')], check=True)
        subprocess.run([str(p / 'rx')], check=True)
        print('PASS', opt, 'first receive, 100 packets, full queue, consume/rearm, teardown')
