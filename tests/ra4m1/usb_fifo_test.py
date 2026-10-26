#!/usr/bin/env python3
"""Exercise production FIFO writes with delayed readiness after buffer clear."""
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
code = r'''
#include <cassert>
#include <cstdint>
#include <vector>
''' + constants + r'''
static std::vector<uint8_t> bytes;
static unsigned delay, delayAfterClear, byteWrites, wordWrites, commits;
static bool ready, pidOk = true;
static unsigned clears, nakCalls;
static uint16_t selectValue, fifoValue;
struct FifoControl
{
	operator uint16_t() volatile
	{
		if (delay != 0U) { --delay; return 0U; }
		ready = true;
		return fifoValue | RA4USB_CFIFO_FRDY;
	}
	void operator=(uint16_t value) volatile
	{
		if (value == RA4USB_CFIFO_BCLR)
		{
			++clears; bytes.clear(); fifoValue = 0U;
			delay = delayAfterClear; ready = false;
		}
		else
		{
			assert(value == RA4USB_CFIFO_BVAL && ready);
			assert((fifoValue & RA4USB_CFIFO_BVAL) == 0U);
			fifoValue |= value; ++commits;
		}
	}
};
struct FifoWord
{
	void operator=(uint16_t value) volatile
	{
		assert(ready && delay == 0U);
		assert((selectValue & RA4USB_CFIFO_MBW16) != 0U);
		bytes.push_back(value & 255U); bytes.push_back(value >> 8U);
		++wordWrites;
		if (bytes.size() == 64U) { fifoValue |= RA4USB_CFIFO_BVAL; }
	}
};
struct FifoByte
{
	void operator=(uint8_t value) volatile
	{
		assert(ready && delay == 0U);
		assert((selectValue & RA4USB_CFIFO_MBW16) == 0U);
		bytes.push_back(value); ++byteWrites;
	}
};
struct Registers
{
	uint16_t &CFIFOSEL = selectValue;
	FifoControl CFIFOCTR;
	FifoWord CFIFOL;
	FifoByte CFIFOLL;
} regs;
#define R_USB_FS0 (&regs)
static bool Ra4UsbSetPid(uint8_t, uint16_t pid)
{
	++nakCalls; assert(pid == RA4USB_PID_NAK); return pidOk;
}
''' + function("static bool Ra4UsbSelectFifo(") + "\n" + function("static bool Ra4UsbFifoWrite(") + r'''
int main()
{
	alignas(4) uint8_t data[66];
	for (unsigned i = 0; i < sizeof(data); ++i) { data[i] = (uint8_t)(i * 17U); }
	for (uint8_t pipe : {0U, 1U})
	{
		for (uint16_t len : {0U, 1U, 8U, 17U, 63U, 64U})
		{
			for (unsigned offset : {0U, 1U})
			{
				bytes.clear(); byteWrites = wordWrites = commits = 0U;
				delay = 0U; delayAfterClear = 3U; ready = true; fifoValue = 0U;
				assert(Ra4UsbFifoWrite(pipe, len ? data + offset : nullptr, len, 64U));
				assert(bytes.size() == len);
				for (unsigned i = 0; i < len; ++i) { assert(bytes[i] == data[i + offset]); }
				assert(wordWrites == len / 2U && byteWrites == len % 2U);
				assert(commits == (len < 64U ? 1U : 0U));
				assert((selectValue & RA4USB_CFIFO_CURPIPE_MASK) == pipe);
				assert((selectValue & RA4USB_CFIFO_ISEL) == (pipe == 0U ? (unsigned)RA4USB_CFIFO_ISEL : 0U));
				assert((selectValue & RA4USB_CFIFO_MBW16) != 0U);
			}
		}
	}
	// ISO fills the free bank without stopping or clearing the active bank.
	bytes.clear(); fifoValue = 0; delay = 0; clears = nakCalls = 0;
	assert(Ra4UsbFifoWrite(1, data, 63, 64, true));
	assert(bytes.size() == 63 && clears == 0 && nakCalls == 0);
	// Both banks occupied: FRDY never asserts, neither bank is overwritten.
	bytes.clear(); fifoValue = 0; delay = RA4USB_WAIT_COUNT + 1;
	assert(!Ra4UsbFifoWrite(1, data, 63, 64, true));
	assert(bytes.empty() && clears == 0 && nakCalls == 0);
	// A stuck FIFO after BCLR must fail without writing or committing data.
	delay = 0U; delayAfterClear = RA4USB_WAIT_COUNT + 1U; commits = 0U;
	assert(!Ra4UsbFifoWrite(0U, data, 8U, 64U));
	assert(bytes.empty() && commits == 0U);
	// Failure to stop the pipe must not clear or load its buffer.
	pidOk = false; bytes.push_back(123U);
	assert(!Ra4UsbFifoWrite(0U, data, 8U, 64U));
	assert(bytes.size() == 1U && bytes[0] == 123U);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "fifo.cpp").write_text(code)
    for opt in ("-O0", "-Os", "-O2"):
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-Wall",
                        "-Wextra", "-Werror", opt, str(path / "fifo.cpp"),
                        "-o", str(path / "fifo")], check=True)
        subprocess.run([str(path / "fifo")], check=True)
        print("PASS", opt, "FIFO byte order, odd/unaligned/zero/full packets, delayed FRDY and timeout")
