#!/usr/bin/env python3
"""RA4M1 UART FIFO behavioral tests using production transfer functions.

Run: python3 tests/ra4m1/run_uart_fifo_validation.py
Set UART_FIFO_SANITIZE=1 to also run ASan/UBSan.

The model implements FIFO/status side effects, not a register address map.
Bit definitions are read from the official Renesas CMSIS header.
It does not validate electrical operation, interrupt timing, or throughput.

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
from pathlib import Path
import re
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
src = (root/'ARM/Renesas/RA4M1/src/uart_ra4m1.cpp').read_text()
header = (root/'ARM/Renesas/RA4M1/include/R7FA4M1AB.h').read_text()

def function(name):
    a = re.search(r'^static [^\n]+\b'+name+r'\(', src, re.M).start()
    b = src.index('{', a)
    depth = 1
    i = b+1
    while depth:
        if src[i]=='{': depth+=1
        if src[i]=='}': depth-=1
        i+=1
    return src[a:i]+'\n'

defs = '\n'.join(line for line in header.splitlines() if re.match(r'\s*#define R_SCI0_',line))
defs += '\n'+'\n'.join(line for line in src.splitlines() if line.startswith('#define SCI_'))+'\n'
names = ['Owner','Receive','WriteTx','TxReady','DmaStart','DmaIrqHandler','FillTx','UartIrq','RxData','TxData']
code = r'''

#include "cfifo.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
'''+defs+r'''

using IRQn_Type = int;
enum UART_EVT
{
	UART_EVT_RXDATA,
	UART_EVT_TXREADY,
	UART_EVT_LINESTATE
};
enum
{
	UART_LINESTATE_OVR = 64,
	UART_LINESTATE_PARERR = 8,
	UART_LINESTATE_FRMERR = 16,
	UART_LINESTATE_BRK = 32
};
struct UARTDEV;
using UARTEvtHandler_t = void (*)(UARTDEV *, UART_EVT, uint8_t *, int);
struct DevIntrf_t
{
	bool bIntEn = true, bTxReady = true;
	int MaxRetry = 2;
	void *pDevData = nullptr;
};
struct UARTDEV
{
	DevIntrf_t DevIntrf;
	hCFifo_t hRxFifo = nullptr, hTxFifo = nullptr;
	unsigned RxOvrErrCnt = 0, ParErrCnt = 0, FramErrCnt = 0, RxDropCnt = 0, TxDropCnt = 0,
	         LineState = 0;
	int DataBits = 8;
	bool bRxReady = false, bTxReady = true;
	UARTEvtHandler_t EvtCallback = nullptr;
};

// Behavioral model, not a hardware register map. Destructive reads and
// write-zero status clears are checked against manual chapter 28.
struct Sci
{
	bool fifo = true;
	unsigned txTrigger = SCI_TX_TRIGGER;
	uint8_t flags = SCI_SSR_TDRE;
	uint8_t SCR = SCI_SCR_RE | SCI_SCR_TE | SCI_SCR_RIE, SPTR = 1;
	std::deque<uint16_t> rx;
	std::deque<uint8_t> tx;
	std::vector<uint8_t> wire;
	unsigned reads = 0, writes = 0;
	bool injectOnClear = false;
	struct Status
	{
		Sci *s;
		operator uint8_t() const
		{
			return s->flags;
		}
		void operator=(uint8_t v)
		{
			if (s->injectOnClear)
			{
				s->injectOnClear = false;
				s->push(0x6a);
			}
			s->flags &= v;
			// The FIFO threshold condition reasserts after one module clock.
			if (s->fifo && s->rx.size() >= SCI_RX_TRIGGER)
			{
				s->flags |= SCI_SSR_RDRF;
			}
			if (s->fifo && s->tx.size() <= s->txTrigger)
			{
				s->flags |= SCI_SSR_TDRE;
			}
		}
	} SSR{this}, SSR_FIFO{this};
	struct Count
	{
		Sci *s;
		bool receive;
		operator unsigned() const
		{
			assert(s->fifo);
			return receive ? s->rx.size() : s->tx.size();
		}
	};
	struct Counts
	{
		Count R, T;
	} FDR_b{{this, true}, {this, false}};
	struct Read
	{
		Sci *s;
		bool fifo;
		operator uint16_t() const
		{
			assert(s->fifo == fifo && !s->rx.empty());
			auto v = s->rx.front();
			s->rx.pop_front();
			++s->reads;
			if (!fifo)
			{
				s->flags &= ~SCI_SSR_RDRF;
			}
			return v;
		}
	} FRDRHL{this, true}, RDR{this, false};
	struct Write
	{
		Sci *s;
		bool fifo;
		void operator=(uint8_t v)
		{
			assert(s->fifo == fifo && s->tx.size() < (fifo ? 16u : 1u));
			s->tx.push_back(v);
			++s->writes;
			if (!fifo)
			{
				s->flags &= ~SCI_SSR_TDRE;
			}
		}
	} FTDRL{this, true}, TDR{this, false};
	void push(uint16_t frame)
	{
		if (rx.size() == (fifo ? 16u : 1u))
		{
			flags |= SCI_SSR_ORER;
			return;
		}
		rx.push_back(frame);
		flags |= (frame >> 8) & (SCI_SSR_FER | SCI_SSR_PER);
		if (!fifo || rx.size() >= SCI_RX_TRIGGER)
		{
			flags |= SCI_SSR_RDRF;
		}
	}
	void timeout()
	{
		if (fifo && !rx.empty() && rx.size() < SCI_RX_TRIGGER)
		{
			flags |= R_SCI0_SSR_FIFO_DR_Msk;
		}
	}
	void tick()
	{
		if (!tx.empty())
		{
			wire.push_back(tx.front());
			tx.pop_front();
		}
		if (tx.size() <= (fifo ? txTrigger : 0u))
		{
			flags |= SCI_SSR_TDRE;
		}
	}
};
struct Ra4m1UartDev
{
	UARTDEV *pDev;
	Sci *Regs;
	bool FifoMode = true, DmaMode = false, DmaBusy = false, Enabled = true, Rx = true, Tx = true;
	unsigned Generation = 0, DmaChannel = 0;
	uint16_t DmaCount = 0;
};
struct R_DMAC0_Type
{
	uint32_t DMSAR = 0, DMCRA = 0, DMSTS = 0;
	struct
	{
		unsigned DTE = 0;
	} DMCNT_b;
} dma;
R_DMAC0_Type *s_DmaRegs[] = {&dma};
static unsigned mask = 0, sections = 0;
static uint32_t DisableInterrupt()
{
	auto old = mask;
	mask = 1;
	++sections;
	return old;
}
static void EnableInterrupt(uint32_t old)
{
	mask = old;
}
static void Sync(Ra4m1UartDev *)
{
}
static void Ra4m1AcknowledgeInt(IRQn_Type)
{
}
'''
code += '\n'.join(function(n) for n in names)
code += r'''

struct Fixture
{
	alignas(8) uint8_t rxmem[CFIFO_MEMSIZE(256)], txmem[CFIFO_MEMSIZE(256)];
	Sci sci;
	UARTDEV u;
	Ra4m1UartDev d{&u, &sci};
	Fixture(bool fifo = true, int capacity = 256, bool blocking = true, bool dmaMode = false)
	{
		sci.fifo = d.FifoMode = fifo;
		d.DmaMode = dmaMode;
		sci.txTrigger = dmaMode ? 15 : SCI_TX_TRIGGER;
		u.hRxFifo = CFifoInit(rxmem, CFIFO_MEMSIZE(capacity), 1, blocking);
		u.hTxFifo = CFifoInit(txmem, CFIFO_MEMSIZE(capacity), 1, blocking);
		u.DevIntrf.pDevData = &d;
		dma = {};
	}
	void step()
	{
		sci.tick();
		if (d.DmaMode && d.DmaBusy && (sci.SCR & SCI_SCR_TIE) && (sci.flags & SCI_SSR_TDRE))
		{
			assert(dma.DMCNT_b.DTE && dma.DMCRA);
			uintptr_t base = (uintptr_t)u.hTxFifo->pMemStart;
			uint32_t offset = dma.DMSAR - (uint32_t)base;
			assert(offset < (unsigned)u.hTxFifo->MaxIdxCnt);
			uint8_t byte = ((uint8_t *)base)[offset];
			if (d.FifoMode)
			{
				sci.FTDRL = byte;
			}
			else
			{
				sci.TDR = byte;
			}
			if (d.FifoMode)
			{
				sci.SSR_FIFO = (uint8_t)~SCI_SSR_TDRE;
			}
			++dma.DMSAR;
			if (--dma.DMCRA == 0)
			{
				DmaIrqHandler(4, &d);
			}
		}
		if (((sci.SCR & SCI_SCR_RIE) && (sci.flags & (SCI_SSR_RDRF | SCI_SSR_ERRORS | 1))) ||
		    (!d.DmaMode && (sci.SCR & SCI_SCR_TIE) && (sci.flags & SCI_SSR_TDRE)))
		{
			UartIrq(0, &d);
		}
	}
	void drain()
	{
		for (int i = 0; i < 20000 && (!sci.tx.empty() || CFifoUsed(u.hTxFifo) || d.DmaBusy); ++i)
		{
			step();
		}
		assert(sci.tx.empty() && !CFifoUsed(u.hTxFifo) && !d.DmaBusy);
	}
};

int callbackCount = 0;
void shutdown(UARTDEV *u, UART_EVT, uint8_t *, int)
{
	++callbackCount;
	auto *d = (Ra4m1UartDev *)u->DevIntrf.pDevData;
	++d->Generation;
	d->Enabled = false;
}

int main()
{
	// A short burst must be delivered on DR, independent of RDF.
	for (int n = 1; n <= 16; ++n)
	{
		Fixture f;
		for (int i = 0; i < n; ++i)
		{
			f.sci.push(i);
		}
		f.sci.timeout();
		unsigned before = sections;
		UartIrq(0, &f.d);
		assert(sections == before);
		uint8_t out[32] = {};
		assert(RxData(&f.u.DevIntrf, out, 32) == n);
		for (int i = 0; i < n; ++i)
		{
			assert(out[i] == i);
		}
		assert(f.sci.rx.empty() && f.sci.reads == (unsigned)n);
		UartIrq(3, &f.d);
		assert(f.sci.reads == (unsigned)n); // Pending ERI cannot consume twice.
	}
	// ORER loses the incoming byte; retain every valid buffered byte.
	for (bool fifo : {false, true})
	{
		Fixture f(fifo);
		int depth = fifo ? 16 : 1;
		for (int i = 0; i < depth + 1; ++i)
		{
			f.sci.push(i);
		}
		UartIrq(3, &f.d);
		assert(f.u.RxOvrErrCnt == 1 && f.u.RxDropCnt == 1 && CFifoUsed(f.u.hRxFifo) == depth);
		assert(!(f.sci.flags & SCI_SSR_ORER));
	}
	// Per-entry FE/PE: do not discard clean neighbors or count global sticky flags twice.
	{
		Fixture f;
		f.sci.push(1);
		f.sci.push(2 | R_SCI0_FRDRHL_FER_Msk);
		f.sci.push(3 | R_SCI0_FRDRHL_PER_Msk);
		f.sci.push(4);
		UartIrq(3, &f.d);
		uint8_t out[4];
		assert(RxData(&f.u.DevIntrf, out, 4) == 2 && out[0] == 1 && out[1] == 4);
		assert(f.u.FramErrCnt == 1 && f.u.ParErrCnt == 1 && f.u.RxDropCnt == 2);
		UartIrq(3, &f.d);
		assert(f.u.FramErrCnt == 1 && f.u.ParErrCnt == 1);
	}
	// Arrivals during a status clear must remain queued for the next entry.
	{
		Fixture f;
		for (int i = 0; i < 8; ++i)
		{
			f.sci.push(i);
		}
		f.sci.injectOnClear = true;
		UartIrq(0, &f.d);
		assert(f.sci.rx.size() == 1);
		f.sci.timeout();
		UartIrq(0, &f.d);
		uint8_t out[10];
		assert(RxData(&f.u.DevIntrf, out, 10) == 9 && out[8] == 0x6a);
	}
	// Software overflow is distinct from hardware ORER, in both FIFO modes.
	for (bool blocking : {false, true})
	{
		Fixture f(true, 4, blocking);
		for (int i = 0; i < 8; ++i)
		{
			f.sci.push(i);
		}
		UartIrq(0, &f.d);
		assert(f.u.RxOvrErrCnt == 0 && f.u.RxDropCnt == 4 && CFifoUsed(f.u.hRxFifo) == 4);
		uint8_t out[4];
		assert(RxData(&f.u.DevIntrf, out, 4) == 4);
		for (int i = 0; i < 4; ++i)
		{
			assert(out[i] == i + (blocking ? 0 : 4));
		}
	}
	// Partial polling reads, then a full read; seven-bit data masking.
	{
		Fixture f;
		f.u.DevIntrf.bIntEn = false;
		f.u.DataBits = 7;
		for (int i = 0; i < 16; ++i)
		{
			f.sci.push(0x80 + i);
		}
		uint8_t out[16];
		assert(RxData(&f.u.DevIntrf, out, 3) == 3 && f.sci.rx.size() == 13);
		assert(RxData(&f.u.DevIntrf, out + 3, 13) == 13);
		for (int i = 0; i < 16; ++i)
		{
			assert(out[i] == i);
		}
	}
	// Continuous full-duplex traffic through the actual producer/consumer functions,
	// with non-power-of-two software FIFO wrap and DMA source retention.
	for (bool fifo : {false, true})
	{
		for (bool dmaMode : {false, true})
		{
			for (int capacity : {17, 256})
			{
				Fixture f(fifo, capacity, true, dmaMode);
				std::vector<uint8_t> expected;
				int seq = 0;
				for (int cycle = 0; cycle < 1000; ++cycle)
				{
					int n = fifo ? (cycle % 16) + 1 : 1;
					for (int i = 0; i < n; ++i)
					{
						f.sci.push((seq++) & 255);
					}
					f.sci.timeout();
					UartIrq(0, &f.d);
					uint8_t out[16];
					int got = RxData(&f.u.DevIntrf, out, sizeof out);
					assert(got == n);
					int pos = 0;
					while (pos < got)
					{
						int sent = TxData(&f.u.DevIntrf, out + pos, got - pos);
						pos += sent;
						f.step();
					}
					expected.insert(expected.end(), out, out + got);
					if (cycle % 23 == 0)
					{
						f.drain();
					}
				}
				f.drain();
				assert(f.sci.wire == expected);
				assert(f.u.RxOvrErrCnt == 0 && f.u.RxDropCnt == 0 && f.u.TxDropCnt == 0);
			}
		}
	}
	// Polling TX accepts only hardware room; unaccepted suffix remains caller-owned.
	for (bool fifo : {false, true})
	{
		Fixture f(fifo);
		f.u.DevIntrf.bIntEn = false;
		uint8_t bytes[32];
		for (int i = 0; i < 32; ++i)
		{
			bytes[i] = i;
		}
		int sent = TxData(&f.u.DevIntrf, bytes, 32);
		assert(sent == (fifo ? 16 : 1));
		assert(CFifoUsed(f.u.hTxFifo) == 0);
		f.drain();
		assert(std::equal(f.sci.wire.begin(), f.sci.wire.end(), bytes));
	}
	// Reinitializing from an error callback must suppress the old RX notification.
	{
		Fixture f;
		f.u.EvtCallback = shutdown;
		f.sci.push(1);
		f.sci.push(2 | R_SCI0_FRDRHL_FER_Msk);
		callbackCount = 0;
		UartIrq(3, &f.d);
		assert(callbackCount == 1);
	}
	assert(mask == 0);
	puts("PASS: 1..16-byte RX, timeout tails, no duplicate consumption, ORER recovery, per-frame "
	     "errors,");
	puts("      concurrent arrival, SW overflow, partial polling, seven-bit RX, FIFO wrap, IRQ/DMA "
	     "TX,");
	puts("      full-duplex ordering, polling TX bounds, and callback lifecycle guard.");
}
'''
# Execute production transfer functions with modeled register side effects.
# Production CMSIS compilation is a separate build check.
cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not cxx or not cc:
    raise SystemExit("A host C and C++ compiler are required.")
with tempfile.TemporaryDirectory(prefix="ra4m1-uart-fifo-") as temp:
    build = Path(temp)
    model = build / "uart_fifo_model.cpp"
    model.write_text(code)
    modes = [("O0", ["-O0"]), ("Os", ["-Os"])]
    if os.environ.get("UART_FIFO_SANITIZE"):
        modes.append(("sanitized", ["-O1", "-g", "-fsanitize=address,undefined",
                                     "-fno-sanitize-recover=all"]))
    for label, flags in modes:
        obj = build / ("cfifo_" + label + ".o")
        exe = build / ("uart_fifo_" + label)
        common = ["-Wall", "-Wextra", "-Werror", "-I" + str(root / "include")]
        subprocess.run([cc, "-std=c17", *common, *flags, "-c",
                        str(root / "src/cfifo.c"), "-o", str(obj)], check=True)
        subprocess.run([cxx, "-std=c++17", *common, *flags, str(model),
                        str(obj), "-o", str(exe)], check=True)
        print(label + ":", flush=True)
        subprocess.run([str(exe)], check=True)

