#!/usr/bin/env python3
"""Exercise USB ISR with the production ICU acknowledge and dispatcher."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
port = root / 'ARM/Renesas/RA4/RA4M1/src'
usb = (port / 'usb_ctrlr_ra4m1.cpp').read_text()
icu = (port / 'interrupt_ra4m1.cpp').read_text()

def function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2]

prefix = r'''
#include <cassert>
#include <cstdint>
using IRQn_Type = int;
using Ra4m1IRQHandler_t = void (*)(int, void *);
enum { RA4M1_IELS_CNT = 32, RA4M1_EVTID_PORT_IRQ15 = 16, USB_EVENT = 0x33 };
struct { uint32_t IELSR[32]; uint8_t IRQCR[16]; } icu;
struct { uint16_t INTSTS0; } usb;
#define R_ICU (&icu)
#define R_USB_FS0 (&usb)
#define __DSB() ((void)0)
#define __NOP() ((void)0)
#define RA4USB_LIVE(...) ((void)0)
struct Ra4m1IntHook {
	Ra4m1IRQHandler_t Handler; void *pCtx; uint32_t Generation; uint8_t Event;
};
static Ra4m1IntHook s_IntHook[32];
static uint32_t mask;
static uint32_t DisableInterrupt() { auto old = mask; mask = 1; return old; }
static void EnableInterrupt(uint32_t old) { mask = old; }
static void NVIC_DisableIRQ(IRQn_Type) {}
static unsigned remaining, serviced;
static bool Ra4UsbHandleOneInterrupt()
{
	if (!remaining) { usb.INTSTS0 = 0x80; return false; }
	--remaining; ++serviced;
	// A subsequent USB event arrives during servicing. ICU IR latches it.
	// Its USB status is still asserted if the ISR reaches its batch limit.
	icu.IELSR[0] |= 1U << 16;
	usb.INTSTS0 = remaining ? 0x2ca2 : 0x80;
	return true;
}
'''
ack = function(icu, 'bool Ra4m1AcknowledgeInt(')
isr = function(usb, 'static void Ra4UsbInterrupt(')
dispatch = function(icu, 'static void Ra4m1Dispatch(int IrqNo)\n{')
test = r'''
static void ordinary(int, void *) {}
int main()
{
	s_IntHook[0] = {Ra4UsbInterrupt, nullptr, 1, USB_EVENT};
	icu.IELSR[0] = USB_EVENT | (1U << 16);
	usb.INTSTS0 = 0x2ca2;
	remaining = 65;
	Ra4m1Dispatch(0);
	assert(serviced == 32 && remaining == 33);
	assert(icu.IELSR[0] & (1U << 16)); // Must survive dispatcher return.
	Ra4m1Dispatch(0);
	assert(serviced == 64 && remaining == 1);
	assert(icu.IELSR[0] & (1U << 16));
	Ra4m1Dispatch(0);
	assert(serviced == 65 && remaining == 0);
	// Service a latched request whose peripheral work is already drained.
	Ra4m1Dispatch(0);
	assert(!(icu.IELSR[0] & (1U << 16)));
	assert(mask == 0);
	// Unplug later creates a fresh request and reaches USB service.
	remaining = 1; icu.IELSR[0] |= 1U << 16;
	Ra4m1Dispatch(0); assert(serviced == 66);
	// Other drivers still receive the dispatcher's ordinary trailing clear.
	s_IntHook[1] = {ordinary, nullptr, 1, 0x98};
	icu.IELSR[1] = 0x98 | (1U << 16);
	Ra4m1Dispatch(1); assert(icu.IELSR[1] == 0x98);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    code = prefix + ack + '\n' + isr + '\n' + dispatch + test
    (p / 'irq.cpp').write_text(code)
    for opt in ('-O0', '-Os', '-O2'):
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall',
                        '-Wextra', '-Werror', opt, str(p / 'irq.cpp'),
                        '-o', str(p / 'irq')], check=True)
        subprocess.run([str(p / 'irq')], check=True)
        print('PASS', opt, '32-event boundary, next batch, idle latch, later event, ordinary dispatch')
    # The previous handler loses the new ICU request on dispatcher return.
    old = code.replace('(void)Ra4m1AcknowledgeInt((IRQn_Type)IntNo);', '(void)IntNo;')
    (p / 'old.cpp').write_text(old)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', str(p / 'old.cpp'),
                    '-o', str(p / 'old')], check=True)
    result = subprocess.run([str(p / 'old')], capture_output=True, text=True)
    assert result.returncode != 0 and 'icu.IELSR[0] & (1U << 16)' in result.stderr
    print('PASS regression: previous ISR loses the pending request')
