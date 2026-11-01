#!/usr/bin/env python3
"""Check the WBA AppWait sleep sequence against the Cube sleep mode entry.

The link layer reads its sleep timer through llhwc_slptmr_get, which waits for
the radio bus clock to resynchronize only when the sleep was reported with
LINKLAYER_PLAT_NotifyWFIEnter and LINKLAYER_PLAT_NotifyWFIExit. The Cube
sequencer reports it around WFI, with interrupts masked and the task check done
again under the mask. This test runs the port AppWait against mocks and checks
the same order, without hardware.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def extract_function(text, name):
    match = re.search(r'^void ' + name + r'\(void\)\s*\{', text, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end]


source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp').read_text()
code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>

static std::string trace;
static uint32_t primask;
static bool pending;
static struct { bool bInitialized; } s_BtAppData;

static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; trace += "M"; }
static void __set_PRIMASK(uint32_t v) { primask = v; trace += v ? "m" : "U"; }
static void __WFE(void) { trace += "E"; }
static void __WFI(void)
{
	// The exit report must sample the sleep timer before any interrupt runs
	assert(primask == 1);
	trace += "I";
}
static bool AppEvtHandlerPending(void)
{
	assert(primask == 1);
	trace += "P";
	return pending;
}
static void LINKLAYER_PLAT_NotifyWFIEnter(void)
{
	assert(primask == 1);
	trace += "<";
}
static void LINKLAYER_PLAT_NotifyWFIExit(void)
{
	assert(primask == 1);
	trace += ">";
}
'''
code += extract_function(source, 'AppWait')
code += r'''
static std::string Run(bool init, bool queued, uint32_t mask)
{
	trace.clear();
	s_BtAppData.bInitialized = init;
	pending = queued;
	primask = mask;
	AppWait();
	assert(primask == mask);
	return trace;
}

int main()
{
	// Before the stack runs, the default wait
	assert(Run(false, false, 0) == "E");
	// Idle: reported sleep under the mask, mask restored
	assert(Run(true, false, 0) == "MP<I>U");
	// Work queued after the AppRun check: no sleep, no report
	assert(Run(true, true, 0) == "MPU");
	// A caller that already masked keeps its mask
	assert(Run(true, false, 1) == "MP<I>m");
	return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='wba-app-wait-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        print('PASS WBA AppWait:', opt, 'masked recheck, reported WFI, mask restored')
