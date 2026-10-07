#!/usr/bin/env python3
"""USB cable interrupt of the nRF52 and its shared POWER_CLOCK vector.

Builds the production power_clock_irq_nrf52.cpp and usb_ctrlr_nrf52_vbus.cpp
(no SoftDevice) against a register stand-in and checks that:
- the USB port enables the cable events and the shared vector,
- a cable edge clears its event and queues the USB process event once,
- the vector calls only the owners that are linked: clock and USB, clock
  alone, or USB alone.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

NRF_H = r'''
#pragma once
#include <cstdint>
struct NRF_POWER_Type { volatile uint32_t EVENTS_USBDETECTED, EVENTS_USBREMOVED, INTENSET; };
extern NRF_POWER_Type g_Power;
#define NRF_POWER (&g_Power)
#define POWER_INTENSET_USBDETECTED_Msk (1UL << 7)
#define POWER_INTENSET_USBREMOVED_Msk (1UL << 8)
enum IRQn_Type { POWER_CLOCK_IRQn = 0 };
uint32_t NVIC_GetEnableIRQ(IRQn_Type);
void NVIC_ClearPendingIRQ(IRQn_Type);
void NVIC_SetPriority(IRQn_Type, uint32_t);
void NVIC_EnableIRQ(IRQn_Type);
'''

USB_H = r'''
#pragma once
extern "C" void UsbProcessQue(int DevNo);
'''

HARNESS = r'''
#include <cassert>
#include <cstdio>
#include "nrf.h"
#include "power_clock_irq_nrf52.h"
NRF_POWER_Type g_Power;
static bool s_Enabled;
static uint32_t s_Prio;
static unsigned s_Enables, s_Ques;
[[maybe_unused]] static unsigned s_Clock;
uint32_t NVIC_GetEnableIRQ(IRQn_Type) { return s_Enabled; }
void NVIC_ClearPendingIRQ(IRQn_Type) {}
void NVIC_SetPriority(IRQn_Type, uint32_t Prio) { s_Prio = Prio; }
void NVIC_EnableIRQ(IRQn_Type) { s_Enabled = true; s_Enables++; }
extern "C" void UsbProcessQue(int DevNo) { assert(DevNo == 0); s_Ques++; }
#ifdef WITH_CLOCK
void nRFClockIrqHandler(void) { s_Clock++; }
#endif
#ifdef WITH_USB
void nRFUsbdVbusIntInit(uint32_t Prio);
#endif
extern "C" void POWER_CLOCK_IRQHandler(void);
int main()
{
#ifdef WITH_USB
	g_Power.EVENTS_USBDETECTED = 1;
	nRFUsbdVbusIntInit(6);
	// Stale events are dropped, both cable events enabled, vector enabled
	assert(g_Power.EVENTS_USBDETECTED == 0);
	assert(g_Power.INTENSET == (POWER_INTENSET_USBDETECTED_Msk | POWER_INTENSET_USBREMOVED_Msk));
	assert(s_Enabled && s_Prio == 6 && s_Enables == 1);
	// Already enabled by the other owner: its priority is kept
	nRFPowerClockIrqEnable(2);
	assert(s_Prio == 6 && s_Enables == 1);

	g_Power.EVENTS_USBDETECTED = 1;
	POWER_CLOCK_IRQHandler();
	assert(g_Power.EVENTS_USBDETECTED == 0 && s_Ques == 1);
	g_Power.EVENTS_USBREMOVED = 1;
	POWER_CLOCK_IRQHandler();
	assert(g_Power.EVENTS_USBREMOVED == 0 && s_Ques == 2);
	// A clock only interrupt queues nothing
	POWER_CLOCK_IRQHandler();
	assert(s_Ques == 2);
#else
	nRFPowerClockIrqEnable(5);
	g_Power.EVENTS_USBDETECTED = 1;
	POWER_CLOCK_IRQHandler();
	// No USB in this build: its event is not touched
	assert(g_Power.EVENTS_USBDETECTED == 1 && s_Ques == 0);
#endif
#ifdef WITH_CLOCK
	assert(s_Clock >= 1);
#endif
	puts("PASS: " VARIANT);
	return 0;
}
'''

VARIANTS = [
    ('clock and USB owners', ['-DWITH_CLOCK', '-DWITH_USB'], True),
    ('USB owner alone', ['-DWITH_USB'], True),
    ('clock owner alone', ['-DWITH_CLOCK'], False),
]

with tempfile.TemporaryDirectory() as directory:
    d = Path(directory)
    (d / 'usb').mkdir()
    (d / 'nrf.h').write_text(NRF_H)
    (d / 'usb' / 'usb.h').write_text(USB_H)
    (d / 'harness.cpp').write_text(HARNESS)
    for name, defs, usb in VARIANTS:
        sources = [str(d / 'harness.cpp'),
                   str(ROOT / 'ARM/Nordic/nRF52/src/power_clock_irq_nrf52.cpp')]
        if usb:
            sources.append(str(ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_vbus.cpp'))
        target = d / 'vbus'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O1',
                        '-Wall', '-Wextra', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I', str(d),
                        '-I', str(ROOT / 'ARM/Nordic/include'),
                        '-DVARIANT="' + name + '"'] + defs + sources +
                       ['-o', str(target)], check=True)
        subprocess.run([str(target)], check=True)
