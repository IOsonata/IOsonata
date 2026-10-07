#!/usr/bin/env python3
"""Exercise the production SystemInit USB-clock block and SAM4L GPIO dispatch."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PORT = ROOT / 'ARM/Microchip/SAM4L'
clock = (PORT / 'src/system_sam4l.c').read_text()
clock = clock[clock.index('\t// MCUOSC owns the USB clock.'):clock.index('\t/** Low-freq clock configuration **/')]
gpio = (PORT / 'src/iopincfg_sam4l.c').read_text()

def function(name):
    start = re.search(r'(?:static )?(?:void|bool) ' + name + r'\(', gpio).start()
    brace = gpio.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (gpio[end] == '{') - (gpio[end] == '}')
        end += 1
    return gpio[start:end]

header = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
typedef uint32_t RwReg;
typedef uint32_t RoReg;
typedef uint32_t WoReg;
typedef uint8_t RoReg8;
#include "component/component_scif.h"
#include "component/component_pm.h"
#include "component/component_gpio.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
static Scif scif;
static Pm pm;
static Gpio gpio;
#define SAM4L_SCIF (&scif)
#define SAM4L_PM (&pm)
#define SAM4L_GPIO (&gpio)
#define GPIO_0_IRQn 25
static unsigned pendingClears;
static int enabledIrq;
static void NVIC_ClearPendingIRQ(int) { ++pendingClears; }
static void NVIC_SetPriority(int, int) {}
static void NVIC_EnableIRQ(int n) { enabledIrq = n; }
static void NVIC_DisableIRQ(int n) { enabledIrq = -1; }
static McuOsc_t g_Osc;
#define g_McuOsc g_Osc
static uint32_t s_PllFreq;
#define USB_FREQ 48000000U
#define PLL0_GEN_CLK_SRC 16U
'''
constants = '\n'.join(re.findall(r'^#define IOPIN_MAX_.*$', gpio, re.M))
typedef = gpio[gpio.index('typedef struct {'):gpio.index('#pragma pack(pop)')]
body = header + constants + '\n' + typedef + r'''
static IOPINSENS_EVTHOOK s_GpIOSenseEvt[IOPIN_MAX_INT] = {};
static void configureUsbClock() {
''' + clock.replace('(uint32_t)&', '(uintptr_t)&').replace('(uint32_t)SAM4L_', '(uintptr_t)SAM4L_') + '\n}\n'
body += '\n'.join(function(n).replace('(uint32_t)&', '(uintptr_t)&') for n in
                  ['IOPinSetSense', 'IOPinEnableInterrupt', 'IOPinDisableInterrupt',
                   'Sam4lGpioInterrupt'] + ['GPIO_%d_Handler' % i for i in range(12)])
body += r'''
static unsigned reportedPins;
static void *reportedContext;
static void onPin(int pins, void *context) {
 reportedPins = static_cast<unsigned>(pins); reportedContext = context;
 assert(gpio.GPIO_PORT[2].GPIO_IFRC == (1U << 11));
}
int main() {
 g_Osc.bUSBClk = false;
 scif.SCIF_GCCTRL[7].SCIF_GCCTRL = 0x12340000U;
 configureUsbClock(); assert(scif.SCIF_GCCTRL[7].SCIF_GCCTRL == 0x12340000U);
 g_Osc.bUSBClk = true; g_Osc.CoreOsc.Type = OSC_TYPE_XTAL;
 scif.SCIF_PCLKSR = SCIF_PCLKSR_PLL0LOCK;
 for (unsigned div = 1; div <= 4; div *= 2) {
  s_PllFreq = USB_FREQ * div; configureUsbClock();
  const auto expected = SCIF_GCCTRL_CEN | SCIF_GCCTRL_OSCSEL(16) |
      (div == 1 ? 0U : SCIF_GCCTRL_DIVEN | SCIF_GCCTRL_DIV(div / 2 - 1));
  assert(scif.SCIF_GCCTRL[7].SCIF_GCCTRL == expected);
  assert((pm.PM_HSBMASK & PM_HSBMASK_USBC) && (pm.PM_PBBMASK & PM_PBBMASK_USBC));
 }
 g_Osc.CoreOsc.Type = OSC_TYPE_RC; configureUsbClock();
 assert(!(scif.SCIF_GCCTRL[7].SCIF_GCCTRL & SCIF_GCCTRL_CEN));
 g_Osc.CoreOsc.Type = OSC_TYPE_TCXO; s_PllFreq = 96000000U;
 configureUsbClock(); assert(scif.SCIF_GCCTRL[7].SCIF_GCCTRL & SCIF_GCCTRL_CEN);
 scif.SCIF_PCLKSR = 0; configureUsbClock();
 assert(!(scif.SCIF_GCCTRL[7].SCIF_GCCTRL & SCIF_GCCTRL_CEN));
 scif.SCIF_PCLKSR = SCIF_PCLKSR_PLL0LOCK; s_PllFreq = 80000000U;
 configureUsbClock(); assert(!(scif.SCIF_GCCTRL[7].SCIF_GCCTRL & SCIF_GCCTRL_CEN));
 assert(IOPIN_MAX_INT == 12);
 assert(!IOPinEnableInterrupt(12, 6, 3, 0, IOPINSENSE_TOGGLE, onPin, nullptr));
 assert(!IOPinEnableInterrupt(8, 6, 2, 11, IOPINSENSE_TOGGLE, onPin, nullptr));
 for (unsigned n = 0; n < 12; ++n) {
  assert(IOPinEnableInterrupt(n, 6, n >> 2, (n & 3) * 8, IOPINSENSE_TOGGLE, nullptr, nullptr));
  assert(enabledIrq == GPIO_0_IRQn + int(n));
 }
 assert(IOPinEnableInterrupt(9, 6, 2, 11, IOPINSENSE_TOGGLE, onPin, &gpio));
 gpio.GPIO_PORT[2].GPIO_IER = (1U << 11);
 gpio.GPIO_PORT[2].GPIO_IFR = (1U << 11) | (1U << 10) | (1U << 3);
 pendingClears = 0;
 GPIO_9_Handler();
 assert(reportedPins == (1U << 11) && reportedContext == &gpio);
 assert(pendingClears == 0); // Do not erase a fresh NVIC edge after the callback.
 IOPinDisableInterrupt(9);
 assert(gpio.GPIO_PORT[2].GPIO_IERC == (1U << 11));
 assert(s_GpIOSenseEvt[9].SensEvtCB == nullptr);
 puts("SAM4L MCUOSC USB clock and PC11 VBUS GPIO: PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='sam4l-platform-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    cpp.write_text(body)
    exe = Path(tmp) / 'test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O1', '-g',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I' + str(ROOT / 'include'), '-I' + str(PORT / 'include'),
                    str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
