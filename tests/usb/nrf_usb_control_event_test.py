#!/usr/bin/env python3
"""Exercise the production SETUP snapshot and host-resume state transitions."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp').read_text()
header = (ROOT / 'ARM/Nordic/include/usb_ctrlr.h').read_text()


def function(name):
    match = re.search(r'static void ' + name + r'\([^;{}]*\)\s*\{', source)
    assert match, name
    brace = source.index('{', match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <initializer_list>
#include "usb_ctrlr.h"
namespace setup {
struct Registers {
 volatile uint32_t BMREQUESTTYPE, BREQUEST, WVALUEL, WVALUEH;
 volatile uint32_t WINDEXL, WINDEXH, WLENGTHL, WLENGTHH;
 volatile uint32_t EVENTS_EP0SETUP, TASKS_EP0RCVOUT;
} regs;
auto *NRF_USBD = &regs;
unsigned stage;
UsbCtrlrEvt_t delivered;
void nRFUsbdAbortEp0() {
 assert(stage++ == 0 && regs.EVENTS_EP0SETUP == 0);
 // A later SETUP must not replace the bytes already captured for this event.
 regs.BMREQUESTTYPE = regs.BREQUEST = regs.WVALUEL = regs.WVALUEH = 0;
 regs.WINDEXL = regs.WINDEXH = regs.WLENGTHL = regs.WLENGTHH = 0;
}
void nRFUsbdHostResume() { assert(stage++ == 1); }
void UsbDevProcessEvent(int dev, const UsbCtrlrEvt_t *evt) {
 assert(stage++ == 2 && dev == 0 && regs.TASKS_EP0RCVOUT == 0);
 delivered = *evt;
}
void nRFUsbdResumeQueuedDmaLocked() { assert(stage++ == 3); }
'''
code += function('nRFUsbdProcessEP0Setup')
code += r'''
void check() {
 static_assert(sizeof(UsbSetupData_t) == 8);
 for(unsigned type = 0; type < 256; ++type)
 for(unsigned request : {unsigned(USB_REQ_SET_ADDRESS), 9U})
 for(unsigned length : {0U, 1U, 0xCAFEU}) {
  regs = {0xA500U | type, 0x5A00U | request, 0xFFFF, 0xFF12,
          0xAB34, 0xCD56, 0x1100U | (length & 255), 0x2200U | (length >> 8), 1, 0};
  stage = 0;
  nRFUsbdProcessEP0Setup();
  assert(stage == 4);
  const bool address = !(type & (USB_REQTYPE_MASK_RECEIPT | USB_REQTYPE_MASK_TYPE)) &&
                       request == USB_REQ_SET_ADDRESS;
  if(address) assert(delivered.Type == USB_CTRLR_EVT_ADDRESS && delivered.Address == 127);
  else {
   assert(delivered.Type == USB_CTRLR_EVT_SETUP);
   assert(delivered.Setup.bmRequestType == type && delivered.Setup.bRequest == request);
   assert(delivered.Setup.wValue == 0x12FF && delivered.Setup.wIndex == 0x5634);
   assert(delivered.Setup.wLength == length);
  }
  assert(regs.TASKS_EP0RCVOUT == unsigned(!address && !(type & USB_REQTYPE_MASK_DIR) && length));
 }
 puts("PASS: SETUP snapshot survives abort, preserves all eight bytes, and arms only OUT data requests");
}
}
namespace resume {
'''
flags = re.search(r'enum\s*\{[^}]*USBD_FLAG_SUSPENDED[^}]*\};', header)
assert flags
code += flags.group(0)
code += r'''
struct { volatile uint8_t Flags; } s_Usbd;
unsigned irqMask, disables, resumes, forces;
bool normal;
uint32_t DisableInterrupt() { ++disables; auto old = irqMask; irqMask = 1; return old; }
void EnableInterrupt(uint32_t old) { irqMask = old; }
bool UsbdIsForceNormal() { return normal; }
void UsbdForceNormal() { ++forces; }
void nRFUsbdEmitSimple(UsbCtrlrEvtType_t event) {
 assert(event == USB_CTRLR_EVT_RESUME); ++resumes;
}
'''
code += function('nRFUsbdHostResume')
code += r'''
void check() {
 for(unsigned flags = 0; flags < 8; ++flags)
 for(unsigned awake = 0; awake < 2; ++awake)
 for(unsigned masked = 0; masked < 2; ++masked) {
  s_Usbd.Flags = flags; normal = awake; irqMask = masked;
  disables = resumes = forces = 0;
  nRFUsbdHostResume();
  const bool suspended = flags & USBD_FLAG_SUSPENDED;
  const bool ready = suspended && (flags & USBD_FLAG_MAC_AWAKE) && normal;
  const unsigned expected = suspended ?
   (flags & ~USBD_FLAG_REMOTE_WAKE & (ready ? ~USBD_FLAG_SUSPENDED : ~0U)) : flags;
  assert(s_Usbd.Flags == expected && irqMask == masked);
  // ISR only: the transition takes no interrupt exclusion of its own.
  assert(disables == 0 && resumes == unsigned(ready));
  assert(forces == unsigned(suspended && !ready));
 }
 puts("PASS: host resume updates wake state without masking and preserves caller IRQ mask");
}
}
int main() { setup::check(); resume::check(); }
'''
with tempfile.TemporaryDirectory(prefix='iosonata-control-') as temp:
    path = Path(temp) / 'control.cpp'
    path.write_text(code)
    target = Path(temp) / 'control'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'tests/usb/hostport'),
                    str(path), '-o', str(target)], check=True)
    subprocess.run([str(target)], check=True)
