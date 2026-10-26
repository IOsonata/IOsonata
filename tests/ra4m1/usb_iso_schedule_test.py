#!/usr/bin/env python3
"""Exercise production RA4 ISO submission and co-pending interrupt ordering."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
src = (ROOT / 'ARM/Renesas/RA4/RA4M1/src/usb_ctrlr_ra4m1.cpp').read_text()

def function(name):
    begin = src.index(name)
    return src[begin:src.index('\n}', begin) + 2]

constants = src[src.index('enum\n{'):src.index('typedef struct __Ra4UsbRegistration')]
code = r'''
#include <cassert>
#include <cstdint>
''' + constants + r'''
enum { ISO = 1, USB_CTRLR_EVT_RESUME, USB_CTRLR_EVT_SUSPEND,
 USB_CTRLR_EVT_ADDRESS, USB_CTRLR_EVT_SOF, USB_CTRLR_EVT_DRDY };
struct UsbCtrlrEvt_t { int Type; uint16_t FrameNo; uint8_t Address; };
struct TestPipe { uint8_t Type; bool Busy, Halted; };
struct { bool Started, Suspended; TestPipe Pipe[10]; } s_Usb;
struct W0C {
 uint16_t value = 0;
 operator uint16_t() const { return value; }
 void operator=(uint16_t v) { value &= v; }
};
struct {
 W0C INTSTS0, BRDYSTS, BEMPSTS, NRDYSTS;
 uint16_t INTENB0, BRDYENB, BEMPENB, NRDYENB, FRMNUM, USBADDR;
} regs;
#define R_USB_FS0 (&regs)
static unsigned sends, requests, queued, drops, offered;
static uint8_t data[63];
static uint8_t Ra4UsbPhysical(uint8_t ep, bool in) { return ep == 3 ? (in ? 1 : 2) : 0; }
static void Ra4UsbNotify(uint8_t, int, uint16_t) { ++requests; }
static bool UsbCtrlrEpSend(int, uint8_t, uint8_t *, uint16_t) {
 if (s_Usb.Pipe[1].Busy) return false;
 s_Usb.Pipe[1].Busy = true; ++sends; return true;
}
''' + function('bool UsbCtrlrIsoSend(') + r'''
static void Ra4UsbBusEvent(int) {}
static void UsbProcessQue(int) {}
static void Ra4UsbReset() {}
static void Ra4UsbHandleNrdy(uint16_t) {}
static void Ra4UsbHandleControlStage(uint16_t) {}
static void Ra4UsbHandleBemp(uint16_t) {
 assert(s_Usb.Pipe[1].Busy && queued);
 s_Usb.Pipe[1].Busy = false; --queued;
}
static void Ra4UsbHandleBrdy(uint16_t) {
 if (queued == 2) ++drops; else ++queued;
}
static void UsbDevProcessEvent(int, UsbCtrlrEvt_t *e) {
 if (e->Type == USB_CTRLR_EVT_SOF && queued &&
     UsbCtrlrIsoSend(0, 3, data, sizeof(data))) ++offered;
}
''' + function('static bool Ra4UsbHandleOneInterrupt(') + r'''
int main() {
 s_Usb.Started = true;
 s_Usb.Pipe[1].Type = s_Usb.Pipe[2].Type = ISO;
 // Empty scheduler service arms OUT but must not invent an IN completion.
 assert(UsbCtrlrIsoSend(0, 3, nullptr, 0));
 assert(requests == 1 && sends == 0 && !s_Usb.Pipe[1].Busy);
 assert(!UsbCtrlrIsoSend(0, 3, nullptr, 1));
 assert(!UsbCtrlrIsoSend(0, 4, data, 1));
 // An application-queued ZLP is real and must complete normally.
 assert(UsbCtrlrIsoSend(0, 3, data, 0));
 assert(sends == 1 && s_Usb.Pipe[1].Busy);
 // Two queued echoes: retire the sent head before accepting OUT, then SOF.
 queued = 2;
 regs.INTENB0 = RA4USB_INT_SOFR | RA4USB_INT_BEMP | RA4USB_INT_BRDY;
 regs.BEMPENB = 2; regs.BRDYENB = 4;
 for (unsigned frame = 0; frame < 100; ++frame) {
  regs.INTSTS0.value = regs.INTENB0;
  regs.BEMPSTS.value = 2; regs.BRDYSTS.value = 4; regs.FRMNUM = frame;
  unsigned steps = 0;
  while (Ra4UsbHandleOneInterrupt()) assert(++steps < 8);
  assert(steps == 3 && drops == 0 && queued == 2);
  assert(offered == frame + 1 && s_Usb.Pipe[1].Busy);
 }
}
'''
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / 'test.cpp').write_text(code)
    for opt in ('-O0', '-Os', '-O2'):
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall',
                        '-Wextra', '-Werror', opt, str(work / 'test.cpp'),
                        '-o', str(work / 'test')], check=True)
        subprocess.run([str(work / 'test')], check=True)
        print('PASS', opt, 'idle ISO, queued ZLP, 100 co-pending BEMP/BRDY/SOF frames')
