#!/usr/bin/env python3
"""Check CDC+ISO descriptors and endpoint lifecycle with a host controller model.

Compile the production example and USB stack against a no-op controller. This is a host test;
it does not exercise the PHY or the TaktOS scheduler.
"""
from pathlib import Path
import os
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]

MODEL = r'''
#include <cassert>
#include <cstdio>
#define main ComboFirmwareMain
#include "exemples/usb/usb_cdc_iso_stress.cpp"
#undef main

bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *) { return true; }
bool UsbCtrlrStart(int) { return true; }
void UsbCtrlrStop(int) {}
void UsbCtrlrProcess(int) {}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return USB_HIGHSPEED_CAPABLE(0); }
bool UsbCtrlrIsoInit(int) { return true; }
void UsbCtrlrIntEnable(int) {}
void UsbCtrlrIntDisable(int) {}
void UsbCtrlrConnect(int) {}
void UsbCtrlrDisconnect(int) {}
void UsbCtrlrRemoteWakeup(int) {}
void UsbCtrlrSofEnable(int, bool) {}
void UsbCtrlrSetAddress(int, uint8_t) {}
static unsigned bulk = 0, interrupt = 0, iso = 0;
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *p) {
 unsigned type = p->bmAttributes & 3;
 if (type == BULK) { ++bulk; assert(bulk <= 3); }
 else if (type == INT) { ++interrupt; assert(interrupt <= 4); }
 else if (type == ISO) { ++iso; assert(iso <= 2); }
 return true;
}
#if defined(RA4M1)
bool UsbCtrlrEpOpenData(int dev, uint8_t ep, bool in, uint8_t type, uint16_t mps) {
 UsbEndPointDesc_t desc = {};
 desc.bEndpointAddress = ep | (in ? 0x80 : 0);
 desc.bmAttributes = type;
 desc.wMaxPacketSize = mps;
 return UsbCtrlrEpOpen(dev, &desc);
}
bool UsbCtrlrIsoOpen(int dev, uint8_t ep, bool in, uint16_t mps) {
 return UsbCtrlrEpOpenData(dev, ep, in, ISO, mps);
}
#endif
void UsbCtrlrEpClose(int, uint8_t ep, bool) { if (ep == static_cast<UsbIsoIntrf_t *>(s_Iso)->EpNo && iso) --iso; }
void UsbCtrlrEpCloseAll(int) {}
void UsbCtrlrEpBind(int, uint8_t, bool, bool, UsbCtrlrEpHandler_t, void *) {}
bool UsbCtrlrEpReceive(int, uint8_t, uint8_t *, uint16_t) { return false; }
void UsbCtrlrEpProcessEvent(int, uint8_t, bool, UsbCtrlrEvtType_t, uint16_t) {}
bool UsbCtrlrEpSend(int, uint8_t, uint8_t *, uint16_t) { return true; }
bool UsbCtrlrIsoSend(int, uint8_t, uint8_t *, uint16_t) { return true; }
int UsbCtrlrEp0Send(int, uint8_t *, int length) { return length; }
bool UsbCtrlrEp0Status(int, uint8_t) { return true; }
void UsbCtrlrEpStall(int, uint8_t, bool) {}
void UsbCtrlrEpClearStall(int, uint8_t, bool) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

int main()
{
 assert(UsbInit(&s_UsbCfg));
 assert(s_LoopbackCdc.Init(s_LoopbackCfg));
 assert(IsoInit());
 assert(s_LoopbackCdc.FirstInterface() == 0);
 assert(s_Iso.FirstInterface() == 2);
 assert((s_LoopbackCdc.EpInMask() & s_Iso.EpInMask()) == 0);
 assert((s_LoopbackCdc.EpOutMask() & s_Iso.EpOutMask()) == 0);
 uint16_t length = 0;
 const uint8_t *desc = UsbGetDescriptor(0, USB_DESCTYPE_CONFIGURATION,
     0, 0, USB_SPEED_FULL, &length);
 assert(desc && desc[4] == 3);
 unsigned isoAlts = 0;
 for (unsigned pos = 0; pos < length; pos += desc[pos]) {
  assert(desc[pos] >= 2 && pos + desc[pos] <= length);
  if (desc[pos+1] == USB_DESCTYPE_INTERFACE && desc[pos+2] == 2) {
   assert(desc[pos+3] == isoAlts);
   assert(desc[pos+4] == (isoAlts == 0 ? 0 : 2));
   ++isoAlts;
  }
 }
 assert(isoAlts == 7);
 assert(s_LoopbackCdc.SelectConfig(1));
 assert(bulk == 2 && interrupt == 1);
 assert(s_Iso.SelectConfig(1));
 for (unsigned alt = 1; alt <= 6; ++alt) {
  assert(s_Iso.SelectInterface(2, alt));
  assert(iso == 2);
  assert(s_Iso.SelectInterface(2, 0));
  assert(iso == 0);
 }
 assert(!s_Iso.SelectInterface(2, 7));
 assert(!s_Iso.SelectInterface(3, 1));
 puts("PASS CDC+ISO descriptors, five pipes, six alternate settings and teardown");
}

'''

SOURCES = [
    'src/usb/usb.cpp', 'src/usb/usb_intrf.cpp',
    'src/usb/usb_iso.cpp', 'src/usb/usbd_cdc.cpp',
    'src/usb/usbd_cdc_desc.cpp',
    'src/usb/usbd_epalloc.cpp', 'src/usb/usbd_periodic.cpp',
    'src/usb/usbd_iso.cpp', 'src/app_evt_handler.cpp', 'src/app_run.cpp',
    'src/device_intrf.cpp', 'src/cfifo.c',
]

with tempfile.TemporaryDirectory(prefix='iosonata-combo-init-') as directory:
    work = Path(directory)
    (work / 'test.cpp').write_text('#include <initializer_list>\n' + MODEL)
    for name, port in [('dedicated', 'tests/usb/hostport'),
                       ('ra4m1', 'ARM/Renesas/include')]:
        output = work / name
        command = shlex.split(os.environ.get('CXX', 'g++')) + [
            '-std=gnu++23', '-O1', '-g', '-fno-exceptions', '-fno-rtti',
            '-ffunction-sections', '-fdata-sections',
            '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-I' + str(ROOT), '-I' + str(ROOT / 'include'),
            '-I' + str(work), '-I' + str(ROOT / port),
            '-I' + str(ROOT / 'tests/usb/example_hostport'),
            str(work / 'test.cpp'), *[str(ROOT / p) for p in SOURCES],
            '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
            '-o', str(output),
        ]
        if name == 'ra4m1':
            command.insert(1, '-DRA4M1')
        subprocess.run(command, check=True)
        subprocess.run([str(output)], check=True)
