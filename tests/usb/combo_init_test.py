#!/usr/bin/env python3
"""Check the real combo example topology with dedicated and shared ISO endpoints.

Compile the production example and USB stack against a no-op controller. The
LM20 case uses the Nordic port header with USBHS_PRESENT. This is a host test;
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
#include "exemples/usb/usb_combo_stress.cpp"
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
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }
#if defined(USBHS_PRESENT)
bool UsbCtrlrEpOpenData(int, uint8_t, bool, uint8_t, uint16_t) { return true; }
bool UsbCtrlrIsoOpen(int, uint8_t, bool, uint16_t) { return true; }
#endif
void UsbCtrlrEpClose(int, uint8_t, bool) {}
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

class EndpointBlocker final : public UsbDeviceClass {
public:
	EndpointBlocker() = default;
	~EndpointBlocker() = default;
};

int main(int argc, char **)
{
	assert(UsbInit(&s_UsbCfg));
	assert(s_LoopbackCdc.Init(s_LoopbackCfg));
	assert(s_PrbsCdc.Init(s_PrbsCfg));
	assert(s_Hid.Init(s_HidCfg));
	assert(IntInit());
	if (argc > 1)
	{
		// Exhaust all remaining ISO candidates. Refusal must be clean.
		EndpointBlocker blocker;
		const uint16_t usedIn = s_LoopbackCdc.EpInMask() |
			s_PrbsCdc.EpInMask() | s_Hid.EpInMask() | s_IntClass.EpInMask();
		const uint16_t usedOut = s_LoopbackCdc.EpOutMask() |
			s_PrbsCdc.EpOutMask() | s_Hid.EpOutMask() | s_IntClass.EpOutMask();
		assert(UsbClassRegister(0, &blocker, 0, 0,
			USB_ISO_EPIN_MASK(0) & ~usedIn,
			USB_ISO_EPOUT_MASK(0) & ~usedOut));
		assert(!IsoInit());
		puts("PASS combo ISO exhaustion");
		return 0;
	}
	assert(IsoInit());
	UsbDeviceClass *classes[] = {
		&s_LoopbackCdc, &s_PrbsCdc, &s_Hid, &s_IntClass, &s_IsoClass,
	};
	uint16_t usedIn = 0U, usedOut = 0U;
	unsigned interfaces = 0U;
	for (UsbDeviceClass *p : classes)
	{
		assert((usedIn & p->EpInMask()) == 0U);
		assert((usedOut & p->EpOutMask()) == 0U);
		assert(p->FirstInterface() == interfaces);
		interfaces += p->InterfaceCount();
		usedIn |= p->EpInMask();
		usedOut |= p->EpOutMask();
	}
	assert(interfaces == 7U);
	assert(s_Fn.IsoEpNo == (USB_HIGHSPEED_CAPABLE(0) ? 7U : 8U));
	for (UsbSpeed_t speed : {USB_SPEED_FULL, USB_SPEED_HIGH})
	{
		if (speed == USB_SPEED_HIGH && !USB_HIGHSPEED_CAPABLE(0)) continue;
		uint16_t length = 0U;
		const uint8_t *desc = UsbGetDescriptor(0, USB_DESCTYPE_CONFIGURATION,
			0, 0, speed, &length);
		assert(desc != nullptr && length > 9U && desc[4] == 7U);
	}
	assert(s_IsoClass.SelectConfig(1U));
	assert(s_IsoClass.SelectInterface(s_Fn.IsoInterfaceNo, 1U));
	printf("PASS combo initialization: ISO endpoint %u\n", s_Fn.IsoEpNo);
}
'''

SOURCES = [
    'src/usb/usb.cpp', 'src/usb/usb_intrf.cpp', 'src/usb/usb_int.cpp',
    'src/usb/usb_iso.cpp', 'src/usb/usbd_cdc.cpp',
    'src/usb/usbd_cdc_desc.cpp', 'src/usb/usbd_hid.cpp',
    'src/usb/usbd_epalloc.cpp', 'src/app_evt_handler.cpp', 'src/app_run.cpp',
    'src/device_intrf.cpp', 'src/cfifo.c',
]

with tempfile.TemporaryDirectory(prefix='iosonata-combo-init-') as directory:
    work = Path(directory)
    (work / 'nrf.h').write_text('#pragma once\n#define USBHS_PRESENT 1\n')
    (work / 'nrf_peripherals.h').write_text('#pragma once\n')
    (work / 'test.cpp').write_text('#include <initializer_list>\n' + MODEL)
    for name, port in [('dedicated', 'tests/usb/hostport'),
                       ('lm20', 'ARM/Nordic/include')]:
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
        subprocess.run(command, check=True)
        subprocess.run([str(output)], check=True)
        subprocess.run([str(output), 'exhausted'], check=True)
