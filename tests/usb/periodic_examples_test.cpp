/**-------------------------------------------------------------------------
@file	periodic_examples_test.cpp

@brief	Host test support: periodic examples test.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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
----------------------------------------------------------------------------*/

// Reuse the controller model, then compile the actual loopback application.
#define main PeriodicTestsMain
#include "usbd_periodic_test.cpp"
#undef main
#define main FirmwareMain
#if TEST_ISO_EXAMPLE
#include "../../exemples/usb/usb_iso_loopback.cpp"
#else
#include "../../exemples/usb/usb_int_loopback.cpp"
#endif
#undef main

int main()
{
	assert(UsbInit(&s_UsbCfg));
#if TEST_ISO_EXAMPLE
	auto &fn = s_Iso;
	const auto &cfg = s_IsoCfg.Function;
	assert(fn.Init(s_IsoCfg));
	assert(s_UsbCfg.Pid == 3 && s_UsbCfg.Vid == 0x1209);
	const uint16_t mps[] = {9, 17, 25, 33, 49, 63};
	const uint8_t intervals[] = {1, 1, 1, 1, 1, 1};
	constexpr uint8_t request = 0x5A;
	constexpr uint16_t diagLength = 48;
	CheckDesc(fn, cfg, ISO);
#else
	auto &fn = s_Int;
	const auto &cfg = s_IntCfg.Function;
	assert(fn.Init(s_IntCfg));
	assert(s_UsbCfg.Pid == 4 && s_UsbCfg.Vid == 0x1209);
	const uint16_t mps[] = {64, 64, 64};
	const uint8_t intervals[] = {1, 4, 16};
	constexpr uint8_t request = 0x5B;
	constexpr uint16_t diagLength = 50;
	CheckDesc(fn, cfg, INT);
#endif
	assert(cfg.AltCount == sizeof(mps) / sizeof(mps[0]));
	assert(cfg.InterfaceString == 4 && !cfg.SubClass && !cfg.Protocol);
	Enumerate();
	for (unsigned i = 0; i < cfg.AltCount; ++i)
	{
		assert(cfg.pAlternates[i].FsMps == mps[i]);
		assert(cfg.pAlternates[i].FsInterval == intervals[i]);
		assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, i+1, 0));
		UsbSetupData_t setup = {};
		setup.bmRequestType = USB_REQTYPE_DIRHOST | USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE;
		setup.bRequest = request; setup.wLength = diagLength;
		uint8_t *reply = nullptr; uint16_t length = 0;
		assert(fn.Control(&setup, USB_CTRL_SETUP, &reply, &length));
		assert(reply && length == diagLength);
		assert(reply[44] == mps[i] && reply[45] == 0);
#if TEST_ISO_EXAMPLE
		assert(reply[46] == i+1 && (reply[47] & ISO_DIAG_FLAG_OPENED));
#else
		assert(reply[46] == intervals[i] && reply[47] == i+1 && (reply[48] & INT_DIAG_FLAG_OPENED));
#endif
		setup.bRequest ^= 1;
		assert(!fn.Control(&setup, USB_CTRL_SETUP, &reply, &length));
	}
	Event(USB_CTRLR_EVT_RESET);
	assert(fn.Alternate() == 0 && !fn.Configured());
	UsbDisable(0);
	puts("PASS production loopback layout and diagnostic protocol");
}
