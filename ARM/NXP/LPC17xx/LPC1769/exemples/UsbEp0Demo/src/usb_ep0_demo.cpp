/**-------------------------------------------------------------------------
@example	usb_ep0_demo.cpp

@brief	LPC1769 EP0-only USB device example.

Uses the real generic core, event queue and LPC1769 controller. The vendor
interface has no data endpoints; request 0xC0/1 reads an eight-byte identity.
Set the oscillator and USB wiring in board.h before use.
Hardware validation is pending.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include "app_evt_handler.h"
#include "coredev/system_core_clock.h"
#include "usb/usb.h"
#include "board.h"

class Ep0Test : public UsbDeviceClass
{
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
			uint8_t **ppData, uint16_t *pLength) override;
};

McuOsc_t g_McuOsc = MCUOSC;
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
static Ep0Test s_Class;
static uint8_t s_Reply[] = {0x4C, 0x50, 0x43, 0x31, 0x37, 0x36, 0x39, 0};
static const uint8_t s_Descriptor[] = {9, 4, 0, 0, 0, 0xFF, 0, 0, 0};
static const IOPinCfg_t s_Pins[] = USB_PINS_MAP;
static const UsbCfg_t s_Config = {
	.DevNo = 0,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0005,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "LPC1769 EP0 Demo",
	.pSerial = nullptr,
	.IntPrio = 6,
	.pIOPinMap = s_Pins,
	.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]),
	.MaxPower = 100,
};
bool Ep0Test::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
		uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup->bmRequestType != 0xC0 || pSetup->bRequest != 1)
		return false;
	if (Stage == USB_CTRL_SETUP)
	{
		*ppData = s_Reply;
		*pLength = sizeof(s_Reply);
	}
	return true;
}
int main(void)
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_Config) || !UsbClassRegister(0, &s_Class, 0, 1, 0, 0) ||
		!UsbDescRegister(0, &s_Class, s_Descriptor, sizeof(s_Descriptor), nullptr) ||
		!UsbEnable(0))
		return 1;
	AppRun();
}
