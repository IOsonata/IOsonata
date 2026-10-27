/**-------------------------------------------------------------------------
@file	usbd_periodic_test.cpp

@brief	Host test support: usbd periodic test.

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

// Host tests of the production device classes, descriptor core and transports.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "usb/usbd_int.h"
#include "usb/usbd_iso.h"
#include "usb/usbd_bulk.h"
#include "usb/usbd_epalloc.h"

static bool s_HighSpeed, s_Suspended, s_Vbus = true, s_IsoInitFail;
static unsigned s_OpenFail, s_Stalls;
static uint16_t s_Open[16][2];
static UsbCtrlrEpHandler_t s_Handler[16][2];
static void *s_Context[16][2];
bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *)
{
	return true;
}
bool UsbCtrlrStart(int)
{
	return true;
}
void UsbCtrlrStop(int)
{
}
void UsbCtrlrProcess(int)
{
}
bool UsbCtrlrVbusDetected(int)
{
	return s_Vbus;
}
bool UsbCtrlrHighSpeed(int)
{
	return s_HighSpeed;
}
bool UsbCtrlrIsoInit(int)
{
	return !s_IsoInitFail;
}
void UsbCtrlrIntEnable(int)
{
}
void UsbCtrlrIntDisable(int)
{
}
void UsbCtrlrConnect(int)
{
}
void UsbCtrlrDisconnect(int)
{
}
void UsbCtrlrRemoteWakeup(int)
{
}
void UsbCtrlrSofEnable(int, bool)
{
}
void UsbCtrlrSetAddress(int, uint8_t)
{
}
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *p)
{
	if (s_OpenFail && --s_OpenFail == 0)
		return false;
	const unsigned ep = p->bEndpointAddress & 15U;
	const bool in = (p->bEndpointAddress & 128U) != 0;
	assert(!s_Open[ep][in]);
	s_Open[ep][in] = p->wMaxPacketSize;
	return true;
}
void UsbCtrlrEpClose(int, uint8_t ep, bool in)
{
	s_Open[ep][in] = 0;
}
void UsbCtrlrEpCloseAll(int)
{
	memset(s_Open, 0, sizeof(s_Open));
}
void UsbCtrlrEpBind(int, uint8_t ep, bool in, bool, UsbCtrlrEpHandler_t h, void *c)
{
	s_Handler[ep][in] = h; s_Context[ep][in] = c;
}
bool UsbCtrlrEpReceive(int, uint8_t, uint8_t *, uint16_t)
{
	return true;
}
void UsbCtrlrEpProcessEvent(int, uint8_t ep, bool in, UsbCtrlrEvtType_t e, uint16_t n)
{
	if (s_Handler[ep][in])
		s_Handler[ep][in](e, n, s_Context[ep][in]);
}
bool UsbCtrlrEpSend(int, uint8_t ep, uint8_t *, uint16_t)
{
	return !s_Suspended && s_Open[ep][1];
}
bool UsbCtrlrIsoSend(int, uint8_t, uint8_t *, uint16_t)
{
	return true;
}
int UsbCtrlrEp0Send(int, uint8_t *, int n)
{
	return n;
}
bool UsbCtrlrEp0Status(int, uint8_t)
{
	return true;
}
void UsbCtrlrEpStall(int, uint8_t, bool)
{
	++s_Stalls;
}
void UsbCtrlrEpClearStall(int, uint8_t, bool)
{
}
size_t UsbCtrlrGetSerial(int, char *p, size_t n)
{
	if (n)
		*p = 0;
	return 0;
}
bool UsbEvtQue(uint32_t, void *, UsbEvtQueHandler_t)
{
	return true;
}

static const UsbdPeriodicAlt_t s_IntAlts[] = {{16, 32, 1, 4}, {64, 64, 4, 6}, {64, 64, 16, 8}};
static const UsbdPeriodicAlt_t s_IsoAlts[] = {{9, 9, 1, 4}, {17, 17, 1, 4}, {25, 25, 1, 4},
	{33, 33, 1, 4}, {49, 49, 1, 4}, {63, 63, 1, 4}};
alignas(4) static uint8_t rx[USB_INT_INTRF_PKT_BLKSIZE], tx[USB_INT_INTRF_PKT_BLKSIZE];
static UsbdIntCfg_t IntCfg()
{
	UsbdIntCfg_t c = {};
	c.Function = {0, s_IntAlts, 3, 2, 3, 4};
	c.pRxBuffer = rx; c.pTxBuffer = tx;
	return c;
}
static UsbdIsoCfg_t IsoCfg()
{
	UsbdIsoCfg_t c = {};
	c.Function = {0, s_IsoAlts, 6, 0, 0, 4};
	return c;
}
static void Init()
{
	UsbDisable(0);
	UsbCfg_t c = {};
	c.Vid = 0x1209; c.Pid = 3;
	assert(UsbInit(&c));
	memset(s_Open, 0, sizeof(s_Open));
	s_Stalls = s_OpenFail = 0;
	s_Suspended = false; s_Vbus = true;
}
static void Event(UsbCtrlrEvtType_t type)
{
	UsbCtrlrEvt_t e = {}; e.Type = type;
	UsbDevProcessEvent(0, &e);
	UsbProcess(0);
}
static bool Setup(uint8_t type, uint8_t req, uint16_t value, uint16_t index = 0)
{
	unsigned stalls = s_Stalls;
	UsbCtrlrEvt_t e = {}; e.Type = USB_CTRLR_EVT_SETUP;
	e.Setup.bmRequestType = type; e.Setup.bRequest = req;
	e.Setup.wValue = value; e.Setup.wIndex = index;
	UsbDevProcessEvent(0, &e);
	if (stalls != s_Stalls)
		return false;
	e.Type = USB_CTRLR_EVT_XFER_CMPL;
	e.Xfer = {}; e.Xfer.EpAddr = 0x80; e.Xfer.Result = USB_CTRLR_XFER_SUCCESS;
	UsbDevProcessEvent(0, &e);
	return true;
}
static void Enumerate()
{
	assert(UsbEnable(0));
	assert(Setup(0, USB_REQ_SET_ADDRESS, 5));
	assert(Setup(0, USB_REQ_SET_CONFIGURATION, 1));
}
static void CheckDesc(UsbdPeriodic &fn, const UsbdPeriodicCfg_t &cfg, uint8_t type)
{
	std::vector<uint8_t> bytes(fn.DescriptorLength() + 2, 0xA5);
	fn.BuildDescriptor(bytes.data() + 1, s_HighSpeed ? USB_SPEED_HIGH : USB_SPEED_FULL);
	assert(bytes.front() == 0xA5 && bytes.back() == 0xA5);
	const uint8_t *p = bytes.data() + 1;
	assert(p[0] == 9 && p[1] == 4 && p[2] == fn.FirstInterface() && p[3] == 0 && p[4] == 0);
	assert(p[5] == 255 && p[6] == cfg.SubClass && p[7] == cfg.Protocol && p[8] == cfg.InterfaceString);
	p += 9;
	for (unsigned i = 0; i < cfg.AltCount; ++i)
	{
		assert(p[0] == 9 && p[1] == 4 && p[2] == fn.FirstInterface() && p[3] == i+1 && p[4] == 2);
		p += 9;
		for (unsigned dir = 0; dir < 2; ++dir)
		{
			assert(p[0] == 7 && p[1] == 5 && p[3] == type);
			assert((p[2] & 128U) == dir * 128U);
			assert((1U << (p[2] & 15U)) == (dir ? fn.EpInMask() : fn.EpOutMask()));
			const auto &a = cfg.pAlternates[i];
			assert((p[4] | (p[5] << 8)) == (s_HighSpeed ? a.HsMps : a.FsMps));
			assert(p[6] == (s_HighSpeed ? a.HsInterval : a.FsInterval));
			p += 7;
		}
	}
	assert(p == bytes.data() + bytes.size() - 1);
}
class Blocker : public UsbDeviceClass
{
};
static void Failures()
{
	Init();
	UsbdInt intr; auto c = IntCfg();
	c.pRxBuffer = nullptr; assert(!intr.Init(c));
	c = IntCfg(); c.pTxBuffer = rx; assert(!intr.Init(c));
	c = IntCfg(); c.Function.DevNo = -1; assert(!intr.Init(c));
	c = IntCfg(); c.Function.AltCount = 0; assert(!intr.Init(c));
	UsbdPeriodicAlt_t bad = {65, 64, 1, 1};
	c = IntCfg(); c.Function.pAlternates = &bad; c.Function.AltCount = 1;
	assert(!intr.Init(c)); bad.FsMps = 16; bad.FsInterval = 0; assert(!intr.Init(c));
	if (USB_HIGHSPEED_CAPABLE(0))
	{
		bad.FsInterval = 1; bad.HsInterval = 17; assert(!intr.Init(c));
		bad.HsInterval = 1; bad.HsMps = 0x840; assert(!intr.Init(c));
	}
	assert(intr.Init(IntCfg())); assert(intr.FirstInterface() == 0 && intr.EpInMask() == 2);
	assert(!intr.Init(IntCfg())); // Duplicate Init must preserve the original object.
	CheckDesc(intr, IntCfg().Function, INT);
	UsbdIso iso; auto ic = IsoCfg();
	ic.pRxFifoMem = rx; assert(!iso.Init(ic));
	// Controller ISO initialization failure after reservation must roll back.
	s_IsoInitFail = true; assert(!iso.Init(IsoCfg())); s_IsoInitFail = false;
	assert(iso.Init(IsoCfg())); assert(iso.FirstInterface() == 1);
	assert(iso.EpInMask() == (1U << 8));
	UsbdIso second; assert(second.Init(IsoCfg())); assert(second.EpInMask() == (1U << 9));
	UsbdIso third; assert(!third.Init(IsoCfg()));
	assert(intr.FirstInterface() == 0 && second.FirstInterface() == 2);
	// Last-registration rollback is unavailable after the bus is enabled.
	Enumerate(); assert(!UsbdClassRollback(0, &second));
	UsbDisable(0);

	Init();
	Blocker huge;
	assert(UsbClassRegister(0, &huge, 0, 1, 0, 0));
	static uint8_t filler[USB_CONFIG_DESC_MAXLEN - 9 - 8] = {};
	assert(UsbDescRegister(0, &huge, filler, sizeof(filler), nullptr));
	UsbdInt noRoom; assert(!noRoom.Init(IntCfg()));
	assert(noRoom.InterfaceCount() == 0 && noRoom.DescriptorLength() == 0);
	assert(UsbdClassRollback(0, &huge));
	assert(noRoom.Init(IntCfg())); assert(noRoom.FirstInterface() == 0 && noRoom.EpInMask() == 2);
	UsbDisable(0);

	Init();
	Blocker full;
	assert(UsbClassRegister(0, &full, 0, 16, 0xFFFE, 0xFFFE));
	UsbdInt denied; assert(!denied.Init(IntCfg()));
	assert(UsbdClassRollback(0, &full));
	assert(denied.Init(IntCfg())); assert(denied.FirstInterface() == 0);
	UsbDisable(0);

	Init();
	Blocker classes[USB_EPIN_CNT(0)];
	for (unsigned i = 0; i < USB_EPIN_CNT(0); ++i)
	{
		assert(UsbClassRegister(0, &classes[i], 0, 0, 0, 0));
	}
	UsbdInt classFull; assert(!classFull.Init(IntCfg()));
	assert(UsbdClassRollback(0, &classes[USB_EPIN_CNT(0)-1]));
	assert(classFull.Init(IntCfg())); assert(classFull.FirstInterface() == 0);
	UsbDisable(0);

	if (USB_HIGHSPEED_CAPABLE(0))
	{
		Init();
		UsbdInt maximum;
		UsbdPeriodicAlt_t maxAlt = {64, 1024, 255, 16};
		auto maxCfg = IntCfg();
		maxCfg.Function.pAlternates = &maxAlt; maxCfg.Function.AltCount = 1;
		assert(maximum.Init(maxCfg));
		s_HighSpeed = true;
		Enumerate();
		assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 1));
		assert(s_Open[1][0] == 1024 && s_Open[1][1] == 1024);
		CheckDesc(maximum, maxCfg.Function, INT);
		UsbDisable(0); s_HighSpeed = false;
	}
}
static void Lifecycle()
{
	Init();
	UsbdInt intr; UsbdIso iso;
	assert(intr.Init(IntCfg())); assert(iso.Init(IsoCfg()));
	CheckDesc(intr, IntCfg().Function, INT); CheckDesc(iso, IsoCfg().Function, ISO);
	uint16_t length = 0;
	const auto *desc = UsbGetDescriptor(0, USB_DESCTYPE_CONFIGURATION, 0, 0,
		s_HighSpeed ? USB_SPEED_HIGH : USB_SPEED_FULL, &length);
	assert(desc && length == 9 + intr.DescriptorLength() + iso.DescriptorLength() && desc[4] == 2);
	assert(!iso.SelectInterface(1, 1));
	Enumerate();
	assert(intr.Configured() && iso.Configured() && !intr.Alternate() && !iso.Alternate());
	for (unsigned alt = 1; alt <= 6; ++alt)
	{
		assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, alt, 1));
		assert(iso.Alternate() == alt && UsbGetAlternate(0, 1) == alt);
		assert(s_Open[8][0] == s_IsoAlts[alt-1].FsMps && s_Open[8][1] == s_Open[8][0]);
	}
	assert(!Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 7, 1));
	assert(iso.Alternate() == 6 && s_Open[8][0] == 63);
	assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 1, 0));
	assert(s_Open[1][0] == (s_HighSpeed ? 32 : 16));
	for (unsigned fail = 1; fail <= 2; ++fail)
	{
		s_OpenFail = fail;
		assert(!Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 2, 0));
		assert(intr.Alternate() == 1 && UsbGetAlternate(0, 0) == 1);
		assert(s_Open[1][0] == (s_HighSpeed ? 32 : 16) && s_Open[1][1] == s_Open[1][0]);
	}
	uint8_t frame[9] = {};
	assert(iso.Tx(0, frame, 9) == 9);
	s_Suspended = true; Event(USB_CTRLR_EVT_SUSPEND);
	assert(static_cast<UsbIsoIntrf_t *>(iso)->Suspended);
	assert(iso.Tx(0, frame, 9) == 0);
	assert(intr.Tx(0, frame, 9) == 0);
	assert(intr.Alternate() == 1);
	s_Suspended = false; Event(USB_CTRLR_EVT_RESUME);
	assert(!static_cast<UsbIsoIntrf_t *>(iso)->Suspended);
	assert(iso.Tx(0, frame, 9) == 9);
	assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 0, 1));
	assert(!s_Open[8][0] && !s_Open[8][1] && !iso.TxReady());
	assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 1, 1));
	assert(iso.TxReady());
	Event(USB_CTRLR_EVT_RESET);
	assert(!intr.Configured() && !iso.Configured() && !s_Open[1][0] && !s_Open[8][0]);
	Enumerate();
	assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 1, 1));
	assert(Setup(0, USB_REQ_SET_CONFIGURATION, 0));
	assert(!iso.Configured() && !s_Open[8][0]);
	assert(Setup(0, USB_REQ_SET_CONFIGURATION, 1));
	assert(Setup(USB_REQTYPE_INTERFACE, USB_REQ_SET_INTERFACE, 1, 1));
	s_Vbus = false; UsbProcess(0);
	assert(!iso.Configured() && !s_Open[8][0]);
	s_Vbus = true; Event(USB_CTRLR_EVT_RESET); Enumerate();
	assert(iso.Configured() && iso.Alternate() == 0);
	UsbDisable(0);
}
int main()
{
	Failures(); Lifecycle();
	if (USB_HIGHSPEED_CAPABLE(0))
	{
		s_HighSpeed = true;
		Lifecycle();
	}
	puts("PASS periodic classes: descriptors, allocation/rollback, alternate failure, reset, reconnect, suspend/resume");
	return 0;
}
