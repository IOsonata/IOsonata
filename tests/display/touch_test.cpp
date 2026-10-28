/**-------------------------------------------------------------------------
@file	touch_test.cpp

@brief	Host test of the touch panel drivers, FT5406 and GT911.

Run by tests/display/run_touch.py.

I2C bus model with an FT5406 and a GT911, each present or not, and a model of
the reset and interrupt pins. The FT5406 is reset by its reset pin low 5 ms
or more and answers 300 ms after it goes high; without the reset pin it is
running. The GT911 takes its address from the interrupt pin when the reset
pin goes high (low 0x5D, high 0x14; the board pull when the host does not
drive it), needs the reset low 10 ms or more, the interrupt level set 100 us
or more before the rise and, when the host drives the interrupt pin, that
pin low 50 ms from 5 ms after the rise before it is released. It answers 55
ms after the rise, with the interrupt pin released by the host. In hibernate (FT5406) or sleep (GT911) a controller
answers only after a reset. Each transfer is checked against the register
map: register address size, the FT5406 contacts in one 32 byte read, the
GT911 status cleared after the contacts are read.

The coordinates are checked against the frame buffer mapping of
LcdLpc546xx in the four orientations: a contact at a panel pixel drawn at
the point read lands on that pixel.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "idelay.h"
#include "display/touch_ft5406.h"
#include "display/touch_gt911.h"

// ---- Time and pins ----

static uint64_t s_Us;

extern "C" void usDelay(uint32_t Delay)
{
	s_Us += Delay;
}

extern "C" void msDelay(uint32_t Delay)
{
	s_Us += Delay * 1000ULL;
}

struct Pin {
	bool Out = false;			// Driven by the host
	bool Level = true;			// Output level
	uint64_t Changed = 0;		// Time of the last change of the driven level
	uint64_t LowFrom = 0;		// Driven low since
	int Configs = 0;
};

#define RST_PORT	2
#define RST_PIN		27
#define INT_PORT	4
#define INT_PIN		0

static Pin s_Rst, s_Int;
static int s_OtherPins;
static void RstEdge(bool bHigh);
static void IntRelease(bool bWasLow);

static Pin *FindPin(int Port, int PinNo)
{
	if (Port == RST_PORT && PinNo == RST_PIN)
	{
		return &s_Rst;
	}
	if (Port == INT_PORT && PinNo == INT_PIN)
	{
		return &s_Int;
	}
	s_OtherPins++;
	return nullptr;
}

// Level seen on the pin
static bool PinLevel(const Pin &P, bool Pull)
{
	return P.Out ? P.Level : Pull;
}

static void PinDrive(Pin *p, bool Out, bool Level)
{
	if (p == nullptr)
	{
		return;
	}
	bool before = PinLevel(*p, true);
	bool beforeLow = PinLevel(*p, false);
	bool wasOut = p->Out;
	bool drivenLow = p->Out && !p->Level;

	if (Out && !Level && !drivenLow)
	{
		p->LowFrom = s_Us;
	}
	p->Out = Out;
	p->Level = Level;
	if (PinLevel(*p, true) != before || PinLevel(*p, false) != beforeLow)
	{
		p->Changed = s_Us;
	}
	if (p == &s_Rst && Out && before != Level)
	{
		RstEdge(Level);
	}
	if (p == &s_Int && wasOut && !Out)
	{
		IntRelease(drivenLow);
	}
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	Pin *p = FindPin(PortNo, PinNo);

	assert(PinOp == IOPINOP_GPIO && Resistor == IOPINRES_NONE && Type == IOPINTYPE_NORMAL);
	if (p != nullptr)
	{
		p->Configs++;
		PinDrive(p, Dir == IOPINDIR_OUTPUT, p->Level);
	}
}

void IOPinDisable(int PortNo, int PinNo)
{
	(void)PortNo;
	(void)PinNo;
	assert(false);
}

extern "C" void IOPinSet(int Port, int PinNo)
{
	Pin *p = FindPin(Port, PinNo);

	if (p != nullptr)
	{
		PinDrive(p, p->Out, true);
	}
}

extern "C" void IOPinClear(int Port, int PinNo)
{
	Pin *p = FindPin(Port, PinNo);

	if (p != nullptr)
	{
		PinDrive(p, p->Out, false);
	}
}

// ---- Controllers ----

struct Ft5406 {
	bool Present = false;
	bool Wired = true;			// Reset pin connected
	bool Running = true;		// Out of reset long ago
	uint64_t Up = 0;			// Answers from this time
	uint64_t LowSince = 0;
	bool Hibernate = false;
	uint8_t Reg[256] = {};
	uint8_t ModeRead = 0;		// DEVICE_MODE read back
	int Writes = 0;
	int Reads = 0;
	int Resets = 0;
	int PmodeWrites = 0;
};

struct Gt911 {
	bool Present = false;
	bool Wired = true;
	bool IntPullHigh = false;	// Board pull of the interrupt pin
	uint8_t Addr = 0x5D;		// Address before any reset
	bool Running = true;
	uint64_t Up = 0;
	uint64_t LowSince = 0;
	uint64_t Rise = 0;
	bool Synced = true;			// Interrupt pin sequence done
	bool Sleep = false;
	char Id[4] = { '9', '1', '1', 0 };
	uint16_t XMax = 480, YMax = 272;
	uint8_t Status = 0;
	uint8_t Points[80] = {};
	int StatusClears = 0;
	int PointReads = 0;
	int Resets = 0;
	int Sleeps = 0;
};

static Ft5406 s_Ft;
static Gt911 s_Gt;

// The reset pin is shared by the two controllers, as on the board connector
static void RstEdge(bool bHigh)
{
	if (bHigh == false)
	{
		s_Ft.LowSince = s_Gt.LowSince = s_Us;
		return;
	}
	if (s_Ft.Wired && s_Us - s_Ft.LowSince >= 5000)
	{
		s_Ft.Running = false;
		s_Ft.Up = s_Us + 300000;
		s_Ft.Hibernate = false;
		s_Ft.Resets++;
	}
	if (s_Gt.Wired && s_Us - s_Gt.LowSince >= 10000)
	{
		// The interrupt level must be stable 100 us before the rise
		if (s_Int.Out)
		{
			assert(s_Us - s_Int.Changed >= 100);
		}
		s_Gt.Addr = PinLevel(s_Int, s_Gt.IntPullHigh) ? 0x14 : 0x5D;
		s_Gt.Rise = s_Us;
		s_Gt.Synced = !s_Int.Out;
		s_Gt.Running = false;
		s_Gt.Up = s_Us + 55000;
		s_Gt.Sleep = false;
		s_Gt.Status = 0;
		s_Gt.Resets++;
	}
}

// The interrupt pin released after the reset: low 50 ms from 5 ms after the
// rise
static void IntRelease(bool bWasLow)
{
	uint64_t from = s_Int.LowFrom > s_Gt.Rise + 5000 ? s_Int.LowFrom : s_Gt.Rise + 5000;

	if (!s_Gt.Running && bWasLow && s_Us >= from && s_Us - from >= 50000)
	{
		s_Gt.Synced = true;
	}
}

static bool FtAnswers(void)
{
	if (!s_Ft.Present || (s_Ft.Wired && s_Rst.Out && !s_Rst.Level) || s_Ft.Hibernate)
	{
		return false;
	}
	return s_Ft.Running || s_Us >= s_Ft.Up;
}

static bool GtAnswers(void)
{
	if (!s_Gt.Present || (s_Gt.Wired && s_Rst.Out && !s_Rst.Level) || s_Gt.Sleep)
	{
		return false;
	}
	return s_Gt.Running || (s_Us >= s_Gt.Up && s_Gt.Synced);
}

class Bus : public DeviceIntrf {
public:
	int Fail = 0;				// Transfer number that fails, 0 none
	int Transfers = 0;

	operator DevIntrf_t *() { return nullptr; }
	uint32_t Rate(uint32_t Value) { return Value; }
	uint32_t Rate() { return 100000; }
	DEVINTRF_TYPE Type() { return DEVINTRF_TYPE_I2C; }

	int Read(uint32_t Addr, const uint8_t *pReg, int RegLen, uint8_t *pData, int Len)
	{
		if (++Transfers == Fail)
		{
			return 0;
		}
		if (Addr == 0x38 && FtAnswers())
		{
			assert(RegLen == 1);
			s_Ft.Reads++;
			if (pReg[0] == 0x01)
			{
				// The contacts in one transfer
				assert(Len == 32);
			}
			for (int i = 0; i < Len; i++)
			{
				pData[i] = pReg[0] + i == 0 ? s_Ft.ModeRead : s_Ft.Reg[(pReg[0] + i) & 0xFF];
			}
			return Len;
		}
		if (Addr == s_Gt.Addr && GtAnswers())
		{
			assert(RegLen == 2);
			uint16_t reg = (uint16_t)(pReg[0] << 8 | pReg[1]);

			// The host released the interrupt pin
			assert(!s_Int.Out);
			if (reg == 0x8140)
			{
				assert(Len == 4);
				memcpy(pData, s_Gt.Id, 4);
			}
			else if (reg == 0x8047)
			{
				assert(Len == 5);
				uint8_t c[5] = { 0x41, (uint8_t)s_Gt.XMax, (uint8_t)(s_Gt.XMax >> 8), (uint8_t)s_Gt.YMax,
								 (uint8_t)(s_Gt.YMax >> 8) };
				memcpy(pData, c, 5);
			}
			else if (reg == 0x814E)
			{
				assert(Len == 1);
				pData[0] = s_Gt.Status;
			}
			else
			{
				// The contacts, only those reported
				assert(reg == 0x814F && (s_Gt.Status & 0x80) && Len == 8 * (s_Gt.Status & 0x0F));
				memcpy(pData, s_Gt.Points, Len);
				s_Gt.PointReads++;
			}
			return Len;
		}
		return 0;
	}

	int Write(uint32_t Addr, const uint8_t *pReg, int RegLen, const uint8_t *pData, int Len)
	{
		if (++Transfers == Fail)
		{
			return 0;
		}
		if (Addr == 0x38 && FtAnswers())
		{
			assert(RegLen == 1 && Len == 1);
			s_Ft.Writes++;
			if (pReg[0] == 0xA5)
			{
				s_Ft.PmodeWrites++;
				if (pData[0] == 3)
				{
					s_Ft.Hibernate = true;
				}
			}
			s_Ft.Reg[pReg[0]] = pData[0];
			return Len;
		}
		if (Addr == s_Gt.Addr && GtAnswers())
		{
			assert(RegLen == 2 && Len == 1);
			uint16_t reg = (uint16_t)(pReg[0] << 8 | pReg[1]);

			if (reg == 0x814E)
			{
				assert(pData[0] == 0 && !s_Int.Out);
				s_Gt.Status = 0;
				s_Gt.StatusClears++;
			}
			else
			{
				assert(reg == 0x8040 && pData[0] == 5);
				// Sleep with the interrupt pin held low
				assert(s_Int.Out && !s_Int.Level);
				s_Gt.Sleep = true;
				s_Gt.Sleeps++;
			}
			return Len;
		}
		return 0;
	}
};

// ---- Helpers ----

static TouchCfg_t Cfg(bool bPins, uint32_t Flags = 0)
{
	TouchCfg_t c;

	memset(&c, 0, sizeof(c));
	c.Width = 480;
	c.Height = 272;
	c.Flags = Flags;
	c.RstPin = { RST_PORT, RST_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	c.IntPin = { INT_PORT, INT_PIN, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	if (!bPins)
	{
		c.RstPin.PortNo = -1;
		c.IntPin.PortNo = -1;
	}
	return c;
}

static void Fresh(void)
{
	s_Ft = Ft5406();
	s_Gt = Gt911();
	s_Rst = Pin();
	s_Int = Pin();
	s_Int.Level = false;
	s_OtherPins = 0;
}

// FT5406 contact i: event, x, y, id
static void FtPoint(int i, int Evt, int X, int Y, int Id)
{
	uint8_t *p = &s_Ft.Reg[3 + 6 * i];

	p[0] = (uint8_t)(Evt << 6 | ((X >> 8) & 0x0F));
	p[1] = (uint8_t)X;
	p[2] = (uint8_t)(Id << 4 | ((Y >> 8) & 0x0F));
	p[3] = (uint8_t)Y;
	p[4] = 0xEE;
	p[5] = 0xEE;
}

static void GtPoints(int Count, bool bReady = true)
{
	s_Gt.Status = (uint8_t)((bReady ? 0x80 : 0) | Count);
}

static void GtPoint(int i, int X, int Y, int Id)
{
	uint8_t *p = &s_Gt.Points[8 * i];

	p[0] = (uint8_t)Id;
	p[1] = (uint8_t)X;
	p[2] = (uint8_t)(X >> 8);
	p[3] = (uint8_t)Y;
	p[4] = (uint8_t)(Y >> 8);
	p[5] = 0x20;
	p[6] = 0;
	p[7] = 0xEE;
}

// Frame buffer pixel of display point X, Y: LcdLpc546xx::Pixel
static void Pixel(DISPL_ORIENT Orient, int W, int H, int X, int Y, int &Px, int &Py)
{
	Px = X;
	Py = Y;
	switch (Orient)
	{
		case DISPL_ORIENT_LANDSCAPE_INV:
			Px = W - 1 - X;
			Py = H - 1 - Y;
			break;
		case DISPL_ORIENT_PORTRAIT:
			Px = W - 1 - Y;
			Py = X;
			break;
		case DISPL_ORIENT_PORTRAIT_INV:
			Px = Y;
			Py = H - 1 - X;
			break;
		default:
			break;
	}
}

static const DISPL_ORIENT s_Orients[] = {
	DISPL_ORIENT_LANDSCAPE, DISPL_ORIENT_PORTRAIT, DISPL_ORIENT_LANDSCAPE_INV, DISPL_ORIENT_PORTRAIT_INV,
};

// ---- Tests ----

static void TestFtInit()
{
	Bus bus;
	TouchFt5406 t;
	TouchCfg_t c;
	TouchPoint_t pt[5];

	Fresh();
	s_Ft.Present = true;
	assert(!t.Init(Cfg(true), nullptr));
	c = Cfg(true); c.Width = 0; assert(!t.Init(c, &bus));
	c = Cfg(true); c.Height = 0; assert(!t.Init(c, &bus));
	c = Cfg(true); c.DevAddr = 0x80; assert(!t.Init(c, &bus));
	c = Cfg(true); c.Flags = 8; assert(!t.Init(c, &bus));
	assert(s_Rst.Configs == 0 && s_Int.Configs == 0 && bus.Transfers == 0);
	assert(t.Read(pt, 5) == -1 && !t.Enable());
	t.Disable();
	t.Reset();
	assert(bus.Transfers == 0);

	// Reset pulse of 5 ms, 300 ms before the first transfer
	uint64_t t0 = s_Us;
	assert(t.Init(Cfg(true), &bus) && t.Valid());
	assert(s_Ft.Resets == 1 && s_Us - t0 >= 305000 && s_Rst.Out && s_Rst.Level && !s_Int.Out);
	assert(t.DeviceAddress() == 0x38 && s_Ft.Reg[0] == 0 && s_Ft.Writes == 1);
	assert(s_OtherPins == 0);
	// The interrupt pin left driven: released
	Fresh();
	s_Ft.Present = true;
	s_Int.Out = true;
	assert(t.Init(Cfg(true), &bus) && !s_Int.Out);
	// Chip id from ID_G_CIPHER
	Fresh();
	s_Ft.Present = true;
	s_Ft.Reg[0xA3] = 0x0A;
	assert(t.Init(Cfg(true), &bus) && t.DeviceID() == 0x0A);
	// No pins: running, no wait, no pin set up
	Fresh();
	s_Ft.Present = true;
	t0 = s_Us;
	assert(t.Init(Cfg(false), &bus) && s_Us == t0 && s_Rst.Configs == 0 && s_Int.Configs == 0 && s_OtherPins == 0);
	// Not there, another address, not in the normal mode, each transfer
	// failing
	Fresh();
	assert(!t.Init(Cfg(true), &bus) && !t.Valid() && t.Read(pt, 5) == -1);
	int n = bus.Transfers;
	t.Disable();
	assert(bus.Transfers == n);
	Fresh();
	s_Ft.Present = true;
	c = Cfg(true);
	c.DevAddr = 0x39;
	assert(!t.Init(c, &bus) && t.DeviceAddress() == 0x39);
	Fresh();
	s_Ft.Present = true;
	s_Ft.ModeRead = 0x10;
	assert(!t.Init(Cfg(true), &bus));
	s_Ft.ModeRead = 0x8F;
	assert(t.Init(Cfg(true), &bus));
	s_Ft.ModeRead = 0;
	for (int f = 1; f <= 3; f++)
	{
		Fresh();
		s_Ft.Present = true;
		bus.Transfers = 0;
		bus.Fail = f;
		assert(!t.Init(Cfg(true), &bus) && !t.Valid());
	}
	bus.Fail = 0;
	printf("FT5406 init: PASS\n");
}

static void TestFtRead()
{
	Bus bus;
	TouchFt5406 t;
	TouchPoint_t pt[6];

	Fresh();
	s_Ft.Present = true;
	assert(t.Init(Cfg(true), &bus));
	// Not touched
	assert(t.Read(pt, 5) == 0);
	// 5 contacts, 12 bit coordinates, ids
	static const int xs[] = { 0, 479, 256, 300, 17 }, ys[] = { 0, 271, 128, 1, 270 };
	for (int i = 0; i < 5; i++)
	{
		FtPoint(i, i & 1 ? 2 : 0, xs[i], ys[i], i + 3);
	}
	s_Ft.Reg[2] = 5;
	memset(pt, 0, sizeof(pt));
	assert(t.Read(pt, 6) == 5);
	for (int i = 0; i < 5; i++)
	{
		assert(pt[i].X == xs[i] && pt[i].Y == ys[i] && pt[i].Id == i + 3);
	}
	// Bits 5:4 of X high are not part of X
	s_Ft.Reg[3 + 6] |= 0x30;
	assert(t.Read(pt, 6) == 5 && pt[1].X == 479);
	// Fewer asked, none asked, the status high bits, more than 5 reported
	assert(t.Read(pt, 2) == 2 && pt[1].X == 479);
	assert(t.Read(pt, 0) == 0 && t.Read(pt, -1) == -1 && t.Read(nullptr, 5) == -1);
	s_Ft.Reg[2] = 0xF2;
	assert(t.Read(pt, 5) == 2);
	s_Ft.Reg[2] = 6;
	assert(t.Read(pt, 5) == 0);
	// Lifted (1) and no event (3) are not contacts
	s_Ft.Reg[2] = 3;
	FtPoint(0, 1, 10, 10, 0);
	FtPoint(1, 3, 20, 20, 1);
	FtPoint(2, 0, 30, 40, 2);
	assert(t.Read(pt, 5) == 1 && pt[0].X == 30 && pt[0].Y == 40 && pt[0].Id == 2);
	s_Ft.Reg[2] = 2;
	FtPoint(0, 2, 11, 12, 0);
	FtPoint(1, 0, 13, 14, 1);
	assert(t.Read(pt, 1) == 1 && pt[0].X == 11);
	// Out of the panel: dropped
	s_Ft.Reg[2] = 3;
	FtPoint(0, 0, 480, 10, 0);
	FtPoint(1, 0, 10, 272, 1);
	FtPoint(2, 2, 479, 271, 2);
	assert(t.Read(pt, 5) == 1 && pt[0].X == 479 && pt[0].Y == 271);
	// The transfer fails
	bus.Transfers = 0;
	bus.Fail = 1;
	assert(t.Read(pt, 5) == -1);
	bus.Fail = 0;
	assert(t.Read(pt, 5) == 1);
	printf("FT5406 read: PASS\n");
}

static void TestFtPower()
{
	Bus bus;
	TouchFt5406 t;
	TouchPoint_t pt[5];

	Fresh();
	s_Ft.Present = true;
	assert(t.Init(Cfg(true), &bus));
	s_Ft.Reg[2] = 1;
	FtPoint(0, 0, 100, 50, 1);
	assert(t.Enable() && s_Ft.Resets == 1);
	// Hibernate, nothing read, then woken up by a reset
	t.Disable();
	assert(s_Ft.Hibernate && s_Ft.PmodeWrites == 1);
	int n = bus.Transfers;
	t.Disable();
	assert(s_Ft.PmodeWrites == 1 && bus.Transfers == n);
	assert(t.Read(pt, 5) == 0);
	uint64_t t0 = s_Us;
	assert(t.Enable() && s_Ft.Resets == 2 && !s_Ft.Hibernate && s_Us - t0 >= 305000);
	assert(t.Read(pt, 5) == 1 && pt[0].X == 100);
	assert(t.Enable() && s_Ft.Resets == 2);
	// Woken up without answer: still asleep, Enable again
	t.Disable();
	s_Ft.Present = false;
	assert(!t.Enable() && t.Read(pt, 5) == 0);
	s_Ft.Present = true;
	assert(t.Enable() && t.Read(pt, 5) == 1);
	// Reset: the controller set up again
	s_Ft.Reg[0] = 0x40;
	t.Reset();
	assert(s_Ft.Resets == 5 && s_Ft.Reg[0] == 0 && t.Read(pt, 5) == 1);
	s_Ft.Present = false;
	t.Reset();
	assert(t.Read(pt, 5) == 0);
	s_Ft.Present = true;
	assert(t.Enable() && t.Read(pt, 5) == 1);
	// Init after Disable
	t.Disable();
	assert(t.Init(Cfg(true), &bus) && t.Read(pt, 5) == 1);
	// No reset pin: Disable does nothing
	Fresh();
	s_Ft.Present = true;
	s_Ft.Reg[2] = 1;
	FtPoint(0, 0, 7, 8, 1);
	assert(t.Init(Cfg(false), &bus));
	t.Disable();
	assert(s_Ft.PmodeWrites == 0 && t.Read(pt, 5) == 1 && t.Enable());
	t.Reset();
	assert(t.Read(pt, 5) == 1 && s_Ft.Writes == 2);
	printf("FT5406 power: PASS\n");
}

static void TestMapping()
{
	Bus bus;
	TouchFt5406 t;
	TouchPoint_t pt[5];

	// Each axis set up and orientation: the point read lands on the panel
	// pixel touched
	for (uint32_t flags = 0; flags < 8; flags++)
	{
		Fresh();
		s_Ft.Present = true;
		assert(t.Init(Cfg(true, flags), &bus));
		static const int px[] = { 0, 479, 0, 479, 123, 400 }, py[] = { 0, 0, 271, 271, 45, 200 };
		for (DISPL_ORIENT o : s_Orients)
		{
			t.Orientation(o);
			assert(t.Orientation() == o);
			bool portrait = o == DISPL_ORIENT_PORTRAIT || o == DISPL_ORIENT_PORTRAIT_INV;
			assert(t.Width() == (portrait ? 272 : 480) && t.Height() == (portrait ? 480 : 272));
			for (int i = 0; i < 6; i++)
			{
				// Controller coordinates of the panel pixel
				int x = flags & TOUCH_FLAG_X_INV ? 479 - px[i] : px[i];
				int y = flags & TOUCH_FLAG_Y_INV ? 271 - py[i] : py[i];
				if (flags & TOUCH_FLAG_XY_SWAP)
				{
					int s = x;
					x = y;
					y = s;
				}
				s_Ft.Reg[2] = 1;
				FtPoint(0, 0, x, y, 9);
				assert(t.Read(pt, 5) == 1 && pt[0].Id == 9);
				assert(pt[0].X < t.Width() && pt[0].Y < t.Height());
				int fx, fy;
				Pixel(o, 480, 272, pt[0].X, pt[0].Y, fx, fy);
				assert(fx == px[i] && fy == py[i]);
			}
		}
		// Out of the controller range with the swap
		s_Ft.Reg[2] = 1;
		FtPoint(0, 0, flags & TOUCH_FLAG_XY_SWAP ? 272 : 480, 0, 0);
		assert(t.Read(pt, 5) == 0);
		FtPoint(0, 0, 0, flags & TOUCH_FLAG_XY_SWAP ? 480 : 272, 0);
		assert(t.Read(pt, 5) == 0);
		FtPoint(0, 0, flags & TOUCH_FLAG_XY_SWAP ? 271 : 479, flags & TOUCH_FLAG_XY_SWAP ? 479 : 271, 0);
		assert(t.Read(pt, 5) == 1);
	}
	printf("orientation and axes: PASS\n");
}

static void TestGtInit()
{
	Bus bus;
	TouchGt911 t;
	TouchCfg_t c;
	TouchPoint_t pt[10];

	Fresh();
	s_Gt.Present = true;
	assert(!t.Init(Cfg(true), nullptr) && t.Read(pt, 5) == -1 && !t.Enable());
	c = Cfg(true); c.Width = 0; assert(!t.Init(c, &bus));
	c = Cfg(true); c.DevAddr = 0x38; assert(!t.Init(c, &bus));
	c = Cfg(true); c.DevAddr = 0x80; assert(!t.Init(c, &bus));
	assert(bus.Transfers == 0);
	t.Disable();
	t.Reset();
	assert(bus.Transfers == 0 && s_Gt.Resets == 0);

	// Address 0x5D: reset sequence, the pins left released
	uint64_t t0 = s_Us;
	s_Gt.Addr = 0x14;
	assert(t.Init(Cfg(true), &bus) && t.Valid() && t.DeviceAddress() == 0x5D && t.DeviceID() == 911);
	assert(s_Gt.Resets == 1 && s_Gt.Addr == 0x5D && s_Rst.Out && s_Rst.Level && !s_Int.Out);
	assert(s_Us - t0 >= 76000 && s_OtherPins == 0);
	// Address 0x14
	Fresh();
	s_Gt.Present = true;
	c = Cfg(true);
	c.DevAddr = 0x14;
	assert(t.Init(c, &bus) && s_Gt.Addr == 0x14 && t.DeviceAddress() == 0x14);
	// No pins: the address the board gives
	Fresh();
	s_Gt.Present = true;
	s_Gt.Addr = 0x14;
	t0 = s_Us;
	assert(!t.Init(Cfg(false), &bus) && s_Us == t0 && s_Rst.Configs == 0 && s_OtherPins == 0);
	c = Cfg(false);
	c.DevAddr = 0x14;
	assert(t.Init(c, &bus));
	// Only the reset pin: the board pull sets the address
	Fresh();
	s_Gt.Present = true;
	s_Gt.IntPullHigh = true;
	c = Cfg(true);
	c.IntPin.PortNo = -1;
	c.DevAddr = 0x14;
	assert(t.Init(c, &bus) && s_Gt.Addr == 0x14 && s_Int.Configs == 0 && s_OtherPins == 0);
	t.Disable();
	assert(s_Gt.Sleeps == 0 && s_OtherPins == 0 && t.Enable());
	// Not there, another product, each transfer failing
	Fresh();
	assert(!t.Init(Cfg(true), &bus));
	Fresh();
	s_Gt.Present = true;
	for (int i = 0; i < 3; i++)
	{
		memcpy(s_Gt.Id, "911", 4);
		s_Gt.Id[i] = '8';
		assert(!t.Init(Cfg(true), &bus));
	}
	memcpy(s_Gt.Id, "911", 4);
	for (int f = 1; f <= 2; f++)
	{
		Fresh();
		s_Gt.Present = true;
		bus.Transfers = 0;
		bus.Fail = f;
		assert(!t.Init(Cfg(true), &bus) && !t.Valid());
	}
	bus.Fail = 0;
	printf("GT911 init: PASS\n");
}

static void TestGtRead()
{
	Bus bus;
	TouchGt911 t;
	TouchPoint_t pt[11];

	Fresh();
	s_Gt.Present = true;
	assert(t.Init(Cfg(true), &bus));
	// Nothing new, nothing yet
	assert(t.Read(pt, 5) == 0 && s_Gt.StatusClears == 0);
	// 3 contacts: read, the status cleared
	GtPoints(3);
	GtPoint(0, 0, 0, 1);
	GtPoint(1, 479, 271, 2);
	GtPoint(2, 300, 200, 7);
	assert(t.Read(pt, 5) == 3 && s_Gt.StatusClears == 1 && s_Gt.PointReads == 1 && s_Gt.Status == 0);
	assert(pt[0].X == 0 && pt[0].Y == 0 && pt[0].Id == 1);
	assert(pt[1].X == 479 && pt[1].Y == 271 && pt[1].Id == 2);
	assert(pt[2].X == 300 && pt[2].Y == 200 && pt[2].Id == 7);
	// No new coordinates: the last ones, in the orientation set
	memset(pt, 0, sizeof(pt));
	assert(t.Read(pt, 2) == 2 && pt[1].X == 479 && s_Gt.StatusClears == 1);
	t.Orientation(DISPL_ORIENT_LANDSCAPE_INV);
	assert(t.Read(pt, 5) == 3 && pt[0].X == 479 && pt[0].Y == 271);
	t.Orientation(DISPL_ORIENT_LANDSCAPE);
	// Lifted: 0 contacts reported
	GtPoints(0);
	assert(t.Read(pt, 5) == 0 && s_Gt.StatusClears == 2 && s_Gt.PointReads == 1);
	// 10 contacts, more than 10
	GtPoints(10);
	for (int i = 0; i < 10; i++)
	{
		GtPoint(i, i * 40, i * 20, i);
	}
	assert(t.Read(pt, 11) == 10 && pt[9].X == 360 && pt[9].Y == 180 && pt[9].Id == 9);
	GtPoints(11);
	assert(t.Read(pt, 11) == 0 && s_Gt.StatusClears == 4);
	// Count bits only
	GtPoints(2);
	s_Gt.Status |= 0x30;
	assert(t.Read(pt, 11) == 2);
	assert(t.Read(pt, 0) == 0 && t.Read(pt, -1) == -1 && t.Read(nullptr, 1) == -1);
	// Out of the range: dropped
	GtPoints(2);
	GtPoint(0, 480, 0, 0);
	GtPoint(1, 0, 272, 1);
	assert(t.Read(pt, 5) == 0);
	// Each transfer failing: the last contacts kept
	GtPoints(1);
	GtPoint(0, 5, 6, 3);
	assert(t.Read(pt, 5) == 1 && pt[0].X == 5);
	for (int f = 1; f <= 3; f++)
	{
		GtPoints(1);
		GtPoint(0, 50, 60, 4);
		bus.Transfers = 0;
		bus.Fail = f;
		assert(t.Read(pt, 5) == -1);
		bus.Fail = 0;
		assert(t.Read(pt, 5) == 1 && pt[0].X == 50);
	}
	// The coordinate range of the controller scaled to the panel
	Fresh();
	s_Gt.Present = true;
	s_Gt.XMax = 800;
	s_Gt.YMax = 480;
	assert(t.Init(Cfg(true), &bus));
	GtPoints(3);
	GtPoint(0, 799, 479, 0);
	GtPoint(1, 400, 240, 1);
	GtPoint(2, 800, 0, 2);
	assert(t.Read(pt, 5) == 2 && pt[0].X == 479 && pt[0].Y == 271 && pt[1].X == 240 && pt[1].Y == 136);
	// No range in the configuration: pixels
	for (int k = 0; k < 2; k++)
	{
		Fresh();
		s_Gt.Present = true;
		(k ? s_Gt.YMax : s_Gt.XMax) = 0;
		assert(t.Init(Cfg(true), &bus));
		GtPoints(2);
		GtPoint(0, 479, 271, 0);
		GtPoint(1, 480, 0, 1);
		assert(t.Read(pt, 5) == 1 && pt[0].X == 479 && pt[0].Y == 271);
	}
	printf("GT911 read: PASS\n");
}

static void TestGtPower()
{
	Bus bus;
	TouchGt911 t;
	TouchPoint_t pt[5];

	Fresh();
	s_Gt.Present = true;
	TouchCfg_t c = Cfg(true);
	c.DevAddr = 0x14;
	assert(t.Init(c, &bus) && t.Enable() && s_Gt.Resets == 1);
	GtPoints(1);
	GtPoint(0, 10, 20, 1);
	assert(t.Read(pt, 5) == 1);
	// Sleep with the interrupt pin low, nothing read, then the reset
	// sequence of the address
	t.Disable();
	assert(s_Gt.Sleep && s_Gt.Sleeps == 1 && s_Int.Out && !s_Int.Level);
	int n = bus.Transfers;
	t.Disable();
	assert(s_Gt.Sleeps == 1 && bus.Transfers == n && t.Read(pt, 5) == 0);
	assert(t.Enable() && s_Gt.Resets == 2 && s_Gt.Addr == 0x14 && !s_Int.Out);
	// The contacts before the sleep are not returned
	assert(t.Read(pt, 5) == 0);
	assert(t.Enable() && s_Gt.Resets == 2);
	// Woken up without answer: still asleep
	t.Disable();
	s_Gt.Present = false;
	assert(!t.Enable() && t.Read(pt, 5) == 0);
	s_Gt.Present = true;
	assert(t.Enable() && t.Read(pt, 5) == 0);
	// Reset
	GtPoints(1);
	GtPoint(0, 10, 20, 1);
	assert(t.Read(pt, 5) == 1);
	t.Reset();
	assert(s_Gt.Resets == 5 && t.Read(pt, 5) == 0);
	s_Gt.Present = false;
	t.Reset();
	assert(t.Read(pt, 5) == 0);
	s_Gt.Present = true;
	assert(t.Enable());
	// No reset pin: Disable does nothing
	Fresh();
	s_Gt.Present = true;
	assert(t.Init(Cfg(false), &bus));
	t.Disable();
	assert(s_Gt.Sleeps == 0 && t.Enable());
	Fresh();
	s_Gt.Present = true;
	c = Cfg(true);
	c.RstPin.PortNo = -1;
	assert(t.Init(c, &bus));
	GtPoints(1);
	GtPoint(0, 10, 20, 1);
	t.Disable();
	assert(s_Gt.Sleeps == 0 && t.Read(pt, 5) == 1 && t.Enable());
	printf("GT911 power: PASS\n");
}

// The board finds the controller: GT911 first, then FT5406
static void TestDetect()
{
	for (int which = 0; which < 3; which++)
	{
		Bus bus;
		TouchGt911 gt;
		TouchFt5406 ft;
		Touch *found = nullptr;

		Fresh();
		s_Gt.Present = which == 0;
		s_Ft.Present = which == 1;
		if (gt.Init(Cfg(true), &bus))
		{
			found = &gt;
		}
		else if (ft.Init(Cfg(true, TOUCH_FLAG_XY_SWAP), &bus))
		{
			found = &ft;
		}
		assert(found == (which == 0 ? (Touch *)&gt : which == 1 ? (Touch *)&ft : nullptr));
	}
	printf("detection: PASS\n");
}

int main()
{
	setvbuf(stdout, NULL, _IONBF, 0);
	TestFtInit();
	TestFtRead();
	TestFtPower();
	TestMapping();
	TestGtInit();
	TestGtRead();
	TestGtPower();
	TestDetect();
	printf("touch: PASS\n");

	return 0;
}
