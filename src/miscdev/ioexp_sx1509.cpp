/**-------------------------------------------------------------------------
@file	ioexp_sx1509.cpp

@brief	SX1509B GPIO and LED driver.

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

/** SX1509B GPIO and LED driver. SPDX-License-Identifier: MIT */
#include "miscdev/ioexp_sx1509.h"
#include "idelay.h"

static const uint8_t s_LedReg[16] = {
	0x29, 0x2C, 0x2F, 0x32, 0x35, 0x3A, 0x3F, 0x44,
	0x49, 0x4C, 0x4F, 0x52, 0x55, 0x5A, 0x5F, 0x64
};

IoExpSx1509::IoExpSx1509() : vOutput(0), vKnown(0)
{
	vEvtHandler = nullptr;
	vIntId = 0;
	vByteOrder = DEV_BYTEORDER_BIG;
}

bool IoExpSx1509::ReadReg(uint8_t Reg, uint8_t &Value)
{
	uint8_t data;

	if (vpIntrf == nullptr || Device::Read(&Reg, 1, &data, 1) != 1)
		return false;
	Value = data;
	return true;
}

bool IoExpSx1509::WriteReg(uint8_t Reg, uint8_t Value)
{
	return vpIntrf != nullptr && Device::Write(&Reg, 1, &Value, 1) == 1;
}

bool IoExpSx1509::Modify(uint8_t Reg, uint8_t Mask, uint8_t Value)
{
	uint8_t old;

	return ReadReg(Reg, old) && WriteReg(Reg, (old & ~Mask) | (Value & Mask));
}

bool IoExpSx1509::PinBit(uint8_t BankBReg, uint8_t Pin, bool Set)
{
	uint8_t bit;

	if (!Valid() || Pin > 15)
		return false;
	bit = 1U << (Pin & 7);
	return Modify(BankBReg + (Pin < 8 ? 1 : 0), bit, Set ? bit : 0);
}

bool IoExpSx1509::ReadPair(uint8_t BankBReg, uint16_t &Value)
{
	uint8_t high;
	uint8_t low;

	if (!Valid() || !ReadReg(BankBReg, high) || !ReadReg(BankBReg + 1, low))
		return false;
	Value = ((uint16_t)high << 8) | low;
	return true;
}

bool IoExpSx1509::WritePair(uint8_t BankBReg, uint16_t Value)
{
	return Valid() && WriteReg(BankBReg, Value >> 8) && WriteReg(BankBReg + 1, Value);
}

bool IoExpSx1509::Init(DeviceIntrf *pIntrf, uint32_t Addr)
{
	uint8_t reset;

	Valid(false);
	vKnown = 0;
	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		(Addr != 0x3E && Addr != 0x3F && Addr != 0x70 && Addr != 0x71))
		return false;
	Interface(pIntrf);
	DeviceAddress(Addr);
	// A non-destructive communication probe, not an identity check.
	if (!ReadReg(0x7D, reset) || reset != 0 || !Modify(0x1F, 1, 1))
		return false;
	Valid(true);
	return true;
}

bool IoExpSx1509::SoftReset()
{
	Valid(false);
	vKnown = 0;
	if (!WriteReg(0x7D, 0x12) || !WriteReg(0x7D, 0x34))
		return false;
	msDelay(2);
	if (!Modify(0x1F, 1, 1))
		return false;
	vOutput = 0xFFFF;
	vKnown = 0xFFFF;
	Valid(true);
	return true;
}

void IoExpSx1509::Reset()
{
	SoftReset();
}

void IoExpSx1509::AdoptOutputLatch(uint16_t Latch)
{
	vOutput = Latch;
	vKnown = Valid() ? 0xFFFF : 0;
}

bool IoExpSx1509::WriteOutputs(uint16_t Values)
{
	if (!Valid())
		return false;
	vKnown = 0;
	if (!WriteReg(0x10, Values >> 8))
		return false;
	vOutput = (vOutput & 0x00FF) | (Values & 0xFF00);
	vKnown = 0xFF00;
	if (!WriteReg(0x11, Values))
		return false;
	vOutput = Values;
	vKnown = 0xFFFF;
	return true;
}

bool IoExpSx1509::WritePins(uint16_t Mask, uint16_t Values)
{
	uint16_t banks;
	uint16_t next;

	banks = (Mask & 0xFF00 ? 0xFF00 : 0) | (Mask & 0x00FF ? 0x00FF : 0);
	if (!Valid() || (vKnown & banks) != banks)
		return false;
	next = (vOutput & ~Mask) | (Values & Mask);
	if ((Mask & 0xFF00) != 0)
	{
		vKnown &= 0x00FF;
		if (!WriteReg(0x10, next >> 8))
			return false;
		vOutput = (vOutput & 0x00FF) | (next & 0xFF00);
		vKnown |= 0xFF00;
	}
	if ((Mask & 0x00FF) != 0)
	{
		vKnown &= 0xFF00;
		if (!WriteReg(0x11, next))
			return false;
		vOutput = (vOutput & 0xFF00) | (next & 0x00FF);
		vKnown |= 0x00FF;
	}
	return true;
}

bool IoExpSx1509::ReadPins(uint16_t &Values)
{
	return ReadPair(0x10, Values);
}

bool IoExpSx1509::ConfigurePin(uint8_t Pin, bool Output, bool InitialHigh,
	SX1509_PULL Pull, bool OpenDrain, bool Inverted)
{
	uint16_t mask;

	if (!Valid() || Pin > 15 || Pull < SX1509_PULL_NONE || Pull > SX1509_PULL_DOWN)
		return false;
	mask = 1U << Pin;
	// Preload the output latch before enabling drive. Other outputs are preserved.
	if (Output && !WritePins(mask, InitialHigh ? mask : 0))
		return false;
	if (!PinBit(0x20, Pin, false) || !PinBit(0x00, Pin, false) ||
		!PinBit(0x06, Pin, false) || !PinBit(0x08, Pin, false) ||
		!PinBit(0x0A, Pin, OpenDrain) || !PinBit(0x0C, Pin, Inverted))
		return false;
	if (Pull != SX1509_PULL_NONE && !PinBit(Pull == SX1509_PULL_UP ? 0x06 : 0x08, Pin, true))
		return false;
	return PinBit(0x0E, Pin, !Output);
}

bool IoExpSx1509::ConfigureDrive(uint8_t Pin, bool LongSlew, bool LowDrive)
{
	return PinBit(0x02, Pin, LongSlew) && PinBit(0x04, Pin, LowDrive);
}

bool IoExpSx1509::ConfigureHighInput(uint8_t Pin, bool Enable)
{
	return PinBit(0x69, Pin, Enable);
}

bool IoExpSx1509::ConfigureInterrupt(uint8_t Pin, SX1509_EDGE Edge)
{
	uint8_t reg;
	uint8_t shift;

	if (!Valid() || Pin > 15 || Edge < SX1509_EDGE_NONE || Edge > SX1509_EDGE_BOTH)
		return false;
	reg = 0x17 - Pin / 4;
	shift = (Pin & 3) * 2;
	return PinBit(0x12, Pin, true) && Modify(reg, 3U << shift, Edge << shift) &&
		PinBit(0x12, Pin, Edge == SX1509_EDGE_NONE);
}

bool IoExpSx1509::ReadInterrupts(uint16_t &Sources)
{
	return ReadPair(0x18, Sources);
}

bool IoExpSx1509::ClearInterrupts(uint16_t Sources)
{
	return WritePair(0x18, Sources);
}

bool IoExpSx1509::ReadEvents(uint16_t &Events)
{
	return ReadPair(0x1A, Events);
}

bool IoExpSx1509::ClearEvents(uint16_t Events)
{
	return WritePair(0x1A, Events);
}

bool IoExpSx1509::ConfigureClock(uint8_t Divider, bool LogBankA, bool LogBankB)
{
	uint8_t misc;

	if (!Valid() || Divider < 1 || Divider > 7)
		return false;
	misc = (Divider << 4) | (LogBankA ? 8 : 0) | (LogBankB ? 0x80 : 0);
	return Modify(0x1E, 0x60, 0x40) && Modify(0x1F, 0xF8, misc);
}

bool IoExpSx1509::ConfigureDebounce(uint16_t Pins, uint8_t TimeCode)
{
	uint8_t clock;

	if (!Valid() || TimeCode > 7 || !ReadReg(0x1E, clock) || (clock & 0x60) == 0)
		return false;
	return WriteReg(0x22, TimeCode) && WritePair(0x23, Pins);
}

bool IoExpSx1509::ConfigureLed(uint8_t Pin, const Sx1509LedCfg_t &Cfg)
{
	uint8_t reg;
	uint8_t clock;
	uint8_t misc;
	bool fading;

	if (!Valid() || Pin > 15 || Cfg.OnTime > 31 || Cfg.OffTime > 31 ||
		Cfg.OffIntensity > 7 || Cfg.RiseTime > 31 || Cfg.FallTime > 31)
		return false;
	fading = (Pin & 7) >= 4;
	if ((!fading && (Cfg.RiseTime != 0 || Cfg.FallTime != 0)) ||
		((Cfg.RiseTime != 0 || Cfg.FallTime != 0) && Cfg.OnIntensity < 4 * Cfg.OffIntensity))
		return false;
	if (!ReadReg(0x1E, clock) || (clock & 0x60) == 0 ||
		!ReadReg(0x1F, misc) || (misc & 0x70) == 0)
		return false;
	// Sink LED current, inactive high until LedEnable. Open drain accommodates VBAT LEDs.
	if (!ConfigurePin(Pin, true, true, SX1509_PULL_NONE, true, false) ||
		!PinBit(0x00, Pin, true))
		return false;
	reg = s_LedReg[Pin];
	if (!WriteReg(reg, Cfg.OnTime) || !WriteReg(reg + 1, Cfg.OnIntensity) ||
		!WriteReg(reg + 2, (Cfg.OffTime << 3) | Cfg.OffIntensity))
		return false;
	if (fading && (!WriteReg(reg + 3, Cfg.RiseTime) || !WriteReg(reg + 4, Cfg.FallTime)))
		return false;
	return PinBit(0x20, Pin, true);
}

bool IoExpSx1509::LedEnable(uint8_t Pin, bool Enable)
{
	uint8_t enabled;
	uint16_t mask;

	if (!Valid() || Pin > 15 || !ReadReg(Pin < 8 ? 0x21 : 0x20, enabled) ||
		(enabled & (1U << (Pin & 7))) == 0)
		return false;
	mask = 1U << Pin;
	return WritePins(mask, Enable ? 0 : mask);
}
