/**-------------------------------------------------------------------------
@file	ioexp_sx1509.h

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
#ifndef __IOEXP_SX1509_H__
#define __IOEXP_SX1509_H__
#include "device.h"

#define SX1509_I2C_ADDR0 0x3E
#define SX1509_I2C_ADDR1 0x3F
#define SX1509_I2C_ADDR2 0x70
#define SX1509_I2C_ADDR3 0x71

enum SX1509_PULL { SX1509_PULL_NONE, SX1509_PULL_UP, SX1509_PULL_DOWN };
enum SX1509_EDGE { SX1509_EDGE_NONE, SX1509_EDGE_RISING, SX1509_EDGE_FALLING, SX1509_EDGE_BOTH };

struct Sx1509LedCfg_t {
	uint8_t OnIntensity; // 0..255
	uint8_t OffIntensity; // 0..7 (linear intensity = 4 * code)
	uint8_t OnTime; // 0..31; 0 = static. Other codes are datasheet clock units.
	uint8_t OffTime; // 0..31; 0 = single shot
	uint8_t RiseTime; // 0..31; supported only on pins 4..7 and 12..15
	uint8_t FallTime;
};

class IoExpSx1509 : public Device {
public:
	IoExpSx1509();
	// Attach without resetting GPIOs or rails. Disables interrupt autoclear on
	// pin reads. No device-ID register exists: caller must identify the hardware.
	bool Init(DeviceIntrf *pIntrf, uint32_t Addr = SX1509_I2C_ADDR0);
	bool Enable() override { return Valid(); }
	void Disable() override {} // Intentionally preserves GPIO/power states.
	void Reset() override;
	bool SoftReset(); // Destructive: all pins become inputs; output latch = 0xFFFF.
	// Before masked writes, either reset, write all outputs, or supply the exact
	// existing latch from the board owner. Never infer a latch from pin levels.
	void AdoptOutputLatch(uint16_t Latch);
	bool WriteOutputs(uint16_t Values);
	bool WritePins(uint16_t Mask, uint16_t Values);
	bool ReadPins(uint16_t &Values);
	bool ConfigurePin(uint8_t Pin, bool Output, bool InitialHigh = false,
		SX1509_PULL Pull = SX1509_PULL_NONE, bool OpenDrain = false, bool Inverted = false);
	bool ConfigureDrive(uint8_t Pin, bool LongSlew, bool LowDrive);
	bool ConfigureHighInput(uint8_t Pin, bool Enable); // Requires VCCx >=1.65V.
	bool ConfigureInterrupt(uint8_t Pin, SX1509_EDGE Edge);
	bool ReadInterrupts(uint16_t &Sources);
	bool ClearInterrupts(uint16_t Sources); // W1C only specified sources.
	bool ReadEvents(uint16_t &Events);
	bool ClearEvents(uint16_t Events);
	// Starts internal 2MHz oscillator. LED clock = 2MHz/2^(Divider-1), 1..7.
	// Clock and logarithmic mode are shared across channels/banks.
	bool ConfigureClock(uint8_t Divider, bool LogBankA = false, bool LogBankB = false);
	bool ConfigureDebounce(uint16_t Pins, uint8_t TimeCode); // 0..7 = 0.5..64ms at 2MHz
	bool ConfigureLed(uint8_t Pin, const Sx1509LedCfg_t &Cfg);
	bool LedEnable(uint8_t Pin, bool Enable);
private:
	bool ReadReg(uint8_t Reg, uint8_t &Value);
	bool WriteReg(uint8_t Reg, uint8_t Value);
	bool Modify(uint8_t Reg, uint8_t Mask, uint8_t Value);
	bool PinBit(uint8_t BankBReg, uint8_t Pin, bool Set);
	bool ReadPair(uint8_t BankBReg, uint16_t &Value);
	bool WritePair(uint8_t BankBReg, uint16_t Value);
	uint16_t vOutput;
	uint16_t vKnown;
};
#endif
