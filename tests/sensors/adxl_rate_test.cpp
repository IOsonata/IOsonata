/**-------------------------------------------------------------------------
@file	adxl_rate_test.cpp

@brief	ADXL367/ADXL372 rate selection and register failure tests.

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

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "sensors/accel_adxl367.h"
#include "sensors/accel_adxl372.h"
#include "thingy52_mock.h"

static AccelAdxl367 s_Adxl367;
static AccelAdxl372 s_Adxl372;

static void Check367(MockIntrf &Bus, uint32_t Request, uint32_t Expected, uint8_t Code)
{
	assert(s_Adxl367.SetSamplingFrequency(Request));
	assert(s_Adxl367.SamplingFrequency() == Expected);
	assert(s_Adxl367.SamplingPeriod() == 1000000000000ULL / Expected);
	assert(Bus.Reg[0x2C] == (0x20 | Code));
}

static void Check372(MockIntrf &Bus, uint32_t Request, uint32_t Expected, uint8_t Code)
{
	assert(s_Adxl372.SetOutputRate(Request));
	assert(s_Adxl372.SamplingFrequency() == Expected);
	assert(s_Adxl372.SamplingPeriod() == 1000000000000ULL / Expected);
	assert(Bus.Reg[0x3D] == (0x1B | (Code << 5)));
}

int main()
{
	MockIntrf bus367;
	MockIntrf bus372;
	AccelSensorCfg_t cfg = {};
	const uint32_t rates367[] = { 12500, 25000, 50000, 100000, 200000, 400000 };
	const uint32_t rates372[] = { 400000, 800000, 1600000, 3200000, 6400000 };

	assert(!s_Adxl367.SetSamplingFrequency(13000));
	assert(!s_Adxl372.SetOutputRate(410000));
	bus367.Reg[0] = bus372.Reg[0] = 0xAD;
	bus367.Reg[1] = bus372.Reg[1] = 0x1D;
	bus367.Reg[2] = ADXL367_PART_ID;
	bus372.Reg[2] = ADXL372_PART_ID;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Scale = 2;
	cfg.Freq = 13000;
	assert(s_Adxl367.Init(cfg, &bus367));
	assert(s_Adxl367.SamplingFrequency() == 12500);
	cfg.Scale = 200;
	cfg.Freq = 410000;
	assert(s_Adxl372.Init(cfg, &bus372));
	assert(s_Adxl372.SamplingFrequency() == 400000);
	bus372.Reg[0x3D] |= 0x1B;

	Check367(bus367, 0, 12500, 0);
	Check367(bus367, UINT32_MAX, 400000, 5);
	Check372(bus372, 0, 400000, 0);
	Check372(bus372, UINT32_MAX, 6400000, 4);
	for (unsigned i = 0; i < 6; i++)
	{
		Check367(bus367, rates367[i], rates367[i], i);
		if (i == 5)
			break;
		uint32_t midpoint = (rates367[i] + rates367[i + 1]) / 2;
		Check367(bus367, midpoint - 1, rates367[i], i);
		Check367(bus367, midpoint, rates367[i], i);
		Check367(bus367, midpoint + 1, rates367[i + 1], i + 1);
	}
	for (unsigned i = 0; i < 5; i++)
	{
		Check372(bus372, rates372[i], rates372[i], i);
		if (i == 4)
			break;
		uint32_t midpoint = (rates372[i] + rates372[i + 1]) / 2;
		Check372(bus372, midpoint - 1, rates372[i], i);
		Check372(bus372, midpoint, rates372[i], i);
		Check372(bus372, midpoint + 1, rates372[i + 1], i + 1);
	}

	bus367.FailWrite = 0x2C;
	assert(!s_Adxl367.SetSamplingFrequency(12500));
	assert(s_Adxl367.SamplingFrequency() == 400000);
	assert(bus367.Reg[0x2C] == 0x25);
	bus367.FailWrite = -1;
	Check367(bus367, 13000, 12500, 0);
	bus372.FailRead = 0x3D;
	assert(!s_Adxl372.SetOutputRate(400000));
	assert(s_Adxl372.SamplingFrequency() == 6400000);
	bus372.FailRead = -1;
	bus372.FailWrite = 0x3D;
	assert(!s_Adxl372.SetOutputRate(400000));
	assert(s_Adxl372.SamplingFrequency() == 6400000);
	assert(bus372.Reg[0x3D] == 0x9B);
	bus372.FailWrite = -1;
	Check372(bus372, 410000, 400000, 0);
	// Decode signed samples and verify the public converted-data path.
	bus367.Reg[0x0B] = 1;
	bus367.Reg[0x0E] = 0x3E; // +4000 counts = +1 g at +/-2 g.
	bus367.Reg[0x0F] = 0x80;
	bus367.Reg[0x10] = 0xC1; // -4000 counts = -1 g.
	bus367.Reg[0x11] = 0x80;
	bus372.Reg[0x04] = 1;
	bus372.Reg[0x08] = 0;
	bus372.Reg[0x09] = 0xA0; // +10 counts = +1 g.
	bus372.Reg[0x0A] = 0xFF;
	bus372.Reg[0x0B] = 0x60; // -10 counts = -1 g.
	AccelSensorData_t data;
	AccelSensorRawData_t raw;
	for (unsigned pass = 0; pass < 2; pass++)
	{
		assert(s_Adxl367.UpdateData());
		assert(s_Adxl367.Read(data));
		assert(fabsf(data.X - 1.0f) < 0.00001f);
		assert(fabsf(data.Y + 1.0f) < 0.00001f);
		assert(s_Adxl367.Read(raw));
		assert(fabsf(raw.GFactor - 0.00025f) < 0.000001f);
		assert(s_Adxl372.UpdateData());
		assert(s_Adxl372.Read(data));
		assert(fabsf(data.X - 1.0f) < 0.00001f);
		assert(fabsf(data.Y + 1.0f) < 0.00001f);
		assert(s_Adxl372.Read(raw));
		assert(fabsf(raw.GFactor - 0.1f) < 0.000001f);
		cfg.Scale = 2;
		assert(s_Adxl367.Init(cfg, &bus367));
		cfg.Scale = 200;
		assert(s_Adxl372.Init(cfg, &bus372));
	}
	assert(s_Adxl367.Scale(4) == 4);
	assert(s_Adxl367.UpdateData() && s_Adxl367.Read(data));
	assert(fabsf(data.X - 2.0f) < 0.00001f);
	assert(s_Adxl367.Read(raw));
	assert(fabsf(raw.GFactor - 0.0005f) < 0.000001f);
	puts("ADXL367/ADXL372 rate tests: PASS");
}
