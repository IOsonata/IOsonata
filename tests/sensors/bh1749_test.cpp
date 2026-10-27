/**-------------------------------------------------------------------------
@file	bh1749_test.cpp

@brief	BH1749 Device lifecycle and transfer tests.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>
#include "sensors/color_bh1749.h"
#include "thingy52_mock.h"

uint64_t g_DelayUs;

class Bh1749Bus : public MockIntrf
{
public:
	uint32_t Address = 0;
	DEVINTRF_TYPE BusType = DEVINTRF_TYPE_I2C;
	unsigned Resets = 0;

	DEVINTRF_TYPE Type() override
	{
		return BusType;
	}
	void Prepare()
	{
		memset(Reg, 0, sizeof(Reg));
		Reg[0x40] = BH1749_PART_ID;
		Reg[0x92] = 0xE0;
	}
	int Read(uint32_t Addr, const uint8_t *pCmd, int CmdLen,
		uint8_t *pData, int Len) override
	{
		Address = Addr;
		assert(CmdLen == 1 && Len > 0 && *pCmd + Len <= 256);
		if (*pCmd == FailRead)
			return 0;
		int count = *pCmd == ShortRead ? Len - 1 : Len;
		memcpy(pData, Reg + *pCmd, count);
		return count;
	}
	int Write(uint32_t Addr, const uint8_t *pCmd, int CmdLen,
		const uint8_t *pData, int Len) override
	{
		Address = Addr;
		int count = MockIntrf::Write(Addr, pCmd, CmdLen, pData, Len);
		if (count == Len && *pCmd == 0x40 && *pData == 0x80)
		{
			Prepare();
			Resets++;
		}
		return count;
	}
};

static_assert(std::is_base_of<Device, ColorBh1749>::value,
	"BH1749 must use Device lifecycle and transfer state");

int main()
{
	Bh1749Bus bus;
	ColorBh1749 sensor;
	Device *device = &sensor;
	ColorBh1749Data_t data = { 99, 98, 97, 96, 95 };

	assert(!device->Valid() && !device->Enable());
	device->Disable();
	device->Reset();
	assert(bus.Ops.empty());
	assert(!sensor.Init(NULL));
	assert(!sensor.Init(&bus, 0x40));
	bus.BusType = DEVINTRF_TYPE_SPI;
	assert(!sensor.Init(&bus));
	bus.BusType = DEVINTRF_TYPE_I2C;
	assert(!sensor.Init(&bus));
	bus.Prepare();
	bus.Reg[0x92] = 0;
	assert(!sensor.Init(&bus));
	bus.Prepare();
	assert(sensor.Init(&bus, BH1749_I2C_ADDR1));
	assert(device->Valid() && device->DeviceID() == BH1749_PART_ID);
	assert(device->DeviceAddress() == BH1749_I2C_ADDR1);
	assert(bus.Address == BH1749_I2C_ADDR1 && bus.Reg[0x41] == 0x2A);
	assert(bus.Reg[0x42] == 0x10 && g_DelayUs >= 2000);
	assert(sensor.Configure(COLOR_BH1749_GAIN_32X, COLOR_BH1749_GAIN_1X,
		COLOR_BH1749_MEASURE_35MS));
	uint8_t config = bus.Reg[0x41];
	device->Disable();
	assert(device->Valid() && bus.Reg[0x42] == 0 && bus.Reg[0x41] == config);
	assert(device->Enable() && bus.Reg[0x42] == 0x10);
	// The generic address setter must control the actual transfer address.
	device->DeviceAddress(BH1749_I2C_ADDR0);
	assert(device->Enable() && bus.Address == BH1749_I2C_ADDR0);
	assert(!sensor.Read(data) && data.Red == 99);
	bus.Reg[0x42] |= 0x80;
	for (unsigned i = 0; i < 12; i++)
		bus.Reg[0x50 + i] = i + 1;
	bus.ShortRead = 0x50;
	assert(!sensor.Read(data) && data.Red == 99 && data.Green2 == 95);
	bus.ShortRead = -1;
	assert(sensor.Read(data));
	assert(data.Red == 0x0201 && data.Green == 0x0403 && data.Blue == 0x0605);
	assert(data.Ir == 0x0A09 && data.Green2 == 0x0C0B);
	assert(sensor.ConfigureInterrupt(COLOR_BH1749_INT_BLUE, 0x1234, 0x5678,
		COLOR_BH1749_INT_4_CONSECUTIVE));
	assert(bus.Reg[0x60] == 9 && bus.Reg[0x61] == 2);
	assert(bus.Reg[0x62] == 0x78 && bus.Reg[0x65] == 0x12);
	device->Reset();
	assert(bus.Resets == 1 && !device->Valid() && !device->Enable());
	assert(bus.Reg[0x41] == 0 && bus.Reg[0x42] == 0);
	assert(!sensor.Read(data));
	assert(sensor.Init(&bus));
	assert(device->Valid() && bus.Reg[0x41] == 0x2A && bus.Reg[0x42] == 0x10);
	bus.ShortWrite = 0x42;
	device->Disable();
	assert(!device->Valid());
	bus.ShortWrite = -1;
	assert(sensor.Init(&bus));
	bus.FailWrite = 0x40;
	device->Reset();
	assert(!device->Valid() && bus.Resets == 1);
	bus.FailWrite = -1;
	for (int reg : { 0x40, 0x92 })
	{
		bus.ShortRead = reg;
		assert(!sensor.Init(&bus) && !device->Valid());
	}
	bus.ShortRead = -1;
	for (int reg : { 0x41, 0x42 })
	{
		bus.ShortWrite = reg;
		assert(!sensor.Init(&bus) && !device->Valid());
	}
	bus.ShortWrite = -1;
	assert(sensor.Init(&bus) && device->Valid());
	assert(!sensor.Init(NULL) && !device->Valid());
	device->Reset();
	assert(!device->Enable());
	puts("BH1749 Device lifecycle and transfers: PASS");
}
