// Thingy:52 motion register-transfer regressions. SPDX-License-Identifier: MIT
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensors/accel_lisxdh.h"
#include "sensors/agm_mpu9250.h"
#include "thingy52_mock.h"

uint64_t g_DelayUs;

class MotionBus : public MockIntrf {
public:
	bool Lis = false;
	bool Spi = false;
	uint8_t Mag[256] = {};
	unsigned Samples = 0;
	int FailMag = -1;

	DEVINTRF_TYPE Type() override { return Spi ? DEVINTRF_TYPE_SPI : DEVINTRF_TYPE_I2C; }
	int Read(uint32_t Addr, const uint8_t *Cmd, int CmdLen, uint8_t *Data, int Len) override
	{
		uint8_t reg = *Cmd;
		uint8_t *source;

		assert(CmdLen == 1);
		if (Spi)
		{
			assert(reg & 0x80);
			reg &= 0x7F;
		}
		if (Lis)
		{
			assert(Addr == (Spi ? 0U : 0x19U));
			if (Len > 1)
				assert(reg & (Spi ? 0x40 : 0x80));
			reg &= Spi ? 0x3F : 0x7F;
			if (reg == 0x28)
				++Samples;
		}
		if (!Lis && Addr == 0x0C)
		{
			assert(!Spi && (Reg[0x37] & 2) && !(Reg[0x6A] & 0x20));
			if (reg == FailMag)
				return 0;
			source = Mag;
		}
		else
		{
			if (!Lis)
				assert(Addr == (Spi ? 0U : 0x68U));
			source = Reg;
		}
		if (reg == FailRead)
			return 0;
		if (reg == ShortRead)
			--Len;
		memcpy(Data, source + reg, Len);
		return Len;
	}
	int Write(uint32_t Addr, const uint8_t *Cmd, int CmdLen, const uint8_t *Data, int Len) override
	{
		uint8_t reg = *Cmd;
		uint8_t *target = Reg;
		unsigned n;

		assert(!(Spi && (reg & 0x80)));
		if (Addr == 0x0C && !Lis)
		{
			assert(!Spi && (Reg[0x37] & 2) && !(Reg[0x6A] & 0x20));
			target = Mag;
		}
		if (reg == FailWrite)
			return 0;
		// The SPI auxiliary-master setup sends register/data in the command phase.
		if (CmdLen > 1)
			memcpy(target + reg, Cmd + 1, CmdLen - 1);
		if (Len)
			memcpy(target + reg + CmdLen - 1, Data, Len);
		if (!Lis && reg == 0x6B && (target[reg] & 0x80))
		{
			memset(Reg, 0, sizeof(Reg));
			Reg[0x75] = 0x71;
			Reg[0x6B] = 0x40;
		}
		if (!Lis && Spi && reg == 0x25 && (Reg[0x27] & 0x80))
		{
			assert((Reg[0x6A] & 0x30) == 0x30);
			n = Reg[0x27] & 15;
			if (Reg[0x25] & 0x80)
				memcpy(Reg + 0x49, Mag + Reg[0x26], n);
			else
				Mag[Reg[0x26]] = Reg[0x63];
		}
		return Len;
	}
	void PrepareMpu()
	{
		Reg[0x75] = 0x71;
		Mag[0] = 0x48;
		Mag[0x10] = 128;
		Mag[0x11] = 192;
		Mag[0x12] = 64;
	}
};

static void TestLis(AccelLisxdh &Sensor, bool Spi)
{
	MotionBus bus;
	AccelSensorCfg_t cfg = {};
	AccelSensorRawData_t sample;
	TempSensorData_t temperature;
	const uint8_t data[] = {0x00, 0x80, 0xF0, 0xFF, 0xF0, 0x7F};

	bus.Lis = true;
	bus.Spi = Spi;
	bus.Reg[0x0F] = 0x33;
	bus.Reg[0x2F] = 0x20; // Empty FIFO is normal in polling mode.
	cfg.DevAddr = Spi ? 0 : 0x19;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = 10000;
	cfg.Scale = 2;
	assert(Sensor.Init(cfg, &bus));
	assert(bus.Reg[0x23] & 0x80); // BDU protects high/low byte coherence.
	assert(!Sensor.UpdateData());
	bus.Reg[0x27] = 8;
	memcpy(bus.Reg + 0x28, data, sizeof(data));
	assert(Sensor.UpdateData() && bus.Samples == 1);
	Sensor.Read(sample);
	assert(sample.X == -2048 && sample.Y == -1 && sample.Z == 2047);
	bus.ShortRead = 0x28;
	assert(!Sensor.UpdateData());
	Sensor.Read(sample);
	assert(sample.X == -2048 && sample.Y == -1 && sample.Z == 2047);
	bus.ShortRead = -1;
	bus.FailRead = 0x27;
	assert(!Sensor.UpdateData());
	bus.FailRead = -1;
	bus.Reg[0x27] = 0;
	bus.Reg[0x07] = 4;
	bus.Reg[0x0C] = 0;
	bus.Reg[0x0D] = 0xF6; // -10 C relative to 25 C.
	assert(Sensor.UpdateData());
	Sensor.Read(temperature);
	assert(temperature.Temperature == 1500);
	bus.Reg[0x07] = 0;
	cfg.Inter = 1;
	assert(Sensor.Init(cfg, &bus));
	bus.Samples = 0;
	bus.Reg[0x2F] = 2; // Even FIFO count must not be treated as not ready.
	assert(Sensor.UpdateData() && bus.Samples == 2);
}

static void TestMpu(AgmMpu9250 &Sensor, bool Spi)
{
	MotionBus bus;
	AccelSensorCfg_t accelCfg = {};
	GyroSensorCfg_t gyroCfg = {};
	MagSensorCfg_t magCfg = {};
	AccelSensorRawData_t accel;
	GyroSensorRawData_t gyro;
	MagSensorRawData_t mag;
	TempSensorData_t temperature;
	const uint8_t data[] = {0x80,0, 0xFF,0xFF, 0x7F,0xFF, 0,0, 0xFF,0x9C, 0,100, 0x80,0};
	const uint8_t magnetic[] = {1, 0x00,0x80, 0x00,0x01, 0x00,0xFF, 0x10};

	bus.Spi = Spi;
	bus.PrepareMpu();
	accelCfg.DevAddr = Spi ? 0 : 0x68;
	accelCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	accelCfg.Freq = 100000;
	accelCfg.Scale = 2;
	gyroCfg.DevAddr = accelCfg.DevAddr;
	gyroCfg.Freq = 32000000;
	gyroCfg.Sensitivity = 250;
	magCfg.DevAddr = accelCfg.DevAddr;
	magCfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	magCfg.Freq = 8000;
	magCfg.Precision = MAGSENSOR_PRECISION_HIGH;
	assert(Sensor.Init(accelCfg, &bus));
	assert(Sensor.Init(gyroCfg, &bus));
	assert(Sensor.Init(magCfg, &bus));
	assert(Sensor.Enable());
	if (!Spi)
		assert((bus.Reg[0x37] & 2) && !(bus.Reg[0x6A] & 0x20));
	memcpy(bus.Reg + 0x3B, data, sizeof(data));
	memcpy(bus.Mag + 2, magnetic, sizeof(magnetic));
	assert(Sensor.UpdateData());
	Sensor.Read(accel);
	Sensor.Read(gyro);
	Sensor.Read(mag);
	Sensor.Read(temperature);
	assert(accel.X == -32768 && accel.Y == -1 && accel.Z == 32767);
	assert(gyro.X == -100 && gyro.Y == 100 && gyro.Z == -32768);
	assert(temperature.Temperature == 2100);
	assert(mag.X == -32768 && mag.Y == 320 && mag.Z == -192);
	assert(mag.Sensitivity[0] == 149);
	bus.Mag[9] |= 8; // Overflow leaves the last valid magnetic sample intact.
	bus.Mag[3] = 10;
	assert(Sensor.UpdateData());
	Sensor.Read(mag);
	assert(mag.X == -32768);
	bus.Mag[9] &= ~8;
	bus.Mag[2] = 0;
	assert(Sensor.UpdateData());
	Sensor.Read(mag);
	assert(mag.X == -32768);
	bus.ShortRead = 0x3B;
	bus.Reg[0x3B] = 0;
	assert(!Sensor.UpdateData());
	Sensor.Read(accel);
	assert(accel.X == -32768);
	bus.ShortRead = -1;
	bus.FailRead = 0x3B;
	assert(!Sensor.UpdateData());
}

int main()
{
	static AccelLisxdh lisI2c;
	static AccelLisxdh lisSpi;
	static AgmMpu9250 mpuI2c;
	static AgmMpu9250 mpuSpi;

	TestLis(lisI2c, false);
	TestLis(lisSpi, true);
	TestMpu(mpuI2c, false);
	TestMpu(mpuSpi, true);
	puts("Thingy52 motion tests: PASS");
	return 0;
}
