#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "sensors/color_bh1745.h"
#include "sensors/gas_ccs811.h"
#include "miscdev/ioexp_sx1509.h"

uint64_t g_DelayUs;

struct Operation {
	bool Write;
	uint8_t Reg;
	int Len;
	uint8_t Data[8];
};

class MockIntrf : public DeviceIntrf {
public:
	MockIntrf() : FailRead(-1), FailWrite(-1), ShortRead(-1), ShortWrite(-1),
		CommandFails(false), Ccs(false), Sx(false), Wake(false), RequireWake(false)
	{
		memset(Reg, 0, sizeof(Reg));
		memset(Algorithm, 0, sizeof(Algorithm));
		memset(Raw, 0, sizeof(Raw));
	}
	operator DevIntrf_t * () override { return nullptr; }
	DEVINTRF_TYPE Type() override { return DEVINTRF_TYPE_I2C; }
	uint32_t Rate(uint32_t Rate) override { return Rate; }
	uint32_t Rate() override { return 100000; }
	void Disable() override {}
	void Enable() override {}
	int Rx(uint32_t, uint8_t *, int) override { return 0; }
	int Tx(uint32_t, const uint8_t *pData, int Len) override
	{
		assert(!RequireWake || Wake);
		assert(Ccs && Len == 1 && pData[0] == 0xF4);
		if (CommandFails)
			return 0;
		Reg[0] |= 0x80;
		return 1;
	}
	bool StartRx(uint32_t) override { return false; }
	int RxData(uint8_t *, int) override { return 0; }
	void StopRx() override {}
	bool StartTx(uint32_t) override { return false; }
	int TxData(const uint8_t *, int) override { return 0; }
	void StopTx() override {}
	bool RequestToSend(int) override { return true; }
	void Reset() override {}
	int Read(uint32_t, const uint8_t *pCmd, int CmdLen, uint8_t *pData, int Len) override
	{
		int count;
		const uint8_t *source;

		assert(CmdLen == 1 && Len > 0 && Len <= 8);
		assert(!RequireWake || Wake);
		if (*pCmd == FailRead)
			return 0;
		count = *pCmd == ShortRead ? Len - 1 : Len;
		source = Ccs && *pCmd == 2 ? Algorithm : Ccs && *pCmd == 3 ? Raw : Reg + *pCmd;
		memcpy(pData, source, count);
		Ops.push_back({ false, *pCmd, Len, {} });
		if (Ccs && *pCmd == 2)
			Reg[0] &= ~8;
		if (Ccs && *pCmd == 0xE0)
			Reg[0] &= ~1;
		return count;
	}
	int Write(uint32_t, const uint8_t *pCmd, int CmdLen, const uint8_t *pData, int Len) override
	{
		Operation op = { true, *pCmd, Len, {} };
		int i;
		int count;

		assert(CmdLen == 1 && Len > 0 && Len <= 8);
		assert(!RequireWake || Wake);
		memcpy(op.Data, pData, Len);
		Ops.push_back(op);
		if (*pCmd == FailWrite)
			return 0;
		count = *pCmd == ShortWrite ? Len - 1 : Len;
		for (i = 0; i < count; ++i)
		{
			if (Sx && *pCmd >= 0x18 && *pCmd <= 0x1B)
				Reg[(uint8_t)(*pCmd + i)] &= ~pData[i];
			else
				Reg[(uint8_t)(*pCmd + i)] = pData[i];
		}
		if (Ccs && *pCmd == 0xFF && count == 4)
		{
			Reg[0] = 0x10;
			Reg[1] = 0;
		}
		if (Sx && *pCmd == 0x7D && pData[0] == 0x34)
		{
			memset(Reg, 0, sizeof(Reg));
			Reg[0x0E] = 0xFF;
			Reg[0x0F] = 0xFF;
			Reg[0x10] = 0xFF;
			Reg[0x11] = 0xFF;
		}
		return count;
	}
	void PrepareCcs()
	{
		Ccs = true;
		Reg[0] = 0x10;
		Reg[0x20] = 0x81;
		Reg[0x21] = 0x12;
	}
	void Sample()
	{
		Reg[0] = 0x98;
		Algorithm[0] = 0x04;
		Algorithm[1] = 0xD2; // 1234 ppm
		Algorithm[2] = 0;
		Algorithm[3] = 56;
		Algorithm[4] = 0x98;
		Algorithm[6] = 0x29; // 10uA, ADC=0x123
		Algorithm[7] = 0x23;
		Raw[0] = 0x33; // 12uA, ADC=0x345; deliberately different from ALG_RESULT
		Raw[1] = 0x45;
	}
	uint8_t Reg[256];
	uint8_t Algorithm[8];
	uint8_t Raw[2];
	int FailRead;
	int FailWrite;
	int ShortRead;
	int ShortWrite;
	bool CommandFails;
	bool Ccs;
	bool Sx;
	bool Wake;
	bool RequireWake;
	std::vector<Operation> Ops;
};

static bool WakeControl(bool Active, void *pContext)
{
	MockIntrf *bus = static_cast<MockIntrf *>(pContext);

	bus->Wake = Active;
	return true;
}

static GasSensorCfg_t GasConfig(uint32_t Freq = 1000)
{
	GasSensorCfg_t cfg = {};

	cfg.DevAddr = CCS811_I2C_ADDR0;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = Freq;
	return cfg;
}

static void TestColor()
{
	MockIntrf bus;
	ColorBh1745 sensor;
	Bh1745Data_t data = { 99, 98, 97, 96 };
	int i;

	assert(!sensor.Read(data));
	assert(!sensor.Init(&bus));
	bus.Reg[0x40] = 0x0B;
	bus.Reg[0x92] = 0xE0;
	assert(sensor.Init(&bus));
	assert(bus.Reg[0x44] == 2);
	assert(!sensor.Configure(6, 1));
	assert(!sensor.Configure(0, 4));
	assert(sensor.Configure(5, 16));
	assert(bus.Reg[0x41] == 5 && bus.Reg[0x42] == 2);
	assert(sensor.Enable());
	assert(bus.Reg[0x42] == 0x12);
	assert(!sensor.Read(data) && data.Red == 99);
	bus.Reg[0x42] |= 0x80;
	for (i = 0; i < 8; ++i)
		bus.Reg[0x50 + i] = i + 1;
	bus.ShortRead = 0x50;
	assert(!sensor.Read(data) && data.Red == 99);
	bus.ShortRead = -1;
	assert(sensor.Read(data));
	assert(data.Red == 0x0201 && data.Green == 0x0403 && data.Blue == 0x0605 && data.Clear == 0x0807);
	assert(sensor.ConfigureInterrupt(true, 3, 2, 0x1234, 0x5678));
	assert(bus.Reg[0x60] == 0x0D && bus.Reg[0x61] == 2);
	assert(bus.Reg[0x62] == 0x78 && bus.Reg[0x63] == 0x56);
	assert(bus.Reg[0x64] == 0x34 && bus.Reg[0x65] == 0x12);
	assert(!sensor.ConfigureInterrupt(true, 4, 0, 0, 1));
	assert(!sensor.ConfigureInterrupt(true, 0, 0, 2, 1));
	assert(sensor.ClearInterrupt() && bus.Reg[0x40] == 0x40);
	sensor.Disable();
	assert(bus.Reg[0x42] == 2);
	bus.FailWrite = 0x41;
	assert(!sensor.Configure(0, 1) && !sensor.Valid());
}

static void TestGas()
{
	MockIntrf bus;
	GasCcs811 sensor;
	Ccs811Data_t data = {};
	GasSensorData_t generic;
	uint16_t baseline = 0;
	uint8_t error = 0;

	bus.PrepareCcs();
	bus.RequireWake = true;
	sensor.WakeControl(WakeControl, &bus);
	assert(sensor.Init(GasConfig(), &bus));
	assert(!bus.Wake && bus.Reg[1] == 0x10 && bus.Reg[0] == 0x90);
	assert(!sensor.Mode(SENSOR_OPMODE_SINGLE, 1000));
	assert(!sensor.SetDriveMode(3));
	assert(sensor.SetDriveMode(0));
	assert(!sensor.SetDriveMode(3, 599999));
	assert(sensor.SetDriveMode(3, 600000));
	assert(sensor.SamplingPeriod() == 60000000000ULL);
	assert(sensor.SetDriveMode(1));
	assert(sensor.SetEnvironment(2350, 4850));
	assert(bus.Reg[5] == 0x61 && bus.Reg[6] == 0 && bus.Reg[7] == 0x61 && bus.Reg[8] == 0);
	assert(sensor.SetEnvironment(-4000, 10000));
	assert(bus.Reg[7] == 0 && bus.Reg[8] == 0);
	assert(!sensor.SetEnvironment(8501, 100));
	assert(!sensor.SetEnvironment(0, 10001));
	assert(sensor.ConfigureInterrupt(true, true) && bus.Reg[1] == 0x1C);
	assert(sensor.SetThresholds(1500, 2500));
	assert(bus.Ops.back().Len == 4 && bus.Reg[0x10] == 5 && bus.Reg[0x11] == 0xDC);
	assert(sensor.WriteBaseline(0xABCD));
	assert(sensor.ReadBaseline(baseline) && baseline == 0xABCD);
	bus.Sample();
	assert(sensor.Read(data));
	assert(data.Eco2 == 1234 && data.Etvoc == 56 && data.AlgorithmValid);
	assert(data.RawAdc == 0x123 && data.Current == 10 && data.Timestamp == 0);
	assert(!sensor.Read(data) && data.Eco2 == 1234);
	bus.Sample();
	bus.ShortRead = 2;
	data.Eco2 = 999;
	assert(!sensor.Read(data) && data.Eco2 == 999 && !bus.Wake);
	bus.ShortRead = -1;
	bus.Sample();
	assert(sensor.Read(generic));
	assert(generic.MeasIdx == 0 && isnan(generic.AirQualIdx));
	assert(generic.GasRes[0] == (uint64_t)0x123 * 1650000 / (1023 * 10));
	bus.Sample();
	bus.Algorithm[6] = 0;
	assert(sensor.Read(generic) && generic.MeasIdx == -1);
	assert(sensor.SetDriveMode(4));
	bus.Sample();
	assert(sensor.Read(data) && !data.AlgorithmValid && data.Eco2 == 0);
	assert(data.RawAdc == 0x345 && data.Current == 12);
	bus.Reg[0] |= 1;
	bus.Reg[0xE0] = 0x20;
	assert(!sensor.Read(data));
	assert(sensor.ReadError(error) && error == 0x20 && (bus.Reg[0] & 1) == 0);
	bus.FailRead = 0;
	assert(!sensor.Read(data) && !bus.Wake);
	bus.FailRead = -1;
	assert(sensor.SoftReset() && !sensor.Valid());
	assert(bus.Ops.back().Reg == 0xFF && bus.Ops.back().Data[3] == 0x8A);
}

static void TestStartupFailures()
{
	MockIntrf bus;
	GasCcs811 sensor;

	bus.PrepareCcs();
	bus.Reg[0] = 0;
	assert(!sensor.Init(GasConfig(), &bus) && !sensor.Valid());
	bus.Reg[0] = 0x10;
	bus.CommandFails = true;
	assert(!sensor.Init(GasConfig(), &bus) && !sensor.Valid());
	bus.CommandFails = false;
	bus.Reg[0x21] = 0x21;
	assert(!sensor.Init(GasConfig(), &bus));
	bus.Reg[0x21] = 0x12;
	bus.ShortWrite = 1;
	assert(!sensor.Init(GasConfig(), &bus) && !sensor.Valid());
}

static void TestNearestRates()
{
	static const uint32_t requested[] = { 0, 59, 550, 551, 2500, 2501, UINT32_MAX };
	static const uint32_t expected[] = { 17, 100, 100, 1000, 1000, 4000, 4000 };
	unsigned i;

	for (i = 0; i < sizeof(requested) / sizeof(requested[0]); ++i)
	{
		MockIntrf bus;
		GasCcs811 sensor;

		bus.PrepareCcs();
		assert(sensor.Init(GasConfig(requested[i]), &bus));
		assert(sensor.SamplingFrequency() == expected[i]);
	}
}

static void TestExpander()
{
	MockIntrf bus;
	IoExpSx1509 exp;
	Sx1509LedCfg_t led = { 128, 2, 3, 4, 5, 6 };
	uint16_t pins = 0xAAAA;
	int pin;

	bus.Sx = true;
	bus.Reg[0x10] = 0xA5;
	bus.Reg[0x11] = 0x5A;
	assert(exp.Init(&bus));
	assert(bus.Reg[0x10] == 0xA5 && bus.Reg[0x11] == 0x5A);
	assert(!exp.WritePins(1, 0)); // Unknown output latch must never be read from pins.
	exp.AdoptOutputLatch(0xFFFF);
	bus.Reg[0x10] = 0; // All physical inputs externally low; latch still FFFF.
	bus.Reg[0x11] = 0;
	assert(exp.WritePins(1, 0));
	assert(bus.Reg[0x11] == 0xFE && bus.Reg[0x10] == 0);
	assert(exp.WritePins(0x100, 0));
	assert(bus.Reg[0x10] == 0xFE);
	assert(exp.ConfigurePin(15, true, false, SX1509_PULL_NONE, true));
	assert((bus.Reg[0x0A] & 0x80) != 0 && (bus.Reg[0x0E] & 0x80) == 0);
	assert(!exp.ConfigurePin(16, true));
	bus.Reg[0x18] = 0xF0;
	bus.Reg[0x19] = 0x0F;
	assert(exp.ReadInterrupts(pins) && pins == 0xF00F);
	assert(exp.ClearInterrupts(0x8002));
	assert(bus.Reg[0x18] == 0x70 && bus.Reg[0x19] == 0x0D);
	assert(exp.ConfigureInterrupt(15, SX1509_EDGE_BOTH));
	assert(bus.Reg[0x14] == 0xC0);
	assert(exp.ConfigureInterrupt(0, SX1509_EDGE_FALLING));
	assert(bus.Reg[0x17] == 2);
	assert(!exp.ConfigureDebounce(1, 2)); // Oscillator initially off.
	assert(exp.ConfigureClock(4));
	assert(bus.Reg[0x1E] == 0x40 && (bus.Reg[0x1F] & 0x71) == 0x41);
	assert(exp.ConfigureDebounce(0x8001, 7));
	assert(bus.Reg[0x23] == 0x80 && bus.Reg[0x24] == 1);
	assert(!exp.ConfigureLed(0, led)); // No breathing on IO0.
	assert(exp.ConfigureLed(15, led));
	assert(bus.Reg[0x64] == 3 && bus.Reg[0x65] == 128 && bus.Reg[0x66] == 34);
	assert(bus.Reg[0x67] == 5 && bus.Reg[0x68] == 6);
	assert(exp.LedEnable(15, true) && (bus.Reg[0x10] & 0x80) == 0);
	assert(exp.LedEnable(15, false) && (bus.Reg[0x10] & 0x80) != 0);
	led.RiseTime = 0;
	led.FallTime = 0;
	for (pin = 0; pin < 16; ++pin)
		assert(exp.ConfigureLed(pin, led));
	bus.FailWrite = 0x11;
	assert(!exp.WritePins(1, 1));
	bus.FailWrite = -1;
	assert(!exp.WritePins(2, 2)); // Ambiguous write invalidated bank A shadow.
	assert(exp.WritePins(0x100, 0x100)); // Bank B remains known.
	assert(exp.WriteOutputs(0xA55A));
	assert(bus.Reg[0x10] == 0xA5 && bus.Reg[0x11] == 0x5A);
	bus.ShortRead = 0x11;
	pins = 0x1234;
	assert(!exp.ReadPins(pins) && pins == 0x1234);
	bus.ShortRead = -1;
	assert(exp.SoftReset());
	assert(exp.WritePins(1, 0) && bus.Reg[0x11] == 0xFE);
}

int main()
{
	TestColor();
	TestGas();
	TestStartupFailures();
	TestExpander();
	TestNearestRates();
	puts("Thingy52 production-driver tests: PASS");
	return 0;
}
