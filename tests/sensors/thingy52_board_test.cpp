#include <assert.h>
#include <stdio.h>
#include <vector>
#include "thingy52_board.h"
#include "sensors/gas_ccs811.h"
#include "thingy52_mock.h"

uint64_t g_DelayUs;
static uint32_t s_PinValues;
static uint32_t s_PinOutputs;
static std::vector<int> s_PinWrites;

void IOPinSet(int PortNo, int PinNo)
{
	assert(PortNo == 0 && PinNo >= 0 && PinNo < 32);
	s_PinValues |= 1UL << PinNo;
	s_PinWrites.push_back(PinNo);
}

void IOPinClear(int PortNo, int PinNo)
{
	assert(PortNo == 0 && PinNo >= 0 && PinNo < 32);
	s_PinValues &= ~(1UL << PinNo);
	s_PinWrites.push_back(-PinNo);
}

int IOPinRead(int PortNo, int PinNo)
{
	assert(PortNo == 0);
	return (s_PinValues >> PinNo) & 1;
}

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp,
	IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)Resistor;
	assert(PortNo == 0 && PinOp == IOPINOP_GPIO && Type == IOPINTYPE_NORMAL);
	if (Dir == IOPINDIR_OUTPUT)
		s_PinOutputs |= 1UL << PinNo;
	else
		s_PinOutputs &= ~(1UL << PinNo);
}

// Route both physical devices through one bus, as on Thingy:52.
class BoardBus : public MockIntrf {
public:
	MockIntrf Gas;
	BoardBus()
	{
		Sx = true;
		Gas.PrepareCcs();
	}
	int Read(uint32_t Addr, const uint8_t *pCmd, int CmdLen, uint8_t *pData, int Len) override
	{
		if (Addr == 0x5A)
		{
			assert((Reg[0x10] & 0x1C) == 0x0C); // Supply/reset high, wake low.
			return Gas.Read(Addr, pCmd, CmdLen, pData, Len);
		}
		assert(Addr == 0x3E);
		return MockIntrf::Read(Addr, pCmd, CmdLen, pData, Len);
	}
	int Write(uint32_t Addr, const uint8_t *pCmd, int CmdLen, const uint8_t *pData, int Len) override
	{
		if (Addr == 0x5A)
		{
			assert((Reg[0x10] & 0x1C) == 0x0C);
			return Gas.Write(Addr, pCmd, CmdLen, pData, Len);
		}
		assert(Addr == 0x3E);
		return MockIntrf::Write(Addr, pCmd, CmdLen, pData, Len);
	}
	int Tx(uint32_t Addr, const uint8_t *pData, int Len) override
	{
		assert(Addr == 0x5A && (Reg[0x10] & 0x1C) == 0x0C);
		return Gas.Tx(Addr, pData, Len);
	}
};

static void TestPowerAndPins()
{
	Thingy52PowerInit();
	assert(s_PinWrites.size() == 3);
	assert(s_PinWrites[0] == -29 && s_PinWrites[1] == 30 && s_PinWrites[2] == 16);
	assert(s_PinOutputs == ((1UL << 29) | (1UL << 30) | (1UL << 16)));
	assert((s_PinValues & (1UL << 29)) == 0 && (s_PinValues & (1UL << 30)) != 0);
	assert(g_DelayUs >= 7000);
	s_PinValues &= ~((1UL << 11) | (1UL << 22) | (1UL << 31));
	assert(Thingy52ButtonPressed() && Thingy52ColorIrqActive() && Thingy52GasIrqActive());
	s_PinValues |= (1UL << 11) | (1UL << 22) | (1UL << 31);
	assert(!Thingy52ButtonPressed() && !Thingy52ColorIrqActive() && !Thingy52GasIrqActive());
}

static void TestBoardSequences()
{
	BoardBus bus;
	IoExpSx1509 expander;
	GasCcs811 gas;
	GasSensorCfg_t cfg = {};
	Ccs811Data_t sample;
	uint64_t before;
	std::vector<unsigned> writes;
	unsigned i;

	assert(Thingy52ExpanderInit(expander, bus));
	assert(bus.Reg[0x0E] == 0 && bus.Reg[0x0F] == 0x1F);
	assert(bus.Reg[0x10] == 0xE0 && bus.Reg[0x11] == 0xE0);
	assert(bus.Reg[0x20] == 0xE0 && bus.Reg[0x21] == 0xE0);
	assert(bus.Reg[0x06] == 0 && bus.Reg[0x07] == 0); // No pull-ups into unpowered domains.
	before = g_DelayUs;
	assert(Thingy52GasPower(expander, true));
	assert(g_DelayUs - before >= 32000);
	assert(bus.Reg[0x10] == 0xFC);
	assert(Thingy52Lightwell(expander, true, false, false));
	assert(bus.Reg[0x11] == 0x60 && bus.Reg[0x10] == 0xFC);
	assert(Thingy52ColorLight(expander, true, true, true));
	assert(bus.Reg[0x10] == 0x1C);
	assert(Thingy52GasWake(true, &expander) && bus.Reg[0x10] == 0x0C);
	assert(Thingy52GasWake(false, &expander) && bus.Reg[0x10] == 0x1C);
	assert(!Thingy52GasWake(true, nullptr));
	gas.WakeControl(Thingy52GasWake, &expander);
	cfg.DevAddr = CCS811_I2C_ADDR0;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = 1000;
	assert(gas.Init(cfg, &bus));
	assert(bus.Reg[0x10] == 0x1C);
	bus.Gas.Sample();
	assert(gas.Read(sample) && sample.Eco2 == 1234);
	assert(bus.Reg[0x10] == 0x1C); // Wake always released; illumination preserved.
	bus.Gas.FailRead = 0;
	assert(!gas.Read(sample) && bus.Reg[0x10] == 0x1C);
	bus.Ops.clear();
	assert(Thingy52GasPower(expander, false));
	for (i = 0; i < bus.Ops.size(); ++i)
	{
		if (bus.Ops[i].Write && bus.Ops[i].Reg == 0x10)
			writes.push_back(bus.Ops[i].Data[0]);
	}
	assert(writes.size() == 2 && writes[0] == 4 && writes[1] == 0);
	assert(bus.Reg[0x11] == 0x60); // Lightwell unaffected by gas power-off.
}

static void TestMotionPower()
{
	BoardBus bus;
	IoExpSx1509 expander;
	uint64_t before;

	assert(Thingy52ExpanderInit(expander, bus));
	assert(Thingy52GasPower(expander, true));
	assert(Thingy52ColorLight(expander, true, false, true));
	before = g_DelayUs;
	assert(Thingy52MotionPower(expander, true));
	assert(g_DelayUs - before >= 100000);
	assert(bus.Reg[0x10] == 0x5D && bus.Reg[0x11] == 0xE0);
	assert(Thingy52GasWake(true, &expander));
	assert(bus.Reg[0x10] == 0x4D); // Motion stays powered during gas access.
	assert(Thingy52MotionPower(expander, false));
	assert(bus.Reg[0x10] == 0x4C); // Gas and illumination remain unchanged.
	bus.FailWrite = 0x10;
	assert(!Thingy52MotionPower(expander, true));
}

static void TestFailures()
{
	BoardBus bus;
	IoExpSx1509 expander;

	bus.FailRead = 0x7D;
	assert(!Thingy52ExpanderInit(expander, bus));
	bus.FailRead = -1;
	assert(Thingy52ExpanderInit(expander, bus));
	bus.FailWrite = 0x10;
	assert(!Thingy52GasPower(expander, true));
	assert(!Thingy52GasWake(true, &expander));
	assert(bus.Reg[0x10] == 0xE0);
	bus.FailWrite = -1;
	assert(!Thingy52GasPower(expander, true)); // Shadow invalid; do not invent latch state.
	assert(Thingy52ExpanderInit(expander, bus));
	assert(Thingy52GasPower(expander, true));
}

int main()
{
	TestPowerAndPins();
	TestBoardSequences();
	TestMotionPower();
	TestFailures();
	puts("Thingy52 board tests: PASS");
	return 0;
}
