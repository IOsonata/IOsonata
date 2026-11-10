/**-------------------------------------------------------------------------
@file i2c_master_test.cpp
@brief Run the production I2C driver and generic transfers against modeled MMIO.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <map>
#include <deque>
#include <vector>
#include "esp32xx_i2c.h"

#define __INTERRUPT_H__
static uintptr_t s_Masked;
static uintptr_t DisableInterrupt()
{
	uintptr_t state = s_Masked;
	s_Masked = 1;
	return state;
}
static void EnableInterrupt(uintptr_t State) { s_Masked = State; }
static std::map<uintptr_t, uint32_t> s_Regs;
static std::deque<uint8_t> s_Tx;
static std::deque<uint8_t> s_Rx;
static std::vector<uint8_t> s_Written;
static std::vector<unsigned> s_Operations;
static unsigned s_Runs;
static unsigned s_FailRun;
static uint32_t s_Failure;
static unsigned s_Nacks;
static unsigned s_Stops;
static uint8_t s_NextRx;
static unsigned s_RawReads;
static uintptr_t s_ModelBase = ESP32_I2C0_BASE;
static unsigned s_ModelDepth = ESP32_I2C_FIFO_DEPTH;
static bool s_SlaveShiftActive;
static bool s_SlaveShiftValid;
static uint8_t s_SlaveShift;

static void SlavePrefetch()
{
	if (s_SlaveShiftActive && !s_SlaveShiftValid && !s_Tx.empty())
	{
		s_SlaveShift = s_Tx.front();
		s_Tx.pop_front();
		s_SlaveShiftValid = true;
	}
}

static uint8_t SlaveReadByte(bool Prefetch = true)
{
	uint8_t value;

	assert(s_SlaveShiftValid);
	value = s_SlaveShift;
	s_SlaveShiftValid = false;
	if (Prefetch)
	{
		SlavePrefetch();
	}
	return value;
}

static void Execute()
{
	uint32_t command;
	unsigned opcode;
	unsigned bytes;
	unsigned slot;
	unsigned i;

	assert(!s_Masked);
	s_Runs++;
	if (s_FailRun == s_Runs)
	{
		s_Regs[s_ModelBase + 0x20] = s_Failure;
		return;
	}
	for (slot = 0; slot < 8; slot++)
	{
		command = s_Regs[s_ModelBase + 0x58 + 4 * slot];
		opcode = (command >> 11) & 7;
		bytes = command & 255;
		s_Operations.push_back(opcode);
		if (opcode == 6)
		{
			s_Regs[s_ModelBase + 8] = 16;
		}
		else if (opcode == 1)
		{
			assert(command & 256);
			assert(!(command & 512));
			assert(bytes <= s_Tx.size());
			for (i = 0; i < bytes; i++)
			{
				s_Written.push_back(s_Tx.front());
				s_Tx.pop_front();
			}
		}
		else if (opcode == 3)
		{
			assert(bytes > 0);
			assert(s_Rx.size() + bytes <= s_ModelDepth);
			if (command & 1024)
			{
				assert(bytes == 1);
				s_Nacks++;
			}
			for (i = 0; i < bytes; i++)
			{
				s_Rx.push_back(s_NextRx++);
			}
		}
		else if (opcode == 2)
		{
			s_Stops++;
			s_Regs[s_ModelBase + 8] = 0;
			s_Regs[s_ModelBase + 0x20] = 128;
			return;
		}
		else
		{
			assert(opcode == 4);
			s_Regs[s_ModelBase + 0x20] = 8;
			return;
		}
	}
	assert(false);
}
struct Register {
	uintptr_t Address;
	operator uint32_t() const
	{
		uint8_t value;

		if (Address == s_ModelBase + 0x1c)
		{
			assert(!s_Rx.empty());
			value = s_Rx.front();
			s_Rx.pop_front();
			return value;
		}
		if (Address == s_ModelBase + 0x20)
		{
			s_RawReads++;
		}
		if (Address == s_ModelBase + 8 && !(s_Regs[s_ModelBase + 4] & 16))
		{
			return (s_Regs[Address] & ~((63U << 8) | (63U << 18))) | (s_Rx.size() << 8) | (s_Tx.size() << 18);
		}
		return s_Regs[Address];
	}
	void operator=(uint32_t Value)
	{
		s_Regs[Address] = Value;
		if (Address == s_ModelBase + 0x24)
		{
			s_Regs[s_ModelBase + 0x20] &= ~Value;
		}
		if (Address == s_ModelBase + 0x18)
		{
			if (Value & (1U << 12))
			{
				s_Rx.clear();
			}
			if (Value & (1U << 13))
			{
				s_Tx.clear();
			}
		}
		if (Address == s_ModelBase + 0x1c)
		{
			assert(s_Tx.size() < s_ModelDepth);
			s_Tx.push_back(Value);
			SlavePrefetch();
		}
		if (Address == s_ModelBase + 4)
		{
			s_Regs[Address] &= ~((1U << 5) | (1U << 10) | (1U << 11));
			if (Value & (1U << 10))
			{
				s_Regs[s_ModelBase + 8] = 0;
			}
			if (Value & (1U << 5))
			{
				Execute();
			}
		}
	}
	void operator|=(uint64_t Value) { *this = uint32_t(*this) | Value; }
	void operator&=(uint64_t Value) { *this = uint32_t(*this) & Value; }
};
#undef ESP32_REG32
#define ESP32_REG32(addr) Register{uintptr_t(addr)}
#include "esp32xx_irq.h"
static Esp32IrqHandler s_Irq;
static bool s_IrqFail;
bool Esp32EnableSourceIrq(uint32_t Source, uint32_t Cpu, uint32_t Priority, Esp32IrqHandler Handler)
{
	(void)Source;
	(void)Cpu;
	assert(Priority > 0 && Priority <= 7);
	if (s_IrqFail)
	{
		return false;
	}
	s_Irq = Handler;
	return true;
}
void Esp32DisableSourceIrq(uint32_t Source, uint32_t Cpu)
{
	(void)Source;
	(void)Cpu;
	s_Irq = NULL;
}
#include "../../RISCV/Espressif/src/i2c_esp32.cpp"
#include "../../src/device_intrf.cpp"

McuOsc_t g_McuOsc = {};
uint32_t SystemCoreClockGet() { return 1000; }
static unsigned s_PinConfigs;
void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	assert(Port == 0);
	assert(Pin >= 4 && Pin <= 7);
	assert(Res == IOPINRES_NONE);
	if (Dir == IOPINDIR_BI)
	{
		assert(Type == IOPINTYPE_OPENDRAIN);
		#if defined(ESP32C6)
		if (Pin >= 6)
		{
			assert(Op == ESP32_PINOP_LP_I2C_SDA);
		}
		else
#endif
		{
			assert(Op == IOPINOP_FUNC0 + (Pin == 4 ? ESP32_I2C_SDA_SIGNAL : ESP32_I2C_SCL_SIGNAL));
		}
	}
	else
	{
		assert(Dir == IOPINDIR_INPUT);
	}
	s_PinConfigs++;
}
static void TraceReset()
{
	s_Written.clear();
	s_Operations.clear();
	s_Runs = 0;
	s_FailRun = 0;
	s_Failure = 0;
	s_Nacks = 0;
	s_Stops = 0;
	s_NextRx = 0;
	s_RawReads = 0;
}
// Compare selection with an exhaustive, independent timing search.
static void CheckRates(DevIntrf_t *pIntrf)
{
	const uint32_t requests[] = {0U, 1U, 1001U, 79123U, 79999U, 80001U, 123456U, 400001U, UINT32_MAX};
	uint32_t request;
	uint32_t wanted;
	uint32_t best;
	uint32_t pre;
	uint32_t half;
	uint32_t actual;
	uint32_t wait;
	uint32_t error;
	uint32_t rate;
	unsigned i;

	for (i = 0; i < sizeof(requests) / sizeof(requests[0]); i++)
	{
		request = requests[i];
		wanted = request < 1000U ? 1000U : (request > 400000U ? 400000U : request);
		best = UINT32_MAX;
		for (pre = 1; pre <= 256; pre++)
		{
			for (half = 32; half <= 511; half++)
			{
				actual = 40000000U / (2U * pre * half);
				wait = actual >= 80000U ? half / 2U - 2U : half / 4U;
				if (actual >= 1000U && actual <= 400000U && wait <= 127U)
				{
					error = actual > wanted ? actual - wanted : wanted - actual;
					if (error < best)
					{
						best = error;
					}
				}
			}
		}
		rate = DeviceIntrfSetRate(pIntrf, request);
		assert((rate > wanted ? rate - wanted : wanted - rate) == best);
	}
}
#if !defined(ESP32C2)
static uint8_t s_SlaveRx[64];
static uint8_t s_SlaveTx[64];
static int s_SlaveCount;
static int s_Previous;
static int SlaveEvent(DevIntrf_t *pIntrf, DEVINTRF_EVT Event, uint8_t *pBuffer, int Length)
{
	I2CDev_t *dev = (I2CDev_t *)pIntrf->GetHandle(pIntrf);

	(void)pBuffer;
	if (Event == DEVINTRF_EVT_WRITE_RQST)
	{
		dev->pTRBuff[0] = s_SlaveRx;
		dev->TRBuffLen[0] = sizeof(s_SlaveRx);
	}
	if (Event == DEVINTRF_EVT_READ_RQST)
	{
		s_Previous = Length;
		dev->pRRData[0] = s_SlaveTx;
		dev->RRDataLen[0] = sizeof(s_SlaveTx);
	}
	if (Event == DEVINTRF_EVT_COMPLETED)
	{
		s_SlaveCount = Length;
	}
	return 0;
}
static void SlaveIrq(uint32_t Status, uint32_t State)
{
	s_Regs[ESP32_I2C0_BASE + 0x2c] = Status;
	s_Regs[ESP32_I2C0_BASE + 8] = State;
	assert(s_Irq);
	if ((Status & (1U << 16)) && (State & 2U))
	{
		s_SlaveShiftActive = true;
		SlavePrefetch();
	}
	s_Irq();
	if ((Status & (1U << 7)) && !(Status & (1U << 16)))
	{
		s_SlaveShiftActive = false;
		s_SlaveShiftValid = false;
	}
}
#endif

int main()
{
	I2CDev_t dev = {};
	I2CDev_t other = {};
	IOPinCfg_t pins[] = {
		{0, 4, ESP32_PINOP_I2C0_SDA, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
		{0, 5, ESP32_PINOP_I2C0_SCL, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	};
	I2CCfg_t cfg = {};
	uint8_t data[97];
	uint8_t reg = 0x42;
	unsigned i;
	unsigned runs;
	uint32_t rate;

	g_McuOsc.CoreOsc.Freq = 40000000;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
	cfg.bIntEn = true;
	assert(!I2CInit(&dev, &cfg));
	cfg.bIntEn = false;
	cfg.bDmaEn = true;
	assert(!I2CInit(&dev, &cfg));
	cfg.bDmaEn = false;
	cfg.Mode = I2CMODE_SLAVE;
	assert(!I2CInit(&dev, &cfg));
	cfg.Mode = I2CMODE_MASTER;
	pins[0].PinOp = IOPINOP_GPIO;
	assert(!I2CInit(&dev, &cfg));
	pins[0].PinOp = ESP32_PINOP_I2C0_SDA;
	assert(I2CInit(&dev, &cfg));
	assert(!I2CInit(&other, &cfg));
	assert(!s_Masked);
	assert(s_PinConfigs == 2);
	assert(I2CGetDevHandle(&dev.DevIntrf) == &dev);
	assert(DeviceIntrfGetRate(&dev.DevIntrf) == 100000);
	assert(DeviceIntrfSetRate(&dev.DevIntrf, UINT32_MAX) == 400000);
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 1) == 1000);
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 100000) == 100000);
	CheckRates(&dev.DevIntrf);
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 100000) == 100000);
	TraceReset();
	assert(I2CStartTx(&dev, 0x50));
	assert(!I2CStartTx(&dev, 0x50));
	I2CStopTx(&dev);
	assert(s_Written == std::vector<uint8_t>({0xa0}));
	assert(s_Operations == std::vector<unsigned>({6, 1, 4, 2}));
	TraceReset();
	assert(DeviceIntrfRead(&dev.DevIntrf, 0x50, &reg, 1, data, sizeof(data)) == sizeof(data));
	assert(s_Written == std::vector<uint8_t>({0xa0, 0x42, 0xa1}));
	assert(s_Nacks == 1 && s_Stops == 1);
	for (i = 0; i < sizeof(data); i++)
	{
		assert(data[i] == i);
	}
	TraceReset();
	assert(DeviceIntrfTx(&dev.DevIntrf, 0x50, data, sizeof(data)) == sizeof(data));
	assert(s_Written.size() == sizeof(data) + 1);
	for (i = 0; i < sizeof(data); i++)
	{
		assert(s_Written[i + 1] == data[i]);
	}
	for (i = 1; i <= 3; i++)
	{
		TraceReset();
		s_FailRun = i;
		s_Failure = 1U << 10;
		assert(DeviceIntrfRead(&dev.DevIntrf, 0x50, &reg, 1, data, 1) == 0);
		assert(s_Nacks == 0 && s_Stops == 1);
		TraceReset();
		assert(DeviceIntrfRx(&dev.DevIntrf, 0x50, data, 1) == 1);
	}
	TraceReset();
	s_FailRun = 1;
	s_Failure = 1U << 5;
	assert(!I2CStartTx(&dev, 0x50));
	assert(s_Stops == 0);
	TraceReset();
	s_FailRun = 1;
	s_Failure = 0;
	assert(!I2CStartTx(&dev, 0x50));
	assert(s_RawReads > 100 && s_RawReads < 1000);
	assert(s_Stops == 1);
	TraceReset();
	s_FailRun = 1;
	s_Failure = 1U << 8;
	assert(!I2CStartTx(&dev, 0x50));
	assert(s_Stops == 1);
	TraceReset();
	s_Regs[ESP32_I2C0_BASE + 8] = 16;
	assert(!I2CStartTx(&dev, 0x50));
	assert(s_Runs == 0 && s_Stops == 0);
	s_Regs[ESP32_I2C0_BASE + 8] = 0;
	TraceReset();
	dev.DevIntrf.MaxRetry = 1;
	s_FailRun = 1;
	s_Failure = 1U << 10;
	assert(DeviceIntrfRx(&dev.DevIntrf, 0x50, data, 1) == 1);
	dev.DevIntrf.MaxRetry = 0;
	TraceReset();
	assert(I2CStartRx(&dev, 0x50));
	assert(I2CRxData(&dev, data, 1) == 1);
	assert(I2CRxData(&dev, data, 1) == 0);
	I2CStopRx(&dev);
	TraceReset();
	assert(!I2CStartTx(&dev, 128));
	assert(I2CStartTx(&dev, 0x50));
	runs = s_Runs;
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 400000) == 0);
	assert(s_Runs == runs);
	I2CStopTx(&dev);
	DeviceIntrfDisable(&dev.DevIntrf);
	assert(!I2CStartTx(&dev, 0x50));
	rate = DeviceIntrfSetRate(&dev.DevIntrf, 123456);
	assert(rate > 120000 && rate < 126000);
	DeviceIntrfEnable(&dev.DevIntrf);
	assert(DeviceIntrfGetRate(&dev.DevIntrf) == rate);
	assert(I2CStartTx(&dev, 0x50));
	I2CStopTx(&dev);
	dev.DevIntrf.PowerOff(&dev.DevIntrf);
	assert(!I2CStartTx(&dev, 0x50));
	g_McuOsc.CoreOsc.Freq = 26000000;
	assert(I2CInit(&other, &cfg));
	assert(DeviceIntrfGetRate(&other.DevIntrf) == 100000);
	other.DevIntrf.PowerOff(&other.DevIntrf);
	cfg.Mode = I2CMODE_SLAVE;
	cfg.bIntEn = true;
	cfg.IntPrio = 4;
	cfg.NbSlaveAddr = 1;
	cfg.SlaveAddr[0] = 0x22;
#if defined(ESP32C2)
	assert(!I2CInit(&dev, &cfg));
#else
	cfg.EvtCB = SlaveEvent;
	assert(I2CInit(&dev, &cfg));
	assert(s_Irq && dev.DevIntrf.bIntEn);
	assert(!I2CStartTx(&dev, 0x22));
	// A write need not generate an address-stretch interrupt. Receive it at
	// STOP, or preserve its command when the next address starts a read.
	s_Rx = {4, 0xa0, 0xa1};
	SlaveIrq(1U << 7, 0);
	assert(s_SlaveCount == 3 && s_SlaveRx[0] == 4 && s_SlaveRx[1] == 0xa0);
	s_Rx = {4};
	SlaveIrq((1U << 16) | (1U << 7), 2);
	assert(s_Previous == 1 && s_SlaveRx[0] == 4);
	SlaveReadByte();
	SlaveIrq((1U << 4) | (1U << 10) | (1U << 7), 2);
	assert(s_SlaveCount == 1);
	// A longer write first becomes visible at the RX watermark.
	s_Rx = {1, 2, 3};
	SlaveIrq(1U, 0);
	s_Rx = {4, 5};
	SlaveIrq(1U << 7, 0);
	assert(s_SlaveCount == 5 && s_SlaveRx[0] == 1 && s_SlaveRx[4] == 5);
	SlaveIrq(1U << 16, 0);
	s_Rx = {2, 0xab, 0xcd};
	SlaveIrq(1U << 7, 0);
	assert(s_SlaveCount == 3 && s_SlaveRx[0] == 2 && s_SlaveRx[2] == 0xcd);
	SlaveIrq(1U << 16, 0);
	s_Rx = {7};
	for (i = 0; i < 64; i++)
	{
		s_SlaveTx[i] = i + 0x40;
	}
	SlaveIrq(1U << 16, 2);
	assert(s_Previous == 1 && s_SlaveRx[0] == 7);
	for (i = 0; i < 4; i++)
	{
		assert(SlaveReadByte() == i + 0x40);
		SlaveIrq(1U << 4, 2);
	}
	SlaveIrq(1U << 7, 2);
	assert(s_SlaveCount == 4);
	assert(s_Tx.empty());
	// A new response must not begin with bytes remaining from the old one.
	for (runs = 1; runs <= 8; runs *= 2)
	{
		s_Rx = {4};
		SlaveIrq(1U << 16, 2);
		for (i = 0; i < runs; i++)
		{
			assert(SlaveReadByte() == i + 0x40);
			SlaveIrq(1U << 4, 2);
		}
		SlaveIrq(1U << 10, 2);
		SlaveIrq(1U << 7, 2);
		assert(s_SlaveCount == (int)runs);
		assert(s_Tx.empty());
	}
	// Both final-byte FIFO states occur on C6: the next byte may or may
	// not have been prefetched. Completion must report one byte either way.
	for (runs = 0; runs < 2; runs++)
	{
		SlaveIrq(1U << 16, 2);
		assert(SlaveReadByte(runs != 0) == 0x40);
		SlaveIrq((1U << 4) | (1U << 10) | (1U << 7), 2);
		assert(s_SlaveCount == 1);
		assert(s_Tx.empty());
		assert(!(s_Regs[ESP32_I2C0_BASE + 0x28] & (1U << 4)));
	}
	// Watermark refill crosses the hardware FIFO and application buffer ends.
	SlaveIrq(1U << 16, 2);
	for (i = 0; i < 80; i++)
	{
		assert(SlaveReadByte() == (i < 64 ? i + 0x40 : 0xff));
		SlaveIrq(1U << 4, 2);
		if (s_Tx.size() == 8)
		{
			SlaveIrq(1U << 1, 2);
		}
	}
	SlaveIrq(1U << 7, 2);
	assert(s_SlaveCount == 80);
	SlaveIrq(1U << 16, 0);
	for (i = 0; i < 65; i++)
	{
		s_Rx.push_back(i);
		if (s_Rx.size() == 16)
		{
			SlaveIrq(1U, 0);
		}
	}
	SlaveIrq(1U << 7, 0);
	assert(s_SlaveCount == -1);
	for (i = 0; i < 64; i++)
	{
		assert(s_SlaveRx[i] == i);
	}
	dev.DevIntrf.Reset(&dev.DevIntrf);
	assert(s_Irq && (s_Regs[ESP32_I2C0_BASE + 0x28] & (1U << 16)));
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 400000) > 0);
	assert(s_Regs[ESP32_I2C0_BASE + 0xc] == 0);
	SlaveIrq(1U << 16, 0);
	s_Rx = {0x55};
	SlaveIrq(1U << 7, 0);
	assert(s_SlaveCount == 1 && s_SlaveRx[0] == 0x55);
	dev.DevIntrf.PowerOff(&dev.DevIntrf);
	assert(!s_Irq);
	s_IrqFail = true;
	assert(!I2CInit(&dev, &cfg));
	s_IrqFail = false;
	assert(I2CInit(&dev, &cfg));
	dev.DevIntrf.PowerOff(&dev.DevIntrf);
#endif
#if defined(ESP32C6)
	// Keep the HP slave active while LP master transactions and recovery run.
	assert(I2CInit(&dev, &cfg));
	s_ModelBase = ESP32_LP_I2C_BASE;
	s_ModelDepth = 16;
	g_McuOsc.CoreOsc.Freq = 40000000;
	cfg.DevNo = 1;
	assert(!I2CInit(&other, &cfg));
	cfg.Mode = I2CMODE_MASTER;
	cfg.bIntEn = false;
	cfg.EvtCB = NULL;
	assert(!I2CInit(&other, &cfg));
	pins[0].PinNo = 6;
	pins[1].PinNo = 7;
	pins[0].PinOp = ESP32_PINOP_LP_I2C_SDA;
	pins[1].PinOp = ESP32_PINOP_LP_I2C_SCL;
	rate = ESP32_I2C_PCR_SCLK_CONF_REG;
	assert(!I2CInit(&dev, &cfg));
	assert(I2CInit(&other, &cfg));
	assert(s_Irq);
	assert(uint32_t(ESP32_I2C_PCR_SCLK_CONF_REG) == rate);
	assert(uint32_t(ESP32_LP_CLKRST_PERI_REG) & (1U << 30));
	assert(uint32_t(ESP32_LP_PERI_CLK_EN_REG) & (1U << 28));
	assert(!(s_Regs[ESP32_LP_I2C_BASE + 4] & 0x13U));
	assert(DeviceIntrfSetRate(&other.DevIntrf, 100000) == 100000);
	assert(s_Regs[ESP32_LP_I2C_BASE] == 99);
	assert(DeviceIntrfSetRate(&other.DevIntrf, 400000) == 400000);
	assert(s_Regs[ESP32_LP_I2C_BASE] == 24);
	assert(uint32_t(ESP32_I2C_PCR_SCLK_CONF_REG) == rate);
	TraceReset();
	assert(DeviceIntrfRead(&other.DevIntrf, 0x22, &reg, 1, data, sizeof(data)) == sizeof(data));
	assert(s_Nacks == 1 && s_Stops == 1);
	assert(s_Written == std::vector<uint8_t>({0x44, 0x42, 0x45}));
	for (i = 0; i < sizeof(data); i++)
	{
		assert(data[i] == i);
	}
	TraceReset();
	assert(DeviceIntrfTx(&other.DevIntrf, 0x22, data, sizeof(data)) == sizeof(data));
	assert(s_Written.size() == sizeof(data) + 1);
	// BUS_BUSY may read clear while a successful sequence still needs STOP.
	TraceReset();
	assert(I2CStartTx(&other, 0x22));
	s_Regs[ESP32_LP_I2C_BASE + 8] = 0;
	I2CStopTx(&other);
	assert(s_Stops == 1);
	TraceReset();
	s_FailRun = 1;
	s_Failure = 1U << 10;
	assert(DeviceIntrfRx(&other.DevIntrf, 0x22, data, 1) == 0);
	assert(s_Stops == 0);
	assert(s_Runs == 1);
	TraceReset();
	assert(DeviceIntrfRx(&other.DevIntrf, 0x22, data, 1) == 1);
	other.DevIntrf.PowerOff(&other.DevIntrf);
	assert(s_Irq);
	assert(uint32_t(ESP32_I2C_PCR_SCLK_CONF_REG) == rate);
	assert(!(uint32_t(ESP32_LP_PERI_CLK_EN_REG) & (1U << 28)));
	s_ModelBase = ESP32_I2C0_BASE;
	s_ModelDepth = ESP32_I2C_FIFO_DEPTH;
	SlaveIrq(1U << 16, 0);
	s_Rx = {0x77};
	SlaveIrq(1U << 7, 0);
	assert(s_SlaveCount == 1 && s_SlaveRx[0] == 0x77);
	// Restore HP pin descriptors before releasing the HP slave.
	pins[0].PinNo = 4;
	pins[1].PinNo = 5;
	dev.DevIntrf.PowerOff(&dev.DevIntrf);
#endif
	return 0;
}
