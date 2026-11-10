/**-------------------------------------------------------------------------
@file spi_master_test.cpp
@brief Run the production SPI driver and generic transfers with modeled registers.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <map>
#include <vector>
#include "esp32xx_spi.h"
#include "esp32xx_gpio.h"

#define __INTERRUPT_H__
static uintptr_t s_Masked;
static uintptr_t DisableInterrupt()
{
	uintptr_t state;

	state = s_Masked;
	s_Masked = 1;
	return state;
}
static void EnableInterrupt(uintptr_t State)
{
	s_Masked = State;
}
static std::map<uintptr_t, uint32_t> s_Regs;
static std::vector<uint8_t> s_Written;
static std::vector<unsigned> s_Chunks;
static unsigned s_Runs;
static unsigned s_FailRun;
static unsigned s_Updates;
static unsigned s_FailUpdate;
static unsigned s_Reads;
static unsigned s_Asserts;
static unsigned s_Releases;
static uint32_t s_Cs;

struct Register {
	uintptr_t Address;
	operator uint32_t() const
	{
		if (Address == ESP32_SPI2_BASE)
		{
			s_Reads++;
		}
		return s_Regs[Address];
	}
	void operator=(uint32_t Value)
	{
		unsigned bytes;
		unsigned i;

		s_Regs[Address] = Value;
		if (Address == ESP32_GPIO_BASE + ESP32_GPIO_OUT_W1TS_OFFSET)
		{
			s_Cs &= ~Value;
			s_Releases++;
		}
		if (Address == ESP32_GPIO_BASE + ESP32_GPIO_OUT_W1TC_OFFSET)
		{
			s_Cs |= Value;
			s_Asserts++;
		}
		if (Address == ESP32_SPI2_BASE && (Value & (1U << 23)))
		{
			s_Updates++;
			if (s_Updates != s_FailUpdate)
			{
				s_Regs[Address] &= ~(1U << 23);
			}
		}
		if (Address == ESP32_SPI2_BASE && (Value & (1U << 24)) &&
			!(s_Regs[ESP32_SPI2_BASE + 0xe0] & (1U << 26)))
		{
			assert(!s_Masked);
			s_Runs++;
			if (s_Runs == s_FailRun)
			{
				return;
			}
			bytes = (s_Regs[ESP32_SPI2_BASE + 0x1c] + 1) / 8;
			assert(bytes > 0 && bytes <= 64);
			s_Chunks.push_back(bytes);
			for (i = 0; i < bytes; i++)
			{
				s_Written.push_back(s_Regs[ESP32_SPI2_BASE + 0x98 + (i / 4) * 4] >> ((i % 4) * 8));
			}
			// A physical MOSI-to-MISO jumper returns the same data words.
			s_Regs[Address] &= ~(1U << 24);
		}
	}
	void operator|=(uint64_t Value)
	{
		*this = uint32_t(*this) | Value;
	}
	void operator&=(uint64_t Value)
	{
		*this = uint32_t(*this) & Value;
	}
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
#include "../../RISCV/Espressif/src/spi_esp32.cpp"
#include "../../src/device_intrf.cpp"
#define __IDELAY_H__
static unsigned s_Delay;
static void usDelay(uint32_t Delay)
{
	s_Delay += Delay;
}
#include "../../src/miscdev/led_ws2812.cpp"

McuOsc_t g_McuOsc = {};
uint32_t SystemCoreClockGet()
{
	return 1000;
}
void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	(void)Op;
	(void)Dir;
	(void)Res;
	if (Port == -1 && Pin == -1)
	{
		return;
	}
	assert(Port == 0 && Pin >= 2 && Pin <= 7);
	assert(Type == IOPINTYPE_NORMAL);
}
static void TraceReset()
{
	s_Written.clear();
	s_Chunks.clear();
	s_Runs = 0;
	s_FailRun = 0;
	s_Updates = 0;
	s_FailUpdate = 0;
	s_Reads = 0;
	s_Asserts = 0;
	s_Releases = 0;
}
static void CheckRates(SPIDev_t *pDev)
{
	const uint32_t requests[] = {0, 1, 39062, 76543, 999999, 1234567, 19999999, 29999999, UINT32_MAX};
	unsigned index;
	uint32_t request;
	uint32_t actual;
	uint32_t source;
	uint32_t pre;
	uint32_t n;
	double best;
	double error;
	double candidate;

	source = GetHighFreqOscFreq();
	for (index = 0; index < sizeof(requests) / sizeof(requests[0]); index++)
	{
		request = requests[index];
		best = source;
		for (pre = 1; pre <= 16; pre++)
		{
			for (n = 2; n <= 64; n++)
			{
				candidate = double(source) / (pre * n);
				error = candidate > request ? candidate - request : request - candidate;
				if (error < (best > request ? best - request : request - best))
				{
					best = candidate;
				}
			}
		}
		actual = pDev->DevIntrf.SetRate(&pDev->DevIntrf, request);
		assert(actual == uint32_t(best + 0.5));
#if defined(ESP32C2) || defined(ESP32C3)
		// ESP-IDF spi_ll_set_clk_source: bit 2 clear selects XTAL.
		// A set bit selects PLL 80 MHz and invalidates the divider calculation.
		assert((s_Regs[SPI_BASE + 0xe8] & (1U << 2)) == 0);
#else
		assert(((s_Regs[ESP32_PCR_BASE + ESP32_SPI_PCR_CLKM_CONF_OFFSET] >> 20) & 3U) == 0);
#endif
	}
}
static uint8_t s_SlaveRx[68];
static uint8_t s_SlaveTx[64];
static int s_SlaveCount;
static unsigned s_Armed;
static int SlaveEvent(DevIntrf_t *pIntrf, DEVINTRF_EVT Event, uint8_t *pBuffer, int Length)
{
	SPIDev_t *spi = (SPIDev_t *)pIntrf->GetHandle(pIntrf);

	(void)pBuffer;
	if (Event == DEVINTRF_EVT_STATECHG)
	{
		s_Armed++;
		SPISetSlaveRxBuffer(spi, 0, s_SlaveRx + 2, 64);
		SPISetSlaveTxData(spi, 0, s_SlaveTx, 64);
	}
	if (Event == DEVINTRF_EVT_COMPLETED)
	{
		s_SlaveCount = Length;
	}
	return 0;
}

int main()
{
	SPI bus;
	LedWs2812 led;
	WS2812_CFG ledCfg = {1, WS2812_ORDER_GRB, 255, 0};
	const uint8_t wire[] = {0xab, 0x12, 0xef};
	uint8_t expected;
	unsigned pair;
	SPIDev_t dev = {};
	SPIDev_t other = {};
	IOPinCfg_t pins[] = {
		{0, 6, ESP32_PINOP_SPI2_SCK, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 7, ESP32_PINOP_SPI2_MISO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 2, ESP32_PINOP_SPI2_MOSI, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 4, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	};
	SPICfg_t cfg = {};
	uint8_t data[132];
	uint8_t command[] = {0x9f, 0x12, 0x34};
	unsigned i;
	int mode;
	int order;
	uint32_t rate;

	g_McuOsc.CoreOsc.Freq = 40000000;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 5;
	cfg.Rate = 1000000;
	cfg.DataSize = 8;
	cfg.DummyByte = 0xa5;
	cfg.bDmaEn = true;
	assert(!SPIInit(&dev, &cfg));
	cfg.bDmaEn = false;
	cfg.bIntEn = true;
	assert(!SPIInit(&dev, &cfg));
	cfg.bIntEn = false;
	cfg.Mode = SPIMODE_SLAVE;
	assert(!SPIInit(&dev, &cfg));
	cfg.Mode = SPIMODE_MASTER;
	pins[1].PinNo = 6;
	assert(!SPIInit(&dev, &cfg));
	pins[1].PinNo = 7;
	pins[0].PinOp = IOPINOP_GPIO;
	assert(!SPIInit(&dev, &cfg));
	pins[0].PinOp = ESP32_PINOP_SPI2_SCK;
	for (mode = 0; mode < 4; mode++)
	{
		for (order = 0; order < 2; order++)
		{
			cfg.ClkPol = mode & 2 ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
			cfg.DataPhase = mode & 1 ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
			cfg.BitOrder = order ? SPIDATABIT_LSB : SPIDATABIT_MSB;
			assert(SPIInit(&dev, &cfg));
			assert(!SPIInit(&other, &cfg));
			assert(SPIGetHandle(&dev.DevIntrf) == &dev);
			assert(SPISetPhy(&dev, SPIPHY_QUAD_SDR) == SPIPHY_NORMAL);
			assert(((s_Regs[SPI_BASE + 0x20] >> 29) & 1) == unsigned(mode >> 1));
			assert(((s_Regs[SPI_BASE + 0x10] >> 9) & 1) == unsigned((mode >> 1) ^ (mode & 1)));
			assert(((s_Regs[SPI_BASE + 8] >> ESP32_SPI_CTRL_RD_BIT_ORDER_Pos) & 1) == unsigned(order));
			TraceReset();
			memset(data, 0, sizeof(data));
			assert(SPIRx(&dev, 1, data + 1, 129) == 129);
			assert(s_Chunks == std::vector<unsigned>({64, 64, 1}));
			assert(s_Asserts == 1 && s_Releases == 1 && s_Cs == 0);
			assert(data[0] == 0 && data[130] == 0);
			for (i = 1; i <= 129; i++)
			{
				assert(data[i] == 0xa5);
				data[i] = i;
			}
			TraceReset();
			assert(SPITx(&dev, 0, data + 1, 129) == 129);
			assert(s_Written == std::vector<uint8_t>(data + 1, data + 130));
			assert(dev.FirstRdData == 1);
			TraceReset();
			assert(DeviceIntrfRead(&dev.DevIntrf, 0, command, 3, data, 129) == 129);
			assert(dev.FirstRdData == 0x9f);
			assert(s_Chunks == std::vector<unsigned>({3, 64, 64, 1}));
			assert(s_Asserts == 1 && s_Releases == 1);
		}
	}
	TraceReset();
	assert(SPIStartTx(&dev, 0));
	assert(!SPIStartTx(&dev, 0));
	assert(!SPIInit(&dev, &cfg));
	assert(SPISetRate(&dev, 2000000) == 1000000);
	SPIStopTx(&dev);
	assert(!SPIStartTx(&dev, 2));
	for (i = 0; i < 2; i++)
	{
		TraceReset();
		if (i == 0)
		{
			s_FailRun = 1;
		}
		else
		{
			s_FailUpdate = 1;
		}
		assert(DeviceIntrfRead(&dev.DevIntrf, 0, command, 3, data, 1) == 0);
		assert(s_Runs == (i == 0 ? 1U : 0U));
		assert(s_Reads >= 100 && s_Reads < 1000);
		assert(s_Asserts == 1 && s_Releases == 1 && s_Cs == 0);
		TraceReset();
		assert(SPIRx(&dev, 0, data, 1) == 1);
	}
	TraceReset();
	s_FailRun = 2;
	assert(SPITx(&dev, 0, data, 129) == 64);
	assert(s_Cs == 0);
	TraceReset();
	CheckRates(&dev);
	assert(SPISetRate(&dev, 0) == 39063);
	assert(SPISetRate(&dev, 1) == 39063);
	assert(dev.DevIntrf.SetRate(&dev.DevIntrf, UINT32_MAX) == 40000000);
	assert(SPISetRate(&dev, 1000000) == 1000000);
	SPIEnable(&dev);
	assert(!SPIInit(&dev, &cfg));
	SPIDisable(&dev);
	assert(SPIRx(&dev, 0, data, 1) == 1);
	SPIDisable(&dev);
	assert(!SPIStartTx(&dev, 0));
	rate = SPISetRate(&dev, 123456);
	SPIEnable(&dev);
	assert(SPIGetRate(&dev) == (int)rate);
	assert(SPIRx(&dev, 0, data, 1) == 1);
	SPIPowerOff(&dev);
	assert(!SPIStartTx(&dev, 0));
	g_McuOsc.CoreOsc.Freq = 26000000;
	cfg.ChipSel = SPICSEL_MAN;
	cfg.NbIOPins = 3;
	TraceReset();
	s_FailUpdate = 1;
	assert(!SPIInit(&other, &cfg));
	TraceReset();
	assert(SPIInit(&other, &cfg));
	CheckRates(&other);
	TraceReset();
	assert(SPIRx(&other, 0, data, 1) == 1);
	assert(s_Asserts == 0 && s_Releases == 0);
	SPIPowerOff(&other);
	pins[0].PortNo = -1;
	pins[0].PinNo = -1;
	pins[1].PortNo = -1;
	pins[1].PinNo = -1;
	cfg.BitOrder = SPIDATABIT_MSB;
	cfg.ClkPol = SPICLKPOL_HIGH;
	cfg.DataPhase = SPIDATAPHASE_FIRST_CLK;
	cfg.Rate = 3333333;
	g_McuOsc.CoreOsc.Freq = 40000000;
	assert(bus.Init(cfg));
	TraceReset();
	assert(bus.Rx(0, data, 1) == 0);
	assert(s_Runs == 0);
	assert(led.Init(ledCfg, bus));
	TraceReset();
	s_Delay = 0;
	led.Level(0x12abef);
	assert(s_Chunks == std::vector<unsigned>({12}));
	assert(s_Asserts == 0 && s_Releases == 0);
	assert(s_Delay == 600);
	for (i = 0; i < 12; i++)
	{
		pair = (wire[i / 4] >> (6 - 2 * (i % 4))) & 3;
		expected = 0x88 | (pair & 2 ? 0x60 : 0) | (pair & 1 ? 0x06 : 0);
		assert(s_Written[i] == expected);
	}
	TraceReset();
	led.Off();
	assert(s_Written == std::vector<uint8_t>(12, 0x88));
	bus.PowerOff();
	assert(!s_Masked);
	cfg.Mode = SPIMODE_SLAVE;
	cfg.bIntEn = true;
	cfg.IntPrio = 4;
	cfg.NbIOPins = 4;
	cfg.ChipSel = SPICSEL_AUTO;
	cfg.EvtCB = SlaveEvent;
	pins[0] = {0, 6, ESP32_PINOP_SPI2_SCK, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	pins[1] = {0, 7, ESP32_PINOP_SPI2_MISO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	pins[2].PinDir = IOPINDIR_INPUT;
	pins[3].PinDir = IOPINDIR_INPUT;
	pins[3].PinOp = ESP32_PINOP_SPI2_CS;
	for (i = 0; i < 64; i++)
	{
		s_SlaveTx[i] = i ^ 0x5a;
	}
	for (mode = 0; mode < 4; mode++)
	{
		cfg.ClkPol = mode & 2 ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
		cfg.DataPhase = mode & 1 ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
		assert(SPIInit(&dev, &cfg));
		assert(s_Irq && dev.DevIntrf.bIntEn);
		assert(!SPIStartTx(&dev, 0));
		assert(DeviceIntrfSetRate(&dev.DevIntrf, 10000) == 10000);
		assert(s_Regs[SPI_BASE + 0xc] == 0);
		assert(((s_Regs[SPI_BASE + 0x10] >> 8) & 1) == unsigned((mode >> 1) ^ (mode & 1)));
		assert(((s_Regs[SPI_BASE + 0xe0] >> 2) & 1) == unsigned(mode & 1));
		for (unsigned length : {1U, 16U, 64U, 1U})
		{
			memset(s_SlaveRx, 0xcc, sizeof(s_SlaveRx));
			assert((s_Regs[SPI_BASE + 0x98] & 255) == s_SlaveTx[0]);
			for (i = 0; i < 16; i++)
			{
				s_Regs[SPI_BASE + 0x98 + i * 4] = 0x03020100 + i * 0x04040404;
			}
			s_Regs[SPI_BASE + 0xe4] = length * 8;
			s_Regs[SPI_BASE + 0x3c] = 1U << 12;
			s_SlaveCount = -1;
			s_Irq();
			assert(s_SlaveCount == int(length));
			for (i = 0; i < length; i++)
			{
				assert(s_SlaveRx[i + 2] == i);
			}
			assert(s_SlaveRx[0] == 0xcc && s_SlaveRx[1] == 0xcc && s_SlaveRx[length + 2] == 0xcc);
		}
		for (unsigned bits : {7U, 520U})
		{
			s_Regs[SPI_BASE + 0xe4] = bits;
			s_Regs[SPI_BASE + 0x3c] = 1U << 12;
			s_Irq();
			assert(s_SlaveCount == -1);
		}
		SPIPowerOff(&dev);
		assert(!s_Irq);
	}
	s_IrqFail = true;
	assert(!SPIInit(&dev, &cfg));
	s_IrqFail = false;
	assert(SPIInit(&dev, &cfg));
	SPIPowerOff(&dev);
	return 0;
}
