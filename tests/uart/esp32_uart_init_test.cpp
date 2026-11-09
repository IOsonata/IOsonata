/**-------------------------------------------------------------------------
@file esp32_uart_init_test.cpp
@brief Host MMIO checks of the production UART initializer and clock helpers.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <map>
#include <vector>
#include <utility>

#define __INTERRUPT_H__
static uintptr_t DisableInterrupt() { return 0; }
static void EnableInterrupt(uintptr_t State) { (void)State; }

static std::map<uintptr_t, uint32_t> s_Regs;
static std::vector<std::pair<uintptr_t, uint32_t>> s_Writes;
struct Register {
	uintptr_t Address;
	operator uint32_t() const
	{
		uint32_t value = s_Regs[Address];
		// Synchronized UART configuration commits clear in hardware.
		if ((Address & 0xFFFU) == 0x98U)
		{
			value = 0;
		}
		return value;
	}
	void operator=(uint32_t Value)
	{
		s_Regs[Address] = Value;
		s_Writes.emplace_back(Address, Value);
	}
	void operator|=(uint64_t Value) { *this = uint32_t(*this) | Value; }
	void operator&=(uint64_t Value) { *this = uint32_t(*this) & Value; }
};
#define ESP32_REG32(addr) Register{uintptr_t(addr)}
#include "../../RISCV/Espressif/src/uart_esp32.cpp"
#include "../../RISCV/Espressif/src/system_esp32_uart_clock.c"

McuOsc_t g_McuOsc;
volatile uint32_t g_Esp32C3Uart0IrqCount;
volatile uint32_t g_Esp32C3Uart1IrqCount;
static uint32_t s_Source;
static unsigned s_PinCalls;
void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	(void)Port;
	(void)Pin;
	(void)Op;
	(void)Dir;
	(void)Res;
	(void)Type;
	s_PinCalls++;
}
bool Esp32EnableSourceIrq(uint32_t Source, uint32_t Cpu, uint32_t Priority,
	Esp32IrqHandler Handler)
{
	assert(Cpu > 0 && Priority > 0 && Handler != nullptr);
	s_Source = Source;
	return true;
}

int main()
{
	UARTDev_t dev = {};
	UARTCfg_t cfg = {};
	IOPinCfg_t pins[4] = {};
	uint32_t base;
	uint32_t clock;
	unsigned before;
	int instance;

	g_McuOsc.CoreOsc.Freq = 40000000;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.StopBits = 1;
	cfg.Parity = UART_PARITY_NONE;
	cfg.bFifoBlocking = true;
	#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	cfg.bIntMode = false;
#else
	cfg.bIntMode = true;
#endif
	for (instance = 0; instance < ESP32_HP_UART_COUNT; instance++)
	{
		s_Regs.clear();
		s_Writes.clear();
		cfg.DevNo = instance;
		assert(UARTInit(&dev, &cfg));
		base = s_Esp32UartDev[instance].Base;
		assert(s_Regs[base + 0x14] == 40);
		assert(dev.Rate == 1000000);
		assert(s_Regs[base + 0x20] == 0x1C); // 8N1, no inversion/flow.
		assert(s_Regs[base + 0x48] == 0);
#if defined(ESP32C2) || defined(ESP32C3)
		assert(s_Regs[base + 0x24] == (3U | (16U << 9) | (1U << 21)));
		assert((s_Regs[base + 0x60] >> 16 & 1023U) == 10U);
		assert((s_Regs[base + 0x78] >> 20 & 3U) == 3U);
		assert((s_Regs[base + 0x78] & (7U << 23)) == (3U << 24));
		assert(s_Regs[base + 0x80] == 0); // No artificial ID commit.
#else
		assert(s_Regs[base + 0x24] == (3U | (16U << 8)));
		assert(s_Regs[base + 0x64] == 41U);
		assert(s_Regs[base + 0x88] == (3U << 24));
		assert(s_Regs[base + 0x98] == 1);
		clock = s_Regs[ESP32_PCR_BASE + instance * 12 + 4];
#if defined(ESP32C6)
		assert(clock == ((3U << 20) | (1U << 22)));
#else
		assert(clock == (1U << 22));
#endif
#endif
#if defined(ESP32C2)
		assert(s_Source == 0);
#elif defined(ESP32C3)
		assert(s_Source == 21U + instance);
#elif defined(ESP32C5)
		assert(s_Source == 0);
#elif defined(ESP32C6)
		assert(s_Source == 43U + instance);
#else
		assert(s_Source == 0);
#endif
		assert(Esp32UartSetRate(&dev.DevIntrf, 9600) == 9600);
		assert(Esp32UartSetRate(&dev.DevIntrf, 115200) == 115191);
		assert(Esp32UartSetRate(&dev.DevIntrf, 0) == 0);
		assert(dev.Rate == 115191);
		assert(Esp32UartSetRate(&dev.DevIntrf, 32800000) == 32000000);
		assert(Esp32UartSetRate(&dev.DevIntrf, UINT32_MAX) == 40000000);
		assert(Esp32UartSetRate(&dev.DevIntrf, 1) >= 2);
	}
	// C2 boards may use a 26 MHz crystal. Use the configured oscillator.
	g_McuOsc.CoreOsc.Freq = 26000000;
	cfg.DevNo = 0;
	assert(UARTInit(&dev, &cfg));
	assert(s_Regs[ESP32_UART0_BASE + 0x14] == 26);
	cfg.NbIOPins = 4;
	cfg.FlowControl = UART_FLWCTRL_HW;
	assert(UARTInit(&dev, &cfg));
#if defined(ESP32C2) || defined(ESP32C3)
	assert(s_Regs[ESP32_UART0_BASE + 0x20] == (0x1CU | (1U << 15)));
	assert(s_Regs[ESP32_UART0_BASE + 0x24] & (1U << 20));
#else
	assert(s_Regs[ESP32_UART0_BASE + 0x20] == (0x1CU | (1U << 13)));
	assert(s_Regs[ESP32_UART0_BASE + 0x2C] == (96U | (1U << 8)));
#endif
	before = s_Writes.size();
	cfg.NbIOPins = 2;
	assert(!UARTInit(&dev, &cfg));
	cfg.FlowControl = UART_FLWCTRL_NONE;
	cfg.Rate = 0;
	assert(!UARTInit(&dev, &cfg));
	cfg.Rate = 1000000;
	cfg.bDMAMode = true;
	assert(!UARTInit(&dev, &cfg));
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
	cfg.bDMAMode = false;
	cfg.bIntMode = true;
	assert(!UARTInit(&dev, &cfg));
#endif
	assert(s_Writes.size() == before);
	assert(s_PinCalls == ESP32_HP_UART_COUNT * 2 + 6);
	return 0;
}
