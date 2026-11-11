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
#include "esp32xx.h"
#ifdef ESP32_UART_REAL_IRQ
#include <sys/mman.h>
#endif

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
		if (Address == ESP32_UART0_BASE + 0x98U || Address == ESP32_UART1_BASE + 0x98U
#if defined(ESP32C61)
			|| Address == ESP32_UART2_BASE + 0x98U
#endif
		)
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
#ifndef ESP32_UART_REAL_IRQ
bool Esp32EnableSourceIrq(uint32_t Source, uint32_t Cpu, uint32_t Priority,
	Esp32IrqHandler Handler)
{
	assert(Cpu > 0 && Priority > 0 && Handler != nullptr);
	s_Source = Source;
	return true;
}
#else
#include "../../RISCV/Espressif/src/esp32xx_irq.c"
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
#include "../../RISCV/Espressif/src/Vectors_esp32.c"
#else
static Esp32IrqHandler s_HostHandlers[32];
void Esp32C3SetCpuIrqHandler(uint32_t Cpu, Esp32IrqHandler Handler)
{
	s_HostHandlers[Cpu] = Handler;
}
void Esp32C3ClearCpuIrqHandler(uint32_t Cpu) { s_HostHandlers[Cpu] = nullptr; }
void Esp32C3SetSourceIrqHandler(uint32_t Source, Esp32IrqHandler Handler)
{
	(void)Source;
	(void)Handler;
}
void Esp32C3ClearSourceIrqHandler(uint32_t Source) { (void)Source; }
void Esp32C3IntMtxDispatch(void) {}
#endif
static unsigned s_Events;
static int Event(UARTDev_t *pDev, UART_EVT Evt, uint8_t *pBuffer, int Size)
{
	(void)pDev;
	(void)pBuffer;
	(void)Size;
	assert(Evt == UART_EVT_TXREADY);
	s_Events++;
	return 0;
}
#endif

static void CheckSource(uint32_t Source, unsigned Instance)
{
#ifdef ESP32_UART_REAL_IRQ
	unsigned cpu;

	cpu = Instance == 0 ? 2 : 3;
#if defined(ESP32C6)
	cpu = Instance == 0 ? 2 : 5;
#elif defined(ESP32C61)
	if (Instance == 2)
	{
		cpu = 8;
	}
#endif
	assert(s_Regs[ESP32_INTMTX_BASE + Source * 4] == cpu + ESP32_INTMTX_CPU_OFFSET);
#else
	(void)Instance;
	assert(s_Source == Source);
#endif
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

#if defined(ESP32_UART_REAL_IRQ) && (defined(ESP32C5) || defined(ESP32C61))
	void *mapped;

	mapped = mmap(reinterpret_cast<void *>(ESP32_CLIC_BASE), 8192,
		PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(mapped == reinterpret_cast<void *>(ESP32_CLIC_BASE));
#endif
#ifdef ESP32_UART_REAL_IRQ
	cfg.EvtCallback = Event;
#endif
	g_McuOsc.CoreOsc.Freq = 40000000;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.StopBits = 1;
	cfg.Parity = UART_PARITY_NONE;
	cfg.bFifoBlocking = true;
		cfg.bIntMode = true;
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
		CheckSource(17U + instance, instance);
#elif defined(ESP32C3)
		CheckSource(21U + instance, instance);
#elif defined(ESP32C5)
		CheckSource(47U + instance, instance);
#elif defined(ESP32C6)
		CheckSource(43U + instance, instance);
#else
		CheckSource(40U + instance, instance);
#endif
#ifdef ESP32_UART_REAL_IRQ
		s_Regs[base + 0x08] = 2; // TX FIFO empty interrupt.
#if defined(ESP32C2) || defined(ESP32C5) || defined(ESP32C61)
		assert(Esp32DispatchCpuIrq(0x80000000U | ESP32_INTMTX_SOURCE_MAP_REG(
			instance == 0 ? ESP32_UART_INTR_SOURCE_UART0 :
			(instance == 1 ? ESP32_UART_INTR_SOURCE_UART1 : ESP32_UART_INTR_SOURCE_UART1 + 1))));
#else
		s_HostHandlers[instance == 0 ? UART0_CPU_INT_LEVEL : UART1_CPU_INT_LEVEL]();
#endif
		assert(s_Events == unsigned(instance + 1));
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
	assert(s_Writes.size() == before);
	assert(s_PinCalls == ESP32_HP_UART_COUNT * 2 + 6);
	return 0;
}
