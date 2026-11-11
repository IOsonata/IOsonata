/**-------------------------------------------------------------------------
@file gpio_uart_test.c
@brief Check actual UART pad routing on all C-series host MMIO.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#define __INTERRUPT_H__
static uintptr_t DisableInterrupt(void) { return 0; }
static void EnableInterrupt(uintptr_t State) { (void)State; }
#include "RISCV/Espressif/src/iopincfg_esp32.c"
#include "esp32xx_uart.h"

#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))

int main(void)
{
	void *mem;
	uintptr_t gpio;
	uintptr_t mux;
	uintptr_t input;
	uintptr_t output;
	uintptr_t enable;
	uint32_t select;

	mem = mmap((void *)0x60000000, 0xD0000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(mem != MAP_FAILED);
#if defined(ESP32C2) || defined(ESP32C3)
	gpio = 0x60004000;
	mux = 0x60009004;
	input = gpio + 0x154;
	output = gpio + 0x554;
	enable = gpio + 0x24;
	select = 1U << 6;
#elif defined(ESP32C6)
	gpio = 0x60091000;
	mux = 0x60090004;
	input = gpio + 0x154;
	output = gpio + 0x554;
	enable = gpio + 0x24;
	select = 1U << 7;
#else
	gpio = 0x60091000;
	mux = 0x60090000;
	input = gpio + 0x2D4;
	output = gpio + 0xAD4;
	enable = gpio + 0x38;
#if defined(ESP32C61)
	select = 1U << 7;
#else
	select = 1U << 8;
#endif
#endif
	IOPinConfig(0, 4, ESP32_PINOP_U0TXD, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 5, ESP32_PINOP_U0RXD, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	assert(REG(output + 4 * 4) == ESP32_PINOP_U0TXD - IOPINOP_FUNC0);
	assert(REG(input + 4 * (ESP32_PINOP_U0RXD - IOPINOP_FUNC0)) == (5U | select));
	assert(REG(enable) == (1U << 4));
	assert((REG(mux + 4 * 4) & (7U << 12)) == (1U << 12));
	assert((REG(mux + 5 * 4) & ((1U << 9) | (1U << 8))) == ((1U << 9) | (1U << 8)));
#if defined(ESP32C6)
	// Literal hardware encodings: matrix select is bit 7, inversion is bit 6.
	assert(ESP32GPIOConnectInput(68, 1, false));
	assert(REG(input + 68 * 4) == 0x81);
	assert(ESP32GPIOConnectInput(63, 3, false));
	assert(REG(input + 63 * 4) == 0x83);
	assert(ESP32GPIOConnectInput(65, 6, true));
	assert(REG(input + 65 * 4) == 0xC6);
	assert(ESP32_GPIO_FUNC_IN_CONST_HIGH == 0x38);
	assert(ESP32_GPIO_FUNC_IN_CONST_LOW == 0x3C);
#elif defined(ESP32C61)
	assert(ESP32_GPIO_FUNC_IN_CONST_HIGH == 0x20);
	assert(ESP32_GPIO_FUNC_IN_CONST_LOW == 0x30);
#endif
	return 0;
}
