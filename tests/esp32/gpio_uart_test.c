/**-------------------------------------------------------------------------
@file gpio_uart_test.c
@brief Check actual UART pad routing on C2/C5/C61 host MMIO.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#define __INTERRUPT_H__
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
#if defined(ESP32C2)
	gpio = 0x60004000;
	mux = 0x60009004;
	input = gpio + 0x154;
	output = gpio + 0x554;
	enable = gpio + 0x24;
	select = 1U << 6;
#else
	gpio = 0x60091000;
	mux = 0x60090000;
	input = gpio + 0x2D4;
	output = gpio + 0xAD4;
	enable = gpio + 0x38;
	select = 1U << 8;
#endif
	IOPinConfig(0, 4, ESP32_PINOP_U0TXD, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 5, ESP32_PINOP_U0RXD, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	assert(REG(output + 4 * 4) == ESP32_PINOP_U0TXD - IOPINOP_FUNC0);
	assert(REG(input + 4 * (ESP32_PINOP_U0RXD - IOPINOP_FUNC0)) == (5U | select));
	assert(REG(enable) == (1U << 4));
	assert((REG(mux + 4 * 4) & (7U << 12)) == (1U << 12));
	assert((REG(mux + 5 * 4) & ((1U << 9) | (1U << 8))) == ((1U << 9) | (1U << 8)));
	return 0;
}
