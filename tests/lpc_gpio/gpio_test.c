/* LPC11Uxx GPIO regressions using vendor CMSIS peripheral declarations.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_UartPins[] = UART_PINS;

static void Config(int Port, int Pin, int Op)
{
	IOPinConfig(Port, Pin, Op, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

int main(void)
{
	assert(mmap((void *)0x40000000, 0x100000, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x40000000);
	assert(mmap((void *)0x50000000, 0x10000, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x50000000);

	// UART board constants must select FUNC1, not the generic FUNC0 value.
	LPC_SYSCON->SYSAHBCLKCTRL = 1U << 12;
	IOPinCfg(s_UartPins, 4);
	assert((LPC_IOCON->PIO0_18 & 7U) == 1U);
	assert((LPC_IOCON->PIO0_19 & 7U) == 1U);
	assert((LPC_IOCON->PIO0_7 & 7U) == 1U);
	assert((LPC_IOCON->PIO0_17 & 7U) == 1U);
	assert(LPC_SYSCON->SYSAHBCLKCTRL == ((1U << 12) | (1U << 6) | (1U << 16)));
	assert((LPC_GPIO->DIR[0] & ((1U << 18) | (1U << 7))) == 0U);
	assert((LPC_GPIO->DIR[0] & ((1U << 19) | (1U << 17))) == ((1U << 19) | (1U << 17)));

	// Sweep register selection against documented IOCON addresses, including
	// the P0/P1 boundary. Neighboring registers must remain unchanged.
	for (unsigned i = 0; i < 56; i++)
	{
		volatile uint32_t *regs = (volatile uint32_t *)0x40044000;
		for (unsigned j = 0; j < 56; j++)
		{
			regs[j] = 0x80U;
		}
		Config(i < 24 ? 0 : 1, i < 24 ? i : i - 24, IOPINOP_GPIO);
		for (unsigned j = 0; j < 56; j++)
		{
			if (j != i)
			{
				assert(regs[j] == 0x80U);
			}
		}
		unsigned gpiofunc = i == 0 || (i >= 10 && i <= 15) ? 1U : 0U;
		assert((regs[i] & 7U) == gpiofunc);
	}

	// P1_27 supports FUNC4; bit 2 must survive, then clear on return to GPIO.
	Config(1, 27, IOPINOP_FUNC4);
	assert((LPC_IOCON->PIO1_27 & 7U) == 4U);
	Config(1, 27, IOPINOP_GPIO);
	assert((LPC_IOCON->PIO1_27 & 7U) == 0U);
	Config(0, 0, IOPINOP_FUNC0);
	assert((LPC_IOCON->RESET_PIO0_0 & 7U) == 0U);
	Config(0, 0, IOPINOP_GPIO);
	assert((LPC_IOCON->RESET_PIO0_0 & 7U) == 1U);

	// ADC selection disables digital input and pulls; GPIO restores digital.
	for (int pin = 11; pin <= 23; pin++)
	{
		if (pin > 16 && pin < 22)
		{
			continue;
		}
		volatile uint32_t *reg = (volatile uint32_t *)(uintptr_t)(0x40044000U + 4U * pin);
		*reg = 0x7FFU;
		IOPinConfig(0, pin, pin <= 15 ? IOPINOP_FUNC2 : IOPINOP_FUNC1,
					IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);
		assert((*reg & 0x4F8U) == 0U);
		assert((LPC_GPIO->DIR[0] & (1U << pin)) == 0U);
		Config(0, pin, IOPINOP_GPIO);
		assert((*reg & 0x80U) != 0U);
	}

	// Reserved I2C pad bits must not be written as pull/HYS/open-drain fields.
	for (int pin = 4; pin <= 5; pin++)
	{
		volatile uint32_t *reg = pin == 4 ? &LPC_IOCON->PIO0_4 : &LPC_IOCON->PIO0_5;
		*reg = 0xA5A50480U;
		IOPinConfig(0, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);
		assert(*reg == 0xA5A50580U);
		Config(0, pin, IOPINOP_FUNC1);
		assert(*reg == 0xA5A50481U);
	}

	LPC_IOCON->PIO0_2 = 0xA5000780U;
	IOPinConfig(0, 2, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_OPENDRAIN);
	assert(LPC_IOCON->PIO0_2 == 0xA50007A8U);
	IOPinConfig(0, 2, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	assert((LPC_IOCON->PIO0_2 & 0x478U) == 0x30U);
	IOPinConfig(0, 2, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_FOLLOW, IOPINTYPE_NORMAL);
	assert((LPC_IOCON->PIO0_2 & 0x18U) == 0x18U);
	IOPinDisable(0, 2);
	assert((LPC_GPIO->DIR[0] & 4U) == 0U);
	assert((LPC_IOCON->PIO0_2 & 0x418U) == 0U);

	// Whole-port writes must not touch the per-pin byte/word aliases.
	LPC_GPIO->B[1] = 0xAA;
	LPC_GPIO->W[1] = 0x12345678;
	IOPinWritePort(1, 0x87654321);
	assert(LPC_GPIO->PIN[1] == 0x87654321);
	IOPinWrite32Port(1, 0x80000000);
	assert(IOPinReadPort(1) == 0x80000000);
	assert(IOPinRead(1, 31) == 1);
	assert(LPC_GPIO->B[1] == 0xAA && LPC_GPIO->W[1] == 0x12345678);
	IOPinWrite8Port(1, 0xA5);
	assert(LPC_GPIO->CLR[1] == 0x5A && LPC_GPIO->SET[1] == 0xA5);
	IOPinWrite16Port(0, 0x5AA5);
	assert(LPC_GPIO->CLR[0] == 0xA55A && LPC_GPIO->SET[0] == 0x5AA5);
	IOPinSet(1, 31);
	IOPinClear(1, 31);
	IOPinToggle(1, 31);
	assert(LPC_GPIO->SET[1] == 0x80000000 && LPC_GPIO->CLR[1] == 0x80000000);
	assert(LPC_GPIO->NOT[1] == 0x80000000);
	LPC_GPIO->DIR[1] = 1;
	IOPinSetDir(1, 31, IOPINDIR_OUTPUT);
	assert(LPC_GPIO->DIR[1] == 0x80000001);
	IOPinSetDir(1, 31, IOPINDIR_INPUT);
	assert(LPC_GPIO->DIR[1] == 1);

	// Invalid inputs must leave both peripheral blocks and clocks untouched.
	unsigned char iocon[sizeof(LPC_IOCON_Type)];
	unsigned char gpio[sizeof(LPC_GPIO_Type)];
	memcpy(iocon, (void *)LPC_IOCON, sizeof(iocon));
	memcpy(gpio, (void *)LPC_GPIO, sizeof(gpio));
	uint32_t clocks = LPC_SYSCON->SYSAHBCLKCTRL;
	const int bad[][2] = {{-1, 0}, {2, 0}, {0, -1}, {0, 24}, {1, 32}};
	for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
	{
		int port = bad[i][0];
		int pin = bad[i][1];
		Config(port, pin, IOPINOP_GPIO);
		IOPinDisable(port, pin);
		IOPinSetDir(port, pin, IOPINDIR_OUTPUT);
		IOPinSet(port, pin);
		IOPinClear(port, pin);
		IOPinToggle(port, pin);
		assert(IOPinRead(port, pin) == 0);
	}
	Config(0, 2, -1);
	Config(0, 2, IOPINOP_FUNC8);
	IOPinWritePort(-1, 1);
	IOPinWrite8Port(2, 1);
	IOPinWrite16Port(-1, 1);
	IOPinWrite32Port(2, 1);
	assert(IOPinReadPort(2) == 0);
	assert(memcmp(iocon, (void *)LPC_IOCON, sizeof(iocon)) == 0);
	assert(memcmp(gpio, (void *)LPC_GPIO, sizeof(gpio)) == 0);
	assert(LPC_SYSCON->SYSAHBCLKCTRL == clocks);
	assert(!IOPinEnableInterrupt(0, 0, 0, 2, IOPINSENSE_TOGGLE, NULL, NULL));
	assert(IOPinAllocateInterrupt(0, 0, 2, IOPINSENSE_TOGGLE, NULL, NULL) == -1);
	IOPinSetSpeed(0, 2, IOPINSPEED_HIGH);
	IOPinSetStrength(0, 2, IOPINSTRENGTH_STRONG);
	IOPinSetSense(0, 2, IOPINSENSE_TOGGLE);
	puts("LPC11Uxx GPIO regressions passed");
	return 0;
}
