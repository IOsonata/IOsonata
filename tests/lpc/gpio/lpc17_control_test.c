/* LPC17xx GPIO access-width regressions using vendor CMSIS declarations.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "iopinctrl.h"
#include "board.h"
#include "ibb_mbedpresso.h"

static const IOPinCfg_t s_UartPins[] = UART_PINS;

static void CheckSense(void)
{
	unsigned char expected[sizeof(LPC_GPIOINT_TypeDef)];
	const IOPINSENSE senses[] = {IOPINSENSE_DISABLE, IOPINSENSE_HIGH_TRANSITION,
		IOPINSENSE_LOW_TRANSITION, IOPINSENSE_TOGGLE};

	for (int port = -1; port <= 5; port++)
	{
		for (int pin = -1; pin <= 32; pin++)
		{
			bool valid = (port == 0 && ((pin >= 0 && pin <= 11) || (pin >= 15 && pin <= 30))) ||
				(port == 2 && pin >= 0 && pin <= 13);

			for (unsigned sense = 0; sense < 4; sense++)
			{
				memset((void *)LPC_GPIOINT, 0xA5, sizeof(LPC_GPIOINT_TypeDef));
				if (valid)
				{
					volatile uint32_t *rise = port == 0 ? &LPC_GPIOINT->IO0IntEnR : &LPC_GPIOINT->IO2IntEnR;
					volatile uint32_t *fall = port == 0 ? &LPC_GPIOINT->IO0IntEnF : &LPC_GPIOINT->IO2IntEnF;
					uint32_t mask = 1U << pin;

					*rise = (0xA5A5A5A5U & ~mask) | ((sense == 1 || sense == 3) ? mask : 0);
					*fall = (0xA5A5A5A5U & ~mask) | ((sense == 2 || sense == 3) ? mask : 0);
				}
				memcpy(expected, (const void *)LPC_GPIOINT, sizeof(expected));
				memset((void *)LPC_GPIOINT, 0xA5, sizeof(LPC_GPIOINT_TypeDef));
				IOPinSetSense(port, pin, senses[sense]);
				assert(memcmp(expected, (const void *)LPC_GPIOINT, sizeof(expected)) == 0);
			}
		}
	}
	memcpy(expected, (const void *)LPC_GPIOINT, sizeof(expected));
	IOPinSetSense(0, 0, (IOPINSENSE)99);
	assert(memcmp(expected, (const void *)LPC_GPIOINT, sizeof(expected)) == 0);
}

static void CheckMux(void)
{
	const int pins[][2] = {{0, 0}, {0, 15}, {0, 16}, {0, 30},
		{1, 0}, {1, 15}, {1, 16}, {1, 31}, {2, 0}, {2, 13},
		{3, 25}, {3, 26}, {4, 28}, {4, 29}};
	const IOPINRES pulls[] = {IOPINRES_NONE, IOPINRES_PULLUP, IOPINRES_PULLDOWN, IOPINRES_FOLLOW};
	const uint32_t modes[] = {2, 0, 3, 1};
	unsigned char saved[sizeof(LPC_PINCON_TypeDef)];

	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
	{
		int port = pins[i][0];
		int pin = pins[i][1];
		unsigned index = (unsigned)port * 2U + (unsigned)pin / 16U;
		unsigned shift = ((unsigned)pin % 16U) * 2U;
		uint32_t mask = 3U << shift;
		volatile uint32_t *sel = (volatile uint32_t *)(uintptr_t)(LPC_PINCON_BASE + index * 4U);
		volatile uint32_t *mode = (volatile uint32_t *)(uintptr_t)(LPC_PINCON_BASE + 0x40U + index * 4U);
		volatile uint32_t *od = (volatile uint32_t *)(uintptr_t)(LPC_PINCON_BASE + 0x68U + (unsigned)port * 4U);

		for (int op = IOPINOP_GPIO; op <= IOPINOP_FUNC3; op++)
		{
			uint32_t expected = op == IOPINOP_GPIO ? 0U : (uint32_t)(op - IOPINOP_FUNC0);

			*sel = 0xA5A5A5A5U;
			IOPinConfig(port, pin, op, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
			assert(*sel == ((0xA5A5A5A5U & ~mask) | (expected << shift)));
		}
		for (unsigned r = 0; r < 4; r++)
		{
			*mode = 0x5A5A5A5AU;
			*od = 0;
			LPC_GPIO0[port].FIODIR = 0;
			IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, pulls[r], IOPINTYPE_OPENDRAIN);
			assert(*mode == ((0x5A5A5A5AU & ~mask) | (modes[r] << shift)));
			assert(*od == (1U << pin));
			assert(LPC_GPIO0[port].FIODIR == (1U << pin));
			IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_INPUT, pulls[r], IOPINTYPE_NORMAL);
			assert(*od == 0 && LPC_GPIO0[port].FIODIR == 0);
		}
	}
	IOPinCfg(s_UartPins, 2);
	assert(((LPC_PINCON->PINSEL0 >> 4) & 15U) == 5U);
	IOPinConfig(0, 27, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN);
	assert(((LPC_PINCON->PINSEL1 >> 22) & 3U) == 1U);
	IOPinConfig(0, 0, IOPINOP_FUNC3, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN);
	assert((LPC_PINCON->PINSEL0 & 3U) == 3U);
	IOPinConfig(0, 10, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN);
	assert(((LPC_PINCON->PINSEL0 >> 20) & 3U) == 2U);
	assert(MBEDPRESSO_SRPWM_PWM1_OP == IOPINOP_FUNC1);
	assert(MBEDPRESSO_SRPWM_PWM2_OP == IOPINOP_FUNC1);
	assert(MBEDPRESSO_SRPWM_PWM3_OP == IOPINOP_FUNC1);
	memcpy(saved, (const void *)LPC_PINCON, sizeof(saved));
	IOPinConfig(-1, 0, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(5, 0, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 32, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 0, -1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 0, IOPINOP_FUNC4, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	assert(memcmp(saved, (const void *)LPC_PINCON, sizeof(saved)) == 0);
}

int main(void)
{
	assert(mmap((void *)LPC_GPIO_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_GPIO_BASE);
	assert(mmap((void *)LPC_PINCON_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_PINCON_BASE);
	assert(mmap((void *)(LPC_GPIOINT_BASE & ~0xFFFUL), 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)(LPC_GPIOINT_BASE & ~0xFFFUL));
	CheckSense();
	CheckMux();

	// Narrow writes must keep every supplied bit and preserve higher pins.
	for (int port = 0; port < 5; port++)
	{
		IOPinWrite32Port(port, 0xA55A1234U);
		IOPinWrite8Port(port, 0xBCU);
		assert(IOPinReadPort(port) == 0xA55A12BCU);
		IOPinWrite16Port(port, 0xCDEFU);
		assert(IOPinReadPort(port) == 0xA55ACDEFU);
		IOPinWrite16Port(port, 0);
		assert(IOPinReadPort(port) == 0xA55A0000U);
		IOPinWrite16Port(port, UINT16_MAX);
		assert(IOPinReadPort(port) == 0xA55AFFFFU);
		IOPinWritePort(port, 0x12345678U);
		assert(IOPinReadPort(port) == 0x12345678U);
	}

	// P1.31 exercises the sign bit in C as well as C++ builds.
	LPC_GPIO1->FIODIR = 0x12345678U;
	IOPinSetDir(1, 31, IOPINDIR_OUTPUT);
	assert(LPC_GPIO1->FIODIR == 0x92345678U);
	IOPinSetDir(1, 31, IOPINDIR_INPUT);
	assert(LPC_GPIO1->FIODIR == 0x12345678U);
	IOPinSet(1, 31);
	assert(LPC_GPIO1->FIOSET == 0x80000000U);
	IOPinClear(1, 31);
	assert(LPC_GPIO1->FIOCLR == 0x80000000U);
	IOPinWritePort(1, 0x12345678U);
	IOPinToggle(1, 31);
	assert(IOPinReadPort(1) == 0x92345678U);
	assert(IOPinRead(1, 31) == 1);
	IOPinToggle(1, 31);
	assert(IOPinReadPort(1) == 0x12345678U);
	assert(IOPinRead(1, 31) == 0);
	puts("LPC17xx GPIO access-width regressions passed");
	return 0;
}
