/* LPC17xx GPIO access-width regressions using vendor CMSIS declarations.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "iopinctrl.h"
#include "board.h"
#include "ibb_mbedpresso.h"

static const IOPinCfg_t s_UartPins[] = UART_PINS;
static unsigned s_Priority;
static bool s_IrqEnabled;
static unsigned s_CallbackCount[46];
static bool s_CancelSecond;
static bool s_RepeatEdge;

#ifdef __cplusplus
extern "C" {
#endif
void EINT3_IRQHandler(void);
#ifdef __cplusplus
}
#endif

static void WriteStatus(size_t Offset, uint32_t Value)
{
	memcpy((uint8_t *)LPC_GPIOINT + Offset, &Value, sizeof(Value));
}

static void ApplyClear(void)
{
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), LPC_GPIOINT->IO0IntStatR & ~LPC_GPIOINT->IO0IntClr);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatF), LPC_GPIOINT->IO0IntStatF & ~LPC_GPIOINT->IO0IntClr);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO2IntStatR), LPC_GPIOINT->IO2IntStatR & ~LPC_GPIOINT->IO2IntClr);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO2IntStatF), LPC_GPIOINT->IO2IntStatF & ~LPC_GPIOINT->IO2IntClr);
	LPC_GPIOINT->IO0IntClr = 0;
	LPC_GPIOINT->IO2IntClr = 0;
}

void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	assert(Irq == EINT3_IRQn);
	s_Priority = Priority;
}

void NVIC_EnableIRQ(IRQn_Type Irq)
{
	assert(Irq == EINT3_IRQn);
	s_IrqEnabled = true;
	ApplyClear();
}

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert(Irq == EINT3_IRQn);
	s_IrqEnabled = false;
	ApplyClear();
}

static void PinCallback(int IntNo, void *pContext)
{
	assert(pContext == &s_CallbackCount[IntNo]);
	assert((IntNo < 32 ? LPC_GPIOINT->IO0IntClr : LPC_GPIOINT->IO2IntClr) == (1U << (IntNo % 32)));
	ApplyClear();
	s_CallbackCount[IntNo]++;
	if (s_RepeatEdge && IntNo == 0)
	{
		WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), 1);
	}
	if (s_CancelSecond && IntNo == 0)
	{
		IOPinDisableInterrupt(45);
	}
}

static void CheckInterrupts(void)
{
	unsigned char before[sizeof(LPC_GPIOINT_TypeDef)];

	memset((void *)LPC_GPIOINT, 0, sizeof(LPC_GPIOINT_TypeDef));
	assert(IOPinAllocateInterrupt(INT_MAX, 0, 0, IOPINSENSE_TOGGLE, PinCallback, &s_CallbackCount[0]) == 0);
	assert(s_IrqEnabled && s_Priority == 31);
	assert(IOPinAllocateInterrupt(-1, 2, 13, IOPINSENSE_LOW_TRANSITION, PinCallback, &s_CallbackCount[45]) == 45);
	assert(s_Priority == 0);
	memcpy(before, (const void *)LPC_GPIOINT, sizeof(before));
	assert(!IOPinEnableInterrupt(0, 1, 0, 0, IOPINSENSE_TOGGLE, PinCallback, NULL));
	assert(!IOPinEnableInterrupt(1, 1, 0, 0, IOPINSENSE_TOGGLE, PinCallback, NULL));
	assert(IOPinAllocateInterrupt(1, 0, 12, IOPINSENSE_TOGGLE, PinCallback, NULL) == -1);
	assert(IOPinAllocateInterrupt(1, -1, 0, IOPINSENSE_TOGGLE, PinCallback, NULL) == -1);
	assert(IOPinAllocateInterrupt(1, 2, 32, IOPINSENSE_TOGGLE, PinCallback, NULL) == -1);
	assert(IOPinAllocateInterrupt(1, 0, 1, IOPINSENSE_DISABLE, PinCallback, NULL) == -1);
	assert(IOPinAllocateInterrupt(1, 0, 1, IOPINSENSE_TOGGLE, NULL, NULL) == -1);
	IOPinDisableInterrupt(INT_MIN);
	IOPinDisableInterrupt(46);
	assert(memcmp(before, (const void *)LPC_GPIOINT, sizeof(before)) == 0);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), 1);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatF), 1);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO2IntStatF), 1U << 13);
	EINT3_IRQHandler();
	ApplyClear();
	assert(s_CallbackCount[0] == 1 && s_CallbackCount[45] == 1);
	EINT3_IRQHandler();
	assert(s_CallbackCount[0] == 1 && s_CallbackCount[45] == 1);
	// A new edge arriving during the callback survives for the next dispatch.
	s_RepeatEdge = true;
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), 1);
	EINT3_IRQHandler();
	s_RepeatEdge = false;
	assert(LPC_GPIOINT->IO0IntStatR == 1);
	EINT3_IRQHandler();
	assert(s_CallbackCount[0] == 3);
	// Callback cancellation must suppress a later pending pin in this IRQ.
	s_CancelSecond = true;
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), 1);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO2IntStatF), 1U << 13);
	EINT3_IRQHandler();
	s_CancelSecond = false;
	assert(s_CallbackCount[0] == 4 && s_CallbackCount[45] == 1);
	assert(s_IrqEnabled && s_Priority == 31);
	IOPinSetSense(0, 0, IOPINSENSE_DISABLE);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), 1);
	EINT3_IRQHandler();
	assert(s_CallbackCount[0] == 4);
	assert(LPC_GPIOINT->IO0IntClr == 1);
	ApplyClear();
	assert(IOPinAllocateInterrupt(1, 0, 0, IOPINSENSE_TOGGLE, PinCallback, NULL) == -1);
	IOPinSetSense(0, 0, IOPINSENSE_HIGH_TRANSITION);
	WriteStatus(offsetof(LPC_GPIOINT_TypeDef, IO0IntStatR), 1);
	EINT3_IRQHandler();
	assert(s_CallbackCount[0] == 5);
	IOPinDisableInterrupt(0);
	assert(!s_IrqEnabled);
	// Every hardware-supported pin has its own callback slot.
	for (int id = 0; id < 46; id++)
	{
		bool valid = id <= 11 || (id >= 15 && id <= 30) || id >= 32;
		int result = IOPinAllocateInterrupt(7, id < 32 ? 0 : 2, id % 32,
			IOPINSENSE_TOGGLE, PinCallback, &s_CallbackCount[id]);

		assert(result == (valid ? id : -1));
	}
	for (int id = 0; id < 46; id++)
	{
		IOPinDisableInterrupt(id);
	}
	assert(!s_IrqEnabled);
}


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

static void CheckBounds(void)
{
	const int badports[] = {-1, 5, INT_MIN, INT_MAX};
	const int badpins[] = {-1, 32, INT_MIN, INT_MAX};
	unsigned char before[5 * sizeof(LPC_GPIO_TypeDef)];

	memcpy(before, (const void *)LPC_GPIO0, sizeof(before));
	for (unsigned i = 0; i < sizeof(badports) / sizeof(badports[0]); i++)
	{
		int port = badports[i];

		assert(IOPinRead(port, 0) == 0);
		assert(IOPinReadPort(port) == 0);
		IOPinSet(port, 0);
		IOPinClear(port, 0);
		IOPinWritePort(port, UINT32_MAX);
		IOPinWrite8Port(port, UINT8_MAX);
		IOPinWrite16Port(port, UINT16_MAX);
		IOPinWrite32Port(port, UINT32_MAX);
	}
	for (int port = 0; port < 5; port++)
	{
		for (unsigned i = 0; i < sizeof(badpins) / sizeof(badpins[0]); i++)
		{
			assert(IOPinRead(port, badpins[i]) == 0);
			IOPinSet(port, badpins[i]);
			IOPinClear(port, badpins[i]);
		}
	}
	assert(memcmp(before, (const void *)LPC_GPIO0, sizeof(before)) == 0);
	// Boundary pin accesses still reach only their selected set/clear bit.
	for (int port = 0; port < 5; port++)
	{
		for (int pin = 0; pin < 32; pin++)
		{
			uint32_t bit = 1U << pin;

			IOPinSet(port, pin);
			assert(LPC_GPIO0[port].FIOSET == bit);
			IOPinClear(port, pin);
			assert(LPC_GPIO0[port].FIOCLR == bit);
			IOPinWritePort(port, bit);
			assert(IOPinRead(port, pin) == 1);
			IOPinWritePort(port, ~bit);
			assert(IOPinRead(port, pin) == 0);
		}
	}
}

static void CheckI2cPads(void)
{
	const int ops[] = {IOPINOP_GPIO, IOPINOP_FUNC0, IOPINOP_FUNC1, IOPINOP_FUNC2};
	const IOPINRES pulls[] = {IOPINRES_NONE, IOPINRES_PULLUP, IOPINRES_PULLDOWN, IOPINRES_FOLLOW};
	const IOPINTYPE types[] = {IOPINTYPE_NORMAL, IOPINTYPE_OPENDRAIN};
	LPC_PINCON_TypeDef expected;

	for (int pin = 27; pin <= 28; pin++)
	{
		uint32_t padbit = pin == 27 ? 2U : 8U;
		unsigned shift = (unsigned)(pin - 16) * 2U;

		for (unsigned initial = 0; initial < 16; initial++)
		{
			for (unsigned op = 0; op < 4; op++)
			{
				for (unsigned r = 0; r < 4; r++)
				{
					for (unsigned type = 0; type < 2; type++)
					{
						memset((void *)LPC_PINCON, 0xA5, sizeof(expected));
						LPC_PINCON->I2CPADCFG = 0xA5A50000U | initial;
						memcpy(&expected, (const void *)LPC_PINCON, sizeof(expected));
						expected.PINSEL1 = (expected.PINSEL1 & ~(3U << shift)) |
							((op < 2 ? 0U : op - 1U) << shift);
						if (op < 2)
						{
							expected.I2CPADCFG |= padbit;
						}
						else
						{
							expected.I2CPADCFG &= ~padbit;
						}
						IOPinConfig(0, pin, ops[op], IOPINDIR_OUTPUT, pulls[r], types[type]);
						assert(memcmp(&expected, (const void *)LPC_PINCON, sizeof(expected)) == 0);
						assert((LPC_GPIO0->FIODIR & (1U << pin)) != 0);
						// Cleanup restores GPIO pad mode without writing reserved controls.
						expected.PINSEL1 &= ~(3U << shift);
						expected.I2CPADCFG |= padbit;
						IOPinDisable(0, pin);
						assert(memcmp(&expected, (const void *)LPC_PINCON, sizeof(expected)) == 0);
						assert((LPC_GPIO0->FIODIR & (1U << pin)) == 0);
					}
				}
			}
		}
	}
}

static void CheckMux(void)
{
	const int pins[][2] = {{0, 0}, {0, 15}, {0, 16}, {0, 29}, {0, 30},
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
		int usbpad = port == 0 && (pin == 29 || pin == 30);
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
			assert(*mode == (usbpad ? 0x5A5A5A5AU :
				((0x5A5A5A5AU & ~mask) | (modes[r] << shift))));
			assert(*od == (1U << pin));
			assert(LPC_GPIO0[port].FIODIR == (1U << pin));
			IOPinConfig(port, pin, IOPINOP_GPIO, IOPINDIR_INPUT, pulls[r], IOPINTYPE_NORMAL);
			assert(*od == 0 && LPC_GPIO0[port].FIODIR == 0);
		}
		// Disable a peripheral/output pin without altering its neighbors.
		*sel = UINT32_MAX;
		*mode = 0;
		*od = UINT32_MAX;
		LPC_GPIO0[port].FIODIR = UINT32_MAX;
		IOPinDisable(port, pin);
		assert(*sel == (UINT32_MAX & ~mask));
		assert(*mode == (usbpad ? 0U : (2U << shift)));
		assert(*od == (UINT32_MAX & ~(1U << pin)));
		assert(LPC_GPIO0[port].FIODIR == (UINT32_MAX & ~(1U << pin)));
	}
	// USB pull requests must preserve the entire reserved PINMODE1 region.
	for (int pin = 29; pin <= 30; pin++)
	{
		for (unsigned r = 0; r < 4; r++)
		{
			LPC_PINCON_TypeDef expected;

			memset((void *)LPC_PINCON, 0x5A, sizeof(expected));
			memcpy(&expected, (const void *)LPC_PINCON, sizeof(expected));
			expected.PINSEL1 &= ~(3U << ((pin - 16) * 2));
			expected.PINMODE_OD0 |= 1U << pin;
			IOPinConfig(0, pin, IOPINOP_GPIO, IOPINDIR_OUTPUT, pulls[r], IOPINTYPE_OPENDRAIN);
			assert(memcmp(&expected, (const void *)LPC_PINCON, sizeof(expected)) == 0);
			expected.PINMODE_OD0 &= ~(1U << pin);
			IOPinDisable(0, pin);
			assert(memcmp(&expected, (const void *)LPC_PINCON, sizeof(expected)) == 0);
		}
	}

	IOPinCfg(s_UartPins, 2);
	assert(((LPC_PINCON->PINSEL0 >> 4) & 15U) == 5U);
	IOPinDis(s_UartPins, 2);
	assert(((LPC_PINCON->PINSEL0 >> 4) & 15U) == 0);
	assert(((LPC_PINCON->PINMODE0 >> 4) & 15U) == 10U);
	assert((LPC_GPIO0->FIODIR & 12U) == 0);
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
	// Optional unsupported pad controls still link from both C and C++.
	IOPinSetStrength(0, 27, IOPINSTRENGTH_STRONG);
	IOPinSetSpeed(0, 28, IOPINSPEED_TURBO);
	assert(memcmp(saved, (const void *)LPC_PINCON, sizeof(saved)) == 0);
	IOPinDisable(-1, 0);
	IOPinDisable(5, 0);
	IOPinDisable(0, -1);
	IOPinDisable(0, 32);
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
	CheckInterrupts();
	CheckMux();
	CheckI2cPads();
	CheckBounds();

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
	LPC_GPIO1->FIOSET = 0;
	IOPinToggle(1, 31);
	assert(LPC_GPIO1->FIOSET == 0x80000000U);
	LPC_GPIO1->FIOPIN |= LPC_GPIO1->FIOSET;
	assert(IOPinReadPort(1) == 0x92345678U);
	assert(IOPinRead(1, 31) == 1);
	LPC_GPIO1->FIOCLR = 0;
	IOPinToggle(1, 31);
	assert(LPC_GPIO1->FIOCLR == 0x80000000U);
	LPC_GPIO1->FIOPIN &= ~LPC_GPIO1->FIOCLR;
	assert(IOPinReadPort(1) == 0x12345678U);
	assert(IOPinRead(1, 31) == 0);
	puts("LPC17xx GPIO access-width regressions passed");
	return 0;
}
