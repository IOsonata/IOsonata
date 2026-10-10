/**-------------------------------------------------------------------------
@file	iopincfg_lpc546xx.cpp

@brief	LPC546xx I/O pin configuration and pin interrupts

		Pin function, resistor and output type are set in IOCON, direction
		in the GPIO block. Pin interrupts use the 8 PINT channels, each
		selecting one pin of port 0 or 1 through INPUTMUX PINTSEL.

		PinOp mapping: IOPINOP_GPIO and IOPINOP_FUNC0 select IOCON function
		0, which is GPIO on the digital pins. IOPINOP_FUNCn selects
		function n.

		The I2C pins (IOCON type I, P3_23 and P3_24) have I2CSLEW in bit 6,
		I2CDRIVE in bit 10 and I2CFILTER in bit 11 instead of slew rate and
		open drain. On them IOPinConfig leaves I2CSLEW at 0, I2C mode, keeps
		I2CDRIVE, and IOPINTYPE_OPENDRAIN sets I2CFILTER, the 50 ns glitch
		filter off, which is also the NXP SDK setting for these pins.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "LPC546xx.h"
#include "iopinctrl.h"

#define LPC546XX_GPIO_PORT_MAX		(sizeof(GPIO->DIR) / sizeof(GPIO->DIR[0]))
#define LPC546XX_GPIO_PIN_MAX		32U
#define LPC546XX_PINT_MAX			(sizeof(INPUTMUX->PINTSEL) / sizeof(INPUTMUX->PINTSEL[0]))

// PINTSEL selects among PIO0_0 to PIO1_31.
#define LPC546XX_PINT_PORT_MAX		2U

// IOCON MODE field values
#define LPC546XX_IOCON_MODE_INACTIVE	0U
#define LPC546XX_IOCON_MODE_PULLDOWN	1U
#define LPC546XX_IOCON_MODE_PULLUP		2U
#define LPC546XX_IOCON_MODE_REPEATER	3U

// GPIO0 to GPIO3 clocks are in AHBCLKCTRL0, GPIO4 and GPIO5 in AHBCLKCTRL2.
#define LPC546XX_GPIO_CLKREG_SPLIT	4

typedef struct {
	IOPinEvtHandler_t Handler;		//!< Event handler, NULL when the channel is free
	void *pContext;					//!< Handler context
} Lpc546xxPintHook_t;

static Lpc546xxPintHook_t s_PintHook[LPC546XX_PINT_MAX];
static const IRQn_Type s_PintIrq[LPC546XX_PINT_MAX] = PINT_IRQS;

static void Lpc546xxGpioClockEnable(int PortNo);
static void Lpc546xxPintEdges(uint32_t Mask, IOPINSENSE Sense);
static void Lpc546xxPintDispatch(int IntNo);

static void Lpc546xxGpioClockEnable(int PortNo)
{
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_IOCON_MASK;

	if (PortNo < LPC546XX_GPIO_CLKREG_SPLIT)
	{
		SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_GPIO0_MASK << (unsigned)PortNo;
	}
	else
	{
		SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_GPIO4_MASK << (unsigned)(PortNo - LPC546XX_GPIO_CLKREG_SPLIT);
	}
}

static void Lpc546xxPintEdges(uint32_t Mask, IOPINSENSE Sense)
{
	PINT->CIENR = Mask;
	PINT->CIENF = Mask;

	if (Sense == IOPINSENSE_HIGH_TRANSITION || Sense == IOPINSENSE_TOGGLE)
	{
		PINT->SIENR = Mask;
	}
	if (Sense == IOPINSENSE_LOW_TRANSITION || Sense == IOPINSENSE_TOGGLE)
	{
		PINT->SIENF = Mask;
	}
}

static void Lpc546xxPintDispatch(int IntNo)
{
	// Edge mode: writing IST clears both the rising and falling detection.
	PINT->IST = 1UL << (unsigned)IntNo;

	if (s_PintHook[IntNo].Handler != NULL)
	{
		s_PintHook[IntNo].Handler(IntNo, s_PintHook[IntNo].pContext);
	}
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if ((unsigned)PortNo >= LPC546XX_GPIO_PORT_MAX || (unsigned)PinNo >= LPC546XX_GPIO_PIN_MAX ||
		PinOp < IOPINOP_GPIO || PinOp > IOPINOP_FUNC15)
	{
		return;
	}

	Lpc546xxGpioClockEnable(PortNo);

	uint32_t func = PinOp == IOPINOP_GPIO ? 0U : (uint32_t)(PinOp - IOPINOP_FUNC0);
	uint32_t mode = LPC546XX_IOCON_MODE_INACTIVE;

	switch (Resistor)
	{
		case IOPINRES_PULLUP:
			mode = LPC546XX_IOCON_MODE_PULLUP;
			break;
		case IOPINRES_PULLDOWN:
			mode = LPC546XX_IOCON_MODE_PULLDOWN;
			break;
		case IOPINRES_FOLLOW:
			mode = LPC546XX_IOCON_MODE_REPEATER;
			break;
		default:
			break;
	}

	// Digital mode with the input filter off, slew rate kept.
	uint32_t iocon = (IOCON->PIO[PortNo][PinNo] & IOCON_PIO_SLEW_MASK) |
					 IOCON_PIO_FUNC(func) | IOCON_PIO_MODE(mode) |
					 IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_FILTEROFF_MASK;

	if (Type == IOPINTYPE_OPENDRAIN)
	{
		iocon |= IOCON_PIO_OD_MASK;
	}

	IOCON->PIO[PortNo][PinNo] = iocon;

	IOPinSetDir(PortNo, PinNo, Dir);
}

/**
 * @brief	Disable I/O pin.
 *
 * Function 0, no resistor, digital input buffer off and direction input,
 * the lowest power state of the pin. IOPinConfig enables it again.
 */
void IOPinDisable(int PortNo, int PinNo)
{
	if ((unsigned)PortNo >= LPC546XX_GPIO_PORT_MAX || (unsigned)PinNo >= LPC546XX_GPIO_PIN_MAX)
	{
		return;
	}

	Lpc546xxGpioClockEnable(PortNo);

	IOCON->PIO[PortNo][PinNo] = 0;
	IOPinSetDir(PortNo, PinNo, IOPINDIR_INPUT);
}

/**
 * @brief	Set I/O pin drive strength.
 *
 * The digital pins have one fixed drive strength. IOPinSetSpeed sets their
 * slew rate.
 */
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	(void)PortNo;
	(void)PinNo;
	(void)Strength;
}

/**
 * @brief	Set I/O pin speed.
 *
 * IOPINSPEED_LOW and IOPINSPEED_MEDIUM select the standard slew rate,
 * IOPINSPEED_HIGH and IOPINSPEED_TURBO the fast one.
 */
void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	if ((unsigned)PortNo >= LPC546XX_GPIO_PORT_MAX || (unsigned)PinNo >= LPC546XX_GPIO_PIN_MAX)
	{
		return;
	}

	if (Speed == IOPINSPEED_HIGH || Speed == IOPINSPEED_TURBO)
	{
		IOCON->PIO[PortNo][PinNo] |= IOCON_PIO_SLEW_MASK;
	}
	else
	{
		IOCON->PIO[PortNo][PinNo] &= ~IOCON_PIO_SLEW_MASK;
	}
}

void IOPinDisableInterrupt(int IntNo)
{
	if ((unsigned)IntNo >= LPC546XX_PINT_MAX || s_PintHook[IntNo].Handler == NULL)
	{
		return;
	}

	uint32_t mask = 1UL << (unsigned)IntNo;

	NVIC_DisableIRQ(s_PintIrq[IntNo]);

	PINT->CIENR = mask;
	PINT->CIENF = mask;
	PINT->IST = mask;

	s_PintHook[IntNo].Handler = NULL;
	s_PintHook[IntNo].pContext = NULL;

	NVIC_ClearPendingIRQ(s_PintIrq[IntNo]);
}

/**
 * @brief	Enable I/O pin sensing interrupt event.
 *
 * IntNo is the PINT channel, 0 to 7. Only port 0 and 1 pins can be
 * selected.
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense,
						  IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if ((unsigned)IntNo >= LPC546XX_PINT_MAX || PortNo >= LPC546XX_PINT_PORT_MAX ||
		PinNo >= LPC546XX_GPIO_PIN_MAX || pEvtCB == NULL ||
		(unsigned)IntPrio >= (1UL << __NVIC_PRIO_BITS) ||
		(Sense != IOPINSENSE_LOW_TRANSITION && Sense != IOPINSENSE_HIGH_TRANSITION &&
		 Sense != IOPINSENSE_TOGGLE) ||
		s_PintHook[IntNo].Handler != NULL)
	{
		return false;
	}

	uint32_t mask = 1UL << (unsigned)IntNo;

	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_PINT_MASK | SYSCON_AHBCLKCTRL_INPUTMUX_MASK;

	PINT->CIENR = mask;
	PINT->CIENF = mask;
	INPUTMUX->PINTSEL[IntNo] = PortNo * LPC546XX_GPIO_PIN_MAX + PinNo;
	PINT->ISEL &= ~mask;
	PINT->IST = mask;

	s_PintHook[IntNo].pContext = pCtx;
	s_PintHook[IntNo].Handler = pEvtCB;

	Lpc546xxPintEdges(mask, Sense);

	NVIC_SetPriority(s_PintIrq[IntNo], (uint32_t)IntPrio);
	NVIC_ClearPendingIRQ(s_PintIrq[IntNo]);
	NVIC_EnableIRQ(s_PintIrq[IntNo]);

	return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	for (int i = 0; i < (int)LPC546XX_PINT_MAX; i++)
	{
		if (s_PintHook[i].Handler == NULL)
		{
			return IOPinEnableInterrupt(i, IntPrio, (uint32_t)PortNo, (uint32_t)PinNo, Sense, pEvtCB, pCtx) ? i : -1;
		}
	}

	return -1;
}

/**
 * @brief	Change the sense of a pin with an interrupt enabled.
 */
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	if ((unsigned)PortNo >= LPC546XX_PINT_PORT_MAX || (unsigned)PinNo >= LPC546XX_GPIO_PIN_MAX)
	{
		return;
	}

	uint32_t sel = (uint32_t)PortNo * LPC546XX_GPIO_PIN_MAX + (uint32_t)PinNo;

	for (unsigned i = 0; i < LPC546XX_PINT_MAX; i++)
	{
		if (s_PintHook[i].Handler != NULL && INPUTMUX->PINTSEL[i] == sel)
		{
			Lpc546xxPintEdges(1UL << i, Sense);
		}
	}
}

extern "C" void PIN_INT0_IRQHandler(void)
{
	Lpc546xxPintDispatch(0);
}

extern "C" void PIN_INT1_IRQHandler(void)
{
	Lpc546xxPintDispatch(1);
}

extern "C" void PIN_INT2_IRQHandler(void)
{
	Lpc546xxPintDispatch(2);
}

extern "C" void PIN_INT3_IRQHandler(void)
{
	Lpc546xxPintDispatch(3);
}

extern "C" void PIN_INT4_IRQHandler(void)
{
	Lpc546xxPintDispatch(4);
}

extern "C" void PIN_INT5_IRQHandler(void)
{
	Lpc546xxPintDispatch(5);
}

extern "C" void PIN_INT6_IRQHandler(void)
{
	Lpc546xxPintDispatch(6);
}

extern "C" void PIN_INT7_IRQHandler(void)
{
	Lpc546xxPintDispatch(7);
}
