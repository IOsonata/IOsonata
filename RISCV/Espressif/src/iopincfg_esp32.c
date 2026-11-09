/**-------------------------------------------------------------------------
@file   iopincfg_esp32.c

@brief  I/O pin configuration for Espressif ESP32 RISC-V targets.

This implementation uses the target-aware GPIO/IO_MUX register helpers from
esp32xx_gpio.h.  The ESP32-C3/C5/C6 GPIO blocks are not offset-identical, so
active register access must go through the raw offset macros instead of the
common ESP32_GPIO_Type structure.

Implemented scope:
  - GPIO input/output/bidirectional pad setup
  - pull-up / pull-down
  - open-drain output
  - drive strength
  - simple GPIO output routing through the GPIO matrix
  - peripheral input/output GPIO-matrix helper functions
  - GPIO edge interrupt setup with per-pin callbacks

@author Hoang Nguyen Hoan
@date   Mar. 2026

@license

MIT

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

#include "coredev/iopincfg.h"

#include "esp32xx_gpio.h"
#include "esp32xx_irq.h"


/* Register offsets, field masks, GPIO matrix constants, and interrupt-matrix
 * selection macros are defined in esp32xx_gpio.h.  Keep this source file as
 * implementation only. */

typedef struct __ESP32_GPIO_ISR_SLOT {
	IOPinEvtHandler_t Handler;
	void *Context;
	int IntNo;
} ESP32_GPIO_ISR_SLOT;

static volatile ESP32_GPIO_ISR_SLOT s_GpioIsr[ESP32_GPIO_PIN_COUNT];
static volatile uint32_t s_GpioIntEnableMask;
static volatile bool s_GpioIntInstalled;

volatile uint32_t g_Esp32GpioIrqCount;
volatile uint32_t g_Esp32GpioLastPending;
volatile uint32_t g_Esp32GpioUnhandledPending;

static inline bool ESP32IsValidGpioPin(int PinNo)
{
	return (PinNo >= 0) && ((uint32_t)PinNo < ESP32_GPIO_PIN_COUNT);
}

static inline uint32_t ESP32GpioMask(void)
{
	if (ESP32_GPIO_PIN_COUNT >= 32U)
	{
		return 0xFFFFFFFFUL;
	}
	return (1UL << ESP32_GPIO_PIN_COUNT) - 1UL;
}

static inline uint32_t ESP32PinMask(uint32_t PinNo)
{
	return 1UL << PinNo;
}

static inline uint32_t ESP32PadDriveFromStrength(IOPINSTRENGTH Strength)
{
	return (Strength == IOPINSTRENGTH_STRONG) ? 3UL : 2UL;
}

/* PinOp interpretation on ESP32:
 *
 *   IOPINOP_GPIO (0)        : plain GPIO, controlled via GPIO_OUT/IN_REG
 *   IOPINOP_FUNCn (1..N)    : matrix-routed peripheral signal index n
 *
 * Both cases use IOMUX MCU_SEL = 1 (GPIO matrix mode).  The IOMUX direct
 * path option (MCU_SEL = 0 or 2..7 on chips that support it) is
 * deliberately not exposed here -- matrix routing covers every signal
 * with one fewer corner case.  If a user later needs the direct path
 * for a high-baud-rate signal, that's an orthogonal flag, not a
 * different MCU_SEL value here.
 */
static inline uint32_t ESP32IomuxMcuSelFromPinOp(int PinOp)
{
	(void)PinOp;
	return 1U;
}

static uint32_t ESP32SenseToIntType(IOPINSENSE Sense)
{
	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			return ESP32_GPIO_INT_FALLING_EDGE;

		case IOPINSENSE_HIGH_TRANSITION:
			return ESP32_GPIO_INT_RISING_EDGE;

		case IOPINSENSE_TOGGLE:
			return ESP32_GPIO_INT_ANY_EDGE;

		case IOPINSENSE_DISABLE:
		default:
			return ESP32_GPIO_INT_DISABLE;
	}
}

static void ESP32GpioSetPinRegField(uint32_t PinNo, uint32_t Mask, uint32_t Value)
{
	uint32_t reg;

	reg = ESP32_GPIO_PIN_REG32(PinNo);
	reg &= ~Mask;
	reg |= Value & Mask;
	ESP32_GPIO_PIN_REG32(PinNo) = reg;
}

static void ESP32GpioSetInterruptType(uint32_t PinNo, IOPINSENSE Sense)
{
	uint32_t intType;

	intType = ESP32SenseToIntType(Sense);
	ESP32GpioSetPinRegField(PinNo,
							ESP32_GPIO_PIN_INT_TYPE_Msk,
							intType << ESP32_GPIO_PIN_INT_TYPE_Pos);
}

static void ESP32GpioEnablePinInterrupt(uint32_t PinNo)
{
	ESP32GpioSetPinRegField(PinNo,
							ESP32_GPIO_PIN_INT_ENA_Msk,
							ESP32_GPIO_PIN_INT_ENA_CPU_Msk);
}

static void ESP32GpioDisablePinInterrupt(uint32_t PinNo)
{
	ESP32GpioSetPinRegField(PinNo,
							ESP32_GPIO_PIN_INT_ENA_Msk | ESP32_GPIO_PIN_INT_TYPE_Msk,
							0U);
	ESP32_GPIO_STATUS_W1TC_REG32 = ESP32PinMask(PinNo);
}

__attribute__((section(".iram.text"), used))
void ESP32_GPIO_IRQHandler(void)
{
	uint32_t pending;
	uint32_t pin;
	uint32_t mask;
	IOPinEvtHandler_t cb;

	pending = ESP32_GPIO_STATUS_REG32 & s_GpioIntEnableMask & ESP32GpioMask();

	if (pending == 0U)
	{
		return;
	}

	g_Esp32GpioIrqCount++;
	g_Esp32GpioLastPending = pending;

	ESP32_GPIO_STATUS_W1TC_REG32 = pending;

	for (pin = 0U; pin < ESP32_GPIO_PIN_COUNT; pin++)
	{
		mask = 1UL << pin;
		if ((pending & mask) == 0U)
		{
			continue;
		}

		cb = s_GpioIsr[pin].Handler;
		if (cb != 0)
		{
			cb(s_GpioIsr[pin].IntNo, s_GpioIsr[pin].Context);
		}
		else
		{
			g_Esp32GpioUnhandledPending |= mask;
		}
	}
}

static bool ESP32InstallGpioInterrupt(int IntPrio)
{
#if defined(ESP32_GPIO_INTR_SOURCE_ID)
	uint32_t prio = (IntPrio > 0) ? (uint32_t)IntPrio : ESP32_GPIO_INTR_CPU_INT_PRIO;

	if (!Esp32EnableSourceIrq(ESP32_GPIO_INTR_SOURCE_ID,
							  ESP32_GPIO_INTR_CPU_INT_ID,
							  prio,
							  ESP32_GPIO_IRQHandler))
	{
		return false;
	}

	s_GpioIntInstalled = true;
	return true;
#else
	(void)IntPrio;
	return false;
#endif
}

static bool ESP32GPIOConnectInput(uint32_t SignalIdx, int PinNo, bool Invert)
{
	uint32_t cfg;

	if (SignalIdx >= ESP32_GPIO_FUNC_IN_COUNT)
	{
		return false;
	}

	cfg = ESP32_GPIO_SIG_IN_SEL_Msk;

	if (PinNo >= 0)
	{
		if (!ESP32IsValidGpioPin(PinNo))
		{
			return false;
		}
		cfg |= ((uint32_t)PinNo << ESP32_GPIO_FUNC_IN_SEL_Pos) & ESP32_GPIO_FUNC_IN_SEL_Msk;
	}
	else
	{
		cfg |= (ESP32_GPIO_FUNC_IN_CONST_LOW << ESP32_GPIO_FUNC_IN_SEL_Pos) & ESP32_GPIO_FUNC_IN_SEL_Msk;
	}

	if (Invert)
	{
		cfg |= ESP32_GPIO_FUNC_IN_INV_SEL_Msk;
	}

	ESP32_GPIO_FUNC_IN_REG32(SignalIdx) = cfg;
	return true;
}

static bool ESP32GPIOConnectOutput(int PinNo, uint32_t SignalIdx, bool Invert, bool OenInvert)
{
	uint32_t cfg;

	if (!ESP32IsValidGpioPin(PinNo))
	{
		return false;
	}

	cfg = (SignalIdx << ESP32_GPIO_FUNC_OUT_SEL_Pos) & ESP32_GPIO_FUNC_OUT_SEL_Msk;

	if (Invert)
	{
		cfg |= ESP32_GPIO_FUNC_OUT_INV_SEL_Msk;
	}
	if (OenInvert)
	{
		cfg |= ESP32_GPIO_FUNC_OEN_INV_SEL_Msk;
	}

	ESP32_GPIO_FUNC_OUT_REG32((uint32_t)PinNo) = cfg;
	return true;
}

static void ESP32GPIODisconnectOutput(int PinNo)
{
	if (ESP32IsValidGpioPin(PinNo))
	{
		ESP32_GPIO_FUNC_OUT_REG32((uint32_t)PinNo) = ESP32_GPIO_FUNC_OUT_GPIO_VALUE;
	}
}

void IOPinConfig(int PortNo, int PinNo, int PinOp,
				 IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	uint32_t pin;
	uint32_t mask;
	uint32_t mcuSel;
	uint32_t iomux;
	uint32_t pincfg;
	uint32_t sig;

	(void)PortNo;

	if (!ESP32IsValidGpioPin(PinNo))
	{
		return;
	}

	pin = (uint32_t)PinNo;
	mask = ESP32PinMask(pin);
	mcuSel = ESP32IomuxMcuSelFromPinOp(PinOp);

	iomux = ESP32_IOMUX_PAD_REG(pin);
	iomux &= ~(ESP32_IOMUX_MCU_SEL_Msk |
			   ESP32_IOMUX_FUN_DRV_Msk |
			   (1UL << ESP32_IOMUX_FUN_IE_Pos) |
			   (1UL << ESP32_IOMUX_FUN_WPU_Pos) |
			   (1UL << ESP32_IOMUX_FUN_WPD_Pos));

	iomux |= (mcuSel << ESP32_IOMUX_MCU_SEL_Pos) & ESP32_IOMUX_MCU_SEL_Msk;
	iomux |= ESP32PadDriveFromStrength(IOPINSTRENGTH_REGULAR) << ESP32_IOMUX_FUN_DRV_Pos;

	if ((Dir == IOPINDIR_INPUT) || (Dir == IOPINDIR_BI))
	{
		iomux |= (1UL << ESP32_IOMUX_FUN_IE_Pos);
	}

	if (Resistor == IOPINRES_PULLUP)
	{
		iomux |= (1UL << ESP32_IOMUX_FUN_WPU_Pos);
	}
	else if (Resistor == IOPINRES_PULLDOWN)
	{
		iomux |= (1UL << ESP32_IOMUX_FUN_WPD_Pos);
	}

	// Force pad output disabled (hi-Z) BEFORE touching IOMUX or matrix
	// routing.  If we change MCU_SEL from "IOMUX direct" (e.g. ROM left
	// GPIO 21 driving U0TXD) to "matrix mode" while GPIO_ENABLE was
	// already set, the pad would briefly drive whatever FUNC_OUT_SEL
	// pointed to (typically SIG_GPIO_OUT_IDX -> GPIO_OUT_REG = 0).
	// That LOW pulse is decoded by the receiver as a stream of 0x00
	// start bits.  Hi-Z holds the line at its idle level (UART idle =
	// HIGH via external pull-up or the receiver's bias), avoiding any
	// spurious bytes.
	ESP32_GPIO_ENABLE_W1TC_REG32 = mask;

	ESP32_IOMUX_PAD_REG(pin) = iomux;

	// Matrix routing (must be in place before GPIO_ENABLE is asserted):
	//
	//   PinOp == IOPINOP_GPIO    : pin reads/writes through GPIO_OUT/IN.
	//                              FUNC_OUT_SEL = SIG_GPIO_OUT_IDX.
	//   PinOp >= IOPINOP_FUNC0   : matrix signal index = PinOp - IOPINOP_FUNC0.
	//                              Output side: FUNC_OUT_SEL[pin] = sig.
	//                              Input  side: FUNC_IN_SEL[sig]  = pin.
	//
	// U0RXD_IN_IDX and U0TXD_OUT_IDX share the same numeric value (6 on
	// all C-series chips); PinDir disambiguates the side.
	if (PinOp >= IOPINOP_FUNC0)
	{
		sig = (uint32_t)PinOp - (uint32_t)IOPINOP_FUNC0;
		if ((Dir == IOPINDIR_OUTPUT) || (Dir == IOPINDIR_BI))
		{
			ESP32GPIOConnectOutput(PinNo, sig, false, false);
		}
		if ((Dir == IOPINDIR_INPUT) || (Dir == IOPINDIR_BI))
		{
			ESP32GPIOConnectInput(sig, PinNo, false);
		}
	}
	else
	{
		// Plain GPIO: pin output is driven by GPIO_OUT_REG.
		ESP32_GPIO_FUNC_OUT_REG32(pin) = ESP32_GPIO_FUNC_OUT_GPIO_VALUE;
	}

	pincfg = ESP32_GPIO_PIN_REG32(pin);
	if (Type == IOPINTYPE_OPENDRAIN)
	{
		pincfg |= ESP32_GPIO_PIN_PAD_DRIVER_Msk;
	}
	else
	{
		pincfg &= ~ESP32_GPIO_PIN_PAD_DRIVER_Msk;
	}
	ESP32_GPIO_PIN_REG32(pin) = pincfg;

	// Output enable goes last.  By this point IOMUX, FUNC_OUT_SEL, and
	// PAD_DRIVER are all consistent with the requested direction and
	// matrix routing.  For input-only pins the pad stays hi-Z.
	if ((Dir == IOPINDIR_OUTPUT) || (Dir == IOPINDIR_BI))
	{
		ESP32_GPIO_ENABLE_W1TS_REG32 = mask;
	}
}

// Call with CPU interrupts masked. Hardware remains pin-indexed; the public
// interrupt number stored in the slot may differ from the physical pin.
static void ESP32ReleasePinInterrupt(uint32_t Pin)
{
	ESP32GpioDisablePinInterrupt(Pin);
	s_GpioIntEnableMask &= ~ESP32PinMask(Pin);
	s_GpioIsr[Pin].Handler = 0;
	s_GpioIsr[Pin].Context = 0;
	s_GpioIsr[Pin].IntNo = 0;
}

void IOPinDisable(int PortNo, int PinNo)
{
	uint32_t pin;
	uintptr_t state;

	if (PortNo != 0 || !ESP32IsValidGpioPin(PinNo))
	{
		return;
	}
	pin = (uint32_t)PinNo;
	state = DisableInterrupt();
	ESP32_GPIO_ENABLE_W1TC_REG32 = ESP32PinMask(pin);
	ESP32ReleasePinInterrupt(pin);
	ESP32_GPIO_FUNC_OUT_REG32(pin) = ESP32_GPIO_FUNC_OUT_GPIO_VALUE;
	ESP32_IOMUX_PAD_REG(pin) = 0U;
	EnableInterrupt(state);
}

void IOPinDisableInterrupt(int IntNo)
{
	uint32_t pin;
	uintptr_t state;

	if (IntNo < 0)
	{
		return;
	}
	state = DisableInterrupt();
	for (pin = 0; pin < ESP32_GPIO_PIN_COUNT; pin++)
	{
		if (s_GpioIsr[pin].Handler != 0 && s_GpioIsr[pin].IntNo == IntNo)
		{
			ESP32ReleasePinInterrupt(pin);
			break;
		}
	}
	EnableInterrupt(state);
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio,
						  uint32_t PortNo, uint32_t PinNo,
						  IOPINSENSE Sense,
						  IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	uint32_t pin;
	uint32_t mask;
	uintptr_t state;
	int intNo;

	if (PortNo != 0 || PinNo >= ESP32_GPIO_PIN_COUNT)
	{
		return false;
	}
	if (Sense == IOPINSENSE_DISABLE)
	{
		state = DisableInterrupt();
		ESP32ReleasePinInterrupt(PinNo);
		EnableInterrupt(state);
		return true;
	}
	if (pEvtCB == 0 || ESP32SenseToIntType(Sense) == ESP32_GPIO_INT_DISABLE)
	{
		return false;
	}
	if (!s_GpioIntInstalled && !ESP32InstallGpioInterrupt(IntPrio))
	{
		return false;
	}
	intNo = IntNo >= 0 ? IntNo : (int)PinNo;
	state = DisableInterrupt();
	for (pin = 0; pin < ESP32_GPIO_PIN_COUNT; pin++)
	{
		if (pin != PinNo && s_GpioIsr[pin].Handler != 0 &&
			s_GpioIsr[pin].IntNo == intNo)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	mask = ESP32PinMask(PinNo);
	ESP32GpioDisablePinInterrupt(PinNo);
	s_GpioIsr[PinNo].Handler = pEvtCB;
	s_GpioIsr[PinNo].Context = pCtx;
	s_GpioIsr[PinNo].IntNo = intNo;
	s_GpioIntEnableMask |= mask;
	ESP32GpioSetInterruptType(PinNo, Sense);
	ESP32GpioEnablePinInterrupt(PinNo);
	EnableInterrupt(state);
	return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo,
						   IOPINSENSE Sense,
						   IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if (PortNo != 0 || !ESP32IsValidGpioPin(PinNo))
	{
		return -1;
	}
	if (!IOPinEnableInterrupt(PinNo, IntPrio, (uint32_t)PortNo, (uint32_t)PinNo,
							  Sense, pEvtCB, pCtx))
	{
		return -1;
	}
	return PinNo;
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	(void)PortNo;

	if (!ESP32IsValidGpioPin(PinNo))
	{
		return;
	}

	ESP32GpioSetInterruptType((uint32_t)PinNo, Sense);
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	uint32_t pin;
	uint32_t v;

	(void)PortNo;

	if (!ESP32IsValidGpioPin(PinNo))
	{
		return;
	}

	pin = (uint32_t)PinNo;
	v = ESP32_IOMUX_PAD_REG(pin) & ~ESP32_IOMUX_FUN_DRV_Msk;
	v |= ESP32PadDriveFromStrength(Strength) << ESP32_IOMUX_FUN_DRV_Pos;
	ESP32_IOMUX_PAD_REG(pin) = v;
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	IOPINSTRENGTH strength;

	(void)PortNo;

	if (!ESP32IsValidGpioPin(PinNo))
	{
		return;
	}

	strength = IOPINSTRENGTH_REGULAR;

	if ((Speed == IOPINSPEED_HIGH) || (Speed == IOPINSPEED_TURBO))
	{
		strength = IOPINSTRENGTH_STRONG;
	}

	IOPinSetStrength(PortNo, PinNo, strength);
}

bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	// LP-core pad routing is not implemented by this port.
#if defined(ESP32C3)
	return CoreNo == 0 && PortNo == 0 && ESP32IsValidGpioPin(PinNo);
#else
	(void)PortNo;
	(void)PinNo;
	(void)CoreNo;
	return false;
#endif
}
