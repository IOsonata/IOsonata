/** Thingy:52 production PCA20020 (v1.x) board helpers. SPDX-License-Identifier: MIT */
#include "thingy52_board.h"
#include "iopinctrl.h"
#include "idelay.h"

static const uint8_t s_LedPins[] = {
	THINGY52_SX_LIGHTWELL_RED, THINGY52_SX_LIGHTWELL_GREEN, THINGY52_SX_LIGHTWELL_BLUE,
	THINGY52_SX_COLOR_RED, THINGY52_SX_COLOR_GREEN, THINGY52_SX_COLOR_BLUE
};
static const uint16_t s_LedMask = (1U << THINGY52_SX_LIGHTWELL_RED) |
	(1U << THINGY52_SX_LIGHTWELL_GREEN) | (1U << THINGY52_SX_LIGHTWELL_BLUE) |
	(1U << THINGY52_SX_COLOR_RED) | (1U << THINGY52_SX_COLOR_GREEN) |
	(1U << THINGY52_SX_COLOR_BLUE);
static const uint16_t s_GasPower = 1U << THINGY52_SX_GAS_POWER;
static const uint16_t s_GasReset = 1U << THINGY52_SX_GAS_RESET;
static const uint16_t s_GasWake = 1U << THINGY52_SX_GAS_WAKE;

void Thingy52PowerInit()
{
	// Hold the speaker amplifier off. Do not drive any expansion connector pins.
	IOPinClear(THINGY52_SPEAKER_POWER_PORT, THINGY52_SPEAKER_POWER_PIN);
	IOPinConfig(THINGY52_SPEAKER_POWER_PORT, THINGY52_SPEAKER_POWER_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(THINGY52_VDD_PORT, THINGY52_VDD_PIN);
	IOPinConfig(THINGY52_VDD_PORT, THINGY52_VDD_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	msDelay(5);
	IOPinSet(THINGY52_SX_RESET_PORT, THINGY52_SX_RESET_PIN);
	IOPinConfig(THINGY52_SX_RESET_PORT, THINGY52_SX_RESET_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	msDelay(2);
	IOPinConfig(THINGY52_BUTTON_PORT, THINGY52_BUTTON_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY52_COLOR_IRQ_PORT, THINGY52_COLOR_IRQ_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY52_GAS_IRQ_PORT, THINGY52_GAS_IRQ_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

bool Thingy52ExpanderInit(IoExpSx1509 &Expander, DeviceIntrf &Bus)
{
	Sx1509LedCfg_t led = { 32, 0, 0, 0, 0, 0 };
	unsigned i;
	uint8_t pin;

	if (!Expander.Init(&Bus, SX1509_I2C_ADDR0) || !Expander.SoftReset() ||
		!Expander.WriteOutputs(s_LedMask))
		return false;
	// Preload every latch before enabling outputs. No readback of physical pins.
	for (pin = THINGY52_SX_MOTION_POWER; pin <= THINGY52_SX_GAS_WAKE; ++pin)
	{
		if (!Expander.ConfigurePin(pin, true, false))
			return false;
	}
	if (!Expander.ConfigureClock(4))
		return false;
	for (i = 0; i < sizeof(s_LedPins) / sizeof(s_LedPins[0]); ++i)
	{
		if (!Expander.ConfigureLed(s_LedPins[i], led))
			return false;
	}
	return true;
}

bool Thingy52MotionPower(IoExpSx1509 &Expander, bool Enable)
{
	uint16_t mask = 1U << THINGY52_SX_MOTION_POWER;

	if (!Expander.WritePins(mask, Enable ? mask : 0))
		return false;
	if (Enable)
		msDelay(100);
	return true;
}

bool Thingy52GasPower(IoExpSx1509 &Expander, bool Enable)
{
	// Keep reset/wake low while switching the supply, avoiding back-powering.
	if (!Expander.WritePins(s_GasReset | s_GasWake, 0))
		return false;
	if (!Enable)
		return Expander.WritePins(s_GasPower, 0);
	if (!Expander.WritePins(s_GasPower, s_GasPower))
		return false;
	msDelay(30);
	if (!Expander.WritePins(s_GasReset, s_GasReset))
		return false;
	msDelay(2);
	return Expander.WritePins(s_GasWake, s_GasWake);
}

bool Thingy52GasWake(bool Active, void *pContext)
{
	IoExpSx1509 *expander = static_cast<IoExpSx1509 *>(pContext);

	return expander != nullptr && expander->WritePins(s_GasWake, Active ? 0 : s_GasWake);
}

bool Thingy52Lightwell(IoExpSx1509 &Expander, bool Red, bool Green, bool Blue)
{
	uint16_t mask = (1U << THINGY52_SX_LIGHTWELL_RED) |
		(1U << THINGY52_SX_LIGHTWELL_GREEN) | (1U << THINGY52_SX_LIGHTWELL_BLUE);
	uint16_t active = (Red ? 1U << THINGY52_SX_LIGHTWELL_RED : 0) |
		(Green ? 1U << THINGY52_SX_LIGHTWELL_GREEN : 0) |
		(Blue ? 1U << THINGY52_SX_LIGHTWELL_BLUE : 0);

	return Expander.WritePins(mask, mask & ~active);
}

bool Thingy52ColorLight(IoExpSx1509 &Expander, bool Red, bool Green, bool Blue)
{
	uint16_t mask = (1U << THINGY52_SX_COLOR_RED) |
		(1U << THINGY52_SX_COLOR_GREEN) | (1U << THINGY52_SX_COLOR_BLUE);
	uint16_t active = (Red ? 1U << THINGY52_SX_COLOR_RED : 0) |
		(Green ? 1U << THINGY52_SX_COLOR_GREEN : 0) |
		(Blue ? 1U << THINGY52_SX_COLOR_BLUE : 0);

	return Expander.WritePins(mask, mask & ~active);
}

bool Thingy52ButtonPressed()
{
	return IOPinRead(THINGY52_BUTTON_PORT, THINGY52_BUTTON_PIN) == 0;
}

bool Thingy52ColorIrqActive()
{
	return IOPinRead(THINGY52_COLOR_IRQ_PORT, THINGY52_COLOR_IRQ_PIN) == 0;
}

bool Thingy52GasIrqActive()
{
	return IOPinRead(THINGY52_GAS_IRQ_PORT, THINGY52_GAS_IRQ_PIN) == 0;
}
