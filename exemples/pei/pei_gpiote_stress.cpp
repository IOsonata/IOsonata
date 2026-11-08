/**-------------------------------------------------------------------------
@file	pei_gpiote_stress.cpp
@brief	nRF52832 GPIOTE channel and PEI lifecycle stress test.

All resource tests use IOsonata APIs, with no raw MCU register access.
No semihosting and no CPU interrupt forwarding PEI events.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

enum {
	STRESS_CHANNELS = 8,
	STRESS_EXHAUST_ROUNDS = 64,
	STRESS_LIFECYCLE_ROUNDS = 128
};

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

// Unique nRF52832 GPIO pins. Temporarily inputs, including LED/UART pins;
// reconfigure LED outputs and button pulls before the live test.
static const int s_Pins[STRESS_CHANNELS] = {
	BLYST_NANO_DK_BUT1_PIN,
	BLYST_NANO_DK_BUT2_PIN,
	BLYST_NANO_DK_LED1_PIN,
	BLYST_NANO_DK_LEDR_PIN,
	BLYST_NANO_DK_LEDG_PIN,
	BLYST_NANO_DK_LEDB_PIN,
	BLYST_NANO_DK_UART_RX_PIN,
	BLYST_NANO_DK_UART_TX_PIN
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiGpioteStress requires two LEDs");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiGpioteStress requires two buttons");

typedef struct {
	volatile int Status;		// 0 setup, 1 testing, 2 armed, -1 failed
	volatile int Error;		// Negative error code for failing step
	volatile int LastIrqNo;
	volatile uint32_t ExhaustionRounds;
	volatile uint32_t ReusedChannels;
	volatile uint32_t RouteRounds;
	volatile uint32_t IrqRounds;
	volatile uint32_t IrqCount;
} GpioteStressResult_t;

extern "C" {
volatile GpioteStressResult_t g_GpioteStress;
}

static void StressButtonIrq(int IntNo, void *pCtx)
{
	(void)pCtx;
	g_GpioteStress.LastIrqNo = IntNo;
	g_GpioteStress.IrqCount++;
	// Conventional ISR owns LED2; the PPI-routed LED1 is independent.
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
}

static bool StressExhaust(Pei &pei)
{
	PeiEvent_t evt[STRESS_CHANNELS] = {0U, };
	PeiEvent_t extra = 0U;
	int irq;
	int i;

	for (i = 0; i < STRESS_CHANNELS; i++)
	{
		if (!IOPinEnableEvent(pei, evt[i], 0, s_Pins[i],
			IOPINSENSE_LOW_TRANSITION))
		{
			g_GpioteStress.Error = -10;
			return false;
		}
	}

	if (IOPinEnableEvent(pei, extra, 0, STRESS_EXTRA_PIN,
		IOPINSENSE_LOW_TRANSITION) || extra != 0U)
	{
		g_GpioteStress.Error = -11;
		return false;
	}
	// All channels are owned: an ordinary IRQ cannot replace CC0.
	if (IOPinEnableInterrupt(0, IRQ_PRIO_NORMAL, 0, s_Pins[0],
		IOPINSENSE_LOW_TRANSITION, StressButtonIrq, nullptr))
	{
		g_GpioteStress.Error = -12;
		return false;
	}

	if (!IOPinDisableEvent(pei, evt[3]))
	{
		g_GpioteStress.Error = -13;
		return false;
	}
	if (!pei.ReleaseEvent(evt[3]))
	{
		g_GpioteStress.Error = -14;
		return false;
	}
	// One channel is now free. The pin belonging to another event is not.
	irq = IOPinAllocateInterrupt(IRQ_PRIO_NORMAL, 0, s_Pins[0],
		IOPINSENSE_LOW_TRANSITION, StressButtonIrq, nullptr);
	if (irq >= 0)
	{
		IOPinDisableInterrupt(irq);
		g_GpioteStress.Error = -15;
		return false;
	}
	if (!IOPinEnableEvent(pei, extra, 0, STRESS_EXTRA_PIN,
		IOPINSENSE_LOW_TRANSITION))
	{
		g_GpioteStress.Error = -16;
		return false;
	}
	g_GpioteStress.ReusedChannels++;

	for (i = 0; i < STRESS_CHANNELS; i++)
	{
		if (evt[i] == 0U)
		{
			continue;
		}
		if (!IOPinDisableEvent(pei, evt[i]))
		{
			g_GpioteStress.Error = -17;
			return false;
		}
		if (!pei.ReleaseEvent(evt[i]))
		{
			g_GpioteStress.Error = -18;
			return false;
		}
	}
	if (!IOPinDisableEvent(pei, extra))
	{
		g_GpioteStress.Error = -19;
		return false;
	}
	if (!pei.ReleaseEvent(extra))
	{
		g_GpioteStress.Error = -20;
		return false;
	}
	g_GpioteStress.ExhaustionRounds++;
	return true;
}

static bool StressRoute(Pei &pei)
{
	PeiEvent_t evt = 0U;

	if (!IOPinEnableEvent(pei, evt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_GpioteStress.Error = -30;
		return false;
	}
	if (!IOPinEnableTask(pei, evt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_GpioteStress.Error = -31;
		return false;
	}
	if (!pei.Enable(evt))
	{
		g_GpioteStress.Error = -32;
		return false;
	}
	if (!pei.Disable(evt))
	{
		g_GpioteStress.Error = -33;
		return false;
	}
	if (!IOPinDisableTask(pei, evt, s_Leds[0].PortNo,
		s_Leds[0].PinNo))
	{
		g_GpioteStress.Error = -34;
		return false;
	}
	if (!IOPinDisableEvent(pei, evt))
	{
		g_GpioteStress.Error = -35;
		return false;
	}
	if (!pei.ReleaseEvent(evt))
	{
		g_GpioteStress.Error = -36;
		return false;
	}
	g_GpioteStress.RouteRounds++;
	return true;
}

static bool StressIrq(void)
{
	int irq;

	irq = IOPinAllocateInterrupt(IRQ_PRIO_NORMAL,
		s_Buttons[0].PortNo, s_Buttons[0].PinNo,
		IOPINSENSE_LOW_TRANSITION, StressButtonIrq, nullptr);
	if (irq < 0)
	{
		g_GpioteStress.Error = -40;
		return false;
	}
	IOPinDisableInterrupt(irq);
	g_GpioteStress.IrqRounds++;
	return true;
}

int main(void)
{
	Pei pei;
	PeiEvent_t evt = 0U;
	PeiEvent_t duplicate = 0U;
	int irq;
	int i;
	int round;

	g_GpioteStress.Status = 1;
	g_GpioteStress.LastIrqNo = -1;

	for (i = 0; i < STRESS_CHANNELS; i++)
	{
		IOPinConfig(0, s_Pins[i], 0, IOPINDIR_INPUT,
			IOPINRES_NONE, IOPINTYPE_NORMAL);
	}
	IOPinConfig(0, STRESS_EXTRA_PIN, 0, IOPINDIR_INPUT,
		IOPINRES_NONE, IOPINTYPE_NORMAL);

	for (round = 0; round < STRESS_EXHAUST_ROUNDS; round++)
	{
		if (!StressExhaust(pei))
		{
			g_GpioteStress.Status = -1;
			return 1;
		}
	}

	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
	}

	for (round = 0; round < STRESS_LIFECYCLE_ROUNDS; round++)
	{
		if (!StressRoute(pei))
		{
			g_GpioteStress.Status = -1;
			return 1;
		}
		if (!StressIrq())
		{
			g_GpioteStress.Status = -1;
			return 1;
		}
	}

	irq = IOPinAllocateInterrupt(IRQ_PRIO_NORMAL,
		s_Buttons[0].PortNo, s_Buttons[0].PinNo,
		IOPINSENSE_LOW_TRANSITION, StressButtonIrq, nullptr);
	if (irq < 0)
	{
		g_GpioteStress.Error = -50;
		g_GpioteStress.Status = -1;
		return 1;
	}
	// PEI cannot take a pin already owned by an IRQ.
	if (IOPinEnableEvent(pei, duplicate, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_GpioteStress.Error = -51;
		g_GpioteStress.Status = -1;
		return 1;
	}
	if (!IOPinEnableEvent(pei, evt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_GpioteStress.Error = -52;
		g_GpioteStress.Status = -1;
		return 1;
	}
	if (!IOPinEnableTask(pei, evt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_GpioteStress.Error = -53;
		g_GpioteStress.Status = -1;
		return 1;
	}
	if (IOPinEnableTask(pei, evt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_GpioteStress.Error = -54;
		g_GpioteStress.Status = -1;
		return 1;
	}
	// A second IRQ registration must not silently replace BUT1.
	if (IOPinEnableInterrupt(irq, IRQ_PRIO_NORMAL, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION,
		StressButtonIrq, nullptr))
	{
		g_GpioteStress.Error = -55;
		g_GpioteStress.Status = -1;
		return 1;
	}
	if (!pei.Enable(evt))
	{
		g_GpioteStress.Error = -56;
		g_GpioteStress.Status = -1;
		return 1;
	}
	g_GpioteStress.Status = 2;

	for (;;)
	{
		__WFE();
	}
}
