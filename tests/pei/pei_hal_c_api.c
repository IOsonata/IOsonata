/**-------------------------------------------------------------------------
@file	pei_hal_c_api.c
@brief	Compile-only proof that PEI, Timer and GPIO APIs work from C

The test is intentionally not executed: hardware setup and routing require
a target MCU. tests/pei/run.py compiles this as C11 with -Werror.
----------------------------------------------------------------------------*/
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "coredev/pwm.h"
#include "coredev/spi.h"
#include "coredev/pdm.h"
#include "coredev/i2s.h"
#include "adc_nrf52_pei.h"
#include "converters/analog_comp.h"

bool PeiHalCApplication(TimerDev_t *pTimer)
{
	PeiDev_t pei;
	PeiEvent_t evt;
	int trigger;

	evt = 0U;
	if (!PeiInit(&pei, 0))
	{
		return false;
	}
	if (!IOPinEnableEvent(&pei, &evt, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		return false;
	}
	if (!IOPinEnableTask(&pei, evt, 0, 30, IOPIN_TASK_TOGGLE))
	{
		return false;
	}
	trigger = TimerEnablePeiTrigger(pTimer, &pei, &evt, 500,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trigger < 0)
	{
		return false;
	}
	if (!TimerConnectTask(pTimer, &pei, evt, TIMER_TASK_START))
	{
		return false;
	}
	TimerExecuteTask(pTimer, TIMER_TASK_STOP);
	PeiEnableEvent(&pei, evt);
	PeiDisableEvent(&pei, evt);
	TimerDisableTrigger(pTimer, trigger);
	IOPinDisableTask(&pei, evt, 0, 30);
	IOPinDisableEvent(&pei, evt);
	PeiReleaseEvent(&pei, &evt);
	return PeiDeinit(&pei);
}

// Compile-only check of RTC event producer and hardware task endpoint.
bool PeiHalRtcCApplication(TimerDev_t *pTimer, PeiDev_t *pPei)
{
	PeiEvent_t event;

	event = 0U;
	if (!TimerEnablePeiEvent(pTimer, pPei, &event, TIMER_EVT_TICK))
	{
		return false;
	}
	if (!TimerConnectTask(pTimer, pPei, event, TIMER_TASK_CLEAR))
	{
		TimerDisablePeiEvent(pTimer, pPei, event);
		return false;
	}
	if (!TimerDisablePeiEvent(pTimer, pPei, event))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &event);
}

/* Compile-only check of the shared PORT source and its lifetime. */
bool PeiHalPortCApplication(PeiDev_t *pPei)
{
	PeiEvent_t evt;

	evt = 0U;
	IOPinSetSense(0, 2, IOPINSENSE_LOW_TRANSITION);
	if (!IOPinEnablePortEvent(pPei, &evt))
	{
		return false;
	}
	if (!IOPinDisablePortEvent(pPei, evt))
	{
		return false;
	}
	IOPinSetSense(0, 2, IOPINSENSE_DISABLE);
	return PeiReleaseEvent(pPei, &evt);
}

// Compile-only check of a software event starting a Timer and a GPIO task,
// fired from code and from a GPIO event.
bool PeiHalSwEventCApplication(TimerDev_t *pTimer, PeiDev_t *pPei)
{
	PeiEvent_t sw;
	PeiEvent_t button;

	sw = 0U;
	button = 0U;
	if (!PeiBindSwEvent(pPei, &sw))
	{
		return false;
	}
	if (!TimerConnectTask(pTimer, pPei, sw, TIMER_TASK_START) ||
		!IOPinEnableTask(pPei, sw, 0, 30, IOPIN_TASK_SET))
	{
		PeiReleaseEvent(pPei, &sw);
		return false;
	}
	if (IOPinEnableEvent(pPei, &button, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		PeiConnectTask(pPei, button, PeiGetSwEventTask(pPei, sw));
		PeiEnableEvent(pPei, button);
	}
	PeiEnableEvent(pPei, sw);
	PeiTriggerSwEvent(pPei, sw);
	return PeiReleaseEvent(pPei, &sw);
}

// Compile-only: SAADC remains configured by the ADC HAL, not by PEI.
// These are real C-linkage routing functions, not C++ debugger globals.
bool PeiSaadcCApplication(PeiDev_t *pPei, PeiEvent_t trigger)
{
	PeiEvent_t endEvent;

	endEvent = 0U;
	if (!AdcNrf52PreparePeiSampling())
	{
		return false;
	}
	if (!AdcNrf52PeiBindEvent(pPei, &endEvent, ADC_NRF52_PEI_EVENT_END))
	{
		return false;
	}
	if (!AdcNrf52PeiConnectTask(pPei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52PeiConnectTask(pPei, endEvent, ADC_NRF52_PEI_TASK_START))
	{
		return false;
	}
	if (!AdcNrf52PeiDisconnectTask(pPei, endEvent, ADC_NRF52_PEI_TASK_START))
	{
		return false;
	}
	if (!AdcNrf52PeiDisconnectTask(pPei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52PeiUnbindEvent(pPei, endEvent))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &endEvent);
}

// Compile-only: comparator and PEI are both C-callable and independent of ISR.
bool PeiComparatorCApplication(AnalogCompDev_t *pComp, PeiDev_t *pPei,
	PeiEvent_t startEvent)
{
	PeiEvent_t evt;

	evt = 0U;
	if (!AnalogCompPeiBindEvent(pComp, pPei, &evt,
		ANALOG_COMP_PEI_EVENT_CROSS))
	{
		return false;
	}
	if (!AnalogCompPeiConnectTask(pComp, pPei, startEvent,
		ANALOG_COMP_PEI_TASK_START))
	{
		return false;
	}
	if (!AnalogCompPeiDisconnectTask(pComp, pPei, startEvent,
		ANALOG_COMP_PEI_TASK_START))
	{
		return false;
	}
	if (!AnalogCompPeiUnbindEvent(pComp, pPei, evt))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &evt);
}

// Compile-only: C PEI routing to a prepared PWM peripheral.
bool PeiHalPwmCApplication(PwmDev_t *pPwm, PeiDev_t *pPei,
	PeiEvent_t startEvent)
{
	PeiEvent_t stopped;

	stopped = 0U;
	if (!PWMEnablePeiEvent(pPwm, pPei, &stopped,
		PWM_PEI_EVENT_STOPPED))
	{
		return false;
	}
	if (!PWMConnectPeiTask(pPwm, pPei, startEvent,
		PWM_PEI_TASK_SEQSTART0))
	{
		return false;
	}
	if (!PWMDisconnectPeiTask(pPwm, pPei, startEvent,
		PWM_PEI_TASK_SEQSTART0))
	{
		return false;
	}
	if (!PWMDisablePeiEvent(pPwm, pPei, stopped))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &stopped);
}


// Compile-only C use: the driver, not PEI, owns capture DMA buffers.
bool PeiAudioCApplication(PdmDev_t *pPdm, I2SDev_t *pI2s,
	PeiDev_t *pPei, PeiEvent_t source)
{
	PeiEvent_t pdmEnd;
	PeiEvent_t i2sRx;

	pdmEnd = 0U;
	i2sRx = 0U;
	if (!PdmPreparePeiStart(pPdm))
	{
		return false;
	}
	if (!PdmEnablePeiEvent(pPdm, pPei, &pdmEnd, PDM_PEI_EVENT_END))
	{
		return false;
	}
	if (!PdmConnectPeiTask(pPdm, pPei, source, PDM_PEI_TASK_START))
	{
		return false;
	}
	if (!PdmDisconnectPeiTask(pPdm, pPei, source, PDM_PEI_TASK_START))
	{
		return false;
	}
	if (!PdmDisablePeiEvent(pPdm, pPei, pdmEnd))
	{
		return false;
	}
	if (!PeiReleaseEvent(pPei, &pdmEnd))
	{
		return false;
	}

	// A normal I2SInit starts streaming. Disable before arming PEI START.
	I2SDisable(pI2s);
	if (!I2SPreparePeiStart(pI2s))
	{
		return false;
	}
	if (!I2SEnablePeiEvent(pI2s, pPei, &i2sRx,
		I2S_PEI_EVENT_RXPTRUPD))
	{
		return false;
	}
	if (!I2SConnectPeiTask(pI2s, pPei, source, I2S_PEI_TASK_START))
	{
		return false;
	}
	if (!I2SDisconnectPeiTask(pI2s, pPei, source, I2S_PEI_TASK_START))
	{
		return false;
	}
	if (!I2SDisablePeiEvent(pI2s, pPei, i2sRx))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &i2sRx);
}


// Compile-only: mode-specific events and DMA task preparation from C.
bool PeiSpiCApplication(SPIDev_t *pSpi, PeiDev_t *pPei,
	PeiEvent_t source, uint8_t *pRx)
{
	PeiEvent_t end;
	const uint8_t tx[] = { 0xA5, 0x5A, 0x33, 0xCC };

	end = 0U;
	if (!SPIPreparePeiTransfer(pSpi, tx, sizeof(tx), pRx, sizeof(tx)))
	{
		return false;
	}
	if (!SPIEnablePeiEvent(pSpi, pPei, &end, SPI_PEI_EVENT_END))
	{
		return false;
	}
	if (!SPIConnectPeiTask(pSpi, pPei, source, SPI_PEI_TASK_START))
	{
		return false;
	}
	if (!SPIDisconnectPeiTask(pSpi, pPei, source, SPI_PEI_TASK_START))
	{
		return false;
	}
	if (!SPIDisablePeiEvent(pSpi, pPei, end))
	{
		return false;
	}
	if (!PeiReleaseEvent(pPei, &end))
	{
		return false;
	}
	return SPIReleasePeiTransfer(pSpi);
}
