/**-------------------------------------------------------------------------
@file	pei_hal_cpp_api.cpp
@brief	Compile-only check of C++ wrappers over the shared C PEI matrix
----------------------------------------------------------------------------*/
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "coredev/pwm.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "coredev/qdec.h"
#include "coredev/i2c.h"
#include "coredev/pdm.h"
#include "coredev/i2s.h"
#include "adc_nrf52_pei.h"
#include "converters/analog_comp.h"

bool PeiHalCppApplication(Timer &timer)
{
	Pei pei;
	PeiEvent_t evt = 0U;
	int trigger;

	if (!IOPinEnableEvent(pei, evt, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		return false;
	}
	if (!IOPinEnableTask(pei, evt, 0, 30, IOPIN_TASK_TOGGLE))
	{
		return false;
	}
	trigger = timer.EnableTimerTrigger(pei, evt, 500,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trigger < 0)
	{
		return false;
	}
	if (!timer.ConnectTask(pei, evt, TIMER_TASK_START))
	{
		return false;
	}
	timer.ExecuteTask(TIMER_TASK_STOP);
	pei.Enable(evt);
	pei.Disable(evt);
	timer.DisableTimerTrigger(trigger);
	IOPinDisableTask(pei, evt, 0, 30);
	IOPinDisableEvent(pei, evt);
	return pei.ReleaseEvent(evt);
}

// Compile-only proof that RTC C++ event methods delegate to the C matrix.
bool PeiHalRtcCppApplication(Timer &timer, Pei &pei)
{
	PeiEvent_t event = 0U;

	if (!timer.EnableTimerEvent(pei, event, TIMER_EVT_COUNTER_OVR))
	{
		return false;
	}
	if (!timer.ConnectTask(pei, event, TIMER_TASK_TRIGOVRFLW))
	{
		timer.DisableTimerEvent(pei, event);
		return false;
	}
	if (!timer.DisableTimerEvent(pei, event))
	{
		return false;
	}
	return pei.ReleaseEvent(event);
}

/* Compile-only check: the shared source also accepts a Pei object. */
bool PeiHalPortCppApplication(Pei &pei)
{
	PeiEvent_t evt = 0U;

	if (!IOPinEnablePortEvent(pei, evt))
	{
		return false;
	}
	if (!IOPinDisablePortEvent(pei, evt))
	{
		return false;
	}
	return pei.ReleaseEvent(evt);
}

// Compile-only check of the software event wrappers.
bool PeiHalSwEventCppApplication(Timer &timer, Pei &pei)
{
	PeiEvent_t sw = 0U;

	if (!pei.BindSwEvent(sw))
	{
		return false;
	}
	if (!timer.ConnectTask(pei, sw, TIMER_TASK_START) ||
		pei.SwEventTask(sw) == 0U)
	{
		pei.ReleaseEvent(sw);
		return false;
	}
	pei.Enable(sw);
	pei.Trigger(sw);
	return pei.ReleaseEvent(sw);
}

// Compile-only C++ client: functions use the same C PEI matrix.
bool PeiSaadcCppApplication(Pei &pei, PeiEvent_t trigger)
{
	PeiEvent_t evt = 0U;

	if (!AdcNrf52PreparePeiSampling())
	{
		return false;
	}
	if (!AdcNrf52EnablePeiEvent(pei, &evt, ADC_NRF52_PEI_EVENT_END))
	{
		return false;
	}
	if (!AdcNrf52ConnectTask(pei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52DisconnectTask(pei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52DisablePeiEvent(pei, evt))
	{
		return false;
	}
	return pei.ReleaseEvent(evt);
}

// Compile-only: the comparator wrapper uses the same Pei C matrix.
bool PeiComparatorCppApplication(AnalogComp &comp, Pei &pei,
	PeiEvent_t startEvent)
{
	PeiEvent_t cross = 0U;

	if (!comp.EnablePeiEvent(pei, cross, ANALOG_COMP_PEI_EVENT_CROSS))
	{
		return false;
	}
	if (!comp.ConnectTask(pei, startEvent, ANALOG_COMP_PEI_TASK_START))
	{
		return false;
	}
	if (!comp.DisconnectTask(pei, startEvent, ANALOG_COMP_PEI_TASK_START))
	{
		return false;
	}
	if (!comp.DisablePeiEvent(pei, cross))
	{
		return false;
	}
	return pei.ReleaseEvent(cross);
}

// Compile-only: C++ PWM wrapper uses the same C PEI device.
bool PeiHalPwmCppApplication(Pwm &pwm, Pei &pei,
	PeiEvent_t button)
{
	PeiEvent_t started = 0U;

	if (!pwm.EnablePeiEvent(pei, started,
		PWM_PEI_EVENT_SEQSTARTED0))
	{
		return false;
	}
	if (!pwm.ConnectTask(pei, button, PWM_PEI_TASK_STOP))
	{
		return false;
	}
	if (!pwm.DisconnectTask(pei, button, PWM_PEI_TASK_STOP))
	{
		return false;
	}
	if (!pwm.DisablePeiEvent(pei, started))
	{
		return false;
	}
	return pei.ReleaseEvent(started);
}


// Compile-only C++ use of the same PEI C matrix for PDM and I2S.
bool PeiAudioCppApplication(PdmDev_t *pPdm, I2S &i2s,
	Pei &pei, PeiEvent_t source)
{
	PeiEvent_t pdmStarted = 0U;
	PeiEvent_t i2sStopped = 0U;

	if (!PdmPreparePeiStart(pPdm))
	{
		return false;
	}
	if (!PdmEnablePeiEvent(pPdm, pei, &pdmStarted,
		PDM_PEI_EVENT_STARTED))
	{
		return false;
	}
	if (!PdmConnectTask(pPdm, pei, source, PDM_PEI_TASK_STOP))
	{
		return false;
	}
	if (!PdmDisconnectTask(pPdm, pei, source, PDM_PEI_TASK_STOP))
	{
		return false;
	}
	if (!PdmDisablePeiEvent(pPdm, pei, pdmStarted))
	{
		return false;
	}
	if (!pei.ReleaseEvent(pdmStarted))
	{
		return false;
	}

	i2s.Disable();
	if (!i2s.PreparePeiStart())
	{
		return false;
	}
	if (!i2s.EnablePeiEvent(pei, i2sStopped,
		I2S_PEI_EVENT_STOPPED))
	{
		return false;
	}
	if (!i2s.ConnectTask(pei, source, I2S_PEI_TASK_START))
	{
		return false;
	}
	if (!i2s.DisconnectTask(pei, source, I2S_PEI_TASK_START))
	{
		return false;
	}
	if (!i2s.DisablePeiEvent(pei, i2sStopped))
	{
		return false;
	}
	return pei.ReleaseEvent(i2sStopped);
}


// Compile-only: C++ SPI wrapper delegates PEI to the same C matrix.
bool PeiSpiCppApplication(SPI &spi, Pei &pei,
	PeiEvent_t button, uint8_t *pRx)
{
	PeiEvent_t end = 0U;
	const uint8_t tx[] = { 0x5A, 0xA5, 0xCC, 0x33 };

	if (!spi.PreparePeiTransfer(tx, sizeof(tx), pRx, sizeof(tx)))
	{
		return false;
	}
	if (!spi.EnablePeiEvent(pei, end, SPI_PEI_EVENT_END))
	{
		return false;
	}
	if (!spi.ConnectTask(pei, button, SPI_PEI_TASK_START))
	{
		return false;
	}
	if (!spi.DisconnectTask(pei, button, SPI_PEI_TASK_START))
	{
		return false;
	}
	if (!spi.DisablePeiEvent(pei, end))
	{
		return false;
	}
	if (!pei.ReleaseEvent(end))
	{
		return false;
	}
	return spi.ReleasePeiTransfer();
}


// Compile-only: C++ I2C delegates to the same C PeiDev_t matrix.
bool PeiTwiCppApplication(I2C &i2c, Pei &pei, PeiEvent_t button,
	uint8_t *pRx)
{
	PeiEvent_t error = 0U;

	if (!i2c.PreparePeiTransfer(0x42U, nullptr, 0U, pRx, 4U))
	{
		return false;
	}
	if (!i2c.EnablePeiEvent(pei, error, I2C_PEI_EVENT_ERROR))
	{
		return false;
	}
	if (!i2c.ConnectTask(pei, button, I2C_PEI_TASK_STARTRX))
	{
		return false;
	}
	if (!i2c.ConnectTask(pei, error, I2C_PEI_TASK_STOP))
	{
		return false;
	}
	if (!i2c.DisconnectTask(pei, button, I2C_PEI_TASK_STARTRX))
	{
		return false;
	}
	if (!i2c.DisconnectTask(pei, error, I2C_PEI_TASK_STOP))
	{
		return false;
	}
	if (!i2c.DisablePeiEvent(pei, error))
	{
		return false;
	}
	if (!pei.ReleaseEvent(error))
	{
		return false;
	}
	return i2c.ReleasePeiTransfer();
}


// Compile-only: UART's C++ methods use the same C interconnect matrix.
bool PeiUartCppApplication(UART &uart, Pei &pei, PeiEvent_t button)
{
	const uint8_t data[] = { 0x5A, 0xA5, 0x33, 0xCC };
	PeiEvent_t end = 0U;

	if (!uart.PreparePeiTx(data, sizeof(data)))
	{
		return false;
	}
	if (!uart.EnablePeiEvent(pei, end, UART_PEI_EVENT_ENDTX))
	{
		return false;
	}
	if (!uart.ConnectTask(pei, button, UART_PEI_TASK_STARTTX))
	{
		return false;
	}
	if (!uart.ConnectTask(pei, end, UART_PEI_TASK_STOPTX))
	{
		return false;
	}
	if (!uart.DisconnectTask(pei, button, UART_PEI_TASK_STARTTX))
	{
		return false;
	}
	if (!uart.DisconnectTask(pei, end, UART_PEI_TASK_STOPTX))
	{
		return false;
	}
	if (!uart.DisablePeiEvent(pei, end))
	{
		return false;
	}
	if (!pei.ReleaseEvent(end))
	{
		return false;
	}
	return uart.ReleasePeiTx();
}


// Compile-only QDEC C++ wrapper uses the same Pei matrix.
bool PeiQdecCppApplication(Qdec &qdec, Pei &pei, PeiEvent_t start)
{
	PeiEvent_t sample = 0U;

	if (!qdec.PreparePeiStart())
	{
		return false;
	}
	if (!qdec.EnablePeiEvent(pei, sample, QDEC_PEI_EVENT_SAMPLERDY))
	{
		return false;
	}
	if (!qdec.ConnectTask(pei, start, QDEC_PEI_TASK_START))
	{
		return false;
	}
	if (!qdec.DisconnectTask(pei, start, QDEC_PEI_TASK_START))
	{
		return false;
	}
	if (!qdec.DisablePeiEvent(pei, sample))
	{
		return false;
	}
	return pei.ReleaseEvent(sample);
}

// Compile-only: PPI group automation through the same C++ Pei matrix.
bool PeiGroupCppApplication(Pei &pei, PeiEvent_t grouped, PeiEvent_t arm)
{
	PeiGroup_t group = {};

	if (!pei.GroupInit(group))
	{
		return false;
	}
	if (!pei.GroupAddEvent(group, grouped))
	{
		return false;
	}
	if (!pei.Connect(arm, pei.GroupEnableTask(group)))
	{
		return false;
	}
	if (!pei.Disconnect(arm, pei.GroupEnableTask(group)))
	{
		return false;
	}
	if (!pei.GroupRemoveEvent(group, grouped))
	{
		return false;
	}
	return pei.GroupDeinit(group);
}
