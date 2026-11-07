/**-------------------------------------------------------------------------
@file	pei_blinky.cpp

@brief	PEI blinky - timer event toggles a GPIO task without an ISR

The target port prepares the timer event and output task. PEI configures
only the event-to-task relationship. The shared application is independent
of the MCU; the target adapter is temporary until Timer/GPIO PEI endpoint
methods are implemented in IOsonata.

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "pei_blinky_port.h"

extern "C" {
volatile uint32_t g_PeiStatus;
}

int main(void)
{
	Pei pei;
	PeiEvent_t evt = 0U;
	PeiEndpoint_t source;
	PeiEndpoint_t task;

	if (!PeiBlinkyPrepare(&source, &task))
	{
		g_PeiStatus = 2U;
		return 1;
	}
	if (!pei.BindEvent(evt, source))
	{
		g_PeiStatus = 3U;
		return 1;
	}
	if (!pei.Connect(evt, task))
	{
		g_PeiStatus = 3U;
		return 1;
	}
	if (!pei.Enable(evt))
	{
		g_PeiStatus = 4U;
		return 1;
	}

	g_PeiStatus = 1U;
	PeiBlinkyStart();

	for (;;)
	{
		PeiBlinkyIdle();
	}
}
