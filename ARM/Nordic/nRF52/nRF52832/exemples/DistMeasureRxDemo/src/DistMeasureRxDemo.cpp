/**-------------------------------------------------------------------------
@example	DistMeasureRxDemo.cpp

@brief	Work-in-progress Nordic distance measurement receiver example.

@author	Nguyen Hoan Hoang
@date	2023

@license

MIT License

Copyright 2023, I-SYST inc. All rights reserved.

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
#include "nrf_dm.h"

// The two PPI channels and TIMER2 belong exclusively to this bare-metal
// ranging example. Do not start the SoftDevice at the same time.
static uint8_t s_PpiChannels[] = { 0, 1 };
static nrf_dm_ppi_config_t s_PpiCfg = { 2, s_PpiChannels };
static nrf_dm_antenna_config_t s_AntennaCfg = { 0, NULL, 0, NULL };

// The initiator must use the same seed and MCPD ranging mode.
static nrf_dm_config_t s_DmCfg = {
	NRF_DM_ROLE_REFLECTOR,
	NRF_DM_RANGING_MODE_MCPD,
	0x12345678UL,
	NRF_DM_DEFAULT_TX_POWER,
	NRF_DM_DEFAULT_ANTENNA_COMP
};

static nrf_dm_report_t s_LastReport;
static volatile uint32_t s_MeasurementCount;
static volatile uint32_t s_ErrorCount;
static volatile nrf_dm_status_t s_LastStatus;

int main()
{
	nrf_dm_init(&s_PpiCfg, &s_AntennaCfg, NRF_TIMER2);
	s_LastStatus = nrf_dm_configure(&s_DmCfg);
	if (s_LastStatus != NRF_DM_STATUS_SUCCESS)
	{
		return 1;
	}

	for (;;)
	{
		// The reflector waits up to one second for an initiator to start.
		s_LastStatus = nrf_dm_proc_execute(1000000UL);
		if (s_LastStatus == NRF_DM_STATUS_SUCCESS)
		{
			nrf_dm_populate_report(&s_LastReport);
			s_MeasurementCount++;
		}
		else
		{
			s_ErrorCount++;
		}
	}
}
