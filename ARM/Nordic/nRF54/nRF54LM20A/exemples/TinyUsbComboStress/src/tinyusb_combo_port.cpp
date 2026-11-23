/**-------------------------------------------------------------------------
@example	tinyusb_combo_port.cpp

@brief	nRF54LM20 chip glue for the TinyUSB composite stress benchmark.

Powers the USBHS controller for TinyUSB's DWC2 driver, owns
USBHS_IRQHandler and provides the FICR device identifier.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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

#include "nrf.h"
#include "lib/nrfx_coredep.h"
#include "tusb.h"
#include "tinyusb_combo_port.h"

extern "C" void USBHS_IRQHandler(void)
{
	tusb_int_handler(0, true);
}

// TinyUSB's DWC2 port runs the USBHS power-up only for the LM20A engineering
// sample, and its LM20 board support starts the 24 MHz crystal clock. This is
// the same sequence, run before tusb_init reads any core register: crystal
// clock, USB regulator, core, PHY reset release, core reset release. The
// VBUSVALID override is dropped at the end so the PHY reports the real VBUS.
bool TinyUsbPortInit(void)
{
	NRF_CLOCK->EVENTS_XO24MSTARTED = 0;
	NRF_CLOCK->TASKS_XO24MSTART = CLOCK_TASKS_XO24MSTART_TASKS_XO24MSTART_Trigger;
	while (NRF_CLOCK->EVENTS_XO24MSTARTED == 0)
	{
	}
	NRF_CLOCK->EVENTS_XO24MSTARTED = 0;

	NRF_VREGUSB->TASKS_START = VREGUSB_TASKS_START_TASKS_START_Trigger;

	NRF_USBHS->ENABLE = USBHS_ENABLE_CORE_Msk;
	NRF_USBHS->PHY.OVERRIDEVALUES =
		USBHS_PHY_OVERRIDEVALUES_ID_Device << USBHS_PHY_OVERRIDEVALUES_ID_Pos;
	NRF_USBHS->PHY.INPUTOVERRIDE =
		USBHS_PHY_INPUTOVERRIDE_ID_Msk | USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;
	NRF_USBHS->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;
	nrfx_coredep_delay_us(45);

	NRF_USBHS->TASKS_START = USBHS_TASKS_START_TASKS_START_Trigger;
	nrfx_coredep_delay_us(2);

	NRF_USBHS->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk;

	// The wrapper and the DWC2 core are separate peripheral blocks
	__DSB();

	NVIC_SetPriority(USBHS_IRQn, 6U);

	return true;
}

// TinyUSB's DWC2 driver takes all its events from the controller interrupt;
// this port has no USB power events to report.
void TinyUsbPortProcess(void)
{
}

void TinyUsbPortDeviceId(uint32_t Id[2])
{
	Id[0] = NRF_FICR->INFO.DEVICEID[0];
	Id[1] = NRF_FICR->INFO.DEVICEID[1];
}
