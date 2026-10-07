/**-------------------------------------------------------------------------
@example	tinyusb_combo_port.cpp

@brief	nRF52840 chip glue for the TinyUSB composite stress benchmark.

Reports the USB regulator state to TinyUSB's nRF5x DCD each main loop pass,
owns USBD_IRQHandler and provides the FICR device identifier.

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
#include "tusb.h"
#include "tinyusb_combo_port.h"

extern "C" void tusb_hal_nrf_power_event(uint32_t Event);

#if TINYUSB_COMBO_ISO_DIAG
extern "C" void dcd_nrf5x_iso_diag_get(
	uint32_t Counts[TINYUSB_COMBO_DCD_DIAG_COUNT]);
#endif

static bool s_Vbus;
static bool s_Ready;

extern "C" void USBD_IRQHandler(void)
{
	tusb_int_handler(0, true);
}

bool TinyUsbPortInit(void)
{
	NVIC_SetPriority(USBD_IRQn, 6U);

	return true;
}

// Event values are those of NRFX_POWER_USB_EVT_*: 0 detected, 1 removed,
// 2 ready. USB power may already be ready at startup, which raises no event,
// so the state is polled.
void TinyUsbPortProcess(void)
{
	const uint32_t status = NRF_POWER->USBREGSTATUS;
	const bool vbus =
		(status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0U;
	const bool ready =
		(status & POWER_USBREGSTATUS_OUTPUTRDY_Msk) != 0U;

	if (!vbus)
	{
		if (s_Vbus)
			tusb_hal_nrf_power_event(1U);
		s_Vbus = false;
		s_Ready = false;
		return;
	}

	if (!s_Vbus)
	{
		s_Vbus = true;
		tusb_hal_nrf_power_event(0U);
	}

	if (ready && !s_Ready)
	{
		s_Ready = true;
		tusb_hal_nrf_power_event(2U);
	}
	else if (!ready)
	{
		s_Ready = false;
	}
}

void TinyUsbPortDeviceId(uint32_t Id[2])
{
	Id[0] = NRF_FICR->DEVICEID[0];
	Id[1] = NRF_FICR->DEVICEID[1];
}

#if TINYUSB_COMBO_ISO_DIAG
void TinyUsbPortIsoDiag(uint32_t Counts[TINYUSB_COMBO_DCD_DIAG_COUNT])
{
	dcd_nrf5x_iso_diag_get(Counts);
}
#endif
