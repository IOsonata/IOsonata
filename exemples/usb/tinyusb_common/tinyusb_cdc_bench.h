/**-------------------------------------------------------------------------
@file	tinyusb_cdc_bench.h

@brief	TinyUSB nRF52840 CDC benchmark support.

This file provides the common device descriptors, nRF52 USB power handling and
interrupt forwarding used by the TinyUSB CDC performance examples. It is kept
outside the IOsonata USB stack so the benchmark measures TinyUSB's native CDC
device path.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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
#ifndef __TINYUSB_CDC_BENCH_H__
#define __TINYUSB_CDC_BENCH_H__

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "tusb.h"

#ifndef TINYUSB_BENCH_PID
#error TINYUSB_BENCH_PID must be defined before including tinyusb_cdc_bench.h
#endif

#ifndef TINYUSB_BENCH_PRODUCT
#error TINYUSB_BENCH_PRODUCT must be defined before including tinyusb_cdc_bench.h
#endif

#define TINYUSB_BENCH_VID			0x1209U
#define TINYUSB_BENCH_EP_NOTIFY		0x81U
#define TINYUSB_BENCH_EP_OUT			0x02U
#define TINYUSB_BENCH_EP_IN			0x82U
#define TINYUSB_BENCH_NOTIFY_MPS		8U
#define TINYUSB_BENCH_DATA_MPS			64U

enum {
	TINYUSB_BENCH_ITF_CDC = 0,
	TINYUSB_BENCH_ITF_CDC_DATA,
	TINYUSB_BENCH_ITF_COUNT
};

enum {
	TINYUSB_BENCH_STR_LANGID = 0,
	TINYUSB_BENCH_STR_MANUFACTURER,
	TINYUSB_BENCH_STR_PRODUCT,
	TINYUSB_BENCH_STR_SERIAL,
	TINYUSB_BENCH_STR_CDC
};

#define TINYUSB_BENCH_CONFIG_LEN \
	(TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const tusb_desc_device_t s_TinyUsbBenchDeviceDesc = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = TINYUSB_BENCH_VID,
	.idProduct = TINYUSB_BENCH_PID,
	.bcdDevice = 0x0100,
	.iManufacturer = TINYUSB_BENCH_STR_MANUFACTURER,
	.iProduct = TINYUSB_BENCH_STR_PRODUCT,
	.iSerialNumber = TINYUSB_BENCH_STR_SERIAL,
	.bNumConfigurations = 1
};

static const uint8_t s_TinyUsbBenchConfigDesc[] = {
	TUD_CONFIG_DESCRIPTOR(1, TINYUSB_BENCH_ITF_COUNT, 0,
						  TINYUSB_BENCH_CONFIG_LEN, 0, 100),
	TUD_CDC_DESCRIPTOR(TINYUSB_BENCH_ITF_CDC, TINYUSB_BENCH_STR_CDC,
					   TINYUSB_BENCH_EP_NOTIFY,
					   TINYUSB_BENCH_NOTIFY_MPS,
					   TINYUSB_BENCH_EP_OUT,
					   TINYUSB_BENCH_EP_IN,
					   TINYUSB_BENCH_DATA_MPS)
};

static const char * const s_TinyUsbBenchStrings[] = {
	nullptr,
	"I-SYST",
	TINYUSB_BENCH_PRODUCT,
	nullptr,
	"TinyUSB CDC"
};

static uint16_t s_TinyUsbBenchStringDesc[33];

extern "C" uint8_t const *tud_descriptor_device_cb(void)
{
	return reinterpret_cast<const uint8_t *>(&s_TinyUsbBenchDeviceDesc);
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t Index)
{
	(void)Index;

	return s_TinyUsbBenchConfigDesc;
}

static size_t TinyUsbBenchSerialString(uint16_t *pString, size_t MaxLen)
{
	static const char Hex[] = "0123456789ABCDEF";
	const uint32_t id[2] = {
		NRF_FICR->DEVICEID[0],
		NRF_FICR->DEVICEID[1]
	};
	size_t count = 0;

	for (unsigned word = 0; word < 2 && count < MaxLen; word++)
	{
		for (int shift = 28; shift >= 0 && count < MaxLen; shift -= 4)
		{
			pString[count++] = (uint16_t)Hex[(id[word] >> shift) & 0x0fU];
		}
	}

	return count;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t Index,
											 uint16_t LangId)
{
	(void)LangId;

	size_t count = 0;

	if (Index == TINYUSB_BENCH_STR_LANGID)
	{
		s_TinyUsbBenchStringDesc[1] = 0x0409;
		count = 1;
	}
	else if (Index == TINYUSB_BENCH_STR_SERIAL)
	{
		count = TinyUsbBenchSerialString(&s_TinyUsbBenchStringDesc[1], 32);
	}
	else
	{
		if (Index >= sizeof(s_TinyUsbBenchStrings) /
					 sizeof(s_TinyUsbBenchStrings[0]) ||
			s_TinyUsbBenchStrings[Index] == nullptr)
		{
			return nullptr;
		}

		const char *p = s_TinyUsbBenchStrings[Index];
		count = strlen(p);
		if (count > 32)
		{
			count = 32;
		}

		for (size_t i = 0; i < count; i++)
		{
			s_TinyUsbBenchStringDesc[1 + i] = (uint16_t)p[i];
		}
	}

	s_TinyUsbBenchStringDesc[0] =
		(uint16_t)((TUSB_DESC_STRING << 8) | (2U * count + 2U));

	return s_TinyUsbBenchStringDesc;
}

// TinyUSB's Nordic DCD requires the application/BSP to forward USB regulator
// state transitions to it.
extern "C" void tusb_hal_nrf_power_event(uint32_t Event);

static bool s_TinyUsbBenchVbus;
static bool s_TinyUsbBenchReady;

static void TinyUsbBenchPowerProcess(void)
{
	const uint32_t status = NRF_POWER->USBREGSTATUS;
	const bool vbus =
		(status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0U;
	const bool ready =
		(status & POWER_USBREGSTATUS_OUTPUTRDY_Msk) != 0U;

	if (!vbus)
	{
		if (s_TinyUsbBenchVbus)
		{
			tusb_hal_nrf_power_event(1U);
		}
		s_TinyUsbBenchVbus = false;
		s_TinyUsbBenchReady = false;
		return;
	}

	if (!s_TinyUsbBenchVbus)
	{
		s_TinyUsbBenchVbus = true;
		tusb_hal_nrf_power_event(0U);
	}

	if (ready && !s_TinyUsbBenchReady)
	{
		s_TinyUsbBenchReady = true;
		tusb_hal_nrf_power_event(2U);
	}
	else if (!ready)
	{
		s_TinyUsbBenchReady = false;
	}
}

extern "C" void USBD_IRQHandler(void)
{
	tusb_int_handler(0, true);
}

static bool TinyUsbBenchInit(void)
{
	NVIC_SetPriority(USBD_IRQn, 6U);

	tusb_rhport_init_t devInit = {};
	devInit.role = TUSB_ROLE_DEVICE;
	devInit.speed = TUSB_SPEED_FULL;

	if (!tusb_init(0, &devInit))
	{
		return false;
	}

	TinyUsbBenchPowerProcess();

	return true;
}

#endif	// __TINYUSB_CDC_BENCH_H__
