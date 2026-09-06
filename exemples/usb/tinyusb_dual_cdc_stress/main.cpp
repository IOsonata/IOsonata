/**-------------------------------------------------------------------------
@example	tinyusb_dual_cdc_stress/main.cpp

@brief	TinyUSB dual CDC shared-controller stress test

CDC instance zero echoes a sustained host stream while CDC instance one
transmits PRBS continuously. The behavior mirrors usb_dual_cdc_stress.cpp so
Python/usb_dual_cdc_stress.py can compare the two USB stacks with the same
host-side workload.

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "prbs.h"
#include "tusb.h"

#define TINYUSB_DUAL_VID			0x1209U
#define TINYUSB_DUAL_PID			0x0004U
#define TINYUSB_DUAL_NOTIFY_MPS		8U
#define TINYUSB_DUAL_DATA_MPS		64U

#define TINYUSB_DUAL_CDC0_NOTIFY_EP	0x81U
#define TINYUSB_DUAL_CDC0_OUT_EP	0x02U
#define TINYUSB_DUAL_CDC0_IN_EP		0x82U
#define TINYUSB_DUAL_CDC1_NOTIFY_EP	0x83U
#define TINYUSB_DUAL_CDC1_OUT_EP	0x04U
#define TINYUSB_DUAL_CDC1_IN_EP		0x84U

enum {
	TINYUSB_DUAL_ITF_CDC0 = 0,
	TINYUSB_DUAL_ITF_CDC0_DATA,
	TINYUSB_DUAL_ITF_CDC1,
	TINYUSB_DUAL_ITF_CDC1_DATA,
	TINYUSB_DUAL_ITF_COUNT
};

enum {
	TINYUSB_DUAL_STR_LANGID = 0,
	TINYUSB_DUAL_STR_MANUFACTURER,
	TINYUSB_DUAL_STR_PRODUCT,
	TINYUSB_DUAL_STR_SERIAL,
	TINYUSB_DUAL_STR_CDC0,
	TINYUSB_DUAL_STR_CDC1
};

#define TINYUSB_DUAL_CONFIG_LEN \
	(TUD_CONFIG_DESC_LEN + 2U * TUD_CDC_DESC_LEN)

static const tusb_desc_device_t s_DeviceDesc = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = TINYUSB_DUAL_VID,
	.idProduct = TINYUSB_DUAL_PID,
	.bcdDevice = 0x0100,
	.iManufacturer = TINYUSB_DUAL_STR_MANUFACTURER,
	.iProduct = TINYUSB_DUAL_STR_PRODUCT,
	.iSerialNumber = TINYUSB_DUAL_STR_SERIAL,
	.bNumConfigurations = 1
};

static const uint8_t s_ConfigDesc[] = {
	TUD_CONFIG_DESCRIPTOR(1, TINYUSB_DUAL_ITF_COUNT, 0,
						  TINYUSB_DUAL_CONFIG_LEN, 0, 100),
	TUD_CDC_DESCRIPTOR(TINYUSB_DUAL_ITF_CDC0, TINYUSB_DUAL_STR_CDC0,
					   TINYUSB_DUAL_CDC0_NOTIFY_EP,
					   TINYUSB_DUAL_NOTIFY_MPS,
					   TINYUSB_DUAL_CDC0_OUT_EP,
					   TINYUSB_DUAL_CDC0_IN_EP,
					   TINYUSB_DUAL_DATA_MPS),
	TUD_CDC_DESCRIPTOR(TINYUSB_DUAL_ITF_CDC1, TINYUSB_DUAL_STR_CDC1,
					   TINYUSB_DUAL_CDC1_NOTIFY_EP,
					   TINYUSB_DUAL_NOTIFY_MPS,
					   TINYUSB_DUAL_CDC1_OUT_EP,
					   TINYUSB_DUAL_CDC1_IN_EP,
					   TINYUSB_DUAL_DATA_MPS)
};

static const char * const s_Strings[] = {
	nullptr,
	"I-SYST",
	"TinyUSB Dual CDC Stress",
	nullptr,
	"TinyUSB Loopback CDC",
	"TinyUSB PRBS CDC"
};

static uint16_t s_StringDesc[33];

extern "C" uint8_t const *tud_descriptor_device_cb(void)
{
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t Index)
{
	(void)Index;
	return s_ConfigDesc;
}

static size_t SerialString(uint16_t *pString, size_t MaxLen)
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
			pString[count++] =
				static_cast<uint16_t>(Hex[(id[word] >> shift) & 0x0fU]);
		}
	}

	return count;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t Index,
											 uint16_t LangId)
{
	(void)LangId;

	size_t count = 0;

	if (Index == TINYUSB_DUAL_STR_LANGID)
	{
		s_StringDesc[1] = 0x0409;
		count = 1;
	}
	else if (Index == TINYUSB_DUAL_STR_SERIAL)
	{
		count = SerialString(&s_StringDesc[1], 32);
	}
	else
	{
		if (Index >= sizeof(s_Strings) / sizeof(s_Strings[0]) ||
			s_Strings[Index] == nullptr)
		{
			return nullptr;
		}

		const char *p = s_Strings[Index];
		count = strlen(p);
		if (count > 32)
		{
			count = 32;
		}

		for (size_t i = 0; i < count; i++)
		{
			s_StringDesc[1 + i] = static_cast<uint16_t>(p[i]);
		}
	}

	s_StringDesc[0] =
		static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2U * count + 2U));

	return s_StringDesc;
}

extern "C" void tusb_hal_nrf_power_event(uint32_t Event);

static bool s_Vbus;
static bool s_Ready;

static void PowerProcess(void)
{
	const uint32_t status = NRF_POWER->USBREGSTATUS;
	const bool vbus =
		(status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0U;
	const bool ready =
		(status & POWER_USBREGSTATUS_OUTPUTRDY_Msk) != 0U;

	if (!vbus)
	{
		if (s_Vbus)
		{
			tusb_hal_nrf_power_event(1U);
		}
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

extern "C" void USBD_IRQHandler(void)
{
	tusb_int_handler(0, true);
}

static bool UsbInit(void)
{
	NVIC_SetPriority(USBD_IRQn, 6U);

	tusb_rhport_init_t devInit = {};
	devInit.role = TUSB_ROLE_DEVICE;
	devInit.speed = TUSB_SPEED_FULL;

	if (!tusb_init(0, &devInit))
	{
		return false;
	}

	PowerProcess();
	return true;
}

int main()
{
	static constexpr uint8_t LoopbackCdc = 0;
	static constexpr uint8_t PrbsCdc = 1;
	static constexpr unsigned BufferSize = TINYUSB_DUAL_DATA_MPS;

	uint8_t loopbackBuffer[BufferSize];
	uint8_t loopbackExpected = Prbs8(0xff);
	uint8_t prbs = 0xff;
	uint32_t loopbackRxErrorNotify = 0;
	unsigned loopbackPending = 0;
	unsigned loopbackOffset = 0;
	bool loopbackConnected = false;

	if (!UsbInit())
	{
		return -1;
	}

	while (1)
	{
		PowerProcess();
		tud_task_ext(0, false);

		const bool connected = tud_cdc_n_connected(LoopbackCdc);
		if (connected != loopbackConnected)
		{
			loopbackConnected = connected;
			loopbackPending = 0;
			loopbackOffset = 0;

			if (connected)
			{
				static const char Msg[] =
					"\r\nIOsonata USB Dual CDC Loopback\r\n";

				loopbackExpected = Prbs8(0xff);
				tud_cdc_n_write(LoopbackCdc, Msg, sizeof(Msg) - 1U);
				tud_cdc_n_write_flush(LoopbackCdc);
			}
		}

		// Service at most one loopback operation per pass so the PRBS
		// producer below always gets a chance to queue data as well.
		if (loopbackConnected)
		{
			if (loopbackPending > 0U)
			{
				const uint32_t room =
					tud_cdc_n_write_available(LoopbackCdc);
				if (room > 0U)
				{
					const unsigned count =
						loopbackPending < room ? loopbackPending : room;
					const uint32_t written = tud_cdc_n_write(
						LoopbackCdc, &loopbackBuffer[loopbackOffset],
						count);

					if (written > 0U)
					{
						loopbackOffset += written;
						loopbackPending -= written;
						tud_cdc_n_write_flush(LoopbackCdc);
					}
				}
			}
			else
			{
				const uint32_t available =
					tud_cdc_n_available(LoopbackCdc);
				if (available > 0U)
				{
					const uint32_t count =
						available < BufferSize ? available : BufferSize;
					const uint32_t length = tud_cdc_n_read(
						LoopbackCdc, loopbackBuffer, count);

					for (uint32_t i = 0; i < length; i++)
					{
						if (loopbackBuffer[i] != loopbackExpected)
						{
							loopbackRxErrorNotify++;
						}
						loopbackExpected = Prbs8(loopbackBuffer[i]);
					}

					loopbackPending = length;
					loopbackOffset = 0;
				}
			}
		}

		uint8_t prbsByte =
			loopbackRxErrorNotify > 0U ? 0U : prbs;
		if (tud_cdc_n_connected(PrbsCdc) &&
			tud_cdc_n_write_available(PrbsCdc) > 0U &&
			tud_cdc_n_write(PrbsCdc, &prbsByte, 1U) == 1U)
		{
			if (loopbackRxErrorNotify > 0U)
			{
				loopbackRxErrorNotify--;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
	}

	return 0;
}
