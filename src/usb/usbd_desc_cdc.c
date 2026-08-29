/**-------------------------------------------------------------------------
@file	usbd_desc_cdc.c

@brief	Default CDC ACM descriptors.

Builds device, configuration and string descriptors from UsbDevCfg_t so that
an ordinary serial port needs no descriptor code in the application.

Nothing else lives in this file. An application that needs its own
descriptors defines all of the tud_descriptor callbacks itself, and then
nothing in the archive refers to this object, so the linker never pulls it in
and the two definitions never meet.

@author	Hoang Nguyen Hoan
@date	Aug. 28, 2026

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
#include <string.h>

#include "tusb.h"

#include "usb/usb_dev.h"

//
// String indices. Zero is the language list and is not a string.
//
#define USBD_DESC_STR_MANUFACTURER		1
#define USBD_DESC_STR_PRODUCT			2
#define USBD_DESC_STR_SERIAL			3
#define USBD_DESC_STR_FUNC				4
#define USBD_DESC_STR_MAXCNT			5

#define USBD_DESC_STR_MAXLEN			32

//
// Endpoints. Each CDC function takes one interrupt IN for notifications and
// one bulk pair. Numbered from one upwards, two per function.
//
#define USBD_DESC_EP_NOTIF(n)			(0x81 + ((n) * 2))
#define USBD_DESC_EP_OUT(n)				(0x02 + ((n) * 2))
#define USBD_DESC_EP_IN(n)				(0x82 + ((n) * 2))

#define USBD_DESC_EP_NOTIF_SIZE			8

#define USBD_DESC_CFG_MAXLEN \
	(TUD_CONFIG_DESC_LEN + (CFG_TUD_CDC * TUD_CDC_DESC_LEN))

static uint8_t s_UsbdDescCfg[USBD_DESC_CFG_MAXLEN];
static uint16_t s_UsbdDescStr[1 + USBD_DESC_STR_MAXLEN];
static tusb_desc_device_t s_UsbdDescDev;

static uint16_t UsbdDescBulkSize(void)
{
	//
	// A high speed bulk endpoint is 512 octets and nothing else is legal.
	// tud_speed_get reports what the bus actually negotiated, so a high speed
	// part plugged into a full speed host still describes 64.
	//
	return tud_speed_get() == TUSB_SPEED_HIGH ? 512 : 64;
}

static uint16_t UsbdDescBuildCfg(uint8_t *pBuff, uint16_t BulkSize)
{
	const UsbDevCfg_t *cfg = UsbDevGetCfg();
	uint16_t ofs = 0;
	uint8_t attr = TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP;
	int nbcdc;

	if (cfg == NULL)
	{
		return 0;
	}

	nbcdc = cfg->NbCdc;

	if (cfg->bSelfPowered)
	{
		attr |= TUSB_DESC_CONFIG_ATT_SELF_POWERED;
	}

	{
		uint16_t total = TUD_CONFIG_DESC_LEN + (uint16_t)(nbcdc * TUD_CDC_DESC_LEN);
		uint8_t d[TUD_CONFIG_DESC_LEN] = {
			TUD_CONFIG_DESCRIPTOR(1, nbcdc * 2, 0, total, attr, cfg->MaxPower)
		};

		memcpy(&pBuff[ofs], d, sizeof(d));
		ofs += sizeof(d);
	}

	for (int i = 0; i < nbcdc; i++)
	{
		uint8_t d[TUD_CDC_DESC_LEN] = {
			TUD_CDC_DESCRIPTOR(i * 2, USBD_DESC_STR_FUNC,
							   USBD_DESC_EP_NOTIF(i), USBD_DESC_EP_NOTIF_SIZE,
							   USBD_DESC_EP_OUT(i), USBD_DESC_EP_IN(i),
							   BulkSize)
		};

		memcpy(&pBuff[ofs], d, sizeof(d));
		ofs += sizeof(d);
	}

	return ofs;
}

const uint8_t *tud_descriptor_device_cb(void)
{
	const UsbDevCfg_t *cfg = UsbDevGetCfg();

	memset(&s_UsbdDescDev, 0, sizeof(s_UsbdDescDev));

	s_UsbdDescDev.bLength = sizeof(tusb_desc_device_t);
	s_UsbdDescDev.bDescriptorType = TUSB_DESC_DEVICE;
	s_UsbdDescDev.bcdUSB = 0x0200;

	//
	// The interface association tuple. Used even for a single function, so
	// that adding a second one does not change how the host sees the first.
	//
	s_UsbdDescDev.bDeviceClass = TUSB_CLASS_MISC;
	s_UsbdDescDev.bDeviceSubClass = MISC_SUBCLASS_COMMON;
	s_UsbdDescDev.bDeviceProtocol = MISC_PROTOCOL_IAD;
	s_UsbdDescDev.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE;

	if (cfg != NULL)
	{
		s_UsbdDescDev.idVendor = cfg->Vid;
		s_UsbdDescDev.idProduct = cfg->Pid;
		s_UsbdDescDev.bcdDevice = cfg->DevVer;
		s_UsbdDescDev.iManufacturer =
			cfg->pManufacturer != NULL ? USBD_DESC_STR_MANUFACTURER : 0;
		s_UsbdDescDev.iProduct =
			cfg->pProduct != NULL ? USBD_DESC_STR_PRODUCT : 0;
		s_UsbdDescDev.iSerialNumber = USBD_DESC_STR_SERIAL;
	}

	s_UsbdDescDev.bNumConfigurations = 1;

	return (const uint8_t *)&s_UsbdDescDev;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
	(void)index;

	if (UsbdDescBuildCfg(s_UsbdDescCfg, UsbdDescBulkSize()) == 0)
	{
		return NULL;
	}

	return s_UsbdDescCfg;
}

#if TUD_OPT_HIGH_SPEED

//
// A high speed device has to answer both of these. Without them the host
// cannot find out what the device would do at the other speed, and some hosts
// refuse the device rather than assume.
//
static tusb_desc_device_qualifier_t s_UsbdDescQual;

const uint8_t *tud_descriptor_device_qualifier_cb(void)
{
	const uint8_t *dev = tud_descriptor_device_cb();

	(void)dev;

	memset(&s_UsbdDescQual, 0, sizeof(s_UsbdDescQual));

	s_UsbdDescQual.bLength = sizeof(tusb_desc_device_qualifier_t);
	s_UsbdDescQual.bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER;
	s_UsbdDescQual.bcdUSB = 0x0200;
	s_UsbdDescQual.bDeviceClass = TUSB_CLASS_MISC;
	s_UsbdDescQual.bDeviceSubClass = MISC_SUBCLASS_COMMON;
	s_UsbdDescQual.bDeviceProtocol = MISC_PROTOCOL_IAD;
	s_UsbdDescQual.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE;
	s_UsbdDescQual.bNumConfigurations = 1;

	return (const uint8_t *)&s_UsbdDescQual;
}

const uint8_t *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
	(void)index;

	//
	// The same layout at the other speed, which for bulk means the other
	// packet size.
	//
	uint16_t bulk = tud_speed_get() == TUSB_SPEED_HIGH ? 64 : 512;

	if (UsbdDescBuildCfg(s_UsbdDescCfg, bulk) == 0)
	{
		return NULL;
	}

	s_UsbdDescCfg[1] = TUSB_DESC_OTHER_SPEED_CONFIG;

	return s_UsbdDescCfg;
}

#endif

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	const UsbDevCfg_t *cfg = UsbDevGetCfg();
	const char *str = NULL;
	size_t len = 0;

	(void)langid;

	if (index == 0)
	{
		// English, United States
		s_UsbdDescStr[1] = 0x0409;
		len = 1;
	}
	else
	{
		if (cfg == NULL || index >= USBD_DESC_STR_MAXCNT)
		{
			return NULL;
		}

		switch (index)
		{
			case USBD_DESC_STR_MANUFACTURER:
				str = cfg->pManufacturer;
				break;
			case USBD_DESC_STR_PRODUCT:
				str = cfg->pProduct;
				break;
			case USBD_DESC_STR_SERIAL:
				str = UsbDevGetSerial();
				break;
			case USBD_DESC_STR_FUNC:
				str = cfg->pFuncName;
				break;
			default:
				return NULL;
		}

		if (str == NULL)
		{
			return NULL;
		}

		len = strlen(str);

		if (len > USBD_DESC_STR_MAXLEN)
		{
			len = USBD_DESC_STR_MAXLEN;
		}

		for (size_t i = 0; i < len; i++)
		{
			s_UsbdDescStr[1 + i] = (uint16_t)str[i];
		}
	}

	s_UsbdDescStr[0] = (uint16_t)((TUSB_DESC_STRING << 8) |
								  (2 * len + 2));

	return s_UsbdDescStr;
}
