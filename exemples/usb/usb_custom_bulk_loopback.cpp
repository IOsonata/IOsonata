/**-------------------------------------------------------------------------
@example	usb_custom_bulk_loopback.cpp

@brief	USB custom bulk loopback example

This example creates one custom USB interface with a bulk OUT endpoint and a
bulk IN endpoint. The application does not choose interface or endpoint
numbers. UsbdBulk allocates the USB topology when it is initialized and the
descriptor provider asks the class for the resulting descriptor fragment.

The custom interface uses USB class 0xFF. Subclass and protocol are left at
zero so an application can define its own protocol on top of the bulk data
path. The loop below simply returns every byte received from the host.

@author	Hoang Nguyen Hoan
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

#include <stdint.h>
#include <string.h>

#include "cfifo.h"
#include "usb/usb.h"
#include "usb/usbd_bulk.h"

#define USB_DEVNO				0
#define CUSTOM_STR_MANUFACTURER	1U
#define CUSTOM_STR_PRODUCT		2U
#define CUSTOM_STR_SERIAL		3U
#define CUSTOM_STR_INTERFACE	4U
#define CUSTOM_STR_MAXLEN		32U

#define CUSTOM_RXFIFO_PKTCNT	4
#define CUSTOM_RXFIFO_MEMSIZE \
	USBD_BULK_RXMEM_SIZE(CUSTOM_RXFIFO_PKTCNT)
#define CUSTOM_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)

#define LOOPBACK_BUFFER_SIZE	USB_PKT_MAXLEN(USB_DEVNO, BULK)

alignas(4) static uint8_t s_RxFifoMem[CUSTOM_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_TxFifoMem[CUSTOM_TXFIFO_MEMSIZE];

static UsbdBulk g_CustomBulk;

#pragma pack(push, 1)
typedef struct __Custom_Config_Descriptor {
	UsbCfgDesc_t Config;
	UsbdBulkDesc_t Bulk;
} CustomConfigDesc_t;
#pragma pack(pop)

static UsbDevDesc_t s_DeviceDesc;
static UsbDevQualDesc_t s_QualifierDesc;
static CustomConfigDesc_t s_ConfigDesc;
static uint8_t s_StringDesc[2U + (CUSTOM_STR_MAXLEN * 2U)];

const uint8_t *UsbGetDescriptor(int DevNo, uint8_t DescType,
								 uint8_t DescIndex, uint16_t LangId,
								 UsbSpeed_t Speed, uint16_t *pLength);

static const UsbdBulkCfg_t s_BulkCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CUSTOM_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_RxFifoMem,
	.TxFifoMemSize = CUSTOM_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_TxFifoMem,
	.SubClass = 0U,
	.Protocol = 0U,
	.InterfaceString = CUSTOM_STR_INTERFACE,
	.FsMps = 0U,
	.HsMps = 0U,
	.Mode = USBD_BULK_MODE_BYTE,
	.pDesc = &s_ConfigDesc.Bulk,
	.RequestHandler = nullptr,
	.pRequestContext = nullptr,
	.EvtCB = nullptr,
};

// These VID/PID values are for the example. Use IDs assigned to your product
// before shipping a device.
static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0002,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata Custom Bulk Loopback",
	.pSerial = nullptr,
	.pFuncName = "Custom Bulk",
	.NbCdc = 0,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

static uint8_t CustomMaxPower(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0U;
	}

	uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static const uint8_t *CustomDeviceDescriptor(uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr)
	{
		return nullptr;
	}

	memset(&s_DeviceDesc, 0, sizeof(s_DeviceDesc));
	s_DeviceDesc.bLength = sizeof(s_DeviceDesc);
	s_DeviceDesc.bDescriptorType = USB_DESCTYPE_DEVICE;
	s_DeviceDesc.bcdUSB = 0x0200U;
	s_DeviceDesc.bDeviceClass = USB_DEVCLASS_NONE;
	s_DeviceDesc.bDeviceSubClass = 0U;
	s_DeviceDesc.bDeviceProtocol = 0U;
	s_DeviceDesc.bMaxPacketSize = USB_PKT_MAXLEN(USB_DEVNO, CONTROL);
	s_DeviceDesc.idVendor = pCfg->Vid;
	s_DeviceDesc.idProduct = pCfg->Pid;
	s_DeviceDesc.bcdDevice = pCfg->DevVer;
	s_DeviceDesc.iManufacturer = pCfg->pManufacturer != nullptr ?
		CUSTOM_STR_MANUFACTURER : 0U;
	s_DeviceDesc.iProduct = pCfg->pProduct != nullptr ? CUSTOM_STR_PRODUCT : 0U;
	s_DeviceDesc.iSerialNumber = UsbGetSerial(USB_DEVNO) != nullptr ?
		CUSTOM_STR_SERIAL : 0U;
	s_DeviceDesc.bNumConfigurations = 1U;

	*pLength = sizeof(s_DeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

static const uint8_t *CustomQualifierDescriptor(uint16_t *pLength)
{
	if (pLength == nullptr || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
	{
		return nullptr;
	}

	memset(&s_QualifierDesc, 0, sizeof(s_QualifierDesc));
	s_QualifierDesc.bLength = sizeof(s_QualifierDesc);
	s_QualifierDesc.bDescriptorType = USB_DESCTYPE_DEVICE_QUALIFIER;
	s_QualifierDesc.bcdUSB = 0x0200U;
	s_QualifierDesc.bDeviceClass = USB_DEVCLASS_NONE;
	s_QualifierDesc.bDeviceSubClass = 0U;
	s_QualifierDesc.bDeviceProtocol = 0U;
	s_QualifierDesc.bMaxPacketSize0 = USB_PKT_MAXLEN(USB_DEVNO, CONTROL);
	s_QualifierDesc.bNumConfigurations = 1U;
	s_QualifierDesc.bReserved = 0U;

	*pLength = sizeof(s_QualifierDesc);
	return reinterpret_cast<const uint8_t *>(&s_QualifierDesc);
}

static const uint8_t *CustomConfigurationDescriptor(UsbSpeed_t Speed,
												 bool OtherSpeed,
												 uint16_t *pLength)
{
	// The bulk interface and endpoint fragment (s_ConfigDesc.Bulk) was filled
	// by UsbdBulk::Init. Only the configuration header is assembled here.
	(void)Speed;
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr)
	{
		return nullptr;
	}

	memset(&s_ConfigDesc.Config, 0, sizeof(s_ConfigDesc.Config));
	s_ConfigDesc.Config.bLength = sizeof(s_ConfigDesc.Config);
	s_ConfigDesc.Config.bDescriptorType = OtherSpeed ?
		USB_DESCTYPE_OSC : USB_DESCTYPE_CONFIGURATION;
	s_ConfigDesc.Config.wTotalLength = sizeof(s_ConfigDesc);
	s_ConfigDesc.Config.bNumInterfaces = 1U;
	s_ConfigDesc.Config.bConfigurationValue = USBD_BULK_CONFIG_VALUE;
	s_ConfigDesc.Config.iConfiguration = 0U;
	s_ConfigDesc.Config.bmAttributes = USB_CONFATT_RESERVED;
	if (pCfg->bSelfPowered)
	{
		s_ConfigDesc.Config.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	s_ConfigDesc.Config.bMaxPower = CustomMaxPower(pCfg);

	*pLength = sizeof(s_ConfigDesc);
	return reinterpret_cast<const uint8_t *>(&s_ConfigDesc);
}

static const uint8_t *CustomStringDescriptor(uint8_t Index, uint16_t LangId,
											 uint16_t *pLength)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}

	if (Index == 0U)
	{
		s_StringDesc[0] = 4U;
		s_StringDesc[1] = USB_DESCTYPE_STRING;
		s_StringDesc[2] = 0x09U;
		s_StringDesc[3] = 0x04U;
		*pLength = 4U;
		return s_StringDesc;
	}

	if (Index > CUSTOM_STR_INTERFACE ||
		(LangId != 0U && LangId != 0x0409U))
	{
		return nullptr;
	}

	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr)
	{
		return nullptr;
	}

	const char *pStr = nullptr;
	switch (Index)
	{
		case CUSTOM_STR_MANUFACTURER:
			pStr = pCfg->pManufacturer;
			break;

		case CUSTOM_STR_PRODUCT:
			pStr = pCfg->pProduct;
			break;

		case CUSTOM_STR_SERIAL:
			pStr = UsbGetSerial(USB_DEVNO);
			break;

		case CUSTOM_STR_INTERFACE:
			pStr = pCfg->pFuncName;
			break;

		default:
			return nullptr;
	}

	if (pStr == nullptr)
	{
		return nullptr;
	}

	size_t length = strlen(pStr);
	if (length > CUSTOM_STR_MAXLEN)
	{
		length = CUSTOM_STR_MAXLEN;
	}

	s_StringDesc[0] = (uint8_t)(2U + (length * 2U));
	s_StringDesc[1] = USB_DESCTYPE_STRING;
	for (size_t i = 0; i < length; i++)
	{
		s_StringDesc[2U + (i * 2U)] = (uint8_t)pStr[i];
		s_StringDesc[3U + (i * 2U)] = 0U;
	}

	*pLength = s_StringDesc[0];
	return s_StringDesc;
}

const uint8_t *UsbGetDescriptor(int DevNo, uint8_t DescType,
								 uint8_t DescIndex, uint16_t LangId,
								 UsbSpeed_t Speed, uint16_t *pLength)
{
	(void)DevNo;

	if (pLength == nullptr)
	{
		return nullptr;
	}
	*pLength = 0U;

	switch (DescType)
	{
		case USB_DESCTYPE_DEVICE:
			return DescIndex == 0U ? CustomDeviceDescriptor(pLength) : nullptr;

		case USB_DESCTYPE_CONFIGURATION:
			return DescIndex == 0U ?
				CustomConfigurationDescriptor(Speed, false, pLength) : nullptr;

		case USB_DESCTYPE_STRING:
			return CustomStringDescriptor(DescIndex, LangId, pLength);

		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return DescIndex == 0U ? CustomQualifierDescriptor(pLength) : nullptr;

		case USB_DESCTYPE_OSC:
			if (DescIndex != 0U || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
			{
				return nullptr;
			}
			return CustomConfigurationDescriptor(
				Speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH,
				true, pLength);

		default:
			return nullptr;
	}
}

int main()
{
	uint8_t buffer[LOOPBACK_BUFFER_SIZE];

	if (!UsbInit(&s_UsbCfg))
	{
		return -1;
	}

	// UsbdBulk allocates one interface and one bidirectional endpoint pair.
	// No interface or endpoint number is part of s_BulkCfg.
	if (!g_CustomBulk.Init(s_BulkCfg))
	{
		return -1;
	}

	// A board may start without a cable. UsbProcess retries the connection
	// when VBUS appears, so a false result here is not fatal.
	(void)UsbEnable(USB_DEVNO);

	int pending = 0;
	int offset = 0;

	while (1)
	{
		UsbProcess(USB_DEVNO);

		if (pending > 0)
		{
			int n = g_CustomBulk.TxData(&buffer[offset], pending);
			if (n > 0)
			{
				offset += n;
				pending -= n;
			}
			continue;
		}

		int len = g_CustomBulk.RxData(buffer, sizeof(buffer));
		if (len > 0)
		{
			pending = len;
			offset = 0;
		}
	}

	return 0;
}
