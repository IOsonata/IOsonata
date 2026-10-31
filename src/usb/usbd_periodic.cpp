/**-------------------------------------------------------------------------
@file	usbd_periodic.cpp

@brief	Device-side periodic endpoint descriptors and alternate selection.

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

#include "usb/usbd_periodic.h"
#include "usb/usbd_epalloc.h"

bool UsbdPeriodic::Register(const UsbdPeriodicCfg_t &Cfg, uint8_t Type,
	uint16_t BufferSize)
{
	if (UsbGetCfg(Cfg.DevNo) == nullptr || Cfg.pAlternates == nullptr ||
		Cfg.AltCount == 0U || BufferSize == 0U ||
		(Type != USB_ENDPATT_TRANS_INT && Type != USB_ENDPATT_TRANS_ISO))
	{
		return false;
	}
	const bool iso = Type == USB_ENDPATT_TRANS_ISO;
	if (iso && !USB_ISO_SUPPORTED(Cfg.DevNo))
	{
		return false;
	}
	const uint16_t fsLimit = iso ? 1023U : 64U;
	for (unsigned i = 0; i < Cfg.AltCount; i++)
	{
		const UsbdPeriodicAlt_t &alt = Cfg.pAlternates[i];
		if (alt.FsMps == 0U || alt.FsMps > fsLimit ||
			USB_CTRLR_RX_CAPACITY(Cfg.DevNo, alt.FsMps) > BufferSize ||
			alt.FsInterval == 0U ||
			(iso && alt.FsInterval > 16U) ||
			(USB_HIGHSPEED_CAPABLE(Cfg.DevNo) &&
			 (alt.HsMps == 0U || alt.HsMps > 1024U ||
			  USB_CTRLR_RX_CAPACITY(Cfg.DevNo, alt.HsMps) > BufferSize ||
			  alt.HsInterval == 0U ||
			  alt.HsInterval > 16U)))
		{
			return false;
		}
	}

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	UsbdEpAllocRes_t alloc = {};
	uint8_t ep = 0U;
	if (iso)
	{
		// Dedicated ISO endpoints may lie outside the ordinary endpoint
		// count (nRF52). Try every permitted pair, not just the first bit.
		uint16_t mask = USB_ISO_EPIN_MASK(Cfg.DevNo) &
			USB_ISO_EPOUT_MASK(Cfg.DevNo) & 0xFFFEU;
		while (mask != 0U)
		{
			ep = (uint8_t)__builtin_ctz(mask);
			req.FixedInMask = req.FixedOutMask = (uint16_t)(1U << ep);
			if (UsbdEpAlloc(Cfg.DevNo, &req, this, &alloc))
			{
				break;
			}
			mask &= (uint16_t)(mask - 1U);
		}
		if (mask == 0U)
		{
			return false;
		}
	}
	else
	{
		req.BidirectionalCount = 1U;
		req.BidirectionalType = Type;
		if (!UsbdEpAlloc(Cfg.DevNo, &req, this, &alloc))
		{
			return false;
		}
		ep = alloc.Bidirectional[0];
	}

	vCfg = Cfg;
	vType = Type;
	vEndpoint = ep;
	vAlternate = 0U;
	vConfigured = false;
	const uint16_t length = sizeof(UsbIntrfDesc_t) + Cfg.AltCount *
		(sizeof(UsbIntrfDesc_t) + 2U * sizeof(UsbEndPointDesc_t));
	if (!UsbDescRegister(Cfg.DevNo, this, nullptr, length, Build))
	{
		Rollback();
		return false;
	}
	return true;
}

void UsbdPeriodic::Rollback(void)
{
	(void)UsbdClassRollback(vCfg.DevNo, this);
	vCfg = {};
	vEndpoint = 0U;
	vAlternate = 0U;
	vConfigured = false;
}

void UsbdPeriodic::Build(const UsbDeviceClass *pClass, uint8_t *pDesc,
	UsbSpeed_t Speed)
{
	const UsbdPeriodic *fn = static_cast<const UsbdPeriodic *>(pClass);
	UsbIntrfDesc_t intrf = {};
	intrf.bLength = sizeof(intrf);
	intrf.bDescriptorType = USB_DESCTYPE_INTERFACE;
	intrf.bInterfaceNumber = fn->FirstInterface();
	intrf.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	intrf.bInterfaceSubClass = fn->vCfg.SubClass;
	intrf.bInterfaceProtocol = fn->vCfg.Protocol;
	intrf.iInterface = fn->vCfg.InterfaceString;
	memcpy(pDesc, &intrf, sizeof(intrf));
	pDesc += sizeof(intrf);
	intrf.bNumEndpoints = 2U;
	for (unsigned i = 0; i < fn->vCfg.AltCount; i++)
	{
		const UsbdPeriodicAlt_t &alt = fn->vCfg.pAlternates[i];
		intrf.bAlternateSetting = (uint8_t)(i + 1U);
		memcpy(pDesc, &intrf, sizeof(intrf));
		pDesc += sizeof(intrf);
		UsbEndPointDesc_t ep = {};
		ep.bLength = sizeof(ep);
		ep.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		ep.bEndpointAddress = USB_ENDPADDR_DIROUT(fn->vEndpoint);
		ep.bmAttributes = fn->vType;
		ep.wMaxPacketSize = Speed == USB_SPEED_HIGH ? alt.HsMps : alt.FsMps;
		ep.bInterval = Speed == USB_SPEED_HIGH ? alt.HsInterval : alt.FsInterval;
		memcpy(pDesc, &ep, sizeof(ep));
		pDesc += sizeof(ep);
		ep.bEndpointAddress = USB_ENDPADDR_DIRIN(fn->vEndpoint);
		memcpy(pDesc, &ep, sizeof(ep));
		pDesc += sizeof(ep);
	}
}

bool UsbdPeriodic::SelectConfig(uint8_t ConfigValue)
{
	if (vEndpoint == 0U || ConfigValue > 1U)
	{
		return false;
	}
	CloseTransport();
	vAlternate = 0U;
	vConfigured = ConfigValue == 1U;
	return true;
}

bool UsbdPeriodic::OpenAlternate(uint8_t Option)
{
	if (Option == 0U)
	{
		CloseTransport();
		return true;
	}
	const UsbdPeriodicAlt_t &alt = vCfg.pAlternates[Option - 1U];
	const bool hs = UsbGetSpeed(vCfg.DevNo) == USB_SPEED_HIGH;
	return OpenTransport(hs ? alt.HsMps : alt.FsMps,
		hs ? alt.HsInterval : alt.FsInterval);
}

bool UsbdPeriodic::SelectInterface(uint8_t InterfaceNo, uint8_t Option)
{
	if (!vConfigured || InterfaceNo != FirstInterface() ||
		Option > vCfg.AltCount)
	{
		return false;
	}
	if (!OpenAlternate(Option))
	{
		// The core retains the previous alternate on a refused request.
		// Restore its transport after an endpoint-open failure.
		(void)OpenAlternate(vAlternate);
		return false;
	}
	vAlternate = Option;
	return true;
}

void UsbdPeriodic::Reset(void)
{
	if (vEndpoint != 0U)
	{
		CloseTransport();
	}
	vConfigured = false;
	vAlternate = 0U;
}
