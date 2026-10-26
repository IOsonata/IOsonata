/**-------------------------------------------------------------------------
@file	usbd_periodic.h

@brief	Device-side alternate settings shared by Interrupt and ISO.

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
#ifndef __USBD_PERIODIC_H__
#define __USBD_PERIODIC_H__

#include "usb/usb.h"

/// One active alternate setting. The table entries are numbered 1..AltCount;
/// alternate zero is generated automatically and has no endpoints.
/// Intervals are USB bInterval values: FS Interrupt in ms, ISO and HS
/// Interrupt as 2^(Interval-1) frames/microframes. HS fields are required
/// only on high-speed-capable targets. High-bandwidth transactions are not
/// supported. The const table must remain alive and unchanged while registered.
typedef struct __Usbd_Periodic_Alternate {
	uint16_t FsMps;
	uint16_t HsMps;
	uint8_t FsInterval;
	uint8_t HsInterval;
} UsbdPeriodicAlt_t;

typedef struct __Usbd_Periodic_Config {
	int DevNo;
	const UsbdPeriodicAlt_t *pAlternates;
	uint8_t AltCount;
	uint8_t SubClass;
	uint8_t Protocol;
	uint8_t InterfaceString;
} UsbdPeriodicCfg_t;

#ifdef __cplusplus
/// Shared device policy for a vendor interface with one bidirectional pair.
/// Applications instantiate UsbdInt or UsbdIso. Override Control only for
/// application-specific requests; enumeration belongs to this class.
class UsbdPeriodic : public UsbDeviceClass {
public:
	bool SelectConfig(uint8_t ConfigValue) override;
	bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override;
	void Reset(void) override;
	void Detach(void) override { Reset(); }
	uint8_t Alternate(void) const
	{
		return vAlternate;
	}
	bool Configured(void) const
	{
		return vConfigured;
	}

protected:
	UsbdPeriodic() = default;
	bool Register(const UsbdPeriodicCfg_t &Cfg, uint8_t Type,
		uint16_t BufferSize);
	void Rollback(void);
	int DevNo(void) const
	{
		return vCfg.DevNo;
	}
	uint8_t Endpoint(void) const
	{
		return vEndpoint;
	}
	virtual bool OpenTransport(uint16_t Mps, uint8_t Interval) = 0;
	virtual void CloseTransport(void) = 0;

private:
	static void Build(const UsbDeviceClass *pClass, uint8_t *pDesc,
		UsbSpeed_t Speed);
	bool OpenAlternate(uint8_t Option);
	UsbdPeriodicCfg_t vCfg = {};
	uint8_t vType = 0;
	uint8_t vEndpoint = 0;
	uint8_t vAlternate = 0;
	bool vConfigured = false;
};
#endif
#endif
