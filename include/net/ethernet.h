/**-------------------------------------------------------------------------
@file	ethernet.h

@brief	Ethernet frame interface, shared C implementation and C++ wrapper.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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

#ifndef __ETHERNET_H__
#define __ETHERNET_H__

#include "device_intrf.h"
#include "coredev/iopincfg.h"

#define ETHERNET_ADDR_SIZE		6
#define ETHERNET_FRAME_SIZE		1514

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	const IOPinCfg_t *pIOPinMap;
	int NbIOPins;
	uint8_t MacAddr[ETHERNET_ADDR_SIZE];
	int PhyAddr;				//!< MDIO address, 0..31
	int IntPrio;
	bool bIntEn;
	DevIntrfEvtHandler_t EvtCB;
} EthernetCfg_t;

typedef struct {
	DevIntrf_t DevIntrf;
	uint8_t MacAddr[ETHERNET_ADDR_SIZE];
} EthernetDev_t;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// Initialize the MAC/PHY. Link negotiation continues after Init returns.
/// Raw Tx accepts one frame, copies it to driver storage, and returns its
/// length, or 0 when no descriptor is free. The driver supplies padding/FCS.
/// Rx returns one complete frame without FCS, or 0 when none is available.
/// An undersized receive buffer drops that frame and returns 0.
/// DevAddr is ignored. No hook takes or clears the generic transfer lock.
bool EthernetInit(EthernetDev_t *pDev, const EthernetCfg_t *pCfg);
void EthernetClose(EthernetDev_t *pDev);

/// Foreground only. Read PHY negotiation status and update the MAC speed.
/// Rate() returns the last negotiated link rate, or 0 while down.
/// RX/TX events may run in the Ethernet ISR; link events run in this call.
void EthernetPollLink(EthernetDev_t *pDev);

#ifdef __cplusplus
}

class Ethernet : public DeviceIntrf {
public:
	Ethernet() = default;
	virtual ~Ethernet()
	{
		EthernetClose(&vDevData);
	}

	bool Init(const EthernetCfg_t &Cfg)
	{
		return EthernetInit(&vDevData, &Cfg);
	}

	void PollLink(void)
	{
		EthernetPollLink(&vDevData);
	}

	uint32_t Rate(uint32_t DataRate) override
	{
		return DeviceIntrfSetRate(&vDevData.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override
	{
		return DeviceIntrfGetRate(&vDevData.DevIntrf);
	}

	operator DevIntrf_t *() override
	{
		return &vDevData.DevIntrf;
	}

	operator EthernetDev_t *()
	{
		return &vDevData;
	}

private:
	EthernetDev_t vDevData = {};
};
#endif

#endif
