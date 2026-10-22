/**-------------------------------------------------------------------------
@file	rtc_stm32.h

@brief	STM32 realtime clock (RTC)

RtcDevice (miscdev/rtc.h) on the RTC of the STM32 families: F0, F4, L1, L4,
L4+, G4, H7, WBA. DevNo 0, no interface. One RTC: the object of the last Init
owns it, the other objects are refused.

Clock: RTC_CLKSRC_DEFAULT starts the LSE and takes the LSI when the LSE does
not start within 4 s, RTC_CLKSRC_LFXTAL fails instead, RTC_CLKSRC_LFRC takes
the LSI. An RTC found running keeps its clock and its time. Calendar 2000 to
2099, sub-seconds in steps of one synchronous prescaler count. Two alarms,
one on the F0. Wakeup timer up to 131072 s, none on the F030x8. 0 to 32
backup registers. Enable is true for the owner, Disable and Reset stop the
alarms and the wakeup timer, the calendar keeps counting. See rtc_stm32.cpp
and ARM/ST/RTC.md.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#ifndef __RTC_STM32_H__
#define __RTC_STM32_H__

#include <stdint.h>

#include "miscdev/rtc.h"

/// RTC of the STM32
class RtcStm32 : public RtcDevice {
public:
	RtcStm32() {}
	~RtcStm32() override;
	bool Init(const RtcCfg_t &Cfg, DeviceIntrf * const pIntrf = nullptr) override;
	bool SetTime(const struct tm &Time) override;
	bool GetTime(struct tm &Time, uint32_t * const pUsec = nullptr) override;
	bool SetAlarm(int Idx, const RtcAlarm_t &Alarm) override;
	void DisableAlarm(int Idx) override;
	bool SetWakeup(uint32_t msPeriod) override;
	bool WriteBackup(int Idx, uint32_t Data) override;
	bool ReadBackup(int Idx, uint32_t &Data) override;
	void IntHandler() override;
	bool Enable() override;
	void Disable() override;
	void Reset() override;

private:
	RtcStm32(const RtcStm32 &) = delete;
	RtcStm32 &operator=(const RtcStm32 &) = delete;
	bool Owned() const;

	uint32_t vPredivS = 0;		//!< Synchronous prescaler, sub-second counter top
};

#endif // __RTC_STM32_H__
