/**-------------------------------------------------------------------------
@file	rtc_lpc546xx.h

@brief	LPC546xx realtime clock (RTC)

RtcDevice (miscdev/rtc.h) on the RTC of the LPC546xx. DevNo 0, no interface.
One RTC: the object of the last Init owns it, the other objects are refused.

The RTC counts seconds in a 32 bit counter on the 32.768 kHz crystal, the
calendar is the counter as seconds since 1970-01-01, 1970 to 2105, no
sub-seconds. A fresh RTC starts at 2000-01-01 00:00:00. RTC_CLKSRC_DEFAULT
and RTC_CLKSRC_LFXTAL take the crystal, the RTC has no RC clock and
RTC_CLKSRC_LFRC fails. One alarm on the match register, re-armed by the
interrupt for the next time that matches its fields. Wakeup timer on the
1.024 kHz countdown, 1 to 63999 ms, restarted by the interrupt. 7 backup
registers (GPREG0 to 6), GPREG7 marks the calendar as set. Enable is true for
the owner, Disable and Reset stop the alarm and the wakeup timer, the
calendar keeps counting. See rtc_lpc546xx.cpp.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __RTC_LPC546XX_H__
#define __RTC_LPC546XX_H__

#include <stdint.h>

#include "miscdev/rtc.h"

/// RTC of the LPC546xx
class RtcLpc546xx : public RtcDevice {
public:
	RtcLpc546xx() {}
	~RtcLpc546xx() override;

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
	RtcLpc546xx(const RtcLpc546xx &) = delete;
	RtcLpc546xx &operator=(const RtcLpc546xx &) = delete;

	bool Owned() const;
	void ArmAlarm();

	RtcAlarm_t vAlarm = {};		//!< Alarm fields
	bool vbAlarm = false;		//!< Alarm set
	uint32_t vWakeTicks = 0;	//!< Wakeup period in 1.024 kHz counts, 0 off
};

#endif // __RTC_LPC546XX_H__
