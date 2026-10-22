/**-------------------------------------------------------------------------
@file	rtc.h

@brief	Generic realtime clock (RTC)

Calendar, alarms, periodic wakeup and backup registers of the MCU realtime
clock. The calendar is in the C runtime time types: struct tm for the date
and time, time_t and struct timespec for the seconds since the epoch.

The calendar has no time zone. The time_t functions treat it as the local
time of the C runtime (localtime_r, mktime): UTC unless the application sets
a time zone (TZ).

Each MCU implementation documents its calendar range, alarm and wakeup
limits.

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
#ifndef __RTC_H__
#define __RTC_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/** @addtogroup MiscDev
  * @{
  */

/// RTC clock source
typedef enum __Rtc_Clk_Src {
	RTC_CLKSRC_DEFAULT,			//!< Low frequency crystal, the low frequency RC when the crystal does not start
	RTC_CLKSRC_LFXTAL,			//!< Low frequency crystal or external oscillator only
	RTC_CLKSRC_LFRC,			//!< Low frequency RC oscillator
} RTC_CLKSRC;

// Events of RtcEvtHandler_t, bits
#define RTC_EVT_ALARM0			(1U << 0)	//!< Alarm 0 match
#define RTC_EVT_ALARM1			(1U << 1)	//!< Alarm 1 match
#define RTC_EVT_WAKEUP			(1U << 2)	//!< Periodic wakeup

typedef struct __Rtc_Device RtcDev_t;

/**
 * @brief	RTC event handler, called in interrupt context
 *
 * @param	pDev : RTC device data
 * @param	Evt  : RTC_EVT_xxx bits
 */
typedef void (*RtcEvtHandler_t)(RtcDev_t * const pDev, uint32_t Evt);

/// RTC configuration data
typedef struct __Rtc_Config {
	int DevNo;					//!< RTC instance number, 0 based
	RTC_CLKSRC ClkSrc;			//!< Clock source when the RTC starts
	int IntPrio;				//!< Interrupt priority of the events
	RtcEvtHandler_t EvtHandler;	//!< Event handler, NULL for none
} RtcCfg_t;

/// Alarm time, a field of -1 matches any value
typedef struct __Rtc_Alarm {
	int Sec;					//!< Second 0 - 59
	int Min;					//!< Minute 0 - 59
	int Hour;					//!< Hour 0 - 23
	int MDay;					//!< Day of the month 1 - 31
	int WDay;					//!< Day of the week 0 Sunday - 6 Saturday, not with MDay
} RtcAlarm_t;

/// RTC device data
struct __Rtc_Device {
	int DevNo;					//!< RTC instance number
	RTC_CLKSRC ClkSrc;			//!< Clock in use, RTC_CLKSRC_LFXTAL or RTC_CLKSRC_LFRC
	uint32_t ClkFreq;			//!< Frequency of that clock, Hz
	int NbAlarm;				//!< Number of alarms
	int NbBackup;				//!< Number of 32 bit backup registers
	uint32_t msWakeup;			//!< Wakeup period in effect, msec, 0 off
	bool bTimeValid;			//!< The calendar has been set since the RTC started
	RtcEvtHandler_t EvtHandler;	//!< Event handler
	void *pDevData;				//!< Implementation data
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Start the RTC
 *
 * An RTC already running keeps its clock and its time.
 *
 * @param	pDev : RTC device data to initialize
 * @param	pCfg : Configuration data
 *
 * @return	true - success
 */
bool RtcInit(RtcDev_t * const pDev, const RtcCfg_t * const pCfg);

/**
 * @brief	Set the date and time
 *
 * tm_wday and tm_yday are computed from the date, tm_isdst is not used.
 *
 * @param	pDev  : RTC device data
 * @param	pTime : Date and time
 *
 * @return	false with a date or time out of range
 */
bool RtcSetTime(RtcDev_t * const pDev, const struct tm * const pTime);

/**
 * @brief	Read the date and time
 *
 * @param	pDev  : RTC device data
 * @param	pTime : Date and time, tm_isdst -1 (not known)
 * @param	pUsec : Microseconds in the second, NULL when not needed
 *
 * @return	true - success
 */
bool RtcGetTime(RtcDev_t * const pDev, struct tm * const pTime, uint32_t * const pUsec);

/**
 * @brief	Set an alarm and enable its event
 *
 * @param	pDev   : RTC device data
 * @param	Idx    : Alarm 0 to NbAlarm - 1
 * @param	pAlarm : Alarm time
 *
 * @return	false with an alarm the RTC cannot match
 */
bool RtcSetAlarm(RtcDev_t * const pDev, int Idx, const RtcAlarm_t * const pAlarm);

/**
 * @brief	Disable an alarm
 *
 * @param	pDev : RTC device data
 * @param	Idx  : Alarm 0 to NbAlarm - 1
 */
void RtcDisableAlarm(RtcDev_t * const pDev, int Idx);

/**
 * @brief	Set the periodic wakeup event
 *
 * RtcDev_t::msWakeup gives the period in effect.
 *
 * @param	pDev     : RTC device data
 * @param	msPeriod : Period in msec, 0 to stop
 *
 * @return	false with a period the RTC cannot make
 */
bool RtcSetWakeup(RtcDev_t * const pDev, uint32_t msPeriod);

/**
 * @brief	Write a backup register, kept while the RTC runs
 *
 * @param	pDev : RTC device data
 * @param	Idx  : Register 0 to NbBackup - 1
 * @param	Data : Value
 *
 * @return	true - success
 */
bool RtcWriteBackup(RtcDev_t * const pDev, int Idx, uint32_t Data);

/**
 * @brief	Read a backup register
 *
 * @param	pDev  : RTC device data
 * @param	Idx   : Register 0 to NbBackup - 1
 * @param	pData : Value read
 *
 * @return	true - success
 */
bool RtcReadBackup(RtcDev_t * const pDev, int Idx, uint32_t * const pData);

/**
 * @brief	Set the date and time from the seconds since the epoch
 *
 * The calendar takes the local time of Time (localtime_r).
 *
 * @param	pDev : RTC device data
 * @param	Time : Seconds since the epoch
 *
 * @return	false with a time out of the calendar range
 */
bool RtcSetTimeEpoch(RtcDev_t * const pDev, time_t Time);

/**
 * @brief	Read the seconds since the epoch, with the fraction of second
 *
 * The calendar is converted as a local time (mktime).
 *
 * @param	pDev  : RTC device data
 * @param	pTime : Seconds since the epoch and nanoseconds
 *
 * @return	true - success
 */
bool RtcGetTimeEpoch(RtcDev_t * const pDev, struct timespec * const pTime);

#ifdef __cplusplus
}

/// RTC class
class Rtc {
public:
	virtual ~Rtc() {}

	virtual bool Init(const RtcCfg_t &Cfg) { return RtcInit(&vDev, &Cfg); }
	virtual bool SetTime(const struct tm &Time) { return RtcSetTime(&vDev, &Time); }
	virtual bool GetTime(struct tm &Time, uint32_t *pUsec = nullptr) { return RtcGetTime(&vDev, &Time, pUsec); }
	virtual bool SetTime(time_t Time) { return RtcSetTimeEpoch(&vDev, Time); }
	virtual bool GetTime(struct timespec &Time) { return RtcGetTimeEpoch(&vDev, &Time); }
	virtual bool SetAlarm(int Idx, const RtcAlarm_t &Alarm) { return RtcSetAlarm(&vDev, Idx, &Alarm); }
	virtual void DisableAlarm(int Idx) { RtcDisableAlarm(&vDev, Idx); }
	virtual bool SetWakeup(uint32_t msPeriod) { return RtcSetWakeup(&vDev, msPeriod); }
	virtual bool WriteBackup(int Idx, uint32_t Data) { return RtcWriteBackup(&vDev, Idx, Data); }
	virtual bool ReadBackup(int Idx, uint32_t &Data) { return RtcReadBackup(&vDev, Idx, &Data); }

	bool TimeValid() { return vDev.bTimeValid; }
	uint32_t ClockFrequency() { return vDev.ClkFreq; }
	RTC_CLKSRC ClockSource() { return vDev.ClkSrc; }
	int NbAlarm() { return vDev.NbAlarm; }
	int NbBackup() { return vDev.NbBackup; }
	uint32_t Wakeup() { return vDev.msWakeup; }

	operator RtcDev_t * () { return &vDev; }

private:
	RtcDev_t vDev;
};

#endif

/** @} */

#endif // __RTC_H__
