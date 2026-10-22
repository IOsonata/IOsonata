/**-------------------------------------------------------------------------
@file	rtc.h

@brief	Generic realtime clock (RTC)

Realtime clock device: the RTC of an MCU or an RTC chip on an I2C or SPI
interface. Each implementation derives from RtcDevice: the MCU RTC takes no
interface in Init, an RTC chip takes its device interface and its address
(RtcCfg_t::DevAddr). A chip implementation keeps them with Interface and
DeviceAddress of Device and reaches its registers with Device::Read and
Device::Write. An implementation sets the protected members the accessors
return (clock, alarms, backup registers, wakeup period, calendar set) and
gives its events with RtcEvt.

The calendar is in the C runtime time types: struct tm for the date and
time, time_t and struct timespec for the seconds since the epoch. The
calendar has no time zone. The time_t functions treat it as the local time
of the C runtime (localtime_r, mktime): UTC unless the application sets a
time zone (TZ).

Setting and reading the time is required of an implementation. Alarms,
periodic wakeup and backup registers are optional, refused by default. Each
implementation documents its calendar range, sub-second resolution, alarm,
wakeup and backup register limits.

Events (RTC_EVT_xxx bits) go to the event handler of the configuration, in
interrupt context. The MCU RTC handles its interrupts. For an RTC chip, the
application calls IntHandler from the interrupt of the pin wired to the
interrupt output of the chip.

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

#include "device.h"
#include "device_intrf.h"

/** @addtogroup MiscDev
  * @{
  */

/// RTC clock source
typedef enum __Rtc_Clk_Src {
	RTC_CLKSRC_DEFAULT,			//!< Default of the implementation: the low frequency crystal with the RC
								//!< when the crystal does not start on an MCU RTC, the oscillator of an
								//!< RTC chip
	RTC_CLKSRC_LFXTAL,			//!< Low frequency crystal or external oscillator only
	RTC_CLKSRC_LFRC,			//!< Low frequency RC oscillator
} RTC_CLKSRC;

// Events of RtcEvtHandler_t, bits
#define RTC_EVT_ALARM0			(1U << 0)	//!< Alarm 0 match
#define RTC_EVT_ALARM1			(1U << 1)	//!< Alarm 1 match
#define RTC_EVT_WAKEUP			(1U << 2)	//!< Periodic wakeup

#pragma pack(push, 4)

/// Alarm time, a field of -1 matches any value
typedef struct __Rtc_Alarm {
	int Sec;					//!< Second 0 - 59
	int Min;					//!< Minute 0 - 59
	int Hour;					//!< Hour 0 - 23
	int MDay;					//!< Day of the month 1 - 31
	int WDay;					//!< Day of the week 0 Sunday - 6 Saturday, not with MDay
} RtcAlarm_t;

#pragma pack(pop)

#ifdef __cplusplus

class RtcDevice;

/**
 * @brief	RTC event handler, called in interrupt context
 *
 * @param	pDev : RTC device
 * @param	Evt  : RTC_EVT_xxx bits
 */
typedef void (*RtcEvtHandler_t)(RtcDevice * const pDev, uint32_t Evt);

#pragma pack(push, 4)

/// RTC configuration data
typedef struct __Rtc_Config {
	int DevNo;					//!< MCU RTC instance, 0 based
	uint32_t DevAddr;			//!< RTC chip: I2C address or SPI chip select index
	RTC_CLKSRC ClkSrc;			//!< Clock source when the RTC starts
	int IntPrio;				//!< Interrupt priority of the MCU RTC events
	RtcEvtHandler_t EvtHandler;	//!< Event handler, nullptr for none
} RtcCfg_t;

#pragma pack(pop)

/// RTC generic base class, implementations derive from it
class RtcDevice : public Device {
public:
	virtual ~RtcDevice() {}

	/**
	 * @brief	Start the RTC
	 *
	 * An RTC already running keeps its time.
	 *
	 * @param	Cfg    : Configuration data
	 * @param	pIntrf : Interface of an RTC chip, nullptr for the MCU RTC
	 *
	 * @return	true - success
	 */
	virtual bool Init(const RtcCfg_t &Cfg, DeviceIntrf * const pIntrf = nullptr) = 0;

	/**
	 * @brief	Set the date and time
	 *
	 * tm_wday and tm_yday are computed from the date, tm_isdst is not used.
	 *
	 * @param	Time : Date and time
	 *
	 * @return	false with a date or time out of the range of the RTC
	 */
	virtual bool SetTime(const struct tm &Time) = 0;

	/**
	 * @brief	Read the date and time
	 *
	 * @param	Time  : Date and time, tm_isdst -1 (not known)
	 * @param	pUsec : Microseconds in the second, 0 on an RTC without
	 * 					sub-seconds, nullptr when not needed
	 *
	 * @return	true - success
	 */
	virtual bool GetTime(struct tm &Time, uint32_t * const pUsec = nullptr) = 0;

	/**
	 * @brief	Set an alarm and enable its event
	 *
	 * @param	Idx   : Alarm 0 to NbAlarm() - 1
	 * @param	Alarm : Alarm time
	 *
	 * @return	false with an alarm the RTC cannot match, or no alarm
	 */
	virtual bool SetAlarm(int Idx, const RtcAlarm_t &Alarm) { (void)Idx; (void)Alarm; return false; }

	/**
	 * @brief	Disable an alarm
	 *
	 * @param	Idx : Alarm 0 to NbAlarm() - 1
	 */
	virtual void DisableAlarm(int Idx) { (void)Idx; }

	/**
	 * @brief	Set the periodic wakeup event
	 *
	 * Wakeup() gives the period in effect.
	 *
	 * @param	msPeriod : Period in msec, 0 to stop
	 *
	 * @return	false with a period the RTC cannot make
	 */
	virtual bool SetWakeup(uint32_t msPeriod) { return msPeriod == 0; }

	/**
	 * @brief	Write a backup register, kept while the RTC runs
	 *
	 * @param	Idx  : 32 bit register 0 to NbBackup() - 1
	 * @param	Data : Value
	 *
	 * @return	true - success
	 */
	virtual bool WriteBackup(int Idx, uint32_t Data) { (void)Idx; (void)Data; return false; }

	/**
	 * @brief	Read a backup register
	 *
	 * @param	Idx  : 32 bit register 0 to NbBackup() - 1
	 * @param	Data : Value read
	 *
	 * @return	true - success
	 */
	virtual bool ReadBackup(int Idx, uint32_t &Data) { (void)Idx; (void)Data; return false; }

	/**
	 * @brief	Interrupt of the RTC
	 *
	 * RTC chip: called by the application from the interrupt of the pin wired
	 * to the interrupt output of the chip. MCU RTC: called by its interrupt.
	 * Calls the event handler with the events of the RTC.
	 */
	virtual void IntHandler() {}

	/**
	 * @brief	Set the date and time from the seconds since the epoch
	 *
	 * The calendar takes the local time of Time (localtime_r).
	 *
	 * @param	Time : Seconds since the epoch
	 *
	 * @return	false with a time out of the calendar range
	 */
	bool SetTimeEpoch(time_t Time);

	/**
	 * @brief	Read the seconds since the epoch, with the fraction of second
	 *
	 * The calendar is converted as a local time (mktime).
	 *
	 * @param	Time : Seconds since the epoch and nanoseconds
	 *
	 * @return	true - success
	 */
	bool GetTimeEpoch(struct timespec &Time);

	/// The calendar has been set since the RTC started
	bool TimeValid() const { return vbTimeValid; }

	/// Clock in use, RTC_CLKSRC_LFXTAL or RTC_CLKSRC_LFRC
	RTC_CLKSRC ClockSource() const { return vClkSrc; }

	/// Frequency of the clock in use, Hz
	uint32_t ClockFrequency() const { return vClkFreq; }

	/// Number of alarms
	int NbAlarm() const { return vNbAlarm; }

	/// Number of 32 bit backup registers
	int NbBackup() const { return vNbBackup; }

	/// Wakeup period in effect, msec, 0 off
	uint32_t Wakeup() const { return vmsWakeup; }

protected:
	/// Event handler call, RTC_EVT_xxx bits
	void RtcEvt(uint32_t Evt) { if (vRtcEvtHandler != nullptr) vRtcEvtHandler(this, Evt); }

	RTC_CLKSRC vClkSrc = RTC_CLKSRC_DEFAULT;	//!< Clock in use
	uint32_t vClkFreq = 0;					//!< Frequency of that clock, Hz
	int vNbAlarm = 0;						//!< Number of alarms
	int vNbBackup = 0;						//!< Number of 32 bit backup registers
	uint32_t vmsWakeup = 0;					//!< Wakeup period in effect, msec
	bool vbTimeValid = false;				//!< Calendar set since the RTC started
	RtcEvtHandler_t vRtcEvtHandler = nullptr;	//!< Event handler
};

#endif

/** @} */

#endif // __RTC_H__
