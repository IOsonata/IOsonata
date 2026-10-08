/**-------------------------------------------------------------------------
@file	gnss.h

@brief	Generic GNSS receiver (GPS, Galileo, QZSS, ...).

A GNSS receiver is a device on a DeviceIntrf: a receiver chip on a UART, I2C
or SPI interface of any MCU, or a receiver built into the MCU on the
interface of its internal link (the IPC interface of the nRF91 modem). Each
receiver driver derives from Gnss, the same way a sensor driver derives from
its sensor type class:

	GnssXxx g_Gnss;

	g_Gnss.Init(cfg, &g_Intrf, &g_Timer);	// receiver on, configured, search started
	...										// GNSS_EVT_FIX arrives through cfg.EvtHandler
	g_Gnss.Status().Fix;					// last fix

The interface reports what the receiver sends through the event callback of
its configuration (EvtCB), from its interrupt. The application passes the
events of the receiver interface to IntrfEvtHandler:

	int IntrfEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
	{
		return g_Gnss.IntrfEvtHandler(EvtId, pBuffer, Len);
	}

Disable stops the receiver, Enable starts it again with the configuration of
Init, Reset does both. FixInterval selects the mode: 1 for a fix every
second, 0 for a single fix after which the receiver stops, more for a fix
every FixInterval seconds with the receiver off in between.

Work leaves the receiver interrupt through GnssEvtQue, the same way UsbEvtQue
and BtEvtQue do: bare metal it runs from the application event queue
(AppRun), with an RTOS from the thread that serves GNSS. The configuration
EvtHandler is called from there, never from the interrupt.

Each receiver update leaves the latest values: an update that arrives before
the previous one was handled replaces it, and a valid position waiting is kept
for the status even when a later update has none. Events other than updates
are reported once each. What waits together is reported in this order:
GNSS_EVT_BLOCKED and GNSS_EVT_UNBLOCKED (the last one reported last), the
update, GNSS_EVT_TIMEOUT. GNSS_EVT_FAULT drops what waits with it.

GnssFix_t uses the field names and units of NavGnssMeas_t (motion/nav.h): a
fix fills the GNSS aiding measurement of a navigation filter field by field.
The time stamp comes from the timer given to Init.

What is the same for every receiver lives in src/gnss/gnss.cpp: the status,
the events and the work out of the interrupt. A driver implements Init,
Enable, Disable, Reset, GetSat and IntrfEvtHandler, starts Init with
CoreInit, and reports from IntrfEvtHandler with UpdatePut and EvtPut.
Drivers: GnssNrf91 (gnss_nrf91.h), the receiver of the nRF91 modem on
ModemIpcIntrf.

@author	Hoang Nguyen Hoan
@date	Oct. 8, 2026

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
#ifndef __GNSS_H__
#define __GNSS_H__

#include <stdint.h>
#include <stdbool.h>

#include "device.h"
#include "coredev/timer.h"

/** @addtogroup GNSS
  * @{
  */

/// Satellite systems, a bit each
typedef enum __Gnss_Sys {
	GNSS_SYS_GPS = 1,					//!< GPS (USA)
	GNSS_SYS_GALILEO = 2,				//!< Galileo (EU)
	GNSS_SYS_QZSS = 4,					//!< QZSS (Japan)
	GNSS_SYS_GLONASS = 8,				//!< GLONASS (Russia)
	GNSS_SYS_BEIDOU = 0x10,				//!< BeiDou (China)
} GNSS_SYS;

/// Motion model of the receiver
typedef enum __Gnss_Dyn {
	GNSS_DYN_GENERAL,					//!< General purpose
	GNSS_DYN_STATIONARY,				//!< Not moving
	GNSS_DYN_PEDESTRIAN,				//!< Walking speed
	GNSS_DYN_AUTOMOTIVE,				//!< Road vehicle
} GNSS_DYN;

/// Events to the configuration EvtHandler
typedef enum __Gnss_Evt {
	GNSS_EVT_FIX,						//!< Update with a valid position, in Fix
	GNSS_EVT_NOFIX,						//!< Update without a position: satellites in the status,
										//!< Fix keeps the last valid one
	GNSS_EVT_TIMEOUT,					//!< No fix within FixTimeout: a single fix stops, periodic
										//!< fixes wait for the next period
	GNSS_EVT_BLOCKED,					//!< Receiver held off for now, its radio is in use
	GNSS_EVT_UNBLOCKED,					//!< Receiver running again
	GNSS_EVT_FAULT,						//!< Receiver stopped by a fault: Init starts it again
} GNSS_EVT;

#pragma pack(push, 4)

/// UTC date and time
typedef struct __Gnss_Time {
	uint16_t Year;						//!< 4 digits
	uint8_t Month;						//!< 1 to 12
	uint8_t Day;						//!< 1 to 31
	uint8_t Hour;						//!< 0 to 23
	uint8_t Min;						//!< 0 to 59
	uint8_t Sec;						//!< 0 to 60, 60 in a leap second
	uint16_t Msec;						//!< 0 to 999
} GnssTime_t;

/// Position fix. Names and units of NavGnssMeas_t (motion/nav.h).
typedef struct __Gnss_Fix {
	uint64_t Timestamp;					//!< Time stamp count in usec, from the device timer,
										//!< 0 without one
	GnssTime_t Time;					//!< UTC time of the fix
	double Lat;							//!< Latitude, deg, WGS-84
	double Lon;							//!< Longitude, deg, WGS-84
	float Alt;							//!< Altitude above the WGS-84 ellipsoid, m
	float PosAccH;						//!< Horizontal position std-dev, m
	float PosAccV;						//!< Vertical position std-dev, m
	float Vel[3];						//!< NED velocity, m/s (valid when bVelValid)
	float VelAcc;						//!< Velocity std-dev, m/s
	bool bVelValid;						//!< Vel and VelAcc are usable
	float Hdop;							//!< Horizontal dilution of precision
	float Vdop;							//!< Vertical dilution of precision
	uint8_t NbSat;						//!< Satellites used
} GnssFix_t;

/// Receiver state, as the updates and events report it
typedef struct __Gnss_Status {
	bool bFix;							//!< The last update had a valid position
	bool bBlocked;						//!< Receiver held off for now
	uint8_t NbSatTracked;				//!< Satellites tracked at the last update
	uint8_t NbSatUsed;					//!< Satellites used at the last update
	uint32_t NbFix;						//!< Updates with a position since Init, 0 while Fix is
										//!< empty
	GnssFix_t Fix;						//!< Last valid fix, kept by updates without one
} GnssStatus_t;

/// A satellite of the last update
typedef struct __Gnss_Sat {
	GNSS_SYS Sys;						//!< System, a single bit
	uint16_t Id;						//!< Satellite number in its system
	uint16_t Cn0;						//!< Carrier to noise density, 0.1 dB-Hz
	int16_t Elev;						//!< Elevation, deg
	int16_t Azim;						//!< Azimuth, deg
	bool bUsed;							//!< Used in the fix
} GnssSat_t;

#pragma pack(pop)

/// Deferred GNSS work: runs outside the interrupt with the values it was
/// queued with. Same signature as AppEvtHandler_t.
typedef void (*GnssEvtQueHandler_t)(uint32_t EvtId, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Queue deferred GNSS work for execution outside the interrupt.
 *
 * Called from the receiver interrupt, and by GnssCheckStatus from the thread
 * that runs the GNSS work, so an override must take both. The library has a
 * weak default for an application without an OS: it puts the work in the
 * application event queue (AppEvtHandlerQue), which the main loop runs.
 *
 * An application using an RTOS defines its own GnssEvtQue. It stores the
 * three values in the queue of the thread that serves GNSS, and that thread
 * calls Handler(EvtId, pCtx) for each one, and GnssCheckStatus when it is
 * idle.
 *
 * @param	EvtId	: Value to pass to Handler
 * @param	pCtx	: Value to pass to Handler
 * @param	Handler	: Function to call outside the interrupt
 *
 * @return	true - queued
 * 			false - queue full, GnssCheckStatus queues it again
 */
bool GnssEvtQue(uint32_t EvtId, void *pCtx, GnssEvtQueHandler_t Handler);

/// Queue again the work GnssEvtQue refused, for every receiver. AppRun calls
/// it when idle.
void GnssCheckStatus(void);

#ifdef __cplusplus
}

class Gnss;

/**
 * @brief	Receiver event handler.
 *
 * Called from the queued GNSS work, never from the interrupt.
 *
 * @param	pGnss	: The receiver
 * @param	Evt		: Event
 * @param	pStatus	: Receiver state after the event
 */
typedef void (*GnssEvtHandler_t)(Gnss * const pGnss, GNSS_EVT Evt, const GnssStatus_t * const pStatus);

#pragma pack(push, 4)

/// Receiver configuration
typedef struct __Gnss_Cfg {
	uint32_t DevAddr;					//!< I2C address or SPI chip select index of a receiver
										//!< chip, unused otherwise
	uint32_t SysMask;					//!< GNSS_SYS bits of the systems to use, 0 to keep the
										//!< receiver setting
	uint32_t FixInterval;				//!< s between fixes: 1 every second, 0 a single fix, more
										//!< for periodic fixes (the receiver may have a minimum)
	uint32_t FixTimeout;				//!< Longest search for a single or periodic fix in s,
										//!< 0 for no limit
	GNSS_DYN Dyn;						//!< Motion model
	const char * const *ppCmd;			//!< Receiver specific commands sent before the start,
										//!< board settings such as the antenna and LNA control,
										//!< NULL for none. Used by Init only.
	int NbCmd;							//!< Number of entries in ppCmd
	GnssEvtHandler_t EvtHandler;		//!< Receiver events, may be NULL
} GnssCfg_t;

#pragma pack(pop)

/// GNSS receiver base class. A receiver driver derives from it.
class Gnss : public Device {
public:
	Gnss();
	virtual ~Gnss();

	/**
	 * @brief	Initialize the receiver, configure it and start the search.
	 *
	 * Fixes are reported later by GNSS_EVT_FIX. Device Enable starts the
	 * receiver again with this configuration, Disable stops it.
	 *
	 * @param	Cfg		: Configuration, copied
	 * @param	pIntrf	: Interface of the receiver, initialized
	 * @param	pTimer	: Time base of the time stamps, nullptr for none
	 *
	 * @return	true - search started
	 */
	virtual bool Init(const GnssCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = nullptr) = 0;

	/**
	 * @brief	Event of the receiver interface.
	 *
	 * Called by the application from the event callback of the receiver
	 * interface, in its interrupt context.
	 *
	 * @param	EvtId	: Interface event
	 * @param	pBuffer	: Event data, see the interface
	 * @param	Len		: Size of pBuffer
	 *
	 * @return	Number of bytes processed
	 */
	virtual int IntrfEvtHandler(DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len) = 0;

	/**
	 * @brief	Satellites of the last update.
	 *
	 * Interrupt safe.
	 *
	 * @param	pSat	: Receives up to MaxSat satellites
	 * @param	MaxSat	: Size of pSat
	 *
	 * @return	Number of satellites written
	 */
	virtual int GetSat(GnssSat_t *pSat, int MaxSat) = 0;

	/// Receiver state as last reported
	const GnssStatus_t &Status() const { return vStatus; }

	/// Queue again the work GnssEvtQue refused. GnssCheckStatus calls it.
	void CheckStatus();

	friend void GnssCheckStatus(void);

protected:

	/**
	 * @brief	Check and keep the configuration, keep the interface and the
	 * 			timer, clear the status. First call of a driver Init.
	 *
	 * @return	false - invalid configuration or no interface
	 */
	bool CoreInit(const GnssCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer);

	/**
	 * @brief	A receiver update. Interrupt safe, the fix is copied and time
	 * 			stamped.
	 *
	 * @param	bFix		: The update has a valid position
	 * @param	pFix		: The position, read only when bFix
	 * @param	NbTracked	: Satellites tracked
	 * @param	NbUsed		: Satellites used
	 */
	void UpdatePut(bool bFix, const GnssFix_t * const pFix, int NbTracked, int NbUsed);

	/**
	 * @brief	A receiver event other than an update. Interrupt safe.
	 *
	 * @param	Evt : GNSS_EVT_TIMEOUT, GNSS_EVT_BLOCKED, GNSS_EVT_UNBLOCKED or
	 * 				  GNSS_EVT_FAULT
	 */
	void EvtPut(GNSS_EVT Evt);

	GnssCfg_t vCfg;						//!< Configuration of Init

private:
	static void WorkEvt(uint32_t EvtId, void *pCtx);
	void WorkQue();
	void Evt(GNSS_EVT Evt);

	GnssStatus_t vStatus;
	GnssFix_t vUpdFix;					// Last valid position not yet in the status
	uint32_t vUpdNbFix;					// Updates with a position not yet counted
	uint8_t vUpdTracked;				// Satellites of the last update
	uint8_t vUpdUsed;
	bool vbUpd;							// An update is waiting
	bool vbUpdFix;						// The last update has a valid position
	bool vbBlockedIn;					// Last of GNSS_EVT_BLOCKED and GNSS_EVT_UNBLOCKED
	uint32_t vEvtPend;					// Bit of each event waiting
	volatile uint8_t vWorkState;
	Gnss *vpNext;						// Receivers GnssCheckStatus checks
};

#endif

/** @} End of group GNSS */

#endif
