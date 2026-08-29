/**-------------------------------------------------------------------------
@file	usbd.h

@brief	USB device port layer.

Everything a USB device controller needs from the MCU around the device stack :
voltage regulator, clock, cable detect, controller start/stop sequencing,
interrupt priority and the unique id used for the serial number string.

One implementation per MCU family, selected by which port library is linked.
Nothing above this header knows which part it is running on.

The controller backend owns endpoint and transfer handling. This layer owns
the MCU-specific power and controller sequencing around it, so the same
interface serves a full speed controller and a high speed one without either
knowing about the other.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#ifndef __USBD_H__
#define __USBD_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @addtogroup USBD
  * @{
  */

/// Bus speed the controller is able to run at.
typedef enum __Usbd_Speed {
	USBD_SPEED_FULL,			//!< 12 Mbit/s
	USBD_SPEED_HIGH			//!< 480 Mbit/s
} UsbdSpeed_t;

/// Static capabilities of the USB device controller instance.
typedef struct __Usbd_Capabilities {
	UsbdSpeed_t MaxSpeed;		//!< Fastest bus speed supported
	uint8_t EpInCnt;			//!< IN endpoint numbers including endpoint zero
	uint8_t EpOutCnt;			//!< OUT endpoint numbers including endpoint zero
	uint8_t Ep0Mps;			//!< Endpoint zero maximum packet size
} UsbdCaps_t;

/// Cable events. Reported from UsbdProcess, never from an interrupt.
typedef enum __Usbd_Evt {
	USBD_EVT_ATTACHED,		//!< Bus power appeared
	USBD_EVT_DETACHED			//!< Bus power went away
} UsbdEvt_t;

typedef void (*UsbdEvtHandler_t)(UsbdEvt_t Evt);

#pragma pack(push, 4)

typedef struct __Usbd_Config {
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	bool bLowPowerSuspend;			//!< true - Let the part sit in USB low power when
									//!< the host suspends. false - Leave low power on
									//!< every pass, which costs suspend current and
									//!< removes a resume path that is easy to get wrong
	UsbdEvtHandler_t EvtHandler;	//!< Cable event callback, may be NULL
} UsbdCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Return static controller capabilities.
 *
 * Endpoint counts are direction-specific and include endpoint zero. The port
 * reports the instantiated controller resources rather than an architectural
 * maximum so common USB code can size its function topology correctly on
 * every MCU.
 *
 * @return	Pointer to the immutable capability record
 */
const UsbdCaps_t *UsbdGetCaps(void);

/**
 * @brief	Configure the USB hardware without powering it.
 *
 * Records the interrupt priority and the callback, arms whatever the part
 * uses to detect bus power, and leaves the controller off. Safe to call with
 * no cable attached, which is the ordinary state of a battery powered board.
 *
 * @param	pCfg : Configuration
 *
 * @return	true - Success
 */
bool UsbdInit(const UsbdCfg_t *pCfg);

/**
 * @brief	Bring power and clock up to the point the device stack may run.
 *
 * Starts the USB voltage regulator, requests the high frequency crystal and
 * holds it, and waits for the part to report itself usable. On first start
 * the device stack is initialised after this returns true. On restart the
 * stack may already be initialised, but it remains disconnected until the
 * controller is ready.
 *
 * Failing because there is no bus power is not an error. The caller retries
 * when the attach event arrives.
 *
 * @return	true - Ready for the device stack
 *			false - No bus power, or the part did not come up in time
 */
bool UsbdStart(void);

/**
 * @brief	Power the controller down and release the clock.
 *
 * Disables the controller interrupt, stops the hardware and releases the
 * clock held by UsbdStart. It does not deinitialise the device stack, so the
 * controller can be started again without rebuilding the software state.
 */
void UsbdStop(void);

/**
 * @brief	Per pass housekeeping, called from the same context as the device
 *			stack pump.
 *
 * Bus power is polled here and level changes are delivered through the cable
 * event callback. That avoids requiring a separate cable-detect interrupt
 * and keeps attach/detach handling in the same context as the stack pump.
 *
 * Anything the part will not do for itself belongs here as well, the exit
 * from USB low power being the one that costs a power cycle when it is
 * missing.
 */
void UsbdProcess(void);

/**
 * @brief	Whether bus power is present right now.
 *
 * @return	true - Bus power present
 */
bool UsbdVbusDetected(void);

/**
 * @brief	Fastest speed this controller supports.
 *
 * @return	Bus speed
 */
UsbdSpeed_t UsbdMaxSpeed(void);

/**
 * @brief	Request the high frequency crystal and hold it.
 *
 * USB needs the crystal for as long as the port is up. The port supplies a
 * default that asks the SoftDevice when one is running and drives the clock
 * directly otherwise.
 *
 * An application whose radio stack owns the clock peripheral defines these
 * two itself. An application object is always linked, so a definition there
 * replaces the library default with no build configuration involved.
 *
 * @return	true - The crystal is running
 */
bool UsbdXtalRequest(void);

/**
 * @brief	Release the high frequency crystal taken by UsbdXtalRequest.
 */
void UsbdXtalRelease(void);

/**
 * @brief	Serial number string from the MCU unique id.
 *
 * Written as an ASCII hexadecimal string, always terminated. The port knows
 * where its unique id lives, so the descriptor layer does not have to.
 *
 * @param	pBuff : Buffer to receive the string
 * @param	BuffLen : Size of the buffer in bytes, terminator included
 *
 * @return	Number of characters written, terminator excluded
 */
size_t UsbdGetSerial(char *pBuff, size_t BuffLen);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_H__
