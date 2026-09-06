/**-------------------------------------------------------------------------
@file	usb.h

@brief	Generic USB layer.

One master init for a USB controller, then the class or function inits, the
same shape as BtAppInit followed by service registration.

	UsbInit(&cfg);			// controller, protocol engine, identity
	UsbCdcInit(&cdc, &cdccfg);	// one call per function
	UsbEnable();			// connect to the bus

Everything below this header that does not change from one target to the next
lives here. The UsbCtrlr port entry points live in usb_ctrlr.h. What
does change per target is in the port's own usb_ctrlr.h, the same way
iopincfg.h declares the pin API once and each port supplies iopinctrl.h.

DevNo selects the controller on parts that have more than one, matching the
DevNo convention of UARTCfg_t, SPICfg_t and I2CCfg_t. It is the controller
index, never the USB device address assigned by SET_ADDRESS.

Packet sizes and endpoint counts are compile-time values from usb_ctrlr.h, so
an application sizes its CFifo and DMA staging memory statically before any
endpoint exists.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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
#ifndef __USB_H__
#define __USB_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usb/usb_def.h"
#include "usb_ctrlr.h"			// Port supplied, describes this target

/** @addtogroup USB
  * @{
  */

/// Bus signaling rate in bits per second, what DeviceIntrf Rate reports. This
/// is the line rate, not a payload figure. Bulk moves less because the host
/// schedules the bus and every transaction pays token and handshake overhead.
#define USB_LINK_RATE_FULL			12000000U
#define USB_LINK_RATE_HIGH			480000000U

/// Bus speed. What the controller can do is USB_HIGHSPEED_CAPABLE() at compile
/// time. This is what enumeration actually negotiated.
typedef enum __Usb_Speed {
	USB_SPEED_FULL,				//!< 12 Mbit/s
	USB_SPEED_HIGH				//!< 480 Mbit/s
} UsbSpeed_t;

/// Cable events. Reported from UsbProcess, never from an interrupt.
typedef enum __Usb_Evt {
	USB_EVT_ATTACHED,			//!< Bus power appeared
	USB_EVT_DETACHED			//!< Bus power went away
} UsbEvt_t;

typedef void (*UsbEvtHandler_t)(int DevNo, UsbEvt_t Evt);

//
// Function layer. One registration per class or vendor function.
//

/// Control transfer stage a request handler is being called for.
typedef enum __Usb_Ctrl_Stage {
	USB_CTRL_SETUP,
	USB_CTRL_DATA,
	USB_CTRL_COMPLETE,
	USB_CTRL_ABORT,				//!< Transfer abandoned, drop anything staged
} UsbCtrlStage_t;

typedef const uint8_t *(*UsbDescHandler_t)(uint8_t DescType, uint8_t DescIndex,
										   uint16_t LangId, UsbSpeed_t Speed,
										   uint16_t *pLength, void *pContext);

typedef bool (*UsbRequestHandler_t)(const UsbSetupData_t *pSetup,
									UsbCtrlStage_t Stage, uint8_t **ppData,
									uint16_t *pLength, void *pContext);

typedef bool (*UsbConfigHandler_t)(uint8_t Configuration, void *pContext);
typedef bool (*UsbSetInterfaceHandler_t)(uint8_t InterfaceNo, uint8_t Alt,
										 void *pContext);
typedef void (*UsbXferHandler_t)(uint8_t EpAddr, uint16_t Length,
								 UsbCtrlrXferResult_t Result, void *pContext);
typedef void (*UsbResetHandler_t)(void *pContext);
typedef void (*UsbSofHandler_t)(uint16_t FrameNo, void *pContext);

/// Polled from UsbProcess in application context. Work a function cannot do
/// inside the USB interrupt goes here.
typedef void (*UsbProcessHandler_t)(void *pContext);

#pragma pack(push, 4)

/// One USB function. Endpoint zero belongs to the generic layer, so bit zero
/// must be clear in both masks, and masks may not overlap between functions.
typedef struct __Usb_Func_Config {
	uint8_t FirstInterface;			//!< First interface owned by function
	uint8_t InterfaceCount;			//!< Number of interfaces, zero for none
	uint16_t EpInMask;				//!< IN endpoint ownership, bit n = endpoint n
	uint16_t EpOutMask;				//!< OUT endpoint ownership, bit n = endpoint n
	UsbRequestHandler_t RequestHandler;
	UsbConfigHandler_t ConfigHandler;
	UsbSetInterfaceHandler_t SetInterfaceHandler;
	UsbXferHandler_t XferHandler;
	UsbResetHandler_t ResetHandler;
	UsbSofHandler_t SofHandler;		//!< Optional, NULL when not needed
	UsbProcessHandler_t ProcessHandler;	//!< Optional, polled from UsbProcess
	void *pContext;
} UsbFuncCfg_t;

/// Everything UsbInit needs. Endpoint zero packet size and maximum speed are
/// not here, they come from usb_ctrlr.h for this DevNo.
typedef struct __Usb_Config {
	int DevNo;						//!< USB controller number, not the USB device address
	uint16_t Vid;					//!< USB vendor id
	uint16_t Pid;					//!< USB product id
	uint16_t DevVer;				//!< Device release, BCD, 0x0100 is version 1.00
	const char *pManufacturer;		//!< Manufacturer string, NULL for none
	const char *pProduct;			//!< Product string, NULL for none
	const char *pSerial;			//!< Serial string, NULL to take the MCU unique id
	const char *pFuncName;			//!< Function name string, NULL for none
	int NbCdc;						//!< Number of CDC ACM functions, 0 uses 1
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	bool bSelfPowered;				//!< true - Device does not draw from the bus
	bool bLowPowerSuspend;			//!< true - Sit in USB low power while the host
									//!< suspends. Leave false on a device that has
									//!< to come back without a power cycle
	uint16_t MaxPower;				//!< Bus current drawn in mA, ignored when self powered
	UsbEvtHandler_t EvtHandler;		//!< Cable event callback, may be NULL
	UsbDescHandler_t DescHandler;	//!< NULL uses the built in descriptor builder
	void *pDescContext;				//!< Descriptor callback context
} UsbCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

//
// Generic layer. Implemented once in src/usb/usb.cpp.
//

/**
 * @brief	Bring up one USB controller and its protocol engine.
 *
 * Records the identity, initializes the controller software state and the
 * hardware power and clock path, and leaves the bus alone. Functions are
 * registered after this and before UsbEnable. Fails when DevNo is not a
 * controller this target has.
 */
bool UsbInit(const UsbCfg_t *pCfg);

/**
 * @brief	Register one USB function.
 *
 * Registration is static and expected to be complete before the device is
 * connected to the bus.
 */
bool UsbRegisterFunc(int DevNo, const UsbFuncCfg_t *pCfg);

/** @brief Enable the controller interrupt and connect the bus pull-up. */
bool UsbEnable(int DevNo);

/** @brief Disconnect, disable the interrupt and clear state. */
void UsbDisable(int DevNo);

/** @brief Cable and housekeeping pass. Call from the application loop. */
void UsbProcess(int DevNo);

/** @brief Speed enumeration settled on, valid once configured. */
UsbSpeed_t UsbGetSpeed(int DevNo);

bool UsbConfigured(int DevNo);
bool UsbSuspended(int DevNo);
uint8_t UsbGetAddress(int DevNo);
uint8_t UsbGetConfiguration(int DevNo);
uint8_t UsbGetAlternate(int DevNo, uint8_t InterfaceNo);
bool UsbRemoteWakeupEnabled(int DevNo);
bool UsbRemoteWakeup(int DevNo);

/** @brief Configuration this controller was initialized with, NULL before init. */
const UsbCfg_t *UsbGetCfg(int DevNo);

/** @brief Serial string in use, either the configured one or the MCU unique id. */
const char *UsbGetSerial(int DevNo);

//

#ifdef __cplusplus
}
#endif

/** @} End of group USB */

#endif	// __USB_H__
