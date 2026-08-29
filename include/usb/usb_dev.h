/**-------------------------------------------------------------------------
@file	usb_dev.h

@brief	Generic USB device.

The device level object : identity, descriptors, start and stop, and the pump
that turns the device stack over. Function level interfaces such as
UsbdCdcIntrf register themselves with it and are pumped from UsbDevProcess.

There is one USB device controller per MCU, so this is a single instance and
takes no handle.

@author	Hoang Nguyen Hoan
@date	Aug. 28, 2026

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
#ifndef __USB_DEV_H__
#define __USB_DEV_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usbd.h"

/** @addtogroup USBD
  * @{
  */

#define USBDEV_SERIAL_MAXLEN		33	//!< 32 hexadecimal characters and a terminator

#pragma pack(push, 4)

typedef struct __Usb_Dev_Config {
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
									//!< suspends. Leave false on a device that has to
									//!< come back without a power cycle
	uint16_t MaxPower;				//!< Bus current drawn in mA, ignored when self powered
} UsbDevCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Configure the USB device.
 *
 * Records the identity, configures the port layer and leaves the bus
 * alone. Function interfaces are registered after this and before Enable.
 *
 * @param	pCfg : Configuration
 *
 * @return	true - Success
 */
bool UsbDevInit(const UsbDevCfg_t *pCfg);

/**
 * @brief	Bring the device onto the bus.
 *
 * Starts the hardware, then the device stack. Returns false when there is no
 * bus power, which is not a failure : the attach event brings it back and
 * UsbDevProcess retries. Idempotent.
 *
 * @return	true - The device stack is running
 */
bool UsbDevEnable(void);

/**
 * @brief	Take the device off the bus and power the controller down.
 */
void UsbDevDisable(void);

/**
 * @brief	Turn the device stack over and move data for every registered
 *			function interface.
 *
 * Call this from the main loop or from a thread, never from an interrupt.
 * Nothing moves in either direction without it.
 */
void UsbDevProcess(void);

/**
 * @brief	Register a function interface to be pumped.
 *
 * A class interface such as UsbdCdcIntrf calls this from its own Init so the
 * application has one thing to call in its loop. The device level knows
 * nothing about the class, which is why the callback is opaque : adding a
 * mass storage or HID interface later adds no code here.
 *
 * @param	Pump : Called from UsbDevProcess with pCtx
 * @param	pCtx : Passed back to Pump untouched
 *
 * @return	true - Registered
 *			false - No room left
 */
bool UsbDevRegisterFunc(void (*Pump)(void *pCtx), void *pCtx);

/**
 * @brief	Whether the host has configured the device.
 *
 * @return	true - Enumerated and configured
 */
bool UsbDevMounted(void);

/**
 * @brief	Whether the bus is suspended.
 *
 * @return	true - Suspended
 */
bool UsbDevSuspended(void);

/**
 * @brief	Configuration in use, for the descriptor layer.
 *
 * @return	Pointer to the configuration, NULL before Init
 */
const UsbDevCfg_t *UsbDevGetCfg(void);

/**
 * @brief	Serial number string in use.
 *
 * Either the string given at Init or the one derived from the MCU unique id.
 *
 * @return	Pointer to a terminated string, never NULL after Init
 */
const char *UsbDevGetSerial(void);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USB_DEV_H__
