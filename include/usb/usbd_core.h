/**-------------------------------------------------------------------------
@file	usbd_core.h

@brief	Native IOsonata USB device protocol core.

Implements the USB device state machine above usbd_ctrlr.h. The core owns
endpoint zero, Chapter 9 standard requests, configuration state, alternate
settings and class/vendor request dispatch. USB functions register their
interface range and callbacks; controller-specific details remain below this
layer.

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
#ifndef __USBD_CORE_H__
#define __USBD_CORE_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usbd.h"
#include "usb/usbd_ctrlr.h"
#include "usb/usb_def.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_CORE_FUNC_MAXCNT		8
#define USBD_CORE_INTRF_MAXCNT		16

/**
 * @brief Control-request callback stage.
 *
 * SETUP asks a USB function to accept the request and provide its data buffer.
 * DATA is called after the data stage completes. COMPLETE is called when the
 * status stage has been accepted by the controller.
 */
typedef enum __Usbd_Core_Ctrl_Stage {
	USBD_CORE_CTRL_SETUP,
	USBD_CORE_CTRL_DATA,
	USBD_CORE_CTRL_COMPLETE,
} UsbdCoreCtrlStage_t;

/**
 * @brief Descriptor provider.
 *
 * The returned memory must remain valid until the EP0 IN transfer completes.
 * The provider supplies the complete descriptor length in pLength; the core
 * applies wLength and any required control-transfer terminating ZLP.
 */
typedef const uint8_t *(*UsbdCoreDescHandler_t)(uint8_t DescType,
										uint8_t DescIndex,
										uint16_t LangId,
										UsbdSpeed_t Speed,
										uint16_t *pLength,
										void *pContext);

/**
 * @brief USB function control-request callback.
 *
 * On USBD_CORE_CTRL_SETUP:
 * - Device-to-host: ppData/pLength identify data to send.
 * - Host-to-device: ppData/pLength identify writable storage. pLength must be
 *   at least wLength or the request is stalled.
 * - No-data request: ppData may be NULL and pLength zero.
 *
 * On USBD_CORE_CTRL_DATA, pLength is the number of bytes transferred. On
 * USBD_CORE_CTRL_COMPLETE it is the final data-stage length.
 *
 * Callbacks execute from the USB interrupt and must remain bounded.
 */
typedef bool (*UsbdCoreRequestHandler_t)(const UsbSetupData_t *pSetup,
										 UsbdCoreCtrlStage_t Stage,
										 uint8_t **ppData,
										 uint16_t *pLength,
										 void *pContext);

/**
 * @brief Configuration callback.
 *
 * Configuration zero means unconfigured. A non-zero value asks the function
 * to open the endpoints belonging to that configuration. Returning false
 * stalls SET_CONFIGURATION and the core rolls every function back to zero.
 */
typedef bool (*UsbdCoreConfigHandler_t)(uint8_t Configuration,
										void *pContext);

/**
 * @brief Alternate-setting callback.
 *
 * The core tracks the selected alternate setting. The function validates and
 * applies a requested setting before the core acknowledges SET_INTERFACE.
 */
typedef bool (*UsbdCoreSetInterfaceHandler_t)(uint8_t InterfaceNo,
											 uint8_t Alternate,
											 void *pContext);

/**
 * @brief Non-control endpoint transfer completion callback.
 *
 * Called from the USB interrupt for an endpoint owned by the function.
 */
typedef void (*UsbdCoreXferHandler_t)(uint8_t EpAddr,
									 uint16_t Length,
									 UsbdCtrlrXferResult_t Result,
									 void *pContext);

/** @brief Bus-reset callback for one USB function. */
typedef void (*UsbdCoreResetHandler_t)(void *pContext);

/**
 * @brief Start-of-frame callback for one USB function.
 *
 * Called from the USB interrupt on every frame while the device is
 * configured. The core enables controller SOF events only when at least one
 * registered function provides this callback. Work must remain bounded.
 */
typedef void (*UsbdCoreSofHandler_t)(uint16_t FrameNo, void *pContext);

#pragma pack(push, 4)

typedef struct __Usbd_Core_Config {
	UsbdCoreDescHandler_t DescHandler;	//!< Device descriptor provider
	void *pDescContext;						//!< Descriptor callback context
	UsbdSpeed_t Speed;						//!< Current bus speed for descriptors
	uint8_t Ep0Mps;							//!< EP0 max packet size, 0 uses 64
} UsbdCoreCfg_t;

typedef struct __Usbd_Core_Function_Config {
	uint8_t FirstInterface;					//!< First interface owned by function
	uint8_t InterfaceCount;					//!< Number of interfaces, zero for none
	uint16_t EpInMask;						//!< IN endpoint ownership, bit n = endpoint n
	uint16_t EpOutMask;						//!< OUT endpoint ownership, bit n = endpoint n
	UsbdCoreRequestHandler_t RequestHandler;
	UsbdCoreConfigHandler_t ConfigHandler;
	UsbdCoreSetInterfaceHandler_t SetInterfaceHandler;
	UsbdCoreXferHandler_t XferHandler;
	UsbdCoreResetHandler_t ResetHandler;
	UsbdCoreSofHandler_t SofHandler;		//!< Optional, NULL when not needed
	void *pContext;
} UsbdCoreFuncCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the native USB device core and controller software state.
 */
bool UsbdCoreInit(const UsbdCoreCfg_t *pCfg);

/**
 * @brief Register one USB function with the device core.
 *
 * Registration is static and intended to be complete before the device is
 * connected to the bus. Endpoint zero is owned by the core and bit zero must
 * be clear in both endpoint masks. Endpoint masks may not overlap between
 * functions.
 */
bool UsbdCoreRegisterFunction(const UsbdCoreFuncCfg_t *pCfg);

/**
 * @brief Enable the controller interrupt and connect the device pull-up.
 */
void UsbdCoreStart(void);

/**
 * @brief Disconnect, disable the controller interrupt and clear device state.
 */
void UsbdCoreStop(void);

/**
 * @brief Request remote wakeup when the host enabled it and the bus is asleep.
 *
 * @return true when the request was issued.
 */
bool UsbdCoreRemoteWakeup(void);

/** @brief Return true after SET_CONFIGURATION selected a non-zero value. */
bool UsbdCoreConfigured(void);

/** @brief Return true while the bus is suspended. */
bool UsbdCoreSuspended(void);

/** @brief Current USB device address tracked by the core. */
uint8_t UsbdCoreAddress(void);

/** @brief Current configuration value, zero while unconfigured. */
uint8_t UsbdCoreConfiguration(void);

/** @brief Current alternate setting for an interface, zero when unavailable. */
uint8_t UsbdCoreAlternate(uint8_t InterfaceNo);

/** @brief Whether DEVICE_REMOTE_WAKEUP is currently enabled by the host. */
bool UsbdCoreRemoteWakeupEnabled(void);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CORE_H__
