/**-------------------------------------------------------------------------
@file	usb.h

@brief	Generic USB layer.

One master init for a USB controller, then the device class inits, the
same shape as BtAppInit followed by service registration.

	UsbInit(&cfg);			// controller, protocol engine, identity
	cdc.Init(cdccfg);			// one call per class instance
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

#ifndef USB_CONFIG_DESC_MAXLEN
#define USB_CONFIG_DESC_MAXLEN		1024U
#endif

/// Bus speed. What the controller can do is USB_HIGHSPEED_CAPABLE() at compile
/// time. This is what enumeration actually negotiated.
typedef enum __Usb_Speed {
	USB_SPEED_FULL,				//!< 12 Mbit/s
	USB_SPEED_HIGH				//!< 480 Mbit/s
} UsbSpeed_t;

/// Operating role. Host and OTG need a dual role controller; UsbInit rejects
/// them on device only silicon.
typedef enum __Usb_Mode {
	USB_MODE_DEVICE,			//!< Peripheral, responds to a host
	USB_MODE_HOST,				//!< Host, enumerates and drives devices
	USB_MODE_OTG				//!< Dual role, starts from the ID pin
} UsbMode_t;

/// Cable events. Reported from UsbProcess, never from an interrupt.
typedef enum __Usb_Evt {
	USB_EVT_ATTACHED,			//!< Bus power appeared
	USB_EVT_DETACHED			//!< Bus power went away
} UsbEvt_t;

typedef void (*UsbEvtHandler_t)(int DevNo, UsbEvt_t Evt);

//
// Device class layer. One registration per class instance.
// Non-control endpoint events go directly from the controller to the endpoint
// callback registered with UsbCtrlrEpRegister; they are not class events.
//

/// Control transfer stage a request handler is being called for.
typedef enum __Usb_Ctrl_Stage {
	USB_CTRL_SETUP,
	USB_CTRL_DATA,
	USB_CTRL_COMPLETE,
	USB_CTRL_ABORT,				//!< Transfer abandoned, drop anything staged
} UsbCtrlStage_t;

#pragma pack(push, 4)

/// Everything UsbInit needs. Endpoint zero packet size and maximum speed are
/// not here, they come from usb_ctrlr.h for this DevNo.
typedef struct __Usb_Config {
	int DevNo;						//!< USB controller number, not the USB device address
	UsbMode_t Mode;					//!< Device, host or OTG. Host and OTG need dual role silicon
	uint16_t Vid;					//!< USB vendor id
	uint16_t Pid;					//!< USB product id
	uint16_t DevVer;				//!< Device release, BCD, 0x0100 is version 1.00
	const char *pManufacturer;		//!< Manufacturer string, NULL for none
	const char *pProduct;			//!< Product string, NULL for none
	const char *pSerial;			//!< Serial string, NULL to take the MCU unique id
	const char *pFuncName;			//!< Function name string, NULL for none
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	uint8_t DeviceClass;			//!< Device descriptor class, zero uses interface classes
	uint8_t DeviceSubClass;		//!< Device descriptor subclass
	uint8_t DeviceProtocol;		//!< Device descriptor protocol
	bool bSelfPowered;				//!< true - Device does not draw from the bus
	bool bRemoteWakeup;			//!< true - Device advertises remote wakeup support
	bool bLowPowerSuspend;			//!< true - Sit in USB low power while the host
									//!< suspends. Leave false on a device that has
									//!< to come back without a power cycle
	uint16_t MaxPower;				//!< Bus current drawn in mA, ignored when self powered
	UsbEvtHandler_t EvtHandler;		//!< Cable event callback, may be NULL
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
 * hardware power and clock path, and leaves the bus alone. Device classes are
 * registered after this and before UsbEnable. Fails when DevNo is not a
 * controller this target has.
 */
bool UsbInit(const UsbCfg_t *pCfg);

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

/**
 * @brief Return a descriptor assembled by the generic device layer.
 *
 * Device, qualifier and string descriptors come from UsbCfg_t. Configuration
 * descriptors are a generic header followed by class fragments registered
 * during class Init(). Applications normally do not call this directly.
 */
const uint8_t *UsbGetDescriptor(int DevNo, uint8_t Type, uint8_t Index,
								 uint16_t LangId, UsbSpeed_t Speed,
								 uint16_t *pLength);

//

#ifdef __cplusplus
}

/// Common lifecycle for statically owned USB class objects.
/// Role-specific routing is provided by the device and host class bases.
class UsbClass {
public:
	UsbClass(const UsbClass &) = delete;
	UsbClass &operator = (const UsbClass &) = delete;

	virtual void Reset() {}
	virtual void Process() {}

protected:
	UsbClass() = default;
	~UsbClass() = default;
};

/// Base for a class implemented by the local USB device.
class UsbDeviceClass : public UsbClass {
public:
	/// Handle one class, vendor or interface descriptor control transfer.
	virtual bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
						 uint8_t **ppData, uint16_t *pLength) {
		(void)pSetup;
		(void)Stage;
		(void)ppData;
		(void)pLength;
		return false;
	}

	/// Select a device configuration, or zero for the unconfigured state.
	virtual bool SelectConfig(uint8_t ConfigValue) {
		(void)ConfigValue;
		return true;
	}

	/// Select an advertised option for an interface owned by this class.
	virtual bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) {
		(void)InterfaceNo;
		(void)Option;
		return false;
	}

	uint8_t FirstInterface(void) const { return vFirstInterface; }
	uint8_t InterfaceCount(void) const { return vInterfaceCount; }
	uint16_t EpInMask(void) const { return vEpInMask; }
	uint16_t EpOutMask(void) const { return vEpOutMask; }
	const uint8_t *Descriptor(UsbSpeed_t Speed) const {
		return Speed == USB_SPEED_HIGH ? vHsDescriptor : vFsDescriptor;
	}
	uint16_t DescriptorLength(UsbSpeed_t Speed) const {
		return Speed == USB_SPEED_HIGH ? vHsDescriptorLength :
			vFsDescriptorLength;
	}

protected:
	UsbDeviceClass() = default;
	~UsbDeviceClass() = default;

private:
	friend bool UsbClassRegister(int DevNo, UsbDeviceClass *pClass,
								 uint8_t FirstInterface, uint8_t InterfaceCount,
								 uint16_t EpInMask, uint16_t EpOutMask);
	friend bool UsbDescriptorRegister(int DevNo, UsbDeviceClass *pClass,
									 const void *pFsDescriptor,
									 uint16_t FsDescriptorLength,
									 const void *pHsDescriptor,
									 uint16_t HsDescriptorLength);

	uint8_t vFirstInterface = 0;
	uint8_t vInterfaceCount = 0;
	uint16_t vEpInMask = 0;
	uint16_t vEpOutMask = 0;
	const uint8_t *vFsDescriptor = nullptr;
	const uint8_t *vHsDescriptor = nullptr;
	uint16_t vFsDescriptorLength = 0;
	uint16_t vHsDescriptorLength = 0;
};

/// Base for a class driver used by the local USB host.
class UsbHostClass : public UsbClass {
protected:
	UsbHostClass() = default;
	~UsbHostClass() = default;
};

/// Atomically register one statically owned device class and its ownership.
bool UsbClassRegister(int DevNo, UsbDeviceClass *pClass,
					  uint8_t FirstInterface, uint8_t InterfaceCount,
					  uint16_t EpInMask, uint16_t EpOutMask);

/// Register the static configuration descriptor fragment owned by pClass.
/// Full-speed data is required. High-speed data is required only on a
/// high-speed-capable controller. The generic layer supplies the configuration
/// descriptor header and concatenates fragments in class registration order.
bool UsbDescriptorRegister(int DevNo, UsbDeviceClass *pClass,
						   const void *pFsDescriptor,
						   uint16_t FsDescriptorLength,
						   const void *pHsDescriptor = nullptr,
						   uint16_t HsDescriptorLength = 0);
#endif

/** @} End of group USB */

#endif	// __USB_H__
