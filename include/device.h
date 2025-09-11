/**-------------------------------------------------------------------------
@file	device.h

@brief	Generic device base class

This is the base class to implement all sort devices, hardware or software.
For example a sensor device or a software audio/video decoder.
The device can transfer data via its DeviceIntrf object.

Important NOTE : For performance, there is no pointer or
parameter validation at this low level layer.  It is the responsibility of
caller to pre-validate all access

@author	Hoang Nguyen Hoan
@date	Feb. 12, 2017

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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
#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "device_intrf.h"

/// @brief	Defines interrupt pin polarity of the device.
///
/// Many hardware devices can have interrupt pin polarity configurable.
typedef enum __Dev_Interrupt_Polarity {
	DEVINTR_POL_LOW,	//!< Interrupt pin active low
	DEVINTR_POL_HIGH	//!< Interrupt pin active high
} DEVINTR_POL;

/// @brief	Device data endianness
///
/// To indicate data byte order endianness
///
typedef enum __Dev_Data_Byte_Order {
	DEV_BYTEORDER_LITTLE,		//!< Data order little endian
	DEV_BYTEORDER_BIG,			//!< Data order big endian
} DEV_BYTEORDER;

typedef enum __Device_Event {
	DEV_EVT_DATA_RDY
} DEV_EVT;

#ifdef __cplusplus

class Device;

typedef void (*DevEvtHandler_t)(Device * const pDev, DEV_EVT Evt);
//typedef DevEvtHandler_t		DEVEVTCB;

/// @brief	Device base class.
///
/// Base class for both hardware and software devices (e.g., sensors or software audio decoders).
/// The device transfers data via its DeviceIntrf.
/// Important NOTE: For performance, there is no pointer or parameter validation
/// at this low-level layer. The caller must pre-validate all access.
class Device {
public:
	Device();
	virtual ~Device() {}

	//
	// *** Require implementations ***
	//

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true on success
	 */
	virtual bool Enable() = 0;

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * Put the device into the lowest power mode possible such that
	 * a subsequent Enable() can wake it without a full initialization.
	 */
	virtual void Disable() = 0;

	/**
	 * @brief	Reset device to its initial default state
	 */
	virtual void Reset() = 0;

	//
	// *** Optional implementations ***
	//

	/**
	 * @brief	Power off the device completely.
	 *
	 * If supported, this fully powers down the device. A full re-initialization
	 * is required to re-enable the device.
	 */
	virtual void PowerOff() {}

	/**
	 * @brief	Set the device's map address
	 *
	 * The address format depends on the interface/type. For I²C this is usually
	 * a 7-bit address; for SPI it may be a chip-select index; for MMIO, a 32-bit address.
	 *
	 * @param 	Addr : Device address or zero based chip select index
	 */
	virtual void DeviceAddress(uint32_t Addr) { vDevAddr =  Addr; }

	/**
	 * @brief	Get the device's map address
	 *
	 * @return	Address or chip select pin zero based index
	 */
	virtual uint32_t DeviceAddress() const { return vDevAddr; }

	/**
	 * @brief	Get device ID.
	 *
	 * This device id value is implementation specific.  It can store hardware
	 * device identifier or a serial number at the implementor’s discretion.
	 *
	 * @return	64 Bits device ID
	 */
	virtual uint64_t DeviceID() const { return vDevId; }

	/**
	 * @brief	Read the device's register/memory block.
	 *
	 * Default SPI convention: set bit 7 of the first command/address byte for READ.
	 * Override if your device uses a different SPI protocol.
	 *
	 * @param 	pCmdAddr 	: Command/address bytes to send before reading
	 * @param	CmdAddrLen 	: Length of @p pCmdAddr in bytes
	 * @param	pBuff		: Destination buffer
	 * @param	BuffLen		: Size of @p pBuff in bytes
	 *
	 * @return	Number of bytes read
	 */
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

	/**
	 * @brief	Write to device's register/memory block
	 *
	 * This default implementation clears bit 7 of the Cmd/Addr byte for SPI write access as most
	 * devices work this way on SPI interface.  Overwrite this implementation if SPI access is different
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior writing data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pData		: Data buffer to be written to the device
	 * @param	DataLen		: Size of data
	 *
	 * @return	Actual number of bytes written
	 */
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen);

	/**
	 * @brief	Read device's 8 bits register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to read
	 * @param	RegAddrLen	: Address buffer size
	 *
	 * @return	Data read
	 */
	virtual uint8_t Read8(uint8_t *pRegAddr, int RegAddrLen) {
		uint8_t val = 0;
		Read(pRegAddr, RegAddrLen, &val, 1);
		return val;
	}

	/**
	 * @brief	Read device's 16 bits register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to read
	 * @param	RegAddrLen	: Address buffer size
	 *
	 * @return	Data read
	 */
	virtual uint16_t Read16(uint8_t *pRegAddr, int RegAddrLen) {
		uint16_t val = 0;
		Read(pRegAddr, RegAddrLen,(uint8_t*) &val, 2);
		return val;
	}

	/**
	 * @brief	Read device's 32 bit register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to read
	 * @param	RegAddrLen	: Address buffer size
	 *
	 * @return	Data read
	 */
	virtual uint32_t Read32(uint8_t *pRegAddr, int RegAddrLen) {
		uint32_t val = 0;
		Read(pRegAddr, RegAddrLen, (uint8_t*)&val, 4);
		return val;
	}

	/**
	 * @brief	Write 8 bits data to device's register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to write
	 * @param	RegAddrLen	: Address buffer size
	 * @param	Data		: Data to be written to the device
	 *
	 * @return	true - Success
	 */
	virtual bool Write8(uint8_t *pRegAddr, int RegAddrLen, uint8_t Data) {
		return Write(pRegAddr, RegAddrLen, &Data, 1) > 0;
	}

	/**
	 * @brief	Write 16 bits data to device's register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to write
	 * @param	RegAddrLen	: Address buffer size
	 * @param	Data		: Data to be written to the device
	 *
	 * @return	true - Success
	 */
	virtual bool Write16(uint8_t *pRegAddr, int RegAddrLen, uint16_t Data) {
		return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 2) > 1;
	}

	/**
	 * @brief	Write 32 bits data to device's register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to write
	 * @param	RegAddrLen	: Address buffer size
	 * @param	Data		: Data to be written to the device
	 *
	 * @return	true - Success
	 */
	virtual bool Write32(uint8_t *pRegAddr, int RegAddrLen, uint32_t Data) {
		return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 4) > 3;
	}

	/**
	 * @brief	Return availability of the device
	 *
	 * Returns true if the device has been detected and is ready to use.
	 *
	 * @return	true if device is valid.
	 */
	bool Valid() { return vbValid; }

	DEVINTRF_TYPE InterfaceType() const { return vpIntrf != nullptr ? vpIntrf->Type() : DEVINTRF_TYPE_NULL; }

	/**
	 * @brief	Get timer pointer used for timestamping
	 *
	 * @return	Pointer to Timer object.
	 * 			Never delete the returned pointer. In embedded systems, objects
	 *          are typically static rather than dynamically allocated.
	 */
	virtual operator Timer *() const { return vpTimer; }	// Get device interface data (handle)

	void SetEvtHandler(DevEvtHandler_t EvtHandler) { vEvtHandler = EvtHandler; }
	virtual void EvtHandler(DEV_EVT Evt) { if (vEvtHandler) vEvtHandler(this, Evt); }

protected:

	void InterruptId(uint8_t IntId) { vIntId = IntId; }
	uint8_t InterruptId(void) const { return vIntId; }

	/**
	 * @brief	Store device id.
	 *
	 * This value is implementation-specific. It may store a hardware identifier
	 * or a serial number, at the implementor’s discretion.
	 *
	 * @param	DevId : Device id value to store
	 */
	void DeviceID(uint64_t DevId) { vDevId = DevId; }

	/**
	 * @brief	Set device validity.
	 *
	 * Set valid to true if device is detect and initialized.  Otherwise set it to false
	 *
	 */
	void Valid(bool bVal) { vbValid = bVal; }

	/**
	 * @brief	Set the device’s communication interface
	 *
	 * @param 	pIntrf : Pointer to a pre-initialized (typically static) interface.
	 */
	void Interface(DeviceIntrf * const pIntrf) { vpIntrf = pIntrf; }

	/**
	 * @brief	Get the device’s communication interface
	 *
	 * @return	Pointer to the interface in use, or nullptr
	 */
	DeviceIntrf* Interface() const { return vpIntrf; }

	void InterruptEnabled(bool En) { vbIntEn = En; }
	bool InterruptEnabled() const { return vbIntEn; }

	virtual DEV_BYTEORDER ByteOrder(DEV_BYTEORDER Val) { vByteOrder = Val; return vByteOrder; }
	virtual DEV_BYTEORDER ByteOrder(void) const { return vByteOrder; }
	operator DEV_BYTEORDER () const { return vByteOrder; }


	bool		vbValid;		//!< Device is detected/initialized and ready to use
	uint32_t 	vDevAddr;		//!< Device address or chip-select index
	DeviceIntrf *vpIntrf;		//!< Device interface
	Timer 		*vpTimer;		//!< Timer for timestamping or timer events
	uint64_t	vDevId;			//!< Implementation-specific device identifier (e.g., HW register or S/N)
	bool 		vbIntEn;		//!< Interrupts enabled
	DevEvtHandler_t	vEvtHandler;	//!< Event handler callback
	uint8_t		vIntId;			//!< If multiple interrupt pins, which one is used (0 = none)
	DEV_BYTEORDER vByteOrder; 	//!< Data endianness
};

extern "C" {
#endif	// __cplusplus


#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __DEVICE_H__
