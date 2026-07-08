/**-------------------------------------------------------------------------
@file	rftag_controller.h

@brief	RF tag controller facet

The controller is the active RF device, the reader or PCD side. It detects and
accesses remote tags over a DeviceIntrf transport, an external reader IC on
I2C, SPI or UART, or an MCU internal reader peripheral through an adapter.

RFTagController is an RFTag peer on the initiator side. It derives from Device
like the tag facet, reaches the transport through the DeviceIntrf, and reports
activity through a per instance EvtHandler override. The adapter command codes
and the memory command layout describe the IOsonata adapter protocol, not any
reader IC command format. A reader IC port translates them to its own
sequences.

@author	Hoang Nguyen Hoan
@date	Jul. 7, 2026

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
#ifndef __RFTAG_CONTROLLER_H__
#define __RFTAG_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "device.h"
#include "device_intrf.h"

#pragma pack(push, 4)

/// RF transport protocol identifiers, one bit per protocol for mask use.
typedef enum {
	RF_PROTO_NONE     = 0,
	RF_PROTO_NFCA     = (1u << 0),
	RF_PROTO_NFCB     = (1u << 1),
	RF_PROTO_NFCF     = (1u << 2),
	RF_PROTO_ISO15693 = (1u << 3),
	RF_PROTO_T4T      = (1u << 4),
	RF_PROTO_T5T      = (1u << 5),
	RF_PROTO_MIFARE   = (1u << 6),
} RF_PROTO;

/// Controller event codes reported to the EvtHandler.
typedef enum {
	RFTAGCTRL_EVT_FIELD_ON,			//!< RF field turned on
	RFTAGCTRL_EVT_FIELD_OFF,		//!< RF field turned off
	RFTAGCTRL_EVT_TAG_DETECTED,		//!< A tag answered detection
	RFTAGCTRL_EVT_TAG_REMOVED,		//!< A detected tag left the field
	RFTAGCTRL_EVT_RX_DATA,			//!< Data received from a tag
	RFTAGCTRL_EVT_TX_DONE,			//!< Data sent to a tag
	RFTAGCTRL_EVT_ERROR,			//!< Transport or protocol error
} RFTAGCTRL_EVT;

/// Detected tag description filled by Detect.
typedef struct {
	uint32_t Proto;					//!< Tag protocol, one of RF_PROTO
	uint8_t Uid[16];				//!< Tag UID
	uint8_t UidLen;					//!< UID length in bytes
	uint8_t Type;					//!< Protocol specific type code
	uint32_t Size;					//!< Tag memory size in bytes
	uint16_t BlockSize;				//!< Tag block size in bytes
	uint32_t Flags;					//!< Protocol specific flags
} RFTagInfo_t;

/// Controller configuration.
typedef struct {
	uint8_t DevAddr;				//!< Transport device address of the reader adapter
	uint32_t ProtoMask;				//!< Protocols to scan for, mask of RF_PROTO bits
	uint32_t Bitrate;				//!< Transport bitrate in Hertz, 0 leaves it unchanged
	uint32_t TimeoutMs;				//!< Operation timeout in milliseconds
} RFTagControllerCfg_t;

/**
 * @brief IOsonata RFTagController adapter command code.
 *
 * These commands are used by adapter implementations that expose a reader
 * through DeviceIntrf. Transceive is raw and does not wrap data with one of
 * these command values.
 */
typedef enum {
	RFTAGCTRL_CMD_DETECT = 1,
	RFTAGCTRL_CMD_SELECT,
	RFTAGCTRL_CMD_TAG_READ,
	RFTAGCTRL_CMD_TAG_WRITE,
} RFTAGCTRL_CMD;

/**
 * @brief IOsonata adapter command for remote tag memory access.
 *
 * This is not a chip command format. Reader IC ports translate this
 * structure to their own command sequence.
 */
typedef struct {
	uint8_t Cmd;
	uint8_t UidLen;
	uint16_t Len;
	uint32_t Proto;
	uint32_t Addr;
	uint8_t Uid[16];
} RFTagControllerMemCmd_t;

#pragma pack(pop)

class RFTagController : virtual public Device {
public:
	RFTagController() {
		memset(&vCfg, 0, sizeof(vCfg));
	}
	virtual ~RFTagController() {}

	/**
	 * @brief	Initialize the controller.
	 *
	 * @param	Cfg		Controller configuration
	 * @param	pIntrf	Transport interface the reader adapter sits on
	 *
	 * @return	true on success
	 */
	virtual bool Init(const RFTagControllerCfg_t &Cfg, DeviceIntrf * const pIntrf);

	/**
	 * @brief	Turn the reader transport on.
	 *
	 * @return	true on success
	 */
	virtual bool Enable();

	/**
	 * @brief	Turn the reader transport off.
	 */
	virtual void Disable();

	/**
	 * @brief	Reset the reader transport.
	 */
	virtual void Reset();

	/**
	 * @brief	Scan the field for a tag.
	 *
	 * @param	pTag	Filled with the detected tag description on success
	 *
	 * @return	true when a tag answered
	 */
	virtual bool Detect(RFTagInfo_t * const pTag);

	/**
	 * @brief	Select a detected tag for access.
	 *
	 * @param	pTag	Tag description from Detect
	 *
	 * @return	true on success
	 */
	virtual bool Select(const RFTagInfo_t * const pTag);

	/**
	 * @brief	Read remote tag memory.
	 *
	 * @param	pTag	Target tag description
	 * @param	Addr	Tag memory offset
	 * @param	pBuff	Destination
	 * @param	Len		Byte count
	 *
	 * @return	Bytes read, 0 on error
	 */
	virtual int TagRead(const RFTagInfo_t * const pTag, uint32_t Addr, uint8_t *pBuff, int Len);

	/**
	 * @brief	Write remote tag memory.
	 *
	 * @param	pTag	Target tag description
	 * @param	Addr	Tag memory offset
	 * @param	pData	Source
	 * @param	Len		Byte count
	 *
	 * @return	Bytes written, 0 on error
	 */
	virtual int TagWrite(const RFTagInfo_t * const pTag, uint32_t Addr, const uint8_t *pData, int Len);

	/**
	 * @brief	Raw transceive over the transport.
	 *
	 * Sends TxLen bytes. When pRx is given, reads the response into it. The
	 * data is not wrapped with an adapter command.
	 *
	 * @param	pTx		Bytes to send
	 * @param	TxLen	Send length
	 * @param	pRx		Response buffer, null for send only
	 * @param	RxLen	Response buffer capacity
	 *
	 * @return	Bytes received, or bytes sent when pRx is null, 0 on error
	 */
	virtual int Transceive(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxLen);

	/**
	 * @brief	Controller event sink, override to observe reader activity.
	 *
	 * @param	Evt	Event code
	 * @param	P0	Event parameter, protocol for detect, length for data
	 * @param	P1	Event parameter, reserved
	 */
	virtual void EvtHandler(RFTAGCTRL_EVT Evt, uint32_t P0, uint32_t P1) {
		(void)Evt; (void)P0; (void)P1;
	}

	uint8_t DevAddr() const { return vCfg.DevAddr; }
	uint32_t ProtoMask() const { return vCfg.ProtoMask; }

protected:
	RFTagControllerCfg_t vCfg;		//!< Controller configuration copy
};

#endif	// __RFTAG_CONTROLLER_H__
