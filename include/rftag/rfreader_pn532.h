/**-------------------------------------------------------------------------
@file	rfreader_pn532.h

@brief	PN532 NFC reader port of the RFTagController facet

The PN532 is a reader IC with an onboard protocol stack. It runs the RF
activation, anticollision and the ISO/IEC 14443 and FeliCa protocol layers in
firmware on the chip. The host exchanges command and response frames over
I2C, SPI or HSU.

Because the protocol lives on the chip, RfReaderPn532 maps each controller
method to a PN532 command frame. InListPassiveTarget performs detection and
activation, InDataExchange passes application data such as ISO7816 APDUs to
the activated tag. No initiator protocol engine is attached, the same shape
as the ST25DV tag whose chip runs the tag protocol in silicon.

Frame format, host to PN532 (UM0701):
  00 00 FF LEN LCS TFI PD0..PDn DCS 00
where TFI is 0xD4 for host to PN532, LEN counts TFI plus the command data,
LCS is the length checksum, DCS the data checksum. The chip answers with an
ACK frame then a response frame with TFI 0xD5.

The bus (I2C, SPI or HSU) is whatever DeviceIntrf is handed to Init. This port
does not bind the bus type.

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
#ifndef __RFREADER_PN532_H__
#define __RFREADER_PN532_H__

#include <stdint.h>

#include "rftag/rftag_controller.h"

// Frame markers, host to PN532 and PN532 to host direction bytes.
#define PN532_PREAMBLE			0x00
#define PN532_STARTCODE1		0x00
#define PN532_STARTCODE2		0xFF
#define PN532_POSTAMBLE			0x00
#define PN532_HOSTTOPN532		0xD4
#define PN532_PN532TOHOST		0xD5

// The response command code is the request code plus one.
#define PN532_RSP_CODE(cmd)		((uint8_t)((cmd) + 1))

// Command codes used by this port. The full set is in the User Manual.
#define PN532_CMD_GETFIRMWAREVERSION	0x02
#define PN532_CMD_SAMCONFIGURATION		0x14
#define PN532_CMD_RFCONFIGURATION		0x32
#define PN532_CMD_INLISTPASSIVETARGET	0x4A
#define PN532_CMD_INDATAEXCHANGE		0x40
#define PN532_CMD_INCOMMUNICATETHRU		0x42
#define PN532_CMD_INRELEASE				0x52

// InListPassiveTarget baud and modulation selector for the target type.
#define PN532_BRTY_106K_TYPEA	0x00		//!< ISO14443 Type A at 106 kbps
#define PN532_BRTY_106K_TYPEB	0x03		//!< ISO14443 Type B at 106 kbps
#define PN532_BRTY_ISO15693		0x01		//!< ISO15693 at 26 kbps, ST reader modes vary

// Largest frame data this port builds or parses.
#define PN532_FRAME_DATA_MAX	262

/// PN532 specific configuration.
typedef struct __RfReaderPn532_Config {
	uint8_t TargetType;			//!< PN532_BRTY_* passive target type to poll
	uint8_t MaxTargets;			//!< InListPassiveTarget MaxTg, 1 or 2
	bool bSamNormal;			//!< Run SAMConfiguration normal mode at init
} RfReaderPn532Cfg_t;

class RfReaderPn532 : public RFTagController {
public:
	RfReaderPn532() : vTargetType(PN532_BRTY_106K_TYPEA), vMaxTargets(1),
		vbSamNormal(true), vTg(1) {}

	/**
	 * @brief	Initialize the PN532 reader.
	 *
	 * Runs SAMConfiguration when requested and reads the firmware version as
	 * a presence check. The bus is whatever DeviceIntrf is given.
	 *
	 * @param	Cfg		Controller configuration
	 * @param	Pn		PN532 specific configuration
	 * @param	pIntrf	Bus the PN532 sits on
	 *
	 * @return	true on success
	 */
	bool Init(const RFTagControllerCfg_t &Cfg, const RfReaderPn532Cfg_t &Pn,
			  DeviceIntrf * const pIntrf);

	// Base default Init, uses PN532 defaults.
	virtual bool Init(const RFTagControllerCfg_t &Cfg, DeviceIntrf * const pIntrf);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	/**
	 * @brief	Detect maps to InListPassiveTarget.
	 *
	 * Polls for one passive target of the configured type and fills the tag
	 * description from the target data.
	 */
	virtual bool Detect(RFTagInfo_t * const pTag);

	/**
	 * @brief	Select records the logical target number.
	 *
	 * InListPassiveTarget already activated the target, so Select stores the
	 * target number for the InDataExchange calls that follow.
	 */
	virtual bool Select(const RFTagInfo_t * const pTag);

	/**
	 * @brief	TagRead builds the protocol read command and exchanges it.
	 *
	 * For a Type 4 tag this is a ReadBinary APDU through InDataExchange. The
	 * mapping is protocol specific, see the source.
	 */
	virtual int TagRead(const RFTagInfo_t * const pTag, uint32_t Addr, uint8_t *pBuff, int Len);

	/**
	 * @brief	TagWrite builds the protocol write command and exchanges it.
	 */
	virtual int TagWrite(const RFTagInfo_t * const pTag, uint32_t Addr, const uint8_t *pData, int Len);

	/**
	 * @brief	Transceive wraps the payload in InDataExchange to the target.
	 *
	 * The payload is passed to the activated target and the answer returned.
	 * For a Type 4 tag pTx is an ISO7816 APDU.
	 */
	virtual int Transceive(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxLen);

private:
	// Build a host to PN532 command frame into pFrame, return the frame length.
	int BuildFrame(uint8_t Cmd, const uint8_t *pData, int DataLen, uint8_t *pFrame, int Cap);

	// Send a command and read the response data after the ACK. Returns the
	// response data length, negative on error. pRsp receives the bytes after
	// the response command code.
	int Command(uint8_t Cmd, const uint8_t *pData, int DataLen, uint8_t *pRsp, int RspCap);

	// InDataExchange to the current target, returns response data length.
	int DataExchange(const uint8_t *pTx, int TxLen, uint8_t *pRx, int RxCap);

	uint8_t vTargetType;		//!< Configured passive target type
	uint8_t vMaxTargets;		//!< InListPassiveTarget MaxTg
	bool vbSamNormal;			//!< Run SAMConfiguration at init
	uint8_t vTg;				//!< Active logical target number for InDataExchange
};

#endif	// __RFREADER_PN532_H__
