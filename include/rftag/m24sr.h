/**-------------------------------------------------------------------------
@file	m24sr.h

@brief	ST M24SR dynamic NFC tag driver

The M24SR is an NFC Forum Type 4 tag with an I2C side to the host and an RF
side to the reader. The chip runs the whole tag. Unlike the memory mapped
ST25DV, its I2C side speaks the same ISO 7816-4 commands as the RF side: an
I2C frame is a protocol control byte, the command or the response, and the
ISO 14443-A CRC. One side at a time holds the tag: the host opens an I2C
session, selects the NDEF application and a file, reads or updates it, then
releases the tag with a deselect so that a reader in the field can reach it.

TagM24sr is an RFTag facet that attaches no protocol engine, as TagSt25dv.
The tag memory of the facet is the NDEF file: a 2 byte big endian length then
the NDEF message, RFTAG_NDEF_FMT_NLEN16 at offset 0. Each MemRead and
MemWrite opens and closes its own session.

The chip is busy while it processes a command and does not acknowledge its
address then: the driver retries each transfer for up to 200 ms. Updates that
need more time answer with a waiting time extension that the driver accepts.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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
#ifndef __M24SR_H__
#define __M24SR_H__

#include <stdint.h>

#include "rftag/rftag.h"

#define M24SR_I2C_DEV_ADDR			0x56		//!< 7 bit address, 0xAC on the bus

// Session commands, a single byte without CRC
#define M24SR_CMD_GET_I2C_SESSION	0x26		//!< Open an I2C session when no reader holds the tag
#define M24SR_CMD_KILL_RF_SESSION	0x52		//!< Open an I2C session, ending the session of a reader

// Protocol control byte of the I2C frames
#define M24SR_PCB_IBLOCK			0x02		//!< Command or response, block number in bit 0
#define M24SR_PCB_DESELECT			0xC2		//!< End of the I2C session
#define M24SR_PCB_WTX				0xF2		//!< Waiting time extension request and answer

#define M24SR_CRC_INIT				0x6363		//!< ISO 14443-A CRC, sent low byte first
#define M24SR_DATA_MAX				0xF6		//!< Largest ReadBinary or UpdateBinary data

// Files of the NDEF application
#define M24SR_FILE_CC				0xE103		//!< Capability container
#define M24SR_FILE_SYSTEM			0xE101		//!< ST system file
#define M24SR_CC_SIZE				15
#define M24SR_SYSTEM_SIZE			18
#define M24SR_SYSTEM_UID			8			//!< Offset of the 7 byte UID in the system file
#define M24SR_UID_LEN				7
#define M24SR_SYSTEM_PRODUCT		17			//!< Offset of the product code in the system file

#define M24SR_SW_OK					0x9000		//!< Status word of a command done

/// M24SR specific configuration
typedef struct __M24sr_Config {
	uint8_t I2cAddr;				//!< 7 bit I2C address, 0 uses M24SR_I2C_DEV_ADDR
	bool bKillRfSession;			//!< Take the tag from a reader in the field, else fail while one holds it
} M24srCfg_t;

class TagM24sr : public RFTag {
public:
	TagM24sr();

	/**
	 * @brief	Initialize the M24SR tag.
	 *
	 * Reads the capability container and the system file: the NDEF file id
	 * and size, the largest read and update, the UID and the product code.
	 * The facet memory becomes the NDEF file, NLEN16 format at offset 0, of
	 * Cfg.MemSize bytes when not 0 and smaller than the file. The RF read
	 * only policy is not implemented: Init fails when Cfg.bReadOnly is set.
	 *
	 * @param	Cfg		Tag configuration
	 * @param	pIntrf	I2C interface the chip is on
	 *
	 * @return	true on success
	 */
	virtual bool Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf = nullptr);

	/**
	 * @brief	M24SR specific init taking the chip configuration.
	 */
	bool Init(const RFTagCfg_t &Cfg, const M24srCfg_t &M24, DeviceIntrf * const pIntrf);

	/**
	 * @brief	Read the NDEF file over I2C, in one session.
	 *
	 * @return	Bytes read, fewer when a command fails
	 */
	virtual int MemRead(uint32_t Addr, uint8_t *pBuff, int Len);

	/**
	 * @brief	Update the NDEF file over I2C, in one session.
	 *
	 * @return	Bytes written, fewer when a command fails
	 */
	virtual int MemWrite(uint32_t Addr, const uint8_t *pData, int Len);

	uint8_t ProductCode() const { return vProduct; }
	uint16_t NdefFileSize() const { return vNdefFileSize; }

	/**
	 * @brief	Status word of the last command, M24SR_SW_OK when done.
	 *
	 * 0 when the last command got no valid response.
	 */
	uint16_t LastStatus() const { return vLastSw; }

private:
	bool Send(const uint8_t *pFrame, int Len);
	bool Receive(uint8_t *pFrame, int Len);
	bool OpenSession();
	void CloseSession();
	int Command(const uint8_t *pApdu, int ApduLen, uint8_t *pData, int DataLen);
	bool SelectApp();
	bool SelectFile(uint16_t FileId);
	int ReadBinary(uint16_t Offset, uint8_t *pBuff, int Len);
	int UpdateBinary(uint16_t Offset, const uint8_t *pData, int Len);

	uint8_t vAddr;					//!< Resolved I2C address
	bool vbKillRf;					//!< Session opened with KillRFsession
	uint8_t vBlock;					//!< Block number of the next command
	uint8_t vProduct;				//!< Product code from the system file
	uint16_t vNdefFileId;			//!< NDEF file id from the capability container
	uint16_t vNdefFileSize;			//!< NDEF file size from the capability container
	uint16_t vMaxRead;				//!< Largest ReadBinary data
	uint16_t vMaxWrite;				//!< Largest UpdateBinary data
	uint16_t vLastSw;				//!< Status word of the last command
};

#endif // __M24SR_H__
