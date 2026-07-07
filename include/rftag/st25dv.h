/**-------------------------------------------------------------------------
@file	st25dv.h

@brief	ST25DVxxK dynamic NFC / RFID tag driver

The ST25DV is a full NFC Forum Type 5 (ISO15693) tag with an I2C side to the
host and an RF side to the reader. The chip runs the whole tag: RF activation,
anticollision, protocol and framing are done in hardware. The host only fills
the tag memory over I2C.

Because the protocol lives in silicon, TagSt25dv is an RFTag facet that
attaches no protocol engine, the same shape as a sensor whose chip runs the
processing on die. MemRead and MemWrite reach the user EEPROM over I2C, Enable
and Disable drive the dynamic RF management register, and SetWriteProt uses
the native RF area security mechanism.

Two I2C device addresses share the same 16 bit memory address space. The user
address reaches the user EEPROM and the volatile dynamic registers. The system
address reaches the static system configuration, which is guarded by an I2C
password session.

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
#ifndef __ST25DV_H__
#define __ST25DV_H__

#include <stdint.h>

#include "rftag/rftag.h"

// Default 7 bit I2C device select codes. The user code reaches the EEPROM and
// the dynamic registers. The system code reaches the static configuration and
// needs an open I2C password session to change.
#define ST25DV_I2C_ADDR_USER		0x53
#define ST25DV_I2C_ADDR_SYS			0x57

// User EEPROM size per part variant, in bytes.
#define ST25DV_MEMSIZE_04K			512
#define ST25DV_MEMSIZE_16K			2048
#define ST25DV_MEMSIZE_64K			8192

// EEPROM page write time. A write must settle before the next access.
#define ST25DV_WRITE_DELAY_MS		5

// I2C password length and the present password validation code.
#define ST25DV_I2C_PWD_LEN			8
#define ST25DV_I2C_PWD_VALIDATE		0x09

// Static system configuration registers, addressed on the system I2C code.
#define ST25DV_REG_GPO				0x0000
#define ST25DV_REG_IT_TIME			0x0001
#define ST25DV_REG_EH_MODE			0x0002
#define ST25DV_REG_RF_MNGT			0x0003		//!< Static RF management, RF_DISABLE bit0
#define ST25DV_REG_RFA1SS			0x0004		//!< RF area 1 security status
#define ST25DV_REG_ENDA1			0x0005		//!< RF area 1 end
#define ST25DV_REG_LOCK_CCFILE		0x000C
#define ST25DV_REG_MB_MODE			0x000D
#define ST25DV_REG_I2C_CFG			0x000E
#define ST25DV_REG_I2C_PWD			0x0900		//!< 8 byte I2C password, present sequence

// Read only identification, addressed on the system I2C code.
#define ST25DV_REG_UID				0x0018		//!< 8 byte UID, LSB first
#define ST25DV_REG_IC_REF			0x0017		//!< IC reference code
#define ST25DV_REG_MEM_SIZE			0x0014		//!< Block count and block size

// Volatile dynamic registers, addressed on the user I2C code, no session.
#define ST25DV_REG_GPO_DYN			0x2000
#define ST25DV_REG_EH_CTRL_DYN		0x2002
#define ST25DV_REG_RF_MNGT_DYN		0x2003		//!< RF_DISABLE bit0, RF_SLEEP bit1
#define ST25DV_REG_I2C_SSO_DYN		0x2004		//!< I2C security session open, bit0
#define ST25DV_REG_MB_CTRL_DYN		0x2006
#define ST25DV_REG_MB_LEN_DYN		0x2007

// RF management bits, shared by static RF_MNGT and dynamic RF_MNGT_DYN.
#define ST25DV_RF_DISABLE			(1 << 0)
#define ST25DV_RF_SLEEP				(1 << 1)

// RF area security status field values for RFAnSS, RF access rights.
#define ST25DV_RFSS_RW				0x00		//!< RF read and write
#define ST25DV_RFSS_RO				0x01		//!< RF read only

/// ST25DV specific configuration, extends the tag configuration.
typedef struct __St25dv_Config {
	uint8_t I2cUserAddr;			//!< 7 bit user area device code, 0 uses ST25DV_I2C_ADDR_USER
	uint8_t I2cSysAddr;				//!< 7 bit system area device code, 0 uses ST25DV_I2C_ADDR_SYS
	const uint8_t *pI2cPwd;			//!< 8 byte I2C password, null uses the all zero default
	bool bRfEnable;					//!< Turn the RF side on at init
	bool bWriteCc;					//!< Write the Type 5 CC at user offset 0 at init
} St25dvCfg_t;

class TagSt25dv : public RFTag {
public:
	TagSt25dv() : vUserAddr(0), vSysAddr(0), vCcLen(0) {
		memset(&vSt, 0, sizeof(vSt));
	}

	/**
	 * @brief	Initialize the ST25DV tag.
	 *
	 * Resolves the device codes, checks presence by reading the UID, applies
	 * the RF read only policy from Cfg.bReadOnly through the native area
	 * security, sets the RF side and writes the Type 5 CC. The tag runs the
	 * protocol in silicon, no engine is attached.
	 *
	 * @param	Cfg		Tag configuration. bReadOnly drives the RF area
	 *					security, MemSize the user EEPROM size, NdefAddr and
	 *					NdefMaxLen and NdefFmt the NDEF container placement.
	 * @param	pIntrf	I2C interface the chip is on
	 *
	 * @return	true on success
	 */
	virtual bool Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf = nullptr);

	/**
	 * @brief	ST25DV specific init taking the chip configuration.
	 *
	 * @param	Cfg		Tag configuration, as above
	 * @param	St		ST25DV specific configuration
	 * @param	pIntrf	I2C interface the chip is on
	 *
	 * @return	true on success
	 */
	bool Init(const RFTagCfg_t &Cfg, const St25dvCfg_t &St, DeviceIntrf * const pIntrf);

	/**
	 * @brief	Turn the RF side on through the dynamic RF management register.
	 */
	virtual bool Enable();

	/**
	 * @brief	Turn the RF side off through the dynamic RF management register.
	 */
	virtual void Disable();

	/**
	 * @brief	Reset does not clear the chip. It rewrites the CC.
	 */
	virtual void Reset();

	/**
	 * @brief	Read the user EEPROM over I2C.
	 */
	virtual int MemRead(uint32_t Addr, uint8_t *pBuff, int Len);

	/**
	 * @brief	Write the user EEPROM over I2C. Blocks for the page settle time.
	 */
	virtual int MemWrite(uint32_t Addr, const uint8_t *pData, int Len);

	/**
	 * @brief	Set or clear RF read only through the RF area 1 security.
	 *
	 * Needs an open I2C password session. The host still writes over I2C.
	 *
	 * @param	bVal	true sets RF read only, false RF read and write
	 *
	 * @return	true on success
	 */
	virtual bool SetWriteProt(bool bVal);

	/**
	 * @brief	Present the I2C password to open the system config session.
	 *
	 * @param	pPwd	8 byte password, null uses the all zero default
	 *
	 * @return	true if the session reports open
	 */
	bool PresentI2cPwd(const uint8_t *pPwd);

	/**
	 * @brief	Read the 8 byte UID from the system area, LSB first.
	 */
	bool ReadUid(uint8_t *pUid);

	/**
	 * @brief	Write the NFC Forum Type 5 CC at user offset 0.
	 *
	 * Picks a 4 byte CC for memory up to 2040 bytes, an 8 byte CC otherwise.
	 *
	 * @param	bReadOnly	Advertise RF read only access in the CC
	 *
	 * @return	CC length written, 0 on error
	 */
	int WriteType5Cc(bool bReadOnly);

private:
	int ReadAt(uint8_t DevAddr, uint16_t Addr, uint8_t *pBuff, int Len);
	int WriteAt(uint8_t DevAddr, uint16_t Addr, const uint8_t *pData, int Len);

	St25dvCfg_t vSt;				//!< ST25DV specific configuration copy
	uint8_t vUserAddr;				//!< Resolved user area device code
	uint8_t vSysAddr;				//!< Resolved system area device code
	uint8_t vCcLen;					//!< CC length written, 4 or 8
};

#endif // __ST25DV_H__
