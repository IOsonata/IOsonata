/**-------------------------------------------------------------------------
@file	st25dv.cpp

@brief	ST25DVxxK dynamic NFC / RFID tag driver implementation

TagSt25dv is an RFTag facet over an ST25DV chip. The chip runs the whole NFC
Forum Type 5 tag in silicon, so no protocol engine is attached. MemRead and
MemWrite reach the user EEPROM over I2C with a 16 bit big endian memory
address, Enable and Disable drive the dynamic RF management register, and
SetWriteProt sets the RF area 1 security through a password session.

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
#include <string.h>

#include "idelay.h"
#include "device_intrf.h"
#include "rftag/st25dv.h"

// Largest single memory range covered by a 4 byte Type 5 CC. Above this an
// 8 byte CC is required, since MLEN in the 4 byte form is a single byte.
#define ST25DV_CC4_MAX_BYTES		2040

// Encode a 16 bit memory address big endian for the I2C address phase.
static void St25dvAddr(uint16_t Addr, uint8_t *pOut)
{
	pOut[0] = (uint8_t)(Addr >> 8);
	pOut[1] = (uint8_t)Addr;
}

int TagSt25dv::ReadAt(uint8_t DevAddr, uint16_t Addr, uint8_t *pBuff, int Len)
{
	uint8_t ad[2];

	if (vpIntrf == nullptr || pBuff == nullptr || Len <= 0)
	{
		return -1;
	}

	St25dvAddr(Addr, ad);

	return DeviceIntrfRead((DevIntrf_t *)*vpIntrf, DevAddr, ad, sizeof(ad), pBuff, Len);
}

int TagSt25dv::WriteAt(uint8_t DevAddr, uint16_t Addr, const uint8_t *pData, int Len)
{
	uint8_t ad[2];
	int l;

	if (vpIntrf == nullptr || pData == nullptr || Len <= 0)
	{
		return -1;
	}

	St25dvAddr(Addr, ad);

	l = DeviceIntrfWrite((DevIntrf_t *)*vpIntrf, DevAddr, ad, sizeof(ad), pData, Len);

	// The dynamic registers ignore the settle time, it only costs a delay.
	msDelay(ST25DV_WRITE_DELAY_MS);

	return l;
}

int TagSt25dv::MemRead(uint32_t Addr, uint8_t *pBuff, int Len)
{
	return ReadAt(vUserAddr, (uint16_t)Addr, pBuff, Len);
}

int TagSt25dv::MemWrite(uint32_t Addr, const uint8_t *pData, int Len)
{
	return WriteAt(vUserAddr, (uint16_t)Addr, pData, Len);
}

bool TagSt25dv::ReadUid(uint8_t *pUid)
{
	if (pUid == nullptr)
	{
		return false;
	}

	return ReadAt(vSysAddr, ST25DV_REG_UID, pUid, 8) == 8;
}

bool TagSt25dv::PresentI2cPwd(const uint8_t *pPwd)
{
	static const uint8_t s_ZeroPwd[ST25DV_I2C_PWD_LEN] = { 0 };
	uint8_t seq[ST25DV_I2C_PWD_LEN * 2 + 1];
	uint8_t sso = 0;

	if (pPwd == nullptr)
	{
		pPwd = s_ZeroPwd;
	}

	// Present sequence is password, the validation code, password again.
	memcpy(&seq[0], pPwd, ST25DV_I2C_PWD_LEN);
	seq[ST25DV_I2C_PWD_LEN] = ST25DV_I2C_PWD_VALIDATE;
	memcpy(&seq[ST25DV_I2C_PWD_LEN + 1], pPwd, ST25DV_I2C_PWD_LEN);

	if (WriteAt(vSysAddr, ST25DV_REG_I2C_PWD, seq, sizeof(seq)) != (int)sizeof(seq))
	{
		return false;
	}

	// I2C_SSO_DYN bit0 reports the session open state.
	if (ReadAt(vUserAddr, ST25DV_REG_I2C_SSO_DYN, &sso, 1) != 1)
	{
		return false;
	}

	return (sso & 0x01) != 0;
}

bool TagSt25dv::Enable()
{
	uint8_t v = 0;

	// RF_MNGT_DYN is a dynamic register on the user code, no session needed.
	return WriteAt(vUserAddr, ST25DV_REG_RF_MNGT_DYN, &v, 1) == 1;
}

void TagSt25dv::Disable()
{
	uint8_t v = ST25DV_RF_DISABLE;

	WriteAt(vUserAddr, ST25DV_REG_RF_MNGT_DYN, &v, 1);
}

bool TagSt25dv::SetWriteProt(bool bVal)
{
	uint8_t ss = bVal ? ST25DV_RFSS_RO : ST25DV_RFSS_RW;

	// RF area 1 security is a static system register, needs an open session.
	if (PresentI2cPwd(vSt.pI2cPwd) == false)
	{
		return false;
	}

	// RF area 1 covers the memory by default boundaries.
	return WriteAt(vSysAddr, ST25DV_REG_RFA1SS, &ss, 1) == 1;
}

int TagSt25dv::WriteType5Cc(bool bReadOnly)
{
	uint8_t cc[8];
	int len;

	// Access byte, version 1.0. Write access set to never when read only.
	uint8_t access = bReadOnly ? 0x4C : 0x40;

	// Feature byte advertises multiple block read support.
	uint8_t feature = 0x01;

	if (vCfg.MemSize <= ST25DV_CC4_MAX_BYTES)
	{
		cc[0] = 0xE1;						// 4 byte CC magic
		cc[1] = access;
		cc[2] = (uint8_t)(vCfg.MemSize / 8);	// MLEN in 8 byte blocks
		cc[3] = feature;
		len = 4;
	}
	else
	{
		uint16_t mlen = (uint16_t)(vCfg.MemSize / 8);

		cc[0] = 0xE2;						// 8 byte CC magic
		cc[1] = access;
		cc[2] = 0x00;
		cc[3] = feature;
		cc[4] = 0x00;						// reserved
		cc[5] = 0x00;						// reserved
		cc[6] = (uint8_t)(mlen >> 8);		// MLEN high
		cc[7] = (uint8_t)mlen;				// MLEN low
		len = 8;
	}

	if (MemWrite(0, cc, len) != len)
	{
		return 0;
	}

	vCcLen = (uint8_t)len;

	return len;
}

void TagSt25dv::Reset()
{
	WriteType5Cc(vCfg.bReadOnly);
}

bool TagSt25dv::Init(const RFTagCfg_t &Cfg, const St25dvCfg_t &St, DeviceIntrf * const pIntrf)
{
	uint8_t uid[8];

	// A failed Init must not leave a previously initialized object usable.
	Valid(false);

	if (pIntrf == nullptr)
	{
		return false;
	}

	vCfg = Cfg;
	vSt = St;
	vpProto = nullptr;
	vCcLen = 0;

	Interface(pIntrf);

	vUserAddr = vSt.I2cUserAddr != 0 ? vSt.I2cUserAddr : ST25DV_I2C_ADDR_USER;
	vSysAddr = vSt.I2cSysAddr != 0 ? vSt.I2cSysAddr : ST25DV_I2C_ADDR_SYS;

	if (vCfg.MemSize == 0)
	{
		vCfg.MemSize = ST25DV_MEMSIZE_04K;
	}

	// Presence check. The UID read must complete for the chip to be usable.
	if (ReadUid(uid) == false)
	{
		return false;
	}

	// RF read only applies the RF area security. This opens a session.
	if (vCfg.bReadOnly)
	{
		if (SetWriteProt(true) == false)
		{
			return false;
		}
	}
	else if (vSt.pI2cPwd != nullptr)
	{
		if (PresentI2cPwd(vSt.pI2cPwd) == false)
		{
			return false;
		}
	}

	if (vSt.bRfEnable)
	{
		if (Enable() == false)
		{
			return false;
		}
	}
	else
	{
		Disable();
	}

	if (vSt.bWriteCc)
	{
		if (WriteType5Cc(vCfg.bReadOnly) == 0)
		{
			return false;
		}
	}

	Valid(true);

	return true;
}

bool TagSt25dv::Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	St25dvCfg_t st;

	// Default ST25DV configuration: resolve device codes to the defaults,
	// no password, RF on, write the CC.
	memset(&st, 0, sizeof(st));
	st.bRfEnable = true;
	st.bWriteCc = true;

	return Init(Cfg, st, pIntrf);
}
