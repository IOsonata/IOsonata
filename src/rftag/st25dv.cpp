/**-------------------------------------------------------------------------
@file	st25dv.cpp

@brief	ST25DVxxK dynamic NFC / RFID tag driver

Host side driver for the ST25DV family. The chip is a full Type 5 tag on the
RF side, so the host only manages the memory image and the RF enable over I2C.
Static system configuration lives behind an I2C password session, the user
EEPROM and the volatile dynamic registers do not.

@author	Hoang Nguyen Hoan
@date	Jul. 6, 2026

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
#include <stdint.h>
#include <string.h>

#include "device_intrf.h"
#include "idelay.h"
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

// Read Len bytes at Addr from the given device code.
static int St25dvReadAt(St25dvDev_t * const pDev, uint8_t DevAddr, uint16_t Addr, uint8_t *pBuff, int Len)
{
	uint8_t ad[2];

	if (pDev == nullptr || pDev->pIntrf == nullptr || pBuff == nullptr || Len <= 0)
	{
		return -1;
	}

	St25dvAddr(Addr, ad);

	return DeviceIntrfRead(pDev->pIntrf, DevAddr, ad, sizeof(ad), pBuff, Len);
}

// Write Len bytes at Addr to the given device code, then wait the page settle
// time. The dynamic registers ignore the settle time, it only costs a delay.
static int St25dvWriteAt(St25dvDev_t * const pDev, uint8_t DevAddr, uint16_t Addr, const uint8_t *pData, int Len)
{
	uint8_t ad[2];
	int l;

	if (pDev == nullptr || pDev->pIntrf == nullptr || pData == nullptr || Len <= 0)
	{
		return -1;
	}

	St25dvAddr(Addr, ad);

	l = DeviceIntrfWrite(pDev->pIntrf, DevAddr, ad, sizeof(ad), pData, Len);

	msDelay(ST25DV_WRITE_DELAY_MS);

	return l;
}

int St25dvReadMem(St25dvDev_t * const pDev, uint16_t Addr, uint8_t *pBuff, int Len)
{
	return St25dvReadAt(pDev, pDev->UserAddr, Addr, pBuff, Len);
}

int St25dvWriteMem(St25dvDev_t * const pDev, uint16_t Addr, const uint8_t *pData, int Len)
{
	return St25dvWriteAt(pDev, pDev->UserAddr, Addr, pData, Len);
}

bool St25dvReadUid(St25dvDev_t * const pDev, uint8_t *pUid)
{
	if (pUid == nullptr)
	{
		return false;
	}

	return St25dvReadAt(pDev, pDev->SysAddr, ST25DV_REG_UID, pUid, 8) == 8;
}

bool St25dvPresentI2cPwd(St25dvDev_t * const pDev, const uint8_t *pPwd)
{
	static const uint8_t s_ZeroPwd[ST25DV_I2C_PWD_LEN] = { 0 };
	uint8_t seq[ST25DV_I2C_PWD_LEN * 2 + 1];
	uint8_t sso = 0;

	if (pDev == nullptr)
	{
		return false;
	}

	if (pPwd == nullptr)
	{
		pPwd = s_ZeroPwd;
	}

	// Present sequence is password, the validation code, password again.
	memcpy(&seq[0], pPwd, ST25DV_I2C_PWD_LEN);
	seq[ST25DV_I2C_PWD_LEN] = ST25DV_I2C_PWD_VALIDATE;
	memcpy(&seq[ST25DV_I2C_PWD_LEN + 1], pPwd, ST25DV_I2C_PWD_LEN);

	if (St25dvWriteAt(pDev, pDev->SysAddr, ST25DV_REG_I2C_PWD, seq, sizeof(seq)) != (int)sizeof(seq))
	{
		return false;
	}

	// I2C_SSO_DYN bit0 reports the session open state.
	if (St25dvReadAt(pDev, pDev->UserAddr, ST25DV_REG_I2C_SSO_DYN, &sso, 1) != 1)
	{
		return false;
	}

	return (sso & 0x01) != 0;
}

bool St25dvRfEnable(St25dvDev_t * const pDev, bool bEnable)
{
	uint8_t v = bEnable ? 0 : ST25DV_RF_DISABLE;

	if (pDev == nullptr)
	{
		return false;
	}

	// RF_MNGT_DYN is a dynamic register on the user code, no session needed.
	return St25dvWriteAt(pDev, pDev->UserAddr, ST25DV_REG_RF_MNGT_DYN, &v, 1) == 1;
}

int St25dvWriteType5Cc(St25dvDev_t * const pDev, bool bReadOnly)
{
	uint8_t cc[8];
	int len;

	if (pDev == nullptr)
	{
		return 0;
	}

	// Access byte, version 1.0. Write access set to never when read only.
	uint8_t access = bReadOnly ? 0x4C : 0x40;

	// Feature byte advertises multiple block read support.
	uint8_t feature = 0x01;

	if (pDev->MemSize <= ST25DV_CC4_MAX_BYTES)
	{
		cc[0] = 0xE1;						// 4 byte CC magic
		cc[1] = access;
		cc[2] = (uint8_t)(pDev->MemSize / 8);	// MLEN in 8 byte blocks
		cc[3] = feature;
		len = 4;
	}
	else
	{
		uint16_t mlen = (uint16_t)(pDev->MemSize / 8);

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

	if (St25dvWriteMem(pDev, 0, cc, len) != len)
	{
		return 0;
	}

	pDev->CcLen = (uint8_t)len;

	return len;
}

bool St25dvInit(St25dvDev_t * const pDev, const St25dvCfg_t *pCfg, DevIntrf_t * const pIntrf)
{
	uint8_t uid[8];

	if (pDev == nullptr || pCfg == nullptr || pIntrf == nullptr)
	{
		return false;
	}

	pDev->pIntrf = pIntrf;
	pDev->UserAddr = pCfg->I2cUserAddr != 0 ? pCfg->I2cUserAddr : ST25DV_I2C_ADDR_USER;
	pDev->SysAddr = pCfg->I2cSysAddr != 0 ? pCfg->I2cSysAddr : ST25DV_I2C_ADDR_SYS;
	pDev->MemSize = pCfg->MemSize != 0 ? pCfg->MemSize : ST25DV_MEMSIZE_04K;
	pDev->CcLen = 0;

	// Presence check. The UID read must complete for the chip to be usable.
	if (St25dvReadUid(pDev, uid) == false)
	{
		return false;
	}

	// A session is only needed to change static system config, used here for
	// the RF area security when read only is requested.
	if (pCfg->bRfReadOnly || pCfg->pI2cPwd != nullptr)
	{
		if (St25dvPresentI2cPwd(pDev, pCfg->pI2cPwd) == false)
		{
			return false;
		}
	}

	if (pCfg->bRfReadOnly)
	{
		uint8_t ss = ST25DV_RFSS_RO;

		// RF area 1 read only. Area 1 covers the memory by default boundaries.
		if (St25dvWriteAt(pDev, pDev->SysAddr, ST25DV_REG_RFA1SS, &ss, 1) != 1)
		{
			return false;
		}
	}

	if (St25dvRfEnable(pDev, pCfg->bRfEnable) == false)
	{
		return false;
	}

	if (pCfg->bWriteCc)
	{
		if (St25dvWriteType5Cc(pDev, pCfg->bRfReadOnly) == 0)
		{
			return false;
		}
	}

	return true;
}
