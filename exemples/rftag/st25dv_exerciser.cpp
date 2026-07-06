/**-------------------------------------------------------------------------
@file	st25dv_exerciser.cpp

@brief	Host side exerciser for the ST25DV driver

Drives the ST25DV driver against a mock I2C interface that records the last
transaction and returns canned identification. It checks the byte level
protocol: the Type 5 CC layout for the 4 byte and 8 byte forms, the RF
management value, and the present password sequence. Build and run on the
host, no hardware.

Link rule: build this file with st25dv.cpp and device_intrf.cpp.

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
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "device_intrf.h"
#include "miscdev/st25dv.h"

static int s_Pass = 0;
static int s_Fail = 0;

// Mock transaction capture.
static uint32_t s_LastDevAddr = 0;
static uint8_t s_LastTx[64];
static int s_LastTxLen = 0;
static uint16_t s_LastAddr = 0;

// Dedicated capture of the present password write, which a following status
// read would otherwise clobber in s_LastTx.
static uint8_t s_PwdTx[32];
static int s_PwdTxLen = 0;

static void Check(const char *pName, bool bOk)
{
	if (bOk)
	{
		s_Pass++;
		printf("  [PASS] %s\n", pName);
	}
	else
	{
		s_Fail++;
		printf("  [FAIL] %s\n", pName);
	}
}

static bool MockStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	s_LastDevAddr = DevAddr;
	return true;
}

static int MockTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	(void)pDev;
	int l = DataLen < (int)sizeof(s_LastTx) ? DataLen : (int)sizeof(s_LastTx);
	memcpy(s_LastTx, pData, l);
	s_LastTxLen = DataLen;
	if (DataLen >= 2)
	{
		s_LastAddr = (uint16_t)((pData[0] << 8) | pData[1]);
		if (s_LastAddr == ST25DV_REG_I2C_PWD)
		{
			int pl = DataLen < (int)sizeof(s_PwdTx) ? DataLen : (int)sizeof(s_PwdTx);
			memcpy(s_PwdTx, pData, pl);
			s_PwdTxLen = DataLen;
		}
	}
	return DataLen;
}

static void MockStopTx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool MockStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	s_LastDevAddr = DevAddr;
	return true;
}

static int MockRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	(void)pDev;
	if (s_LastAddr == ST25DV_REG_UID)
	{
		for (int i = 0; i < BuffLen; i++)
		{
			pBuff[i] = (uint8_t)(0xE0 + i);		// canned UID, LSB first
		}
	}
	else if (s_LastAddr == ST25DV_REG_I2C_SSO_DYN)
	{
		if (BuffLen > 0)
		{
			pBuff[0] = 0x01;					// session open
		}
	}
	else
	{
		memset(pBuff, 0, BuffLen);
	}
	return BuffLen;
}

static void MockStopRx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void MockIntrfInit(DevIntrf_t *pIntrf)
{
	memset(pIntrf, 0, sizeof(DevIntrf_t));
	pIntrf->StartTx = MockStartTx;
	pIntrf->TxData = MockTxData;
	pIntrf->StopTx = MockStopTx;
	pIntrf->StartRx = MockStartRx;
	pIntrf->RxData = MockRxData;
	pIntrf->StopRx = MockStopRx;
	pIntrf->MaxRetry = 1;
	atomic_flag_clear(&pIntrf->bBusy);
}

int main(void)
{
	DevIntrf_t intrf;
	MockIntrfInit(&intrf);

	printf("== 1. CC 4 byte read write ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.UserAddr = ST25DV_I2C_ADDR_USER;
		d.SysAddr = ST25DV_I2C_ADDR_SYS;
		d.MemSize = ST25DV_MEMSIZE_04K;			// 512 bytes, MLEN 64

		int l = St25dvWriteType5Cc(&d, false);
		bool ok = (l == 4)
			&& s_LastDevAddr == ST25DV_I2C_ADDR_USER
			&& s_LastTx[0] == 0x00 && s_LastTx[1] == 0x00	// offset 0
			&& s_LastTx[2] == 0xE1 && s_LastTx[3] == 0x40
			&& s_LastTx[4] == 0x40 && s_LastTx[5] == 0x01;	// MLEN 64, feature
		Check("4 byte CC E1 40 40 01 at offset 0", ok);
		Check("CcLen recorded as 4", d.CcLen == 4);
	}

	printf("== 2. CC 8 byte for large memory ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.UserAddr = ST25DV_I2C_ADDR_USER;
		d.MemSize = ST25DV_MEMSIZE_64K;			// 8192 bytes, MLEN 1024

		int l = St25dvWriteType5Cc(&d, false);
		bool ok = (l == 8)
			&& s_LastTx[2] == 0xE2 && s_LastTx[3] == 0x40
			&& s_LastTx[4] == 0x00 && s_LastTx[5] == 0x01
			&& s_LastTx[6] == 0x00 && s_LastTx[7] == 0x00
			&& s_LastTx[8] == 0x04 && s_LastTx[9] == 0x00;	// MLEN 1024 big endian
		Check("8 byte CC E2 40 00 01 00 00 04 00", ok);
	}

	printf("== 3. Read only CC access byte ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.UserAddr = ST25DV_I2C_ADDR_USER;
		d.MemSize = ST25DV_MEMSIZE_04K;

		St25dvWriteType5Cc(&d, true);
		Check("read only sets access byte 0x4C", s_LastTx[3] == 0x4C);
	}

	printf("== 4. RF management dynamic register ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.UserAddr = ST25DV_I2C_ADDR_USER;

		St25dvRfEnable(&d, true);
		bool on = s_LastDevAddr == ST25DV_I2C_ADDR_USER
			&& s_LastAddr == ST25DV_REG_RF_MNGT_DYN
			&& s_LastTx[2] == 0x00;
		Check("RF enable writes RF_MNGT_DYN 0x00", on);

		St25dvRfEnable(&d, false);
		Check("RF disable writes RF_MNGT_DYN 0x01", s_LastTx[2] == ST25DV_RF_DISABLE);
	}

	printf("== 5. Present password sequence ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.SysAddr = ST25DV_I2C_ADDR_SYS;
		d.UserAddr = ST25DV_I2C_ADDR_USER;

		uint8_t pwd[ST25DV_I2C_PWD_LEN];
		for (int i = 0; i < ST25DV_I2C_PWD_LEN; i++)
		{
			pwd[i] = (uint8_t)(0x10 + i);
		}

		s_PwdTxLen = 0;
		bool open = St25dvPresentI2cPwd(&d, pwd);

		Check("present password reports session open", open);

		// Write is address 0x0900 then password, validation code, password.
		bool seq = (s_PwdTxLen == 2 + ST25DV_I2C_PWD_LEN * 2 + 1)
			&& s_PwdTx[0] == 0x09 && s_PwdTx[1] == 0x00
			&& memcmp(&s_PwdTx[2], pwd, ST25DV_I2C_PWD_LEN) == 0
			&& s_PwdTx[2 + ST25DV_I2C_PWD_LEN] == ST25DV_I2C_PWD_VALIDATE
			&& memcmp(&s_PwdTx[3 + ST25DV_I2C_PWD_LEN], pwd, ST25DV_I2C_PWD_LEN) == 0;
		Check("password, validation code, password layout", seq);
	}

	printf("== 7. Init sequence ==\n");
	{
		St25dvDev_t d;
		St25dvCfg_t cfg;
		memset(&cfg, 0, sizeof(cfg));
		cfg.MemSize = ST25DV_MEMSIZE_04K;
		cfg.bRfEnable = true;
		cfg.bWriteCc = true;

		bool ok = St25dvInit(&d, &cfg, &intrf);
		Check("init succeeds with canned UID", ok);
		Check("init resolves default user addr", d.UserAddr == ST25DV_I2C_ADDR_USER);
		Check("init resolves default sys addr", d.SysAddr == ST25DV_I2C_ADDR_SYS);
		// Last write in init is the CC.
		Check("init last write is the CC", s_LastTx[2] == 0xE1);
	}

	printf("== 8. UID read ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.SysAddr = ST25DV_I2C_ADDR_SYS;
		d.UserAddr = ST25DV_I2C_ADDR_USER;

		uint8_t uid[8];
		bool ok = St25dvReadUid(&d, uid);
		Check("UID read returns 8 bytes", ok && uid[0] == 0xE0 && uid[7] == 0xE7);
	}

	printf("== 9. Memory write address prefix ==\n");
	{
		St25dvDev_t d;
		memset(&d, 0, sizeof(d));
		d.pIntrf = &intrf;
		d.UserAddr = ST25DV_I2C_ADDR_USER;
		d.MemSize = ST25DV_MEMSIZE_04K;

		uint8_t data[3] = { 0xAA, 0xBB, 0xCC };
		int l = St25dvWriteMem(&d, 0x0104, data, 3);
		bool ok = (l == 3)
			&& s_LastTx[0] == 0x01 && s_LastTx[1] == 0x04		// big endian address
			&& s_LastTx[2] == 0xAA && s_LastTx[3] == 0xBB && s_LastTx[4] == 0xCC;
		Check("write prefixes big endian address", ok);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);
	return s_Fail == 0 ? 0 : 1;
}
