/**-------------------------------------------------------------------------
@file	m24sr.cpp

@brief	ST M24SR dynamic NFC tag driver implementation

Commands follow the M24SR datasheet: NDEF Tag Application Select, Select of a
file by its id, ReadBinary and UpdateBinary, in I-blocks with the block
number toggled on each command of a session, and the Deselect S-block that
ends the session.

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
#include <string.h>

#include "idelay.h"
#include "device_intrf.h"
#include "rftag/m24sr.h"

// Transfer retries, 1 ms apart, while the chip does not acknowledge its address
#define M24SR_POLL_MAX				200

// Waiting time extensions accepted for one command
#define M24SR_WTX_MAX				20

// PCB, CLA INS P1 P2 Lc, data, CRC
#define M24SR_FRAME_MAX				(1 + 5 + M24SR_DATA_MAX + 2)

// PCB, SW1 SW2, CRC
#define M24SR_STATUS_FRAME_SIZE		5

// ISO 14443-A CRC, not inverted
static uint16_t M24srCrc(const uint8_t *pData, int Len)
{
	uint16_t crc = M24SR_CRC_INIT;

	for (int i = 0; i < Len; i++)
	{
		uint8_t b = pData[i] ^ (uint8_t)crc;

		b ^= (uint8_t)(b << 4);
		crc = (crc >> 8) ^ ((uint16_t)b << 8) ^ ((uint16_t)b << 3) ^ (b >> 4);
	}

	return crc;
}

// Appends the CRC low byte first, returns the frame length
static int M24srAddCrc(uint8_t *pFrame, int Len)
{
	uint16_t crc = M24srCrc(pFrame, Len);

	pFrame[Len] = (uint8_t)crc;
	pFrame[Len + 1] = (uint8_t)(crc >> 8);

	return Len + 2;
}

// A frame with its CRC leaves a 0 CRC
static bool M24srCrcOk(const uint8_t *pFrame, int Len)
{
	return Len > 2 && M24srCrc(pFrame, Len) == 0;
}

TagM24sr::TagM24sr()
{
	vAddr = M24SR_I2C_DEV_ADDR;
	vbKillRf = false;
	vBlock = 0;
	vProduct = 0;
	vNdefFileId = 0;
	vNdefFileSize = 0;
	vMaxRead = 0;
	vMaxWrite = 0;
	vLastSw = 0;
}

bool TagM24sr::Send(const uint8_t *pFrame, int Len)
{
	for (int i = 0; i < M24SR_POLL_MAX; i++)
	{
		if (vpIntrf->Tx(vAddr, pFrame, Len) == Len)
		{
			return true;
		}
		msDelay(1);
	}

	return false;
}

bool TagM24sr::Receive(uint8_t *pFrame, int Len)
{
	for (int i = 0; i < M24SR_POLL_MAX; i++)
	{
		if (vpIntrf->Rx(vAddr, pFrame, Len) == Len)
		{
			return true;
		}
		msDelay(1);
	}

	return false;
}

bool TagM24sr::OpenSession()
{
	uint8_t cmd = vbKillRf ? M24SR_CMD_KILL_RF_SESSION : M24SR_CMD_GET_I2C_SESSION;

	vBlock = 0;

	return Send(&cmd, 1);
}

void TagM24sr::CloseSession()
{
	uint8_t f[3] = { M24SR_PCB_DESELECT };
	int len = M24srAddCrc(f, 1);

	// The session ends once the answer is read
	if (Send(f, len))
	{
		Receive(f, len);
	}
}

int TagM24sr::Command(const uint8_t *pApdu, int ApduLen, uint8_t *pData, int DataLen)
{
	uint8_t f[M24SR_FRAME_MAX];
	int len;

	vLastSw = 0;

	if (ApduLen + 3 > (int)sizeof(f) || DataLen + 5 > (int)sizeof(f))
	{
		return -1;
	}

	f[0] = M24SR_PCB_IBLOCK | vBlock;
	memcpy(&f[1], pApdu, ApduLen);
	len = M24srAddCrc(f, ApduLen + 1);

	if (Send(f, len) == false)
	{
		return -1;
	}
	vBlock ^= 1;

	// PCB, data, SW1 SW2, CRC
	len = DataLen + M24SR_STATUS_FRAME_SIZE;

	for (int i = 0; ; i++)
	{
		if (Receive(f, len) == false)
		{
			return -1;
		}

		if (f[0] != M24SR_PCB_WTX)
		{
			break;
		}

		// More time needed, answered with the same extension
		if (i >= M24SR_WTX_MAX || M24srCrcOk(f, 4) == false)
		{
			return -1;
		}

		uint8_t w[4] = { M24SR_PCB_WTX, f[1] };

		if (Send(w, M24srAddCrc(w, 2)) == false)
		{
			return -1;
		}
	}

	if ((f[0] & ~1) != M24SR_PCB_IBLOCK)
	{
		return -1;
	}

	if (M24srCrcOk(f, len) == false)
	{
		// A failed command answers with its status only
		if (len > M24SR_STATUS_FRAME_SIZE && M24srCrcOk(f, M24SR_STATUS_FRAME_SIZE))
		{
			vLastSw = (uint16_t)((f[1] << 8) | f[2]);
		}

		return -1;
	}

	vLastSw = (uint16_t)((f[len - 4] << 8) | f[len - 3]);

	if (vLastSw != M24SR_SW_OK)
	{
		return -1;
	}

	if (DataLen > 0)
	{
		memcpy(pData, &f[1], DataLen);
	}

	return DataLen;
}

bool TagM24sr::SelectApp()
{
	// NDEF Tag Application, AID D2760000850101, Le 0
	static const uint8_t s_Apdu[] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00 };

	return Command(s_Apdu, sizeof(s_Apdu), nullptr, 0) == 0;
}

bool TagM24sr::SelectFile(uint16_t FileId)
{
	uint8_t apdu[] = { 0x00, 0xA4, 0x00, 0x0C, 0x02, (uint8_t)(FileId >> 8), (uint8_t)FileId };

	return Command(apdu, sizeof(apdu), nullptr, 0) == 0;
}

int TagM24sr::ReadBinary(uint16_t Offset, uint8_t *pBuff, int Len)
{
	uint8_t apdu[] = { 0x00, 0xB0, (uint8_t)(Offset >> 8), (uint8_t)Offset, (uint8_t)Len };

	return Command(apdu, sizeof(apdu), pBuff, Len);
}

int TagM24sr::UpdateBinary(uint16_t Offset, const uint8_t *pData, int Len)
{
	uint8_t apdu[5 + M24SR_DATA_MAX] = { 0x00, 0xD6, (uint8_t)(Offset >> 8), (uint8_t)Offset, (uint8_t)Len };

	memcpy(&apdu[5], pData, Len);

	return Command(apdu, 5 + Len, nullptr, 0) == 0 ? Len : -1;
}

bool TagM24sr::Init(const RFTagCfg_t &Cfg, const M24srCfg_t &M24, DeviceIntrf * const pIntrf)
{
	uint8_t cc[M24SR_CC_SIZE];
	uint8_t sys[M24SR_SYSTEM_SIZE];

	Valid(false);
	Interface(nullptr);

	if (pIntrf == nullptr || Cfg.bReadOnly)
	{
		return false;
	}

	vCfg = Cfg;
	vpProto = nullptr;
	Interface(pIntrf);
	vAddr = M24.I2cAddr != 0 ? M24.I2cAddr : M24SR_I2C_DEV_ADDR;
	vbKillRf = M24.bKillRfSession;

	if (OpenSession() == false)
	{
		return false;
	}

	bool res = SelectApp() && SelectFile(M24SR_FILE_CC) &&
			   ReadBinary(0, cc, sizeof(cc)) == (int)sizeof(cc) &&
			   SelectFile(M24SR_FILE_SYSTEM) &&
			   ReadBinary(0, sys, sizeof(sys)) == (int)sizeof(sys);

	CloseSession();

	// CC: length, version, MLe, MLc, then the NDEF file control TLV
	if (res == false || ((cc[0] << 8) | cc[1]) < M24SR_CC_SIZE || cc[7] != 0x04 || cc[8] < 6)
	{
		return false;
	}

	vMaxRead = (uint16_t)((cc[3] << 8) | cc[4]);
	vMaxWrite = (uint16_t)((cc[5] << 8) | cc[6]);
	vNdefFileId = (uint16_t)((cc[9] << 8) | cc[10]);
	vNdefFileSize = (uint16_t)((cc[11] << 8) | cc[12]);

	if (vMaxRead > M24SR_DATA_MAX)
	{
		vMaxRead = M24SR_DATA_MAX;
	}
	if (vMaxWrite > M24SR_DATA_MAX)
	{
		vMaxWrite = M24SR_DATA_MAX;
	}

	if (vMaxRead == 0 || vMaxWrite == 0 || vNdefFileSize < 2)
	{
		return false;
	}

	memset(vCfg.NfcId, 0, sizeof(vCfg.NfcId));
	memcpy(vCfg.NfcId, &sys[M24SR_SYSTEM_UID], M24SR_UID_LEN);
	vCfg.IdLen = M24SR_UID_LEN;
	vProduct = sys[M24SR_SYSTEM_PRODUCT];

	if (vCfg.MemSize == 0 || vCfg.MemSize > vNdefFileSize)
	{
		vCfg.MemSize = vNdefFileSize;
	}
	vCfg.pMem = nullptr;
	vCfg.NdefAddr = 0;
	vCfg.NdefMaxLen = vCfg.MemSize;
	vCfg.NdefFmt = RFTAG_NDEF_FMT_NLEN16;

	Valid(true);

	return true;
}

bool TagM24sr::Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	M24srCfg_t m24;

	// Default address, leave a reader in the field alone
	memset(&m24, 0, sizeof(m24));

	return Init(Cfg, m24, pIntrf);
}

int TagM24sr::MemRead(uint32_t Addr, uint8_t *pBuff, int Len)
{
	if (Valid() == false || pBuff == nullptr || Len <= 0 || Addr >= vCfg.MemSize)
	{
		return 0;
	}

	int len = (uint32_t)Len > vCfg.MemSize - Addr ? (int)(vCfg.MemSize - Addr) : Len;
	int done = 0;

	if (OpenSession() == false)
	{
		return 0;
	}

	if (SelectApp() && SelectFile(vNdefFileId))
	{
		while (done < len)
		{
			int l = len - done > vMaxRead ? vMaxRead : len - done;

			if (ReadBinary((uint16_t)(Addr + done), pBuff + done, l) != l)
			{
				break;
			}
			done += l;
		}
	}

	CloseSession();

	return done;
}

int TagM24sr::MemWrite(uint32_t Addr, const uint8_t *pData, int Len)
{
	if (Valid() == false || pData == nullptr || Len <= 0 || Addr >= vCfg.MemSize)
	{
		return 0;
	}

	int len = (uint32_t)Len > vCfg.MemSize - Addr ? (int)(vCfg.MemSize - Addr) : Len;
	int done = 0;

	if (OpenSession() == false)
	{
		return 0;
	}

	if (SelectApp() && SelectFile(vNdefFileId))
	{
		while (done < len)
		{
			int l = len - done > vMaxWrite ? vMaxWrite : len - done;

			if (UpdateBinary((uint16_t)(Addr + done), pData + done, l) != l)
			{
				break;
			}
			done += l;
		}
	}

	CloseSession();

	return done;
}
