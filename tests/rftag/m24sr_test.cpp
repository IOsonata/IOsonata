/**-------------------------------------------------------------------------
@file	m24sr_test.cpp

@brief	Host test of the M24SR NFC tag driver.

Runs src/rftag/m24sr.cpp against a model of the chip on I2C: sessions, I2C
frames with their CRC and block numbers, NDEF application and file select,
ReadBinary and UpdateBinary with the size limits of the capability container,
busy periods, waiting time extensions, error status, reader held tag.

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
#include <cstdio>
#include <cstring>

#include "idelay.h"
#include "rftag/m24sr.h"

#define MODEL_NDEF_SIZE			8192
#define MODEL_PRODUCT			0x84

static uint64_t s_DelayUs = 0;

void usDelay(uint32_t Delay)
{
	s_DelayUs += Delay;
}

// Reference CRC written apart from the driver, bit by bit, reflected
// polynomial 0x8408, initial value 0x6363, not inverted
static uint16_t RefCrc(const uint8_t *p, int Len)
{
	uint16_t crc = 0x6363;

	for (int i = 0; i < Len; i++)
	{
		crc ^= p[i];
		for (int b = 0; b < 8; b++)
		{
			crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
		}
	}

	return crc;
}

static int RefAddCrc(uint8_t *p, int Len)
{
	uint16_t crc = RefCrc(p, Len);

	p[Len] = (uint8_t)crc;
	p[Len + 1] = (uint8_t)(crc >> 8);

	return Len + 2;
}

// Model of the M24SR on I2C, at the frame level
class M24srModel : public DeviceIntrf {
public:
	M24srModel(uint8_t I2cAddr = M24SR_I2C_DEV_ADDR) : vDev{} {
		Addr = I2cAddr;
		static const uint8_t cc[M24SR_CC_SIZE] = {
			0x00, 0x0F, 0x20, 0x00, 0xF6, 0x00, 0xF6, 0x04, 0x06, 0x00, 0x01,
			(uint8_t)(MODEL_NDEF_SIZE >> 8), (uint8_t)MODEL_NDEF_SIZE, 0x00, 0x00
		};
		memcpy(Cc, cc, sizeof(Cc));
		memset(Sys, 0, sizeof(Sys));
		Sys[1] = M24SR_SYSTEM_SIZE;
		Sys[2] = 0x01;
		Sys[4] = 0x11;
		Sys[6] = 0x01;
		const uint8_t uid[M24SR_UID_LEN] = { 0x02, 0x84, 0x00, 0x2C, 0x51, 0x6A, 0xF3 };
		memcpy(&Sys[M24SR_SYSTEM_UID], uid, sizeof(uid));
		Sys[15] = 0x1F;
		Sys[16] = 0xFF;
		Sys[M24SR_SYSTEM_PRODUCT] = MODEL_PRODUCT;
		memset(Ndef, 0, sizeof(Ndef));
	}
	operator DevIntrf_t * () { return &vDev; }
	DEVINTRF_TYPE Type() { return DEVINTRF_TYPE_I2C; }
	uint32_t Rate(uint32_t DataRate) { return DataRate; }
	uint32_t Rate(void) { return 400000; }

	int Tx(uint32_t DevAddr, const uint8_t *p, int Len) {
		if (DevAddr != Addr || Len <= 0)
		{
			return 0;
		}
		if (Busy > 0)
		{
			// Address not acknowledged while busy
			Busy--;
			Nacks++;
			return 0;
		}
		if (Len == 1 && (p[0] == M24SR_CMD_GET_I2C_SESSION || p[0] == M24SR_CMD_KILL_RF_SESSION))
		{
			if (p[0] == M24SR_CMD_KILL_RF_SESSION)
			{
				Kills++;
				RfHeld = false;
			}
			if (RfHeld == false)
			{
				Session = true;
				Opens++;
				Block = 0;
				AppSelected = false;
				File = 0;
			}
			Busy = BusyCycles;
			return 1;
		}
		if (Session == false)
		{
			Nacks++;
			return 0;
		}
		if (Len < 3 || RefCrc(p, Len) != 0)
		{
			BadCrc++;
			return Len;
		}
		Busy = BusyCycles;
		switch (p[0])
		{
			case M24SR_PCB_DESELECT:
				memcpy(LastDeselect, p, 3);
				Deselects++;
				Resp[0] = M24SR_PCB_DESELECT;
				RespLen = RefAddCrc(Resp, 1);
				bDeselectPending = true;
				break;
			case M24SR_PCB_WTX:
				if (WtxPending == false || Len != 4 || p[1] != WtxValue)
				{
					BadWtx++;
					break;
				}
				WtxAnswers++;
				WtxPending = false;
				memcpy(Resp, FinalResp, FinalLen);
				RespLen = FinalLen;
				break;
			case M24SR_PCB_IBLOCK:
			case M24SR_PCB_IBLOCK | 1:
				if ((p[0] & 1) != Block)
				{
					BadBlock++;
				}
				Block ^= 1;
				Apdu(p, Len - 2);
				break;
			default:
				BadPcb++;
				break;
		}
		return Len;
	}

	int Rx(uint32_t DevAddr, uint8_t *p, int Len) {
		if (DevAddr != Addr || Len <= 0)
		{
			return 0;
		}
		if (Busy > 0)
		{
			Busy--;
			Nacks++;
			return 0;
		}
		if (RespLen == 0)
		{
			NoResp++;
			return 0;
		}
		memset(p, 0xFF, Len);
		memcpy(p, Resp, RespLen < Len ? RespLen : Len);
		if (RespLen > Len)
		{
			ShortReads++;
		}
		RespLen = 0;
		if (bDeselectPending)
		{
			bDeselectPending = false;
			Session = false;
		}
		return Len;
	}

	// Answer of an I-block: same PCB, data, status word, CRC
	void Answer(uint8_t Pcb, const uint8_t *pData, int Len, uint16_t Sw) {
		uint8_t *r = WtxNext > 0 ? FinalResp : Resp;
		int l = 0;

		r[l++] = RespPcb != 0 ? RespPcb : Pcb;
		RespPcb = 0;
		if (Len > 0)
		{
			memcpy(&r[l], pData, Len);
			l += Len;
		}
		r[l++] = (uint8_t)(Sw >> 8);
		r[l++] = (uint8_t)Sw;
		l = RefAddCrc(r, l);
		if (CorruptCrc)
		{
			r[l - 1] ^= 0x55;
			CorruptCrc = false;
		}
		if (WtxNext > 0)
		{
			// More time asked first, the answer follows the host extension
			WtxNext--;
			FinalLen = l;
			WtxPending = true;
			Resp[0] = M24SR_PCB_WTX;
			Resp[1] = WtxValue;
			RespLen = RefAddCrc(Resp, 2);
		}
		else
		{
			RespLen = l;
		}
	}

	void Apdu(const uint8_t *p, int Len) {
		uint8_t pcb = p[0];
		const uint8_t *a = &p[1];
		int alen = Len - 1;

		Commands++;
		if (alen < 4 || a[0] != 0x00)
		{
			Answer(pcb, nullptr, 0, 0x6E00);
			return;
		}
		switch (a[1])
		{
			case 0xA4:
				Selects++;
				if (a[2] == 0x04 && alen == 13 && a[4] == 7 &&
					memcmp(&a[5], "\xD2\x76\x00\x00\x85\x01\x01", 7) == 0 && a[12] == 0)
				{
					AppSelected = true;
					File = 0;
					Answer(pcb, nullptr, 0, 0x9000);
				}
				else if (a[2] == 0x00 && a[3] == 0x0C && alen == 7 && a[4] == 2 && AppSelected)
				{
					uint16_t id = (uint16_t)((a[5] << 8) | a[6]);

					if (id == M24SR_FILE_CC || id == M24SR_FILE_SYSTEM || id == 0x0001)
					{
						File = id;
						Answer(pcb, nullptr, 0, 0x9000);
					}
					else
					{
						Answer(pcb, nullptr, 0, 0x6A82);
					}
				}
				else
				{
					Answer(pcb, nullptr, 0, 0x6A82);
				}
				break;
			case 0xB0:
			{
				uint16_t off = (uint16_t)((a[2] << 8) | a[3]);
				int le = alen == 5 ? a[4] : -1;
				const uint8_t *f = File == M24SR_FILE_CC ? Cc : File == M24SR_FILE_SYSTEM ? Sys : Ndef;
				int fsize = File == M24SR_FILE_CC ? (int)sizeof(Cc) : File == M24SR_FILE_SYSTEM ?
							(int)sizeof(Sys) : (int)sizeof(Ndef);

				Reads++;
				if (le > MaxLe)
				{
					MaxLe = le;
				}
				if (File == 0x0001 && ReadLocked)
				{
					Answer(pcb, nullptr, 0, 0x6982);
				}
				else if (File == 0 || le <= 0 || le > M24SR_DATA_MAX || off + le > fsize)
				{
					Answer(pcb, nullptr, 0, 0x6700);
				}
				else
				{
					Answer(pcb, &f[off], le, 0x9000);
				}
				break;
			}
			case 0xD6:
			{
				uint16_t off = (uint16_t)((a[2] << 8) | a[3]);
				int lc = alen >= 5 ? a[4] : -1;

				Updates++;
				if (lc > MaxLc)
				{
					MaxLc = lc;
				}
				if (File != 0x0001 || WriteLocked)
				{
					Answer(pcb, nullptr, 0, 0x6982);
				}
				else if (lc <= 0 || lc > M24SR_DATA_MAX || alen != 5 + lc || off + lc > (int)sizeof(Ndef))
				{
					Answer(pcb, nullptr, 0, 0x6700);
				}
				else
				{
					memcpy(&Ndef[off], &a[5], lc);
					Answer(pcb, nullptr, 0, 0x9000);
				}
				break;
			}
			default:
				Answer(pcb, nullptr, 0, 0x6D00);
				break;
		}
	}

	uint8_t Addr;
	uint8_t Cc[M24SR_CC_SIZE];
	uint8_t Sys[M24SR_SYSTEM_SIZE];
	uint8_t Ndef[MODEL_NDEF_SIZE];
	uint8_t Resp[300];
	int RespLen = 0;
	uint8_t FinalResp[300];
	int FinalLen = 0;
	uint8_t LastDeselect[3] = { 0 };
	bool Session = false;
	bool bDeselectPending = false;
	bool AppSelected = false;
	uint16_t File = 0;
	uint8_t Block = 0;
	bool RfHeld = false;
	bool WriteLocked = false;
	bool ReadLocked = false;
	uint8_t RespPcb = 0;
	bool CorruptCrc = false;
	int BusyCycles = 2;
	int Busy = 0;
	int WtxNext = 0;
	uint8_t WtxValue = 0x02;
	bool WtxPending = false;
	int Opens = 0, Deselects = 0, Kills = 0, Commands = 0, Selects = 0, Reads = 0, Updates = 0;
	int Nacks = 0, NoResp = 0, ShortReads = 0, BadCrc = 0, BadBlock = 0, BadPcb = 0, BadWtx = 0;
	int WtxAnswers = 0;
	int MaxLe = 0, MaxLc = 0;

private:
	DevIntrf_t vDev;
};

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static const RFTagCfg_t s_TagCfg = {
	.NfcId = { 0 },
	.IdLen = 0,
	.bReadOnly = false,
	.pMem = nullptr,
	.MemSize = 0,
	.NdefAddr = 0,
	.NdefMaxLen = 0,
	.NdefFmt = RFTAG_NDEF_FMT_NONE,
	.WrProtCB = nullptr,
	.pWrProtCtx = nullptr,
};

// No protocol error seen by the model, sessions closed
static void CheckClean(M24srModel &M)
{
	CHECK(M.BadCrc == 0 && M.BadBlock == 0 && M.BadPcb == 0 && M.BadWtx == 0);
	CHECK(M.ShortReads == 0 && M.NoResp == 0);
	CHECK(M.Session == false && M.Opens == M.Deselects);
	CHECK(M.MaxLe <= M24SR_DATA_MAX && M.MaxLc <= M24SR_DATA_MAX);
}

static void TestInit(void)
{
	M24srModel m;
	TagM24sr tag;

	CHECK(tag.Init(s_TagCfg, &m) == true);
	CHECK(tag.Valid() == true);
	CHECK(tag.MemSize() == MODEL_NDEF_SIZE && tag.NdefFileSize() == MODEL_NDEF_SIZE);
	CHECK(tag.NdefAddr() == 0 && tag.NdefMaxLen() == MODEL_NDEF_SIZE);
	CHECK(tag.NdefFmt() == RFTAG_NDEF_FMT_NLEN16);
	CHECK(tag.IdLen() == M24SR_UID_LEN);
	CHECK(memcmp(tag.NfcId(), &m.Sys[M24SR_SYSTEM_UID], M24SR_UID_LEN) == 0);
	CHECK(tag.ProductCode() == MODEL_PRODUCT);
	CHECK(tag.LastStatus() == M24SR_SW_OK);
	CHECK(m.Opens == 1 && m.Kills == 0);
	// Application, CC, read, system file, read
	CHECK(m.Commands == 5 && m.Selects == 3 && m.Reads == 2);
	CHECK(m.Nacks > 0);
	// Deselect frame with the CRC of the datasheet example
	CHECK(memcmp(m.LastDeselect, "\xC2\xE0\xB4", 3) == 0);
	CheckClean(m);

	// Smaller memory asked
	M24srModel s;
	TagM24sr st;
	RFTagCfg_t cfg = s_TagCfg;

	cfg.MemSize = 1000;
	CHECK(st.Init(cfg, &s) == true);
	CHECK(st.MemSize() == 1000 && st.NdefMaxLen() == 1000);
}

static void TestNdef(void)
{
	M24srModel m;
	TagM24sr tag;
	uint8_t msg[600];
	uint8_t rd[MODEL_NDEF_SIZE];

	for (int i = 0; i < (int)sizeof(msg); i++)
	{
		msg[i] = (uint8_t)(i * 7 + 3);
	}

	CHECK(tag.Init(s_TagCfg, &m) == true);

	// Empty file
	CHECK(tag.GetNdef(rd, sizeof(rd)) == 0);

	int opens = m.Opens, updates = m.Updates;

	CHECK(tag.SetNdef(msg, sizeof(msg)) == true);
	// NLEN 0, the message in 3 updates of at most 246 bytes, then NLEN
	CHECK(m.Updates == updates + 5);
	CHECK(m.Opens == opens + 3);
	CHECK(m.Ndef[0] == (sizeof(msg) >> 8) && m.Ndef[1] == (sizeof(msg) & 0xFF));
	CHECK(memcmp(&m.Ndef[2], msg, sizeof(msg)) == 0);
	CHECK(m.MaxLc == M24SR_DATA_MAX);

	memset(rd, 0, sizeof(rd));
	CHECK(tag.GetNdef(rd, sizeof(rd)) == (int)sizeof(msg));
	CHECK(memcmp(rd, msg, sizeof(msg)) == 0);
	CHECK(m.MaxLe == M24SR_DATA_MAX);

	// Reads clipped at the end of the file
	CHECK(tag.MemRead(MODEL_NDEF_SIZE - 2, rd, 10) == 2);
	CHECK(tag.MemRead(MODEL_NDEF_SIZE, rd, 10) == 0);
	CHECK(tag.MemWrite(MODEL_NDEF_SIZE - 1, msg, 10) == 1);
	CHECK(m.Ndef[MODEL_NDEF_SIZE - 1] == msg[0]);

	// A large read
	CHECK(tag.MemRead(0, rd, sizeof(rd)) == (int)sizeof(rd));
	CHECK(memcmp(rd, m.Ndef, sizeof(rd)) == 0);
	CheckClean(m);
}

static void TestWtxBusy(void)
{
	M24srModel m;
	TagM24sr tag;
	uint8_t d[300];

	for (int i = 0; i < (int)sizeof(d); i++)
	{
		d[i] = (uint8_t)(255 - i);
	}

	m.BusyCycles = 7;
	CHECK(tag.Init(s_TagCfg, &m) == true);

	// The first two updates ask for more time
	m.WtxNext = 2;
	m.WtxValue = 0x05;
	CHECK(tag.MemWrite(100, d, sizeof(d)) == (int)sizeof(d));
	CHECK(m.WtxAnswers == 2);
	CHECK(memcmp(&m.Ndef[100], d, sizeof(d)) == 0);
	CheckClean(m);

	// Busy longer than the retries: fails within the bound
	s_DelayUs = 0;
	m.BusyCycles = 100000;
	CHECK(tag.MemRead(0, d, 10) == 0);
	CHECK(s_DelayUs >= 200000 && s_DelayUs <= 2000000);
}

static void TestReaderHeld(void)
{
	// A reader holds the tag: GetI2Csession is ignored
	M24srModel m;
	TagM24sr tag;

	m.RfHeld = true;
	m.BusyCycles = 0;
	CHECK(tag.Init(s_TagCfg, &m) == false);
	CHECK(tag.Valid() == false);
	CHECK(m.Kills == 0 && m.Opens == 0);

	// KillRFsession takes it
	M24srCfg_t kcfg = { 0, true };
	TagM24sr ktag;

	m.BusyCycles = 2;
	CHECK(ktag.Init(s_TagCfg, kcfg, &m) == true);
	CHECK(m.Kills == 1 && m.RfHeld == false);
	CheckClean(m);
}

static void TestErrors(void)
{
	M24srModel m;
	TagM24sr tag;
	uint8_t d[20];

	memset(d, 0x5A, sizeof(d));
	CHECK(tag.Init(s_TagCfg, &m) == true);

	// Write protected file: status 6982, nothing written, session closed
	m.WriteLocked = true;
	CHECK(tag.MemWrite(0, d, sizeof(d)) == 0);
	CHECK(tag.LastStatus() == 0x6982);
	CHECK(m.Ndef[0] == 0);
	CHECK(tag.SetNdef(d, sizeof(d)) == false);
	m.WriteLocked = false;
	CheckClean(m);

	// Read protected file: the status only answer of a read
	m.ReadLocked = true;
	CHECK(tag.MemRead(0, d, sizeof(d)) == 0);
	CHECK(tag.LastStatus() == 0x6982);
	m.ReadLocked = false;
	CheckClean(m);

	// Answer that is not an I-block
	m.RespPcb = 0xA2;
	CHECK(tag.MemRead(0, d, sizeof(d)) == 0);
	CHECK(m.Session == false);

	// Response with a wrong CRC
	m.CorruptCrc = true;
	CHECK(tag.MemRead(0, d, sizeof(d)) == 0);
	CHECK(tag.LastStatus() == 0);
	CHECK(m.Session == false);

	// Read only policy not implemented
	M24srModel r;
	TagM24sr rtag;
	RFTagCfg_t cfg = s_TagCfg;

	cfg.bReadOnly = true;
	CHECK(rtag.Init(cfg, &r) == false);
	CHECK(r.Opens == 0 && r.Commands == 0);

	// No interface
	CHECK(rtag.Init(s_TagCfg, nullptr) == false);

	// Other address
	M24srModel a(0x57);
	TagM24sr atag;
	M24srCfg_t acfg = { 0x57, false };

	a.BusyCycles = 0;
	CHECK(atag.Init(s_TagCfg, &a) == false);
	CHECK(atag.Init(s_TagCfg, acfg, &a) == true);
	CheckClean(a);

	// CC larger than the frames: limited to 246 bytes
	M24srModel big;
	TagM24sr btag;
	uint8_t rd[600];

	big.Cc[3] = 0x01;
	big.Cc[4] = 0x00;
	big.Cc[5] = 0x01;
	big.Cc[6] = 0x00;
	CHECK(btag.Init(s_TagCfg, &big) == true);
	CHECK(btag.MemRead(0, rd, sizeof(rd)) == (int)sizeof(rd));
	CHECK(btag.MemWrite(0, rd, sizeof(rd)) == (int)sizeof(rd));
	CHECK(big.MaxLe == M24SR_DATA_MAX && big.MaxLc == M24SR_DATA_MAX);
	CheckClean(big);

	// Not valid before Init
	TagM24sr ntag;

	CHECK(ntag.MemRead(0, d, sizeof(d)) == 0);
	CHECK(ntag.MemWrite(0, d, sizeof(d)) == 0);
}

int main()
{
	TestInit();
	TestNdef();
	TestWtxBusy();
	TestReaderHeld();
	TestErrors();

	printf("M24SR driver model: %d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}
