// Host regression of the generic SD card driver (src/storage/sdcard_impl.cpp)
//
// SPI mode: an SPI interface model (DEVINTRF_TYPE_SPI, chip select low from
// start to stop, DummyByte sent on reads) with a card that answers byte by
// byte. The card takes commands only with the chip select low, needs 74
// clocks or more with the data line high before its first CMD0, checks the
// CRC7 of every command and the CRC16 of the data written, answers after
// Ncr bytes, sends its data blocks after Nac bytes, streams the blocks of
// CMD18 until CMD12 (a stuff byte, then R1, then busy), takes the start
// tokens 0xFE and 0xFC, the stop token 0xFD, answers each block written with
// a data response, then holds the data line low while it programs. A chip
// select released in the middle of a command or of a multiple block read
// fails the test, the data line must be high (0xFF) when the host only reads.
//
// SD bus: an SD/MMC host model (DEVINTRF_TYPE_SDMMC) that takes the frames of
// coredev/sdmmc.h: the response asked must be the one of the command, the
// data direction the one of the command, CMD0 with the initialization clocks
// first, the stop flag on the multiple block commands only, the bus width
// flag on ACMD6 only. Commands of the identification at 400 kHz or less, data
// at the TRAN_SPEED of the card or less. Byte addresses on the standard
// capacity cards, sector numbers on the others.
//
// Cards: MMC (SPI mode), SDSC version 1, SDSC version 2, SDHC. Faults: no
// card, no response to a command, a wrong CMD8 pattern, a card never ready,
// CSD fields, R1 errors, data CRC errors, a missing start token, data
// refused, programming errors, a card that stays busy.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

#include "crc.h"
#include "coredev/spi.h"
#include "coredev/sdmmc.h"
#include "storage/sdcard.h"

static double s_Us = 0;

extern "C" void ModelDelayUs(uint32_t Us)
{
	s_Us += Us;
}

// ---- Card ----

enum { T_MMC, T_V1, T_V2, T_HC };
enum { ST_IDLE, ST_READY, ST_IDENT, ST_STBY, ST_TRAN, ST_DATA, ST_RCV };
enum { R_NONE, R_R1, R_R1B, R_R2, R_R3, R_R6, R_R7 };

#define MEM_SIZE		(4U << 20)
#define NB_SECT			(MEM_SIZE / 512U)
#define CARD_RCA		0x4567U
#define R1_ERRORS		0xFDF98008U

static uint8_t s_Mem[MEM_SIZE];

static const uint8_t s_Cid[16] = {
	0x03, 0x53, 0x44, 0x53, 0x44, 0x31, 0x36, 0x47, 0x80, 0x12, 0x34, 0x56, 0x78, 0x01, 0x4A, 0x01
};

struct CardM {
	int Type = T_HC;
	bool Present = true;
	bool Spi = false;			// SPI mode: CMD0 with the chip select low
	bool Bus = false;			// SD bus: CMD0 with the initialization clocks
	int State = ST_IDLE;
	bool Acmd = false;
	bool Ready = false;			// Power up done
	int OpBusy = 2;				// ACMD41 or CMD1 answers busy this many times
	double OpMs = 0;			// and this time from the first one
	int OpCount = 0;
	double OpStart = 0;
	bool NeverReady = false;
	int Width = 1;
	uint32_t BlockLen = 512;
	uint8_t TranSpeed = 0x32;	// 25 MHz
	uint32_t Pend = 0;			// SD bus: error bits of the next R1
	uint8_t Pend2 = 0;			// SPI: second byte of the next R2
	uint32_t Addr = 0;			// Byte address of the data
	double BusyEnd = 0;
	bool BusyForever = false;
	uint32_t FfBytes = 0;		// SPI: bytes 0xFF before the first CMD0
	bool Cmd0Seen = false;
	// Faults
	int NoResp = -1;			// No response to this command once, 100 + n for ACMDn
	int ErrCmd = -1;			// Error bits ErrBits in the R1 of this command once
	uint32_t ErrBits = 0;
	int NoAppAt = -1;			// SD bus: this CMD55 (0 the first) answered without APP_CMD
	int Apps = 0;
	int Cmd0R1 = -1;			// SPI: R1 of every CMD0
	bool StopStuck = false;		// Busy for ever after the next CMD12
	bool Cmd8Bad = false;
	int CsdVer = -1;			// CSD_STRUCTURE other than the one of the card
	int BlLen = -1;				// READ_BL_LEN other than the one of the card
	int BadCrcBlock = -1;		// Block of the next read with a data CRC error
	int NoToken = -1;			// SPI: block of the next read without start token
	int RejectBlock = -1;		// Block of the next write refused
	uint32_t WriteErr = 0;		// Error bits in the status after the next write
	bool StuckWrite = false;	// Busy for ever after the next block written
	bool StuckSelect = false;	// SD bus: busy for ever after CMD7
	int Ncr = 1;				// SPI: bytes before a response
	int Nac = 2;				// SPI: bytes before a data block
	double ProgUs = 300;		// Programming time of a block
	// Counts
	std::vector<int> Log;
	int BlocksRead = 0, BlocksWritten = 0, Stops = 0;
	double MaxIdRate = 0, MaxDataRate = 0;
};
static CardM s_C;
static uint32_t s_Rate = 400000;	// Clock of the interface
static int s_Xfer = 0;				// Block of the current transfer

static bool CardBusy() { return s_C.BusyForever || s_Us < s_C.BusyEnd; }

static uint32_t CardMaxRate()
{
	static const uint8_t v[16] = { 0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80 };
	static const uint32_t u[4] = { 10000, 100000, 1000000, 10000000 };

	uint32_t r = (s_C.TranSpeed & 7) < 4 ? u[s_C.TranSpeed & 7] * v[(s_C.TranSpeed >> 3) & 15] : 0;

	// 25 MHz for a reserved value
	return r != 0 ? r : 25000000;
}

static void PutBits(uint8_t *p, int Msb, int Lsb, uint32_t V)
{
	for (int b = Lsb; b <= Msb; b++)
	{
		uint8_t m = (uint8_t)(1U << (b % 8));

		if ((V >> (b - Lsb)) & 1U)
		{
			p[(127 - b) / 8] |= m;
		}
		else
		{
			p[(127 - b) / 8] &= (uint8_t)~m;
		}
	}
}

static void Csd(uint8_t *p)
{
	memset(p, 0, 16);
	PutBits(p, 103, 96, s_C.TranSpeed);
	if (s_C.Type == T_HC)
	{
		PutBits(p, 127, 126, 1);
		PutBits(p, 83, 80, 9);
		PutBits(p, 69, 48, MEM_SIZE / (512U * 1024U) - 1U);
		// Reserved bits above C_SIZE, not read
		PutBits(p, 71, 70, 3);
	}
	else
	{
		int bl = s_C.BlLen >= 0 ? s_C.BlLen : s_C.Type == T_V2 ? 10 : 9;

		// MMC CSD structure 1.2, read the same way
		PutBits(p, 127, 126, s_C.Type == T_MMC ? 2 : 0);
		// C_SIZE_MULT 7 (SDSC version 2), 0 (version 1, the high bits of
		// C_SIZE), 3 (MMC)
		int mult = s_C.Type == T_V2 ? 7 : s_C.Type == T_V1 ? 0 : 3;

		PutBits(p, 83, 80, (uint32_t)bl);
		PutBits(p, 73, 62, (MEM_SIZE >> ((bl > 11 ? 11 : bl) + mult + 2)) - 1U);
		PutBits(p, 49, 47, (uint32_t)mult);
	}
	if (s_C.CsdVer >= 0)
	{
		PutBits(p, 127, 126, (uint32_t)s_C.CsdVer);
	}
	p[15] |= 1;
}

static uint32_t AddrOf(uint32_t Arg)
{
	if (s_C.Type == T_HC)
	{
		return Arg < NB_SECT ? Arg * 512U : 0xFFFFFFFFU;
	}
	// Byte address, a multiple of the block length
	assert(s_C.BlockLen == 512 && Arg % 512U == 0);
	return Arg < MEM_SIZE ? Arg : 0xFFFFFFFFU;
}

static void CheckRate()
{
	if (s_C.Ready == false || s_C.State < ST_STBY)
	{
		assert(s_Rate <= 400000);
		if (s_Rate > s_C.MaxIdRate)
		{
			s_C.MaxIdRate = s_Rate;
		}
	}
	else
	{
		assert(s_Rate <= CardMaxRate());
		if (s_Rate > s_C.MaxDataRate)
		{
			s_C.MaxDataRate = s_Rate;
		}
	}
}

// Power up: ACMD41 (SD) or CMD1 (MMC) answers busy a number of times and a
// time from the first, never for an SDHC card without HCS
static bool OpCond(bool Hcs)
{
	if (s_C.OpCount++ == 0)
	{
		s_C.OpStart = s_Us;
	}
	if (s_C.NeverReady || (s_C.Type == T_HC && Hcs == false))
	{
		return false;
	}
	if (s_C.OpCount > s_C.OpBusy && s_Us - s_C.OpStart >= s_C.OpMs * 1000.0)
	{
		s_C.Ready = true;
	}

	return s_C.Ready;
}

// ---- SD bus card ----

static uint32_t BusR1()
{
	uint32_t st = s_C.Pend | ((uint32_t)s_C.State << 9) | (1U << 8) | (s_C.Acmd ? (1U << 5) : 0U);

	s_C.Pend = 0;
	return st;
}

static bool RcaArg(uint32_t Arg)
{
	assert((Arg & 0xFFFFU) == 0);
	return (Arg >> 16) == CARD_RCA;
}

static int BusCardCmd(int Idx, uint32_t Arg, uint32_t &Resp, uint8_t *pLong, bool bInit)
{
	if (s_C.Present == false)
	{
		return R_NONE;
	}

	bool acmd = s_C.Acmd;
	int code = acmd ? 100 + Idx : Idx;

	s_C.Log.push_back(code);
	s_C.Acmd = false;
	assert(CardBusy() == false || Idx == 13);
	if (Idx == 0)
	{
		// The first CMD0 comes with the initialization clocks
		assert(bInit || s_C.Bus);
		s_C.Bus = true;
		s_C.State = ST_IDLE;
		s_C.Ready = false;
		s_C.OpCount = 0;
		s_C.Width = 1;
		return R_NONE;
	}
	assert(s_C.Bus && bInit == false);
	CheckRate();
	if (s_C.NoResp == code)
	{
		s_C.NoResp = -1;
		return R_NONE;
	}
	if (s_C.ErrCmd == code)
	{
		s_C.ErrCmd = -1;
		s_C.Pend |= s_C.ErrBits;
	}
	if (s_C.Type == T_MMC)
	{
		// No CMD8, CMD55, ACMD41: an MMC does not answer them
		if (Idx == 8 || Idx == 55 || acmd)
		{
			return R_NONE;
		}
	}
	if (acmd)
	{
		switch (Idx)
		{
			case 41:
				if (s_C.State != ST_IDLE)
				{
					return R_NONE;
				}
				assert(s_C.Type != T_V1 || (Arg & 0x40000000U) == 0);
				assert(Arg & 0x00FF8000U);
				Resp = 0x00FF8000U;
				if (OpCond(Arg & 0x40000000U))
				{
					Resp |= 0x80000000U | (s_C.Type == T_HC ? 0x40000000U : 0U);
					s_C.State = ST_READY;
				}
				return R_R3;
			case 6:
				if (s_C.State != ST_TRAN || (Arg != 0 && Arg != 2))
				{
					return R_NONE;
				}
				Resp = BusR1();
				if ((Resp & R1_ERRORS) == 0)
				{
					s_C.Width = Arg == 2 ? 4 : 1;
				}
				return R_R1;
			default:
				return R_NONE;
		}
	}
	switch (Idx)
	{
		case 8:
			if (s_C.Type == T_V1 || s_C.State != ST_IDLE)
			{
				return R_NONE;
			}
			Resp = (Arg & 0xFFFU) ^ (s_C.Cmd8Bad ? 1U : 0U);
			return R_R7;
		case 55:
			if (s_C.State != ST_IDLE && RcaArg(Arg) == false)
			{
				return R_NONE;
			}
			s_C.Acmd = s_C.NoAppAt != s_C.Apps++;
			Resp = BusR1();
			return R_R1;
		case 6:
			// Switch function, its status not read by the host
			if (s_C.State != ST_TRAN)
			{
				return R_NONE;
			}
			Resp = BusR1();
			return R_R1;
		case 2:
			if (s_C.State != ST_READY)
			{
				return R_NONE;
			}
			memcpy(pLong, s_Cid, 16);
			s_C.State = ST_IDENT;
			return R_R2;
		case 3:
			if (s_C.State != ST_IDENT && s_C.State != ST_STBY)
			{
				return R_NONE;
			}
			s_C.State = ST_STBY;
			Resp = (CARD_RCA << 16) | 0x0500U;
			return R_R6;
		case 9:
			if (s_C.State != ST_STBY || RcaArg(Arg) == false)
			{
				return R_NONE;
			}
			Csd(pLong);
			return R_R2;
		case 7:
			if (RcaArg(Arg) == false || s_C.State != ST_STBY)
			{
				return R_NONE;
			}
			Resp = BusR1();
			if ((Resp & R1_ERRORS) == 0)
			{
				s_C.State = ST_TRAN;
				s_C.BusyEnd = s_Us + 50;
				s_C.BusyForever = s_C.StuckSelect;
			}
			return R_R1B;
		case 16:
			if (s_C.State != ST_TRAN)
			{
				return R_NONE;
			}
			Resp = BusR1();
			if ((Resp & R1_ERRORS) == 0)
			{
				s_C.BlockLen = Arg;
			}
			return R_R1;
		case 13:
			if (RcaArg(Arg) == false)
			{
				return R_NONE;
			}
			Resp = BusR1();
			return R_R1;
		case 17: case 18: case 24: case 25:
		{
			if (s_C.State != ST_TRAN)
			{
				return R_NONE;
			}

			uint32_t a = AddrOf(Arg);

			if (a == 0xFFFFFFFFU || (s_C.Pend & R1_ERRORS))
			{
				// OUT_OF_RANGE or the error given, no data
				s_C.Pend |= a == 0xFFFFFFFFU ? 0x80000000U : 0U;
				Resp = BusR1();
				return R_R1;
			}
			Resp = BusR1();
			s_C.Addr = a;
			s_C.State = Idx < 20 ? ST_DATA : ST_RCV;
			return R_R1;
		}
		default:
			return R_NONE;
	}
}

// ---- SD/MMC host model ----

static SdmmcDev_t s_Host;
static int s_HostSlot = 0;
static bool s_HostStartFail = false;
static bool s_TxFail = false;		// The next TxData fails

static uint32_t SelectRate(const uint32_t *pRates, int Nb, uint32_t Rate)
{
	uint32_t best = 0;

	for (int i = 0; i < Nb; i++)
	{
		if (pRates[i] <= Rate && pRates[i] > best)
		{
			best = pRates[i];
		}
	}

	return best != 0 ? best : pRates[0];
}

static uint32_t HostSetRate(DevIntrf_t * const, uint32_t Rate)
{
	static const uint32_t r[] = { 400000, 1000000, 12500000, 20000000, 25000000, 50000000 };

	s_Rate = SelectRate(r, sizeof(r) / sizeof(r[0]), Rate);
	return s_Rate;
}

static uint32_t HostGetRate(DevIntrf_t * const) { return s_Rate; }

static bool HostStart(DevIntrf_t * const, uint32_t Slot)
{
	if (s_HostStartFail)
	{
		return false;
	}
	assert((int)Slot == s_HostSlot);
	return true;
}

static bool HostStartTx(DevIntrf_t * const p, uint32_t Slot)
{
	if (HostStart(p, Slot) == false)
	{
		return false;
	}
	s_Host.FrameLen = 0;
	s_Host.bSent = false;
	s_Host.RespLen = 0;
	return true;
}

static void HostStop(DevIntrf_t * const)
{
	s_Host.FrameLen = 0;
	s_Host.bSent = false;
}

// Card busy on data line 0, waited one second
static bool HostWaitBusy()
{
	for (int i = 0; i < 100000 && CardBusy(); i++)
	{
		s_Us += 10;
	}
	return CardBusy() == false;
}

// The command of the frame, with its data
static bool HostRun(uint8_t *pData, int Len)
{
	const uint8_t *f = s_Host.Frame;
	uint8_t crc = crc8_ccitt((uint8_t *)f, 5, 0) | 1U;
	int idx = f[0] & 0x3F;
	uint32_t arg = ((uint32_t)f[1] << 24) | ((uint32_t)f[2] << 16) | ((uint32_t)f[3] << 8) | f[4];
	uint8_t fl = f[SDMMC_FRAME_FLAGS_IDX];
	uint32_t resp = 0;
	uint8_t lng[16];
	bool acmd = s_C.Acmd;

	assert((f[0] & 0xC0) == 0x40 && f[5] == crc);
	assert((fl & SDMMC_FRAME_RESP_MASK) <= SDMMC_FRAME_RESP_R3);
	s_Host.bSent = true;
	s_Host.RespLen = 0;

	// Data direction of the command, the stop on the multiple block commands
	uint32_t dir = idx == 17 || idx == 18 ? SDMMC_FRAME_DATA_READ :
				   idx == 24 || idx == 25 ? SDMMC_FRAME_DATA_WRITE : SDMMC_FRAME_DATA_NONE;

	assert((fl & SDMMC_FRAME_DATA_MASK) == dir);
	assert(((fl & SDMMC_FRAME_STOP) != 0) == (idx == 18 || idx == 25));
	assert(((fl & SDMMC_FRAME_BUS_WIDTH) != 0) == (acmd && idx == 6));
	if (fl & SDMMC_FRAME_BUS_WIDTH)
	{
		assert(s_Host.Cfg.BusWidth >= 4 || arg == 0);
	}

	// The card done with a programming before a command, the status aside
	if (idx != 13 && HostWaitBusy() == false)
	{
		return false;
	}

	if (fl & SDMMC_FRAME_INIT)
	{
		s_Host.BusWidth = 1;
	}

	int kind = BusCardCmd(idx, arg, resp, lng, (fl & SDMMC_FRAME_INIT) != 0);

	if (kind == R_NONE)
	{
		return (fl & SDMMC_FRAME_RESP_MASK) == SDMMC_FRAME_RESP_NONE;
	}

	uint32_t want = kind == R_R1B ? SDMMC_FRAME_RESP_R1B : kind == R_R2 ? SDMMC_FRAME_RESP_R2 :
					kind == R_R3 ? SDMMC_FRAME_RESP_R3 : SDMMC_FRAME_RESP_R1;

	assert((fl & SDMMC_FRAME_RESP_MASK) == want);
	if (kind == R_R2)
	{
		memcpy(s_Host.Resp, lng, 16);
		s_Host.RespLen = 16;
	}
	else
	{
		s_Host.Resp[0] = (uint8_t)(resp >> 24);
		s_Host.Resp[1] = (uint8_t)(resp >> 16);
		s_Host.Resp[2] = (uint8_t)(resp >> 8);
		s_Host.Resp[3] = (uint8_t)resp;
		s_Host.RespLen = 4;
	}
	if (kind == R_R1B && HostWaitBusy() == false)
	{
		s_Host.RespLen = 0;
		return false;
	}
	// The host width set by the response, the card one when it took it
	if (fl & SDMMC_FRAME_BUS_WIDTH)
	{
		s_Host.BusWidth = arg == 2 ? 4 : 1;
	}

	if (dir == SDMMC_FRAME_DATA_NONE)
	{
		return true;
	}
	assert(s_Host.BusWidth == s_C.Width);

	// Data: blocks of 512 bytes, the card refusing it sends nothing
	assert(Len > 0 && Len % 512 == 0 && (Len > 512) == (idx == 18 || idx == 25));
	bool ok = s_C.State == ST_DATA || s_C.State == ST_RCV;

	for (int b = 0; ok && b < Len / 512; b++)
	{
		if (s_C.Addr + 512U > MEM_SIZE)
		{
			ok = false;
		}
		else if (dir == SDMMC_FRAME_DATA_READ)
		{
			ok = s_C.BadCrcBlock != b;
			memcpy(pData + b * 512, &s_Mem[s_C.Addr], 512);
			s_C.BlocksRead++;
		}
		else
		{
			if (s_C.RejectBlock == b)
			{
				ok = false;
				break;
			}
			memcpy(&s_Mem[s_C.Addr], pData + b * 512, 512);
			s_C.BlocksWritten++;
			s_C.BusyEnd = s_Us + s_C.ProgUs;
		}
		s_C.Addr += 512;
	}
	s_C.BadCrcBlock = -1;
	s_C.RejectBlock = -1;
	if (dir == SDMMC_FRAME_DATA_WRITE)
	{
		s_C.Pend |= s_C.WriteErr;
		s_C.WriteErr = 0;
		if (s_C.StuckWrite)
		{
			s_C.BusyForever = true;
			s_C.StuckWrite = false;
		}
	}
	// End of the data: the stop of a multiple block command, or the stop of
	// the host after an error
	if ((fl & SDMMC_FRAME_STOP) || ok == false)
	{
		if (s_C.State == ST_DATA || s_C.State == ST_RCV)
		{
			s_C.Stops++;
		}
	}
	if (s_C.State == ST_DATA || s_C.State == ST_RCV)
	{
		s_C.State = ST_TRAN;
	}
	if (ok && dir == SDMMC_FRAME_DATA_WRITE)
	{
		ok = HostWaitBusy();
	}

	return ok;
}

static int HostTxData(DevIntrf_t * const, const uint8_t *pData, int Len)
{
	int n = 0;

	if (s_TxFail)
	{
		s_TxFail = false;
		return 0;
	}

	if (s_Host.FrameLen < SDMMC_FRAME_SIZE)
	{
		while (s_Host.FrameLen < SDMMC_FRAME_SIZE && n < Len)
		{
			s_Host.Frame[s_Host.FrameLen++] = pData[n++];
		}
		if (s_Host.FrameLen < SDMMC_FRAME_SIZE)
		{
			return n;
		}

		uint8_t dir = s_Host.Frame[SDMMC_FRAME_FLAGS_IDX] & SDMMC_FRAME_DATA_MASK;

		if (dir == SDMMC_FRAME_DATA_NONE)
		{
			assert(n == Len);
			return HostRun(nullptr, 0) ? n : 0;
		}
		if (dir == SDMMC_FRAME_DATA_READ || n == Len)
		{
			return n;
		}
	}
	if ((s_Host.Frame[SDMMC_FRAME_FLAGS_IDX] & SDMMC_FRAME_DATA_MASK) != SDMMC_FRAME_DATA_WRITE || s_Host.bSent)
	{
		return 0;
	}

	return HostRun((uint8_t *)pData + n, Len - n) ? Len : 0;
}

static int HostRxData(DevIntrf_t * const, uint8_t *pBuff, int Len)
{
	if (s_Host.FrameLen < SDMMC_FRAME_SIZE)
	{
		return 0;
	}
	if ((s_Host.Frame[SDMMC_FRAME_FLAGS_IDX] & SDMMC_FRAME_DATA_MASK) == SDMMC_FRAME_DATA_READ)
	{
		return s_Host.bSent == false && HostRun(pBuff, Len) ? Len : 0;
	}
	if (s_Host.bSent == false || s_Host.RespLen == 0)
	{
		return 0;
	}
	assert(Len == s_Host.RespLen);
	memcpy(pBuff, s_Host.Resp, Len);
	return Len;
}

static void HostNop(DevIntrf_t * const) {}
static void *HostHandle(DevIntrf_t * const) { return &s_Host; }

// ---- SPI interface and the card in SPI mode ----

static SPIDev_t s_Spi;
static int s_SpiCs = 0;
static bool s_CsLow = false;
static bool s_SpiStartFail = false;
static std::deque<uint8_t> s_Out;
static uint8_t s_Cmd[6];
static int s_CmdLen = 0;
enum { IN_CMD, IN_TOKEN, IN_DATA };
static int s_In = IN_CMD;
static bool s_InMulti = false;
static uint8_t s_InBuf[514];
static int s_InLen = 0;
static bool s_Stream = false;
static bool s_Stall = false;		// Multiple block read: the card sends no more blocks
static bool s_PastEnd = false;		// Multiple block read past the end of the card
static int s_Cs1 = 0;			// Bytes with the chip select 1
static int s_Tail = 0;			// Bytes after the last byte of a command, response or data

static void Resp(uint8_t R1, const uint8_t *p = nullptr, int n = 0)
{
	for (int i = 0; i < s_C.Ncr; i++)
	{
		s_Out.push_back(0xFF);
	}
	s_Out.push_back(R1);
	for (int i = 0; i < n; i++)
	{
		s_Out.push_back(p[i]);
	}
}

// A data block after Nac bytes: token, data, CRC16
static void Block(const uint8_t *p, int n, bool bBadCrc)
{
	uint16_t crc = crc16_ccitt((uint8_t *)p, n, 0) ^ (bBadCrc ? 1U : 0U);

	for (int i = 0; i < s_C.Nac; i++)
	{
		s_Out.push_back(0xFF);
	}
	s_Out.push_back(0xFE);
	for (int i = 0; i < n; i++)
	{
		s_Out.push_back(p[i]);
	}
	s_Out.push_back((uint8_t)(crc >> 8));
	s_Out.push_back((uint8_t)crc);
}

// Next block of a read
static void ReadBlock()
{
	if (s_C.NoToken == s_Xfer)
	{
		// The card does not send this block
		s_C.NoToken = -1;
		s_Stall = true;
		return;
	}
	if (s_C.Addr + 512U > MEM_SIZE)
	{
		// Past the end: error token, out of range, also in the R1 of CMD12
		s_Out.push_back(0x08);
		s_Stall = true;
		s_PastEnd = true;
		return;
	}
	Block(&s_Mem[s_C.Addr], 512, s_C.BadCrcBlock == s_Xfer);
	if (s_C.BadCrcBlock == s_Xfer)
	{
		s_C.BadCrcBlock = -1;
	}
	s_C.Addr += 512;
	s_C.BlocksRead++;
	s_Xfer++;
}

static void SpiExec()
{
	int idx = s_Cmd[0] & 0x3F;
	uint32_t arg = ((uint32_t)s_Cmd[1] << 24) | ((uint32_t)s_Cmd[2] << 16) | ((uint32_t)s_Cmd[3] << 8) | s_Cmd[4];
	bool acmd = s_C.Acmd;
	int code = acmd ? 100 + idx : idx;
	uint8_t idle = s_C.Ready ? 0 : 1;

	// CRC7 of every command
	assert(s_Cmd[5] == (uint8_t)(crc8_ccitt(s_Cmd, 5, 0) | 1U));
	s_C.Log.push_back(code);
	s_C.Acmd = false;

	if (idx == 0)
	{
		// The first one after 74 clocks or more with the data line high
		assert(s_C.Cmd0Seen || s_C.FfBytes >= 10);
		s_C.Cmd0Seen = true;
		s_C.Spi = true;
		s_C.State = ST_IDLE;
		s_C.Ready = false;
		s_C.OpCount = 0;
		if (s_C.NoResp == code)
		{
			s_C.NoResp = -1;
			return;
		}
		Resp(s_C.Cmd0R1 >= 0 ? (uint8_t)s_C.Cmd0R1 : 0x01);
		return;
	}
	assert(s_C.Spi);
	CheckRate();
	if (s_C.NoResp == code)
	{
		s_C.NoResp = -1;
		if (idx == 12)
		{
			// The data stops all the same
			s_Stream = false;
			s_Stall = false;
			s_Out.clear();
		}
		return;
	}

	uint8_t err = 0;

	if (s_C.ErrCmd == code)
	{
		s_C.ErrCmd = -1;
		err = (uint8_t)s_C.ErrBits;
	}
	if (acmd)
	{
		assert(idx == 41);
		if (err)
		{
			Resp(idle | err);
			return;
		}
		Resp(OpCond(arg & 0x40000000U) ? 0x00 : 0x01);
		if (s_C.Ready)
		{
			s_C.State = ST_TRAN;
		}
		if (s_C.Type == T_V1)
		{
			assert((arg & 0x40000000U) == 0);
		}
		return;
	}
	switch (idx)
	{
		case 1:
			// MMC, also taken by the SD cards in SPI mode
			if (err)
			{
				Resp(idle | err);
				return;
			}
			Resp(OpCond(false) ? 0x00 : 0x01);
			if (s_C.Ready)
			{
				s_C.State = ST_TRAN;
			}
			return;
		case 8:
			if (s_C.Type == T_MMC || s_C.Type == T_V1)
			{
				Resp(idle | 0x04);
				return;
			}
			{
				uint8_t r7[4] = { 0, 0, (uint8_t)((arg >> 8) & 0xF), (uint8_t)((arg & 0xFF) ^ (s_C.Cmd8Bad ? 1 : 0)) };

				Resp(idle | err, r7, 4);
			}
			return;
		case 55:
			if (s_C.Type == T_MMC)
			{
				Resp(idle | 0x04);
				return;
			}
			s_C.Acmd = true;
			Resp(idle | err);
			return;
		case 58:
		{
			uint32_t ocr = 0x00FF8000U | (s_C.Ready ? 0x80000000U : 0U) |
						   (s_C.Ready && s_C.Type == T_HC ? 0x40000000U : 0U);
			uint8_t o[4] = { (uint8_t)(ocr >> 24), (uint8_t)(ocr >> 16), (uint8_t)(ocr >> 8), (uint8_t)ocr };

			Resp(idle | err, o, 4);
			return;
		}
		case 12:
			if (s_Stream)
			{
				// The data stopped, a stuff byte, R1, then busy
				s_Stream = false;
				s_Stall = false;
				s_Out.clear();
				s_Out.push_back(0x3C);
				Resp(err | (s_PastEnd ? 0x40 : 0));
				s_PastEnd = false;
				s_C.BusyEnd = s_Us + 16 * 8e6 / s_Rate;
				s_C.BusyForever = s_C.StopStuck;
				s_C.StopStuck = false;
				s_C.Stops++;
				return;
			}
			Resp(idle | 0x04);
			return;
		case 13:
			Resp(idle | err, &s_C.Pend2, 1);
			s_C.Pend2 = 0;
			return;
		default:
			break;
	}

	// The commands of a card ready
	assert(s_C.Ready);
	switch (idx)
	{
		case 9:
		case 10:
		{
			uint8_t r[16];

			if (idx == 9)
			{
				Csd(r);
			}
			else
			{
				memcpy(r, s_Cid, 16);
			}
			Resp(err);
			if (err == 0)
			{
				Block(r, 16, false);
			}
			return;
		}
		case 16:
			s_C.BlockLen = arg;
			Resp(err);
			return;
		case 17:
		case 18:
		case 24:
		case 25:
		{
			uint32_t a = AddrOf(arg);

			if (a == 0xFFFFFFFFU || err)
			{
				Resp(err | (a == 0xFFFFFFFFU ? 0x40 : 0));
				return;
			}
			Resp(0);
			s_C.Addr = a;
			s_Xfer = 0;
			if (idx == 17)
			{
				ReadBlock();
			}
			else if (idx == 18)
			{
				s_Stream = true;
			}
			else
			{
				s_In = IN_TOKEN;
				s_InMulti = idx == 25;
			}
			return;
		}
		default:
			Resp(0x04);
			return;
	}
}

// A block written: data response, then busy while the card programs
static void WriteBlock()
{
	uint16_t crc = (uint16_t)((s_InBuf[512] << 8) | s_InBuf[513]);

	assert(crc == crc16_ccitt(s_InBuf, 512, 0));
	if (s_C.RejectBlock == s_Xfer)
	{
		s_C.RejectBlock = -1;
		s_Out.push_back(0xED);		// Write error
	}
	else
	{
		assert(s_C.Addr + 512U <= MEM_SIZE);
		memcpy(&s_Mem[s_C.Addr], s_InBuf, 512);
		s_C.Addr += 512;
		s_C.BlocksWritten++;
		s_Out.push_back(0xE5);
		s_C.BusyEnd = s_Us + s_C.ProgUs;
		if (s_C.StuckWrite)
		{
			s_C.StuckWrite = false;
			s_C.BusyForever = true;
		}
		s_C.Pend2 |= (uint8_t)s_C.WriteErr;
		s_C.WriteErr = 0;
	}
	s_Xfer++;
	s_In = s_InMulti ? IN_TOKEN : IN_CMD;
}

static uint8_t SpiByte(uint8_t Mosi)
{
	assert(s_CsLow);
	s_Us += 8e6 / s_Rate;
	if (s_SpiCs != 0)
	{
		s_Cs1++;
	}
	if (s_C.Present == false)
	{
		return 0xFF;
	}

	uint8_t miso;

	if (s_Out.empty() && s_Stream && s_Stall == false)
	{
		ReadBlock();
	}
	if (s_Out.empty() == false)
	{
		miso = s_Out.front();
		s_Out.pop_front();
		s_Tail = 0;
	}
	else
	{
		miso = CardBusy() ? 0x00 : 0xFF;
		s_Tail = Mosi != 0xFF ? 0 : s_Tail + 1;
	}

	switch (s_In)
	{
		case IN_CMD:
			if (s_CmdLen == 0)
			{
				if ((Mosi & 0xC0) == 0x40)
				{
					assert(CardBusy() == false || (Mosi & 0x3F) == 13);
					s_Cmd[s_CmdLen++] = Mosi;
				}
				else
				{
					// The data line high when the host only reads
					assert(Mosi == 0xFF);
					if (s_C.Cmd0Seen == false)
					{
						s_C.FfBytes++;
					}
				}
			}
			else
			{
				s_Cmd[s_CmdLen++] = Mosi;
				if (s_CmdLen == 6)
				{
					s_CmdLen = 0;
					SpiExec();
				}
			}
			break;
		case IN_TOKEN:
			if (Mosi == 0xFF)
			{
				break;
			}
			// A start token when the card is ready, the stop token at any time
			assert(CardBusy() == false || Mosi == 0xFD);
			if (Mosi == (s_InMulti ? 0xFC : 0xFE))
			{
				s_In = IN_DATA;
				s_InLen = 0;
			}
			else
			{
				// The end of a multiple block write, the card busy after a byte
				assert(s_InMulti && Mosi == 0xFD);
				s_In = IN_CMD;
				s_C.BusyEnd = s_Us + 8 * 8e6 / s_Rate + s_C.ProgUs;
				s_C.Stops++;
			}
			break;
		case IN_DATA:
			s_InBuf[s_InLen++] = Mosi;
			if (s_InLen == 514)
			{
				WriteBlock();
			}
			break;
	}

	return miso;
}

static bool SpiStart(DevIntrf_t * const, uint32_t Cs)
{
	if (s_SpiStartFail)
	{
		return false;
	}
	assert((int)Cs == s_SpiCs);
	s_CsLow = true;
	return true;
}

static void SpiStop(DevIntrf_t * const)
{
	// Whole commands, a multiple block read stopped, 8 clocks after the last
	// byte of the transaction
	assert(s_CmdLen == 0 && s_Stream == false);
	assert(s_C.Present == false || s_C.Ncr > 8 || s_Tail >= 1);
	s_CsLow = false;
	s_Out.clear();
	if (s_In != IN_CMD)
	{
		// A write left before its data, a failure only
		s_In = IN_CMD;
	}
}

static int SpiTxData(DevIntrf_t * const, const uint8_t *pData, int Len)
{
	if (s_TxFail)
	{
		s_TxFail = false;
		return 0;
	}
	for (int i = 0; i < Len; i++)
	{
		(void)SpiByte(pData[i]);
	}
	return Len;
}

static int SpiRxData(DevIntrf_t * const, uint8_t *pBuff, int Len)
{
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = SpiByte(s_Spi.Cfg.DummyByte);
	}
	return Len;
}

static bool s_SpiClosest = false;	// Rates of an nRF SPIM, the closest one taken

static uint32_t SpiSetRate(DevIntrf_t * const, uint32_t Rate)
{
	static const uint32_t r[] = { 250000, 400000, 1000000, 4000000, 8000000, 16000000, 20000000, 32000000 };
	static const uint32_t n[] = { 125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000 };

	if (s_SpiClosest)
	{
		uint32_t err = 0xFFFFFFFFU;

		for (uint32_t f : n)
		{
			uint32_t d = f > Rate ? f - Rate : Rate - f;

			if (d < err)
			{
				err = d;
				s_Rate = f;
			}
		}
		return s_Rate;
	}
	s_Rate = SelectRate(r, sizeof(r) / sizeof(r[0]), Rate);
	return s_Rate;
}

static void *SpiHandle(DevIntrf_t * const) { return &s_Spi; }

// ---- Interfaces ----

class ModelIntrf : public DeviceIntrf {
public:
	DevIntrf_t *p;
	operator DevIntrf_t * () override { return p; }
	uint32_t Rate(uint32_t R) override { return p->SetRate(p, R); }
	uint32_t Rate(void) override { return p->GetRate(p); }
};

static ModelIntrf s_SpiIntrf, s_BusIntrf;

static void Table(DevIntrf_t *p)
{
	p->Disable = HostNop;
	p->Enable = HostNop;
	p->Reset = HostNop;
	p->PowerOff = HostNop;
	p->MaxRetry = 0;
	atomic_flag_clear(&p->bBusy);
}

static void SetupSpi(uint32_t Rate = 20000000)
{
	memset((void *)&s_Spi, 0, sizeof(s_Spi));
	DevIntrf_t *p = &s_Spi.DevIntrf;

	Table(p);
	p->Type = DEVINTRF_TYPE_SPI;
	p->GetRate = HostGetRate;
	p->SetRate = SpiSetRate;
	p->StartRx = SpiStart;
	p->StartTx = SpiStart;
	p->StopRx = SpiStop;
	p->StopTx = SpiStop;
	p->RxData = SpiRxData;
	p->TxData = SpiTxData;
	p->GetHandle = SpiHandle;
	s_Spi.Cfg.DummyByte = 0xFF;
	s_SpiIntrf.p = p;
	s_SpiCs = 0;
	s_CsLow = false;
	s_SpiStartFail = false;
	s_TxFail = false;
	s_Tail = 0;
	s_Out.clear();
	s_CmdLen = 0;
	s_In = IN_CMD;
	s_Stream = false;
	s_Stall = false;
	s_PastEnd = false;
	SpiSetRate(p, Rate);
}

static void SetupHost(int Width, uint32_t Rate = 25000000)
{
	memset((void *)&s_Host, 0, sizeof(s_Host));
	DevIntrf_t *p = &s_Host.DevIntrf;

	Table(p);
	p->Type = DEVINTRF_TYPE_SDMMC;
	p->GetRate = HostGetRate;
	p->SetRate = HostSetRate;
	p->StartRx = HostStart;
	p->StartTx = HostStartTx;
	p->StopRx = HostStop;
	p->StopTx = HostStop;
	p->RxData = HostRxData;
	p->TxData = HostTxData;
	p->GetHandle = HostHandle;
	s_Host.Cfg.BusWidth = Width;
	s_Host.BusWidth = 1;
	s_BusIntrf.p = p;
	s_HostSlot = 0;
	s_HostStartFail = false;
	s_TxFail = false;
	HostSetRate(p, Rate);
}

// A new card, powered up
static void Fresh(int Type)
{
	s_C = CardM();
	s_C.Type = Type;
	s_Us = 0;
}

// The busy flag of the interface released, no transfer left open
static void Released(DevIntrf_t *p)
{
	assert(atomic_flag_test_and_set(&p->bBusy) == false);
	atomic_flag_clear(&p->bBusy);
	assert(p->bNoStop == false);
	assert(s_CsLow == false);
}

static bool Same(const std::vector<int> &Log, std::vector<int> Exp)
{
	if (Log != Exp)
	{
		for (int c : Log)
		{
			fprintf(stderr, " %d", c);
		}
		fprintf(stderr, "\n");
		return false;
	}
	return true;
}

static void Fill(uint8_t *p, uint32_t Len, uint32_t Seed)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		Seed = Seed * 1103515245U + 12345U;
		p[i] = (uint8_t)(Seed >> 16);
	}
}

// ---- Tests ----

static ModelIntrf *Intrf(bool Bus) { return Bus ? &s_BusIntrf : &s_SpiIntrf; }

static void Setup(bool Bus, int Width = 4)
{
	if (Bus)
	{
		SetupHost(Width);
	}
	else
	{
		SetupSpi();
	}
}

static void TestInitSpi()
{
	SDCard sd;

	// SDHC: CMD8, ACMD41 with HCS, CMD58 with CCS, no CMD16
	Fresh(T_HC);
	SetupSpi(20000000);
	assert(sd.Init(&s_SpiIntrf));
	assert(sd.Type() == SDCARD_TYPE_SDHC && sd.GetNbSect() == NB_SECT && sd.GetSize() == MEM_SIZE / 1024U);
	assert(sd.GetSectSize() == 512 && sd.Rate() == 20000000 && s_Rate == 20000000);
	assert(Same(s_C.Log, { 0, 8, 55, 141, 55, 141, 55, 141, 58, 9, 10 }));
	assert(memcmp(sd.Cid(), s_Cid, 16) == 0 && (sd.Csd()[0] >> 6) == 1);
	assert(((const SDCardDev_t *)sd)->Ocr & 0x40000000U);
	assert(s_C.MaxIdRate == 400000 && s_C.FfBytes >= 10);
	Released(&s_Spi.DevIntrf);

	// SDSC version 2: CMD16
	Fresh(T_V2);
	SetupSpi(32000000);
	assert(sd.Init(&s_SpiIntrf));
	assert(sd.Type() == SDCARD_TYPE_SDSC && sd.GetNbSect() == NB_SECT);
	// The card maximum, 25 MHz: 20 MHz on the interface
	assert(sd.Rate() == 20000000);
	assert(Same(s_C.Log, { 0, 8, 55, 141, 55, 141, 55, 141, 58, 9, 10, 16 }));

	// SDSC version 1: no CMD58, HCS 0
	Fresh(T_V1);
	s_C.OpBusy = 0;
	SetupSpi();
	assert(sd.Init(&s_SpiIntrf));
	assert(sd.Type() == SDCARD_TYPE_SDSC_V1 && sd.GetNbSect() == NB_SECT);
	assert(Same(s_C.Log, { 0, 8, 55, 141, 9, 10, 16 }));

	// MMC: CMD1
	Fresh(T_MMC);
	SetupSpi();
	assert(sd.Init(&s_SpiIntrf));
	assert(sd.Type() == SDCARD_TYPE_MMC && sd.GetNbSect() == NB_SECT);
	assert(Same(s_C.Log, { 0, 8, 55, 1, 1, 1, 9, 10, 16 }));

	// A slower card: TRAN_SPEED 10 Mb/s
	Fresh(T_HC);
	s_C.TranSpeed = 0x0A;
	SetupSpi(20000000);
	assert(sd.Init(&s_SpiIntrf));
	assert(sd.Rate() == 8000000);

	// Power up in 300 ms
	Fresh(T_HC);
	s_C.OpBusy = 0;
	s_C.OpMs = 300;
	SetupSpi();
	assert(sd.Init(&s_SpiIntrf));
	assert(s_Us >= 300000);

	// CMD0 answered the second time, a response 8 bytes after the command
	Fresh(T_HC);
	s_C.NoResp = 0;
	s_C.Ncr = 8;
	SetupSpi();
	assert(sd.Init(&s_SpiIntrf));
	assert(s_C.Log[0] == 0 && s_C.Log[1] == 0);

	// An interface giving the closest clock (nRF SPIM): 500 kHz for 400 kHz,
	// asked again lower, identification at 250 kHz
	Fresh(T_HC);
	s_SpiClosest = true;
	SetupSpi(8000000);
	assert(sd.Init(&s_SpiIntrf));
	assert(s_C.MaxIdRate == 250000 && sd.Rate() == 8000000);
	// A card of 3.5 MHz: 4 MHz the closest, then 2 MHz
	Fresh(T_HC);
	s_C.TranSpeed = 0x41;
	SetupSpi(8000000);
	assert(sd.Init(&s_SpiIntrf));
	uint8_t blk[512];
	assert(sd.Rate() == 2000000 && sd.ReadSect(1, blk, 1) == 1 && s_C.MaxDataRate == 2000000);
	Released(&s_Spi.DevIntrf);
	s_SpiClosest = false;

	// Chip select 1
	Fresh(T_HC);
	SetupSpi();
	s_SpiCs = 1;
	s_Cs1 = 0;
	sd.DeviceAddress(1);
	assert(sd.DeviceAddress() == 1);
	assert(sd.Init(&s_SpiIntrf) && s_Cs1 > 0);
	sd.DeviceAddress(0);

	printf("PASS : SPI mode, identification\r\n");
}

static void TestInitBus()
{
	SDCard sd;

	// SDHC, 4 data lines: no CMD16
	Fresh(T_HC);
	SetupHost(4, 50000000);
	assert(sd.Init(&s_BusIntrf));
	assert(sd.Type() == SDCARD_TYPE_SDHC && sd.GetNbSect() == NB_SECT && sd.GetSize() == MEM_SIZE / 1024U);
	assert(sd.Rate() == 25000000 && s_C.Width == 4 && s_Host.BusWidth == 4);
	assert(Same(s_C.Log, { 0, 8, 55, 141, 55, 141, 55, 141, 2, 3, 9, 7, 55, 106 }));
	assert(memcmp(sd.Cid(), s_Cid, 16) == 0);
	assert(((const SDCardDev_t *)sd)->Rca == CARD_RCA << 16);
	assert(s_C.MaxIdRate == 400000);
	Released(&s_Host.DevIntrf);

	// SDSC version 2, 1 data line: no ACMD6, CMD16
	Fresh(T_V2);
	SetupHost(1, 20000000);
	assert(sd.Init(&s_BusIntrf));
	assert(sd.Type() == SDCARD_TYPE_SDSC && s_C.Width == 1 && sd.Rate() == 20000000);
	assert(Same(s_C.Log, { 0, 8, 55, 141, 55, 141, 55, 141, 2, 3, 9, 7, 16 }));

	// SDSC version 1: no answer to CMD8
	Fresh(T_V1);
	SetupHost(4);
	assert(sd.Init(&s_BusIntrf));
	assert(sd.Type() == SDCARD_TYPE_SDSC_V1 && s_C.Width == 4);
	assert(Same(s_C.Log, { 0, 8, 55, 141, 55, 141, 55, 141, 2, 3, 9, 7, 55, 106, 16 }));

	// No rate set on the interface before: the card maximum
	Fresh(T_HC);
	SetupHost(4);
	s_Rate = 0;
	assert(sd.Init(&s_BusIntrf));
	assert(sd.Rate() == 25000000);

	// TRAN_SPEED 100 Mb/s: the 50 MHz of the host
	Fresh(T_HC);
	s_C.TranSpeed = 0x0B;
	SetupHost(4, 50000000);
	assert(sd.Init(&s_BusIntrf) && sd.Rate() == 50000000);

	// A reserved TRAN_SPEED: 25 MHz
	Fresh(T_HC);
	s_C.TranSpeed = 0x00;
	SetupHost(4, 50000000);
	assert(sd.Init(&s_BusIntrf) && sd.Rate() == 25000000);
	Fresh(T_HC);
	s_C.TranSpeed = 0x34;
	SetupHost(4, 50000000);
	assert(sd.Init(&s_BusIntrf) && sd.Rate() == 25000000);

	// Slot 1
	Fresh(T_HC);
	SetupHost(4);
	s_HostSlot = 1;
	sd.DeviceAddress(1);
	assert(sd.Init(&s_BusIntrf));
	sd.DeviceAddress(0);

	printf("PASS : SD bus, identification\r\n");
}

static void InitFail(bool Bus, int Type, void (*Fault)(), int Width = 4)
{
	SDCard sd;
	uint8_t b[512];

	Fresh(Type);
	Setup(Bus, Width);
	Fault();

	uint32_t before = s_Rate;

	assert(sd.Init(Intrf(Bus)) == false);
	// The rate of the interface back, no transfers
	assert(s_Rate == before && sd.Type() == SDCARD_TYPE_NONE);
	assert(sd.ReadSect(0, b, 1) == 0 && sd.WriteSect(0, b, 1) == 0);
	Released(*Intrf(Bus));
}

static uint32_t s_BusErr;

static void TestInitFail()
{
	for (int bus = 0; bus < 2; bus++)
	{
		InitFail(bus, T_HC, [] { s_C.Present = false; });
		InitFail(bus, T_HC, [] { s_C.Cmd8Bad = true; });
		InitFail(bus, T_HC, [] { s_C.NeverReady = true; });
		assert(s_Us >= 990000.0);
		InitFail(bus, T_HC, [] { s_C.CsdVer = 0; });
		InitFail(bus, T_HC, [] { s_C.CsdVer = 2; });
		InitFail(bus, T_V2, [] { s_C.CsdVer = 1; });
		InitFail(bus, T_V2, [] { s_C.BlLen = 8; });
		InitFail(bus, T_V2, [] { s_C.BlLen = 12; });
		InitFail(bus, T_HC, [] { s_C.NoResp = 8; s_C.Type = T_HC; });
		InitFail(bus, T_HC, [] { s_C.NoResp = 141; });
		InitFail(bus, T_HC, [] { s_C.NoResp = 9; });
		InitFail(bus, T_V2, [] { s_C.NoResp = 16; });
		s_BusErr = bus ? 0x20000000U : 0x40U;
		InitFail(bus, T_V2, [] { s_C.ErrCmd = 16; s_C.ErrBits = s_BusErr; });
		InitFail(bus, T_HC, [] { s_HostStartFail = true; s_SpiStartFail = true; });
	}
	// SPI mode
	InitFail(false, T_HC, [] { s_C.NoResp = 58; });
	InitFail(false, T_HC, [] { s_C.NoResp = 10; });
	InitFail(false, T_HC, [] { s_C.ErrCmd = 55; s_C.ErrBits = 0x08; });
	InitFail(false, T_HC, [] { s_C.ErrCmd = 9; s_C.ErrBits = 0x08; });
	InitFail(false, T_HC, [] { s_C.ErrCmd = 10; s_C.ErrBits = 0x08; });
	InitFail(false, T_HC, [] { s_C.ErrCmd = 58; s_C.ErrBits = 0x08; });
	InitFail(false, T_MMC, [] { s_C.NeverReady = true; });
	InitFail(false, T_HC, [] { s_C.Ncr = 10; });
	InitFail(false, T_HC, [] { s_C.Cmd0R1 = 0x09; });
	InitFail(false, T_HC, [] { s_TxFail = true; });
	InitFail(false, T_V2, [] { s_C.NoResp = 8; });
	InitFail(false, T_V2, [] { s_C.ErrCmd = 8; s_C.ErrBits = 0x08; });
	InitFail(false, T_V2, [] { s_C.ErrCmd = 141; s_C.ErrBits = 0x04; });
	InitFail(false, T_V1, [] { s_C.NoResp = 141; });
	InitFail(false, T_V1, [] { s_C.ErrCmd = 141; s_C.ErrBits = 0x08; });
	InitFail(false, T_MMC, [] { s_C.ErrCmd = 1; s_C.ErrBits = 0x08; });
	assert(s_Us < 100000.0);
	// SD bus
	InitFail(true, T_MMC, [] {});
	InitFail(true, T_HC, [] { s_C.NoResp = 2; });
	InitFail(true, T_HC, [] { s_C.NoResp = 3; });
	InitFail(true, T_HC, [] { s_C.NoResp = 7; });
	InitFail(true, T_HC, [] { s_C.NoResp = 106; });
	InitFail(true, T_HC, [] { s_C.NoResp = 55; });
	InitFail(true, T_HC, [] { s_C.NoAppAt = 0; });
	// The CMD55 of ACMD6: the card takes CMD6 as the switch function
	InitFail(true, T_HC, [] { s_C.NoAppAt = 3; });
	InitFail(true, T_HC, [] { s_C.ErrCmd = 55; s_C.ErrBits = 0x00400000U; });
	InitFail(true, T_HC, [] { s_C.ErrCmd = 7; s_C.ErrBits = 0x00080000U; });
	InitFail(true, T_HC, [] { s_C.ErrCmd = 7; s_C.ErrBits = 0x00080000U; }, 1);
	InitFail(true, T_HC, [] { s_TxFail = true; });
	InitFail(true, T_HC, [] { s_C.ErrCmd = 106; s_C.ErrBits = 0x00400000U; });
	InitFail(true, T_HC, [] { s_C.ErrCmd = 106; s_C.ErrBits = 0x00000008U; });
	InitFail(true, T_HC, [] { s_C.StuckSelect = true; });

	// Not an SPI or SD/MMC interface
	{
		SDCard sd;

		Fresh(T_HC);
		SetupSpi();
		s_Spi.DevIntrf.Type = DEVINTRF_TYPE_I2C;
		assert(sd.Init(&s_SpiIntrf) == false && s_C.Log.empty());
		assert(sd.Init((DeviceIntrf *)nullptr) == false);
	}

	printf("PASS : identification failures\r\n");
}

static void TestTransfer(bool Bus, int Type)
{
	static uint8_t out[8 * 512], in[8 * 512];
	SDCard sd;

	Fresh(Type);
	Setup(Bus);
	assert(sd.Init(Intrf(Bus)));

	// Single and multiple blocks, at both ends of the card
	const uint32_t sect[] = { 0, 1, 77, NB_SECT - 8, NB_SECT - 1 };

	for (uint32_t s : sect)
	{
		for (uint32_t n : { 1U, 2U, 8U })
		{
			if (s + n > NB_SECT)
			{
				continue;
			}
			Fill(out, n * 512, s * 7 + n);
			s_C.Log.clear();
			int stops = s_C.Stops;
			assert(sd.WriteSect(s, out, n) == (int)n);
			assert(memcmp(&s_Mem[s * 512], out, n * 512) == 0);
			assert(s_C.Log.front() == (n > 1 ? 25 : 24) && s_C.Log.back() == 13);
			assert(s_C.Stops == stops + (n > 1 ? 1 : 0));
			memset(in, 0, sizeof(in));
			s_C.Log.clear();
			assert(sd.ReadSect(s, in, n) == (int)n);
			assert(memcmp(in, out, n * 512) == 0);
			assert(Same(s_C.Log, n > 1 ? (Bus ? std::vector<int>{ 18 } : std::vector<int>{ 18, 12 }) :
										  std::vector<int>{ 17 }));
			assert(s_C.Stops == stops + (n > 1 ? 2 : 0));
			assert(s_C.MaxDataRate <= 25000000 && s_C.MaxDataRate > 400000);
			Released(*Intrf(Bus));
		}
	}

	// One sector, the DiskIO calls
	Fill(out, 512, 99);
	assert(sd.SectWrite(5, out) && sd.SectRead(5, in) && memcmp(in, out, 512) == 0);
	assert(sd.WriteSingleBlock(6, out, 512) == 512 && sd.ReadSingleBlock(6, in, 512) == 512);
	assert(memcmp(&s_Mem[6 * 512], out, 512) == 0);
	assert(sd.WriteSingleBlock(6, out, 511) == 0 && sd.ReadSingleBlock(6, in, 1024) == 0);

	// Out of the card or no buffer: no command
	s_C.Log.clear();
	assert(sd.ReadSect(NB_SECT, in, 1) == 0 && sd.ReadSect(NB_SECT - 1, in, 2) == 0);
	assert(sd.WriteSect(NB_SECT, out, 1) == 0 && sd.WriteSect(NB_SECT - 2, out, 3) == 0);
	assert(sd.ReadSect(0, in, 0) == 0 && sd.WriteSect(0, out, 0) == 0);
	assert(sd.ReadSect(0, nullptr, 1) == 0 && sd.WriteSect(0, nullptr, 1) == 0);
	assert(sd.ReadSect(0xFFFFFFFFU, in, 2) == 0);
	assert(s_C.Log.empty());

	// Status: ready, in its transfer state
	uint32_t st = sd.Status();

	assert(st == ((const SDCardDev_t *)sd)->Status);
	if (Bus)
	{
		assert(((st >> 9) & 15) == ST_TRAN && (st & R1_ERRORS) == 0);
	}
	else
	{
		assert(st == 0);
	}

	printf("PASS : %s, %s transfers\r\n", Bus ? "SD bus" : "SPI mode",
		   Type == T_HC ? "SDHC" : Type == T_V2 ? "SDSC" : Type == T_V1 ? "SDSC version 1" : "MMC");
}

static void TestFault(bool Bus)
{
	static uint8_t out[4 * 512], in[4 * 512];
	SDCard sd;

	Fresh(T_HC);
	Setup(Bus);
	assert(sd.Init(Intrf(Bus)));
	Fill(out, sizeof(out), 5);

	// Data CRC error on the third block of a read
	s_C.BadCrcBlock = 2;
	s_C.Log.clear();
	int n = sd.ReadSect(100, in, 4);
	assert(Bus ? n == 0 : n == 2);
	assert(s_C.Log.back() == 13);
	Released(*Intrf(Bus));
	assert(sd.ReadSect(100, in, 4) == 4);

	// One block
	s_C.BadCrcBlock = 0;
	assert(sd.ReadSect(100, in, 1) == 0);
	assert(sd.ReadSect(100, in, 1) == 1);

	// An error in the response of the read command: no data, no wait
	double t0 = s_Us;
	s_C.ErrCmd = 17;
	s_C.ErrBits = Bus ? 0x40000000U : 0x20U;
	assert(sd.ReadSect(3, in, 1) == 0);
	s_C.ErrCmd = 18;
	s_C.ErrBits = Bus ? 0x40000000U : 0x20U;
	assert(sd.ReadSect(3, in, 2) == 0);
	assert(s_Us - t0 < 50000.0);
	s_C.ErrCmd = 24;
	s_C.ErrBits = Bus ? 0x04000000U : 0x20U;
	assert(sd.WriteSect(3, out, 1) == 0);
	s_C.ErrCmd = 25;
	s_C.ErrBits = Bus ? 0x04000000U : 0x20U;
	assert(sd.WriteSect(3, out, 2) == 0);
	Released(*Intrf(Bus));

	// No answer to the read command
	s_C.NoResp = 17;
	assert(sd.ReadSect(3, in, 1) == 0);
	s_C.NoResp = 18;
	assert(sd.ReadSect(3, in, 2) == 0);
	assert(sd.ReadSect(3, in, 2) == 2);

	// The write refused on its second block
	s_C.RejectBlock = 1;
	n = sd.WriteSect(200, out, 3);
	assert(Bus ? n == 0 : n == 1);
	Released(*Intrf(Bus));
	assert(sd.WriteSect(200, out, 3) == 3 && memcmp(&s_Mem[200 * 512], out, 3 * 512) == 0);
	s_C.RejectBlock = 0;
	assert(sd.WriteSect(200, out, 1) == 0);

	// Programming errors reported by the status
	s_C.WriteErr = Bus ? 0x04000000U : 0x20U;		// WP_VIOLATION
	assert(sd.WriteSect(300, out, 2) == 0);
	s_C.WriteErr = Bus ? 0x04000000U : 0x20U;
	assert(sd.WriteSect(300, out, 1) == 0);
	assert(sd.WriteSect(300, out, 1) == 1);

	// No answer to the write command
	s_C.NoResp = 25;
	assert(sd.WriteSect(300, out, 2) == 0);
	s_C.NoResp = 24;
	assert(sd.WriteSect(300, out, 1) == 0);

	// Interface busy
	s_HostStartFail = s_SpiStartFail = true;
	assert(sd.ReadSect(3, in, 1) == 0 && sd.WriteSect(3, out, 1) == 0);
	assert(sd.Status() == 0xFFFFFFFFU);
	s_HostStartFail = s_SpiStartFail = false;
	Released(*Intrf(Bus));

	// The card stays busy after a write: one second, then the failure
	s_C.StuckWrite = true;
	double t = s_Us;
	assert(sd.WriteSect(400, out, 1) == 0);
	assert(s_Us - t >= 490000.0 && s_Us - t < 5e6);
	Released(*Intrf(Bus));

	printf("PASS : %s, faults\r\n", Bus ? "SD bus" : "SPI mode");
}

static void TestSpiFault()
{
	static uint8_t out[4 * 512], in[4 * 512];
	SDCard sd;

	Fresh(T_V2);
	SetupSpi();
	assert(sd.Init(&s_SpiIntrf));

	// No start token for the second block: 100 ms of clocks, then the stop
	s_C.NoToken = 1;
	double t = s_Us;
	assert(sd.ReadSect(10, in, 3) == 1);
	assert(s_Us - t >= 99000.0);
	assert(s_C.Log.back() == 13);

	// A data block at the latest
	s_C.Nac = 1000;
	assert(sd.ReadSect(10, in, 3) == 3);
	s_C.Nac = 2;

	// A failed stop
	s_C.NoResp = 12;
	assert(sd.ReadSect(10, in, 2) == 0);
	assert(sd.ReadSect(10, in, 2) == 2);
	s_C.ErrCmd = 12;
	s_C.ErrBits = 0x08;
	assert(sd.ReadSect(10, in, 2) == 0);
	// Out of range in the R1 of the stop is not an error
	s_C.ErrCmd = 12;
	s_C.ErrBits = 0x40;
	assert(sd.ReadSect(10, in, 2) == 2);
	s_C.ErrCmd = 12;
	s_C.ErrBits = 0x20;
	assert(sd.ReadSect(10, in, 2) == 2);

	// The card stays busy after the stop
	s_C.StopStuck = true;
	assert(sd.ReadSect(10, in, 2) == 0);
	s_C.BusyForever = false;

	// The status unreadable after a write
	s_C.NoResp = 13;
	assert(sd.WriteSect(10, out, 1) == 0);
	// An R1 error on the status
	s_C.ErrCmd = 13;
	s_C.ErrBits = 0x40;
	assert(sd.WriteSect(10, out, 1) == 0);
	assert(sd.Status() == 0);
	// Card locked: not an error
	s_C.WriteErr = 0x01;
	assert(sd.WriteSect(10, out, 1) == 1);

	// The card stays busy after the stop token
	s_C.ProgUs = 2e6;
	assert(sd.WriteSect(20, out, 2) == 0);
	s_C.BusyEnd = 0;
	s_C.ProgUs = 300;
	Released(&s_Spi.DevIntrf);

	printf("PASS : SPI mode, tokens and timeouts\r\n");
}

static void TestCache(bool Bus)
{
	static uint8_t mem[3 * 512 + 100];
	uint8_t b[700], c[700];
	SDCard sd;

	Fresh(T_HC);
	Setup(Bus);
	// Room for 3 sectors, 2 used
	assert(sd.Init(Intrf(Bus), mem, sizeof(mem)));
	Fill(b, sizeof(b), 3);
	assert(sd.Write((uint64_t)1000, b, sizeof(b)) == (int)sizeof(b));
	sd.Flush();
	assert(memcmp(&s_Mem[1000], b, sizeof(b)) == 0);
	assert(sd.Read((uint64_t)1000, c, sizeof(c)) == (int)sizeof(c) && memcmp(b, c, sizeof(b)) == 0);

	// Less than a sector: no cache
	SDCard sd2;

	Fresh(T_HC);
	Setup(Bus);
	assert(sd2.Init(Intrf(Bus), mem, 100));
	assert(sd2.Read((uint64_t)1000, c, 10) == 10 && memcmp(b, c, 10) == 0);
	int rd = s_C.BlocksRead;
	assert(sd2.Read((uint64_t)1000, c, 10) == 10 && s_C.BlocksRead == rd + 1);
	assert(sd2.Init(Intrf(Bus), (uint8_t *)nullptr, 512));
	assert(sd2.Read((uint64_t)1000, c, 10) == 10 && memcmp(b, c, 10) == 0);

	// With the cache, a sector read once
	rd = s_C.BlocksRead;
	assert(sd.Read((uint64_t)3000, c, 10) == 10 && sd.Read((uint64_t)3010, c, 10) == 10);
	assert(s_C.BlocksRead == rd + 1);

	// A new card in the same object: sectors of the old one not kept
	SDCard sd3;
	static uint8_t mem3[512];

	Fresh(T_HC);
	Setup(Bus);
	assert(sd3.Init(Intrf(Bus), mem3, sizeof(mem3)));
	assert(sd3.Read((uint64_t)0, c, 16) == 16);
	memset(s_Mem, 0x5A, 512);
	Fresh(T_HC);
	Setup(Bus);
	assert(sd3.Init(Intrf(Bus), mem3, sizeof(mem3)));
	assert(sd3.Read((uint64_t)0, c, 16) == 16 && c[0] == 0x5A && c[15] == 0x5A);
	// Again, the cache kept
	memset(s_Mem, 0xA5, 512);
	Fresh(T_HC);
	Setup(Bus);
	assert(sd3.Init(Intrf(Bus), (DiskIOCache_t *)nullptr, 0));
	assert(sd3.Read((uint64_t)0, c, 16) == 16 && c[0] == 0xA5 && c[15] == 0xA5);

	printf("PASS : %s, sector cache\r\n", Bus ? "SD bus" : "SPI mode");
}

static void TestNotReady()
{
	SDCard sd;
	uint8_t b[512];

	assert(sd.Type() == SDCARD_TYPE_NONE && sd.GetNbSect() == 0 && sd.GetSize() == 0);
	assert(sd.ReadSect(0, b, 1) == 0 && sd.WriteSect(0, b, 1) == 0);
	assert(sd.Status() == 0xFFFFFFFFU);
	assert(sd.SectRead(0, b) == false && sd.SectWrite(0, b) == false);

	printf("PASS : not set up\r\n");
}

int main()
{
	setvbuf(stdout, nullptr, _IONBF, 0);
	TestNotReady();
	TestInitSpi();
	TestInitBus();
	TestInitFail();
	for (int bus = 0; bus < 2; bus++)
	{
		TestTransfer(bus, T_HC);
		TestTransfer(bus, T_V2);
		TestTransfer(bus, T_V1);
		TestFault(bus);
		TestCache(bus);
	}
	TestTransfer(false, T_MMC);
	TestSpiFault();

	printf("ALL PASS\r\n");

	return 0;
}
