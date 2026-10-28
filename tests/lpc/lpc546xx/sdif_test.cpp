// Host regression of the LPC546xx SDIF SD/MMC host interface
// (ARM/NXP/LPC546xx/src/sdmmc_lpc546xx.cpp) with the generic SD card driver
// (src/storage/sdcard_impl.cpp) on a model of the SDIF host and of an SD
// memory card.
//
// Host model. The registers need the SDIO bus clock (AHBCLKCTRL2) and the
// SDIF out of reset (PRESETCTRL2); the reset clears PWREN. CTRL reset bits
// clear by themselves, no other register is written before. The card clock
// is the main clock / (SDIOCLKDIV + 1), the SDIF clock, 52 MHz or less, / (2
// x CLKDIV, or 1 when 0). CLKDIV and CLKENA reach the card interface through an update clock command, a new
// divider only with the clock off in that update, SDIOCLKDIV changed only
// with the clock off at the card interface. A command (START) is taken after
// two reads of CMD, no register is written while START is set. A card
// command needs the hold register, waits the data in progress unless it is a
// stop command, the card supply (PWREN), the main clock (SDIOCLKSEL) and the
// clock taken by the card interface. Its response (RESP, CDONE) comes after
// the command and response bits at the card clock, a response timeout (RTO)
// after 64 clocks without response. R2 needs the long response bit, a CRC
// check on R3 (no CRC) gives a response CRC error. Data commands: BLKSIZ
// 512, BYTCNT the sectors, the direction and the auto stop of a multiple
// block command, CTYPE the bus width of the card. A 64 word FIFO filled and
// emptied by the card at the card clock; the card clock stops while the FIFO
// is full on a read, a read of an empty FIFO or a write to a full one fails
// the test. DTO at the end of the data, then the auto stop (response in
// RESP1, ACD). No command but a stop before ACD. A stop command ends the
// data path when it completes, and it needs the card clock. A data command
// the card refuses leaves the host waiting for data until the data timeout.
// STATUS DATA_BUSY while the card programs.
//
// Card model. SDSC version 1 or 2, or SDHC. CMD0 with the 80 initialization
// clocks after the supply, CMD8 (version 2), CMD55 and ACMD41 (busy a number
// of times and a time, never ready for an SDHC card without HCS), CMD2,
// CMD3, CMD9, CMD7, ACMD6, CMD16, CMD13, CMD17, CMD18, CMD24, CMD25, CMD12,
// in their states, CMD6 without CMD55 is the switch function command. No
// response to a command out of its state. Identification at 400 kHz or
// less, the transfers at the TRAN_SPEED of the card or less. Byte addresses
// on SDSC, sector numbers on SDHC, OUT_OF_RANGE past the end without data.
// The card resets when its supply is off 1 ms or more. Only CMD12 and CMD13
// while it is busy. Faults: data CRC errors, missing responses, R1 errors,
// response CRC errors, a card that stays busy, a card that stops its data,
// CMD55 not taken, a wrong CMD8 pattern, CSD fields.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "LPC546xx.h"

uint32_t SystemCoreClock = 220000000;
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }

static double s_Ns = 0;
static double s_AccessNs = 10;		// Time of a register access

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];

static uint32_t s_Ahb[3], s_Rst[3];
static uint32_t s_SdioClkSel = 7, s_SdioClkDiv = 0, s_SdioClkCtrl = 0;

static void Process();

void ModelDelayUs(uint32_t Us)
{
	s_Ns += Us * 1000.0;
	Process();
}

// ---- Card ----

enum { CARD_V1, CARD_V2, CARD_HC };
enum { ST_IDLE, ST_READY, ST_IDENT, ST_STBY, ST_TRAN, ST_DATA, ST_RCV };

#define CARD_SIZE		(8U * 1024U * 1024U)
#define FIFO_WORDS		64U
#define CARD_RCA		0x0002U		// R6 without R1 error bits

static uint8_t s_Mem[CARD_SIZE];

struct CardM {
	int Type = CARD_HC;
	int State = ST_IDLE;
	bool Powered = false, InitClocks = false, Acmd = false;
	double OffAt = -1e18;		// Supply off since
	int OpBusy = 3;				// ACMD41 answers busy this many times
	double OpMs = 0;			// and this time from the first ACMD41
	int OpCount = 0;
	double OpStart = 0;
	int Width = 1;
	uint32_t BlockLen = 512;
	uint32_t MaxRate = 25000000;
	uint8_t TranSpeed = 0x32;
	uint32_t Status = 0;		// Error bits of the next R1
	uint32_t Addr = 0;			// Byte address of the data
	double BusyEnd = 0;
	bool BusyForever = false;
	// Faults
	int CrcBlock = -1;			// Data CRC error on this block of the next transfer
	int NoResp = -1;			// No response to this command once, 100 + n for ACMDn
	int ErrCmd = -1;			// Error bits ErrBits in the R1 of this command once
	uint32_t ErrBits = 0;
	uint32_t WriteErr = 0;		// Error bits after the next block written
	bool StuckAfterWrite = false;	// Busy forever after the next block written
	bool StuckAfterSelect = false;
	int StallAt = -1;			// The card stops its data at this word
	bool Cmd8Bad = false;		// CMD8 echoes a wrong check pattern
	int NoApp = -1;				// This CMD55 not taken (0 the first)
	int CsdStruct = -1;			// CSD_STRUCTURE other than the card type
	int BlLen = -1;				// READ_BL_LEN other than 10 of SDSC
	int CSize = -1;				// C_SIZE of SDHC other than 8 MB
	// Counts
	int Reads = 0, Writes = 0, Stops = 0, Selects = 0, Cmd16 = 0, Acmd6 = 0, Aborts = 0;
	std::vector<int> Log;
};
static CardM s_C;

static bool CardBusy() { return s_C.BusyForever || s_Ns < s_C.BusyEnd; }

static void CardSupply(bool On)
{
	if (On == s_C.Powered)
	{
		return;
	}
	if (On)
	{
		if (s_Ns - s_C.OffAt >= 1e6)
		{
			// Power on reset
			s_C.InitClocks = false;
			s_C.State = ST_IDLE;
			s_C.Width = 1;
			s_C.BlockLen = 512;
			s_C.BusyForever = false;
			s_C.BusyEnd = 0;
			s_C.Acmd = false;
			s_C.OpCount = 0;
		}
	}
	else
	{
		s_C.OffAt = s_Ns;
	}
	s_C.Powered = On;
}

static uint32_t R1()
{
	uint32_t st = s_C.Status | ((uint32_t)s_C.State << 9) | (1U << 8) | (s_C.Acmd ? (1U << 5) : 0U);
	s_C.Status = 0;
	return st;
}

static void Csd(uint32_t *pResp)
{
	// Bits 127:0 into RESP[3]..RESP[0]
	auto put = [&](uint32_t Msb, uint32_t Lsb, uint32_t V) {
		for (uint32_t b = Lsb; b <= Msb; b++)
		{
			if ((V >> (b - Lsb)) & 1U)
			{
				pResp[b / 32] |= 1U << (b % 32);
			}
		}
	};
	memset(pResp, 0, 16);
	put(103, 96, s_C.TranSpeed);
	if (s_C.Type == CARD_HC)
	{
		put(127, 126, 1);
		put(83, 80, 9);
		put(69, 48, s_C.CSize >= 0 ? (uint32_t)s_C.CSize : CARD_SIZE / (512U * 1024U) - 1U);
	}
	else
	{
		// 8 MB: (C_SIZE + 1) x 2^(MULT + 2) x 2^READ_BL_LEN, READ_BL_LEN 10 so
		// the size is not taken as 512 byte blocks
		put(83, 80, s_C.BlLen >= 0 ? s_C.BlLen : 10);
		put(73, 62, CARD_SIZE / (1024U * 512U) - 1U);
		put(49, 47, 7);
	}
	if (s_C.CsdStruct >= 0)
	{
		pResp[3] &= 0x3FFFFFFFU;
		put(127, 126, s_C.CsdStruct);
	}
	put(0, 0, 1);
}

// Its bits 127:126 read as CSD version 2
static const uint32_t s_CidWords[4] = { 0x89ABCDEFU, 0x01234567U, 0x544F5348U, 0x41534D49U };

static bool CardIdent() { return s_C.State == ST_IDLE || s_C.State == ST_READY || s_C.State == ST_IDENT; }

enum { R_NONE, R_R1, R_R1B, R_R2, R_R3, R_R6, R_R7 };

static bool RcaArg(uint32_t Arg)
{
	// Stuff bits 0
	assert((Arg & 0xFFFFU) == 0);
	return (Arg >> 16) == CARD_RCA;
}

// A command to the card: the response kind and value, R_NONE without response
static int CardCmd(int Idx, uint32_t Arg, uint32_t *pResp, bool bInit, bool bPresent)
{
	if (!bPresent)
	{
		return R_NONE;
	}

	bool acmd = s_C.Acmd;
	int code = acmd ? 100 + Idx : Idx;

	s_C.Log.push_back(code);
	s_C.Acmd = false;
	assert(s_C.Powered);
	if (CardBusy() && Idx != 13 && Idx != 12)
	{
		fprintf(stderr, "CMD%d while busy\n", Idx);
		assert(0);
	}
	if (s_C.NoResp == code)
	{
		s_C.NoResp = -1;
		return R_NONE;
	}
	if (s_C.ErrCmd == code)
	{
		s_C.ErrCmd = -1;
		s_C.Status |= s_C.ErrBits;
	}
	if (Idx == 0)
	{
		// The first CMD0 after the supply comes with the initialization clocks
		assert(bInit || s_C.InitClocks);
		s_C.InitClocks = true;
		s_C.State = ST_IDLE;
		s_C.Width = 1;
		s_C.OpCount = 0;
		return R_NONE;
	}
	assert(s_C.InitClocks && !bInit);
	if (acmd)
	{
		switch (Idx)
		{
			case 41:
				if (s_C.State != ST_IDLE)
				{
					return R_NONE;
				}
				// HCS 0 to a card without CMD8
				assert(s_C.Type != CARD_V1 || (Arg & 0x40000000U) == 0);
				if (s_C.OpCount++ == 0)
				{
					s_C.OpStart = s_Ns;
				}
				pResp[0] = 0x00FF8000U;
				if (s_C.OpCount > s_C.OpBusy && s_Ns - s_C.OpStart >= s_C.OpMs * 1e6 &&
					(s_C.Type != CARD_HC || (Arg & 0x40000000U)))
				{
					pResp[0] |= 0x80000000U | (s_C.Type == CARD_HC ? 0x40000000U : 0U);
					s_C.State = ST_READY;
				}
				return R_R3;
			case 6:
				if (s_C.State != ST_TRAN || (Arg != 0 && Arg != 2))
				{
					return R_NONE;
				}
				pResp[0] = R1();
				if ((pResp[0] & 0xFDF98008U) == 0)
				{
					s_C.Width = Arg == 2 ? 4 : 1;
					s_C.Acmd6++;
				}
				return R_R1;
			default:
				return R_NONE;
		}
	}
	switch (Idx)
	{
		case 8:
			if (s_C.Type == CARD_V1 || s_C.State != ST_IDLE)
			{
				return R_NONE;
			}
			pResp[0] = (Arg & 0xFFFU) ^ (s_C.Cmd8Bad ? 1U : 0U);
			return R_R7;
		case 55:
			if (s_C.State != ST_IDLE && !RcaArg(Arg))
			{
				return R_NONE;
			}
			s_C.Acmd = s_C.NoApp != 0;
			if (s_C.NoApp >= 0)
			{
				s_C.NoApp--;
			}
			pResp[0] = R1();
			return R_R1;
		case 2:
			if (s_C.State != ST_READY)
			{
				return R_NONE;
			}
			memcpy(pResp, s_CidWords, 16);
			s_C.State = ST_IDENT;
			return R_R2;
		case 3:
			if (s_C.State != ST_IDENT && s_C.State != ST_STBY)
			{
				return R_NONE;
			}
			s_C.State = ST_STBY;
			pResp[0] = (CARD_RCA << 16) | 0x0500U;
			return R_R6;
		case 9:
			if (s_C.State != ST_STBY || !RcaArg(Arg))
			{
				return R_NONE;
			}
			Csd(pResp);
			return R_R2;
		case 7:
			if (!RcaArg(Arg) || s_C.State != ST_STBY)
			{
				return R_NONE;
			}
			pResp[0] = R1();
			if ((pResp[0] & 0xFDF98008U) == 0)
			{
				s_C.State = ST_TRAN;
				s_C.Selects++;
			}
			return R_R1B;
		case 6:
			// Switch function: the card sends its 64 status bytes, not seen
			// by the host
			if (s_C.State != ST_TRAN)
			{
				return R_NONE;
			}
			pResp[0] = R1();
			return R_R1;
		case 16:
			if (s_C.State != ST_TRAN)
			{
				return R_NONE;
			}
			pResp[0] = R1();
			if ((pResp[0] & 0xFDF98008U) == 0)
			{
				s_C.BlockLen = Arg;
				s_C.Cmd16++;
			}
			return R_R1;
		case 13:
			if (!RcaArg(Arg))
			{
				return R_NONE;
			}
			pResp[0] = R1();
			return R_R1;
		case 17: case 18: case 24: case 25:
		{
			if (s_C.State != ST_TRAN)
			{
				return R_NONE;
			}
			uint64_t addr = s_C.Type == CARD_HC ? (uint64_t)Arg * 512U : Arg;
			if (s_C.Type != CARD_HC)
			{
				assert(s_C.BlockLen == 512 && Arg % 512 == 0);
			}
			if (addr >= CARD_SIZE)
			{
				s_C.Status |= 0x80000000U;	// OUT_OF_RANGE, no data
				pResp[0] = R1();
				return R_R1;
			}
			pResp[0] = R1();
			s_C.Addr = (uint32_t)addr;
			s_C.State = Idx < 20 ? ST_DATA : ST_RCV;
			return R_R1;
		}
		case 12:
			if (s_C.State != ST_DATA && s_C.State != ST_RCV)
			{
				return R_NONE;
			}
			s_C.Stops++;
			pResp[0] = R1();
			s_C.State = ST_TRAN;
			return R_R1B;
		default:
			return R_NONE;
	}
}

// ---- Host ----

enum { H_IDLE, H_RX, H_TX, H_WAIT };

struct HostM {
	uint32_t Ctrl = 0, Pwren = 0, Clkdiv = 0, Clkena = 0, Tmout = 0, Ctype = 0, Blksiz = 0, Bytcnt = 0;
	uint32_t Intmask = 0, Cmdarg = 0, Cmd = 0, Resp[4] = {}, Rintsts = 0, Fifoth = 0, Debnce = 0;
	bool Present = true;
	int ResetReads = 0, StartReads = 0;
	// Taken by the card interface
	uint32_t EffDiv = 0, EffEna = 0;
	// Command in progress
	bool Pend = false;
	double PendAt = 0;
	uint32_t PendResp[4] = {}, PendInt = 0;
	bool PendLong = false, PendStop = false;
	int PendData = H_IDLE;
	// Data
	int Mode = H_IDLE;
	std::deque<uint32_t> Fifo;
	uint32_t Words = 0, Moved = 0;
	double Next = 0;			// Time of the next word on the card side
	bool AutoStop = false;
	double AcdAt = -1;
	double WaitEnd = 0;			// Data timeout of H_WAIT
	double Last = 0;
	uint32_t Out[512 / 4];
	// Faults of the host
	int Stuck = 0;				// START of the next commands never clears,
	int StuckSkip = 0;			// after this number of commands
	int CrcCmd = -1;			// Response CRC error on this command once
	int NoDone = -1;			// This command never completes
	bool NoDetPin = false;		// Card detect not wired, CDETECT 1
	int Updates = 0;
	int Full = 0;				// Times the FIFO was full on a read
	int Cmds = 0;				// Card commands sent
};
static HostM s_H;

static double CardClk()
{
	double s = (double)SystemCoreClock / (s_SdioClkDiv + 1U);
	return s_H.EffDiv ? s / (2.0 * s_H.EffDiv) : s;
}

static double ClkNs(double Clocks) { return Clocks * 1e9 / CardClk(); }

static double WordNs()
{
	int lines = (s_H.Ctype & SDIF_CTYPE_CARD_WIDTH0_MASK) ? 4 : 1;
	return ClkNs(32.0 / lines);
}

static void HostReset()
{
	HostM h;

	h.Present = s_H.Present;
	h.Updates = s_H.Updates;
	h.Stuck = s_H.Stuck;
	h.StuckSkip = s_H.StuckSkip;
	h.CrcCmd = s_H.CrcCmd;
	h.NoDone = s_H.NoDone;
	h.NoDetPin = s_H.NoDetPin;
	h.Cmds = s_H.Cmds;
	h.Full = s_H.Full;
	h.Last = s_Ns;
	s_H = h;
	CardSupply(false);
}

static void DataEnd()
{
	s_H.Rintsts |= SDIF_RINTSTS_DTO_MASK;
	s_H.Mode = H_IDLE;
	if (s_H.AutoStop)
	{
		uint32_t r[4];
		int k = CardCmd(12, 0, r, false, true);
		assert(k == R_R1B);
		s_H.Resp[1] = r[0];
		s_H.AcdAt = s_Ns + ClkNs(96);
	}
	else if (s_C.State == ST_DATA || s_C.State == ST_RCV)
	{
		s_C.State = ST_TRAN;
	}
}

static void Process()
{
	double dt = s_Ns - s_H.Last;

	s_H.Last = s_Ns;
	if (s_H.Mode == H_RX && s_H.Fifo.size() >= FIFO_WORDS)
	{
		// Card clock stopped
		if (s_H.Pend)
		{
			s_H.PendAt += dt;
		}
		if (s_H.Next < s_Ns)
		{
			s_H.Next = s_Ns;
		}
	}
	if (s_H.Pend && s_Ns >= s_H.PendAt)
	{
		s_H.Pend = false;
		s_H.Rintsts |= s_H.PendInt;
		if (s_H.PendInt & SDIF_RINTSTS_CDONE_MASK)
		{
			s_H.Resp[0] = s_H.PendResp[0];
			if (s_H.PendLong)
			{
				memcpy(s_H.Resp, s_H.PendResp, sizeof(s_H.Resp));
			}
		}
		if (s_H.PendStop)
		{
			// The stop command ends the data path
			s_H.Mode = H_IDLE;
			s_H.AcdAt = -1;
		}
		if (s_H.PendData != H_IDLE)
		{
			s_H.Mode = s_H.PendData;
			s_H.Next = s_Ns + ClkNs(2);
			s_H.WaitEnd = s_Ns + ClkNs(s_H.Tmout >> 8);
		}
	}
	if (s_H.Mode == H_RX)
	{
		while (s_H.Moved < s_H.Words && s_H.Fifo.size() < FIFO_WORDS && s_Ns >= s_H.Next && (int)s_H.Moved != s_C.StallAt)
		{
			uint32_t w;
			memcpy(&w, s_Mem + s_C.Addr + s_H.Moved * 4, 4);
			s_H.Fifo.push_back(w);
			s_H.Moved++;
			s_H.Next += WordNs();
			s_H.Full += s_H.Fifo.size() == FIFO_WORDS;
			if (s_H.Moved % 128 == 0)
			{
				if ((int)(s_H.Moved / 128 - 1) == s_C.CrcBlock)
				{
					s_C.CrcBlock = -1;
					s_H.Rintsts |= SDIF_RINTSTS_DCRC_MASK;
				}
				s_C.Reads++;
			}
		}
		if ((int)s_H.Moved == s_C.StallAt && s_H.Next < s_Ns)
		{
			s_H.Next = s_Ns;
		}
		if (s_H.Moved == s_H.Words)
		{
			DataEnd();
		}
	}
	else if (s_H.Mode == H_TX)
	{
		while (s_H.Moved < s_H.Words && !s_H.Fifo.empty() && s_Ns >= s_H.Next && !CardBusy() &&
			   (int)s_H.Moved != s_C.StallAt)
		{
			uint32_t w = s_H.Fifo.front();
			s_H.Fifo.pop_front();
			s_H.Out[s_H.Moved % 128] = w;
			s_H.Moved++;
			s_H.Next += WordNs();
			if (s_H.Moved % 128 == 0)
			{
				uint32_t base = s_C.Addr + (s_H.Moved / 128 - 1) * 512;
				if ((int)(s_H.Moved / 128 - 1) == s_C.CrcBlock)
				{
					s_C.CrcBlock = -1;
					s_H.Rintsts |= SDIF_RINTSTS_DCRC_MASK;
				}
				else
				{
					memcpy(s_Mem + base, s_H.Out, 512);
					s_C.Writes++;
				}
				s_C.BusyEnd = s_Ns + 50000;
				s_C.Status |= s_C.WriteErr;
				s_C.WriteErr = 0;
				s_C.BusyForever |= s_C.StuckAfterWrite;
				s_C.StuckAfterWrite = false;
			}
		}
		if (s_H.Next < s_Ns)
		{
			s_H.Next = s_Ns;
		}
		if (s_H.Moved == s_H.Words)
		{
			DataEnd();
		}
	}
	else if (s_H.Mode == H_WAIT && s_Ns >= s_H.WaitEnd)
	{
		s_H.Rintsts |= SDIF_RINTSTS_DRTO_BDS_MASK | SDIF_RINTSTS_DTO_MASK;
		s_H.Mode = H_IDLE;
	}
	if (s_H.AcdAt >= 0 && s_Ns >= s_H.AcdAt)
	{
		s_H.Rintsts |= SDIF_RINTSTS_ACD_MASK;
		s_H.AcdAt = -1;
	}
	if (s_H.Mode == H_RX && s_H.Fifo.size() > 15)
	{
		s_H.Rintsts |= SDIF_RINTSTS_RXDR_MASK;
	}
	if (s_H.Mode == H_TX && s_H.Fifo.size() <= 16)
	{
		s_H.Rintsts |= SDIF_RINTSTS_TXDR_MASK;
	}
}

static void Access()
{
	assert(s_Ahb[2] & SYSCON_AHBCLKCTRL_SDIO_MASK);
	assert((s_Rst[2] & SYSCON_PRESETCTRL_SDIO_RST_MASK) == 0);
	s_Ns += s_AccessNs;
	Process();
}

// A register write: no reset in progress, no command waiting to be taken
static void AccessW()
{
	Access();
	assert(s_H.ResetReads == 0);
	assert((s_H.Cmd & SDIF_CMD_START_CMD_MASK) == 0);
}

static void CmdWrite(uint32_t v)
{
	AccessW();
	s_H.Cmd = v;
	if ((v & SDIF_CMD_START_CMD_MASK) == 0)
	{
		return;
	}
	s_H.StartReads = 2;
	if (s_H.Stuck > 0 && s_H.StuckSkip-- <= 0)
	{
		s_H.Stuck--;
		s_H.StartReads = -1;
		return;
	}
	if (v & SDIF_CMD_UPDATE_CLOCK_REGISTERS_ONLY_MASK)
	{
		// A new divider with the clock off
		assert(s_H.Clkdiv == s_H.EffDiv || (s_H.Clkena & SDIF_CLKENA_CCLK_ENABLE_MASK) == 0);
		assert(!s_H.Pend);
		s_H.EffDiv = s_H.Clkdiv;
		s_H.EffEna = s_H.Clkena;
		s_H.Updates++;
		return;
	}

	bool stop = (v & SDIF_CMD_STOP_ABORT_CMD_MASK) != 0;

	// The driver waits the end of the data, of the auto stop and of the
	// command before
	assert(!s_H.Pend);
	assert((s_H.Mode == H_IDLE && s_H.AcdAt < 0) || stop);
	assert(v & SDIF_CMD_USE_HOLD_REG_MASK);
	assert(((v & SDIF_CMD_WAIT_PRVDATA_COMPLETE_MASK) != 0) == !stop);
	assert(s_H.Pwren & SDIF_PWREN_POWER_ENABLE_MASK);
	assert(s_SdioClkSel == 0);
	// SDIF clock 52 MHz or less
	assert(SystemCoreClock / (s_SdioClkDiv + 1U) <= 52000000U);
	assert(s_H.EffDiv == s_H.Clkdiv && s_H.EffEna == s_H.Clkena && (s_H.EffEna & SDIF_CLKENA_CCLK_ENABLE_MASK));

	int idx = v & SDIF_CMD_CMD_INDEX_MASK;

	if (CardIdent())
	{
		assert(CardClk() <= 400000.0 + 1e-6);
	}
	else
	{
		assert(CardClk() <= s_C.MaxRate + 1e-6);
	}
	if (stop)
	{
		// The command needs the card clock, stopped while the FIFO is full
		assert(idx == 12 && !(s_H.Mode == H_RX && s_H.Fifo.size() >= FIFO_WORDS));
		s_C.Aborts++;
	}

	bool init = (v & SDIF_CMD_SEND_INITIALIZATION_MASK) != 0;
	uint32_t r[4] = {};
	int kind = CardCmd(idx, s_H.Cmdarg, r, init, s_H.Present);
	double bits = 48 + (init ? 80 : 0);

	s_H.Cmds++;
	if (s_H.NoDone == idx)
	{
		// The host hangs
		s_H.NoDone = -1;
		s_H.Pend = true;
		s_H.PendAt = 1e300;
		s_H.PendInt = 0;
		s_H.PendStop = false;
		s_H.PendData = H_IDLE;
		return;
	}

	s_H.Pend = true;
	s_H.PendStop = stop;
	s_H.PendData = H_IDLE;
	s_H.PendLong = false;
	if (kind == R_NONE)
	{
		s_H.PendInt = SDIF_RINTSTS_CDONE_MASK;
		if (v & SDIF_CMD_RESPONSE_EXPECT_MASK)
		{
			s_H.PendInt |= SDIF_RINTSTS_RTO_BAR_MASK;
			bits += 64;
		}
		s_H.PendAt = s_Ns + ClkNs(bits);
		return;
	}
	assert(v & SDIF_CMD_RESPONSE_EXPECT_MASK);

	bool lng = (v & SDIF_CMD_RESPONSE_LENGTH_MASK) != 0, crc = (v & SDIF_CMD_CHECK_RESPONSE_CRC_MASK) != 0;

	assert(lng == (kind == R_R2));
	assert(crc || kind == R_R3);
	memcpy(s_H.PendResp, r, sizeof(r));
	s_H.PendLong = lng;
	s_H.PendInt = SDIF_RINTSTS_CDONE_MASK;
	if ((crc && kind == R_R3) || s_H.CrcCmd == idx)
	{
		s_H.PendInt |= SDIF_RINTSTS_RCRC_MASK;
		s_H.CrcCmd = -1;
	}
	bits += lng ? 136 : 48;
	s_H.PendAt = s_Ns + ClkNs(bits);
	if (kind == R_R1B)
	{
		s_C.BusyEnd = s_H.PendAt + 1e6;
		s_C.BusyForever |= idx == 7 && s_C.StuckAfterSelect;
	}

	if (v & SDIF_CMD_DATA_EXPECTED_MASK)
	{
		bool wr = (v & SDIF_CMD_READ_WRITE_MASK) != 0;
		assert(s_H.Blksiz == 512 && s_H.Bytcnt % 512 == 0 && s_H.Bytcnt > 0);
		assert(wr == (idx == 24 || idx == 25));
		assert(((v & SDIF_CMD_SEND_AUTO_STOP_MASK) != 0) == (idx == 18 || idx == 25));
		assert(((s_H.Ctype & SDIF_CTYPE_CARD_WIDTH0_MASK) ? 4 : 1) == s_C.Width &&
			   (s_H.Ctype & SDIF_CTYPE_CARD_WIDTH1_MASK) == 0);
		assert(s_H.Fifo.empty());
		if (idx == 17 || idx == 24)
		{
			assert(s_H.Bytcnt == 512);
		}
		s_H.Words = s_H.Bytcnt / 4;
		s_H.Moved = 0;
		s_H.AutoStop = (v & SDIF_CMD_SEND_AUTO_STOP_MASK) != 0;
		if (s_C.State != ST_DATA && s_C.State != ST_RCV)
		{
			// The card refused it: the host waits for data until the data
			// timeout
			s_H.PendData = H_WAIT;
			return;
		}
		s_H.PendData = wr ? H_TX : H_RX;
		assert(s_C.Addr + s_H.Bytcnt <= CARD_SIZE);
	}
}

struct CtrlR {
	operator uint32_t() const
	{
		Access();
		if (s_H.ResetReads > 0 && --s_H.ResetReads == 0)
		{
			s_H.Ctrl &= ~(SDIF_CTRL_CONTROLLER_RESET_MASK | SDIF_CTRL_FIFO_RESET_MASK | SDIF_CTRL_DMA_RESET_MASK);
		}
		return s_H.Ctrl;
	}
	CtrlR &operator=(uint32_t v)
	{
		Access();
		assert((v & (SDIF_CTRL_INT_ENABLE_MASK | SDIF_CTRL_USE_INTERNAL_DMAC_MASK)) == 0);
		s_H.Ctrl = v;
		if (v & SDIF_CTRL_FIFO_RESET_MASK)
		{
			s_H.Fifo.clear();
		}
		if (v & SDIF_CTRL_CONTROLLER_RESET_MASK)
		{
			s_H.Mode = H_IDLE;
			s_H.AcdAt = -1;
			s_H.Pend = false;
			s_H.Cmd &= ~SDIF_CMD_START_CMD_MASK;
			s_H.StartReads = 0;
		}
		if (v & (SDIF_CTRL_CONTROLLER_RESET_MASK | SDIF_CTRL_FIFO_RESET_MASK | SDIF_CTRL_DMA_RESET_MASK))
		{
			s_H.ResetReads = 2;
		}
		return *this;
	}
	CtrlR &operator|=(uint32_t v) { return *this = (uint32_t)*this | v; }
};
struct CmdR {
	operator uint32_t() const
	{
		Access();
		if (s_H.StartReads > 0 && --s_H.StartReads == 0)
		{
			s_H.Cmd &= ~SDIF_CMD_START_CMD_MASK;
		}
		return s_H.Cmd;
	}
	CmdR &operator=(uint32_t v) { CmdWrite(v); return *this; }
};
struct RintR {
	operator uint32_t() const { Access(); return s_H.Rintsts; }
	RintR &operator=(uint32_t v) { AccessW(); s_H.Rintsts &= ~v; Process(); return *this; }
};
struct StatusR {
	operator uint32_t() const
	{
		Access();
		return ((uint32_t)s_H.Fifo.size() << SDIF_STATUS_FIFO_COUNT_SHIFT) |
			   (CardBusy() ? SDIF_STATUS_DATA_BUSY_MASK : 0U);
	}
};
struct FifoR {
	operator uint32_t() const
	{
		Access();
		assert(!s_H.Fifo.empty());
		uint32_t w = s_H.Fifo.front();
		s_H.Fifo.pop_front();
		Process();
		return w;
	}
	FifoR &operator=(uint32_t v)
	{
		AccessW();
		assert(s_H.Mode == H_TX && s_H.Fifo.size() < FIFO_WORDS);
		s_H.Fifo.push_back(v);
		Process();
		return *this;
	}
};
struct PlainR {
	uint32_t *p;
	operator uint32_t() const { Access(); return *p; }
	PlainR &operator=(uint32_t v) { AccessW(); *p = v; return *this; }
};
struct PwrenR {
	operator uint32_t() const { Access(); return s_H.Pwren; }
	PwrenR &operator=(uint32_t v)
	{
		AccessW();
		s_H.Pwren = v;
		CardSupply((v & SDIF_PWREN_POWER_ENABLE_MASK) != 0);
		return *this;
	}
};
struct CdetR {
	operator uint32_t() const { Access(); return s_H.Present && !s_H.NoDetPin ? 0U : 1U; }
};
struct RespR {
	int Idx;
	operator uint32_t() const { Access(); return s_H.Resp[Idx]; }
};
struct SdifRegs {
	CtrlR CTRL;
	PwrenR PWREN;
	PlainR CLKDIV = { &s_H.Clkdiv };
	PlainR CLKENA = { &s_H.Clkena };
	PlainR TMOUT = { &s_H.Tmout };
	PlainR CTYPE = { &s_H.Ctype };
	PlainR BLKSIZ = { &s_H.Blksiz };
	PlainR BYTCNT = { &s_H.Bytcnt };
	PlainR INTMASK = { &s_H.Intmask };
	PlainR CMDARG = { &s_H.Cmdarg };
	CmdR CMD;
	RespR RESP[4] = { { 0 }, { 1 }, { 2 }, { 3 } };
	RintR RINTSTS;
	StatusR STATUS;
	PlainR FIFOTH = { &s_H.Fifoth };
	CdetR CDETECT;
	PlainR DEBNCE = { &s_H.Debnce };
	FifoR FIFO[1];
};
static SdifRegs s_SdifRegs;

enum { AHB_SET, AHB_CLR, RST_SET, RST_CLR };

static void SysWrite(int Kind, int Idx, uint32_t v)
{
	switch (Kind)
	{
		case AHB_SET: s_Ahb[Idx] |= v; break;
		case AHB_CLR: s_Ahb[Idx] &= ~v; break;
		case RST_SET:
			if (Idx == 2 && (v & SYSCON_PRESETCTRL_SDIO_RST_MASK))
			{
				HostReset();
			}
			s_Rst[Idx] |= v;
			break;
		case RST_CLR: s_Rst[Idx] &= ~v; break;
	}
}

struct WReg {
	int Kind, Idx;
	WReg &operator=(uint32_t v) { SysWrite(Kind, Idx, v); return *this; }
};
struct AhbR {
	int Idx;
	operator uint32_t() const { return s_Ahb[Idx]; }
};
struct ClkSelR {
	ClkSelR &operator=(uint32_t v) { s_SdioClkSel = v; return *this; }
};
struct ClkDivR {
	ClkDivR &operator=(uint32_t v)
	{
		assert((v & ~SYSCON_SDIOCLKDIV_DIV_MASK) == 0);
		// The card clock is off at the card interface
		assert((s_H.EffEna & SDIF_CLKENA_CCLK_ENABLE_MASK) == 0);
		s_SdioClkDiv = v;
		return *this;
	}
};
struct ClkCtrlR {
	ClkCtrlR &operator=(uint32_t v) { s_SdioClkCtrl = v; return *this; }
};
struct AhbDivR {
	operator uint32_t() const { return 0; }
};
struct SysRegs {
	AhbR AHBCLKCTRL[3] = { { 0 }, { 1 }, { 2 } };
	WReg AHBCLKCTRLSET[3] = { { AHB_SET, 0 }, { AHB_SET, 1 }, { AHB_SET, 2 } };
	WReg AHBCLKCTRLCLR[3] = { { AHB_CLR, 0 }, { AHB_CLR, 1 }, { AHB_CLR, 2 } };
	WReg PRESETCTRLSET[3] = { { RST_SET, 0 }, { RST_SET, 1 }, { RST_SET, 2 } };
	WReg PRESETCTRLCLR[3] = { { RST_CLR, 0 }, { RST_CLR, 1 }, { RST_CLR, 2 } };
	ClkSelR SDIOCLKSEL;
	ClkDivR SDIOCLKDIV;
	ClkCtrlR SDIOCLKCTRL;
	AhbDivR AHBCLKDIV;
};
static SysRegs s_SysRegs;

#undef IOCON
#undef GPIO
#undef SYSCON
#undef PINT
#undef INPUTMUX
#undef SDIF
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define SYSCON (&s_SysRegs)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)
#define SDIF (&s_SdifRegs)
#define LPC546XX_SDIF_WAIT_MAX 200000U
#define LPC546XX_SDIF_BUSY_MAX 2000U

#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/sdmmc_lpc546xx.cpp"
#include "storage/sdcard.h"

// ---- Tests ----

static Sdmmc s_Host;

static const IOPinCfg_t s_Pins[] = {
	{2, 3, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CLK
	{2, 4, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// CMD
	{2, 6, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// D0
	{2, 7, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// D1
	{2, 8, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// D2
	{2, 9, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// D3
	{2, 10, IOPINOP_FUNC2, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CARD_DET
	{2, 5, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// POW_EN
};

static SdmmcCfg_t Cfg(int Width = 4, uint32_t Rate = 25000000)
{
	SdmmcCfg_t c;

	memset(&c, 0, sizeof(c));
	c.DevNo = 0;
	c.pIOPinMap = s_Pins;
	c.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]);
	c.Rate = Rate;
	c.BusWidth = Width;
	c.bCardDetect = true;
	c.bPowerCtrl = true;
	return c;
}

// The host, then the card on it
static bool Init(SDCard &Sd, const SdmmcCfg_t &Cfg, DiskIOCache_t *pCache = nullptr, int NbCache = 0)
{
	return s_Host.Init(Cfg) && Sd.Init(&s_Host, pCache, NbCache);
}

static uint32_t CardStatus(SDCard &Sd)
{
	return ((const SDCardDev_t *)Sd)->Status;
}

// A new card in the slot
static void Fresh(int Type)
{
	s_C = CardM();
	s_C.Type = Type;
	s_H.Present = true;
	s_H.CrcCmd = -1;
	s_H.Stuck = 0;
	s_H.StuckSkip = 0;
	s_H.NoDone = -1;
	s_H.NoDetPin = false;
	s_AccessNs = 10;
	s_SdioClkCtrl = 0xFFFFFFFFU;
	for (uint32_t i = 0; i < CARD_SIZE; i += 4)
	{
		uint32_t v = i * 2654435761U ^ 0x5A5A1234U;
		memcpy(s_Mem + i, &v, 4);
	}
	s_Ns += 1e7;
}

static void Fill(uint8_t *p, uint32_t Len, uint32_t Seed)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		Seed = Seed * 1103515245U + 12345U;
		p[i] = (uint8_t)(Seed >> 16);
	}
}

static bool Same(const std::vector<int> &Log, const int *pExp, size_t n)
{
	if (Log.size() != n)
	{
		return false;
	}
	for (size_t i = 0; i < n; i++)
	{
		if (Log[i] != pExp[i])
		{
			return false;
		}
	}
	return true;
}

static void TestInit()
{
	static const struct { int Type; bool Hc; } t[] = { { CARD_HC, true }, { CARD_V2, false }, { CARD_V1, false } };

	for (auto &e : t)
	{
		SDCard sd;

		Fresh(e.Type);
		assert(Init(sd, Cfg()));
		assert((sd.Type() == SDCARD_TYPE_SDHC) == e.Hc && sd.GetNbSect() == CARD_SIZE / 512U && sd.GetSize() == CARD_SIZE / 1024U);
		assert(sd.Type() == (e.Type == CARD_V1 ? SDCARD_TYPE_SDSC_V1 : e.Hc ? SDCARD_TYPE_SDHC : SDCARD_TYPE_SDSC));
		assert(sd.GetSectSize() == 512);
		assert(s_C.State == ST_TRAN && s_C.Width == 4 && s_C.Selects == 1 && s_C.Acmd6 == 1);
		assert(s_C.Cmd16 == (e.Hc ? 0 : 1));
		assert((s_H.Ctype & SDIF_CTYPE_CARD_WIDTH0_MASK) && s_H.Pwren == 1 && s_Host.BusWidth() == 4);
		// The order of the commands, ACMD41 4 times
		static const int hc[] = { 0, 8, 55, 141, 55, 141, 55, 141, 55, 141, 2, 3, 9, 7, 55, 106 };
		static const int sc[] = { 0, 8, 55, 141, 55, 141, 55, 141, 55, 141, 2, 3, 9, 7, 55, 106, 16 };
		if (e.Type == CARD_V1)
		{
			// No answer to CMD8
			static const int v1[] = { 0, 8, 55, 141, 55, 141, 55, 141, 55, 141, 2, 3, 9, 7, 55, 106, 16 };
			assert(Same(s_C.Log, v1, sizeof(v1) / sizeof(int)));
		}
		else
		{
			assert(e.Hc ? Same(s_C.Log, hc, sizeof(hc) / sizeof(int)) : Same(s_C.Log, sc, sizeof(sc) / sizeof(int)));
		}
		// Clock: 24.4 MHz (220 MHz / 9) for 25 MHz
		assert(sd.Rate() == 220000000U / 9U && s_Host.Rate() == sd.Rate() && s_SdioClkDiv == 8 && s_H.Clkdiv == 0);
		const uint8_t *cid = sd.Cid();
		static const uint8_t c[16] = { 0x41, 0x53, 0x4D, 0x49, 0x54, 0x4F, 0x53, 0x48,
									   0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
		assert(memcmp(cid, c, 16) == 0);
		assert((sd.Csd()[0] >> 6) == (e.Hc ? 1 : 0));
		assert(((const SDCardDev_t *)sd)->Rca == CARD_RCA << 16);
		assert(s_SdioClkSel == 0);
		assert(s_SdioClkCtrl == (SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY_ACTIVE_MASK | SYSCON_SDIOCLKCTRL_CCLK_SAMPLE_DELAY(12) |
								 SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY_ACTIVE_MASK | SYSCON_SDIOCLKCTRL_CCLK_DRV_DELAY(31)));
		for (const IOPinCfg_t &p : s_Pins)
		{
			uint32_t v = IOCON->PIO[p.PortNo][p.PinNo];
			assert((v & IOCON_PIO_FUNC_MASK) == IOCON_PIO_FUNC(2) && (v & IOCON_PIO_SLEW_MASK));
		}
		assert(s_H.Tmout == 0xFFFFFF40U && s_H.Debnce == 0xFFFFFFU && s_H.Intmask == 0);
	}
	printf("init: PASS\n");
}

static void TestInitFail()
{
	SDCard sd;
	static uint8_t b[1024];

	// No card: the host set up, nothing sent to the card
	Fresh(CARD_HC);
	s_H.Present = false;
	int cmds = s_H.Cmds;
	assert(s_Host.Init(Cfg()) && !sd.Init(&s_Host));
	assert(!sd.SectRead(0, b) && s_H.Cmds == cmds);
	s_H.Present = true;

	// Configurations refused
	Fresh(CARD_HC);
	SdmmcCfg_t c = Cfg();
	c.BusWidth = 8;
	assert(!s_Host.Init(c));
	c.BusWidth = 2;
	assert(!s_Host.Init(c));
	c = Cfg();
	c.pIOPinMap = NULL;
	assert(!s_Host.Init(c));
	c = Cfg();
	c.NbIOPins = 5;
	assert(!s_Host.Init(c));
	c.BusWidth = 1;
	assert(s_Host.Init(c));
	c = Cfg();
	c.DevNo = 1;
	assert(!s_Host.Init(c));
	c = Cfg();
	c.bDmaEn = true;
	assert(!s_Host.Init(c));
	c = Cfg();
	c.bIntEn = true;
	assert(!s_Host.Init(c));
	assert(SdmmcInit(NULL, &c) == false && SdmmcInit(s_Host, NULL) == false);
	assert(s_C.Log.empty());

	// ACMD41 busy: one second (100 tries 10 ms apart), the tries spaced
	Fresh(CARD_HC);
	s_C.OpBusy = 100;
	assert(!Init(sd, Cfg()));
	Fresh(CARD_HC);
	s_C.OpBusy = 99;
	assert(Init(sd, Cfg()));
	Fresh(CARD_HC);
	s_C.OpBusy = 0;
	s_C.OpMs = 50;
	assert(Init(sd, Cfg()));

	// Card failures: no answer, R1 errors, response CRC errors
	static const struct { int Type, NoResp, ErrCmd, Crc; } f[] = {
		{ CARD_HC, 141, -1, -1 }, { CARD_HC, 2, -1, -1 }, { CARD_HC, 3, -1, -1 }, { CARD_HC, 9, -1, -1 },
		{ CARD_HC, 7, -1, -1 }, { CARD_HC, 106, -1, -1 }, { CARD_V2, 16, -1, -1 }, { CARD_HC, 55, -1, -1 },
		{ CARD_HC, -1, 7, -1 }, { CARD_HC, -1, 106, -1 }, { CARD_V1, -1, 16, -1 }, { CARD_HC, -1, -1, 8 },
		{ CARD_HC, -1, -1, 2 }, { CARD_HC, -1, -1, 3 }, { CARD_HC, -1, -1, 55 }, { CARD_HC, -1, -1, 9 },
	};
	for (auto &e : f)
	{
		Fresh(e.Type);
		s_C.NoResp = e.NoResp;
		s_C.ErrCmd = e.ErrCmd;
		s_C.ErrBits = 0x00080000U;		// ERROR
		s_H.CrcCmd = e.Crc;
		assert(!Init(sd, Cfg()));
		assert(sd.ReadSect(0, b, 1) == 0 && sd.Type() == SDCARD_TYPE_NONE);
	}
	// A response CRC error on CMD8 of an SDSC card: taken as a version 1 card
	Fresh(CARD_V2);
	s_H.CrcCmd = 8;
	assert(Init(sd, Cfg()) && sd.Type() == SDCARD_TYPE_SDSC_V1);
	// No select on a 1 bit bus (no ACMD6 after it)
	for (int n : { 0, 1 })
	{
		Fresh(CARD_HC);
		s_C.NoResp = n == 0 ? 7 : -1;
		s_C.ErrCmd = n == 1 ? 7 : -1;
		s_C.ErrBits = 0x00080000U;
		assert(!Init(sd, Cfg(1)));
	}
	// A wrong CMD8 pattern, CMD55 not taken before ACMD41 or ACMD6 (the
	// card takes CMD6 as the switch function command)
	Fresh(CARD_HC);
	s_C.Cmd8Bad = true;
	assert(!Init(sd, Cfg()));
	for (int n : { 0, 4 })
	{
		Fresh(CARD_HC);
		s_C.NoApp = n;
		assert(!Init(sd, Cfg()));
	}
	// The card stays busy after CMD7
	Fresh(CARD_HC);
	s_C.StuckAfterSelect = true;
	assert(!Init(sd, Cfg()) && s_C.BusyForever);
	// The supply cycle resets it, without it the card stays busy
	s_C.StuckAfterSelect = false;
	assert(Init(sd, Cfg()));

	// CSD
	Fresh(CARD_HC);
	s_C.CsdStruct = 2;
	assert(!Init(sd, Cfg()));
	for (int bl : { 8, 12 })
	{
		Fresh(CARD_V2);
		s_C.BlLen = bl;
		assert(!Init(sd, Cfg()));
	}
	for (int bl : { 9, 11 })
	{
		Fresh(CARD_V2);
		s_C.BlLen = bl;
		assert(Init(sd, Cfg()) && sd.GetNbSect() == ((CARD_SIZE / 512U) << (bl - 9)) / 2U);
	}
	Fresh(CARD_HC);
	s_C.CSize = 0x3FFFFF;
	assert(Init(sd, Cfg()) && sd.GetNbSect() == 0xFFFFFFFFU);
	s_C.CSize = 0x3FFFFE;
	assert(Init(sd, Cfg()) && sd.GetNbSect() == 0xFFFFFC00U);

	// The host does not take a command: the update clock command retried
	// after a reset of the host, a card command given up
	Fresh(CARD_HC);
	s_H.Stuck = 3;
	assert(!s_Host.Init(Cfg()));
	Fresh(CARD_HC);
	s_H.Stuck = 2;
	assert(Init(sd, Cfg()) && s_H.Stuck == 0);
	// The 3 clock updates of the host, the 3 of the identification clock
	Fresh(CARD_HC);
	s_H.Stuck = 1;
	s_H.StuckSkip = 6;
	assert(!Init(sd, Cfg()) && s_C.Log.empty());
	printf("init failures: PASS\n");
}

static void TestInitOther()
{
	SDCard sd;
	static uint8_t b[1024];

	// 1 bit bus, no card detect, no supply switch
	Fresh(CARD_HC);
	s_H.NoDetPin = true;
	SdmmcCfg_t c = Cfg(1);
	c.bCardDetect = false;
	c.bPowerCtrl = false;
	assert(Init(sd, c));
	assert(s_C.Width == 1 && s_C.Acmd6 == 0 && (s_H.Ctype & SDIF_CTYPE_CARD_WIDTH0_MASK) == 0 && s_Host.BusWidth() == 1);
	assert(sd.ReadSect(4, b, 2) == 2 && memcmp(b, s_Mem + 2048, 1024) == 0);
	// Set up again on 4 lines, then on 1 line
	c.BusWidth = 4;
	assert(Init(sd, c) && s_C.Width == 4);
	assert(sd.ReadSect(4, b, 2) == 2 && memcmp(b, s_Mem + 2048, 1024) == 0);
	c.BusWidth = 1;
	c.bPowerCtrl = true;
	assert(Init(sd, c) && s_C.Width == 1 && (s_H.Ctype & SDIF_CTYPE_CARD_WIDTH0_MASK) == 0);
	assert(sd.ReadSect(4, b, 2) == 2 && memcmp(b, s_Mem + 2048, 1024) == 0);
	// The card set up again on the same host: back to 1 line, then 4
	c.BusWidth = 4;
	assert(Init(sd, c) && sd.Init(&s_Host) && s_C.Width == 4 && s_Host.BusWidth() == 4);
	assert(sd.ReadSect(4, b, 2) == 2 && memcmp(b, s_Mem + 2048, 1024) == 0);

	// Transfer clock: the card maximum, then the request
	static const struct { uint8_t Tran; uint32_t Max, Req, Rate, Div, ClkDiv; } r[] = {
		{ 0x32, 25000000, 100000000, 220000000 / 9, 8, 0 },		// default speed
		{ 0x5A, 50000000, 100000000, 44000000, 4, 0 },			// 50 MHz
		{ 0x5A, 50000000, 30000000, 27500000, 7, 0 },			// SDIF clock 52 MHz or less
		{ 0x2A, 20000000, 100000000, 20000000, 10, 0 },			// 20 MHz
		{ 0x34, 25000000, 100000000, 220000000 / 9, 8, 0 },		// unit not known
		{ 0x02, 25000000, 100000000, 220000000 / 9, 8, 0 },		// value not known
		{ 0x32, 25000000, 0, 220000000 / 9, 8, 0 },				// the card maximum
		{ 0x32, 25000000, 1, 220000000 / 256 / 510, 255, 255 },	// the slowest
		{ 0x32, 25000000, 400000, 400000, 4, 55 },				// 220 MHz / 5 / 110
		{ 0x32, 25000000, 50000, 50000, 9, 220 },				// CLKDIV 255 or less
	};
	for (auto &e : r)
	{
		Fresh(CARD_HC);
		s_C.TranSpeed = e.Tran;
		s_C.MaxRate = e.Max;
		// At 1.7 kHz a command takes 56 ms: the waits of the driver are polls
		s_AccessNs = e.Req < 100000 ? 1000 : 10;
		assert(Init(sd, Cfg(4, e.Req)));
		assert(sd.Rate() == e.Rate && s_SdioClkDiv == e.Div && s_H.Clkdiv == e.ClkDiv);
		assert(sd.ReadSect(0, b, 2) == 2 && memcmp(b, s_Mem, 1024) == 0);
	}
	Fresh(CARD_HC);
	assert(Init(sd, Cfg(4, 10000000)));
	assert(sd.Rate() == 10000000U && s_SdioClkDiv == 10 && s_H.Clkdiv == 1);
	assert(sd.ReadSect(0, b, 2) == 2 && memcmp(b, s_Mem, 1024) == 0);
	printf("init cases: PASS\n");
}

static void TestTransfer()
{
	static const int types[] = { CARD_HC, CARD_V2 };
	static uint8_t w[200 * 512 + 2], r[200 * 512 + 2];

	for (int type : types)
	{
		SDCard sd;
		uint8_t cache[512];
		DiskIOCache_t desc = { 0, (uint32_t)-1, cache };

		Fresh(type);
		assert(Init(sd, Cfg(), &desc, 1));

		// Single and multiple, aligned and not
		for (uint32_t n : { 1U, 2U, 7U })
		{
			for (uint32_t off : { 0U, 1U })
			{
				uint32_t sect = 100 + n * 10 + off;
				Fill(w + off, n * 512, sect);
				int writes = s_C.Writes, stops = s_C.Stops;
				assert(sd.WriteSect(sect, w + off, n) == (int)n);
				assert(memcmp(s_Mem + sect * 512, w + off, n * 512) == 0);
				assert(s_C.Writes == writes + (int)n && s_C.Stops == stops + (n > 1 ? 1 : 0));
				memset(r, 0, sizeof(r));
				int reads = s_C.Reads;
				assert(sd.ReadSect(sect, r + off, n) == (int)n);
				assert(memcmp(r + off, w + off, n * 512) == 0);
				assert(s_C.Reads == reads + (int)n && s_C.Stops == stops + (n > 1 ? 2 : 0));
				assert(s_C.State == ST_TRAN && !CardBusy());
			}
		}
		assert(sd.SectWrite(5, w) && sd.SectRead(5, r) && memcmp(r, w, 512) == 0);
		assert(CardStatus(sd) == 0x900U && sd.Status() == 0x900U);

		// A long transfer
		Fill(w, 200 * 512, 3);
		assert(sd.WriteSect(1000, w, 200) == 200 && memcmp(s_Mem + 1000 * 512, w, 200 * 512) == 0);
		assert(sd.ReadSect(1000, r, 200) == 200 && memcmp(r, w, 200 * 512) == 0);

		// The last sectors, then past the end
		uint32_t last = CARD_SIZE / 512 - 1;
		assert(sd.WriteSect(last - 1, w, 2) == 2 && memcmp(s_Mem + (last - 1) * 512, w, 1024) == 0);
		assert(sd.ReadSect(last, r, 1) == 1);
		size_t cmds = s_C.Log.size();
		assert(sd.ReadSect(last, r, 2) == 0 && sd.ReadSect(last + 1, r, 1) == 0 && sd.WriteSect(last + 1, w, 1) == 0);
		assert(sd.ReadSect(last + 2, r, 1) == 0 && !sd.SectRead(last + 2, r) && !sd.SectWrite(0xFFFFFFFFU, w));
		assert(sd.ReadSect(0, r, 0) == 0 && sd.ReadSect(0, NULL, 1) == 0 && sd.WriteSect(0, NULL, 1) == 0);
		assert(!sd.SectRead(0, NULL) && !sd.SectWrite(0, NULL));
		assert(s_C.Log.size() == cmds);

		// The cached DiskIO path: the sector written on the flush
		Fill(w, 100, 77);
		assert(sd.Write((uint64_t)300 * 512 + 50, w, 100) == 100);
		assert(memcmp(s_Mem + 300 * 512 + 50, w, 100) != 0);
		sd.Flush();
		assert(memcmp(s_Mem + 300 * 512 + 50, w, 100) == 0);
		memset(r, 0, 100);
		assert(sd.Read((uint64_t)300 * 512 + 50, r, 100) == 100 && memcmp(r, w, 100) == 0);
	}

	// A large card
	SDCard sd;
	Fresh(CARD_HC);
	s_C.CSize = 0x3FFFFF;
	assert(Init(sd, Cfg()));
	assert(sd.ReadSect(0, r, 2) == 2 && memcmp(r, s_Mem, 1024) == 0);

	// A fast card and slow register accesses: the FIFO full on reads
	Fresh(CARD_HC);
	s_C.TranSpeed = 0x5A;
	s_C.MaxRate = 50000000;
	assert(Init(sd, Cfg(4, 50000000)));
	s_AccessNs = 500;
	Fill(w, 8 * 512, 5);
	assert(sd.WriteSect(60, w, 8) == 8 && memcmp(s_Mem + 60 * 512, w, 8 * 512) == 0);
	int full = s_H.Full;
	assert(sd.ReadSect(60, r, 8) == 8 && memcmp(r, w, 8 * 512) == 0 && s_H.Full > full + 8);
	s_AccessNs = 10;
	printf("transfers: PASS\n");
}

static void TestFault()
{
	SDCard sd;
	static uint8_t w[4 * 512], r[8 * 512];

	Fresh(CARD_HC);
	assert(Init(sd, Cfg()));
	Fill(w, sizeof(w), 9);

	// A data CRC error: the transfer fails, the stop ends it, the card back in
	// its transfer state, the next transfer works
	static const struct { int n, Block; } c[] = { { 1, 0 }, { 4, 1 }, { 4, 3 } };
	for (auto &e : c)
	{
		int aborts = s_C.Aborts;
		s_C.CrcBlock = e.Block;
		assert(sd.ReadSect(10, r, e.n) == 0);
		assert(s_C.State == ST_TRAN && s_C.Aborts == aborts + 1 && s_H.Mode == H_IDLE);
		assert(sd.ReadSect(10, r, e.n) == e.n && memcmp(r, s_Mem + 10 * 512, e.n * 512) == 0);
		s_C.CrcBlock = e.Block;
		assert(sd.WriteSect(20, w, e.n) == 0);
		assert(s_C.State == ST_TRAN && !CardBusy() && s_C.Aborts == aborts + 2);
		assert(sd.WriteSect(20, w, e.n) == e.n && memcmp(s_Mem + 20 * 512, w, e.n * 512) == 0);
	}

	// No response to the data command
	for (int n : { 17, 18, 24 })
	{
		s_C.NoResp = n;
		assert(sd.ReadSect(10, r, n == 18 ? 2 : 1) == 0 || n == 24);
		assert(n != 24 || sd.WriteSect(10, w, 1) == 0);
		assert(sd.ReadSect(10, r, 2) == 2 && memcmp(r, s_Mem + 10 * 512, 1024) == 0);
	}

	// An error in the R1 of a data command, the card sends the data: the
	// stop ends it
	for (int n : { 1, 3 })
	{
		int aborts = s_C.Aborts;
		s_C.ErrCmd = n == 1 ? 17 : 18;
		s_C.ErrBits = 0x00800000U;			// COM_CRC_ERROR
		assert(sd.ReadSect(40, r, n) == 0);
		assert(s_C.State == ST_TRAN && s_C.Aborts == aborts + 1);
		assert(sd.ReadSect(40, r, n) == n && memcmp(r, s_Mem + 40 * 512, n * 512) == 0);
		s_C.ErrCmd = n == 1 ? 24 : 25;
		assert(sd.WriteSect(40, w, n) == 0);
		assert(s_C.State == ST_TRAN && !CardBusy());
		assert(sd.WriteSect(40, w, n) == n && memcmp(s_Mem + 40 * 512, w, n * 512) == 0);
	}

	// The card stops its data
	s_C.StallAt = 200;
	assert(sd.ReadSect(70, r, 2) == 0);
	assert(sd.WriteSect(70, w, 2) == 0);
	s_C.StallAt = -1;
	assert(sd.ReadSect(70, r, 2) == 2 && memcmp(r, s_Mem + 70 * 512, 1024) == 0);

	// No status after a write, the host hangs on it
	s_C.NoResp = 13;
	assert(sd.WriteSect(30, w, 1) == 0 && CardStatus(sd) == 0xFFFFFFFFU);
	s_H.NoDone = 13;
	assert(sd.WriteSect(30, w, 1) == 0);
	assert(Init(sd, Cfg()) && sd.WriteSect(30, w, 1) == 1);

	// The card stays busy after a write
	s_C.StuckAfterWrite = true;
	assert(sd.WriteSect(30, w, 1) == 0);
	s_C.BusyForever = false;
	s_C.BusyEnd = 0;
	assert(sd.WriteSect(30, w, 1) == 1);

	// A programming error: in the status after a single block, in the
	// response of the auto stop after multiple blocks
	for (int n : { 1, 3 })
	{
		s_C.WriteErr = 0x04000000U;			// WP_VIOLATION
		assert(sd.WriteSect(50, w, n) == 0);
		assert(n > 1 || (CardStatus(sd) & 0x04000000U));
		assert(sd.WriteSect(50, w, n) == n && CardStatus(sd) == 0x900U);
	}

	// A card larger in its CSD than in fact: the card refuses an address past
	// its end, the stop ends the wait for its data
	Fresh(CARD_V2);
	s_C.BlLen = 11;
	assert(Init(sd, Cfg()) && sd.GetNbSect() == CARD_SIZE / 256U);
	int aborts = s_C.Aborts;
	assert(sd.ReadSect(CARD_SIZE / 512U, r, 1) == 0);
	assert(s_C.Aborts == aborts + 1 && s_H.Mode == H_IDLE);
	assert(sd.WriteSect(CARD_SIZE / 512U + 2, w, 2) == 0);
	assert(s_C.Aborts == aborts + 2 && s_H.Mode == H_IDLE);
	assert(sd.ReadSect(3, r, 2) == 2 && memcmp(r, s_Mem + 3 * 512, 1024) == 0);

	// A data CRC error with the FIFO full: the FIFO emptied before the stop
	Fresh(CARD_HC);
	s_C.TranSpeed = 0x5A;
	s_C.MaxRate = 50000000;
	assert(Init(sd, Cfg(4, 50000000)));
	s_AccessNs = 500;
	s_C.CrcBlock = 2;
	assert(sd.ReadSect(60, r, 8) == 0);
	assert(sd.ReadSect(60, r, 8) == 8 && memcmp(r, s_Mem + 60 * 512, 8 * 512) == 0);
	s_AccessNs = 10;
	printf("faults: PASS\n");
}

// The interface alone: frames, responses, data, refusals
static uint8_t *Frame(uint8_t *f, uint8_t Cmd, uint32_t Arg, uint8_t Flags)
{
	f[0] = 0x40U | Cmd;
	f[1] = (uint8_t)(Arg >> 24);
	f[2] = (uint8_t)(Arg >> 16);
	f[3] = (uint8_t)(Arg >> 8);
	f[4] = (uint8_t)Arg;
	f[5] = 1;
	f[6] = Flags;
	return f;
}

static void TestIntrf()
{
	SDCard sd;
	static uint8_t w[1024], r[1024];
	uint8_t f[SDMMC_FRAME_SIZE], resp[16];
	DevIntrf_t *ip = s_Host;
	uint32_t rca = CARD_RCA << 16;

	Fresh(CARD_HC);
	assert(Init(sd, Cfg()));
	assert(s_Host.Type() == DEVINTRF_TYPE_SDMMC && SdmmcGetHandle(ip) == (SdmmcDev_t *)s_Host);
	assert(ip->MaxRetry == 0 && ip->bDma == false && ip->bIntEn == false);

	// Status: 4 bytes, most significant first
	assert(s_Host.Read(0, Frame(f, 13, rca, SDMMC_FRAME_RESP_R1), SDMMC_FRAME_SIZE, resp, 4) == 4);
	assert(resp[0] == 0 && resp[1] == 0 && resp[2] == 0x09 && resp[3] == 0);
	// A shorter buffer
	assert(s_Host.Read(0, Frame(f, 13, rca, SDMMC_FRAME_RESP_R1), SDMMC_FRAME_SIZE, resp, 2) == 2);
	// The frame in pieces
	assert(DeviceIntrfStartTx(ip, 0));
	assert(DeviceIntrfTxData(ip, Frame(f, 13, rca, SDMMC_FRAME_RESP_R1), 3) == 3);
	assert(DeviceIntrfRxData(ip, resp, 4) == 0);
	assert(DeviceIntrfTxData(ip, f + 3, 4) == 4);
	ip->StartRx(ip, 0);
	assert(DeviceIntrfRxData(ip, resp, 4) == 4 && resp[2] == 0x09);
	// The command sent once
	assert(DeviceIntrfTxData(ip, f, 1) == 0);
	DeviceIntrfStopRx(ip);
	// No response kept for a new transfer
	assert(DeviceIntrfStartRx(ip, 0));
	assert(DeviceIntrfRxData(ip, resp, 4) == 0);
	DeviceIntrfStopRx(ip);

	// No answer: nothing to read
	assert(s_Host.Read(0, Frame(f, 9, rca, SDMMC_FRAME_RESP_R2), SDMMC_FRAME_SIZE, resp, 16) == 0);

	// A no data command with more bytes than its frame: run, the bytes refused
	assert(DeviceIntrfStartTx(ip, 0));
	assert(DeviceIntrfTxData(ip, NULL, SDMMC_FRAME_SIZE) == 0);
	memcpy(w, Frame(f, 13, rca, SDMMC_FRAME_RESP_R1), SDMMC_FRAME_SIZE);
	assert(DeviceIntrfTxData(ip, w, SDMMC_FRAME_SIZE + 1) == 0);
	DeviceIntrfStopTx(ip);

	// Frame and data in one transfer, the write through Write
	Fill(w, 512, 31);
	assert(s_Host.Write(0, Frame(f, 24, 40, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_WRITE), SDMMC_FRAME_SIZE,
						w, 512) == 512);
	assert(memcmp(s_Mem + 40 * 512, w, 512) == 0);
	assert(s_Host.Read(0, Frame(f, 17, 40, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_READ), SDMMC_FRAME_SIZE,
					   r, 512) == 512 && memcmp(r, w, 512) == 0);
	// The data of a transfer moved once
	int reads = s_C.Reads, writes = s_C.Writes;
	assert(DeviceIntrfStartTx(ip, 0));
	assert(DeviceIntrfTxData(ip, Frame(f, 17, 41, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_READ), SDMMC_FRAME_SIZE) ==
		   SDMMC_FRAME_SIZE);
	ip->StartRx(ip, 0);
	assert(DeviceIntrfRxData(ip, r, 512) == 512 && DeviceIntrfRxData(ip, r, 512) == 0);
	DeviceIntrfStopRx(ip);
	// A frame not complete after a read: nothing run
	assert(DeviceIntrfStartTx(ip, 0));
	assert(DeviceIntrfTxData(ip, Frame(f, 17, 41, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_READ), 3) == 3);
	ip->StartRx(ip, 0);
	assert(DeviceIntrfRxData(ip, r, 512) == 0);
	DeviceIntrfStopRx(ip);
	assert(DeviceIntrfStartTx(ip, 0));
	assert(DeviceIntrfTxData(ip, Frame(f, 24, 41, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_WRITE), SDMMC_FRAME_SIZE) ==
		   SDMMC_FRAME_SIZE);
	assert(DeviceIntrfTxData(ip, w, 512) == 512 && DeviceIntrfTxData(ip, w, 512) == 0);
	DeviceIntrfStopTx(ip);
	assert(s_C.Reads == reads + 1 && s_C.Writes == writes + 1);
	// CMD0 with the initialization clocks: 1 data line again
	assert(s_Host.BusWidth() == 4);
	assert(s_Host.Tx(0, Frame(f, 0, 0, SDMMC_FRAME_INIT), SDMMC_FRAME_SIZE) == SDMMC_FRAME_SIZE);
	assert(s_Host.BusWidth() == 1 && (s_H.Ctype & SDIF_CTYPE_CARD_WIDTH0_MASK) == 0 && s_C.State == ST_IDLE);
	assert(sd.Init(&s_Host) && s_Host.BusWidth() == 4);
	// The host set up again: 1 data line
	assert(s_Host.Init(Cfg()) && s_Host.BusWidth() == 1);
	assert(sd.Init(&s_Host));

	// Refused, nothing sent: a length not in blocks, a bad start of frame,
	// the 4 bit bus on a 1 bit host, both directions, a response kind not
	// known, another slot
	int cmds = s_H.Cmds;
	assert(s_Host.Read(0, Frame(f, 18, 40, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_READ | SDMMC_FRAME_STOP),
					   SDMMC_FRAME_SIZE, r, 513) == 0);
	assert(s_Host.Read(0, Frame(f, 17, 40, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_READ), SDMMC_FRAME_SIZE,
					   r, 0) == 0);
	f[0] = 13;
	assert(s_Host.Read(0, f, SDMMC_FRAME_SIZE, resp, 4) == 0);
	assert(s_Host.Read(0, Frame(f, 13, rca, 5), SDMMC_FRAME_SIZE, resp, 4) == 0);
	assert(s_Host.Read(0, Frame(f, 17, 40, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_MASK), SDMMC_FRAME_SIZE,
					   r, 512) == 0);
	assert(s_Host.Read(1, Frame(f, 13, rca, SDMMC_FRAME_RESP_R1), SDMMC_FRAME_SIZE, resp, 4) == 0);
	assert(s_Host.Tx(0, w, 0) == 0);
	assert(s_H.Cmds == cmds);
	Fresh(CARD_HC);
	assert(Init(sd, Cfg(1)));
	cmds = s_H.Cmds;
	assert(s_Host.Read(0, Frame(f, 6, 2, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_BUS_WIDTH), SDMMC_FRAME_SIZE,
					   resp, 4) == 0 && s_H.Cmds == cmds);
	// A write frame without its data
	assert(s_Host.Tx(0, Frame(f, 24, 40, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_WRITE), SDMMC_FRAME_SIZE) ==
		   SDMMC_FRAME_SIZE && s_H.Cmds == cmds);

	// Rate, clock off and on, reset, retry count
	assert(s_Host.Rate(400000) == 400000 && s_Host.Rate() == 400000);
	assert(s_Host.Rate(25000000) == 220000000U / 9U);
	s_Host.Disable();
	assert(s_H.EffEna == 0);
	s_Host.Enable();
	assert(s_H.EffEna != 0);
	s_Host.Reset();
	assert(sd.ReadSect(4, r, 2) == 2 && memcmp(r, s_Mem + 2048, 1024) == 0);
	printf("interface: PASS\n");
}

static void TestPower()
{
	SDCard sd;
	uint8_t b[512];

	Fresh(CARD_HC);
	assert(Init(sd, Cfg()));
	s_Host.PowerOff();
	assert((s_Ahb[2] & SYSCON_AHBCLKCTRL_SDIO_MASK) == 0 && s_SdioClkSel == 7 && !s_C.Powered);
	assert((s_Rst[2] & SYSCON_PRESETCTRL_SDIO_RST_MASK) == 0);
	for (const IOPinCfg_t &p : s_Pins)
	{
		assert(IOCON->PIO[p.PortNo][p.PinNo] == 0);
	}
	// No register access powered off
	assert(!sd.SectRead(0, b) && s_Host.Rate() == 0 && s_Host.Rate(400000) == 0);
	s_Host.Disable();
	s_Host.Enable();
	s_Host.Reset();
	s_Host.PowerOff();
	assert(Init(sd, Cfg()) && sd.SectRead(0, b) && memcmp(b, s_Mem, 512) == 0);

	// Never set up
	Sdmmc other;
	SDCard sd2;
	other.PowerOff();
	assert(!sd2.Init(&other) && other.Type() == DEVINTRF_TYPE_NULL);
	printf("power: PASS\n");
}

int main()
{
	setvbuf(stdout, NULL, _IONBF, 0);
	TestInit();
	TestInitFail();
	TestInitOther();
	TestTransfer();
	TestFault();
	TestIntrf();
	TestPower();
	printf("sdif: PASS\n");

	return 0;
}
