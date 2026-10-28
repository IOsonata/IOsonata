/**-------------------------------------------------------------------------
@file	sdcard_impl.cpp

@brief	SD card driver

See sdcard.h. Card commands as the SD physical layer simplified
specification: identification (CMD0, CMD8, ACMD41, then CMD2, CMD3, CMD9 and
CMD7 on the SD bus, CMD58, CMD9 and CMD10 in SPI mode), CMD16 for the byte
addressed cards, ACMD6 for 4 data lines, block commands CMD17, CMD18, CMD24,
CMD25, stopped by CMD12 or the stop token in SPI mode, CMD13 for the status.

In SPI mode a command and its data are one transaction: the chip select
stays low from the command token to 8 clocks after the last byte. The data
blocks are checked with their CRC16.

@author	Hoang Nguyen Hoan
@date	June 9, 2011

@license

MIT license

Copyright (c) 2011, I-SYST, all rights reserved

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

#include "crc.h"
#include "idelay.h"
#include "coredev/sdmmc.h"
#include "storage/sdcard.h"

#define SDCARD_ID_RATE			400000U		// Identification clock
#define SDCARD_DEFAULT_RATE		25000000U	// Default speed
#define SDCARD_INIT_TRY			100U		// ACMD41 10 ms apart, one second
#define SDCARD_INIT_DELAY_US	10000U
#define SDCARD_CMD0_TRY			10
#define SDCARD_READ_MS			100U		// Read access time
#define SDCARD_WRITE_MS			500U		// Programming time of a block
#define SDCARD_NCR_MAX			10			// Bytes before an SPI response
// Sectors of one SD bus transfer, the length is an int
#define SDCARD_BUS_XFER_MAX		(0x7FFFFFFFU / SDCARD_SECT_SIZE)

// Commands
#define SD_CMD_GO_IDLE			0
#define SD_CMD_SEND_OP_COND		1			// MMC
#define SD_CMD_ALL_SEND_CID		2
#define SD_CMD_SEND_RCA			3
#define SD_CMD_SELECT			7
#define SD_CMD_SEND_IF_COND		8
#define SD_CMD_SEND_CSD			9
#define SD_CMD_SEND_CID			10
#define SD_CMD_STOP				12
#define SD_CMD_SEND_STATUS		13
#define SD_CMD_SET_BLOCKLEN		16
#define SD_CMD_READ_SINGLE		17
#define SD_CMD_READ_MULTIPLE	18
#define SD_CMD_WRITE_SINGLE		24
#define SD_CMD_WRITE_MULTIPLE	25
#define SD_CMD_APP				55
#define SD_CMD_READ_OCR			58			// SPI mode
#define SD_ACMD_SET_BUS_WIDTH	6
#define SD_ACMD_SEND_OP_COND	41

#define SD_IF_COND_ARG			0x1AAU		// 2.7 to 3.6 V, check pattern 0xAA
#define SD_OCR_VOLT				0x00FF8000U	// 2.7 to 3.6 V
#define SD_OCR_HCS				0x40000000U	// Host takes high capacity cards
#define SD_OCR_CCS				0x40000000U	// High capacity card
#define SD_OCR_READY			0x80000000U	// Power up done

// Card status (R1) on the SD bus
#define SD_R1_ERRORS			SDMMC_R1_ERRORS
#define SD_R1_APP_CMD			SDMMC_R1_APP_CMD

// SPI mode
#define SD_SPI_R1_IDLE			0x01U
#define SD_SPI_R1_ILLEGAL		0x04U
#define SD_SPI_R1_RANGE			0x60U		// Address and parameter errors
#define SD_SPI_R2_ERRORS		0xFFFEU		// R1 and second byte, card locked aside
#define SD_SPI_TOKEN_SINGLE		0xFEU		// Start of a block, single block write and reads
#define SD_SPI_TOKEN_MULTI		0xFCU		// Start of a block of a multiple block write
#define SD_SPI_TOKEN_STOP		0xFDU		// End of a multiple block write
#define SD_SPI_DATA_ACCEPTED	0x05U		// Data response xxx0sss1, sss 010

static inline uint32_t SDCardBe32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// Command token: start and transmission bits with the index, the argument,
// the CRC7 (crc8_ccitt gives it in bits 7:1) and the end bit
static void SDCardToken(uint8_t *pTok, uint8_t Cmd, uint32_t Arg)
{
	pTok[0] = 0x40U | (Cmd & 0x3FU);
	pTok[1] = (uint8_t)(Arg >> 24);
	pTok[2] = (uint8_t)(Arg >> 16);
	pTok[3] = (uint8_t)(Arg >> 8);
	pTok[4] = (uint8_t)Arg;
	pTok[5] = crc8_ccitt(pTok, 5, 0) | 1U;
}

SDCard::SDCard() : vpInterf(nullptr), vDevAddr(0), vbSdBus(false), vbTxPhase(false)
{
	memset(&vDev, 0, sizeof(vDev));
	memset(vCacheDesc, 0, sizeof(vCacheDesc));
}

bool SDCard::Init(DeviceIntrf * const pDevInterf, uint8_t * const pCacheMem, int CacheMemSize)
{
	int nb = 0;

	if (pCacheMem != nullptr && CacheMemSize > 0)
	{
		nb = CacheMemSize / SDCARD_SECT_SIZE;
		if (nb > SDCARD_CACHE_MAX)
		{
			nb = SDCARD_CACHE_MAX;
		}
		for (int i = 0; i < nb; i++)
		{
			vCacheDesc[i].pSectData = pCacheMem + i * SDCARD_SECT_SIZE;
		}
	}

	return Init(pDevInterf, nb > 0 ? vCacheDesc : (DiskIOCache_t *)nullptr, nb);
}

bool SDCard::Init(DeviceIntrf * const pDevInterf, DiskIOCache_t * const pCacheBlk, int NbCacheBlk)
{
	// Sectors of a card before are not kept
	DiskIO::Reset();
	memset(&vDev, 0, sizeof(vDev));
	vpInterf = pDevInterf;

	if (pDevInterf == nullptr)
	{
		return false;
	}

	DEVINTRF_TYPE type = pDevInterf->Type();

	if (type != DEVINTRF_TYPE_SPI && type != DEVINTRF_TYPE_SDMMC)
	{
		return false;
	}
	vbSdBus = type == DEVINTRF_TYPE_SDMMC;

	uint32_t rate = pDevInterf->Rate();

	(void)RateNotAbove(SDCARD_ID_RATE);

	uint32_t max = Identify() ? SetupCsd() : 0;

	if (max == 0)
	{
		vDev.Type = SDCARD_TYPE_NONE;
		pDevInterf->Rate(rate);

		return false;
	}

	// The rate of the interface, the card maximum when lower
	if (rate == 0 || rate > max)
	{
		rate = max;
	}
	vDev.Rate = RateNotAbove(rate);

	if (pCacheBlk != nullptr && NbCacheBlk > 0)
	{
		SetCache(pCacheBlk, NbCacheBlk);
	}

	return true;
}

// The interface clock closest to Max. An interface giving a clock above Max
// is asked again, lower by the excess: 400 kHz giving 500 kHz asks 320 kHz.
// The slowest clock of the interface when none is at or below Max.
uint32_t SDCard::RateNotAbove(uint32_t Max)
{
	uint32_t ask = Max;
	uint32_t r = vpInterf->Rate(ask);

	for (int i = 0; i < 8 && r > Max && ask > 1; i++)
	{
		ask = (uint32_t)((uint64_t)ask * Max / r);
		r = vpInterf->Rate(ask > 0 ? ask : 1);
	}

	return r;
}

// Card idle (CMD0), its interface condition (CMD8, version 2), then the power
// up and registers on its bus. Sets the card type.
bool SDCard::Identify(void)
{
	uint8_t r[4];
	bool v2 = false;

	if (vbSdBus)
	{
		if (BusCmd(SD_CMD_GO_IDLE, 0, SDMMC_FRAME_RESP_NONE | SDMMC_FRAME_INIT) == false)
		{
			return false;
		}
		// No response from a version 1 card
		if (BusCmd(SD_CMD_SEND_IF_COND, SD_IF_COND_ARG, SDMMC_FRAME_RESP_R1, r))
		{
			if ((SDCardBe32(r) & 0xFFFU) != SD_IF_COND_ARG)
			{
				return false;
			}
			v2 = true;
		}

		return BusIdentify(v2);
	}

	// 80 clocks with the data line high, then CMD0 with the chip select low:
	// the card goes to SPI mode
	uint8_t ff[10];

	memset(ff, 0xFF, sizeof(ff));
	if (SpiBegin() == false)
	{
		return false;
	}
	bool ok = SpiTx(ff, sizeof(ff));
	SpiEnd();
	if (ok == false)
	{
		return false;
	}

	int r1 = -1;

	for (int i = 0; i < SDCARD_CMD0_TRY && r1 != (int)SD_SPI_R1_IDLE; i++)
	{
		r1 = SpiOneCmd(SD_CMD_GO_IDLE, 0);
	}
	if (r1 != (int)SD_SPI_R1_IDLE)
	{
		return false;
	}

	// A version 1 card or an MMC does not know CMD8
	r1 = SpiOneCmd(SD_CMD_SEND_IF_COND, SD_IF_COND_ARG, r, 4);
	if (r1 == (int)SD_SPI_R1_IDLE)
	{
		if ((SDCardBe32(r) & 0xFFFU) != SD_IF_COND_ARG)
		{
			return false;
		}
		v2 = true;
	}
	else if (r1 < 0 || (r1 & SD_SPI_R1_ILLEGAL) == 0)
	{
		return false;
	}

	return SpiIdentify(v2);
}

// Capacity and highest clock from the CSD. Returns that clock, 0 when the CSD
// is not taken.
uint32_t SDCard::SetupCsd(void)
{
	const uint8_t *c = vDev.Csd;
	uint32_t ver = c[0] >> 6;
	uint64_t nb;

	if (vDev.Type == SDCARD_TYPE_SDHC)
	{
		// CSD version 2: (C_SIZE + 1) x 512 KB. Version 3 (SDUC) is not taken.
		if (ver != 1)
		{
			return 0;
		}

		uint32_t csize = ((uint32_t)(c[7] & 0x3FU) << 16) | ((uint32_t)c[8] << 8) | c[9];

		nb = ((uint64_t)csize + 1U) * 1024U;
	}
	else
	{
		// CSD version 1 and MMC: (C_SIZE + 1) x 2^(C_SIZE_MULT + 2) blocks of
		// 2^READ_BL_LEN bytes
		if (vDev.Type != SDCARD_TYPE_MMC && ver != 0)
		{
			return 0;
		}

		uint32_t bl = c[5] & 0x0FU;
		uint32_t csize = ((uint32_t)(c[6] & 3U) << 10) | ((uint32_t)c[7] << 2) | (c[8] >> 6);
		uint32_t mult = ((uint32_t)(c[9] & 3U) << 1) | (c[10] >> 7);

		if (bl < 9 || bl > 11)
		{
			return 0;
		}
		nb = (((uint64_t)csize + 1U) << (mult + 2U + bl)) / SDCARD_SECT_SIZE;
	}
	vDev.NbSect = nb > 0xFFFFFFFFU ? 0xFFFFFFFFU : (uint32_t)nb;

	// TRAN_SPEED: unit 100 kb/s to 100 Mb/s times a value 1.0 to 8.0
	static const uint8_t s_TranVal[16] = { 0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80 };
	static const uint32_t s_TranUnit[4] = { 10000, 100000, 1000000, 10000000 };
	uint32_t ts = c[3];
	uint32_t rate = (ts & 7U) < 4U ? s_TranUnit[ts & 7U] * s_TranVal[(ts >> 3) & 15U] : 0;

	return rate > 0 ? rate : SDCARD_DEFAULT_RATE;
}

uint32_t SDCard::CardAddr(uint32_t SectNo) const
{
	// Sector numbers on high capacity cards, bytes on the others
	return vDev.Type == SDCARD_TYPE_SDHC ? SectNo : SectNo * SDCARD_SECT_SIZE;
}

int SDCard::ReadSect(uint32_t SectNo, uint8_t *pBuff, uint32_t NbSect)
{
	if (Ready() == false || pBuff == nullptr || NbSect == 0 || SectNo >= vDev.NbSect ||
		NbSect > vDev.NbSect - SectNo)
	{
		return 0;
	}

	return vbSdBus ? BusRead(SectNo, pBuff, NbSect) : SpiRead(SectNo, pBuff, NbSect);
}

int SDCard::WriteSect(uint32_t SectNo, const uint8_t *pData, uint32_t NbSect)
{
	if (Ready() == false || pData == nullptr || NbSect == 0 || SectNo >= vDev.NbSect ||
		NbSect > vDev.NbSect - SectNo)
	{
		return 0;
	}

	return vbSdBus ? BusWrite(SectNo, pData, NbSect) : SpiWrite(SectNo, pData, NbSect);
}

int SDCard::ReadSingleBlock(uint32_t Addr, uint8_t *pData, int Len)
{
	if (Len != SDCARD_SECT_SIZE)
	{
		return 0;
	}

	return ReadSect(Addr, pData, 1) == 1 ? Len : 0;
}

int SDCard::WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len)
{
	if (Len != SDCARD_SECT_SIZE)
	{
		return 0;
	}

	return WriteSect(Addr, pData, 1) == 1 ? Len : 0;
}

uint32_t SDCard::Status(void)
{
	uint32_t st = 0xFFFFFFFFU;

	if (vpInterf != nullptr)
	{
		uint8_t r[4];

		if (vbSdBus)
		{
			if (BusCmd(SD_CMD_SEND_STATUS, vDev.Rca, SDMMC_FRAME_RESP_R1, r))
			{
				st = SDCardBe32(r);
			}
		}
		else
		{
			// R2: R1 then a second byte
			int r1 = SpiOneCmd(SD_CMD_SEND_STATUS, 0, r, 1);

			if (r1 >= 0)
			{
				st = ((uint32_t)r1 << 8) | r[0];
			}
		}
	}
	vDev.Status = st;

	return st;
}

// ---- SD bus ----

// One command: the frame, then the response when the command has one, into
// pResp when given (4 bytes, 16 for R2). true when the card answered.
bool SDCard::BusCmd(uint8_t Cmd, uint32_t Arg, uint8_t Flags, uint8_t *pResp)
{
	uint8_t f[SDMMC_FRAME_SIZE];
	uint8_t r[SDMMC_RESP_LONG_LEN];
	uint32_t resp = Flags & SDMMC_FRAME_RESP_MASK;
	int len = resp == SDMMC_FRAME_RESP_NONE ? 0 :
			  resp == SDMMC_FRAME_RESP_R2 ? SDMMC_RESP_LONG_LEN : SDMMC_RESP_SHORT_LEN;

	SDCardToken(f, Cmd, Arg);
	f[SDMMC_FRAME_FLAGS_IDX] = Flags;

	if (len == 0)
	{
		return vpInterf->Tx(vDevAddr, f, sizeof(f)) == (int)sizeof(f);
	}

	return vpInterf->Read(vDevAddr, f, sizeof(f), pResp != nullptr ? pResp : r, len) == len;
}

// CMD55 with the card address, taken by the card, then the command
bool SDCard::BusAppCmd(uint8_t Cmd, uint32_t Arg, uint8_t Flags, uint8_t *pResp)
{
	uint8_t r[SDMMC_RESP_SHORT_LEN];

	if (BusCmd(SD_CMD_APP, vDev.Rca, SDMMC_FRAME_RESP_R1, r) == false)
	{
		return false;
	}

	uint32_t st = SDCardBe32(r);

	if ((st & SD_R1_APP_CMD) == 0 || (st & SD_R1_ERRORS))
	{
		return false;
	}

	return BusCmd(Cmd, Arg, Flags, pResp);
}

// Power up (ACMD41), CID (CMD2), address (CMD3), CSD (CMD9), selection
// (CMD7), bus width (ACMD6) and block length (CMD16)
bool SDCard::BusIdentify(bool bV2)
{
	uint8_t r[SDMMC_RESP_SHORT_LEN];
	uint32_t ocr = 0;

	for (uint32_t i = 0; i < SDCARD_INIT_TRY; i++)
	{
		if (BusAppCmd(SD_ACMD_SEND_OP_COND, SD_OCR_VOLT | (bV2 ? SD_OCR_HCS : 0U), SDMMC_FRAME_RESP_R3, r) == false)
		{
			return false;
		}
		ocr = SDCardBe32(r);
		if (ocr & SD_OCR_READY)
		{
			break;
		}
		usDelay(SDCARD_INIT_DELAY_US);
	}
	if ((ocr & SD_OCR_READY) == 0)
	{
		return false;
	}
	vDev.Ocr = ocr;

	SDCARD_TYPE type = bV2 == false ? SDCARD_TYPE_SDSC_V1 :
					   (ocr & SD_OCR_CCS) ? SDCARD_TYPE_SDHC : SDCARD_TYPE_SDSC;

	if (BusCmd(SD_CMD_ALL_SEND_CID, 0, SDMMC_FRAME_RESP_R2, vDev.Cid) == false ||
		BusCmd(SD_CMD_SEND_RCA, 0, SDMMC_FRAME_RESP_R1, r) == false)
	{
		return false;
	}
	vDev.Rca = SDCardBe32(r) & 0xFFFF0000U;

	if (BusCmd(SD_CMD_SEND_CSD, vDev.Rca, SDMMC_FRAME_RESP_R2, vDev.Csd) == false ||
		BusCmd(SD_CMD_SELECT, vDev.Rca, SDMMC_FRAME_RESP_R1B, r) == false || (SDCardBe32(r) & SD_R1_ERRORS))
	{
		return false;
	}

	// 4 data lines when the host has them
	SdmmcDev_t *host = SdmmcGetHandle(*vpInterf);

	if (host != nullptr && host->Cfg.BusWidth >= 4)
	{
		if (BusAppCmd(SD_ACMD_SET_BUS_WIDTH, 2, SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_BUS_WIDTH, r) == false ||
			(SDCardBe32(r) & SD_R1_ERRORS))
		{
			return false;
		}
	}

	if (type != SDCARD_TYPE_SDHC &&
		(BusCmd(SD_CMD_SET_BLOCKLEN, SDCARD_SECT_SIZE, SDMMC_FRAME_RESP_R1, r) == false ||
		 (SDCardBe32(r) & SD_R1_ERRORS)))
	{
		return false;
	}
	vDev.Type = type;

	return true;
}

int SDCard::BusRead(uint32_t SectNo, uint8_t *pBuff, uint32_t NbSect)
{
	uint32_t done = 0;

	while (done < NbSect)
	{
		uint32_t n = NbSect - done;
		uint8_t f[SDMMC_FRAME_SIZE];

		if (n > SDCARD_BUS_XFER_MAX)
		{
			n = SDCARD_BUS_XFER_MAX;
		}

		int len = (int)(n * SDCARD_SECT_SIZE);

		SDCardToken(f, n > 1 ? SD_CMD_READ_MULTIPLE : SD_CMD_READ_SINGLE, CardAddr(SectNo + done));
		f[SDMMC_FRAME_FLAGS_IDX] = SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_READ | (n > 1 ? SDMMC_FRAME_STOP : 0U);

		if (vpInterf->Read(vDevAddr, f, sizeof(f), pBuff + done * SDCARD_SECT_SIZE, len) != len)
		{
			// The card errors
			(void)Status();
			break;
		}
		done += n;
	}

	return (int)done;
}

int SDCard::BusWrite(uint32_t SectNo, const uint8_t *pData, uint32_t NbSect)
{
	DevIntrf_t *ip = *vpInterf;
	uint32_t done = 0;

	while (done < NbSect)
	{
		uint32_t n = NbSect - done;
		uint8_t f[SDMMC_FRAME_SIZE];

		if (n > SDCARD_BUS_XFER_MAX)
		{
			n = SDCARD_BUS_XFER_MAX;
		}

		int len = (int)(n * SDCARD_SECT_SIZE);
		int cnt = 0;

		SDCardToken(f, n > 1 ? SD_CMD_WRITE_MULTIPLE : SD_CMD_WRITE_SINGLE, CardAddr(SectNo + done));
		f[SDMMC_FRAME_FLAGS_IDX] = SDMMC_FRAME_RESP_R1 | SDMMC_FRAME_DATA_WRITE | (n > 1 ? SDMMC_FRAME_STOP : 0U);

		// The frame and its data are one transfer
		if (DeviceIntrfStartTx(ip, vDevAddr))
		{
			ip->bNoStop = true;
			if (DeviceIntrfTxData(ip, f, sizeof(f)) == (int)sizeof(f))
			{
				cnt = DeviceIntrfTxData(ip, pData + done * SDCARD_SECT_SIZE, len);
			}
			ip->bNoStop = false;
			DeviceIntrfStopTx(ip);
		}

		// The errors of the programming
		uint32_t st = Status();

		if (cnt != len || st == 0xFFFFFFFFU || (st & SD_R1_ERRORS))
		{
			break;
		}
		done += n;
	}

	return (int)done;
}

// ---- SPI mode ----

// Opens a transaction: chip select low
bool SDCard::SpiBegin(void)
{
	DevIntrf_t *ip = *vpInterf;

	if (DeviceIntrfStartTx(ip, vDevAddr) == false)
	{
		return false;
	}
	ip->bNoStop = true;
	vbTxPhase = true;

	return true;
}

// 8 clocks after the last byte, then the chip select released
void SDCard::SpiEnd(void)
{
	DevIntrf_t *ip = *vpInterf;
	uint8_t d;

	(void)SpiRx(&d, 1);
	ip->bNoStop = false;
	if (vbTxPhase)
	{
		DeviceIntrfStopTx(ip);
	}
	else
	{
		DeviceIntrfStopRx(ip);
	}
}

bool SDCard::SpiTx(const uint8_t *pData, int Len)
{
	DevIntrf_t *ip = *vpInterf;

	if (vbTxPhase == false)
	{
		// Transmit phase of the open transaction
		if (ip->StartTx(ip, vDevAddr) == false)
		{
			return false;
		}
		vbTxPhase = true;
	}

	return DeviceIntrfTxData(ip, pData, Len) == Len;
}

bool SDCard::SpiRx(uint8_t *pBuff, int Len)
{
	DevIntrf_t *ip = *vpInterf;

	if (vbTxPhase)
	{
		// Receive phase of the open transaction
		if (ip->StartRx(ip, vDevAddr) == false)
		{
			return false;
		}
		vbTxPhase = false;
	}

	return DeviceIntrfRxData(ip, pBuff, Len) == Len;
}

// Bytes read until one is Val (bEqual) or is not Val, for the clocks of Ms
// milliseconds. The last byte read in pLast.
bool SDCard::SpiWait(uint8_t Val, bool bEqual, uint32_t Ms, uint8_t *pLast)
{
	uint32_t rate = vpInterf->Rate();
	uint64_t n = (uint64_t)(rate > 0 ? rate : SDCARD_ID_RATE) * Ms / 8000U + 1U;
	uint8_t b = 0;
	bool found = false;

	for (uint64_t i = 0; i < n && found == false; i++)
	{
		if (SpiRx(&b, 1) == false)
		{
			break;
		}
		found = (b == Val) == bEqual;
	}
	if (pLast != nullptr)
	{
		*pLast = b;
	}

	return found;
}

// A command in the open transaction, then Len response bytes after R1.
// Returns R1, -1 when the card does not answer.
int SDCard::SpiCmd(uint8_t Cmd, uint32_t Arg, uint8_t *pResp, int Len)
{
	uint8_t tok[6];
	uint8_t r1 = 0xFF;

	// The card ready, data line high. CMD12 stops data on its way.
	if (Cmd != SD_CMD_GO_IDLE && Cmd != SD_CMD_STOP && SpiWait(0xFF, true, SDCARD_WRITE_MS) == false)
	{
		return -1;
	}

	SDCardToken(tok, Cmd, Arg);
	if (SpiTx(tok, sizeof(tok)) == false)
	{
		return -1;
	}
	if (Cmd == SD_CMD_STOP)
	{
		// Stuff byte
		if (SpiRx(&r1, 1) == false)
		{
			return -1;
		}
		r1 = 0xFF;
	}

	// The response comes 0 to 8 bytes after the command
	for (int i = 0; i < SDCARD_NCR_MAX && (r1 & 0x80U); i++)
	{
		if (SpiRx(&r1, 1) == false)
		{
			return -1;
		}
	}
	if (r1 & 0x80U)
	{
		return -1;
	}
	if (pResp != nullptr && Len > 0 && SpiRx(pResp, Len) == false)
	{
		return -1;
	}

	return r1;
}

// A command in its own transaction
int SDCard::SpiOneCmd(uint8_t Cmd, uint32_t Arg, uint8_t *pResp, int Len)
{
	if (SpiBegin() == false)
	{
		return -1;
	}

	int r1 = SpiCmd(Cmd, Arg, pResp, Len);

	SpiEnd();

	return r1;
}

// A data block from the card: start token, data, CRC16
bool SDCard::SpiReadBlock(uint8_t *pBuff, int Len)
{
	uint8_t t = 0;
	uint8_t crc[2];

	// The start token, or an error token (0000xxxx)
	if (SpiWait(0xFF, false, SDCARD_READ_MS, &t) == false || t != SD_SPI_TOKEN_SINGLE)
	{
		return false;
	}
	if (SpiRx(pBuff, Len) == false || SpiRx(crc, 2) == false)
	{
		return false;
	}

	return crc16_ccitt(pBuff, Len, 0) == (uint16_t)((crc[0] << 8) | crc[1]);
}

// A data block to the card: a byte, the start token, the data, the CRC16,
// then the data response and the programming (busy, data line low)
bool SDCard::SpiWriteBlock(uint8_t Token, const uint8_t *pData)
{
	uint8_t hdr[2] = { 0xFF, Token };
	uint16_t crc = crc16_ccitt((uint8_t *)pData, SDCARD_SECT_SIZE, 0);
	uint8_t c[2] = { (uint8_t)(crc >> 8), (uint8_t)crc };
	uint8_t resp = 0;

	if (SpiTx(hdr, 2) == false || SpiTx(pData, SDCARD_SECT_SIZE) == false || SpiTx(c, 2) == false)
	{
		return false;
	}
	if (SpiWait(0xFF, false, SDCARD_READ_MS, &resp) == false || (resp & 0x1FU) != SD_SPI_DATA_ACCEPTED)
	{
		return false;
	}

	return SpiWait(0x00, false, SDCARD_WRITE_MS);
}

// Power up (ACMD41, CMD1 for an MMC), OCR (CMD58), CSD (CMD9), CID (CMD10),
// block length (CMD16)
bool SDCard::SpiIdentify(bool bV2)
{
	int r1 = -1;
	bool mmc = false;

	for (uint32_t i = 0; i < SDCARD_INIT_TRY; i++)
	{
		r1 = SpiOneCmd(SD_CMD_APP, 0);
		if (r1 == 0 || r1 == (int)SD_SPI_R1_IDLE)
		{
			r1 = SpiOneCmd(SD_ACMD_SEND_OP_COND, bV2 ? SD_OCR_HCS : 0U);
		}
		if (r1 == 0)
		{
			break;
		}
		if (r1 != (int)SD_SPI_R1_IDLE)
		{
			// An MMC does not know CMD55 and ACMD41
			if (bV2 || r1 < 0 || (r1 & SD_SPI_R1_ILLEGAL) == 0)
			{
				return false;
			}
			mmc = true;
			break;
		}
		usDelay(SDCARD_INIT_DELAY_US);
	}

	for (uint32_t i = 0; mmc && i < SDCARD_INIT_TRY; i++)
	{
		r1 = SpiOneCmd(SD_CMD_SEND_OP_COND, 0);
		if (r1 != (int)SD_SPI_R1_IDLE)
		{
			break;
		}
		usDelay(SDCARD_INIT_DELAY_US);
	}
	if (r1 != 0)
	{
		return false;
	}

	SDCARD_TYPE type = mmc ? SDCARD_TYPE_MMC : bV2 ? SDCARD_TYPE_SDSC : SDCARD_TYPE_SDSC_V1;

	if (bV2)
	{
		uint8_t ocr[4];

		if (SpiOneCmd(SD_CMD_READ_OCR, 0, ocr, 4) != 0)
		{
			return false;
		}
		vDev.Ocr = SDCardBe32(ocr);
		if (vDev.Ocr & SD_OCR_CCS)
		{
			type = SDCARD_TYPE_SDHC;
		}
	}

	// CSD and CID, each a data block of 16 bytes
	for (int i = 0; i < 2; i++)
	{
		bool ok = SpiBegin();

		if (ok)
		{
			ok = SpiCmd(i == 0 ? SD_CMD_SEND_CSD : SD_CMD_SEND_CID, 0) == 0 &&
				 SpiReadBlock(i == 0 ? vDev.Csd : vDev.Cid, 16);
			SpiEnd();
		}
		if (ok == false)
		{
			return false;
		}
	}

	if (type != SDCARD_TYPE_SDHC && SpiOneCmd(SD_CMD_SET_BLOCKLEN, SDCARD_SECT_SIZE) != 0)
	{
		return false;
	}
	vDev.Type = type;

	return true;
}

int SDCard::SpiRead(uint32_t SectNo, uint8_t *pBuff, uint32_t NbSect)
{
	uint32_t done = 0;

	if (SpiBegin() == false)
	{
		return 0;
	}
	if (SpiCmd(NbSect > 1 ? SD_CMD_READ_MULTIPLE : SD_CMD_READ_SINGLE, CardAddr(SectNo)) == 0)
	{
		while (done < NbSect && SpiReadBlock(pBuff + done * SDCARD_SECT_SIZE, SDCARD_SECT_SIZE))
		{
			done++;
		}
		if (NbSect > 1)
		{
			// Stop, the card busy after its response. A read to the last
			// sector can give an out of range error (parameter or address
			// error) in the response, not to be taken as one.
			int r1 = SpiCmd(SD_CMD_STOP, 0);

			if (r1 < 0 || (r1 & ~SD_SPI_R1_RANGE) != 0 || SpiWait(0x00, false, SDCARD_WRITE_MS) == false)
			{
				done = 0;
			}
		}
	}
	SpiEnd();

	if (done < NbSect)
	{
		(void)Status();
	}

	return (int)done;
}

int SDCard::SpiWrite(uint32_t SectNo, const uint8_t *pData, uint32_t NbSect)
{
	uint32_t done = 0;
	bool multi = NbSect > 1;

	if (SpiBegin() == false)
	{
		return 0;
	}
	if (SpiCmd(multi ? SD_CMD_WRITE_MULTIPLE : SD_CMD_WRITE_SINGLE, CardAddr(SectNo)) == 0)
	{
		while (done < NbSect &&
			   SpiWriteBlock(multi ? SD_SPI_TOKEN_MULTI : SD_SPI_TOKEN_SINGLE, pData + done * SDCARD_SECT_SIZE))
		{
			done++;
		}
		if (multi)
		{
			// Stop token, a byte, then busy while the card programs
			uint8_t stop[2] = { SD_SPI_TOKEN_STOP, 0xFF };

			if (SpiTx(stop, 2) == false || SpiWait(0x00, false, SDCARD_WRITE_MS) == false)
			{
				done = 0;
			}
		}
	}
	SpiEnd();

	// The errors of the programming
	uint32_t st = Status();

	if (st == 0xFFFFFFFFU || (st & SD_SPI_R2_ERRORS))
	{
		done = 0;
	}

	return (int)done;
}
