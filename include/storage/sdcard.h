/**-------------------------------------------------------------------------
@file	sdcard.h

@brief	SD card driver

SD memory card (SDSC, SDHC, SDXC) as a DiskIO of 512 byte sectors, on the
interface given to Init.

On an SPI interface (DEVINTRF_TYPE_SPI) the card is in SPI mode, the selector
is the chip select. The SPI configuration sends 0xFF while it reads
(DummyByte), keeps the chip select low from StartTx to the stop, and its
transfers end in the call. MMC cards are taken in this mode.

On an SD/MMC host (DEVINTRF_TYPE_SDMMC, coredev/sdmmc.h) the card is on the SD
bus, 1 or 4 data lines, the selector is the slot of the host.

Init identifies the card at 400 kHz or less, then sets the clock of the
interface to the rate it had before, or to the card maximum (TRAN_SPEED) when
lower. Set that rate on the interface before Init. An interface that gives a
clock above the one asked is asked again lower. ReadSect and WriteSect move several
sectors with one multiple block command.

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
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include <stdint.h>

#include "device_intrf.h"
#include "diskio.h"

#define SDCARD_SECT_SIZE		512
#define SDCARD_CACHE_MAX		2

/// Card found by Init
typedef enum __SDCard_Type {
	SDCARD_TYPE_NONE,			//!< No card set up
	SDCARD_TYPE_MMC,			//!< MultiMediaCard, SPI mode
	SDCARD_TYPE_SDSC_V1,		//!< SD standard capacity, physical layer version 1
	SDCARD_TYPE_SDSC,			//!< SD standard capacity, version 2 or later
	SDCARD_TYPE_SDHC,			//!< SD high or extended capacity (SDHC, SDXC)
} SDCARD_TYPE;

#pragma pack(push, 4)

/// Card data
typedef struct __SDCard_Dev_Data {
	SDCARD_TYPE Type;			//!< Card type
	uint32_t Ocr;				//!< Operation conditions register
	uint32_t Rca;				//!< Relative card address in bits 31:16, SD bus
	uint32_t NbSect;			//!< Number of 512 byte sectors
	uint32_t Rate;				//!< Clock of the transfers, Hz
	uint32_t Status;			//!< Card status: R1 (SD bus), R2 R1 byte << 8 | second byte (SPI mode)
	uint8_t Cid[16];			//!< CID register, most significant byte first
	uint8_t Csd[16];			//!< CSD register, most significant byte first
} SDCardDev_t;

#pragma pack(pop)

#ifdef __cplusplus

class SDCard : public DiskIO {
public:
	SDCard();
	virtual ~SDCard() {}
	SDCard(SDCard&) = delete;

	/**
	 * @brief	Identify the card and set it up for the transfers.
	 *
	 * @param	pDevInterf		: SPI or SD/MMC host interface, initialized
	 * @param	pCacheMem		: Sector cache memory, SDCARD_SECT_SIZE bytes per sector
	 * @param	CacheMemSize	: Its size, SDCARD_CACHE_MAX sectors at most are used
	 *
	 * @return	true when a card is ready
	 */
	bool Init(DeviceIntrf * const pDevInterf, uint8_t * const pCacheMem, int CacheMemSize);
	bool Init(DeviceIntrf * const pDevInterf, DiskIOCache_t * const pCacheBlk = nullptr, int NbCacheBlk = 0);

	/// Chip select (SPI) or slot (SD/MMC host) of the card, set before Init. 0 by default.
	void DeviceAddress(uint32_t Addr) { vDevAddr = Addr; }
	uint32_t DeviceAddress(void) const { return vDevAddr; }

	uint16_t GetSectSize(void) { return SDCARD_SECT_SIZE; }
	uint32_t GetNbSect(void) { return vDev.NbSect; }
	/// Size in KBytes
	uint32_t GetSize(void) { return vDev.NbSect / 2U; }

	bool SectRead(uint32_t SectNo, uint8_t *pData) { return ReadSect(SectNo, pData, 1) == 1; }
	bool SectWrite(uint32_t SectNo, uint8_t *pData) { return WriteSect(SectNo, pData, 1) == 1; }

	/**
	 * @brief	Read NbSect sectors, one multiple block command for more than one.
	 *
	 * @return	Number of sectors read, 0 on failure
	 */
	int ReadSect(uint32_t SectNo, uint8_t *pBuff, uint32_t NbSect);

	/**
	 * @brief	Write NbSect sectors, one multiple block command for more than one.
	 * 			Returns once the card has programmed them.
	 *
	 * @return	Number of sectors written, 0 on failure
	 */
	int WriteSect(uint32_t SectNo, const uint8_t *pData, uint32_t NbSect);

	/// One sector: Addr the sector number, Len SDCARD_SECT_SIZE. Bytes moved.
	int ReadSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	int WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len);

	/**
	 * @brief	Card status (CMD13), also kept in the card data.
	 *
	 * @return	Status: R1 on the SD bus, R1 << 8 | second byte of R2 in SPI
	 * 			mode. 0xFFFFFFFF when the card does not answer.
	 */
	uint32_t Status(void);

	SDCARD_TYPE Type(void) const { return vDev.Type; }
	/// Clock of the transfers, Hz
	uint32_t Rate(void) const { return vDev.Rate; }
	const uint8_t *Cid(void) const { return vDev.Cid; }
	const uint8_t *Csd(void) const { return vDev.Csd; }

	operator SDCardDev_t const *() { return &vDev; };

private:
	bool Identify(void);
	uint32_t SetupCsd(void);
	uint32_t RateNotAbove(uint32_t Max);
	bool Ready(void) const { return vDev.Type != SDCARD_TYPE_NONE; }
	uint32_t CardAddr(uint32_t SectNo) const;

	// SD bus
	bool BusCmd(uint8_t Cmd, uint32_t Arg, uint8_t Flags, uint8_t *pResp = nullptr);
	bool BusAppCmd(uint8_t Cmd, uint32_t Arg, uint8_t Flags, uint8_t *pResp = nullptr);
	bool BusIdentify(bool bV2);
	int BusRead(uint32_t SectNo, uint8_t *pBuff, uint32_t NbSect);
	int BusWrite(uint32_t SectNo, const uint8_t *pData, uint32_t NbSect);

	// SPI mode
	bool SpiBegin(void);
	void SpiEnd(void);
	bool SpiTx(const uint8_t *pData, int Len);
	bool SpiRx(uint8_t *pBuff, int Len);
	bool SpiWait(uint8_t Val, bool bEqual, uint32_t Ms, uint8_t *pLast = nullptr);
	int SpiCmd(uint8_t Cmd, uint32_t Arg, uint8_t *pResp = nullptr, int Len = 0);
	int SpiOneCmd(uint8_t Cmd, uint32_t Arg, uint8_t *pResp = nullptr, int Len = 0);
	bool SpiReadBlock(uint8_t *pBuff, int Len);
	bool SpiWriteBlock(uint8_t Token, const uint8_t *pData);
	bool SpiIdentify(bool bV2);
	int SpiRead(uint32_t SectNo, uint8_t *pBuff, uint32_t NbSect);
	int SpiWrite(uint32_t SectNo, const uint8_t *pData, uint32_t NbSect);

	DeviceIntrf *vpInterf;
	uint32_t vDevAddr;
	bool vbSdBus;				//!< SD/MMC host, else SPI mode
	bool vbTxPhase;				//!< SPI transaction in its transmit phase
	SDCardDev_t vDev;
	DiskIOCache_t vCacheDesc[SDCARD_CACHE_MAX];
};

#endif	// __cplusplus

#endif	// __SDCARD_H__
