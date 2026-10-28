/**-------------------------------------------------------------------------
@file	sdmmc.h

@brief	SD/MMC host interface, SD bus

The host of the SD bus is a DeviceIntrf of type DEVINTRF_TYPE_SDMMC. The card
driver (SDCard, storage/sdcard.h) runs one card command per transfer, opened
by StartTx: the command frame by TxData, then the response or the data blocks
of the command.

A command without data: Tx(Slot, Frame, SDMMC_FRAME_SIZE) when the response
is not needed, Read(Slot, Frame, SDMMC_FRAME_SIZE, pResp, Len) to get it.
Data from the card: Read(Slot, Frame, SDMMC_FRAME_SIZE, pData, Len). Data to
the card: StartTx(Slot), TxData(Frame, SDMMC_FRAME_SIZE), TxData(pData, Len),
StopTx, with bNoStop set between the frame and the data. Frame and data in one
TxData are taken the same way. The transfer returns once the card has taken
the data and released its busy signal.

The block size is Len when Len is less than 512, else 512 and Len is a
multiple of 512. A command that fails moves nothing: RxData and the TxData
of the data return 0. A data command also fails when its R1 response, or the
response of the stop command at its end, has an error bit (SDMMC_R1_ERRORS):
the host ends the data with a stop command. The selector of StartTx and
StartRx is the card slot of the host, 0 for the first.

Rate sets the card clock: the highest the host makes at or below the rate
asked, the slowest when none is. The data bus is 1 bit after Init and after a
command with SDMMC_FRAME_INIT, 4 bits after a command with
SDMMC_FRAME_BUS_WIDTH and the argument 2 (ACMD6).

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __SDMMC_H__
#define __SDMMC_H__

#include <stdint.h>
#include <string.h>

#include "coredev/iopincfg.h"
#include "device_intrf.h"

/// Command frame sent by TxData after StartTx. Bytes 0 to 5 are the command
/// token of the bus, the same as in SPI mode.
#define SDMMC_FRAME_SIZE			7
#define SDMMC_FRAME_CMD_IDX			0			//!< 0x40 | command index
#define SDMMC_FRAME_ARG_IDX			1			//!< Argument, 4 bytes, most significant first
#define SDMMC_FRAME_CRC_IDX			5			//!< CRC7 << 1 | 1, the host makes its own
#define SDMMC_FRAME_FLAGS_IDX		6			//!< SDMMC_FRAME_ flags

/// Response of the command
#define SDMMC_FRAME_RESP_MASK		(7U << 0)
#define SDMMC_FRAME_RESP_NONE		(0U << 0)	//!< No response
#define SDMMC_FRAME_RESP_R1			(1U << 0)	//!< 48 bits with CRC: R1, R6, R7
#define SDMMC_FRAME_RESP_R1B		(2U << 0)	//!< R1, then the card busy until it releases data line 0
#define SDMMC_FRAME_RESP_R2			(3U << 0)	//!< 136 bits: CID or CSD
#define SDMMC_FRAME_RESP_R3			(4U << 0)	//!< 48 bits without CRC: OCR

/// Data of the command
#define SDMMC_FRAME_DATA_MASK		(3U << 3)
#define SDMMC_FRAME_DATA_NONE		(0U << 3)	//!< No data
#define SDMMC_FRAME_DATA_READ		(1U << 3)	//!< Blocks from the card, by RxData
#define SDMMC_FRAME_DATA_WRITE		(2U << 3)	//!< Blocks to the card, by TxData

#define SDMMC_FRAME_INIT			(1U << 5)	//!< Initialization clocks (74 or more) before the command, 1 bit bus
#define SDMMC_FRAME_STOP			(1U << 6)	//!< Stop transmission (CMD12) at the end of the data
#define SDMMC_FRAME_BUS_WIDTH		(1U << 7)	//!< Data bus width set by the command: argument 2 4 bits, 0 1 bit

/// Response bytes, most significant first: 4 for a 48 bit response (bits
/// 39:8: card status, OCR, RCA, interface condition), 16 for R2 (bits 127:0:
/// the CID or CSD register with its CRC7)
#define SDMMC_RESP_SHORT_LEN		4
#define SDMMC_RESP_LONG_LEN			16

/// Error bits of the card status (R1): OUT_OF_RANGE to ERROR (bits 31 to 19
/// but 25), CSD_OVERWRITE, WP_ERASE_SKIP, AKE_SEQ_ERROR
#define SDMMC_R1_ERRORS				0xFDF98008U
/// Command taken as an application command (CMD55 answered)
#define SDMMC_R1_APP_CMD			(1U << 5)

#define SDMMC_BLOCK_SIZE			512

/// Pin map indexes. The pins of the target (card detect, supply switch) follow
/// the data lines.
#define SDMMC_CLK_IOPIN_IDX			0
#define SDMMC_CMD_IOPIN_IDX			1
#define SDMMC_D0_IOPIN_IDX			2			//!< D1 to D3 follow for a 4 bit bus

#pragma pack(push, 4)

/// Host configuration
typedef struct __Sdmmc_Config {
	int DevNo;						//!< Host instance, 0 based
	const IOPinCfg_t *pIOPinMap;	//!< CLK, CMD, D0, D1 to D3 for a 4 bit bus, then the pins of the target
	int NbIOPins;					//!< Number of pins in the map
	uint32_t Rate;					//!< Highest card clock, Hz, 0 for 25 MHz (default speed)
	int BusWidth;					//!< Data lines wired to the card, 1 or 4
	bool bCardDetect;				//!< Card detect input wired
	bool bPowerCtrl;				//!< Card supply switched by the host
	bool bDmaEn;					//!< DMA transfers
	bool bIntEn;					//!< Interrupt transfers
	int IntPrio;					//!< Interrupt priority
	DevIntrfEvtHandler_t EvtCB;		//!< Event callback
} SdmmcCfg_t;

/// Host data of the low level functions
typedef struct __Sdmmc_Device {
	SdmmcCfg_t Cfg;					//!< Configuration
	DevIntrf_t DevIntrf;			//!< Device interface implementation
	int BusWidth;					//!< Data lines in use, 1 or 4
	int CurSlot;					//!< Card slot of the transfer
	uint8_t Frame[SDMMC_FRAME_SIZE];//!< Command frame of the transfer
	int FrameLen;					//!< Frame bytes taken
	bool bSent;						//!< Command of the frame sent
	int RespLen;					//!< Response bytes, 0 when none or failed
	uint8_t Resp[SDMMC_RESP_LONG_LEN];	//!< Response, most significant byte first
} SdmmcDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Set up the host: pins, clocks, card supply on, card clock at the
 * 			configured rate, 1 bit bus. Target implementation.
 *
 * @param	pDev	: Host data
 * @param	pCfg	: Configuration
 *
 * @return	true on success
 */
bool SdmmcInit(SdmmcDev_t * const pDev, const SdmmcCfg_t *pCfg);

static inline uint32_t SdmmcGetRate(SdmmcDev_t * const pDev) { return pDev->DevIntrf.GetRate(&pDev->DevIntrf); }
static inline uint32_t SdmmcSetRate(SdmmcDev_t * const pDev, uint32_t Rate) {
	return pDev->DevIntrf.SetRate(&pDev->DevIntrf, Rate);
}
static inline void SdmmcEnable(SdmmcDev_t * const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void SdmmcDisable(SdmmcDev_t * const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }
static inline void SdmmcPowerOff(SdmmcDev_t * const pDev) { DeviceIntrfPowerOff(&pDev->DevIntrf); }
static inline SdmmcDev_t *SdmmcGetHandle(DevIntrf_t * const pDevIntrf) {
	return (SdmmcDev_t *)DeviceIntrfGetHandle(pDevIntrf);
}

#ifdef __cplusplus
}

/// C++ SD/MMC host class
class Sdmmc : public DeviceIntrf {
public:
	Sdmmc() { memset((void*)&vDevData, 0, sizeof(vDevData)); }
	virtual ~Sdmmc() {}
	Sdmmc(Sdmmc&) = delete;

	bool Init(const SdmmcCfg_t &Cfg) { return SdmmcInit(&vDevData, &Cfg); }

	operator DevIntrf_t * () { return &vDevData.DevIntrf; }
	operator SdmmcDev_t * () { return &vDevData; }

	uint32_t Rate(uint32_t RateHz) { return SdmmcSetRate(&vDevData, RateHz); }
	uint32_t Rate(void) { return SdmmcGetRate(&vDevData); }

	/// Data lines in use
	int BusWidth(void) const { return vDevData.BusWidth; }

private:
	SdmmcDev_t vDevData;
};

#endif	// __cplusplus

#endif	// __SDMMC_H__
