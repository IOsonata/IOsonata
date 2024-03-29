/**--------------------------------------------------------------------------
@file	lpc_iap.h

@brief	Implementation of NXP LPC IAP (In-Application Programming)

@author	Hoang Nguyen Hoan
@date	Jan. 26, 2015

@license

Copyright (c) 2015-2020, I-SYST inc, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __LPC_IAP_H__
#define __LPC_IAP_H__

#include <stdint.h>

// **** EEPROM data must be 16 bits aligned for both source and destination

typedef enum {
	IAPCMD_READ_FACTORY_SETTINGS	= 40,	// Read factory settings
	IAPCMD_PREP_SECTOR 				= 50,	// Prepare sector(s) for write operation
	IAPCMD_COPY_RAM_TO_FLASH		= 51,	// Copy RAM to flash
	IAPCMD_ERASE_SECT				= 52,	// Erase sector(s)
	IAPCMD_BLANK_CHECK 				= 53,	// Blank check sector(s)
	IAPCMD_READ_PARTID				= 54,	// Read Part ID
	IAPCMD_READ_BOOTCODE_VER		= 55,	// Read Boot code version
	IAPCMD_COMPARE					= 56,	// Compare
	IAPCMD_INVOKE_ISP				= 57,	// Reinvoke ISP
	IAPCMD_READ_UID					= 58,	// Read UID
	IAPCMD_ERASE_PAGE				= 59,	// Erase page
	IAPCMD_EEPROM_WRITE				= 61,	// EEPROM Write
	IAPCMD_EEPROM_READ				= 62,	// EEPROM Read
	IAPCMD_READ_FLASH_SIG			= 70,	// Read flash signature
	IAPCMD_READ_FLASH_EXTSIG		= 73,	// Read extended flash signature
	IAPCMD_EEPROM_READ_PAGE			= 80,	// EEPROM Read page
	IAPCMD_EEPROM_WRITE_PAGE		= 81,	// EEPROM Write page
} IAPCMD;

typedef enum {
	IAPSTATUS_SUCCESS,					// Success
	IAPSTATUS_INVALID_COMMAND,			// Invalid command
	IAPSTATUS_SRC_ADDR_ERROR,			// Source address is not on a word boundary.
	IAPSTATUS_DST_ADDR_ERROR,			// Destination address is not on a correct boundary.
	IAPSTATUS_SRC_ADDR_NOT_MAPPED,		// Source address is not mapped in the memory map.
	IAPSTATUS_DST_ADDR_NOT_MAPPED,		// Destination address is not mapped in the memory map.
	IAPSTATUS_COUNT_ERROR,				// Byte count is not multiple of 4 or is not a permitted value.
	IAPSTATUS_INVALID_SECT,				// Sector number is invalid.
	IAPSTATUS_SECTOR_NOT_BLANK,			// Sector is not blank.
	IAPSTATUS_SECTOR_NOT_PREP,			// Command to prepare sector for write operation was not executed.
	IAPSTATUS_COMPARE_ERROR,			// Source and destination data is not same.
	IAPSTATUS_BUSY,						// flash programming hardware interface is busy
	IAPSTATUS_PARAM_ERROR,				// Insufficient number of parameters or invalid parameter
	IAPSTATUS_ADDR_ERROR,				// Address is not on word boundary
	IAPSTATUS_ADDR_NOT_MAPPED,			// Address is not mapped in the memory map
	IAPSTATUS_NO_POWER,					// Flash memory block is powered down
	IAPSTATUS_NO_CLOCK,					// Flash memory block or controller is not clocked
	IAPSTATUS_ISP_CONFIG_ERROR			// Reinvoke configuration error
} IAPSTATUS;

#pragma pack(push, 4)

typedef struct {
	uint32_t CmdStatus;			// Command code or Status
	uint32_t Data[4];
} IAP_CMD_STATUS;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

IAPSTATUS IAPReadFactorySettings(uint32_t RegAddr, uint32_t *pBuff);
IAPSTATUS IAPPrepSectWrite(int StartSect, int EndSect);
IAPSTATUS IAPCopyRamToFlash(uint8_t *pDst, uint8_t *pSrc, int Size);
IAPSTATUS IAPEraseSector(int StartSect, int EndSect);
IAPSTATUS IAPBlankCheck(int StartSect, int EndSect, IAP_CMD_STATUS *pRes);
IAPSTATUS IAPReadPartId(IAP_CMD_STATUS *pRes);
IAPSTATUS IAPReadBootCodeVers(IAP_CMD_STATUS *pRes);
IAPSTATUS IAPCompare(uint8_t *pDst, uint8_t *pSrc, int Size, IAP_CMD_STATUS *pRes);
IAPSTATUS IAPJumpISP();
IAPSTATUS IAPReadUID(IAP_CMD_STATUS *pRes);
IAPSTATUS IAPErasePage(int StartPage, int EndPage);
IAPSTATUS IAPEepromWrite(uint32_t EepAddr, uint8_t *pData, int Size);
IAPSTATUS IAPEepromRead(uint32_t EepAddr, uint8_t *pData, int Size);
IAPSTATUS IAPReadFlashSignature(uint32_t *pSig);
IAPSTATUS IAPReadExtFlashSignature(uint32_t StartPage, uint32_t EndPage, uint32_t NbState, uint32_t *pSig);
IAPSTATUS IAPEepromWritePage(uint32_t PageNo, uint8_t *pData);
//IAPSTATUS IAPEepromRead(uint32_t PageNo, uint8_t *pBuff);


#ifdef __cplusplus
}
#endif

#endif // __LPC_IAP_H__
