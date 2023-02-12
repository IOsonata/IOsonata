/**-------------------------------------------------------------------------
@file	sdcard.h

@brief	SD card driver


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

#define SDCARD_CACHE_MAX		2

#pragma pack(push, 4)

// CSD register
typedef struct __SDCard_CSD {
	uint32_t Vers:2;
	uint32_t Rsvd1:6;
	uint32_t Taac:8;
	uint32_t Nsac:8;
} SDCardCSD_t;

typedef struct __SDCard_Config {
	int Rate;
	DevIntrf_t *pSerIntrf;
} SDCardCfg_t;

typedef struct __SDCard_Dev_Data {
	int SectSize;
	int TotalSect;
	SDCardCSD_t Csd;
	DevIntrf_t *pIntrf;
} SDCardDev_t;

#pragma pack(pop)

// Define SD device handle.  This is a pointer to SDCard class for C
// function wrapper for use with C application code.
// Internal implementation will cast this to proper class pointer
//typedef void*		HSDDEV;


#ifdef __cplusplus

#include <memory>

class SDCard : public DiskIO {
public:
	SDCard();
	virtual ~SDCard();

	virtual bool Init(DeviceIntrf * const pDevInterf, uint8_t * const pCacheMem = NULL, int CacheMemSize = 0);
	virtual bool Init(DeviceIntrf * const pDevInterf, DiskIOCache_t * const pCacheBlk = NULL, int NbCacheBlk = 0);
	int Cmd(uint8_t Cmd, uint32_t param);
	int GetResponse(uint8_t *pBuff, int BuffLen);
	int ReadData(uint8_t *pBuff, int BuffLen);
	int WriteData(uint8_t *pData, int Len);
	uint16_t GetSectSize(void);
	uint32_t GetNbSect(void);
	// @return size in KB
	uint32_t GetSize(void);
	int ReadSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	int WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	bool SectRead(uint32_t SectNo, uint8_t *pData) {
		return ReadSingleBlock(SectNo, pData, vDev.SectSize) == vDev.SectSize;
	}
	bool SectWrite(uint32_t SectNo, uint8_t *pData) {
		return WriteSingleBlock(SectNo, pData, vDev.SectSize) == vDev.SectSize;
	}
	operator SDCardDev_t const *() { return &vDev; };

protected:
private:
	//std::shared_ptr<SerialIntrf> vpInterf;
	DeviceIntrf *vpInterf;
	SDCardDev_t vDev;
	int NbCacheBlk;
	DiskIOCache_t vCacheDesc[SDCARD_CACHE_MAX];
};

extern "C" {
#endif

bool SDInit(SDCardDev_t * const pDev, const SDCardCfg_t *pCfg);
int SDCmd(SDCardDev_t * const pDev, uint8_t Cmd, uint32_t param);
int SDGetResponse(SDCardDev_t * const pDev, uint8_t *pData, int len);
int SDReadData(SDCardDev_t * const pDev, uint8_t *pBuff, int BuffLen);
int SDWriteData(SDCardDev_t * const pDev, uint8_t *pData, int DataLen);
int SDReadSingleBlock(SDCardDev_t * const pDev, uint32_t Addr, uint8_t *pData, int len);
int SDWriteSingleBlock(SDCardDev_t * const pDev, uint32_t Addr, uint8_t *pData, int len);
uint32_t SDGetSize(SDCardDev_t * const pDev);

#ifdef __cplusplus
}
#endif

#endif	// __SDCARD_H__

