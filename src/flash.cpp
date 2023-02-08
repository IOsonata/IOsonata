/**-------------------------------------------------------------------------
@file	flash.cpp

@brief	Generic flash driver class


@author	Hoang Nguyen Hoan
@date	Aug. 30, 2016


@license

MIT License

Copyright (c) 2016 I-SYST inc. All rights reserved.

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

#include "istddef.h"
#include "idelay.h"
#include "coredev/spi.h"
#include "flash.h"

bool FlashInit(FlashDev_t * const pDev, const FlashCfg_t *pCfg, DevIntrf_t * const pDevIntrf)
{
    pDev->pDevIntrf = NULL;
    pDev->pWaitCB = NULL;

    if (pDevIntrf == NULL)
        return false;

    pDev->pDevIntrf	= pDevIntrf;

    if (pCfg->pInitCB)
    {
        if (pCfg->pInitCB(pCfg->DevNo, pDevIntrf) == false)
            return false;
    }

    if (pCfg->pWaitCB)
    	pDev->pWaitCB = pCfg->pWaitCB;

    pDev->DevNo		= pCfg->DevNo;
    pDev->SectSize	= pCfg->SectSize;
    pDev->BlkSize	= pCfg->BlkSize;
    pDev->PageSize	= pCfg->PageSize;
    pDev->TotalSize	= pCfg->TotalSize;
    pDev->AddrSize	= pCfg->AddrSize;
    pDev->RdCmd		= pCfg->RdCmd;
	pDev->WrCmd		= pCfg->WrCmd;

    if (pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDevIntrf);
    	QuadSPISetMemSize(dev, pDev->TotalSize);
    }

    FlashReset(pDev);

    if (pCfg->DevIdSize > 0 && pCfg->DevId != 0)
    {
    	int rtry = 5;

    	do {
    		uint32_t d = FlashReadId(pDev, pCfg->DevIdSize);
    		if (d == pCfg->DevId)
    		{
    			break;
    		}
    	} while (rtry-- > 0);

    	if (rtry <= 0)
    	{
    		return false;
    	}
    }

    if (pDev->AddrSize > 3)
    {
    	// Enable 4 bytes address
        uint8_t cmd = FLASH_CMD_EN4B;

        if (pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
        {
        	SPIDev_t *dev = SPIGetHandle(pDevIntrf);

			SPIStartRx(dev, pDev->DevNo);
			QuadSPISendCmd(dev, cmd, -1, 0, 0, 0);
			SPIStopRx(dev);
        }
        else
        {
            DeviceIntrfTx(pDevIntrf, pDev->DevNo, (uint8_t*)&cmd, 1);
        }
    }

	return true;
}

/**
 * @brief	Read Flash ID
 *
 * @param	Len : Length of id to read in bytes
 *
 * @return	Flash ID
 */
uint32_t FlashReadId(FlashDev_t * const pDev, int Len)
{
	uint32_t id = -1;

	if (Len > 0)
	{
		id = 0;

		if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
		{
	    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

	    	SPIStartRx(dev, pDev->DevNo);
			QuadSPISendCmd(dev, FLASH_CMD_READID, -1, 0, Len, 0);
			SPIRxData(dev, (uint8_t*)&id, Len);
			SPIStopRx(dev);
		}
		else
		{
			uint8_t cmd = FLASH_CMD_READID;
#if 1
			DeviceIntrfRead(pDev->pDevIntrf, pDev->DevNo, &cmd, 1, (uint8_t*)&id, Len);
#else
			DeviceIntrfStartRx(pDev->pDevIntrf, pDev->DevNo);
			DeviceIntrfTxData(pDev->pDevIntrf, &cmd, 1);
			DeviceIntrfRxData(pDev->pDevIntrf, (uint8_t*)&id, Len);
			DeviceIntrfStopRx(pDev->pDevIntrf, );
#endif
		}

	}

    return id;
}


uint8_t FlashReadStatus(FlashDev_t * const pDev)
{
    uint8_t d = 0;

	if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

		SPIStartRx(dev, pDev->DevNo);
    	QuadSPISendCmd(dev, FLASH_CMD_READSTATUS, -1, 0, 1, 0);
    	SPIRxData(dev, (uint8_t*)&d, 1);
    	SPIStopRx(dev);
    }
    else
    {
		d = FLASH_CMD_READSTATUS;
		DeviceIntrfStartRx(pDev->pDevIntrf, pDev->DevNo);
		DeviceIntrfTxData(pDev->pDevIntrf, &d, 1);
		DeviceIntrfRxData(pDev->pDevIntrf, &d, 1);
		DeviceIntrfStopRx(pDev->pDevIntrf);
    }

    return d;
}

bool FlashWaitReady(FlashDev_t * const pDev, uint32_t Timeout, uint32_t usRtyDelay)
{
    uint8_t d;

    do {
    	d = FlashReadStatus(pDev);

        if (!(d & FLASH_STATUS_WIP))
            return true;

        if (usRtyDelay > 0)
        {
            if (pDev->pWaitCB)
            	pDev->pWaitCB(pDev->DevNo, pDev->pDevIntrf);
            else
            	usDelay(usRtyDelay);
        }

    } while (Timeout-- > 0);

    return false;
}

void FlashWriteDisable(FlashDev_t * const pDev)
{
	if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

    	SPIStartTx(dev, pDev->DevNo);
    	QuadSPISendCmd(dev, FLASH_CMD_WRDISABLE, -1, 0, 0, 0);
    	SPIStopTx(dev);
    }
    else
    {
    	uint8_t d = FLASH_CMD_WRDISABLE;
    	DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, &d, 1);
    }
}

bool FlashWriteEnable(FlashDev_t * const pDev, uint32_t Timeout)
{
    uint8_t d;

    FlashWaitReady(pDev, Timeout, 0);

	if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

    	SPIStartTx(dev, pDev->DevNo);
    	QuadSPISendCmd(dev, FLASH_CMD_WRENABLE, -1, 0, 0, 0);
		SPIStopTx(dev);
    }
    else
    {
    	d = FLASH_CMD_WRENABLE;
    	DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, &d, 1);
    }
    return false;
}

void FlashErase(FlashDev_t * const pDev)
{
    uint8_t d;

    FlashWriteEnable(pDev, 10000);
    FlashWaitReady(pDev, 100000, 0);

	if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

		SPIStartTx(dev, pDev->DevNo);
    	QuadSPISendCmd(dev, FLASH_CMD_BULK_ERASE, -1, 0, 0, 0);
		SPIStopTx(dev);
    }
    else
    {
		d = FLASH_CMD_BULK_ERASE;

		DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, &d, 1);
    }
    // This is a long wait polling at every second only
    FlashWaitReady(pDev, -1, 1000000);
    FlashWriteDisable(pDev);
}

/**
 * Erase Flash block.
 *
 * @param   BlkNo   : Starting block number to erase.
 *          NbBlk   : Number of consecutive blocks to erase
 */
void FlashEraseBlock(FlashDev_t * const pDev, uint32_t BlkNo, int NbBlk)
{
    uint8_t d[8];
    uint32_t addr = BlkNo * pDev->BlkSize;
    uint8_t *p = (uint8_t*)&addr;

    d[0] = FLASH_CMD_BLOCK_ERASE;

    for (int k = 0; k < NbBlk; k++)
    {
        FlashWaitReady(pDev, -1, 100);

        // Need to re-enable write here, because some flash
        // devices may reset write enable after a write
        // complete
        FlashWriteEnable(pDev, 10000);

    	if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
        {
        	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

    		SPIStartTx(dev, pDev->DevNo);
    		QuadSPISendCmd(dev, FLASH_CMD_BLOCK_ERASE, addr, pDev->AddrSize, 0, 0);
    		SPIStopTx(dev);
        }
        else
        {
            for (int i = 1; i <= pDev->AddrSize; i++)
            {
                d[i] = p[pDev->AddrSize - i];
            }
        	DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, d, pDev->AddrSize + 1);
        }
        addr += pDev->BlkSize;
    }
    FlashWaitReady(pDev, -1, 100);
    FlashWriteDisable(pDev);
}

/**
 * Erase Flash sector.
 *
 * @param   SectNo   : Starting block number to erase.
 *          NbSect   : Number of consecutive blocks to erase
 */
void FlashEraseSector(FlashDev_t * const pDev, uint32_t SectNo, int NbSect)
{
    uint8_t d[8];
    uint32_t addr = SectNo * pDev->SectSize;// * 1024;
    uint8_t *p = (uint8_t*)&addr;

    if (pDev->AddrSize > 3)
    {
    	d[0] = FLASH_CMD_SECTOR_ERASE_4B;
    }
    else
    {
    	d[0] = FLASH_CMD_SECTOR_ERASE;
    }

    for (int k = 0; k < NbSect; k++)
    {
        FlashWaitReady(pDev, -1, 100);

        // Need to re-enable write here, because some flash
        // devices may reset write enable after a write
        // complete
        FlashWriteEnable(pDev, 10000);

    	if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
        {
        	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

    		SPIStartTx(dev, pDev->DevNo);
    		QuadSPISendCmd(dev, d[0], addr, pDev->AddrSize, 0, 0);
    		SPIStopTx(dev);
        }
        else
        {
            for (int i = 1; i <= pDev->AddrSize; i++)
            {
                d[i] = p[pDev->AddrSize - i];
            }
        	DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, d, pDev->AddrSize + 1);
        }
        addr += pDev->SectSize;// * 1024;
    }
    FlashWaitReady(pDev, -1, 100);
    FlashWriteDisable(pDev);
}

/**
 * Read one sector from physical device
 */
bool FlashSectRead(FlashDev_t * const pDev, uint32_t SectNo, uint8_t *pBuff)
{
   	uint8_t d[9];
    uint32_t addr = SectNo * pDev->SectSize;//DISKIO_SECT_SIZE;
    uint8_t *p = (uint8_t*)&addr;
    int cnt = pDev->SectSize;//DISKIO_SECT_SIZE;

    // Makesure there is no write access pending
    FlashWaitReady(pDev, 100000, 0);

    if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

		SPIStartRx(dev, pDev->DevNo);
    	QuadSPISendCmd(dev, pDev->RdCmd.Cmd , addr, pDev->AddrSize, pDev->SectSize/*DISKIO_SECT_SIZE*/, pDev->RdCmd.DummyCycle);
		SPIRxData(dev, pBuff, pDev->SectSize/*DISKIO_SECT_SIZE*/);
		SPIStopRx(dev);
    }
    else
    {
    	d[0] = FLASH_CMD_READ;
		while (cnt > 0)
		{
			for (int i = 1; i <= pDev->AddrSize; i++)
				d[i] = p[pDev->AddrSize - i];

			DeviceIntrfStartRx(pDev->pDevIntrf, pDev->DevNo);
			DeviceIntrfTxData(pDev->pDevIntrf, (uint8_t*)d, pDev->AddrSize + 1);
			int l = DeviceIntrfRxData(pDev->pDevIntrf, pBuff, pDev->SectSize/*DISKIO_SECT_SIZE*/);
			DeviceIntrfStopRx(pDev->pDevIntrf);
			if (l <= 0)
				return false;
			cnt -= l;
			addr += l;
			pBuff += l;
		}
    }

    return true;
}

/**
 * Write one sector to physical device
 */
bool FlashSectWrite(FlashDev_t * const pDev, uint32_t SectNo, uint8_t *pData)
{
    uint8_t d[9];
    uint32_t addr = SectNo * pDev->SectSize;//DISKIO_SECT_SIZE;
    uint8_t *p = (uint8_t*)&addr;

    int cnt = pDev->SectSize;//DISKIO_SECT_SIZE;

    if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

		while (cnt > 0)
		{
			int l = min(cnt, (int)pDev->PageSize);

			FlashWriteEnable(pDev, 10000);
			SPIStartTx(dev, pDev->DevNo);

			QuadSPISendCmd(dev, pDev->WrCmd.Cmd, addr, pDev->AddrSize, l, pDev->WrCmd.DummyCycle);

			l = SPITxData(dev, pData, l);
			SPIStopTx(dev);
			cnt -= l;
			pData += l;
			addr += l;
		}
    }
    else
    {
		d[0] = FLASH_CMD_WRITE;

		while (cnt > 0)
		{
			for (int i = 1; i <= pDev->AddrSize; i++)
				d[i] = p[pDev->AddrSize - i];

			int l = min(cnt, (int)pDev->PageSize);

			FlashWaitReady(pDev, 10000, 0);

			// Some Flash will reset write enable bit at completion
			// when page size is less than 512 bytes.
			// We need to set it again
			FlashWriteEnable(pDev, 10000);

			DeviceIntrfStartTx(pDev->pDevIntrf, pDev->DevNo);
			DeviceIntrfTxData(pDev->pDevIntrf, (uint8_t*)d, pDev->AddrSize + 1);
			l = DeviceIntrfTxData(pDev->pDevIntrf, pData, l);
			DeviceIntrfStopTx(pDev->pDevIntrf);
			if (l <= 0)
				return false;
			cnt -= l;
			pData += l;
			addr += l;
		}
    }

	FlashWriteDisable(pDev);

	return true;
}

/**
 * @brief	Reset flash to its default state
 */
void FlashReset(FlashDev_t * const pDev)
{
    if (pDev->pDevIntrf->Type == DEVINTRF_TYPE_QSPI)
    {
    	SPIDev_t *dev = SPIGetHandle(pDev->pDevIntrf);

    	SPIStartTx(dev, pDev->DevNo);
		QuadSPISendCmd(dev, FLASH_CMD_RESET_ENABLE, -1, 0, 0, 0);
		SPIStopTx(dev);
    	SPIStartTx(dev, pDev->DevNo);
		QuadSPISendCmd(dev, FLASH_CMD_RESET_DEVICE, -1, 0, 0, 0);
		SPIStopTx(dev);
    }
    else
    {
    	uint8_t d = FLASH_CMD_RESET_ENABLE;

    	DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, &d, 1);

    	d = FLASH_CMD_RESET_DEVICE;
    	DeviceIntrfTx(pDev->pDevIntrf, pDev->DevNo, &d, 1);
    }
}


