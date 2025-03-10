/**-------------------------------------------------------------------------
@file	device_intrf.h

@brief	Generic data transfer interface class

This class is used to implement device communication interfaces such as I2C, UART, etc...
Not limited to wired or physical interface.  It could be soft interface as well such
as SLIP protocol or any mean of transferring data between 2 entities.

@author	Hoang Nguyen Hoan
@date	Nov. 25, 2011

@license

MIT License

Copyright (c) 2011 I-SYST inc. All rights reserved.

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

#include "istddef.h"
#include "device_intrf.h"

// NOTE : For thread safe use
//
// DeviceIntrfStartRx
// DeviceIntrfStopRx
// DeviceIntrfStartTx
// DeviceIntrfStopTx
//
int DeviceIntrfRx(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == NULL || BuffLen <= 0)
		return 0;

	int count = 0;
	int nrtry = pDev->MaxRetry;

	do {
		if (DeviceIntrfStartRx(pDev, DevAddr))
		{
			pDev->bNoStop = false;
			count = pDev->RxData(pDev, pBuff, BuffLen);

			if (count < 0)
			{
        		break;
			}
			DeviceIntrfStopRx(pDev);

		}
	} while(count == 0 && nrtry-- > 0);

	return count;
}

void DeviceIntrfTxComplete(DevIntrf_t * const pDev)
{
	atomic_store(&pDev->bTxReady, true);

	if (pDev->bNoStop == false)
	{
		DeviceIntrfStopTx(pDev);
	}
}

int DeviceIntrfTx(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == NULL || BuffLen <= 0)
		return 0;

	int count = 0;
	int nrtry = pDev->MaxRetry;

	do {
		if (DeviceIntrfStartTx(pDev, DevAddr))
		{
			pDev->bNoStop = false;
			count = pDev->TxData(pDev, pBuff, BuffLen);
        	if (count < 0)
			{
        		break;
			}
			DeviceIntrfStopTx(pDev);
		}
	} while (count == 0 && nrtry-- > 0);

	return count;
}


bool DeviceIntrfWaitTxComplete(DevIntrf_t * const pDev, int Timeout)
{
	while (pDev->bTxReady == false && --Timeout > 0);

	return Timeout > 0;
}

int DeviceIntrfRead(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                 uint8_t *pRxBuff, int RxLen)
{
    int count = 0;
    int nrtry = pDev->MaxRetry;

    if (pRxBuff == NULL || RxLen <= 0)
        return 0;

    do {
        if (DeviceIntrfStartTx(pDev, DevAddr))
        {
            if (pAdCmd)
            {
            	pDev->bNoStop = true;
            	if (pDev->TxSrData)
            	{
            		count = pDev->TxSrData(pDev, pAdCmd, AdCmdLen);
            	}
            	else
            	{
            		count = pDev->TxData(pDev, pAdCmd, AdCmdLen);
            	}

            	if (count < 0)
            	{
            		DeviceIntrfWaitTxComplete(pDev, 1000000);
            	}
				pDev->bNoStop = false;
            }
           // if (pDev->TxSrData)
            {
            	// Note : this is restart condition in read mode,
            	// must not generate any stop condition here
            	pDev->StartRx(pDev, DevAddr);
            }
           	count = pDev->RxData(pDev, pRxBuff, RxLen);

        	if (count < 0)
        	{
        		break;
        	}
			DeviceIntrfStopRx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    return count;
}

int DeviceIntrfWrite(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                  uint8_t *pData, int DataLen)
{
    int count = 0, txlen = AdCmdLen;
    int nrtry = pDev->MaxRetry;

    if (pAdCmd == NULL || (AdCmdLen + DataLen) <= 0)
        return 0;

#if defined(WIN32) || defined(__ICCARM__)
	uint8_t d[100];
#else
	uint8_t d[AdCmdLen + DataLen];
#endif

	// NOTE : Some I2C devices that uses DMA transfer may require that the tx to be combined
    // into single tx. Because it may generate a end condition at the end of the DMA
    memcpy(d, pAdCmd, AdCmdLen);
    if (pData != NULL && DataLen > 0)
    {
    	int l = min(DataLen, (int)(sizeof(d) - AdCmdLen));
    	memcpy(&d[AdCmdLen], pData, l);
    	txlen += l;
    }

    do {
        if (DeviceIntrfStartTx(pDev, DevAddr))
        {
    		pDev->bNoStop = false;
            count = pDev->TxData(pDev, d, txlen);
        	if (count < 0)
        	{
        		break;
        	}
			DeviceIntrfStopTx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    if (count >= AdCmdLen)
        count -= AdCmdLen;
    else
        count = 0;

    return count;
}

