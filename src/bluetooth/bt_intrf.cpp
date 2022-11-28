/**-------------------------------------------------------------------------
@file	bt_intrf.cpp

@brief	Implementation allow the creation of generic serial interface of
a custom Bluetooth service with multiple user defined characteristics.


@author	Hoang Nguyen Hoan
@date	Feb. 6, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include "cfifo.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/bt_dev.h"
#include "interrupt.h"
//#include "bluetooth/ble_srvc.h"
#include "bluetooth/bt_gatt.h"

#define BTINTRF_PACKET_SIZE		(20)// + sizeof(BLEINTRF_PKT) - 1)
#define BTINTRF_CFIFO_SIZE		BTINTRF_CFIFO_TOTAL_MEMSIZE(2, BTINTRF_PACKET_SIZE)

alignas(4) static uint8_t s_BtDevIntrfRxFifoMem[BTINTRF_CFIFO_SIZE];
alignas(4) static uint8_t s_BtDevIntrfTxFifoMem[BTINTRF_CFIFO_SIZE];

/**
 * @brief - Disable
 * 		Turn off the interface.  If this is a physical interface, provide a
 * way to turn off for energy saving. Make sure the turn off procedure can
 * be turned back on without going through the full init sequence
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return None
 */
void BtIntrfDisable(DevIntrf_t *pDevIntrf)
{
	// TODO:
}

/**
 * @brief - Enable
 * 		Turn on the interface.
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return None
 */
void BtIntrfEnable(DevIntrf_t *pDevIntrf)
{
	// TODO:
}

/**
 * @brief - GetRate
 * 		Get data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return Transfer rate per second
 */
uint32_t BtIntrfGetRate(DevIntrf_t *pDevIntrf)
{
	return 0;	// BLE has no rate
}

/**
 * @brief - SetRate
 * 		Set data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		Rate 	  : Data rate to be set in Hertz (transfer per second)
 *
 * @return 	Actual transfer rate per second set.  It is the real capable rate
 * 			closes to rate being requested.
 */
uint32_t BtIntrfSetRate(DevIntrf_t *pDevIntrf, uint32_t Rate)
{
	return 0; // BLE has no rate
}

/**
 * @brief - StartRx
 * 		Prepare start condition to receive data with subsequence RxData.
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		DevAddr   : The device selection id scheme
 *
 * @return 	true - Success
 * 			false - failed.
 */
bool BtIntrfStartRx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	return true;
}

/**
 * @brief - RxData : retrieve 1 packet of received data
 * 		Receive data into pBuff passed in parameter.  Assuming StartRx was
 * called prior calling this function to get the actual data. BufferLen
 * to receive data must be at least 1 packet in size.  Otherwise remaining
 * bytes are dropped.
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		pBuff 	  : Pointer to memory area to receive data.
 * 		BuffLen   : Length of buffer memory in bytes. Must be at least 1 packet
 * 		            in size.  Otherwise remaining bytes are dropped.
 *
 * @return	Number of bytes read
 */
int BtIntrfRxData(DevIntrf_t *pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	BtDevIntrf_t *intrf = (BtDevIntrf_t*)pDevIntrf->pDevData;
	BtIntrfPkt_t *pkt;
	int cnt = 0;

	pkt = (BtIntrfPkt_t *)CFifoGet(intrf->hRxFifo);
	if (pkt != NULL)
	{
	    cnt = min(BuffLen, pkt->Len);
		memcpy(pBuff, pkt->Data, cnt);
	}

	return cnt;
}

/**
 * @brief - StopRx
 * 		Completion of read data phase. Do require post processing
 * after data has been received via RxData
 * This function must clear the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return	None
 */
void BtIntrfStopRx(DevIntrf_t *pSerDev)
{
	// TODO:
}

/**
 * @brief - StartTx
 * 		Prepare start condition to transfer data with subsequence TxData.
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		DevAddr   : The device selection id scheme
 *
 * @return 	true - Success
 * 			false - failed
 */
bool BtIntrfStartTx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	return true;
}

bool BtIntrfNotify(BtDevIntrf_t *pIntrf)
{
	BtIntrfPkt_t *pkt = NULL;
    bool res = true;

    if (pIntrf->TransBuffLen > 0)
    {
        res = BtDevNotify(&pIntrf->pSrvc->pCharArray[pIntrf->TxCharIdx], pIntrf->TransBuff, pIntrf->TransBuffLen);
    }

    while (res == true)
	{
		pIntrf->TransBuffLen = 0;
		uint32_t state = DisableInterrupt();
		pkt = (BtIntrfPkt_t *)CFifoGet(pIntrf->hTxFifo);
		EnableInterrupt(state);
		if (pkt == NULL)
		{
			return true;
		}
		res = BtDevNotify(&pIntrf->pSrvc->pCharArray[pIntrf->TxCharIdx], pkt->Data, pkt->Len);
	}

	if (pkt != NULL)
	{
		memcpy(pIntrf->TransBuff, pkt->Data, pkt->Len);
		pIntrf->TransBuffLen = pkt->Len;
	}

	return false;
}

/**
 * @brief - TxData
 * 		Transfer data from pData passed in parameter.  Assuming StartTx was
 * called prior calling this function to send the actual data
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		pData 	: Pointer to memory area of data to send.
 * 		DataLen : Length of data memory in bytes
 *
 * @return	Number of bytes sent
 */
int BtIntrfTxData(DevIntrf_t *pDevIntrf, uint8_t *pData, int DataLen)
{
	BtDevIntrf_t *intrf = (BtDevIntrf_t*)pDevIntrf->pDevData;
	BtIntrfPkt_t *pkt;
    int maxlen = intrf->PacketSize - BTINTRF_PKHDR_LEN;
	int cnt = 0;

	while (DataLen > 0)
	{
	    uint32_t state = DisableInterrupt();
		pkt = (BtIntrfPkt_t *)CFifoPut(intrf->hTxFifo);
		EnableInterrupt(state);
		if (pkt == NULL)
		{
			intrf->TxDropCnt++;
			break;
		}
		int l = min(DataLen, maxlen);
		memcpy(pkt->Data, pData, l);
		pkt->Len = l;
		DataLen -= l;
		pData += l;
		cnt += l;
	}

    BtIntrfNotify(intrf);

	return cnt;
}

/**
 * @brief - StopTx
 * 		Completion of sending data via TxData.  Do require post processing
 * after all data was transmitted via TxData.
 * This function must clear the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return	None
 */
void BtIntrfStopTx(DevIntrf_t *pDevIntrf)
{

}

/**
 * @brief - Reset
 *      This function perform a reset of interface.  Must provide empty
 * function of not used.
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return  None
 */
void BtIntrfReset(DevIntrf_t *pDevIntrf)
{

}

/**
 *
 *
 */
void BtIntrfTxComplete(BtGattChar_t *pChar, int CharIdx)
{
    BtIntrfNotify((BtDevIntrf_t*)pChar->pSrvc->pContext);
}

void BtIntrfRxWrCB(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{
	BtDevIntrf_t *intrf = (BtDevIntrf_t*)pChar->pSrvc->pContext;
	BtIntrfPkt_t *pkt;
    //int maxlen = intrf->hTxFifo->BlkSize - sizeof(pkt->Len);

	while (Len > 0) {
		pkt = (BtIntrfPkt_t *)CFifoPut(intrf->hRxFifo);
		if (pkt == NULL)
		{
			intrf->RxDropCnt++;
			break;
		}
		int l = min(intrf->PacketSize - (int)BTINTRF_PKHDR_LEN, Len);
		memcpy(pkt->Data, pData, l);
		pkt->Len = l;
		Len -= l;
		pData += l;
	}

	if (CFifoUsed(intrf->hRxFifo) > 0 && intrf->DevIntrf.EvtCB != NULL)
	{
		intrf->DevIntrf.EvtCB(&intrf->DevIntrf, DEVINTRF_EVT_RX_DATA, NULL, 0);
	}
}

bool BtIntrfInit(BtDevIntrf_t *pIntrf, const BtIntrfCfg_t *pCfg)
{
	if (pIntrf == NULL || pCfg == NULL)
		return false;

	if (pCfg->PacketSize <= 0)
	{
		pIntrf->PacketSize = BTINTRF_PACKET_SIZE + BTINTRF_PKHDR_LEN;
	}
	else
	{
		pIntrf->PacketSize = pCfg->PacketSize + BTINTRF_PKHDR_LEN;
	}

	if (pCfg->pRxFifoMem == NULL || pCfg->pTxFifoMem == NULL)
	{
		pIntrf->hRxFifo = CFifoInit(s_BtDevIntrfRxFifoMem, BTINTRF_CFIFO_SIZE, pIntrf->PacketSize, pCfg->bBlocking);
		pIntrf->hTxFifo = CFifoInit(s_BtDevIntrfRxFifoMem, BTINTRF_CFIFO_SIZE, pIntrf->PacketSize, pCfg->bBlocking);
	}
	else
	{
		pIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
		pIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, pIntrf->PacketSize, pCfg->bBlocking);
	}

	pIntrf->DevIntrf.pDevData = (void*)pIntrf;
	pIntrf->pSrvc = pCfg->pSrvc;
	pIntrf->pSrvc->pContext = pIntrf;

	pIntrf->RxCharIdx = pCfg->RxCharIdx;
	pIntrf->TxCharIdx = pCfg->TxCharIdx;

	pIntrf->pSrvc->pCharArray[pIntrf->RxCharIdx].WrCB = BtIntrfRxWrCB;
	pIntrf->pSrvc->pCharArray[pIntrf->TxCharIdx].TxCompleteCB = BtIntrfTxComplete;
	//pIntrf->pSrvc->pCharArray[pIntrf->RxCharIdx].pSrvc = pIntrf->pSrvc;
	//pIntrf->pSrvc->pCharArray[pIntrf->TxCharIdx].pSrvc = pIntrf->pSrvc;

	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_BLE;
	pIntrf->DevIntrf.Enable = BtIntrfEnable;
	pIntrf->DevIntrf.Disable = BtIntrfDisable;
	pIntrf->DevIntrf.GetRate = BtIntrfGetRate;
	pIntrf->DevIntrf.SetRate = BtIntrfSetRate;
	pIntrf->DevIntrf.StartRx = BtIntrfStartRx;
	pIntrf->DevIntrf.RxData = BtIntrfRxData;
	pIntrf->DevIntrf.StopRx = BtIntrfStopRx;
	pIntrf->DevIntrf.StartTx = BtIntrfStartTx;
	pIntrf->DevIntrf.TxData = BtIntrfTxData;
	pIntrf->DevIntrf.StopTx = BtIntrfStopTx;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->TransBuffLen = 0;
	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);

	DeviceIntrfEnable(&pIntrf->DevIntrf);

//	BtGattUpdate(pIntrf->pSrvc->pCharArray[pIntrf->RxCharIdx].ValHdl,
//				&pIntrf->pSrvc->pCharArray[pIntrf->RxCharIdx], sizeof(BtGattChar_t));

	return true;
}

bool BtIntrf::Init(const BtIntrfCfg_t &Cfg)
{
	return BtIntrfInit(&vBtDevIntrf, &Cfg);
}

bool BtIntrf::RequestToSend(int NbBytes)
{
	bool retval = false;

	// ****
	// Some threaded application firmware may stop sending when queue full
	// causing lockup
	// Try to send to free up the queue before validating.
	//
	BtIntrfNotify(&vBtDevIntrf);

	if (vBtDevIntrf.hTxFifo)
	{
		int avail = CFifoAvail(vBtDevIntrf.hTxFifo);
		if ((avail * (vBtDevIntrf.PacketSize - BTINTRF_PKHDR_LEN)) > NbBytes)
			retval = true;
	}
	else
	{
		retval = true;
	}

	return retval;
}

