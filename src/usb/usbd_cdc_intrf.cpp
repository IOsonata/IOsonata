/**-------------------------------------------------------------------------
@file	usbd_cdc_intrf.cpp

@brief	Generic implementation of USBD CDC device interface


@author	Hoang Nguyen Hoan
@date	May 2, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#include <memory.h>

#include "istddef.h"
#include "cfifo.h"
#include "usb/usbd_cdc_intrf.h"
#include "coredev/interrupt.h"

#define USBD_CDC_PACKET_SIZE			(64)
#define USBD_CDC_CFIFO_MEMSIZE			CFIFO_MEMSIZE(4 * USBD_CDC_PACKET_SIZE)

alignas(4) static uint8_t s_UsbdCdcDevIntrfRxFifoMem[USBD_CDC_CFIFO_MEMSIZE];
alignas(4) static uint8_t s_UsbdCdcDevIntrfTxFifoMem[USBD_CDC_CFIFO_MEMSIZE];

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
void UsbdCdcIntrfDisable(DevIntrf_t *pDevIntrf)
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
void UsbdCdcIntrfEnable(DevIntrf_t *pDevIntrf)
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
uint32_t UsbdCdcIntrfGetRate(DevIntrf_t *pDevIntrf)
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
uint32_t UsbdCdcIntrfSetRate(DevIntrf_t *pDevIntrf, uint32_t Rate)
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
bool UsbdCdcIntrfStartRx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
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
int UsbdCdcIntrfRxData(DevIntrf_t *pDevIntrf, uint8_t *pBuff, int Bufflen)
{
	UsbdCdcDevIntrf_t *intrf = (UsbdCdcDevIntrf_t*)pDevIntrf->pDevData;
	int cnt = 0;
	int l = Bufflen;

	uint8_t *p = CFifoGetMultiple(intrf->hRxFifo, &l);
	if (p != nullptr)
	{
	    cnt = min(Bufflen, l);
		memcpy(pBuff, p, cnt);
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
void UsbdCdcIntrfStopRx(DevIntrf_t *pSerDev)
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
bool UsbdCdcIntrfStartTx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	return true;
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
int UsbdCdcIntrfTxData(DevIntrf_t *pDevIntrf, uint8_t *pData, int Datalen)
{
	UsbdCdcDevIntrf_t *intrf = (UsbdCdcDevIntrf_t*)pDevIntrf->pDevData;
	int cnt = 0;

	while (Datalen > 0)
	{
	    uint32_t state = DisableInterrupt();
	    int l = Datalen;
		uint8_t *p = CFifoPutMultiple(intrf->hTxFifo, &l);
		EnableInterrupt(state);
		if (p == nullptr)
		{
			intrf->TxDropCnt++;
			break;
		}
		memcpy(p, pData, l);
		Datalen -= l;
		pData += l;
		cnt += l;
	}

	// TODO : Send via USB

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
void UsbdCdcIntrfStopTx(DevIntrf_t *pDevIntrf)
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
void UsbdCdcIntrfReset(DevIntrf_t *pDevIntrf)
{

}

bool UsbdCdcIntrfInit(UsbdCdcDevIntrf_t *pIntrf, const UsbdCdcIntrfCfg_t *pCfg)
{
	if (pIntrf == NULL || pCfg == NULL)
		return false;

	if (pCfg->pRxFifoMem == nullptr)
	{
		pIntrf->hRxFifo = CFifoInit(s_UsbdCdcDevIntrfRxFifoMem, USBD_CDC_CFIFO_MEMSIZE, 1, pCfg->bBlocking);
	}
	else
	{
		pIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, 1, pCfg->bBlocking);
	}

	if (pCfg->pTxFifoMem == nullptr)
	{
		pIntrf->hTxFifo = CFifoInit(s_UsbdCdcDevIntrfTxFifoMem, USBD_CDC_CFIFO_MEMSIZE, 1, pCfg->bBlocking);
	}
	else
	{
		pIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, 1, pCfg->bBlocking);
	}

	pIntrf->DevIntrf.pDevData = (void*)pIntrf;

	pIntrf->DevIntrf.Type = DEVINTRF_TYPE_USB;
	pIntrf->DevIntrf.Enable = UsbdCdcIntrfEnable;
	pIntrf->DevIntrf.Disable = UsbdCdcIntrfDisable;
	pIntrf->DevIntrf.GetRate = UsbdCdcIntrfGetRate;
	pIntrf->DevIntrf.SetRate = UsbdCdcIntrfSetRate;
	pIntrf->DevIntrf.StartRx = UsbdCdcIntrfStartRx;
	pIntrf->DevIntrf.RxData = UsbdCdcIntrfRxData;
	pIntrf->DevIntrf.StopRx = UsbdCdcIntrfStopRx;
	pIntrf->DevIntrf.StartTx = UsbdCdcIntrfStartTx;
	pIntrf->DevIntrf.TxData = UsbdCdcIntrfTxData;
	pIntrf->DevIntrf.StopTx = UsbdCdcIntrfStopTx;
	pIntrf->DevIntrf.MaxRetry = 0;
	pIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pIntrf->TransBuffLen = 0;
	pIntrf->RxDropCnt = 0;
	pIntrf->TxDropCnt = 0;
	atomic_flag_clear(&pIntrf->DevIntrf.bBusy);

	DeviceIntrfEnable(&pIntrf->DevIntrf);

	return true;
}

bool UsbdCdcIntrf::Init(const UsbdCdcIntrfCfg_t &Cfg)
{
	return UsbdCdcIntrfInit(&vUsbDevIntrf, &Cfg);
}

bool UsbdCdcIntrf::RequestToSend(int NbBytes)
{
	bool retval = false;

	// ****
	// Some threaded application firmware may stop sending when queue full
	// causing lockup
	// Try to send to free up the queue before validating.
	//
	if (vUsbDevIntrf.hTxFifo)
	{
		if (CFifoAvail(vUsbDevIntrf.hTxFifo) > 0)
		{
			retval = true;
		}
	}
	else
	{
		retval = true;
	}

	return retval;
}
