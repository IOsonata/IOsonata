/**-------------------------------------------------------------------------
@file	bt_host.h

@brief	Generic implementation of Bluetooth host device.


@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#ifndef __BT_HOST_H__
#define __BT_HOST_H__

#include "device.h"
#include "bt_ctlr.h"

/** @addtogroup Bluetooth
  * @{
  */

#pragma pack(push, 4)

typedef struct __Bt_Host_Config {

} BtHostCfg_t;

typedef struct __Bt_Host_Dev {
	void *pObj;
} BtHostDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool BtHostInit(const BtHostCfg_t *pCfg, BtCtlrDev_t * const pDev);

#ifdef __cplusplus
}

class BtHost : public Device {
public:
	virtual bool Init(const BtHostCfg_t &Cfg, BtCtlr * const pCtlr) { return BtHostInit(&Cfg, *pCtlr); }
	virtual int Send(uint8_t * const pData, int DataLen);
	virtual int Receive(uint8_t * const pBuff, int BuffLen);

protected:
private:
	BtHostDev_t vDevData;
};

#endif

/** @} end group Bluetooth */

#endif // __BT_HOST_H__
