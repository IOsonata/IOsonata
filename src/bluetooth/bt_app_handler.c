/**-------------------------------------------------------------------------
@file	bt_app_handler.cpp

@brief	Bluetooth application firmware event callback

Dummy user app handler.  All functions can be overloaded

@author	Hoang Nguyen Hoan
@date	Feb. 23, 2017

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdbool.h>

#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_app.h"

__attribute__((weak)) void BtAppInitCustomData()
{

}

__attribute__((weak)) void BtDevInitCustomSrvc()
{

}

__attribute__((weak)) void BtAppAdvTimeoutHandler()
{

}

__attribute__((weak))  void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{

}

__attribute__((weak))  void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx)
{

}

__attribute__((weak)) void BtAppRtosWaitEvt(void)
{

}

__attribute__((weak)) void BtAppEvtConnected(uint16_t ConnHdl)
{

}

__attribute__((weak)) void BtAppEvtDisconnected(uint16_t ConnHdl)
{

}

