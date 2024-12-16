/**-------------------------------------------------------------------------
@file	button.cpp

@brief	Button class


@author	Hoang Nguyen Hoan
@date	Apr. 16, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#include <stdint.h>

#include "miscdev/button.h"
#include "coredev/iopincfg.h"

bool ButtonInit(ButtonDev_t * const pBut, ButtonCfg_t * const pCfg, TimerDev_t * const pTimer)
{
	if (pBut == nullptr || pCfg == nullptr)
	{
		return false;
	}

	pBut->Act = pCfg->Act;
	pBut->Pin = pCfg->Pin;
	pBut->Port = pCfg->Port;
	pBut->Type = pCfg->Type;
	pBut->bInt = pCfg->bInt;
	pBut->State = BUTTON_STATE_UP;
	pBut->pTimerDev = pTimer;

	if (pBut->Act == BUTTON_LOGIC_HIGH)
	{
		IOPinConfig(pBut->Port, pBut->Pin, pBut->PinOp, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL);
	}
	else
	{
		IOPinConfig(pBut->Port, pBut->Pin, pBut->PinOp, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	}

	if (pBut->bInt == true)
	{

	}
	return true;
}

