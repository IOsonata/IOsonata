/**-------------------------------------------------------------------------
@file	button.h

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
#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <stdint.h>

#include "coredev/timer.h"
#include "iopinctrl.h"

typedef enum __Button_State{
	BUTTON_STATE_DOWN,
	BUTTON_STATE_UP,
} BUTTON_STATE;

typedef enum __Button_Logic {
	BUTTON_LOGIC_LOW = 0,
	BUTTON_LOGIC_HIGH = 1,
} BUTTON_LOGIC;

typedef enum __Button_Type {
	BUTTON_TYPE_NORMAL,
	BUTTON_TYPE_CAPSENSE
} BUTTON_TYPE;


typedef struct __Button_Dev		ButtonDev_t;

typedef void (*ButEvtHandler_t)(ButtonDev_t *pBut, BUTTON_STATE State);

#pragma pack(push, 4)

typedef struct __Button_Config {
	BUTTON_TYPE Type;			//!< Button type
	int Port;				//!< Gpio port #
	int Pin;				//!< Gpio pin #
	uint8_t PinOp;
	BUTTON_LOGIC Act;			//!< Button active logic
	bool bInt;					//!< Interrupt enable
	uint8_t IntNo;				//!< Interrupt number to use
	ButEvtHandler_t EvtHandler;
} ButtonCfg_t;

struct __Button_Dev {
	BUTTON_TYPE Type;			//!< Button type
	uint8_t Port;				//!< Gpio port #
	uint8_t Pin;				//!< Gpio pin #
	uint8_t PinOp;
	BUTTON_LOGIC Act;			//!< Button active logic
	bool bInt;					//!< Interrupt enable
	uint8_t IntNo;				//!< Interrupt number to use
	TimerDev_t * pTimerDev;		//!< Pointer to timer device
	volatile BUTTON_STATE State;//!< Current button state
	ButEvtHandler_t EvtHandler;
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool ButtonInit(ButtonDev_t * const pBut, ButtonCfg_t * const pCfg, TimerDev_t * const pTimer);
static inline BUTTON_STATE ButtonGetState(ButtonDev_t * const pBut) { return (BUTTON_STATE)(IOPinRead(pBut->Port, pBut->Pin) ^ (int)pBut->Act); }

#ifdef __cplusplus
}

class Button {
public:
	virtual ~Button() {}

	virtual bool Init(ButtonCfg_t &Cfg, Timer * const pTimerObj = nullptr) { return ButtonInit(&vButDev, &Cfg, (pTimerObj ? (TimerDev_t * const)*pTimerObj : nullptr)); }
	operator BUTTON_STATE const () { return vButDev.State; }

protected:
private:
	ButtonDev_t vButDev;
};

#endif

#endif // __BUTTON_H__
