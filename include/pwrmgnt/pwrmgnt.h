/**-------------------------------------------------------------------------
@file	pwrmgnt.h

@brief	Generic power management definition

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	July 25, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#ifndef __PWRMGNT_H__
#define __PWRMGNT_H__

#include <stdint.h>

#include "device_intrf.h"
#include "device.h"

/** @addtogroup Power
  * @{
  */


#define PWRMGNT_VOUT_MAXCNT			4	//!< Max number of Vout

typedef struct __Power_Config {
	uint32_t DevAddr;					//!< Device address
	int Vout[PWRMGNT_VOUT_MAXCNT];		//!< V out voltage in mV
	int CurLimit[PWRMGNT_VOUT_MAXCNT];	//!< Current limit for each V out
} PWRCFG;

class PowerMgnt : public Device {
public:
	virtual bool Init(const PWRCFG &Cfg, DeviceIntrf *pIntrf) = 0;
	virtual int EnableVout(int VoutIdx, int mVolt) = 0;

protected:
private:

};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif //__PWRMGNT_H__
