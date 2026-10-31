/**-------------------------------------------------------------------------
@file	sercom_samd.c

@brief	SAM D11/D21 SERCOM common functions

SERCOM clocks and the pad of each SERCOM pin. The pad tables come from the
device header pin definitions PIN_PxyyC_SERCOMn_PADm (function C) and
PIN_PxyyD_SERCOMn_PADm (function D, SERCOM-ALT).

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "samdxxx.h"

#include "coredev/iopincfg.h"
#include "coredev/shared_intrf.h"
#include "clock_samd.h"
#include "sercom_samd.h"

// SERCOM pad entry: valid bit, SERCOM number, pad
#define SAMD_SERCOM_PAD(Dev, Pad)	(0x80U | ((Dev) << 2) | (Pad))
#define SAMD_SERCOM_NONE			0U
#define SAMD_SERCOM_VALID			0x80U

#pragma pack(push, 4)

typedef struct {
	uint8_t PortNo;
	uint8_t PinNo;
	uint8_t FuncC;			//!< SERCOM pad on function C
	uint8_t FuncD;			//!< SERCOM pad on function D
} SamdSercomPin_t;

#pragma pack(pop)

static sercom_registers_t * const s_SamdSercomReg[SAMD_SERCOM_CNT] = {
	SERCOM0_REGS, SERCOM1_REGS, SERCOM2_REGS,
#if SAMD_SERCOM_CNT > 3
	SERCOM3_REGS, SERCOM4_REGS, SERCOM5_REGS,
#endif
};

static const SamdSercomPin_t s_SamdSercomPins[] = {
#if defined(__SAMD11D14AM__)
	{ IOPORTA, 4, SAMD_SERCOM_PAD(0, 2), SAMD_SERCOM_PAD(0, 0) },
	{ IOPORTA, 5, SAMD_SERCOM_PAD(0, 3), SAMD_SERCOM_PAD(0, 1) },
	{ IOPORTA, 6, SAMD_SERCOM_PAD(0, 0), SAMD_SERCOM_PAD(0, 2) },
	{ IOPORTA, 7, SAMD_SERCOM_PAD(0, 1), SAMD_SERCOM_PAD(0, 3) },
	{ IOPORTA, 8, SAMD_SERCOM_PAD(1, 2), SAMD_SERCOM_PAD(0, 2) },
	{ IOPORTA, 9, SAMD_SERCOM_PAD(1, 3), SAMD_SERCOM_PAD(0, 3) },
	{ IOPORTA, 10, SAMD_SERCOM_PAD(0, 2), SAMD_SERCOM_PAD(2, 2) },
	{ IOPORTA, 11, SAMD_SERCOM_PAD(0, 3), SAMD_SERCOM_PAD(2, 3) },
	{ IOPORTA, 14, SAMD_SERCOM_PAD(0, 0), SAMD_SERCOM_PAD(2, 0) },
	{ IOPORTA, 15, SAMD_SERCOM_PAD(0, 1), SAMD_SERCOM_PAD(2, 1) },
	{ IOPORTA, 16, SAMD_SERCOM_PAD(1, 2), SAMD_SERCOM_PAD(2, 2) },
	{ IOPORTA, 17, SAMD_SERCOM_PAD(1, 3), SAMD_SERCOM_PAD(2, 3) },
	{ IOPORTA, 22, SAMD_SERCOM_PAD(1, 0), SAMD_SERCOM_PAD(2, 0) },
	{ IOPORTA, 23, SAMD_SERCOM_PAD(1, 1), SAMD_SERCOM_PAD(2, 1) },
	{ IOPORTA, 24, SAMD_SERCOM_PAD(1, 2), SAMD_SERCOM_PAD(2, 2) },
	{ IOPORTA, 25, SAMD_SERCOM_PAD(1, 3), SAMD_SERCOM_PAD(2, 3) },
	{ IOPORTA, 30, SAMD_SERCOM_PAD(1, 0), SAMD_SERCOM_PAD(1, 2) },
	{ IOPORTA, 31, SAMD_SERCOM_PAD(1, 1), SAMD_SERCOM_PAD(1, 3) },
#elif defined(__SAMD21J18A__)
	{ IOPORTA, 0, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(1, 0) },
	{ IOPORTA, 1, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(1, 1) },
	{ IOPORTA, 4, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(0, 0) },
	{ IOPORTA, 5, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(0, 1) },
	{ IOPORTA, 6, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(0, 2) },
	{ IOPORTA, 7, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(0, 3) },
	{ IOPORTA, 8, SAMD_SERCOM_PAD(0, 0), SAMD_SERCOM_PAD(2, 0) },
	{ IOPORTA, 9, SAMD_SERCOM_PAD(0, 1), SAMD_SERCOM_PAD(2, 1) },
	{ IOPORTA, 10, SAMD_SERCOM_PAD(0, 2), SAMD_SERCOM_PAD(2, 2) },
	{ IOPORTA, 11, SAMD_SERCOM_PAD(0, 3), SAMD_SERCOM_PAD(2, 3) },
	{ IOPORTA, 12, SAMD_SERCOM_PAD(2, 0), SAMD_SERCOM_PAD(4, 0) },
	{ IOPORTA, 13, SAMD_SERCOM_PAD(2, 1), SAMD_SERCOM_PAD(4, 1) },
	{ IOPORTA, 14, SAMD_SERCOM_PAD(2, 2), SAMD_SERCOM_PAD(4, 2) },
	{ IOPORTA, 15, SAMD_SERCOM_PAD(2, 3), SAMD_SERCOM_PAD(4, 3) },
	{ IOPORTA, 16, SAMD_SERCOM_PAD(1, 0), SAMD_SERCOM_PAD(3, 0) },
	{ IOPORTA, 17, SAMD_SERCOM_PAD(1, 1), SAMD_SERCOM_PAD(3, 1) },
	{ IOPORTA, 18, SAMD_SERCOM_PAD(1, 2), SAMD_SERCOM_PAD(3, 2) },
	{ IOPORTA, 19, SAMD_SERCOM_PAD(1, 3), SAMD_SERCOM_PAD(3, 3) },
	{ IOPORTA, 20, SAMD_SERCOM_PAD(5, 2), SAMD_SERCOM_PAD(3, 2) },
	{ IOPORTA, 21, SAMD_SERCOM_PAD(5, 3), SAMD_SERCOM_PAD(3, 3) },
	{ IOPORTA, 22, SAMD_SERCOM_PAD(3, 0), SAMD_SERCOM_PAD(5, 0) },
	{ IOPORTA, 23, SAMD_SERCOM_PAD(3, 1), SAMD_SERCOM_PAD(5, 1) },
	{ IOPORTA, 24, SAMD_SERCOM_PAD(3, 2), SAMD_SERCOM_PAD(5, 2) },
	{ IOPORTA, 25, SAMD_SERCOM_PAD(3, 3), SAMD_SERCOM_PAD(5, 3) },
	{ IOPORTA, 30, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(1, 2) },
	{ IOPORTA, 31, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(1, 3) },
	{ IOPORTB, 0, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 2) },
	{ IOPORTB, 1, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 3) },
	{ IOPORTB, 2, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 0) },
	{ IOPORTB, 3, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 1) },
	{ IOPORTB, 8, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(4, 0) },
	{ IOPORTB, 9, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(4, 1) },
	{ IOPORTB, 10, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(4, 2) },
	{ IOPORTB, 11, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(4, 3) },
	{ IOPORTB, 12, SAMD_SERCOM_PAD(4, 0), SAMD_SERCOM_NONE },
	{ IOPORTB, 13, SAMD_SERCOM_PAD(4, 1), SAMD_SERCOM_NONE },
	{ IOPORTB, 14, SAMD_SERCOM_PAD(4, 2), SAMD_SERCOM_NONE },
	{ IOPORTB, 15, SAMD_SERCOM_PAD(4, 3), SAMD_SERCOM_NONE },
	{ IOPORTB, 16, SAMD_SERCOM_PAD(5, 0), SAMD_SERCOM_NONE },
	{ IOPORTB, 17, SAMD_SERCOM_PAD(5, 1), SAMD_SERCOM_NONE },
	{ IOPORTB, 22, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 2) },
	{ IOPORTB, 23, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 3) },
	{ IOPORTB, 30, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 0) },
	{ IOPORTB, 31, SAMD_SERCOM_NONE, SAMD_SERCOM_PAD(5, 1) },
#else
#error Unsupported SAM D device
#endif
};

static const int s_NbSamdSercomPins = sizeof(s_SamdSercomPins) / sizeof(SamdSercomPin_t);

// Pins with the I2C pad drive, port in bits 7:5, pin in bits 4:0
#define SAMD_I2C_PIN(Port, Pin)		(((Port) << 5) | (Pin))

static const uint8_t s_SamdI2cPins[] = {
#if defined(__SAMD11D14AM__)
	SAMD_I2C_PIN(IOPORTA, 14), SAMD_I2C_PIN(IOPORTA, 15), SAMD_I2C_PIN(IOPORTA, 22), SAMD_I2C_PIN(IOPORTA, 23),
#elif defined(__SAMD21J18A__)
	SAMD_I2C_PIN(IOPORTA, 8), SAMD_I2C_PIN(IOPORTA, 9), SAMD_I2C_PIN(IOPORTA, 12), SAMD_I2C_PIN(IOPORTA, 13),
	SAMD_I2C_PIN(IOPORTA, 16), SAMD_I2C_PIN(IOPORTA, 17), SAMD_I2C_PIN(IOPORTA, 22), SAMD_I2C_PIN(IOPORTA, 23),
	SAMD_I2C_PIN(IOPORTB, 12), SAMD_I2C_PIN(IOPORTB, 13), SAMD_I2C_PIN(IOPORTB, 16), SAMD_I2C_PIN(IOPORTB, 17),
	SAMD_I2C_PIN(IOPORTB, 30), SAMD_I2C_PIN(IOPORTB, 31),
#endif
};

sercom_registers_t *SamdSercomGetReg(int DevNo)
{
	if (DevNo < 0 || DevNo >= SAMD_SERCOM_CNT)
	{
		return NULL;
	}

	return s_SamdSercomReg[DevNo];
}

bool SamdSercomClockEnable(int DevNo)
{
	if (DevNo < 0 || DevNo >= SAMD_SERCOM_CNT)
	{
		return false;
	}

	// Register access needs the APBC bus clock, the SERCOM runs on the core
	// generic clock
	PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM0_Msk << DevNo;

	return SamdGclkChannelEnable((uint8_t)(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + DevNo), SAMD_GCLK_MAIN);
}

void SamdSercomClockDisable(int DevNo)
{
	if (DevNo < 0 || DevNo >= SAMD_SERCOM_CNT)
	{
		return;
	}

	(void)SamdGclkChannelDisable((uint8_t)(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + DevNo));
	PM_REGS->PM_APBCMASK &= ~(PM_APBCMASK_SERCOM0_Msk << DevNo);
}

int SamdSercomPad(int DevNo, const IOPinCfg_t *pPin)
{
	if (pPin == NULL || (pPin->PinOp != IOPINOP_PERIPHC && pPin->PinOp != IOPINOP_PERIPHD))
	{
		return -1;
	}

	for (int i = 0; i < s_NbSamdSercomPins; i++)
	{
		if (s_SamdSercomPins[i].PortNo == pPin->PortNo && s_SamdSercomPins[i].PinNo == pPin->PinNo)
		{
			uint32_t pad = pPin->PinOp == IOPINOP_PERIPHC ? s_SamdSercomPins[i].FuncC : s_SamdSercomPins[i].FuncD;

			if ((pad & SAMD_SERCOM_VALID) == 0U || ((pad >> 2) & 0x7U) != (uint32_t)DevNo)
			{
				return -1;
			}

			return (int)(pad & 3U);
		}
	}

	return -1;
}

bool SamdSercomI2cPin(const IOPinCfg_t *pPin)
{
	if (pPin == NULL || pPin->PortNo < 0 || pPin->PortNo > 1 || pPin->PinNo < 0 || pPin->PinNo > 31)
	{
		return false;
	}

	uint8_t id = (uint8_t)SAMD_I2C_PIN(pPin->PortNo, pPin->PinNo);

	for (unsigned i = 0; i < sizeof(s_SamdI2cPins); i++)
	{
		if (s_SamdI2cPins[i] == id)
		{
			return true;
		}
	}

	return false;
}

bool SamdSercomOwned(int DevNo, const DevIntrf_t *pIntrf)
{
	sercom_registers_t *reg = SamdSercomGetReg(DevNo);

	if (reg == NULL)
	{
		return true;
	}

	if (g_SharedIntrf[DevNo].pIntrf == NULL || g_SharedIntrf[DevNo].pIntrf == pIntrf)
	{
		return false;
	}

	// ENABLE is bit 1 of CTRLA in every SERCOM mode
	return (PM_REGS->PM_APBCMASK & (PM_APBCMASK_SERCOM0_Msk << DevNo)) != 0U &&
		   (reg->USART_INT.SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE_Msk) != 0U;
}
