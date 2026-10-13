/**-------------------------------------------------------------------------
@file	dac_r9a02.cpp

@brief	R9A02G021 8-bit D/A converter (DAC8)

See dac_r9a02.h. R9A02G021 User's Manual R01UH1036EJ0110, chapter 30.

Module stop: MSTPCRC.MSTPC22 and MSTPCRD.MSTPD20 hold the same value
(10.2.4, 10.2.5), both are written. Data right aligned (DADPR.DPSEL = 0).
DACR bits 4:0 read 1 and are written 1. The pins are analog (ASEL) before
DAEXOUT.DAEXOn connects the output, DAOEn starts the conversion (30.3).

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <stdint.h>
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "system_r9a02.h"
#include "elc_r9a02.h"
#include "dac_r9a02.h"

static_assert(offsetof(R9A02_DAC8_Type, DAEXOUT) == 0x700, "DAEXOUT at 0x700");

// DACOUT0 P000, DACOUT1 P001 (table 16.6)
static const uint8_t s_R9a02DacPin[R9A02_DAC_CHAN_CNT] = { 0, 1 };

static const void *s_R9a02DacOwner[R9A02_DAC_CHAN_CNT] = { NULL, NULL };

static bool R9a02DacValid(int Chan)
{
	return Chan >= 0 && Chan < R9A02_DAC_CHAN_CNT;
}

static void R9a02DacModule(bool bOn)
{
	if (bOn)
	{
		R9a02ModuleStart(R9A02_MSTPCR_C, R9A02_MSTPCRC_DAC8);
		R9a02ModuleStart(R9A02_MSTPCR_D, R9A02_MSTPCRD_DAC8);
	}
	else
	{
		R9a02ModuleStop(R9A02_MSTPCR_C, R9A02_MSTPCRC_DAC8);
		R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_DAC8);
	}
}

bool R9a02DacOpen(int Chan, bool bPinOut, const void *pOwner)
{
	if (!R9a02DacValid(Chan) || pOwner == NULL)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	if (s_R9a02DacOwner[Chan] != NULL && s_R9a02DacOwner[Chan] != pOwner)
	{
		EnableInterrupt(state);

		return false;
	}

	bool first = s_R9a02DacOwner[0] == NULL && s_R9a02DacOwner[1] == NULL;

	if (first)
	{
		R9a02DacModule(true);
		R9A02_DAC8->DADPR = 0U;
		R9A02_DAC8->DACR = R9A02_DACR_FIXED;
		R9A02_DAC8->DAEXOUT = 0U;
	}
	s_R9a02DacOwner[Chan] = pOwner;

	R9A02_DAC8_Type *reg = R9A02_DAC8;
	uint16_t pin = (uint16_t)R9A02_DAEXOUT_DAEXO(Chan);

	reg->DADR[Chan] = 0U;
	if (bPinOut)
	{
		IOPinConfig(0, s_R9a02DacPin[Chan], IOPINOP_FUNC31, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		reg->DAEXOUT = (uint16_t)(reg->DAEXOUT | pin);
	}
	else
	{
		reg->DAEXOUT = (uint16_t)(reg->DAEXOUT & ~pin);
	}
	reg->DACR = (uint8_t)((reg->DACR & (R9A02_DACR_DAOE(0) | R9A02_DACR_DAOE(1))) | R9A02_DACR_FIXED |
						  R9A02_DACR_DAOE(Chan));

	EnableInterrupt(state);

	return true;
}

void R9a02DacClose(int Chan, const void *pOwner)
{
	if (!R9a02DacValid(Chan) || pOwner == NULL || s_R9a02DacOwner[Chan] != pOwner)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	R9A02_DAC8_Type *reg = R9A02_DAC8;
	bool pin = (reg->DAEXOUT & R9A02_DAEXOUT_DAEXO(Chan)) != 0U;

	R9a02ElcLink(Chan ? R9A02_ELC_DEST_DAC1 : R9A02_ELC_DEST_DAC0, 0U);
	reg->DACR = (uint8_t)((reg->DACR & ~R9A02_DACR_DAOE(Chan)) | R9A02_DACR_FIXED);
	reg->DAEXOUT = (uint16_t)(reg->DAEXOUT & ~R9A02_DAEXOUT_DAEXO(Chan));
	if (pin)
	{
		IOPinDisable(0, s_R9a02DacPin[Chan]);
	}
	s_R9a02DacOwner[Chan] = NULL;

	if (s_R9a02DacOwner[0] == NULL && s_R9a02DacOwner[1] == NULL)
	{
		R9a02DacModule(false);
	}

	EnableInterrupt(state);
}

static bool R9a02DacOwned(int Chan, const void *pOwner)
{
	return R9a02DacValid(Chan) && pOwner != NULL && s_R9a02DacOwner[Chan] == pOwner;
}

bool R9a02DacWrite(int Chan, uint8_t Code, const void *pOwner)
{
	if (!R9a02DacOwned(Chan, pOwner))
	{
		return false;
	}

	R9A02_DAC8->DADR[Chan] = Code;

	return true;
}

bool R9a02DacOutput(int Chan, bool bOn, const void *pOwner)
{
	if (!R9a02DacOwned(Chan, pOwner))
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();
	R9A02_DAC8_Type *reg = R9A02_DAC8;
	uint8_t dacr = (uint8_t)(reg->DACR | R9A02_DACR_FIXED);

	reg->DACR = bOn ? (uint8_t)(dacr | R9A02_DACR_DAOE(Chan)) : (uint8_t)(dacr & ~R9A02_DACR_DAOE(Chan));

	EnableInterrupt(state);

	return true;
}

bool R9a02DacTrigger(int Chan, uint8_t EvtId, const void *pOwner)
{
	if (!R9a02DacOwned(Chan, pOwner))
	{
		return false;
	}

	R9A02_ELC_DEST dest = Chan ? R9A02_ELC_DEST_DAC1 : R9A02_ELC_DEST_DAC0;

	if (EvtId == 0U)
	{
		return R9a02ElcLink(dest, 0U);
	}

	// Off first, the event sets DAOEn (30.4)
	R9a02DacOutput(Chan, false, pOwner);

	return R9a02ElcLink(dest, EvtId);
}

DacR9a02::DacR9a02()
{
	vVcc = 3.3f;
	for (int i = 0; i < R9A02_DAC_CHAN_CNT; i++)
	{
		vCode[i] = 0;
		vbOpen[i] = false;
		vbPinOut[i] = false;
	}
}

bool DacR9a02::Init(const DAC_CFG &Cfg, DeviceIntrf * const pIntrf)
{
	if (Cfg.Mode != DAC_MODE_SINGLE_ENDED)
	{
		return false;
	}

	PowerOff();

	return true;
}

bool DacR9a02::OpenChannel(int Chan, bool bPinOut)
{
	if (!R9a02DacOpen(Chan, bPinOut, this))
	{
		return false;
	}

	vbOpen[Chan] = true;
	vbPinOut[Chan] = bPinOut;
	vCode[Chan] = 0;

	return true;
}

void DacR9a02::CloseChannel(int Chan)
{
	if (R9a02DacValid(Chan) && vbOpen[Chan])
	{
		R9a02DacClose(Chan, this);
		vbOpen[Chan] = false;
	}
}

bool DacR9a02::WriteCode(int Chan, uint8_t Code)
{
	if (!R9a02DacValid(Chan) || !vbOpen[Chan] || !R9a02DacWrite(Chan, Code, this))
	{
		return false;
	}

	vCode[Chan] = Code;

	return true;
}

float DacR9a02::WriteVolt(int Chan, float Volt)
{
	float code = Volt * 256.0f / vVcc;

	if (code < 0.0f)
	{
		code = 0.0f;
	}
	if (code > 255.0f)
	{
		code = 255.0f;
	}

	uint8_t c = (uint8_t)(code + 0.5f > 255.0f ? 255.0f : code + 0.5f);

	if (!WriteCode(Chan, c))
	{
		return -1.0f;
	}

	return (float)c * vVcc / 256.0f;
}

bool DacR9a02::Trigger(int Chan, uint8_t EvtId)
{
	return R9a02DacValid(Chan) && vbOpen[Chan] && R9a02DacTrigger(Chan, EvtId, this);
}

bool DacR9a02::Enable()
{
	bool ok = true;

	for (int i = 0; i < R9A02_DAC_CHAN_CNT; i++)
	{
		if (vbOpen[i])
		{
			ok = R9a02DacWrite(i, vCode[i], this) && R9a02DacOutput(i, true, this) && ok;
		}
	}

	return ok;
}

void DacR9a02::Disable()
{
	for (int i = 0; i < R9A02_DAC_CHAN_CNT; i++)
	{
		if (vbOpen[i])
		{
			R9a02DacOutput(i, false, this);
		}
	}
}

void DacR9a02::Reset()
{
	for (int i = 0; i < R9A02_DAC_CHAN_CNT; i++)
	{
		if (vbOpen[i])
		{
			WriteCode(i, 0);
		}
	}
}

void DacR9a02::PowerOff()
{
	for (int i = 0; i < R9A02_DAC_CHAN_CNT; i++)
	{
		CloseChannel(i);
	}
}
