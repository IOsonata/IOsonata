/**-------------------------------------------------------------------------
@file	dac_samd.cpp

@brief	SAM D11/D21 DAC implementation

One 10-bit channel written by software. See dac_samd.h.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include <math.h>

#include "samdxxx.h"
#include "clock_samd.h"
#include "coredev/interrupt.h"
#include "idelay.h"
#include "analog_samd.h"
#include "dac_samd.h"

#define SAMD_DAC_SYNC_TIMEOUT		100000U
#define SAMD_DAC_STARTUP_US			10U			// Start-up time, VDDANA below 2.6 V

// VOUT on PA02 and VREFA on PA03, function B
#define SAMD_DAC_VOUT_PIN			PIN_PA02B_DAC_VOUT
#define SAMD_DAC_VREFA_PIN			PIN_PA03B_DAC_VREFP

static DacSamd *s_pSamdDac = nullptr;

static bool SamdDacSync(void)
{
	for (uint32_t i = 0; i < SAMD_DAC_SYNC_TIMEOUT; i++)
	{
		if ((DAC_REGS->DAC_STATUS & DAC_STATUS_SYNCBUSY_Msk) == 0U)
		{
			return true;
		}
	}

	return false;
}

DacSamd::DacSamd()
{
	vRefVolt = 3.3f;
	vRefSel = DAC_CTRLB_REFSEL_AVCC_Val;
	vOut = 0;
	vCode = 0;
	vbOn = false;
}

DacSamd::~DacSamd()
{
	PowerOff();
}

bool DacSamd::Owned()
{
	return s_pSamdDac == this;
}

bool DacSamd::Init(const DAC_CFG &Cfg, DeviceIntrf * const pIntrf)
{
	if ((s_pSamdDac && !Owned()) || Cfg.Mode != DAC_MODE_SINGLE_ENDED || pIntrf)
	{
		return false;
	}

	PowerOff();
	s_pSamdDac = this;

	PM_REGS->PM_APBCMASK |= PM_APBCMASK_DAC_Msk;
	if (!SamdGclkChannelEnable(DAC_GCLK_ID, SAMD_GCLK_MAIN))
	{
		PowerOff();

		return false;
	}

	DAC_REGS->DAC_CTRLA = DAC_CTRLA_SWRST_Msk;
	if (!SamdDacSync() || (DAC_REGS->DAC_CTRLA & DAC_CTRLA_SWRST_Msk))
	{
		PowerOff();

		return false;
	}

	vCode = 0;
	vbOn = false;
	Valid(true);

	return true;
}

// DAC disabled, CTRLB can be written. A write during a synchronization that
// does not end could be lost or stall the bus: none is made then.
bool DacSamd::Stop()
{
	uint32_t state = DisableInterrupt();
	bool ok = SamdDacSync();

	if (ok)
	{
		DAC_REGS->DAC_CTRLA = 0;
		ok = SamdDacSync();
	}
	EnableInterrupt(state);

	if (!ok)
	{
		Valid(false);
	}

	return ok;
}

// CTRLB of the open channel and the code written with the DAC disabled,
// then the DAC enabled if the application has it on, and the start-up time
// waited (usDelay runs short at 48 MHz: twice the time)
bool DacSamd::Start()
{
	uint32_t state = DisableInterrupt();
	bool ok = true;

	DAC_REGS->DAC_CTRLB = (uint8_t)(DAC_CTRLB_REFSEL(vRefSel) | vOut);
	if (vOut)
	{
		DAC_REGS->DAC_DATA = vCode;
		ok = SamdDacSync();
		if (ok && vbOn)
		{
			DAC_REGS->DAC_CTRLA = DAC_CTRLA_ENABLE_Msk;
			ok = SamdDacSync();
		}
	}
	EnableInterrupt(state);

	if (!ok)
	{
		Valid(false);

		return false;
	}
	if (vOut && vbOn)
	{
		usDelay(SAMD_DAC_STARTUP_US * 2U);
	}

	return true;
}

// Pins and VREF outputs of the open channel: VOUT with the output buffer,
// VREFA with the external reference, the bandgap output with INT1V (as ASF)
void DacSamd::Resources(bool bOn)
{
	bool on = bOn && vOut;

	if (vOut & DAC_CTRLB_EOEN_Msk)
	{
		SamdAnalogPin(SAMD_ANALOG_DAC, SAMD_DAC_VOUT_PIN, on);
	}
	if (vRefSel == DAC_CTRLB_REFSEL_VREFP_Val)
	{
		SamdAnalogPin(SAMD_ANALOG_DAC, SAMD_DAC_VREFA_PIN, on);
	}
	SamdAnalogVref(SAMD_ANALOG_DAC, on && vRefSel == DAC_CTRLB_REFSEL_INT1V_Val ? SYSCTRL_VREF_BGOUTEN_Msk : 0U);
}

bool DacSamd::Reference(const AdcRefVolt_t &Ref)
{
	uint8_t sel;

	if (!isfinite(Ref.Voltage) || Ref.Voltage <= 0)
	{
		return false;
	}

	switch (Ref.Type)
	{
		case ADC_REFVOLT_TYPE_INTERNAL:
			sel = DAC_CTRLB_REFSEL_INT1V_Val;
			break;
		case ADC_REFVOLT_TYPE_SUPPLY:
			sel = DAC_CTRLB_REFSEL_AVCC_Val;
			break;
		case ADC_REFVOLT_TYPE_EXTERNAL:
			if (Ref.Pin != 0)
			{
				return false;
			}
			sel = DAC_CTRLB_REFSEL_VREFP_Val;
			break;
		default:
			return false;
	}

	if (!Owned() || vOut == 0)
	{
		vRefSel = sel;
		vRefVolt = Ref.Voltage;

		return true;
	}

	// Open channel: DAC stopped, reference pin and bandgap moved, restarted
	if (!Valid() || !Stop())
	{
		return false;
	}
	Resources(false);
	vRefSel = sel;
	vRefVolt = Ref.Voltage;
	Resources(true);

	return Start();
}

bool DacSamd::OpenChannel(int Chan, bool bPinOut, bool bInternal)
{
	if (!Owned() || !Valid() || Chan != 0 || (!bPinOut && !bInternal))
	{
		return false;
	}

	CloseChannel(Chan);
	if (!Valid())
	{
		return false;
	}

	vOut = (uint8_t)((bPinOut ? DAC_CTRLB_EOEN_Msk : 0U) | (bInternal ? DAC_CTRLB_IOEN_Msk : 0U));
	vCode = 0;
	vbOn = true;
	Resources(true);
	if (!Start())
	{
		CloseChannel(Chan);

		return false;
	}

	return true;
}

void DacSamd::CloseChannel(int Chan)
{
	if (!Owned() || Chan != 0 || vOut == 0)
	{
		return;
	}

	// Outputs off before the pins are released
	if (Stop())
	{
		uint8_t out = vOut;

		vOut = 0;
		(void)Start();
		vOut = out;
	}
	Resources(false);
	vOut = 0;
	vbOn = false;
}

bool DacSamd::WriteCode(int Chan, uint16_t Code)
{
	if (!Owned() || !Valid() || Chan != 0 || vOut == 0 || Code > DAC_SAMD_CODE_MAX)
	{
		return false;
	}

	// Also from an interrupt: the register sequences of the main code run
	// with interrupts disabled
	uint32_t state = DisableInterrupt();
	bool ok = SamdDacSync();

	if (ok)
	{
		DAC_REGS->DAC_DATA = Code;
		ok = SamdDacSync();
	}
	if (ok)
	{
		vCode = Code;
	}
	EnableInterrupt(state);

	if (!ok)
	{
		Valid(false);
	}

	return ok;
}

float DacSamd::WriteVolt(int Chan, float Volt)
{
	if (!isfinite(Volt))
	{
		return -1.0f;
	}

	float code = Volt * (float)DAC_SAMD_CODE_MAX / vRefVolt + 0.5f;

	if (code < 0.0f)
	{
		code = 0.0f;
	}
	if (code > (float)DAC_SAMD_CODE_MAX)
	{
		code = (float)DAC_SAMD_CODE_MAX;
	}

	uint16_t c = (uint16_t)code;

	if (!WriteCode(Chan, c))
	{
		return -1.0f;
	}

	return (float)c * vRefVolt / (float)DAC_SAMD_CODE_MAX;
}

bool DacSamd::Enable()
{
	if (!Owned() || !Valid())
	{
		return false;
	}
	if (vOut == 0 || vbOn)
	{
		return true;
	}

	vbOn = true;

	return Start();
}

void DacSamd::Disable()
{
	if (!Owned() || vOut == 0 || !(PM_REGS->PM_APBCMASK & PM_APBCMASK_DAC_Msk))
	{
		return;
	}

	vbOn = false;
	(void)Stop();
}

void DacSamd::Reset()
{
	if (Owned() && vOut)
	{
		(void)WriteCode(0, 0);
	}
}

void DacSamd::PowerOff()
{
	if (!Owned())
	{
		return;
	}

	CloseChannel(0);
	if ((PM_REGS->PM_APBCMASK & PM_APBCMASK_DAC_Msk) && SamdDacSync())
	{
		DAC_REGS->DAC_CTRLA = DAC_CTRLA_SWRST_Msk;
		(void)SamdDacSync();
	}
	(void)SamdGclkChannelDisable(DAC_GCLK_ID);
	PM_REGS->PM_APBCMASK &= ~PM_APBCMASK_DAC_Msk;

	Valid(false);
	vOut = 0;
	vbOn = false;
	s_pSamdDac = nullptr;
}
